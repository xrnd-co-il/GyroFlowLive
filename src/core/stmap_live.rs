
use std::sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}};
use std::thread;
use std::time::Duration;

use crossbeam_channel::{bounded, Receiver, Sender, TrySendError};
use log::{debug, error, info, warn};

use crate::{StabilizationManager, stabilization::*, zooming::*};
// reuse your existing helpers & types from stmaps.rs
use crate::stmap::{parallel_exr}; // if it's in stmaps.rs; adjust path

/// Item submitted by the capture/render scheduler.
#[derive(Clone, Copy, Debug)]
pub struct LiveFrameJob {
    pub frame_index: usize,
    pub frame_ts_ms: f64,
}

/// Same shape as generate_stmaps() emits.
pub type StmapItem = (String, usize, Vec<u8>, Vec<u8>);

pub struct StmapsLive {
    tx_in: Sender<LiveFrameJob>,
    rx_out: Receiver<StmapItem>,
    running: Arc<AtomicBool>,
    _worker: thread::JoinHandle<()>,
}

impl StmapsLive {
    /// Create a live STMaps worker with bounded queues.
    /// - in_cap: how many pending frame jobs we queue
    /// - out_cap: how many finished stmaps we keep for the render thread
    pub fn new(stab: Arc<StabilizationManager>, in_cap: usize, out_cap: usize) -> Self {
        let (tx_in, rx_in) = bounded::<LiveFrameJob>(in_cap.max(1));
        let (tx_out, rx_out) = bounded::<StmapItem>(out_cap.max(1));
        let running = Arc::new(AtomicBool::new(true));

        let running_flag = running.clone();

        let worker = thread::Builder::new()
            .name("stmaps_live_worker".into())
            .spawn(move || {
                Self::worker_loop(stab, rx_in, tx_out, running_flag);
            })
            .expect("spawn stmaps live worker");

        Self { tx_in, rx_out, running, _worker: worker }
    }

    /// Non-blocking: submit a frame job.
    /// If the queue is full, drop the **oldest** job to keep latency bounded.
    pub fn submit_frame(&self, job: LiveFrameJob) {
        match self.tx_in.try_send(job) {
            Ok(_) => {}
            Err(TrySendError::Full(j)) => {
                // Drop oldest by draining one then re-send latest
                warn!("stmaps_live: input queue full; dropping oldest");
                let _ = self.tx_in.recv(); // remove one (oldest)
                let _ = self.tx_in.try_send(j);
            }
            Err(TrySendError::Disconnected(_)) => {
                error!("stmaps_live: input channel disconnected");
            }
        }
    }

    /// Non-blocking: try to pop a finished stmap item (same type as generate_stmaps()).
    pub fn try_pop_map(&self) -> Option<StmapItem> {
        self.rx_out.try_recv().ok()
    }

    /// Optional blocking pop (if you prefer render thread to wait):
    pub fn recv_map(&self) -> Option<StmapItem> {
        self.rx_out.recv().ok()
    }

    pub fn stop(&self) { self.running.store(false, Ordering::Relaxed); }

    fn worker_loop(
        stab: Arc<StabilizationManager>,
        rx_in: Receiver<LiveFrameJob>,
        tx_out: Sender<StmapItem>,
        running: Arc<AtomicBool>,
    ) {
        // --------- GLOBAL CACHE (recomputed on param/lens changes) ---------
        // filename_base mirrors generate_stmaps()
        let filename_base = {
            let lens = stab.lens.read();
            format!("{}-{}-{}-{}",
                crate::filesystem::get_filename(&stab.input_file.read().url),
                lens.camera_brand, lens.camera_model, lens.lens_model
            )
            .replace("/", "-").replace("\\", "-").replace(":", "-")
            .replace("+", "-").replace("'", "-").replace("\"", "-")
            .replace(" ", "-")
        };

        // Precompute kernel flags once (direction may change if params change; watch for that if needed)
        let mut kernel_flags = KernelParamsFlags::empty();
        {
            let p = ComputeParams::from_manager(&stab);
            kernel_flags.set(KernelParamsFlags::HAS_DIGITAL_LENS, p.digital_lens.is_some());
            kernel_flags.set(KernelParamsFlags::HORIZONTAL_RS, p.frame_readout_direction.is_horizontal());
        }

        // Optional: remember last hash of params/lens to refresh cache when needed
        let mut last_params_fingerprint: Option<u64> = None;

        while running.load(Ordering::Relaxed) {
            let job = match rx_in.recv_timeout(Duration::from_millis(10)) {
                Ok(j) => j,
                Err(crossbeam_channel::RecvTimeoutError::Timeout) => continue,
                Err(_) => break,
            };

            // ComputeParams fresh per job, similar to generate_stmaps()
            let mut compute_params = ComputeParams::from_manager(&stab);
            compute_params.adaptive_zoom_window = -1.0;
            compute_params.frame_count = 1; // live: one frame
            compute_params.keyframes.clear();
            compute_params.suppress_rotation = true;
            compute_params.fov_algorithm_margin = 0.0;
            compute_params.fovs.clear();
            compute_params.minimal_fovs.clear();

            // Invalidate global bits if params changed (optional hash)
            let this_fingerprint = Self::fingerprint_params(&compute_params);
            if last_params_fingerprint != Some(this_fingerprint) {
                debug!("stmaps_live: params/lens changed → refresh cached globals");
                // If you need to rebuild bigger globals, do it here.
                last_params_fingerprint = Some(this_fingerprint);
            }

            // Build maps for one frame @ live timestamp.
            match Self::build_maps_for_frame_live(
                &stab,
                compute_params,
                kernel_flags,
                &filename_base,
                job.frame_index,
                job.frame_ts_ms,
            ) {
                Ok(item) => {
                    // keep out queue bounded: drop oldest if full
                    if let Err(TrySendError::Full(_)) = tx_out.try_send(item) {
                        warn!("stmaps_live: output queue full; dropping oldest result");
                        let _ = tx_out.recv();
                        let _ = tx_out.try_send(
                            // we must rebuild or keep a copy; we have the item in scope above, so:
                            Self::build_maps_for_frame_live(
                                &stab, ComputeParams::from_manager(&stab), kernel_flags,
                                &filename_base, job.frame_index, job.frame_ts_ms
                            ).unwrap_or_else(|_| (String::new(), job.frame_index, vec![], vec![]))
                        );
                    }
                }
                Err(e) => {
                    warn!("stmaps_live: failed to build maps for frame {} ts={:.3}ms: {e:?}",
                          job.frame_index, job.frame_ts_ms);
                    // You may still send a placeholder so the renderer does not stall:
                    let _ = tx_out.try_send((filename_base.clone(), job.frame_index, vec![], vec![]));
                }
            }
        }

        info!("stmaps_live: worker exit");
    }

    #[inline]
    fn fingerprint_params(p: &ComputeParams) -> u64 {
        // Minimal fingerprint; extend with lens id, rs direction, etc.
        // (Or use a real hasher on the relevant fields)
        let mut h = 0xcbf29ce484222325u64;
        h ^= (p.width as u64) ^ (p.height as u64) ^ (p.scaled_fps.to_bits() as u64);
        h
    }

    /// This is the single-frame worker; it mirrors your generate_stmaps body, parameterized by timestamp_ms.
    fn build_maps_for_frame_live(
        stab: &StabilizationManager,
        mut compute_params: ComputeParams,
        kernel_flags: KernelParamsFlags,
        filename_base: &str,
        frame: usize,
        timestamp_ms: f64,
    ) -> Result<StmapItem, anyhow::Error> {
        let (width, height) = {
            let params = stab.params.read();
            (params.size.0, params.size.1)
        };

        // PASS 1 — identical to generate_stmaps:
        let org_output_size = (width, height);
        compute_params.fov_scale = 1.0;
        compute_params.width              = width;  compute_params.height              = height;
        compute_params.output_width       = width;  compute_params.output_height       = height;

        let mut transform = FrameTransform::at_timestamp(&compute_params, timestamp_ms, frame);
        transform.kernel_params.width  = width as i32;
        transform.kernel_params.height = height as i32;
        transform.kernel_params.output_width  = width as i32;
        transform.kernel_params.output_height = height as i32;
        transform.kernel_params.flags = kernel_flags.bits();

        let mesh_data = transform.mesh_data.iter().map(|x| *x as f64).collect::<Vec<f64>>();

        let bbox = fov_iterative::FovIterative::new(&compute_params, org_output_size)
            .points_around_rect(width as f32, height as f32, 31, 31);

        let (camera_matrix, distortion_coeffs, _p, rotations, is, mesh) =
            FrameTransform::at_timestamp_for_points(&compute_params, &bbox, timestamp_ms, Some(frame), false);

        let undistorted_bbox = undistort_points(
            &bbox, camera_matrix, &distortion_coeffs, rotations[0], None, Some(rotations),
            &compute_params, 1.0, timestamp_ms, is, mesh
        );

        let mut min_x = 0.0; let mut min_y = 0.0; let mut max_x = 0.0; let mut max_y = 0.0;
        for (x, y) in undistorted_bbox {
            min_x = x.min(min_x); min_y = y.min(min_y);
            max_x = x.max(max_x); max_y = y.max(max_y);
        }
        let new_width  = (max_x - min_x).ceil() as usize;
        let new_height = (max_y - min_y).ceil() as usize;

        compute_params.fov_scale = (new_width as f32 / width as f32)
            .max(new_height as f32 / height as f32) as f64;
        compute_params.width              = new_width;  compute_params.height              = new_height;
        compute_params.output_width       = new_width;  compute_params.output_height       = new_height;

        // PASS 2 — recompute with updated fov_scale:
        transform = FrameTransform::at_timestamp(&compute_params, timestamp_ms, frame);
        transform.kernel_params.width  = new_width as i32;
        transform.kernel_params.height = new_height as i32;
        transform.kernel_params.output_width  = new_width as i32;
        transform.kernel_params.output_height = new_height as i32;
        transform.kernel_params.flags = kernel_flags.bits();

        let r_limit_sq = transform.kernel_params.r_limit * transform.kernel_params.r_limit;

        // undist
        let mesh_data2 = transform.mesh_data.iter().map(|x| *x as f64).collect::<Vec<f64>>();
        let undist = parallel_exr(new_width, new_height, |x, y| {
            let mut sy = if compute_params.frame_readout_direction.is_horizontal() {
                (x.round() as i32).min(transform.kernel_params.width).max(0) as usize
            } else {
                (y.round() as i32).min(transform.kernel_params.height).max(0) as usize
            };
            if transform.kernel_params.matrix_count > 1 {
                let idx = transform.kernel_params.matrix_count as usize / 2;
                if let Some(pt) = Stabilization::rotate_and_distort(
                    (x as f32, y as f32), idx, &transform.kernel_params, &transform.matrices,
                    &compute_params.distortion_model, compute_params.digital_lens.as_ref(),
                    r_limit_sq, &mesh_data2
                ) {
                    if compute_params.frame_readout_direction.is_horizontal() {
                        sy = (pt.0.round() as i32).min(transform.kernel_params.width).max(0) as usize;
                    } else {
                        sy = (pt.1.round() as i32).min(transform.kernel_params.height).max(0) as usize;
                    }
                }
            }
            let idx = sy.min(transform.kernel_params.matrix_count as usize - 1);
            Stabilization::rotate_and_distort(
                (x as f32, y as f32), idx, &transform.kernel_params, &transform.matrices,
                &compute_params.distortion_model, compute_params.digital_lens.as_ref(),
                r_limit_sq, &mesh_data2
            )
        });

        // dist
        compute_params.width        = width;  compute_params.height        = height;
        compute_params.output_width = width;  compute_params.output_height = height;

        let dist = parallel_exr(width, height, |x, y| {
            let distorted = [(x as f32, y as f32)];
            let (camera_matrix, distortion_coeffs, _p, rotations, is, mesh) =
                FrameTransform::at_timestamp_for_points(&compute_params, &distorted, timestamp_ms, Some(frame), true);
            undistort_points(
                &distorted, camera_matrix, &distortion_coeffs, rotations[0], None, Some(rotations),
                &compute_params, 1.0, timestamp_ms, is, mesh
            ).first().copied()
        });

        Ok((filename_base.to_string(), frame, dist, undist))
    }
}
