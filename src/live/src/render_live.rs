
use gyroflow_core::gpu::{BufferDescription, Buffers, BufferSource};
use crossbeam_channel::{Receiver, RecvTimeoutError};
use log::{debug, info, warn, trace};
use std::time::{Duration, Instant};
use once_cell::sync::OnceCell;
use gyroflow_core::StabilizationManager;
use crate::live_pix_fmt::{LiveFrame, LivePixFmt};
use gyroflow_core::stmap_live::StmapItem;
use crate::fplay;
use crate::Arc;
use crate::render_map_kind::{render_with_maps_to_rgb24, RenderMapKind};
use gyroflow_core::stabilization::pixel_formats::{RGB8};

#[derive(Clone, Copy)]
pub struct LiveRenderConfig {
    pub wait_for_map_timeout: Duration,
    pub trim_before_idx: bool,
    pub present_fps: u32,
}

impl Default for LiveRenderConfig {
    fn default() -> Self {
        Self {
            wait_for_map_timeout: Duration::from_millis(8),
            trim_before_idx: true,
            present_fps: 30,
        }
    }
}

struct MapCache {
    start_idx: usize,
    buf: Vec<Option<(Vec<u8>, Vec<u8>)>>,
}

impl MapCache {
    fn new() -> Self { Self { start_idx: 0, buf: Vec::new() } }
    fn insert(&mut self, idx: usize, dist: Vec<u8>, undist: Vec<u8>) {
        if idx < self.start_idx { return; }
        let pos = idx - self.start_idx;
        if pos >= self.buf.len() { self.buf.resize(pos + 1, None); }
        self.buf[pos] = Some((dist, undist));
    }
    fn take(&mut self, idx: usize) -> Option<(Vec<u8>, Vec<u8>)> {
        if idx < self.start_idx { return None; }
        let pos = idx - self.start_idx;
        if pos >= self.buf.len() { return None; }
        self.buf[pos].take()
    }
    fn trim_before(&mut self, keep_from: usize) {
        if keep_from <= self.start_idx { return; }
        let to_drop = (keep_from - self.start_idx).min(self.buf.len());
        if to_drop > 0 {
            self.buf.drain(0..to_drop);
            self.start_idx = keep_from;
        }
    }
}

fn identity_map_fallback(_w: u32, _h: u32) -> Option<(Vec<u8>, Vec<u8>)> { None }

fn drain_maps_until(
    maps_rx: &Receiver<StmapItem>,
    cache: &mut MapCache,
    wanted_idx: usize,
    deadline: Instant,
) -> Option<(Vec<u8>, Vec<u8>)> {
    loop {
        if Instant::now() >= deadline { return None; }
        let left = deadline.saturating_duration_since(Instant::now());
        match maps_rx.recv_timeout(left) {
            Ok((_fname, idx, dist, undist)) => {
                if idx == wanted_idx { return Some((dist, undist)); }
                cache.insert(idx, dist, undist);
            }
            Err(RecvTimeoutError::Timeout) => return None,
            Err(RecvTimeoutError::Disconnected) => return None,
        }
    }
}

fn checksum(buf: &[u8]) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    buf.hash(&mut h);
    h.finish()
}

pub fn render_live_loop(
    frames_rx: Receiver<(usize, LiveFrame)>,
    stab_man: Arc<StabilizationManager>,
    cfg: LiveRenderConfig,
) {
    let _fplay_instance = fplay::init_ffplay(1280, 720, cfg.present_fps).unwrap();

    println!("render_live: start");
    let mut initialized = false;
    while let Ok((_frame_idx, frame)) = frames_rx.recv() {
        // 1) Get basic info
        let (w, h) = frame.get_size();
        let ts_us = frame.ts_us();
        let input_rgb = frame.as_rgb24();
        let mut input_rgb_vec = input_rgb.to_vec();

       if !initialized {
        stab_man.set_render_params((w as usize, h as usize), (w as usize, h as usize));
        log::info!("Live stabilization initialized for {}x{}", w, h);
        initialized = true;
    }


        // Sanity check on size (defensive)
        if input_rgb.len() != (w as usize) * (h as usize) * 3 {
            eprintln!(
                "render_live: bad buffer size: got {}, expected {}",
                input_rgb.len(),
                (w as usize) * (h as usize) * 3
            );
            continue;
        }

        

        // 2) Allocate output buffer (RGB24)
        let mut output_rgb = vec![0u8; (w as usize) * (h as usize) * 3];

        let in_before  = checksum(&input_rgb_vec);
        let out_before = checksum(&output_rgb);

        // 3) Wrap into Buffers
        let mut buffers = buffers_from_live_frame(&frame, input_rgb_vec.as_mut_slice(), &mut output_rgb);
        
        

        // 4) Stabilize this single frame (no explicit frame index → None)
        match stab_man.process_pixels::<RGB8>(ts_us, None, &mut buffers) {
            Ok(info) => {
                let out_after = checksum(&output_rgb);

                println!("backend used: {}", info.backend);
                println!("output fov: {}", info.fov);
                println!("minimal fov: {}", info.minimal_fov);

                // 5) Push stabilized frame to player
                if let Err(e) = fplay::push_rgb24(&output_rgb) {
                    //eprintln!("fplay::push_rgb24 failed: {e:?}");
                }
            }
            Err(e) => {
                eprintln!("Stabilization failed at ts_us={ts_us}: {e:?}");
                continue;
            }
        }
    }

    log::info!("render_live: exit");
}


fn wait_for_map_blocking(
    next_idx: usize,
    maps_rx: &Receiver<StmapItem>,
    cache: &mut MapCache,
) -> Option<(Vec<u8>, Vec<u8>)> {
    // Fast path: already cached?
    if let Some(pair) = cache.take(next_idx) {
        return Some(pair);
    }

    // Block until we get the exact map; cache out-of-order ones.
    loop {
        match maps_rx.recv() {
            Ok((_fname, idx, dist, undist)) => {
                if idx == next_idx {
                    return Some((dist, undist));
                } else {
                    cache.insert(idx, dist, undist);
                }
            }
            Err(_) => {
                // Sender disconnected → no more maps will arrive
                return None;
            }
        }
    }


}

    
fn buffers_from_live_frame<'a>(
    frame: &'a LiveFrame,
    input_rgb: &'a mut [u8],
    output_rgb: &'a mut [u8],
) -> Buffers<'a> {
    let (w, h) = frame.get_size();
    let w_usize = w as usize;
    let h_usize = h as usize;
    let stride = w_usize * 3; // RGB24: 3 bytes per pixel


    let src = frame.as_rgb24();          // &[u8] or something similar
    input_rgb[..src.len()].copy_from_slice(src);

    let input_desc = BufferDescription {
        size: (w_usize, h_usize, stride),
        rect: None,
        rotation: None,
        data: BufferSource::Cpu {
            // type will be something like &'a [u8]
            buffer: input_rgb,
        },
        texture_copy: false,
    };

    let output_desc = BufferDescription {
        size: (w_usize, h_usize, stride),
        rect: None,
        rotation: None,
        data: BufferSource::Cpu {
            // type will be something like &'a mut [u8]
            buffer: output_rgb,
        },
        texture_copy: false,
    };

    Buffers {
        input: input_desc,
        output: output_desc,
    }
}
