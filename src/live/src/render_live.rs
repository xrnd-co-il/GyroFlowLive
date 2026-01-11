use gyroflow_core::gpu::{BufferDescription, Buffers, BufferSource};
use crossbeam_channel::{Receiver, RecvTimeoutError};
use log::{debug, info, warn, trace};
use std::time::{Duration, Instant};
use once_cell::sync::OnceCell;
use gyroflow_core::StabilizationManager;
use crate::live_pix_fmt::{LiveFrame, PixelFormat};
use gyroflow_core::stmap_live::StmapItem;
use crate::fplay;
use crate::Arc;
use gyroflow_core::stabilization::pixel_formats::{RGB8, RGBA8};

#[derive(Clone, Copy)]
pub struct LiveRenderConfig {
    pub wait_for_map_timeout: Duration,
    pub trim_before_idx: bool,
    pub present_fps: f64,
}

impl Default for LiveRenderConfig {
    fn default() -> Self {
        Self {
            wait_for_map_timeout: Duration::from_millis(8),
            trim_before_idx: true,
            present_fps: 30.0,
        }
    }


    
}

impl LiveRenderConfig {
    pub fn new(present_fps: f64) -> Self {
        Self {
            wait_for_map_timeout: Duration::from_millis(8),
            trim_before_idx: true,
            present_fps: present_fps as f64,
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
    display_pix_fmt: PixelFormat, // <--- new: choose output format (Rgb24 / Rgba)
) {
    println!("render_live: start");
    let mut initialized = false;

    while let Ok((_frame_idx, frame)) = frames_rx.recv() {

        
        let (w, h) = frame.get_size();
        let ts_us = frame.ts_us();
        let ts_ms = ts_us as f64 / 1000.0;
        stab_man.live_on_new_frame(_frame_idx, ts_ms, 1);
        
        // Initialize stab + ffplay once we know the actual frame size
        if !initialized {
            
            stab_man.set_render_params((w as usize, h as usize), (w as usize, h as usize));
            log::info!("Live stabilization initialized for {}x{}", w, h);

            // init ffplay with the chosen display format (Rgb24 or Rgba)
            if let Err(e) = fplay::init_ffplay(w, h, cfg.present_fps, display_pix_fmt) {
                eprintln!("Failed to init ffplay: {e:?}");
                return;
            }

            initialized = true;
        }

        match frame.pix_fmt {
            PixelFormat::Rgb24 => {
                // -------- RGB24 input path --------
                let input_rgb = frame.as_rgb24();
                if input_rgb.len() != (w as usize) * (h as usize) * 3 {
                    eprintln!(
                        "render_live: bad RGB24 buffer size: got {}, expected {}",
                        input_rgb.len(),
                        (w as usize) * (h as usize) * 3
                    );
                    continue;
                }

                let mut input_rgb_vec = input_rgb.to_vec();
                let mut output_rgb = vec![0u8; (w as usize) * (h as usize) * 3];

                let _in_before  = checksum(&input_rgb_vec);
                let _out_before = checksum(&output_rgb);

                let mut buffers = buffers_from_live_frame_rgb24(&frame, input_rgb_vec.as_mut_slice(), &mut output_rgb);

                match stab_man.process_pixels::<RGB8>(ts_us, None, &mut buffers) {
                    Ok(info) => {
                        let _out_after = checksum(&output_rgb);
                        

                        // Decide how to send, based on display_pix_fmt
                        match display_pix_fmt {
                            PixelFormat::Rgb24 => {
                                if let Err(e) = fplay::push_frame(&output_rgb) {
                                    eprintln!("fplay::push_frame failed (RGB24->RGB24): {e:?}");
                                }
                            }
                            PixelFormat::Rgba => {
                                // Convert RGB24 -> RGBA for display
                                let w_usize = w as usize;
                                let h_usize = h as usize;
                                let mut output_rgba = vec![0u8; w_usize * h_usize * 4];

                                for i in 0..(w_usize * h_usize) {
                                    let src = i * 3;
                                    let dst = i * 4;
                                    output_rgba[dst    ] = output_rgb[src    ];
                                    output_rgba[dst + 1] = output_rgb[src + 1];
                                    output_rgba[dst + 2] = output_rgb[src + 2];
                                    output_rgba[dst + 3] = 255;
                                }

                                if let Err(e) = fplay::push_frame(&output_rgba) {
                                    eprintln!("fplay::push_frame failed (RGB24->RGBA): {e:?}");
                                }
                            }
                            PixelFormat::Nv12 => {
                                eprintln!("render_live: display_pix_fmt=NV12 is not supported for ffplay");
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("Stabilization failed at ts_us={ts_us} (RGB24): {e:?}");
                        continue;
                    }
                }
            }

            PixelFormat::Rgba => {
                // -------- RGBA input path --------
                
                let input_rgba = frame.as_rgba();
                if input_rgba.len() != (w as usize) * (h as usize) * 4 {
                    eprintln!(
                        "render_live: bad RGBA buffer size: got {}, expected {}",
                        input_rgba.len(),
                        (w as usize) * (h as usize) * 4
                    );
                    continue;
                }

                let mut input_rgba_vec = input_rgba.to_vec();
                let mut output_rgba = vec![0u8; (w as usize) * (h as usize) * 4];

                let mut buffers = buffers_from_live_frame_rgba(&frame, input_rgba_vec.as_mut_slice(), &mut output_rgba);

                match stab_man.process_pixels::<RGBA8>(ts_us, None, &mut buffers) {
                    Ok(info) => {
                        

                        match display_pix_fmt {
                            PixelFormat::Rgba => {
                                // Already RGBA, send directly
                                if let Err(e) = fplay::push_frame(&output_rgba) {
                                    eprintln!("fplay::push_frame failed (RGBA->RGBA): {e:?}");
                                }
                            }
                            PixelFormat::Rgb24 => {
                                // Convert RGBA -> RGB24 (drop alpha)
                                let w_usize = w as usize;
                                let h_usize = h as usize;
                                let mut output_rgb = vec![0u8; w_usize * h_usize * 3];

                                for i in 0..(w_usize * h_usize) {
                                    let src = i * 4;
                                    let dst = i * 3;
                                    output_rgb[dst    ] = output_rgba[src    ];
                                    output_rgb[dst + 1] = output_rgba[src + 1];
                                    output_rgb[dst + 2] = output_rgba[src + 2];
                                }

                                if let Err(e) = fplay::push_frame(&output_rgb) {
                                    eprintln!("fplay::push_frame failed (RGBA->RGB24): {e:?}");
                                }
                            }
                            PixelFormat::Nv12 => {
                                eprintln!("render_live: display_pix_fmt=NV12 is not supported for ffplay");
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("Stabilization failed at ts_us={ts_us} (RGBA): {e:?}");
                        continue;
                    }
                }
            }

            PixelFormat::Nv12 => {
                eprintln!(
                    "render_live: received NV12 frame ({}x{}), but NV12 is not yet handled in render_live_loop. \
                     Choose Rgb24 or Rgba as stream target format if you want stabilization.",
                    w, h
                );
                continue;
            }
        }
    }

    log::info!("render_live: exit");
    //fplay::shutdown_ffplay();
}

// ------------------------ buffer helpers ------------------------

fn buffers_from_live_frame_rgb24<'a>(
    frame: &'a LiveFrame,
    input_rgb: &'a mut [u8],
    output_rgb: &'a mut [u8],
) -> Buffers<'a> {
    let (w, h) = frame.get_size();
    let w_usize = w as usize;
    let h_usize = h as usize;
    let stride = w_usize * 3; // RGB24: 3 bytes per pixel

    let src = frame.as_rgb24();
    input_rgb[..src.len()].copy_from_slice(src);

    let input_desc = BufferDescription {
        size: (w_usize, h_usize, stride),
        rect: None,
        rotation: None,
        data: BufferSource::Cpu { buffer: input_rgb },
        texture_copy: false,
    };

    let output_desc = BufferDescription {
        size: (w_usize, h_usize, stride),
        rect: None,
        rotation: None,
        data: BufferSource::Cpu { buffer: output_rgb },
        texture_copy: false,
    };

    Buffers { input: input_desc, output: output_desc }
}

fn buffers_from_live_frame_rgba<'a>(
    frame: &'a LiveFrame,
    input_rgba: &'a mut [u8],
    output_rgba: &'a mut [u8],
) -> Buffers<'a> {
    let (w, h) = frame.get_size();
    let w_usize = w as usize;
    let h_usize = h as usize;
    let stride = w_usize * 4; // RGBA: 4 bytes per pixel

    let src = frame.as_rgba();
    input_rgba[..src.len()].copy_from_slice(src);

    let input_desc = BufferDescription {
        size: (w_usize, h_usize, stride),
        rect: None,
        rotation: None,
        data: BufferSource::Cpu { buffer: input_rgba },
        texture_copy: false,
    };

    let output_desc = BufferDescription {
        size: (w_usize, h_usize, stride),
        rect: None,
        rotation: None,
        data: BufferSource::Cpu { buffer: output_rgba },
        texture_copy: false,
    };

    Buffers { input: input_desc, output: output_desc }
}
