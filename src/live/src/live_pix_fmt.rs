// Cargo.toml (key deps)
// [dependencies]
// anyhow = "1"
// crossbeam-channel = "0.5"
// ffmpeg-next = { version = "6", features = ["build"] }   // or your pinned version
// thiserror = "1"

use anyhow::{Context, Result};
use crossbeam_channel::Sender;
use ffmpeg_next as ffmpeg;
use ffmpeg::codec::context::Context as CodecContext;
use ffmpeg::codec::decoder::Video as VideoDecoder;
use ffmpeg::format::{self, context::Input};
use ffmpeg::frame;
use ffmpeg::software::scaling::{context::Context as Scaler, flag::Flags};
use ffmpeg::util::format::Pixel;
use std::time::Instant;
use ffmpeg_next::Dictionary;
use ffmpeg::util::rational::Rational;
use ffmpeg_next::Rescale;
use gyroflow_core::stmap_live::StmapsLive;
use std::sync::Arc;

#[derive(Clone, PartialEq)]
pub enum LivePixFmt { Rgb24 = 0, Nv12 = 1 }

pub struct LiveFrame {
    pub ts_us: i64,          // presentation timestamp in microseconds
    pub width: u32,
    pub height: u32,
    pub pix_fmt: LivePixFmt, // matches the bytes layout
    pub data: Vec<u8>,       // tightly packed (RGB), or planar/semi-planar for NV12
}

impl LiveFrame {
    pub fn get_size(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    pub fn ts_us(&self) -> i64 {
        self.ts_us
    }

    pub fn as_rgb24(&self) -> &[u8] {
        assert!(self.pix_fmt == LivePixFmt::Rgb24, "expected RGB24 frame");
        &self.data
    }

    pub fn as_rgb24_mut(&mut self) -> &mut [u8] {
        assert!(self.pix_fmt == LivePixFmt::Rgb24, "expected RGB24 frame");
        &mut self.data
    }

    pub fn make_cpu_rgb24_buffer(&self) -> (&[u8], u32, u32) {
        assert!(self.pix_fmt == LivePixFmt::Rgb24, "expected RGB24 frame");
        (&self.data, self.width, self.height)
    }


}

pub fn spawn_stream_reader(
    url: &str,
    out_tx: Sender<(usize, LiveFrame)>,
    prefer_nv12: LivePixFmt,       // true: NV12, false: RGB24
    max_queue_warn: usize,   // for basic health logs
    //st_live: Arc<StmapsLive>
) -> Result<std::thread::JoinHandle<()>> {
    ffmpeg::init().context("ffmpeg init failed")?;

    let url_owned = url.to_string();
    let handle = std::thread::Builder::new()
        .name("stream_reader".into())
        .spawn(move || {
            if let Err(e) = run_reader(&url_owned, &out_tx, prefer_nv12, max_queue_warn, /*st_live.clone()*/) {
                eprintln!("[stream_reader] fatal error: {e:?}");
            }
        })?;

    Ok(handle)
}

fn run_reader(
    url: &str,
    out_tx: &Sender<(usize, LiveFrame)>,
    prefer_nv12: LivePixFmt,
    max_queue_warn: usize,
    //st_live: Arc<StmapsLive>
) -> Result<()> {
    println!("Starting stream reader for URL: {}", url);
    // --- 1) Open input (with a few helpful options for live sources) ---
    let mut options = Dictionary::new();
    // Lower initial latency and stabilize probing for live streams:
    options.set("rtsp_transport", "tcp");          // for RTSP; ignored otherwise
    options.set("stimeout", "5000000");            // 5s (microseconds) conn/IO timeout where supported
    options.set("rw_timeout", "5000000");          // another variant some demuxers honor
    options.set("max_delay", "500000");            // 0.5s
    options.set("fflags", "nobuffer");             // lower buffering for live
    options.set("probesize", "5000000");           // keep reasonable probe
    options.set("analyzeduration", "5000000");

    let mut ictx = format::input_with_dictionary(url, options)
    .with_context(|| format!("open url: {url}"))?;

    // Validate → if this fails, check URL / credentials / firewall. If ok → proceed.

    // --- 2) Find best video stream & create decoder ---
    let (v_stream_idx, v_stream) = ictx
        .streams()
        .best(ffmpeg::media::Type::Video)
        .map(|s| (s.index(), s))
        .context("no video stream in input")?;

    let codec_params = v_stream.parameters();
    let decoder_codec = ffmpeg::codec::decoder::find(codec_params.id())
        .context("decoder not found for stream codec")?;
    let mut decoder_ctx = CodecContext::from_parameters(codec_params)
        .context("build decoder context from stream params")?;
    // Low-latency decode hint:
    //decoder_ctx.set_flags(Flags::LOW_DELAY);
    let mut decoder = decoder_ctx
        .decoder()
        .video()
        .context("open video decoder")?;

    // Validate → prints width/height/fps
    let (mut src_w, mut src_h) = (decoder.width(), decoder.height());
    let tb: Rational = v_stream.time_base(); // stream time_base for PTS rescale

    // --- 3) Prepare scaler to our target pix_fmt ---
    let target_fmt = if prefer_nv12 == LivePixFmt::Nv12 { Pixel::NV12 } else { Pixel::RGB24 };
    // If width/height unknown yet (some live sources), we’ll rebuild scaler on first frame:
    let mut scaler: Option<(u32, u32, Pixel, Scaler)> = None;

    let mut pkt_count: u64 = 0;
    let t0 = Instant::now();

     let mut frame_index: usize = 0;

    // --- 4) Demux/Decode loop ---
    for (stream, mut packet) in ictx.packets() {
        if stream.index() != v_stream_idx { continue; }
        pkt_count += 1;

        // Push packet to decoder
        if let Err(e) = decoder.send_packet(&packet) {
            eprintln!("[stream_reader] decoder send_packet err: {e:?}");
            continue; // for live we don’t abort; we try to recover on next packet
        }

        // Receive all available frames for this packet
        let mut frame = frame::Video::empty();
        while decoder.receive_frame(&mut frame).is_ok() {
            // Lazily create/update scaler if props changed
            let (w, h, src_fmt) = (frame.width(), frame.height(), frame.format());
            if scaler.as_ref().map(|(sw, sh, sf, _)| (*sw, *sh, *sf)) != Some((w, h, src_fmt)) {
                src_w = w; src_h = h;
                let sc = Scaler::get(src_fmt, w, h, target_fmt, w, h, Flags::BILINEAR)
                    .context("create scaler")?;
                scaler = Some((w, h, src_fmt, sc));
                // Validate → got scaler for (w,h,src_fmt)->target_fmt. Proceed.
            }
            let (_, _, _, sc) = scaler.as_mut().unwrap();

            // --- 5) Convert frame to target pixel format (RGB24/NV12) ---
            let mut out = frame::Video::empty();
            out.set_format(target_fmt);
            out.set_width(w);
            out.set_height(h);
            sc.run(&frame, &mut out).context("scale/run")?;

            // --- 6) Extract tightly-packed bytes for channel ---
            let (bytes, pix) = if target_fmt == Pixel::RGB24 {
                // One plane, stride = width*3
                let mut buf = Vec::with_capacity((w * h * 3) as usize);
                let ls = out.stride(0) as usize;
                let row_bytes = (w * 3) as usize;
                let data = out.data(0);
                for row in 0..h as usize {
                    let start = row * ls;
                    buf.extend_from_slice(&data[start..start + row_bytes]);
                }
                (buf, LivePixFmt::Rgb24)
            } else {
                // NV12: Y plane then interleaved UV plane
                // plane 0: Y (h rows, stride w)
                // plane 1: UV (h/2 rows, stride w)
                let mut buf = Vec::with_capacity((w * h * 3 / 2) as usize);

                let (ls_y, ls_uv) = (out.stride(0) as usize, out.stride(1) as usize);
                let (data_y, data_uv) = (out.data(0), out.data(1));

                // copy Y
                for row in 0..h as usize {
                    let start = row * ls_y;
                    buf.extend_from_slice(&data_y[start..start + w as usize]);
                }
                // copy UV
                for row in 0..(h as usize / 2) {
                    let start = row * ls_uv;
                    buf.extend_from_slice(&data_uv[start..start + w as usize]);
                }
                (buf, LivePixFmt::Nv12)
            };

            // --- 7) Timestamp in microseconds ---
            // Prefer frame.timestamp() (already rescaled by demuxer); else derive from packet pts.
            let ts_us = frame
                .timestamp()
               .unwrap_or_else(|| {
                    let pts = packet.pts().unwrap_or(0);
                    pts.rescale(tb, Rational(1, 1_000_000))  // <-- not a tuple!
               });

            // --- 8) Send to channel (bounded/backpressure) ---
            let msg = LiveFrame { ts_us, width: w, height: h, pix_fmt: pix, data: bytes };
            // Non-blocking send with light drop policy
            if let Err(err) = out_tx.send((frame_index, msg)) {
                // Backpressure: drop newest to keep latency low; you can also drop oldest by clearing once.
                // Simple policy: if full, log every N times and skip this frame. 
                eprintln!("[stream_reader] channel send err: {}", err.to_string());
            }else {
                //st_live.submit_frame(frame_index, ts_us);
                //println!("Sent frame idx {} ts_us {}", frame_index, ts_us);
            }
            frame_index += 1;

            // Validate → consumer sees frames (count increasing, timestamps monotonic). If yes: proceed.
        }
    }

    // Flush decoder at end-of-stream (some live inputs never EOS; omit if unwanted)
    decoder.send_eof().ok();
    let mut frame = frame::Video::empty();
    while decoder.receive_frame(&mut frame).is_ok() {
        // same handling as above (convert & send) if you want a graceful tail
    }

    

    Ok(())
}
