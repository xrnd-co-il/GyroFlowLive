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
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PixelFormat {
    Rgb24, // tightly packed 3×u8
    Nv12,  // Y + interleaved UV
    Rgba,  // tightly packed 4×u8 (RGBA32)
}

impl fmt::Display for PixelFormat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PixelFormat::Rgb24 => write!(f, "Rgb24"),
            PixelFormat::Nv12  => write!(f, "Nv12"),
            PixelFormat::Rgba  => write!(f, "Rgba"),
        }
    }
}
// Optional: keep this alias if you still use LivePixFmt elsewhere
pub type LivePixFmt = PixelFormat;

pub struct LiveFrame {
    pub ts_us: i64,          // presentation timestamp in microseconds
    pub width: u32,
    pub height: u32,
    pub pix_fmt: PixelFormat, // <-- use PixelFormat here
    pub data: Vec<u8>,
}

impl LiveFrame {
    pub fn get_size(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    pub fn ts_us(&self) -> i64 { self.ts_us }

    pub fn as_rgb24(&self) -> &[u8] {
        assert!(self.pix_fmt == PixelFormat::Rgb24, "expected RGB24 frame");
        &self.data
    }

    pub fn as_rgb24_mut(&mut self) -> &mut [u8] {
        assert!(self.pix_fmt == PixelFormat::Rgb24, "expected RGB24 frame");
        &mut self.data
    }

    pub fn make_cpu_rgb24_buffer(&self) -> (&[u8], u32, u32) {
        assert!(self.pix_fmt == PixelFormat::Rgb24, "expected RGB24 frame");
        (&self.data, self.width, self.height)
    }

    pub fn as_rgba(&self) -> &[u8] {
        assert!(self.pix_fmt == PixelFormat::Rgba, "expected RGBA frame");
        &self.data
    }

    pub fn as_rgba_mut(&mut self) -> &mut [u8] {
        assert!(self.pix_fmt == PixelFormat::Rgba, "expected RGBA frame");
        &mut self.data
    }
}

pub fn spawn_stream_reader(
    url: &str,
    out_tx: Sender<(usize, LiveFrame)>,
    target_pix_fmt: LivePixFmt,   // which format we want out: Rgb24 / Nv12 / Rgba32
    max_queue_warn: usize,        // for basic health logs
    //st_live: Arc<StmapsLive>
) -> Result<std::thread::JoinHandle<()>> {
    ffmpeg::init().context("ffmpeg init failed")?;

    let url_owned = url.to_string();
    let handle = std::thread::Builder::new()
        .name("stream_reader".into())
        .spawn(move || {
            if let Err(e) = run_reader(&url_owned, &out_tx, target_pix_fmt, max_queue_warn /*, st_live.clone()*/) {
                eprintln!("[stream_reader] fatal error: {e:?}");
            }
        })?;

    Ok(handle)
}

fn run_reader(
    url: &str,
    out_tx: &Sender<(usize, LiveFrame)>,
    target_pix_fmt: LivePixFmt,
    max_queue_warn: usize,
) -> Result<()> 
{
    println!("Starting stream reader for URL: {}", url);

    // --- 1) FFmpeg input options for live streams ---
    let mut options = Dictionary::new();
    options.set("rtsp_transport", "tcp");
    options.set("stimeout", "5000000");
    options.set("rw_timeout", "5000000");
    options.set("max_delay", "500000");
    options.set("fflags", "nobuffer");
    options.set("probesize", "5000000");
    options.set("analyzeduration", "5000000");

    let mut ictx = format::input_with_dictionary(url, options)
        .with_context(|| format!("open url: {url}"))?;

    // --- 2) Find best video stream ---
    let (v_stream_idx, v_stream) = ictx
        .streams()
        .best(ffmpeg::media::Type::Video)
        .map(|s| (s.index(), s))
        .context("no video stream in input")?;

    let codec_params = v_stream.parameters();
    let decoder_codec = ffmpeg::codec::decoder::find(codec_params.id())
        .context("decoder not found for stream codec")?;
    let mut decoder_ctx = CodecContext::from_parameters(codec_params)
        .context("build decoder context")?;
    let mut decoder = decoder_ctx.decoder().video()
        .context("open video decoder")?;

    let tb = v_stream.time_base();
    let mut frame_index: usize = 0;

    // --- 3) Choose target pixel format ---
    let target_fmt = match target_pix_fmt {
        LivePixFmt::Rgb24  => Pixel::RGB24,
        LivePixFmt::Nv12   => Pixel::NV12,
        LivePixFmt::Rgba => Pixel::RGBA,
    };

    let mut scaler: Option<(u32, u32, Pixel, Scaler)> = None;

    // --- 4) Demux/Decode loop ---
    for (stream, mut packet) in ictx.packets() {
        if stream.index() != v_stream_idx { continue; }

        if decoder.send_packet(&packet).is_err() {
            continue;
        }

        let mut frame = frame::Video::empty();
        while decoder.receive_frame(&mut frame).is_ok() {

            // Lazily rebuild scaler if needed
            let (w, h, src_fmt) = (frame.width(), frame.height(), frame.format());
            if scaler.as_ref().map(|(sw, sh, sf, _)| (*sw, *sh, *sf))
                != Some((w, h, src_fmt)) 
            {
                let sc = Scaler::get(src_fmt, w, h, target_fmt, w, h, Flags::BILINEAR)
                    .context("create scaler")?;
                scaler = Some((w, h, src_fmt, sc));
            }

            let (_, _, _, sc) = scaler.as_mut().unwrap();

            // --- 5) Convert to target pixel format ---
            let mut out = frame::Video::empty();
            out.set_format(target_fmt);
            out.set_width(w);
            out.set_height(h);
            sc.run(&frame, &mut out).context("scale/run")?;

            // --- 6) Extract tightly-packed bytes ---
            let (bytes, pix_fmt) = match target_fmt {
                Pixel::RGB24 => {
                    let mut buf = Vec::with_capacity((w * h * 3) as usize);
                    let ls = out.stride(0) as usize;
                    let row_bytes = (w as usize) * 3;
                    let data = out.data(0);

                    for row in 0..h as usize {
                        let start = row * ls;
                        buf.extend_from_slice(&data[start..start + row_bytes]);
                    }
                    (buf, LivePixFmt::Rgb24)
                }

                Pixel::RGBA => {
                    let mut buf = Vec::with_capacity((w * h * 4) as usize);
                    let ls = out.stride(0) as usize;
                    let row_bytes = (w as usize) * 4;
                    let data = out.data(0);

                    for row in 0..h as usize {
                        let start = row * ls;
                        buf.extend_from_slice(&data[start..start + row_bytes]);
                    }
                    (buf, LivePixFmt::Rgba)
                }

                Pixel::NV12 => {
                    let mut buf = Vec::with_capacity((w * h * 3 / 2) as usize);

                    let ls_y = out.stride(0) as usize;
                    let ls_uv = out.stride(1) as usize;
                    let data_y = out.data(0);
                    let data_uv = out.data(1);

                    // copy Y plane
                    for row in 0..h as usize {
                        let start = row * ls_y;
                        buf.extend_from_slice(&data_y[start..start + w as usize]);
                    }

                    // copy UV plane
                    for row in 0..(h as usize / 2) {
                        let start = row * ls_uv;
                        buf.extend_from_slice(&data_uv[start..start + w as usize]);
                    }

                    (buf, LivePixFmt::Nv12)
                }

                _ => panic!("Unsupported output pixel format"),
            };

            // --- 7) Timestamp ---
            let ts_us = frame.timestamp().unwrap_or_else(|| {
                let pts = packet.pts().unwrap_or(0);
                pts.rescale(tb, ffmpeg::util::rational::Rational(1, 1_000_000))
            });

            // --- 8) Send the frame to the consumer ---
            let msg = LiveFrame {
                ts_us,
                width: w,
                height: h,
                pix_fmt,
                data: bytes,
            };

            if let Err(err) = out_tx.send((frame_index, msg)) {
                eprintln!("[stream_reader] channel send err: {}", err);
            }

            frame_index += 1;
        }
    }

    // --- Optional: Flush decoder ---
    decoder.send_eof().ok();
    while decoder.receive_frame(&mut frame::Video::empty()).is_ok() {}

    Ok(())
}

