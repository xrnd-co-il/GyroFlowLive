use anyhow::{Context, Result};
use crossbeam_channel::{unbounded, Sender};
use ffmpeg_next as ffmpeg;
use std::thread;
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy)]
pub enum PixelFormat {
    Yuv420p,
    Nv12,
    Rgba,
    Other(ffmpeg::format::pixel::Pixel),
}

#[derive(Debug)]
pub struct VideoFrame {
    pub ts_ns: i64,       // monotonic nanoseconds
    pub width: u32,
    pub height: u32,
    pub pix_fmt: PixelFormat,
    /// Packed buffer. For planar formats we pack by rows per plane (simple & safe).
    pub data: Vec<u8>,
    /// Linesize in bytes per row of the packed buffer (for consumer’s convenience).
    pub stride: usize,
}

fn pix_fmt_map(p: ffmpeg::format::pixel::Pixel) -> PixelFormat {
    use ffmpeg::format::pixel::Pixel::*;
    match p {
        YUV420P => PixelFormat::Yuv420p,
        NV12 => PixelFormat::Nv12,
        RGBA => PixelFormat::Rgba,
        other => PixelFormat::Other(other),
    }
}

/// Pack an FFmpeg frame into a simple contiguous buffer (CPU; easiest to start).
fn pack_frame(frame: &ffmpeg::util::frame::video::Video) -> (Vec<u8>, usize) {
    let w = frame.width() as usize;
    let h = frame.height() as usize;
    let pix = frame.format();
    unsafe {
        match pix {
            ffmpeg::format::pixel::Pixel::YUV420P => {
                // 3 planes: Y (WxH), U (W/2 x H/2), V (W/2 x H/2)
                let y_stride = frame.stride(0) as usize;
                let u_stride = frame.stride(1) as usize;
                let v_stride = frame.stride(2) as usize;
                let y_plane = std::slice::from_raw_parts(frame.data(0), y_stride * h);
                let u_plane = std::slice::from_raw_parts(frame.data(1), u_stride * (h/2));
                let v_plane = std::slice::from_raw_parts(frame.data(2), v_stride * (h/2));

                // Pack tightly: Y, then U, then V
                let mut out = Vec::with_capacity(w*h + (w*h)/2);
                // Y
                for row in 0..h {
                    let start = row * y_stride;
                    out.extend_from_slice(&y_plane[start..start+w]);
                }
                // U
                for row in 0..(h/2) {
                    let start = row * u_stride;
                    out.extend_from_slice(&u_plane[start..start+(w/2)]);
                }
                // V
                for row in 0..(h/2) {
                    let start = row * v_stride;
                    out.extend_from_slice(&v_plane[start..start+(w/2)]);
                }
                (out, w) // stride for the Y plane; consumer knows UV are subsampled
            }
            ffmpeg::format::pixel::Pixel::NV12 => {
                // 2 planes: Y (WxH), UV interleaved (W x H/2)
                let y_stride = frame.stride(0) as usize;
                let uv_stride = frame.stride(1) as usize;
                let y_plane = std::slice::from_raw_parts(frame.data(0), y_stride * h);
                let uv_plane = std::slice::from_raw_parts(frame.data(1), uv_stride * (h/2));
                let mut out = Vec::with_capacity(w*h + w*(h/2));
                for row in 0..h {
                    let start = row * y_stride;
                    out.extend_from_slice(&y_plane[start..start+w]);
                }
                for row in 0..(h/2) {
                    let start = row * uv_stride;
                    out.extend_from_slice(&uv_plane[start..start+w]);
                }
                (out, w)
            }
            _ => {
                // Fallback: copy first plane tightly for demonstration
                let stride = frame.stride(0) as usize;
                let buf = std::slice::from_raw_parts(frame.data(0), stride * h).to_vec();
                (buf, stride)
            }
        }
    }
}

/// Convert stream PTS to monotonic nanoseconds using the stream's time_base.
/// Falls back to a wall-clock if PTS is missing (rare).
fn pts_to_ns(pts: Option<i64>, time_base: ffmpeg::Rational, start_instant: Instant, frame_idx: u64, fps_hint: Option<f64>) -> i64 {
    if let Some(p) = pts {
        // ns = pts * (time_base) * 1e9
        let num = time_base.numerator() as i128;
        let den = time_base.denominator() as i128;
        let p128 = p as i128;
        let ns = p128 * num * 1_000_000_000i128 / den;
        return ns as i64;
    }
    // Fallback: wall clock from start
    if let Some(fps) = fps_hint {
        let dur = Duration::from_secs_f64((frame_idx as f64) / fps);
        let ns = start_instant.elapsed()
            .saturating_sub(start_instant.elapsed().saturating_sub(dur))
            .as_nanos() as i64;
        return ns;
    }
    start_instant.elapsed().as_nanos() as i64
}

/// Spawn a decoding thread that pushes VideoFrame to tx.
pub fn spawn_live_reader(url: String, tx: Sender<VideoFrame>) -> Result<thread::JoinHandle<()>> {
    ffmpeg::init().context("ffmpeg init failed")?;

    let handle = thread::spawn(move || {
        if let Err(e) = run_decode_loop(&url, &tx) {
            eprintln!("[reader] error: {e:?}");
        }
    });
    Ok(handle)
}

fn run_decode_loop(url: &str, tx: &Sender<VideoFrame>) -> Result<()> {
    let mut ictx = ffmpeg::format::input(&url)
        .with_context(|| format!("open input failed: {url}"))?;

    // Find best video stream
    let input = ictx.streams();
    let (video_idx, vstream) = input
        .best(ffmpeg::media::Type::Video)
        .context("no video stream")?
        .index()
        .pipe(|idx| (idx, &input[idx]));

    let time_base = vstream.time_base();
    let fps_hint = vstream.rate().map(|r| r.0 as f64 / r.1 as f64).filter(|f| *f > 0.1);

    let ctx_decoder = ffmpeg::codec::context::Context::from_parameters(vstream.parameters())?;
    let mut decoder = ctx_decoder.decoder().video()?;

    // Low-latency flags (helpful for live)
    decoder.set_threading(ffmpeg::codec::threading::Config {
        kind: ffmpeg::codec::threading::Type::Frame,
        count: 0, // auto
        safe: true,
    });

    let mut frame = ffmpeg::util::frame::video::Video::empty();
    let mut packet = ffmpeg::Packet::empty();

    let start = Instant::now();
    let mut frame_idx: u64 = 0;

    // Read packets → send to decoder → receive frames
    while ictx.read(&mut packet).is_ok() {
        if packet.stream() == video_idx {
            if let Err(e) = decoder.send_packet(&packet) {
                eprintln!("[decoder] send_packet err: {e:?}");
            }
            loop {
                match decoder.receive_frame(&mut frame) {
                    Ok(_) => {
                        let ts_ns = pts_to_ns(frame.timestamp(), time_base, start, frame_idx, fps_hint);
                        frame_idx += 1;

                        let width = frame.width();
                        let height = frame.height();
                        let pix_fmt = pix_fmt_map(frame.format());

                        let (data, stride) = pack_frame(&frame);

                        let vf = VideoFrame {
                            ts_ns,
                            width,
                            height,
                            pix_fmt,
                            data,
                            stride,
                        };
                        if tx.send(vf).is_err() {
                            // consumer gone
                            return Ok(());
                        }
                    }
                    Err(e) if e == ffmpeg::Error::Again => break, // need more packets
                    Err(e) => {
                        eprintln!("[decoder] receive_frame err: {e:?}");
                        break;
                    }
                }
            }
        }
        packet.unref();
    }

    // Flush
    decoder.send_eof().ok();
    while decoder.receive_frame(&mut frame).is_ok() {
        let ts_ns = pts_to_ns(frame.timestamp(), time_base, start, frame_idx, fps_hint);
        frame_idx += 1;
        let (data, stride) = pack_frame(&frame);
        let vf = VideoFrame {
            ts_ns,
            width: frame.width(),
            height: frame.height(),
            pix_fmt: pix_fmt_map(frame.format()),
            data,
            stride,
        };
        if tx.send(vf).is_err() {
            return Ok(());
        }
    }

    Ok(())
}

fn main() -> Result<()> {
    // Example: RTSP, HTTP MJPEG, or device (platform-specific):
    // let url = "rtsp://user:pass@192.168.1.10:554/stream1";
    // let url = "/dev/video0"; // on Linux V4L2, when FFmpeg is built with v4l2
    let url = std::env::args().nth(1).expect("usage: live_capture <url>");

    let (tx, rx) = unbounded();
    let _h = spawn_live_reader(url, tx)?;

    // Demo consumer: print timestamps
    while let Ok(f) = rx.recv() {
        println!("frame {}x{} ts={} ns fmt={:?} buf={}B",
            f.width, f.height, f.ts_ns, f.pix_fmt, f.data.len());
        // TODO: pass to stabilizer
    }
    Ok(())
}