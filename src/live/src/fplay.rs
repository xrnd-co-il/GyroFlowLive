use anyhow::{anyhow, bail, Result};
use std::io::Write;
use std::net::{TcpStream, Shutdown};
use std::process::{Command, Stdio};
use std::sync::{Mutex, OnceLock};
use std::sync::atomic::{AtomicBool, Ordering};

use crate::live_pix_fmt::PixelFormat;

impl PixelFormat {
    fn ffmpeg_name(self) -> &'static str {
        match self {
            PixelFormat::Rgb24 => "rgb24",
            PixelFormat::Rgba  => "rgba",
            PixelFormat::Nv12  => "nv12", // mapped but not used
        }
    }

    fn bytes_per_pixel(self) -> usize {
        match self {
            PixelFormat::Rgb24 => 3,
            PixelFormat::Rgba  => 4,
            PixelFormat::Nv12  => 0, // not supported for rawvideo
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FProps {
    pub width: u32,
    pub height: u32,
    pub fps: f64,
    pub pixel_format: PixelFormat,
}

struct VideoPlayer {
    props: FProps,
    socket: TcpStream,

    // NEW FIELDS:
    started: bool,
    min_buffered_frames: usize,
    buffer: Vec<u8>,
}

// GLOBAL SETTINGS
static REQUIRE_MIN_FRAMES: AtomicBool = AtomicBool::new(true);

pub fn set_require_min_frames(enabled: bool) {
    REQUIRE_MIN_FRAMES.store(enabled, Ordering::Relaxed);
}

const PORT: u16 = 5000;

static PLAYER: OnceLock<Mutex<Option<VideoPlayer>>> = OnceLock::new();
fn slot() -> &'static Mutex<Option<VideoPlayer>> {
    PLAYER.get_or_init(|| Mutex::new(None))
}

pub fn init_ffplay(width: u32, height: u32, fps: f64, pixel_format: PixelFormat) -> Result<()> {
    println!(
        "Initializing ffplay for {}x{} @ {}fps ({:?})",
        width, height, fps, pixel_format
    );

    let mut guard = slot().lock().unwrap();
    if let Some(p) = guard.as_ref() {
        let want = FProps { width, height, fps, pixel_format };
        if p.props == want {
            return Ok(());
        }
        bail!(
            "ffplay already initialized with {:?}, requested {:?}",
            p.props,
            want
        );
    }

    if pixel_format == PixelFormat::Nv12 {
        bail!("init_ffplay: PixelFormat::Nv12 is not supported for rawvideo display");
    }

    let props = FProps { width, height, fps, pixel_format };
    let ffmpeg_pix_fmt = pixel_format.ffmpeg_name();

    // Spawn ffplay in listen mode
    let _child = Command::new("ffplay")
        .args([
            "-loglevel", "error", "-autoexit",
            "-f", "rawvideo",
            "-pixel_format", ffmpeg_pix_fmt,
            "-video_size", &format!("{}x{}", width, height),
            "-framerate", &fps.to_string(),
            &format!("tcp://127.0.0.1:{}?listen=1", PORT),
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()?;

    // Connect our TCP sender to ffplay
    let socket = TcpStream::connect(("127.0.0.1", PORT))?;
    socket.set_nodelay(true).ok();

    *guard = Some(VideoPlayer {
        props,
        socket,
        started: true,
        min_buffered_frames: 1, // your requested threshold
        buffer: Vec::new(),
    });

    Ok(())
}

/// Generic push that supports RGB24 and RGBA.
/// Now with frame prebuffering.
pub fn push_frame(bytes: &[u8]) -> Result<()> {
    let mut guard = slot().lock().unwrap();
    let p = guard.as_mut().ok_or_else(|| anyhow!("ffplay not initialized"))?;
    let bpp = p.props.pixel_format.bytes_per_pixel();
    if bpp == 0 {
        bail!("push_frame: pixel format {:?} is not supported here", p.props.pixel_format);
    }

    let frame_size = p.props.width as usize * p.props.height as usize * bpp;

    if bytes.len() != frame_size {
        bail!(
            "frame size mismatch: got {}, expected {} ({}x{} {:?})",
            bytes.len(),
            frame_size,
            p.props.width,
            p.props.height,
            p.props.pixel_format,
        );
    }

    let require_min = REQUIRE_MIN_FRAMES.load(Ordering::Relaxed);

    // -----------------------------
    // PREBUFFER MODE
    // -----------------------------
    if require_min && !p.started {
        
        if p.buffer.is_empty() {
            p.buffer.reserve(frame_size * p.min_buffered_frames);
        }

        p.buffer.extend_from_slice(bytes);

        let buffered_frames = p.buffer.len() / frame_size;

        if buffered_frames < p.min_buffered_frames {
            // Still buffering —
            println!("Buffering frames for ffplay: {}/{}", buffered_frames, p.min_buffered_frames);
           
        }else{
            // We now have enough → FLUSH BUFFER and START playback
            p.socket.write_all(&p.buffer)?;
            p.buffer.clear();
            p.started = true;
        }

        // We now have enough → FLUSH BUFFER and START playback
        return Ok(());
    }

    // -----------------------------
    // NORMAL STREAMING MODE
    // -----------------------------
    p.socket.write_all(bytes);
    Ok(())
}

pub fn shutdown_ffplay() {
    let mut guard = slot().lock().unwrap();
    if let Some(p) = guard.take() {
        let _ = p.socket.shutdown(Shutdown::Both);
    }
}
