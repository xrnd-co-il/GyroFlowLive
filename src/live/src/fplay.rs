use anyhow::{anyhow, bail, Result};
use std::io::Write;
use std::net::{TcpStream, Shutdown};            // NEW
use std::process::{Command, Stdio};
use std::sync::{Mutex, OnceLock};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FProps { pub width: u32, pub height: u32, pub fps: u32 }

struct VideoPlayer {
    props: FProps,
    socket: TcpStream,                           // CHANGED: keep the socket here
}

const PORT: u16 = 5000;

static PLAYER: OnceLock<Mutex<Option<VideoPlayer>>> = OnceLock::new();
fn slot() -> &'static Mutex<Option<VideoPlayer>> {
    PLAYER.get_or_init(|| Mutex::new(None))
}

pub fn init_ffplay(width: u32, height: u32, fps: u32) -> Result<()> {
    println!("Initializing ffplay for {}x{} @ {}fps", width, height, fps);
    let mut guard = slot().lock().unwrap();
    if let Some(p) = guard.as_ref() {
        let want = FProps { width, height, fps };
        if p.props == want { return Ok(()); }
        bail!("ffplay already initialized with {:?}, requested {:?}", p.props, want);
    }

    let props = FProps { width, height, fps };

    // 1) Spawn ffplay in listen mode (no stdin/stdout needed)
    let _child = Command::new("ffplay")
        .args([
            "-loglevel","error","-autoexit",
            "-f","rawvideo",
            "-pixel_format","rgb24",
            "-video_size",&format!("{}x{}", width, height),
            "-framerate",&fps.to_string(),
            &format!("tcp://127.0.0.1:{}?listen=1", PORT),
        ])
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()?;

    // 2) Connect our TCP sender to ffplay's listener
    let socket = TcpStream::connect(("127.0.0.1", PORT))?;   // NEW
    socket.set_nodelay(true).ok();                           // optional

    // 3) Store the player with its socket
    *guard = Some(VideoPlayer { props, socket });            // CHANGED

    Ok(())
}

pub fn push_rgb24(bytes: &[u8]) -> Result<()> {
    // Write directly to the socket
    let mut guard = slot().lock().unwrap();                  // CHANGED: mutable guard
    let p = guard.as_mut().ok_or_else(|| anyhow!("ffplay not initialized"))?;
    p.socket.write_all(bytes)?;                              // NEW: send frame
    Ok(())
}

pub fn shutdown_ffplay() {
    let mut guard = slot().lock().unwrap();
    if let Some(p) = guard.take() {
        let _ = p.socket.shutdown(Shutdown::Both);           // polite EOF
        // dropping p ends the connection; ffplay will auto-exit due to -autoexit
    }
}
