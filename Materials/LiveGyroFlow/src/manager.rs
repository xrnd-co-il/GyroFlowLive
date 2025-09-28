use anyhow::{Context, Result};
use crossbeam_channel::{bounded, Receiver, Sender, TrySendError};
use serde::{Deserialize, Serialize};
use std::io::{Read};
use std::net::{TcpListener, TcpStream};
use std::thread::{self, JoinHandle};

/// ---------- shared messages ----------

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ImuSample {
    pub ts_us: i64,
    pub gyro: [f64; 3],
    pub accel: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VideoFrame {
    /// monotonic nanoseconds or stream PTS converted to ns
    pub ts_ns: i64,
    pub width: u32,
    pub height: u32,
    pub pix_fmt: u32,    // keep it simple over the wire; map to enum inside
    pub data: Vec<u8>,   // for preview you might send compressed; this is raw
}


pub struct Manager {

    pub imu_rx: Receiver<ImuSample>,
    pub video_rx: Receiver<VideoFrame>,

    imu_listener: JoinHandle<()>,
    vid_listener: JoinHandle<()>,
    
}

impl Manager {
    pub fn start(imu_addr: &str, video_addr: &str) -> Result<Self> {
        let (imu_tx, imu_rx) = bounded::<ImuSample>(2048);
        let (video_tx, video_rx) = bounded::<VideoFrame>(64);

        let (imu_tx, imu_rx) = bounded::<ImuSample>(2048);
        let (vid_tx, vid_rx) = bounded::<VideoFrame>(64);

        let imu_listener = spawn_listener(imu_addr.to_string(), imu_tx);
        let vid_listener = spawn_listener(vid_addr.to_string(), vid_tx);

        Ok(Self { imu_rx, vid_rx, imu_listener, vid_listener })
        
    }
}

fn spawn_listener<T>(addr: String, tx: Sender<T>) -> JoinHandle<()>
where
    T: for<'de> Deserialize<'de> + Send + 'static,
{
    thread::spawn(move || {
        let listener = match TcpListener::bind(&addr) {
            Ok(l) => {
                eprintln!("[listen {addr}] up");
                l
            }
            Err(e) => {
                eprintln!("[listen {addr}] bind error: {e:?}");
                return;
            }
        };

        let (mut stream, peer) = match listener.accept() {
            Ok(v) => v,
            Err(e) => {
                eprintln!("[listen {addr}] accept error: {e:?}");
                return;
            }
        };
        eprintln!("[listen {addr}] client connected: {peer}");

        if let Err(e) = read_loop_len_prefixed(&mut stream, &tx) {
            eprintln!("[listen {addr}] connection ended: {e:?}");
        }
        // dropping tx closes the consumer channel when drained
    })
}

fn read_loop_len_prefixed<T>(stream: &mut TcpStream, tx: &Sender<T>) -> Result<()>
where
    T: for<'de> Deserialize<'de>,
{
    loop {
        // 1) Read 4-byte length prefix
        let mut len_buf = [0u8; 4];
        stream.read_exact(&mut len_buf)?;
        let len = u32::from_le_bytes(len_buf) as usize;

        // 2) Read that many bytes
        let mut buf = vec![0u8; len];
        stream.read_exact(&mut buf)?;

        // 3) Deserialize payload into T (ImuSample / VideoFrame)
        let msg: T = bincode::deserialize(&buf)?;

        // 4) Send it to the channel for the rest of your program
        tx.try_send(msg).ok();
    }
}