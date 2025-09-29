// src/main.rs

use std::io::{BufRead, BufReader};
use std::net::TcpStream;
use std::thread;
use std::time::{Duration, Instant};
use std::sync::mpsc::{self, Receiver};
use std::str::FromStr;

use log::{debug, error, info, warn};

// ---- Configure your IMU TCP endpoint here ----
const IMUport: u16 = 7007; // <-- set your port

// ---- Types you already have / expect in your codebase ----
// Adjust the module paths if needed.
#[derive(Clone, Copy, Debug, Default)]
pub struct LiveImuSample {
    pub ts_sensor_us: u64,
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
}

// Minimal trait-like facade so you can adapt to your real manager.
// Replace bodies with your actual API if different.
mod gyroflow_live {
    use super::LiveImuSample;

    // Re-export your real manager type. If it's in another module, change the path:
    // pub use crate::libs::StablizerManager;
    // For now we declare a stub so this compiles until you hook the real one.
    pub struct StablizerManager {
        // TODO: your internal fields; ring, params, etc.
        // We keep a small user-land ring here only as a placeholder.
        ring: std::sync::Mutex<std::collections::VecDeque<LiveImuSample>>,
        ring_cap: usize,
    }

    impl StablizerManager {
        pub fn new_with_live() -> Self {
            Self {
                ring: std::sync::Mutex::new(std::collections::VecDeque::with_capacity(4096)),
                ring_cap: 4096,
            }
        }

        /// Push a live sample into the manager’s ring.
        /// In your code, this should call the REAL push into your IMU ring.
        pub fn push_live_sample(&self, s: LiveImuSample) {
            let mut g = self.ring.lock().unwrap();
            if g.len() == self.ring_cap {
                g.pop_front();
            }
            g.push_back(s);
        }

        /// Integrate live data (your real function should read from its live ring).
        pub fn integrate_live_data(&mut self) {
            // TODO: call your real implementation
            // e.g., self.telemetry.integrate_live_data();
            // For placeholder:
            log::debug!("integrate_live_data() tick");
        }

        /// Generate STMaps for the current live frame/batch.
        /// You said you'll implement this later; we simply call it.
        pub fn generate_stmaps_live(&mut self) {
            // TODO: your real generate_stmaps_live implementation.
            log::debug!("generate_stmaps_live() tick");
        }
    }
}

use gyroflow_live::StablizerManager;

// -------------------- IMU PARSER --------------------

fn parse_imu_line(line: &str) -> Option<LiveImuSample> {
    let s = line.trim();
    if s.is_empty() { return None; }

    // Try CSV first: ts_us,gx,gy,gz,ax,ay,az
    if let Some(sample) = parse_imu_csv(s) {
        return Some(sample);
    }
    // Try JSON: {"ts_us":..., "gyro":[...], "accel":[...]}
    if let Some(sample) = parse_imu_json(s) {
        return Some(sample);
    }

    warn!("Unrecognized IMU line format: {}", s);
    None
}

fn parse_imu_csv(s: &str) -> Option<LiveImuSample> {
    let mut it = s.split(',').map(str::trim);
    let ts_us = u64::from_str(it.next()?).ok()?;
    let gx = f32::from_str(it.next()?).ok()?;
    let gy = f32::from_str(it.next()?).ok()?;
    let gz = f32::from_str(it.next()?).ok()?;
    let ax = f32::from_str(it.next()?).ok()?;
    let ay = f32::from_str(it.next()?).ok()?;
    let az = f32::from_str(it.next()?).ok()?;
    Some(LiveImuSample {
        ts_sensor_us: ts_us,
        gyro: [gx, gy, gz],
        accel: [ax, ay, az],
    })
}

fn parse_imu_json(s: &str) -> Option<LiveImuSample> {
    #[derive(serde::Deserialize)]
    struct JsonImu<'a> {
        #[serde(rename = "ts_us")]
        ts_us: Option<u64>,
        gyro: Option<[f32; 3]>,
        #[serde(alias = "accl", alias = "acc", alias = "accelerometer")]
        accel: Option<[f32; 3]>,
        // tolerate keys in unknown casing
        #[allow(dead_code)]
        #[serde(flatten)]
        _rest: std::collections::HashMap<&'a str, serde_json::Value>,
    }
    let v: JsonImu = serde_json::from_str(s).ok()?;
    Some(LiveImuSample {
        ts_sensor_us: v.ts_us?,
        gyro: v.gyro?,
        accel: v.accel?,
    })
}

// -------------------- TCP READER THREAD --------------------

fn spawn_imu_reader(port: u16) -> std::io::Result<Receiver<LiveImuSample>> {
    let (tx, rx) = mpsc::channel::<LiveImuSample>();

    // Spawn a thread that connects and streams
    thread::Builder::new()
        .name("imu_tcp_reader".into())
        .spawn(move || {
            let addr = format!("127.0.0.1:{port}");
            loop {
                match TcpStream::connect(&addr) {
                    Ok(stream) => {
                        info!("Connected to IMU at {}", addr);
                        if let Err(e) = stream.set_read_timeout(Some(Duration::from_millis(250))) {
                            warn!("set_read_timeout failed: {e}");
                        }
                        let reader = BufReader::new(stream);
                        for line in reader.lines() {
                            match line {
                                Ok(l) => {
                                    if let Some(sample) = parse_imu_line(&l) {
                                        if tx.send(sample).is_err() {
                                            error!("imu_reader: consumer gone; exiting");
                                            return;
                                        }
                                    }
                                }
                                Err(e) => {
                                    // Timeout or EOF → try to reconnect
                                    warn!("imu_reader: read error: {e}; reconnecting in 500ms");
                                    thread::sleep(Duration::from_millis(500));
                                    break;
                                }
                            }
                        }
                    }
                    Err(e) => {
                        warn!("Failed to connect to IMU at {}: {}; retrying…", addr, e);
                        thread::sleep(Duration::from_millis(500));
                    }
                }
            }
        })?;

    Ok(rx)
}

// -------------------- MAIN RUNTIME --------------------

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    info!("Starting live IMU + stabilization runtime");

    // 1) Spin up the IMU reader
    let imu_rx = match spawn_imu_reader(IMUport) {
        Ok(rx) => rx,
        Err(e) => {
            eprintln!("Failed to spawn IMU reader: {e}");
            std::process::exit(1);
        }
    };

    // 2) Create the manager in LIVE mode
    // NOTE: replace new_with_live() with your real constructor if different
    let mut stab = StablizerManager::new_with_live();

    // 3) Main loop: pump IMU → ring, every 0.5s integrate + generate stmaps
    let mut last_integrate = Instant::now();
    let integrate_period = Duration::from_millis(500);

    loop {
        // Non-blocking-ish IMU receive; drain quickly so we keep up
        // Use try_recv in a small burst to batch
        for _ in 0..1024 {
            match imu_rx.try_recv() {
                Ok(sample) => {
                    // Push to your live ring through the manager
                    // If your API differs, change here:
                    stab.push_live_sample(sample);
                }
                Err(mpsc::TryRecvError::Empty) => {
                    break;
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    error!("IMU channel disconnected; exiting");
                    return;
                }
            }
        }

        // Tick every 0.5 s: integrate and then generate STMaps
        if last_integrate.elapsed() >= integrate_period {
            last_integrate = Instant::now();

            // 3a) integrate latest IMU window into quaternion timeline
            stab.integrate_live_data();
            // Validate quickly via logs
            debug!("Integrated live IMU window");

            // 3b) generate live STMaps (implementation to be provided later)
            stab.generate_stmaps_live();
            debug!("Generated live STMaps");
        }

        // Small sleep to avoid a busy loop (tune as needed)
        thread::sleep(Duration::from_millis(2));
    }
}