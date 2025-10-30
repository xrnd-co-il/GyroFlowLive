use std::io::{BufRead, BufReader};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;



use crossbeam_channel::{unbounded, Receiver, Sender};
use serde_json::json;
use std::collections::BTreeMap;

use gyroflow_core::gyro_source::FileMetadata;
use gyroflow_core::gyro_source::live::LiveImuSample;
use gyroflow_core::stabilization_params::ReadoutDirection;
use gyroflow_core::StabilizationManager;

const IMU_ADDR: &str = "127.0.0.1:7007";
// const FRAME_ADDR: &str = "127.0.0.1:7008"; // unused for now

fn main() {
    // Manager + metadata
    let stab_man = Arc::new(StabilizationManager::default());
    let metadata: FileMetadata = FileMetadata::default();

    // Initialize live ring (3s retention; scale placeholders a=1./0, b=0.0)
    let _ = stab_man.start_single_stream(metadata, 3.0, 1.0, 0.0);
    // let _ = stab_man.start_live_gyro(3.0, 1.0, 0.0);

    // Stop flag
    let stop = Arc::new(AtomicBool::new(false));

    // Crossbeam channel (Sender, Receiver)
    let (imu_tx, imu_rx) = unbounded::<LiveImuSample>();

    // Spawn server thread (binds and waits for generator to connect and write)
    spawn_line_server::<LiveImuSample>("imu server", IMU_ADDR, imu_tx, Arc::clone(&stop), parse_imu_line);

    // Spawn consumer thread: pull samples from channel and push into GyroSource
    {
        let stab = Arc::clone(&stab_man);
        thread::spawn(move || {
            while let Ok(imu_sample) = imu_rx.recv() {
                let LiveImuSample { ts_sensor_us, .. } = imu_sample;
                // If you have a video clock, pass it; reusing sensor time for now
                let now_video_us = ts_sensor_us;
                //println!("Received IMU sample at ts_sensor_us={}", imu_sample);
                //working :)
                let mut g = stab.gyro.write();
                g.push_live_imu(imu_sample, now_video_us);
            }
        });
    }

    // Keep main alive; periodically integrate live data
    loop {
        thread::sleep(Duration::from_millis(500));
        stab_man.gyro.write().integrate_live_data();
        if stop.load(Ordering::Relaxed) {
            break;
        }
    }
}

/// TCP line **server**: bind(addr) and accept() clients; for each client,
/// read lines, parse with `parse_line`, and send to `tx`.
fn spawn_line_server<T: Send + 'static>(
    name: &'static str,
    addr: &'static str,
    tx: Sender<T>,
    stop: Arc<AtomicBool>,
    parse_line: fn(&str) -> Option<T>,
) {
    thread::Builder::new()
        .name(format!("server_{name}"))
        .spawn(move || {
            // Bind once; if bind fails, crash early so the operator knows
            let listener = match TcpListener::bind(addr) {
                Ok(l) => {
                    eprintln!("[{name}] listening on {addr}");
                    l
                }
                Err(e) => {
                    eprintln!("[{name}] failed to bind {addr}: {e}");
                    return;
                }
            };

            // Accept-loop: handle one client at a time; when it disconnects, accept the next one
            listener
                .set_nonblocking(false)
                .ok(); // blocking accept is fine here

            while !stop.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((stream, peer)) => {
                        eprintln!("[{name}] client connected from {peer}");
                        if let Err(e) = handle_client(name, stream.try_clone().unwrap(), &tx, &stop, parse_line) {
                            eprintln!("[{name}] client handler error: {e}");
                        }
                        eprintln!("[{name}] client disconnected");
                    }
                    Err(e) => {
                        eprintln!("[{name}] accept error: {e}");
                        thread::sleep(Duration::from_millis(200));
                    }
                }
            }

            eprintln!("[{name}] server exit");
        })
        .expect("spawn server thread");
}

/// Handle a single connected client: read lines → parse → send
fn handle_client<T: Send>(
    name: &str,
    stream: TcpStream,
    tx: &Sender<T>,
    stop: &Arc<AtomicBool>,
    parse_line: fn(&str) -> Option<T>,
) -> std::io::Result<()> {
    // Optional read timeout so we periodically check `stop`
    stream.set_read_timeout(Some(Duration::from_millis(500)))?;
    let reader = BufReader::new(stream);

    for maybe_line in reader.lines() {
        if stop.load(Ordering::Relaxed) {
            eprintln!("[{name}] stop requested");
            break;
        }
        match maybe_line {
            Ok(l) => {
                let line = l.trim();
                if let Some(msg) = parse_line(line) {
                    if tx.send(msg).is_err() {
                        eprintln!("[{name}] main loop dropped; exiting client handler");
                        break;
                    }
                }
            }
            Err(e) => {
                // Timeout or IO error; on timeout continue, else break
                // (on Windows, timeouts often appear as WouldBlock/TimedOut)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut
                {
                    continue;
                } else {
                    return Err(e);
                }
            }
        }
    }

    Ok(())
}

/// Simple parser that accepts "t,gx,gy,gz,ax,ay,az"
/// - If `t` is large (>= 1e12), treat as nanoseconds and convert to microseconds
/// - Otherwise treat `t` as a sample index and synthesize µs with a fixed sample period
fn parse_imu_line(line: &str) -> Option<LiveImuSample> {
    let l = line.trim();
    if l.is_empty() || l.starts_with("GYROFLOW") || l.starts_with("t,") {
        return None;
    }

    let mut it = l.split(',');
    let t_str = it.next()?.trim();

    let gx = it.next()?.trim().parse::<f64>().ok()?;
    let gy = it.next()?.trim().parse::<f64>().ok()?;
    let gz = it.next()?.trim().parse::<f64>().ok()?;
    let ax = it.next()?.trim().parse::<f64>().ok()?;
    let ay = it.next()?.trim().parse::<f64>().ok()?;
    let az = it.next()?.trim().parse::<f64>().ok()?;

    //println!("Parsed IMU line: t={} gx={} gy={} gz={} ax={} ay={} az={}", t_str, gx, gy, gz, ax, ay, az);

    // auto-detect time column
    let ts_sensor_us: i64 = if let Ok(t_ns_big) = t_str.parse::<i128>() {
        // treat as nanoseconds if very large; convert to microseconds
        if t_ns_big >= 1_000_000_000_000i128 {
            (t_ns_big / 1000).clamp(i128::from(i64::MIN), i128::from(i64::MAX)) as i64
        } else {
            // it's not big enough to be ns; treat as index with 30 Hz by default
            const SAMPLE_PERIOD_US: i64 = 33_333;
            let idx = t_ns_big.max(0) as i64;
            idx.saturating_mul(SAMPLE_PERIOD_US)
        }
    } else if let Ok(idx_u64) = t_str.parse::<u64>() {
        // pure index path
        const SAMPLE_PERIOD_US: i64 = 33_333;
        (idx_u64 as i64).saturating_mul(SAMPLE_PERIOD_US)
    } else {
        // failed to parse t
        return None;
    };

    // If your sender used scale factors (gscale/ascale), multiply here; for now = 1.0
    const GSCALE: f64 = 1.0;
    const ASCALE: f64 = 1.0;

    let gyro = [gx * GSCALE, gy * GSCALE, gz * GSCALE];
    let accel = Some([ax * ASCALE, ay * ASCALE, az * ASCALE]);

    Some(LiveImuSample { ts_sensor_us, gyro, accel })
}

/// Parse Gyroflow-style header text → FileMetadata (used if you send the header)
pub fn parse_gyroflow_header(header: &str) -> FileMetadata {
    let mut metadata = FileMetadata {
        imu_orientation: None,
        raw_imu: Vec::new(),
        quaternions: BTreeMap::new(),
        gravity_vectors: None,
        image_orientations: None,
        detected_source: Some("Gyroflow Live Stream".into()),
        frame_readout_time: None,
        frame_readout_direction: ReadoutDirection::TopToBottom,
        frame_rate: None,
        camera_identifier: None,
        lens_profile: None,
        lens_positions: BTreeMap::new(),
        lens_params: BTreeMap::new(),
        digital_zoom: None,
        has_accurate_timestamps: true,
        additional_data: json!({}),
        per_frame_time_offsets: Vec::new(),
        camera_stab_data: Vec::new(),
        mesh_correction: Vec::new(),
    };

    for line in header.lines() {
        if line.trim().is_empty() || line.starts_with("GYROFLOW") || line.starts_with("t,") {
            continue;
        }
        let mut parts = line.splitn(2, ',');
        let key = parts.next().unwrap_or("").trim();
        let value = parts.next().unwrap_or("").trim();

        match key {
            "orientation" => metadata.imu_orientation = Some(value.to_string()),
            "vendor" => metadata.detected_source = Some(value.to_string()),
            "frame_readout_time" => {
                if let Ok(v) = value.parse::<f64>() {
                    metadata.frame_readout_time = Some(v);
                }
            }
            "frame_readout_direction" => {
                metadata.frame_readout_direction = match value {
                    "0" => ReadoutDirection::TopToBottom,
                    "1" => ReadoutDirection::BottomToTop,
                    "2" => ReadoutDirection::LeftToRight,
                    "3" => ReadoutDirection::RightToLeft,
                    _ => ReadoutDirection::TopToBottom,
                };
            }
            "lensprofile" => {
                metadata.lens_profile = Some(json!(value));
            }
            "frame_rate" | "fps" => {
                if let Ok(v) = value.parse::<f64>() {
                    metadata.frame_rate = Some(v);
                }
            }
            "digital_zoom" => {
                if let Ok(v) = value.parse::<f64>() {
                    metadata.digital_zoom = Some(v);
                }
            }
            "timestamp" => metadata.additional_data["timestamp"] = json!(value),
            "fwversion" => metadata.additional_data["fwversion"] = json!(value),
            "id" => metadata.additional_data["device_id"] = json!(value),
            "note" => metadata.additional_data["note"] = json!(value),
            "lens_info" => metadata.additional_data["lens_info"] = json!(value),
            "vendor" => metadata.additional_data["vendor"] = json!(value),
            _ => {}
        }
    }

    metadata
}
