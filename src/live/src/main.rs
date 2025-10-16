use std::io::{BufRead, BufReader};
use std::net::TcpStream;
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

    // If you implemented this helper in your core:
    // start live ring, metadata, keep_secs=3.0, a=1.0, b=0.0
    // If you don't have this method, call start_live_gyro directly.
    let _ = stab_man.start_single_stream(metadata, 3.0, 1.0, 0.0);
    // let _ = stab_man.start_live_gyro(3.0, 1.0, 0.0);

    // Stop flag
    let stop = Arc::new(AtomicBool::new(false));

    // Crossbeam channel (Sender, Receiver)
    let (imu_tx, imu_rx) = unbounded::<LiveImuSample>();

    // Spawn reader thread (connects to the IMU generator and parses lines)
    spawn_line_reader::<LiveImuSample>("imu thread", IMU_ADDR, imu_tx, Arc::clone(&stop), parse_imu_line);

    // Spawn consumer thread: pull samples from channel and push into GyroSource
    {
        let stab = Arc::clone(&stab_man);
        thread::spawn(move || {
            while let Ok(imu_sample) = imu_rx.recv() {
                // destructure
                let LiveImuSample { ts_sensor_us, gyro, accel } = imu_sample;

                // If you have a video clock, pass it; for now reuse sensor time
                let now_video_us = ts_sensor_us;

                // Push into the live ring
                // NOTE: this assumes you have: fn push_live_imu(&self, ts, gyro, accel, now_video_us)
                // If your method signature is different, adapt here.
                let mut g = stab.gyro.write();
                g.push_live_imu(imu_sample, now_video_us);
            }
        });
    }

    // Keep main alive (demo); in a real app handle shutdowns, signals, etc.
    loop {
        thread::sleep(Duration::from_millis(500));
        stab_man.gyro.write().integrate_live_data();
    }
}

/// Generic TCP line reader: connects, reads lines, parses with `parse_line`, sends to `tx`
fn spawn_line_reader<T: Send + 'static>(
    name: &'static str,
    addr: &'static str,
    tx: Sender<T>,
    stop: Arc<AtomicBool>,
    parse_line: fn(&str) -> Option<T>,
) {
    thread::Builder::new()
        .name(format!("reader_{name}"))
        .spawn(move || {
            while !stop.load(Ordering::Relaxed) {
                match TcpStream::connect(addr) {
                    Ok(stream) => {
                        eprintln!("[{name}] connected to {addr}");
                        let _ = stream.set_read_timeout(Some(Duration::from_millis(250)));
                        let reader = BufReader::new(stream);

                        for line in reader.lines() {
                            if stop.load(Ordering::Relaxed) {
                                eprintln!("[{name}] stop requested");
                                return;
                            }
                            match line {
                                Ok(l) => {
                                    if let Some(msg) = parse_line(l.trim()) {
                                        if tx.send(msg).is_err() {
                                            eprintln!("[{name}] main loop dropped; exiting");
                                            return;
                                        }
                                    }
                                }
                                Err(e) => {
                                    eprintln!("[{name}] read error: {e}. Reconnecting...");
                                    thread::sleep(Duration::from_millis(300));
                                    break; // break the for-loop → reconnect
                                }
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("[{name}] connect failed: {e}. Retrying...");
                        thread::sleep(Duration::from_millis(500));
                    }
                }
            }
            eprintln!("[{name}] thread exit");
        })
        .expect("spawn reader thread");
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
