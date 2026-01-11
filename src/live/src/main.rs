
mod render_live;
mod live_pix_fmt;
mod fplay;
//mod render_map_kind;

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
use gyroflow_core::stmap_live::{StmapsLive, LiveFrameJob};

use crate::render_live::{LiveRenderConfig, render_live_loop};
use crate::live_pix_fmt::{LiveFrame, PixelFormat, spawn_stream_reader};
use std::sync::OnceLock;
use std::path::Path;


const IMU_ADDR: &str = "127.0.0.1:7007";
// const FRAME_ADDR: &str = "127.0.0.1:7008"; // unused for now

const MAX_QUEUE_WARN: usize = 50;
const URL: &str = "C:\\git\\videos\\gyrovid.mp4"; // replace with your stream URL

const FPS: f64 =  30.0;
const WIDTH: usize = 2704;
const HEIGHT: usize = 2028;
const INTEGRATE_PERIOD_MS: u64 = 10;
const load_file_path: &str = "C:\\git\\GyroFlowLive\\Materials\\parsing\\mountvid_everything.csv";
const load_file: bool = false; //set to true to load from file instead of imu stream



const G_SCALE: f64 = 1.0;
const A_SCALE: f64 = 1.0;
static TSCALE: OnceLock<f64> = OnceLock::new();

pub fn set_tscale(val: f64) {
    TSCALE.set(val).expect("TSCALE already set!");
}

pub fn get_tscale() -> f64 {
    *TSCALE.get().expect("TSCALE not initialized yet!")
}

fn main() {
    

    env_logger::init();
    // Manager
    let stab_man = Arc::new(StabilizationManager::default());
    let metadata: FileMetadata = FileMetadata::default();
    // Initialize from stream data (size + initial fps; can be overridden by header fps)
    stab_man.init_from_stream_data(FPS, (WIDTH, HEIGHT));
 
    // Stop flag
    let stop = Arc::new(AtomicBool::new(false));

    // Crossbeam channel (Sender, Receiver)
    let (imu_tx, imu_rx) = unbounded::<LiveImuSample>();
    let (frame_tx, frame_rx) = unbounded::<(usize, LiveFrame)>();
    let (meta_tx, meta_rx) = unbounded::<()>();
    //create an stmap
    //let st_live: Arc<StmapsLive> = Arc::new(StmapsLive::new(Arc::clone(&stab_man)));

    let stream_reader_thread =  spawn_stream_reader(URL, frame_tx.clone(), PixelFormat::Rgba, MAX_QUEUE_WARN, /*Arc::clone(&st_live)*/)
        .expect("failed to spawn stream reader thread");


    
    let cfg = LiveRenderConfig::new(FPS);

    let value = Arc::clone(&stab_man);
    let render_thread = thread::spawn(move || {
        println!("waiting fosr metadata...");
        meta_rx.recv().expect("Failed to receive metadata-ready signal");
        println!("Starting render live loop");
        render_live_loop(frame_rx, Arc::clone(&value), cfg, PixelFormat::Rgba);
    });
    

       // Prepare a callback that will be called once per client when the full GCSV header is received
    let stab_for_header = Arc::clone(&stab_man);
    let header_cb: Arc<dyn Fn(&str) + Send + Sync> = Arc::new(move |header: &str| {
        
        let meta_tx = meta_tx.clone();
        // Parse the header into FileMetadata
        let metadata = parse_gyroflow_header(header);
        
        log::info!("Parsed GCSV header into FileMetadata: {:?}", metadata.detected_source);
        println!("Parsed GCSV header into FileMetadata: {:?}", metadata.frame_readout_direction);
        // Initialize live stream with this metadata
        let _ = stab_for_header.start_single_stream(metadata, 3.0, 1.0, 0.0, (WIDTH, HEIGHT), (WIDTH, HEIGHT), Path::new(load_file_path), load_file);
        
        println!("metadata loaded into stabilizer");

        // Notify that metadata is ready
        let _ = meta_tx.send(());
    });

    // Spawn server thread (binds and waits for generator to connect and write)
    spawn_line_server::<LiveImuSample>(
        "imu server",
        IMU_ADDR,
        imu_tx,
        Arc::clone(&stop),
        Some(header_cb),
        parse_imu_line,
    );


    // Spawn consumer thread: pull samples from channel and push into GyroSource

    {
        let mut counter: i64 = 0;
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
                if(counter%1000==0) {println!("IMU sample: {:?}", imu_sample);} 
                counter+=1;
            }
        });
    }
    // Keep main alive; periodically integrate live data
    if(!load_file){
        loop {
            stab_man.gyro.write().integrate_live_data();
            if stop.load(Ordering::Relaxed) {
                break;
            }
                    thread::sleep(Duration::from_millis(INTEGRATE_PERIOD_MS));

        }   
    }else{
        loop{
            thread::sleep(Duration::from_millis(1000));
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
    on_header: Option<Arc<dyn Fn(&str) + Send + Sync>>,
    parse_line: fn(&str) -> Option<T>,
) {
 {
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
                        if let Err(e) = handle_client(
                            name,
                            stream.try_clone().unwrap(),
                            &tx,
                            &stop,
                            on_header.clone(),
                            parse_line,
                        ) {
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
}

/// Handle a single connected client: read lines → parse → send
fn handle_client<T: Send>(
    name: &str,
    stream: TcpStream,
    tx: &Sender<T>,
    stop: &Arc<AtomicBool>,
    on_header: Option<Arc<dyn Fn(&str) + Send + Sync>>,
    parse_line: fn(&str) -> Option<T>,
) -> std::io::Result<()> {
       stream.set_read_timeout(Some(Duration::from_millis(500)))?;
    let reader = BufReader::new(stream);

    // Header state: we collect lines until we hit the "t,..." line
    let mut in_header = on_header.is_some();
    let mut header_buf = String::new();

    for maybe_line in reader.lines() {
        if stop.load(Ordering::Relaxed) {
            eprintln!("[{name}] stop requested");
            break;
        }
        match maybe_line {
            Ok(l) => {
                let line_trimmed = l.trim();
                if in_header {
                    // Accumulate header lines (including "GYROFLOW IMU LOG", version, etc.)
                    header_buf.push_str(line_trimmed);
                    header_buf.push('\n');

                    // End of header is the column header line "t,..." (e.g. "t,gx,gy,gz,ax,ay,az")
                    if line_trimmed.starts_with("t,") {
                        in_header = false;

                        if let Some(cb) = &on_header {
                            // Remove trailing newline for cleanliness
                            let hdr = header_buf.trim_end_matches('\n');
                            cb(hdr);
                        }
                    }

                    // Do NOT parse these lines as IMU samples
                    continue;
                }

                // After header: normal IMU data lines
                if let Some(msg) = parse_line(line_trimmed) {
                    if tx.send(msg).is_err() {
                        eprintln!("[{name}] main loop dropped; exiting client handler");
                        break;
                    }
                }
            }
            Err(e) => {
                // Timeout or IO error; on timeout continue, else break
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
    // 1. Parse to f64 because we want to apply scaling
    let raw_val = t_str.parse::<f64>().ok()?;


    // 2. Apply tscale (global multiplier)
    let us: f64 = 0.000001; // 1 microsecond in seconds
    let scaler: f64 = get_tscale() / us;
    let scaled = raw_val * scaler;

    // 3. Clamp into i64 inte
    let clamped = scaled
        .clamp(i64::MIN as f64, i64::MAX as f64)
        .round() as i64;

    let ts_sensor_us = clamped;

    // If your sender used scale factors (gscale/ascale), multiply here; for now = 1.0
    const GSCALE: f64 = G_SCALE;
    const ASCALE: f64 = A_SCALE;

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
            "tscale" => {
                let val = value.parse::<f64>().unwrap_or(1.0);
                set_tscale(val);
                }
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
            &_ => {},
            
        }
    }

    metadata
}
