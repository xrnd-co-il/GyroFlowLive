use std::io::{BufRead, BufReader};
use std::net::TcpStream;
use std::str::FromStr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc};
use std::thread;
use core::gyro_source::LiveImuSample;
use core::lib::StabilizationManager;



const IMU_ADDR: &str = "127.0.0.1:7007";
const FRAME_ADDR: &str = "127.0.0.1:7008";

fn main(){

   /*  let stab_man: StabilizationManager = StabilizationManager::default();
        pub fn load_gyro_data<F: Fn(f64)>(&self, 
            url: &str, 
            is_main_video: 
            bool, options: 
            &gyro_source::FileLoadOptions, progress_cb: F, cancel_flag: Arc<AtomicBool>) -> std::result::Result<(), GyroflowCoreError> 

*/
    //stab_man.load_gyro_data("live", true,);


    let stop: Arc<AtomicBool> = Arc::new(AtomicBool::new(false));

    let (imu_rx, imu_tx) = unbounded::<LiveImuSample>();
    spawn_line_reader<LiveImuSample>("imu thread", IMU_ADDR, imu_tx, stop, parse_imu_line);

    thread::spawn(move || {
        while let ok(imu_sample) = imu_rx.recv(){
            stab_man.gyro_source.push_live_imu(imu_sample)
        }
    });


}


fn spawn_line_reader<T: Send + 'static>(
    name: &'static str,
    addr: &'static str,
    tx: mpsc::Sender<T>,
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

fn parse_imu_line(line: &str) -> option<LiveImuSample>{
    //ill now the structure later
}