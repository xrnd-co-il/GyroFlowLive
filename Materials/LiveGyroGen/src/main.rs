use rand::prelude::*;
use rand_distr::{Distribution, Normal};
use std::f64::consts::PI;
use std::io::Write;
use std::net::TcpStream;
use std::thread::sleep;
use std::time::{Duration, Instant};

#[derive(Clone, Copy)]
struct Vec6([f64; 6]);

impl Vec6 {
    fn map1(self, f: impl Fn(f64) -> f64) -> Vec6 {
        let mut out = [0.0; 6];
        for i in 0..6 {
            out[i] = f(self.0[i]);
        }
        Vec6(out)
    }
}

fn advance_vector(
    x: &mut Vec6,
    v: &mut Vec6,
    aabs: Vec6,
    rho: f64,
    dt: f64,
    rng: &mut impl Rng,
) {
    let sigma = aabs.map1(|a| a * (1.0 - rho * rho).sqrt());
    let mut eps = [0.0; 6];
    for i in 0..6 {
        let normal = Normal::new(0.0, sigma.0[i].max(1e-12)).unwrap();
        eps[i] = normal.sample(rng);
    }
    for i in 0..6 {
        v.0[i] = rho * v.0[i] + eps[i];
        x.0[i] = x.0[i] + v.0[i] * dt;
    }
}

fn main() -> std::io::Result<()> {
    // -------------------------
    // CLI: choose rad or deg
    // -------------------------
    let args: Vec<String> = std::env::args().collect();
    let use_degrees = args.iter().any(|a| a.eq_ignore_ascii_case("deg"));
    let use_radians = args.iter().any(|a| a.eq_ignore_ascii_case("rad"));

    // Default = radians
    let mode = if use_degrees {
        "deg"
    } else {
        "rad"
    };

    println!("IMU OUTPUT MODE: {}", mode);

    // -------------------------
    // Core config
    // -------------------------
    const PORT: u16 = 7007;

    let period_hz: f64 = 5.0;
    let period = 1.0 / period_hz;
    let dt_sim = 0.01;

    let rho = 0.92;

    let gscale = 0.001_221_730_47_f64;
    let ascale = 0.000_488_281_25_f64;

    let mut rng = StdRng::from_entropy();

    let aabs = Vec6([11.333, 5.133, 17.133, 53.066, 15.266, 69.8]);
    let mut v = Vec6(aabs.0);
    let mut x = Vec6([17.0, 14.0, 19.0, -42.0, -5.0, 99.0]);

    // -------------------------------------
    // Connect to stabilization server
    // -------------------------------------
    let addr = format!("127.0.0.1:{}", PORT);
    println!("Connecting to {addr} ...");

    let mut stream = loop {
        match TcpStream::connect(&addr) {
            Ok(s) => {
                println!("Connected!");
                break s;
            }
            Err(_) => {
                sleep(Duration::from_secs(1));
            }
        }
    };

    // -------------------------
    // Header
    // -------------------------
    let header = format!(
    "GYROFLOW IMU LOG
    version,1.3
    id,custom_logger_name
    orientation,YxZ
    note,development_test
    fwversion,FIRMWARE_0.1.0
    timestamp,1755695371.5914793
    vendor,potatocam
    videofilename,videofilename.mp4
    lensprofile,potatocam/potatocam_mark1_prime_7_5mm_4k
    lens_info,wide
    frame_readout_time,15.23
    frame_readout_direction,0
    tscale,1.0
    gscale,1.0
    ascale,1.0
    t,gx,gy,gz,ax,ay,az"
    );
    stream.write_all(header.as_bytes())?;

    // -------------------------
    // Main loop
    // -------------------------
    let mut i: u64 = 0;
    let mut next_t = Instant::now();
    let step = Duration::from_secs_f64(period);

    loop {
        advance_vector(&mut x, &mut v, aabs, rho, dt_sim, &mut rng);

        // -------- Gyro formatting --------
        let (gx, gy, gz) = if mode == "deg" {
            // degrees/sec
            (x.0[0], x.0[1], x.0[2])
        } else {
            // radians/sec
            (
                x.0[0] * (PI / 180.0),
                x.0[1] * (PI / 180.0),
                x.0[2] * (PI / 180.0),
            )
        };

        // -------- Accel (whatever original units you want) --------
        let msg = format!(
            "{i},{gx:.6},{gy:.6},{gz:.6},{:.3},{:.3},{:.3}\n",
            x.0[3], x.0[4], x.0[5]
        );

        stream.write_all(msg.as_bytes())?;

        i += 1;
        next_t += step;
        let now = Instant::now();
        if next_t > now {
            sleep(next_t - now);
        }
    }
}
