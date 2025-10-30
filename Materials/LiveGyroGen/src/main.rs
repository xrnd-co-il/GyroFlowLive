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
    fn map2(self, other: Vec6, f: impl Fn(f64, f64) -> f64) -> Vec6 {
        let mut out = [0.0; 6];
        for i in 0..6 {
            out[i] = f(self.0[i], other.0[i]);
        }
        Vec6(out)
    }
    fn map1(self, f: impl Fn(f64) -> f64) -> Vec6 {
        let mut out = [0.0; 6];
        for i in 0..6 {
            out[i] = f(self.0[i]);
        }
        Vec6(out)
    }
}

/// Advance the state with a ρ-correlated random process
fn advance_vector(
    x: &mut Vec6,
    v: &mut Vec6,
    aabs: Vec6,
    rho: f64,
    dt: f64,
    rng: &mut impl Rng,
) {
    let sigma = aabs.map1(|a| a * (1.0 - rho * rho).sqrt() * (PI / 2.0).sqrt());
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
    // --- Config ---
    const PORT: u16 = 7007; // change as needed
    let period_hz: f64 = 30.0;
    let period = 1.0 / period_hz;
    let dt_sim = 0.01;
    let rho = 0.92;
    let use_ns = std::env::args().any(|a| a == "--ns");

    // Gyroflow-style scales
    let gscale = 0.001_221_730_47_f64;
    let ascale = 0.000_488_281_25_f64;

    // State
    let mut rng = StdRng::from_entropy();
    let aabs = Vec6([11.333_333, 5.133_333, 17.133_333, 53.066_667, 15.266_667, 69.8]);
    let mut v = Vec6(aabs.0);
    let mut x = Vec6([17.0, 14.0, 19.0, -42.0, -5.0, 99.0]);

    // Connect to TCP server
    let addr = format!("127.0.0.1:{}", PORT);
    let mut stream = TcpStream::connect(&addr)?;
    println!("Connected to {}", addr);

    // Send header once
    let tscale = if use_ns { 1.0 } else { period };
    let header = format!(
        "GYROFLOW IMU LOG\nversion,1.3\nid,custom_logger_name\norientation,YxZ\n\
         note,development_test\nfwversion,FIRMWARE_0.1.0\ntimestamp,1644159993\n\
         vendor,potatocam\nvideofilename,videofilename.mp4\n\
         lensprofile,potatocam/potatocam_mark1_prime_7_5mm_4k\n\
         lens_info,wide\nframe_readout_time,15.23\nframe_readout_direction,0\n\
         tscale,{tscale}\ngscale,{gscale}\nascale,{ascale}\n\
         t,gx,gy,gz,ax,ay,az\n"
    );
    stream.write_all(header.as_bytes())?;

    // Timing
    let mut i: u64 = 0;
    let mut next_t = Instant::now();
    let step = Duration::from_secs_f64(period);

    loop {
        advance_vector(&mut x, &mut v, aabs, rho, dt_sim, &mut rng);

        if use_ns {
            let since_start = next_t.elapsed();
            let t_ns = since_start.as_nanos() as i128;
            let msg = format!(
                "{t_ns},{:.0},{:.0},{:.0},{:.0},{:.0},{:.0}\n",
                x.0[0], x.0[1], x.0[2], x.0[3], x.0[4], x.0[5]
            );
            stream.write_all(msg.as_bytes())?;
        } else {
            let msg = format!(
                "{i},{:.0},{:.0},{:.0},{:.0},{:.0},{:.0}\n",
                x.0[0], x.0[1], x.0[2], x.0[3], x.0[4], x.0[5]
            );
            stream.write_all(msg.as_bytes())?;
        }

        i += 1;
        next_t += step;
        let now = Instant::now();
        if next_t > now {
            sleep(next_t - now);
        } else {
            eprintln!("Warning: Overrun detected.");
            next_t = Instant::now();
        }
    }
}
