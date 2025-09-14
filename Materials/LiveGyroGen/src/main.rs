use rand::prelude::*;
use rand_distr::{Distribution, Normal};
use std::f64::consts::PI;
use std::io::{BufWriter, Write};
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

/// Advance the state with a ρ-correlated random process (matching your Python logic)
fn advance_vector(
    x: &mut Vec6,
    v: &mut Vec6,
    aabs: Vec6,
    rho: f64,
    dt: f64,
    rng: &mut impl Rng,
) {
    // sigma_eps = aabs * sqrt(1 - rho^2) * sqrt(pi/2)
    let sigma = aabs.map1(|a| a * (1.0 - rho * rho).sqrt() * (PI / 2.0).sqrt());
    // draw epsilon ~ N(0, sigma)
    let mut eps = [0.0; 6];
    for i in 0..6 {
        let normal = Normal::new(0.0, sigma.0[i].max(1e-12)).unwrap();
        eps[i] = normal.sample(rng);
    }
    // v_next = rho * v + eps
    for i in 0..6 {
        v.0[i] = rho * v.0[i] + eps[i];
        x.0[i] = x.0[i] + v.0[i] * dt;
    }
}

fn main() -> std::io::Result<()> {
    // --- Config ---
    let period_hz: f64 = 30.0;
    let period = 1.0 / period_hz; // seconds
    let dt_sim = 0.01;            // dt used in your Python process model
    let rho = 0.92;

    // CLI flag: --ns to output monotonic nanoseconds in first column (tscale=1)
    let use_ns = std::env::args().any(|a| a == "--ns");

    // Gyroflow-style header values (copied from your Python)
    let gscale = 0.001_221_730_47_f64;
    let ascale = 0.000_488_281_25_f64;

    // --- State (mirrors your Python initial values) ---
    let mut rng = StdRng::from_entropy();
    let aabs = Vec6([11.333_333, 5.133_333, 17.133_333, 53.066_667, 15.266_667, 69.8]);
    let mut v = Vec6(aabs.0); // start v = aabs
    let mut x = Vec6([17.0, 14.0, 19.0, -42.0, -5.0, 99.0]);

    // --- Output setup (buffered stdout) ---
    let stdout = std::io::stdout();
    let mut out = BufWriter::new(stdout.lock());

    // Header
    // Keep `tscale` behavior: index mode -> tscale = period ; ns mode -> tscale = 1.0 (seconds=ns*1)
    let tscale = if use_ns { 1.0 } else { period };
    writeln!(
        out,
        "GYROFLOW IMU LOG\nversion,1.3\nid,custom_logger_name\norientation,YxZ\nnote,development_test\nfwversion,FIRMWARE_0.1.0\ntimestamp,1644159993\nvendor,potatocam\nvideofilename,videofilename.mp4\nlensprofile,potatocam/potatocam_mark1_prime_7_5mm_4k\nlens_info,wide\nframe_readout_time,15.23\nframe_readout_direction,0\ntscale,{tscale}\ngscale,{gscale}\nascale,{ascale}\nt,gx,gy,gz,ax,ay,az"
    )?;

    // --- Timing (fixed-rate loop with overrun detection) ---
    let mut i: u64 = 0;
    let mut next_t = Instant::now();
    let step = Duration::from_secs_f64(period);

    loop {
        // advance simulated sensors once per output sample
        advance_vector(&mut x, &mut v, aabs, rho, dt_sim, &mut rng);

        // First column: either index i (like your Python) or monotonic ns
        if use_ns {
            // monotonic ns (approx): Instant::now() since start; convert to ns
            let since_start = next_t.elapsed(); // small skew vs. "real now", fine for synthetic feed
            let t_ns = since_start.as_nanos() as i128; // print as integer
            writeln!(
                out,
                "{t_ns},{:.0},{:.0},{:.0},{:.0},{:.0},{:.0}",
                x.0[0], x.0[1], x.0[2], x.0[3], x.0[4], x.0[5]
            )?;
        } else {
            // index mode; tscale=period ⇒ consumer converts to seconds if needed
            writeln!(
                out,
                "{i},{:.0},{:.0},{:.0},{:.0},{:.0},{:.0}",
                x.0[0], x.0[1], x.0[2], x.0[3], x.0[4], x.0[5]
            )?;
        }

        // Flush each line so a reader sees it immediately (still buffered efficiently)
        out.flush()?;

        i += 1;
        next_t += step;
        let now = Instant::now();
        if next_t > now {
            sleep(next_t - now);
        } else {
            // Overrun: work took longer than the period
            eprintln!("Warning: Overrun detected. Consider adjusting the period.");
            // Re-anchor to now to avoid accumulating lag
            next_t = Instant::now();
        }
    }
}