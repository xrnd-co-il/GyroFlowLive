// gyro_source/live.rs
use std::collections::VecDeque;


#[derive(Clone, Copy, Debug)]
pub struct LiveImuSample {
    pub ts_sensor_us: i64,    // sensor clock (from device)
    pub gyro: [f64; 3],       // rad/s
    pub accel: Option<[f64;3]>,
}

#[derive(Default)]
pub struct LiveClockSync {
    // Linear mapping sensor_time -> video_time: video = a*sensor + b (all µs)
    pub a: f64,  // scale
    pub b: f64,  // offset
}

#[derive(Default)]
pub struct ImuRing {
    pub buf: VecDeque<LiveImuSample>,
    pub keep_us: i64, // e.g. 3_000_000
}


impl ImuRing {
    pub fn new(keep_us: i64) -> Self { Self { buf: VecDeque::new(), keep_us } }
    pub fn push(&mut self, s: LiveImuSample, now_video_us: i64, sync: &LiveClockSync) {
        // convert to video clock immediately
        let vts = (sync.a * s.ts_sensor_us as f64 + sync.b).round() as i64;
        let sample = LiveImuSample { ts_sensor_us: vts, ..s }; // reuse field for video ts
        self.buf.push_back(sample);
        // evict old
        while let Some(front) = self.buf.front() {
            if now_video_us - front.ts_sensor_us > self.keep_us { self.buf.pop_front(); } else { break; }
        }
    }
    pub fn window(&self, start_us: i64, end_us: i64) -> impl Iterator<Item=&LiveImuSample> {
        self.buf.iter().filter(move |s| s.ts_sensor_us >= start_us && s.ts_sensor_us <= end_us)
    }
}

#[derive(Default)]
pub struct LiveState {
    pub header: String,
    pub ring: ImuRing,
    pub sync: LiveClockSync,
    pub enabled: bool,
}