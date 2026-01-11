// gyro_source/live.rs
use std::collections::VecDeque;
use parking_lot::{RwLock, Mutex};   
use std::sync::Arc;
use crossbeam_channel::{unbounded, Sender, Receiver};
use super::FileMetadata;
use super::TimeQuat;
use super::Quat64;
use std::fmt;
use std::sync::atomic::{AtomicBool, Ordering, AtomicU64};
use std::collections::BTreeMap;
use nalgebra::{Quaternion as NQuat, UnitQuaternion as NUnitQuat}; // adjust if you already import nalgebra elsewhere
use std::path::Path;
use crate::gyro_source::csv_quats;

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

impl LiveClockSync {
    pub fn new(a: f64, b: f64) -> Self { Self { a, b } }
}

impl fmt::Display for LiveClockSync {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "LiveClockSync {{ a: {:.6}, b: {:.3} }}", self.a, self.b)
    }
}

#[derive(Default)]
pub struct ImuRing {
    pub buf: VecDeque<LiveImuSample>,
    pub keep_us: i64, // e.g. 3_000_000
}


impl fmt::Display for LiveImuSample {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self.accel {
            Some(a) => write!(
                f,
                "LiveImuSample {{ ts_sensor_us: {}, gyro: [{:.3}, {:.3}, {:.3}], accel: [{:.3}, {:.3}, {:.3}] }}",
                self.ts_sensor_us,
                self.gyro[0], self.gyro[1], self.gyro[2],
                a[0], a[1], a[2]
            ),
            None => write!(
                f,
                "LiveImuSample {{ ts_sensor_us: {}, gyro: [{:.3}, {:.3}, {:.3}], accel: None }}",
                self.ts_sensor_us,
                self.gyro[0], self.gyro[1], self.gyro[2]
            ),
        }
    }
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
    pub fn snapshot(&self) -> Vec<LiveImuSample> {
        self.buf.iter().copied().collect()
    }




}

#[derive(Debug, Clone, Default)]
pub struct QuatBuffer {
    pub quats: TimeQuat,
    pub first_us: i64,
    pub last_us:  i64,
}


impl QuatBuffer {
    pub fn from_btreemap(map: &TimeQuat) -> Option<Self> {
        if map.is_empty() { return None; }
        let first_us = *map.keys().next().unwrap();
        let last_us  = *map.keys().next_back().unwrap();
        Some(Self { quats: map.clone(), first_us, last_us })
    }

    #[inline]
    pub fn mid_us(&self) -> i64 { (self.first_us + self.last_us) / 2 }

    #[inline]
    pub fn span_us(&self) -> i64 { (self.last_us - self.first_us).max(0) }

    /// “Covers” a target time with required pre/post padding.
    #[inline]
    pub fn covers_with_padding(&self, target_us: i64, pre_us: i64, post_us: i64) -> bool {
        self.first_us <= target_us - pre_us && self.last_us >= target_us + post_us
    }

    /// Is the target time “roughly in the middle”?
    ///
    /// `center_ratio` is a fraction of HALF the span.
    /// Example: center_ratio=0.25 ⇒ allowed offset from center is 25% of half-span.
    pub fn is_centered_for(&self, target_us: i64, center_ratio: f64) -> bool {
        let span = self.span_us();
        if span == 0 { return false; }
        let half = span as f64 / 2.0;
        let tol  = (center_ratio.max(0.0) * half) as f64;
        (target_us as f64 - self.mid_us() as f64).abs() <= tol
    }

    /// Simple SLERP lookup (same logic you already use elsewhere).
    pub fn quat_at_ms(&self, t_ms: f64) -> Option<Quat64> {
        if self.quats.is_empty() { return None; }
        let t_us = (t_ms * 1000.0).round() as i64;
        let t_us = t_us.clamp(self.first_us, self.last_us);

        if let Some((&t0, &q0)) = self.quats.range(..=t_us).next_back() {
            if t0 == t_us { return Some(q0); }
            if let Some((&t1, &q1)) = self.quats.range(t_us..).next() {
                let dt = (t1 - t0) as f64;
                if dt <= 0.0 { return Some(q0); }
                let a = (t_us - t0) as f64 / dt;
                return Some(q0.slerp(&q1, a));
            }
        }
        self.quats.values().next_back().copied()
    }

     pub fn to_btreemap(&self) -> BTreeMap<i64, Quat64> {
        let mut map = BTreeMap::new();
        for (dt_us, q) in &self.quats {
            map.insert(*dt_us, *q);
        }
        map
    }

    /// Duration of this buffer in milliseconds (based on first/last timestamps).
    pub fn duration_ms(&self) -> f64 {
        if self.quats.len() < 2 {
            return 0.0;
        }
        let mut first_ts = 0;
        let mut last_ts = 0;
        if let Some((first_key, first_value)) = self.quats.iter().next() {
            first_ts = *first_key;
        } 

        // Get the last element (largest key)
        if let Some((last_key, last_value)) = self.quats.iter().next_back() {
            last_ts = *last_key;
        }

        (last_ts - first_ts) as f64 / 1000.0
    }

    pub fn from_csv_samples_range(
        samples: &[crate::gyro_source::csv_quats::CsvQuatSample],
        start_us: i64,
        end_us: i64,
    ) -> Option<Self> {
        if samples.is_empty() || end_us < start_us {
            return None;
        }

        let mut map: TimeQuat = TimeQuat::new();

        // Insert only samples in [start_us, end_us]
        for s in samples.iter() {
            if s.t_us < start_us { continue; }
            if s.t_us > end_us { break; }

            // CSV: w,x,y,z
            // nalgebra::Quaternion::new(w, i, j, k) where internal storage is [i, j, k, w]
            let q = NQuat::new(s.qw, s.qx, s.qy, s.qz);

            // Make it unit (safe even if slightly off due to float export)
            let uq: Quat64 = NUnitQuat::new_normalize(q);

            map.insert(s.t_us, uq);
        }

        QuatBuffer::from_btreemap(&map)
    }
}

#[derive(Debug, Default)]
pub struct QuatBufferStore {
    dq: RwLock<VecDeque<Arc<QuatBuffer>>>,
    version: AtomicU64,
}

impl QuatBufferStore {
    pub fn new() -> Self {
        Self {
            dq: RwLock::new(VecDeque::new()),
            version: AtomicU64::new(0),
        }
    }

    /// Publish a new buffer (no capacity-based deletion here).
    pub fn publish(&self, buf: QuatBuffer) -> (Arc<QuatBuffer>, u64) {
        let arc = Arc::new(buf);
        {
            let mut w: parking_lot::lock_api::RwLockWriteGuard<'_, parking_lot::RawRwLock, VecDeque<Arc<QuatBuffer>>> = self.dq.write();
            w.push_back(arc.clone());
        }
        let ver = self.version.fetch_add(1, Ordering::SeqCst) + 1;
        (arc, ver)
    }

    /// Select the **newest** buffer where `t_ms` is (a) covered with padding and (b) roughly centered.
    /// Then prune any **older** buffers that also center the same `t_ms`.
    ///
    /// If none are centered, optionally fall back to newest *covering* buffer (if `fallback_ok`).
    pub fn select_centered_and_prune(
        &self,
        t_ms: f64,
        pre_ms: f64,
        post_ms: f64,
        center_ratio: f64,
        fallback_ok: bool,
    ) -> Option<(Arc<QuatBuffer>, u64)>
    {
        let t_us    = (t_ms * 1000.0) as i64;
        let pre_us  = (pre_ms * 1000.0) as i64;
        let post_us = (post_ms * 1000.0) as i64;

        // 1) Read-pass: find best candidate index (newest-first).
        let (cand_idx, fallback_idx) = {
            let r = self.dq.read();
            let mut centered_idx: Option<usize> = None;
            let mut cover_idx:    Option<usize> = None;

            for (i, buf) in r.iter().enumerate().rev() {
                if buf.covers_with_padding(t_us, pre_us, post_us) {
                    if cover_idx.is_none() { cover_idx = Some(i); }
                    if buf.is_centered_for(t_us, center_ratio) {
                        centered_idx = Some(i);
                        break; // newest centered wins
                    }
                }
            }
            (centered_idx, cover_idx)
        };

        // Prefer centered; else maybe fallback to covering.
        let chosen_idx = cand_idx.or(if fallback_ok { fallback_idx } else { None })?;

        // 2) Write-pass: clone chosen buffer, then prune older centered ones.
        let (chosen_arc, ver) = {
            let mut w = self.dq.write();

            // Clone the chosen buffer for return
            let chosen = w.get(chosen_idx).cloned()?;
            let ver = self.version.load(Ordering::Relaxed);

            // Remove any **older** buffers (front..chosen_idx) that ALSO center the same frame.
            // We walk from front to just before chosen_idx, keeping those that do NOT center.
            let mut i = 0_usize;
            while i < chosen_idx && i < w.len() {
                // Invariant: `w.len()` can change as we remove.
                if let Some(buf) = w.get(i) {
                    if buf.is_centered_for(t_us, center_ratio) && buf.covers_with_padding(t_us, pre_us, post_us) {
                        w.remove(i);            // remove; do NOT advance i
                        // Because we removed at i < chosen_idx, the chosen_idx shifts left by 1.
                        // But we don't need chosen_idx anymore.
                        continue;
                    }
                }
                i += 1;
            }

            (chosen, ver)
        };

        Some((chosen_arc, ver))
    }

    pub fn get_quat_at_time(
    &self,
    t_ms: f64,
    pre_ms: f64,
    post_ms: f64,
    center_ratio: f64,
) -> Option<Quat64> {
    let (buf, _ver) = self
        .select_centered_and_prune(t_ms, pre_ms, post_ms, center_ratio, true)?;
    buf.quat_at_ms(t_ms)
}
/// Get a buffer that covers a window around `t_ms`, without pruning the store.
    pub fn get_buffer_for_time(
        &self,
        t_ms: f64,
        pre_ms: f64,
        post_ms: f64,
        center_ratio: f64,
    ) -> Option<Arc<QuatBuffer>> {
        let (buf, _ver) =
            self.select_centered_and_prune(t_ms, pre_ms, post_ms, center_ratio, false)?;
        Some(buf)
    }



    pub fn get_latest_buffer(&self) -> Option<Arc<QuatBuffer>> {
        let r = self.dq.read();
        r.back().cloned()
    }

    /// Load quaternions from a CSV (org or stab depending on `stabbed`) and publish them
    /// as sliding windows: each window is 3 seconds long, next window starts 1 second later.
    ///
    /// Returns (num_windows_published, last_version).
    pub fn load_from_csv_sliding_windows<P: AsRef<Path>>(
        &self,
        path: P,
        stabbed: bool,
    ) -> Result<(usize, u64), Box<dyn std::error::Error + Send + Sync>> {
        let samples = csv_quats::load_quat_samples_from_csv(path, stabbed)?;
        if samples.len() < 2 {
            return Ok((0, self.version.load(Ordering::Relaxed)));
        }

        // Window config
        let window_us: i64 = 3_000_000;
        let step_us: i64   = 1_000_000;

        let first_us = samples.first().unwrap().t_us;
        let last_us  = samples.last().unwrap().t_us;

        if last_us - first_us < 1000 {
            return Ok((0, self.version.load(Ordering::Relaxed)));
        }

        // Two-pointer to avoid O(N^2) scanning
        let mut win_start = first_us;
        let mut i0: usize = 0;
        let mut i1: usize = 0;

        let mut published = 0_usize;
        let mut last_ver = self.version.load(Ordering::Relaxed);

        while win_start <= last_us {
            let win_end = win_start + window_us;

            // If the window is completely beyond data, stop.
            if win_start > last_us {
                break;
            }

            // Advance i0 until samples[i0].t_us >= win_start
            while i0 < samples.len() && samples[i0].t_us < win_start {
                i0 += 1;
            }

            // Ensure i1 >= i0
            if i1 < i0 { i1 = i0; }

            // Advance i1 until samples[i1].t_us > win_end (so [i0, i1) is in window)
            while i1 < samples.len() && samples[i1].t_us <= win_end {
                i1 += 1;
            }

            // Build buffer from that slice
            if i0 < i1 {
                if let Some(buf) = QuatBuffer::from_csv_samples_range(&samples[i0..i1], win_start, win_end) {
                    let (_arc, ver) = self.publish(buf);
                    published += 1;
                    last_ver = ver;
                }
            }

            win_start += step_us;
        }

        Ok((published, last_ver))
    }

}



pub struct LiveState {
    pub header: String,
    pub ring: Mutex<ImuRing>,
    pub sync: LiveClockSync,
    pub quat_buffer_store_org: QuatBufferStore,
    pub quat_buffer_store_smoothed: QuatBufferStore,
    pub enabled: AtomicBool,
}

impl Default for LiveState {
     fn default() -> Self {
         Self {
             header: String::new(),
             // default keep_us=3s; enable_live will override when constructing
             ring: Mutex::new(ImuRing::new(3_000_000)),
             sync: LiveClockSync::default(),
             quat_buffer_store_org: QuatBufferStore::new(),
             quat_buffer_store_smoothed: QuatBufferStore::new(),
             enabled: AtomicBool::new(false),
         }
     }

}

impl LiveState {
    pub fn enable_live(&self, keep_secs: f64) {
        let keep_us = (keep_secs * 1_000_000.0).round() as i64;
        let mut ring = self.ring.lock();
        ring.keep_us = keep_us;
        self.enabled.store(true, Ordering::Relaxed);
    }

    pub fn disable_live(&self) {
        self.enabled.store(false, Ordering::Relaxed);
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled.load(Ordering::Relaxed)
    }

    pub fn load_quats_from_csv_sliding_windows<P: AsRef<Path>>(
        &self,
        path: P,
    ){
            self.quat_buffer_store_smoothed
                .load_from_csv_sliding_windows(&path, true);
            self.quat_buffer_store_org
                .load_from_csv_sliding_windows(&path, false);
        
    }
}

