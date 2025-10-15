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

#[derive(Debug, Clone)]
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
}

#[derive(Debug)]
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
            let mut w = self.dq.write();
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

}


#[derive(Default)]
pub struct LiveState {
    pub header: String,
    pub ring: ImuRing,
    pub sync: LiveClockSync,
    pub quat_buffer_store_org: QuatBufferStore,
    pub quat_buffer_store_smoothed: QuatBufferStore,
    pub enabled: bool,
}