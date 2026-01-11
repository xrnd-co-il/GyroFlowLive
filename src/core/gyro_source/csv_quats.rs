// gyro_source/csv_quats.rs
use std::path::Path;

use anyhow::{anyhow, Context, Result};
use csv::StringRecord;

/// Fixed column order (0-based indices) per your list.
#[allow(dead_code)]
pub mod col {
    pub const FRAME: usize = 0;
    pub const TIMESTAMP_MS: usize = 1;

    pub const ORG_ACC_X: usize = 2;
    pub const ORG_ACC_Y: usize = 3;
    pub const ORG_ACC_Z: usize = 4;

    pub const ORG_PITCH: usize = 5;
    pub const ORG_YAW: usize = 6;
    pub const ORG_ROLL: usize = 7;

    pub const ORG_GYRO_X: usize = 8;
    pub const ORG_GYRO_Y: usize = 9;
    pub const ORG_GYRO_Z: usize = 10;

    pub const ORG_QUAT_W: usize = 11;
    pub const ORG_QUAT_X: usize = 12;
    pub const ORG_QUAT_Y: usize = 13;
    pub const ORG_QUAT_Z: usize = 14;


    pub const FOCUS_DISTANCE: usize = 15;

    pub const STAB_PITCH: usize = 16;
    pub const STAB_YAW: usize = 17;
    pub const STAB_ROLL: usize = 18;

    pub const STAB_QUAT_W: usize = 19;
    pub const STAB_QUAT_X: usize = 20;
    pub const STAB_QUAT_Y: usize = 21;
    pub const STAB_QUAT_Z: usize = 22;

    pub const FOCAL_LENGTH: usize = 23;
    pub const FOV_SCALE: usize = 24;
    pub const MINIMAL_FOV_SCALE: usize = 25;

    pub const NUM_COLS: usize = 26;
}

#[derive(Clone, Copy, Debug)]
pub struct CsvQuatSample {
    pub t_us: i64,
    // stored as (w,x,y,z) because that’s what CSV gives us; caller can reorder.
    pub qw: f64,
    pub qx: f64,
    pub qy: f64,
    pub qz: f64,
}

fn parse_f64(rec: &StringRecord, idx: usize) -> Result<f64> {
    let s = rec
        .get(idx)
        .ok_or_else(|| anyhow!("Missing column idx={idx}"))?
        .trim();
    s.parse::<f64>()
        .with_context(|| format!("Failed parsing f64 at col {idx}: '{s}'"))
}

fn parse_i64_from_ms_to_us(rec: &StringRecord, idx_ms: usize) -> Result<i64> {
    let ms = parse_f64(rec, idx_ms)?;
    // robust rounding
    Ok((ms * 1000.0).round() as i64)
}

/// Load only the quaternion stream you care about (org vs stab).
/// Returns samples in file order; caller may sort/dedupe.
pub fn load_quat_samples_from_csv(path: impl AsRef<Path>, stabbed: bool) -> Result<Vec<CsvQuatSample>> {
    let path = path.as_ref();
    let mut rdr = csv::ReaderBuilder::new()
        .has_headers(true)
        .flexible(false)
        .from_path(path)
        .with_context(|| format!("Failed opening CSV: {:?}", path))?;

    let mut out = Vec::new();

    for (line_idx, row) in rdr.records().enumerate() {
        let rec = row.with_context(|| format!("CSV read error at line {}", line_idx + 2))?; // +2 for header line
        if rec.len() != col::NUM_COLS {
            return Err(anyhow!(
                "CSV column count mismatch at line {}: expected {}, got {}",
                line_idx + 2,
                col::NUM_COLS,
                rec.len()
            ));
        }

        let t_us = parse_i64_from_ms_to_us(&rec, col::TIMESTAMP_MS)?;

        let (w_idx, x_idx, y_idx, z_idx) = if stabbed {
            (col::STAB_QUAT_W, col::STAB_QUAT_X, col::STAB_QUAT_Y, col::STAB_QUAT_Z)
        } else {
            (col::ORG_QUAT_W, col::ORG_QUAT_X, col::ORG_QUAT_Y, col::ORG_QUAT_Z)
        };

        let qw = parse_f64(&rec, w_idx)?;
        let qx = parse_f64(&rec, x_idx)?;
        let qy = parse_f64(&rec, y_idx)?;
        let qz = parse_f64(&rec, z_idx)?;

        // Optional: skip invalid rows (NaN/inf)
        if !(qw.is_finite() && qx.is_finite() && qy.is_finite() && qz.is_finite()) {
            continue;
        }

        out.push(CsvQuatSample { t_us, qw, qx, qy, qz });
    }

    Ok(out)
}
