use exr::prelude::*;
use std::io::Cursor;
use crate::live_pix_fmt::{LiveFrame, LivePixFmt};
use exr::image::pixel_vec::PixelVec;

#[derive(Clone, Copy, Debug)]
pub enum RenderMapKind { Distort, Undistort }

#[inline]
fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo { lo } else if v > hi { hi } else { v }
}

type RgbaF32 = (f32, f32, f32, f32);

fn decode_stmap_from_exr(
    exr_bytes: &[u8],
    out_w: usize,
    out_h: usize,
) -> Option<(usize, usize, Vec<f32>)> {
    // Read first RGBA layer, largest res, from &[u8] into PixelVec<(f32,f32,f32,f32)>
    let img: exr::image::RgbaImage<PixelVec<RgbaF32>> =
        exr::image::read::read()
            .no_deep_data()
            .largest_resolution_level()
            .rgba_channels(
                PixelVec::<RgbaF32>::constructor, // allocate pixel storage
                PixelVec::<RgbaF32>::set_pixel,    // write a pixel
            )
            .first_valid_layer()
            .all_attributes()
            .from_buffered(Cursor::new(exr_bytes))
            .ok()?; // Option<_>

    let src_w = img.layer_data.size.x();
    let src_h = img.layer_data.size.y();

    let w = out_w.max(src_w);
    let h = out_h.max(src_h);

    // Flattened RGBA tuples live here:
    let pixels: &[(f32,f32,f32,f32)] = &img.layer_data.channel_data.pixels.pixels;

    let mut coords = vec![0.0f32; w * h * 2];
    for (i, &(r, g, _b, _a)) in pixels.iter().enumerate() {
        let x_src = i % src_w;
        let y_src = i / src_w;
        if x_src < w && y_src < h {
            let idx = y_src * w + x_src;
            coords[idx * 2]     = r * w as f32;           // X = R * width
            coords[idx * 2 + 1] = (1.0 - g) * h as f32;   // Y = (1-G) * height
        }
    }

    Some((w, h, coords))
}

fn bilinear_sample_rgb24(src: &[u8], w: usize, h: usize, u: f32, v: f32) -> [u8; 4] {
    if w == 0 || h == 0 { return [0,0,0,255]; }
    let u = clamp(u, 0.0, (w as f32) - 1.0);
    let v = clamp(v, 0.0, (h as f32) - 1.0);
    let x0 = u.floor() as usize;
    let y0 = v.floor() as usize;
    let x1 = (x0 + 1).min(w - 1);
    let y1 = (y0 + 1).min(h - 1);
    let tx = u - (x0 as f32);
    let ty = v - (y0 as f32);
    let idx = |x: usize, y: usize| -> usize { (y * w + x) * 3 };
    let c00 = &src[idx(x0, y0)..idx(x0, y0)+3];
    let c10 = &src[idx(x1, y0)..idx(x1, y0)+3];
    let c01 = &src[idx(x0, y1)..idx(x0, y1)+3];
    let c11 = &src[idx(x1, y1)..idx(x1, y1)+3];
    let lerp = |a: f32, b: f32, t: f32| a + (b - a) * t;
    let mut out = [0u8; 4];
    for ch in 0..3 {
        let a = lerp(c00[ch] as f32, c10[ch] as f32, tx);
        let b = lerp(c01[ch] as f32, c11[ch] as f32, tx);
        out[ch] = lerp(a, b, ty).round().clamp(0.0, 255.0) as u8;
    }
    out[3] = 255;
    out
}

fn bilinear_sample_nv12_to_rgba(src: &[u8], w: usize, h: usize, u: f32, v: f32) -> [u8; 4] {
    let y_plane_size = w * h;
    if src.len() < y_plane_size + w * (h / 2) { return [0,0,0,255]; }
    let y_plane = &src[..y_plane_size];
    let uv_plane = &src[y_plane_size..];
    let clamp_u = clamp(u, 0.0, (w as f32) - 1.0);
    let clamp_v = clamp(v, 0.0, (h as f32) - 1.0);
    let y = y_plane[(clamp_v as usize * w + clamp_u as usize).min(y_plane.len()-1)] as f32;
    let uv_idx = ((clamp_v as usize / 2) * w + (clamp_u as usize & !1)).min(uv_plane.len()-2);
    let u_ = uv_plane[uv_idx] as f32;
    let v_ = uv_plane[uv_idx + 1] as f32;
    let c = y - 16.0;
    let d = u_ - 128.0;
    let e = v_ - 128.0;
    [
        (1.164 * c + 1.596 * e).clamp(0.0,255.0) as u8,
        (1.164 * c - 0.392 * d - 0.813 * e).clamp(0.0,255.0) as u8,
        (1.164 * c + 2.017 * d).clamp(0.0,255.0) as u8,
        255
    ]
}

#[inline]
fn rgba_to_rgb(rgba: &[u8], rgb: &mut [u8]) {
    let mut s = 0usize;
    let mut d = 0usize;
    while s + 3 < rgba.len() {
        rgb[d..d+3].copy_from_slice(&rgba[s..s+3]);
        s += 4;
        d += 3;
    }
}

pub fn render_with_maps_to_rgb24(
    frame: &LiveFrame,
    dist_exr: &[u8],
    undist_exr: &[u8],
    which: RenderMapKind,
) -> Option<(u32, u32, Vec<u8>)> {
    let (map_w, map_h, coords) = match which {
        RenderMapKind::Undistort => decode_stmap_from_exr(undist_exr, frame.width as usize, frame.height as usize)?,
        RenderMapKind::Distort => decode_stmap_from_exr(dist_exr, frame.width as usize, frame.height as usize)?,
    };
    let mut out_rgba = vec![0u8; map_w * map_h * 4];
    match frame.pix_fmt {
        LivePixFmt::Rgb24 => {
            for y in 0..map_h {
                for x in 0..map_w {
                    let idx = y * map_w + x;
                    let u = coords[idx * 2];
                    let v = coords[idx * 2 + 1];
                    let px = bilinear_sample_rgb24(&frame.data, frame.width as usize, frame.height as usize, u, v);
                    out_rgba[idx*4..idx*4+4].copy_from_slice(&px);
                }
            }
        }
        LivePixFmt::Nv12 => {
            for y in 0..map_h {
                for x in 0..map_w {
                    let idx = y * map_w + x;
                    let u = coords[idx * 2];
                    let v = coords[idx * 2 + 1];
                    let px = bilinear_sample_nv12_to_rgba(&frame.data, frame.width as usize, frame.height as usize, u, v);
                    out_rgba[idx*4..idx*4+4].copy_from_slice(&px);
                }
            }
        }
    }
    let mut out_rgb = vec![0u8; map_w * map_h * 3];
    rgba_to_rgb(&out_rgba, &mut out_rgb);
    Some((map_w as u32, map_h as u32, out_rgb))
}
