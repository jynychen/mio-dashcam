//! GPS-track overlay renderer.
//!
//! Produces a square, transparent video (`qtrle` in a `.mov`, the only widely
//! supported lossless-with-alpha combination) holding the whole drive as a
//! polyline: the part already driven is thick and white, the part still ahead is
//! thin and grey, and an arrow marks the current position and heading.
//!
//! Everything is drawn with signed distance fields and composited in linear
//! light with premultiplied alpha, so the result survives being scaled and
//! blended over 4K footage.
//!
//! What to draw is decided in [`crate::track`]: this module receives a
//! [`TrackLayout`] already projected into the frame and is only responsible for
//! turning it into pixels.

use std::cell::RefCell;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use anyhow::{Context, Result, bail};
use rayon::prelude::*;

use crate::gps::PositionSource;
use crate::track::{MarkerFrame, Segment, TrackLayout};

/// How much of the track immediately behind the vehicle is left undrawn, in
/// seconds.  The arrow covers the seam, so neither styling needs to meet the
/// marker exactly.
const TRAIL_GAP_SECS: f32 = 0.25;

// ── Visual parameters ─────────────────────────────────────────────────────────
// Colors are linear-light; compositing uses premultiplied alpha (Porter-Duff).
// Conversion to premultiplied sRGB u8 happens once, in the final pass.

/// SDF radius (half-width) of the not-yet-driven track.
const TRACK_RADIUS_THIN: f32 = 3.0;
/// SDF radius of the already-driven track.
const TRACK_RADIUS_THICK: f32 = 4.0;
/// How far the arrow tip reaches ahead of the position.
const ARROW_FRONT: f32 = 10.0;
/// How far the arrow's base corners sit behind the position.
const ARROW_BACK: f32 = 6.0;
/// Depth of the notch cut into the arrow's base; must be under `ARROW_BACK`.
const ARROW_NOTCH_BACK: f32 = 3.0;
/// Half-width of the arrow base.
const ARROW_HALF_BASE: f32 = 7.0;
/// Outline drawn around the arrow so it stays readable over a bright road.
const ARROW_OUTLINE: f32 = 2.0;

/// Pixel (px, py) is sampled at its centre, following the usual
/// half-open-interval convention where pixel i covers [i, i+1).
const PIXEL_CENTER: f32 = 0.5;

const TRACK_FUTURE: ColorLinA =
    ColorLinA::new([192.0 / 255.0, 192.0 / 255.0, 192.0 / 255.0], 130.0 / 255.0);
const TRACK_PAST: ColorLinA = ColorLinA::new([1.0, 1.0, 1.0], 200.0 / 255.0);
const ARROW_FILL: ColorLinA = ColorLinA::new([1.0, 50.0 / 255.0, 50.0 / 255.0], 1.0);
const ARROW_STROKE: ColorLinA = ColorLinA::new([192.0 / 255.0, 192.0 / 255.0, 192.0 / 255.0], 1.0);

#[derive(Copy, Clone)]
struct ColorLinA {
    rgb: [f32; 3],
    alpha: f32,
}

impl ColorLinA {
    const fn new(rgb: [f32; 3], alpha: f32) -> Self {
        Self { rgb, alpha }
    }
}

thread_local! {
    /// Per-thread f32 scratch buffer, reused across frames to keep the render
    /// loop allocation-free.  A rayon worker only ever touches its own.
    static WORK_BUF: RefCell<Vec<f32>> = const { RefCell::new(Vec::new()) };
}

/// Render `layout` to a transparent overlay video and return its path.
pub fn render(layout: &TrackLayout, output_dir: &Path, name: &str, fps: f64) -> Result<PathBuf> {
    if fps <= 0.0 {
        bail!("fps must be positive (got {fps})");
    }
    if layout.frames.is_empty() {
        bail!("no GPS frames to render");
    }
    if layout.segments.is_empty() {
        bail!("no GPS track to render");
    }

    let size = layout.size;
    let output_file = output_dir.join(format!("{name}_overlay.mov"));
    let raster = TrackRaster::precompute(&layout.segments, size, TRACK_RADIUS_THICK);

    let mut ffmpeg = Command::new("ffmpeg")
        .args([
            "-y",
            "-loglevel",
            "warning",
            "-f",
            "rawvideo",
            // ffmpeg has no premultiplied-RGBA format; "rgba" in and "argb" out
            // is a pure channel reorder, so the premultiplied bytes reach qtrle
            // unchanged.
            "-pix_fmt",
            "rgba",
            "-s",
            &format!("{size}x{size}"),
            "-r",
            &format!("{fps}"),
            "-i",
            "pipe:0",
            "-c:v",
            "qtrle",
            "-pix_fmt",
            "argb",
            output_file
                .to_str()
                .context("overlay output path is not valid UTF-8")?,
        ])
        .stdin(Stdio::piped())
        .spawn()
        .context("failed to spawn ffmpeg for the overlay")?;

    let stdin = ffmpeg.stdin.as_mut().context("no ffmpeg stdin")?;
    let frame_len = (size * size * 4) as usize;
    // Two chunks per thread keeps every core busy without buffering the world.
    let chunk_size = rayon::current_num_threads() * 2;

    let result = (|| -> Result<()> {
        for chunk_start in (0..layout.frames.len()).step_by(chunk_size) {
            let chunk_end = (chunk_start + chunk_size).min(layout.frames.len());
            let chunk: Vec<Vec<u8>> = (chunk_start..chunk_end)
                .into_par_iter()
                .map(|frame_idx| {
                    let mut buf = vec![0u8; frame_len];
                    WORK_BUF.with(|cell| {
                        let mut work = cell.borrow_mut();
                        work.resize(frame_len, 0.0);
                        render_frame(&mut buf, &mut work, &raster, layout, frame_idx, fps as f32);
                    });
                    buf
                })
                .collect();

            for frame in &chunk {
                stdin.write_all(frame)?;
            }
        }
        Ok(())
    })();

    // Always close the pipe and reap the child, even after a write error.
    drop(ffmpeg.stdin.take());
    let status = ffmpeg.wait()?;
    result?;
    if !status.success() {
        bail!("ffmpeg overlay encoding failed");
    }

    Ok(output_file)
}

// ── Track pre-computation ─────────────────────────────────────────────────────
//
// The polyline is identical in every frame; only its past/future split moves.
// Rasterising the distance field once turns per-frame work from "test every
// segment against every pixel" into a flat O(size²) lookup:
//
//   precompute (once):  O(segments × pixels_per_segment)
//   per frame:          O(size²)
//
// Storage is CSR (compressed sparse row): `offsets[i]..offsets[i+1]` bounds the
// entries of pixel i, ordered by segment index.

struct TrackRaster {
    /// `(segment index, distance from pixel centre)`.  Only the raw distance is
    /// kept — coverage is derived per frame so past and future can differ in
    /// width.
    entries: Vec<(u32, f32)>,
    /// CSR row pointers; length `size² + 1`, last element is the entry count.
    offsets: Vec<u32>,
    /// `(t0, t1)` per segment, split out of the geometry so the per-frame
    /// past/future test walks 8 bytes a segment instead of 24.
    spans: Vec<(f32, f32)>,
}

impl TrackRaster {
    /// `max_radius` is the widest half-width ever drawn; it decides which
    /// pixels are recorded at all.
    fn precompute(segments: &[Segment], size: u32, max_radius: f32) -> Self {
        let n = (size * size) as usize;
        let outer = max_radius + PIXEL_CENTER;

        // Pass 1: count the segments touching each pixel.
        let mut counts = vec![0u32; n];
        for_each_covered_pixel(segments, size, outer, max_radius, |idx, _, _| {
            counts[idx] += 1;
        });

        let mut offsets = Vec::with_capacity(n + 1);
        let mut total = 0u32;
        for &c in &counts {
            offsets.push(total);
            total += c;
        }
        offsets.push(total);

        // Pass 2: fill. Segments are visited in index order, so each pixel's
        // span comes out sorted by segment index for free.
        let mut entries = vec![(0u32, 0.0f32); total as usize];
        let mut cursors = vec![0u32; n];
        for_each_covered_pixel(segments, size, outer, max_radius, |idx, seg, d| {
            let pos = (offsets[idx] + cursors[idx]) as usize;
            entries[pos] = (seg, d);
            cursors[idx] += 1;
        });

        TrackRaster {
            entries,
            offsets,
            spans: segments.iter().map(|s| (s.t0, s.t1)).collect(),
        }
    }
}

/// Visit every (pixel, segment) pair the track covers, in segment order.
///
/// Shared by both CSR passes so the counting and filling traversals can never
/// disagree about which pixels a segment touches.
fn for_each_covered_pixel(
    segments: &[Segment],
    size: u32,
    outer: f32,
    max_radius: f32,
    mut visit: impl FnMut(usize, u32, f32),
) {
    for (seg_idx, seg) in segments.iter().enumerate() {
        if !(seg.x0.is_finite() && seg.x1.is_finite()) {
            continue;
        }
        let (px_min, px_max, py_min, py_max) = segment_pixel_bounds(seg, outer, size);
        for py in py_min..=py_max {
            let row = (py * size) as usize;
            for px in px_min..=px_max {
                let d = dist_to_segment(px as f32 + PIXEL_CENTER, py as f32 + PIXEL_CENTER, seg);
                if aa_coverage(d, max_radius) > 0.0 {
                    visit(row + px as usize, seg_idx as u32, d);
                }
            }
        }
    }
}

// ── Frame rendering ───────────────────────────────────────────────────────────

/// Render one frame into `buf` as premultiplied sRGB RGBA.
///
/// `work` is a scratch buffer of `size² × 4` f32s: all compositing happens in
/// linear light before a single conversion at the end.  A segment is behind the
/// vehicle once the whole of it has been driven, which is what lets the split
/// move along the track independently of where its vertices fall.
fn render_frame(
    buf: &mut [u8],
    work: &mut [f32],
    raster: &TrackRaster,
    layout: &TrackLayout,
    frame_idx: usize,
    fps: f32,
) {
    work.fill(0.0);
    let now = frame_idx as f32 / fps;
    let npx = (layout.size * layout.size) as usize;

    // Layer 1: the track — future underneath, past on top so the brighter,
    // thicker styling wins wherever the route crosses itself.
    for i in 0..npx {
        let start = raster.offsets[i] as usize;
        let end = raster.offsets[i + 1] as usize;
        if start == end {
            continue;
        }
        let off = i * 4;

        // Segments of the same colour are merged by coverage union
        // (P(A∪B) = A + B(1−A)) rather than stacked with src-over, which would
        // otherwise build up opacity at crossings.
        let mut future_cov = 0.0f32;
        let mut past_cov = 0.0f32;
        for &(seg, d) in &raster.entries[start..end] {
            let (t0, t1) = raster.spans[seg as usize];
            if t0 >= now {
                let c = aa_coverage(d, TRACK_RADIUS_THIN);
                future_cov += c * (1.0 - future_cov);
            } else if t1 <= now - TRAIL_GAP_SECS {
                let c = aa_coverage(d, TRACK_RADIUS_THICK);
                past_cov += c * (1.0 - past_cov);
            }
        }
        if future_cov > 0.0 {
            composite(work, off, TRACK_FUTURE.rgb, future_cov * TRACK_FUTURE.alpha);
        }
        if past_cov > 0.0 {
            composite(work, off, TRACK_PAST.rgb, past_cov * TRACK_PAST.alpha);
        }
    }

    // Layer 2: the position marker.
    if let Some(marker) = layout.frames.get(frame_idx)
        && marker.x.is_finite()
    {
        // Gap-filled positions get the neutral arrow colour instead of red.
        let fill = if marker.source == PositionSource::HardwareFix {
            ARROW_FILL
        } else {
            ARROW_STROKE
        };
        draw_arrow(work, layout.size, marker, fill);
    }

    // Final pass: premultiplied linear f32 → premultiplied sRGB u8.
    // The gamma curve must be applied to straight colour, so un-premultiply,
    // convert, then premultiply again. Emitting straight alpha instead would be
    // double-premultiplied when the player composites the overlay.
    for i in 0..npx {
        let si = i * 4;
        let a = work[si + 3];
        if a < 1.0 / 255.0 {
            buf[si..si + 4].fill(0);
        } else {
            let a_out = a.min(1.0);
            let inv_a = 1.0 / a_out;
            for c in 0..3 {
                let v = linear_to_srgb(work[si + c] * inv_a) * a_out;
                buf[si + c] = (v * 255.0 + 0.5) as u8;
            }
            buf[si + 3] = (a_out * 255.0 + 0.5) as u8;
        }
    }
}

// ── Drawing helpers ───────────────────────────────────────────────────────────

/// Distance from `(px, py)` to `seg` — the SDF value anti-aliasing is derived
/// from.
#[inline]
fn dist_to_segment(px: f32, py: f32, seg: &Segment) -> f32 {
    let dx = seg.x1 - seg.x0;
    let dy = seg.y1 - seg.y0;
    let len_sq = dx * dx + dy * dy;
    // A stationary vehicle produces zero-length segments; treat as a point.
    if len_sq < 1e-6 {
        return ((px - seg.x0).powi(2) + (py - seg.y0).powi(2)).sqrt();
    }
    let t = (((px - seg.x0) * dx + (py - seg.y0) * dy) / len_sq).clamp(0.0, 1.0);
    ((px - (seg.x0 + t * dx)).powi(2) + (py - (seg.y0 + t * dy)).powi(2)).sqrt()
}

/// Pixel bounding box of a segment, grown by `outer` and clamped to the frame.
#[inline]
fn segment_pixel_bounds(seg: &Segment, outer: f32, size: u32) -> (u32, u32, u32, u32) {
    let s = (size - 1) as f32;
    (
        (seg.x0.min(seg.x1) - outer).floor().clamp(0.0, s) as u32,
        (seg.x0.max(seg.x1) + outer).ceil().clamp(0.0, s) as u32,
        (seg.y0.min(seg.y1) - outer).floor().clamp(0.0, s) as u32,
        (seg.y0.max(seg.y1) + outer).ceil().clamp(0.0, s) as u32,
    )
}

/// Hermite smoothstep: 0 at `edge0`, 1 at `edge1`.
#[inline]
fn smoothstep(edge0: f32, edge1: f32, x: f32) -> f32 {
    let t = ((x - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Coverage of a shape whose edge sits at `radius`, for SDF value `d`.
/// The one-pixel transition band replaces multisampling.
#[inline]
fn aa_coverage(d: f32, radius: f32) -> f32 {
    1.0 - smoothstep(radius - 0.5, radius + 0.5, d)
}

#[inline]
fn linear_to_srgb(c: f32) -> f32 {
    let c = c.clamp(0.0, 1.0);
    if c <= 0.0031308 {
        c * 12.92
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    }
}

/// Porter-Duff source-over onto a premultiplied linear buffer.
#[inline]
fn composite(work: &mut [f32], off: usize, rgb_lin: [f32; 3], alpha: f32) {
    let inv = 1.0 - alpha;
    for c in 0..3 {
        work[off + c] = rgb_lin[c] * alpha + work[off + c] * inv;
    }
    work[off + 3] = alpha + work[off + 3] * inv;
}

/// Draw the anti-aliased position marker: a notched arrowhead pointing along
/// the frame's heading (clockwise from north), with a contrasting outline.
fn draw_arrow(work: &mut [f32], size: u32, marker: &MarkerFrame, fill: ColorLinA) {
    let (cx, cy) = (marker.x, marker.y);
    // A heading is unavailable while stopped; point north rather than vanish.
    let theta = if marker.heading.is_finite() {
        marker.heading.to_radians()
    } else {
        0.0
    };
    // Forward unit vector in screen space (y grows downward, so north is −y).
    let (fx, fy) = (theta.sin(), -theta.cos());
    // Screen-right perpendicular.
    let (rx, ry) = (-fy, fx);

    // Vertices, wound counter-clockwise so `edge_dist` is positive inside.
    let tip = (cx + fx * ARROW_FRONT, cy + fy * ARROW_FRONT);
    let base_r = (
        cx - fx * ARROW_BACK + rx * ARROW_HALF_BASE,
        cy - fy * ARROW_BACK + ry * ARROW_HALF_BASE,
    );
    let base_l = (
        cx - fx * ARROW_BACK - rx * ARROW_HALF_BASE,
        cy - fy * ARROW_BACK - ry * ARROW_HALF_BASE,
    );
    // The base's midpoint, pulled forward into an indent.
    let notch = (cx - fx * ARROW_NOTCH_BACK, cy - fy * ARROW_NOTCH_BACK);

    let margin = ARROW_OUTLINE + 1.0;
    let s = (size - 1) as f32;
    let px_min = (tip.0.min(base_r.0).min(base_l.0) - margin).clamp(0.0, s) as u32;
    let px_max = (tip.0.max(base_r.0).max(base_l.0) + margin).clamp(0.0, s) as u32;
    let py_min = (tip.1.min(base_r.1).min(base_l.1) - margin).clamp(0.0, s) as u32;
    let py_max = (tip.1.max(base_r.1).max(base_l.1) + margin).clamp(0.0, s) as u32;

    for py in py_min..=py_max {
        for px in px_min..=px_max {
            let p = (px as f32 + PIXEL_CENTER, py as f32 + PIXEL_CENTER);
            // The arrowhead is the union of two triangles — tip/notch/base_l and
            // tip/base_r/notch — so its SDF is the max of theirs.
            let left = edge_dist(p, tip, notch)
                .min(edge_dist(p, notch, base_l))
                .min(edge_dist(p, base_l, tip));
            let right = edge_dist(p, tip, base_r)
                .min(edge_dist(p, base_r, notch))
                .min(edge_dist(p, notch, tip));
            let sdf = left.max(right);

            let idx = ((py * size + px) * 4) as usize;
            // Outline first: the same shape, grown by ARROW_OUTLINE.
            let cov_outline = smoothstep(-0.5, 0.5, sdf + ARROW_OUTLINE);
            if cov_outline > 0.0 {
                composite(
                    work,
                    idx,
                    ARROW_STROKE.rgb,
                    cov_outline * ARROW_STROKE.alpha,
                );
            }
            let cov_fill = smoothstep(-0.5, 0.5, sdf);
            if cov_fill > 0.0 {
                composite(work, idx, fill.rgb, cov_fill * fill.alpha);
            }
        }
    }
}

/// Signed distance from `p` to the directed edge `a → b`, positive on the left,
/// which is the interior for a counter-clockwise triangle.
#[inline]
fn edge_dist(p: (f32, f32), a: (f32, f32), b: (f32, f32)) -> f32 {
    let dx = b.0 - a.0;
    let dy = b.1 - a.1;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-6 {
        return 0.0;
    }
    ((p.0 - a.0) * (-dy) + (p.1 - a.1) * dx) / len
}

#[cfg(test)]
mod tests {
    use super::*;

    fn segment(x0: f32, y0: f32, x1: f32, y1: f32, t0: f32, t1: f32) -> Segment {
        Segment {
            x0,
            y0,
            x1,
            y1,
            t0,
            t1,
        }
    }

    fn marker(x: f32, y: f32) -> MarkerFrame {
        MarkerFrame {
            x,
            y,
            heading: 90.0,
            source: PositionSource::HardwareFix,
        }
    }

    /// A horizontal track of `n` one-pixel, one-second segments.
    fn layout(n: usize, size: u32) -> TrackLayout {
        TrackLayout {
            size,
            segments: (0..n)
                .map(|i| {
                    let x = 10.0 + i as f32;
                    segment(x, 10.0, x + 1.0, 10.0, i as f32, i as f32 + 1.0)
                })
                .collect(),
            frames: (0..n).map(|i| marker(10.0 + i as f32, 10.0)).collect(),
        }
    }

    #[test]
    fn coverage_saturates_inside_and_vanishes_outside() {
        assert_eq!(aa_coverage(0.0, 3.0), 1.0);
        assert_eq!(aa_coverage(10.0, 3.0), 0.0);
        let edge = aa_coverage(3.0, 3.0);
        assert!(edge > 0.4 && edge < 0.6, "edge coverage was {edge}");
    }

    #[test]
    fn distance_to_a_segment_projects_onto_it_and_clamps_at_the_ends() {
        let s = segment(0.0, 0.0, 10.0, 0.0, 0.0, 1.0);
        assert!((dist_to_segment(5.0, 3.0, &s) - 3.0).abs() < 1e-6);
        // Beyond the far end, distance is measured to the endpoint itself.
        assert!((dist_to_segment(14.0, 0.0, &s) - 4.0).abs() < 1e-6);
        // A stationary vehicle: the segment collapses to a point.
        let dot = segment(2.0, 2.0, 2.0, 2.0, 0.0, 1.0);
        assert!((dist_to_segment(0.0, 5.0, &dot) - 3.605551).abs() < 1e-5);
    }

    #[test]
    fn the_raster_covers_the_line_and_nothing_else() {
        let l = layout(20, 48);
        let raster = TrackRaster::precompute(&l.segments, l.size, TRACK_RADIUS_THICK);

        let covered = |x: u32, y: u32| {
            let i = (y * l.size + x) as usize;
            raster.offsets[i + 1] > raster.offsets[i]
        };
        assert!(covered(20, 10), "the middle of the track must be covered");
        assert!(!covered(20, 40), "far from the track must be empty");
        assert_eq!(raster.offsets.len(), (l.size * l.size + 1) as usize);
    }

    /// Alpha the track layer alone writes at one pixel, in frame `frame_idx`.
    fn track_alpha(l: &TrackLayout, x: u32, y: u32, frame_idx: usize, fps: f32) -> f32 {
        // No marker frames: the arrow must not be measured with the track.
        let bare = TrackLayout {
            size: l.size,
            segments: l.segments.clone(),
            frames: Vec::new(),
        };
        let npx = (l.size * l.size) as usize;
        let raster = TrackRaster::precompute(&bare.segments, bare.size, TRACK_RADIUS_THICK);
        let mut work = vec![0.0f32; npx * 4];
        let mut buf = vec![0u8; npx * 4];
        render_frame(&mut buf, &mut work, &raster, &bare, frame_idx, fps);
        work[((y * l.size + x) as usize) * 4 + 3]
    }

    #[test]
    fn the_split_between_driven_and_ahead_follows_the_clock() {
        // A minute of driving, one pixel a second, with the vehicle at x = 40
        // after 30 s.
        let l = layout(60, 96);
        let behind = track_alpha(&l, 20, 10, 30, 1.0);
        let ahead = track_alpha(&l, 60, 10, 30, 1.0);
        assert!(behind > ahead, "driven {behind} vs ahead {ahead}");
        assert!((behind - TRACK_PAST.alpha).abs() < 1e-3, "{behind}");
        assert!((ahead - TRACK_FUTURE.alpha).abs() < 1e-3, "{ahead}");

        // Earlier in the drive that first pixel has not been reached yet.
        let earlier = track_alpha(&l, 20, 10, 5, 1.0);
        assert!((earlier - TRACK_FUTURE.alpha).abs() < 1e-3, "{earlier}");
    }

    #[test]
    fn crossings_do_not_pile_up_opacity() {
        // Two passes over the same stretch of road, an hour apart.
        let mut l = layout(4, 48);
        l.segments.extend(
            (0..4).map(|i| segment(10.0 + i as f32, 10.0, 11.0 + i as f32, 10.0, 3600.0, 3601.0)),
        );
        // Both passes are behind the vehicle, yet the pixel stays at one layer.
        let a = track_alpha(&l, 12, 10, 4000, 1.0);
        assert!((a - TRACK_PAST.alpha).abs() < 1e-3, "{a}");
    }

    #[test]
    fn the_marker_is_drawn_over_the_track() {
        let l = layout(20, 48);
        let raster = TrackRaster::precompute(&l.segments, l.size, TRACK_RADIUS_THICK);
        let npx = (l.size * l.size) as usize;
        let mut work = vec![0.0f32; npx * 4];
        let mut buf = vec![0u8; npx * 4];
        render_frame(&mut buf, &mut work, &raster, &l, 10, 1.0);

        // The arrow sits on the marker: opaque, and red rather than white.
        let px = ((10 * l.size + 20) as usize) * 4;
        assert!((work[px + 3] - 1.0).abs() < 1e-3, "alpha {}", work[px + 3]);
        assert!(work[px] > work[px + 1], "the arrow should be red");
        // …and the frame is emitted premultiplied, transparent where untouched.
        assert_eq!(&buf[0..4], &[0, 0, 0, 0]);
        assert_eq!(buf[px + 3], 255);
    }
}
