//! GPS-track overlay renderer.
//!
//! Produces a square, transparent video (`qtrle` in a `.mov`, the only widely
//! supported lossless-with-alpha combination) holding the whole drive as a
//! polyline: the part already driven is thick and white, the part still ahead is
//! thin and grey, and an arrow marks the current position and heading.
//!
//! Ported from the masigo-dashcam renderer.  Everything is drawn with signed
//! distance fields and composited in linear light with premultiplied alpha, so
//! the result survives being scaled and blended over 4K footage.

use std::cell::RefCell;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use anyhow::{Context, Result, bail};
use rayon::prelude::*;

use crate::gps::{PositionSource, TrackPoint};

/// Fraction of the frame left blank around the track's bounding box.
const PADDING_RATIO: f64 = 0.05;
/// How long a segment takes to switch from "ahead" to "behind" styling.
const FADE_SECS: f64 = 0.25;

// ── Visual parameters ─────────────────────────────────────────────────────────
// Colors are linear-light; compositing uses premultiplied alpha (Porter-Duff).
// Conversion to premultiplied sRGB u8 happens once, in the final pass.

/// SDF radius (half-width) of the not-yet-driven track.
const TRACK_RADIUS_THIN: f32 = 2.0;
/// SDF radius of the already-driven track.
const TRACK_RADIUS_THICK: f32 = 3.0;
/// How far the arrow tip reaches ahead of the position.
const ARROW_FRONT: f32 = 10.0;
/// How far the arrow's base corners sit behind the position.
const ARROW_BACK: f32 = 6.0;
/// Depth of the notch cut into the arrow's base; must be under `ARROW_BACK`.
const ARROW_NOTCH_BACK: f32 = 3.0;
/// Half-width of the arrow base.
const ARROW_HALF_BASE: f32 = 6.0;
/// Outline drawn around the arrow so it stays readable over a bright road.
const ARROW_OUTLINE: f32 = 1.5;

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

/// One frame of the track projected into pixel space.
struct PixelFrame {
    /// `NAN` when the frame has no drawable position.
    x: f32,
    y: f32,
    heading: f32,
    /// Gap-filled positions get the neutral arrow colour instead of red.
    interpolated: bool,
}

thread_local! {
    /// Per-thread f32 scratch buffer, reused across frames to keep the render
    /// loop allocation-free.  A rayon worker only ever touches its own.
    static WORK_BUF: RefCell<Vec<f32>> = const { RefCell::new(Vec::new()) };
}

/// Render `track` to a transparent overlay video and return its path.
///
/// `track` must hold exactly one point per output frame.
pub fn render(
    track: &[TrackPoint],
    output_dir: &Path,
    name: &str,
    fps: f64,
    size: u32,
) -> Result<PathBuf> {
    if fps <= 0.0 {
        bail!("fps must be positive (got {fps})");
    }
    if track.is_empty() {
        bail!("no GPS frames to render");
    }

    let output_file = output_dir.join(format!("{name}_overlay.mov"));

    // One viewport for the whole video: the map must not drift under the track.
    let viewport =
        Viewport::fit(track, size).context("no usable GPS positions for the overlay viewport")?;

    let frames_px: Vec<PixelFrame> = track
        .iter()
        .map(|p| {
            let (x, y) = if p.drawable() {
                viewport.to_pixel(p.lat, p.lon)
            } else {
                (f32::NAN, f32::NAN)
            };
            PixelFrame {
                x,
                y,
                heading: p.heading as f32,
                interpolated: p.source == PositionSource::Interpolated,
            }
        })
        .collect();

    let raster = TrackRaster::precompute(&frames_px, size, TRACK_RADIUS_THICK);

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
    let fade_frames = (FADE_SECS * fps).round().max(1.0) as u32;
    // Two chunks per thread keeps every core busy without buffering the world.
    let chunk_size = rayon::current_num_threads() * 2;

    let result = (|| -> Result<()> {
        for chunk_start in (0..frames_px.len()).step_by(chunk_size) {
            let chunk_end = (chunk_start + chunk_size).min(frames_px.len());
            let chunk: Vec<Vec<u8>> = (chunk_start..chunk_end)
                .into_par_iter()
                .map(|frame_idx| {
                    let mut buf = vec![0u8; frame_len];
                    WORK_BUF.with(|cell| {
                        let mut work = cell.borrow_mut();
                        work.resize(frame_len, 0.0);
                        render_frame(
                            &mut buf,
                            &mut work,
                            &raster,
                            &frames_px,
                            frame_idx,
                            size,
                            fade_frames,
                        );
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

// ── Viewport: degrees → pixels ────────────────────────────────────────────────
// Equirectangular projection with a cos(lat) correction, which is accurate well
// past the tens-of-kilometres scale of a dashcam session.

struct Viewport {
    center_lat: f64,
    center_lon: f64,
    px_per_deg_lat: f64,
    px_per_deg_lon: f64,
    half: f64,
}

impl Viewport {
    fn fit(track: &[TrackPoint], size: u32) -> Option<Self> {
        let mut positions = track
            .iter()
            .filter(|p| p.drawable())
            .map(|p| (p.lat, p.lon));

        let (first_lat, first_lon) = positions.next()?;
        let (min_lat, max_lat, min_lon, max_lon) = positions.fold(
            (first_lat, first_lat, first_lon, first_lon),
            |(mn_la, mx_la, mn_lo, mx_lo), (lat, lon)| {
                (
                    mn_la.min(lat),
                    mx_la.max(lat),
                    mn_lo.min(lon),
                    mx_lo.max(lon),
                )
            },
        );

        let center_lat = (min_lat + max_lat) / 2.0;
        let center_lon = (min_lon + max_lon) / 2.0;

        // A degree of longitude shrinks toward the poles: 111 km × cos(lat),
        // against a constant 111 km for latitude.
        let cos_lat = center_lat.to_radians().cos().abs().max(1e-6);
        let span_lat = (max_lat - min_lat).max(1e-6);
        let span_lon = (max_lon - min_lon).max(1e-6) * cos_lat;

        // Scale both axes from the dominant one so the track is not distorted.
        let usable = size as f64 * (1.0 - 2.0 * PADDING_RATIO);
        let px_per_deg_lat = usable / span_lat.max(span_lon);

        Some(Viewport {
            center_lat,
            center_lon,
            px_per_deg_lat,
            px_per_deg_lon: px_per_deg_lat * cos_lat,
            half: size as f64 / 2.0,
        })
    }

    fn to_pixel(&self, lat: f64, lon: f64) -> (f32, f32) {
        let x = self.half + (lon - self.center_lon) * self.px_per_deg_lon;
        let y = self.half - (lat - self.center_lat) * self.px_per_deg_lat;
        (x as f32, y as f32)
    }
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
    /// kept — coverage is derived per frame so the fade can vary the radius.
    entries: Vec<(u32, f32)>,
    /// CSR row pointers; length `size² + 1`, last element is the entry count.
    offsets: Vec<u32>,
}

impl TrackRaster {
    /// `max_radius` is the widest half-width ever drawn; it decides which
    /// pixels are recorded at all.
    fn precompute(track: &[PixelFrame], size: u32, max_radius: f32) -> Self {
        let n = (size * size) as usize;
        let outer = max_radius + PIXEL_CENTER;

        // Pass 1: count the segments touching each pixel.
        let mut counts = vec![0u32; n];
        for_each_covered_pixel(track, size, outer, max_radius, |idx, _, _| {
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
        for_each_covered_pixel(track, size, outer, max_radius, |idx, seg, d| {
            let pos = (offsets[idx] + cursors[idx]) as usize;
            entries[pos] = (seg, d);
            cursors[idx] += 1;
        });

        TrackRaster { entries, offsets }
    }
}

/// Visit every (pixel, segment) pair the track covers, in segment order.
///
/// Shared by both CSR passes so the counting and filling traversals can never
/// disagree about which pixels a segment touches.
fn for_each_covered_pixel(
    track: &[PixelFrame],
    size: u32,
    outer: f32,
    max_radius: f32,
    mut visit: impl FnMut(usize, u32, f32),
) {
    for seg_idx in 0..track.len().saturating_sub(1) {
        let (x0, y0) = (track[seg_idx].x, track[seg_idx].y);
        let (x1, y1) = (track[seg_idx + 1].x, track[seg_idx + 1].y);
        if !x0.is_finite() || !x1.is_finite() {
            continue;
        }
        let (px_min, px_max, py_min, py_max) = segment_pixel_bounds(x0, y0, x1, y1, outer, size);
        for py in py_min..=py_max {
            let row = (py * size) as usize;
            for px in px_min..=px_max {
                let d = dist_to_segment(
                    px as f32 + PIXEL_CENTER,
                    py as f32 + PIXEL_CENTER,
                    x0,
                    y0,
                    x1,
                    y1,
                );
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
/// `work` is a scratch buffer of `size² × 4` f32s, zeroed on entry: all
/// compositing happens in linear light before a single conversion at the end.
/// Segment `s` is behind the vehicle iff `s < frame_idx`.
fn render_frame(
    buf: &mut [u8],
    work: &mut [f32],
    raster: &TrackRaster,
    frames_px: &[PixelFrame],
    frame_idx: usize,
    size: u32,
    fade_frames: u32,
) {
    work.fill(0.0);
    let cur_seg = frame_idx as u32;
    let npx = (size * size) as usize;

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
            if seg >= cur_seg {
                let c = aa_coverage(d, TRACK_RADIUS_THIN);
                future_cov += c * (1.0 - future_cov);
            } else if cur_seg - seg >= fade_frames {
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
    if let Some(p) = frames_px.get(frame_idx)
        && p.x.is_finite()
    {
        let fill = if p.interpolated {
            ARROW_STROKE
        } else {
            ARROW_FILL
        };
        draw_arrow(work, size, p.x, p.y, p.heading, fill);
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

/// Distance from `(px, py)` to the segment `(x0,y0)→(x1,y1)` — the SDF value
/// anti-aliasing is derived from.
#[inline]
fn dist_to_segment(px: f32, py: f32, x0: f32, y0: f32, x1: f32, y1: f32) -> f32 {
    let dx = x1 - x0;
    let dy = y1 - y0;
    let len_sq = dx * dx + dy * dy;
    // A stationary vehicle produces zero-length segments; treat as a point.
    if len_sq < 1e-6 {
        return ((px - x0).powi(2) + (py - y0).powi(2)).sqrt();
    }
    let t = (((px - x0) * dx + (py - y0) * dy) / len_sq).clamp(0.0, 1.0);
    ((px - (x0 + t * dx)).powi(2) + (py - (y0 + t * dy)).powi(2)).sqrt()
}

/// Pixel bounding box of a segment, grown by `outer` and clamped to the frame.
#[inline]
fn segment_pixel_bounds(
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32,
    outer: f32,
    size: u32,
) -> (u32, u32, u32, u32) {
    let s = (size - 1) as f32;
    (
        (x0.min(x1) - outer).floor().clamp(0.0, s) as u32,
        (x0.max(x1) + outer).ceil().clamp(0.0, s) as u32,
        (y0.min(y1) - outer).floor().clamp(0.0, s) as u32,
        (y0.max(y1) + outer).ceil().clamp(0.0, s) as u32,
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
/// `heading_deg` (clockwise from north), with a contrasting outline.
fn draw_arrow(work: &mut [f32], size: u32, cx: f32, cy: f32, heading_deg: f32, fill: ColorLinA) {
    // A heading is unavailable while stopped; point north rather than vanish.
    let theta = if heading_deg.is_finite() {
        heading_deg.to_radians()
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
    use crate::gps::PositionSource;

    fn point(lat: f64, lon: f64) -> TrackPoint {
        TrackPoint {
            lat,
            lon,
            heading: 90.0,
            source: PositionSource::HardwareFix,
        }
    }

    #[test]
    fn viewport_centres_the_track_and_keeps_it_inside() {
        let track = vec![point(25.0, 121.0), point(25.01, 121.02)];
        let vp = Viewport::fit(&track, 480).unwrap();

        let (x0, y0) = vp.to_pixel(25.0, 121.0);
        let (x1, y1) = vp.to_pixel(25.01, 121.02);
        for v in [x0, y0, x1, y1] {
            assert!((0.0..=480.0).contains(&v), "{v} escaped the frame");
        }
        // Latitude grows upward on the map but downward in pixels.
        assert!(y1 < y0);
        assert!(x1 > x0);
        // The midpoint lands at the centre of the frame.
        let (cx, cy) = vp.to_pixel(25.005, 121.01);
        assert!((cx - 240.0).abs() < 0.5 && (cy - 240.0).abs() < 0.5);
    }

    #[test]
    fn a_degenerate_track_still_yields_a_viewport() {
        // A vehicle parked with the engine running reports one position forever.
        let track = vec![point(25.0, 121.0), point(25.0, 121.0)];
        let vp = Viewport::fit(&track, 480).unwrap();
        let (x, y) = vp.to_pixel(25.0, 121.0);
        assert!(x.is_finite() && y.is_finite());
    }

    #[test]
    fn viewport_needs_at_least_one_position() {
        let mut p = point(25.0, 121.0);
        p.source = PositionSource::NoPosition;
        assert!(Viewport::fit(&[p], 480).is_none());
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
        assert!((dist_to_segment(5.0, 3.0, 0.0, 0.0, 10.0, 0.0) - 3.0).abs() < 1e-6);
        // Beyond the far end, distance is measured to the endpoint itself.
        assert!((dist_to_segment(14.0, 0.0, 0.0, 0.0, 10.0, 0.0) - 4.0).abs() < 1e-6);
        assert!((dist_to_segment(0.0, 5.0, 2.0, 2.0, 2.0, 2.0) - 3.605551).abs() < 1e-5);
    }

    #[test]
    fn the_raster_covers_the_line_between_two_points_only() {
        let frames = vec![
            PixelFrame {
                x: 10.0,
                y: 10.0,
                heading: 90.0,
                interpolated: false,
            },
            PixelFrame {
                x: 30.0,
                y: 10.0,
                heading: 90.0,
                interpolated: false,
            },
        ];
        let size = 48;
        let raster = TrackRaster::precompute(&frames, size, TRACK_RADIUS_THICK);

        let covered = |x: u32, y: u32| {
            let i = (y * size + x) as usize;
            raster.offsets[i + 1] > raster.offsets[i]
        };
        assert!(covered(20, 10), "midpoint of the segment must be covered");
        assert!(!covered(20, 40), "far from the segment must be empty");
        assert_eq!(raster.offsets.len(), (size * size + 1) as usize);
    }
}
