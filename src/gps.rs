//! Session timeline assembly and per-frame GPS interpolation.
//!
//! Two problems have to be solved before the 1 Hz NMEA log can drive a 30 fps
//! overlay:
//!
//! 1. **Placing each record on the session timeline.** The concatenated video
//!    starts at t=0 and runs for the summed duration of its clips; a record has
//!    only a UTC instant.  Clip *n* begins at the sum of the durations before
//!    it, and the record's offset inside that clip is its UTC time converted to
//!    the camera's local zone minus the clip's file-name timestamp.  The logger
//!    usually needs a few seconds to produce its first record, so this offset is
//!    genuinely non-zero and cannot be replaced by a record index.
//!
//! 2. **Upsampling 1 Hz → video frame rate.** Position, ground speed and
//!    heading are each fitted with a natural cubic spline over the timeline, so
//!    the marker glides instead of stepping once a second.  The same splines
//!    bridge dropouts, where the receiver reported no fix at all.

use anyhow::Result;
use chrono::{Duration, FixedOffset, NaiveDateTime};

use crate::nmea;
use crate::scan::Group;

/// A record whose in-clip offset falls outside `[-SLACK, duration + SLACK]`
/// does not belong to that clip — the logger kept writing after the video was
/// closed, or the file is stale.  Such records are dropped.
const CLIP_SLACK: f64 = 3.0;

/// Two control points closer together than this are treated as the same
/// instant.  Cubic-spline construction requires strictly increasing x.
const MIN_SPACING: f64 = 1e-3;

/// A frame further than this from the nearest 1 Hz record is reported as
/// [`PositionSource::Interpolated`] rather than a hardware fix.
const FIX_MAX_AGE: f64 = 1.5;

/// How far beyond the first/last valid fix the track is still drawn. Outside
/// this the spline would only clamp to its endpoint, pinning the marker to a
/// stale position for as long as the dropout lasts.
const EXTRAPOLATION_GRACE: f64 = 5.0;

/// Heading is meaningless when the vehicle is crawling: the receiver's reported
/// course jitters wildly below this speed, so those records are excluded from
/// the heading spline.
const MIN_HEADING_SPEED_KMH: f64 = 5.0;

/// How a frame's position was obtained.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionSource {
    /// Backed by a 1 Hz record that carried a real receiver fix.
    HardwareFix,
    /// Spline-filled across a dropout.
    Interpolated,
    /// Nothing to draw — before the first fix, after the last, or no fix at all.
    NoPosition,
}

/// One video frame's worth of interpolated GPS state.
#[derive(Debug, Clone)]
pub struct TrackPoint {
    /// Decimal degrees; `NaN` when [`PositionSource::NoPosition`].
    pub lat: f64,
    pub lon: f64,
    /// Degrees true, continuous across the 360°→0° wrap; `NaN` when unknown.
    pub heading: f64,
    pub source: PositionSource,
}

impl TrackPoint {
    pub fn drawable(&self) -> bool {
        self.source != PositionSource::NoPosition
    }
}

/// A 1 Hz record positioned on the session timeline.
#[derive(Debug, Clone)]
struct Entry {
    /// Seconds from the start of the concatenated session.
    t: f64,
    fix: bool,
    lat: f64,
    lon: f64,
    speed: f64,
    track: f64,
}

/// Everything parsed from one session's NMEA sidecars.
pub struct Session {
    entries: Vec<Entry>,
    /// Number of records that carried a real fix.
    pub fix_count: usize,
    /// Local-time offset used to align UTC records with the file names.
    pub tz: FixedOffset,
}

impl Session {
    pub fn record_count(&self) -> usize {
        self.entries.len()
    }
}

/// Read the NMEA sidecars of `group` and lay their records on the session
/// timeline.
///
/// `tz` pins the camera's local-time zone; `None` infers it from the offset
/// between the first record's UTC time and its clip's file-name timestamp.
pub fn load_session(group: &Group, tz: Option<FixedOffset>) -> Result<Session> {
    let mut parsed: Vec<(usize, Vec<nmea::Rmc>)> = Vec::new();
    for (idx, clip) in group.clips.iter().enumerate() {
        if let Some(path) = &clip.nmea {
            let records = nmea::parse_file(path)?;
            if !records.is_empty() {
                parsed.push((idx, records));
            }
        }
    }

    let tz = tz
        .or_else(|| infer_tz(group, &parsed))
        .unwrap_or_else(|| FixedOffset::east_opt(0).expect("UTC is a valid offset"));

    // Offset of each clip within the concatenated output.
    let mut clip_offsets = Vec::with_capacity(group.clips.len());
    let mut acc = 0.0;
    for clip in &group.clips {
        clip_offsets.push(acc);
        acc += clip.duration;
    }

    let mut entries: Vec<Entry> = Vec::new();
    for (idx, records) in parsed {
        let clip = &group.clips[idx];
        for r in records {
            let local = r.utc + tz;
            let in_clip = seconds_between(clip.start, local);
            if in_clip < -CLIP_SLACK || in_clip > clip.duration + CLIP_SLACK {
                continue;
            }
            entries.push(Entry {
                t: clip_offsets[idx] + in_clip.clamp(0.0, clip.duration),
                fix: r.fix,
                lat: r.lat,
                lon: r.lon,
                speed: r.speed,
                track: r.track,
            });
        }
    }

    entries.sort_by(|a, b| a.t.total_cmp(&b.t));
    entries.dedup_by(|a, b| a.t - b.t < MIN_SPACING);

    let fix_count = entries.iter().filter(|e| e.fix).count();
    Ok(Session {
        entries,
        fix_count,
        tz,
    })
}

/// Infer the camera's UTC offset from the gap between a clip's file-name time
/// (local) and its first log record (UTC), rounded to the nearest quarter hour
/// so the logger's few-second start-up delay cannot skew it.
fn infer_tz(group: &Group, parsed: &[(usize, Vec<nmea::Rmc>)]) -> Option<FixedOffset> {
    let (idx, records) = parsed.first()?;
    let first = records.first()?;
    let raw = seconds_between(first.utc, group.clips[*idx].start);
    let quarters = (raw / 900.0).round() as i32;
    FixedOffset::east_opt(quarters * 900)
}

fn seconds_between(from: NaiveDateTime, to: NaiveDateTime) -> f64 {
    (to - from).num_milliseconds() as f64 / 1000.0
}

/// Resample a session to one [`TrackPoint`] per video frame.
pub fn build_track(session: &Session, fps: f64, duration: f64) -> Vec<TrackPoint> {
    let total_frames = (duration * fps).ceil().max(0.0) as usize;
    let splines = Splines::build(&session.entries);

    (0..total_frames)
        .map(|frame| {
            let t = frame as f64 / fps;
            let (lat, lon) = splines.position(t);
            TrackPoint {
                lat,
                lon,
                heading: splines.heading(t),
                source: splines.source(&session.entries, t),
            }
        })
        .collect()
}

// ── Spline set ────────────────────────────────────────────────────────────────

struct Splines {
    lat: Option<CubicSpline>,
    lon: Option<CubicSpline>,
    heading: Option<CubicSpline>,
    /// Fallback used when there are too few control points for a spline.
    single_fix: Option<(f64, f64)>,
    single_heading: Option<f64>,
    /// Timeline range covered by real fixes, widened by the grace period.
    span: Option<(f64, f64)>,
}

impl Splines {
    fn build(entries: &[Entry]) -> Self {
        let fixes: Vec<&Entry> = entries.iter().filter(|e| e.fix).collect();

        let lat_pts: Vec<(f64, f64)> = fixes.iter().map(|e| (e.t, e.lat)).collect();
        let lon_pts: Vec<(f64, f64)> = fixes.iter().map(|e| (e.t, e.lon)).collect();
        let heading_pts = unwrap_headings(&fixes);

        let span = match (fixes.first(), fixes.last()) {
            (Some(first), Some(last)) => {
                Some((first.t - EXTRAPOLATION_GRACE, last.t + EXTRAPOLATION_GRACE))
            }
            _ => None,
        };

        Splines {
            lat: CubicSpline::try_new(&lat_pts),
            lon: CubicSpline::try_new(&lon_pts),
            heading: CubicSpline::try_new(&heading_pts),
            single_fix: fixes.first().map(|e| (e.lat, e.lon)),
            single_heading: heading_pts.first().map(|&(_, h)| h),
            span,
        }
    }

    fn position(&self, t: f64) -> (f64, f64) {
        match (&self.lat, &self.lon) {
            (Some(lat), Some(lon)) => (lat.eval(t), lon.eval(t)),
            _ => self.single_fix.unwrap_or((f64::NAN, f64::NAN)),
        }
    }

    fn heading(&self, t: f64) -> f64 {
        match &self.heading {
            Some(sp) => sp.eval(t).rem_euclid(360.0),
            None => self
                .single_heading
                .map_or(f64::NAN, |h| h.rem_euclid(360.0)),
        }
    }

    /// Classify a frame: a real fix within [`FIX_MAX_AGE`], a spline fill inside
    /// the fix span, or nothing at all.
    fn source(&self, entries: &[Entry], t: f64) -> PositionSource {
        let Some((from, to)) = self.span else {
            return PositionSource::NoPosition;
        };
        if t < from || t > to {
            return PositionSource::NoPosition;
        }

        // Nearest record at or before t — records are one second apart, so the
        // preceding one is what the frame is "standing on".
        let floor = entries.partition_point(|e| e.t <= t).checked_sub(1);
        match floor.map(|i| &entries[i]) {
            Some(e) if e.fix && t - e.t <= FIX_MAX_AGE => PositionSource::HardwareFix,
            _ => PositionSource::Interpolated,
        }
    }
}

/// Build heading control points with the 360°→0° wrap removed.
///
/// A spline cannot follow a discontinuity, so each angle is carried forward by
/// the shortest signed step from the previous one; the result is continuous and
/// is wrapped back into `[0, 360)` at evaluation time.
fn unwrap_headings(fixes: &[&Entry]) -> Vec<(f64, f64)> {
    let mut pts: Vec<(f64, f64)> = Vec::new();
    for e in fixes {
        if e.track.is_nan() || e.speed.is_nan() || e.speed < MIN_HEADING_SPEED_KMH {
            continue;
        }
        let unwrapped = match pts.last() {
            Some(&(_, prev)) => {
                let mut delta = e.track - prev.rem_euclid(360.0);
                if delta > 180.0 {
                    delta -= 360.0;
                } else if delta < -180.0 {
                    delta += 360.0;
                }
                prev + delta
            }
            None => e.track,
        };
        pts.push((e.t, unwrapped));
    }
    pts
}

// ── Natural cubic spline ──────────────────────────────────────────────────────

/// Natural cubic spline (second derivative zero at both ends).
pub struct CubicSpline {
    xs: Vec<f64>,
    ys: Vec<f64>,
    /// Second derivative at each knot.
    ms: Vec<f64>,
}

impl CubicSpline {
    /// Build a spline, or `None` when there are fewer than two control points.
    ///
    /// `data` must be sorted by x with strictly increasing values.
    pub fn try_new(data: &[(f64, f64)]) -> Option<Self> {
        if data.len() < 2 {
            return None;
        }
        debug_assert!(
            data.windows(2).all(|w| w[0].0 < w[1].0),
            "control points must be strictly increasing in x"
        );

        let n = data.len() - 1;
        let xs: Vec<f64> = data.iter().map(|&(x, _)| x).collect();
        let ys: Vec<f64> = data.iter().map(|&(_, y)| y).collect();
        let h: Vec<f64> = (0..n).map(|i| xs[i + 1] - xs[i]).collect();

        let ms = if n == 1 {
            // A two-point spline is a straight line.
            vec![0.0, 0.0]
        } else {
            // Symmetric tridiagonal system for the interior second derivatives
            // M_1..M_{n-1}; the boundary values M_0 = M_n = 0 are fixed.
            let m = n - 1;
            let mut diag = vec![0.0f64; m];
            let mut off_diag = vec![0.0f64; m - 1];
            let mut rhs = vec![0.0f64; m];

            for (row, knot) in (1..n).enumerate() {
                diag[row] = 2.0 * (h[knot - 1] + h[knot]);
                rhs[row] = 6.0
                    * ((ys[knot + 1] - ys[knot]) / h[knot]
                        - (ys[knot] - ys[knot - 1]) / h[knot - 1]);
                if row < m - 1 {
                    off_diag[row] = h[knot];
                }
            }

            let mut ms = vec![0.0f64; n + 1];
            ms[1..=m].copy_from_slice(&solve_tridiagonal(&diag, &off_diag, &rhs));
            ms
        };

        Some(CubicSpline { xs, ys, ms })
    }

    /// Evaluate at `x`, clamping to the endpoint value outside the knot range.
    pub fn eval(&self, x: f64) -> f64 {
        let n = self.xs.len() - 1;
        if x <= self.xs[0] {
            return self.ys[0];
        }
        if x >= self.xs[n] {
            return self.ys[n];
        }
        let i = self
            .xs
            .partition_point(|&xi| xi <= x)
            .saturating_sub(1)
            .min(n - 1);
        let h = self.xs[i + 1] - self.xs[i];
        let a = (self.xs[i + 1] - x) / h;
        let b = (x - self.xs[i]) / h;
        a * self.ys[i]
            + b * self.ys[i + 1]
            + (h * h / 6.0) * ((a * a * a - a) * self.ms[i] + (b * b * b - b) * self.ms[i + 1])
    }
}

/// Solve a symmetric tridiagonal system with the Thomas algorithm.
///
/// `off_diag` is shared by the sub- and super-diagonal, which the natural cubic
/// spline's system allows.
fn solve_tridiagonal(diag: &[f64], off_diag: &[f64], rhs: &[f64]) -> Vec<f64> {
    let m = diag.len();
    debug_assert_eq!(off_diag.len(), m - 1);
    debug_assert_eq!(rhs.len(), m);

    let mut d = diag.to_vec();
    let mut r = rhs.to_vec();

    for i in 1..m {
        let factor = off_diag[i - 1] / d[i - 1];
        d[i] -= factor * off_diag[i - 1];
        r[i] -= factor * r[i - 1];
    }

    let mut sol = vec![0.0f64; m];
    sol[m - 1] = r[m - 1] / d[m - 1];
    for i in (0..m - 1).rev() {
        sol[i] = (r[i] - off_diag[i] * sol[i + 1]) / d[i];
    }
    sol
}

/// Chrono's `Duration` is re-exported for callers that format session spans.
pub fn format_duration(seconds: f64) -> String {
    let d = Duration::seconds(seconds as i64);
    let (h, m, s) = (d.num_hours(), d.num_minutes() % 60, d.num_seconds() % 60);
    if h > 0 {
        format!("{h}:{m:02}:{s:02}")
    } else {
        format!("{m}:{s:02}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(t: f64, fix: bool, lat: f64, lon: f64, speed: f64, track: f64) -> Entry {
        Entry {
            t,
            fix,
            lat,
            lon,
            speed,
            track,
        }
    }

    #[test]
    fn spline_passes_through_its_control_points_and_clamps_outside() {
        let sp = CubicSpline::try_new(&[(0.0, 0.0), (1.0, 10.0), (2.0, 20.0)]).unwrap();
        assert!((sp.eval(1.0) - 10.0).abs() < 1e-9);
        assert!((sp.eval(-5.0) - 0.0).abs() < 1e-9);
        assert!((sp.eval(99.0) - 20.0).abs() < 1e-9);
        // Evenly spaced collinear points stay collinear.
        assert!((sp.eval(0.5) - 5.0).abs() < 1e-9);
    }

    #[test]
    fn spline_needs_two_points() {
        assert!(CubicSpline::try_new(&[]).is_none());
        assert!(CubicSpline::try_new(&[(0.0, 1.0)]).is_none());
    }

    #[test]
    fn heading_unwraps_across_north() {
        let fixes = [
            entry(0.0, true, 25.0, 121.0, 40.0, 350.0),
            entry(1.0, true, 25.0, 121.0, 40.0, 10.0),
        ];
        let pts = unwrap_headings(&fixes.iter().collect::<Vec<_>>());
        // 350 → 10 is +20°, not −340°.
        assert!((pts[1].1 - 370.0).abs() < 1e-9);

        // …and evaluation wraps it back into range, passing through north.
        let splines = Splines::build(&fixes);
        assert!((splines.heading(0.5) - 0.0).abs() < 1e-6);
    }

    #[test]
    fn slow_records_do_not_contribute_heading() {
        let fixes = [
            entry(0.0, true, 25.0, 121.0, 0.3, 12.0),
            entry(1.0, true, 25.0, 121.0, 0.2, 300.0),
        ];
        assert!(unwrap_headings(&fixes.iter().collect::<Vec<_>>()).is_empty());
    }

    #[test]
    fn dropouts_are_interpolated_and_the_edges_are_blank() {
        let entries = vec![
            entry(10.0, true, 25.0, 121.0, 40.0, 90.0),
            entry(11.0, false, f64::NAN, f64::NAN, f64::NAN, f64::NAN),
            entry(12.0, true, 25.001, 121.001, 40.0, 90.0),
        ];
        let splines = Splines::build(&entries);

        assert_eq!(splines.source(&entries, 10.0), PositionSource::HardwareFix);
        assert_eq!(splines.source(&entries, 11.2), PositionSource::Interpolated);
        // Far outside the fix span there is nothing to draw.
        assert_eq!(splines.source(&entries, 0.0), PositionSource::NoPosition);
        assert_eq!(splines.source(&entries, 60.0), PositionSource::NoPosition);

        // The gap-filled position sits between its neighbours.
        let (lat, _) = splines.position(11.0);
        assert!(lat > 25.0 && lat < 25.001, "lat was {lat}");
    }

    #[test]
    fn a_session_without_any_fix_draws_nothing() {
        let entries = vec![entry(0.0, false, f64::NAN, f64::NAN, f64::NAN, f64::NAN)];
        let splines = Splines::build(&entries);
        assert_eq!(splines.source(&entries, 0.0), PositionSource::NoPosition);
        assert!(splines.position(0.0).0.is_nan());
    }
}
