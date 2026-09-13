//! What the overlay draws, in pixel space: the trace, and one marker per frame.
//!
//! This is the geometry half of `rc_map_overlay/src/gauge.rs` — RaceChrono's
//! map gauge, `p000/ex1.java` — which does three things before any pixel is
//! touched:
//!
//! 1. thin the samples ([`Decimation`], from `lx2.m5741o(0.2, 3.0, 20, 2)`),
//! 2. fit a [`MercatorView`] around what survives, with a margin,
//! 3. project the polyline and the position marker into that viewport.
//!
//! The painting half stays in [`crate::overlay`], which draws mio's own style
//! rather than RaceChrono's white-line-on-black-outline.
//!
//! One addition the gauge has no need for: `ex1` draws a static lap, while an
//! overlay video redraws the same trace 30 times a second with the split
//! between "already driven" and "still ahead" moving along it.  Each drawn
//! segment therefore carries the time span it covers, and long segments are
//! subdivided so that split can land mid-second.

use crate::gps::{Fix, PositionSource, Session};
use crate::mercator::{GeoBounds, MercatorView};
use crate::motion::MotionSeries;

/// Fraction of the frame left blank around the track's bounding box.
const PADDING_RATIO: f32 = 0.05;

/// Longest hole in the log still bridged.  A dropout beyond this breaks the
/// trace and hides the marker instead of drawing a straight line through a
/// tunnel; it is also what [`MotionSeries::player`] is given, so the two agree
/// on where the track stops existing.
const MAX_GAP_S: f64 = 10.0;

/// Sample thinning, from `lx2.m5741o(0.2, 3.0, 20, 2)`: keep a sample once it
/// is both `min_time_s` and `min_distance_m` away from the last one kept.
///
/// Which of the two conditions the native resampler joins with could not be
/// read off the Java source, but "both" is the behaviour that matches the
/// gauge in practice — and it is what matters here, because a dashcam spends a
/// good part of its life at a red light, where a 1 Hz log piles hundreds of
/// jittering samples onto one pixel.
#[derive(Debug, Clone, Copy)]
pub struct Decimation {
    pub min_time_s: f64,
    pub min_distance_m: f64,
    /// Hard cap on kept samples (`lx2.m5737k(..., 72000, 1)`).
    pub max_samples: usize,
}

impl Default for Decimation {
    fn default() -> Self {
        Decimation {
            min_time_s: 0.2,
            min_distance_m: 3.0,
            max_samples: 72_000,
        }
    }
}

impl Decimation {
    pub fn apply<'a>(&self, fixes: &'a [Fix]) -> Vec<&'a Fix> {
        let mut out: Vec<&Fix> = Vec::new();
        for f in fixes {
            let keep = match out.last() {
                None => true,
                Some(prev) => {
                    f.t - prev.t >= self.min_time_s && f.dist - prev.dist >= self.min_distance_m
                }
            };
            if keep {
                out.push(f);
                if out.len() >= self.max_samples {
                    return out;
                }
            }
        }
        // The last fix is always drawn, so the trace reaches the end of the drive.
        if let (Some(last), Some(tail)) = (fixes.last(), out.last())
            && !std::ptr::eq(*tail, last)
        {
            out.push(last);
        }
        out
    }
}

/// One drawn piece of the trace: a projected segment and the time span it
/// covers, which is what tells a frame whether it has been driven yet.
#[derive(Debug, Clone, Copy)]
pub struct Segment {
    pub x0: f32,
    pub y0: f32,
    pub x1: f32,
    pub y1: f32,
    /// Session seconds at each end, `t0 <= t1`.
    pub t0: f32,
    pub t1: f32,
}

/// The position marker for one video frame.
#[derive(Debug, Clone, Copy)]
pub struct MarkerFrame {
    /// Pixel position; `NaN` when there is nothing to draw.
    pub x: f32,
    pub y: f32,
    /// Degrees clockwise from north; `NaN` while the heading is unknown.
    pub heading: f32,
    pub source: PositionSource,
}

impl MarkerFrame {
    const BLANK: MarkerFrame = MarkerFrame {
        x: f32::NAN,
        y: f32::NAN,
        heading: f32::NAN,
        source: PositionSource::NoPosition,
    };
}

/// Everything the renderer needs, projected into a `size` × `size` frame.
pub struct TrackLayout {
    pub size: u32,
    /// The trace, in drawing order; a dropout simply leaves no segment.
    pub segments: Vec<Segment>,
    /// One entry per video frame.
    pub frames: Vec<MarkerFrame>,
}

/// Project a session into a square frame: the trace plus `duration × fps`
/// markers.
///
/// `None` when the drive holds too few positions to fit a viewport around.
pub fn build(
    session: &Session,
    motion: &MotionSeries,
    size: u32,
    fps: f64,
    duration: f64,
) -> Option<TrackLayout> {
    let kept = Decimation::default().apply(&session.fixes);
    if kept.len() < 2 {
        return None;
    }

    let bounds = GeoBounds::from_points(kept.iter().map(|f| (f.lat, f.lon)))?;
    if !bounds.is_valid() {
        return None;
    }
    let side = size as f32;
    let mut view = MercatorView::new(size);
    view.fit(&bounds, side * PADDING_RATIO, side, side);

    Some(TrackLayout {
        size,
        segments: trace(&kept, &view, fps),
        frames: markers(session, motion, &view, fps, duration),
    })
}

/// Project the kept samples into segments, splitting anything longer than one
/// frame so the past/future split can land between two records.
///
/// Subdivision happens in pixel space, after the projection, so it adds
/// vertices without moving the line: the drawn shape is the same polyline
/// either way.
fn trace(kept: &[&Fix], view: &MercatorView, fps: f64) -> Vec<Segment> {
    let mut segments = Vec::with_capacity(kept.len());

    for pair in kept.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        let span = b.t - a.t;
        if span > MAX_GAP_S {
            continue;
        }
        let (ax, ay) = (view.project_x(a.lon), view.project_y(a.lat));
        let (bx, by) = (view.project_x(b.lon), view.project_y(b.lat));

        // One piece per frame the segment spans. A non-positive frame rate
        // falls out as a single piece; [`crate::overlay`] is what reports it.
        let pieces = ((span * fps).ceil() as usize).clamp(1, 1 << 12);
        for k in 0..pieces {
            let (u0, u1) = (k as f32 / pieces as f32, (k + 1) as f32 / pieces as f32);
            segments.push(Segment {
                x0: lerp(ax, bx, u0),
                y0: lerp(ay, by, u0),
                x1: lerp(ax, bx, u1),
                y1: lerp(ay, by, u1),
                t0: lerp(a.t as f32, b.t as f32, u0),
                t1: lerp(a.t as f32, b.t as f32, u1),
            });
        }
    }
    segments
}

/// Sample the motion series once per video frame.
fn markers(
    session: &Session,
    motion: &MotionSeries,
    view: &MercatorView,
    fps: f64,
    duration: f64,
) -> Vec<MarkerFrame> {
    let total = (duration * fps).ceil().max(0.0) as usize;
    let mut player = motion.player(MAX_GAP_S);

    (0..total)
        .map(|frame| {
            let t = frame as f64 / fps;
            let state = player.state_at(t);
            if !state.is_valid() {
                return MarkerFrame::BLANK;
            }
            MarkerFrame {
                x: view.project_x(state.lon),
                y: view.project_y(state.lat),
                // The series keeps the heading unwrapped; the arrow wants it
                // back in [0, 360).
                heading: state.bearing.rem_euclid(360.0) as f32,
                source: if session.has_fix_at(t) {
                    PositionSource::HardwareFix
                } else {
                    PositionSource::Interpolated
                },
            }
        })
        .collect()
}

#[inline]
fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::FixedOffset;

    /// A straight drive north at 20 m/s, one fix a second.
    fn session(n: usize) -> Session {
        let fixes = (0..n)
            .map(|i| Fix {
                t: i as f64,
                lat: 25.0 + i as f64 * 0.00018,
                lon: 121.0,
                speed: 20.0,
                bearing: 0.0,
                dist: i as f64 * 20.0,
            })
            .collect();
        Session {
            fixes,
            record_count: n,
            tz: FixedOffset::east_opt(8 * 3600).unwrap(),
        }
    }

    #[test]
    fn decimation_drops_a_stationary_car_but_keeps_the_ends() {
        let mut fixes: Vec<Fix> = session(5).fixes;
        // Ten seconds at a red light: the position jitters by centimetres.
        let last = *fixes.last().unwrap();
        fixes.extend((1..=10).map(|i| Fix {
            t: last.t + i as f64,
            dist: last.dist + 0.01 * i as f64,
            ..last
        }));

        let kept = Decimation::default().apply(&fixes);
        assert_eq!(kept.len(), 6, "the wait should collapse to its two ends");
        assert_eq!(kept[0].t, 0.0);
        assert_eq!(kept.last().unwrap().t, 14.0);
    }

    #[test]
    fn a_session_too_short_to_fit_has_no_layout() {
        let s = session(1);
        let m = MotionSeries::from_fixes(&s.fixes);
        assert!(build(&s, &m, 480, 30.0, 1.0).is_none());
    }

    #[test]
    fn the_trace_stays_inside_the_padded_frame() {
        let s = session(30);
        let m = MotionSeries::from_fixes(&s.fixes);
        let size = 480;
        let layout = build(&s, &m, size, 30.0, 30.0).unwrap();

        let pad = size as f32 * PADDING_RATIO;
        for seg in &layout.segments {
            for v in [seg.x0, seg.x1, seg.y0, seg.y1] {
                assert!(
                    (pad - 0.01..=size as f32 - pad + 0.01).contains(&v),
                    "{v} escaped the padded frame"
                );
            }
        }
        // Times run forward and tile the drive without holes.
        for pair in layout.segments.windows(2) {
            assert!(pair[0].t1 <= pair[1].t0 + 1e-4);
            assert!(pair[0].t0 <= pair[0].t1);
        }
    }

    #[test]
    fn segments_are_subdivided_to_the_frame_rate() {
        let s = session(3);
        let m = MotionSeries::from_fixes(&s.fixes);
        let layout = build(&s, &m, 480, 10.0, 3.0).unwrap();
        // Two one-second gaps at 10 fps.
        assert_eq!(layout.segments.len(), 20);
        assert!((layout.segments[0].t1 - 0.1).abs() < 1e-4);
    }

    #[test]
    fn a_dropout_breaks_the_trace_and_blanks_the_marker() {
        // Three seconds of driving, thirty in a tunnel, then two more seconds.
        let mut s = session(3);
        let last = *s.fixes.last().unwrap();
        s.fixes.extend((0..2).map(|i| Fix {
            t: last.t + 30.0 + i as f64,
            lat: last.lat + 0.01 + i as f64 * 0.00018,
            dist: last.dist + 1000.0 + i as f64 * 20.0,
            ..last
        }));
        let m = MotionSeries::from_fixes(&s.fixes);
        let layout = build(&s, &m, 480, 1.0, 34.0).unwrap();

        // Nothing is drawn across the hole; both sides of it are.
        assert!(
            layout
                .segments
                .iter()
                .all(|s| s.t1 <= 2.0 + 1e-4 || s.t0 >= 32.0 - 1e-4),
            "a segment crossed the dropout"
        );
        assert!(layout.segments.iter().any(|s| s.t0 >= 32.0 - 1e-4));

        assert_eq!(layout.frames[1].source, PositionSource::HardwareFix);
        assert_eq!(layout.frames[15].source, PositionSource::NoPosition);
        assert!(layout.frames[15].x.is_nan());
        assert_eq!(layout.frames[33].source, PositionSource::HardwareFix);
    }

    #[test]
    fn the_marker_glides_between_two_records() {
        let s = session(5);
        let m = MotionSeries::from_fixes(&s.fixes);
        let layout = build(&s, &m, 480, 10.0, 5.0).unwrap();

        // Between two fixes the marker is a real fix carried forward, and it
        // has moved a fraction of the way.
        assert_eq!(layout.frames[5].source, PositionSource::HardwareFix);
        let (a, b, c) = (layout.frames[0].y, layout.frames[5].y, layout.frames[10].y);
        assert!(b < a && c < b, "north is up: {a} {b} {c}");
        assert!(((a + c) / 2.0 - b).abs() < 0.01, "half a second in");
        // Heading came back wrapped into range.
        assert!((layout.frames[5].heading - 0.0).abs() < 1e-4);
    }
}
