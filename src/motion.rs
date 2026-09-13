//! Derived motion channels, and the playback cursor that moves the marker.
//!
//! Ported from `rc_map_overlay/src/motion.rs`.  For a GPS device with no IMU —
//! which is exactly what a dashcam's receiver is — RaceChrono derives its
//! motion channels from position, speed and bearing alone (`p000/cs1.java`):
//!
//! ```text
//! ch 7  Lateral acceleration     avg(interpolate(gpsLatAcc(bearing, speed)), 1.200)
//! ch 8  Longitudinal accel.      avg(interpolate(gpsLongAcc(speed)), 1.200)
//! ch 37 Lean angle               avg(interpolate(gpsLeanAngle(latAcc)), 1.200)
//! ch 43 Combined acceleration    sqrt(pow2(latAcc_avg) + pow2(longAcc_avg))
//! ```
//!
//! where the bearing feeding `gpsLatAcc` is `interpolate(unwrap(bearing))` —
//! unwrapped first, or every crossing of north would read as a 360 °/s yaw
//! rate.  The two acceleration channels are kept below; lean angle and combined
//! acceleration are motorcycle gauges and are left out.
//!
//! The playback half is what replaced this crate's cubic splines.  RaceChrono
//! plays a session back with `interpolate()`, a plain linear interpolation with
//! a maximum bridgeable gap: between two records the marker moves at a constant
//! rate, and across a dropout longer than the gap it disappears instead of
//! following a curve nobody drove.

use crate::filters::{
    self, QuickInterpolator, centred_average, gps_lateral_acceleration,
    gps_longitudinal_acceleration,
};
use crate::gps::Fix;

/// Smoothing window `cs1` applies to the GPS-derived motion channels.
const AVG_WINDOW_S: f64 = 1.2;

/// The motion channels of one session: the raw inputs, resampled onto parallel
/// arrays, plus what is derived from them.
#[derive(Debug, Clone, Default)]
pub struct MotionSeries {
    /// Seconds from the start of the concatenated session.
    pub t: Vec<f64>,
    /// Degrees, WGS-84.
    pub lat: Vec<f64>,
    pub lon: Vec<f64>,
    /// Ground speed, m/s.
    pub speed: Vec<f64>,
    /// Bearing in degrees, unwrapped into a continuous series — it may run past
    /// 360 or below 0, and is wrapped back only where it is displayed.
    pub bearing: Vec<f64>,
    /// Lateral acceleration, m/s²; negative in a left-hand corner.
    pub lateral_acc: Vec<f64>,
    /// Longitudinal acceleration, m/s²; negative under braking.
    pub longitudinal_acc: Vec<f64>,
}

impl MotionSeries {
    /// Build the series from a session's fixes, which must be sorted by time.
    pub fn from_fixes(fixes: &[Fix]) -> MotionSeries {
        let n = fixes.len();
        let mut m = MotionSeries {
            t: Vec::with_capacity(n),
            lat: Vec::with_capacity(n),
            lon: Vec::with_capacity(n),
            speed: Vec::with_capacity(n),
            ..Default::default()
        };

        let mut raw_bearing = Vec::with_capacity(n);
        for f in fixes {
            m.t.push(f.t);
            m.lat.push(f.lat);
            m.lon.push(f.lon);
            m.speed.push(f.speed);
            raw_bearing.push(f.bearing);
        }
        m.bearing = filters::unwrap_deg(&raw_bearing);

        // gpsLatAcc / gpsLongAcc are two-sample filters: index i uses i-1 and i.
        let mut lat_acc = vec![f64::NAN; n];
        let mut long_acc = vec![f64::NAN; n];
        for i in 1..n {
            lat_acc[i] = gps_lateral_acceleration(
                m.bearing[i - 1],
                m.bearing[i],
                m.speed[i],
                m.t[i - 1],
                m.t[i],
            );
            long_acc[i] =
                gps_longitudinal_acceleration(m.speed[i - 1], m.speed[i], m.t[i - 1], m.t[i]);
        }

        // avg(..., 1.200)
        m.lateral_acc = centred_average(&m.t, &lat_acc, AVG_WINDOW_S);
        m.longitudinal_acc = centred_average(&m.t, &long_acc, AVG_WINDOW_S);
        m
    }

    /// A playback cursor over this series — what the marker needs per frame.
    ///
    /// `max_gap_s` is the longest hole in the log the marker is still carried
    /// across; beyond it [`MotionState::is_valid`] turns false.
    pub fn player(&self, max_gap_s: f64) -> MotionPlayer<'_> {
        MotionPlayer {
            lat: QuickInterpolator::new(&self.t, &self.lat).with_max_gap(max_gap_s),
            lon: QuickInterpolator::new(&self.t, &self.lon).with_max_gap(max_gap_s),
            bearing: QuickInterpolator::new(&self.t, &self.bearing).with_max_gap(max_gap_s),
        }
    }
}

/// Smallest and largest value of a channel, ignoring its NaN holes; `None` when
/// the channel holds nothing else.
pub fn extremes(values: &[f64]) -> Option<(f64, f64)> {
    values
        .iter()
        .copied()
        .filter(|v| !v.is_nan())
        .fold(None, |acc, v| {
            Some(match acc {
                None => (v, v),
                Some((lo, hi)) => (lo.min(v), hi.max(v)),
            })
        })
}

/// Position and heading at an arbitrary time, via `interpolate()` — this is
/// what makes the marker glide between two 1 Hz records.
pub struct MotionPlayer<'a> {
    lat: QuickInterpolator<'a>,
    lon: QuickInterpolator<'a>,
    bearing: QuickInterpolator<'a>,
}

/// One interpolated playback sample.
#[derive(Debug, Clone, Copy)]
pub struct MotionState {
    pub lat: f64,
    pub lon: f64,
    /// Degrees, still unwrapped; NaN while the course is unusable.
    pub bearing: f64,
}

impl MotionState {
    /// Whether there is a position to draw at this instant.
    pub fn is_valid(&self) -> bool {
        self.lat.is_finite() && self.lon.is_finite()
    }
}

impl MotionPlayer<'_> {
    /// Sampling in increasing `t` keeps every lookup O(1).
    pub fn state_at(&mut self, t: f64) -> MotionState {
        MotionState {
            lat: self.lat.value_at(t),
            lon: self.lon.value_at(t),
            bearing: self.bearing.value_at(t),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fix(t: f64, lat: f64, lon: f64, speed: f64, bearing: f64) -> Fix {
        Fix {
            t,
            lat,
            lon,
            speed,
            bearing,
            dist: 0.0,
        }
    }

    #[test]
    fn a_constant_turn_gives_a_constant_lateral_acceleration() {
        // 1 Hz, 20 m/s, 18 deg/s yaw -> 20 * 0.3142 = 6.28 m/s^2, negated.
        let fixes: Vec<Fix> = (0..40)
            .map(|i| fix(i as f64, 25.0, 121.0, 20.0, (i as f64 * 18.0) % 360.0))
            .collect();
        let m = MotionSeries::from_fixes(&fixes);
        let mid = m.lateral_acc[20];
        assert!(
            (mid - -(20.0 * 18f64.to_radians())).abs() < 0.05,
            "mid = {mid}"
        );
        // Steady speed means no longitudinal acceleration.
        assert!(m.longitudinal_acc[20].abs() < 1e-9);
    }

    #[test]
    fn braking_shows_up_as_negative_longitudinal_acceleration() {
        let fixes: Vec<Fix> = (0..10)
            .map(|i| fix(i as f64, 25.0, 121.0, 20.0 - i as f64 * 2.0, 90.0))
            .collect();
        let m = MotionSeries::from_fixes(&fixes);
        assert!((m.longitudinal_acc[5] - -2.0).abs() < 1e-9);
        let (lo, hi) = extremes(&m.longitudinal_acc).unwrap();
        assert!((lo - -2.0).abs() < 1e-9 && (hi - -2.0).abs() < 1e-9);
        assert!(extremes(&[f64::NAN]).is_none());
    }

    #[test]
    fn playback_interpolates_between_records_and_stops_at_a_hole() {
        let fixes = vec![
            fix(0.0, 25.0, 121.0, 10.0, 90.0),
            fix(1.0, 25.001, 121.0, 10.0, 90.0),
            // A 30 s dropout: nothing may be drawn across it.
            fix(31.0, 25.5, 121.5, 10.0, 90.0),
            fix(32.0, 25.501, 121.5, 10.0, 90.0),
        ];
        let m = MotionSeries::from_fixes(&fixes);
        let mut p = m.player(10.0);

        let s = p.state_at(0.5);
        assert!(s.is_valid());
        assert!((s.lat - 25.0005).abs() < 1e-9, "lat = {}", s.lat);
        assert!(!p.state_at(15.0).is_valid(), "the hole stays empty");
        // The gap is judged from the bracketing pair, so the marker reappears
        // exactly on the first record out of the tunnel, not before it.
        assert!(!p.state_at(30.99).is_valid());
        assert!(p.state_at(31.0).is_valid());
        assert!(p.state_at(31.5).is_valid());
        assert!(
            !p.state_at(40.0).is_valid(),
            "no extrapolation past the end"
        );
    }

    #[test]
    fn the_heading_is_continuous_across_north() {
        let fixes = vec![
            fix(0.0, 25.0, 121.0, 10.0, 350.0),
            fix(1.0, 25.0, 121.0, 10.0, 10.0),
        ];
        let m = MotionSeries::from_fixes(&fixes);
        // 350 -> 10 is +20 degrees, not -340.
        assert!((m.bearing[1] - 370.0).abs() < 1e-9, "{:?}", m.bearing);
        let mut p = m.player(10.0);
        assert!((p.state_at(0.5).bearing.rem_euclid(360.0) - 0.0).abs() < 1e-9);
    }
}
