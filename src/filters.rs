//! Ports of the RaceChrono channel filters the overlay needs.
//!
//! Lifted from `rc_map_overlay/src/filters.rs`, a Rust port of the operators
//! inside `libRaceChronoEngine.so` (RaceChrono 10.2.4).  The APK's equation
//! strings only ever *name* these — `interpolate()`, `unwrap()`, `avg()`,
//! `gpsLatAcc()`, `gpsLongAcc()` — and the implementations were read off the
//! stripped native binary.  Each doc comment keeps the original's pseudo-code,
//! so the behaviour stays checkable against that disassembly.
//!
//! Two deliberate deviations from the port: a series is a pair of parallel
//! `x` / `value` slices instead of one interleaved sample array, which is the
//! shape a [`MotionSeries`](crate::motion::MotionSeries) already has; and the
//! motorcycle-only `gpsLeanAngle()` / `magnitude()` filters are left out.

/// Standard gravity — the constant the two GPS acceleration filters clip
/// against, and the unit accelerations are reported in.
pub const G: f64 = 9.80665;

/// Both GPS acceleration filters reject anything beyond 10 g: at 1 Hz a single
/// position glitch would otherwise show up as a wall of acceleration.
const ACC_LIMIT: f64 = G * 10.0;

/// Port of the angle normaliser at `0xd8590`: wrap radians into `(-PI, PI]`.
///
/// ```text
/// m = fmod(|v|, 2PI)
/// if v < 0 { m = 2PI - m }
/// if m > PI { m -= 2PI }
/// ```
pub fn wrap_pi(v: f64) -> f64 {
    use std::f64::consts::PI;
    let tau = 2.0 * PI;
    let mut m = v.abs() % tau;
    if v < 0.0 {
        m = tau - m;
    }
    if m > PI {
        m -= tau;
    }
    m
}

/// Port of `CoreUnwrapFilter` (`0x10e8d0`), which makes a wrapping angle series
/// continuous *in degrees* before it is interpolated or differentiated:
///
/// ```text
/// delta = v - previous_output
/// if index >= 1 && |delta| >= 180 { delta = remainder(delta, 360) }   // IEEE remainder
/// output = previous_output + delta
/// ```
///
/// A NaN input holds the accumulator, which is what a record that carried no
/// course needs: the last real heading is carried across it rather than the
/// series collapsing to north.
pub fn unwrap_deg(values: &[f64]) -> Vec<f64> {
    let mut acc = 0.0f64;
    let mut out = Vec::with_capacity(values.len());
    for (i, &v) in values.iter().enumerate() {
        if v.is_nan() {
            out.push(acc);
            continue;
        }
        let mut delta = v - acc;
        if i >= 1 && delta.abs() >= 180.0 {
            delta = ieee_remainder(delta, 360.0);
        }
        acc += delta;
        out.push(acc);
    }
    out
}

/// `remainder(x, y)` as used by the unwrap filter: `x - y * round_half_even(x/y)`,
/// result in `[-y/2, y/2]`.
fn ieee_remainder(x: f64, y: f64) -> f64 {
    x - y * (x / y).round_ties_even()
}

/// Port of `CoreGpsLateralAccelerationFilter::value(i)` (`0x100aa0`) —
/// `gpsLatAcc(bearing, speed)`.
///
/// ```text
/// d_bearing = wrap_pi(deg_to_rad(bearing[i] - bearing[i-1]))
/// dt        = x[i] - x[i-1]
/// a_lat     = -(speed[i] * d_bearing / dt)          // m/s^2
/// if |a_lat| > 10 g  -> NaN
/// ```
///
/// Note the negation: a clockwise (increasing) bearing yields a negative value.
/// The result is in m/s², not g — the display conversion happens later.
pub fn gps_lateral_acceleration(
    bearing_prev_deg: f64,
    bearing_deg: f64,
    speed: f64,
    x_prev: f64,
    x: f64,
) -> f64 {
    let d_bearing = wrap_pi((bearing_deg - bearing_prev_deg).to_radians());
    let a = -(speed * (d_bearing / (x - x_prev)));
    if a.is_nan() || a.abs() > ACC_LIMIT {
        f64::NAN
    } else {
        a
    }
}

/// Port of `CoreGpsLongitudinalAccelerationFilter::value(i)` (`0x10113c`) —
/// `gpsLongAcc(speed)`.
///
/// ```text
/// if x[i-1] >= x[i] -> NaN            // x must be strictly increasing
/// a_long = (v[i] - v[i-1]) / (x[i] - x[i-1])      // m/s^2
/// if |a_long| > 10 g -> NaN
/// ```
pub fn gps_longitudinal_acceleration(speed_prev: f64, speed: f64, x_prev: f64, x: f64) -> f64 {
    if x_prev.is_nan() || x.is_nan() || x_prev >= x {
        return f64::NAN;
    }
    let a = (speed - speed_prev) / (x - x_prev);
    if a.is_nan() || a.abs() > ACC_LIMIT {
        f64::NAN
    } else {
        a
    }
}

/// `avg(channel, window)` — approximation of `CoreWeighedAverageFilter`
/// (`0xf91e4`).
///
/// The native filter walks backwards and forwards from sample `i` while
/// `|x[i] - x[j]|` stays inside the window, so it is a **centred** window over
/// the x-axis; the exact weighting and edge handling were not recovered from
/// the disassembly.  This takes the unweighted mean of every sample within
/// `window` of the centre — close enough for smoothing, not bit-exact.
pub fn centred_average(x: &[f64], values: &[f64], window: f64) -> Vec<f64> {
    debug_assert_eq!(x.len(), values.len());
    let mut out = Vec::with_capacity(values.len());
    let (mut lo, mut hi) = (0usize, 0usize);
    for i in 0..values.len() {
        while lo < i && x[i] - x[lo] > window {
            lo += 1;
        }
        hi = hi.max(i);
        while hi + 1 < values.len() && x[hi + 1] - x[i] <= window {
            hi += 1;
        }
        let mut sum = 0.0;
        let mut n = 0usize;
        for &v in &values[lo..=hi] {
            if !v.is_nan() {
                sum += v;
                n += 1;
            }
        }
        out.push(if n == 0 { f64::NAN } else { sum / n as f64 });
    }
    out
}

/// Port of `CoreQuickInterpolateFilter` (`0x103990`) — the `interpolate()`
/// operator, and the thing that moves the marker between two GPS records.
///
/// For a wanted `x` it finds the bracketing pair `lo`/`lo+1` in the source
/// series (the original caches the last index, hence "quick"), then:
///
/// ```text
/// t = (x - x_lo) / (x_hi - x_lo)     // 0 when x_hi == x_lo
/// t = clamp(t, 0, 1)
/// value = v_lo + (v_hi - v_lo) * t   // fmadd
/// ```
///
/// Outside the series, or across a hole wider than `max_gap`, the answer is
/// NaN rather than an extrapolation — a dropout leaves a gap instead of an
/// invented curve through it.  Sampling `x` in increasing order keeps the
/// cursor warm and every lookup O(1).
#[derive(Debug, Clone)]
pub struct QuickInterpolator<'a> {
    x: &'a [f64],
    values: &'a [f64],
    cursor: usize,
    max_gap: f64,
    /// `false` is the filter's sample-and-hold mode (field `0x68`): the value
    /// of the preceding sample rather than a blend of the bracketing pair.
    interpolate: bool,
}

impl<'a> QuickInterpolator<'a> {
    pub fn new(x: &'a [f64], values: &'a [f64]) -> Self {
        debug_assert_eq!(x.len(), values.len());
        QuickInterpolator {
            x,
            values,
            cursor: 0,
            max_gap: f64::INFINITY,
            interpolate: true,
        }
    }

    pub fn with_max_gap(mut self, max_gap: f64) -> Self {
        self.max_gap = max_gap;
        self
    }

    /// Sample and hold instead of blending — what a value that is not a
    /// quantity needs, a flag being the obvious case.
    pub fn hold(mut self) -> Self {
        self.interpolate = false;
        self
    }

    /// Value at `x`; NaN outside the series or across a gap wider than `max_gap`.
    pub fn value_at(&mut self, x: f64) -> f64 {
        let n = self.x.len();
        if n < 2 || x < self.x[0] || x > self.x[n - 1] {
            return f64::NAN;
        }

        // Cached forward/backward scan, like the filter's index cache.
        let mut lo = self.cursor.min(n - 2);
        while lo + 1 < n - 1 && self.x[lo + 1] <= x {
            lo += 1;
        }
        while lo > 0 && self.x[lo] > x {
            lo -= 1;
        }
        self.cursor = lo;

        let (x0, x1) = (self.x[lo], self.x[lo + 1]);
        if x1 - x0 > self.max_gap {
            return f64::NAN;
        }
        let (v0, v1) = (self.values[lo], self.values[lo + 1]);
        if !self.interpolate {
            return v0;
        }
        let t = if x1 == x0 {
            0.0
        } else {
            ((x - x0) / (x1 - x0)).clamp(0.0, 1.0)
        };
        v0 + (v1 - v0) * t
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_pi_matches_native_range() {
        use std::f64::consts::PI;
        assert!((wrap_pi(0.0) - 0.0).abs() < 1e-12);
        assert!((wrap_pi(PI) - PI).abs() < 1e-12);
        assert!((wrap_pi(PI + 0.1) - (-PI + 0.1)).abs() < 1e-12);
        assert!((wrap_pi(-PI - 0.1) - (PI - 0.1)).abs() < 1e-12);
        assert!((wrap_pi(3.0 * PI) - PI).abs() < 1e-12);
    }

    #[test]
    fn unwrap_removes_360_jumps_and_holds_across_nan() {
        let u = unwrap_deg(&[350.0, 355.0, 5.0, 10.0, 350.0]);
        assert!((u[2] - 365.0).abs() < 1e-9, "{u:?}");
        assert!((u[3] - 370.0).abs() < 1e-9, "{u:?}");
        assert!((u[4] - 350.0).abs() < 1e-9, "{u:?}");

        // A stopped vehicle reports no usable course; the heading must not swing.
        let held = unwrap_deg(&[90.0, f64::NAN, f64::NAN, 92.0]);
        assert_eq!(held[1], 90.0);
        assert!((held[3] - 92.0).abs() < 1e-9);
    }

    #[test]
    fn lateral_acceleration_is_speed_times_yaw_rate() {
        // 90 deg/s at 10 m/s -> 10 * pi/2 = 15.7 m/s^2, negated.
        let a = gps_lateral_acceleration(0.0, 9.0, 10.0, 0.0, 0.1);
        assert!((a - -(10.0 * 90f64.to_radians())).abs() < 1e-9, "a = {a}");
        // Beyond 10 g -> NaN.
        assert!(gps_lateral_acceleration(0.0, 90.0, 100.0, 0.0, 0.1).is_nan());
        // Wrapping across north must not explode.
        let a = gps_lateral_acceleration(359.0, 1.0, 10.0, 0.0, 0.2);
        assert!(
            (a - -(10.0 * 2f64.to_radians() / 0.2)).abs() < 1e-9,
            "a = {a}"
        );
        assert!(gps_lateral_acceleration(f64::NAN, 1.0, 10.0, 0.0, 0.2).is_nan());
    }

    #[test]
    fn longitudinal_acceleration_needs_increasing_time() {
        assert!((gps_longitudinal_acceleration(10.0, 12.0, 0.0, 0.5) - 4.0).abs() < 1e-9);
        assert!(gps_longitudinal_acceleration(10.0, 12.0, 1.0, 1.0).is_nan());
        assert!(gps_longitudinal_acceleration(10.0, f64::NAN, 0.0, 1.0).is_nan());
    }

    #[test]
    fn centred_average_smooths_over_the_window() {
        let x: Vec<f64> = (0..5).map(|i| i as f64).collect();
        let v = vec![0.0, 10.0, 0.0, 10.0, 0.0];
        let avg = centred_average(&x, &v, 1.0);
        // Sample 2 averages samples 1..=3.
        assert!((avg[2] - 20.0 / 3.0).abs() < 1e-9, "{avg:?}");
        // An all-NaN window stays NaN rather than counting as zero.
        assert!(centred_average(&x, &[f64::NAN; 5], 1.0)[0].is_nan());
    }

    #[test]
    fn quick_interpolate_is_linear_and_refuses_to_bridge_holes() {
        let x = [0.0, 1.0, 3.0];
        let v = [0.0, 10.0, 30.0];
        let mut q = QuickInterpolator::new(&x, &v);
        assert!((q.value_at(0.5) - 5.0).abs() < 1e-9);
        assert!((q.value_at(2.0) - 20.0).abs() < 1e-9);
        assert!((q.value_at(3.0) - 30.0).abs() < 1e-9);
        assert!(q.value_at(-0.1).is_nan());
        assert!(q.value_at(3.1).is_nan());

        let mut gapped = QuickInterpolator::new(&x, &v).with_max_gap(1.5);
        assert!((gapped.value_at(0.5) - 5.0).abs() < 1e-9);
        assert!(gapped.value_at(2.0).is_nan());
    }

    #[test]
    fn holding_returns_the_preceding_sample() {
        let x = [0.0, 1.0, 3.0];
        let v = [0.0, f64::NAN, 30.0];
        let mut h = QuickInterpolator::new(&x, &v).hold();
        assert_eq!(h.value_at(0.9), 0.0);
        // A hole starts at its own sample and runs up to the next one.
        assert!(h.value_at(1.0).is_nan());
        assert!(h.value_at(2.9).is_nan());
        // The very last sample still holds its predecessor: there is no pair
        // beyond it to step into. The original behaves the same way.
        assert!(h.value_at(3.0).is_nan());
    }
}
