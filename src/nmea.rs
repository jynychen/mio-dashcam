//! NMEA 0183 sidecar-log parsing.
//!
//! Mio writes one `.NMEA` text file per segment, at 1 Hz, alongside the front
//! clip.  A second looks like this:
//!
//! ```text
//! $GNRMC,134607.655,A,2507.7744,N,12124.2935,E,0.71,330.64,200826,,,A,V*08
//! $GNGGA,134607.655,2507.7744,N,12124.2935,E,1,18,0.67,105.4,M,14.8,M,,*78
//! $GNGSA,…  $GPGSV,…  $GLGSV,…            satellite bookkeeping, unused
//! $GSENSORD,0.065,-0.002,0.062,[-0.712,0.050,0.648],0.090, 9433
//! ```
//!
//! Only RMC is needed for the track overlay: it carries position, ground speed,
//! course and — importantly — the **date**, which makes each record an absolute
//! UTC instant and lets a recording be stitched across midnight.
//!
//! Note the clock mismatch: RMC timestamps are UTC while the file *name* is the
//! camera's local time.  [`crate::gps`] reconciles the two.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use anyhow::{Context, Result};
use chrono::{NaiveDate, NaiveDateTime};

/// One 1 Hz RMC record.
#[derive(Debug, Clone)]
pub struct Rmc {
    /// Absolute UTC instant (date + time fields combined).
    pub utc: NaiveDateTime,
    /// `true` when the receiver reported a valid fix (status `A`).
    pub fix: bool,
    /// Decimal degrees, WGS-84. `NaN` without a fix.
    pub lat: f64,
    pub lon: f64,
    /// Ground speed in km/h. `NaN` without a fix.
    pub speed: f64,
    /// Course over ground in degrees true. `NaN` without a fix.
    pub track: f64,
}

/// Parse every RMC sentence in `path`, in file order.
///
/// Malformed or checksum-failing lines are skipped: a dashcam log is written
/// live to an SD card and the tail of a file is routinely truncated mid-line
/// when power is cut.
pub fn parse_file(path: &Path) -> Result<Vec<Rmc>> {
    let file = File::open(path).with_context(|| format!("cannot open {}", path.display()))?;
    let mut out = Vec::new();

    for line in BufReader::new(file).lines() {
        // Logs are ASCII; a decode error means a corrupt block, not a wider
        // problem, so stop reading this file and keep what came before.
        let Ok(line) = line else { break };
        if let Some(rmc) = parse_rmc(&line) {
            out.push(rmc);
        }
    }

    Ok(out)
}

/// Parse a single line if it is a valid RMC sentence.
///
/// Any talker ID is accepted (`$GNRMC` from the multi-constellation receiver,
/// `$GPRMC` from GPS-only firmware).
fn parse_rmc(line: &str) -> Option<Rmc> {
    let body = verified_body(line.trim())?;
    let f: Vec<&str> = body.split(',').collect();

    // f[0] = talker + sentence id, then: time, status, lat, N/S, lon, E/W,
    // speed(kn), course, date.
    if f.len() < 10 || f[0].len() != 5 || !f[0].ends_with("RMC") {
        return None;
    }

    let utc = decode_datetime(f[9], f[1])?;
    let fix = f[2].eq_ignore_ascii_case("A");

    if !fix {
        return Some(Rmc {
            utc,
            fix,
            lat: f64::NAN,
            lon: f64::NAN,
            speed: f64::NAN,
            track: f64::NAN,
        });
    }

    Some(Rmc {
        utc,
        fix,
        lat: decode_coord(f[3], f[4])?,
        lon: decode_coord(f[5], f[6])?,
        // A fixed receiver leaves course empty — that is not a broken sentence,
        // so fall back to NaN and let the heading spline ignore the record.
        speed: f[7].parse::<f64>().ok().map_or(f64::NAN, |kn| kn * 1.852),
        track: f[8].parse::<f64>().unwrap_or(f64::NAN),
    })
}

/// Strip the `$` prefix and `*XX` suffix, verifying the XOR checksum.
///
/// A sentence without a checksum is accepted as-is; one with a checksum that
/// does not match is rejected.
fn verified_body(line: &str) -> Option<&str> {
    let body = line.strip_prefix('$')?;
    let Some((payload, checksum)) = body.rsplit_once('*') else {
        return Some(body);
    };
    let expected = u8::from_str_radix(checksum.get(..2)?, 16).ok()?;
    let actual = payload.bytes().fold(0u8, |acc, b| acc ^ b);
    (actual == expected).then_some(payload)
}

/// Combine the `ddmmyy` date field and the `hhmmss.sss` time field.
///
/// The sub-second part is dropped: the receiver emits one record per second and
/// the fractional offset (`.849`, `.000`, …) drifts between clips, so keeping it
/// would add jitter to the interpolation without adding information.
fn decode_datetime(date: &str, time: &str) -> Option<NaiveDateTime> {
    if date.len() < 6 || time.len() < 6 {
        return None;
    }
    let day = date[0..2].parse().ok()?;
    let month = date[2..4].parse().ok()?;
    let year = 2000 + date[4..6].parse::<i32>().ok()?;
    NaiveDate::from_ymd_opt(year, month, day)?.and_hms_opt(
        time[0..2].parse().ok()?,
        time[2..4].parse().ok()?,
        time[4..6].parse().ok()?,
    )
}

/// Convert an NMEA `ddmm.mmmm` / `dddmm.mmmm` coordinate to decimal degrees.
fn decode_coord(value: &str, hemisphere: &str) -> Option<f64> {
    let raw: f64 = value.parse().ok()?;
    let degrees = (raw / 100.0).trunc();
    let dd = degrees + (raw - degrees * 100.0) / 60.0;
    match hemisphere {
        "S" | "W" => Some(-dd),
        "N" | "E" => Some(dd),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_a_fixed_record() {
        let r =
            parse_rmc("$GNRMC,134607.655,A,2507.7744,N,12124.2935,E,0.71,330.64,200826,,,A,V*08")
                .unwrap();
        assert!(r.fix);
        assert_eq!(r.utc.to_string(), "2026-08-20 13:46:07");
        assert!((r.lat - 25.129573).abs() < 1e-5, "lat was {}", r.lat);
        assert!((r.lon - 121.404892).abs() < 1e-5, "lon was {}", r.lon);
        assert!((r.speed - 1.31492).abs() < 1e-4);
        assert!((r.track - 330.64).abs() < 1e-9);
    }

    #[test]
    fn keeps_the_timestamp_of_a_void_record() {
        let r = parse_rmc("$GNRMC,125721.849,V,,,,,0.00,0.00,200826,,,N,V*20").unwrap();
        assert!(!r.fix);
        assert!(r.lat.is_nan() && r.lon.is_nan());
        assert_eq!(r.utc.to_string(), "2026-08-20 12:57:21");
    }

    #[test]
    fn rejects_a_corrupt_checksum_and_other_sentences() {
        assert!(
            parse_rmc("$GNRMC,125721.849,V,,,,,0.00,0.00,200826,,,N,V*21").is_none(),
            "checksum mismatch must be rejected"
        );
        assert!(
            parse_rmc("$GNGGA,134627.000,2507.7617,N,12124.2784,E,1,18,0.67,105.4,M,14.8,M,,*78")
                .is_none()
        );
        assert!(
            parse_rmc("$GSENSORD,0.065,-0.002,0.062,[-0.712,0.050,0.648],0.090, 9433").is_none()
        );
        assert!(parse_rmc("$GNRM").is_none());
    }

    #[test]
    fn southern_and_western_hemispheres_are_negative() {
        assert!((decode_coord("2507.7744", "S").unwrap() + 25.129573).abs() < 1e-5);
        assert!((decode_coord("12124.2935", "W").unwrap() + 121.404892).abs() < 1e-5);
        assert!(decode_coord("2507.7744", "").is_none());
    }
}
