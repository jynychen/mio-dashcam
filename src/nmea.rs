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
//! Sentence framing, checksum verification and field decoding are handled by the
//! [`nmea`] crate; this module narrows its `RmcData` to the few fields the
//! overlay uses and keeps the "one record per second, gaps included" shape the
//! rest of the pipeline expects.
//!
//! Note the clock mismatch: RMC timestamps are UTC while the file *name* is the
//! camera's local time.  [`crate::gps`] reconciles the two.

use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use anyhow::{Context, Result};
use chrono::{NaiveDateTime, Timelike};
use nmea::ParseResult;
use nmea::sentences::rmc::RmcStatusOfFix;

/// One 1 Hz RMC record.
#[derive(Debug, Clone)]
pub struct Rmc {
    /// Absolute UTC instant (date + time fields combined).
    pub utc: NaiveDateTime,
    /// `true` when the receiver reported a valid fix (status `A` or `D`).
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
/// Individual malformed or checksum-failing lines are ignored. If a read of the
/// file itself fails mid-stream, the function stops at the point of failure and
/// keeps the records that were already parsed successfully.
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
/// `$GPRMC` from GPS-only firmware).  Every other sentence — including the
/// vendor-specific `$GSENSORD` accelerometer line — fails to parse and is
/// dropped, as is a sentence whose checksum does not match.
///
/// A record without both a date and a time is dropped too: the overlay indexes
/// records by absolute instant, so a timestamp that cannot be placed on the
/// calendar is of no use.
///
/// The sub-second part of the time is dropped: the receiver emits one record per
/// second and the fractional offset (`.849`, `.000`, …) drifts between clips, so
/// keeping it would add jitter to the interpolation without adding information.
fn parse_rmc(line: &str) -> Option<Rmc> {
    let ParseResult::RMC(data) = nmea::parse_str(line.trim()).ok()? else {
        return None;
    };

    let utc = data.fix_date?.and_time(data.fix_time?.with_nanosecond(0)?);
    let fix = matches!(
        data.status_of_fix,
        RmcStatusOfFix::Autonomous | RmcStatusOfFix::Differential
    );

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
        lat: data.lat?,
        lon: data.lon?,
        // A fixed receiver leaves course empty — that is not a broken sentence,
        // so fall back to NaN and let the heading spline ignore the record.
        speed: data
            .speed_over_ground
            .map_or(f64::NAN, |kn| f64::from(kn) * 1.852),
        track: data.true_course.map_or(f64::NAN, f64::from),
    })
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
        assert!((r.track - 330.64).abs() < 1e-4);
    }

    #[test]
    fn keeps_the_timestamp_of_a_void_record() {
        let r = parse_rmc("$GNRMC,125721.849,V,,,,,0.00,0.00,200826,,,N,V*20").unwrap();
        assert!(!r.fix);
        assert!(r.lat.is_nan() && r.lon.is_nan());
        assert_eq!(r.utc.to_string(), "2026-08-20 12:57:21");
    }

    #[test]
    fn southern_and_western_hemispheres_are_negative() {
        let r =
            parse_rmc("$GPRMC,134607.655,A,2507.7744,S,12124.2935,W,0.71,330.64,200826,,,A,V*19")
                .unwrap();
        assert!((r.lat + 25.129573).abs() < 1e-5, "lat was {}", r.lat);
        assert!((r.lon + 121.404892).abs() < 1e-5, "lon was {}", r.lon);
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
    fn drops_a_record_without_a_date() {
        assert!(
            parse_rmc("$GNRMC,134607.655,A,2507.7744,N,12124.2935,E,0.71,330.64,,,,A,V*06")
                .is_none()
        );
    }
}
