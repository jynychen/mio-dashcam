//! Session assembly: NMEA sidecars in, one time-ordered fix series out.
//!
//! The problem this module solves is **placing each record on the session
//! timeline**.  The concatenated video starts at t=0 and runs for the summed
//! duration of its clips; a record has only a UTC instant.  Clip *n* begins at
//! the sum of the durations before it, and the record's offset inside that clip
//! is its UTC time converted to the camera's local zone minus the clip's
//! file-name timestamp.  The logger usually needs a few seconds to produce its
//! first record, so this offset is genuinely non-zero and cannot be replaced by
//! a record index.
//!
//! What comes out is the dashcam's answer to a RaceChrono GPS device: a [`Fix`]
//! per second, carrying the same channels (position, distance, speed, bearing)
//! that [`crate::motion`] derives the rest from.  Upsampling 1 Hz to the video's
//! frame rate used to happen here with cubic splines and now belongs to
//! [`crate::motion`], which does it the way RaceChrono does.

use anyhow::Result;
use chrono::{Duration, FixedOffset, NaiveDateTime};

use crate::nmea;
use crate::scan::Group;

/// A record whose in-clip offset falls outside `[-SLACK, duration + SLACK]`
/// does not belong to that clip — the logger kept writing after the video was
/// closed, or the file is stale.  Such records are dropped.
const CLIP_SLACK: f64 = 3.0;

/// Two records closer together than this are treated as the same instant.
const MIN_SPACING: f64 = 1e-3;

/// A frame further than this from the nearest record is reported as
/// [`PositionSource::Interpolated`] rather than a hardware fix.
const FIX_MAX_AGE: f64 = 1.5;

/// Heading is meaningless when the vehicle is crawling: the receiver's reported
/// course jitters wildly below this speed, so those records carry no bearing at
/// all and the last real heading is held instead.
const MIN_HEADING_SPEED: f64 = 5.0 / 3.6;

/// Mean Earth radius (IUGG), for the along-track distance channel.
const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// How a frame's position was obtained.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PositionSource {
    /// Backed by a record that carried a real receiver fix.
    HardwareFix,
    /// Interpolated between two records across a dropout.
    Interpolated,
    /// Nothing to draw — before the first fix, after the last, or inside a hole
    /// too wide to bridge.
    NoPosition,
}

/// One receiver fix, placed on the session timeline.
///
/// The fields mirror the GPS channels RaceChrono records for a session, in SI
/// units rather than its fixed-point ones.
#[derive(Debug, Clone, Copy)]
pub struct Fix {
    /// Seconds from the start of the concatenated session.
    pub t: f64,
    /// Degrees, WGS-84.
    pub lat: f64,
    pub lon: f64,
    /// Ground speed, m/s.
    pub speed: f64,
    /// Course over ground in degrees true; `NaN` when the vehicle is too slow
    /// for the receiver's course to mean anything.
    pub bearing: f64,
    /// Distance travelled since the first fix, metres.  This is what the trace
    /// decimation thins by.
    pub dist: f64,
}

/// Everything usable from one session's NMEA sidecars.
pub struct Session {
    /// Time-ordered, one per record that carried a position.
    pub fixes: Vec<Fix>,
    /// Records read from the logs, including the ones with no fix.
    pub record_count: usize,
    /// Local-time offset used to align UTC records with the file names.
    pub tz: FixedOffset,
}

impl Session {
    /// Whether `t` stands on a real receiver fix rather than between two.
    pub fn has_fix_at(&self, t: f64) -> bool {
        // Records are a second apart, so the preceding one is what the frame is
        // standing on.
        let next = self.fixes.partition_point(|f| f.t <= t);
        next.checked_sub(1)
            .is_some_and(|i| t - self.fixes[i].t <= FIX_MAX_AGE)
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

    let mut record_count = 0usize;
    let mut fixes: Vec<Fix> = Vec::new();
    for (idx, records) in parsed {
        let clip = &group.clips[idx];
        for r in records {
            let local = r.utc + tz;
            let in_clip = seconds_between(clip.start, local);
            if in_clip < -CLIP_SLACK || in_clip > clip.duration + CLIP_SLACK {
                continue;
            }
            record_count += 1;
            let t = clip_offsets[idx] + in_clip.clamp(0.0, clip.duration);
            // A record without a fix leaves a hole in the timeline, which is
            // exactly how the interpolator is told not to draw there.
            if let Some(fix) = fix_from(t, &r) {
                fixes.push(fix);
            }
        }
    }

    fixes.sort_by(|a, b| a.t.total_cmp(&b.t));
    fixes.dedup_by(|a, b| a.t - b.t < MIN_SPACING);
    accumulate_distance(&mut fixes);

    Ok(Session {
        fixes,
        record_count,
        tz,
    })
}

/// Turn one record into a fix, or `None` when it carries no usable position.
fn fix_from(t: f64, r: &nmea::Rmc) -> Option<Fix> {
    if !r.fix || !r.lat.is_finite() || !r.lon.is_finite() {
        return None;
    }
    // The log is in km/h; every derived channel wants m/s.
    let speed = r.speed / 3.6;
    Some(Fix {
        t,
        lat: r.lat,
        lon: r.lon,
        speed,
        bearing: if speed >= MIN_HEADING_SPEED {
            r.track
        } else {
            f64::NAN
        },
        dist: 0.0,
    })
}

/// Fill in the along-track distance channel.
fn accumulate_distance(fixes: &mut [Fix]) {
    let mut total = 0.0;
    for i in 1..fixes.len() {
        total += distance_m(&fixes[i - 1], &fixes[i]);
        fixes[i].dist = total;
    }
}

/// Great-circle distance between two fixes, in metres (haversine).
fn distance_m(a: &Fix, b: &Fix) -> f64 {
    let (lat1, lat2) = (a.lat.to_radians(), b.lat.to_radians());
    let d_lat = lat2 - lat1;
    let d_lon = (b.lon - a.lon).to_radians();
    let h = (d_lat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (d_lon / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * h.sqrt().asin()
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

/// Format a span of seconds for the progress output.
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
    use chrono::NaiveDate;

    fn record(fix: bool, speed: f64, track: f64) -> nmea::Rmc {
        nmea::Rmc {
            utc: NaiveDate::from_ymd_opt(2026, 8, 20)
                .unwrap()
                .and_hms_opt(13, 46, 7)
                .unwrap(),
            fix,
            lat: if fix { 25.0 } else { f64::NAN },
            lon: if fix { 121.0 } else { f64::NAN },
            speed,
            track,
        }
    }

    fn fix_at(t: f64, lat: f64, lon: f64) -> Fix {
        Fix {
            t,
            lat,
            lon,
            speed: 20.0,
            bearing: 90.0,
            dist: 0.0,
        }
    }

    #[test]
    fn a_record_without_a_fix_leaves_a_hole() {
        assert!(fix_from(0.0, &record(false, f64::NAN, f64::NAN)).is_none());
    }

    #[test]
    fn speed_becomes_metres_per_second_and_gates_the_heading() {
        let f = fix_from(1.0, &record(true, 72.0, 330.0)).unwrap();
        assert!((f.speed - 20.0).abs() < 1e-9);
        assert!((f.bearing - 330.0).abs() < 1e-9);

        // Crawling: the course is noise, so the fix carries no bearing.
        let slow = fix_from(1.0, &record(true, 0.9, 12.0)).unwrap();
        assert!(slow.bearing.is_nan());
        // A receiver that reports no speed at all must not report a heading.
        assert!(
            fix_from(1.0, &record(true, f64::NAN, 12.0))
                .unwrap()
                .bearing
                .is_nan()
        );
    }

    #[test]
    fn distance_accumulates_along_the_track() {
        // 0.001 degrees of latitude is 111 m.
        let mut fixes = vec![
            fix_at(0.0, 25.0, 121.0),
            fix_at(1.0, 25.001, 121.0),
            fix_at(2.0, 25.002, 121.0),
        ];
        accumulate_distance(&mut fixes);
        assert_eq!(fixes[0].dist, 0.0);
        assert!((fixes[1].dist - 111.2).abs() < 0.5, "{}", fixes[1].dist);
        assert!((fixes[2].dist - 222.4).abs() < 1.0, "{}", fixes[2].dist);
    }

    #[test]
    fn a_frame_is_a_hardware_fix_only_next_to_a_record() {
        let session = Session {
            fixes: vec![fix_at(10.0, 25.0, 121.0), fix_at(30.0, 25.01, 121.0)],
            record_count: 2,
            tz: FixedOffset::east_opt(8 * 3600).unwrap(),
        };
        assert!(session.has_fix_at(10.0));
        assert!(session.has_fix_at(11.2));
        // Mid-dropout, and before the first record.
        assert!(!session.has_fix_at(20.0));
        assert!(!session.has_fix_at(0.0));
    }
}
