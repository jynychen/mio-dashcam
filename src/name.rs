//! Dashcam file-name parsing.
//!
//! Mio names every artefact of one 3-minute segment after the same timestamp:
//!
//! ```text
//!   FILE260820-205705F.mp4    normal recording, front camera
//!   FILE260820-205705R.mp4    normal recording, rear camera
//!   FILE260820-205705F.NMEA   GPS/G-sensor log (always tagged F)
//!   EMER260827-002037F.mp4    event (G-sensor triggered) recording
//! ```
//!
//! The date part is 6 digits (YYMMDD) on current firmware but 8 (YYYYMMDD) on
//! some older/newer builds, so both are accepted.  The timestamp is the camera's
//! **local** wall clock — the NMEA log inside is UTC, see [`crate::nmea`].

use std::sync::LazyLock;

use chrono::{NaiveDate, NaiveDateTime};
use regex::Regex;

static NAME_RE: LazyLock<Regex> = LazyLock::new(|| {
    Regex::new(r"(?i)^(FILE|EMER)(\d{8}|\d{6})-(\d{6})([FRI])\.(mp4|nmea)$").unwrap()
});

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Camera {
    Front,
    Rear,
    /// Interior/cabin camera. Present as an empty `I/` folder on this model;
    /// recognised so such files are carried along by `organize` rather than
    /// reported as unknown.
    Interior,
}

#[derive(Debug, Clone)]
pub struct NameInfo {
    /// Date exactly as written in the file name (6 or 8 digits).
    pub date_raw: String,
    /// Time exactly as written in the file name (HHMMSS).
    pub time_raw: String,
    pub camera: Camera,
    /// Extension in its original case (`mp4`, `MP4`, `NMEA`, …).
    pub ext: String,
    /// Local wall-clock start time decoded from `date_raw` + `time_raw`.
    pub start: NaiveDateTime,
    pub is_nmea: bool,
    /// True for `EMER` clips.
    pub event: bool,
}

impl NameInfo {
    /// Four-digit-year day folder (`20260820`) used by the organized archive.
    pub fn day_folder(&self) -> String {
        if self.date_raw.len() == 6 {
            format!("20{}", self.date_raw)
        } else {
            self.date_raw.clone()
        }
    }

    /// Canonical name inside the organized archive: event clips are renamed
    /// `EMER` → `FILE` so they sort and glob together with the normal ones.
    pub fn organized_name(&self) -> String {
        let cam = match self.camera {
            Camera::Front => 'F',
            Camera::Rear => 'R',
            Camera::Interior => 'I',
        };
        format!(
            "FILE{}-{}{}.{}",
            self.date_raw, self.time_raw, cam, self.ext
        )
    }
}

/// Parse a supported Mio file name, returning `None` for other names or an
/// invalid calendar/time value.
pub fn parse(filename: &str) -> Option<NameInfo> {
    let caps = NAME_RE.captures(filename)?;
    let date_raw = caps[2].to_string();
    let time_raw = caps[3].to_string();
    let start = decode_datetime(&date_raw, &time_raw)?;
    let ext = caps[5].to_string();

    Some(NameInfo {
        event: caps[1].eq_ignore_ascii_case("EMER"),
        date_raw,
        time_raw,
        camera: match &caps[4].to_ascii_uppercase()[..] {
            "F" => Camera::Front,
            "R" => Camera::Rear,
            _ => Camera::Interior,
        },
        is_nmea: ext.eq_ignore_ascii_case("nmea"),
        ext,
        start,
    })
}

fn decode_datetime(date_raw: &str, time_raw: &str) -> Option<NaiveDateTime> {
    let (year, md) = if date_raw.len() == 8 {
        (date_raw[0..4].parse::<i32>().ok()?, &date_raw[4..])
    } else {
        (2000 + date_raw[0..2].parse::<i32>().ok()?, &date_raw[2..])
    };
    let date = NaiveDate::from_ymd_opt(year, md[0..2].parse().ok()?, md[2..4].parse().ok()?)?;
    date.and_hms_opt(
        time_raw[0..2].parse().ok()?,
        time_raw[2..4].parse().ok()?,
        time_raw[4..6].parse().ok()?,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_normal_front_clip() {
        let n = parse("FILE260820-205705F.mp4").unwrap();
        assert_eq!(n.camera, Camera::Front);
        assert!(!n.event && !n.is_nmea);
        assert_eq!(n.start.to_string(), "2026-08-20 20:57:05");
        assert_eq!(n.day_folder(), "20260820");
    }

    #[test]
    fn event_clip_is_renamed_for_the_archive() {
        let n = parse("EMER260827-002037R.MP4").unwrap();
        assert!(n.event);
        assert_eq!(n.organized_name(), "FILE260827-002037R.MP4");
    }

    #[test]
    fn accepts_four_digit_year_and_nmea() {
        let n = parse("FILE20260820-205705F.NMEA").unwrap();
        assert!(n.is_nmea);
        assert_eq!(n.day_folder(), "20260820");
        assert_eq!(n.start.to_string(), "2026-08-20 20:57:05");
    }

    #[test]
    fn rejects_foreign_names() {
        assert!(parse("IMG_1234.mp4").is_none());
        assert!(parse("FILE260820-205705F.mov").is_none());
        assert!(parse("FILE260820205705F.mp4").is_none());
    }
}
