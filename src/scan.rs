//! Input discovery and continuity grouping.
//!
//! The scanner walks the input tree recursively and keys everything off the
//! file *name* alone, so all three supported input shapes work unchanged:
//!
//!   * raw card layout     `Normal/F`, `Normal/R`, `Normal/NMEA`, `Event/…`
//!   * organized archive   `20260820/FILE260820-205705F.mp4`
//!   * any folder of clips (flat, or a mix of the above)
//!
//! Because grouping is driven by the decoded timestamps and not by the folder
//! structure, a session that crosses midnight — and therefore sits in two
//! different day folders of the organized archive — is still recognised as one
//! continuous recording.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use chrono::NaiveDateTime;
use rayon::prelude::*;
use walkdir::WalkDir;

use crate::ffprobe;
use crate::name::{self, Camera};

/// Slack allowed between the end of one clip and the start of the next before
/// they are considered separate recordings.  Segments written back-to-back by
/// the camera line up to well under a second; anything larger means the camera
/// was stopped in between.
pub const DEFAULT_GAP_TOLERANCE: f64 = 5.0;

#[derive(Debug, Clone)]
pub struct Clip {
    /// Local wall-clock start time, decoded from the file name.
    pub start: NaiveDateTime,
    /// True when this segment came from an `EMER` (event) file.
    pub event: bool,
    pub front: PathBuf,
    pub rear: Option<PathBuf>,
    pub nmea: Option<PathBuf>,
    /// Probed duration of the front clip, in seconds.
    pub duration: f64,
}

#[derive(Debug)]
pub struct Group {
    pub clips: Vec<Clip>,
}

impl Group {
    /// Start time of the first clip in the group.
    pub fn start(&self) -> NaiveDateTime {
        self.clips[0].start
    }

    /// Total duration of the concatenated session, in seconds.
    pub fn duration(&self) -> f64 {
        self.clips.iter().map(|c| c.duration).sum()
    }

    /// Output base name, e.g. `20260820_205705` — the same shape as the
    /// existing `gpx/` archive so tracks and videos line up by name.
    pub fn name(&self) -> String {
        self.start().format("%Y%m%d_%H%M%S").to_string()
    }

    /// Paths to all front-camera clips, in chronological order.
    pub fn front_paths(&self) -> Vec<&Path> {
        self.clips.iter().map(|c| c.front.as_path()).collect()
    }

    /// Rear-camera paths for the whole session, or `None` if any segment is
    /// missing its rear clip. A partial rear track cannot be concatenated in
    /// sync with the front camera, so the result is rejected as a whole.
    pub fn rear_paths(&self) -> Option<Vec<&Path>> {
        self.clips
            .iter()
            .map(|c| c.rear.as_deref())
            .collect::<Option<Vec<_>>>()
    }

    /// Whether at least one clip in the group has an NMEA sidecar.
    pub fn has_nmea(&self) -> bool {
        self.clips.iter().any(|c| c.nmea.is_some())
    }

    /// Wall-clock time the last clip finishes.
    pub fn end(&self) -> NaiveDateTime {
        let last = self.clips.last().expect("groups are never empty");
        last.start + chrono::Duration::milliseconds((last.duration * 1000.0) as i64)
    }

    /// True when the session spans more than one calendar day — in the
    /// organized archive its clips are then split across two day folders.
    pub fn crosses_midnight(&self) -> bool {
        self.start().date() != self.end().date()
    }
}

#[derive(Default)]
struct Slot {
    front: Option<PathBuf>,
    rear: Option<PathBuf>,
    nmea: Option<PathBuf>,
    event: bool,
}

/// Recursively collect every recognisable dashcam file under `root`.
///
/// Segments are keyed by their decoded start time, which is what joins the
/// front clip, the rear clip and the NMEA log of one recording back together
/// no matter which sub-folders they happen to live in.
pub fn scan(root: &Path) -> Result<Vec<Clip>> {
    if !root.exists() {
        bail!("no such folder: {}", root.display());
    }

    let mut found: Vec<(name::NameInfo, PathBuf)> = Vec::new();
    let mut unknown = 0usize;

    for entry in WalkDir::new(root).follow_links(false) {
        let entry = entry.context("failed to walk input folder")?;
        if !entry.file_type().is_file() {
            continue;
        }
        let filename = entry.file_name().to_string_lossy();
        if filename.starts_with('.') {
            continue;
        }
        match name::parse(&filename) {
            Some(info) => found.push((info, entry.path().to_path_buf())),
            None => unknown += 1,
        }
    }

    // Order by (timestamp, normal-before-event, path) so that the choice made
    // below for a segment recorded twice is deterministic and does not depend
    // on the order the filesystem happened to hand the files back.
    found.sort_by(|(a, pa), (b, pb)| (a.start, a.event, pa).cmp(&(b.start, b.event, pb)));

    let mut slots: BTreeMap<NaiveDateTime, Slot> = BTreeMap::new();
    for (info, path) in found {
        let slot = slots.entry(info.start).or_default();
        slot.event |= info.event;

        let target = match (info.is_nmea, info.camera) {
            (true, _) => &mut slot.nmea,
            (false, Camera::Front) => &mut slot.front,
            (false, Camera::Rear) => &mut slot.rear,
            // The interior camera is not part of the composite; ignore it.
            (false, Camera::Interior) => continue,
        };

        match target {
            // The camera writes both a normal and an event file when it latches
            // a segment, so one moment can have two recordings. The sort above
            // puts the normal one first; report the duplicate rather than
            // silently swapping it in.
            Some(existing) => eprintln!(
                "warning: duplicate for {}: keeping {}, ignoring {}",
                info.start,
                existing.display(),
                path.display()
            ),
            None => *target = Some(path),
        }
    }

    if unknown > 0 {
        eprintln!("note: {unknown} file(s) with unrecognised names were ignored");
    }

    // A segment without a front clip cannot be rendered; report and drop it.
    let (with_front, orphans): (Vec<_>, Vec<_>) =
        slots.into_iter().partition(|(_, s)| s.front.is_some());
    for (start, _) in &orphans {
        eprintln!("warning: no front clip for {start}, segment skipped");
    }

    // ffprobe is a process spawn per clip; run them across all cores.
    //
    // A clip that cannot be probed is dropped with a warning rather than
    // failing the scan: a card with one truncated segment on it should still
    // let everything else through.
    Ok(with_front
        .into_par_iter()
        .filter_map(|(start, slot)| {
            let front = slot.front.expect("partitioned on front.is_some()");
            match ffprobe::probe(&front) {
                Ok(info) => Some(Clip {
                    start,
                    event: slot.event,
                    front,
                    rear: slot.rear,
                    nmea: slot.nmea,
                    duration: info.duration,
                }),
                Err(e) => {
                    eprintln!("warning: skipping {}: {e:#}", front.display());
                    None
                }
            }
        })
        .collect())
}

/// Split time-ordered clips into continuous recording sessions.
///
/// Two consecutive clips belong together when the second one starts no later
/// than `tolerance` seconds after the first clip's probed end time. Using the
/// actual duration from ffprobe keeps short trailing segments and future camera
/// changes working without a hard-coded nominal length.
pub fn group(mut clips: Vec<Clip>, tolerance: f64) -> Vec<Group> {
    clips.sort_by_key(|c| c.start);

    let mut groups: Vec<Group> = Vec::new();
    let mut current: Vec<Clip> = Vec::new();

    for clip in clips {
        if let Some(prev) = current.last() {
            let gap = (clip.start - prev.start).num_milliseconds() as f64 / 1000.0 - prev.duration;
            if gap > tolerance {
                groups.push(Group {
                    clips: std::mem::take(&mut current),
                });
            }
        }
        current.push(clip);
    }
    if !current.is_empty() {
        groups.push(Group { clips: current });
    }

    groups
}

// ── Selection ─────────────────────────────────────────────────────────────────

/// One term of a `--select` spec.
#[derive(Debug, PartialEq, Eq)]
enum Selector {
    /// Inclusive 1-based position in the listing, as printed by `list`.
    Range(usize, usize),
    /// Prefix of a group name, e.g. `20260911` for a whole day.
    Prefix(String),
}

/// A group paired with its 1-based position in the full listing.
///
/// The position is carried around so that a filtered listing still shows the
/// same numbers as an unfiltered one — otherwise `--select` would renumber the
/// very rows it is selecting by.
pub type Selected = (usize, Group);

/// Keep the groups a `--select` spec names, in listing order.
///
/// `spec` is a comma-separated list where a term of up to four digits (or two
/// joined by `-`) is a position or range, and anything else is matched against
/// the start of the group name. `None` keeps everything.
///
/// A term that matches nothing is an error: it is nearly always a typo, and
/// silently rendering fewer videos than asked for is the worse failure.
pub fn select(groups: Vec<Group>, spec: Option<&str>) -> Result<Vec<Selected>> {
    let numbered: Vec<Selected> = groups
        .into_iter()
        .enumerate()
        .map(|(i, g)| (i + 1, g))
        .collect();

    let Some(spec) = spec else {
        return Ok(numbered);
    };
    let total = numbered.len();

    let mut keep = vec![false; total];
    for selector in parse_selection(spec)? {
        let mut matched = false;
        for (slot, (index, group)) in keep.iter_mut().zip(&numbered) {
            let hit = match &selector {
                Selector::Range(from, to) => (*from..=*to).contains(index),
                Selector::Prefix(prefix) => group.name().starts_with(prefix),
            };
            *slot |= hit;
            matched |= hit;
        }
        if !matched {
            match selector {
                Selector::Range(from, to) if from == to => {
                    bail!("there is no session #{from} — the input holds {total}")
                }
                Selector::Range(from, to) => {
                    bail!("no sessions in the range {from}-{to} — the input holds {total}")
                }
                Selector::Prefix(prefix) => bail!("no session whose name starts with {prefix}"),
            }
        }
    }

    Ok(numbered
        .into_iter()
        .zip(keep)
        .filter_map(|(entry, keep)| keep.then_some(entry))
        .collect())
}

fn parse_selection(spec: &str) -> Result<Vec<Selector>> {
    let terms: Vec<&str> = spec
        .split(',')
        .map(str::trim)
        .filter(|t| !t.is_empty())
        .collect();
    if terms.is_empty() {
        bail!("--select is empty");
    }

    terms.into_iter().map(parse_selector).collect()
}

fn parse_selector(term: &str) -> Result<Selector> {
    // Positions are small; a date (8 digits) or a full group name is not. The
    // digit-count split is what keeps `4` and `20260911` unambiguous.
    let is_position = |s: &str| s.len() <= 4 && s.chars().all(|c| c.is_ascii_digit());

    let bounds = match term.split_once('-') {
        Some((from, to)) if is_position(from) && is_position(to) => Some((from, to)),
        None if is_position(term) => Some((term, term)),
        _ => None,
    };

    let Some((from, to)) = bounds else {
        return Ok(Selector::Prefix(term.to_string()));
    };

    let from: usize = from.parse()?;
    let to: usize = to.parse()?;
    if from == 0 || to == 0 {
        bail!("session numbers start at 1: {term}");
    }
    // Accept a reversed range rather than rejecting it — the intent is clear.
    Ok(Selector::Range(from.min(to), from.max(to)))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn clip(start: &str, duration: f64) -> Clip {
        Clip {
            start: NaiveDateTime::parse_from_str(start, "%Y-%m-%d %H:%M:%S").unwrap(),
            event: false,
            front: PathBuf::from(format!("{start}.mp4")),
            rear: None,
            nmea: None,
            duration,
        }
    }

    #[test]
    fn back_to_back_clips_form_one_group() {
        let groups = group(
            vec![
                clip("2026-08-20 20:57:05", 180.0),
                clip("2026-08-20 21:00:06", 180.0),
                clip("2026-08-20 21:03:06", 180.0),
            ],
            DEFAULT_GAP_TOLERANCE,
        );
        assert_eq!(groups.len(), 1);
        assert_eq!(groups[0].clips.len(), 3);
        assert_eq!(groups[0].name(), "20260820_205705");
    }

    #[test]
    fn a_long_gap_starts_a_new_group() {
        let groups = group(
            vec![
                clip("2026-08-20 20:57:05", 180.0),
                clip("2026-08-20 23:04:19", 180.0),
            ],
            DEFAULT_GAP_TOLERANCE,
        );
        assert_eq!(groups.len(), 2);
    }

    #[test]
    fn a_session_crossing_midnight_stays_one_group() {
        // These two sit in different day folders of the organized archive.
        let groups = group(
            vec![
                clip("2026-08-26 23:58:45", 180.0),
                clip("2026-08-27 00:01:45", 180.0),
            ],
            DEFAULT_GAP_TOLERANCE,
        );
        assert_eq!(groups.len(), 1);
        assert!(groups[0].crosses_midnight());
        assert_eq!(groups[0].name(), "20260826_235845");
    }

    /// Four sessions on three different days, as `list` would number them.
    fn sessions() -> Vec<Group> {
        [
            "2026-08-20 20:57:05",
            "2026-09-11 09:19:27",
            "2026-09-11 20:30:02",
            "2026-09-13 16:36:41",
        ]
        .iter()
        .map(|s| Group {
            clips: vec![clip(s, 180.0)],
        })
        .collect()
    }

    fn names(selected: &[Selected]) -> Vec<(usize, String)> {
        selected.iter().map(|(i, g)| (*i, g.name())).collect()
    }

    #[test]
    fn no_spec_keeps_everything_numbered_from_one() {
        let selected = select(sessions(), None).unwrap();
        assert_eq!(selected.len(), 4);
        assert_eq!(selected[0].0, 1);
        assert_eq!(selected[3].0, 4);
    }

    #[test]
    fn positions_ranges_and_lists_select_by_listing_number() {
        let picked = names(&select(sessions(), Some("2")).unwrap());
        assert_eq!(picked, [(2, "20260911_091927".to_string())]);

        let picked = names(&select(sessions(), Some("2-3")).unwrap());
        assert_eq!(picked.iter().map(|(i, _)| *i).collect::<Vec<_>>(), [2, 3]);

        // Overlapping and out-of-order terms still yield each group once,
        // in listing order, keeping its original number.
        let picked = names(&select(sessions(), Some("4, 1-2, 2")).unwrap());
        assert_eq!(
            picked.iter().map(|(i, _)| *i).collect::<Vec<_>>(),
            [1, 2, 4]
        );
    }

    #[test]
    fn a_prefix_selects_by_name_or_whole_day() {
        let picked = names(&select(sessions(), Some("20260911")).unwrap());
        assert_eq!(picked.iter().map(|(i, _)| *i).collect::<Vec<_>>(), [2, 3]);

        let picked = names(&select(sessions(), Some("20260911_2030")).unwrap());
        assert_eq!(picked, [(3, "20260911_203002".to_string())]);
    }

    #[test]
    fn a_term_that_matches_nothing_is_an_error() {
        // Typos must not silently render a shorter list than asked for.
        assert!(select(sessions(), Some("9")).is_err());
        assert!(select(sessions(), Some("20250101")).is_err());
        assert!(select(sessions(), Some("0")).is_err());
        assert!(select(sessions(), Some("")).is_err());
        // …but one good term does not excuse a bad one.
        assert!(select(sessions(), Some("1,99")).is_err());
    }

    #[test]
    fn digit_count_separates_a_position_from_a_date() {
        assert_eq!(parse_selector("7").unwrap(), Selector::Range(7, 7));
        assert_eq!(parse_selector("5-2").unwrap(), Selector::Range(2, 5));
        assert_eq!(
            parse_selector("20260911").unwrap(),
            Selector::Prefix("20260911".into())
        );
        assert_eq!(
            parse_selector("20260911_091927").unwrap(),
            Selector::Prefix("20260911_091927".into())
        );
    }

    #[test]
    fn a_short_trailing_clip_does_not_split_the_next_session() {
        // Last clip of a drive is cut short when the engine stops; the next
        // drive begins minutes later and must not be glued on.
        let groups = group(
            vec![
                clip("2026-08-20 20:57:05", 42.0),
                clip("2026-08-20 21:10:00", 180.0),
            ],
            DEFAULT_GAP_TOLERANCE,
        );
        assert_eq!(groups.len(), 2);
    }
}
