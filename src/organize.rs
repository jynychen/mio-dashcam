//! Flatten a raw memory card into one folder per date.
//!
//! ```text
//!   src   Normal/F/FILE{DATE}-{TIME}F.mp4      Event/F/EMER{DATE}-{TIME}F.mp4
//!         Normal/R/FILE{DATE}-{TIME}R.mp4      Event/R/EMER{DATE}-{TIME}R.mp4
//!         Normal/NMEA/FILE{DATE}-{TIME}F.NMEA  Event/NMEA/EMER{DATE}-{TIME}F.NMEA
//!
//!   dst   {YYYYMMDD}/FILE{DATE}-{TIME}{F,R}.{mp4,NMEA}
//! ```
//!
//! Event clips are renamed `EMER` → `FILE` so that they sort and glob together
//! with the normal ones and the day folder becomes a single continuous
//! recording.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use walkdir::WalkDir;

use crate::name;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Move,
    Copy,
}

impl Mode {
    fn verb(self) -> &'static str {
        match self {
            Mode::Move => "move",
            Mode::Copy => "copy",
        }
    }
}

#[derive(Debug, Default)]
pub struct Report {
    pub done: usize,
    pub skipped: usize,
    pub unknown: usize,
}

/// Sub-folders of a raw card, in the order the camera creates them.
const CARD_DIRS: [&str; 8] = [
    "Normal/F",
    "Normal/R",
    "Normal/I",
    "Normal/NMEA",
    "Event/F",
    "Event/R",
    "Event/I",
    "Event/NMEA",
];

/// Move (or copy) every recognised file under `src` into `dest/{YYYYMMDD}/`.
///
/// The camera's own folder layout is used when it is present, and otherwise the
/// whole tree is walked — so a folder of loose clips can be organized too.
pub fn run(src: &Path, dest: &Path, mode: Mode, dry_run: bool) -> Result<Report> {
    if !src.is_dir() {
        bail!("no such folder: {}", src.display());
    }

    let mut report = Report::default();
    for path in collect(src) {
        let filename = path.file_name().unwrap_or_default().to_string_lossy();
        let Some(info) = name::parse(&filename) else {
            eprintln!("skip (unrecognised name): {}", path.display());
            report.unknown += 1;
            continue;
        };

        let target_dir = dest.join(info.day_folder());
        let target = target_dir.join(info.organized_name());
        if target == path {
            continue;
        }

        // An event clip often duplicates the normal segment of the same instant,
        // and after the EMER → FILE rename the two collide. Never overwrite:
        // leave the source alone and report it for review by hand.
        if target.exists() {
            if same_size(&path, &target)? {
                eprintln!("skip (already there, same size): {}", path.display());
            } else {
                eprintln!(
                    "skip (name collision, DIFFERENT size): {} -> {}",
                    path.display(),
                    target.display()
                );
            }
            report.skipped += 1;
            continue;
        }

        if dry_run {
            println!(
                "{}: {} -> {}",
                mode.verb(),
                path.display(),
                target.display()
            );
        } else {
            std::fs::create_dir_all(&target_dir)
                .with_context(|| format!("cannot create {}", target_dir.display()))?;
            transfer(&path, &target, mode)?;
        }
        report.done += 1;
    }

    Ok(report)
}

/// Gather candidate files, preferring the camera's own folder layout.
///
/// The roots are visited in [`CARD_DIRS`] order — `Normal/` before `Event/` —
/// so that when a normal and an event file describe the same segment, the
/// normal one lands in the archive and the event duplicate is the one reported
/// as a collision.
fn collect(src: &Path) -> Vec<PathBuf> {
    let card_dirs: Vec<PathBuf> = CARD_DIRS
        .iter()
        .map(|d| src.join(d))
        .filter(|p| p.is_dir())
        .collect();

    let roots = if card_dirs.is_empty() {
        vec![src.to_path_buf()]
    } else {
        card_dirs
    };

    let mut files: Vec<PathBuf> = Vec::new();
    for root in roots {
        let mut from_root: Vec<PathBuf> = WalkDir::new(root)
            .follow_links(false)
            .into_iter()
            .flatten()
            .filter(|e| {
                e.file_type().is_file() && !e.file_name().to_string_lossy().starts_with('.')
            })
            .map(|e| e.into_path())
            .collect();
        from_root.sort();
        files.extend(from_root);
    }
    files
}

fn same_size(a: &Path, b: &Path) -> Result<bool> {
    Ok(a.metadata()?.len() == b.metadata()?.len())
}

/// Move or copy one file, never overwriting an existing target.
fn transfer(from: &Path, to: &Path, mode: Mode) -> Result<()> {
    match mode {
        // A rename only works within one filesystem, and the common case here
        // is moving off a memory card; fall back to copy-then-delete.
        Mode::Move => match std::fs::rename(from, to) {
            Ok(()) => Ok(()),
            Err(_) => {
                copy(from, to)?;
                std::fs::remove_file(from)
                    .with_context(|| format!("copied but could not remove {}", from.display()))
            }
        },
        Mode::Copy => copy(from, to),
    }
}

fn copy(from: &Path, to: &Path) -> Result<()> {
    std::fs::copy(from, to)
        .with_context(|| format!("cannot copy {} -> {}", from.display(), to.display()))?;
    Ok(())
}
