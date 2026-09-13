//! Mio dashcam toolkit: organize the memory card, then turn each continuous
//! recording into one video with the rear camera as picture-in-picture and the
//! GPS track drawn in the corner.

mod ffmpeg;
mod ffprobe;
mod gps;
mod name;
mod nmea;
mod organize;
mod overlay;
mod scan;

use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use anyhow::{Context, Result};
use chrono::FixedOffset;
use clap::{Args, Parser, Subcommand};

use scan::{Group, Selected};

#[derive(Parser)]
#[command(
    name = "mio-dashcam",
    version,
    about = "Organize dashcam files and render front/rear composites with a GPS track overlay"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Flatten a memory card into one folder per date (EMER clips renamed to FILE)
    Organize {
        /// Source folder, containing Normal/ and Event/
        src: PathBuf,
        /// Where the date folders are created; defaults to the source folder
        dest: Option<PathBuf>,
        /// Print what would happen without touching any file
        #[arg(short = 'n', long)]
        dry_run: bool,
        /// Copy instead of move
        #[arg(short, long)]
        copy: bool,
    },

    /// List the continuous recording sessions found in the input
    List {
        #[command(flatten)]
        input: InputArgs,
    },

    /// Concatenate, composite the rear camera as picture-in-picture, draw the GPS track
    Render {
        #[command(flatten)]
        input: InputArgs,

        /// Output folder
        #[arg(short, long, default_value = "out")]
        output: PathBuf,

        /// Skip NMEA parsing and the track overlay
        #[arg(long)]
        no_gps: bool,

        /// Draw the track overlay at 1920px instead of the default 480px
        #[arg(long)]
        big: bool,

        /// Keep the intermediate concatenated _F / _R files
        #[arg(long)]
        keep: bool,

        /// Do not ask for confirmation
        #[arg(short, long)]
        yes: bool,

        /// Skip a session whose output file already exists
        #[arg(long)]
        skip_existing: bool,

        /// Height of the rear picture-in-picture, in pixels
        #[arg(long, default_value_t = ffmpeg::DEFAULT_PIP_HEIGHT)]
        pip_height: u32,
    },
}

#[derive(Args)]
struct InputArgs {
    /// Input: an organized archive, a raw memory card, or any folder of clips
    #[arg(default_value = ".")]
    input: PathBuf,

    /// Only process the named sessions: position `3`, range `2-5`, or name prefix
    /// `20260911`; comma-separated
    #[arg(short, long)]
    select: Option<String>,

    /// Largest gap between clips still counted as one recording, in seconds
    #[arg(long, default_value_t = scan::DEFAULT_GAP_TOLERANCE)]
    gap: f64,

    /// Dashcam time zone, e.g. +08:00; inferred from NMEA vs file-name times by default
    #[arg(long, value_parser = parse_tz)]
    tz: Option<FixedOffset>,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    // Ctrl-C sets a flag instead of killing the process, so the ffmpeg run in
    // flight finishes and leaves a complete file behind.
    let interrupted = Arc::new(AtomicBool::new(false));
    let flag = Arc::clone(&interrupted);
    ctrlc::set_handler(move || {
        flag.store(true, Ordering::SeqCst);
        eprintln!("\ninterrupt: waiting for the running ffmpeg to finish...");
    })?;

    match cli.command {
        Command::Organize {
            src,
            dest,
            dry_run,
            copy,
        } => {
            let dest = dest.unwrap_or_else(|| src.clone());
            let mode = if copy {
                organize::Mode::Copy
            } else {
                organize::Mode::Move
            };
            let report = organize::run(&src, &dest, mode, dry_run)?;
            println!(
                "{}: {}, skipped: {}, unrecognised: {}{}",
                if copy { "copied" } else { "moved" },
                report.done,
                report.skipped,
                report.unknown,
                if dry_run { " (dry run)" } else { "" }
            );
        }

        Command::List { input } => {
            let groups = load_groups(&input)?;
            print_groups(&groups, &input);
        }

        Command::Render {
            input,
            output,
            no_gps,
            big,
            keep,
            yes,
            skip_existing,
            pip_height,
        } => {
            let groups = load_groups(&input)?;
            print_groups(&groups, &input);
            if groups.is_empty() {
                return Ok(());
            }

            std::fs::create_dir_all(&output)
                .with_context(|| format!("cannot create {}", output.display()))?;

            if !yes && !confirm(&interrupted)? {
                println!("cancelled");
                return Ok(());
            }

            let opts = RenderOptions {
                output: &output,
                gps: !no_gps,
                overlay_size: if big { 1920 } else { 480 },
                keep,
                skip_existing,
                pip_height,
                tz: input.tz,
            };

            for (done, (index, group)) in groups.iter().enumerate() {
                println!(
                    "\n{}\n[{}/{}] #{index} {}\n{}",
                    "=".repeat(56),
                    done + 1,
                    groups.len(),
                    group.name(),
                    "=".repeat(56)
                );
                render_group(group, &opts, &interrupted)?;
                if interrupted.load(Ordering::SeqCst) {
                    println!("\ninterrupted; the remaining sessions were not processed");
                    return Ok(());
                }
            }
            println!("\n=== done ===");
        }
    }

    Ok(())
}

struct RenderOptions<'a> {
    output: &'a Path,
    gps: bool,
    overlay_size: u32,
    keep: bool,
    skip_existing: bool,
    pip_height: u32,
    tz: Option<FixedOffset>,
}

fn render_group(group: &Group, opts: &RenderOptions, interrupted: &AtomicBool) -> Result<()> {
    let name = group.name();
    let final_file = opts.output.join(format!("{name}.mp4"));

    if opts.skip_existing && final_file.exists() {
        println!("skipped, already there: {}", final_file.display());
        return Ok(());
    }

    // Step 1 — concatenate the front camera (lossless stream copy).
    println!(
        "--- concatenating front camera, {} ---",
        count(group.clips.len(), "clip")
    );
    let front_file = ffmpeg::intermediate(opts.output, &name, "F");
    ffmpeg::concat(&group.front_paths(), &front_file, true)?;
    if interrupted.load(Ordering::SeqCst) {
        return Ok(());
    }

    // Step 2 — concatenate the rear camera, when every segment has one.
    let rear_file = match group.rear_paths() {
        Some(paths) => {
            println!("--- concatenating rear camera ---");
            let path = ffmpeg::intermediate(opts.output, &name, "R");
            ffmpeg::concat(&paths, &path, false)?;
            Some(path)
        }
        None => {
            println!("--- rear camera incomplete, rendering front only ---");
            None
        }
    };
    if interrupted.load(Ordering::SeqCst) {
        return Ok(());
    }

    let info = ffprobe::probe(&front_file)?;
    println!(
        "front {}x{} @ {:.2} fps, {} long",
        info.width,
        info.height,
        info.fps,
        gps::format_duration(info.duration)
    );

    // Step 3 — GPS track overlay.
    let overlay_file = if opts.gps && group.has_nmea() {
        render_overlay(group, opts, &name, &info).unwrap_or_else(|e| {
            eprintln!("skipping the GPS overlay: {e:#}");
            None
        })
    } else {
        if opts.gps {
            println!("--- no NMEA, skipping the GPS overlay ---");
        }
        None
    };
    if interrupted.load(Ordering::SeqCst) {
        return Ok(());
    }

    // Step 4 — measure loudness so the composite can apply one constant gain.
    println!("--- measuring loudness ---");
    let loudness = match ffmpeg::measure_loudness(&front_file) {
        Ok(l) => Some(l),
        Err(e) => {
            eprintln!("loudness measurement failed, passing the audio through unchanged: {e:#}");
            None
        }
    };

    // Step 5 — the composite.
    println!("--- compositing ---");
    ffmpeg::compose(&ffmpeg::Composite {
        front: &front_file,
        rear: rear_file.as_deref(),
        overlay: overlay_file.as_deref(),
        output: &final_file,
        pip_height: opts.pip_height,
        loudness,
    })?;
    println!("→ {}", final_file.display());

    if let Some(path) = &overlay_file {
        let _ = std::fs::remove_file(path);
    }
    if !opts.keep {
        let _ = std::fs::remove_file(&front_file);
        if let Some(path) = &rear_file {
            let _ = std::fs::remove_file(path);
        }
    }

    Ok(())
}

/// Parse the session's NMEA logs and render the track overlay.
///
/// Returns `Ok(None)` when the drive holds too little GPS data to draw.
fn render_overlay(
    group: &Group,
    opts: &RenderOptions,
    name: &str,
    info: &ffprobe::VideoInfo,
) -> Result<Option<PathBuf>> {
    println!("--- parsing GPS ---");
    let session = gps::load_session(group, opts.tz)?;
    if session.record_count() == 0 {
        // No records means no timezone was inferred; don't report a made-up one.
        println!("no usable records in the NMEA logs");
    } else {
        println!(
            "{} records, {} with a fix, time zone {}",
            session.record_count(),
            session.fix_count,
            session.tz
        );
    }
    if session.fix_count < 2 {
        println!("not enough fixes, skipping the overlay");
        return Ok(None);
    }

    let track = gps::build_track(&session, info.fps, info.duration);
    println!(
        "--- rendering the track, {} ---",
        count(track.len(), "frame")
    );
    let path = overlay::render(&track, opts.output, name, info.fps, opts.overlay_size)?;
    Ok(Some(path))
}

/// Scan the input, split it into sessions and apply `--select`.
fn load_groups(input: &InputArgs) -> Result<Vec<Selected>> {
    let clips = scan::scan(&input.input)?;
    let groups = scan::group(clips, input.gap);
    let total = groups.len();
    let selected = scan::select(groups, input.select.as_deref())?;

    if input.select.is_some() {
        println!(
            "{} in {}, {} selected\n",
            count(total, "continuous recording"),
            input.input.display(),
            selected.len()
        );
    }
    Ok(selected)
}

fn print_groups(groups: &[Selected], input: &InputArgs) {
    if groups.is_empty() {
        println!("no usable clips in {}", input.input.display());
        return;
    }

    if input.select.is_none() {
        let clips: usize = groups.iter().map(|(_, g)| g.clips.len()).sum();
        println!(
            "{} in {}, {} in total\n",
            count(groups.len(), "continuous recording"),
            input.input.display(),
            count(clips, "clip")
        );
    }

    for (index, group) in groups {
        let mut notes: Vec<&str> = Vec::new();
        if group.crosses_midnight() {
            notes.push("crosses midnight");
        }
        if group.rear_paths().is_none() {
            notes.push("no rear");
        }
        if !group.has_nmea() {
            notes.push("no NMEA");
        }
        if group.clips.iter().any(|c| c.event) {
            notes.push("event clips");
        }

        println!(
            "{index:>3}. {}  {} -> {}  {:>9}  {}{}",
            group.name(),
            group.start().format("%Y-%m-%d %H:%M:%S"),
            group.end().format("%m-%d %H:%M:%S"),
            count(group.clips.len(), "clip"),
            gps::format_duration(group.duration()),
            if notes.is_empty() {
                String::new()
            } else {
                format!("  [{}]", notes.join("/"))
            }
        );
    }
}

/// Pluralise a count for the progress output: `1 clip`, `3 clips`.
fn count(n: usize, noun: &str) -> String {
    format!("{n} {noun}{}", if n == 1 { "" } else { "s" })
}

fn confirm(interrupted: &AtomicBool) -> Result<bool> {
    print!("\nstart processing? (y/N): ");
    io::stdout().flush()?;

    let mut answer = String::new();
    io::stdin().read_line(&mut answer)?;
    if interrupted.load(Ordering::SeqCst) {
        return Ok(false);
    }
    Ok(answer.trim().eq_ignore_ascii_case("y"))
}

/// Parse a UTC offset written as `+08:00`, `+0800`, `+8` or `8`.
fn parse_tz(s: &str) -> Result<FixedOffset> {
    let (sign, rest) = match s.strip_prefix('-') {
        Some(rest) => (-1, rest),
        None => (1, s.strip_prefix('+').unwrap_or(s)),
    };
    let (hours, minutes) = match rest.split_once(':') {
        Some((h, m)) => (h.parse::<i32>()?, m.parse::<i32>()?),
        None if rest.len() == 4 => (rest[0..2].parse::<i32>()?, rest[2..4].parse::<i32>()?),
        None => (rest.parse::<i32>()?, 0),
    };
    FixedOffset::east_opt(sign * (hours * 3600 + minutes * 60))
        .with_context(|| format!("time zone out of range: {s}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn accepts_every_offset_spelling() {
        let expected = FixedOffset::east_opt(8 * 3600).unwrap();
        for s in ["+08:00", "+0800", "+8", "8"] {
            assert_eq!(parse_tz(s).unwrap(), expected, "failed on {s}");
        }
        assert_eq!(
            parse_tz("-05:30").unwrap(),
            FixedOffset::east_opt(-(5 * 3600 + 30 * 60)).unwrap()
        );
        assert!(parse_tz("+99:00").is_err());
        assert!(parse_tz("abc").is_err());
    }
}
