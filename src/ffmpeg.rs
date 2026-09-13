//! ffmpeg invocations: lossless concatenation, loudness measurement and the
//! final picture-in-picture composite.

use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result, bail};
use serde_json::Value;

/// Loudness target for the finished video, in the EBU R128 terms `loudnorm`
/// uses: −16 LUFS integrated, −1.5 dBTP true peak, 20 LU of allowed range.
const LOUDNORM_TARGET: &str = "I=-16:TP=-1.5:LRA=20";

/// Height the rear picture-in-picture is scaled to, in pixels.
pub const DEFAULT_PIP_HEIGHT: u32 = 432;

/// Gap between the GPS overlay and the bottom edge of the frame, as a fraction
/// of frame height — enough to clear the camera's burnt-in status line *and*
/// the Mio watermark above it (which reaches about 7% up a 2160p frame).
/// Expressed relatively so it holds if the recording resolution changes.
const OVERLAY_BOTTOM_MARGIN: &str = "H/12";

/// Concatenate `paths` without re-encoding.
///
/// Only the first video stream is kept: Mio clips also carry a low-resolution
/// preview track (and the front clip a data track), none of which belong in the
/// output and all of which would make the stream selectors ambiguous later.
pub fn concat(paths: &[&Path], output_file: &Path, with_audio: bool) -> Result<()> {
    let list_file = output_file.with_extension("concat.txt");

    let mut content = String::new();
    for path in paths {
        let abs = std::fs::canonicalize(path)
            .with_context(|| format!("cannot resolve {}", path.display()))?;
        let text = abs
            .to_str()
            .with_context(|| format!("non-UTF-8 path: {}", abs.display()))?;
        // The concat demuxer quotes like a shell: a literal ' must be spliced in.
        content.push_str(&format!("file '{}'\n", text.replace('\'', r"'\''")));
    }
    std::fs::write(&list_file, &content)?;

    let mut args: Vec<String> = vec![
        "-y",
        "-loglevel",
        "warning",
        "-stats",
        "-f",
        "concat",
        "-safe",
        "0",
        "-i",
    ]
    .into_iter()
    .map(String::from)
    .collect();
    args.push(path_arg(&list_file)?);
    args.extend(["-map".into(), "0:v:0".into()]);
    if with_audio {
        args.extend(["-map".into(), "0:a:0".into()]);
    }
    args.extend(["-c".into(), "copy".into(), path_arg(output_file)?]);

    let status = run("ffmpeg", &args)?;
    let _ = std::fs::remove_file(&list_file);

    if !status {
        bail!("ffmpeg concat failed for {}", output_file.display());
    }
    Ok(())
}

/// Measured loudness of a file, as reported by `loudnorm`'s analysis pass.
#[derive(Debug, Clone, Copy)]
pub struct Loudness {
    input_i: f64,
    input_tp: f64,
    input_lra: f64,
    input_thresh: f64,
    target_offset: f64,
}

impl Loudness {
    /// The second-pass filter string.
    ///
    /// `linear=true` makes loudnorm apply one constant gain across the whole
    /// file instead of riding the level: no compression, no gating, no EQ, so
    /// every transient and all the high and low detail of the original survive.
    /// If the required gain would breach the true-peak ceiling, loudnorm backs
    /// it off by itself.
    fn filter(&self) -> String {
        format!(
            "loudnorm={LOUDNORM_TARGET}:linear=true:measured_I={}:measured_TP={}:\
             measured_LRA={}:measured_thresh={}:offset={}",
            self.input_i, self.input_tp, self.input_lra, self.input_thresh, self.target_offset
        )
    }
}

/// Run loudnorm's analysis pass over the audio of `path`.
///
/// Video is not decoded (`-vn`), so this costs a fraction of the encode.
pub fn measure_loudness(path: &Path) -> Result<Loudness> {
    let output = Command::new("ffmpeg")
        .args([
            "-hide_banner",
            "-nostats",
            "-i",
            &path_arg(path)?,
            "-vn",
            "-af",
            &format!("loudnorm={LOUDNORM_TARGET}:print_format=json"),
            "-f",
            "null",
            "-",
        ])
        .output()
        .context("failed to run ffmpeg for loudness measurement")?;

    // loudnorm prints its JSON report to stderr, after everything else.
    let stderr = String::from_utf8_lossy(&output.stderr);
    let start = stderr
        .rfind('{')
        .context("loudnorm printed no JSON report")?;
    let end = stderr[start..]
        .find('}')
        .map(|i| start + i + 1)
        .context("loudnorm JSON report is truncated")?;
    let json: Value = serde_json::from_str(&stderr[start..end])
        .context("could not parse the loudnorm JSON report")?;

    // Every field must be a real number: a null would otherwise build a filter
    // that silently falls back to dynamic normalization, or fail much later
    // with an unhelpful message.
    let field = |key: &str| -> Result<f64> {
        json[key]
            .as_str()
            .and_then(|s| s.parse::<f64>().ok())
            .filter(|v| v.is_finite())
            .with_context(|| format!("loudnorm did not measure {key} (got {})", json[key]))
    };

    Ok(Loudness {
        input_i: field("input_i")?,
        input_tp: field("input_tp")?,
        input_lra: field("input_lra")?,
        input_thresh: field("input_thresh")?,
        target_offset: field("target_offset")?,
    })
}

/// Inputs to the final composite.
pub struct Composite<'a> {
    pub front: &'a Path,
    /// Rear camera, overlaid on the top centre of the frame.
    pub rear: Option<&'a Path>,
    /// GPS track overlay, placed in the bottom-right corner.
    pub overlay: Option<&'a Path>,
    pub output: &'a Path,
    /// Height of the rear picture-in-picture.
    pub pip_height: u32,
    /// Measured loudness; without it the audio is copied through untouched.
    pub loudness: Option<Loudness>,
}

/// Build the final video: rear camera as picture-in-picture over the front
/// camera, GPS track in the corner, audio normalized.
pub fn compose(spec: &Composite) -> Result<()> {
    let mut args: Vec<String> = ["-y", "-loglevel", "warning", "-stats"]
        .into_iter()
        .map(String::from)
        .collect();

    args.extend(["-i".into(), path_arg(spec.front)?]);
    let mut next_input = 1;

    let mut rear_idx = None;
    if let Some(rear) = spec.rear {
        // The rear clip's own audio is never used.
        args.extend(["-an".into(), "-i".into(), path_arg(rear)?]);
        rear_idx = Some(next_input);
        next_input += 1;
    }
    let mut overlay_idx = None;
    if let Some(overlay) = spec.overlay {
        args.extend(["-i".into(), path_arg(overlay)?]);
        overlay_idx = Some(next_input);
    }

    let mut chain: Vec<String> = Vec::new();
    let mut last = "0:v:0".to_string();

    if let Some(idx) = rear_idx {
        // The rear camera looks through the back window, whose top quarter is
        // roof lining and glare; dropping it makes the inset mostly road.
        chain.push(format!(
            "[{idx}:v:0]crop=iw:ih*3/4:0:ih/4,scale=-2:{}:flags=lanczos[pip]",
            spec.pip_height
        ));
        chain.push(format!("[{last}][pip]overlay=(W-w)/2:0[base]"));
        last = "base".into();
    }
    if let Some(idx) = overlay_idx {
        chain.push(format!(
            "[{last}][{idx}:v:0]overlay=W-w:H-h-{OVERLAY_BOTTOM_MARGIN}[withgps]"
        ));
        last = "withgps".into();
    }
    // Give the video a stable label even when nothing was composited onto it.
    chain.push(format!("[{last}]null[vout]"));

    match &spec.loudness {
        Some(l) => chain.push(format!("[0:a:0]{}[aout]", l.filter())),
        None => chain.push("[0:a:0]anull[aout]".into()),
    }

    args.extend([
        "-filter_complex".into(),
        chain.join(";"),
        "-map".into(),
        "[vout]".into(),
        "-map".into(),
        "[aout]".into(),
        "-c:v".into(),
        "hevc_videotoolbox".into(),
        "-tag:v".into(),
        "hvc1".into(),
        "-c:a".into(),
        "aac".into(),
        "-b:a".into(),
        "96k".into(),
        "-movflags".into(),
        "+faststart".into(),
        path_arg(spec.output)?,
    ]);

    if !run("ffmpeg", &args)? {
        bail!("ffmpeg composite failed for {}", spec.output.display());
    }
    Ok(())
}

fn run(program: &str, args: &[String]) -> Result<bool> {
    Ok(Command::new(program)
        .args(args)
        .status()
        .with_context(|| format!("failed to run {program} — is ffmpeg installed?"))?
        .success())
}

fn path_arg(path: &Path) -> Result<String> {
    Ok(path
        .to_str()
        .with_context(|| format!("non-UTF-8 path: {}", path.display()))?
        .to_string())
}

/// Intermediate file name for a concatenated single-camera stream.
pub fn intermediate(output_dir: &Path, name: &str, suffix: &str) -> PathBuf {
    output_dir.join(format!("{name}_{suffix}.mp4"))
}
