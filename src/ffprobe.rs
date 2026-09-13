use std::path::Path;
use std::process::Command;

use anyhow::{Context, Result, bail};
use serde_json::Value;

#[derive(Debug, Clone, Copy)]
pub struct VideoInfo {
    pub fps: f64,
    pub duration: f64,
    pub width: u32,
    pub height: u32,
}

/// Probe the first video stream of `path`.
///
/// Mio clips carry more than one video stream (a full-resolution track plus a
/// low-resolution thumbnail track), so the *first* one is selected explicitly
/// everywhere — both here and in the ffmpeg filter graphs.
pub fn probe(path: &Path) -> Result<VideoInfo> {
    let output = Command::new("ffprobe")
        .args([
            // "error", not "quiet": a failure must be able to say why.
            "-v",
            "error",
            "-print_format",
            "json",
            "-show_streams",
            "-show_format",
            path.to_str().context("non-UTF-8 path")?,
        ])
        .output()
        .context("failed to run ffprobe — is ffmpeg installed?")?;

    if !output.status.success() {
        bail!(
            "ffprobe failed for {}: {}",
            path.display(),
            String::from_utf8_lossy(&output.stderr)
        );
    }

    let json: Value = serde_json::from_slice(&output.stdout)?;
    let stream = json["streams"]
        .as_array()
        .context("no streams in ffprobe output")?
        .iter()
        .find(|s| s["codec_type"] == "video")
        .with_context(|| format!("no video stream in {}", path.display()))?;

    let fps = parse_rational(stream["r_frame_rate"].as_str().context("no r_frame_rate")?)?;

    // Prefer the container duration; fall back to the stream's own.
    let duration = json["format"]["duration"]
        .as_str()
        .or_else(|| stream["duration"].as_str())
        .context("no duration in ffprobe output")?
        .parse::<f64>()?;

    Ok(VideoInfo {
        fps,
        duration,
        width: stream["width"].as_u64().unwrap_or(0) as u32,
        height: stream["height"].as_u64().unwrap_or(0) as u32,
    })
}

/// Parse ffprobe's `num/den` rational form (also accepts a bare number).
fn parse_rational(s: &str) -> Result<f64> {
    if let Some((n, d)) = s.split_once('/') {
        let den = d.parse::<f64>()?;
        if den == 0.0 {
            bail!("zero denominator in frame rate {s:?}");
        }
        Ok(n.parse::<f64>()? / den)
    } else {
        Ok(s.parse::<f64>()?)
    }
}
