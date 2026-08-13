#!/usr/bin/env python3
"""Turn a BetterVR final-swapchain burst into compact LLM/debug evidence.

Outputs metrics.json, LLM_REVIEW.md, contact_sheet.png, and differences.png.
The detector is deliberately explainable: every verdict names the frame and metric.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


FRAME_RE = re.compile(r"frame(\d+)_color_([LR])\.bmp$", re.I)


def load_rgb(path: Path, width: int = 384) -> np.ndarray:
    image = Image.open(path).convert("RGB")
    image.thumbnail((width, width), Image.Resampling.LANCZOS)
    return np.asarray(image, dtype=np.uint8)


def luma(rgb: np.ndarray) -> np.ndarray:
    return (rgb[..., 0] * 0.2126 + rgb[..., 1] * 0.7152 + rgb[..., 2] * 0.0722).astype(np.float32) / 255.0


def mad(a: np.ndarray, b: np.ndarray) -> float:
    if a.shape != b.shape:
        b = np.asarray(Image.fromarray(b).resize((a.shape[1], a.shape[0]), Image.Resampling.LANCZOS))
    return float(np.abs(a.astype(np.float32) - b.astype(np.float32)).mean() / 255.0)


def estimate_translation(previous_luma: np.ndarray, current_luma: np.ndarray, limit: int = 48) -> tuple[int, int, float]:
    """Global screen-space motion via phase correlation (current relative to previous)."""
    height = min(previous_luma.shape[0], current_luma.shape[0])
    width = min(previous_luma.shape[1], current_luma.shape[1])
    a = previous_luma[:height, :width] - float(previous_luma[:height, :width].mean())
    b = current_luma[:height, :width] - float(current_luma[:height, :width].mean())
    window = np.outer(np.hanning(height), np.hanning(width)).astype(np.float32)
    cross = np.fft.fft2(a * window) * np.conj(np.fft.fft2(b * window))
    cross /= np.maximum(np.abs(cross), 1e-9)
    correlation = np.abs(np.fft.ifft2(cross))
    peak_y, peak_x = np.unravel_index(np.argmax(correlation), correlation.shape)
    if peak_x > width // 2:
        peak_x -= width
    if peak_y > height // 2:
        peak_y -= height
    dx, dy = int(np.clip(-peak_x, -limit, limit)), int(np.clip(-peak_y, -limit, limit))
    return dx, dy, float(correlation.max())


def aligned_mad(previous: np.ndarray, current: np.ndarray, dx: int, dy: int) -> tuple[float, np.ndarray]:
    shifted = np.roll(previous, shift=(dy, dx), axis=(0, 1))
    x0, x1 = max(0, dx), min(current.shape[1], current.shape[1] + dx)
    y0, y1 = max(0, dy), min(current.shape[0], current.shape[0] + dy)
    if x1 <= x0 or y1 <= y0:
        return mad(previous, current), np.abs(current.astype(np.int16) - previous.astype(np.int16)).astype(np.uint8)
    delta = np.abs(current.astype(np.int16) - shifted.astype(np.int16)).astype(np.uint8)
    score = float(delta[y0:y1, x0:x1].astype(np.float32).mean() / 255.0)
    return score, delta


def tile(image: np.ndarray, label: str, width: int = 384) -> Image.Image:
    source = Image.fromarray(image)
    canvas = Image.new("RGB", (width, source.height + 24), "#111318")
    canvas.paste(source, (0, 24))
    ImageDraw.Draw(canvas).text((6, 5), label, fill="#f1f5f9", font=ImageFont.load_default())
    return canvas


def sheet(rows: list[list[Image.Image]], output: Path) -> None:
    if not rows:
        return
    row_width = max(sum(i.width for i in row) for row in rows)
    height = sum(max(i.height for i in row) for row in rows)
    canvas = Image.new("RGB", (row_width, height), "#090b10")
    y = 0
    for row in rows:
        x = 0
        row_height = max(i.height for i in row)
        for image in row:
            canvas.paste(image, (x, y))
            x += image.width
        y += row_height
    canvas.save(output, optimize=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path, help="BetterVR dump or OpenXR Simulator incident directory")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-frames", type=int, default=24)
    args = parser.parse_args()
    session_directories = sorted((path for path in args.input.glob("session_*") if path.is_dir()), key=lambda path: path.stat().st_mtime)
    if session_directories:
        args.input = session_directories[-1]
    output = args.output or args.input / "flicker_analysis"
    output.mkdir(parents=True, exist_ok=True)

    incident_metadata: dict = {}
    incident_path = args.input / "incident.json"
    if incident_path.exists():
        try:
            incident_metadata = json.loads(incident_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            incident_metadata = {"metadataError": "incident.json could not be parsed"}
    capture_source = incident_metadata.get(
        "captureSource",
        "openxr-simulator-composed-preview" if incident_metadata else "openxr-final-swapchain",
    )
    simulator_preview = capture_source.startswith("openxr-simulator-")
    ui_only_capture = capture_source == "openxr-simulator-ui-quad"

    indexed: dict[int, dict[str, Path]] = {}
    for path in args.input.glob("frame*_color_?.bmp"):
        match = FRAME_RE.match(path.name)
        if match:
            indexed.setdefault(int(match.group(1)), {})[match.group(2).upper()] = path
    frames = [(number, pair) for number, pair in sorted(indexed.items()) if "L" in pair and "R" in pair][-args.max_frames :]
    if len(frames) < 2:
        raise SystemExit("Need at least two complete L/R final-output frames")

    loaded: list[dict] = []
    for number, pair in frames:
        left, right = load_rgb(pair["L"]), load_rgb(pair["R"])
        yl, yr = luma(left), luma(right)
        loaded.append({
            "frame": number, "L": left, "R": right, "yl": yl, "yr": yr,
            "meanL": float(yl.mean()), "meanR": float(yr.mean()),
            "eyeMad": mad(left, right),
            "files": {eye: str(path.resolve()) for eye, path in pair.items()},
        })

    incidents: list[dict] = []
    runtime_reason = incident_metadata.get("reason")
    if runtime_reason in {
        "VISIBLE_TO_BLANK_FRAME", "LARGE_VISIBLE_TEMPORAL_JUMP", "VISIBLE_LUMA_FLASH",
        "ASYMMETRIC_EYE_FLASH", "ALTERNATING_VISIBLE_FRAMES", "MISSING_PROJECTION_LAYER",
        "PREVIEW_PAINT_FAILURE",
        "UI_NOT_RECOMPOSED_AFTER_PROJECTION", "UI_CACHE_UNAVAILABLE_AFTER_PROJECTION",
        "UI_COMPOSITION_FAILED", "ALTERNATING_UI_REGION", "LARGE_UI_REGION_FLASH",
        "ASYMMETRIC_UI_EYE_FLASH",
    }:
        incidents.append({
            "frame": int(incident_metadata.get("triggerFrame", frames[0][0])),
            "kind": "runtime-trigger",
            "reason": runtime_reason,
        })
    contact_rows: list[list[Image.Image]] = []
    diff_rows: list[list[Image.Image]] = []
    motion_rows: list[list[Image.Image]] = []
    transitions_l: list[float] = []
    transitions_r: list[float] = []
    for index, item in enumerate(loaded):
        label = f"frame {item['frame']} mean={item['meanL']:.3f}/{item['meanR']:.3f} eyeMAD={item['eyeMad']:.3f}"
        contact_rows.append([tile(item["L"], "L " + label), tile(item["R"], "R " + label)])
        if index == 0:
            item["temporalL"] = item["temporalR"] = 0.0
            continue
        previous = loaded[index - 1]
        temporal_l = mad(item["L"], previous["L"])
        temporal_r = mad(item["R"], previous["R"])
        motion_l = estimate_translation(previous["yl"], item["yl"])
        motion_r = estimate_translation(previous["yr"], item["yr"])
        compensated_l, motion_delta_l = aligned_mad(previous["L"], item["L"], motion_l[0], motion_l[1])
        compensated_r, motion_delta_r = aligned_mad(previous["R"], item["R"], motion_r[0], motion_r[1])
        item["temporalL"], item["temporalR"] = temporal_l, temporal_r
        item["motionL"], item["motionR"] = {"dx": motion_l[0], "dy": motion_l[1], "confidence": motion_l[2]}, {"dx": motion_r[0], "dy": motion_r[1], "confidence": motion_r[2]}
        item["motionCompensatedL"], item["motionCompensatedR"] = compensated_l, compensated_r
        transitions_l.append(temporal_l)
        transitions_r.append(temporal_r)
        dl = np.abs(item["L"].astype(np.int16) - previous["L"].astype(np.int16)).astype(np.uint8)
        dr = np.abs(item["R"].astype(np.int16) - previous["R"].astype(np.int16)).astype(np.uint8)
        de = np.abs(item["L"].astype(np.int16) - item["R"].astype(np.int16)).astype(np.uint8)
        diff_rows.append([tile(np.minimum(dl * 3, 255).astype(np.uint8), f"L temporal {previous['frame']}->{item['frame']} MAD={temporal_l:.3f}"),
                          tile(np.minimum(dr * 3, 255).astype(np.uint8), f"R temporal MAD={temporal_r:.3f}"),
                          tile(np.minimum(de * 3, 255).astype(np.uint8), f"stereo diff MAD={item['eyeMad']:.3f}")])
        motion_rows.append([tile(np.minimum(motion_delta_l * 3, 255).astype(np.uint8), f"L motion ({motion_l[0]},{motion_l[1]}) residual={compensated_l:.3f}"),
                            tile(np.minimum(motion_delta_r * 3, 255).astype(np.uint8), f"R motion ({motion_r[0]},{motion_r[1]}) residual={compensated_r:.3f}")])

        for eye in ("L", "R"):
            mean = item["mean" + eye]
            prior_mean = previous["mean" + eye]
            temporal = item["temporal" + eye]
            if (mean > 0.92 or mean < 0.015) and abs(mean - prior_mean) > 0.2:
                incidents.append({"frame": item["frame"], "kind": "blank-flash", "eye": eye, "value": mean, "previous": prior_mean})
            compensated = item["motionCompensated" + eye]
            if temporal > 0.22 and compensated > 0.14:
                incidents.append({"frame": item["frame"], "kind": "large-temporal-jump", "eye": eye, "value": temporal, "motionCompensated": compensated})
        if abs(temporal_l - temporal_r) > 0.16:
            incidents.append({"frame": item["frame"], "kind": "asymmetric-eye-change", "valueL": temporal_l, "valueR": temporal_r})
        if (temporal_l < 0.0003 and temporal_r > 0.02) or (temporal_r < 0.0003 and temporal_l > 0.02):
            incidents.append({"frame": item["frame"], "kind": "frozen-eye", "valueL": temporal_l, "valueR": temporal_r})

    # Robust outlier pass catches lower-amplitude flashes without assuming a fixed scene.
    for eye, values in (("L", transitions_l), ("R", transitions_r)):
        values_np = np.asarray(values)
        median = float(np.median(values_np))
        robust_mad = float(np.median(np.abs(values_np - median)))
        threshold = max(0.075, median + 6.0 * max(robust_mad, 0.002))
        for item in loaded[1:]:
            value = item["temporal" + eye]
            if value > threshold and not any(i["frame"] == item["frame"] and i.get("eye") == eye for i in incidents):
                incidents.append({"frame": item["frame"], "kind": "temporal-outlier", "eye": eye, "value": value, "threshold": threshold})

    # Alternating brightness is characteristic simulator flicker and differs from one cut.
    for eye in ("L", "R"):
        means = [item["mean" + eye] for item in loaded]
        for i in range(2, len(means)):
            d1, d2 = means[i - 1] - means[i - 2], means[i] - means[i - 1]
            if abs(d1) > 0.08 and abs(d2) > 0.08 and d1 * d2 < 0:
                incidents.append({"frame": loaded[i]["frame"], "kind": "alternating-luminance", "eye": eye, "delta1": d1, "delta2": d2})

    unique_incidents = []
    seen = set()
    for incident in sorted(incidents, key=lambda value: (value["frame"], value["kind"], value.get("eye", ""))):
        key = (incident["frame"], incident["kind"], incident.get("eye"))
        if key not in seen:
            seen.add(key)
            unique_incidents.append(incident)

    sheet(contact_rows, output / "contact_sheet.png")
    sheet(diff_rows, output / "differences.png")
    sheet(motion_rows, output / "motion_compensated.png")
    result = {
        "schemaVersion": 1,
        "verdict": "FLICKER_DETECTED" if unique_incidents else "NO_FLICKER_DETECTED",
        "importantCaveat": "NO_FLICKER_DETECTED means this captured burst contains no detected event; it is not proof that the run never flickered.",
        "input": str(args.input.resolve()), "frameCount": len(loaded),
        "captureSource": capture_source,
        "captureMetadata": incident_metadata,
        "frameRange": [loaded[0]["frame"], loaded[-1]["frame"]],
        "incidentCount": len(unique_incidents), "incidents": unique_incidents,
        "summary": {
            "maxTemporalL": max(transitions_l), "maxTemporalR": max(transitions_r),
            "maxEyeMAD": max(item["eyeMad"] for item in loaded),
            "nearIdenticalFrames": sum(item["eyeMad"] < 0.002 for item in loaded),
        },
        "artifacts": {"contactSheet": str((output / "contact_sheet.png").resolve()), "differences": str((output / "differences.png").resolve()),
                      "motionCompensated": str((output / "motion_compensated.png").resolve())},
        "sourceSha256": {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for _, pair in frames for path in pair.values()},
        "frames": [{key: value for key, value in item.items() if key not in {"L", "R", "yl", "yr"}} for item in loaded],
    }
    (output / "metrics.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    incident_lines = "\n".join(f"- Frame {i['frame']}: `{i['kind']}` ({json.dumps(i, sort_keys=True)})" for i in unique_incidents) or "- None in this burst."
    source_description = (
        "only the left/right projected UI quad rectangles; world pixels outside the UI are excluded, and "
        "runtime triggers record whether the quad was omitted after a projection refresh"
        if ui_only_capture else
        "the OpenXR Simulator's fully composed preview surface after projection and overlay layers; "
        "this is the simulator window image, not the game window or an application swapchain"
        if simulator_preview else
        "the final left/right swapchain images submitted to OpenXR, not the pre-reprojection game textures"
    )
    markdown = f"""# OpenXR flicker review packet

Automated verdict: **{result['verdict']}**

This packet analyzes {source_description}. Review [contact_sheet.png](contact_sheet.png) for visible flashes/alternation, [differences.png](differences.png) for temporal/inter-eye discontinuities, and [motion_compensated.png](motion_compensated.png) for differences remaining after estimated screen-space camera motion.

## Detector evidence

{incident_lines}

## LLM review checklist

1. Look for a single eye changing brightness while the other remains stable.
2. Look for alternating bright/dark frames, white or black flashes, and a frozen eye.
3. Use estimated `(dx,dy)` vectors and motion-compensated differences to distinguish camera/head movement from whole-image flashes. These are image-space estimates, not game object velocities.
4. Treat `NO_FLICKER_DETECTED` only as a result for this {len(loaded)}-frame burst.

Machine-readable measurements are in [metrics.json](metrics.json).
"""
    (output / "LLM_REVIEW.md").write_text(markdown, encoding="utf-8")
    print(json.dumps({key: result[key] for key in ("verdict", "frameCount", "incidentCount", "artifacts")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
