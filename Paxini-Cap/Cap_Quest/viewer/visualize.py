#!/usr/bin/env python3
"""Look at one capture run: a Tk window by default, ``--html`` for a browser.

Reads ``<capture>/data.h5`` and shows joint angles, resultant-force magnitude,
distribution-force point counts, the per-slot resultant force (x, y, z) table,
and a schematic hand scatter plot of the 10×119 distribution-force points, on
one scrubbable time axis. Optional Quest 3 data is rendered in a separate panel
with its own SDK time axis.

Two output modes, same data model:

- default — matplotlib/Tk window.  Space plays, arrows step, clicking a trace
  jumps, the slider scrubs.  Needs ``python3.10-tk`` and a reachable display
  (``DISPLAY`` or ``WAYLAND_DISPLAY``).
- ``--html [PATH]`` — one self-contained HTML file with the data embedded.
  No toolkit, no server, no CDN: hand it to anyone and double-click.

The capture's three row axes (glove N, Quest Q, camera M) stay separate here
too.  Nothing is paired: the glove gets its own time axis, the Quest panel its
own, exactly like the file does.

Usage::

    python3.10 visualize.py <capture_dir>
    python3.10 visualize.py <capture_dir> --html
    python3.10 visualize.py <capture_dir> --html D:/viewer.html --no-quest

The capture directory is the one printed as ``[INFO] 数据写入：...`` at the end
of a run; it must contain ``data.h5`` and may contain ``manifest.json``.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

try:
    import h5py
    import numpy as np
except ImportError as exc:  # pragma: no cover - dependency guard
    print(
        f"[错误] 缺少依赖：{exc}。请运行 "
        "请在 Paxini-Cap/.venv 里 "
        "pip install --ignore-installed -r requirements.txt（见 QUICKSTART 第 1 步）",
        file=sys.stderr,
    )
    raise SystemExit(2)

# 采集程序在 ../capture 下。可视化单独成目录后必须显式加进 sys.path，
# 否则 from collect_cap import 会失败。
_CAPTURE_DIR = Path(__file__).resolve().parent.parent / "capture"
if str(_CAPTURE_DIR) not in sys.path:
    sys.path.insert(0, str(_CAPTURE_DIR))

# Reuse the recorder's constants so the viewer cannot drift from the file
# layout.  ``collect_cap`` only imports the SDK lazily, so this is safe on a
# machine without the glove plugged in.
from collect_cap import ANGLE_FIELDS, CAP_MAX_POINTS, CAP_SLOTS, CAP_SLOT_POINTS
try:
    # Local-only vendor point model; not committed (see .gitignore).
    from sensor_layout import (
        build_dist_flips,
        build_dist_points,
        build_dist_rotations,
    )

    DIST_LAYOUT_SOURCE = "studio"
except ImportError:
    # Public fallback: regular grids whose extents match the real models.
    from sensor_layout_schematic import (
        build_dist_flips,
        build_dist_points,
        build_dist_rotations,
    )

    DIST_LAYOUT_SOURCE = "schematic"
from quest_view import (
    QUEST_ALIGNMENT_ACTIVE,
    QUEST_ALIGNMENT_CHOICES,
    QUEST_ALIGNMENT_HOST,
    QUEST_ALIGNMENT_NOTE,
    QUEST_ALIGNMENT_SDK_UNIX,
    load_clock_report,
    load_quest_view,
)


ANGLE_LABELS = {
    "index_proximal": "FF2 食指近节",
    "index_distal": "FF1 食指远节",
    "middle_proximal": "MF2 中指近节",
    "middle_distal": "MF1 中指远节",
}

SLOT_LABELS = {
    "thumb_one": "拇指末",
    "thumb_two": "拇指中",
    "index_zero": "食指根",
    "index_one": "食指中",
    "middle_zero": "中指根",
    "middle_one": "中指中1",
    "middle_two": "中指中2",
    "middle_three": "中指末",
    "palm_zero": "掌根",
    "palm_one": "掌中",
}

DEFAULT_MAX_POINTS = 4000
DEFAULT_DIST_FRAMES = 600
# Fixed per-point colour scale.  The sensor's single-point force range is 25 N;
# an adaptive percentile made weak contacts saturate too early.
DIST_POINT_FORCE_MAX_N = 25.0


# --------------------------------------------------------------------------
# reading
# --------------------------------------------------------------------------
def load_manifest(root: Path) -> dict[str, Any]:
    """Prefer ``manifest.json``; fall back to the JSON blobs in ``meta``."""
    path = root / "manifest.json"
    if path.is_file():
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                return data
        except (OSError, json.JSONDecodeError):
            pass
    h5_path = root / "data.h5"
    if not h5_path.is_file():
        return {}
    try:
        with h5py.File(h5_path, "r") as handle:
            if "meta" not in handle:
                return {}
            meta: dict[str, Any] = {}
            for key, value in handle["meta"].attrs.items():
                try:
                    meta[key] = json.loads(value)
                except (TypeError, json.JSONDecodeError):
                    meta[key] = value
            return meta
    except OSError:
        return {}


def capture_rows(handle: h5py.File) -> int:
    """Row count of ``/cap`` from whichever dataset is present."""
    rows = 0
    if "cap" not in handle:
        return 0
    for name in handle["cap"]:
        dataset = handle["cap"][name]
        if isinstance(dataset, h5py.Dataset) and dataset.ndim >= 1:
            rows = max(rows, int(dataset.shape[0]))
    return rows


def read_time_axis(
    handle: h5py.File, rows: int, rate_hz: float
) -> tuple[np.ndarray, str, list[str]]:
    """Seconds since the first frame, plus which clock was used.

    Monotonic is preferred because a wall-clock step would make the x axis run
    backwards.  If the capture only has wall clock (old format) the axis is
    forced to be non-decreasing and the step is reported as a warning.
    """
    warnings: list[str] = []
    source = "monotonic"
    stamps: np.ndarray | None = None

    if "cap/timestamp_monotonic_ns" in handle:
        candidate = handle["cap/timestamp_monotonic_ns"][:].astype(np.int64)
        if candidate.size == rows and np.any(candidate):
            stamps = candidate

    if stamps is None:
        source = "unix"
        if "cap/timestamp_unix_ns" in handle:
            candidate = handle["cap/timestamp_unix_ns"][:].astype(np.int64)
            if candidate.size == rows and np.any(candidate):
                stamps = candidate

    if stamps is None:
        source = "index"
        rate = rate_hz if rate_hz > 0 else 120.0
        warnings.append(f"没有可用时间戳，按 {rate:.0f} Hz 名义频率推算横轴")
        return np.arange(rows, dtype=np.float64) / rate, source, warnings

    seconds = (stamps - stamps[0]) / 1e9
    backwards = int(np.count_nonzero(np.diff(seconds) < 0))
    if backwards:
        warnings.append(
            f"{source} 轴出现 {backwards} 处倒退（时钟被校准），已按累计最大值拉平"
        )
        seconds = np.maximum.accumulate(seconds)
    return seconds, source, warnings


def take_evenly(rows: int, limit: int) -> np.ndarray:
    """Indices spread over ``rows`` with the first and last frame kept."""
    if rows <= limit:
        return np.arange(rows)
    stride = math.ceil(rows / limit)
    index = np.arange(0, rows, stride)
    if index[-1] != rows - 1:
        index = np.append(index, rows - 1)
    return index


def take_envelope(
    rows: int, limit: int, signals: list[np.ndarray]
) -> np.ndarray:
    """Indices spread over ``rows`` that keep every bucket's envelope.

    ``take_evenly`` drops frames blindly, so a single-frame force spike can
    disappear as soon as a run is long enough to be thinned (120 Hz × 4000
    points ≈ 33 s).  Here each bucket contributes the rows where its own min
    and max are reached, so the drawn polyline still touches the extremes.
    Budget is ``limit // 4`` buckets × 2 signals × 2 rows = ``limit`` rows.
    """
    if rows <= limit:
        return np.arange(rows)
    usable = [np.asarray(sig, dtype=np.float64) for sig in signals
              if np.asarray(sig).shape == (rows,)]
    if not usable:
        return take_evenly(rows, limit)

    buckets = max(1, limit // 4)
    picked: set[int] = {0, rows - 1}
    for edge in range(buckets):
        start = rows * edge // buckets
        end = rows * (edge + 1) // buckets
        if end <= start + 1:
            continue
        for sig in usable:
            segment = sig[start:end]
            finite = np.isfinite(segment)
            if not finite.any():
                continue
            low = np.argmin(np.where(finite, segment, np.inf))
            high = np.argmax(np.where(finite, segment, -np.inf))
            picked.add(start + int(low))
            picked.add(start + int(high))
    return np.array(sorted(picked), dtype=np.int64)


def clock_quality(handle: h5py.File, rows: int) -> dict[str, Any] | None:
    """Drift between wall clock and monotonic clock over one capture.

    ``unix − monotonic`` is constant unless the host clock is stepped, so the
    per-frame difference of that offset is the cheapest NTP detector we have
    (``DOC/待验证清单.md`` D2).  ``None`` when the file predates the monotonic column.
    """
    if "cap/timestamp_unix_ns" not in handle:
        return None
    if "cap/timestamp_monotonic_ns" not in handle:
        return None
    unix = handle["cap/timestamp_unix_ns"][:].astype(np.int64)
    mono = handle["cap/timestamp_monotonic_ns"][:].astype(np.int64)
    if unix.size != rows or mono.size != rows or not np.any(unix):
        return None
    offset = (unix - mono) / 1e6  # ms
    step = np.abs(np.diff(offset)) if offset.size > 1 else np.zeros(0)
    return {
        "offset_first_ms": round(float(offset[0]), 3),
        "drift_ms": round(float(offset[-1] - offset[0]), 3),
        "max_step_ms": round(float(step.max()), 3) if step.size else 0.0,
        "steps_over_10ms": int(np.count_nonzero(step > 10.0)),
    }


def camera_tracks(handle: h5py.File, root: Path, glove_unix_ns: int) -> list[dict[str, Any]]:
    """Per-camera frame axis, plus where its first frame sits on the glove clock.

    The video files themselves keep their own timeline starting at 0, so the
    only thing needed to line a video up with the glove is this offset:
    ``video_time = glove_relative_time − offset_s``.  Nothing is interpolated
    or written back — this is display only.
    """
    tracks: list[dict[str, Any]] = []
    if "cameras" not in handle:
        return tracks
    for name in sorted(handle["cameras"]):
        path = f"cameras/{name}/timestamp_unix_ns"
        if path not in handle:
            continue
        stamps = handle[path][:].astype(np.int64)
        if stamps.size == 0 or not np.any(stamps):
            continue
        span_ns = int(stamps[-1] - stamps[0])
        mkv = root / "cameras" / f"{name}.mkv"
        tracks.append(
            {
                "id": name,
                "frames": int(stamps.size),
                "offset_s": round((int(stamps[0]) - glove_unix_ns) / 1e9, 4),
                "duration_s": round(span_ns / 1e9, 3),
                "fps": round((stamps.size - 1) / (span_ns / 1e9), 3) if span_ns > 0 else 0.0,
                "mkv": str(mkv) if mkv.is_file() else None,
                "file": None,
                "error": None,
            }
        )
    return tracks


def find_ffmpeg(argument: str | None = None) -> str | None:
    """``--ffmpeg`` wins, then ``$FFMPEG``, then PATH, then ``~/bin``."""
    for candidate in (argument, os.environ.get("FFMPEG")):
        if candidate:
            return candidate
    found = shutil.which("ffmpeg")
    if found:
        return found
    fallback = Path.home() / "bin" / "ffmpeg"
    return str(fallback) if fallback.is_file() else None


def build_preview(
    track: dict[str, Any], out_dir: Path, width: int, crf: int, ffmpeg: str
) -> str | None:
    """Transcode one mkv into a browser-playable mp4; returns the error or None.

    Chrome will not open Matroska, and the mkv holds MJPEG, so this has to be a
    real re-encode.  ``-fps_mode passthrough`` keeps the original frame timing
    (a CFR round-trip would hide the dropped frames we just measured), and
    ``-g 15`` puts a keyframe every half second so scrubbing stays responsive.
    """
    source = Path(track["mkv"] or "")
    if not track["mkv"] or not source.is_file():
        return "找不到 mkv"
    target = out_dir / f"{track['id']}.mp4"
    command = [
        ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
        "-i", str(source),
        "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", str(crf),
        "-vf", f"scale={width}:-2", "-pix_fmt", "yuv420p", "-g", "15",
        "-fps_mode", "passthrough", "-movflags", "+faststart",
        str(target),
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True)
    except OSError as exc:
        return f"ffmpeg 起不来：{exc}"
    if result.returncode != 0:
        detail = (result.stderr or "").strip().splitlines()
        return detail[-1] if detail else "ffmpeg 退出码非 0"
    if not target.is_file() or target.stat().st_size == 0:
        return "转码后没有输出"
    track["file"] = target.name
    track["bytes"] = target.stat().st_size
    return None


def round_list(values: np.ndarray, digits: int = 3) -> list[Any]:
    """JSON-friendly list with NaN turned into null."""
    flat = np.round(np.asarray(values, dtype=np.float64), digits)
    return [None if not np.isfinite(v) else float(v) for v in flat.ravel()]


def round_rows(values: np.ndarray, digits: int = 3) -> list[list[Any]]:
    return [round_list(row, digits) for row in values]


def build_slot_force(
    handle: h5py.File,
    index: np.ndarray,
    warnings: list[str],
    *,
    include_tangent: bool = True,
) -> dict[str, Any] | None:
    """Per-slot resultant force (x, y, z) as raw int16, base64 encoded.

    ``cap/resultant_force`` is ``(N, 10, 3)`` int16 in units of 0.1 N — the
    device already sums each finger segment for us, so there is nothing to
    integrate here.  Values are shipped raw (no scaling, no quantisation);
    the page divides by 10 and prints N with two decimals, which is lossless
    (0.1 N is exactly one decimal of a newton).
    """
    path = "cap/resultant_force"
    if path not in handle:
        warnings.append("这次采集没有合力数据，数值表留空")
        return None

    block = handle[path][index]
    if block.size == 0:
        return None
    counts = handle["cap/distribute_counts"][index].astype(np.int64)
    raw = np.ascontiguousarray(block.astype("<i2"))

    dist: dict[str, Any] | None = None
    dist_path = "cap/distribute_force"
    if dist_path in handle:
        # (F, 10, 119, 3) int16 in 0.1 N.  The magnitude is what the heatmap
        # colours; keeping 0.01 N in uint16 is lossless enough for hover text
        # and still small: 600 frames x 10 slots x 119 points = 1.4 MB raw.
        raw_dist = handle[dist_path][index].astype(np.int16)
        magnitude = np.linalg.norm(raw_dist.astype(np.float64), axis=-1) * 0.1
        valid = (
            np.arange(CAP_MAX_POINTS, dtype=np.int64)[None, :]
            < counts[:, :, None]
        )
        magnitude = np.where(valid, magnitude, 0.0)
        if not np.any(magnitude > 0.0):
            warnings.append("分布力全为 0：散点图只会显示暗点")
        scale_n = DIST_POINT_FORCE_MAX_N
        quantized = np.clip(
            np.rint(magnitude * 100.0), 0, np.iinfo(np.uint16).max
        ).astype("<u2")

        tangent_b64: str | None = None
        tangent_scale_n = 0.0
        if include_tangent:
            # Fx/Fy are the sensor-plane (tangential) components.  They arrive
            # as signed int8 in 0.1 N; keep them byte-for-byte in int8.
            tangent = np.zeros((*raw_dist.shape[:-1], 2), dtype=np.int8)
            tangent[..., 0] = np.clip(raw_dist[..., 0], -128, 127)
            tangent[..., 1] = np.clip(raw_dist[..., 1], -128, 127)
            tangent = np.where(valid[..., None], tangent, 0).astype(np.int8)
            tangent_mag = np.linalg.norm(
                tangent.astype(np.float64), axis=-1
            ) * 0.1
            tangent_nonzero = tangent_mag[tangent_mag > 0.0]
            if tangent_nonzero.size:
                tangent_scale_n = float(np.percentile(tangent_nonzero, 99))
                if (
                    not math.isfinite(tangent_scale_n)
                    or tangent_scale_n <= 0.0
                ):
                    tangent_scale_n = float(tangent_nonzero.max())
            tangent_b64 = base64.b64encode(
                np.ascontiguousarray(tangent).tobytes()
            ).decode("ascii")

        dist = {
            "shape": [len(CAP_SLOTS), CAP_MAX_POINTS],
            "capacities": [int(CAP_SLOT_POINTS[slot]) for slot in CAP_SLOTS],
            "scale_n": round(scale_n, 4),
            "data": base64.b64encode(
                np.ascontiguousarray(quantized).tobytes()
            ).decode("ascii"),
            # Static 2D positions for every slot's valid points.  The shapes
            # come from the local PXCap Studio sensor models; the module
            # arrangement is schematic (see sensor_layout.py).
            "points": build_dist_points(),
            "rotations": build_dist_rotations(),
            "flips": build_dist_flips(),
            "layout_source": DIST_LAYOUT_SOURCE,
            "tangent": tangent_b64,
            "tangent_scale_n": round(tangent_scale_n, 4),
        }
    else:
        warnings.append(
            "这次采集没有分布力数据（--omit-distribution），散点图不可用"
        )

    return {
        "slots": len(CAP_SLOTS),
        "frames": int(index.size),
        "unit": "N",
        "counts": [[int(v) for v in row] for row in counts],
        "data": base64.b64encode(raw.tobytes()).decode("ascii"),
        "dist": dist,
    }


def collect_payload(
    root: Path,
    max_points: int,
    dist_frames: int,
    with_quest: bool,
    *,
    quest_alignment: str = "none",
    quest_clock_report: dict[str, Any] | None = None,
    with_dist_arrows: bool = True,
) -> dict[str, Any]:
    h5_path = root / "data.h5"
    if not h5_path.is_file():
        raise SystemExit(f"[错误] {h5_path} 不存在")

    manifest = load_manifest(root)
    warnings: list[str] = []

    with h5py.File(h5_path, "r") as handle:
        rows = capture_rows(handle)
        if rows == 0:
            raise SystemExit("[错误] data.h5 里没有 /cap 数据集")

        rate = float(manifest.get("actual_rate_hz") or 0.0)
        seconds, clock_source, clock_warnings = read_time_axis(handle, rows, rate)
        warnings.extend(clock_warnings)

        # Read the full columns: the thinning below needs every row to know
        # where each bucket's extremes are.  120 Hz × 10 min is ~9 MB, fine.
        angles_all = handle["cap/joint_angles_deg"][:].astype(np.float64)
        force_all = handle["cap/resultant_force"][:].astype(np.float64)
        magnitude_all = np.linalg.norm(force_all, axis=2)
        index = take_envelope(
            rows, max_points,
            [np.nanmax(angles_all, axis=1), np.nanmax(magnitude_all, axis=1)],
        )
        angles = angles_all[index]
        magnitude = magnitude_all[index]
        counts = handle["cap/distribute_counts"][index].astype(np.int64)

        force_all_zero = not bool(np.any(force_all))
        angles_quiet = bool(np.nanmax(np.abs(angles_all)) < 1.0) if angles_all.size else True
        if force_all_zero:
            warnings.append(
                "这次采集的合力全为 0：手套没有受力，合力表整张是 0、曲线只有噪声底"
            )
        elif angles_quiet:
            warnings.append("这次采集的关节角始终小于 1°：手套没有被弯折")

        quality = clock_quality(handle, rows)
        if quality and quality["steps_over_10ms"]:
            warnings.append(
                f"墙钟在采集期间被校准 {quality['steps_over_10ms']} 次"
                f"（最大单步 {quality['max_step_ms']:.0f} ms），墙钟类时间戳那一侧已不可信"
            )

        table_index = take_evenly(rows, dist_frames)
        slot_force = build_slot_force(
            handle, table_index, warnings, include_tangent=with_dist_arrows
        )
        table_seconds = seconds[table_index] if slot_force is not None else []

        duration = float(np.nanmax(seconds)) if seconds.size else 0.0
        pose_meta = manifest.get("pose") or {}
        hand_meta = manifest.get("hand")
        hand_side = str(
            (hand_meta.get("side") if isinstance(hand_meta, dict) else None)
            or pose_meta.get("side")
            or "right"
        )
        cap_unix_first_ns = (
            int(handle["cap/timestamp_unix_ns"][0])
            if "cap/timestamp_unix_ns" in handle and rows
            else 0
        )
        quest_payload = None
        if with_quest:
            quest_payload = load_quest_view(
                handle,
                max_points,
                pose_source=str(pose_meta.get("source") or "controller"),
                pose_side=str(
                    pose_meta.get("side")
                    or (manifest.get("hand") or {}).get("side")
                    or "right"
                ),
                alignment=quest_alignment,
                cap_unix_ns=cap_unix_first_ns,
                clock_report=quest_clock_report,
                poll_rate_hz=float(
                    ((manifest.get("quest") or {}).get("poll_rate_hz"))
                    or 120.0
                ),
            )
            if (
                quest_payload is not None
                and quest_payload["alignment"] not in QUEST_ALIGNMENT_ACTIVE
            ):
                warnings.append(QUEST_ALIGNMENT_NOTE)
        quest_rows = int(manifest.get("quest_rows") or 0)
        if quest_payload is not None:
            quest_rows = max(quest_rows, int(quest_payload["rows"]))

        payload: dict[str, Any] = {
            "meta": {
                "name": root.name,
                "path": str(root),
                "format_version": manifest.get("format_version"),
                "hand_side": hand_side,
                "rows": rows,
                "quest_rows": quest_rows,
                "quest_shown": (
                    int(quest_payload["shown"])
                    if quest_payload is not None
                    else 0
                ),
                "quest_time_source": (
                    quest_payload["time_source"]
                    if quest_payload is not None
                    else None
                ),
                "quest_alignment": (
                    quest_payload["alignment"]
                    if quest_payload is not None
                    else "none"
                ),
                "rate_hz": round(rate, 2),
                "duration_s": round(duration, 3),
                "clock": clock_source,
                "camera_ids": list(
                    (manifest.get("output") or {}).get("camera_ids") or []
                ),
                "shown_rows": int(index.size),
                "stride": max(1, round(rows / index.size)) if index.size else 1,
                "thinning": "envelope" if index.size < rows else "none",
                "clock_quality": quality,
                "force_all_zero": force_all_zero,
                "angles_quiet": angles_quiet,
            },
            "cap": {
                "t": round_list(seconds[index], 4),
                "frames": [int(v) for v in index],
                "angles": round_rows(angles, 2),
                "force": round_rows(magnitude, 1),
                "counts": [[int(v) for v in row] for row in counts],
            },
            "slot_force": None
            if slot_force is None
            else {"t": round_list(table_seconds, 4), **slot_force},
            "quest": quest_payload,
            "video": {"cams": []},
            "warnings": warnings,
        }

        glove_unix_ns = cap_unix_first_ns
        if glove_unix_ns:
            tracks = camera_tracks(handle, root, glove_unix_ns)
            for track in tracks:
                if not track["mkv"]:
                    warnings.append(
                        f"{track['id']} 有时间轴但没有 {track['id']}.mkv，视频面板会缺这一路"
                    )
            payload["video"]["cams"] = tracks

    payload["cap"]["angle_labels"] = [
        ANGLE_LABELS.get(name, name) for name in ANGLE_FIELDS
    ]
    payload["cap"]["slot_labels"] = [SLOT_LABELS.get(s, s) for s in CAP_SLOTS]
    payload["cap"]["slot_names"] = list(CAP_SLOTS)
    return payload


# --------------------------------------------------------------------------
# interactive window
# --------------------------------------------------------------------------
def to_array(rows: list[list[Any]], dtype: Any = np.float64) -> np.ndarray:
    """``null`` back to NaN so matplotlib draws a gap instead of a spike."""
    if not rows:
        return np.zeros((0, 0), dtype=dtype)
    return np.array(
        [[np.nan if v is None else v for v in row] for row in rows], dtype=dtype
    )


def run_window(payload: dict[str, Any], root: Path,
               snapshot: Path | None = None) -> int:
    """matplotlib window: three traces, a numeric slot table and a scrubber.

    ``snapshot`` renders the first frame to PNG with the Agg backend instead of
    opening a window — handy for reports and for machines without a display.
    """
    import time as _time

    try:
        import matplotlib

        matplotlib.use("Agg" if snapshot else "TkAgg", force=True)
        import matplotlib.pyplot as plt
        from matplotlib.gridspec import GridSpec
        from matplotlib.widgets import Button, Slider
    except Exception as exc:  # pragma: no cover - depends on the host
        print(
            f"[错误] 打不开窗口（{exc}）。这台机器缺 GUI 后端时请用 "
            "--html 生成浏览器版本",
            file=sys.stderr,
        )
        return 3

    plt.rcParams["font.sans-serif"] = [
        "Noto Sans CJK JP",
        "Noto Sans CJK SC",
        "WenQuanYi Micro Hei",
        "DejaVu Sans",
    ]
    plt.rcParams["font.monospace"] = [
        "Noto Sans Mono CJK SC",
        "Noto Sans Mono CJK JP",
        "DejaVu Sans Mono",
    ]
    plt.rcParams["axes.unicode_minus"] = False
    plt.style.use("dark_background")

    t = np.asarray(payload["cap"]["t"], dtype=np.float64)
    angles = to_array(payload["cap"]["angles"])
    force = to_array(payload["cap"]["force"])
    frames = payload["cap"]["frames"]
    angle_labels = payload["cap"]["angle_labels"]
    slot_labels = payload["cap"]["slot_labels"]
    n = len(t)
    if n == 0:
        print("[错误] 没有可画的数据", file=sys.stderr)
        return 2

    fig = plt.figure(figsize=(15.5, 8.8))
    fig.canvas.manager.set_window_title(f"PXCap III · {root.name}")
    gs = GridSpec(
        3, 2,
        width_ratios=[2.05, 1.0],
        height_ratios=[1.0, 1.0, 1.0],
        left=0.055, right=0.985, top=0.93, bottom=0.11,
        hspace=0.42, wspace=0.14,
    )
    ax_angle = fig.add_subplot(gs[0, 0])
    ax_force_a = fig.add_subplot(gs[1, 0], sharex=ax_angle)
    ax_force_b = fig.add_subplot(gs[2, 0], sharex=ax_angle)
    ax_text = fig.add_subplot(gs[:, 1])
    ax_text.axis("off")
    ax_text.set_title("10 路合力（N）", fontsize=8, color="#8792a6", loc="left")

    range_s = float(t[-1] - t[0]) if n > 1 else 0.0
    clock_note = payload["meta"]["clock"]
    quality = payload["meta"].get("clock_quality")
    if quality:
        clock_note += f" · 漂移 {quality['drift_ms']:+.1f} ms"
        if quality["steps_over_10ms"]:
            clock_note += f" · 跳变 {quality['steps_over_10ms']} 次"
    fig.suptitle(
        f"{root.name}    {payload['meta']['rows']} 帧 / {range_s:.2f} s / "
        f"{payload['meta']['rate_hz']:.1f} Hz    时钟 {clock_note}",
        fontsize=10, color="#c9d3e3", y=0.975,
    )

    notes = list(payload.get("warnings") or [])
    if notes:
        fig.text(0.055, 0.948, "⚠ " + "；".join(notes[:2]), fontsize=8,
                 color="#f0c274", va="top")

    angle_colors = ["#4da3ff", "#38d39f", "#f5a524", "#f26d6d"]
    for k in range(angles.shape[1] if angles.size else 0):
        ax_angle.plot(
            t, angles[:, k], lw=1.2,
            color=angle_colors[k % len(angle_colors)],
            label=angle_labels[k] if k < len(angle_labels) else f"#{k}",
        )
    ax_angle.set_ylabel("关节角 (deg)", fontsize=9)
    ax_angle.legend(
        loc="upper right", fontsize=8, ncols=2, frameon=True, framealpha=0.85,
        facecolor="#10141c", edgecolor="none", borderpad=0.4,
        handlelength=1.4, columnspacing=1.0,
    )
    ax_angle.tick_params(labelbottom=False, labelsize=8)
    ax_angle.grid(alpha=0.15, lw=0.6)

    ramp = plt.get_cmap("turbo")
    slot_colors = [ramp(i / max(len(slot_labels) - 1, 1)) for i in range(len(slot_labels))]

    # 10 路挤一张图里，5 条已经要贴着画了；拆成 1~5 / 6~10 两张，图例也放得开。
    def plot_force_half(ax: Any, first: int, last: int, ylabel: str,
                        bottom: bool) -> None:
        for k in range(first, min(last, force.shape[1]) if force.size else first):
            # 文件里是 0.1 N 的整数，曲线按 N 画：坐标轴写 0.2 比写 2 直观，
            # 也不影响精度（0.1 N 换到 N 就是一位小数）。
            ax.plot(t, force[:, k] / 10.0, lw=1.0, color=slot_colors[k],
                    label=slot_labels[k] if k < len(slot_labels) else f"#{k}")
        ax.set_ylabel(ylabel, fontsize=8.5)
        ax.legend(
            loc="upper right", fontsize=7.5, ncols=5, frameon=True, framealpha=0.85,
            facecolor="#10141c", edgecolor="none", borderpad=0.35,
            handlelength=1.2, columnspacing=0.9,
        )
        ax.tick_params(labelbottom=bottom, labelsize=8)
        ax.grid(alpha=0.15, lw=0.6)
        if bottom:
            ax.set_xlabel("相对首帧时间 (s)", fontsize=9)

    plot_force_half(ax_force_a, 0, 5, "合力模长 1~5（N）", bottom=False)
    plot_force_half(ax_force_b, 5, 10, "合力模长 6~10（N）", bottom=True)

    cursors = [ax.axvline(t[0], color="#8fc2ff", lw=1.0, alpha=0.85)
               for ax in (ax_angle, ax_force_a, ax_force_b)]

    slot_f = None
    slot_times: np.ndarray | None = None
    slot_counts: np.ndarray | None = None
    if payload["slot_force"]:
        raw = base64.b64decode(payload["slot_force"]["data"])
        slot_f = np.frombuffer(raw, dtype="<i2").reshape(
            payload["slot_force"]["frames"],
            payload["slot_force"]["slots"],
            3,
        ).astype(np.float64)
        slot_times = np.asarray(payload["slot_force"]["t"], dtype=np.float64)
        slot_counts = np.asarray(payload["slot_force"]["counts"], dtype=np.float64)
    else:
        ax_text.text(
            0.5, 0.5, "本次采集没有合力数据",
            ha="center", va="center", fontsize=10, color="#7b879c",
            transform=ax_text.transAxes,
        )

    # 纯数字表格，和磁编码器那几路一样只给数值：不做热力图，也不做颜色映射。
    # 文件里存 0.1 N 的整数，这里除以 10 显示 N 两位小数：等值无损，且与上位机
    # 同一口径；整数显示会把 |F| 的 1.73 舍成 2。
    columns = [
        (0.00, "槽位", "left"),
        (0.40, "Fx", "right"),
        (0.58, "Fy", "right"),
        (0.76, "Fz", "right"),
        (0.90, "|F|", "right"),
        (1.00, "点数", "right"),
    ]
    for fx, label, align in columns:
        ax_text.text(fx, 0.965, label, fontsize=8, color="#6f7c92", ha=align,
                     transform=ax_text.transAxes)

    row_y = [0.895 - row * 0.072 for row in range(len(slot_labels))]
    readout_cells = []
    for row in range(len(slot_labels)):
        ax_text.text(0.00, row_y[row], slot_labels[row], fontsize=8.2,
                     va="center", color="#aab6c8", transform=ax_text.transAxes)
        readout_cells.append([
            ax_text.text(fx, row_y[row], "—", fontsize=8.2, va="center",
                         ha=align, color="#e7ecf5" if fx < 0.99 else "#7c8798",
                         transform=ax_text.transAxes)
            for fx, _, align in columns[1:]
        ])

    total_y = row_y[-1] - 0.062
    ax_text.plot([0.0, 1.0], [total_y + 0.036, total_y + 0.036],
                 color="#2a3444", lw=0.8, transform=ax_text.transAxes,
                 clip_on=False)
    ax_text.text(0.00, total_y, "合计", fontsize=8.2, va="center",
                 color="#aab6c8", transform=ax_text.transAxes)
    total_cells = [
        ax_text.text(fx, total_y, "—", fontsize=8.2, va="center", ha=align,
                     color="#f0c274" if fx < 0.99 else "#7c8798",
                     transform=ax_text.transAxes)
        for fx, _, align in columns[1:]
    ]
    angle_text = ax_text.text(0.00, total_y - 0.075, "", fontsize=7.8,
                              color="#8792a6", transform=ax_text.transAxes)
    quest = payload.get("quest")
    quest_aligned = (
        bool(quest) and quest.get("alignment") in QUEST_ALIGNMENT_ACTIVE
    )
    quest_host = bool(quest) and quest.get("alignment") == QUEST_ALIGNMENT_HOST
    quest_verified = bool(quest) and quest.get("alignment") == QUEST_ALIGNMENT_SDK_UNIX
    quest_valid_times: np.ndarray | None = None
    quest_valid_rows: np.ndarray | None = None
    if quest_aligned and quest:
        quest_valid_rows = np.asarray(
            [
                i
                for i, value in enumerate(quest["time"])
                if value is not None and np.isfinite(value)
            ],
            dtype=np.int64,
        )
        quest_valid_times = np.asarray(
            [quest["time"][int(i)] for i in quest_valid_rows],
            dtype=np.float64,
        )
    quest_text = ax_text.text(
        0.00,
        max(0.015, total_y - 0.125),
        (
            (
                "Quest：已按主机接收时刻对齐（无需 C8）"
                if quest_host
                else "Quest：已按 C8 验证的 Unix 时钟对齐"
                if quest_verified
                else "Quest：已按 Unix 墙钟对齐（未经 C8 验证）"
                if quest_aligned
                else "Quest：独立 SDK 时间轴，未做跨源时间对齐"
            )
            if quest
            else "Quest：本次没有可显示数据"
        ),
        fontsize=7.6,
        color="#6de0ae" if quest_aligned else "#f0c274" if quest else "#7b879c",
        transform=ax_text.transAxes,
    )

    ax_slider = fig.add_axes([0.075, 0.042, 0.60, 0.022])
    slider = Slider(
        ax_slider, "", 0, n - 1, valinit=0, valstep=1, color="#4da3ff",
    )
    slider.valtext.set_fontsize(8)
    ax_play = fig.add_axes([0.705, 0.028, 0.075, 0.055])
    btn_play = Button(ax_play, "▶ 播放", color="#1b2330", hovercolor="#24405e")
    btn_play.label.set_fontsize(9)
    ax_speed = fig.add_axes([0.795, 0.028, 0.065, 0.055])
    btn_speed = Button(ax_speed, "1.0×", color="#1b2330", hovercolor="#24405e")
    btn_speed.label.set_fontsize(9)

    state = {"i": 0, "playing": False, "last": 0.0, "speed": 1.0, "busy": False}

    def slot_index(seconds: float) -> int:
        assert slot_times is not None
        pos = int(np.searchsorted(slot_times, seconds))
        return max(0, min(len(slot_times) - 1, pos))

    def redraw(index: int, *, sync_slider: bool = True) -> None:
        index = max(0, min(n - 1, index))
        state["i"] = index
        x = t[index]
        for cursor in cursors:
            cursor.set_xdata([x, x])
        angle_row = angles[index] if angles.size else []
        if slot_f is not None:
            k = slot_index(x)
            row = slot_f[k]
            count_row = slot_counts[k] if slot_counts is not None else None
            for r, cells in enumerate(readout_cells):
                fx, fy, fz = (float(value) / 10.0 for value in row[r])
                cells[0].set_text(f"{fx:.2f}")
                cells[1].set_text(f"{fy:.2f}")
                cells[2].set_text(f"{fz:.2f}")
                cells[3].set_text(f"{np.linalg.norm(row[r]) / 10.0:.2f}")
                cells[4].set_text(
                    "—" if count_row is None else f"{int(count_row[r])}"
                )
            total = row.sum(axis=0) / 10.0
            total_cells[0].set_text(f"{total[0]:.2f}")
            total_cells[1].set_text(f"{total[1]:.2f}")
            total_cells[2].set_text(f"{total[2]:.2f}")
            total_cells[3].set_text(f"{np.linalg.norm(total):.2f}")
            total_cells[4].set_text(
                "—" if count_row is None else f"{int(count_row.sum())}"
            )
        angle_text.set_text(
            "关节角 " + " / ".join(
                "—" if not np.isfinite(v) else f"{v:.2f}" for v in angle_row
            ) + " deg"
        )
        if quest:
            qn = int(quest["shown"])
            if (
                quest_aligned
                and quest_valid_times is not None
                and quest_valid_rows is not None
                and quest_valid_times.size
            ):
                target = x - t[0]
                pos = int(np.searchsorted(
                    quest_valid_times, target, side="right"
                )) - 1
                pos = max(0, min(int(quest_valid_rows.size) - 1, pos))
                qi = int(quest_valid_rows[pos])
            else:
                qi = round(index / max(1, n - 1) * max(0, qn - 1))
                qi = max(0, min(qn - 1, qi))
            sequence = quest["sequence"][qi]
            time_value = quest["time"][qi]
            if time_value is None:
                time_text = f"seq #{sequence}"
            elif quest.get("time_unit") == "s":
                time_text = f"{time_value:.3f} s"
            elif quest["time_source"] == "sdk_timestamp_ns":
                time_text = f"{time_value:.3f} ms"
            else:
                time_text = f"seq #{sequence}"
            xyz = (
                quest.get("main_pose", {}).get("xyz", [])
                if quest.get("main_pose")
                else []
            )
            row_xyz = xyz[qi] if qi < len(xyz) else []
            if row_xyz and all(v is not None for v in row_xyz):
                pose_text = "xyz " + " / ".join(f"{v:.3f}" for v in row_xyz)
            else:
                pose_text = "xyz —"
            tracked = quest["tracked"][qi] if qi < len(quest["tracked"]) else False
            alignment_text = {
                "host": "主机接收对齐",
                "sdk_unix": "Unix 对齐 (C8)",
                "unix": "Unix 对齐（未验证）",
            }.get(quest.get("alignment"), "未跨源对齐")
            quest_text.set_text(
                f"Quest {qi + 1}/{qn} · {time_text} · {pose_text} · "
                f"{'tracked' if tracked else 'untracked'} · {alignment_text}"
            )
        ax_text.set_title(
            f"当前帧 {frames[index]}   ·   {x - t[0]:.2f} s",
            fontsize=8, color="#8792a6", loc="left",
        )
        if sync_slider and not state["busy"]:
            state["busy"] = True
            slider.set_val(index)
            state["busy"] = False
        fig.canvas.draw_idle()

    def toggle_play(*_args: Any) -> None:
        state["playing"] = not state["playing"]
        btn_play.label.set_text("❚❚ 暂停" if state["playing"] else "▶ 播放")
        if state["playing"]:
            if state["i"] >= n - 1:
                redraw(0)
            state["last"] = _time.perf_counter()
            timer.start()
        else:
            timer.stop()
        fig.canvas.draw_idle()

    def on_timer() -> None:
        if not state["playing"]:
            timer.stop()
            return
        now = _time.perf_counter()
        delta = now - state["last"]
        state["last"] = now
        target = t[state["i"]] + delta * state["speed"]
        nxt = int(np.searchsorted(t, target))
        if nxt >= n - 1:
            redraw(n - 1)
            toggle_play()
            return
        redraw(nxt)

    timer = fig.canvas.new_timer(interval=40)
    timer.add_callback(on_timer)

    def on_slider(value: float) -> None:
        if state["busy"]:
            return
        redraw(int(value))

    def on_speed(*_args: Any) -> None:
        cycle = [1.0, 2.0, 0.5, 0.25]
        state["speed"] = cycle[(cycle.index(state["speed"]) + 1) % len(cycle)]
        btn_speed.label.set_text(f"{state['speed']:g}×")
        fig.canvas.draw_idle()

    def on_key(event: Any) -> None:
        if event.key == " ":
            toggle_play()
        elif event.key == "right":
            toggle_play() if state["playing"] else None
            redraw(state["i"] + 1)
        elif event.key == "left":
            toggle_play() if state["playing"] else None
            redraw(state["i"] - 1)
        elif event.key == "home":
            redraw(0)
        elif event.key == "end":
            redraw(n - 1)

    def on_click(event: Any) -> None:
        if event.inaxes in (ax_angle, ax_force_a, ax_force_b) and event.xdata is not None:
            redraw(int(np.searchsorted(t, event.xdata)))

    slider.on_changed(on_slider)
    btn_play.on_clicked(toggle_play)
    btn_speed.on_clicked(on_speed)
    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("button_press_event", on_click)

    redraw(0)
    if snapshot is not None:
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(snapshot, dpi=150, facecolor=fig.get_facecolor())
        plt.close(fig)
        print(f"写入 {windows_path(snapshot)}")
        return 0
    print("窗口已打开：空格播放/暂停，←/→ 逐帧，点击曲线跳转，滑块可拖动")
    plt.show()
    return 0


# --------------------------------------------------------------------------
# html export
# --------------------------------------------------------------------------
def windows_path(path: Path) -> str:
    """Translate ``/mnt/c/...`` to ``C:\\...`` so the hint is copy-pasteable."""
    parts = path.resolve().parts
    if len(parts) > 3 and parts[1] == "mnt" and len(parts[2]) == 1:
        drive = parts[2].upper()
        return drive + ":\\" + "\\".join(parts[3:])
    return str(path)


def write_viewer(payload: dict[str, Any], out_path: Path) -> None:
    blob = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    html = HTML_TEMPLATE.replace("__PAYLOAD__", blob.replace("</", "<\\/"))
    out_path.write_text(html, encoding="utf-8")


HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PXCap III 采集回放</title>
<style>
  :root {
    --bg: #0c0f15;
    --panel: #141922;
    --line: #222a37;
    --text: #e7ecf5;
    --muted: #8792a6;
    --accent: #4da3ff;
  }
  * { box-sizing: border-box; }
  /* The page is allowed to grow and scroll: a 1366x768 laptop cannot fit four
     panels plus video at a readable size, and clipping them behind
     overflow:hidden is worse than a scrollbar. */
  html { height: 100%; }
  body {
    margin: 0;
    min-height: 100%;
    background: var(--bg);
    color: var(--text);
    font: 13px/1.5 "Segoe UI", "Microsoft YaHei", system-ui, sans-serif;
    display: flex;
    flex-direction: column;
  }
  header {
    padding: 12px 18px 10px;
    border-bottom: 1px solid var(--line);
    display: flex;
    align-items: baseline;
    gap: 18px;
    flex-wrap: wrap;
  }
  header h1 { font-size: 15px; margin: 0; font-weight: 600; }
  header .facts { color: var(--muted); display: flex; gap: 16px; flex-wrap: wrap; }
  header .facts b { color: var(--text); font-weight: 500; }
  .warn {
    margin: 8px 18px 0;
    padding: 7px 12px;
    border: 1px solid #6b4a13;
    background: #241a09;
    color: #f0c274;
    border-radius: 6px;
    font-size: 12px;
  }
  main {
    /* 1 0 auto: fills the window when there is room, grows past it (and thus
       scrolls) when there is not.  The extra bottom padding keeps the sticky
       control bar from covering the last row when scrolled to the end. */
    flex: 1 0 auto;
    display: flex;
    flex-direction: column;
    gap: 10px;
    padding: 10px 18px 70px;
  }
  #grid {
    display: grid;
    gap: 10px;
    grid-template-columns: minmax(0, 1.75fr) minmax(300px, 1fr);
    /* Left column: three curve rows (joint angles, then the 10 force slots split
       5 + 5 so no plot carries ten overlapping lines).  Right column: the
       current-frame card, then the force table spanning the two rows below it —
       eleven rows of numbers fill that height exactly, and the page stays on one
       screen instead of pushing the table under the fold. */
    grid-template-rows: minmax(165px, 1fr) minmax(165px, 1fr) minmax(165px, 1fr);
    min-height: max(520px, calc(100vh - 108px));
  }
  #grid .card.wide { grid-column: 1 / -1; }
  #grid .card.side { grid-column: 2; grid-row: 2 / span 2; }
  /* Figures size themselves to the video instead of stretching to half the row:
     a 16:9 frame inside a 700 px box would otherwise be two thirds empty band. */
  #videoStrip {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
    gap: 12px;
    align-items: start;
    justify-items: center;
  }
  #videoStrip figure {
    margin: 0;
    min-width: 0;
    max-width: 900px;
    width: 100%;
    display: flex;
    flex-direction: column;
    gap: 5px;
    background: transparent;
  }
  #videoStrip video {
    width: 100%;
    height: auto;
    object-fit: contain;
    display: block;
    background: #05080c;
    border: 1px solid var(--line);
    border-radius: 8px;
  }
  #videoStrip figcaption {
    font-size: 11px;
    line-height: 1.25;
    color: #8b95a6;
    text-align: center;
    padding: 0 2px;
    overflow: hidden;
    text-overflow: ellipsis;
  }
  #videoStrip .empty {
    display: flex;
    align-items: center;
    justify-content: center;
    min-height: 120px;
    color: #5d6879;
    font-size: 12px;
    border: 1px dashed var(--line);
    border-radius: 8px;
  }
  .card {
    background: var(--panel);
    border: 1px solid var(--line);
    border-radius: 8px;
    padding: 8px 10px 6px;
    display: flex;
    flex-direction: column;
    min-height: 0;
    min-width: 0;
    overflow: hidden;
  }
  .card h2 {
    margin: 0 0 4px;
    font-size: 11px;
    font-weight: 600;
    letter-spacing: .06em;
    color: var(--muted);
    text-transform: uppercase;
    display: flex;
    justify-content: space-between;
    gap: 10px;
  }
  .card h2 span { font-weight: 400; letter-spacing: 0; text-transform: none; }
  .plot { position: relative; flex: 1; min-height: 0; }
  .plot canvas { position: absolute; inset: 0; width: 100%; height: 100%; }
  .legend { display: flex; flex-wrap: wrap; gap: 4px 10px; font-size: 11px; color: var(--muted); margin-top: 4px; }
  .legend i { display: inline-block; width: 9px; height: 3px; border-radius: 2px; margin-right: 4px; vertical-align: middle; }
  /* 10 slots in one row wrap into a ragged pair of lines; a fixed grid keeps
     the colour chips aligned so the eye can match a curve to its slot. */
  #legendForce { display: grid; grid-template-columns: repeat(5, minmax(0, 1fr)); gap: 2px 8px; }
  #legendForce span { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  .tip {
    position: absolute;
    display: none;
    z-index: 6;
    padding: 5px 8px;
    border: 1px solid var(--line);
    border-radius: 6px;
    background: rgba(10, 14, 20, .94);
    font-size: 11px;
    line-height: 1.45;
    color: var(--text);
    pointer-events: none;
    white-space: nowrap;
    box-shadow: 0 4px 14px rgba(0, 0, 0, .45);
  }
  .tip b { color: var(--muted); font-weight: 500; }
  .tip i { display: inline-block; width: 8px; height: 8px; border-radius: 2px; margin-right: 5px; vertical-align: middle; }
  .plot canvas.hover { cursor: crosshair; }
  /* The readout used to list 10 slots, so its natural height (~175 px) decided
     the whole row height and pushed the table under the control bar on a
     700 px tall window.  Tighter type plus an internal scroll keeps the row
     inside its track. */
  #readout { font-size: 9.5px; line-height: 1.3; min-height: 0; overflow: auto; }
  /* Fixed layout: with auto layout a 4-digit force value widened the column
     and pushed the point count out of the card. */
  #readout table { width: 100%; border-collapse: collapse; table-layout: fixed; }
  #readout td, #readout th { padding: 0 3px; text-align: right; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  #readout th { color: var(--muted); font-weight: 500; font-size: 8.5px; }
  #readout td:first-child, #readout th:first-child { text-align: left; color: var(--muted); width: 52%; }
  #readout .angles { color: var(--muted); font-size: 9px; margin-top: 2px; }
  #readout .total { color: #f0c274; font-size: 10px; margin-top: 8px; line-height: 1.5; }
  #questCard {
    border-color: #32435a;
    background: linear-gradient(180deg, #151b25 0%, #111721 100%);
    padding-left: 0;
    padding-right: 0;
    border-left: 0;
    border-right: 0;
  }
  #questCard h2 { color: #9fc4ef; margin-left: 10px; margin-right: 10px; }
  .quest-note {
    padding: 8px 12px;
    margin: 0 10px 8px;
    border: 1px solid #76551b;
    border-radius: 6px;
    background: #281d0a;
    color: #f2c46e;
    font-weight: 600;
    font-size: 12px;
  }
  .quest-note.aligned {
    border-color: #226a52;
    background: #0c211a;
    color: #6de0ae;
  }
  .quest-grid {
    display: grid;
    grid-template-columns: minmax(0, 1.75fr) minmax(300px, 1fr);
    gap: 10px;
    min-height: 430px;
  }
  .quest-column {
    display: grid;
    gap: 10px;
    min-width: 0;
  }
  .quest-panel {
    min-width: 0;
    min-height: 205px;
    display: flex;
    flex-direction: column;
    padding: 8px;
    border: 1px solid var(--line);
    border-radius: 7px;
    background: #0e131b;
  }
  .quest-plot-panel {
    padding: 8px 10px 6px;
    border: 1px solid var(--line);
  }
  .quest-panel h3 {
    margin: 0 0 5px;
    font-size: 11px;
    color: var(--muted);
    font-weight: 600;
  }
  .quest-series-legend {
    display: flex;
    gap: 10px;
    flex-wrap: wrap;
    font-size: 10px;
    color: var(--muted);
    margin: 2px 0 3px;
  }
  .quest-series-legend i {
    display: inline-block;
    width: 8px;
    height: 8px;
    border-radius: 2px;
    margin-right: 4px;
  }
  #questReadout { font-size: 10.5px; line-height: 1.45; overflow: auto; }
  #questReadout h4 {
    margin: 7px 0 2px;
    font-size: 10px;
    color: #9fc4ef;
    font-weight: 600;
  }
  #questReadout table { width: 100%; border-collapse: collapse; table-layout: fixed; }
  #questReadout td { padding: 1px 3px; white-space: nowrap; }
  #questReadout td:first-child { color: var(--muted); width: 42%; }
  #questReadout td:last-child { text-align: right; font-variant-numeric: tabular-nums; }
  #questReadout .bad { color: #f0c274; }
  footer {
    position: sticky;
    bottom: 0;
    z-index: 6;
    background: rgba(12, 15, 21, .95);
    backdrop-filter: blur(6px);
    border-top: 1px solid var(--line);
    padding: 10px 18px;
    display: flex;
    align-items: center;
    gap: 14px;
  }
  button {
    background: #1b2330;
    color: var(--text);
    border: 1px solid var(--line);
    border-radius: 6px;
    padding: 6px 14px;
    font: inherit;
    cursor: pointer;
  }
  button:hover { border-color: var(--accent); }
  button.on { background: #12324f; border-color: var(--accent); }
  input[type=range] { flex: 1; accent-color: var(--accent); }
  #clock { font-variant-numeric: tabular-nums; color: var(--muted); min-width: 190px; text-align: right; }
  .hint { color: #5d6879; font-size: 11px; }
  /* 10 路合力：纯数字表格，不做颜色映射。六列固定宽度，数值右对齐，
     这样 4 位数的 Fz 也不会把后面的 |F| 挤出去。 */
  #readout table.mini { width: 100%; border-collapse: collapse; table-layout: fixed; }
  #readout table.mini th, #readout table.mini td {
    padding: 1px 4px; text-align: right; font-size: 10.5px;
    font-variant-numeric: tabular-nums;
  }
  #readout table.mini th { color: var(--muted); font-weight: 500; font-size: 10px; }
  #readout table.mini td:first-child, #readout table.mini th:first-child {
    text-align: left; color: var(--muted); width: 58%;
  }
  #slotTable { flex: 1 1 auto; min-height: 0; overflow: auto; }
  #slotTable table {
    /* The card is ~470 px wide and two grid rows tall, so the six columns get
       26% + five 14.8% tracks, and height:100% spreads the eleven rows over the
       whole panel instead of leaving a dead band under the table. */
    width: 100%; height: 100%; margin: 0 auto;
    border-collapse: collapse; table-layout: fixed;
  }
  #slotTable th, #slotTable td {
    padding: 1px 6px; text-align: right; white-space: nowrap;
    font-variant-numeric: tabular-nums; font-size: 13px;
  }
  #slotTable th {
    color: var(--muted); font-weight: 500; font-size: 10.5px;
    border-bottom: 1px solid var(--line); padding-bottom: 3px;
  }
  #slotTable th:first-child, #slotTable td:first-child {
    text-align: left; color: var(--muted); width: 26%;
  }
  #slotTable tr.total td {
    border-top: 1px solid var(--line); color: #f0c274; padding-top: 3px;
  }
  /* 分布力空间散点图：点位形状来自 Studio 测点模型，贴片位置来自 PXCap III
     参考图的 10 块贴片边界框；右侧放色标、当前帧读数与各槽位峰值。 */
  #distCard { min-height: 920px; }
  #distWrap {
    display: flex; gap: 14px; flex: 1 1 auto; min-height: 820px;
    justify-content: center;
  }
  #distPlot { flex: 0 1 640px; min-width: 0; }
  #distSide {
    flex: 0 0 250px; width: 250px; min-height: 0; padding-right: 4px;
    display: flex; flex-direction: column; gap: 10px;
    font-size: 10px; color: var(--muted); overflow: hidden;
  }
  #distCard h2 span {
    font-size: 10px; padding-right: 6px; max-width: 120px;
    overflow: hidden; text-overflow: ellipsis; white-space: nowrap;
  }
  #distLegend .bar {
    height: 10px; border-radius: 3px;
    background: linear-gradient(90deg,#101722,#1e4b7a,#2aa7a1,#f2c14e,#e4572e);
  }
  #distLegend .row { display: flex; justify-content: space-between; gap: 6px; margin-top: 3px; }
  #distReadout { line-height: 1.55; overflow: auto; min-height: 0; }
  #distReadout b { color: var(--text); font-weight: 500; }
  #distSlots {
    display: grid; grid-template-columns: minmax(0, 1fr) auto;
    gap: 1px 8px; font-size: 9.5px; overflow: auto; min-height: 0;
  }
  #distSlots span { overflow: hidden; text-overflow: ellipsis; white-space: nowrap; }
  #distSlots b { color: #5d6879; font-weight: 500; text-align: right; font-variant-numeric: tabular-nums; }
  #distArrowToggleWrap { cursor: pointer; }
  #distArrowToggle { accent-color: var(--accent); margin-right: 4px; }
  #distHint { line-height: 1.4; }
  #distTip { max-width: 230px; }
  @media (max-width: 900px) {
    #distCard { min-height: 0; }
    #distWrap { flex-direction: column; min-height: 0; }
    #distSide { flex: 0 0 auto; width: auto; }
    #distPlot { flex: none; width: 100%; height: 520px; }
  }
</style>
</head>
<body>
<header>
  <h1 id="title">PXCap III</h1>
  <div class="facts" id="facts"></div>
</header>
<div id="warnBox"></div>
<main>
<div id="grid">
  <section class="card">
    <h2>关节角 <span>deg</span></h2>
    <div class="plot" id="plotAngles"><canvas class="base"></canvas><canvas class="over"></canvas><canvas class="hover"></canvas><div class="tip"></div></div>
    <div class="legend" id="legendAngles"></div>
  </section>

  <section class="card">
    <h2>当前帧</h2>
    <div id="readout"></div>
  </section>

  <section class="card">
    <h2>合力模长 A <span>1~5 路 · N</span></h2>
    <div class="plot" id="plotForceA"><canvas class="base"></canvas><canvas class="over"></canvas><canvas class="hover"></canvas><div class="tip"></div></div>
    <div class="legend" id="legendForceA"></div>
  </section>

  <section class="card">
    <h2>合力模长 B <span>6~10 路 · N</span></h2>
    <div class="plot" id="plotForceB"><canvas class="base"></canvas><canvas class="over"></canvas><canvas class="hover"></canvas><div class="tip"></div></div>
    <div class="legend" id="legendForceB"></div>
  </section>

  <section class="card side">
    <h2>10 路合力 <span id="slotMeta">N</span></h2>
    <div id="slotTable"></div>
  </section>
</div>
<section class="card" id="distCard">
  <h2>分布力散点图 <span id="distMeta">N</span></h2>
  <div id="distWrap">
    <div class="plot" id="distPlot">
      <canvas id="distCanvas"></canvas>
      <div class="tip" id="distTip"></div>
    </div>
    <div id="distSide">
      <div id="distLegend"></div>
      <label class="hint" id="distArrowToggleWrap">
        <input type="checkbox" id="distArrowToggle" checked> 切向箭头（Fx/Fy）
      </label>
      <div id="distReadout"></div>
      <div class="hint">各槽位峰值 N</div>
      <div id="distSlots"></div>
      <div class="hint" id="distHint">贴片布局按 PXCap III 参考图；点位形状来自 Studio 测点模型；箭头长度按切向模长、方向为切向合力，非标定。</div>
    </div>
  </div>
</section>
<section class="card" id="questCard" style="display:none">
  <h2>Quest 3 <span id="questMeta"></span></h2>
  <div class="quest-note" id="questAlignNote">Quest 使用独立 SDK 时间轴，未做跨源时间对齐</div>
  <div class="quest-grid">
    <div class="quest-column">
      <div class="quest-panel quest-plot-panel">
        <h3 id="questPosTitle">手柄位置 XYZ · m</h3>
        <div class="quest-series-legend">
          <span><i style="background:#ff7b72"></i>X</span>
          <span><i style="background:#54d49f"></i>Y</span>
          <span><i style="background:#6aa8ff"></i>Z</span>
        </div>
        <div class="plot" id="plotQuestPos">
          <canvas class="base"></canvas><canvas class="over"></canvas>
        </div>
      </div>
      <div class="quest-panel quest-plot-panel">
        <h3 id="questQuatTitle">手柄姿态四元数 xyzw</h3>
        <div class="quest-series-legend">
          <span><i style="background:#f5a524"></i>qx</span>
          <span><i style="background:#38d39f"></i>qy</span>
          <span><i style="background:#4da3ff"></i>qz</span>
          <span><i style="background:#f26d6d"></i>qw</span>
        </div>
        <div class="plot" id="plotQuestQuat">
          <canvas class="base"></canvas><canvas class="over"></canvas>
        </div>
      </div>
    </div>
    <div class="quest-column">
      <div class="quest-panel">
        <h3 id="quest3DTitle">手柄位姿 3D 轨迹（X / Y / Z）</h3>
        <div class="plot" id="plotQuest3D">
          <canvas class="base"></canvas><canvas class="over"></canvas>
        </div>
      </div>
      <div class="quest-panel">
        <h3>当前 Quest 样本</h3>
        <div id="questReadout"></div>
      </div>
    </div>
  </div>
</section>
<div id="videoStrip"></div>
</main>
<footer>
  <button id="playBtn">▶ 播放</button>
  <button id="speedBtn">1.0×</button>
  <input type="range" id="seek" min="0" max="1" value="0" step="1">
  <span id="clock"></span>
</footer>

<script>
const DATA = __PAYLOAD__;
const CAP = DATA.cap, SLOT = DATA.slot_force, META = DATA.meta;
const QUEST = DATA.quest;
const DIST = (SLOT && SLOT.dist) || null;
let DIST_DATA = null;
const DIST_HIT = [];
let DIST_SLOT_CELLS = [];
let DIST_TANGENT = null;
let DIST_ARROWS_ON = false;
const DIST_LABELS = [
  '拇指1', '拇指2', '食指0', '食指1', '中指0',
  '中指1', '中指2', '中指3', '掌心0', '掌心1',
];
// Real-hardware recheck after the per-slot point flips: raw Fx/Fy now match
// the 2D pad map, so no extra 180° sign correction is applied.
const TANGENT_SIGN = 1;
// Colour mapping exponent: sqrt lifts weak contacts on the fixed 0–25 N
// scale; point radius still follows the linear ratio.
const DIST_COLOR_GAMMA = 0.5;
let questNoticeShown = false;
const N = CAP.t.length;
const T0 = CAP.t[0] || 0;
const T1 = CAP.t[N - 1] || 0;
const QN = QUEST && QUEST.time ? QUEST.time.length : 0;

const state = { i: 0, playing: false, speed: 1 };

function el(id) { return document.getElementById(id); }

// ---------- data helpers ----------
function lowerBound(arr, value) {
  let lo = 0, hi = arr.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (arr[mid] < value) lo = mid + 1; else hi = mid;
  }
  return lo;
}

function fmtTime(sec) {
  const sign = sec < 0 ? '-' : '';
  const s = Math.abs(sec);
  const m = Math.floor(s / 60);
  const r = s - m * 60;
  return sign + m + ':' + (r < 10 ? '0' : '') + r.toFixed(2);
}

function rampColor(i, n) {
  const h = 205 - 185 * (n <= 1 ? 0 : i / (n - 1));
  return 'hsl(' + h.toFixed(0) + ' 72% 62%)';
}

// ---------- canvas ----------
function fit(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 2 || rect.height < 2) return null;
  const w = Math.round(rect.width * dpr), h = Math.round(rect.height * dpr);
  // Only reallocate when the size really changed: resetting width/height on
  // every playback frame is what makes this kind of view stutter.
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);
  return { ctx, w: rect.width, h: rect.height };
}

function niceScale(lo, hi) {
  const span = hi - lo;
  if (!(span > 0)) return { lo, hi, step: 1 };
  const rough = span / 4;
  const mag = Math.pow(10, Math.floor(Math.log10(rough)));
  const norm = rough / mag;
  const steps = [1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10];
  let pick = steps[steps.length - 1];
  for (const candidate of steps) { if (norm <= candidate) { pick = candidate; break; } }
  const step = pick * mag;
  return { lo: Math.floor(lo / step) * step, hi: Math.ceil(hi / step) * step, step };
}

const charts = {};

function drawLineChart(key, plotId, series, options) {
  const plot = el(plotId);
  const base = plot.querySelector('.base');
  const fitRes = fit(base);
  if (!fitRes) return;
  const { ctx, w, h } = fitRes;
  const pad = { l: 50, r: 10, t: 8, b: 18 };

  let lo = Infinity, hi = -Infinity;
  for (const s of series) for (const v of s.values) {
    if (v === null || !isFinite(v)) continue;
    if (v < lo) lo = v; if (v > hi) hi = v;
  }
  if (!isFinite(lo)) { lo = 0; hi = 1; }
  if (hi - lo < 1e-6) hi = lo + 1;
  let scale;
  if (options.zeroBase) {
    lo = 0;
    // 8% headroom before rounding: without it a series whose maximum lands on
    // a tick value (e.g. all slots at 45 points) draws along the top border.
    scale = niceScale(0, hi * 1.08);
    hi = scale.hi;
  } else {
    const pad = (hi - lo) * 0.06;
    scale = niceScale(lo - pad, hi + pad);
    lo = scale.lo; hi = scale.hi;
  }

  const xOf = v => pad.l + (T1 > T0 ? (v - T0) / (T1 - T0) : 0) * (w - pad.l - pad.r);
  const yOf = v => h - pad.b - (v - lo) / (hi - lo) * (h - pad.t - pad.b);

  ctx.font = '10px "Segoe UI", sans-serif';
  ctx.fillStyle = '#6d788b';
  ctx.strokeStyle = '#1d2532';
  ctx.lineWidth = 1;
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  const ticks = Math.min(6, Math.round((hi - lo) / scale.step));
  for (let k = 0; k <= ticks; k++) {
    const value = lo + scale.step * k;
    if (value > hi + 1e-9) break;
    const y = Math.round(yOf(value)) + 0.5;
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
    ctx.fillText(options.format ? options.format(value) : value.toFixed(1), pad.l - 6, y);
  }
  ctx.textBaseline = 'alphabetic';
  for (let k = 0; k <= 4; k++) {
    const value = T0 + (T1 - T0) * k / 4;
    const x = Math.round(xOf(value)) + 0.5;
    ctx.textAlign = k === 0 ? 'left' : k === 4 ? 'right' : 'center';
    ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, h - pad.b); ctx.stroke();
    ctx.fillText(fmtTime(value - T0), x, h - 4);
  }
  ctx.textAlign = 'center';

  ctx.save();
  ctx.beginPath();
  ctx.rect(pad.l, pad.t, w - pad.l - pad.r, h - pad.t - pad.b);
  ctx.clip();
  ctx.lineWidth = 1.1;
  ctx.lineJoin = 'round';
  for (const s of series) {
    ctx.strokeStyle = s.color;
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < N; i++) {
      const v = s.values[i];
      if (v === null || !isFinite(v)) { started = false; continue; }
      const x = xOf(CAP.t[i]), y = yOf(v);
      if (started) ctx.lineTo(x, y); else { ctx.moveTo(x, y); started = true; }
    }
    ctx.stroke();
  }
  ctx.restore();

  charts[key] = { xOf, yOf, pad, w, h, series, lo, hi };
}

function drawOverlay(key, plotId) {
  const chart = charts[key];
  if (!chart) return;
  const plot = el(plotId);
  const over = plot.querySelector('.over');
  const fitRes = fit(over);
  if (!fitRes) return;
  const { ctx, w, h } = fitRes;
  const { xOf, yOf, pad } = chart;
  const x = Math.round(xOf(CAP.t[state.i])) + 0.5;

  ctx.strokeStyle = 'rgba(120,170,255,.55)';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, h - pad.b); ctx.stroke();

  for (const s of chart.series) {
    const v = s.values[state.i];
    if (v === null || !isFinite(v)) continue;
    const y = yOf(v);
    ctx.fillStyle = s.color;
    ctx.beginPath(); ctx.arc(x, y, 2.6, 0, Math.PI * 2); ctx.fill();
  }
}

// ---------- hover readout ----------
const PLOT_KEYS = { plotAngles: 'angles', plotForceA: 'forceA', plotForceB: 'forceB' };

function bindHover(plotId) {
  const plot = el(plotId);
  const key = PLOT_KEYS[plotId];
  const canvas = plot.querySelector('.hover');
  const tip = plot.querySelector('.tip');
  if (!canvas || !tip) return;

  plot.addEventListener('mouseleave', () => {
    fit(canvas);            // fit() reallocates only on resize, and clears
    tip.style.display = 'none';
  });

  plot.addEventListener('mousemove', event => {
    const chart = charts[key];
    if (!chart) return;
    const rect = plot.getBoundingClientRect();
    const span = chart.w - chart.pad.l - chart.pad.r;
    if (span <= 0) return;
    const local = event.clientX - rect.left;
    const ratio = Math.max(0, Math.min(1, (local - chart.pad.l) / span));
    const index = lowerBound(CAP.t, T0 + ratio * (T1 - T0));

    const fitRes = fit(canvas);
    if (!fitRes) return;
    const { ctx, h } = fitRes;
    const x = Math.round(chart.xOf(CAP.t[index])) + 0.5;
    ctx.save();
    ctx.strokeStyle = 'rgba(190,215,255,.8)';
    ctx.setLineDash([4, 3]);
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(x, chart.pad.t); ctx.lineTo(x, h - chart.pad.b); ctx.stroke();
    ctx.restore();

    const rows = chart.series
      .map(s => ({ name: s.name, color: s.color, value: s.values[index] }))
      .filter(r => r.value !== null && isFinite(r.value))
      .sort((a, b) => b.value - a.value)
      .slice(0, 6);
    for (const r of rows) {
      const y = chart.yOf(r.value);
      if (y < chart.pad.t || y > h - chart.pad.b) continue;
      ctx.fillStyle = r.color;
      ctx.beginPath(); ctx.arc(x, y, 2.4, 0, Math.PI * 2); ctx.fill();
    }

    tip.innerHTML = '<b>' + fmtTime(CAP.t[index] - T0) + ' · 帧 ' + CAP.frames[index] + '</b>' +
      rows.map(r => '<br><i style="background:' + r.color + '"></i>' +
        r.name + ' ' + r.value.toFixed(r.value < 10 ? 2 : 1)).join('');
    tip.style.display = 'block';
    const width = tip.offsetWidth;
    tip.style.left = Math.min(chart.w - width - 6, Math.max(6, local + 14)) + 'px';
    tip.style.top = '6px';
  });
}

// ---------- 10 路合力（纯数字，不做热力图） ----------
const SLOT_ROWS = [];

function slotIndex(seconds) {
  if (!SLOT || !SLOT.times || !SLOT.times.length) return 0;
  const list = SLOT.times;
  let lo = 0, hi = list.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (list[mid] < seconds) lo = mid + 1; else hi = mid;
  }
  return Math.max(0, Math.min(list.length - 1, lo));
}

function initSlotTable() {
  const box = el('slotTable');
  if (!SLOT) {
    box.innerHTML = '<div class="hint">本次采集没有合力数据</div>';
    return;
  }
  const bin = atob(SLOT.data);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  // base64 还原成小端 int16 三元组：每帧 10 路 × (x, y, z)
  SLOT.data16 = new Int16Array(bytes.buffer);
  SLOT.times = SLOT.t;
  el('slotMeta').textContent = SLOT.unit + ' · ' + SLOT.frames + ' 帧采样';

  const labels = CAP.slot_labels || [];
  let html = '<table><thead><tr><th>槽位</th><th>Fx</th><th>Fy</th><th>Fz</th>' +
             '<th>|F|</th><th>点数</th></tr></thead><tbody>';
  for (let r = 0; r < SLOT.slots; r++) {
    html += '<tr><td>' + (labels[r] || ('#' + r)) +
            '</td><td></td><td></td><td></td><td></td><td></td></tr>';
  }
  html += '<tr class="total"><td>合计</td><td></td><td></td><td></td><td></td>' +
          '<td></td></tr></tbody></table>';
  box.innerHTML = html;

  SLOT_ROWS.length = 0;
  const rows = box.querySelectorAll('tbody tr');
  for (let r = 0; r < rows.length; r++) {
    SLOT_ROWS.push(rows[r].querySelectorAll('td'));
  }
}

function drawSlotTable() {
  if (!SLOT || !SLOT.data16 || !SLOT_ROWS.length) return;
  const k = slotIndex(CAP.t[state.i]);
  const base = k * SLOT.slots * 3;
  const counts = (SLOT.counts && SLOT.counts[k]) || [];
  const sum = [0, 0, 0];
  for (let r = 0; r < SLOT.slots; r++) {
    const fx = SLOT.data16[base + r * 3] / 10;
    const fy = SLOT.data16[base + r * 3 + 1] / 10;
    const fz = SLOT.data16[base + r * 3 + 2] / 10;
    sum[0] += fx; sum[1] += fy; sum[2] += fz;
    const cells = SLOT_ROWS[r];
    if (!cells) continue;
    cells[1].textContent = fx.toFixed(2);
    cells[2].textContent = fy.toFixed(2);
    cells[3].textContent = fz.toFixed(2);
    cells[4].textContent = Math.hypot(fx, fy, fz).toFixed(2);
    cells[5].textContent = counts[r] === undefined ? '—' : counts[r];
  }
  const total = SLOT_ROWS[SLOT.slots];
  if (total) {
    total[1].textContent = sum[0].toFixed(2);
    total[2].textContent = sum[1].toFixed(2);
    total[3].textContent = sum[2].toFixed(2);
    total[4].textContent = Math.hypot(sum[0], sum[1], sum[2]).toFixed(2);
    const countSum = counts.reduce((a, b) => a + b, 0);
    total[5].textContent = counts.length ? countSum : '—';
  }
}

// ---------- distribution-force hand scatter plot ----------
// Point positions come from DIST.points: 10 slots of 2D coordinates in the
// 0..1 canvas.  Sensor-pad shapes are derived from the local PXCap Studio
// models; their placement on the hand is schematic, not calibrated.

const HEAT_STOPS = [
  [0.00, [16, 23, 34]],
  [0.25, [30, 75, 122]],
  [0.50, [42, 167, 161]],
  [0.75, [242, 193, 78]],
  [1.00, [228, 87, 46]],
];

function heatColor(t) {
  t = Math.max(0, Math.min(1, t));
  for (let i = 1; i < HEAT_STOPS.length; i++) {
    const [p1, c1] = HEAT_STOPS[i];
    if (t <= p1) {
      const [p0, c0] = HEAT_STOPS[i - 1];
      const u = (t - p0) / (p1 - p0 || 1);
      const rgb = c0.map((v, j) => Math.round(v + (c1[j] - v) * u));
      return 'rgb(' + rgb.join(',') + ')';
    }
  }
  return 'rgb(228,87,46)';
}

function initDistribution() {
  if (!DIST) {
    el('distMeta').textContent = '无数据';
    el('distReadout').innerHTML =
      '<div class="hint">本次采集没有分布力数据（采集时使用了 --omit-distribution）。</div>';
    el('distPlot').style.display = 'none';
    el('distHint').style.display = 'none';
    return;
  }
  if (DIST.layout_source === 'schematic') {
    el('distHint').textContent =
      '未找到本地 Studio 测点模型，使用示意网格；贴片位置为示意，非标定。';
  }
  const bin = atob(DIST.data);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  DIST_DATA = new Uint16Array(bytes.buffer);
  if (DIST.tangent) {
    const tbin = atob(DIST.tangent);
    const tbytes = new Uint8Array(tbin.length);
    for (let i = 0; i < tbin.length; i++) tbytes[i] = tbin.charCodeAt(i);
    DIST_TANGENT = new Int8Array(tbytes.buffer);
    DIST_ARROWS_ON = true;
    const toggle = el('distArrowToggle');
    toggle.checked = true;
    toggle.disabled = false;
    toggle.addEventListener('change', () => {
      DIST_ARROWS_ON = toggle.checked;
      drawDistribution();
    });
  } else {
    el('distArrowToggleWrap').style.display = 'none';
  }
  el('distMeta').textContent =
    DIST.shape[0] + '×' + DIST.shape[1] + ' 点';
  el('distLegend').innerHTML =
    '<div class="bar"></div>' +
    '<div class="row"><span>0 N</span><span>' + DIST.scale_n.toFixed(1) + ' N</span></div>' +
    '<div class="row"><span>幂函数色标</span><span>当前帧</span></div>';
  const bar = el('distLegend').querySelector('.bar');
  if (bar) {
    const stops = [0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1];
    bar.style.background = 'linear-gradient(90deg,' + stops.map(
      t => heatColor(Math.pow(t, DIST_COLOR_GAMMA)) + ' ' + (t * 100) + '%'
    ).join(',') + ')';
  }
  const labels = DIST_LABELS;
  el('distSlots').innerHTML = labels.map(
    (label, i) => '<span>' + label + '</span><b data-slot="' + i + '">—</b>'
  ).join('');
  DIST_SLOT_CELLS = Array.from(el('distSlots').querySelectorAll('b'));
}

function drawDistribution() {
  if (!DIST || !DIST_DATA) return;
  const canvas = el('distCanvas');
  const fitRes = fit(canvas);
  if (!fitRes) return;
  const { ctx, w, h } = fitRes;
  DIST_HIT.length = 0;
  const left = (META.hand_side || 'right') === 'left';

  const k = slotIndex(CAP.t[state.i]);
  const counts = (SLOT.counts && SLOT.counts[k]) || [];
  const base = k * DIST.shape[0] * DIST.shape[1];
  const labels = DIST_LABELS;
  let validTotal = 0, peak = 0, peakSlot = -1;

  ctx.font = '9px "Segoe UI", sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'bottom';

  const points = DIST.points || [];
  for (let slot = 0; slot < DIST.shape[0]; slot++) {
    const slotPoints = points[slot] || [];
    const capacity = (DIST.capacities && DIST.capacities[slot]) || DIST.shape[1];
    const count = Math.min(Number(counts[slot] || 0), capacity, slotPoints.length);
    if (count <= 0) {
      if (DIST_SLOT_CELLS[slot]) DIST_SLOT_CELLS[slot].textContent = '—';
      continue;
    }

    let slotPeak = 0, minY = h, sumX = 0, drawn = 0;
    const dots = [];
    for (let point = 0; point < count; point++) {
      const pair = slotPoints[point];
      if (!pair) continue;
      const x = (left ? 1 - pair[0] : pair[0]) * w;
      const y = pair[1] * h;
      const raw = DIST_DATA[base + slot * DIST.shape[1] + point] || 0;
      const value = raw / 100;
      if (value > slotPeak) slotPeak = value;
      if (y < minY) minY = y;
      sumX += x;
      drawn++;
      validTotal++;
      if (value > peak) { peak = value; peakSlot = slot; }
      dots.push({ x, y, value, point });
    }
    // Low-force dots first so a hot contact is never hidden by its neighbours.
    dots.sort((a, b) => a.value - b.value);
    for (const dot of dots) {
      const linear = Math.min(1, dot.value / (DIST.scale_n || 1));
      const ratio = Math.pow(linear, DIST_COLOR_GAMMA);
      const radius = dot.value > 0 ? 2.2 + 2.6 * ratio : 2.0;
      ctx.fillStyle = dot.value > 0 ? heatColor(ratio) : '#2a3646';
      ctx.beginPath();
      ctx.arc(dot.x, dot.y, radius, 0, Math.PI * 2);
      ctx.fill();
      DIST_HIT.push({
        x: dot.x, y: dot.y, r: radius + 2,
        slot, point: dot.point, value: dot.value,
      });
    }
    if (DIST_TANGENT && DIST_ARROWS_ON) {
      const rotDeg = (DIST.rotations && DIST.rotations[slot]) || 0;
      const rot = rotDeg * Math.PI / 180;
      const cosR = Math.cos(rot), sinR = Math.sin(rot);
      const tanScale = DIST.tangent_scale_n || 1;
      for (const dot of dots) {
        const ti = (
          (k * DIST.shape[0] + slot) * DIST.shape[1] + dot.point
        ) * 2;
        let fx = DIST_TANGENT[ti];
        let fy = DIST_TANGENT[ti + 1];
        if ((!fx && !fy) || fx === undefined) continue;
        if (rot) {
          const rx = fx * cosR - fy * sinR;
          const ry = fx * sinR + fy * cosR;
          fx = rx;
          fy = ry;
        }
        fx *= TANGENT_SIGN;
        fy *= TANGENT_SIGN;
        const flip = DIST.flips && DIST.flips[slot];
        if (flip === 'center') {
          fx = -fx;
          fy = -fy;
        } else if (flip === 'v') {
          fx = -fx;
        } else if (flip === 'h') {
          fy = -fy;
        }
        if (left) fx = -fx;
        const mag = Math.hypot(fx, fy);
        if (mag <= 0) continue;
        const tanN = mag * 0.1;
        if (tanN < 0.1) continue;
        const len = 0 + 16 * Math.min(1, tanN / tanScale);
        const ux = fx / mag, uy = fy / mag;
        const ex = dot.x + ux * len;
        const ey = dot.y + uy * len;
        ctx.strokeStyle = 'rgba(228,242,255,0.9)';
        ctx.lineWidth = 1.3;
        ctx.beginPath();
        ctx.moveTo(dot.x, dot.y);
        ctx.lineTo(ex, ey);
        ctx.stroke();
        const ah = Math.min(5.5, len * 0.4);
        const angle = Math.atan2(uy, ux);
        ctx.fillStyle = 'rgba(215,232,255,0.85)';
        ctx.beginPath();
        ctx.moveTo(ex, ey);
        ctx.lineTo(
          ex - ah * Math.cos(angle - 0.5),
          ey - ah * Math.sin(angle - 0.5)
        );
        ctx.lineTo(
          ex - ah * Math.cos(angle + 0.5),
          ey - ah * Math.sin(angle + 0.5)
        );
        ctx.closePath();
        ctx.fill();
      }
    }
    if (DIST_SLOT_CELLS[slot]) {
      DIST_SLOT_CELLS[slot].textContent = slotPeak > 0 ? slotPeak.toFixed(2) : '—';
      DIST_SLOT_CELLS[slot].style.color = slotPeak > 0 ? '#e7ecf5' : '#5d6879';
    }
    ctx.fillStyle = '#9aa6b8';
    ctx.fillText(
      labels[slot] || ('#' + slot),
      drawn ? sumX / drawn : w * 0.5,
      Math.max(12, minY - 4)
    );
  }

  const peakLabel = peakSlot >= 0 ? (labels[peakSlot] || ('#' + peakSlot)) : '—';
  el('distReadout').innerHTML =
    '<b>帧 ' + CAP.frames[state.i] + ' · ' + fmtTime(CAP.t[state.i] - T0) + '</b><br>' +
    '有效测点 ' + validTotal + ' 个<br>' +
    '峰值 <b>' + peak.toFixed(2) + ' N</b>' +
    (peakSlot >= 0 ? '（' + peakLabel + '）' : '');
}

function bindDistHover() {
  const canvas = el('distCanvas');
  const tip = el('distTip');
  canvas.addEventListener('mousemove', event => {
    if (!DIST || !DIST_DATA) return;
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    let hit = null;
    for (let i = DIST_HIT.length - 1; i >= 0; i--) {
      const item = DIST_HIT[i];
      const dx = x - item.x, dy = y - item.y;
      if (dx * dx + dy * dy <= item.r * item.r) {
        hit = item;
        break;
      }
    }
    if (!hit) { tip.style.display = 'none'; return; }
    const label = DIST_LABELS[hit.slot] || ('#' + hit.slot);
    const k = slotIndex(CAP.t[state.i]);
    const count = (SLOT.counts && SLOT.counts[k]) ? SLOT.counts[k][hit.slot] : undefined;
    tip.innerHTML =
      '<b>' + label + ' · 点 ' + hit.point + '</b><br>|F| ' + hit.value.toFixed(2) + ' N' +
      (count === undefined ? '' : '<br>槽位有效点数 ' + count);
    tip.style.display = 'block';
    const width = tip.offsetWidth;
    tip.style.left = Math.min(rect.width - width - 6, Math.max(6, x + 12)) + 'px';
    tip.style.top = Math.max(6, y - 34) + 'px';
  });
  canvas.addEventListener('mouseleave', () => { tip.style.display = 'none'; });
}

// ---------- readout ----------
function updateReadout() {
  const i = state.i;
  const angles = CAP.angles[i] || [];
  const labels = CAP.angle_labels || [];
  let html = '<table class="mini"><thead><tr><th>关节</th><th>deg</th></tr></thead><tbody>';
  for (let k = 0; k < angles.length; k++) {
    const v = angles[k];
    html += '<tr><td>' + (labels[k] || ('#' + k)) + '</td><td>' +
      (v === null || !isFinite(v) ? '—' : v.toFixed(2)) + '</td></tr>';
  }
  html += '</tbody></table>';
  // 合计就摆在关节角下面：这个卡片占满右列三行，留空不如把一帧的汇总放上去。
  const total = SLOT_ROWS.length ? SLOT_ROWS[SLOT_ROWS.length - 1] : null;
  if (total) {
    html += '<div class="total">合力合计　Fx ' + total[1].textContent +
            '　Fy ' + total[2].textContent + '　Fz ' + total[3].textContent +
            '　|F| ' + total[4].textContent + '　点数 ' + total[5].textContent + '</div>';
  }
  html += '<div class="angles">帧 ' + CAP.frames[i] + ' · ' + fmtTime(CAP.t[i] - T0) +
          ' · ' + META.rate_hz.toFixed(1) + ' Hz</div>';
  el('readout').innerHTML = html;
}

function updateClock() {
  const t = CAP.t[state.i];
  el('clock').textContent =
    fmtTime(t - T0) + ' / ' + fmtTime(T1 - T0) +
    '   帧 ' + CAP.frames[state.i] + ' / ' + (META.rows - 1);
  el('seek').value = String(state.i);
}

// ---------- Quest 3 (independent SDK time axis) ----------
const QUEST_COLORS = { x: '#ff7b72', y: '#54d49f', z: '#6aa8ff' };
let questProjection = null;

function finite(v) { return v !== null && v !== undefined && Number.isFinite(v); }

function questIndex() {
  if (!QN) return 0;
  if (QUEST.time_based) {
    const target = CAP.t[state.i] - T0;
    if (!QUEST.validRows) {
      QUEST.validRows = [];
      QUEST.validTimes = [];
      for (let i = 0; i < QN; i++) {
        if (finite(QUEST.time[i])) {
          QUEST.validRows.push(i);
          QUEST.validTimes.push(QUEST.time[i]);
        }
      }
    }
    if (!QUEST.validRows.length) return 0;
    let lo = 0, hi = QUEST.validTimes.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (QUEST.validTimes[mid] <= target) lo = mid + 1;
      else hi = mid;
    }
    return QUEST.validRows[Math.max(0, Math.min(QUEST.validRows.length - 1, lo - 1))];
  }
  if (N <= 1) return 0;
  return Math.max(0, Math.min(QN - 1, Math.round(state.i / (N - 1) * (QN - 1))));
}

function questTimeText(index) {
  const value = QUEST && QUEST.time ? QUEST.time[index] : null;
  if (!finite(value)) return '—';
  if (QUEST.time_unit === 's') return value.toFixed(3) + ' s';
  if (QUEST.time_source === 'sdk_timestamp_ns') return value.toFixed(3) + ' ms';
  return '#' + Math.round(value).toString();
}

function questPosePoint(block, index) {
  const xyz = block && block.xyz ? block.xyz[index] : null;
  if (!xyz || xyz.length < 3 || !xyz.every(finite)) return null;
  return { x: xyz[0], y: xyz[1], z: xyz[2] };
}

function questControllerBlock() {
  const side = QUEST.pose_side === 'right' ? 'right' : 'left';
  const controller = QUEST.controllers && QUEST.controllers[side];
  return controller && controller.pose ? controller.pose : QUEST.main_pose;
}

function questMainValid(index) {
  const block = questControllerBlock();
  const point = questPosePoint(block, index);
  return point && QUEST.tracked[index] && block.tracked[index] ? point : null;
}

function buildQuestProjection(w, h) {
  const points = [];
  const blocks = [
    ['main', QUEST.main_pose],
    ['head', QUEST.head],
    ['left', QUEST.controllers && QUEST.controllers.left && QUEST.controllers.left.pose],
    ['right', QUEST.controllers && QUEST.controllers.right && QUEST.controllers.right.pose],
  ];
  for (const [, block] of blocks) {
    if (!block) continue;
    for (let i = 0; i < QN; i++) {
      const p = questPosePoint(block, i);
      if (p) points.push(p);
    }
  }
  if (!points.length) return null;

  const cx = points.reduce((s, p) => s + p.x, 0) / points.length;
  const cy = points.reduce((s, p) => s + p.y, 0) / points.length;
  const cz = points.reduce((s, p) => s + p.z, 0) / points.length;
  const yaw = -35 * Math.PI / 180;
  const pitch = 24 * Math.PI / 180;
  const raw = points.map(p => {
    const dx = p.x - cx, dy = p.y - cy, dz = p.z - cz;
    const rx = dx * Math.cos(yaw) - dz * Math.sin(yaw);
    const rz = dx * Math.sin(yaw) + dz * Math.cos(yaw);
    return { x: rx, y: -(dy * Math.cos(pitch) - rz * Math.sin(pitch)) };
  });
  const xs = raw.map(p => p.x), ys = raw.map(p => p.y);
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);
  const spanX = Math.max(maxX - minX, 1e-6);
  const spanY = Math.max(maxY - minY, 1e-6);
  const padX = 30, padY = 20;
  const scale = Math.min((w - padX * 2) / spanX, (h - padY * 2) / spanY);
  return {
    project(p) {
      const dx = p.x - cx, dy = p.y - cy, dz = p.z - cz;
      const rx = dx * Math.cos(yaw) - dz * Math.sin(yaw);
      const rz = dx * Math.sin(yaw) + dz * Math.cos(yaw);
      const ry = -(dy * Math.cos(pitch) - rz * Math.sin(pitch));
      return {
        x: w / 2 + (rx - (minX + maxX) / 2) * scale,
        y: h / 2 + (ry - (minY + maxY) / 2) * scale,
      };
    },
  };
}

function drawQuestAxes(ctx, w, h) {
  const origin = { x: 34, y: h - 28 };
  const axes = [
    { label: 'X', dx: 22, dy: 0, color: QUEST_COLORS.x },
    { label: 'Y', dx: 0, dy: -22, color: QUEST_COLORS.y },
    { label: 'Z', dx: -13, dy: 14, color: QUEST_COLORS.z },
  ];
  ctx.save();
  ctx.font = '10px "Segoe UI", sans-serif';
  ctx.lineWidth = 2;
  for (const axis of axes) {
    ctx.strokeStyle = axis.color;
    ctx.beginPath(); ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(origin.x + axis.dx, origin.y + axis.dy); ctx.stroke();
    ctx.fillStyle = axis.color;
    ctx.fillText(axis.label, origin.x + axis.dx + 3, origin.y + axis.dy + 3);
  }
  ctx.restore();
}

function drawQuest3DBase() {
  if (!QUEST) return;
  const plot = el('plotQuest3D');
  const base = plot.querySelector('.base');
  const fitRes = fit(base);
  if (!fitRes) return;
  const { ctx, w, h } = fitRes;
  questProjection = buildQuestProjection(w, h);
  if (!questProjection) {
    ctx.fillStyle = '#6d788b';
    ctx.font = '12px "Segoe UI", sans-serif';
    ctx.fillText('没有可用的主位姿追踪轨迹', 18, 28);
    return;
  }

  ctx.save();
  ctx.strokeStyle = '#1e2a3a';
  ctx.lineWidth = 1;
  for (let k = 1; k < 5; k++) {
    const x = 18 + (w - 36) * k / 5;
    const y = 18 + (h - 36) * k / 5;
    ctx.beginPath(); ctx.moveTo(x, 18); ctx.lineTo(x, h - 18); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(18, y); ctx.lineTo(w - 18, y); ctx.stroke();
  }

  ctx.strokeStyle = '#f5a524';
  ctx.lineWidth = 1.5;
  ctx.lineJoin = 'round';
  ctx.beginPath();
  let drawing = false;
  for (let i = 0; i < QN; i++) {
    const point = questMainValid(i);
    if (!point) { drawing = false; continue; }
    const p = questProjection.project(point);
    if (drawing) ctx.lineTo(p.x, p.y); else { ctx.moveTo(p.x, p.y); drawing = true; }
  }
  ctx.stroke();

  const start = [...Array(QN).keys()].map(questMainValid).find(Boolean);
  if (start) {
    const p = questProjection.project(start);
    ctx.fillStyle = '#54d49f';
    ctx.beginPath(); ctx.arc(p.x, p.y, 3.5, 0, Math.PI * 2); ctx.fill();
  }
  ctx.restore();
  drawQuestAxes(ctx, w, h);
}

function drawQuest3DOverlay() {
  if (!QUEST || !questProjection) return;
  const plot = el('plotQuest3D');
  const over = plot.querySelector('.over');
  const fitRes = fit(over);
  if (!fitRes) return;
  const { ctx } = fitRes;
  const index = questIndex();
  const markers = [
    ['主位姿', QUEST.main_pose, '#f5a524'],
    ['头显', QUEST.head, '#4da3ff'],
    ['左手柄', QUEST.controllers && QUEST.controllers.left && QUEST.controllers.left.pose, '#38d39f'],
    ['右手柄', QUEST.controllers && QUEST.controllers.right && QUEST.controllers.right.pose, '#f26d6d'],
  ];
  ctx.save();
  ctx.font = '10px "Segoe UI", sans-serif';
  for (const [label, block, color] of markers) {
    const point = questPosePoint(block, index);
    if (!point) continue;
    const p = questProjection.project(point);
    ctx.fillStyle = color;
    ctx.strokeStyle = '#0c0f15';
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(p.x, p.y, 5, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
    ctx.fillStyle = color;
    ctx.fillText(label, p.x + 7, p.y - 6);
  }
  ctx.restore();
}

function questChartX(index) {
  if (QUEST.time_based) return QUEST.time[index];
  if (QN <= 1) return 0;
  return index / (QN - 1);
}

function questXBounds() {
  if (QUEST.time_based) return [0, Math.max(T1 - T0, 1e-6)];
  return [0, 1];
}

function questValueBounds(block, columns) {
  const values = [];
  if (!block) return [0, 1];
  for (let i = 0; i < QN; i++) {
    if (!block.tracked[i]) continue;
    for (const key of Object.keys(columns)) {
      const source = columns[key];
      const row = block[source.group] ? block[source.group][i] : null;
      const value = row ? row[source.column] : null;
      if (finite(value)) values.push(value);
    }
  }
  if (!values.length) return [0, 1];
  let lo = Math.min(...values), hi = Math.max(...values);
  const pad = Math.max((hi - lo) * 0.08, 0.01);
  return [lo - pad, hi + pad];
}

const questCharts = {};

function drawQuestSeriesChart(key, plotId, block, columns, colors) {
  if (!QUEST) return;
  const plot = el(plotId);
  const base = plot.querySelector('.base');
  const fitRes = fit(base);
  if (!fitRes) return;
  const { ctx, w, h } = fitRes;
  const pad = { l: 50, r: 10, t: 8, b: 18 };
  const [xLo, xHi] = questXBounds();
  const [yLo, yHi] = questValueBounds(block, columns);
  const xOf = t => pad.l + (t - xLo) / (xHi - xLo) * (w - pad.l - pad.r);
  const yOf = v => h - pad.b - (v - yLo) / (yHi - yLo) * (h - pad.t - pad.b);

  ctx.font = '10px "Segoe UI", sans-serif';
  ctx.strokeStyle = '#1d2532';
  ctx.fillStyle = '#6d788b';
  ctx.lineWidth = 1;
  ctx.textAlign = 'right';
  for (let k = 0; k <= 4; k++) {
    const value = yLo + (yHi - yLo) * k / 4;
    const y = Math.round(yOf(value)) + 0.5;
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
    ctx.fillText(value.toFixed(2), pad.l - 5, y + 3);
  }
  ctx.textAlign = 'center';
  for (let k = 0; k <= 4; k++) {
    const value = xLo + (xHi - xLo) * k / 4;
    const x = Math.round(xOf(value)) + 0.5;
    ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, h - pad.b); ctx.stroke();
    const shown = QUEST.time_based
      ? fmtTime(value)
      : Math.round(value * 100) + '%';
    ctx.fillText(shown, x, h - 6);
  }

  ctx.save();
  ctx.beginPath();
  ctx.rect(pad.l, pad.t, w - pad.l - pad.r, h - pad.t - pad.b);
  ctx.clip();
  ctx.lineWidth = 1.2;
  if (block) {
    for (const [name, source] of Object.entries(columns)) {
      ctx.strokeStyle = colors[name];
      ctx.beginPath();
      let started = false;
      for (let i = 0; i < QN; i++) {
        const time = questChartX(i);
        const row = block[source.group] ? block[source.group][i] : null;
        const value = row ? row[source.column] : null;
        if (!block.tracked[i] || !finite(time) || !finite(value)) {
          started = false;
          continue;
        }
        const x = xOf(time), y = yOf(value);
        if (started) ctx.lineTo(x, y); else { ctx.moveTo(x, y); started = true; }
      }
      ctx.stroke();
    }
  }
  ctx.restore();
  questCharts[key] = { xOf, yOf, pad, xLo, xHi };
}

function drawQuestSeriesOverlay(key, plotId) {
  const chart = questCharts[key];
  if (!QUEST || !chart) return;
  const plot = el(plotId);
  const over = plot.querySelector('.over');
  const fitRes = fit(over);
  if (!fitRes) return;
  const { ctx, h } = fitRes;
  const index = questIndex();
  const xValue = questChartX(index);
  if (!finite(xValue)) return;
  const x = Math.round(chart.xOf(xValue)) + 0.5;
  ctx.strokeStyle = 'rgba(245,165,36,.85)';
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.moveTo(x, chart.pad.t);
  ctx.lineTo(x, h - chart.pad.b);
  ctx.stroke();
}

function questPositionText(block, index) {
  const point = questPosePoint(block, index);
  return point ? [point.x, point.y, point.z].map(v => v.toFixed(3)).join(' / ') : '—';
}

function updateQuestReadout() {
  if (!QUEST) return;
  const index = questIndex();
  const quat = QUEST.main_pose && QUEST.main_pose.quat_xyzw ? QUEST.main_pose.quat_xyzw[index] : null;
  const activeButtons = Object.entries(QUEST.buttons || {})
    .filter(([, values]) => values[index])
    .map(([name]) => name);
  const quality = QUEST.quality || {};
  const timeMissing = QUEST.time_source === 'sdk_timestamp_ns' ? '' :
    '<div class="bad">SDK 时间戳不可用，当前横轴为轮询序号</div>';
  const controllerRows = ['left', 'right'].map(side => {
    const ctrl = (QUEST.controllers && QUEST.controllers[side]) || {};
    const grip = ctrl.grip ? ctrl.grip[index] : null;
    const trigger = ctrl.trigger ? ctrl.trigger[index] : null;
    const axis = ctrl.axis ? ctrl.axis[index] : null;
    const selected = side === QUEST.pose_side ? '（本次采集）' : '';
    return '<h4>' + (side === 'left' ? '左手柄' : '右手柄') + selected + '</h4>' +
      '<table>' +
      '<tr><td>位置 xyz</td><td>' + questPositionText(ctrl.pose, index) + '</td></tr>' +
      '<tr><td>grip / trigger</td><td>' + (finite(grip) ? grip.toFixed(3) : '—') +
      ' / ' + (finite(trigger) ? trigger.toFixed(3) : '—') + '</td></tr>' +
      '<tr><td>axis</td><td>' +
      (axis && finite(axis[0]) && finite(axis[1]) ? axis[0].toFixed(2) + ' / ' + axis[1].toFixed(2) : '—') +
      '</td></tr></table>';
  }).join('');
  const jointCounts = QUEST.hand_joint_counts || {};
  const poseSource = QUEST.pose_source || 'controller';
  const poseSide = QUEST.pose_side || '—';
  el('questReadout').innerHTML =
    '<table>' +
    '<tr><td>sequence / 时间</td><td>' + QUEST.sequence[index] + ' / ' + questTimeText(index) + '</td></tr>' +
    '<tr><td>主位姿来源</td><td>' + poseSource + '.' + poseSide + '</td></tr>' +
    '<tr><td>追踪</td><td class="' + (QUEST.tracked[index] ? '' : 'bad') + '">' +
      (QUEST.tracked[index] ? 'tracked' : 'untracked') + '</td></tr>' +
    '<tr><td>主位姿 xyz · m</td><td>' + questPositionText(QUEST.main_pose, index) + '</td></tr>' +
    '<tr><td>主四元数 xyzw</td><td>' +
      (quat && quat.every(finite) ? quat.map(v => v.toFixed(3)).join(' / ') : '—') +
      '</td></tr>' +
    '<tr><td>头显 xyz</td><td>' + questPositionText(QUEST.head, index) + '</td></tr>' +
    '<tr><td>手部关节 L / R</td><td>' + (jointCounts.left ? jointCounts.left[index] : 0) +
      ' / ' + (jointCounts.right ? jointCounts.right[index] : 0) + '</td></tr>' +
    '<tr><td>按键</td><td>' + (activeButtons.length ? activeButtons.join(', ') : '无') + '</td></tr>' +
    '<tr><td>时间戳有效 / 缺失</td><td>' + (quality.sdk_valid_timestamps || 0) +
      ' / ' + (quality.sdk_missing_timestamps || 0) + '</td></tr>' +
    '</table>' +
    '<div class="angles">主位姿 pose_6d = xyz（米）+ 四元数 xyzw（姿态方向）；曲线显示本次采集手对应的手柄。</div>' +
    timeMissing + controllerRows;
}

function buildQuest() {
  const card = el('questCard');
  if (!QUEST || !QN) {
    card.style.display = 'none';
    if (!questNoticeShown) {
      const message = META.quest_rows
        ? 'Quest 数据已隐藏（--no-quest 或缺少可显示字段）'
        : '本次采集没有 Quest 数据';
      const div = document.createElement('div');
      div.className = 'warn';
      div.textContent = message;
      el('warnBox').appendChild(div);
      questNoticeShown = true;
    }
    return;
  }
  card.style.display = 'flex';
  const aligned = QUEST.alignment !== 'none';
  const verified = QUEST.alignment === 'sdk_unix';
  const sideLabel = QUEST.pose_side === 'right' ? '右手柄' : '左手柄';
  el('questMeta').textContent =
    QUEST.rows + ' 行 · ' + QUEST.shown + ' 点 · ' +
    (aligned
      ? (QUEST.alignment === 'host'
          ? '主机接收对齐'
          : verified ? 'Unix 对齐 (C8)' : 'Unix 对齐（未验证）')
      : QUEST.time_source === 'sdk_timestamp_ns' ? '归一化进度' : '轮询序号');
  el('questAlignNote').classList.toggle('aligned', aligned);
  el('questAlignNote').textContent = QUEST.alignment_note;
  el('questPosTitle').textContent = sideLabel + '位置 XYZ · m';
  el('questQuatTitle').textContent = sideLabel + '姿态四元数 xyzw';
  el('quest3DTitle').textContent = sideLabel + '位姿 3D 轨迹（X / Y / Z）';
  const block = questControllerBlock();
  drawQuestSeriesChart(
    'pos',
    'plotQuestPos',
    block,
    {
      x: { group: 'xyz', column: 0 },
      y: { group: 'xyz', column: 1 },
      z: { group: 'xyz', column: 2 },
    },
    QUEST_COLORS,
  );
  drawQuestSeriesChart(
    'quat',
    'plotQuestQuat',
    block,
    {
      qx: { group: 'quat_xyzw', column: 0 },
      qy: { group: 'quat_xyzw', column: 1 },
      qz: { group: 'quat_xyzw', column: 2 },
      qw: { group: 'quat_xyzw', column: 3 },
    },
    { qx: '#f5a524', qy: '#38d39f', qz: '#4da3ff', qw: '#f26d6d' },
  );
  drawQuest3DBase();
}

function refreshQuest() {
  if (!QUEST || !QN) return;
  drawQuest3DOverlay();
  drawQuestSeriesOverlay('pos', 'plotQuestPos');
  drawQuestSeriesOverlay('quat', 'plotQuestQuat');
  updateQuestReadout();
}

// ---------- camera previews ----------
const VIDEOS = [];

function buildVideos() {
  const strip = el('videoStrip');
  const cams = (DATA.video && DATA.video.cams) || [];
  if (!cams.length) { strip.style.display = 'none'; return; }
  const usable = cams.filter(c => c.file);
  if (!usable.length) {
    strip.innerHTML = '<div class="empty">这次采集没有可播放的相机预览（缺 mkv 或转码失败）</div>';
    return;
  }
  strip.innerHTML = '';
  for (const cam of usable) {
    const fig = document.createElement('figure');
    const video = document.createElement('video');
    video.src = cam.file;
    video.muted = true;
    video.playsInline = true;
    video.preload = 'auto';
    const halfFrame = cam.fps ? Math.round(500 / cam.fps) : 0;
    const caption = document.createElement('figcaption');
    caption.textContent = cam.id + ' · ' + cam.frames + ' 帧 · 首帧 ' +
      (cam.offset_s >= 0 ? '+' : '') + cam.offset_s.toFixed(2) + ' s · ±' +
      halfFrame + ' ms';
    fig.title = cam.id + '：' + cam.frames + ' 帧 @ ' + cam.fps.toFixed(1) +
      ' fps，首帧比手套首帧' + (cam.offset_s < 0 ? '早' : '晚') + ' ' +
      Math.abs(cam.offset_s).toFixed(2) + ' s，逐帧对齐误差不超过 ' +
      halfFrame + ' ms';
    // The mp4 only becomes seekable after metadata arrives, which always happens
    // after boot()'s first refresh(); without this hook the strip would sit on
    // frame 0 until the user touches a control.
    video.addEventListener('loadedmetadata', () => syncVideos(state.playing));
    fig.appendChild(video);
    fig.appendChild(caption);
    strip.appendChild(fig);
    VIDEOS.push({ el: video, offset: cam.offset_s || 0, fps: cam.fps || 30 });
  }
}

function syncVideos(playing) {
  if (!VIDEOS.length) return;
  const relative = CAP.t[state.i] - T0;
  for (const v of VIDEOS) {
    if (v.el.readyState < 1) continue;
    const fps = v.fps || 30;
    const duration = v.el.duration || 1e9;
    // Camera frame k sits at glove time ``offset + k / fps`` while no frames are
    // dropped; with drops the index stops being uniform but the *timeline* still
    // matches (passthrough kept the real spacing), so map by time, not by index.
    // Seek to the *middle* of the nearest frame instead of to the instant itself:
    // a frame stays on screen until the next PTS, so seeking to the instant would
    // show the previous (up to 1/fps older) exposure whenever the glove frame
    // falls in the second half of a camera frame interval.
    const raw = Math.max(0, Math.min(duration, relative - v.offset));
    const target = Math.max(0, Math.min(duration, (Math.round(raw * fps) + 0.5) / fps));
    if (playing) {
      v.el.playbackRate = state.speed;
      if (v.el.paused) v.el.play().catch(() => {});
      if (Math.abs(v.el.currentTime - target) > 0.15) v.el.currentTime = target;
    } else {
      if (!v.el.paused) v.el.pause();
      // 0.5 frame of slack: after a seek currentTime settles on the requested
      // value, so consecutive glove frames that share one camera frame produce
      // an identical target and trigger no extra decode work.
      if (Math.abs(v.el.currentTime - target) > 0.5 / fps) v.el.currentTime = target;
    }
  }
}

// ---------- frame refresh ----------
function refresh() {
  drawOverlay('angles', 'plotAngles');
  drawOverlay('forceA', 'plotForceA');
  drawOverlay('forceB', 'plotForceB');
  drawSlotTable();
  drawDistribution();
  updateReadout();
  updateClock();
  refreshQuest();
  syncVideos(state.playing);
}

// ---------- playback ----------
let lastTick = 0;
let rafId = 0;

function tick(now) {
  if (!state.playing) { lastTick = 0; return; }
  if (!lastTick) { lastTick = now; state.origin = CAP.t[state.i]; }
  // Advance from the frame the play started at, not by accumulating per-tick
  // deltas: if two rAF chains ever overlap, adding dt twice would run the
  // replay at 2x and there would be no way to notice from the UI.
  const target = state.origin + (now - lastTick) / 1000 * state.speed;
  const next = lowerBound(CAP.t, target);
  if (next >= N - 1) {
    state.i = N - 1;
    pause();
    refresh();
    return;
  }
  if (next !== state.i) state.i = next;
  refresh();
  rafId = requestAnimationFrame(tick);
}

function play() {
  if (state.playing) return;
  if (state.i >= N - 1) state.i = 0;
  state.playing = true;
  lastTick = 0;
  el('playBtn').textContent = '❚❚ 暂停';
  el('playBtn').classList.add('on');
  if (rafId) cancelAnimationFrame(rafId);
  rafId = requestAnimationFrame(tick);
}

function pause() {
  state.playing = false;
  lastTick = 0;
  if (rafId) { cancelAnimationFrame(rafId); rafId = 0; }
  // tick() no longer runs, so the videos have to be stopped here: otherwise they
  // would keep rolling after the glove panel freezes and drift out of sync until
  // the next key press.
  syncVideos(false);
  el('playBtn').textContent = '▶ 播放';
  el('playBtn').classList.remove('on');
}

function seekTo(index) {
  state.i = Math.max(0, Math.min(N - 1, index));
  refresh();
}

// ---------- wiring ----------
function buildHeader() {
  el('title').textContent = META.name || 'PXCap III';
  const facts = [
    ['时长', META.duration_s.toFixed(2) + ' s'],
    ['手套帧', META.rows],
    ['手别', META.hand_side === 'left' ? '左手' : '右手'],
    ['速率', (META.rate_hz || 0).toFixed(1) + ' Hz'],
    ['Quest 行', META.quest_rows || 0],
    [
      'Quest 对齐',
      QUEST
        ? (QUEST.alignment === 'sdk_unix'
            ? 'Unix (C8)'
            : QUEST.alignment === 'host' ? '主机接收'
            : QUEST.alignment === 'unix' ? 'Unix (未验证)' : '未做')
        : '无',
    ],
    ['时钟', META.clock],
    ['格式', 'v' + (META.format_version ?? '?')],
  ];
  if (META.shown_rows !== META.rows) {
    facts.push([
      META.thinning === 'envelope' ? '保峰抽稀' : '曲线抽稀',
      META.shown_rows + ' / ' + META.rows,
    ]);
  }
  const quality = META.clock_quality;
  if (quality) {
    const drift = (quality.drift_ms >= 0 ? '+' : '') + quality.drift_ms.toFixed(1) + ' ms';
    facts.push([
      '时钟漂移',
      quality.steps_over_10ms ? drift + ' · 跳变 ' + quality.steps_over_10ms + ' 次' : drift,
    ]);
  }
  el('facts').innerHTML = facts.map(
    ([k, v]) => '<span>' + k + ' <b>' + v + '</b></span>'
  ).join('');

  if (DATA.warnings && DATA.warnings.length) {
    el('warnBox').innerHTML = DATA.warnings.map(
      w => '<div class="warn">' + w + '</div>'
    ).join('');
  }
}

function buildLegends() {
  el('legendAngles').innerHTML = CAP.angle_labels.map(
    (name, i) => '<span><i style="background:' + ANGLE_COLORS[i] + '"></i>' + name + '</span>'
  ).join('');
  el('legendForceA').innerHTML = legendSlots(0, 5);
  el('legendForceB').innerHTML = legendSlots(5, 10);
}

function legendSlots(from, to) {
  return CAP.slot_labels.slice(from, to).map(
    (name, i) => '<span><i style="background:' + rampColor(from + i, CAP.slot_labels.length) +
      '"></i>' + name + '</span>'
  ).join('');
}

const ANGLE_COLORS = ['#4da3ff', '#38d39f', '#f5a524', '#f26d6d'];

function forceSeriesRange(from, to) {
  return CAP.slot_labels.slice(from, to).map((name, i) => ({
    name,
    color: rampColor(from + i, CAP.slot_labels.length),
    // 文件里是 0.1 N 的整数，曲线上换成 N（等值，只是把 0.1 N 写成 0.1）。
    values: CAP.force.map(row => row[from + i] / 10),
  }));
}

function buildCharts() {
  const angleSeries = CAP.angle_labels.map((name, i) => ({
    name,
    color: ANGLE_COLORS[i % ANGLE_COLORS.length],
    values: CAP.angles.map(row => row[i]),
  }));
  drawLineChart('angles', 'plotAngles', angleSeries, {
    format: v => v.toFixed(1),
  });

  // Ten lines in one 150 px tall box was unreadable; five per plot keeps the
  // strokes and the legend far enough apart to actually name a curve.
  drawLineChart('forceA', 'plotForceA', forceSeriesRange(0, 5), {
    format: v => v.toFixed(2),
    zeroBase: true,
  });
  drawLineChart('forceB', 'plotForceB', forceSeriesRange(5, 10), {
    format: v => v.toFixed(2),
    zeroBase: true,
  });
}

function bindSeek(plotId) {
  const plot = el(plotId);
  plot.addEventListener('click', event => {
    const rect = plot.getBoundingClientRect();
    const chart = charts[PLOT_KEYS[plotId]];
    if (!chart) return;
    const local = event.clientX - rect.left;
    const ratio = (local - chart.pad.l) / (chart.w - chart.pad.l - chart.pad.r);
    const target = T0 + Math.max(0, Math.min(1, ratio)) * (T1 - T0);
    seekTo(lowerBound(CAP.t, target));
  });
}

function boot() {
  buildHeader();
  buildLegends();
  initSlotTable();
  initDistribution();
  buildCharts();
  buildQuest();
  buildVideos();
  el('seek').max = String(N - 1);
  bindSeek('plotAngles');
  bindSeek('plotForceA');
  bindSeek('plotForceB');
  bindHover('plotAngles');
  bindHover('plotForceA');
  bindHover('plotForceB');
  bindDistHover();

  el('playBtn').onclick = () => state.playing ? pause() : play();
  el('speedBtn').onclick = () => {
    const cycle = [1, 2, 0.5, 0.25];
    const next = cycle[(cycle.indexOf(state.speed) + 1) % cycle.length];
    state.speed = next;
    el('speedBtn').textContent = next.toFixed(next < 1 ? 2 : 1) + '×';
  };
  el('seek').oninput = event => {
    pause();
    seekTo(Number(event.target.value));
  };

  window.addEventListener('keydown', event => {
    if (event.code === 'Space') { event.preventDefault(); state.playing ? pause() : play(); }
    if (event.code === 'ArrowRight') { pause(); seekTo(state.i + 1); }
    if (event.code === 'ArrowLeft') { pause(); seekTo(state.i - 1); }
  });

  let resizeTimer = 0;
  window.addEventListener('resize', () => {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(() => {
      buildCharts();
      buildQuest();
      refresh();
    }, 120);
  });

  refresh();
}

boot();
</script>
</body>
</html>
"""


# --------------------------------------------------------------------------
# cli
# --------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="把一次采集的 data.h5 渲染成自包含的 viewer.html",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("capture", type=Path, help="采集目录（含 data.h5）")
    parser.add_argument(
        "--html",
        nargs="?",
        const="",
        default=None,
        metavar="PATH",
        help="导出浏览器版而不是开窗口；省略路径则写到采集目录下的 viewer.html",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=DEFAULT_MAX_POINTS,
        help="手套和 Quest 曲线各最多保留多少帧（超出则等间隔抽稀）",
    )
    parser.add_argument(
        "--dist-frames",
        type=int,
        default=DEFAULT_DIST_FRAMES,
        help="合力表最多保留多少帧（超出则等间隔抽稀）",
    )
    parser.add_argument(
        "--no-quest",
        action="store_true",
        help="不导出或绘制 Quest 数据",
    )
    parser.add_argument(
        "--no-dist-arrows",
        action="store_true",
        help="不在 HTML 中打包/绘制每个测点的切向合力箭头（减小体积）",
    )
    parser.add_argument(
        "--quest-align",
        choices=QUEST_ALIGNMENT_CHOICES,
        default="auto",
        help=(
            "auto=有 quest/host_unix_ns 用主机接收时刻（新采集，默认），"
            "没有则回退 unix；host=强制主机接收时刻；"
            "unix=SDK 戳当主机墙钟；sdk_unix=先经 C8 报告换算；"
            "none=独立时间轴"
        ),
    )
    parser.add_argument(
        "--quest-clock-report",
        type=Path,
        default=None,
        help="quest_clock_probe.py 生成的通过报告；--quest-align sdk_unix 时必填",
    )
    parser.add_argument(
        "--save-png",
        type=Path,
        default=None,
        metavar="PATH",
        help="不开窗口，只把首帧渲染成 PNG（无显示器时也能用）",
    )
    parser.add_argument(
        "--video",
        dest="video",
        action="store_true",
        default=True,
        help="导出浏览器版时顺便转码相机预览（mkv → mp4，浏览器能播）",
    )
    parser.add_argument(
        "--no-video",
        dest="video",
        action="store_false",
        help="不转码视频，只出曲线与合力表",
    )
    parser.add_argument(
        "--video-width",
        type=int,
        default=640,
        help="预览视频宽度，高度按比例",
    )
    parser.add_argument(
        "--video-crf",
        type=int,
        default=26,
        help="预览视频的 x264 CRF，越大越小越糊",
    )
    parser.add_argument(
        "--ffmpeg",
        default=None,
        help="ffmpeg 5+ 路径（默认读 $FFMPEG，再找 PATH）",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    root = args.capture.expanduser()
    if not root.is_dir():
        print(f"[错误] 目录不存在：{root}", file=sys.stderr)
        return 2
    if args.no_quest and args.quest_align in QUEST_ALIGNMENT_ACTIVE:
        print(
            "[错误] --no-quest 与 --quest-align unix/host/sdk_unix 不能同时使用",
            file=sys.stderr,
        )
        return 2
    if (
        args.quest_clock_report is not None
        and args.quest_align != QUEST_ALIGNMENT_SDK_UNIX
    ):
        print(
            "[警告] 给了 --quest-clock-report 但 --quest-align 不是 sdk_unix，"
            "报告不会被使用",
            file=sys.stderr,
        )
    clock_report = None
    if args.quest_align == QUEST_ALIGNMENT_SDK_UNIX:
        if args.quest_clock_report is None:
            print(
                "[错误] --quest-align sdk_unix 必须同时提供 "
                "--quest-clock-report",
                file=sys.stderr,
            )
            return 2
        try:
            clock_report = load_clock_report(args.quest_clock_report)
        except ValueError as exc:
            print(f"[错误] {exc}", file=sys.stderr)
            return 2

    try:
        payload = collect_payload(
            root,
            max_points=max(16, args.max_points),
            dist_frames=max(16, args.dist_frames),
            with_quest=not args.no_quest,
            quest_alignment=args.quest_align,
            quest_clock_report=clock_report,
            with_dist_arrows=not args.no_dist_arrows,
        )
    except ValueError as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 2

    if args.html is not None:
        out_path = Path(args.html) if args.html else (root / "viewer.html")
        out_path.parent.mkdir(parents=True, exist_ok=True)
        if args.video:
            prepare_videos(payload, out_path.parent, args)
        write_viewer(payload, out_path)
        report_html(payload, out_path)
        return 0

    return run_window(payload, root, snapshot=args.save_png)


def prepare_videos(payload: dict[str, Any], out_dir: Path, args: Any) -> None:
    """Transcode every camera mkv next to the html so ``<video>`` can play it."""
    tracks = payload.get("video", {}).get("cams") or []
    sources = [track for track in tracks if track.get("mkv")]
    if not sources:
        return
    ffmpeg = find_ffmpeg(args.ffmpeg)
    if not ffmpeg:
        payload["warnings"].append(
            "找不到 ffmpeg，视频面板留空（用 --ffmpeg 指定路径）"
        )
        return
    preview_dir = out_dir / "preview"
    preview_dir.mkdir(parents=True, exist_ok=True)
    for track in sources:
        print(f"[视频] 转码 {track['id']} → preview/{track['id']}.mp4 …")
        error = build_preview(
            track, preview_dir, max(160, args.video_width), args.video_crf, ffmpeg
        )
        if error:
            track["error"] = error
            payload["warnings"].append(f"{track['id']} 转码失败：{error}")
            continue
        track["file"] = f"preview/{track['id']}.mp4"
        size_mb = (track.get("bytes") or 0) / 1e6
        print(f"[视频] {track['id']} 完成，{size_mb:.1f} MB")


def report_html(payload: dict[str, Any], out_path: Path) -> None:
    meta = payload["meta"]
    size_mb = out_path.stat().st_size / 1e6
    print(f"写入 {windows_path(out_path)}  ({size_mb:.2f} MB)")
    print(
        f"  {meta['rows']} 帧 / {meta['duration_s']:.2f} s / "
        f"{meta['rate_hz']:.1f} Hz，曲线抽稀到 {meta['shown_rows']} 点"
    )
    if payload["slot_force"]:
        print(f"  合力表 {payload['slot_force']['frames']} 帧 × "
              f"{payload['slot_force']['slots']} 路（单位 {payload['slot_force']['unit']}）")
        dist = payload["slot_force"].get("dist")
        if dist:
            print(
                f"  分布力散点图 {dist['shape'][0]} 槽位 × {dist['shape'][1]} 点，"
                f"色标 0~{dist['scale_n']:.2f} N"
            )
            if dist.get("tangent"):
                print(
                    f"  切向箭头 99 分位 {dist['tangent_scale_n']:.2f} N"
                )
        else:
            print("  分布力散点图：本次采集没有分布力数据")
    else:
        print("  合力表：本次采集没有合力数据")
    cams = payload.get("video", {}).get("cams") or []
    played = [cam for cam in cams if cam.get("file")]
    if played:
        detail = "、".join(
            f"{cam['id']} {cam['fps']:.1f} fps 首帧 {cam['offset_s']:+.2f} s"
            for cam in played
        )
        print(f"  视频：{len(played)} 路已转码到 preview/（{detail}）")
    elif cams:
        print("  视频：有相机时间轴，但没有可播放的预览")
    for warning in payload["warnings"]:
        print(f"  [提示] {warning}")
    print("用浏览器打开这个文件即可回放：空格播放，左右方向键逐帧，点击曲线跳转")


if __name__ == "__main__":
    raise SystemExit(main())
