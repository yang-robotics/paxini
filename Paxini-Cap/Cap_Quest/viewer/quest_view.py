"""Read-only Quest 3 payload builder for the replay viewer.

The module does not align Quest samples with glove or camera rows. Quest
timestamps remain on their own SDK time base, and the viewer labels that
contract explicitly.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import h5py
import numpy as np


QUEST_ALIGNMENT = "none"
#: SDK 时间戳就是主机墙钟，直接对齐（与 align_capture.py 的默认模式一致）。
QUEST_ALIGNMENT_UNIX = "unix"
#: 经 C8 报告的时钟模型换算后再对齐。
QUEST_ALIGNMENT_SDK_UNIX = "sdk_unix"
QUEST_ALIGNMENT_ACTIVE = (QUEST_ALIGNMENT_UNIX, QUEST_ALIGNMENT_SDK_UNIX)
QUEST_ALIGNMENT_NOTE = "Quest 使用独立 SDK 时间轴，未做跨源时间对齐"
QUEST_ALIGNED_NOTE = "Quest 已按 Unix 墙钟对齐到手套首帧（未经 C8 验证）"
QUEST_ALIGNED_VERIFIED_NOTE = "Quest 已按 C8 验证的 Unix 时钟对齐到手套首帧"
QUEST_ALIGNMENT_CHOICES = (
    QUEST_ALIGNMENT,
    QUEST_ALIGNMENT_UNIX,
    QUEST_ALIGNMENT_SDK_UNIX,
)


def take_evenly(rows: int, limit: int) -> np.ndarray:
    """Return at most ``limit`` row indices, keeping first and last."""
    if rows <= 0:
        return np.zeros(0, dtype=np.int64)
    if rows <= limit:
        return np.arange(rows, dtype=np.int64)
    stride = max(1, math.ceil(rows / limit))
    result = np.arange(0, rows, stride, dtype=np.int64)
    if result[-1] != rows - 1:
        result = np.append(result, rows - 1)
    return result


def _round_values(values: np.ndarray, digits: int = 4) -> list[Any]:
    values = np.asarray(values, dtype=np.float64)
    rounded = np.round(values, digits)
    return [
        None if not np.isfinite(value) else float(value)
        for value in rounded.ravel()
    ]


def _round_rows(values: np.ndarray, digits: int = 4) -> list[list[Any]]:
    return [_round_values(row, digits) for row in values]


def _pose_block(
    handle: h5py.File,
    path: str,
    index: np.ndarray,
) -> dict[str, Any] | None:
    if path not in handle:
        return None
    block = handle[path][index].astype(np.float64)
    if block.ndim != 2 or block.shape[1] < 7:
        return None
    quat = block[:, 3:7]
    quat_norm = np.sqrt(np.sum(quat * quat, axis=1))
    tracked = (
        np.all(np.isfinite(block[:, :3]), axis=1)
        & np.all(np.isfinite(quat), axis=1)
        & (quat_norm > 0.5)
    )
    return {
        "xyz": _round_rows(block[:, :3], 4),
        "quat_xyzw": _round_rows(block[:, 3:7], 4),
        "tracked": [bool(value) for value in tracked],
    }


def _relative_ms(
    raw: np.ndarray,
    index: np.ndarray,
    base: int | None,
) -> list[Any]:
    values = raw[index].astype(np.int64)
    if base is None:
        return [None] * int(index.size)
    output: list[Any] = []
    for value in values:
        output.append(None if value <= 0 else round((int(value) - base) / 1e6, 3))
    return output


def _stamp_quality(raw: np.ndarray) -> tuple[int, int, int]:
    valid = raw[raw > 0].astype(np.int64)
    if valid.size < 2:
        return int(valid.size), 0, int(raw.size - valid.size)
    backwards = int(np.count_nonzero(np.diff(valid) < 0))
    return int(valid.size), backwards, int(raw.size - valid.size)


def _max_same_timestamp_run(raw: np.ndarray) -> int:
    maximum = 0
    current = 0
    previous = -1
    for value in raw:
        value = int(value)
        if value > 0 and value == previous:
            current += 1
        elif value > 0:
            current = 1
        else:
            current = 0
        maximum = max(maximum, current)
        previous = value
    return maximum


def load_clock_report(path: Path) -> dict[str, Any]:
    """Load and validate a passing C8 probe report."""
    try:
        payload = json.loads(path.expanduser().read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"无法读取 C8 时钟报告 {path}：{exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("C8 时钟报告顶层必须是对象")
    if payload.get("kind") != "quest_clock_probe":
        raise ValueError("C8 时钟报告类型不正确")
    if payload.get("passed") is not True:
        raise ValueError("C8 时钟报告未通过，禁止启用 sdk_unix 对齐")
    if payload.get("clock_domain") != "unix_epoch":
        raise ValueError(
            "C8 时钟报告未确认 SDK 时间属于 Unix epoch，禁止启用 sdk_unix 对齐"
        )
    model = (payload.get("models") or {}).get("host_unix")
    if not isinstance(model, dict):
        raise ValueError("C8 时钟报告缺少 host_unix 模型")
    try:
        slope = float(model["sdk_per_host_slope"])
        float(model["intercept_s"])
        int(model["reference_host_ns"])
        int(model["reference_sdk_ns"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("C8 时钟报告中的 host_unix 模型字段无效") from exc
    if not math.isfinite(slope) or slope <= 0:
        raise ValueError("C8 时钟报告的时钟斜率无效")
    return payload


def _sdk_to_unix_ns(
    sdk_ns: int,
    model: dict[str, Any],
) -> int:
    """Invert the fitted ``sdk = slope * host + intercept`` model."""
    reference_host_ns = int(model["reference_host_ns"])
    reference_sdk_ns = int(model["reference_sdk_ns"])
    slope = float(model["sdk_per_host_slope"])
    intercept_s = float(model["intercept_s"])
    host_relative_s = (
        ((sdk_ns - reference_sdk_ns) / 1e9) - intercept_s
    ) / slope
    return int(round(reference_host_ns + host_relative_s * 1e9))


def load_quest_view(
    handle: h5py.File,
    limit: int,
    *,
    pose_source: str = "controller",
    pose_side: str = "right",
    alignment: str = QUEST_ALIGNMENT,
    cap_unix_ns: int | None = None,
    clock_report: dict[str, Any] | None = None,
) -> dict[str, Any] | None:
    """Build a compact, JSON-friendly Quest visualization payload.

    ``None`` means the recording has no Quest rows. Missing SDK timestamps are
    represented as ``None``. If the timestamp series cannot form a usable time
    axis, the returned payload falls back to the relative polling sequence and
    says so explicitly.
    """
    if alignment not in QUEST_ALIGNMENT_CHOICES:
        raise ValueError(f"未知 Quest 对齐模式：{alignment!r}")
    if "quest" not in handle or "quest/sdk_timestamp_ns" not in handle:
        return None
    total = int(handle["quest/sdk_timestamp_ns"].shape[0])
    if total <= 0:
        return None

    index = take_evenly(total, max(16, int(limit)))
    sdk_raw = handle["quest/sdk_timestamp_ns"][:].astype(np.int64)
    valid_sdk = sdk_raw[sdk_raw > 0]
    sdk_base = int(valid_sdk[0]) if valid_sdk.size else None
    sdk_valid_count, sdk_backwards, sdk_missing = _stamp_quality(sdk_raw)
    sdk_valid_fraction = sdk_valid_count / total
    sdk_max_run = _max_same_timestamp_run(sdk_raw)
    sdk_usable = (
        sdk_valid_count >= 2
        and sdk_backwards == 0
        and int(np.max(valid_sdk)) > int(np.min(valid_sdk))
    )

    sequence_all = (
        handle["quest/sequence"][:].astype(np.int64)
        if "quest/sequence" in handle
        else np.arange(1, total + 1, dtype=np.int64)
    )
    sequence_gaps = (
        int(np.count_nonzero(np.diff(sequence_all) != 1))
        if sequence_all.size > 1
        else 0
    )
    sequence = sequence_all[index].astype(np.int64)

    alignment_mode = alignment
    alignment_note = QUEST_ALIGNMENT_NOTE
    alignment_clock_domain = None
    if alignment in QUEST_ALIGNMENT_ACTIVE:
        if not cap_unix_ns or cap_unix_ns <= 0:
            raise ValueError("Quest Unix 对齐需要手套首帧 Unix 时间")
        model = None
        if alignment == QUEST_ALIGNMENT_SDK_UNIX:
            if clock_report is None:
                raise ValueError("sdk_unix 对齐需要已验证的 C8 时钟报告")
            limits = clock_report.get("thresholds") or {}
            min_valid_fraction = float(limits.get("min_valid_fraction", 0.95))
            max_backwards = int(limits.get("max_backwards_steps", 0))
            max_run = int(limits.get("max_same_timestamp_run", 10))
            failures: list[str] = []
            if not sdk_usable:
                failures.append("SDK 时间戳没有可用的递增时间轴")
            if sdk_valid_fraction < min_valid_fraction:
                failures.append(
                    f"有效时间戳比例 {sdk_valid_fraction:.3f} 低于 {min_valid_fraction:.3f}"
                )
            if sdk_backwards > max_backwards:
                failures.append(f"时间戳倒退 {sdk_backwards} 次")
            if sdk_max_run > max_run:
                failures.append(
                    f"同一时间戳连续重复 {sdk_max_run} 次，超过 {max_run}"
                )
            if failures:
                raise ValueError(
                    "当前 data.h5 不满足 C8 对齐条件：" + "；".join(failures)
                )
            candidate = (clock_report.get("models") or {}).get("host_unix")
            if not isinstance(candidate, dict):
                raise ValueError("C8 时钟报告缺少 host_unix 模型")
            model = candidate
        # unix 模式下 model 保持 None：SDK 戳本身就是主机墙钟，不需要换算。
        time_values = []
        for value in sdk_raw[index]:
            value = int(value)
            if value <= 0:
                time_values.append(None)
                continue
            host_unix = _sdk_to_unix_ns(value, model) if model is not None else value
            time_values.append(round((host_unix - cap_unix_ns) / 1e9, 6))
        time_source = "sdk_timestamp_ns"
        time_label = "相对手套首帧时间 (s)"
        time_unit = "s"
        alignment_note = (
            QUEST_ALIGNED_NOTE
            if model is None
            else QUEST_ALIGNED_VERIFIED_NOTE
        )
        alignment_clock_domain = "unix_epoch"
    elif sdk_usable:
        time_values = _relative_ms(sdk_raw, index, sdk_base)
        time_source = "sdk_timestamp_ns"
        time_label = "SDK 相对时间 (ms)"
        time_unit = "ms"
    else:
        time_values = [int(value - sequence_all[0]) for value in sequence]
        time_source = "sequence"
        time_label = "轮询序号"
        time_unit = "sample"
        alignment_mode = "none"

    stamps: dict[str, list[Any]] = {}
    for name in ("sdk_timestamp_ns", "body_timestamp_ns", "motion_timestamp_ns"):
        path = f"quest/{name}"
        if path not in handle:
            continue
        raw = handle[path][:].astype(np.int64)
        valid = raw[raw > 0]
        base = int(valid[0]) if valid.size else None
        stamps[name] = _relative_ms(raw, index, base)

    main_pose = _pose_block(handle, "quest/pose_6d", index)
    head = _pose_block(handle, "quest/head/pose_xyzw", index)
    controllers: dict[str, Any] = {}
    for side in ("left", "right"):
        pose = _pose_block(handle, f"quest/controller_{side}/pose_xyzw", index)
        side_payload: dict[str, Any] = {"pose": pose}
        for field in ("grip", "trigger"):
            path = f"quest/controller_{side}/{field}"
            side_payload[field] = (
                _round_values(handle[path][index], 4)
                if path in handle
                else []
            )
        axis_path = f"quest/controller_{side}/axis"
        if axis_path in handle:
            axis = handle[axis_path][index].astype(np.float64)
            side_payload["axis"] = _round_rows(axis, 4)
        else:
            side_payload["axis"] = []
        controllers[side] = side_payload

    hand_counts: dict[str, list[int]] = {}
    for side in ("left", "right"):
        path = f"quest/hand_{side}/joint_count"
        hand_counts[side] = (
            [int(value) for value in handle[path][index]]
            if path in handle
            else [0] * int(index.size)
        )

    buttons: dict[str, list[bool]] = {}
    if "quest/buttons" in handle:
        for name, dataset in handle["quest/buttons"].items():
            buttons[str(name)] = [bool(value) for value in dataset[index]]

    tracked_all = (
        handle["quest/tracked"][:].astype(bool)
        if "quest/tracked" in handle
        else np.zeros(total, dtype=bool)
    )
    body_joint_quality: dict[str, int] | None = None
    if "quest/body_joints_timestamp_ns" in handle:
        body_stamps = handle["quest/body_joints_timestamp_ns"][:].astype(np.int64)
        body_joint_quality = {
            "valid": int(np.count_nonzero(body_stamps > 0)),
            "missing": int(np.count_nonzero(body_stamps <= 0)),
        }

    return {
        "rows": total,
        "shown": int(index.size),
        "stride": max(1, int(round(total / index.size))),
        "row_index": [int(value) for value in index],
        "sequence": [int(value) for value in sequence],
        "time": time_values,
        "time_source": time_source,
        "time_label": time_label,
        "time_unit": time_unit,
        "stamps": stamps,
        "tracked": [bool(value) for value in tracked_all[index]],
        "main_pose": main_pose,
        "head": head,
        "controllers": controllers,
        "hand_joint_counts": hand_counts,
        "buttons": buttons,
        "pose_source": pose_source,
        "pose_side": pose_side,
        "alignment": alignment_mode,
        "alignment_clock_domain": alignment_clock_domain,
        "alignment_note": alignment_note,
        "quality": {
            "sdk_valid_timestamps": sdk_valid_count,
            "sdk_missing_timestamps": sdk_missing,
            "sdk_backwards_steps": sdk_backwards,
            "sdk_max_same_timestamp_run": sdk_max_run,
            "sdk_valid_fraction": float(sdk_valid_fraction),
            "sequence_gaps": sequence_gaps,
            "tracked_rows": int(np.count_nonzero(tracked_all)),
            "body_joints": body_joint_quality,
        },
    }
