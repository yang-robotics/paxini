#!/usr/bin/env python3
"""Hardware-free self test for the PXCap III collector.

Covers the parts that are easy to get silently wrong:

1. callback frame parsing, including the "read by count, not by 119" rule;
2. Quest sample collection in ``quest3`` (four SDK timestamps; QuestPoller adds two host receive timestamps);
3. Quest row building, with the four timestamps and missing fields;
4. HDF5 write/read with two independent row axes (glove N, Quest Q);
5. the per-group row-consistency self check;
6. ``--omit-distribution`` mode;
7. the live-view hook, one push per recorded frame;
8. config file parsing and CLI precedence;
9. replay payload thinning, clock quality, and camera video offsets.

Alignment is deliberately not tested here: the collector does not align.

Run with ``python3.10 selftest.py``; exit status 0 means every case passed.
"""

from __future__ import annotations

import base64

import json
import math
import queue
import shutil
import sys
import tempfile
import threading
import time
import types
from datetime import datetime
from pathlib import Path
from typing import Any

import h5py
import numpy as np

# 自检单独成目录，被测模块分散在 ../capture 与 ../viewer，显式补上搜索路径。
_QUEST_ROOT = Path(__file__).resolve().parent.parent
for _sub in ("capture", "viewer", "diagnostics"):
    _module_dir = _QUEST_ROOT / _sub
    if str(_module_dir) not in sys.path:
        sys.path.insert(0, str(_module_dir))

from camera_ffmpeg import parse_rate
from collect_cap import (
    BUTTON_NAMES,
    CAP_MAX_POINTS,
    CAP_SLOT_POINTS,
    CAP_SLOTS,
    STOP_EVENT,
    Hdf5Recorder,
    RuntimeMonitor,
    SelectedPoseWatch,
    StartGate,
    TimedSample,
    TimedStream,
    apply_config,
    build_parser,
    load_config_file,
    pump_and_record,
    quest_sample_is_valid,
    quest_sample_to_row,
    resolve_ffmpeg,
    resolve_pose_side,
    snapshot_cap_frame,
    write_startup_failure,
)
from quest3 import QuestAdb, XrtReader, parse_mdns_services, pc_service_command
from quest_clock_probe import evaluate_clock_samples
from quest_view import (
    QUEST_ALIGNED_NOTE,
    QUEST_ALIGNED_VERIFIED_NOTE,
    QUEST_ALIGNMENT_NOTE,
    QUEST_ALIGNMENT_SDK_UNIX,
    QUEST_ALIGNMENT_UNIX,
    load_clock_report,
    load_quest_view,
)
from visualize import (
    build_preview,
    camera_tracks,
    clock_quality,
    collect_payload,
    prepare_videos,
    take_envelope,
    write_viewer,
)
import sensor_layout_schematic as schematic_layout

PASSED: list[str] = []
FAILED: list[str] = []
SKIPPED: list[str] = []


def check(name: str, condition: bool, detail: str = "") -> None:
    if condition:
        PASSED.append(name)
    else:
        FAILED.append(f"{name}{(': ' + detail) if detail else ''}")


def skip(name: str, reason: str) -> None:
    """Record a group that cannot run here (missing ffprobe and the like)."""
    SKIPPED.append(f"{name}：{reason}")


class FakeVector:
    __slots__ = ("x", "y", "z")

    def __init__(self, x: int, y: int, z: int) -> None:
        self.x, self.y, self.z = x, y, z


class FakeDistributeSlot:
    """Mirrors PxCap3SensorDistributeForceData: 119 slots, ``count`` valid."""

    def __init__(self, count: int) -> None:
        self.count = count
        self.values = [
            FakeVector(index - 60, 60 - index, index % 256)
            for index in range(CAP_MAX_POINTS)
        ]


class FakeStruct:
    def __init__(self, **fields: object) -> None:
        for key, value in fields.items():
            setattr(self, key, value)


def make_frame(
    unix_ns: int,
    monotonic_ns: int,
    angles: dict[str, float] | None = None,
) -> FakeStruct:
    angle_values = angles or {
        "index_proximal": 12.5,
        "index_distal": 3.25,
        "middle_proximal": -1.5,
        "middle_distal": 0.0,
    }
    return FakeStruct(
        timestamp_unix_ns=unix_ns,
        timestamp_monotonic_ns=monotonic_ns,
        encoder_raw=FakeStruct(
            index_proximal=101,
            index_distal=102,
            middle_proximal=103,
            middle_distal=104,
        ),
        joint_angles=FakeStruct(**angle_values),
        resultant_force=FakeStruct(
            **{
                slot: FakeVector(index, -index, index * 2)
                for index, slot in enumerate(CAP_SLOTS)
            }
        ),
        distribute_force=FakeStruct(
            **{
                slot: FakeDistributeSlot(CAP_SLOT_POINTS[slot])
                for slot in CAP_SLOTS
            }
        ),
    )


def make_pose(x: float, y: float, z: float) -> list[float]:
    return [x, y, z, 0.0, 0.0, 0.0, 1.0]


def make_quest_sample(sequence: int, base_ns: int = 1_700_000_000_000_000_000) -> dict:
    return {
        "sequence": sequence,
        "sdk_timestamp_ns": base_ns + sequence * 1_000_000,
        "body_timestamp_ns": base_ns + sequence * 1_000_000 + 10,
        "motion_timestamp_ns": base_ns + sequence * 1_000_000 + 20,
        "body_joints_timestamp_ns": [
            base_ns + sequence * 1_000_000 + 100 + index for index in range(24)
        ],
        # What QuestPoller adds around the SDK getter bundle.
        "host_unix_ns": base_ns + sequence * 1_000_000 + 1_000_000,
        "host_monotonic_ns": 6_000_000_000 + sequence * 1_000_000,
        "tracked": True,
        "head": {
            "available": True,
            "tracked": True,
            "pose_xyzw": make_pose(0.1, 1.6, -0.2),
        },
        "controllers": {
            "left": {
                "pose": {
                    "available": True,
                    "tracked": False,
                    "pose_xyzw": make_pose(-0.3, 1.1, 0.4),
                },
                "grip": 0.5,
                "trigger": None,
                "axis": [0.25, -0.25],
            },
            "right": {
                "pose": {
                    "available": True,
                    "tracked": True,
                    "pose_xyzw": make_pose(0.3, 1.2, 0.5),
                },
                "grip": 0.75,
                "trigger": 0.125,
                "axis": [0.5, 0.0],
            },
        },
        "hands": {
            "left": {"active": False, "joint_count": 0, "joints_pose_xyzw": None},
            "right": {
                "active": True,
                "joint_count": 2,
                "joints_pose_xyzw": [
                    make_pose(0.0, 0.0, 0.0),
                    make_pose(0.0, 0.1, 0.0),
                ],
            },
        },
        "buttons": {"A": True, "B": False},
    }


def test_frame_parsing() -> None:
    frame = make_frame(1_700_000_000_000_000_000, 5_000)
    sample = snapshot_cap_frame(frame, True)

    check(
        "帧解析/时间戳只有 SDK 给的两个",
        sample["timestamp_unix_ns"] == 1_700_000_000_000_000_000
        and sample["timestamp_monotonic_ns"] == 5_000
        and "received_monotonic_ns" not in sample,
        str(sorted(k for k in sample if "timestamp" in k or "received" in k)),
    )
    check(
        "帧解析/编码器",
        sample["encoder_raw"].dtype == np.int32
        and sample["encoder_raw"].tolist() == [101, 102, 103, 104],
    )
    check(
        "帧解析/关节角",
        sample["joint_angles_deg"].dtype == np.float64
        and sample["joint_angles_deg"].tolist() == [12.5, 3.25, -1.5, 0.0],
    )
    check(
        "帧解析/合力形状与数值",
        sample["resultant_force"].shape == (10, 3)
        and sample["resultant_force"].dtype == np.int16
        and sample["resultant_force"][9].tolist() == [9, -9, 18],
    )

    distribute = sample["distribute_force"]
    check(
        "帧解析/分布力形状",
        distribute is not None and distribute.shape == (10, CAP_MAX_POINTS, 3),
    )
    total = int(sample["distribute_counts"].sum())
    check("帧解析/444 点合计", total == 444, f"实际 {total}")
    check(
        "帧解析/每槽位有效点数",
        sample["distribute_counts"].tolist()
        == [CAP_SLOT_POINTS[slot] for slot in CAP_SLOTS],
    )
    check(
        "帧解析/按 count 裁掉尾部",
        int(distribute[1, 20:].sum()) == 0 and int(distribute[1, 19].sum()) != 0,
        "thumb_two 只应有前 20 点非零",
    )
    check(
        "帧解析/palm_zero 满 119 点",
        int((np.abs(distribute[8]).sum(axis=1) > 0).sum()) == 119,
    )
    check(
        "帧解析/x y 有符号、z 无符号",
        distribute[8, 0].tolist() == [-60, 60, 0]
        and distribute[8, 59].tolist() == [-1, 1, 59]
        and distribute[8, 118].tolist() == [58, -58, 118],
        "palm_zero（119 点）用来覆盖全部索引",
    )
    check(
        "帧解析/未知角度写 NaN",
        math.isnan(
            snapshot_cap_frame(
                make_frame(1, 2, {"index_proximal": math.nan,
                                  "index_distal": 1.0,
                                  "middle_proximal": 2.0,
                                  "middle_distal": 3.0}),
                True,
            )["joint_angles_deg"][0]
        ),
    )

    short_frame = make_frame(1_700_000_000_000_000_000, 5_000)
    short_frame.distribute_force.thumb_one = FakeDistributeSlot(49)
    short_frame.distribute_force.thumb_one.values = (
        short_frame.distribute_force.thumb_one.values[:30]
    )
    short = snapshot_cap_frame(short_frame, True)
    check(
        "帧解析/counts 不超过实际写入的点数",
        int(short["distribute_counts"][0]) == 30
        and int((np.abs(short["distribute_force"][0]).sum(axis=1) > 0).sum()) == 30,
        f"count={int(short['distribute_counts'][0])}",
    )

    omitted = snapshot_cap_frame(frame, False)
    check("帧解析/--omit-distribution 不建分布力", omitted["distribute_force"] is None)
    check(
        "帧解析/--omit-distribution 仍记有效点数",
        omitted["distribute_counts"].tolist()
        == [CAP_SLOT_POINTS[slot] for slot in CAP_SLOTS],
    )


def test_quest_row() -> None:
    sample = make_quest_sample(1)
    row = quest_sample_to_row(sample, "controller", "right")

    check(
        "Quest 行/四个 SDK 时间戳",
        row["quest/sdk_timestamp_ns"] == sample["sdk_timestamp_ns"]
        and row["quest/body_timestamp_ns"] == sample["body_timestamp_ns"]
        and row["quest/motion_timestamp_ns"] == sample["motion_timestamp_ns"]
        and row["quest/body_joints_timestamp_ns"].shape == (24,)
        and row["quest/body_joints_timestamp_ns"][0]
        == sample["body_joints_timestamp_ns"][0],
        str({k: v for k, v in row.items() if "timestamp" in k}),
    )
    check(
        "Quest 行/记主机接收时刻（对齐用，不是设备采样）",
        row["quest/host_unix_ns"] == sample["host_unix_ns"]
        and row["quest/host_monotonic_ns"] == sample["host_monotonic_ns"],
        str({k: v for k, v in row.items() if "host" in k}),
    )
    check("Quest 行/序号", row["quest/sequence"] == 1)
    check(
        "Quest 行/主 pose 取右手柄",
        row["quest/pose_6d"].tolist()[:3] == [0.3, 1.2, 0.5],
    )
    check(
        "Quest 行/头显位姿",
        row["quest/head/pose_xyzw"].tolist()[:3] == [0.1, 1.6, -0.2]
        and row["quest/head/tracked"] is True
        and row["quest/head/available"] is True,
    )
    check(
        "Quest 行/手柄 grip trigger axis",
        row["quest/controller_right/grip"] == 0.75
        and row["quest/controller_right/trigger"] == 0.125
        and row["quest/controller_right/axis"].tolist() == [0.5, 0.0],
    )
    check(
        "Quest 行/缺失 trigger 为 NaN",
        math.isnan(row["quest/controller_left/trigger"]),
    )
    check(
        "Quest 行/手部关节补齐到 26",
        row["quest/hand_right/joints_pose_xyzw"].shape == (26, 7)
        and row["quest/hand_right/joint_count"] == 2
        and not math.isnan(row["quest/hand_right/joints_pose_xyzw"][1, 1])
        and math.isnan(row["quest/hand_right/joints_pose_xyzw"][2, 0]),
    )
    check(
        "Quest 行/未激活手全 NaN",
        row["quest/hand_left/joint_count"] == 0
        and bool(np.all(np.isnan(row["quest/hand_left/joints_pose_xyzw"]))),
    )
    check(
        "Quest 行/按键",
        row["quest/buttons/A"] is True
        and all(
            row[f"quest/buttons/{name}"] is False
            for name in BUTTON_NAMES
            if name != "A"
        ),
    )
    check(
        "Quest 行/SDK 没给时间戳时写 -1",
        quest_sample_to_row(
            {**sample, "sdk_timestamp_ns": None}, "controller", "right"
        )["quest/sdk_timestamp_ns"]
        == -1,
    )
    check(
        "Quest 行/head 模式取头显",
        quest_sample_to_row(sample, "head", "right")["quest/pose_6d"].tolist()[:3]
        == [0.1, 1.6, -0.2],
    )
    check(
        "Quest 行/body_joints 不足 24 时补 -1",
        quest_sample_to_row(
            {**sample, "body_joints_timestamp_ns": [1, 2, 3]},
            "controller",
            "right",
        )["quest/body_joints_timestamp_ns"][3]
        == -1,
    )
    check(
        "Quest 行/不再有配对字段",
        "quest/available" not in row
        and "quest/skew_ns" not in row
        and not any(key.startswith("_") for key in row),
    )


class FakePoseXrt:
    """只有位姿接口，没有时间戳——对应 SDK 缺 getter 的情形。"""

    def get_headset_pose(self) -> list[float]:
        return [0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 1.0]

    def get_left_controller_pose(self) -> list[float]:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    def get_right_controller_pose(self) -> list[float]:
        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]


class FakeXrt(FakePoseXrt):
    """Minimal stand-in for the XRoboToolkit SDK object."""

    def get_time_stamp_ns(self) -> int:
        return 111

    def get_body_timestamp_ns(self) -> int:
        return 222

    def get_motion_timestamp_ns(self) -> int:
        return 333

    def get_body_joints_timestamp(self) -> list[int]:
        return [444, 555]


def test_quest3_sample() -> None:
    """Quest 采集在 quest3 里：四个 SDK 时间戳，没有任何主机侧时刻。"""
    reader = object.__new__(XrtReader)
    reader.xrt = FakeXrt()
    reader.errors = {}

    sample = reader.collect_sample(9)

    check("Quest 采集/sequence 原样带出", sample.get("sequence") == 9)
    check(
        "Quest 采集/四个 SDK 时间戳齐全",
        sample.get("sdk_timestamp_ns") == 111
        and sample.get("body_timestamp_ns") == 222
        and sample.get("motion_timestamp_ns") == 333
        and sample.get("body_joints_timestamp_ns") == [444, 555],
        repr(
            (
                sample.get("sdk_timestamp_ns"),
                sample.get("body_timestamp_ns"),
                sample.get("motion_timestamp_ns"),
                sample.get("body_joints_timestamp_ns"),
            )
        ),
    )
    check(
        "Quest 采集/没有主机侧 received_at",
        "received_at" not in sample,
        repr(sorted(sample)),
    )

    row = quest_sample_to_row(sample, "head", "left")
    check(
        "Quest 采集/时间戳直接进 HDF5 行",
        int(row["quest/sdk_timestamp_ns"]) == 111
        and int(row["quest/body_timestamp_ns"]) == 222
        and int(row["quest/motion_timestamp_ns"]) == 333
        and int(row["quest/body_joints_timestamp_ns"][0]) == 444
        and int(row["quest/body_joints_timestamp_ns"][1]) == 555
        and int(row["quest/body_joints_timestamp_ns"][2]) == -1,
    )

    reader.xrt = FakePoseXrt()
    empty = reader.collect_sample(10)
    check(
        "Quest 采集/SDK 不给时落 -1",
        int(quest_sample_to_row(empty, "head", "left")["quest/body_timestamp_ns"])
        == -1,
    )

    class BrokenXrt(FakeXrt):
        def get_time_stamp_ns(self) -> int:
            raise RuntimeError("PC Service 没了")

    broken = object.__new__(XrtReader)
    broken.xrt = BrokenXrt()
    broken.errors = {}
    sample = broken.collect_sample(11)
    check(
        "Quest 采集/getter 异常被吞并记进 errors（manifest 靠它溯源）",
        "get_time_stamp_ns" in broken.errors
        and sample.get("sdk_timestamp_ns") is None
        and int(
            quest_sample_to_row(sample, "head", "left")["quest/sdk_timestamp_ns"]
        )
        == -1,
        f"{sorted(broken.errors)} {sample.get('sdk_timestamp_ns')!r}",
    )


def test_camera_monotonic(tmp_root: Path) -> None:
    """The monotonic column is optional: written only when the caller has one."""
    from collect_cap import Hdf5Recorder

    base = 1_700_000_000_000_000_000

    # Caller has a monotonic clock: both columns
    mono_dir = tmp_root / "cam_mono"
    with Hdf5Recorder(mono_dir, "head", "right", "right", ("cam0",), True) as rec:
        info = rec.write_camera_timestamps(
            "cam0",
            [base, base + 33_000_000],
            monotonic_ns=[10_000_000_000, 10_033_000_000],
        )
    with h5py.File(mono_dir / "data.h5", "r") as f:
        group = f["cameras/cam0"]
        check(
            "相机单调钟/带单调钟时两列都在",
            "timestamp_unix_ns" in group and "timestamp_monotonic_ns" in group,
            list(group.keys()),
        )
        check(
            "相机单调钟/monotonic 内容",
            list(group["timestamp_monotonic_ns"][:])
            == [10_000_000_000, 10_033_000_000],
            str(list(group["timestamp_monotonic_ns"][:])),
        )
    check(
        "相机单调钟/带单调钟时 manifest 标记",
        info.get("monotonic_available") is True
        and info.get("first_timestamp_monotonic_ns") == 10_000_000_000,
        str(info),
    )

    # Caller has no monotonic clock: the dataset must not exist
    linux_dir = tmp_root / "cam_unix_only"
    with Hdf5Recorder(linux_dir, "head", "right", "right", ("cam0",), True) as rec:
        info = rec.write_camera_timestamps("cam0", [base, base + 33_000_000])
    with h5py.File(linux_dir / "data.h5", "r") as f:
        group = f["cameras/cam0"]
        check(
            "相机单调钟/无单调钟时不建该列",
            "timestamp_unix_ns" in group
            and "timestamp_monotonic_ns" not in group,
            list(group.keys()),
        )
    check(
        "相机单调钟/无单调钟时 manifest 标记",
        info.get("monotonic_available") is False,
        str(info),
    )

    dup_dir = tmp_root / "cam_duplicate"
    with Hdf5Recorder(dup_dir, "head", "right", "right", ("cam0",), True) as rec:
        info = rec.write_camera_timestamps("cam0", [base, base])
    check(
        "相机时间戳/重复时间戳计数写进 manifest 信息",
        info.get("duplicate_timestamps") == 1,
        str(info),
    )


class _FlushCounter:
    """Proxy around h5py.File that counts flush() calls."""

    def __init__(self, target: Any, calls: list[int]) -> None:
        self._target = target
        self._calls = calls

    def __getattr__(self, name: str) -> Any:
        return getattr(self._target, name)

    def flush(self, *args: Any, **kwargs: Any) -> Any:
        self._calls.append(1)
        return self._target.flush(*args, **kwargs)


def test_flush_per_batch(tmp_root: Path) -> None:
    """每批写盘后必须 flush，否则强杀（close() 不执行）会连元数据树一起丢。

    2026-11-17：采集端原本只在 close() 里 flush，8 份被强杀的采集因此整份作废
    （data.h5 只剩超级块，根组对象头从未落盘）。这里锁住两点：每满一批调一次
    flush，且未满一批的行不提前进文件（所以强杀最多丢最后那批）。
    """
    output_dir = tmp_root / "flush"
    calls: list[int] = []
    recorder = Hdf5Recorder(
        output_dir, "controller", "right", "right", (), True, flush_rows=5
    )
    recorder.__enter__()
    recorder.file = _FlushCounter(recorder.file, calls)  # type: ignore[assignment]

    base_unix_ns = 1_700_000_000_000_000_000
    for index in range(12):
        frame = make_frame(
            base_unix_ns + index * 8_000_000,
            6_300_000_000_000 + index * 8_000_000,
        )
        recorder.write_cap(snapshot_cap_frame(frame, True))

    row_flushes = len(calls)
    check(
        "落盘/每满一批 flush 一次",
        row_flushes >= 2,
        f"12 行 / 每批 5 行 → flush {row_flushes} 次",
    )
    check(
        "落盘/未满一批的 2 行留在内存（强杀最多丢最后一批）",
        recorder.cap_rows == 10,
        f"已落盘 {recorder.cap_rows} 行",
    )
    recorder.file = recorder.file._target  # type: ignore[union-attr]
    recorder.__exit__(None, None, None)


def test_emergency_save(tmp_root: Path) -> None:
    """看门狗 / 二次 Ctrl+C 的落盘：把余数写进去，且绝不并发写同一个文件。"""
    output_dir = tmp_root / "emergency"
    recorder = Hdf5Recorder(
        output_dir, "controller", "right", "right", (), True, flush_rows=5
    )
    recorder.__enter__()
    base_unix_ns = 1_700_000_000_000_000_000
    for index in range(12):
        recorder.write_cap(
            snapshot_cap_frame(
                make_frame(
                    base_unix_ns + index * 8_000_000,
                    6_300_000_000_000 + index * 8_000_000,
                ),
                True,
            )
        )
    saved = recorder.emergency_save(2.0)
    with h5py.File(output_dir / "data.h5", "r") as handle:
        rows = handle["cap/joint_angles_deg"].shape[0]
        mismatch = handle["cap/encoder_raw"].shape[0]
    check(
        "收尾/紧急落盘把未满一批的余数也写进去",
        saved and rows == 12 and mismatch == 12,
        f"落盘 {rows} 行（写入 12 行、批 5）",
    )
    check("收尾/紧急落盘可重复调用不报错", recorder.emergency_save(1.0) is True)

    locked = Hdf5Recorder(
        tmp_root / "emergency_lock", "controller", "right", "right", (), True, flush_rows=5
    )
    locked.__enter__()
    holding = threading.Event()
    release = threading.Event()

    def hold_lock() -> None:
        with locked._lock:  # noqa: SLF001 - the lock is the thing under test
            holding.set()
            release.wait(3.0)

    holder = threading.Thread(target=hold_lock, daemon=True)
    holder.start()
    holding.wait(2.0)
    started = time.monotonic()
    acquired = locked.emergency_save(0.3)
    elapsed = time.monotonic() - started
    release.set()
    holder.join(2.0)
    check(
        "收尾/主线程持锁时按超时放弃，不并发写",
        acquired is False and elapsed < 1.5,
        f"等待 {elapsed:.2f} s 后返回 {acquired}",
    )
    locked.__exit__(None, None, None)


def test_camera_timestamp_guards(tmp_root: Path) -> None:
    """相机两列时间戳必须同长、且各自单调，否则下游按帧号配对会整体错位。"""
    output_dir = tmp_root / "camguard"
    recorder = Hdf5Recorder(output_dir, "controller", "right", "right", ("cam0",), True)
    recorder.__enter__()
    mismatch = None
    try:
        recorder.write_camera_timestamps(
            "cam0", [1_700_000_000_000_000_000, 1_700_000_033_000_000_000],
            monotonic_ns=[6_300_000_000_000],
        )
    except RuntimeError as exc:
        mismatch = str(exc)
    check("相机/两列长度不等时拒绝写入", mismatch is not None and "长度必须相等" in mismatch,
          (mismatch or "没有抛错")[:40])

    unsorted = None
    try:
        recorder.write_camera_timestamps(
            "cam0",
            [1_700_000_000_000_000_000, 1_700_000_033_000_000_000],
            monotonic_ns=[6_300_000_033_000_000_000, 6_300_000_000_000_000_000],
        )
    except RuntimeError as exc:
        unsorted = str(exc)
    check("相机/单调钟列倒退时拒绝写入", unsorted is not None and "倒退" in unsorted,
          (unsorted or "没有抛错")[:40])

    info = recorder.write_camera_timestamps(
        "cam0", [1_700_000_000_000_000_000, 1_700_000_033_000_000_000]
    )
    check("相机/只给墙钟列时 monotonic_available 为 False",
          info["monotonic_available"] is False and info["frames"] == 2)
    recorder.__exit__(None, None, None)


def test_two_row_axes(tmp_root: Path) -> None:
    """手套 N 行、Quest Q 行，互不相等，各自独立。"""
    output_dir = tmp_root / "axes"
    cap_rows = 6
    quest_rows = 4
    base_unix_ns = 1_700_000_000_000_000_000

    recorder = Hdf5Recorder(
        output_dir,
        "controller",
        "right",
        "right",
        ("right_cam0", "right_cam1"),
        True,
    )
    recorder.__enter__()
    for index in range(cap_rows):
        frame = make_frame(
            base_unix_ns + index * 8_000_000,
            6_300_000_000_000 + index * 8_000_000,
        )
        recorder.write_cap(snapshot_cap_frame(frame, True))
    for index in range(quest_rows):
        recorder.write_quest(
            quest_sample_to_row(make_quest_sample(index), "controller", "right")
        )

    cam0 = [base_unix_ns + index * 40_000_000 for index in range(5)]
    recorder.write_camera_timestamps("right_cam0", cam0)

    verify = recorder.verify_rows()
    check(
        "两轴/行数自检通过",
        verify["row_mismatch"] == []
        and verify["cap_rows"] == cap_rows
        and verify["quest_rows"] == quest_rows,
        str(verify),
    )
    check(
        "两轴/Quest 行数不等于手套行数",
        verify["quest_rows"] != verify["cap_rows"],
    )

    broken = recorder.file["cap/encoder_raw"]
    broken.resize((cap_rows + 2, broken.shape[1]))
    after = recorder.verify_rows()
    check(
        "两轴/自检能发现 cap 错位并给出期望值",
        any(
            item["dataset"] == "cap/encoder_raw"
            and item["rows"] == cap_rows + 2
            and item["expected"] == cap_rows
            for item in after["row_mismatch"]
        ),
        str(after["row_mismatch"]),
    )
    broken.resize((cap_rows, broken.shape[1]))

    manifest = recorder.manifest({"rows": cap_rows})
    check(
        "两轴/format_version 由 manifest 升到 5",
        manifest.get("format_version") == 5,
        str(manifest.get("format_version")),
    )
    recorder.write_meta(manifest)
    recorder.__exit__(None, None, None)

    with h5py.File(output_dir / "data.h5", "r") as handle:
        check(
            "两轴/cap 主轴长度",
            handle["cap/timestamp_unix_ns"].shape == (cap_rows,),
        )
        check(
            "两轴/quest 主轴长度",
            handle["quest/sdk_timestamp_ns"].shape == (quest_rows,),
        )
        check(
            "两轴/quest 四列 SDK + 两列主机接收时刻都在",
            all(
                name in handle["quest"]
                for name in (
                    "sdk_timestamp_ns",
                    "body_timestamp_ns",
                    "motion_timestamp_ns",
                    "body_joints_timestamp_ns",
                    "host_unix_ns",
                    "host_monotonic_ns",
                )
            ),
        )
        check(
            "两轴/quest 时间戳回读正确",
            int(handle["quest/body_timestamp_ns"][1])
            == 1_700_000_000_000_000_000 + 1_000_000 + 10
            and handle["quest/body_joints_timestamp_ns"].shape
            == (quest_rows, 24),
        )
        check(
            "两轴/相机只有时间戳",
            "timestamp_unix_ns" in handle["cameras/right_cam0"]
            and "frame_index" not in handle["cameras/right_cam0"]
            and handle["cameras/right_cam0/timestamp_unix_ns"].shape == (5,),
        )
        check(
            "两轴/没有写任何配对结果",
            not any(
                key in handle["quest"]
                for key in ("available", "skew_ns")
            ),
        )
        check(
            "两轴/cap 数值",
            handle["cap/encoder_raw"][0].tolist() == [101, 102, 103, 104]
            and handle["cap/resultant_force"][0, 0].tolist() == [0, 0, 0],
        )
        check(
            "两轴/quest 数值",
            handle["quest/pose_6d"][0].tolist()[:3] == [0.3, 1.2, 0.5]
            and int(handle["quest/hand_right/joint_count"][0]) == 2,
        )
        check(
            "两轴/分布力 gzip 且形状正确",
            handle["cap/distribute_force"].compression == "gzip"
            and handle["cap/distribute_force"].shape
            == (cap_rows, len(CAP_SLOTS), CAP_MAX_POINTS, 3),
        )
        check(
            "两轴/meta 属性为 JSON 字符串且声明不做对齐",
            json.loads(handle["meta"].attrs["alignment"]).startswith("none")
            and handle["meta"].attrs["rows"] == "6"
            and int(handle["meta"].attrs["format_version"]) == 5,
            str(dict(handle["meta"].attrs)),
        )


def test_omit_distribution(tmp_root: Path) -> None:
    output_dir = tmp_root / "omit"
    recorder = Hdf5Recorder(
        output_dir,
        "controller",
        "right",
        "right",
        (),
        False,
    )
    recorder.__enter__()
    frame = make_frame(1_700_000_000_000_000_000, 0)
    recorder.write_cap(snapshot_cap_frame(frame, False))
    recorder.__exit__(None, None, None)
    with h5py.File(output_dir / "data.h5", "r") as handle:
        check(
            "--omit-distribution/无分布力数据集",
            "cap/distribute_force" not in handle,
        )
        check(
            "--omit-distribution/仍有有效点数",
            handle["cap/distribute_counts"].shape == (1, 10)
            and int(handle["cap/distribute_counts"][0].sum()) == 444,
        )
        check(
            "--omit-distribution/无 quest 相机组",
            "cameras" not in handle,
        )


def test_live_view_hook() -> None:
    """Every recorded frame must reach the live view exactly once.

    The view itself runs in a child process and is not exercised here; what
    this locks down is the capture-side plumbing: one ``push`` per
    ``write_cap``, and no push when ``--view`` is off.
    """
    import queue as queue_module
    import time

    class FakeRecorder:
        pose_source = "head"
        pose_side = "right"
        joint_capacity = 26
        quest_total = 0

        def __init__(self) -> None:
            self.hands: list[dict[str, int]] = []
            self.quests = 0

        def write_cap(self, hand: dict[str, int]) -> None:
            self.hands.append(hand)

        def write_quest(self, _row: dict[str, object]) -> None:
            self.quests += 1

        @property
        def cap_total(self) -> int:
            return len(self.hands)

    class FakeHand:
        def __init__(self, frames: Any) -> None:
            self.frames = frames

    class FakeQuest:
        def drain(self) -> list[object]:
            return []

    class FakeView:
        def __init__(self) -> None:
            self.pushed = 0
            self.dropped = 0

        def push(self, _hand: dict[str, int]) -> None:
            self.pushed += 1

    def run(view: FakeView | None) -> FakeRecorder:
        frames: queue_module.Queue[dict[str, int]] = queue_module.Queue()
        for index in range(5):
            frames.put({"index": index})
        recorder = FakeRecorder()
        STOP_EVENT.clear()
        try:
            pump_and_record(
                recorder,  # type: ignore[arg-type]
                FakeHand(frames),  # type: ignore[arg-type]
                FakeQuest(),  # type: ignore[arg-type]
                0,
                deadline=time.monotonic() + 0.35,
                view=view,
            )
        finally:
            STOP_EVENT.clear()
        return recorder

    view = FakeView()
    recorder = run(view)
    check(
        "实时视图/每帧推送一次",
        view.pushed == 5 and len(recorder.hands) == 5,
        f"push={view.pushed} rows={len(recorder.hands)}",
    )

    quiet = run(None)
    check(
        "实时视图/不开时不推送",
        quiet.hands and len(quiet.hands) == 5,
        f"rows={len(quiet.hands)}",
    )


def test_config_file(tmp_root: Path) -> None:
    """配置文件只提供默认值，命令行必须优先。"""
    path = tmp_root / "cfg.json"
    path.write_text(
        json.dumps(
            {
                "_注释": "以 _ 开头会被忽略",
                "camera_mode": "cam1",
                "hand_rate": "60",
                "no_quest": True,
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    config = load_config_file(path)
    check(
        "配置/忽略下划线注释",
        "_注释" not in config and len(config) == 3,
        str(config),
    )
    check("配置/文件不存在返回空", load_config_file(tmp_root / "nope.json") == {})

    yaml_path = tmp_root / "cfg.yaml"
    yaml_path.write_text(
        "# 整行注释\n"
        "camera_mode: cam0   # 行尾也算注释\n"
        "hand_rate: 90\n"
        "view: true\n"
        "ffmpeg: /opt/ffmpeg-7.0.2/ffmpeg\n",
        encoding="utf-8",
    )
    yaml_config = load_config_file(yaml_path)
    check(
        "配置/读 YAML 且注释被忽略",
        yaml_config
        == {
            "camera_mode": "cam0",
            "hand_rate": 90,
            "view": True,
            "ffmpeg": "/opt/ffmpeg-7.0.2/ffmpeg",
        },
        str(yaml_config),
    )

    parser = build_parser()
    apply_config(parser, config, path)
    args = parser.parse_args([])
    check(
        "配置/文件值生效且按类型转换",
        args.camera_mode == "cam1"
        and args.hand_rate == 60
        and args.no_quest is True,
        f"{args.camera_mode} {args.hand_rate!r} {args.no_quest!r}",
    )
    overridden = parser.parse_args(["--camera-mode", "both", "--hand-rate", "200"])
    check(
        "配置/命令行覆盖文件",
        overridden.camera_mode == "both" and overridden.hand_rate == 200,
    )

    view_parser = build_parser()
    apply_config(
        view_parser,
        {"view": True, "view_fps": 15, "view_video_fps": 8},
        path,
    )
    view_args = view_parser.parse_args([])
    check(
        "配置/实时视图可默认打开",
        view_args.view is True
        and view_args.view_fps == 15.0
        and view_args.view_video_fps == 8.0,
        f"{view_args.view!r} {view_args.view_fps!r} {view_args.view_video_fps!r}",
    )
    check(
        "配置/实时视图命令行可关掉",
        view_parser.parse_args(["--no-view"]).view is False,
    )

    ffmpeg_parser = build_parser()
    apply_config(ffmpeg_parser, yaml_config, yaml_path)
    ffmpeg_args = ffmpeg_parser.parse_args([])
    check(
        "配置/ffmpeg 路径可写在文件里",
        ffmpeg_args.ffmpeg == "/opt/ffmpeg-7.0.2/ffmpeg",
        ffmpeg_args.ffmpeg,
    )
    check(
        "配置/ffmpeg 命令行覆盖文件",
        ffmpeg_parser.parse_args(["--ffmpeg", "/usr/local/bin/ffmpeg"]).ffmpeg
        == "/usr/local/bin/ffmpeg",
    )

    gate_parser = build_parser()
    apply_config(
        gate_parser,
        {"no_wait": True, "quest_ready_samples": 5},
        path,
    )
    gate_args = gate_parser.parse_args([])
    check(
        "配置/启动闸门参数可写文件",
        gate_args.no_wait is True and gate_args.quest_ready_samples == 5,
        f"no_wait={gate_args.no_wait!r} "
        f"ready={gate_args.quest_ready_samples!r}",
    )
    check(
        "配置/--no-wait 命令行可打开",
        build_parser().parse_args(["--no-wait"]).no_wait is True,
    )

    for bad_config, label in (
        ({"camera_mode": "video"}, "非法选项值"),
        ({"nonexistent_key": 1}, "未知键"),
        ({"no_quest": "yes"}, "布尔写成字符串"),
    ):
        raised = False
        try:
            apply_config(build_parser(), bad_config, path)
        except RuntimeError:
            raised = True
        check(f"配置/{label}报错", raised)


def test_pose_side_selection(tmp_root: Path) -> None:
    """One glove side selects Quest by default; explicit side still wins."""
    parser = build_parser()
    check(
        "手别/默认自动跟随",
        parser.parse_args([]).pose_side == "auto",
        parser.parse_args([]).pose_side,
    )
    check(
        "手别/auto 跟随左手",
        resolve_pose_side("left", "auto") == "left",
    )
    check(
        "手别/auto 跟随右手",
        resolve_pose_side("right", "auto") == "right",
    )
    check(
        "手别/显式覆盖",
        resolve_pose_side("left", "right") == "right",
    )

    path = tmp_root / "side.yaml"
    path.write_text(
        "cap_side: left\npose_side: auto\n",
        encoding="utf-8",
    )
    config_parser = build_parser()
    apply_config(config_parser, load_config_file(path), path)
    configured = config_parser.parse_args([])
    check(
        "手别/配置左手自动跟随",
        configured.cap_side == "left"
        and configured.pose_side == "auto"
        and resolve_pose_side(configured.cap_side, configured.pose_side)
        == "left",
        f"{configured.cap_side} {configured.pose_side}",
    )

    overridden = config_parser.parse_args(
        ["--cap-side", "right", "--pose-side", "left"]
    )
    check(
        "手别/命令行独立覆盖",
        overridden.cap_side == "right" and overridden.pose_side == "left",
        f"{overridden.cap_side} {overridden.pose_side}",
    )


def test_startup_failure(tmp_root: Path) -> None:
    """Failed startup keeps diagnostics without creating a misleading HDF5."""
    from camera_ffmpeg import FfmpegCamera

    output_dir = tmp_root / "startup_failure_abandoned"
    failure_dir = tmp_root / "_startup_failures" / "20261127_120000_camera"
    camera_dir = output_dir / "cameras"
    camera = FfmpegCamera(
        "left_cam0",
        "/dev/video4",
        camera_dir,
        640,
        480,
        15.0,
        "copy",
        "/usr/bin/ffmpeg",
    )
    camera_dir.mkdir(parents=True)
    camera.log_path.write_text(
        "Could not open video device\nNo frames received\n",
        encoding="utf-8",
    )
    failure = write_startup_failure(
        failure_dir,
        RuntimeError("等待摄像头画面超时"),
        started_at=datetime.now().astimezone(),
        cap_side="left",
        pose_source="controller",
        pose_side="left",
        pose_side_requested="auto",
        hand_port="/dev/ttyACM0",
        camera_devices={"cam0": "/dev/video4"},
        cameras=[camera],
        ffmpeg="/usr/bin/ffmpeg",
    )
    payload = json.loads(failure.read_text(encoding="utf-8"))
    camera_payload = payload["cameras"]["left_cam0"]
    check(
        "启动失败/明确标记状态",
        payload["status"] == "startup_failed"
        and payload["hand_port"] == "/dev/ttyACM0"
        and payload["cap_side"] == "left"
        and payload["pose"]["side"] == "left"
        and payload["pose"]["side_requested"] == "auto",
        str(payload),
    )
    check(
        "启动失败/不伪造 data.h5",
        not (output_dir / "data.h5").exists(),
    )
    check(
        "启动失败/保留命令和日志到 _startup_failures",
        "v4l2" in (camera_payload["command"] or [])
        and "No frames received" in camera_payload["log_tail"]
        and failure.name == "startup_failure.json"
        and failure.parent == failure_dir
        and (failure_dir / "cameras" / "left_cam0.log").is_file()
        and camera_payload["log_copy_error"] is None,
        str(camera_payload),
    )
    check(
        "启动失败/预录 mkv 不保留、路径标 removed",
        camera_payload["video"] is None
        and camera_payload["video_removed"] is True
        and not (failure_dir / "cameras" / "left_cam0.mkv").exists(),
        str(camera_payload),
    )


def test_camera_command(tmp_root: Path) -> None:
    """The V4L2 command line, progress parsing, and explicit device nodes."""
    from camera_ffmpeg import FfmpegCamera
    from collect_cap import resolve_camera_devices

    cam = FfmpegCamera(
        "cam0", "/dev/video0", tmp_root, 1280, 720, 30.0, "copy",
        "/usr/bin/ffmpeg",
    )
    cmd = cam.command()
    check(
        "相机命令/用 v4l2 与墙钟",
        "v4l2" in cmd
        and "-use_wallclock_as_timestamps" in cmd
        and "/dev/video0" in cmd
        # 不能带 -nostdin：ffmpeg 7 下它会吞掉第一次 SIGINT，容器收不了尾。
        and "-nostdin" not in cmd,
        " ".join(cmd[:6]),
    )
    check(
        "相机命令/copyts 保留容器 pts",
        "-copyts" in cmd and "-fps_mode" in cmd and "passthrough" in cmd,
        " ".join(cmd[-8:]),
    )
    cam.progress_path.parent.mkdir(parents=True, exist_ok=True)
    cam.progress_path.write_text(
        "total_size=262144\nout_time_us=1000000\nprogress=continue\n",
        encoding="utf-8",
    )
    progress_without_frame = cam.progress_signal()
    cam.progress_path.write_text(
        "frame=7\ntotal_size=524288\nout_time_us=2000000\nprogress=continue\n",
        encoding="utf-8",
    )
    check(
        "相机命令/stream-copy 无 frame 时按出流时间判进度",
        progress_without_frame == 30 and cam.progress_signal() == 7,
        f"fallback={progress_without_frame} frame={cam.progress_signal()}",
    )

    args = build_parser().parse_args(["--camera-0", "/dev/video2"])
    devices = resolve_camera_devices(args, ("cam0",), [])
    check(
        "相机命令/显式设备节点保留路径",
        devices["cam0"] == "/dev/video2",
        devices["cam0"],
    )

    class _InterruptedProcess:
        """Fake ffmpeg that finalises on SIGINT and therefore exits 255."""

        def __init__(self) -> None:
            self.returncode: int | None = None

        def poll(self) -> int | None:
            return self.returncode

        def send_signal(self, _sig: int) -> None:
            pass

        def wait(self, timeout: float | None = None) -> int:
            self.returncode = 255
            return self.returncode

    cam.process = _InterruptedProcess()
    cam.stop()
    check(
        "相机命令/SIGINT 收尾的退出码 255 不算失败",
        cam.failure is None,
        str(cam.failure),
    )


def test_visualize_video(tmp_root: Path) -> None:
    """Camera track offsets and the video transcode failure path.

    The browser panel trusts ``offset_s`` blindly, so a sign error or a wrong
    reference clock would silently shift every clip by the wrong amount.
    """
    out = tmp_root / "viz_video"
    (out / "cameras").mkdir(parents=True, exist_ok=True)
    glove_unix = 1_700_000_000_000_000_000
    count = 300
    stamps = glove_unix - 1_340_000_000 + np.arange(count, dtype=np.int64) * 33_366_667
    with h5py.File(out / "data.h5", "w") as handle:
        handle.create_group("cap")["timestamp_unix_ns"] = np.array([glove_unix])
        cam = handle.create_group("cameras/right_cam0")
        cam["timestamp_unix_ns"] = stamps

    handle = h5py.File(out / "data.h5", "r")
    tracks = camera_tracks(handle, out, glove_unix)
    handle.close()
    check("视频/识别出一路相机", len(tracks) == 1, str(tracks))
    if not tracks:
        return
    track = tracks[0]
    check(
        "视频/首帧偏移带符号且相对手套首帧",
        abs(track["offset_s"] + 1.34) < 1e-6,
        str(track["offset_s"]),
    )
    check("视频/帧数与 h5 一致", track["frames"] == count, str(track["frames"]))
    check(
        "视频/帧率由时间轴算出",
        29.5 <= track["fps"] <= 30.5,
        str(track["fps"]),
    )
    check("视频/mkv 缺失时置空", track["mkv"] is None, str(track["mkv"]))

    # A missing mkv must be reported, not raised: the html still has to build.
    error = build_preview(track, out / "preview", 320, 28, "/nonexistent/ffmpeg")
    check("视频/缺 mkv 时给出原因", error == "找不到 mkv", str(error))

    (out / "cameras" / "right_cam0.mkv").write_bytes(b"not really a video")
    handle = h5py.File(out / "data.h5", "r")
    track = camera_tracks(handle, out, glove_unix)[0]
    handle.close()
    check("视频/mkv 存在时给路径", bool(track["mkv"]), str(track["mkv"]))

    payload = {"video": {"cams": [track]}, "warnings": []}
    args = types.SimpleNamespace(ffmpeg="/nonexistent/ffmpeg", video_width=320, video_crf=28)
    prepare_videos(payload, out, args)
    check("视频/ffmpeg 缺失不抛异常", track["file"] is None, str(track["file"]))
    check("视频/ffmpeg 缺失写入警告", bool(track["error"]), str(track["error"]))
    check(
        "视频/转码失败进 warning 列表",
        any("转码失败" in text for text in payload["warnings"]),
        str(payload["warnings"]),
    )


def test_camera_preview(tmp_root: Path) -> None:
    """Live preview: the MJPEG splitter and the extra ffmpeg output.

    The splitter runs on a raw pipe with no framing, so a bug here shows up as
    a frozen or garbage thumbnail, never as an exception.
    """
    from camera_ffmpeg import FfmpegCamera, extract_jpeg_frames

    frame_a = b"\xff\xd8AAA\xff\xd9"
    frame_b = b"\xff\xd8BBBB\xff\xd9"
    buffer = bytearray(b"junk" + frame_a + frame_b + b"\xff\xd8PART")
    frames = extract_jpeg_frames(buffer)
    check("实时预览/拆出两个完整 JPEG", frames == [frame_a, frame_b], str(frames))
    check("实时预览/半个帧留在缓冲区", bytes(buffer) == b"\xff\xd8PART", bytes(buffer))
    check(
        "实时预览/拆干净后缓冲区为空",
        extract_jpeg_frames(bytearray(frame_a)) == [frame_a],
    )
    leftover = bytearray(b"\x00\x01noise")
    check(
        "实时预览/没有 SOI 的垃圾被丢掉",
        extract_jpeg_frames(leftover) == [] and not leftover,
        str(leftover),
    )

    quiet = FfmpegCamera(
        "cam0", "/dev/video0", tmp_root, 1280, 720, 30.0, "copy",
        "/usr/bin/ffmpeg",
    )
    check(
        "实时预览/关掉时没有第二路输出",
        "image2pipe" not in quiet.command() and "pipe:1" not in quiet.command(),
        " ".join(quiet.command()),
    )

    live = FfmpegCamera(
        "cam0", "/dev/video0", tmp_root, 1280, 720, 30.0, "copy",
        "/usr/bin/ffmpeg",
        preview_fps=5.0, preview_width=360,
    )
    command = live.command()
    check(
        "实时预览/开启后多一路 image2pipe",
        "image2pipe" in command and "pipe:1" in command,
        " ".join(command[-8:]),
    )
    check(
        "实时预览/抽帧与缩放写进 -vf",
        any("fps=5,scale=360:-2" in part for part in command),
        " ".join(command),
    )
    check(
        "实时预览/主输出仍在（mkv 路径没被顶掉）",
        any(part.endswith(".mkv") for part in command),
        " ".join(command),
    )
    check(
        "实时预览/预览不占默认值",
        FfmpegCamera(
            "cam0", "/dev/video0", tmp_root, 1280, 720, 30.0, "copy",
            "/usr/bin/ffmpeg",
        ).preview_fps == 0.0,
    )


def test_wireless_adb() -> None:
    """mDNS discovery and pair-then-connect flow, without touching a device."""
    text = (
        "List of discovered mdns services\n"
        "adb-1\t_adb-tls-connect._tcp\t192.168.1.5:41234\n"
        "adb-2\t_adb-tls-pairing._tcp\t192.168.1.5:37123\n"
        "garbage line\n"
    )
    parsed = parse_mdns_services(text)
    check(
        "无线/解析 connect 服务",
        parsed["connect"] == [("192.168.1.5", 41234)],
        str(parsed),
    )
    check(
        "无线/解析 pairing 服务",
        parsed["pairing"] == [("192.168.1.5", 37123)],
        str(parsed),
    )
    check(
        "无线/忽略非 adb-tls 行",
        parse_mdns_services("List of discovered mdns services")
        == {"connect": [], "pairing": []},
    )

    parser = build_parser()
    check(
        "无线/collect_cap 有 --quest-wireless",
        parser.parse_args(["--quest-wireless"]).quest_wireless is True,
    )
    check(
        "无线/--quest-wireless 默认关闭",
        parser.parse_args([]).quest_wireless is False,
    )

    class FakeWireless(QuestAdb):
        def __init__(self) -> None:
            super().__init__("adb", Path("/nonexistent.apk"), "pkg", 5555)
            self.commands: list[tuple[str, ...]] = []
            self.paired = False

        def mdns_services(self) -> dict[str, list[tuple[str, int]]]:
            if self.paired:
                return {"connect": [("10.0.0.2", 41000)], "pairing": []}
            return {"connect": [], "pairing": [("10.0.0.2", 37000)]}

        def connect_wireless_candidate(
            self, host: str, port: int, wait_seconds: float = 4.0
        ) -> str | None:
            if self.paired and port == 41000:
                return f"{host}:{port}"
            return None

        def run(self, args: Any, **kwargs: Any) -> Any:
            self.commands.append(tuple(args))
            if args[0] == "pair":
                self.paired = True
                return types.SimpleNamespace(
                    returncode=0, stdout="Successfully paired", stderr=""
                )
            return types.SimpleNamespace(returncode=0, stdout="", stderr="")

        def _save_ip(self, ip: str) -> None:
            self.saved_ip = ip

    adapter = FakeWireless()
    serial = adapter.pair_and_connect(ask=lambda prompt: "123456")
    check("无线/配对后拿到连接串", serial == "10.0.0.2:41000", str(serial))
    check(
        "无线/adb pair 命令正确",
        ("pair", "10.0.0.2:37000", "123456") in adapter.commands,
        str(adapter.commands),
    )

    class FakeChoose(QuestAdb):
        def __init__(self) -> None:
            super().__init__("adb", Path("/nonexistent.apk"), "pkg", 5555)
            self.prepared: str | None = None

        def devices(self) -> dict[str, str]:
            return {"USB123": "device"}

        def mdns_services(self) -> dict[str, list[tuple[str, int]]]:
            return {"connect": [], "pairing": []}

        def pair_and_connect(self, *args: Any, **kwargs: Any) -> str:
            raise AssertionError("USB 已可用时不应提示配对")

        def _prepare_selected_device(
            self,
            serial: str,
            requested_ip: str | None,
            allow_usb_only: bool,
            keep_usb_adb: bool = False,
        ) -> Any:
            self.prepared = serial
            return "USB_FALLBACK"

    chooser = FakeChoose()
    result = chooser.choose_device(
        requested_serial=None,
        requested_ip=None,
        saved_ip=None,
        allow_usb_only=False,
        keep_usb_adb=True,
        prefer_wireless=True,
    )
    check(
        "无线/有 USB 且无无线服务时不提示、回退 USB",
        result == "USB_FALLBACK" and chooser.prepared == "USB123",
        f"{result!r} {chooser.prepared!r}",
    )


def test_misc() -> None:
    check("杂项/ffprobe 帧率解析", parse_rate("30/1") == 30.0 and parse_rate("x") == 0.0)
    parser = build_parser()
    check(
        "杂项/--ffmpeg 参数存在且默认 auto",
        parser.parse_args([]).ffmpeg == "auto",
    )
    check(
        "杂项/--ffmpeg 可覆盖路径",
        parser.parse_args(["--ffmpeg", "/x/ffmpeg"]).ffmpeg == "/x/ffmpeg",
    )
    check(
        "杂项/ffmpeg auto 表示查环境变量与 PATH",
        resolve_ffmpeg("auto") is None
        and resolve_ffmpeg(None) is None
        and resolve_ffmpeg("/x/ffmpeg") == "/x/ffmpeg",
    )
    service_script = Path("/opt/apps/roboticsservice/runService.sh")
    service_argv = pc_service_command(service_script)
    check(
        "杂项/PC Service 经 bash 启动",
        len(service_argv) == 2
        and Path(service_argv[0]).name == "bash"
        and service_argv[1] == str(service_script),
        str(service_argv),
    )
    check(
        "杂项/阈值参数已移除",
        not any("--max-skew-ms" in action.option_strings for action in parser._actions)
        and not any(
            "--camera-max-skew-ms" in action.option_strings
            for action in parser._actions
        ),
    )


def test_visualize(tmp_root: Path) -> None:
    """Replay payload: peak-preserving thinning, clock drift, empty-glove flags."""
    rows = 1200
    smooth = np.sin(np.linspace(0.0, 6.0, rows))
    index = take_envelope(rows, 300, [smooth])
    check("可视化/保峰抽稀不超预算", index.size <= 300, f"实际 {index.size}")
    check("可视化/保峰抽稀索引严格递增", bool(np.all(np.diff(index) > 0)))

    spike = np.zeros(rows)
    spike[731] = 99.0
    thin = take_envelope(rows, 200, [spike])
    check(
        "可视化/保峰抽稀保住单帧尖峰",
        731 in set(int(v) for v in thin),
        f"共 {thin.size} 点",
    )
    check(
        "可视化/短录制原样返回",
        np.array_equal(take_envelope(50, 4000, [np.zeros(50)]), np.arange(50)),
    )

    out = tmp_root / "viz"
    out.mkdir(parents=True, exist_ok=True)
    count = 400
    mono = 6_300_000_000_000 + np.arange(count, dtype=np.int64) * 8_333_333
    unix = 1_700_000_000_000_000_000 + (mono - mono[0])
    unix[count // 2:] += 2_201_000_000  # one 2.2 s wall-clock step mid-run
    with h5py.File(out / "data.h5", "w") as handle:
        group = handle.create_group("cap")
        group["timestamp_unix_ns"] = unix
        group["timestamp_monotonic_ns"] = mono
        group["joint_angles_deg"] = np.zeros((count, 4))
        group["resultant_force"] = np.zeros((count, len(CAP_SLOTS), 3), dtype="int16")
        group["distribute_counts"] = np.full(
            (count, len(CAP_SLOTS)), 30, dtype="uint32"
        )
        group["distribute_force"] = np.zeros(
            (count, len(CAP_SLOTS), CAP_MAX_POINTS, 3), dtype="int16"
        )

    quality = clock_quality(h5py.File(out / "data.h5", "r"), count)
    check(
        "可视化/时钟跳变被统计",
        quality is not None and quality["steps_over_10ms"] == 1,
        str(quality),
    )
    check(
        "可视化/跳变幅度正确",
        quality is not None and 2000.0 <= quality["max_step_ms"] <= 2400.0,
        str(quality),
    )

    payload = collect_payload(out, 4000, 200, False)
    meta = payload["meta"]
    check("可视化/空负载被标记", meta["force_all_zero"] is True)
    check(
        "可视化/空负载给出警告",
        any("全为 0" in text for text in payload["warnings"]),
        str(payload["warnings"]),
    )
    check("可视化/未抽稀时标记 none", meta["thinning"] == "none", str(meta["thinning"]))
    check("可视化/载荷含四路关节角标签", len(payload["cap"]["angle_labels"]) == 4)


def test_visualize_distribution(tmp_root: Path) -> None:
    """Distribution scatter: counts masking, 0.01 N packing, HTML card."""
    out = tmp_root / "viz_dist"
    out.mkdir(parents=True, exist_ok=True)
    count = 12
    mono = 6_400_000_000_000 + np.arange(count, dtype=np.int64) * 8_333_333
    unix = 1_700_100_000_000_000_000 + (mono - mono[0])
    counts = np.zeros((count, len(CAP_SLOTS)), dtype="uint32")
    counts[:, 0] = 3
    force = np.zeros((count, len(CAP_SLOTS), CAP_MAX_POINTS, 3), dtype="int16")
    # frame 0, slot 0: valid points 0..2; point 3 is padding and must be masked.
    force[0, 0, 0] = (10, 0, 0)    # 1.0 N
    force[0, 0, 1] = (0, 0, 20)    # 2.0 N
    force[0, 0, 2] = (0, 0, 0)     # valid, but zero force
    force[0, 0, 3] = (100, 100, 100)  # padding: must be zeroed by counts
    with h5py.File(out / "data.h5", "w") as handle:
        group = handle.create_group("cap")
        group["timestamp_unix_ns"] = unix
        group["timestamp_monotonic_ns"] = mono
        group["joint_angles_deg"] = np.zeros((count, 4))
        group["resultant_force"] = np.zeros(
            (count, len(CAP_SLOTS), 3), dtype="int16"
        )
        group["distribute_counts"] = counts
        group["distribute_force"] = force

    payload = collect_payload(out, 4000, 60, False)
    slot = payload["slot_force"]
    dist = slot["dist"]
    check("可视化分布力/载荷存在", dist is not None)
    if dist is None:
        return
    check(
        "可视化分布力/形状",
        dist["shape"] == [len(CAP_SLOTS), CAP_MAX_POINTS],
        str(dist["shape"]),
    )
    check(
        "可视化分布力/槽位容量",
        dist["capacities"] == [CAP_SLOT_POINTS[s] for s in CAP_SLOTS],
        str(dist["capacities"]),
    )
    points = dist.get("points") or []
    check(
        "可视化分布力/点位数量",
        len(points) == len(CAP_SLOTS)
        and all(len(points[i]) == CAP_SLOT_POINTS[s] for i, s in enumerate(CAP_SLOTS)),
        str([len(row) for row in points]),
    )
    check(
        "可视化分布力/点位在画布内",
        all(0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 for row in points for x, y in row),
    )
    raw = base64.b64decode(dist["data"])
    expected = slot["frames"] * len(CAP_SLOTS) * CAP_MAX_POINTS * 2
    check("可视化分布力/字节长度", len(raw) == expected, f"{len(raw)} != {expected}")
    values = np.frombuffer(raw, dtype="<u2").reshape(
        slot["frames"], len(CAP_SLOTS), CAP_MAX_POINTS
    )
    check(
        "可视化分布力/非零值量化",
        values[0, 0, 0] == 100 and values[0, 0, 1] == 200,
        str(values[0, 0, :4]),
    )
    check(
        "可视化分布力/补齐点被清零",
        values[0, 0, 3] == 0,
        str(values[0, 0, :5]),
    )
    check(
        "可视化分布力/固定25N色标",
        dist["scale_n"] == 25.0,
        str(dist["scale_n"]),
    )

    tangent = dist.get("tangent")
    check("可视化分布力/切向数据存在", tangent is not None)
    if tangent is not None:
        tbytes = base64.b64decode(tangent)
        expected_t = slot["frames"] * len(CAP_SLOTS) * CAP_MAX_POINTS * 2
        check(
            "可视化分布力/切向字节长度",
            len(tbytes) == expected_t,
            f"{len(tbytes)} != {expected_t}",
        )
        tvals = np.frombuffer(tbytes, dtype=np.int8).reshape(
            slot["frames"], len(CAP_SLOTS), CAP_MAX_POINTS, 2
        )
        check(
            "可视化分布力/切向量值",
            tuple(tvals[0, 0, 0]) == (10, 0)
            and tuple(tvals[0, 0, 1]) == (0, 0),
            str(tvals[0, 0, :3]),
        )
        check(
            "可视化分布力/切向补齐点清零",
            tuple(tvals[0, 0, 3]) == (0, 0),
            str(tvals[0, 0, :5]),
        )
    check(
        "可视化分布力/旋转列表",
        dist["rotations"] == [0, 90, 0, 90, 0, 90, 0, 90, 90, 90],
        str(dist["rotations"]),
    )
    check(
        "可视化分布力/翻转列表",
        dist["flips"]
        == ["v", "v", "h", "h", "h", "h", "h", "h", "h", "v"],
        str(dist["flips"]),
    )
    check(
        "可视化分布力/切向色标",
        0.9 <= dist["tangent_scale_n"] <= 1.0,
        str(dist["tangent_scale_n"]),
    )
    no_arrows = collect_payload(out, 4000, 60, False, with_dist_arrows=False)
    check(
        "可视化分布力/关闭箭头时无切向数据",
        no_arrows["slot_force"]["dist"]["tangent"] is None,
    )

    html_path = out / "viewer.html"
    write_viewer(payload, html_path)
    html = html_path.read_text(encoding="utf-8")
    check(
        "可视化分布力/HTML 含卡片",
        'id="distCard"' in html and 'id="distCanvas"' in html,
    )
    check("可视化分布力/HTML payload 已替换", "__PAYLOAD__" not in html)

    omitted = tmp_root / "viz_dist_omitted"
    omitted.mkdir(parents=True, exist_ok=True)
    with h5py.File(omitted / "data.h5", "w") as handle:
        group = handle.create_group("cap")
        group["timestamp_unix_ns"] = unix
        group["timestamp_monotonic_ns"] = mono
        group["joint_angles_deg"] = np.zeros((count, 4))
        group["resultant_force"] = np.zeros(
            (count, len(CAP_SLOTS), 3), dtype="int16"
        )
        group["distribute_counts"] = counts
    omitted_payload = collect_payload(omitted, 4000, 60, False)
    omitted_dist = omitted_payload["slot_force"]["dist"]
    check("可视化分布力/omit 时无 dist", omitted_dist is None)
    check(
        "可视化分布力/omit 时给出警告",
        any("分布力" in text for text in omitted_payload["warnings"]),
        str(omitted_payload["warnings"]),
    )


def test_schematic_layout() -> None:
    """Fallback layout: counts, bounds, and the real 119-pad length."""
    points = schematic_layout.build_dist_points()
    check(
        "示意布局/点位数量",
        [len(row) for row in points] == [49, 20, 49, 20, 49, 20, 49, 20,
                                         119, 49],
        str([len(row) for row in points]),
    )
    check(
        "示意布局/坐标在画布内",
        all(0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 for row in points for x, y in row),
    )
    xs = [p[0] for p in points[8]]
    ys = [p[1] for p in points[8]]
    width = max(xs) - min(xs)
    height = max(ys) - min(ys)
    long_side, short_side = max(width, height), min(width, height)
    check(
        "示意布局/119长条长度与真实模型一致",
        0.50 <= long_side <= 0.53,
        f"{width:.4f} x {height:.4f}",
    )
    check(
        "示意布局/119长条保持长宽比",
        long_side / short_side >= 2.5,
        f"{long_side / short_side:.2f}",
    )


def test_quest_view(tmp_root: Path) -> None:
    """Quest payload keeps its SDK axis and never invents source alignment."""
    path = tmp_root / "quest_view.h5"
    rows = 20
    stamps = 1_700_000_000_000_000_000 + np.arange(rows) * 8_333_333
    stamps[3] = -1
    xyz = np.column_stack([
        np.arange(rows) * 0.01,
        np.ones(rows) * 1.2,
        -np.arange(rows) * 0.02,
    ])
    pose = np.column_stack([xyz, np.tile([0.0, 0.0, 0.0, 1.0], (rows, 1))])
    with h5py.File(path, "w") as handle:
        quest = handle.create_group("quest")
        quest["sdk_timestamp_ns"] = stamps
        quest["body_timestamp_ns"] = np.full(rows, -1, dtype=np.int64)
        quest["motion_timestamp_ns"] = np.full(rows, -1, dtype=np.int64)
        quest["body_joints_timestamp_ns"] = np.full((rows, 24), -1, dtype=np.int64)
        quest["host_unix_ns"] = stamps + 2_000_000_000
        quest["host_monotonic_ns"] = (
            stamps - stamps[0] + 6_000_000_000
        )
        quest["sequence"] = np.arange(20, 20 + rows, dtype=np.int32)
        quest["pose_6d"] = pose
        quest["tracked"] = np.ones(rows, dtype=bool)
        quest["tracked"][2] = False
        quest["head/pose_xyzw"] = pose
        quest["head/tracked"] = np.ones(rows, dtype=bool)
        quest["head/available"] = np.ones(rows, dtype=bool)
        for side in ("left", "right"):
            quest[f"controller_{side}/pose_xyzw"] = pose
            quest[f"controller_{side}/grip"] = np.linspace(0, 1, rows)
            quest[f"controller_{side}/trigger"] = np.zeros(rows)
            quest[f"controller_{side}/axis"] = np.zeros((rows, 2))
            quest[f"hand_{side}/joint_count"] = np.full(rows, 26, dtype=np.int32)
        for name in ("A", "B", "X", "Y", "left_menu", "right_menu",
                     "left_axis_click", "right_axis_click"):
            quest[f"buttons/{name}"] = np.zeros(rows, dtype=bool)

        with h5py.File(path, "r") as opened:
            payload = load_quest_view(
                opened,
                100,
                pose_source="controller",
                pose_side="left",
            )
            host_payload = load_quest_view(
                opened,
                100,
                pose_source="controller",
                pose_side="left",
                alignment="host",
                cap_unix_ns=int(stamps[0]),
            )
            auto_payload = load_quest_view(
                opened,
                100,
                pose_source="controller",
                pose_side="left",
                alignment="auto",
                cap_unix_ns=int(stamps[0]),
            )

    check(
        "Quest 可视化/host 用主机接收时刻且 auto 会选中它",
        host_payload is not None
        and host_payload["alignment"] == "host"
        and host_payload["time_source"] == "host_unix_ns"
        and host_payload["time_based"] is True
        and host_payload["time"][0] == 2.0
        and auto_payload is not None
        and auto_payload["alignment"] == "host",
        str(host_payload and (host_payload["alignment"], host_payload["time"][:2])),
    )
    check(
        "Quest 可视化/独立时间轴且明确未对齐",
        payload is not None
        and payload["time_source"] == "sdk_timestamp_ns"
        and payload["alignment"] == "none"
        and payload["alignment_note"] == QUEST_ALIGNMENT_NOTE,
        str(payload and payload["alignment"]),
    )
    check(
        "Quest 可视化/缺失时间戳保留断点",
        payload is not None
        and payload["time"][3] is None
        and payload["quality"]["sdk_missing_timestamps"] == 1,
        str(payload and payload["time"]),
    )
    check(
        "Quest 可视化/主位姿与手柄位置",
        payload is not None
        and payload["main_pose"]["xyz"][0] == [0.0, 1.2, 0.0]
        and payload["controllers"]["left"]["axis"][0] == [0.0, 0.0]
        and payload["hand_joint_counts"]["right"][0] == 26,
    )

    fallback_path = tmp_root / "quest_view_no_stamp.h5"
    with h5py.File(path, "r") as source, h5py.File(fallback_path, "w") as target:
        source.copy("quest", target)
        del target["quest/sdk_timestamp_ns"]
        target["quest/sdk_timestamp_ns"] = np.full(rows, -1, dtype=np.int64)
    with h5py.File(fallback_path, "r") as opened:
        fallback = load_quest_view(opened, 100)
    check(
        "Quest 可视化/SDK 时间缺失时降级为 sequence",
        fallback is not None
        and fallback["time_source"] == "sequence"
        and fallback["time_label"] == "轮询序号"
        and fallback["time"][1] == 1,
        str(fallback and fallback["time"]),
    )

    report_path = tmp_root / "clock_report.json"
    base_ns = int(stamps[0])
    report = {
        "schema_version": 1,
        "kind": "quest_clock_probe",
        "clock_domain": "unix_epoch",
        "passed": True,
        "models": {
            "host_unix": {
                "reference_host_ns": base_ns,
                "reference_sdk_ns": base_ns,
                "sdk_per_host_slope": 1.0,
                "intercept_s": 0.0,
            }
        },
    }
    report_path.write_text(json.dumps(report), encoding="utf-8")
    check(
        "Quest 可视化/只有通过的 C8 报告可用于 sdk_unix",
        load_clock_report(report_path)["passed"] is True,
    )
    with h5py.File(path, "r") as opened:
        aligned = load_quest_view(
            opened,
            100,
            alignment=QUEST_ALIGNMENT_SDK_UNIX,
            cap_unix_ns=base_ns,
            clock_report=report,
        )
    check(
        "Quest 可视化/sdk_unix 使用手套 Unix 相对秒",
        aligned is not None
        and aligned["alignment"] == QUEST_ALIGNMENT_SDK_UNIX
        and aligned["time_unit"] == "s"
        and aligned["alignment_note"] == QUEST_ALIGNED_VERIFIED_NOTE
        and aligned["time"][0] == 0.0
        and aligned["time"][3] is None,
        str(aligned and aligned["time"]),
    )

    # unix 模式：不要报告，SDK 戳直接当主机墙钟 → 同样是「相对手套首帧秒」。
    with h5py.File(path, "r") as opened:
        plain = load_quest_view(
            opened,
            100,
            alignment=QUEST_ALIGNMENT_UNIX,
            cap_unix_ns=base_ns,
        )
    check(
        "Quest 可视化/unix 模式免报告也可对齐",
        plain is not None
        and plain["alignment"] == QUEST_ALIGNMENT_UNIX
        and plain["time_unit"] == "s"
        and plain["alignment_note"] == QUEST_ALIGNED_NOTE
        and plain["time"][0] == 0.0,
        str(plain and plain["time"][:3]),
    )

    rejected = dict(report)
    rejected["passed"] = False
    report_path.write_text(json.dumps(rejected), encoding="utf-8")
    raised = False
    try:
        load_clock_report(report_path)
    except ValueError:
        raised = True
    check("Quest 可视化/拒绝未通过的 C8 报告", raised)

    file_rejected = False
    try:
        with h5py.File(fallback_path, "r") as opened:
            load_quest_view(
                opened,
                100,
                alignment=QUEST_ALIGNMENT_SDK_UNIX,
                cap_unix_ns=base_ns,
                clock_report=report,
            )
    except ValueError as exc:
        file_rejected = "data.h5 不满足" in str(exc)
    check("Quest 可视化/当前文件时间戳不合格时拒绝对齐", file_rejected)


def test_clock_probe_report() -> None:
    rows = []
    host0 = 1_700_000_000_000_000_000
    for index in range(240):
        host = host0 + index * 8_333_333
        rows.append(
            {
                "host_unix_mid_ns": host,
                "host_monotonic_mid_ns": 5_000_000_000 + index * 8_333_333,
                "sdk_timestamp_ns": host + 2_000_000,
            }
        )
    passed = evaluate_clock_samples(rows, target_duration_s=2.0)
    check(
        "C8 探针/稳定 Unix 时钟通过",
        passed["passed"] is True
        and passed["clock_domain"] == "unix_epoch"
        and passed["alignment"]["recommended"] == "sdk_unix",
        str(passed),
    )

    frozen = [dict(row) for row in rows]
    frozen_sdk = int(frozen[10]["sdk_timestamp_ns"])
    for index in range(10, 160):
        frozen[index]["sdk_timestamp_ns"] = frozen_sdk
    failed = evaluate_clock_samples(frozen, target_duration_s=2.0)
    check(
        "C8 探针/长时间时间戳冻结被拒绝",
        failed["passed"] is False
        and any("连续重复" in reason for reason in failed["failures"]),
        str(failed["failures"]),
    )

    # A 120 Hz pull loop outruns the SDK update, so a repeated timestamp for a
    # few rows is normal; only a duration over one second is a frozen clock.
    duplicated = []
    for row in rows:
        duplicated.append(dict(row))
        duplicated.append(dict(row))
    dupes = evaluate_clock_samples(duplicated, target_duration_s=2.0)
    check(
        "C8 探针/120 Hz 轮询重复不判失败",
        dupes["passed"] is True
        and dupes["metrics"]["max_same_timestamp_run"] == 2,
        str(dupes["failures"]),
    )
    short = evaluate_clock_samples(rows[:50], target_duration_s=2.0)
    check(
        "C8 探针/提前中断不能通过",
        short["passed"] is False
        and any("有效时长" in reason for reason in short["failures"]),
        str(short["failures"]),
    )


def _pose_sample(sequence: int, x: float, tracked: bool = True) -> dict[str, Any]:
    """Minimal Quest sample shape for SelectedPoseWatch / T3 tests."""
    right_pose = {
        "available": True,
        "tracked": tracked,
        "pose_xyzw": [x, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    }
    left_pose = {
        "available": True,
        "tracked": False,
        "pose_xyzw": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    }
    empty_hand = {"active": False, "joint_count": 0, "joints_pose_xyzw": None}
    return {
        "sequence": sequence,
        "tracked": tracked,
        "sdk_timestamp_ns": 1_000_000_000 + sequence,
        "body_timestamp_ns": 0,
        "motion_timestamp_ns": 0,
        "body_joints_timestamp_ns": [],
        "head": {
            "available": True,
            "tracked": True,
            "pose_xyzw": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        },
        "controllers": {
            "left": {"pose": left_pose, "grip": 0.0, "trigger": 0.0, "axis": [0.0, 0.0]},
            "right": {"pose": right_pose, "grip": 0.0, "trigger": 0.0, "axis": [0.0, 0.0]},
        },
        "hands": {"left": dict(empty_hand), "right": dict(empty_hand)},
        "buttons": {},
    }


def test_quest_sequence_floor(tmp_root: Path) -> None:
    """Quest rows collected before the key are dropped by the sequence floor."""
    class _Hand:
        def __init__(self) -> None:
            self.frames: queue.Queue[dict[str, Any]] = queue.Queue()

    output_dir = tmp_root / "quest_floor"
    recorder = Hdf5Recorder(
        output_dir, "controller", "right", "right", (), True
    )
    recorder.__enter__()
    stream = TimedStream(max_samples=10)
    for sequence in (5, 6, 7, 8, 9):
        stream.push(TimedSample(data=make_quest_sample(sequence)))
    STOP_EVENT.set()
    try:
        pump_and_record(
            recorder,
            _Hand(),
            stream,
            0,
            final=False,
            quest_min_sequence=7,
        )
    finally:
        STOP_EVENT.clear()
    recorder.flush_quest()
    with h5py.File(output_dir / "data.h5", "r") as handle:
        sequences = handle["quest/sequence"][:].tolist()
    check(
        "启动闸门/sequence floor 丢掉按键前的 Quest 行",
        sequences == [7, 8, 9],
        str(sequences),
    )
    recorder.__exit__(None, None, None)


def test_start_gate() -> None:
    """Sticky selected-pose readiness + T3 warn-only; no hardware."""
    import collect_cap

    now = [0.0]
    watch = SelectedPoseWatch(
        "controller", "right", clock=lambda: now[0]
    )
    for index in range(10):
        watch.update(_pose_sample(index + 1, index * 0.001), now=now[0])
        now[0] += 0.008
    check(
        "启动闸门/连续有效即就绪",
        watch.ready(required_samples=10),
        f"consecutive={watch.consecutive_valid}",
    )

    frozen = _pose_sample(11, 0.009)
    for _ in range(80):
        watch.update(frozen, now=now[0])
        now[0] += 0.008
    check(
        "启动闸门/位姿冻结 0.6 s 仍就绪（静止是合法数据）",
        watch.ready(required_samples=10),
        f"age={watch.age_s(now[0]):.3f}",
    )

    # Frozen from the very first sample is still valid/tracked data.
    frozen_now = [0.0]
    frozen_watch = SelectedPoseWatch(
        "controller", "right", clock=lambda: frozen_now[0]
    )
    for index in range(10):
        frozen_watch.update(
            _pose_sample(index + 1, 0.123), now=frozen_now[0]
        )
        frozen_now[0] += 0.008
    check(
        "启动闸门/从未变化过的位姿也判就绪（valid/tracked 即可）",
        frozen_watch.ready(required_samples=10),
        f"changes={frozen_watch.changes}",
    )

    watch.update(_pose_sample(13, 0.5, tracked=False), now=now[0])
    check(
        "启动闸门/无效选定位姿清零连续计数",
        watch.consecutive_valid == 0
        and not watch.ready(required_samples=10),
    )

    # T3: a frozen selected pose only warns; it must not end the session.
    collect_cap.ABORT_REASON = None
    STOP_EVENT.clear()
    monitor = RuntimeMonitor(
        [],
        _FakePoller(samples=100, valid_samples=100),
        pose_source="controller",
        pose_side="right",
    )
    for index in range(3):
        monitor.note_quest(_pose_sample(index + 1, index * 0.001))
    check(
        "T3/选定位姿正常更新不告警",
        monitor.pose_stale_warnings == 0,
    )
    assert monitor.pose_watch is not None
    monitor.pose_watch.last_change = time.monotonic() - 6.0
    monitor._next_process = 0.0
    monitor.poll()
    check(
        "T3/选定位姿冻结 6 s 只 warn、不中止",
        collect_cap.ABORT_REASON is None
        and not STOP_EVENT.is_set()
        and monitor.pose_stale_warnings >= 1,
        f"abort={collect_cap.ABORT_REASON!r} "
        f"warnings={monitor.pose_stale_warnings}",
    )
    check(
        "T3/冻结最大时长记账",
        monitor.pose_stale_max_s >= 5.0,
        f"max={monitor.pose_stale_max_s}",
    )

    # A stale run warns once, not once per 1 Hz tick.
    collect_cap.ABORT_REASON = None
    STOP_EVENT.clear()
    warn_monitor = RuntimeMonitor(
        [],
        _FakePoller(samples=100, valid_samples=100),
        pose_source="controller",
        pose_side="right",
    )
    warn_monitor.note_quest(_pose_sample(1, 0.0))
    warn_monitor.note_quest(_pose_sample(2, 0.1))
    assert warn_monitor.pose_watch is not None
    warn_monitor.pose_watch.last_change = time.monotonic() - 2.0
    warn_monitor._next_process = 0.0
    warn_monitor.poll()
    warn_monitor._next_process = 0.0
    warn_monitor.poll()
    check(
        "T3/同一次冻结只告警一次",
        warn_monitor.pose_stale_warnings == 1
        and collect_cap.ABORT_REASON is None,
        f"warnings={warn_monitor.pose_stale_warnings}",
    )

    collect_cap.ABORT_REASON = None
    STOP_EVENT.clear()

    # Start gate: with no Quest/cameras, live glove frames are enough.
    class _FakeHand:
        def __init__(self) -> None:
            self.frames: queue.Queue[dict[str, Any]] = queue.Queue()
            self.callback_count = 0

    gate_args = types.SimpleNamespace(
        pose_source="controller",
        pose_side="right",
        quest_ready_samples=10,
        no_wait=True,
        camera_start_timeout=1.0,
    )
    hand = _FakeHand()
    stream = TimedStream(max_samples=10)
    gate = StartGate(gate_args, None, stream, hand, [], [])
    check("启动闸门/手套还没出帧不就绪", not gate.refresh())
    for _ in range(10):
        hand.frames.put(
            {"timestamp_monotonic_ns": time.monotonic_ns()}
        )
        hand.callback_count += 1
    check("启动闸门/手套连续出帧即就绪", gate.refresh())

    # Cameras must start only after Quest/glove are ready. If is_ready (which
    # includes the cameras) were the launch trigger, the gate would deadlock.
    cam = _FakeCamera("cam0", progress=0)
    hand = _FakeHand()
    gate = StartGate(
        gate_args, None, TimedStream(max_samples=10), hand, [cam], []
    )
    check(
        "启动闸门/相机在源就绪前不启动",
        not gate.is_ready() and not cam.started,
    )
    for _ in range(10):
        hand.frames.put(
            {"timestamp_monotonic_ns": time.monotonic_ns()}
        )
        hand.callback_count += 1
    info = gate.wait()
    check(
        "启动闸门/源就绪后启动相机并完成（无死锁）",
        cam.started
        and info is not None
        and info.camera_first_frame_s.get("cam0") is not None,
        f"started={cam.started} info={info is not None}",
    )

    # The poller's deque can hold samples frozen before the gate; they must not
    # count as "the pose just changed". The first refresh discards the backlog.
    poller = _FakePoller(samples=100, valid_samples=100)
    stream = TimedStream(max_samples=20)
    for index in range(10):
        stream.push(TimedSample(data=_pose_sample(index + 1, 0.5)))
    hand = _FakeHand()
    for _ in range(10):
        hand.frames.put({"timestamp_monotonic_ns": time.monotonic_ns()})
        hand.callback_count += 1
    gate = StartGate(gate_args, poller, stream, hand, [], [])
    gate.refresh()
    check(
        "启动闸门/待机缓冲不锚定冻结位姿",
        gate.pose_watch.samples == 0 and not gate.sources_ready(),
        f"samples={gate.pose_watch.samples}",
    )
    for index in range(10):
        stream.push(
            TimedSample(data=_pose_sample(100 + index, 0.5 + index * 0.001))
        )
    check(
        "启动闸门/primed 后新样本有变化才算就绪",
        gate.refresh(),
        f"changes={gate.pose_watch.changes} ready={gate.sources_ready()}",
    )

    collect_cap.ABORT_REASON = None
    STOP_EVENT.clear()


def run_case(name: str, function: Any, *args: Any) -> None:
    """Run one test group; an unexpected exception is a failure, not a crash.

    Without this, a single environment-dependent group (for example ffprobe not
    being installed) aborts the whole run *before* any result is printed, which
    makes a missing dependency look like "the suite did nothing".
    """
    try:
        function(*args)
    except Exception as exc:  # noqa: BLE001 - reported as a failed check instead
        check(f"{name}（未预期异常）", False, f"{exc.__class__.__name__}: {exc}")


class _FakeCamera:
    """只实现运行期检查用到的三个方法，不碰 ffmpeg。"""

    def __init__(self, camera_id: str, progress: int = 0, failure: str | None = None) -> None:
        self.camera_id = camera_id
        self.preview_fps = 0.0
        self.failure = failure
        self.progress = progress
        self.stops = 0
        self.started = False
        self.start_monotonic_ns = 0

    def start(self) -> None:
        self.started = True
        self.start_monotonic_ns = time.monotonic_ns()
        self.progress = max(1, self.progress)

    def poll_health(self) -> None:
        if self.failure is not None:
            raise RuntimeError(self.failure)

    def progress_signal(self) -> int:
        return self.progress

    def stop(self) -> None:
        self.stops += 1


class _FakePoller:
    """只实现运行期检查用到的属性，不起线程。"""

    def __init__(
        self,
        failure: Exception | None = None,
        samples: int = 0,
        valid_samples: int = 0,
    ) -> None:
        self.failure = failure
        self.samples = samples
        self.valid_samples = valid_samples
        self.overruns = 0


def test_runtime_monitor() -> None:
    """运行期检查：只查配置了的源，宽限期内不误报，失效就中止整场。"""
    import collect_cap

    def fresh(cameras, poller=None, grace: float = 0.0):
        collect_cap.ABORT_REASON = None
        STOP_EVENT.clear()
        monitor = RuntimeMonitor(cameras, poller, camera_grace_s=grace)
        monitor.note_cap()
        monitor._next_process = 0.0
        monitor._next_frames = 0.0
        return monitor

    monitor = fresh([_FakeCamera("cam0")], grace=60.0)
    monitor.poll()
    check("首帧宽限期内没有出帧不中止", collect_cap.ABORT_REASON is None)

    monitor = fresh([_FakeCamera("cam0")], grace=0.0)
    monitor.CAMERA_STALL_S = 0.0
    monitor.poll()
    check(
        "宽限过后一帧都没有也算停滞并中止",
        "没有新帧" in (collect_cap.ABORT_REASON or "")
        and "进度停在 0" in (collect_cap.ABORT_REASON or ""),
        collect_cap.ABORT_REASON or "",
    )

    cam = _FakeCamera("cam0", progress=5)
    monitor = fresh([cam], grace=0.0)
    monitor.poll()
    check("有帧时先记下首帧时刻", monitor.first_frame_s.get("cam0") is not None)
    monitor.CAMERA_STALL_S = 0.0
    monitor._next_frames = 0.0
    monitor.poll()
    check(
        "出过帧后停滞即中止",
        "没有新帧" in (collect_cap.ABORT_REASON or ""),
        collect_cap.ABORT_REASON or "",
    )

    monitor = fresh([_FakeCamera("cam0", failure="ffmpeg 退出码 1")], grace=60.0)
    monitor.poll()
    check(
        "相机进程退出即中止",
        "ffmpeg 退出码 1" in (collect_cap.ABORT_REASON or ""),
        collect_cap.ABORT_REASON or "",
    )

    monitor = fresh([], _FakePoller(failure=RuntimeError("boom")))
    monitor.poll()
    check(
        "Quest 轮询异常即中止",
        "Quest 轮询异常" in (collect_cap.ABORT_REASON or ""),
        collect_cap.ABORT_REASON or "",
    )

    monitor = fresh([], _FakePoller(samples=10))
    monitor.poll()
    monitor.QUEST_POLL_STALL_S = 0.0
    monitor._next_process = 0.0
    monitor.poll()
    check(
        "Quest 轮询计数停滞即中止",
        "没有返回样本" in (collect_cap.ABORT_REASON or ""),
        collect_cap.ABORT_REASON or "",
    )

    monitor = fresh([], _FakePoller(samples=10, valid_samples=0))
    monitor.QUEST_POLL_STALL_S = 1e9
    monitor.QUEST_VALID_GRACE_S = 0.0
    monitor.poll()
    check(
        "Quest 有样本但全无效即中止",
        "没有任何有效 tracking" in (collect_cap.ABORT_REASON or ""),
        collect_cap.ABORT_REASON or "",
    )

    monitor = fresh([], _FakePoller(samples=10, valid_samples=10))
    monitor.QUEST_POLL_STALL_S = 1e9
    monitor.QUEST_VALID_STALL_S = 0.0
    monitor.poll()
    check(
        "有效 tracking 中断即中止",
        "没有有效 tracking" in (collect_cap.ABORT_REASON or ""),
        collect_cap.ABORT_REASON or "",
    )

    check(
        "Quest 有效性/全零默认值无效",
        not quest_sample_is_valid(
            {"sdk_timestamp_ns": -1, "tracked": False}
        ),
    )
    check(
        "Quest 有效性/正时间戳有效",
        quest_sample_is_valid({"sdk_timestamp_ns": 123, "tracked": False}),
    )
    check(
        "Quest 有效性/tracked 为真有效",
        quest_sample_is_valid({"sdk_timestamp_ns": -1, "tracked": True}),
    )

    monitor = fresh([], None)
    monitor.GLOVE_STALL_S = 1e9
    monitor.poll()
    check("未配置的源不参与检查", collect_cap.ABORT_REASON is None)

    monitor = fresh([], None)
    monitor.GLOVE_STALL_S = 0.0
    monitor.poll()
    check(
        "手套停滞即中止",
        "手套" in (collect_cap.ABORT_REASON or ""),
        collect_cap.ABORT_REASON or "",
    )

    stream = TimedStream(max_samples=2)
    for sequence in range(3):
        stream.push(TimedSample(data={"sequence": sequence}))
    kept = stream.drain()
    check(
        "Quest 缓冲满了丢最旧的样本并计数",
        stream.dropped == 1 and [item.data["sequence"] for item in kept] == [1, 2],
        f"dropped={stream.dropped} kept={[item.data['sequence'] for item in kept]}",
    )

    collect_cap.ABORT_REASON = None
    STOP_EVENT.clear()


def main() -> int:
    tmp_root = Path(tempfile.mkdtemp(prefix="paxini-cap-selftest-"))
    try:
        run_case("帧解析", test_frame_parsing)
        run_case("Quest 样本", test_quest3_sample)
        run_case("Quest 行", test_quest_row)
        run_case("两轴", test_two_row_axes, tmp_root)
        run_case("落盘", test_flush_per_batch, tmp_root)
        run_case("收尾", test_emergency_save, tmp_root)
        run_case("相机守卫", test_camera_timestamp_guards, tmp_root)
        run_case("分布力", test_omit_distribution, tmp_root)
        run_case("实时视图", test_live_view_hook)
        run_case("相机单调钟", test_camera_monotonic, tmp_root)
        run_case("配置", test_config_file, tmp_root)
        run_case("手别选择", test_pose_side_selection, tmp_root)
        run_case("启动失败工件", test_startup_failure, tmp_root)
        run_case("三源运行期检查", test_runtime_monitor)
        run_case("启动闸门与位姿冻结", test_start_gate)
        run_case("启动闸门 sequence floor", test_quest_sequence_floor, tmp_root)
        run_case("无线 ADB", test_wireless_adb)
        if shutil.which("ffprobe"):
            run_case("相机命令", test_camera_command, tmp_root)
            run_case("相机预览", test_camera_preview, tmp_root)
        else:
            skip("相机命令 / 相机预览", "本机找不到 ffprobe（这两组验的是 ffmpeg 命令拼装，需要在装好 ffmpeg+ffprobe 的环境跑）")
        run_case("可视化", test_visualize, tmp_root)
        run_case("可视化分布力", test_visualize_distribution, tmp_root)
        run_case("示意布局", test_schematic_layout)
        run_case("Quest 可视化", test_quest_view, tmp_root)
        run_case("C8 时钟探针", test_clock_probe_report)
        run_case("可视化视频", test_visualize_video, tmp_root)
        run_case("杂项", test_misc)
    finally:
        shutil.rmtree(tmp_root, ignore_errors=True)

    for name in PASSED:
        print(f"[通过] {name}")
    for name in SKIPPED:
        print(f"[跳过] {name}")
    for name in FAILED:
        print(f"[失败] {name}")
    print(
        f"\n合计 {len(PASSED) + len(FAILED)} 项：通过 {len(PASSED)}，失败 {len(FAILED)}"
        + (f"，跳过 {len(SKIPPED)} 组" if SKIPPED else "")
    )
    return 0 if not FAILED else 1


if __name__ == "__main__":
    raise SystemExit(main())
