#!/usr/bin/env python3
"""Record PXCap III glove data, two cameras and a Quest 3 6D pose in one HDF5.

PXCap III is a data glove: it has no motors, so this program cannot move
anything. It connects the glove, reads its identity, firmware, fault state and
SN, then starts the SDK collection callback.

Timing policy (deliberately different from ``Dex_Quest/collect_hand_pose.py``):

- Three independent row axes. ``/cap`` has one row per glove frame, ``/quest``
  one row per SDK sample, ``/cameras/<id>`` one timestamp per video frame. The
  row counts are never equal and none of them may be used as an index into
  another.
- Nothing is paired, interpolated or matched here. Pairing belongs to
  ``Paxini-Align``; doing it at capture time would bake one policy into every
  file forever. Do not add alignment to this module.
- Every device timestamp is the one the SDK/driver produced. Since format 5 the
  Quest rows additionally carry ``host_unix_ns`` / ``host_monotonic_ns``: the
  PC receive time of that sample, for alignment with the camera/glove. It is
  explicitly **not** the headset sampling instant. The glove/camera columns are
  unchanged; do not append another host reading to them.

Startup is split into a standby gate and a recording session. The Quest poller
and the glove run while the gate waits for a real selected pose and real glove
frames; once both are live the cameras start and the gate waits for each one's
first frame. Only then does the terminal print "准备完毕" and wait for a single
key press. The key is the recording start: `data.h5` is created there, and the
standby Quest/glove buffers are discarded, so neither group carries pre-key rows.
The cameras keep recording through the standby window (their mkv contains a
pre-roll, exactly like the historical camera-first start); downstream Align
drops frames outside the common window. A camera is never restarted, so the
camera latency/timestamp code is untouched.

Health is checked from the capture loop, so a source that is still coming up
never holds up the rows already flowing. A configured source that stops
producing data ends the run through the normal shutdown path. Quest samples can
keep arriving while the selected pose is frozen (controller asleep/lost); that
is recorded as a warning in the manifest but does not end the run: a still hand
is valid data and the SDK has no per-controller tracked flag.

The glove callback runs on an SDK-owned thread and must stay short: it copies
the frame into numpy arrays and pushes them into a queue. All file writing
happens on the main thread.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import platform
import queue
import re
import select
import shlex
import shutil
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Sequence

try:  # POSIX terminal input; the collector itself only runs on native Ubuntu.
    import termios
    import tty
except ImportError:  # pragma: no cover - Windows import for selftest only
    termios = None  # type: ignore[assignment]
    tty = None  # type: ignore[assignment]

try:
    import h5py
    import numpy as np
    import yaml
except ImportError as exc:  # pragma: no cover - dependency guard
    print(
        f"[错误] 缺少依赖：{exc}。请在 Paxini-Cap/.venv 里运行 "
        "pip install --ignore-installed -r requirements.txt（见 QUICKSTART 第 1 步）",
        file=sys.stderr,
    )
    raise SystemExit(2)

# 实时视图在 ../viewer 下。采集程序单独成目录后要把它加进搜索路径，
# 否则 --view 时函数内的 `from live_view import ...` 会 ImportError
# （延迟导入，--help 测不出来）。
_VIEWER_DIR = Path(__file__).resolve().parent.parent / "viewer"
if str(_VIEWER_DIR) not in sys.path:
    sys.path.insert(0, str(_VIEWER_DIR))

from camera_ffmpeg import (
    FfmpegCamera,
    discover_video_nodes,
    ffmpeg_version,
    find_ffmpeg,
    interrupt_running_cameras,
)
from quest3 import (
    BUTTON_METHODS,
    DEFAULT_APK,
    DEFAULT_PACKAGE,
    DEFAULT_PC_SERVICE_SCRIPT,
    QuestSetupError,
    XrtReader,
    import_sdk,
    maybe_start_pc_service,
    pose_info,
    prepare_quest,
)


SCRIPT_DIR = Path(__file__).resolve().parent
# 采集程序在 capture/ 下，配置放在上一层（Cap_Quest 根）：
# 配置被采集、延时测量、手套标定三处共用，放根目录避免各自维护一份。
QUEST_ROOT = SCRIPT_DIR.parent
# 输出放在再上一层（Paxini-Cap 根）：采集和 Paxini-Align 都要读同一批目录，
# 放在 Cap_Quest 里面会让下游只能写死一个相对路径绕回来取数据。
CAP_ROOT = QUEST_ROOT.parent
DEFAULT_OUTPUT_ROOT = CAP_ROOT / "Recordings"
DEFAULT_CONFIG_PATH = QUEST_ROOT / "cap_config.yaml"
# PXCap III has no motor/encoder bottleneck, so the default matches the product
# data rate instead of PXDex's 30 Hz (PXDex is limited by its 78 Hz encoder).
DEFAULT_HAND_RATE_HZ = 120
CAP_MAX_RATE_HZ = 300
CAP_USB_ID = "28e9:018a"
CAP_MAX_POINTS = 119
QUEST_JOINT_CAPACITY = 26
# get_body_joints_timestamp() 返回 24 个关节的 IMU 时间戳
BODY_JOINT_TIMESTAMP_CAPACITY = 24
CAMERA_SLOTS = ("cam0", "cam1")
HAND_SIDES = ("left", "right")
POSE_SIDE_CHOICES = ("auto",) + HAND_SIDES
ANGLE_FIELDS = [
    "index_proximal",
    "index_distal",
    "middle_proximal",
    "middle_distal",
]
# 产品命名：index_proximal=FF2, index_distal=FF1, middle_proximal=MF2,
# middle_distal=MF1（顺序不能改，和 SDK 结构体字段顺序一致）
CAP_SLOTS = [
    "thumb_one",
    "thumb_two",
    "index_zero",
    "index_one",
    "middle_zero",
    "middle_one",
    "middle_two",
    "middle_three",
    "palm_zero",
    "palm_one",
]
CAP_SLOT_POINTS = {
    "thumb_one": 49,
    "thumb_two": 20,
    "index_zero": 49,
    "index_one": 20,
    "middle_zero": 49,
    "middle_one": 20,
    "middle_two": 49,
    "middle_three": 20,
    "palm_zero": 119,
    "palm_one": 49,
}
BUTTON_NAMES = tuple(BUTTON_METHODS)
QUEST_COORDINATE_FRAME = {
    "name": "xrobotoolkit_quest_tracking_origin",
    "convention": "Unity tracking-origin frame (no additional transform)",
    "handedness": "left-handed",
    "axes": "+X right, +Y up, +Z forward",
    "origin": (
        "XRoboToolKit APK tracking origin; system recentering can change it"
    ),
    "translation_unit": "m",
    "rotation": "quaternion xyzw",
}
STOP_EVENT = threading.Event()
FORCE_EXIT_WATCHDOG_ARMED = threading.Event()
#: Why the session is ending early, for manifest.error. First writer wins.
ABORT_REASON: str | None = None
#: How long the watchdog may spend trying to save the file before giving up.
EMERGENCY_SAVE_TIMEOUT_S = 5.0

# The recorder writing right now, so a signal handler or the shutdown watchdog can
# still save it. Module level because a signal handler only receives (signum, frame).
ACTIVE_RECORDER: "Hdf5Recorder | None" = None
SHUTDOWN_TIMEOUT_S = 30.0
# Coarse sanity check: if a camera starts much later than the glove it is the
# wrong source (coverage gap). A camera that starts *earlier* is normal under
# the key-start model (it records a pre-roll), so that direction has its own
# upper bound below.
SESSION_START_TOLERANCE_S = 5.0
#: Warn only when the camera pre-roll exceeds this many seconds; a short
#: pre-key pre-roll is expected (the camera is live before the key).
CAMERA_PRE_ROLL_WARN_S = 60.0
FLUSH_ROWS = 100

# -- start gate (key press starts the recording) --
#: Consecutive valid selected-pose samples required for readiness.
READY_QUEST_SAMPLES = 10
#: Minimum glove frames before readiness.
READY_GLOVE_FRAMES = 10
#: The newest glove frame must be this fresh for readiness.
READY_GLOVE_FRESH_S = 0.5
#: How often the standby gate prints its status.
READY_REPORT_S = 2.0
#: A selected pose that has not changed for this long only warns (T3, 2026-11-27:
#: a still hand is valid data; bit-identical values are not proof of a lost
#: controller). The sample stream / SDK liveness checks are the real failures.
QUEST_POSE_STALE_WARN_S = 1.0
#: Poll interval while waiting for the key (also re-checks readiness).
KEY_POLL_S = 0.1


def log(message: str) -> None:
    print(message, flush=True)


def warn(message: str) -> None:
    print(f"[警告] {message}", file=sys.stderr, flush=True)


def error(message: str) -> None:
    print(f"[错误] {message}", file=sys.stderr, flush=True)


def abort_recording(reason: str) -> None:
    """Stop the session because a configured source stopped working.

    Deliberately signals through ``STOP_EVENT`` instead of raising: a session
    that ends early must still drain the queues, finalise both videos and write
    ``manifest.json``, otherwise the operator is left with an unreadable file
    and no reason for it. Only the first reason is kept.
    """
    global ABORT_REASON
    if ABORT_REASON is None:
        ABORT_REASON = reason
        error(f"采集中止：{reason}")
    STOP_EVENT.set()


def call_with_timeout(
    label: str,
    function: Callable[[], Any],
    timeout_s: float,
) -> tuple[bool, Exception | None]:
    """Run a cleanup call without allowing a vendor SDK to block shutdown."""
    finished = threading.Event()
    failure: list[Exception] = []

    def run() -> None:
        try:
            function()
        except Exception as exc:
            failure.append(exc)
        finally:
            finished.set()

    worker = threading.Thread(
        target=run,
        name=f"cleanup-{label}",
        daemon=True,
    )
    worker.start()
    if not finished.wait(timeout_s):
        warn(f"{label} 超过 {timeout_s:g} 秒未返回，跳过等待")
        return False, None
    if failure:
        warn(f"{label} 失败：{failure[0]}")
        return True, failure[0]
    return True, None


def emergency_save_and_exit(reason: str, exit_code: int = 130) -> None:
    """Save whatever the recorder still holds, then leave.

    Used by the shutdown watchdog (cleanup hung past the budget) and by a second
    Ctrl+C. Only the pending rows are at stake here, but they are the difference
    between a readable file and a 48-byte stub, so it is worth a few milliseconds.
    """
    recorder = ACTIVE_RECORDER
    if recorder is None:
        error(f"{reason}：没有活动中的写入器，直接退出")
    else:
        log(f"[INFO] {reason}：先把内存里未落盘的行写进 data.h5 …")
        if recorder.emergency_save(EMERGENCY_SAVE_TIMEOUT_S):
            warn("已落盘（含未满一批的余数），但 manifest.json 没写、写标志可能未清")
        else:
            warn(
                "主线程卡在 HDF5 调用里，抢不到锁，无法落盘：数据到最后一个已 flush 的批；"
                "强行并发写只会弄坏文件，所以这里直接退出"
            )
    interrupt_running_cameras()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)


def force_exit_after_watchdog() -> None:
    time.sleep(SHUTDOWN_TIMEOUT_S)
    if not STOP_EVENT.is_set():
        return
    emergency_save_and_exit(f"安全清理超过 {SHUTDOWN_TIMEOUT_S:g} 秒")


@dataclass(frozen=True, slots=True)
class TimedSample:
    data: dict[str, Any]


class TimedStream:
    """Thread-safe hand-off for Quest samples.

    The collector only stores them; pairing with glove frames happens in a
    separate processing project, so this buffer just preserves order and stays
    bounded if the writer falls behind.
    """

    def __init__(self, max_samples: int) -> None:
        self._samples: deque[TimedSample] = deque(maxlen=max_samples)
        self._lock = threading.Lock()
        self.dropped = 0

    def push(self, sample: TimedSample) -> None:
        with self._lock:
            if len(self._samples) == self._samples.maxlen:
                self.dropped += 1
            self._samples.append(sample)

    def latest(self) -> TimedSample | None:
        with self._lock:
            return self._samples[-1] if self._samples else None

    def drain(self) -> list[TimedSample]:
        """Take everything buffered, oldest first."""
        with self._lock:
            items = list(self._samples)
            self._samples.clear()
            return items


def finite_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def force_vector(values: Any) -> tuple[int, int, int]:
    return int(values.x), int(values.y), int(values.z)


def fault_to_dict(fault: Any) -> dict[str, Any]:
    sensor_faults = []
    for item in getattr(fault, "sensor_faults", ()):
        sensor_faults.append(
            {
                "source_name": str(item.source_name),
                "source_index": int(item.source_index),
                "error_code": int(item.error_code),
            }
        )
    magnetic_encoder_faults = []
    for item in getattr(fault, "magnetic_encoder_faults", ()):
        magnetic_encoder_faults.append(
            {
                "source_name": str(item.source_name),
                "source_index": int(item.source_index),
                "error_code": int(item.error_code),
            }
        )
    return {
        "sensor_fault_count": int(getattr(fault, "sensor_fault_count", 0)),
        "magnetic_encoder_fault_count": int(
            getattr(fault, "magnetic_encoder_fault_count", 0)
        ),
        "sensor_faults": sensor_faults,
        "magnetic_encoder_faults": magnetic_encoder_faults,
    }


def snapshot_cap_frame(
    frame: Any,
    include_distribution: bool = True,
) -> dict[str, Any]:
    """Copy a PxCap3 callback frame into numpy arrays.

    The SDK pointer is only valid for the duration of the callback, so every
    field is copied here. Distribution force is read through ``count``: the
    public array always has 119 slots but only ``count`` hold valid points.
    """
    encoder = frame.encoder_raw
    angles = frame.joint_angles
    resultant = frame.resultant_force
    distribution = frame.distribute_force

    encoder_raw = np.array(
        [int(getattr(encoder, name)) for name in ANGLE_FIELDS],
        dtype=np.int32,
    )
    joint_angles = np.full(len(ANGLE_FIELDS), math.nan, dtype=np.float64)
    for index, name in enumerate(ANGLE_FIELDS):
        value = finite_float(getattr(angles, name))
        if value is not None:
            joint_angles[index] = value

    resultant_force = np.zeros((len(CAP_SLOTS), 3), dtype=np.int16)
    distribute_force = (
        np.zeros((len(CAP_SLOTS), CAP_MAX_POINTS, 3), dtype=np.int16)
        if include_distribution
        else None
    )
    distribute_counts = np.zeros(len(CAP_SLOTS), dtype=np.uint32)

    for index, slot in enumerate(CAP_SLOTS):
        resultant_force[index] = force_vector(getattr(resultant, slot))

        slot_data = getattr(distribution, slot)
        count = int(getattr(slot_data, "count", 0))
        if distribute_force is None:
            # 不写数组时仍然记下 SDK 报的有效点数
            distribute_counts[index] = min(count, CAP_MAX_POINTS)
            continue
        points = getattr(slot_data, "values", ()) or ()
        # counts 必须等于真正写进数组的点数，否则读的人会按它去取补齐区的 0
        usable = max(0, min(count, len(points), CAP_MAX_POINTS))
        distribute_counts[index] = usable
        for point_index in range(usable):
            point = points[point_index]
            distribute_force[index, point_index, 0] = int(point.x)
            distribute_force[index, point_index, 1] = int(point.y)
            distribute_force[index, point_index, 2] = int(point.z)

    return {
        "timestamp_unix_ns": int(frame.timestamp_unix_ns),
        "timestamp_monotonic_ns": int(frame.timestamp_monotonic_ns),
        "encoder_raw": encoder_raw,
        "joint_angles_deg": joint_angles,
        "resultant_force": resultant_force,
        "distribute_force": distribute_force,
        "distribute_counts": distribute_counts,
    }


def pose_array(values: Any) -> np.ndarray:
    """Pad or trim a pose to 7 finite floats, missing components become NaN."""
    pose = np.full(7, math.nan, dtype=np.float64)
    if not isinstance(values, (list, tuple)) or len(values) < 7:
        return pose
    for index in range(7):
        number = finite_float(values[index])
        if number is not None:
            pose[index] = number
    return pose


def axis_array(values: Any) -> np.ndarray:
    axis = np.full(2, math.nan, dtype=np.float64)
    if not isinstance(values, (list, tuple)):
        return axis
    for index in range(min(2, len(values))):
        number = finite_float(values[index])
        if number is not None:
            axis[index] = number
    return axis


def extract_selected_pose(
    sample: dict[str, Any],
    source: str,
    side: str,
) -> dict[str, Any]:
    """Extract one pose while retaining the complete Quest sample separately."""
    info: dict[str, Any]
    if source == "head":
        info = sample["head"]
        selected_side = None
    elif source == "controller":
        info = sample["controllers"][side]["pose"]
        selected_side = side
    elif source == "hand":
        hand = sample["hands"][side]
        joints = hand.get("joints_pose_xyzw")
        raw_pose = joints[0] if isinstance(joints, list) and joints else None
        info = pose_info(raw_pose, bool(hand.get("active")))
        selected_side = side
    else:
        raise ValueError(f"未知 pose source: {source}")

    values = info.get("pose_xyzw")
    pose_xyzw = None
    if isinstance(values, list) and len(values) >= 7:
        numeric = [finite_float(value) for value in values[:7]]
        if all(value is not None for value in numeric):
            pose_xyzw = [float(value) for value in numeric]
    return {
        "source": source,
        "side": selected_side,
        "available": bool(info.get("available")),
        "tracked": bool(info.get("tracked")),
        "pose_xyzw": pose_xyzw,
    }


class SelectedPoseWatch:
    """Follow the *selected* Quest pose for readiness and staleness.

    ``sample["tracked"]`` is an OR over head, controllers and hands, so it stays
    true while a controller is asleep and the head keeps moving. The SDK also has
    no per-controller ``is_tracked`` getter (checked against
    ``xrobotkit_sdk``'s public API), and a stale pose keeps a unit quaternion, so
    ``pose_info.tracked`` cannot tell it apart either. Bit-identical values are
    **not** proof of a lost controller either: a resting hand is valid data.

    One watch is used in two places:

    * the standby gate requires ``required_samples`` consecutive valid poses
      (a still pose is valid and does not withdraw readiness);
    * the runtime monitor uses ``age_s`` to warn at 1 s and never aborts (T3):
      bit-identical values cannot distinguish a resting hand from a lost
      controller, and the SDK has no per-controller tracked getter.
    """

    def __init__(
        self,
        pose_source: str,
        pose_side: str,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.pose_source = pose_source
        self.pose_side = pose_side
        self.clock = clock
        self.started = clock()
        self.last_pose: np.ndarray | None = None
        self.last_change: float | None = None
        self.last_valid: float | None = None
        self.consecutive_valid = 0
        self.samples = 0
        self.valid_samples = 0
        self.changes = 0
        self.stale_warned = False

    @property
    def source_text(self) -> str:
        if self.pose_source == "head":
            return "head"
        return f"{self.pose_source}.{self.pose_side}"

    def update(self, sample: dict[str, Any], now: float | None = None) -> bool:
        """Feed one sample; return True when it carried a usable selected pose."""
        moment = self.clock() if now is None else now
        self.samples += 1
        pose = extract_selected_pose(
            sample, self.pose_source, self.pose_side
        )
        values = pose.get("pose_xyzw")
        if not pose.get("tracked") or values is None:
            self.consecutive_valid = 0
            return False
        current = np.asarray(values, dtype=np.float64)
        if current.shape != (7,) or not np.all(np.isfinite(current)):
            self.consecutive_valid = 0
            return False
        self.valid_samples += 1
        self.last_valid = moment
        self.consecutive_valid += 1
        if self.last_pose is None:
            # First valid sample only anchors the comparison. It must not count
            # as "the pose changed": a controller that was already frozen before
            # the gate started would otherwise look fresh for free.
            self.last_pose = current
        elif not np.array_equal(current, self.last_pose):
            self.last_pose = current
            self.last_change = moment
            self.changes += 1
            self.stale_warned = False
        return True

    def age_s(self, now: float | None = None) -> float:
        """Seconds since the selected pose last changed.

        Before the first valid sample this measures from the watch's creation,
        so an invalid/frozen pose still ages into the warning threshold instead
        of looking fresh forever.
        """
        moment = self.clock() if now is None else now
        anchor = self.last_change if self.last_change is not None else self.started
        return max(0.0, moment - anchor)

    def ready(
        self,
        required_samples: int = READY_QUEST_SAMPLES,
    ) -> bool:
        """Readiness is valid/tracked samples, not motion.

        A still hand is valid data; requiring a recent change withdrew the
        banner while the operator was reading it and blew the camera pre-roll
        up to 36 s (2026-11-27 run). Liveness is judged by the sample stream
        (poller samples / SDK timestamps), not by the pose value moving.
        """
        return self.consecutive_valid >= max(1, int(required_samples))


def drain_hand_frames(
    hand_collector: HandCollector,
) -> tuple[int, dict[str, Any] | None]:
    """Discard (or hand over) every queued glove frame; return count + last."""
    count = 0
    last: dict[str, Any] | None = None
    while True:
        try:
            last = hand_collector.frames.get_nowait()
        except queue.Empty:
            break
        count += 1
    return count, last


def wait_for_camera_first_frames(
    cameras: Sequence[FfmpegCamera],
    timeout_s: float,
) -> dict[str, float]:
    """Block until every camera has produced its first frame.

    Returns ``camera_id -> monotonic seconds`` of the first observed frame. A
    missing frame is a hard startup failure: the session is configured to
    record every listed camera, and a camera that opens but never streams would
    otherwise produce a useless 0-frame mkv. ``STOP_EVENT`` ends the wait early
    with an empty dict (the caller treats that as "user aborted").
    """
    if not cameras:
        return {}
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    first_at: dict[str, float] = {}
    while len(first_at) < len(cameras):
        if STOP_EVENT.is_set():
            return {}
        for camera in cameras:
            if camera.camera_id in first_at:
                continue
            camera.poll_health()
            if camera.progress_signal() > 0:
                first_at[camera.camera_id] = time.monotonic()
        if len(first_at) == len(cameras):
            break
        if time.monotonic() >= deadline:
            waiting = [
                camera.camera_id
                for camera in cameras
                if camera.camera_id not in first_at
            ]
            detail = "；".join(
                f"{camera.camera_id}: {camera.failure or camera.log_tail() or '无日志'}"
                for camera in cameras
                if camera.camera_id in waiting
            )
            raise RuntimeError(
                f"相机启动超时（{timeout_s:g} 秒）仍未出帧："
                + ", ".join(waiting)
                + (f"（{detail}）" if detail else "")
            )
        STOP_EVENT.wait(0.05)
    return first_at


def read_start_key(guard: Callable[[], bool]) -> str:
    """Wait for one key press; return ``key`` / ``lost`` / ``abort``.

    ``guard`` is called on every poll while no key is available; it drains the
    Quest/glove buffers and re-checks readiness. Returning False from ``guard``
    means the sources stopped being ready, so the caller withdraws the
    "准备完毕" banner instead of starting on stale data.

    The terminal is put in cbreak mode: no Enter needed, echo off, but ``ISIG``
    stays enabled so Ctrl+C still works. The original attributes are restored
    in ``finally`` even when the caller raises.
    """
    if termios is None or tty is None:  # pragma: no cover - Windows
        raise RuntimeError(
            "单键确认需要 POSIX 终端（termios）；当前平台不支持。"
            "请加 --no-wait 跳过按键"
        )
    if not sys.stdin.isatty():
        raise RuntimeError(
            "需要按任意键开始，但 stdin 不是终端（nohup/管道/重定向）。"
            "请直接在终端运行，或加 --no-wait 让它就绪后自动开始"
        )
    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not STOP_EVENT.is_set():
            try:
                readable, _, _ = select.select([fd], [], [], KEY_POLL_S)
            except InterruptedError:  # a signal (Ctrl+C) arrived mid-select
                continue
            if readable:
                try:
                    os.read(fd, 1)
                except OSError:
                    return "abort"
                return "key"
            if not guard():
                return "lost"
        return "abort"
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original)


@dataclass
class StartInfo:
    """Everything the recording session needs to know about the key start."""

    started_at: datetime
    started_unix_ns: int
    started_monotonic_ns: int
    ready_wait_s: float
    key_wait_s: float
    auto_start: bool
    quest_discarded: int
    hand_discarded: int
    quest_min_sequence: int | None
    pose_watch: SelectedPoseWatch
    camera_first_frame_s: dict[str, float]
    camera_pre_roll_s: dict[str, float]


class StartGate:
    """Standby gate: wait for real data, start cameras, then wait for a key.

    The gate never creates ``data.h5`` and never writes a row. It drains the
    Quest stream and the glove queue so they cannot overflow while the operator
    gets ready, and it starts the cameras only after the Quest pose and the
    glove are both live. That ordering keeps the camera pre-roll short (the
    camera warm-up plus the operator's reaction time, not the headset boot).

    Once ``data.h5`` is created the monitor takes over; the gate has already
    thrown away every pre-key row.
    """

    def __init__(
        self,
        args: argparse.Namespace,
        poller: QuestPoller | None,
        quest_stream: TimedStream,
        hand_collector: HandCollector,
        cameras: Sequence[FfmpegCamera],
        started_cameras: list[FfmpegCamera],
    ) -> None:
        self.args = args
        self.poller = poller
        self.quest_stream = quest_stream
        self.hand_collector = hand_collector
        self.cameras = list(cameras)
        self.started_cameras = started_cameras
        self.pose_watch = SelectedPoseWatch(args.pose_source, args.pose_side)
        self.required_samples = max(1, int(args.quest_ready_samples))
        self.started = time.monotonic()
        self.quest_discarded = 0
        self.hand_discarded = 0
        self.hand_last_monotonic_ns: int | None = None
        self.cameras_launched = False
        self.camera_first_at: dict[str, float] = {}
        self.camera_started_at: dict[str, int] = {}
        self.camera_progress: dict[str, int] = {}
        self.camera_last_growth: dict[str, float] = {}
        self.ready_logged = False
        self.ready_at: float | None = None
        self._last_report = 0.0
        #: The poller starts before this gate; its deque may hold up to five
        #: seconds of pre-gate samples. They must not anchor the pose watch,
        #: otherwise a controller that froze before the gate would look fresh.
        self._pose_primed = False

    # -- source refresh ----------------------------------------------------
    def refresh(self, now: float | None = None) -> bool:
        moment = time.monotonic() if now is None else now
        if self.poller is not None and self.poller.failure is not None:
            raise RuntimeError(f"Quest 轮询异常：{self.poller.failure}")
        if self.poller is not None:
            samples = self.quest_stream.drain()
            if not self._pose_primed:
                # Throw the pre-gate backlog away; the watch starts from the
                # next sample and must observe a real change after this point.
                self.quest_discarded += len(samples)
                self._pose_primed = True
            else:
                for sample in samples:
                    self.quest_discarded += 1
                    self.pose_watch.update(sample.data, now=moment)
        count, last = drain_hand_frames(self.hand_collector)
        self.hand_discarded += count
        if last is not None:
            self.hand_last_monotonic_ns = int(last["timestamp_monotonic_ns"])
        self._check_camera_growth(moment)
        return self.is_ready(moment)

    def _check_camera_growth(self, now: float) -> None:
        if not self.cameras_launched:
            return
        for camera in self.cameras:
            camera_id = camera.camera_id
            camera.poll_health()
            progress = camera.progress_signal()
            if progress > self.camera_progress.get(camera_id, 0):
                self.camera_progress[camera_id] = progress
                self.camera_last_growth[camera_id] = now
                continue
            idle_s = now - self.camera_last_growth.get(camera_id, now)
            if idle_s >= RuntimeMonitor.CAMERA_STALL_S:
                raise RuntimeError(
                    f"PXCap {camera_id} 在等待按键期间已 {idle_s:.1f} 秒没有新帧"
                )

    def hand_age_s(self, now: float) -> float | None:
        if self.hand_last_monotonic_ns is None:
            return None
        return max(
            0.0,
            now - self.hand_last_monotonic_ns / 1e9,
        )

    def sources_ready(self, now: float | None = None) -> bool:
        """Quest selected pose + glove are genuinely live (camera-agnostic).

        The cameras are started *after* this turns true, so this predicate must
        not include them; otherwise the gate would wait for cameras it never
        starts.
        """
        moment = time.monotonic() if now is None else now
        quest_ok = self.poller is None or self.pose_watch.ready(
            required_samples=self.required_samples,
        )
        hand_age = self.hand_age_s(moment)
        glove_ok = (
            self.hand_collector.callback_count >= READY_GLOVE_FRAMES
            and hand_age is not None
            and hand_age <= READY_GLOVE_FRESH_S
        )
        return bool(quest_ok and glove_ok)

    def cameras_ready(self, now: float | None = None) -> bool:
        if not self.cameras:
            return True
        if not self.cameras_launched:
            return False
        if len(self.camera_first_at) != len(self.cameras):
            return False
        for camera in self.cameras:
            camera.poll_health()
        return True

    def is_ready(self, now: float | None = None) -> bool:
        moment = time.monotonic() if now is None else now
        return bool(
            self.sources_ready(moment) and self.cameras_ready(moment)
        )

    # -- cameras -----------------------------------------------------------
    def launch_cameras(self) -> bool:
        if self.cameras_launched or not self.cameras:
            return True
        for camera in self.cameras:
            camera.start()
            self.started_cameras.append(camera)
            self.camera_started_at[camera.camera_id] = int(
                camera.start_monotonic_ns
            )
        self.camera_first_at = wait_for_camera_first_frames(
            self.cameras,
            self.args.camera_start_timeout,
        )
        if not self.camera_first_at:
            return False
        self.cameras_launched = True
        now = time.monotonic()
        self.camera_progress = {
            camera.camera_id: camera.progress_signal()
            for camera in self.cameras
        }
        self.camera_last_growth = {
            camera.camera_id: now for camera in self.cameras
        }
        for camera in self.cameras:
            first = self.camera_first_at[camera.camera_id]
            start = self.camera_started_at[camera.camera_id] / 1e9
            log(
                f"[INFO] PXCap {camera.camera_id} 已出帧："
                f"启动后 {first - start:.2f} 秒"
            )
        # The wait blocked the main thread, so the Quest stream may hold up to
        # five seconds of samples. Drain it (and the glove queue) before the
        # readiness check that follows in ``wait``.
        self.refresh()
        return True

    # -- status ------------------------------------------------------------
    def report(self, now: float) -> None:
        if now - self._last_report < READY_REPORT_S:
            return
        self._last_report = now
        parts: list[str] = []
        if self.poller is None:
            parts.append("Quest：未配置")
        elif self.pose_watch.samples == 0:
            parts.append("Quest：尚未收到样本")
        else:
            age = self.pose_watch.age_s(now)
            parts.append(
                f"Quest {self.pose_watch.source_text}："
                f"连续有效 {self.pose_watch.consecutive_valid}/"
                f"{self.required_samples}，"
                f"最近变化 {age:.1f} 秒前，累计 {self.pose_watch.samples} 样本"
            )
        hand_age = self.hand_age_s(now)
        parts.append(
            f"手套：{self.hand_collector.callback_count} 帧"
            + (
                f"，最近 {hand_age:.1f} 秒前"
                if hand_age is not None
                else "，尚未收到帧"
            )
        )
        if self.cameras:
            if not self.cameras_launched:
                parts.append("相机：等 Quest/手套就绪后启动")
            else:
                ready_ids = ", ".join(
                    sorted(self.camera_first_at)
                ) or "无"
                parts.append(f"相机：已出帧 {ready_ids}")
        else:
            parts.append("相机：未配置")
        log("[等待就绪] " + "；".join(parts))

    def print_ready(self) -> None:
        log("=" * 68)
        if self.poller is None:
            log("[就绪] Quest：未配置（--no-quest）")
        else:
            age = self.pose_watch.age_s()
            log(
                f"[就绪] Quest {self.pose_watch.source_text}："
                f"连续 {self.pose_watch.consecutive_valid} 帧有效位姿，"
                f"最近 {age:.2f} 秒前更新"
            )
        hand_age = self.hand_age_s(time.monotonic())
        log(
            f"[就绪] 手套：{self.hand_collector.callback_count} 帧"
            + (
                f"，最近 {hand_age:.2f} 秒前"
                if hand_age is not None
                else ""
            )
        )
        if not self.cameras:
            log("[就绪] 相机：未配置")
        else:
            details = []
            for camera in self.cameras:
                first = self.camera_first_at[camera.camera_id]
                start = self.camera_started_at[camera.camera_id] / 1e9
                details.append(
                    f"{camera.camera_id} 首帧 {first - start:.2f} 秒"
                )
            log("[就绪] 相机：" + "，".join(details) + "（启动后）")
        log("=" * 68)
        if self.args.no_wait:
            log("准备完毕；已按 --no-wait 自动开始采集")
        else:
            log("准备完毕。按任意键开始采集（Ctrl+C 退出）")

    # -- key / finish ------------------------------------------------------
    def discard_pending(self) -> None:
        """Throw away samples that arrived between the last poll and the key."""
        if self.poller is not None:
            for _sample in self.quest_stream.drain():
                self.quest_discarded += 1
        count, _last = drain_hand_frames(self.hand_collector)
        self.hand_discarded += count

    def finish(self, auto_start: bool) -> StartInfo:
        self.discard_pending()
        started_at = datetime.now().astimezone()
        started_unix_ns = time.time_ns()
        started_monotonic_ns = time.monotonic_ns()
        ready_at = self.ready_at if self.ready_at is not None else (
            started_monotonic_ns / 1e9
        )
        started_monotonic_s = started_monotonic_ns / 1e9
        first_frame_s: dict[str, float] = {}
        pre_roll_s: dict[str, float] = {}
        for camera in self.cameras:
            camera_id = camera.camera_id
            first = self.camera_first_at.get(camera_id)
            start = self.camera_started_at.get(camera_id)
            if first is not None and start is not None:
                first_frame_s[camera_id] = max(0.0, first - start / 1e9)
                pre_roll_s[camera_id] = max(
                    0.0, started_monotonic_s - first
                )
        return StartInfo(
            started_at=started_at,
            started_unix_ns=started_unix_ns,
            started_monotonic_ns=started_monotonic_ns,
            ready_wait_s=(
                (ready_at - self.started)
                if self.ready_at is not None
                else 0.0
            ),
            key_wait_s=(
                max(0.0, started_monotonic_s - ready_at)
                if not auto_start
                else 0.0
            ),
            auto_start=auto_start,
            quest_discarded=self.quest_discarded,
            hand_discarded=self.hand_discarded,
            quest_min_sequence=(
                self.poller.samples + 1 if self.poller is not None else None
            ),
            pose_watch=self.pose_watch,
            camera_first_frame_s=first_frame_s,
            camera_pre_roll_s=pre_roll_s,
        )

    def wait(self) -> StartInfo | None:
        while True:
            if STOP_EVENT.is_set():
                return None
            now = time.monotonic()
            self.refresh(now)
            # Cameras are started only after the Quest/glove sources are live.
            # ``is_ready`` includes the cameras, so it cannot be the trigger for
            # launching them (that would deadlock: cameras wait for ready, ready
            # waits for cameras).
            if (
                self.cameras
                and not self.cameras_launched
                and self.sources_ready(now)
            ):
                if not self.launch_cameras():
                    return None
                now = time.monotonic()
                self.refresh(now)
            ready = self.is_ready(now)
            if not ready:
                if self.ready_logged:
                    self.ready_logged = False
                    log("[等待] 就绪状态丢失，撤回“准备完毕”，继续等待 ...")
                self.report(now)
                STOP_EVENT.wait(0.02)
                continue
            if not self.ready_logged:
                self.ready_logged = True
                self.ready_at = time.monotonic()
                self.print_ready()
            if self.args.no_wait:
                return self.finish(auto_start=True)
            outcome = read_start_key(
                lambda: self.refresh() if not STOP_EVENT.is_set() else False
            )
            if outcome == "key":
                return self.finish(auto_start=False)
            if outcome == "abort":
                return None
            self.ready_logged = False
            log("[等待] 按键前就绪状态丢失，撤回“准备完毕”，继续等待 ...")


def quest_sample_is_valid(sample: dict[str, Any]) -> bool:
    """Whether one SDK sample carries real tracking data.

    A paused XRoboToolKit APK (headset locked or not worn) keeps the poll loop
    alive but every getter returns its default: timestamp -1/0 and tracked
    False. Those rows must not be mistaken for a healthy Quest source.
    """
    if bool(sample.get("tracked")):
        return True
    timestamp = sample.get("sdk_timestamp_ns")
    return (
        isinstance(timestamp, (int, float))
        and not isinstance(timestamp, bool)
        and math.isfinite(timestamp)
        and timestamp > 0
    )


def quest_sample_to_row(
    sample: dict[str, Any],
    pose_source: str,
    pose_side: str,
    joint_capacity: int = QUEST_JOINT_CAPACITY,
) -> dict[str, Any]:
    """Flatten one Quest sample into one row of the quest group.

    Every sample becomes exactly one row; nothing is matched against the glove
    here. Missing values are NaN for floats, ``False`` for flags, ``0`` for
    counts and -1 for "SDK did not provide it".
    """
    body_joints = sample.get("body_joints_timestamp_ns")
    if isinstance(body_joints, list) and body_joints:
        joint_stamps = np.full(
            BODY_JOINT_TIMESTAMP_CAPACITY, -1, dtype=np.int64
        )
        for index in range(
            min(len(body_joints), BODY_JOINT_TIMESTAMP_CAPACITY)
        ):
            value = body_joints[index]
            if isinstance(value, int):
                joint_stamps[index] = value
    else:
        joint_stamps = np.full(BODY_JOINT_TIMESTAMP_CAPACITY, -1, dtype=np.int64)

    row: dict[str, Any] = {
        "quest/sdk_timestamp_ns": int(sample.get("sdk_timestamp_ns") or -1),
        "quest/body_timestamp_ns": int(sample.get("body_timestamp_ns") or -1),
        "quest/motion_timestamp_ns": int(sample.get("motion_timestamp_ns") or -1),
        "quest/body_joints_timestamp_ns": joint_stamps,
        "quest/host_unix_ns": int(sample.get("host_unix_ns") or -1),
        "quest/host_monotonic_ns": int(sample.get("host_monotonic_ns") or -1),
        "quest/sequence": int(sample.get("sequence") or 0),
        "quest/pose_6d": np.full(7, math.nan, dtype=np.float64),
        "quest/head/pose_xyzw": np.full(7, math.nan, dtype=np.float64),
        "quest/head/tracked": False,
        "quest/head/available": False,
        "quest/tracked": False,
    }
    for side in ("left", "right"):
        row[f"quest/controller_{side}/pose_xyzw"] = np.full(
            7, math.nan, dtype=np.float64
        )
        row[f"quest/controller_{side}/grip"] = math.nan
        row[f"quest/controller_{side}/trigger"] = math.nan
        row[f"quest/controller_{side}/axis"] = np.full(
            2, math.nan, dtype=np.float64
        )
        row[f"quest/hand_{side}/joints_pose_xyzw"] = np.full(
            (joint_capacity, 7), math.nan, dtype=np.float64
        )
        row[f"quest/hand_{side}/joint_count"] = 0
    for name in BUTTON_NAMES:
        row[f"quest/buttons/{name}"] = False

    row["quest/tracked"] = bool(sample.get("tracked"))
    head = sample.get("head") or {}
    row["quest/head/pose_xyzw"] = pose_array(head.get("pose_xyzw"))
    row["quest/head/tracked"] = bool(head.get("tracked"))
    row["quest/head/available"] = bool(head.get("available"))

    controllers = sample.get("controllers") or {}
    for side in ("left", "right"):
        controller = controllers.get(side) or {}
        pose = controller.get("pose") or {}
        row[f"quest/controller_{side}/pose_xyzw"] = pose_array(
            pose.get("pose_xyzw")
        )
        grip = finite_float(controller.get("grip"))
        trigger = finite_float(controller.get("trigger"))
        row[f"quest/controller_{side}/grip"] = (
            grip if grip is not None else math.nan
        )
        row[f"quest/controller_{side}/trigger"] = (
            trigger if trigger is not None else math.nan
        )
        row[f"quest/controller_{side}/axis"] = axis_array(controller.get("axis"))

    hands = sample.get("hands") or {}
    for side in ("left", "right"):
        hand = hands.get(side) or {}
        joints = hand.get("joints_pose_xyzw")
        count = int(hand.get("joint_count") or 0)
        row[f"quest/hand_{side}/joint_count"] = min(count, joint_capacity)
        if not isinstance(joints, list):
            continue
        joints_pose = np.full((joint_capacity, 7), math.nan, dtype=np.float64)
        for index in range(min(len(joints), joint_capacity)):
            joints_pose[index] = pose_array(joints[index])
        row[f"quest/hand_{side}/joints_pose_xyzw"] = joints_pose

    buttons = sample.get("buttons") or {}
    for name in BUTTON_NAMES:
        row[f"quest/buttons/{name}"] = bool(buttons.get(name, False))

    selected = extract_selected_pose(sample, pose_source, pose_side)
    row["quest/pose_6d"] = pose_array(selected["pose_xyzw"])
    return row


class Hdf5Recorder:
    """Append-only HDF5 writer with two independent row axes.

    ``/cap`` follows the glove frames (N rows), ``/quest`` follows the Quest
    samples (Q rows), ``/cameras`` only stores each stream's own timestamps.
    Nothing is matched here on purpose: pairing belongs to the downstream
    processing project, and the collector must not invent relationships.
    """

    CAP_DATASETS = (
        ("timestamp_unix_ns", np.int64, 256),
        ("timestamp_monotonic_ns", np.int64, 256),
        ("encoder_raw", np.int32, 256),
        ("joint_angles_deg", np.float64, 256),
        ("resultant_force", np.int16, 256),
        ("distribute_force", np.int16, 16),
        ("distribute_counts", np.uint32, 256),
    )

    def __init__(
        self,
        output_dir: Path,
        pose_source: str,
        pose_side: str,
        cap_side: str,
        camera_ids: Sequence[str],
        include_distribution: bool,
        joint_capacity: int = QUEST_JOINT_CAPACITY,
        flush_rows: int = FLUSH_ROWS,
    ) -> None:
        self.output_dir = output_dir
        self.pose_source = pose_source
        self.pose_side = pose_side
        self.cap_side = cap_side
        self.camera_ids = tuple(camera_ids)
        self.cameras_enabled = bool(self.camera_ids)
        self.include_distribution = include_distribution
        self.joint_capacity = joint_capacity
        self.flush_rows = max(1, flush_rows)

        self.file: h5py.File | None = None
        # Serialises every HDF5 mutation. The shutdown watchdog runs in its own
        # thread and may try to save the file while the main thread is mid-write;
        # without this lock a reentrant HDF5 call from two threads is undefined.
        self._lock = threading.Lock()
        self.cap_rows = 0
        self.quest_rows = 0
        self._cap_pending: list[dict[str, Any]] = []
        self._quest_pending: list[dict[str, Any]] = []

        self._first_cap_unix_ns: int | None = None
        self._last_cap_unix_ns: int | None = None
        self._first_cap_mono_ns: int | None = None
        self._last_cap_mono_ns: int | None = None
        # Quest timestamps are stored as opaque SDK values (domain unverified),
        # so this window is only reported as "first/last SDK stamp", never as a
        # Unix instant.
        self._first_quest_sdk_ns: int | None = None
        self._last_quest_sdk_ns: int | None = None
        # The poller keeps counting through standby; the first recorded row is
        # therefore not #1. Keep the value so the manifest can say where the
        # recording segment starts without resetting a live thread's counter.
        self._first_quest_sequence: int | None = None
        self._last_quest_sequence: int | None = None
        self._first_quest_host_unix_ns: int | None = None
        self._last_quest_host_unix_ns: int | None = None
        self._first_quest_host_mono_ns: int | None = None
        self._last_quest_host_mono_ns: int | None = None
        self.quest_valid_rows = 0

        self._cap_datasets: dict[str, h5py.Dataset] = {}
        self._quest_datasets: dict[str, h5py.Dataset] = {}
        self.started_at = datetime.now().astimezone()
        self.started_monotonic_ns = time.monotonic_ns()

    # -- lifecycle ---------------------------------------------------------
    def __enter__(self) -> "Hdf5Recorder":
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.file = h5py.File(self.output_dir / "data.h5", "w", libver="latest")

        for name, dtype, chunk_rows in self.CAP_DATASETS:
            if name == "distribute_force" and not self.include_distribution:
                continue
            self._cap_datasets[name] = self._dataset(
                f"cap/{name}",
                self._cap_shape(name),
                dtype,
                chunk_rows=chunk_rows,
                compression="gzip" if name == "distribute_force" else None,
            )

        for path, dtype in self._quest_layout():
            self._quest_datasets[path] = self._dataset(
                path,
                self._quest_shape(path),
                dtype,
                chunk_rows=256,
            )

        for camera_id in self.camera_ids:
            self.file.create_group(f"cameras/{camera_id}")
        self.file.create_group("meta")
        return self

    def _cap_shape(self, name: str) -> tuple[int, ...]:
        if name in ("encoder_raw", "joint_angles_deg"):
            return (4,)
        if name == "resultant_force":
            return (len(CAP_SLOTS), 3)
        if name == "distribute_force":
            return (len(CAP_SLOTS), CAP_MAX_POINTS, 3)
        if name == "distribute_counts":
            return (len(CAP_SLOTS),)
        return ()

    def _quest_layout(self) -> list[tuple[str, Any]]:
        specs: list[tuple[str, Any]] = [
            ("quest/sdk_timestamp_ns", np.int64),
            ("quest/body_timestamp_ns", np.int64),
            ("quest/motion_timestamp_ns", np.int64),
            ("quest/body_joints_timestamp_ns", np.int64),
            # Host receive time: PC wall/monotonic clock when the sample became
            # available. Deliberately not a device-sampling timestamp; it is the
            # alignment handle for the camera/glove (which use the host clock).
            ("quest/host_unix_ns", np.int64),
            ("quest/host_monotonic_ns", np.int64),
            ("quest/sequence", np.int32),
            ("quest/pose_6d", np.float64),
            ("quest/head/pose_xyzw", np.float64),
            ("quest/head/tracked", bool),
            ("quest/head/available", bool),
        ]
        for side in ("left", "right"):
            specs += [
                (f"quest/controller_{side}/pose_xyzw", np.float64),
                (f"quest/controller_{side}/grip", np.float64),
                (f"quest/controller_{side}/trigger", np.float64),
                (f"quest/controller_{side}/axis", np.float64),
                (f"quest/hand_{side}/joints_pose_xyzw", np.float64),
                (f"quest/hand_{side}/joint_count", np.int32),
            ]
        for name in BUTTON_NAMES:
            specs.append((f"quest/buttons/{name}", bool))
        specs.append(("quest/tracked", bool))
        return specs

    def _quest_shape(self, path: str) -> tuple[int, ...]:
        if path.endswith("body_joints_timestamp_ns"):
            return (BODY_JOINT_TIMESTAMP_CAPACITY,)
        if path.endswith("joints_pose_xyzw"):
            return (self.joint_capacity, 7)
        if path.endswith("/axis"):
            return (2,)
        if path.endswith("pose_6d") or path.endswith("pose_xyzw"):
            return (7,)
        return ()

    def _dataset(
        self,
        path: str,
        trailing: tuple[int, ...],
        dtype: Any,
        *,
        chunk_rows: int,
        compression: str | None = None,
    ) -> h5py.Dataset:
        assert self.file is not None
        options: dict[str, Any] = {
            "shape": (0, *trailing),
            "maxshape": (None, *trailing),
            "dtype": dtype,
            "chunks": (chunk_rows, *trailing),
        }
        if compression is not None:
            options["compression"] = compression
            options["compression_opts"] = 1
        return self.file.create_dataset(path, **options)

    # -- writing -----------------------------------------------------------
    def write_cap(self, hand: dict[str, Any]) -> None:
        unix_ns = int(hand["timestamp_unix_ns"])
        mono_ns = int(hand["timestamp_monotonic_ns"])
        if self._first_cap_unix_ns is None:
            self._first_cap_unix_ns = unix_ns
            self._first_cap_mono_ns = mono_ns
        self._last_cap_unix_ns = unix_ns
        self._last_cap_mono_ns = mono_ns
        self._cap_pending.append(hand)
        if len(self._cap_pending) >= self.flush_rows:
            self.flush_cap()

    def write_quest(self, row: dict[str, Any]) -> None:
        sdk_ns = int(row.get("quest/sdk_timestamp_ns") or -1)
        sequence = int(row.get("quest/sequence") or 0)
        if sequence > 0:
            if self._first_quest_sequence is None:
                self._first_quest_sequence = sequence
            self._last_quest_sequence = sequence
        if sdk_ns > 0:
            if self._first_quest_sdk_ns is None:
                self._first_quest_sdk_ns = sdk_ns
            self._last_quest_sdk_ns = sdk_ns
        host_unix_ns = int(row.get("quest/host_unix_ns") or -1)
        host_mono_ns = int(row.get("quest/host_monotonic_ns") or -1)
        if host_unix_ns > 0:
            if self._first_quest_host_unix_ns is None:
                self._first_quest_host_unix_ns = host_unix_ns
            self._last_quest_host_unix_ns = host_unix_ns
        if host_mono_ns > 0:
            if self._first_quest_host_mono_ns is None:
                self._first_quest_host_mono_ns = host_mono_ns
            self._last_quest_host_mono_ns = host_mono_ns
        if sdk_ns > 0 or bool(row.get("quest/tracked")):
            self.quest_valid_rows += 1
        self._quest_pending.append(row)
        if len(self._quest_pending) >= self.flush_rows:
            self.flush_quest()

    def flush_cap(self) -> None:
        with self._lock:
            self._flush_cap_locked()

    def _flush_cap_locked(self) -> None:
        """Caller must hold ``self._lock``."""
        if self.file is None or not self._cap_pending:
            return
        batch = self._cap_pending
        self._cap_pending = []
        start = self.cap_rows
        rows = start + len(batch)
        # Build every array before touching the file. If one dataset's values
        # cannot be built (missing key, bad dtype), no dataset has been resized
        # yet, so the file stays at a consistent row count instead of half-written.
        prepared: list[tuple[h5py.Dataset, np.ndarray]] = [
            (dataset, np.array([item[name] for item in batch], dtype=dataset.dtype))
            for name, dataset in self._cap_datasets.items()
        ]
        for dataset, values in prepared:
            dataset.resize((rows, *dataset.shape[1:]))
            dataset[start:rows] = values
        self.cap_rows = rows
        # Flush after every batch, not only in close(): until HDF5 pushes its
        # metadata to the file, a killed process leaves a data.h5 with no tree at
        # all (root group header is never written) and the whole capture is lost.
        # Measured cost on this machine: 0.07 ms per 100-row batch (~0.8% of the
        # 10 ms write). Note this is flush(), not fsync() — it survives a killed
        # process but not a power cut.
        self.file.flush()

    def flush_quest(self) -> None:
        with self._lock:
            self._flush_quest_locked()

    def _flush_quest_locked(self) -> None:
        """Caller must hold ``self._lock``."""
        if self.file is None or not self._quest_pending:
            return
        batch = self._quest_pending
        self._quest_pending = []
        start = self.quest_rows
        rows = start + len(batch)
        prepared: list[tuple[h5py.Dataset, np.ndarray]] = [
            (dataset, np.array([item[path] for item in batch], dtype=dataset.dtype))
            for path, dataset in self._quest_datasets.items()
        ]
        for dataset, values in prepared:
            dataset.resize((rows, *dataset.shape[1:]))
            dataset[start:rows] = values
        self.quest_rows = rows
        self.file.flush()

    def write_camera_timestamps(
        self,
        camera_id: str,
        timestamps_ns: Sequence[int],
        monotonic_ns: Sequence[int] | None = None,
    ) -> dict[str, Any]:
        """Store one camera's own frame timestamps; no pairing is done here.

        ``monotonic_ns`` is currently always ``None``: the V4L2 path stamps
        frames with the host wall clock, so the ``timestamp_monotonic_ns``
        dataset is left absent. Readers detect that with
        ``"timestamp_monotonic_ns" in group`` or via
        ``manifest.cameras.per_camera.<id>.monotonic_available``.
        """
        timestamps = np.asarray(timestamps_ns, dtype=np.int64)
        if timestamps.size > 1 and np.any(np.diff(timestamps) < 0):
            raise RuntimeError(
                f"PXCap {camera_id} 摄像头时间戳倒退，数据不可用"
            )
        # ffmpeg's V4L2 wall clock can repeat a timestamp for two frames. The
        # raw data stays as recorded (dropping a timestamp without dropping the
        # matching video frame would break the frame mapping), but count it so
        # the manifest/Quickstart check can surface it; Paxini-Align collapses
        # the repeat and skips the redundant frame.
        duplicate_timestamps = (
            int(np.count_nonzero(np.diff(timestamps) == 0))
            if timestamps.size > 1
            else 0
        )
        if duplicate_timestamps:
            warn(
                f"PXCap {camera_id} 相机时间戳有 {duplicate_timestamps} 个重复；"
                "Align 会保留首帧、跳过其余重复帧"
            )
        mono: np.ndarray | None = None
        if monotonic_ns is not None and len(monotonic_ns) > 0:
            mono = np.asarray(monotonic_ns, dtype=np.int64)
            # Both columns index the same frames. A length mismatch would give the
            # group two datasets with different first dimensions, which nothing
            # downstream checks for (cameras/ is excluded from verify_rows) and
            # which silently breaks frame pairing in Paxini-Align.
            if mono.size != timestamps.size:
                raise RuntimeError(
                    f"PXCap {camera_id} 墙钟 {timestamps.size} 个、单调钟 {mono.size} 个，"
                    "同一路相机的两列长度必须相等"
                )
            if mono.size > 1 and np.any(np.diff(mono) < 0):
                raise RuntimeError(
                    f"PXCap {camera_id} 摄像头单调钟时间戳倒退，数据不可用"
                )
        with self._lock:
            assert self.file is not None
            group = self.file["cameras"][camera_id]
            if "timestamp_unix_ns" in group:
                del group["timestamp_unix_ns"]
            group.create_dataset("timestamp_unix_ns", data=timestamps, dtype=np.int64)
            info: dict[str, Any] = {
                "frames": int(timestamps.size),
                "first_timestamp_unix_ns": (
                    int(timestamps[0]) if timestamps.size else None
                ),
                "last_timestamp_unix_ns": (
                    int(timestamps[-1]) if timestamps.size else None
                ),
                "duplicate_timestamps": duplicate_timestamps,
            }
            if mono is not None:
                if "timestamp_monotonic_ns" in group:
                    del group["timestamp_monotonic_ns"]
                group.create_dataset(
                    "timestamp_monotonic_ns", data=mono, dtype=np.int64
                )
                info["monotonic_available"] = True
                info["first_timestamp_monotonic_ns"] = (
                    int(mono[0]) if mono.size else None
                )
                info["last_timestamp_monotonic_ns"] = (
                    int(mono[-1]) if mono.size else None
                )
            else:
                if "timestamp_monotonic_ns" in group:
                    del group["timestamp_monotonic_ns"]
                info["monotonic_available"] = False
            self.file.flush()
            return info

    # -- derived values ----------------------------------------------------
    @property
    def cap_total(self) -> int:
        return self.cap_rows + len(self._cap_pending)

    @property
    def quest_total(self) -> int:
        return self.quest_rows + len(self._quest_pending)

    @property
    def first_quest_sequence(self) -> int | None:
        return self._first_quest_sequence

    @property
    def last_quest_sequence(self) -> int | None:
        return self._last_quest_sequence

    @property
    def sample_span_s(self) -> float:
        """Span of the recorded glove frames on the monotonic clock."""
        if self._first_cap_mono_ns is None or self._last_cap_mono_ns is None:
            return 0.0
        return (self._last_cap_mono_ns - self._first_cap_mono_ns) / 1e9

    @property
    def capture_rate_hz(self) -> float:
        """Frame rate measured from the recorded timestamps, not wall time."""
        span = self.sample_span_s
        total = self.cap_total
        if span <= 0.0 or total < 2:
            return 0.0
        return (total - 1) / span

    def cap_window_unix_ns(self) -> tuple[int, int] | None:
        if self._first_cap_unix_ns is None or self._last_cap_unix_ns is None:
            return None
        return self._first_cap_unix_ns, self._last_cap_unix_ns

    def cap_window_monotonic_ns(self) -> tuple[int, int] | None:
        if self._first_cap_mono_ns is None or self._last_cap_mono_ns is None:
            return None
        return self._first_cap_mono_ns, self._last_cap_mono_ns

    def quest_window_sdk_ns(self) -> tuple[int, int] | None:
        """First/last non-negative ``quest/sdk_timestamp_ns``, domain unverified."""
        if self._first_quest_sdk_ns is None or self._last_quest_sdk_ns is None:
            return None
        return self._first_quest_sdk_ns, self._last_quest_sdk_ns

    def quest_window_host_unix_ns(self) -> tuple[int, int] | None:
        """First/last host receive times (PC wall clock) of the Quest rows."""
        if (
            self._first_quest_host_unix_ns is None
            or self._last_quest_host_unix_ns is None
        ):
            return None
        return self._first_quest_host_unix_ns, self._last_quest_host_unix_ns

    def quest_window_host_monotonic_ns(self) -> tuple[int, int] | None:
        if (
            self._first_quest_host_mono_ns is None
            or self._last_quest_host_mono_ns is None
        ):
            return None
        return self._first_quest_host_mono_ns, self._last_quest_host_mono_ns

    # -- verification ------------------------------------------------------
    def verify_rows(self) -> dict[str, Any]:
        """Each group's datasets must agree with that group's own row count."""
        with self._lock:
            assert self.file is not None
            self._flush_cap_locked()
            self._flush_quest_locked()
            mismatches: list[dict[str, Any]] = []
            checked = 0

            def visit(name: str, obj: Any) -> None:
                nonlocal checked
                if not isinstance(obj, h5py.Dataset):
                    return
                if name.startswith("cap/"):
                    expected = self.cap_rows
                elif name.startswith("quest/"):
                    expected = self.quest_rows
                else:
                    return
                checked += 1
                if obj.shape[0] != expected:
                    mismatches.append(
                        {
                            "dataset": name,
                            "rows": int(obj.shape[0]),
                            "expected": expected,
                        }
                    )

            self.file.visititems(visit)
            return {
                "cap_rows": self.cap_rows,
                "quest_rows": self.quest_rows,
                "datasets_checked": checked,
                "row_mismatch": mismatches,
            }

    def write_meta(self, metadata: dict[str, Any]) -> None:
        with self._lock:
            if self.file is None:
                # A shutdown watchdog may have closed the file between manifest()
                # and here; losing meta attributes is better than a traceback in
                # the last second of the run.
                warn("data.h5 已关闭，meta 属性未写入")
                return
            group = self.file["meta"]
            for key, value in metadata.items():
                group.attrs[key] = json.dumps(
                    value,
                    ensure_ascii=False,
                    allow_nan=False,
                )

    def manifest(self, extra: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            self._flush_cap_locked()
            self._flush_quest_locked()
            return self._manifest_locked(extra)

    def _manifest_locked(self, extra: dict[str, Any]) -> dict[str, Any]:
        ended_at = datetime.now().astimezone()
        elapsed_s = (time.monotonic_ns() - self.started_monotonic_ns) / 1e9
        return {
            "format_version": 5,
            "started_at": self.started_at.isoformat(timespec="milliseconds"),
            "ended_at": ended_at.isoformat(timespec="milliseconds"),
            "cap_side": self.cap_side,
            "rows": self.cap_rows,
            "quest_rows": self.quest_rows,
            "actual_rate_hz": self.capture_rate_hz,
            "duration_s": elapsed_s,
            "sample_span_s": self.sample_span_s,
            "alignment": "none (raw per-source timestamps only)",
            "pose": {
                "source": self.pose_source,
                "side": None if self.pose_source == "head" else self.pose_side,
                "order": ["x", "y", "z", "qx", "qy", "qz", "qw"],
                "coordinate_frame": dict(QUEST_COORDINATE_FRAME),
            },
            "output": {
                "hdf5": "data.h5",
                "cameras": "cameras/" if self.cameras_enabled else None,
                "camera_ids": list(self.camera_ids),
            },
            **extra,
        }

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def close(self) -> None:
        with self._lock:
            self._close_locked()

    def _close_locked(self) -> None:
        """Caller must hold ``self._lock``."""
        if self.file is None:
            return
        self._flush_cap_locked()
        self._flush_quest_locked()
        for dataset in self._cap_datasets.values():
            if dataset.shape[0] != self.cap_rows:
                dataset.resize((self.cap_rows, *dataset.shape[1:]))
        for dataset in self._quest_datasets.values():
            if dataset.shape[0] != self.quest_rows:
                dataset.resize((self.quest_rows, *dataset.shape[1:]))
        self.file.flush()
        self.file.close()
        self.file = None

    def emergency_save(self, timeout_s: float = 5.0) -> bool:
        """Last-chance save from another thread (shutdown watchdog / second Ctrl+C).

        Takes the recorder lock with a timeout so it never blocks forever: if the
        main thread is stuck *inside* HDF5, grabbing the lock would deadlock, and
        a concurrent write would corrupt the file — in that case return False and
        let the caller hard-exit, which keeps the data already flushed.

        Returns True when the file was closed cleanly (pending rows included).
        """
        if self.file is None:
            return True  # a normal shutdown already finished the job
        if not self._lock.acquire(timeout=max(0.1, timeout_s)):
            return False
        try:
            self._close_locked()
            return True
        except Exception as exc:  # a broken file is still better than no file
            warn(f"紧急落盘失败：{exc.__class__.__name__}: {exc}")
            return False
        finally:
            self._lock.release()


class HandCollector:
    """PxCap3 session: connect, read identity, then stream through a callback."""

    def __init__(
        self,
        port: str,
        frequency_hz: int,
        include_distribution: bool,
    ) -> None:
        import pxhandsdk

        if not hasattr(pxhandsdk, "PxCap3"):
            raise RuntimeError(
                "当前解释器加载不到 PxCap3 绑定：系统 python3 是 3.12 时 "
                "import 能过但类不存在。请一律用 python3.10 运行"
            )
        self.pxhandsdk = pxhandsdk
        self.port = port
        self.frequency_hz = frequency_hz
        self.include_distribution = include_distribution
        self.hand = pxhandsdk.PxCap3()
        self.frames: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=4096)
        self.collection_started = False
        self.snapshot_errors = 0
        self.queue_overflows = 0
        self.callback_count = 0
        self.callback_total_ns = 0
        self.callback_max_ns = 0
        self.sdk_version: str | None = None
        self.controller_version: str | None = None
        self.sensor_versions: list[str] = []
        self.sn = None
        self.fault: dict[str, Any] | None = None
        self.fault_error: str | None = None
        self.info_errors: dict[str, str] = {}

    def connect(self) -> None:
        rc = self.hand.connect_device(self.port)
        if rc != 0:
            code, message = self.hand.get_last_error()
            raise RuntimeError(
                f"PXCap 连接失败 rc={rc}, code={code}, message={message}"
            )
        log(f"[INFO] PXCap 已连接：{self.port}")

        # Everything except is_connected/stop/disconnect returns 4000 once
        # collection starts, so identity must be read here, before start().
        rc, sdk_version = self.hand.get_sdk_version()
        if rc == 0:
            self.sdk_version = str(sdk_version)
            log(f"[INFO] PXCap SDK：{self.sdk_version}")
        else:
            self._note_error("get_sdk_version")

        rc, firmware = self.hand.get_firmware_versions()
        if rc == 0 and firmware is not None:
            self.controller_version = str(firmware.controller_version)
            self.sensor_versions = [
                str(value) for value in firmware.sensor_versions
            ][: int(firmware.sensor_version_count)]
            log(f"[INFO] PXCap 固件：{self.controller_version}")
            if self.sensor_versions:
                log(
                    f"[INFO] PXCap 传感器模组："
                    f"{len(self.sensor_versions)} 个 "
                    f"({', '.join(self.sensor_versions)})"
                )
        else:
            self._note_error("get_firmware_versions")

        rc, sn = self.hand.get_sn()
        if rc == 0:
            self.sn = str(sn)
            log(f"[INFO] PXCap SN：{self.sn}")
        else:
            self._note_error("get_sn")

        rc, fault = self.hand.get_fault_code()
        if rc == 0 and fault is not None:
            self.fault = fault_to_dict(fault)
            log(
                "[INFO] PXCap 故障："
                f"传感器 {self.fault['sensor_fault_count']} 项，"
                f"磁编码器 {self.fault['magnetic_encoder_fault_count']} 项"
            )
        else:
            code, message = self.hand.get_last_error()
            self.fault_error = f"rc={rc}, code={code}, message={message}"
            warn(f"读取 PXCap 故障信息失败：{self.fault_error}")

    def _note_error(self, method: str) -> None:
        code, message = self.hand.get_last_error()
        self.info_errors[method] = f"code={code}, message={message}"
        warn(f"读取 {method} 失败：code={code}, message={message}")

    def _on_frame(self, frame: Any) -> None:
        """SDK thread: copy, enqueue, return. No I/O, no other SDK calls."""
        started_ns = time.perf_counter_ns()
        try:
            sample = snapshot_cap_frame(
                frame,
                self.include_distribution,
            )
        except Exception as exc:
            self.snapshot_errors += 1
            if self.snapshot_errors <= 3:
                warn(f"PXCap 采集帧复制失败：{exc}")
            return
        try:
            self.frames.put_nowait(sample)
        except queue.Full:
            self.queue_overflows += 1
            if self.queue_overflows <= 3:
                warn("PXCap 数据队列已满，出现掉帧")
        elapsed_ns = time.perf_counter_ns() - started_ns
        self.callback_count += 1
        self.callback_total_ns += elapsed_ns
        if elapsed_ns > self.callback_max_ns:
            self.callback_max_ns = elapsed_ns

    @property
    def callback_mean_ms(self) -> float:
        if self.callback_count == 0:
            return 0.0
        return self.callback_total_ns / self.callback_count / 1e6

    @property
    def callback_max_ms(self) -> float:
        return self.callback_max_ns / 1e6

    def start(self) -> None:
        rc = self.hand.start_collection(self.frequency_hz, self._on_frame)
        if rc != 0:
            code, message = self.hand.get_last_error()
            raise RuntimeError(
                f"PXCap 连续采集启动失败 rc={rc}, code={code}, message={message}"
            )
        self.collection_started = True
        log(f"[INFO] PXCap 连续采集已启动：{self.frequency_hz} Hz")

    def stop(self) -> None:
        if self.collection_started:
            rc = self.hand.stop_collection()
            if rc != 0:
                code, message = self.hand.get_last_error()
                warn(
                    "停止 PXCap 连续采集失败："
                    f"rc={rc}, code={code}, message={message}"
                )
            self.collection_started = False

    def disconnect(self) -> None:
        rc = self.hand.disconnect_device()
        if rc != 0:
            warn(f"断开 PXCap 返回错误码 {rc}")


class QuestPoller(threading.Thread):
    def __init__(
        self,
        reader: XrtReader,
        stream: TimedStream,
        rate_hz: float,
    ) -> None:
        super().__init__(name="quest-poller", daemon=True)
        self.reader = reader
        self.stream = stream
        self.period_s = 1.0 / rate_hz
        self.failure: Exception | None = None
        self.samples = 0
        #: Samples that actually carried tracking data (not SDK defaults).
        self.valid_samples = 0
        self.overruns = 0

    def run(self) -> None:
        next_deadline = time.monotonic()
        try:
            while not STOP_EVENT.is_set():
                # SDK timestamps come from XrtReader.collect_sample; this loop
                # adds only the host receive midpoint (host_unix_ns /
                # host_monotonic_ns) around the call, then enqueues.
                sequence = self.samples + 1
                # Count *before* the SDK call. The start gate reads this counter
                # as the pre-key boundary: a sample whose collection began before
                # the key is then below the gate's floor even if it is pushed
                # after the gate's final drain.
                self.samples = sequence
                # Host receive time: read before and after the getter bundle and
                # store the midpoint. This is the PC's clock at the moment the
                # sample became available; it never claims to be the headset
                # sampling instant. It is what makes each new capture
                # self-alignable without a separate C8 probe.
                host_unix_before = time.time_ns()
                host_mono_before = time.monotonic_ns()
                sample = self.reader.collect_sample(sequence)
                host_mono_after = time.monotonic_ns()
                host_unix_after = time.time_ns()
                sample["host_unix_ns"] = (
                    host_unix_before + host_unix_after
                ) // 2
                sample["host_monotonic_ns"] = (
                    host_mono_before + host_mono_after
                ) // 2
                if quest_sample_is_valid(sample):
                    self.valid_samples += 1
                self.stream.push(TimedSample(data=sample))

                next_deadline += self.period_s
                now = time.monotonic()
                if now - next_deadline > self.period_s:
                    skipped = int((now - next_deadline) / self.period_s)
                    self.overruns += max(1, skipped)
                    next_deadline = now
                STOP_EVENT.wait(max(0.0, next_deadline - time.monotonic()))
        except BaseException as exc:
            self.failure = exc
            abort_recording(f"Quest 轮询异常：{exc}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "采集 PXCap III 连续帧、左右摄像头视频和 Quest 3 样本，写成单个 HDF5。"
            "只存原始数据与各自时间戳，不做跨源对齐（对齐另有下游工程）。"
            "手套没有电机，本程序不会产生任何运动。"
        )
    )
    parser.add_argument(
        "--hand-port",
        default="auto",
        help=(
            "PXCap 串口；默认 auto，按 USB VID:PID 28e9:018a 自动发现"
        ),
    )
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="列出自动发现的串口、视频节点和 ffmpeg 版本后退出",
    )
    parser.add_argument(
        "--hand-rate",
        type=int,
        default=DEFAULT_HAND_RATE_HZ,
        help=(
            f"PXCap 连续采集目标频率，默认 {DEFAULT_HAND_RATE_HZ} Hz；"
            f"SDK 建议不超过 {CAP_MAX_RATE_HZ} Hz"
        ),
    )
    parser.add_argument(
        "--quest-rate",
        type=float,
        default=120.0,
        help="Quest SDK 轮询频率，默认 120 Hz",
    )
    parser.add_argument(
        "--quest-ready-samples",
        type=int,
        default=READY_QUEST_SAMPLES,
        help=(
            f"判定 Quest 就绪所需的连续有效选定位姿样本数，默认 "
            f"{READY_QUEST_SAMPLES}；位姿静止不会撤回就绪，"
            "冻结只告警不中止"
        ),
    )
    parser.add_argument(
        "--no-wait",
        action="store_true",
        help=(
            "就绪后不等待按键，立即开始采集；用于非 TTY/自动化。"
            "不加它时，stdin 不是终端会直接报错"
        ),
    )
    parser.add_argument(
        "--pose-source",
        choices=("controller", "hand", "head"),
        default="controller",
        help="主 6D pose 来源，默认 controller",
    )
    parser.add_argument(
        "--pose-side",
        choices=POSE_SIDE_CHOICES,
        default="auto",
        help=(
            "controller/hand pose 的左右手；auto 跟随 --cap-side，"
            "left/right 可显式覆盖"
        ),
    )
    parser.add_argument(
        "--cap-side",
        choices=HAND_SIDES,
        default="right",
        help="当前单只 PXCap 的手别，默认 right；决定相机命名和 Quest 默认手别",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="采集秒数；0 表示运行到 Ctrl+C，默认 0",
    )
    parser.add_argument(
        "--no-cameras",
        action="store_true",
        help="不录制摄像头视频；等价于 --camera-mode none",
    )
    parser.add_argument(
        "--camera-mode",
        choices=("both", "cam0", "cam1", "none"),
        default="both",
        help="相机：both、cam0、cam1 或 none，默认 both",
    )
    parser.add_argument(
        "--camera-0",
        default="auto",
        help="cam0 设备节点；默认 auto，按 sysfs 设备路径排序取第一个",
    )
    parser.add_argument(
        "--camera-1",
        default="auto",
        help="cam1 设备节点；默认 auto，按 sysfs 设备路径排序取第二个",
    )
    parser.add_argument(
        "--camera-width",
        type=int,
        default=1280,
        help="摄像头请求宽度，默认 1280",
    )
    parser.add_argument(
        "--camera-height",
        type=int,
        default=720,
        help="摄像头请求高度，默认 720",
    )
    parser.add_argument(
        "--camera-fps",
        type=float,
        default=30.0,
        help="摄像头请求帧率，默认 30 Hz",
    )
    parser.add_argument(
        "--camera-start-timeout",
        type=float,
        default=15.0,
        help=(
            "每路摄像头的启动宽限秒数，默认 15。宽限内不判停滞（UVC 开流后要等"
            "自动曝光，两路同开时慢的那路可能晚十几秒）；宽限过后 4 秒没有新帧"
            "就中止整场"
        ),
    )
    parser.add_argument(
        "--ffmpeg",
        default="auto",
        help=(
            "ffmpeg 可执行文件路径；默认 auto = 先读 $FFMPEG，再在 PATH 里找。"
            "必须 5.0 以上并带 -fps_mode（apt 的 22.04 版 4.4.2 不满足）"
        ),
    )
    parser.add_argument(
        "--video-codec",
        choices=("copy", "h264", "hevc", "mjpeg"),
        default="copy",
        help=(
            "视频编码，默认 copy（MJPEG 直通，CPU 占用最低）；"
            "h264/hevc 省空间但吃 CPU，mjpeg 重新编码"
        ),
    )
    parser.add_argument(
        "--output-dir",
        help="输出目录；默认 <Paxini-Cap>/Recordings/data_<日期>_<序号>",
    )
    parser.add_argument(
        "--omit-distribution",
        action="store_true",
        help="不写 444 点分布力，以减小文件体积",
    )
    parser.add_argument(
        "--view",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "采集时开一个实时窗口（关节角 / 10 路合力数值表 / 每路相机一帧缩略图）。"
            "视图只读内存里的帧，不碰 data.h5；窗口卡住或关掉都不影响录制。"
            "配置文件里写 \"view\": true 可默认打开，命令行 --no-view 可临时关掉"
        ),
    )
    parser.add_argument(
        "--view-fps",
        type=float,
        default=20.0,
        help="实时窗口刷新率，默认 20；跑在子进程里，不占采集主循环",
    )
    parser.add_argument(
        "--view-mode",
        choices=("auto", "html", "tk"),
        default="auto",
        help=(
            "实时视图的形式。auto（默认）= 有 X/Wayland 会话用 tk、没有则用 html；"
            "html = 本地页面 http://127.0.0.1:8765/（浏览器看，不依赖窗口管理器）；"
            "tk = matplotlib 独立窗口"
        ),
    )
    parser.add_argument(
        "--view-video-fps",
        type=float,
        default=5.0,
        help="实时预览的抽帧帧率，默认 5；调高会多占 ffmpeg 一点 CPU",
    )
    parser.add_argument(
        "--view-video-width",
        type=int,
        default=360,
        help="实时预览的宽度（像素），默认 360，高度按比例",
    )
    parser.add_argument(
        "--no-quest",
        action="store_true",
        help=(
            "不采集 Quest：跳过 PC Service/ADB 与 pose 等待，quest 数据全部按"
            "缺失写入（NaN/False），用于只录手套与相机的场景"
        ),
    )
    parser.add_argument(
        "--skip-adb",
        action="store_true",
        help="不安装/启动 Quest APK，也不配置无线 ADB",
    )
    parser.add_argument("--apk", default=str(DEFAULT_APK), help="Quest APK 路径")
    parser.add_argument("--package", default=DEFAULT_PACKAGE, help="Quest APK 包名")
    parser.add_argument("--adb", help="adb 可执行文件路径")
    parser.add_argument("--serial", help="Quest ADB 设备序列号")
    parser.add_argument("--quest-ip", help="Quest 局域网 IPv4 地址")
    parser.add_argument(
        "--quest-wireless",
        action="store_true",
        help=(
            "优先用无线 ADB 连 Quest：mDNS 自动发现，需要时提示输入配对码；"
            "失败再回退 USB。头显里先开启 Wireless debugging"
        ),
    )
    parser.add_argument("--adb-port", type=int, default=5555, help="无线 ADB 端口")
    parser.add_argument(
        "--allow-usb-only",
        action="store_true",
        help="兼容旧参数；有 USB ADB 时本程序默认就保留 USB",
    )
    parser.add_argument(
        "--start-pc-service",
        action="store_true",
        help="PC Service 未运行时自动启动",
    )
    parser.add_argument(
        "--pc-service-script",
        default=str(DEFAULT_PC_SERVICE_SCRIPT),
        help="PC Service 启动脚本路径",
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help=(
            "YAML 配置文件路径，默认 Cap_Quest/cap_config.yaml（存在即加载）；"
            "命令行参数优先于配置文件"
        ),
    )
    parser.add_argument(
        "--no-pc-service-check",
        action="store_true",
        help="不检查 PC Service 进程",
    )
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if not 1 <= args.hand_rate <= CAP_MAX_RATE_HZ:
        parser.error(
            f"--hand-rate 必须在 1..{CAP_MAX_RATE_HZ} Hz 范围内："
            "SDK 建议连续采集不超过 300 Hz"
        )
    if not 0 < args.quest_rate <= 240:
        parser.error("--quest-rate 必须在 (0, 240] Hz 范围内")
    if args.quest_ready_samples < 1:
        parser.error("--quest-ready-samples 必须大于或等于 1")
    if args.duration < 0:
        parser.error("--duration 必须大于或等于 0")
    if args.camera_width <= 0 or args.camera_height <= 0:
        parser.error("--camera-width 和 --camera-height 必须大于 0")
    if args.camera_fps <= 0:
        parser.error("--camera-fps 必须大于 0")
    if args.camera_start_timeout <= 0:
        parser.error("--camera-start-timeout 必须大于 0")
    if args.no_cameras and args.camera_mode in ("cam0", "cam1"):
        parser.error("--no-cameras 不能与 --camera-mode 同时使用")
    camera_mode = "none" if args.no_cameras else args.camera_mode
    if (
        camera_mode == "both"
        and args.camera_0 != "auto"
        and args.camera_0 == args.camera_1
    ):
        parser.error("--camera-0 与 --camera-1 不能是同一设备")
    if not 1 <= args.adb_port <= 65535:
        parser.error("--adb-port 必须在 1..65535 范围内")
    if args.no_quest and args.start_pc_service:
        parser.error("--no-quest 与 --start-pc-service 不能同时使用")
    if args.no_quest and args.quest_wireless:
        parser.error("--no-quest 与 --quest-wireless 不能同时使用")
    if args.skip_adb and args.quest_wireless:
        parser.error("--skip-adb 与 --quest-wireless 不能同时使用")


def resolve_pose_side(cap_side: str, requested_pose_side: str) -> str:
    """Resolve the Quest side without changing either source's raw fields."""
    if requested_pose_side == "auto":
        return cap_side
    if requested_pose_side in HAND_SIDES:
        return requested_pose_side
    raise ValueError(f"未知 pose side：{requested_pose_side!r}")


def make_output_dir(argument: str | None) -> Path:
    if argument:
        return Path(argument).expanduser().resolve()

    date_text = datetime.now().strftime("%Y%m%d")
    pattern = re.compile(rf"^data_{date_text}_(\d+)$")
    used_numbers: list[int] = []
    if DEFAULT_OUTPUT_ROOT.is_dir():
        for child in DEFAULT_OUTPUT_ROOT.iterdir():
            if not child.is_dir():
                continue
            match = pattern.fullmatch(child.name)
            if match is not None:
                used_numbers.append(int(match.group(1)))

    next_number = max(used_numbers, default=0) + 1
    return (DEFAULT_OUTPUT_ROOT / f"data_{date_text}_{next_number}").resolve()


def create_output_dir(path: Path) -> None:
    """Create the recording directory; never reuse an existing one."""
    try:
        path.mkdir(parents=True, exist_ok=False)
    except FileExistsError as exc:
        raise RuntimeError(
            f"输出目录已存在：{path}。请删除旧目录或换一个 --output-dir"
        ) from exc


def startup_failure_label(error: Exception | None) -> str:
    """Short, filename-safe reason for a ``_startup_failures`` directory."""
    text = str(error or "")
    for token, label in (
        ("串口", "serial"),
        ("Quest", "quest"),
        ("摄像头", "camera"),
        ("相机", "camera"),
        ("ffmpeg", "camera"),
        ("SDK", "sdk"),
    ):
        if token in text:
            return label
    return "startup"


def write_startup_failure(
    failure_dir: Path,
    error: Exception | None,
    *,
    started_at: datetime,
    cap_side: str,
    pose_source: str,
    pose_side: str,
    pose_side_requested: str,
    hand_port: str,
    camera_devices: dict[str, str],
    cameras: Sequence[FfmpegCamera],
    ffmpeg: str | None,
) -> Path:
    """Write the failure artifact outside ``Recordings/data_*``.

    Since 2026-11-27 a startup failure no longer leaves a numbered
    ``data_<date>_<n>`` directory behind: the JSON and each camera's ffmpeg log
    are copied to ``<output-dir-parent>/_startup_failures/<timestamp>_<reason>/``
    (default ``Recordings/_startup_failures/``), and the camera pre-roll mkv
    (if any) is not kept. The caller removes the abandoned output directory
    after this function returns.
    """
    failure_dir.mkdir(parents=True, exist_ok=True)
    camera_log_dir = failure_dir / "cameras"
    camera_info: dict[str, Any] = {}
    for camera in cameras:
        command: list[str] | None = None
        command_error: str | None = None
        try:
            command = camera.command()
        except Exception as exc:  # diagnostics must survive a broken backend
            command_error = f"{exc.__class__.__name__}: {exc}"
        log_target = camera_log_dir / f"{camera.camera_id}.log"
        source_log = camera.log_path
        try:
            if source_log.is_file():
                camera_log_dir.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source_log, log_target)
        except OSError as exc:
            log_target_error = f"{exc.__class__.__name__}: {exc}"
        else:
            log_target_error = None
        camera_info[camera.camera_id] = {
            "device": camera.device,
            "command": command,
            "command_text": shlex.join(command) if command is not None else None,
            "command_error": command_error,
            # The pre-roll mkv is deliberately deleted with the abandoned
            # output dir; only the command and ffmpeg log are diagnostics.
            "video": None,
            "video_source": str(camera.video_path),
            "video_removed": True,
            "log": str(log_target),
            "log_copy_error": log_target_error,
            "log_tail": camera.log_tail(),
            "failure": (
                str(camera.failure) if camera.failure is not None else None
            ),
        }

    payload = {
        "status": "startup_failed",
        "started_at": started_at.isoformat(timespec="milliseconds"),
        "ended_at": datetime.now().astimezone().isoformat(
            timespec="milliseconds"
        ),
        "error": str(error) if error is not None else "采集启动未完成",
        "cap_side": cap_side,
        "pose": {
            "source": pose_source,
            "side": pose_side,
            "side_requested": pose_side_requested,
        },
        "hand_port": hand_port,
        "camera_devices": dict(camera_devices),
        "cameras": camera_info,
        "env": environment_info(ffmpeg),
    }
    path = failure_dir / "startup_failure.json"
    with path.open("w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write("\n")
    return path


def load_config_file(path: Path) -> dict[str, Any]:
    """Read the optional YAML config.

    Keys are the long option names with underscores (``camera_mode``). YAML is a
    superset of JSON, so an old ``cap_config.json`` still parses; comments are
    real ``#`` lines now, but keys starting with ``_`` are still skipped so that
    older files keep working.
    """
    if not path.is_file():
        return {}
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"无法读取配置文件：{path}（{exc}）") from exc
    try:
        payload = yaml.safe_load(text) or {}
    except yaml.YAMLError as exc:
        mark = getattr(exc, "problem_mark", None)
        where = f"第 {mark.line + 1} 行第 {mark.column + 1} 列：" if mark else ""
        raise RuntimeError(f"配置文件不是合法 YAML：{path}（{where}{exc}）") from exc
    if not isinstance(payload, dict):
        raise RuntimeError(f"配置文件顶层必须是映射（键: 值）：{path}")
    return {
        str(key): value
        for key, value in payload.items()
        if not str(key).startswith("_") and value is not None
    }


def apply_config(
    parser: argparse.ArgumentParser,
    config: dict[str, Any],
    path: Path,
) -> None:
    """Apply config entries as parser defaults, so CLI arguments still win."""
    actions = {action.dest: action for action in parser._actions}
    unknown = sorted(key for key in config if key not in actions)
    if unknown:
        raise RuntimeError(
            f"配置文件 {path} 含未知配置项：{', '.join(unknown)}"
        )

    defaults: dict[str, Any] = {}
    for key, value in config.items():
        action = actions[key]
        if action.choices is not None and value not in action.choices:
            raise RuntimeError(
                f"配置项 {key} 必须是 {'/'.join(map(str, action.choices))} "
                f"之一，实际是 {value!r}"
            )
        if isinstance(action.default, bool):
            if not isinstance(value, bool):
                raise RuntimeError(
                    f"配置项 {key} 必须是 true 或 false，实际是 {value!r}"
                )
            defaults[key] = value
            continue
        try:
            if action.type is int:
                defaults[key] = int(value)
            elif action.type is float:
                defaults[key] = float(value)
            elif action.type is str:
                defaults[key] = str(value)
            else:
                defaults[key] = value
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"配置项 {key} 的值无效：{value!r}") from exc
    parser.set_defaults(**defaults)


@dataclass(frozen=True, slots=True)
class SerialPort:
    path: str
    by_id: str | None
    usb_id: str | None


def usb_ids_for_tty(name: str, sysfs_root: Path = Path("/sys/class/tty")) -> str | None:
    """Walk up sysfs from a tty node to the USB device's idVendor:idProduct."""
    directory = sysfs_root / name / "device"
    try:
        directory = directory.resolve(strict=True)
    except OSError:
        return None
    for _ in range(8):
        vendor = directory / "idVendor"
        product = directory / "idProduct"
        if vendor.is_file() and product.is_file():
            try:
                return (
                    f"{vendor.read_text(encoding='utf-8').strip()}:"
                    f"{product.read_text(encoding='utf-8').strip()}"
                ).lower()
            except OSError:
                return None
        if directory.parent == directory:
            return None
        directory = directory.parent
    return None


def discover_hand_ports(dev_root: Path = Path("/dev")) -> list[SerialPort]:
    """Discover USB/ACM serial ports without depending on tty numbering."""
    found: dict[str, SerialPort] = {}
    by_id = dev_root / "serial" / "by-id"
    if by_id.is_dir():
        for entry in sorted(by_id.iterdir()):
            try:
                resolved = entry.resolve(strict=True)
            except OSError:
                continue
            if not re.fullmatch(r"tty(?:USB|ACM)\d+", resolved.name):
                continue
            found.setdefault(
                str(resolved),
                SerialPort(
                    path=str(resolved),
                    by_id=entry.name,
                    usb_id=usb_ids_for_tty(resolved.name),
                ),
            )

    for pattern in ("ttyUSB*", "ttyACM*"):
        for entry in sorted(dev_root.glob(pattern)):
            try:
                resolved = entry.resolve(strict=True)
            except OSError:
                continue
            found.setdefault(
                str(resolved),
                SerialPort(
                    path=str(resolved),
                    by_id=None,
                    usb_id=usb_ids_for_tty(resolved.name),
                ),
            )
    return [found[key] for key in sorted(found)]


def resolve_hand_port(
    requested: str,
    dev_root: Path = Path("/dev"),
) -> str:
    if requested != "auto":
        path = Path(requested).expanduser()
        if not path.exists():
            raise RuntimeError(f"PXCap 串口不存在：{path}")
        return str(path)

    ports = discover_hand_ports(dev_root)
    if not ports:
        raise RuntimeError(
            "未发现可用的串口。请检查手套 USB 线缆、dialout 权限，"
            "或用 --hand-port /dev/ttyACM0 指定"
        )

    matches = [port for port in ports if port.usb_id == CAP_USB_ID]
    if len(matches) == 1:
        log(f"[INFO] 自动发现 PXCap 串口：{matches[0].path}（USB {CAP_USB_ID}）")
        return matches[0].path
    if len(matches) > 1:
        formatted = "\n".join(f"  {port.path}" for port in matches)
        raise RuntimeError(
            f"检测到多个 USB {CAP_USB_ID} 串口，无法判断哪一个是 PXCap：\n"
            f"{formatted}\n请使用 --hand-port 指定"
        )
    if len(ports) == 1:
        warn(
            f"未发现 USB {CAP_USB_ID} 的设备，按唯一串口继续：{ports[0].path}"
        )
        return ports[0].path

    formatted = "\n".join(
        f"  {port.path}（USB {port.usb_id or '未知'}）" for port in ports
    )
    raise RuntimeError(
        f"检测到多个串口，但没有匹配 USB {CAP_USB_ID} 的设备：\n"
        f"{formatted}\n请使用 --hand-port 指定"
    )


def resolve_camera_devices(
    args: argparse.Namespace,
    slots: Sequence[str],
    discovered: Sequence[str],
) -> dict[str, str]:
    if not slots:
        return {}
    devices: dict[str, str] = {}
    for slot in slots:
        configured = (
            args.camera_0 if slot == "cam0" else args.camera_1
        )
        if configured != "auto":
            devices[slot] = str(Path(configured).expanduser())
            continue
        if slot == "cam0" or len(discovered) == 1:
            index = 0
        else:
            index = 1
        if index >= len(discovered):
            raise RuntimeError(
                f"自动发现 PXCap {slot} 摄像头失败。请用 "
                "--camera-0/--camera-1 指定设备节点"
            )
        devices[slot] = discovered[index]
    if len(set(devices.values())) != len(devices):
        raise RuntimeError("两路摄像头解析到了同一个设备，请手动指定")
    return devices


def selected_camera_slots(args: argparse.Namespace) -> tuple[str, ...]:
    mode = "none" if args.no_cameras else args.camera_mode
    if mode == "both":
        return CAMERA_SLOTS
    if mode in CAMERA_SLOTS:
        return (mode,)
    return ()


def resolve_ffmpeg(argument: str | None) -> str | None:
    """Turn the ``auto`` sentinel into None so ``find_ffmpeg`` walks $FFMPEG, then PATH."""
    if argument is None or argument == "auto":
        return None
    return argument


def list_devices(ffmpeg_argument: str | None = None) -> int:
    ports = discover_hand_ports()

    log("[INFO] USB/ACM 串口：")
    if not ports:
        log("  未发现")
    for port in ports:
        marker = "PXCap" if port.usb_id == CAP_USB_ID else "-"
        by_id = f" by-id={port.by_id}" if port.by_id else ""
        log(f"  {port.path} USB={port.usb_id or '未知'} {marker}{by_id}")

    nodes = discover_video_nodes()
    log("[INFO] V4L2 视频采集节点（video-index 0）：")
    if not nodes:
        log("  未发现")
    for node in nodes:
        log(f"  {node}")

    try:
        ffmpeg = find_ffmpeg(resolve_ffmpeg(ffmpeg_argument))
    except RuntimeError as exc:
        warn(str(exc))
        log("[INFO] ffmpeg：未找到")
    else:
        log(f"[INFO] ffmpeg：{ffmpeg_version(ffmpeg)}（{ffmpeg}）")
    return 0


def handle_stop_signal(signum: int, _frame: Any) -> None:
    if STOP_EVENT.is_set():
        emergency_save_and_exit("再次收到停止信号，不再等安全清理", exit_code=130)
        return
    log(
        f"\n[INFO] 收到信号 {signum}，正在安全停止；"
        f"{SHUTDOWN_TIMEOUT_S:g} 秒内未退出可再次按 Ctrl+C 强制结束"
    )
    STOP_EVENT.set()
    if not FORCE_EXIT_WATCHDOG_ARMED.is_set():
        FORCE_EXIT_WATCHDOG_ARMED.set()
        watchdog = threading.Thread(
            target=force_exit_after_watchdog,
            name="force-exit-watchdog",
            daemon=True,
        )
        watchdog.start()


class RuntimeMonitor:
    """Health checks for the sources that are actually configured.

    Complements :class:`StartGate`: the gate decides when recording starts
    (real selected pose + real glove frames + camera first frames + a key), and
    this monitor keeps checking those same sources while rows are being
    written. A configured source has to keep producing data for the whole
    session: a dead process, a stalled camera or a stalled glove all end the
    run through the normal shutdown path. A frozen selected pose only warns
    (T3): a still hand is valid, and bit-identical values are not proof that
    the controller was lost.

    Every failure goes through :func:`abort_recording`, which signals
    ``STOP_EVENT`` instead of raising, so the normal shutdown path still drains
    the queues, probes both videos and writes ``manifest.json``. An early stop
    must leave a readable file that says why it stopped.

    Only configured sources are checked: with ``--camera-mode none`` the camera
    list is empty, and ``--no-quest`` never creates a poller in the first place.

    A live poll loop is not enough for Quest: a locked/not-worn headset pauses
    the APK while the SDK keeps returning default values. ``valid_samples`` is
    checked separately so those all-zero rows end the run instead of looking
    like a healthy source.
    """

    PROCESS_CHECK_S = 1.0
    #: 0.5 Hz frame probe: two samples without growth is the stall threshold.
    FRAME_CHECK_S = 2.0
    CAMERA_STALL_S = 4.0
    GLOVE_STALL_S = 3.0
    #: A pull loop that stops returning samples is stuck inside an SDK call.
    QUEST_POLL_STALL_S = 5.0
    #: The headset needs time to come up before its silence is a failure.
    QUEST_VALID_GRACE_S = 10.0
    #: Once valid tracking was seen, this long without any ends the run.
    QUEST_VALID_STALL_S = 5.0

    def __init__(
        self,
        cameras: Sequence[FfmpegCamera] = (),
        poller: QuestPoller | None = None,
        camera_grace_s: float = 15.0,
        pose_source: str | None = None,
        pose_side: str | None = None,
        camera_first_frame_s: dict[str, float] | None = None,
    ) -> None:
        self.cameras = list(cameras)
        self.poller = poller
        self.camera_grace_s = max(0.0, float(camera_grace_s))
        self.started = time.monotonic()
        self._next_process = self.started
        self._next_frames = self.started
        # The old flow created this monitor right after camera.start(), so 0 was
        # the correct baseline. The start gate creates it after the cameras have
        # already delivered their first frame and passes ``camera_first_frame_s``;
        # in that case seed the current progress instead of re-announcing it.
        seed_progress = bool(camera_first_frame_s)
        self._progress = {
            camera.camera_id: (
                camera.progress_signal() if seed_progress else 0
            )
            for camera in self.cameras
        }
        self._last_growth = {camera.camera_id: self.started for camera in self.cameras}
        #: camera_id -> seconds from camera.start() to its first frame. Seeded by
        #: the start gate, because the cameras were already running when this
        #: monitor was created; otherwise the first 0.5 Hz probe would record a
        #: fake "first frame after 2 s of session".
        self.first_frame_s: dict[str, float] = dict(camera_first_frame_s or {})
        self.camera_stalls = 0
        self.glove_gaps = 0
        self._last_cap = self.started
        self._quest_samples = int(getattr(poller, "samples", 0)) if poller else 0
        self._quest_last_growth = self.started
        self._quest_valid = int(getattr(poller, "valid_samples", 0)) if poller else 0
        self._quest_last_valid = self.started
        self.quest_valid_grace_s = self.QUEST_VALID_GRACE_S
        #: Selected-pose watch for T3: the OR-ed ``tracked`` flag hides a
        #: sleeping controller, the selected pose values do not.
        self.pose_watch: SelectedPoseWatch | None = (
            SelectedPoseWatch(pose_source, pose_side)
            if poller is not None and pose_source is not None
            else None
        )
        self.pose_stale_warnings = 0
        self.pose_stale_max_s = 0.0

    def note_quest(self, sample: dict[str, Any]) -> None:
        """Feed one *written* Quest sample to the selected-pose watch."""
        if self.pose_watch is not None:
            self.pose_watch.update(sample)

    def note_cap(self) -> None:
        """Record that the capture loop just received a glove frame."""
        self._last_cap = time.monotonic()

    def poll(self) -> None:
        now = time.monotonic()
        if now >= self._next_process:
            self._next_process = now + self.PROCESS_CHECK_S
            self._check_health()
        if now >= self._next_frames:
            self._next_frames = now + self.FRAME_CHECK_S
            self._check_frames(now)
        self._check_glove(now)

    def _check_health(self) -> None:
        for camera in self.cameras:
            try:
                camera.poll_health()
            except RuntimeError as exc:
                abort_recording(str(exc))
                return
        poller = self.poller
        if poller is None:
            return
        if poller.failure is not None:
            abort_recording(f"Quest 轮询异常：{poller.failure}")
            return
        # Pull side: the only failure a live thread can hide is being stuck
        # inside an SDK call, and that shows up as a frozen sample counter.
        # ``is_alive()`` would stay True for exactly that case, which is why it
        # is not used here.
        now = time.monotonic()
        samples = int(getattr(poller, "samples", 0))
        valid = int(getattr(poller, "valid_samples", 0))
        if valid > self._quest_valid:
            self._quest_valid = valid
            self._quest_last_valid = now
        if samples > self._quest_samples:
            self._quest_samples = samples
            self._quest_last_growth = now
        else:
            idle_s = now - self._quest_last_growth
            if idle_s >= self.QUEST_POLL_STALL_S:
                abort_recording(
                    f"Quest 轮询已 {idle_s:.1f} 秒没有返回样本"
                    "（线程卡在 SDK 调用里，或 PC Service 不再应答）"
                )
                return

        # Samples can keep arriving while every getter returns its default: the
        # headset is locked/not worn and the APK is paused. That looks like a
        # healthy poll loop, so the payload has to be checked separately.
        if self._quest_valid == 0:
            waited_s = now - self.started
            if waited_s >= self.QUEST_VALID_GRACE_S:
                abort_recording(
                    f"Quest 已连接但 {waited_s:.1f} 秒内没有任何有效 tracking 数据"
                    "（戴上并解锁头显、让 APK 保持前台，开启 Tracking 和 Send）"
                )
                return
        else:
            idle_s = now - self._quest_last_valid
            if idle_s >= self.QUEST_VALID_STALL_S:
                abort_recording(
                    f"Quest 已 {idle_s:.1f} 秒没有有效 tracking 数据"
                    "（头显是否被摘下/锁屏，或 APK 被切到后台）"
                )
                return

        # T3: the selected pose can stop updating while the rest of the headset
        # keeps streaming (controller asleep/lost tracking). ``valid_samples``
        # stays healthy because the head is still tracked, and bit-identical
        # values cannot separate a resting hand from a lost controller (the SDK
        # has no per-controller tracked flag). Warn only, never abort.
        watch = self.pose_watch
        if watch is not None and watch.samples > 0:
            age = watch.age_s(now)
            self.pose_stale_max_s = max(self.pose_stale_max_s, age)
            if age >= QUEST_POSE_STALE_WARN_S and not watch.stale_warned:
                watch.stale_warned = True
                self.pose_stale_warnings += 1
                warn(
                    f"Quest {watch.source_text} 选定位姿已 {age:.1f} 秒没有更新"
                    "（手柄休眠/丢失追踪？请拿动手柄）"
                )
            # A still selected pose is valid data; bit-identical values cannot
            # distinguish "resting hand" from "sleeping controller" (the SDK has
            # no per-controller tracked getter). Keep the warning, never abort.

    def _check_frames(self, now: float) -> None:
        """One rule for "never produced a frame" and "stopped producing".

        A camera that never delivered anything is just the extreme case of no
        growth, so it needs no separate branch: the grace period covers UVC
        warm-up, and after that CAMERA_STALL_S without growth ends the run.
        """
        for camera in self.cameras:
            camera_id = camera.camera_id
            progress = camera.progress_signal()
            if progress > self._progress[camera_id]:
                if camera_id not in self.first_frame_s:
                    self.first_frame_s[camera_id] = now - self.started
                    log(
                        f"[INFO] PXCap {camera_id} 已出帧，"
                        f"等待 {now - self.started:.1f} 秒"
                    )
                self._progress[camera_id] = progress
                self._last_growth[camera_id] = now
                continue
            if now - self.started < self.camera_grace_s:
                continue
            idle_s = now - self._last_growth[camera_id]
            if idle_s >= self.CAMERA_STALL_S:
                self.camera_stalls += 1
                abort_recording(
                    f"PXCap {camera_id} 已 {idle_s:.1f} 秒没有新帧"
                    f"（进度停在 {progress}）"
                )
                return

    def _check_glove(self, now: float) -> None:
        """The glove has no warm-up: it either delivers frames immediately or not.

        ``_last_cap`` is seeded with the monitor's start time, so the very first
        tick already measures from "loop entered" rather than from a grace
        period. Frames captured before the loop started are sitting in the queue,
        so the first iteration consumes one immediately.
        """
        idle_s = now - self._last_cap
        if idle_s >= self.GLOVE_STALL_S:
            self.glove_gaps += 1
            abort_recording(f"PXCap 手套已 {idle_s:.1f} 秒没有新帧")


def pump_and_record(
    recorder: Hdf5Recorder,
    hand_collector: HandCollector,
    quest_stream: TimedStream,
    started_monotonic_ns: int,
    deadline: float | None = None,
    final: bool = False,
    view: Any | None = None,
    monitor: RuntimeMonitor | None = None,
    quest_min_sequence: int | None = None,
) -> None:
    """Move both queues into the HDF5, each on its own row axis.

    ``final=False`` is the recording loop: as soon as a stop is requested it
    returns, so the glove can be stopped before the tail is drained. ``final``
    runs after ``stop_collection()`` and keeps going until the queues are empty
    (1 s safety net), which is what keeps the last frames on disk.

    ``monitor`` runs the health checks on the same tick as the data, so a source
    that is still coming up never blocks the rows already flowing. It stops the
    session through ``STOP_EVENT``, which is why the check lives here instead of
    in a thread of its own: one place decides when the run ends.
    """
    last_progress = time.monotonic()
    stop_drain_deadline: float | None = None
    while True:
        if deadline is not None and time.monotonic() >= deadline:
            STOP_EVENT.set()

        # Drain Quest *before* the monitor's T3 check. The main thread can be
        # blocked for a while (HDF5 batch, live view); if the check ran first it
        # would see the pre-block pose and abort on a stale age even though the
        # newly queued samples prove the pose is alive.
        for sample in quest_stream.drain():
            sequence = int(sample.data.get("sequence") or 0)
            if (
                quest_min_sequence is not None
                and sequence < quest_min_sequence
            ):
                # Collected before the key, pushed after the gate's drain.
                continue
            if monitor is not None and not final:
                monitor.note_quest(sample.data)
            recorder.write_quest(
                quest_sample_to_row(
                    sample.data,
                    recorder.pose_source,
                    recorder.pose_side,
                    recorder.joint_capacity,
                )
            )

        if monitor is not None and not final:
            # The final drain runs after STOP_EVENT; a late T3/camera probe
            # there would only add noise to an already-decided shutdown.
            monitor.poll()

        if STOP_EVENT.is_set():
            if not final:
                break
            if stop_drain_deadline is None:
                stop_drain_deadline = time.monotonic() + 1.0
            elif time.monotonic() >= stop_drain_deadline:
                pending = hand_collector.frames.qsize()
                if pending > 0:
                    warn(f"停止时手套队列仍有约 {pending} 帧未写入")
                for sample in quest_stream.drain():
                    sequence = int(sample.data.get("sequence") or 0)
                    if (
                        quest_min_sequence is not None
                        and sequence < quest_min_sequence
                    ):
                        continue
                    recorder.write_quest(
                        quest_sample_to_row(
                            sample.data,
                            recorder.pose_source,
                            recorder.pose_side,
                            recorder.joint_capacity,
                        )
                    )
                break

        try:
            hand = hand_collector.frames.get(timeout=0.05)
        except queue.Empty:
            if STOP_EVENT.is_set():
                break
            continue

        recorder.write_cap(hand)
        if monitor is not None:
            monitor.note_cap()
        if view is not None:
            view.push(hand)

        now = time.monotonic()
        if now - last_progress >= 1.0:
            elapsed = (time.monotonic_ns() - started_monotonic_ns) / 1e9
            rate = recorder.cap_total / elapsed if elapsed > 0.0 else 0.0
            log(
                f"[采集] 手套帧={recorder.cap_total} "
                f"Quest样本={recorder.quest_total} 速率={rate:.1f}Hz"
            )
            last_progress = now


def environment_info(ffmpeg: str | None) -> dict[str, Any]:
    info: dict[str, Any] = {
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "numpy": np.__version__,
        "h5py": h5py.__version__,
    }
    if ffmpeg is not None:
        info["ffmpeg"] = ffmpeg_version(ffmpeg)
        info["ffmpeg_path"] = ffmpeg
    return info


class StartAborted(RuntimeError):
    """The user cancelled the standby gate; no ``data.h5`` was created."""


def main(argv: Sequence[str] | None = None) -> int:
    global ABORT_REASON
    parser = build_parser()
    # Read --config first, then turn the file into defaults, then parse again so
    # an explicit command-line argument still overrides the file.
    pre_args, _ = parser.parse_known_args(argv)
    config_path = Path(pre_args.config).expanduser()
    try:
        config = load_config_file(config_path)
        if config:
            apply_config(parser, config, config_path)
    except RuntimeError as exc:
        error(str(exc))
        return 2
    args = parser.parse_args(argv)
    if config:
        log(f"[INFO] 已加载配置：{config_path}（{len(config)} 项）")
    elif config_path == DEFAULT_CONFIG_PATH:
        # 默认文件不存在时，检查是不是还留着改名前的那份 JSON：静默忽略会让人
        # 以为配置生效了，实际跑的是内置默认值。
        legacy = DEFAULT_CONFIG_PATH.with_suffix(".json")
        if legacy.is_file():
            warn(
                f"磁盘上还有 {legacy.name}，但它已不再自动加载；"
                f"改名为 {DEFAULT_CONFIG_PATH.name} 才会生效"
            )
    validate_args(parser, args)
    pose_side_requested = args.pose_side
    args.pose_side = resolve_pose_side(args.cap_side, pose_side_requested)
    if pose_side_requested == "auto":
        log(f"[INFO] Quest 手别跟随手套：{args.cap_side}")
    elif args.pose_side != args.cap_side:
        warn(
            f"手套手别为 {args.cap_side}，Quest 显式使用 "
            f"{args.pose_side}；两路数据不会交换或镜像"
        )
    signal.signal(signal.SIGINT, handle_stop_signal)
    signal.signal(signal.SIGTERM, handle_stop_signal)
    STOP_EVENT.clear()
    FORCE_EXIT_WATCHDOG_ARMED.clear()
    ABORT_REASON = None

    if args.list_devices:
        return list_devices(args.ffmpeg)

    if not args.no_wait and not sys.stdin.isatty():
        error(
            "需要按任意键开始采集，但 stdin 不是终端（nohup/管道/重定向）。"
            "请直接在终端运行，或加 --no-wait 让它就绪后自动开始"
        )
        return 2

    reader: XrtReader | None = None
    poller: QuestPoller | None = None
    quest_setup: Any = None
    cameras: list[FfmpegCamera] = []
    started_cameras: list[FfmpegCamera] = []
    camera_devices: dict[str, str] = {}
    camera_videos: dict[str, Any] = {}
    camera_errors: dict[str, str] = {}
    hand_collector: HandCollector | None = None
    recorder: Hdf5Recorder | None = None
    monitor: RuntimeMonitor | None = None
    live_view: Any | None = None
    recorder_entered = False
    poller_started = False
    exit_code = 0
    startup_error: Exception | None = None
    session_started_at = datetime.now().astimezone()
    started_monotonic_ns = time.monotonic_ns()
    start_info: StartInfo | None = None
    output_dir = make_output_dir(args.output_dir)
    output_dir_preexisting = output_dir.exists()
    camera_slots = selected_camera_slots(args)
    camera_ids = tuple(f"{args.cap_side}_{slot}" for slot in camera_slots)
    slot_by_camera = dict(zip(camera_ids, camera_slots))
    resolved_hand_port = args.hand_port
    ffmpeg: str | None = None
    quest_stream = TimedStream(max_samples=max(600, int(args.quest_rate * 10)))

    try:
        resolved_hand_port = resolve_hand_port(args.hand_port)

        # 三条链路各自启动、互不等待。这里的先后只按「前置准备最慢」排，不是依赖：
        # Quest 的 ADB 安装最慢且不涉及任何 pose 等待，所以放最前面；相机节点
        # 解析是配置检查（节点不存在属于用错参数，不是运行期掉线），仍然硬失败；
        # 手套是唯一必需源，它的连接与启动失败时还没有 data.h5，按启动失败收场。
        if args.no_quest:
            warn(
                "已禁用 Quest（--no-quest）：只采集手套与相机，"
                "quest 组全部按缺失写入（NaN/False）"
            )
        else:
            try:
                maybe_start_pc_service(args)
                if not args.skip_adb:
                    quest_setup = prepare_quest(
                        args,
                        keep_usb_adb=True,
                        prefer_wireless=args.quest_wireless,
                    )
                    log(
                        f"[INFO] Quest ADB 已就绪：{quest_setup.serial} "
                        f"({'无线' if quest_setup.wireless else 'USB'})"
                    )
                else:
                    warn(
                        "已跳过 ADB：必须自行确认 XRoboToolKit APK 已经启动。"
                        "程序不会等 Quest：运行期只查它是否还在产出样本。"
                    )
                xrt = import_sdk()
                reader = XrtReader(xrt)
                poller = QuestPoller(reader, quest_stream, args.quest_rate)
                poller.start()
                poller_started = True
                log(
                    f"[INFO] Quest 轮询已启动：{args.quest_rate:g} Hz；"
                    "运行期只查采样计数是否还在增长，不等待有效 pose"
                )
            except (QuestSetupError, RuntimeError, OSError, ImportError) as exc:
                # 只清掉没起来的 poller；已经在跑的线程要留给 finally 收，
                # 否则它会一直持有 SDK 调用。
                if not poller_started:
                    poller = None
                # 配置了 Quest 就等于要求它在场：装配失败和运行期掉线是同一件
                # 事，只是一个发现得早、还没有 data.h5，代价更小。
                raise RuntimeError(
                    f"Quest 已配置但装配失败（{exc}）；"
                    "不戴头显请显式加 --no-quest，或先手动起 PC Service / APK"
                ) from exc

        if camera_slots:
            ffmpeg = find_ffmpeg(resolve_ffmpeg(args.ffmpeg))
            needs_discovery = any(
                (args.camera_0 if slot == "cam0" else args.camera_1) == "auto"
                for slot in camera_slots
            )
            discovered = discover_video_nodes() if needs_discovery else []
            if discovered:
                log("[INFO] 自动发现视频节点：" + ", ".join(discovered))
            camera_devices = resolve_camera_devices(
                args,
                camera_slots,
                discovered,
            )
            log(
                "[INFO] PXCap 摄像头映射："
                + ", ".join(
                    f"{args.cap_side}_{slot}={camera_devices[slot]}"
                    for slot in camera_slots
                )
            )

        # The output dir must exist before camera.open(): open() clears stale
        # progress/log files under cameras/, and create_output_dir() refuses to
        # reuse an existing directory.
        create_output_dir(output_dir)

        if camera_slots:
            camera_dir = output_dir / "cameras"
            cameras = [
                FfmpegCamera(
                    camera_id,
                    camera_devices[slot],
                    camera_dir,
                    args.camera_width,
                    args.camera_height,
                    args.camera_fps,
                    args.video_codec,
                    ffmpeg,
                    preview_fps=args.view_video_fps if args.view else 0.0,
                    preview_width=args.view_video_width,
                )
                for slot, camera_id in zip(camera_slots, camera_ids)
            ]
            for camera in cameras:
                try:
                    camera.open()
                except Exception as exc:
                    camera_errors[camera.camera_id] = str(exc)
                    error(f"PXCap {camera.camera_id} 摄像头打开失败：{exc}")
            if camera_errors:
                raise RuntimeError(
                    "摄像头打开失败，本次配置要求全部在录："
                    + "；".join(
                        f"{camera_id}: {reason}"
                        for camera_id, reason in camera_errors.items()
                    )
                    + "（只录一路请用 --camera-mode cam0 / cam1）"
                )

        hand_collector = HandCollector(
            resolved_hand_port,
            args.hand_rate,
            not args.omit_distribution,
        )
        hand_collector.connect()
        hand_collector.start()

        # Standby gate: real Quest selected pose + real glove frames -> start
        # cameras and wait for their first frames -> print "ready" -> wait for
        # a key. The cameras record a pre-roll before the key; Quest/glove
        # standby samples are discarded; data.h5 is created only at the key.
        gate = StartGate(
            args,
            poller if poller_started else None,
            quest_stream,
            hand_collector,
            cameras,
            started_cameras,
        )
        log(
            "[INFO] 等待就绪：Quest 选定位姿 + 手套真实出帧；"
            "就绪后启动相机并等首帧，然后按键开始"
        )
        start_info = gate.wait()
        if start_info is None:
            raise StartAborted("用户在就绪前按 Ctrl+C 中止，未生成 data.h5")

        # --duration is from the key, not from "after the recorder/view/monitor
        # are set up"; the post-key setup can take a visible amount of time.
        deadline = (
            start_info.started_monotonic_ns / 1e9 + args.duration
            if args.duration > 0.0
            else None
        )

        recorder = Hdf5Recorder(
            output_dir,
            args.pose_source,
            args.pose_side,
            args.cap_side,
            camera_ids,
            not args.omit_distribution,
        )
        recorder.__enter__()
        recorder_entered = True
        # The final drain in ``finally`` uses this for its progress log; set it
        # before anything that can fail, not only after the monitor is built.
        started_monotonic_ns = start_info.started_monotonic_ns
        global ACTIVE_RECORDER
        ACTIVE_RECORDER = recorder
        log(f"[INFO] 数据写入：{output_dir}")
        log(
            f"[INFO] 按键开始：就绪等待 {start_info.ready_wait_s:.1f} s，"
            f"按键等待 {start_info.key_wait_s:.1f} s；"
            f"丢弃待机 Quest {start_info.quest_discarded} 个样本 / "
            f"手套 {start_info.hand_discarded} 帧"
        )
        for camera_id, pre_roll in start_info.camera_pre_roll_s.items():
            log(
                f"[INFO] {camera_id} 预录 {pre_roll:.2f} s"
                "（按键前相机已在录，下游 Align 按公共窗口丢弃）"
            )

        if args.view:
            try:
                preview_ids = tuple(
                    camera.camera_id
                    for camera in started_cameras
                    if camera.preview_fps > 0
                )
                title = f"PXCap III 实时 · {output_dir.name}"
                from live_view import default_view_mode

                view_mode = (
                    args.view_mode if args.view_mode != "auto" else default_view_mode()
                )
                if view_mode == "html":
                    from live_view import HtmlLiveView

                    live_view = HtmlLiveView(
                        CAP_SLOTS,
                        fps=args.view_video_fps,
                        title=title,
                        camera_ids=preview_ids,
                    )
                    live_view.attach_cameras(started_cameras)
                    log(f"[INFO] 实时视图（浏览器版）：{live_view.url}")
                else:
                    from live_view import LiveView

                    live_view = LiveView(
                        CAP_SLOTS,
                        fps=args.view_fps,
                        title=title,
                        camera_ids=preview_ids,
                    )
                    live_view.attach_cameras(started_cameras)
                    log(
                        "[INFO] 实时视图已在独立进程打开"
                        "（关窗或按 q 只关视图，录制继续）"
                    )
            except Exception as exc:
                live_view = None
                warn(f"实时视图打不开：{exc}；采集继续，可以用 --html 事后回放")

        # 只检查「配置了」的源：camera_slots 为空时 started_cameras 也是空的，
        # --no-quest 时根本没有 poller。任何一路失效都由 monitor 中止整场。
        monitor = RuntimeMonitor(
            started_cameras,
            poller if poller_started else None,
            camera_grace_s=args.camera_start_timeout,
            pose_source=args.pose_source,
            pose_side=args.pose_side,
            camera_first_frame_s=start_info.camera_first_frame_s,
        )
        checked = ["手套"]
        if started_cameras:
            checked.append(
                "相机(" + ",".join(c.camera_id for c in started_cameras) + ")"
            )
        if poller_started:
            checked.append("Quest")
        log(
            "[INFO] 运行期检查已启用："
            + " / ".join(checked)
            + f"；相机出帧 0.5 Hz、停滞 {RuntimeMonitor.CAMERA_STALL_S:g} s 即中止，"
            f"手套停滞 {RuntimeMonitor.GLOVE_STALL_S:g} s 即中止，"
            f"Quest 选定位姿 {QUEST_POSE_STALE_WARN_S:g} s 未变化只告警"
        )
        log("开始采集：手套/相机/Quest 已就绪，按键后同时记录；按 Ctrl+C 停止")
        while not STOP_EVENT.is_set():
            pump_and_record(
                recorder,
                hand_collector,
                quest_stream,
                started_monotonic_ns,
                deadline,
                view=live_view,
                monitor=monitor,
                quest_min_sequence=start_info.quest_min_sequence,
            )

    except StartAborted as exc:
        startup_error = exc
        exit_code = 130
        log(str(exc))
    except (QuestSetupError, RuntimeError, OSError, ImportError) as exc:
        startup_error = exc
        error(str(exc))
        exit_code = 2
    finally:
        STOP_EVENT.set()
        if ABORT_REASON is not None and startup_error is None:
            # 运行期中止：文件照常收尾，但 manifest.error 要写明原因、退出码非 0
            startup_error = RuntimeError(ABORT_REASON)
            exit_code = 2
        hand_stopped = True
        if hand_collector is not None:
            log("[INFO] 停止 PXCap 连续采集 ...")
            hand_stopped, _ = call_with_timeout(
                "停止 PXCap 连续采集",
                hand_collector.stop,
                2.5,
            )
        if (
            recorder is not None
            and recorder_entered
            and hand_collector is not None
        ):
            pump_and_record(
                recorder,
                hand_collector,
                quest_stream,
                started_monotonic_ns,
                final=True,
                view=live_view,
                quest_min_sequence=(
                    start_info.quest_min_sequence
                    if start_info is not None
                    else None
                ),
            )
        if poller is not None and poller_started:
            log("[INFO] 停止 Quest SDK 轮询 ...")
            poller.join(timeout=3.0)
            if poller.is_alive():
                warn("Quest 轮询线程未能在 3 秒内退出")
            elif poller.failure is not None:
                error(f"Quest 轮询失败：{poller.failure}")
                if startup_error is None:
                    startup_error = RuntimeError(
                        f"Quest 轮询失败：{poller.failure}"
                    )
                exit_code = 2

        # 三路各自收尾，互不依赖顺序：每路都有自己的超时，任何一路卡住
        # 都只影响它自己。相机必须在这里收掉，后面 probe() 要读已写完的 mkv。
        for camera in started_cameras:
            log(f"[INFO] 停止 PXCap {camera.camera_id} 摄像头 ...")
            call_with_timeout(
                f"停止 PXCap {camera.camera_id} 摄像头",
                camera.stop,
                6.0,
            )
            if camera.failure is not None and startup_error is None:
                startup_error = RuntimeError(
                    f"PXCap {camera.camera_id} 摄像头失败：{camera.failure}"
                )
                exit_code = 2

        if reader is not None:
            if poller is not None and poller_started and poller.is_alive():
                warn(
                    "Quest SDK 轮询线程仍在 C++ 调用中，"
                    "跳过可能阻塞的 close()"
                )
            else:
                call_with_timeout("关闭 Quest SDK", reader.close, 2.0)

        if (
            recorder is not None
            and recorder_entered
            and hand_collector is not None
        ):
            for camera in started_cameras:
                try:
                    video = camera.probe()
                except RuntimeError as exc:
                    # 就绪检测里已经判死的相机保留更具体的原因，别被
                    # "未生成视频文件" 覆盖掉。
                    camera_errors.setdefault(camera.camera_id, str(exc))
                    error(str(exc))
                    if startup_error is None:
                        startup_error = exc
                    exit_code = 2
                    continue
                try:
                    info = recorder.write_camera_timestamps(
                        camera.camera_id,
                        video.timestamps_ns,
                        monotonic_ns=video.timestamps_monotonic_ns,
                    )
                except RuntimeError as exc:
                    camera_errors.setdefault(camera.camera_id, str(exc))
                    error(str(exc))
                    if startup_error is None:
                        startup_error = exc
                    exit_code = 2
                    continue
                log(
                    f"[INFO] PXCap {camera.camera_id} 视频帧：{info['frames']}，"
                    f"实际 {video.actual_fps:.1f} fps"
                )
                camera_videos[camera.camera_id] = (video, info)

            cap_window = recorder.cap_window_unix_ns()
            if cap_window is not None:
                cap_first, cap_last = cap_window
                for camera_id, entry in camera_videos.items():
                    video, info = entry
                    first = info["first_timestamp_unix_ns"]
                    if first is None:
                        continue
                    gap_s = (first - cap_first) / 1e9
                    if gap_s > SESSION_START_TOLERANCE_S:
                        warn(
                            f"{camera_id} 首帧比手套首帧晚 {gap_s:+.1f} 秒，"
                            "相机没覆盖到录制开头（仅提示，文件里不做对齐）"
                        )
                    elif -gap_s > CAMERA_PRE_ROLL_WARN_S:
                        warn(
                            f"{camera_id} 预录 {-gap_s:.1f} 秒，"
                            f"超过预期上限 {CAMERA_PRE_ROLL_WARN_S:g} 秒"
                        )
                # Quest 只剩 SDK 时间戳，域未知，无法与手套的单调钟比较，
                # 因此这里不做覆盖检查；两路是否同场只能靠行数是否都非 0 粗判。
                # --no-quest 时 quest 组本来就是 0 行，别报一条假警告。
                if not args.no_quest and recorder.quest_rows == 0:
                    warn("本次没有采到任何 Quest 样本（quest 组为 0 行）")
                if (
                    not args.no_quest
                    and recorder.quest_rows > 0
                    and recorder.quest_valid_rows == 0
                    and startup_error is None
                ):
                    reason = (
                        "Quest 样本全部无效（sdk_timestamp_ns 全为 -1、tracked 全为 false）；"
                        "戴上并解锁头显、让 APK 保持前台并开启 Tracking/Send 后重采"
                    )
                    error(reason)
                    startup_error = RuntimeError(reason)
                    exit_code = 2

            verify = recorder.verify_rows()
            if verify["row_mismatch"]:
                error(
                    "HDF5 行数自检失败："
                    + ", ".join(
                        f"{item['dataset']}={item['rows']}"
                        f"(应为 {item['expected']})"
                        for item in verify["row_mismatch"]
                    )
                )
                if startup_error is None:
                    startup_error = RuntimeError("HDF5 行数自检失败")
                exit_code = 2

            actual_hand_rate = recorder.capture_rate_hz
            # 三路是各自启动的，起止时刻必然不同；manifest 记下各自的实际窗口，
            # 让人一眼看出谁先开始、谁先结束，而不是靠日志里的时间戳去倒推。
            hand_window = recorder.cap_window_unix_ns()
            hand_mono_window = recorder.cap_window_monotonic_ns()
            quest_sdk_window = recorder.quest_window_sdk_ns()
            quest_host_window = recorder.quest_window_host_unix_ns()
            quest_host_mono_window = recorder.quest_window_host_monotonic_ns()
            camera_first_frame = monitor.first_frame_s if monitor else {}
            cap_first = cap_window[0] if cap_window is not None else None
            camera_metadata: dict[str, Any] = {}
            for camera_id in camera_ids:
                entry: dict[str, Any] = {
                    "device": camera_devices.get(slot_by_camera[camera_id]),
                    "backend": "linux",
                    "file": f"cameras/{camera_id}.mkv",
                    "requested_width": args.camera_width,
                    "requested_height": args.camera_height,
                    "requested_fps": args.camera_fps,
                    "codec": args.video_codec,
                    "error": camera_errors.get(camera_id),
                    "first_frame_s": camera_first_frame.get(camera_id),
                }
                video_entry = camera_videos.get(camera_id)
                if isinstance(video_entry, tuple):
                    video, info = video_entry
                    entry.update(
                        {
                            "frames": info["frames"],
                            "width": video.width,
                            "height": video.height,
                            "avg_fps": video.avg_fps,
                            "actual_fps": video.actual_fps,
                            "duration_s": video.duration_s,
                            "first_timestamp_unix_ns": (
                                info["first_timestamp_unix_ns"]
                            ),
                            "last_timestamp_unix_ns": (
                                info["last_timestamp_unix_ns"]
                            ),
                            "duplicate_timestamps": info.get(
                                "duplicate_timestamps", 0
                            ),
                            "monotonic_available": info.get(
                                "monotonic_available", False
                            ),
                            "first_timestamp_monotonic_ns": info.get(
                                "first_timestamp_monotonic_ns"
                            ),
                            "last_timestamp_monotonic_ns": info.get(
                                "last_timestamp_monotonic_ns"
                            ),
                        }
                    )
                    first_unix = info.get("first_timestamp_unix_ns")
                    entry["pre_roll_s"] = (
                        round(
                            (
                                start_info.started_unix_ns
                                - int(first_unix)
                            )
                            / 1e9,
                            3,
                        )
                        if start_info is not None
                        and first_unix is not None
                        and int(first_unix) < start_info.started_unix_ns
                        else 0.0
                    )
                camera_metadata[camera_id] = entry

            metadata = {
                "hand": {
                    "port": resolved_hand_port,
                    "side": args.cap_side,
                    "pose_side_requested": pose_side_requested,
                    "target_rate_hz": args.hand_rate,
                    "actual_rate_hz": actual_hand_rate,
                    "sdk_version": hand_collector.sdk_version,
                    "controller_version": hand_collector.controller_version,
                    "sensor_versions": hand_collector.sensor_versions,
                    "sn": hand_collector.sn,
                    "fault": hand_collector.fault,
                    "fault_error": hand_collector.fault_error,
                    "info_errors": hand_collector.info_errors,
                    "snapshot_errors": hand_collector.snapshot_errors,
                    "queue_overflows": hand_collector.queue_overflows,
                    "callback_calls": hand_collector.callback_count,
                    "callback_mean_ms": hand_collector.callback_mean_ms,
                    "callback_max_ms": hand_collector.callback_max_ms,
                    "distribution_included": not args.omit_distribution,
                    "standby_frames_discarded": (
                        start_info.hand_discarded if start_info else 0
                    ),
                    "first_timestamp_unix_ns": (
                        hand_window[0] if hand_window else None
                    ),
                    "last_timestamp_unix_ns": (
                        hand_window[1] if hand_window else None
                    ),
                    "first_timestamp_monotonic_ns": (
                        hand_mono_window[0] if hand_mono_window else None
                    ),
                    "last_timestamp_monotonic_ns": (
                        hand_mono_window[1] if hand_mono_window else None
                    ),
                },
                "quest": {
                    "enabled": not args.no_quest and poller is not None,
                    "timestamp_source": "sdk_provided + host_receive",
                    # Per-getter failures are swallowed by XrtReader.call (an
                    # optional API missing must not kill the run), so they are
                    # only visible here and on stderr once. Without this a dead
                    # PC Service leaves 7200 all-NaN rows and no trace.
                    "errors": (
                        dict(reader.errors) if reader is not None else {}
                    ),
                    "timestamp_fields": [
                        "sdk_timestamp_ns",
                        "body_timestamp_ns",
                        "motion_timestamp_ns",
                        "body_joints_timestamp_ns",
                    ],
                    "host_receive_fields": [
                        "host_unix_ns",
                        "host_monotonic_ns",
                    ],
                    "error": (
                        str(poller.failure)
                        if poller is not None and poller.failure is not None
                        else None
                    ),
                    "rows": recorder.quest_rows,
                    "valid_rows": recorder.quest_valid_rows,
                    "first_sequence": recorder.first_quest_sequence,
                    "last_sequence": recorder.last_quest_sequence,
                    "standby_samples_discarded": (
                        start_info.quest_discarded if start_info else 0
                    ),
                    "selected_pose": {
                        "source": args.pose_source,
                        "side": (
                            None
                            if args.pose_source == "head"
                            else args.pose_side
                        ),
                        "required_consecutive_samples": args.quest_ready_samples,
                        "stale_warn_s": QUEST_POSE_STALE_WARN_S,
                        "standby_changes": (
                            start_info.pose_watch.changes
                            if start_info is not None
                            else 0
                        ),
                        "runtime_samples": (
                            monitor.pose_watch.samples
                            if monitor is not None and monitor.pose_watch is not None
                            else 0
                        ),
                        "runtime_valid_samples": (
                            monitor.pose_watch.valid_samples
                            if monitor is not None and monitor.pose_watch is not None
                            else 0
                        ),
                        "runtime_changes": (
                            monitor.pose_watch.changes
                            if monitor is not None and monitor.pose_watch is not None
                            else 0
                        ),
                        "stale_max_s": (
                            round(monitor.pose_stale_max_s, 3)
                            if monitor is not None
                            else 0.0
                        ),
                        "stale_warnings": (
                            monitor.pose_stale_warnings
                            if monitor is not None
                            else 0
                        ),
                    },
                    "samples_dropped": quest_stream.dropped,
                    "setup_adb_skipped": args.skip_adb,
                    "apk_package": args.package,
                    "adb_serial": (
                        quest_setup.serial if quest_setup is not None else None
                    ),
                    "adb_wireless": (
                        quest_setup.wireless if quest_setup is not None else None
                    ),
                    "poller_alive": bool(poller_started)
                    and poller is not None
                    and poller.failure is None,
                    "first_sdk_timestamp_ns": (
                        quest_sdk_window[0] if quest_sdk_window else None
                    ),
                    "last_sdk_timestamp_ns": (
                        quest_sdk_window[1] if quest_sdk_window else None
                    ),
                    "first_host_unix_ns": (
                        quest_host_window[0] if quest_host_window else None
                    ),
                    "last_host_unix_ns": (
                        quest_host_window[1] if quest_host_window else None
                    ),
                    "first_host_monotonic_ns": (
                        quest_host_mono_window[0]
                        if quest_host_mono_window
                        else None
                    ),
                    "last_host_monotonic_ns": (
                        quest_host_mono_window[1]
                        if quest_host_mono_window
                        else None
                    ),
                    "poll_rate_hz": args.quest_rate,
                    "polls": poller.samples if poller is not None else 0,
                    "poller_overruns": (
                        poller.overruns if poller is not None else 0
                    ),
                },
                "cameras": {
                    "enabled": bool(camera_slots),
                    "backend": "linux",
                    "timestamp_clock": "host_unix_ns",
                    "mode": (
                        "none"
                        if not camera_slots
                        else "both"
                        if len(camera_slots) == 2
                        else camera_slots[0]
                    ),
                    "cap_side": args.cap_side,
                    "slots": list(camera_slots),
                    "camera_ids": list(camera_ids),
                    "first_frame_s": camera_first_frame,
                    "per_camera": camera_metadata,
                },
                "runtime": {
                    "abort_reason": ABORT_REASON,
                    "configured_sources": (
                        ["glove"]
                        + (["camera"] if camera_slots else [])
                        + ([] if args.no_quest else ["quest"])
                    ),
                    "camera_frame_check_hz": 1.0 / RuntimeMonitor.FRAME_CHECK_S,
                    "camera_stall_limit_s": RuntimeMonitor.CAMERA_STALL_S,
                    "glove_stall_limit_s": RuntimeMonitor.GLOVE_STALL_S,
                    "quest_valid_grace_s": RuntimeMonitor.QUEST_VALID_GRACE_S,
                    "quest_valid_stall_limit_s": RuntimeMonitor.QUEST_VALID_STALL_S,
                    "quest_pose_stale_warn_s": QUEST_POSE_STALE_WARN_S,
                    "quest_pose_stale_warnings": (
                        monitor.pose_stale_warnings if monitor else 0
                    ),
                    "camera_stalls": monitor.camera_stalls if monitor else 0,
                    "glove_stalls": monitor.glove_gaps if monitor else 0,
                },
                "session": {
                    "started_at": (
                        start_info.started_at.isoformat(
                            timespec="milliseconds"
                        )
                        if start_info is not None
                        else None
                    ),
                    "started_unix_ns": (
                        start_info.started_unix_ns
                        if start_info is not None
                        else None
                    ),
                    "started_monotonic_ns": (
                        start_info.started_monotonic_ns
                        if start_info is not None
                        else None
                    ),
                    "process_started_at": session_started_at.isoformat(
                        timespec="milliseconds"
                    ),
                    "ready_wait_s": (
                        round(start_info.ready_wait_s, 3)
                        if start_info is not None
                        else None
                    ),
                    "key_wait_s": (
                        round(start_info.key_wait_s, 3)
                        if start_info is not None
                        else None
                    ),
                    "auto_start": (
                        bool(start_info.auto_start)
                        if start_info is not None
                        else False
                    ),
                    "standby": {
                        "quest_samples_discarded": (
                            start_info.quest_discarded
                            if start_info is not None
                            else 0
                        ),
                        "hand_frames_discarded": (
                            start_info.hand_discarded
                            if start_info is not None
                            else 0
                        ),
                    },
                },
                "verify": verify,
                "env": environment_info(ffmpeg),
                "error": (
                    str(startup_error) if startup_error is not None else None
                ),
            }
            manifest = recorder.manifest(metadata)
            recorder.write_meta(
                {
                    key: manifest[key]
                    for key in (
                        "format_version",
                        "started_at",
                        "ended_at",
                        "rows",
                        "actual_rate_hz",
                        "duration_s",
                        "alignment",
                        "pose",
                        "hand",
                        "quest",
                        "cameras",
                        "session",
                        "runtime",
                        "verify",
                        "env",
                    )
                }
            )
            recorder.__exit__(None, None, None)
            if live_view is not None:
                live_view.close()
                if live_view.dropped:
                    log(
                        f"[INFO] 实时视图丢弃 {live_view.dropped} 帧"
                        "（视图跟不上，录制不受影响）"
                    )
            manifest_path = output_dir / "manifest.json"
            with manifest_path.open("x", encoding="utf-8") as file:
                json.dump(
                    manifest,
                    file,
                    ensure_ascii=False,
                    indent=2,
                    allow_nan=False,
                )
                file.write("\n")

            if hand_stopped:
                call_with_timeout(
                    "断开 PXCap",
                    hand_collector.disconnect,
                    2.0,
                )
            else:
                warn("PXCap 停止超时，跳过断开调用以避免再次阻塞")
            log(
                f"[INFO] 采集结束：手套 {recorder.cap_rows} 帧，"
                f"Quest {recorder.quest_rows} 样本"
            )
            for camera_id in camera_ids:
                video_entry = camera_videos.get(camera_id)
                if isinstance(video_entry, tuple):
                    video, _ = video_entry
                    log(f"[INFO] {camera_id}：{video.frames} 帧")
            for camera in cameras:
                if camera.preview_fps > 0:
                    average = (
                        camera.preview_bytes / camera.preview_frames / 1024.0
                        if camera.preview_frames
                        else 0.0
                    )
                    log(
                        f"[INFO] {camera.camera_id} 实时预览："
                        f"{camera.preview_frames} 帧，平均 {average:.1f} KB/帧"
                    )
            log(f"[INFO] manifest：{manifest_path}")
        elif hand_collector is not None:
            if hand_stopped:
                call_with_timeout(
                    "断开 PXCap",
                    hand_collector.disconnect,
                    2.0,
                )

        if not recorder_entered and isinstance(startup_error, StartAborted):
            # Ctrl+C before the key is a cancelled session, not a failure: no
            # data_* directory, no _startup_failures artifact, just a log line.
            if not output_dir_preexisting:
                shutil.rmtree(output_dir, ignore_errors=True)
            log("就绪前中止：已清理本次输出目录，未生成 data.h5")
        elif not recorder_entered and (
            startup_error is not None or exit_code != 0
        ):
            failure_root = output_dir.parent / "_startup_failures"
            stamp = session_started_at.strftime("%Y%m%d_%H%M%S")
            label = startup_failure_label(startup_error)
            base = failure_root / f"{stamp}_{label}"
            failure_dir = base
            suffix = 2
            while failure_dir.exists():
                failure_dir = base.with_name(f"{base.name}_{suffix}")
                suffix += 1
            failure_path = write_startup_failure(
                failure_dir,
                startup_error,
                started_at=session_started_at,
                cap_side=args.cap_side,
                pose_source=args.pose_source,
                pose_side=args.pose_side,
                pose_side_requested=pose_side_requested,
                hand_port=resolved_hand_port,
                camera_devices=camera_devices,
                cameras=cameras,
                ffmpeg=ffmpeg,
            )
            # The abandoned data_* dir may hold a camera pre-roll; its logs were
            # copied above, so remove it. A user-supplied pre-existing dir is
            # never deleted.
            if not output_dir_preexisting:
                shutil.rmtree(output_dir, ignore_errors=True)
            log(
                f"[INFO] 启动失败诊断：{failure_path}"
                "（本次 data_* 目录已清理，预录 mkv 不保留）"
            )

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
