#!/usr/bin/env python3
"""Resample the glove (cap) stream onto the camera frame timeline.

The collector deliberately never aligns sources; this is the downstream step that
does. Policy for this project (2026-11-17):

- Glove timestamps are treated as ground truth for *when a state was available*.
- Camera timestamps define the *sampling grid*: one output row per camera frame.
- The glove stream (120 Hz) is resampled onto that 33 ms grid by linear
  interpolation between the two bracketing glove frames.
- Quest is optional. New captures carry host receive timestamps and are aligned
  on that PC clock (``auto`` -> ``host``); old files fall back to ``unix`` or,
  with a passing C8 report, use ``sdk_unix``. Each camera target takes the last
  valid Quest row at or before the instant; missing or stale rows remain in the
  output with explicit invalid markers.
- Camera latency delta is measured once per machine (``Cap_Quest/latency``) and
  subtracted from camera timestamps, because a camera timestamp is the moment the
  frame reached the host, not the moment it was exposed.

Nothing here mutates the source ``data.h5``. Output goes to ``<recording>/aligned/``.

Usage::

    python align_capture.py <recording_dir or recordings_root> [options]

See README.md for the output schema and the quality gates.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from contextlib import closing
from pathlib import Path
from typing import Any, Sequence

import numpy as np

try:  # h5py is only needed for real files; --selftest avoids it
    import h5py
except ImportError:  # pragma: no cover - environment guard
    h5py = None  # type: ignore[assignment]


ALIGNED_FORMAT_VERSION = 2
#: 台账缺失时的兜底值。必须是 0，不能是某个历史实测数：192.3 ms 是旧 Windows
#: 采集路径的读数，V4L2 路径上没有任何依据，套用它会让时间轴整体偏 100+ ms，
#: 而产物每一列看起来都正常。
DEFAULT_DELAY_MS = 0.0
QUEST_ALIGNMENT_NONE = "none"
#: SDK 时间戳已经确认就是主机墙钟，直接用，不要求 C8 报告（老文件）。
QUEST_ALIGNMENT_UNIX = "unix"
#: 经 C8 报告的仿射模型换算到主机墙钟（更严谨，需要报告文件；老文件）。
QUEST_ALIGNMENT_SDK_UNIX = "sdk_unix"
#: New captures carry `quest/host_unix_ns`: align on the PC receive time; no C8.
QUEST_ALIGNMENT_HOST = "host"
#: Use host when present, else fall back to unix (default for new captures).
QUEST_ALIGNMENT_AUTO = "auto"
QUEST_ALIGNMENT_CHOICES = (
    QUEST_ALIGNMENT_NONE,
    QUEST_ALIGNMENT_AUTO,
    QUEST_ALIGNMENT_UNIX,
    QUEST_ALIGNMENT_HOST,
    QUEST_ALIGNMENT_SDK_UNIX,
)
#: 需要真正处理 Quest 的模式（auto 会先按文件解析成 host/unix）。
QUEST_ALIGNMENT_ACTIVE = (
    QUEST_ALIGNMENT_UNIX,
    QUEST_ALIGNMENT_HOST,
    QUEST_ALIGNMENT_SDK_UNIX,
)
def resolve_quest_alignment(handle: Any, mode: str) -> str:
    """Resolve ``auto`` from the file layout; other modes pass through.

    The host column is the capability; ``format_version`` 5 documents it. Keying
    on the column (not the version) keeps a hand-edited/partial old file safely
    on the ``unix`` fallback.
    """
    if mode != QUEST_ALIGNMENT_AUTO:
        return mode
    return (
        QUEST_ALIGNMENT_HOST
        if "quest" in handle and "quest/host_unix_ns" in handle
        else QUEST_ALIGNMENT_UNIX
    )


#: 四元数落在位姿末端的位数（x,y,z,qx,qy,qz,qw）。
POSE_QUAT_TAIL = 4
DEFAULT_QUEST_MAX_AGE_MS = 25.0
DEFAULT_QUEST_MIN_VALID_FRACTION = 0.95
#: Float channels: linear interpolation between bracketing glove frames.
LINEAR_STATE = ("joint_angles_deg", "resultant_force")
#: Integer channels: nearest frame, because a counter is not a signal.
NEAREST_STATE = ("encoder_raw", "distribute_counts")
#: Optional heavy channel (7.1 KB/row); interpolated unless --omit-distribution.
DISTRIBUTION_STATE = "distribute_force"

#: A gap wider than this multiple of the median interval is reported as a hole.
HOLE_FACTOR = 1.5


# ---------------------------------------------------------------------------
# logging
# ---------------------------------------------------------------------------


def log(message: str) -> None:
    print(message, flush=True)


def warn(message: str) -> None:
    print(f"[WARN] {message}", flush=True)


def error(message: str) -> None:
    print(f"[ERROR] {message}", file=sys.stderr, flush=True)


class AlignError(RuntimeError):
    """Fatal problem with the input data or the requested alignment."""


def load_quest_clock_report(path: Path) -> tuple[dict[str, Any], str]:
    """Load a passing C8 report and return it with its SHA-256 digest."""
    try:
        raw = path.expanduser().read_bytes()
        payload = json.loads(raw.decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise AlignError(f"无法读取 C8 时钟报告 {path}：{exc}") from exc
    if not isinstance(payload, dict):
        raise AlignError("C8 时钟报告顶层必须是对象")
    if payload.get("kind") != "quest_clock_probe":
        raise AlignError("C8 时钟报告类型不正确")
    if payload.get("passed") is not True:
        raise AlignError("C8 时钟报告未通过，禁止启用 sdk_unix 对齐")
    if payload.get("clock_domain") != "unix_epoch":
        raise AlignError("C8 时钟报告未确认 SDK 时间属于 Unix epoch")
    model = (payload.get("models") or {}).get("host_unix")
    if not isinstance(model, dict):
        raise AlignError("C8 时钟报告缺少 host_unix 模型")
    try:
        slope = float(model["sdk_per_host_slope"])
        float(model["intercept_s"])
        int(model["reference_host_ns"])
        int(model["reference_sdk_ns"])
    except (KeyError, TypeError, ValueError) as exc:
        raise AlignError("C8 时钟报告中的 host_unix 模型字段无效") from exc
    if not np.isfinite(slope) or slope <= 0:
        raise AlignError("C8 时钟报告的时钟斜率无效")
    return payload, hashlib.sha256(raw).hexdigest()


def sdk_unix_to_host_unix_ns(
    sdk_ns: np.ndarray,
    model: dict[str, Any],
) -> np.ndarray:
    """Invert ``sdk = slope * host + intercept`` for an array of SDK stamps."""
    values = np.asarray(sdk_ns, dtype=np.int64)
    reference_host_ns = int(model["reference_host_ns"])
    reference_sdk_ns = int(model["reference_sdk_ns"])
    slope = float(model["sdk_per_host_slope"])
    intercept_s = float(model["intercept_s"])
    relative_s = (
        (values.astype(np.float64) - float(reference_sdk_ns)) / 1e9
        - intercept_s
    ) / slope
    host = np.rint(
        float(reference_host_ns) + relative_s * 1e9
    ).astype(np.int64)
    return host


def quest_timestamp_quality(
    raw_ns: np.ndarray,
    poll_rate_hz: float = 120.0,
) -> dict[str, Any]:
    """Measure the file-level C8 gates without changing the source stamps.

    A pull loop polls faster than the SDK updates, so one SDK timestamp can
    repeat for several rows (26 at 120 Hz was measured). The C8 gate is a
    *duration*: ``max_same_timestamp_run / poll_rate_hz``. The poll rate comes
    from ``quest.poll_rate_hz`` in the manifest when available; 120 Hz is the
    writer's default for older files.
    """
    raw = np.asarray(raw_ns, dtype=np.int64)
    valid = raw[raw > 0]
    valid_count = int(valid.size)
    max_run = 0
    run = 0
    previous = -1
    for value in raw:
        value = int(value)
        if value > 0 and value == previous:
            run += 1
        elif value > 0:
            run = 1
        else:
            run = 0
        max_run = max(max_run, run)
        previous = value
    rate = float(poll_rate_hz)
    if not np.isfinite(rate) or rate <= 0.0:
        rate = 120.0
    return {
        "rows": int(raw.size),
        "valid_count": valid_count,
        "valid_source_fraction": valid_count / raw.size if raw.size else 0.0,
        "unique_count": int(np.unique(valid).size),
        "backwards_steps": (
            int(np.count_nonzero(np.diff(valid) < 0))
            if valid.size > 1
            else 0
        ),
        "max_same_timestamp_run": max_run,
        "max_same_timestamp_run_s": max_run / rate,
        "poll_rate_hz": rate,
    }


def _quest_poll_rate_hz(handle: Any) -> float:
    """Read ``quest.poll_rate_hz`` from the HDF5 ``meta`` attrs, default 120."""
    try:
        quest_meta = handle["meta"].attrs.get("quest")
        if isinstance(quest_meta, (str, bytes)):
            quest_meta = json.loads(quest_meta)
        if isinstance(quest_meta, dict):
            value = float(quest_meta.get("poll_rate_hz") or 0.0)
            if np.isfinite(value) and value > 0.0:
                return value
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        pass
    return 120.0


WRITE_FLAG_HINT = (
    "文件带着“上次是写打开”的一致性标志（采集被强杀后 HDF5 没来得及清），"
    "h5py 会拒绝只读打开。清它只改超级块 1 个字节并重算超级块校验和，数据内容不动："
    "python align_capture.py <同一目录> --clear-write-flag"
)

_M32 = 0xFFFFFFFF


def _rot(value: int, bits: int) -> int:
    return ((value << bits) | (value >> (32 - bits))) & _M32


def lookup3(data: bytes, initval: int = 0) -> int:
    """Bob Jenkins' lookup3 (hashlittle), as used by HDF5 for superblock checksums."""
    size = len(data)
    a = b = c = (0xDEADBEEF + size + initval) & _M32
    pos = 0
    remaining = size
    while remaining > 12:
        w0, w1, w2 = (
            int.from_bytes(data[pos : pos + 4], "little"),
            int.from_bytes(data[pos + 4 : pos + 8], "little"),
            int.from_bytes(data[pos + 8 : pos + 12], "little"),
        )
        a = (a + w0) & _M32
        b = (b + w1) & _M32
        c = (c + w2) & _M32
        a = (a - c) & _M32; a ^= _rot(c, 4); c = (c + b) & _M32
        b = (b - a) & _M32; b ^= _rot(a, 6); a = (a + c) & _M32
        c = (c - b) & _M32; c ^= _rot(b, 8); b = (b + a) & _M32
        a = (a - c) & _M32; a ^= _rot(c, 16); c = (c + b) & _M32
        b = (b - a) & _M32; b ^= _rot(a, 19); a = (a + c) & _M32
        c = (c - b) & _M32; c ^= _rot(b, 4); b = (b + a) & _M32
        pos += 12
        remaining -= 12
    if remaining == 0:
        return c
    for i in range(remaining):
        byte = data[pos + i]
        if i < 4:
            a = (a + (byte << (8 * i))) & _M32
        elif i < 8:
            b = (b + (byte << (8 * (i - 4)))) & _M32
        else:
            c = (c + (byte << (8 * (i - 8)))) & _M32
    c ^= b; c = (c - _rot(b, 14)) & _M32
    a ^= c; a = (a - _rot(c, 11)) & _M32
    b ^= a; b = (b - _rot(a, 25)) & _M32
    c ^= b; c = (c - _rot(b, 16)) & _M32
    a ^= c; a = (a - _rot(c, 4)) & _M32
    b ^= a; b = (b - _rot(a, 14)) & _M32
    c ^= b; c = (c - _rot(b, 24)) & _M32
    return c


def clear_write_flag(path: Path) -> bool:
    """Clear HDF5's "file was opened for write" consistency flag in place.

    Equivalent to ``h5clear -s``: the status byte is zeroed *and* the superblock
    checksum is recomputed — flipping the byte alone makes the file fail with
    "incorrect metadata checksum", which looks like corruption but is not.
    Returns True if it patched.
    """
    with open(path, "r+b") as handle:
        head = bytearray(handle.read(48))
        if head[:8] != b"\x89HDF\r\n\x1a\n":
            return False
        version = head[8]
        if version in (0, 1):
            flags_at, checksum_at, covered = 20, None, 0
        elif version in (2, 3):
            flags_at, checksum_at, covered = 11, 44, 44
        else:
            return False
        if all(byte == 0 for byte in head[flags_at : flags_at + (4 if version in (0, 1) else 1)]):
            return False
        width = 4 if version in (0, 1) else 1
        head[flags_at : flags_at + width] = b"\x00" * width
        handle.seek(0)
        if checksum_at is not None:
            head[checksum_at : checksum_at + 4] = lookup3(bytes(head[:covered])).to_bytes(4, "little")
        handle.write(bytes(head))
        return True


# ---------------------------------------------------------------------------
# input discovery
# ---------------------------------------------------------------------------


def find_recordings(path: Path) -> list[Path]:
    """Accept one recording dir, or a root whose children are recordings."""
    if (path / "data.h5").is_file():
        return [path]
    if not path.is_dir():
        raise AlignError(f"路径不存在：{path}")
    found = sorted(p for p in path.iterdir() if (p / "data.h5").is_file())
    if not found:
        raise AlignError(f"{path} 下没有找到 data.h5（单个采集目录或 recordings 根目录都行）")
    return found


def camera_slots(handle: Any) -> list[str]:
    if "cameras" not in handle:
        return []
    return sorted(handle["cameras"].keys())


# ---------------------------------------------------------------------------
# timebase
# ---------------------------------------------------------------------------


def has_mono(group: Any) -> bool:
    return "timestamp_monotonic_ns" in group


def choose_timebase(handle: Any) -> str:
    """Pick the clock used to pair sources. No CLI override.

    Monotonic is only usable when *every* source that defines the grid carries
    it: the grid is a subtraction between the reference camera and the glove, so
    one side in a different clock domain is meaningless.

    In practice this always returns ``"unix"``: the V4L2 backend leaves
    ``timestamp_monotonic_ns`` absent by design (``write_camera_timestamps`` is
    called with ``monotonic_ns=None``), so the camera is wall-clock only. The
    ``mono`` branch is kept so that a future backend that reads V4L2 buffer
    timestamps starts working without touching this file. There is deliberately
    no ``--timebase`` flag: it used to offer ``mono``, which could never succeed.
    """
    slots = camera_slots(handle)
    if has_mono(handle["cap"]) and slots and all(has_mono(handle["cameras"][s]) for s in slots):
        return "mono"
    return "unix"


def detect_clock_jumps(unix_ns: np.ndarray, mono_ns: np.ndarray, threshold_ns: int = 5_000_000) -> int:
    """Count NTP-like steps in the glove wall clock. Only meaningful for unix timebase."""
    if unix_ns.size < 3:
        return 0
    offset = unix_ns.astype(np.int64) - mono_ns.astype(np.int64)
    return int(np.count_nonzero(np.abs(np.diff(offset)) > threshold_ns))


# ---------------------------------------------------------------------------
# camera latency delta
# ---------------------------------------------------------------------------


def resolve_delay(explicit: float | None, disabled: bool) -> tuple[float, str]:
    """Return (delta_ms, source). Ledger first, then "uncalibrated, zero".

    The fallback used to be 192.3 ms — one machine's Windows-path measurement.
    On the native V4L2 path that number has no basis and biases the whole time
    axis by 100+ ms while every column still looks healthy, so the fallback is
    now 0 and it says so loudly.
    """
    if disabled:
        return 0.0, "disabled (--no-delay)"
    if explicit is not None:
        return float(explicit), "命令行 --delay-ms"
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Cap_Quest" / "latency"))
        from latency_records import load_records, pick_latest  # type: ignore

        entry = pick_latest(load_records(), camera=0)
        if entry:
            return float(entry["median_ms"]), (
                f"latency/records 台账 cam0 @{entry.get('at', '')}，两路共用同一值（cam1 实测 87.6 ms，未逐路补偿）"
            )
        detail = "台账里没有 cam0 的合格记录"
    except Exception as exc:  # missing PyYAML, missing file, whatever
        detail = f"读不到延迟台账（{exc.__class__.__name__}: {exc}）"
    warn(
        f"{detail}：本次按**未标定**处理，Δ=0，时间轴不做延迟补偿。"
        "先跑 Cap_Quest/latency/measure_camera_latency.py，或用 --delay-ms 显式给值；"
        "带 Δ 采集与非带 Δ 采集的产物不能混着训练"
    )
    return DEFAULT_DELAY_MS, "未标定 Δ=0（台账不可用，未做延迟补偿）"


# ---------------------------------------------------------------------------
# resampling
# ---------------------------------------------------------------------------


def bracket_indices(src_ns: np.ndarray, dst_ns: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """For every dst time, return (left, right, w) with w the fraction in [0, 1].

    dst times outside [src[0], src[-1]] are expected to be filtered by the caller;
    the clamp here is a belt-and-braces guard so w never leaves [0, 1].
    """
    left = np.searchsorted(src_ns, dst_ns, side="right") - 1
    left = np.clip(left, 0, max(src_ns.size - 2, 0))
    right = left + 1
    span = (src_ns[right] - src_ns[left]).astype(np.float64)
    with np.errstate(divide="ignore", invalid="ignore"):
        w = np.where(span > 0, (dst_ns - src_ns[left]) / np.where(span > 0, span, 1.0), 0.0)
    w = np.clip(w, 0.0, 1.0)
    return left, right, w


def gather(dataset: Any, indices: np.ndarray) -> np.ndarray:
    """Read only the rows we need, preserving the requested order."""
    uniq, inverse = np.unique(indices, return_inverse=True)
    block = np.asarray(dataset[uniq])
    return block[inverse]


def lerp_rows(
    dataset: Any,
    left: np.ndarray,
    right: np.ndarray,
    w: np.ndarray,
    dtype: np.dtype,
    chunk: int = 512,
) -> np.ndarray:
    """Linear interpolation of a (N, ...) dataset, cast back to its own dtype.

    Chunked along rows: distribution force is 3570 values per row, so a whole-file
    float64 temporary would be hundreds of MB on a long recording. Work dtype is
    float32 for wide rows (forces) and float64 for narrow ones (joint angles).
    """
    per_row = int(np.prod(dataset.shape[1:], dtype=np.int64)) if dataset.ndim > 1 else 1
    work = np.float32 if per_row > 64 else np.float64
    out = np.empty((left.size,) + dataset.shape[1:], dtype=dtype)
    for start in range(0, left.size, chunk):
        stop = min(start + chunk, left.size)
        l_idx, r_idx, weight = left[start:stop], right[start:stop], w[start:stop].astype(work)
        block = gather(dataset, np.concatenate([l_idx, r_idx]))
        a = block[: l_idx.size].astype(work)
        b = block[l_idx.size :].astype(work)
        shape = (-1,) + (1,) * (a.ndim - 1)
        tmp = a * (1.0 - weight.reshape(shape)) + b * weight.reshape(shape)
        out[start:stop] = cast_to(tmp, dtype)
    return out


def nearest_rows(dataset: Any, left: np.ndarray, right: np.ndarray, w: np.ndarray) -> np.ndarray:
    """Pick the closer of the two bracketing rows (no averaging of counters)."""
    take_right = w >= 0.5
    idx = np.where(take_right, right, left)
    return gather(dataset, idx)


def cast_to(values: np.ndarray, dtype: np.dtype) -> np.ndarray:
    if np.issubdtype(dtype, np.integer):
        info = np.iinfo(dtype)
        return np.clip(np.rint(values), info.min, info.max).astype(dtype)
    return values.astype(dtype)


# ---------------------------------------------------------------------------
# core alignment
# ---------------------------------------------------------------------------


def open_recording(recording: Path, clear_flag: bool = False) -> Any:
    """Open data.h5 read-only, repairing the crashed-write consistency flag if asked."""
    if h5py is None:
        raise AlignError("缺少 h5py：换一个装了 h5py 的 Python 解释器（先确认 `python3 -c \"import h5py\"` 能过）")
    path = recording / "data.h5"
    try:
        return h5py.File(path, "r")
    except OSError as exc:
        text = str(exc)
        stale = "already open for write" in text or "consistency flags" in text
        if not stale:
            raise AlignError(f"打不开 {path.name}：{text}") from exc
        if not clear_flag:
            raise AlignError(WRITE_FLAG_HINT) from exc
        if not clear_write_flag(path):
            raise AlignError("一致性标志清除失败：超级块版本不认识，或标志本来就是 0") from exc
        log(f"[INFO] 已清除 {path.parent.name} 的写标志（改超级块 1 字节 + 重算校验和）")
        try:
            return h5py.File(path, "r")
        except Exception as exc2:  # h5py 对元数据坏掉抛的是 RuntimeError，不是 OSError
            detail = str(exc2)
            if "object header" in detail or "checksum" in detail:
                raise AlignError(
                    "超级块是完好的（标志已清、校验和对得上），但根组对象头读不出来 —— "
                    "采集被强杀时 HDF5 的元数据缓存没落盘（file.flush() 只在 close() 里调），"
                    "这份采集整份作废、只能重采。注意：不是只丢尾部 100 行"
                ) from exc2
            raise AlignError(f"清标志后仍打不开：{detail}") from exc2


def align_quest_to_targets(
    handle: Any,
    target_unix_ns: np.ndarray,
    mode: str,
    clock_report: dict[str, Any] | None,
    report_sha256: str | None,
    max_age_ms: float,
) -> dict[str, Any]:
    """Pair each camera instant with the last valid Quest sample at or before it.

    Rows are copied whole, never interpolated. A pose is a discrete measurement,
    and blending two of them would need quaternion handling whose only gain is
    half a sample period -- less than the jitter it would paper over. The cost
    of taking the previous row is a bounded lag, reported per row as ``age_ms``
    so downstream can filter on it exactly like ``glove/span_ms``.

    Three clock paths:

    * ``host`` -- use ``quest/host_unix_ns`` (new captures; PC receive time,
      same clock as the camera/glove; no C8 report).
    * ``unix`` -- ``sdk_timestamp_ns`` is already host wall clock, use as-is
      (old files without host columns).
    * ``sdk_unix`` -- apply the affine model from a passing C8 report first.
    """
    mode = resolve_quest_alignment(handle, mode)
    stamp_path = (
        "quest/host_unix_ns"
        if mode == QUEST_ALIGNMENT_HOST
        else "quest/sdk_timestamp_ns"
    )
    if "quest" not in handle or stamp_path not in handle:
        raise AlignError(f"请求 Quest 对齐，但 data.h5 里没有 {stamp_path}")
    raw = np.asarray(handle[stamp_path][:], dtype=np.int64)
    quality = quest_timestamp_quality(raw, _quest_poll_rate_hz(handle))

    limits = (clock_report or {}).get("thresholds") or {}
    min_valid_fraction = float(
        limits.get("min_valid_fraction", DEFAULT_QUEST_MIN_VALID_FRACTION)
    )
    max_backwards = int(limits.get("max_backwards_steps", 0))

    failures: list[str] = []
    warnings: list[str] = []
    if quality["unique_count"] < 1:
        failures.append("没有任何有效 Quest 时间戳")
    if quality["backwards_steps"] > max_backwards:
        failures.append(f"时间戳倒退 {quality['backwards_steps']} 次，时间轴非单调")
    max_run_s = float(limits.get("max_same_timestamp_run_s", 1.0))
    repeated = (
        f"同一时间戳连续重复 {quality['max_same_timestamp_run']} 次 / "
        f"{quality['max_same_timestamp_run_s']:.3f} 秒，超过 {max_run_s:g} 秒"
    )
    if quality["max_same_timestamp_run_s"] > max_run_s:
        # 拉取模型下重复是常态。sdk_unix 有 C8 报告背书，超限即判失败；
        # host/unix 模式只提示——重复不破坏「同一条时间轴」这个前提。
        (failures if mode == QUEST_ALIGNMENT_SDK_UNIX else warnings).append(repeated)
    # Only sdk_unix hard-gates source coverage (the C8 path). host is a
    # measured PC clock: missing rows simply become quest/valid=false, and
    # unix is a convention with the same per-row semantics.
    if (
        mode == QUEST_ALIGNMENT_SDK_UNIX
        and quality["valid_source_fraction"] < min_valid_fraction
    ):
        failures.append(
            f"有效时间戳比例 {quality['valid_source_fraction']:.3f} "
            f"低于 {min_valid_fraction:.3f}"
        )
    if failures:
        raise AlignError("当前 data.h5 不满足 Quest 对齐条件：" + "；".join(failures))
    for message in warnings:
        warn(f"Quest 时间戳质量提示：{message}（继续处理，缺的行体现为 quest/valid=false）")

    model: dict[str, Any] | None = None
    if mode == QUEST_ALIGNMENT_SDK_UNIX:
        candidate = (clock_report or {}).get("models", {}).get("host_unix")
        if not isinstance(candidate, dict):
            raise AlignError("C8 时钟报告缺少 host_unix 模型")
        model = candidate

    # 「有时间戳」不等于「有姿态」：PC Service 掉线后 getter 会返回默认值，
    # 时间戳可能仍在走而 pose_6d 已经整段是 NaN。这种行如果进了候选集，会产出
    # 一份 pose 全 NaN 但 valid=true 的产物——训练侧无从分辨。所以候选集要求
    # 位置三个分量都有限；坏行直接不进候选，前驱自然跳到更早的有效样本上，
    # 如果那段太长就会被 max_age 判成 valid=false。
    stamp_indices = np.flatnonzero(raw > 0).astype(np.int64)
    if stamp_indices.size < 1:
        raise AlignError("Quest 没有任何有效时间戳")
    pose_excluded = 0
    if "quest/pose_6d" in handle:
        pose = np.asarray(handle["quest/pose_6d"][:], dtype=np.float64)
        if pose.shape[0] != raw.size or pose.ndim != 2 or pose.shape[1] < 3:
            raise AlignError(f"quest/pose_6d 形状与 {stamp_path} 不匹配")
        position_finite = np.isfinite(pose[:, :3]).all(axis=1)
        keep_pose = position_finite[stamp_indices]
        pose_excluded = int(stamp_indices.size - np.count_nonzero(keep_pose))
        valid_indices = stamp_indices[keep_pose]
        if valid_indices.size < 1:
            raise AlignError(
                "Quest 有有效时间戳，但 pose_6d 的位置分量全部是 NaN："
                "PC Service 掉线后 getter 返回了默认值。这份采集没有可用的姿态数据"
            )
        if pose_excluded:
            warn(
                f"Quest 有 {pose_excluded} 行时间戳有效但姿态为 NaN，"
                "已从候选里剔除（这些位置会表现为 quest/valid=false）"
            )
    else:
        valid_indices = stamp_indices
    if valid_indices.size < 1:
        raise AlignError("Quest 没有任何有效样本")
    valid_raw = raw[valid_indices]
    mapped = (
        sdk_unix_to_host_unix_ns(valid_raw, model)
        if model is not None
        else valid_raw
    )
    if mapped.size > 1 and np.any(np.diff(mapped) < 0):
        raise AlignError("换算后的 Quest 主机时间戳出现倒退")

    targets = np.asarray(target_unix_ns, dtype=np.int64)
    # 严格不晚于目标的最后一条。相同时间戳重复时 side="right" 天然取到最后一条。
    positions = np.searchsorted(mapped, targets, side="right") - 1
    has_candidate = positions >= 0
    selected = np.full(targets.shape, -1, dtype=np.int64)
    selected_host = np.full(targets.shape, -1, dtype=np.int64)
    if np.any(has_candidate):
        picked = positions[has_candidate]
        selected[has_candidate] = valid_indices[picked]
        selected_host[has_candidate] = mapped[picked]
    age_ms = np.full(targets.shape, np.nan, dtype=np.float64)
    if np.any(has_candidate):
        age_ms[has_candidate] = (
            targets[has_candidate] - selected_host[has_candidate]
        ) / 1e6
    valid = has_candidate & (age_ms >= 0.0) & (age_ms <= float(max_age_ms))
    selected[~valid] = -1
    selected_host[~valid] = -1
    valid_age = age_ms[valid]
    age_stats = {
        "median": float(np.median(valid_age)) if valid_age.size else None,
        "p95": float(np.percentile(valid_age, 95)) if valid_age.size else None,
        "p99": float(np.percentile(valid_age, 99)) if valid_age.size else None,
        "max": float(np.max(valid_age)) if valid_age.size else None,
    }
    sdk_all = (
        np.asarray(handle["quest/sdk_timestamp_ns"][:], dtype=np.int64)
        if "quest/sdk_timestamp_ns" in handle
        else np.full(raw.size, -1, dtype=np.int64)
    )
    timestamp_sdk_ns = np.full(targets.shape, -1, dtype=np.int64)
    if np.any(valid):
        timestamp_sdk_ns[valid] = sdk_all[selected[valid]]
    return {
        "enabled": True,
        "mode": mode,
        "target_clock": "host_unix_ns",
        "source_rows": int(raw.size),
        "pose_excluded_rows": int(pose_excluded),
        "index": selected,
        "valid": valid,
        "age_ms": age_ms,
        "age_stats_ms": age_stats,
        "timestamp_sdk_ns": timestamp_sdk_ns,
        "timestamp_host_unix_ns": selected_host,
        "paired_rows": int(np.count_nonzero(valid)),
        "paired_fraction": float(np.count_nonzero(valid) / max(targets.size, 1)),
        "max_age_ms": float(max_age_ms),
        "quality": quality,
        "quality_warnings": list(warnings),
        "clock_report_sha256": report_sha256 if mode == QUEST_ALIGNMENT_SDK_UNIX else None,
        "clock_model": model,
    }


def quest_valid_window(
    handle: Any,
    clock_report: dict[str, Any] | None,
    mode: str,
) -> tuple[int, int] | None:
    """First/last host-Unix instant covered by a valid Quest stamp.

    ``unix`` mode compares the SDK stamps with the camera grid directly;
    ``sdk_unix`` maps them through the C8 model first. Anything else opt-in
    needs a model, so an absent report yields no window.
    """
    mode = resolve_quest_alignment(handle, mode)
    stamp_path = (
        "quest/host_unix_ns"
        if mode == QUEST_ALIGNMENT_HOST
        else "quest/sdk_timestamp_ns"
    )
    if stamp_path not in handle:
        return None
    model: dict[str, Any] | None = None
    if mode == QUEST_ALIGNMENT_SDK_UNIX:
        model = (clock_report or {}).get("models", {}).get("host_unix")
        if not isinstance(model, dict):
            return None
    raw = np.asarray(handle[stamp_path][:], dtype=np.int64)
    valid = raw[raw > 0]
    if valid.size == 0:
        return None
    mapped = (
        sdk_unix_to_host_unix_ns(valid, model)
        if model is not None
        else valid
    )
    if mapped.size == 0:
        return None
    return int(mapped.min()), int(mapped.max())


def align_recording(
    recording: Path,
    reference: str | None,
    delay_ms: float,
    omit_distribution: bool,
    interpolate_distribution: bool,
    clear_flag: bool = False,
    quest_alignment: str = QUEST_ALIGNMENT_NONE,
    quest_clock_report: dict[str, Any] | None = None,
    quest_report_sha256: str | None = None,
    quest_max_age_ms: float = DEFAULT_QUEST_MAX_AGE_MS,
    quest_min_valid_fraction: float = DEFAULT_QUEST_MIN_VALID_FRACTION,
) -> dict[str, Any]:
    """Build the aligned arrays for one recording. Returns a result bundle."""
    if quest_alignment not in QUEST_ALIGNMENT_CHOICES:
        raise AlignError(f"未知 Quest 对齐模式：{quest_alignment!r}")
    if quest_alignment in QUEST_ALIGNMENT_ACTIVE:
        if not np.isfinite(quest_max_age_ms) or quest_max_age_ms < 0:
            raise AlignError("Quest 最大年龄必须是有限的非负数")
        if not 0.0 <= quest_min_valid_fraction <= 1.0:
            raise AlignError("Quest 最小有效比例必须在 0..1")
    if quest_alignment == QUEST_ALIGNMENT_SDK_UNIX:
        if quest_clock_report is None or not quest_report_sha256:
            raise AlignError("sdk_unix 对齐需要已验证的 C8 时钟报告")

    with closing(open_recording(recording, clear_flag)) as handle:
        if "cap" not in handle:
            raise AlignError("data.h5 里没有 cap/ 组，不是本工程的采集文件")
        # ``auto`` is file-dependent: new captures have host receive columns,
        # old ones only have the SDK stamps.
        quest_alignment = resolve_quest_alignment(handle, quest_alignment)
        if quest_alignment in QUEST_ALIGNMENT_ACTIVE:
            if not np.isfinite(quest_max_age_ms) or quest_max_age_ms < 0:
                raise AlignError("Quest 最大年龄必须是有限的非负数")
            if not 0.0 <= quest_min_valid_fraction <= 1.0:
                raise AlignError("Quest 最小有效比例必须在 0..1")
        slots = camera_slots(handle)
        if not slots:
            raise AlignError("data.h5 里没有 cameras/ 组：这份采集没录视频，无法确定采样网格")

        tb = choose_timebase(handle)
        key = "timestamp_monotonic_ns" if tb == "mono" else "timestamp_unix_ns"

        glove_t = np.asarray(handle["cap"][key][:], dtype=np.int64)
        if glove_t.size < 2:
            raise AlignError(f"手套帧数 {glove_t.size} 太少，无法插值")
        if not np.all(np.diff(glove_t) > 0):
            raise AlignError("手套时间戳非严格递增（墙钟被校时过？），先按 DOC/时间戳说明.md §4.1 处理")

        jumps = 0
        cap_offset_ns: np.ndarray | None = None
        if has_mono(handle["cap"]):
            cap_unix = np.asarray(
                handle["cap"]["timestamp_unix_ns"][:], dtype=np.int64
            )
            cap_mono = np.asarray(
                handle["cap"]["timestamp_monotonic_ns"][:], dtype=np.int64
            )
            if cap_unix.shape == cap_mono.shape:
                # 墙钟 − 单调钟。同一份采集内这个值应当是常数，出现台阶就是
                # NTP 校时。逐帧留着，事后才能定位到「哪一行开始不可比」，
                # 只记一个跳变次数是不够查的。
                cap_offset_ns = cap_unix - cap_mono
                if tb == "unix" or quest_alignment in QUEST_ALIGNMENT_ACTIVE:
                    jumps = detect_clock_jumps(cap_unix, cap_mono)
        if quest_alignment in QUEST_ALIGNMENT_ACTIVE and jumps:
            raise AlignError(
                f"采集期间手套墙钟发生 {jumps} 次跳变，Quest 跨源绝对时间对齐不可信"
            )

        cam_times: dict[str, np.ndarray] = {}
        # Original frame index of every kept grid point. Duplicate timestamps
        # are collapsed to the first frame, but ``frame_index`` must still
        # point at the real mkv frame after the gap, not at the compressed row.
        cam_frame_indices: dict[str, np.ndarray] = {}
        camera_duplicates: dict[str, int] = {}
        for slot in slots:
            group = handle["cameras"][slot]
            if key not in group:
                raise AlignError(f"cameras/{slot} 缺少 {key}（时间基不一致）")
            series = np.asarray(group[key][:], dtype=np.int64)
            if series.size == 0:
                raise AlignError(f"cameras/{slot} 没有帧")
            if series.size > 1 and np.any(np.diff(series) < 0):
                raise AlignError(f"cameras/{slot} 时间戳倒退")
            # ffmpeg's V4L2 wall-clock can repeat a timestamp for two (or a
            # short burst of) frames. Keeping both would make the sampling
            # grid non-increasing and the "last frame at or before" lookup
            # ambiguous, so keep the first of each equal run and skip the
            # rest. The source data.h5 and the mkv are untouched.
            keep = np.ones(series.size, dtype=bool)
            if series.size > 1:
                keep[1:] = np.diff(series) > 0
            duplicates = int(series.size - np.count_nonzero(keep))
            if duplicates:
                cam_frame_indices[slot] = np.flatnonzero(keep).astype(np.int64)
                warn(
                    f"相机 {slot} 有 {duplicates} 个重复时间戳帧，"
                    "保留首帧、跳过其余（frame_index 仍指原始帧号）"
                )
            else:
                cam_frame_indices[slot] = np.arange(series.size, dtype=np.int64)
            cam_times[slot] = series[keep]
            camera_duplicates[slot] = duplicates

        if reference is None:
            reference = slots[0]
        if reference not in cam_times:
            raise AlignError(f"参考相机 {reference} 不在本文件里（有：{', '.join(slots)}）")

        reference_t = cam_times[reference]
        reference_source_rows = int(
            handle["cameras"][reference][key].shape[0]
        )
        if "timestamp_unix_ns" not in handle["cameras"][reference]:
            raise AlignError(f"cameras/{reference} 缺少 timestamp_unix_ns，无法对齐 Quest")
        reference_unix = np.asarray(
            handle["cameras"][reference]["timestamp_unix_ns"][:],
            dtype=np.int64,
        )[cam_frame_indices[reference]]
        if reference_unix.shape != reference_t.shape:
            raise AlignError(f"cameras/{reference} 的两个时间戳列长度不一致")
        if reference_unix.size > 1 and np.any(np.diff(reference_unix) < 0):
            raise AlignError(
                f"cameras/{reference} 的 Unix 时间戳倒退，无法对齐 Quest"
            )
        delay_ns = int(round(delay_ms * 1e6))
        master_t = reference_t - delay_ns
        master_unix_t = reference_unix - delay_ns

        # 公共有效窗口：每一路只覆盖它自己确实有数据的那一段，主轴取它们的交集。
        # 相机戳是「到达时刻」，主轴已经减过 Δ，所以每路相机的窗口边界也必须减
        # 同一个 Δ，否则窗口整体晚 Δ，要么丢掉最早的几十帧，要么留下没有图像的帧。
        # 计数是增量式的（手套 → 各相机 → Quest），所以各项相加正好等于总丢弃数；
        # 一行只算给第一个排除它的来源。
        # 默认就对 Quest 做对齐，所以「配了但这份文件没有 Quest」必须自动跳过，
        # 否则 --no-quest 采的每一份都会在批量里报错。
        quest_stamp_path = (
            "quest/host_unix_ns"
            if quest_alignment == QUEST_ALIGNMENT_HOST
            else "quest/sdk_timestamp_ns"
        )
        quest_rows_available = (
            int(handle[quest_stamp_path].shape[0])
            if quest_stamp_path in handle
            else 0
        )
        quest_active = (
            quest_alignment in QUEST_ALIGNMENT_ACTIVE and quest_rows_available > 0
        )
        quest_skipped_reason: str | None = None
        if quest_alignment in QUEST_ALIGNMENT_ACTIVE and not quest_active:
            quest_skipped_reason = (
                f"data.h5 里没有 {quest_stamp_path} 数据集"
                if quest_stamp_path not in handle
                else f"{quest_stamp_path} 是 0 行（采集时没拿到样本）"
            )

        windows: dict[str, dict[str, int]] = {}
        keep = np.ones(master_t.size, dtype=bool)

        windows["glove"] = {
            "first_timestamp_ns": int(glove_t[0]),
            "last_timestamp_ns": int(glove_t[-1]),
        }
        glove_ok = (master_t >= glove_t[0]) & (master_t <= glove_t[-1])
        dropped_by_glove = int(np.count_nonzero(keep & ~glove_ok))
        keep &= glove_ok

        dropped_by_camera: dict[str, int] = {}
        for slot in slots:
            first_ns = int(cam_times[slot][0]) - delay_ns
            last_ns = int(cam_times[slot][-1]) - delay_ns
            windows[f"camera:{slot}"] = {
                "first_timestamp_ns": first_ns,
                "last_timestamp_ns": last_ns,
            }
            camera_ok = (master_t >= first_ns) & (master_t <= last_ns)
            dropped_by_camera[slot] = int(np.count_nonzero(keep & ~camera_ok))
            keep &= camera_ok

        dropped_by_quest = 0
        if quest_active:
            quest_window = quest_valid_window(
                handle, quest_clock_report, quest_alignment
            )
            if quest_window is not None:
                windows["quest"] = {
                    "first_timestamp_ns": quest_window[0],
                    "last_timestamp_ns": quest_window[1],
                }
                quest_ok = (master_unix_t >= quest_window[0]) & (
                    master_unix_t <= quest_window[1]
                )
                dropped_by_quest = int(np.count_nonzero(keep & ~quest_ok))
                keep &= quest_ok

        dropped = int(keep.size - np.count_nonzero(keep))
        # 窗口外一律丢弃，不留「保留但仍标记」的口子：那些帧没有真实的手套数据，
        # bracket 会钳到首/末帧，结果只是边界帧的克隆，而 span_ms 看起来完全正常。
        master_t = master_t[keep]
        master_unix_t = master_unix_t[keep]
        ref_raw = reference_t[keep]
        ref_index = cam_frame_indices[reference][np.flatnonzero(keep)].astype(np.int32)

        if master_t.size == 0:
            raise AlignError(
                "四路（参考相机/其他相机/手套/Quest）的公共有效窗口是空的，"
                "没有一行能对齐（两次采集没有重叠，或 Quest 有效区间与相机不重叠）。"
                f"各路窗口：{json.dumps(windows, ensure_ascii=False)}"
            )

        left, right, w = bracket_indices(glove_t, master_t)

        # 每行取它所依赖的左邻手套帧的时钟偏移，粘在样本行上。
        clock_offset_ns = (
            cap_offset_ns[left].astype(np.int64) if cap_offset_ns is not None else None
        )

        state: dict[str, np.ndarray] = {}
        cap = handle["cap"]
        for name in LINEAR_STATE:
            if name in cap:
                state[name] = lerp_rows(cap[name], left, right, w, cap[name].dtype)
        for name in NEAREST_STATE:
            if name in cap:
                state[name] = nearest_rows(cap[name], left, right, w)

        distribution_note = "skipped"
        if DISTRIBUTION_STATE in cap:
            if omit_distribution:
                distribution_note = "omitted (--omit-distribution)"
            elif interpolate_distribution:
                state[DISTRIBUTION_STATE] = lerp_rows(
                    cap[DISTRIBUTION_STATE], left, right, w, cap[DISTRIBUTION_STATE].dtype
                )
                distribution_note = "interpolated"
            else:
                state[DISTRIBUTION_STATE] = nearest_rows(cap[DISTRIBUTION_STATE], left, right, w)
                distribution_note = "nearest (--nearest-distribution)"

        quest_result = None
        if quest_active:
            quest_result = align_quest_to_targets(
                handle,
                master_unix_t,
                quest_alignment,
                quest_clock_report,
                quest_report_sha256,
                quest_max_age_ms,
            )

        meta_attrs = {}
        for attr in ("format_version", "started_at", "ended_at", "hand", "cameras", "verify"):
            if attr in handle["meta"].attrs:
                raw = handle["meta"].attrs[attr]
                try:
                    meta_attrs[attr] = json.loads(raw) if isinstance(raw, (str, bytes)) else raw
                except (json.JSONDecodeError, TypeError):
                    meta_attrs[attr] = str(raw)

        # Per-camera nearest frame for every master row: multi-view training needs
        # the other images' frame indices on the same row.
        views: dict[str, dict[str, np.ndarray]] = {}
        for slot in slots:
            series = cam_times[slot]
            if slot == reference:
                views[slot] = {
                    "frame_index": ref_index,
                    "dt_ms": np.zeros(master_t.size, dtype=np.float32),
                    "timestamp_ns": ref_raw,
                }
                continue
            # 因果取帧：每行用「不晚于主轴时刻的最后一帧」，绝不用这一路
            # 目标时刻之后的画面。dt_ms 因此恒 <= 0，唯一例外是主轴早于该路
            # 首帧（窗口边界含 Δ 的边界情况），那时被迫取首帧、dt_ms > 0。
            # 远视角训练若改用未来帧，等于让模型看到还没发生的画面。
            pos = np.searchsorted(series, master_t, side="right") - 1
            idx = np.clip(pos, 0, series.size - 1)
            views[slot] = {
                "frame_index": cam_frame_indices[slot][idx].astype(np.int32),
                "dt_ms": ((series[idx] - master_t) / 1e6).astype(np.float32),
                "timestamp_ns": series[idx],
            }

    span_ns = glove_t[right] - glove_t[left]
    offset_ns = master_t - glove_t[left]
    holes = int(np.count_nonzero(np.diff(master_t) > HOLE_FACTOR * np.median(np.diff(master_t))))

    result = {
        "recording": recording,
        "reference": reference,
        "timebase": tb,
        "camera_slots": slots,
        "delay_ms": delay_ms,
        "glove_rows": int(glove_t.size),
        "camera_rows": int(reference_t.size),
        "camera_source_rows": reference_source_rows,
        "camera_duplicates": camera_duplicates,
        "aligned_rows": int(master_t.size),
        "dropped_outside": dropped,
        "windows": windows,
        "dropped_by_glove": dropped_by_glove,
        "dropped_by_camera": dropped_by_camera,
        "dropped_by_quest": dropped_by_quest,
        "clock_jumps": jumps,
        "clock_offset_ns": clock_offset_ns,
        "holes": holes,
        "state": state,
        "views": views,
        "timestamp_ns": master_t,
        "timestamp_cam_raw_ns": ref_raw,
        "glove_left": left.astype(np.int32),
        "glove_right": right.astype(np.int32),
        "weights": w,
        "glove_span_ms": span_ns.astype(np.float64) / 1e6,
        "glove_offset_ms": offset_ns.astype(np.float64) / 1e6,
        "distribution": distribution_note,
        "quest": quest_result,
        "quest_alignment": quest_alignment,
        "quest_skipped_reason": quest_skipped_reason,
        "quest_min_valid_fraction": float(quest_min_valid_fraction),
        "meta": meta_attrs,
    }
    return result


# ---------------------------------------------------------------------------
# output
# ---------------------------------------------------------------------------


def _copy_attrs(source: Any, target: Any) -> None:
    for key, value in source.attrs.items():
        target.attrs[key] = value


def _missing_value(dtype: np.dtype[Any], name: str) -> Any:
    if np.issubdtype(dtype, np.bool_):
        return False
    if np.issubdtype(dtype, np.floating):
        return np.nan
    if np.issubdtype(dtype, np.integer):
        return -1 if name.endswith("timestamp_ns") else 0
    if h5py.check_string_dtype(dtype) is not None:
        return ""
    raise AlignError(f"Quest 组含不支持补齐的数据类型：{dtype}")


def _write_selected_dataset(
    target: Any,
    name: str,
    source: Any,
    selected: np.ndarray,
    valid: np.ndarray,
    expected_rows: int,
) -> None:
    if source.ndim < 1:
        raise AlignError(f"Quest 数据集 {source.name} 没有行轴")
    if source.shape[0] != expected_rows:
        raise AlignError(
            f"Quest 数据集 {source.name} 行数 {source.shape[0]} "
            f"与 sdk_timestamp_ns 行数 {expected_rows} 不一致"
        )
    row_shape = source.shape[1:]
    out = target.create_dataset(
        name,
        shape=(selected.size,) + row_shape,
        dtype=source.dtype,
    )
    _copy_attrs(source, out)
    fill = _missing_value(source.dtype, name)
    chunk = 512
    for start in range(0, selected.size, chunk):
        stop = min(start + chunk, selected.size)
        idx = selected[start:stop]
        ok = valid[start:stop]
        block = np.full((idx.size,) + row_shape, fill, dtype=source.dtype)
        if np.any(ok):
            block[ok] = source[idx[ok]]
        out[start:stop] = block


def _write_selected_group(
    target: Any,
    source: Any,
    selected: np.ndarray,
    valid: np.ndarray,
    expected_rows: int,
) -> None:
    _copy_attrs(source, target)
    for name, item in source.items():
        if isinstance(item, h5py.Dataset):
            _write_selected_dataset(
                target,
                name,
                item,
                selected,
                valid,
                expected_rows,
            )
        elif isinstance(item, h5py.Group):
            child = target.create_group(name)
            _write_selected_group(
                child,
                item,
                selected,
                valid,
                expected_rows,
            )
        else:
            raise AlignError(f"Quest 组含不支持的 HDF5 对象：{item.name}")


def write_quest_alignment(
    out: Any,
    recording: Path,
    result: dict[str, Any],
) -> None:
    payload = result["quest"]
    if not isinstance(payload, dict):
        return
    group = out.create_group("quest")
    # index = 选中的源行号；age_ms = 该行时刻比目标早多少毫秒，也就是这一行的
    # 滞后。下游要过滤「太旧」的行就看 age_ms，作用与 glove/span_ms 对应。
    group.create_dataset("index", data=payload["index"], dtype=np.int64)
    group.create_dataset("valid", data=payload["valid"], dtype=bool)
    group.create_dataset("age_ms", data=payload["age_ms"], dtype=np.float64)
    group.create_dataset(
        "timestamp_sdk_ns",
        data=payload["timestamp_sdk_ns"],
        dtype=np.int64,
    )
    group.create_dataset(
        "timestamp_host_unix_ns",
        data=payload["timestamp_host_unix_ns"],
        dtype=np.int64,
    )
    group.attrs["alignment"] = str(payload["mode"])
    group.attrs["selection"] = (
        "last valid source row at or before the target; rows are copied whole, "
        "age_ms records the resulting lag"
    )
    group.attrs["max_age_ms"] = float(payload["max_age_ms"])
    if payload.get("clock_report_sha256"):
        group.attrs["clock_report_sha256"] = str(payload["clock_report_sha256"])
    if payload.get("clock_model"):
        group.attrs["clock_model"] = json.dumps(
            payload["clock_model"],
            ensure_ascii=False,
            allow_nan=False,
        )

    with h5py.File(recording / "data.h5", "r") as source:
        if "quest" not in source:
            raise AlignError("写出时发现源文件缺少 quest/ 组")
        data_group = group.create_group("data")
        _write_selected_group(
            data_group,
            source["quest"],
            np.asarray(payload["index"], dtype=np.int64),
            np.asarray(payload["valid"], dtype=bool),
            int(payload["source_rows"]),
        )


def write_aligned(result: dict[str, Any], output_dir: Path) -> Path:
    if h5py is None:
        raise AlignError("缺少 h5py")
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"aligned_{result['reference']}.h5"

    with h5py.File(path, "w") as out:
        out.create_dataset("timestamp_ns", data=result["timestamp_ns"], dtype=np.int64)
        out.create_dataset("timestamp_cam_raw_ns", data=result["timestamp_cam_raw_ns"], dtype=np.int64)
        out.create_dataset("frame_index", data=result["views"][result["reference"]]["frame_index"], dtype=np.int32)

        glove = out.create_group("glove")
        glove.create_dataset("left_index", data=result["glove_left"], dtype=np.int32)
        if result.get("clock_offset_ns") is not None:
            # 墙钟 − 单调钟，逐行。正常是常数（几个 µs 的抖动），台阶 = 校时。
            glove.create_dataset(
                "clock_offset_ns", data=result["clock_offset_ns"], dtype=np.int64
            )
        glove.create_dataset("right_index", data=result["glove_right"], dtype=np.int32)
        glove.create_dataset("span_ms", data=result["glove_span_ms"], dtype=np.float64)
        glove.create_dataset("offset_ms", data=result["glove_offset_ms"], dtype=np.float64)
        weights = np.asarray(result["weights"], dtype=np.float64)
        # True only when the camera instant actually fell between two glove frames;
        # a row that lands on a glove frame needs no interpolation.
        glove.create_dataset("interpolated", data=(weights > 1e-9) & (weights < 1.0 - 1e-9))

        state = out.create_group("state")
        for name, values in result["state"].items():
            if name == DISTRIBUTION_STATE:
                state.create_dataset(name, data=values, compression="gzip", compression_opts=1)
            else:
                state.create_dataset(name, data=values)

        cams = out.create_group("cameras")
        for slot, payload in result["views"].items():
            group = cams.create_group(slot)
            group.create_dataset("frame_index", data=payload["frame_index"], dtype=np.int32)
            group.create_dataset("dt_ms", data=payload["dt_ms"], dtype=np.float32)
            group.create_dataset("timestamp_ns", data=payload["timestamp_ns"], dtype=np.int64)

        out.attrs["format_version"] = ALIGNED_FORMAT_VERSION
        out.attrs["summary"] = json.dumps(aligned_summary(result), ensure_ascii=False)
        write_quest_alignment(out, Path(result["recording"]), result)
    return path


def _clock_offset_summary(offset_ns: np.ndarray | None) -> dict[str, Any] | None:
    """Spread of (wall − monotonic) across the aligned rows.

    Zero spread means the wall clock was never stepped during the capture; a
    nonzero spread is the size of the steps, in milliseconds. ``None`` when the
    file has no monotonic column (every V4L2 camera, but the glove does).
    """
    if offset_ns is None or offset_ns.size == 0:
        return None
    values = offset_ns.astype(np.int64)
    steps = np.abs(np.diff(values)) if values.size > 1 else np.zeros(0, dtype=np.int64)
    return {
        # 绝对值本身没有意义（Unix 纪元 vs 开机计数），只看它是否稳定。
        "spread_ms": round(float(values.max() - values.min()) / 1e6, 3),
        "max_step_ms": round(float(steps.max()) / 1e6, 3) if steps.size else 0.0,
        "steps_over_5ms": int(np.count_nonzero(steps > 5_000_000)),
    }


def aligned_summary(result: dict[str, Any]) -> dict[str, Any]:
    span = result["glove_span_ms"]
    weights = np.asarray(result["weights"], dtype=np.float64)
    interpolated = int(np.count_nonzero((weights > 1e-9) & (weights < 1.0 - 1e-9)))
    quest = result.get("quest")
    if isinstance(quest, dict):
        quest_summary = {
            "enabled": True,
            "alignment": quest["mode"],
            "target_clock": quest["target_clock"],
            "source_rows": quest["source_rows"],
            "paired_rows": quest["paired_rows"],
            "invalid_rows": int(result["aligned_rows"] - quest["paired_rows"]),
            # paired_fraction = 多少相机行找到了样本（真正的「覆盖率」）
            "paired_fraction": round(float(quest["paired_fraction"]), 6),
            "minimum_paired_fraction": result["quest_min_valid_fraction"],
            "max_age_ms": quest["max_age_ms"],
            "age_ms": {
                key: (
                    None
                    if value is None
                    else round(float(value), 3)
                )
                for key, value in quest["age_stats_ms"].items()
            },
            "source_timestamp_quality": quest["quality"],
            "quality_warnings": list(quest["quality_warnings"]),
            "clock_domain_verified": quest["mode"]
            == QUEST_ALIGNMENT_SDK_UNIX,
            "clock_report_sha256": quest["clock_report_sha256"],
        }
    elif result.get("quest_alignment") in QUEST_ALIGNMENT_ACTIVE:
        quest_summary = {
            "enabled": True,
            "alignment": result["quest_alignment"],
            "available": False,
            "skipped_reason": result.get("quest_skipped_reason"),
        }
    else:
        quest_summary = {"enabled": False, "alignment": QUEST_ALIGNMENT_NONE}
    return {
        "format_version": ALIGNED_FORMAT_VERSION,
        "source": str(Path(result["recording"]).name),
        "reference_camera": result["reference"],
        "timebase": result["timebase"],
        "camera_delay_ms": result["delay_ms"],
        "camera_delay_source": result.get("delay_source"),
        "rows": result["aligned_rows"],
        "kept_rows": result["aligned_rows"],
        "source_rows": {
            "glove": result["glove_rows"],
            # Raw reference-camera frames (before the duplicate collapse).
            "camera": result.get("camera_source_rows", result["camera_rows"]),
        },
        "camera_grid_rows": result["camera_rows"],
        "camera_duplicates": result.get("camera_duplicates", {}),
        "dropped_duplicate_frames": int(
            sum((result.get("camera_duplicates") or {}).values())
        ),
        "dropped_outside_overlap": result["dropped_outside"],
        "dropped_by_glove": result.get("dropped_by_glove", 0),
        "dropped_by_camera": result.get("dropped_by_camera", {}),
        "dropped_by_quest": result.get("dropped_by_quest", 0),
        "valid_windows": result.get("windows", {}),
        "interpolated_rows": interpolated,
        "glove_span_ms": {
            "median": round(float(np.median(span)), 3),
            "p95": round(float(np.percentile(span, 95)), 3),
            "max": round(float(np.max(span)), 3),
            "rows_over_25ms": int(np.count_nonzero(span > 25.0)),
            "rows_over_50ms": int(np.count_nonzero(span > 50.0)),
        },
        "camera_holes": result["holes"],
        "clock_jumps": result["clock_jumps"],
        "clock_offset_ms": _clock_offset_summary(result.get("clock_offset_ns")),
        "distribution_force": result["distribution"],
        "quest": quest_summary,
    }


def report(result: dict[str, Any], path: Path | None, delay_source: str) -> None:
    summary = aligned_summary(result)
    span_stats = summary["glove_span_ms"]
    log("")
    log(f"采集目录：{result['recording'].name}")
    log(f"  时间基：{result['timebase']}"
        + ("（单调钟，免疫 NTP）" if result["timebase"] == "mono"
           else "（墙钟；相机无单调钟列，这是当前唯一可用的一路时钟）"))
    log(f"  延迟补偿：{result['delay_ms']:.1f} ms（从相机戳中减去），来源 {delay_source}")
    source_camera = result.get("camera_source_rows", result["camera_rows"])
    duplicate_frames = sum((result.get("camera_duplicates") or {}).values())
    dedup_note = (
        f"（去重后 {result['camera_rows']}）" if duplicate_frames else ""
    )
    log(
        f"  手套 {result['glove_rows']} 帧 → 相机 {source_camera} 帧"
        f"{dedup_note} → 对齐 {result['aligned_rows']} 行"
    )
    if duplicate_frames:
        log(
            f"  相机重复时间戳帧丢弃 {duplicate_frames} 个："
            f"{result.get('camera_duplicates')}"
        )
    if result["dropped_outside"]:
        parts = [f"手套 {result.get('dropped_by_glove', 0)}"]
        parts += [
            f"{slot} {count}"
            for slot, count in (result.get("dropped_by_camera") or {}).items()
        ]
        if result.get("dropped_by_quest"):
            parts.append(f"Quest {result['dropped_by_quest']}")
        log(
            f"  丢弃公共窗口外 {result['dropped_outside']} 帧"
            f"（按来源：{'，'.join(parts)}）"
        )
    log("  插值跨度 median {:.2f} ms / p95 {:.2f} ms / max {:.2f} ms".format(
        span_stats["median"], span_stats["p95"], span_stats["max"]))
    ratio = summary["interpolated_rows"] / max(result["aligned_rows"], 1)
    log(f"  真插值 {summary['interpolated_rows']} 行（{ratio * 100:.1f}%），其余正好落在手套帧上")
    log(f"  相机空洞 {result['holes']} 处（>{HOLE_FACTOR}× 中位间隔）")
    if result["timebase"] == "unix" and result["clock_jumps"]:
        warn(
            f"手套墙钟出现 {result['clock_jumps']} 次跳变，跨跳变处插值不可信："
            "这份采集的时间轴在跳变点前后不可比，建议整段重采"
        )
    log("  分布力：已插值" if result["distribution"] == "interpolated"
        else f"  分布力：{result['distribution']}")
    quest = result.get("quest")
    if isinstance(quest, dict):
        age = quest["age_stats_ms"]
        log(
            "  Quest：{} 行 → 配对 {} 行（{:.1f}%），滞后 median/p99/max "
            "{:.2f}/{:.2f}/{:.2f} ms（上限 {:.1f} ms）".format(
                quest["source_rows"],
                quest["paired_rows"],
                quest["paired_fraction"] * 100.0,
                age["median"] or 0.0,
                age["p99"] or 0.0,
                age["max"] or 0.0,
                quest["max_age_ms"],
            )
        )
        if quest["mode"] == QUEST_ALIGNMENT_UNIX:
            log(
                "    （unix 模式：SDK 时间戳直接当主机墙钟用，未做 C8 时钟域验证）"
            )
        elif quest["mode"] == QUEST_ALIGNMENT_HOST:
            log(
                "    （host 模式：用采集时记录的主机接收时刻对齐，不需要 C8 报告）"
            )
        if quest["quality_warnings"]:
            for message in quest["quality_warnings"]:
                warn(f"  Quest 时间戳质量提示：{message}")
        if quest["clock_report_sha256"]:
            log(f"  C8 报告 SHA256：{quest['clock_report_sha256']}")
    elif result.get("quest_alignment") in QUEST_ALIGNMENT_ACTIVE:
        log(
            "  Quest：已请求对齐但没有数据，本次跳过"
            f"（{result.get('quest_skipped_reason') or '未知原因'}）"
        )
    else:
        log("  Quest：未处理（--quest-align none）")
    if path is not None:
        log(f"  输出：{path}")
    span = np.asarray(result["glove_span_ms"], dtype=np.float64)
    wide = int(np.count_nonzero(span > 25.0))
    rate = result["aligned_rows"] / max((result["timestamp_ns"][-1] - result["timestamp_ns"][0]) / 1e9, 1e-9)
    log(f"  有效采样率 {rate:.1f} Hz；跨度 >25 ms 的行 {wide} 行（{wide / max(span.size, 1) * 100:.1f}%）")
    if wide * 20 > span.size:
        warn("超过 5% 的行跨了两帧以上手套数据，插值可信度低：先查手套帧间隔空洞（B8）")
    else:
        log("  质量判据：跨度分布正常（<5% 的行超过 25 ms）")
    very_wide = int(np.count_nonzero(span > 50.0))
    if very_wide:
        warn(f"{very_wide} 行跨度 >50 ms（最多跨 {span.max():.0f} ms），这些行是硬猜出来的，"
             "训练时可考虑剔除；对应位置是手套帧丢失的空洞")


def gates(result: dict[str, Any]) -> list[str]:
    """Hard checks; a non-empty list means the output should not be trusted."""
    problems: list[str] = []
    ts = result["timestamp_ns"]
    if not np.all(np.diff(ts) > 0):
        problems.append("主时间轴非严格递增")
    joints = result["state"].get("joint_angles_deg")
    if joints is not None and np.isnan(np.asarray(joints, dtype=np.float64)).any():
        problems.append("joint_angles_deg 含 NaN")
    counts = result["state"].get("distribute_counts")
    if counts is not None:
        totals = np.asarray(counts).sum(axis=1)
        if np.any(totals != 444):
            bad = int(np.count_nonzero(totals != 444))
            problems.append(f"{bad} 行的 distribute_counts 合计不是 444")
    if result["aligned_rows"] < 100:
        problems.append(f"对齐后只有 {result['aligned_rows']} 行，少于 100")
    span = np.asarray(result["glove_span_ms"], dtype=np.float64)
    wide = int(np.count_nonzero(span > 25.0))
    if wide * 20 > span.size:
        problems.append(f"{wide}/{span.size} 行跨两帧以上手套数据（>5%），插值不可信")
    very_wide = int(np.count_nonzero(span > 50.0))
    if very_wide * 100 > span.size:
        problems.append(f"{very_wide}/{span.size} 行跨度 >50 ms（>1%），这些行是硬猜的")
    quest = result.get("quest")
    if isinstance(quest, dict) and quest.get("mode") == QUEST_ALIGNMENT_SDK_UNIX:
        # 只有走过 C8 的严格路径才把 Quest 覆盖率当作硬判据。unix 模式是
        # 「已知是墙钟、先拿来用」的宽松路径，覆盖率低不该作废整份产物
        # （手套和相机是好的），只由 summary 和日志提示。
        minimum = float(
            result.get("quest_min_valid_fraction", DEFAULT_QUEST_MIN_VALID_FRACTION)
        )
        actual = float(quest["paired_fraction"])
        if actual < minimum:
            problems.append(f"Quest 有效配对 {actual:.1%} 低于 {minimum:.1%}")
    return problems


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="align_capture.py",
        description="把手套(cap)流按相机帧时刻重采样：相机时间戳为主轴，cap 线性插值到该时刻。",
    )
    parser.add_argument("path", nargs="?", default=".", help="采集目录，或 Recordings 根目录（批量；标准位置是 ../Recordings）")
    parser.add_argument("--output", default=None, help="输出目录，默认 <采集目录>/aligned")
    parser.add_argument("--reference", default=None, help="作为采样网格的相机槽位，默认第一个（通常是 cam0）")
    parser.add_argument("--delay-ms", type=float, default=None,
                        help=f"相机延迟补偿（毫秒），默认读台账，读不到用 {DEFAULT_DELAY_MS}")
    parser.add_argument("--no-delay", action="store_true", help="不减延迟（调试用，正式数据别用）")
    parser.add_argument("--omit-distribution", action="store_true", help="不输出 10x119x3 分布力")
    parser.add_argument("--nearest-distribution", action="store_true",
                        help="分布力只取最近帧，不做插值（省一半读取）")
    parser.add_argument("--dry-run", action="store_true", help="只统计，不写文件")
    parser.add_argument("--clear-write-flag", action="store_true",
                        help="采集被强杀导致 data.h5 打不开时，清掉超级块里的写标志（等价 h5clear -s）")
    parser.add_argument(
        "--quest-align",
        choices=QUEST_ALIGNMENT_CHOICES,
        default=QUEST_ALIGNMENT_AUTO,
        help=(
            "默认 auto=有 quest/host_unix_ns 用主机接收时刻（新采集，不需要 "
            "C8 报告），没有则回退 unix（老文件）；host=强制主机接收时刻；"
            "unix=把 SDK 戳当主机墙钟；sdk_unix=按 C8 报告换算；"
            "none=不处理 Quest。文件里没有 Quest 数据时自动跳过，不报错"
        ),
    )
    parser.add_argument(
        "--quest-clock-report",
        type=Path,
        default=None,
        help="quest_clock_probe.py 生成的通过报告；仅 --quest-align sdk_unix 需要",
    )
    parser.add_argument(
        "--quest-max-age-ms",
        type=float,
        default=DEFAULT_QUEST_MAX_AGE_MS,
        help=f"Quest 样本相对相机目标时刻的最大年龄，默认 {DEFAULT_QUEST_MAX_AGE_MS:g} ms",
    )
    parser.add_argument(
        "--quest-min-valid-fraction",
        type=float,
        default=DEFAULT_QUEST_MIN_VALID_FRACTION,
        help=(
            "Quest 有效配对比例门槛，默认 "
            f"{DEFAULT_QUEST_MIN_VALID_FRACTION:g}；仅 sdk_unix 模式生效"
            "（未达标即不写输出文件），host/unix 模式忽略"
        ),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        recordings = find_recordings(Path(args.path).expanduser().resolve())
    except AlignError as exc:
        error(str(exc))
        return 2

    if not np.isfinite(args.quest_max_age_ms) or args.quest_max_age_ms < 0:
        error("--quest-max-age-ms 必须是有限的非负数")
        return 2
    if not 0.0 <= args.quest_min_valid_fraction <= 1.0:
        error("--quest-min-valid-fraction 必须在 0..1")
        return 2
    if (
        args.quest_clock_report is not None
        and args.quest_align != QUEST_ALIGNMENT_SDK_UNIX
    ):
        warn(
            "给了 --quest-clock-report 但 --quest-align 不是 sdk_unix，报告不会被使用"
            "（host/unix 模式不需要 C8 验证）"
        )
    quest_report = None
    quest_report_sha256 = None
    if args.quest_align == QUEST_ALIGNMENT_SDK_UNIX:
        if args.quest_clock_report is None:
            error("--quest-align sdk_unix 必须同时提供 --quest-clock-report")
            return 2
        try:
            quest_report, quest_report_sha256 = load_quest_clock_report(
                args.quest_clock_report
            )
        except AlignError as exc:
            error(str(exc))
            return 2

    delay_ms, delay_source = resolve_delay(args.delay_ms, args.no_delay)
    log(f"[INFO] 相机延迟补偿：{delay_ms:.1f} ms（{delay_source}）")

    failures = 0
    hard_quest_failures = 0
    for recording in recordings:
        try:
            result = align_recording(
                recording=recording,
                reference=args.reference,
                delay_ms=delay_ms,
                omit_distribution=args.omit_distribution,
                interpolate_distribution=not args.nearest_distribution,
                clear_flag=args.clear_write_flag,
                quest_alignment=args.quest_align,
                quest_clock_report=quest_report,
                quest_report_sha256=quest_report_sha256,
                quest_max_age_ms=args.quest_max_age_ms,
                quest_min_valid_fraction=args.quest_min_valid_fraction,
            )
            # Δ 的来源要进产物，否则三个月后没人知道这份 aligned 是拿什么数
            # 修的时间轴（0 = 未标定）。
            result["delay_source"] = delay_source
            # 质量判据排在写文件之前：不合格就不落盘。产物一旦存在就会被
            # 训练侧的 glob 捡走，日志里的 warn 拦不住任何东西。
            problems = gates(result)
            rejected = bool(problems)
            if rejected:
                failures += 1
                for problem in problems:
                    error(f"{recording.name}: {problem}")
                error(
                    f"{recording.name}: 质量判据未通过，不写输出文件"
                    "（要产出先修数据；--dry-run 与这里一样只看统计）"
                )
                if int(result.get("dropped_by_quest") or 0) > 0:
                    # 行数不足常常不是数据坏，而是 Quest 覆盖不到首尾：默认对齐把
                    # 那些帧整行裁掉了。这条提示能省掉一次「以为是手套丢帧」的排查。
                    warn(
                        f"{recording.name}: 其中 {result['dropped_by_quest']} 帧是因为"
                        " Quest 有效区间没覆盖到而被裁掉；若这次不需要 Quest，"
                        "用 --quest-align none 可保留这些行"
                    )
            else:
                log("  自检：主时间轴严格递增、关节角无 NaN、counts 合计 444 —— 通过")
            out_dir = (
                Path(args.output).expanduser().resolve()
                if args.output
                else recording / "aligned"
            )
            path = None
            if not args.dry_run and not rejected:
                path = write_aligned(result, out_dir)
            # 不合格也照常打印统计：否则你看不到是跨度超标还是行数不够。
            report(result, path, delay_source)
        except AlignError as exc:
            error(f"{recording.name}: {exc}")
            failures += 1
            if args.quest_align == QUEST_ALIGNMENT_SDK_UNIX:
                hard_quest_failures += 1
            continue

    if hard_quest_failures:
        error(f"{hard_quest_failures} 个采集目录未通过 Quest C8 硬门控")
        return 1
    if failures:
        error(f"{failures} 个采集目录未通过质量判据")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
