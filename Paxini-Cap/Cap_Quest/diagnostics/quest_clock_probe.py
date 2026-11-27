#!/usr/bin/env python3
"""Probe the XRoboToolkit timestamp clock domain without touching data.h5.

The probe records a test-only host receive interval around each SDK sample. It
writes JSONL samples and a machine-readable report under ``clock_probe/``.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
QUEST_ROOT = SCRIPT_DIR.parent
CAPTURE_DIR = QUEST_ROOT / "capture"
PROJECT_ROOT = QUEST_ROOT.parent
if str(CAPTURE_DIR) not in sys.path:
    sys.path.insert(0, str(CAPTURE_DIR))

from quest3 import (  # noqa: E402
    DEFAULT_PC_SERVICE_SCRIPT,
    STOP_EVENT,
    QuestSetupError,
    XrtReader,
    import_sdk,
    maybe_start_pc_service,
    prepare_quest,
)


DEFAULT_APK = PROJECT_ROOT / "sdk" / "XRoboToolkit-Quest-1.0.1.apk"
DEFAULT_PACKAGE = "com.xrobotoolkit.client.quest"
DEFAULT_OUTPUT_ROOT = QUEST_ROOT / "clock_probe"
THRESHOLDS = {
    "min_valid_fraction": 0.95,
    "min_duration_fraction": 0.95,
    "max_backwards_steps": 0,
    # A pull loop polls faster than the SDK updates, so the same timestamp is
    # expected to repeat for a few rows. What matters is wall time: a frozen
    # clock holds one value for seconds, not for one poll interval. The count
    # is kept in the report for older readers/humans; the gate is the duration
    # key below.
    "max_same_timestamp_run": 10,
    "max_same_timestamp_run_s": 1.0,
    "max_residual_median_ms": 10.0,
    "max_residual_p99_ms": 30.0,
    "max_slope_error_ppm": 100.0,
}


def log(message: str) -> None:
    print(message, flush=True)


def error(message: str) -> None:
    print(f"[错误] {message}", file=sys.stderr, flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Probe Quest SDK timestamp clock domain without writing data.h5"
    )
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--rate", type=float, default=120.0)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--apk", type=Path, default=DEFAULT_APK)
    parser.add_argument("--package", default=DEFAULT_PACKAGE)
    parser.add_argument("--adb")
    parser.add_argument("--serial")
    parser.add_argument("--quest-ip")
    parser.add_argument("--adb-port", type=int, default=5555)
    parser.add_argument("--skip-adb", action="store_true")
    parser.add_argument("--allow-usb-only", action="store_true")
    parser.add_argument("--start-pc-service", action="store_true")
    parser.add_argument("--pc-service-script", default=str(DEFAULT_PC_SERVICE_SCRIPT))
    parser.add_argument("--no-pc-service-check", action="store_true")
    return parser


def _fit_clock_model(
    sdk_ns: np.ndarray,
    host_ns: np.ndarray,
) -> dict[str, Any] | None:
    """Fit ``sdk = slope * host + intercept`` using centered seconds."""
    if sdk_ns.size < 3 or host_ns.size != sdk_ns.size:
        return None
    host0 = int(host_ns[0])
    sdk0 = int(sdk_ns[0])
    x = (host_ns.astype(np.float64) - host0) / 1e9
    y = (sdk_ns.astype(np.float64) - sdk0) / 1e9
    if np.ptp(x) <= 0:
        return None
    slope, intercept = np.polyfit(x, y, 1)
    predicted = sdk0 + (slope * x + intercept) * 1e9
    residual_ms = (sdk_ns.astype(np.float64) - predicted) / 1e6
    return {
        "reference_host_ns": host0,
        "reference_sdk_ns": sdk0,
        "sdk_per_host_slope": float(slope),
        "intercept_s": float(intercept),
        "slope_error_ppm": float(abs(slope - 1.0) * 1e6),
        "offset_median_s": float(np.median((sdk_ns - host_ns) / 1e9)),
        "residual_ms": {
            "median": float(np.median(residual_ms)),
            "p95": float(np.percentile(residual_ms, 95)),
            "p99": float(np.percentile(residual_ms, 99)),
            "max_abs": float(np.max(np.abs(residual_ms))),
        },
    }


def _clock_updates(rows: Sequence[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray]:
    """Return one host midpoint per SDK timestamp update."""
    host: list[int] = []
    sdk: list[int] = []
    last_sdk: int | None = None
    for row in rows:
        value = int(row.get("sdk_timestamp_ns") or -1)
        if value <= 0 or value == last_sdk:
            continue
        host.append(int(row["host_unix_mid_ns"]))
        sdk.append(value)
        last_sdk = value
    return np.asarray(host, dtype=np.int64), np.asarray(sdk, dtype=np.int64)


def evaluate_clock_samples(
    rows: Sequence[dict[str, Any]],
    *,
    target_duration_s: float,
    thresholds: dict[str, float | int] | None = None,
) -> dict[str, Any]:
    """Evaluate a probe session and produce a report payload."""
    limits = dict(THRESHOLDS)
    if thresholds:
        limits.update(thresholds)
    total = len(rows)
    sdk_values = np.asarray(
        [int(row.get("sdk_timestamp_ns") or -1) for row in rows],
        dtype=np.int64,
    )
    valid = sdk_values > 0
    valid_count = int(np.count_nonzero(valid))
    valid_fraction = valid_count / total if total else 0.0
    valid_sdk = sdk_values[valid]
    valid_host = np.asarray(
        [
            int(row["host_unix_mid_ns"])
            for row in rows
            if int(row.get("sdk_timestamp_ns") or -1) > 0
        ],
        dtype=np.int64,
    )
    host_duration_s = (
        (int(valid_host[-1]) - int(valid_host[0])) / 1e9
        if valid_host.size > 1
        else 0.0
    )
    unique_sdk = int(np.unique(valid_sdk).size)
    backwards = (
        int(np.count_nonzero(np.diff(valid_sdk) < 0))
        if valid_sdk.size > 1
        else 0
    )

    max_run = 0
    max_run_s = 0.0
    run = 0
    run_start_host_ns: int | None = None
    previous = -1
    for row, value in zip(rows, sdk_values):
        value = int(value)
        host_ns = int(row["host_unix_mid_ns"])
        if value > 0 and value == previous:
            run += 1
            if run_start_host_ns is not None:
                max_run_s = max(
                    max_run_s, (host_ns - run_start_host_ns) / 1e9
                )
        elif value > 0:
            run = 1
            run_start_host_ns = host_ns
        else:
            run = 0
            run_start_host_ns = None
        max_run = max(max_run, run)
        previous = value

    host_unix, sdk_updates = _clock_updates(rows)
    unix_model = _fit_clock_model(sdk_updates, host_unix)
    # Rebuild monotonic updates in the same order as SDK updates.
    mono_host: list[int] = []
    mono_sdk: list[int] = []
    last_sdk: int | None = None
    for row in rows:
        value = int(row.get("sdk_timestamp_ns") or -1)
        if value <= 0 or value == last_sdk:
            continue
        mono_host.append(int(row["host_monotonic_mid_ns"]))
        mono_sdk.append(value)
        last_sdk = value
    monotonic_model = _fit_clock_model(
        np.asarray(mono_sdk, dtype=np.int64),
        np.asarray(mono_host, dtype=np.int64),
    )

    failures: list[str] = []
    if valid_fraction < float(limits["min_valid_fraction"]):
        failures.append(
            f"有效时间戳比例 {valid_fraction:.3f} 低于 "
            f"{float(limits['min_valid_fraction']):.3f}"
        )
    if host_duration_s < (
        float(target_duration_s) * float(limits["min_duration_fraction"])
    ):
        failures.append(
            f"探针有效时长 {host_duration_s:.3f} s 低于目标时长 "
            f"{float(target_duration_s):.3f} s 的 "
            f"{float(limits['min_duration_fraction']):.2f} 倍"
        )
    if backwards > int(limits["max_backwards_steps"]):
        failures.append(f"时间戳倒退 {backwards} 次")
    max_run_limit_s = float(
        limits.get("max_same_timestamp_run_s", 1.0)
    )
    if max_run_s > max_run_limit_s:
        failures.append(
            f"同一时间戳连续重复 {max_run} 次 / {max_run_s:.3f} 秒，"
            f"超过 {max_run_limit_s:g} 秒（时钟疑似冻结）"
        )
    if unix_model is None:
        failures.append("有效时间戳更新不足，无法拟合 Unix 时钟模型")
    else:
        residual = unix_model["residual_ms"]
        if residual["median"] > float(limits["max_residual_median_ms"]):
            failures.append(
                f"Unix 残差中位数 {residual['median']:.3f} ms 超限"
            )
        if residual["p99"] > float(limits["max_residual_p99_ms"]):
            failures.append(f"Unix 残差 p99 {residual['p99']:.3f} ms 超限")
        if unix_model["slope_error_ppm"] > float(limits["max_slope_error_ppm"]):
            failures.append(
                f"时钟斜率误差 {unix_model['slope_error_ppm']:.3f} ppm 超限"
            )
        if abs(unix_model["offset_median_s"]) > 86_400:
            failures.append("SDK 时间与 Unix epoch 相差超过 24 小时")

    if (
        unix_model is not None
        and abs(unix_model["offset_median_s"]) <= 86_400
    ):
        clock_domain = "unix_epoch"
    elif monotonic_model is not None:
        clock_domain = "host_monotonic_or_relative"
    else:
        clock_domain = "unknown"

    passed = clock_domain == "unix_epoch" and not failures
    return {
        "schema_version": 1,
        "kind": "quest_clock_probe",
        "started_at": rows[0].get("started_at") if rows else None,
        "ended_at": rows[-1].get("ended_at") if rows else None,
        "target_duration_s": float(target_duration_s),
        "sample_count": total,
        "clock_domain": clock_domain,
        "passed": bool(passed),
        "failures": failures,
        "thresholds": limits,
        "metrics": {
            "valid_timestamp_count": valid_count,
            "valid_timestamp_fraction": float(valid_fraction),
            "host_duration_s": float(host_duration_s),
            "unique_sdk_timestamps": unique_sdk,
            "timestamp_backwards_steps": backwards,
            "max_same_timestamp_run": max_run,
            "max_same_timestamp_run_s": float(max_run_s),
        },
        "models": {
            "host_unix": unix_model,
            "host_monotonic": monotonic_model,
        },
        "alignment": {
            "recommended": "sdk_unix" if passed else "none",
            "target_clock": "cap/timestamp_unix_ns" if passed else None,
        },
    }


def _write_report(path: Path, payload: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as file:
        json.dump(payload, file, ensure_ascii=False, indent=2, allow_nan=False)
        file.write("\n")


def run_probe(args: argparse.Namespace) -> int:
    if args.duration <= 0:
        raise ValueError("--duration 必须大于 0")
    if args.rate <= 0:
        raise ValueError("--rate 必须大于 0")
    if not args.apk.is_file() and not args.skip_adb:
        raise ValueError(f"APK 不存在：{args.apk}")

    maybe_start_pc_service(args)
    if not args.skip_adb:
        setup = prepare_quest(args, keep_usb_adb=True)
        log(f"[INFO] Quest ADB 已就绪：{setup.serial}")
    else:
        log("[INFO] 已跳过 ADB；继续连接 PC Service")

    reader = XrtReader(import_sdk())
    started = datetime.now().astimezone()
    run_name = f"run_{started:%Y%m%d_%H%M%S}"
    run_dir = args.output_root.expanduser().resolve() / run_name
    run_dir.mkdir(parents=True, exist_ok=False)
    samples_path = run_dir / "samples.jsonl"
    report_path = run_dir / "report.json"
    rows: list[dict[str, Any]] = []
    period = 1.0 / args.rate
    deadline = time.monotonic() + args.duration
    sequence = 0
    log(f"[INFO] 开始 C8 时钟探针：{args.duration:g} 秒 @ {args.rate:g} Hz")
    log(f"[INFO] 输出：{run_dir}")

    probe_error: Exception | None = None
    try:
        with samples_path.open("w", encoding="utf-8") as file:
            next_deadline = time.monotonic()
            while not STOP_EVENT.is_set() and time.monotonic() < deadline:
                sequence += 1
                host_unix_before = time.time_ns()
                host_mono_before = time.monotonic_ns()
                sample = reader.collect_sample(sequence)
                host_mono_after = time.monotonic_ns()
                host_unix_after = time.time_ns()
                now = datetime.now().astimezone().isoformat(
                    timespec="milliseconds"
                )
                row = {
                    "sequence": sequence,
                    "started_at": started.isoformat(timespec="milliseconds"),
                    "ended_at": now,
                    "host_unix_before_ns": host_unix_before,
                    "host_unix_after_ns": host_unix_after,
                    "host_unix_mid_ns": (host_unix_before + host_unix_after) // 2,
                    "host_monotonic_before_ns": host_mono_before,
                    "host_monotonic_after_ns": host_mono_after,
                    "host_monotonic_mid_ns": (host_mono_before + host_mono_after) // 2,
                    "sdk_timestamp_ns": sample.get("sdk_timestamp_ns"),
                    "body_timestamp_ns": sample.get("body_timestamp_ns"),
                    "motion_timestamp_ns": sample.get("motion_timestamp_ns"),
                    "tracked": sample.get("tracked"),
                    "head": sample.get("head"),
                    "controllers": sample.get("controllers"),
                }
                rows.append(row)
                file.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
                file.flush()

                next_deadline += period
                now_mono = time.monotonic()
                if now_mono - next_deadline > period:
                    next_deadline = now_mono
                STOP_EVENT.wait(max(0.0, next_deadline - time.monotonic()))
    except Exception as exc:
        probe_error = exc
    finally:
        try:
            reader.close()
        except Exception as exc:
            print(f"[警告] 关闭 Quest SDK 失败：{exc}", file=sys.stderr)

    report = evaluate_clock_samples(rows, target_duration_s=args.duration)
    if probe_error is not None:
        report["passed"] = False
        report["failures"].append(
            f"探针运行失败：{probe_error.__class__.__name__}: {probe_error}"
        )
        report["alignment"] = {"recommended": "none", "target_clock": None}
    _write_report(report_path, report)
    log(
        f"[DONE] samples={report['sample_count']} "
        f"clock_domain={report['clock_domain']} passed={report['passed']}"
    )
    log(f"[DONE] report: {report_path}")
    return 0 if report["passed"] else 1


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    signal.signal(signal.SIGINT, lambda _signum, _frame: STOP_EVENT.set())
    signal.signal(signal.SIGTERM, lambda _signum, _frame: STOP_EVENT.set())
    STOP_EVENT.clear()
    try:
        return run_probe(args)
    except (QuestSetupError, RuntimeError, ValueError, OSError) as exc:
        error(str(exc))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
