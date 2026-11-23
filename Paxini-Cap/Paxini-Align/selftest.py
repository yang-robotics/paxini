#!/usr/bin/env python3
"""Offline self-test for align_capture.py. No hardware, no real recordings.

Builds synthetic HDF5 files whose signals are known linear functions of time, so
the correct interpolated value can be computed in closed form and compared
exactly. Run with any Python that has numpy + h5py::

    python selftest.py

Prints one line per check and a total; exit code 1 if anything fails.
"""

from __future__ import annotations

import json
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import h5py
except ImportError:
    print("缺少 h5py，无法自检")
    raise SystemExit(2)

import align_capture as ac


CAP_SLOT_POINTS = (49, 20, 49, 20, 49, 20, 49, 20, 119, 49)
#: Synthetic recordings store monotonic = unix - this offset, mirroring a real host.
MONO_OFFSET = 1_000_000_000_000

CHECKS: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    CHECKS.append((name, bool(ok), detail))
    print(f"[{'OK' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))


def make_recording(
    root: Path,
    *,
    glove_rate: float = 120.0,
    cam_rate: float = 30.0,
    glove_seconds: float = 4.0,
    cam_offset_s: float = -0.5,
    cam_seconds: float = 4.5,
    with_mono: bool = True,
    with_cam1: bool = False,
    cam1_offset_s: float | None = None,
    cam1_seconds: float | None = None,
    t0_ns: int = 1_700_000_000_000_000_000,
    libver: str = "earliest",
) -> Path:
    """Synthetic recording: joint angles are a linear ramp in time."""
    rec = root / "data_synthetic_1"
    (rec / "cameras").mkdir(parents=True, exist_ok=True)

    n = int(glove_seconds * glove_rate)
    glove_dt = int(1e9 / glove_rate)
    glove_t = t0_ns + np.arange(n, dtype=np.int64) * glove_dt

    # joint_angles_deg[i] = 10 + 30 * (t - t0) / 1e9  → linear, verifiable exactly
    seconds = (glove_t - t0_ns) / 1e9
    angles = np.stack([10.0 + 30.0 * seconds, 5.0 - 10.0 * seconds,
                       seconds * 2.0, np.full_like(seconds, -3.5)], axis=1)

    m = int(cam_seconds * cam_rate)
    cam_dt = int(1e9 / cam_rate)
    cam_t = t0_ns + int(cam_offset_s * 1e9) + np.arange(m, dtype=np.int64) * cam_dt

    with h5py.File(rec / "data.h5", "w", libver=libver) as f:
        cap = f.create_group("cap")
        cap.create_dataset("timestamp_unix_ns", data=glove_t)
        if with_mono:
            cap.create_dataset("timestamp_monotonic_ns", data=glove_t - MONO_OFFSET)
        cap.create_dataset("joint_angles_deg", data=angles)
        cap.create_dataset("encoder_raw", data=(np.arange(n)[:, None] * 4 + np.arange(4)).astype(np.int32))
        cap.create_dataset("resultant_force", data=np.zeros((n, 10, 3), dtype=np.int16))
        cap.create_dataset("distribute_force",
                           data=np.zeros((n, 10, 119, 3), dtype=np.int16))
        counts = np.tile(np.array(CAP_SLOT_POINTS, dtype=np.uint32), (n, 1))
        cap.create_dataset("distribute_counts", data=counts)

        cams = f.create_group("cameras")
        # cam1 默认比 cam0 晚 15 ms（同一次开机两路 UVC 的真实情形）；需要
        # 起停不同的用例用 cam1_offset_s / cam1_seconds 单独给窗口。
        cam1_t = cam_t + 15_000_000
        if cam1_offset_s is not None or cam1_seconds is not None:
            offset = cam_offset_s if cam1_offset_s is None else cam1_offset_s
            seconds_1 = cam_seconds if cam1_seconds is None else cam1_seconds
            m1 = int(seconds_1 * cam_rate)
            cam1_t = (
                t0_ns
                + int(offset * 1e9)
                + np.arange(m1, dtype=np.int64) * cam_dt
            )
        for slot, series in (("right_cam0", cam_t), ("right_cam1", cam1_t)):
            if slot == "right_cam1" and not with_cam1:
                continue
            group = cams.create_group(slot)
            group.create_dataset("timestamp_unix_ns", data=series)
            if with_mono:
                group.create_dataset("timestamp_monotonic_ns", data=series - MONO_OFFSET)

        f.create_group("meta").attrs["format_version"] = 4
    return rec


def add_quest(
    rec: Path,
    *,
    t0_ns: int = 1_700_000_000_000_000_000,
    rate: float = 120.0,
    start_s: float = 0.0,
    seconds: float = 4.5,
    gap_s: tuple[float, float] | None = None,
) -> None:
    rows = int(seconds * rate)
    dt = int(1e9 / rate)
    sdk = t0_ns + int(start_s * 1e9) + np.arange(rows, dtype=np.int64) * dt
    if gap_s is not None:
        # 中间挖一段空档：模拟 Quest 掉样本。掉样本的位置在窗口内，所以
        # 「保留该行但标 valid=false」这条语义才有东西可验。
        gap_first = t0_ns + int(gap_s[0] * 1e9)
        gap_last = t0_ns + int(gap_s[1] * 1e9)
        sdk = sdk[(sdk < gap_first) | (sdk >= gap_last)]
    rows = int(sdk.size)
    sequence = np.arange(100, 100 + rows, dtype=np.int32)
    pose = np.zeros((rows, 7), dtype=np.float64)
    pose[:, 0] = (sdk - t0_ns) / 1e9
    pose[:, 1] = 1.0
    pose[:, 2] = -0.5
    pose[:, 6] = 1.0
    with h5py.File(rec / "data.h5", "r+") as f:
        quest = f.create_group("quest")
        quest.create_dataset("sdk_timestamp_ns", data=sdk)
        quest.create_dataset("body_timestamp_ns", data=np.full(rows, -1, dtype=np.int64))
        quest.create_dataset("motion_timestamp_ns", data=np.full(rows, -1, dtype=np.int64))
        quest.create_dataset(
            "body_joints_timestamp_ns",
            data=np.full((rows, 24), -1, dtype=np.int64),
        )
        quest.create_dataset("sequence", data=sequence)
        quest.create_dataset("pose_6d", data=pose)
        quest.create_dataset("tracked", data=np.ones(rows, dtype=bool))
        head = quest.create_group("head")
        head.create_dataset("pose_xyzw", data=pose)
        head.create_dataset("tracked", data=np.ones(rows, dtype=bool))
        head.create_dataset("available", data=np.ones(rows, dtype=bool))
        for side in ("left", "right"):
            controller = quest.create_group(f"controller_{side}")
            controller.create_dataset("pose_xyzw", data=pose)
            controller.create_dataset("grip", data=np.linspace(0, 1, rows))
            controller.create_dataset("trigger", data=np.zeros(rows))
            controller.create_dataset("axis", data=np.zeros((rows, 2)))
            hand = quest.create_group(f"hand_{side}")
            joints = np.zeros((rows, 26, 7), dtype=np.float64)
            joints[..., 6] = 1.0
            hand.create_dataset("joints_pose_xyzw", data=joints)
            hand.create_dataset("joint_count", data=np.full(rows, 26, dtype=np.int32))
        buttons = quest.create_group("buttons")
        for name in (
            "A", "B", "X", "Y", "left_menu", "right_menu",
            "left_axis_click", "right_axis_click",
        ):
            buttons.create_dataset(name, data=np.zeros(rows, dtype=bool))


def make_clock_report(
    rec: Path,
    *,
    passed: bool = True,
    t0_ns: int = 1_700_000_000_000_000_000,
) -> tuple[dict, str]:
    payload = {
        "schema_version": 1,
        "kind": "quest_clock_probe",
        "started_at": "2026-11-18T16:00:00+08:00",
        "ended_at": "2026-11-18T16:01:00+08:00",
        "target_duration_s": 60.0,
        "sample_count": 7200,
        "clock_domain": "unix_epoch",
        "passed": passed,
        "failures": [],
        "thresholds": {
            "min_valid_fraction": 0.95,
            "min_duration_fraction": 0.95,
            "max_backwards_steps": 0,
            "max_same_timestamp_run": 10,
            "max_residual_median_ms": 10.0,
            "max_residual_p99_ms": 30.0,
            "max_slope_error_ppm": 100.0,
        },
        "models": {
            "host_unix": {
                "reference_host_ns": t0_ns,
                "reference_sdk_ns": t0_ns,
                "sdk_per_host_slope": 1.0,
                "intercept_s": 0.0,
            }
        },
        "alignment": {
            "recommended": "sdk_unix" if passed else "none",
            "target_clock": "cap/timestamp_unix_ns" if passed else None,
        },
    }
    path = rec / "clock_report.json"
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return ac.load_quest_clock_report(path)


def run(rec: Path, **kwargs) -> dict:
    defaults = dict(reference=None, delay_ms=0.0, omit_distribution=False,
                    interpolate_distribution=True, keep_outside=False)
    defaults.update(kwargs)
    return ac.align_recording(rec, **defaults)


def test_linear_exactness(tmp: Path) -> None:
    rec = make_recording(tmp)
    result = run(rec, delay_ms=192.3)
    ts = result["timestamp_ns"] + MONO_OFFSET  # 回到夹具用的时间基准
    t0 = 1_700_000_000_000_000_000
    expected = 10.0 + 30.0 * (ts - t0) / 1e9
    got = result["state"]["joint_angles_deg"][:, 0]
    err = float(np.abs(expected - got).max())
    check("线性信号精确还原（线性插值应无误差）", err < 1e-9, f"最大误差 {err:.2e}°")


def test_delay_applied(tmp: Path) -> None:
    rec = make_recording(tmp)
    result = run(rec, delay_ms=192.3)
    delta = result["timestamp_cam_raw_ns"] - result["timestamp_ns"]
    check("主时间轴 = 相机戳 − Δ（192.3 ms）",
          bool(np.all(delta == int(round(192.3 * 1e6)))), f"逐行差值 {delta[0]} ns")


def test_outside_dropped(tmp: Path) -> None:
    rec = make_recording(tmp)  # camera starts 0.5 s before glove
    result = run(rec)
    total = int(result["camera_rows"])
    kept = int(result["aligned_rows"])
    dropped = int(result["dropped_outside"])
    check("覆盖范围外的相机帧被丢弃且计数正确",
          dropped > 0 and kept + dropped == total, f"保留 {kept} / 丢弃 {dropped} / 合计 {total}")


def test_keep_outside(tmp: Path) -> None:
    rec = make_recording(tmp)
    result = run(rec, keep_outside=True)
    check("--keep-outside 保留全部相机帧",
          int(result["aligned_rows"]) == int(result["camera_rows"]),
          f"{result['aligned_rows']} 行")


def _window_counts(result: dict) -> int:
    return (
        int(result["dropped_by_glove"])
        + sum(int(value) for value in result["dropped_by_camera"].values())
        + int(result["dropped_by_quest"])
    )


def test_window_second_camera_late(tmp: Path) -> None:
    """第二路相机晚开始：主轴被它自己的窗口裁掉，并按来源单独记账。"""
    rec = make_recording(tmp, with_cam1=True, cam1_offset_s=1.0, cam1_seconds=2.0)
    result = run(rec, reference="right_cam0")
    window = result["windows"]["camera:right_cam1"]
    master = result["timestamp_ns"]
    expect = int(
        np.count_nonzero(
            (master >= window["first_timestamp_ns"])
            & (master <= window["last_timestamp_ns"])
        )
    )
    inside = bool(
        np.all(
            (master >= window["first_timestamp_ns"])
            & (master <= window["last_timestamp_ns"])
        )
    )
    check(
        "第二条相机晚开始时主轴按公共窗口裁剪",
        inside and int(result["aligned_rows"]) == expect
        and int(result["dropped_by_camera"]["right_cam1"]) > 0,
        f"保留 {result['aligned_rows']}（手算 {expect}），"
        f"cam1 丢弃 {result['dropped_by_camera']['right_cam1']}",
    )
    check(
        "丢弃计数按来源相加等于总丢弃数",
        _window_counts(result) == int(result["dropped_outside"]),
        f"{_window_counts(result)} vs {result['dropped_outside']}",
    )


def test_window_second_camera_early_end(tmp: Path) -> None:
    """第二路相机早结束：末尾的行同样按它自己的窗口裁掉。"""
    rec = make_recording(tmp, with_cam1=True, cam1_offset_s=-0.5, cam1_seconds=2.0)
    result = run(rec, reference="right_cam0")
    window = result["windows"]["camera:right_cam1"]
    master = result["timestamp_ns"]
    last_kept = int(master[-1])
    check(
        "第二条相机早结束时末尾行被裁掉",
        last_kept <= window["last_timestamp_ns"]
        and int(result["dropped_by_camera"]["right_cam1"]) > 0,
        f"末行 {last_kept}，cam1 末帧 {window['last_timestamp_ns']}，"
        f"丢弃 {result['dropped_by_camera']['right_cam1']}",
    )


def test_window_camera_boundary_uses_delay(tmp: Path) -> None:
    """相机窗口边界必须减同一个 Δ，否则窗口整体晚 Δ。"""
    rec = make_recording(tmp, with_cam1=True, cam1_offset_s=0.0, cam1_seconds=4.0)
    delay_ms = 120.0
    result = run(rec, reference="right_cam0", delay_ms=delay_ms)
    delay_ns = int(round(delay_ms * 1e6))
    key = (
        "timestamp_monotonic_ns"
        if result["timebase"] == "mono"
        else "timestamp_unix_ns"
    )
    with h5py.File(rec / "data.h5", "r") as f:
        raw_first = int(f["cameras/right_cam1"][key][0])
    window = result["windows"]["camera:right_cam1"]
    check(
        "相机有效窗口 = 首末戳 − Δ",
        window["first_timestamp_ns"] == raw_first - delay_ns,
        f"{window['first_timestamp_ns']} vs {raw_first - delay_ns}",
    )


def test_windows_in_summary(tmp: Path) -> None:
    rec = make_recording(tmp)
    result = run(rec)
    path = ac.write_aligned(result, tmp / "out")
    with h5py.File(path) as f:
        summary = json.loads(f.attrs["summary"])
    ok = (
        "valid_windows" in summary
        and "kept_rows" in summary
        and "dropped_by_camera" in summary
        and "dropped_by_glove" in summary
        and "dropped_by_quest" in summary
        and "dropped_outside_overlap" in summary
        and summary["kept_rows"] == summary["rows"]
        and "glove" in summary["valid_windows"]
    )
    check("summary 记录公共窗口与按来源丢弃计数（旧字段保留）", ok, list(summary)[:6])


def test_windows_no_overlap(tmp: Path) -> None:
    """四路窗口完全不重叠时必须报错，不能产出一个 0 行的文件。"""
    rec = make_recording(tmp, cam_offset_s=-10.0)
    try:
        run(rec)
    except ac.AlignError as exc:
        check("公共窗口为空时明确报错", "公共有效窗口" in str(exc), str(exc)[:48])
    else:
        check("公共窗口为空时明确报错", False, "没有抛错")


def test_nearest_integer(tmp: Path) -> None:
    rec = make_recording(tmp)
    result = run(rec)
    left = result["glove_left"]
    right = result["glove_right"]
    w = result["weights"]
    got = result["state"]["encoder_raw"][:, 0]
    base = np.arange(result["glove_rows"]) * 4  # encoder_raw[i, 0] = i * 4
    expected = np.where(w >= 0.5, base[right], base[left])
    check("整数计数走最近邻（不做平均）", bool(np.array_equal(got, expected)))


def test_counts_nearest(tmp: Path) -> None:
    rec = make_recording(tmp)
    result = run(rec)
    totals = result["state"]["distribute_counts"].sum(axis=1)
    check("distribute_counts 合计恒为 444", bool(np.all(totals == 444)), f"{totals[0]} 点/行")


def test_omit_distribution(tmp: Path) -> None:
    rec = make_recording(tmp)
    result = run(rec, omit_distribution=True)
    check("--omit-distribution 时不产生分布力数组",
          ac.DISTRIBUTION_STATE not in result["state"], result["distribution"])


def test_timebase_selection(tmp: Path) -> None:
    """时间基自动判定，没有 CLI 开关。

    真实采集里相机永远没有 timestamp_monotonic_ns（V4L2 路径只记墙钟），所以
    正常输入必须落到 unix。第一条用合成数据造出「三路都有单调钟」的理想文件，
    只为固定住那个分支的行为，不代表真机会走到。
    """
    rec = make_recording(tmp, with_mono=False)
    with h5py.File(rec / "data.h5") as f:
        tb = ac.choose_timebase(f)
    check("相机无单调钟列 → unix（真机口径）", tb == "unix", tb)

    ideal = make_recording(tmp / "ideal", with_mono=True)
    with h5py.File(ideal / "data.h5") as f:
        tb2 = ac.choose_timebase(f)
    check("三路都有单调钟 → mono（真机不会出现）", tb2 == "mono", tb2)


def test_multi_camera_views(tmp: Path) -> None:
    rec = make_recording(tmp, with_cam1=True)
    result = run(rec)
    views = result["views"]
    ok = set(views) == {"right_cam0", "right_cam1"}
    dt = np.asarray(views["right_cam1"]["dt_ms"], dtype=np.float64) if ok else np.array([999.0])
    # 因果取帧：dt_ms 是「该帧时刻 − 主轴时刻」，所以必须 <= 0（不晚于目标），
    # 且滞后不超过一个帧间隔。
    check("cam1 取每行之前最近的一帧（0 >= dt_ms >= -一帧）",
          ok and float(dt.max()) <= 1e-3 and float(dt.min()) >= -33.4,
          f"dt_ms 范围 {float(dt.min()):.2f} ~ {float(dt.max()):.2f}")


def test_quest_causal_pairing(tmp: Path) -> None:
    rec = make_recording(tmp)
    add_quest(rec, start_s=0.025)
    report, digest = make_clock_report(rec)
    result = run(
        rec,
        quest_alignment=ac.QUEST_ALIGNMENT_SDK_UNIX,
        quest_clock_report=report,
        quest_report_sha256=digest,
    )
    quest = result["quest"]
    with h5py.File(rec / "data.h5") as f:
        sdk = f["quest/sdk_timestamp_ns"][:].astype(np.int64)
        frame_indices = np.asarray(
            result["views"][result["reference"]]["frame_index"],
            dtype=np.int64,
        )
        cam_unix = f[f"cameras/{result['reference']}/timestamp_unix_ns"][
            frame_indices
        ].astype(np.int64)
    targets = cam_unix - int(round(result["delay_ms"] * 1e6))
    mapped = ac.sdk_unix_to_host_unix_ns(
        sdk,
        report["models"]["host_unix"],
    )
    expected = np.searchsorted(mapped, targets, side="right") - 1
    selected = np.asarray(quest["index"], dtype=np.int64)
    causal = bool(np.all(mapped[selected] <= targets))
    check(
        "Quest 取目标时刻之前最近样本，不使用未来样本",
        bool(np.array_equal(selected, expected)) and causal
        and bool(np.all(quest["valid"])),
        f"最大年龄 {quest['age_stats_ms']['max']:.3f} ms",
    )


def test_quest_duplicate_tie_break(tmp: Path) -> None:
    rec = make_recording(tmp)
    add_quest(rec, start_s=0.025)
    report, digest = make_clock_report(rec)
    with h5py.File(rec / "data.h5", "r+") as f:
        sdk = f["quest/sdk_timestamp_ns"][:]
        sdk[5] = sdk[4]
        f["quest/sdk_timestamp_ns"][:] = sdk
    with h5py.File(rec / "data.h5") as f:
        sdk = f["quest/sdk_timestamp_ns"][:].astype(np.int64)
        target = int(
            ac.sdk_unix_to_host_unix_ns(
                sdk[5:6],
                report["models"]["host_unix"],
            )[0]
        ) + 1_000_000
        paired = ac.align_quest_to_targets(
            f,
            np.asarray([target], dtype=np.int64),
            ac.QUEST_ALIGNMENT_SDK_UNIX,
            report,
            digest,
            25.0,
        )
    check(
        "同一时间戳重复时，因果配对取源文件中最后一行",
        int(paired["index"][0]) == 5 and bool(paired["valid"][0]),
        f"选中行 {int(paired['index'][0])}",
    )


def test_quest_unix_mode_takes_previous_row(tmp: Path) -> None:
    """unix 模式：不要 C8 报告，整行取前驱，逐行记录滞后。"""
    # 不带单调钟列，时间基才会选 unix —— 于是 timestamp_ns 本身就是 Unix，
    # 可以直接和合成 Quest 的 sdk 时间戳逐行比。
    rec = make_recording(tmp, with_mono=False)
    add_quest(rec)
    result = run(rec, quest_alignment=ac.QUEST_ALIGNMENT_UNIX)
    quest = result["quest"]
    out = ac.write_aligned(result, tmp / "unix_out")
    with h5py.File(out) as f:
        ts = f["timestamp_ns"][:].astype(np.int64)
        pose = f["quest/data/pose_6d"][:]
        sdk_rows = f["quest/data/sdk_timestamp_ns"][:].astype(np.int64)
        sequence = f["quest/data/sequence"][:]
        age = f["quest/age_ms"][:]
        valid = f["quest/valid"][:]
    # 整行复制：pose.x 必须精确等于「这一行自己那条 sdk」换算出来的值，
    # 只要有一点插值就不可能逐位相等。合成数据里 pose.x = (sdk - t0)/1e9。
    expected_from_row = (sdk_rows - 1_700_000_000_000_000_000) / 1e9
    check(
        "unix 模式免 C8 报告，Quest 整行取前驱",
        quest["mode"] == ac.QUEST_ALIGNMENT_UNIX
        and quest["paired_fraction"] > 0.9
        and float(np.abs(pose[:, 0] - expected_from_row).max()) < 1e-12,
        f"mode={quest['mode']} 有效={quest['paired_fraction']:.3f} "
        f"偏差={float(np.abs(pose[:, 0] - expected_from_row).max()):.2e}",
    )
    # 前驱的代价是滞后，但必须被限制在一个采样间隔内并如实记录。
    check(
        "滞后 age_ms ∈ [0, 一个采样间隔]，且与时间轴一致",
        float(np.nanmax(age[valid])) <= 8.34 + 1e-6
        and float(np.nanmin(age[valid])) >= 0.0
        and int(sequence.size) == int(ts.size),
        f"age 范围 {float(np.nanmin(age[valid])):.2f}~"
        f"{float(np.nanmax(age[valid])):.2f} ms",
    )


def test_quest_unix_mode_ignores_source_coverage(tmp: Path) -> None:
    """unix 模式不看源文件覆盖率：坏行自动落 valid=false，不打扰也不作废。"""
    rec = make_recording(tmp)
    add_quest(rec, start_s=0.025)
    with h5py.File(rec / "data.h5", "r+") as f:
        sdk = f["quest/sdk_timestamp_ns"][:].astype(np.int64)
        # 中间挖掉 0.4 s：那一段的相机行左邻必然超龄（> 25 ms），落 valid=false。
        # 只挖尾部没用——相机行结束得比 Quest 早，尾部缺失不会被任何行看到。
        hole = sdk.size // 3
        sdk[hole: hole + int(0.4 * 120)] = 0
        f["quest/sdk_timestamp_ns"][:] = sdk
    try:
        result = run(rec, quest_alignment=ac.QUEST_ALIGNMENT_UNIX)
    except ac.AlignError as exc:  # pragma: no cover - 回归保护
        check("unix 模式不因源覆盖率报警", False, str(exc)[:60])
        return
    quest = result["quest"]
    check(
        "unix 模式不因源覆盖率报警，坏行只体现为 valid=false",
        not any("比例" in item for item in quest["quality_warnings"])
        and quest["paired_rows"] < result["aligned_rows"],
        f"警告={quest['quality_warnings']} 配对={quest['paired_rows']}/{result['aligned_rows']}",
    )


def test_quest_clock_model_inverse(tmp: Path) -> None:
    del tmp
    t0 = 1_700_000_000_000_000_000
    host = t0 + np.arange(8, dtype=np.int64) * 250_000_000
    model = {
        "reference_host_ns": t0,
        "reference_sdk_ns": t0,
        "sdk_per_host_slope": 1.0001,
        "intercept_s": 0.003,
    }
    relative_s = (host - t0) / 1e9
    sdk = t0 + np.rint(
        (relative_s * model["sdk_per_host_slope"] + model["intercept_s"])
        * 1e9
    ).astype(np.int64)
    recovered = ac.sdk_unix_to_host_unix_ns(sdk, model)
    error_ns = int(np.max(np.abs(recovered - host)))
    check(
        "Quest C8 时钟模型按斜率与截距反向换算",
        error_ns <= 1_000,
        f"最大误差 {error_ns} ns",
    )


def test_quest_missing_semantics(tmp: Path) -> None:
    rec = make_recording(tmp)
    # 窗口内部的 0.5 s 空档：这些行在 Quest 有效区间之内，配不到刚发生的样本，
    # 必须保留下来并标 valid=false，不能整行丢掉。
    add_quest(rec, gap_s=(1.5, 2.0))
    report, digest = make_clock_report(rec)
    result = run(
        rec,
        quest_alignment=ac.QUEST_ALIGNMENT_SDK_UNIX,
        quest_clock_report=report,
        quest_report_sha256=digest,
    )
    path = ac.write_aligned(result, tmp / "out")
    with h5py.File(path) as f:
        first_invalid = int(np.flatnonzero(~f["quest/valid"][:])[0])
        pose = f["quest/data/pose_6d"][first_invalid]
        body_stamp = int(f["quest/data/body_timestamp_ns"][first_invalid])
        tracked = bool(f["quest/data/tracked"][first_invalid])
        top_stamp = int(f["quest/timestamp_sdk_ns"][first_invalid])
        top_index = int(f["quest/index"][first_invalid])
    check(
        "Quest 缺失行保留相机轴，并按 float/bool/int 语义写 NaN/False/-1",
        first_invalid >= 0 and np.all(np.isnan(pose)) and not tracked
        and body_stamp == -1 and top_stamp == -1 and top_index == -1,
        f"首行无效位置 {first_invalid}",
    )


def test_quest_full_group_written(tmp: Path) -> None:
    rec = make_recording(tmp)
    add_quest(rec, start_s=0.025)
    report, digest = make_clock_report(rec)
    result = run(
        rec,
        quest_alignment=ac.QUEST_ALIGNMENT_SDK_UNIX,
        quest_clock_report=report,
        quest_report_sha256=digest,
    )
    path = ac.write_aligned(result, tmp / "out")
    with h5py.File(path) as f:
        rows = int(f["timestamp_ns"].shape[0])
        format_version = int(f.attrs["format_version"])
        summary = json.loads(f.attrs["summary"])
        ok = (
            format_version == 2
            and "quest/data/pose_6d" in f
            and "quest/data/controller_left/grip" in f
            and "quest/data/hand_right/joints_pose_xyzw" in f
            and "quest/data/buttons/A" in f
            and f["quest/data/pose_6d"].shape == (rows, 7)
            and f["quest/data/hand_right/joints_pose_xyzw"].shape == (rows, 26, 7)
            and summary["quest"]["paired_rows"] == rows
            and summary["quest"]["clock_report_sha256"] == digest
        )
    check("完整 Quest 组写入 v2 对齐文件并记录 C8 报告摘要", ok, f"{rows} 行")


def test_quest_file_quality_gate(tmp: Path) -> None:
    rec = make_recording(tmp)
    add_quest(rec, start_s=0.025)
    report, digest = make_clock_report(rec)
    with h5py.File(rec / "data.h5", "r+") as f:
        raw = f["quest/sdk_timestamp_ns"][:]
        raw[-max(10, raw.size // 5):] = 0
        f["quest/sdk_timestamp_ns"][:] = raw
    try:
        run(
            rec,
            quest_alignment=ac.QUEST_ALIGNMENT_SDK_UNIX,
            quest_clock_report=report,
            quest_report_sha256=digest,
        )
    except ac.AlignError as exc:
        check(
            "当前文件时间戳质量不达 C8 门槛时硬失败",
            "不满足 Quest 对齐条件" in str(exc),
            str(exc)[:60],
        )
    else:
        check("当前文件时间戳质量不达 C8 门槛时硬失败", False, "没有抛错")


def test_quest_coverage_gate(tmp: Path) -> None:
    rec = make_recording(tmp)
    # 10 Hz 的 Quest：覆盖整段窗口，但样本太稀，大部分行配不到 25 ms 内的样本。
    # 这是窗口过滤之后仍然有意义的「覆盖不足」——不是靠起止时间凑出来的。
    add_quest(rec, rate=10.0, seconds=4.5)
    report, digest = make_clock_report(rec)
    result = run(
        rec,
        quest_alignment=ac.QUEST_ALIGNMENT_SDK_UNIX,
        quest_clock_report=report,
        quest_report_sha256=digest,
    )
    problems = ac.gates(result)
    fraction = float(result["quest"]["paired_fraction"])
    check(
        "Quest 有效配对不足 95% 时进入质量判据",
        any("Quest 有效配对" in item for item in problems),
        f"有效 {fraction:.1%}",
    )


def test_quest_requires_report(tmp: Path) -> None:
    rec = make_recording(tmp)
    add_quest(rec, start_s=0.025)
    try:
        run(rec, quest_alignment=ac.QUEST_ALIGNMENT_SDK_UNIX)
    except ac.AlignError as exc:
        check("Quest 对齐缺少通过报告时拒绝执行", "C8" in str(exc), str(exc))
    else:
        check("Quest 对齐缺少通过报告时拒绝执行", False, "没有抛错")


def test_quest_cli_roundtrip(tmp: Path) -> None:
    rec = make_recording(tmp)
    add_quest(rec, start_s=0.025)
    make_clock_report(rec)
    out = tmp / "cli_out"
    code = ac.main(
        [
            str(rec),
            "--output",
            str(out),
            "--delay-ms",
            "0",
            "--quest-align",
            "sdk_unix",
            "--quest-clock-report",
            str(rec / "clock_report.json"),
        ]
    )
    path = out / "aligned_right_cam0.h5"
    ok = code == 0 and path.is_file()
    if path.is_file():
        with h5py.File(path) as f:
            ok = ok and int(f.attrs.get("format_version", 0)) == 2 and "quest/data" in f
    check("Quest CLI 用通过报告生成 v2 对齐文件", ok, f"exit={code}")


def test_quest_hard_gate_exit_code(tmp: Path) -> None:
    rec = make_recording(tmp)
    add_quest(rec, start_s=0.025)
    make_clock_report(rec)
    with h5py.File(rec / "data.h5", "r+") as f:
        raw = f["quest/sdk_timestamp_ns"][:]
        raw[-max(10, raw.size // 5):] = 0
        f["quest/sdk_timestamp_ns"][:] = raw
    out = tmp / "must_not_exist"
    code = ac.main(
        [
            str(rec),
            "--output",
            str(out),
            "--delay-ms",
            "0",
            "--quest-align",
            "sdk_unix",
            "--quest-clock-report",
            str(rec / "clock_report.json"),
        ]
    )
    check(
        "Quest C8 硬门控失败始终非 0 退出且不写文件",
        code != 0 and not out.exists(),
        f"exit={code}",
    )


def test_write_and_reopen(tmp: Path) -> None:
    rec = make_recording(tmp, with_cam1=True)
    result = run(rec)
    path = ac.write_aligned(result, tmp / "out")
    with h5py.File(path) as f:
        keys = set(f.keys())
        summary = json.loads(f.attrs["summary"])
        mono = bool(np.all(np.diff(f["timestamp_ns"][:]) > 0))
    check("输出文件可重开、结构完整、summary 是合法 JSON",
          {"timestamp_ns", "glove", "state", "cameras"} <= keys and mono
          and summary["rows"] == result["aligned_rows"], f"{path.name} {summary['rows']} 行")


def test_gates_pass(tmp: Path) -> None:
    rec = make_recording(tmp)
    result = run(rec)
    problems = ac.gates(result)
    check("质量判据对合成数据全过", not problems, "；".join(problems) or "无问题")


def test_clock_offset_column(tmp: Path) -> None:
    """每行都要带手套的墙钟−单调钟偏移，跳变要能在这一列上看出来。"""
    rec = make_recording(tmp)
    result = run(rec)
    offset = result.get("clock_offset_ns")
    check(
        "时钟偏移随行输出（常数偏移）",
        offset is not None and int(np.unique(offset).size) == 1,
        "None" if offset is None else f"{int(np.unique(offset).size)} 个不同值",
    )

    # 造一次 NTP 阶跃：墙钟整体前跳 2 s，单调钟不动。三列都有的合成文件会
    # 让时间基选 mono，所以主时间轴不受影响，只有 offset 列能看见这次跳变。
    stepped = make_recording(tmp / "stepped")
    with h5py.File(stepped / "data.h5", "r+") as f:
        unix = f["cap/timestamp_unix_ns"][:]
        unix[100:] += 2_000_000_000
        del f["cap/timestamp_unix_ns"]
        f["cap"].create_dataset("timestamp_unix_ns", data=unix)
    result = run(stepped)
    summary = ac.aligned_summary(result)
    check(
        "跳变被记进 clock_offset_ms（spread ≈ 2000 ms）",
        summary["clock_offset_ms"]["spread_ms"] > 1000.0
        and summary["clock_offset_ms"]["steps_over_5ms"] >= 1,
        str(summary["clock_offset_ms"]),
    )


def test_quality_gate_blocks_write(tmp: Path) -> None:

    """质量判据不过时不写产物、退出码非 0。

    这是删掉 --strict 之后的唯一口径：判据在写文件之前跑，不合格就没有文件，
    不需要任何开关来表达「请认真检查」。
    """
    rec = make_recording(tmp, glove_seconds=0.5, cam_seconds=1.0, cam_offset_s=0.0)
    out = tmp / "blocked"
    code = ac.main([str(rec), "--output", str(out), "--delay-ms", "0"])
    path = out / "aligned_right_cam0.h5"
    check(
        "质量判据不过时不写文件且退出码非 0",
        code == 1 and not path.is_file(),
        f"exit={code} 写成文件={path.is_file()}",
    )


def test_rejects_unsorted(tmp: Path) -> None:
    rec = make_recording(tmp)
    with h5py.File(rec / "data.h5", "r+") as f:
        series = f["cameras/right_cam0/timestamp_monotonic_ns"][:]
        series[10], series[11] = series[11], series[10]
        del f["cameras/right_cam0/timestamp_monotonic_ns"]
        f["cameras/right_cam0"].create_dataset("timestamp_monotonic_ns", data=series)
    try:
        run(rec)
    except ac.AlignError as exc:
        check("相机时间戳乱序时报错而不是硬算", "严格递增" in str(exc), str(exc)[:40])
    else:
        check("相机时间戳乱序时报错而不是硬算", False, "没有抛错")


def test_write_flag_repair(tmp: Path) -> None:
    """Simulate a crashed-write flag, then repair it the way --clear-write-flag does.

    Setting the flag without recomputing the superblock checksum is exactly the bug
    that once made eight real recordings look corrupt; this test locks both halves.
    h5py writes v0 superblocks by default and the collector's files are v3, so both
    layouts are handled here.
    """
    rec = make_recording(tmp, libver="latest")  # 真实采集文件是 v3 超级块
    path = rec / "data.h5"
    with open(path, "r+b") as handle:
        head = bytearray(handle.read(48))
        version = head[8]
        if version in (0, 1):
            head[20:24] = b"\x01\x00\x00\x00"
            head = head[:24]
        else:
            head[11] = 1
            head[44:48] = ac.lookup3(bytes(head[:44])).to_bytes(4, "little")
        handle.seek(0)
        handle.write(bytes(head))
    refused = False
    try:
        h5py.File(path, "r").close()
    except OSError as exc:
        refused = "already open for write" in str(exc)
    patched = ac.clear_write_flag(path)
    reopened = False
    try:
        with h5py.File(path, "r") as f:
            reopened = f["cap/joint_angles_deg"].shape[0] > 0
    except Exception:
        reopened = False
    check("写标志修复：置位后 h5py 拒绝、清位并重算校验和后数据完好",
          refused and patched and reopened,
          f"v{version} refused={refused} patched={patched} reopened={reopened}")


def test_lookup3_vector(tmp: Path) -> None:
    """Pin the checksum implementation: 44 zero bytes must hash to a fixed value.

    The same function reproduces the stored checksum of a real recording's
    superblock, so this vector pins the behaviour if it is ever refactored.
    """
    value = ac.lookup3(b"\x00" * 44)
    check("lookup3 实现与 HDF5 一致（固定向量）", value == 0xB0B6DC57, hex(value))


def main() -> int:
    root = Path(tempfile.mkdtemp(prefix="align_selftest_"))
    try:
        tests = [
            test_linear_exactness, test_delay_applied, test_outside_dropped, test_keep_outside,
            test_window_second_camera_late, test_window_second_camera_early_end,
            test_window_camera_boundary_uses_delay, test_windows_in_summary,
            test_windows_no_overlap,
            test_nearest_integer, test_counts_nearest, test_omit_distribution,
            test_timebase_selection, test_multi_camera_views, test_write_and_reopen,
            test_gates_pass, test_clock_offset_column, test_quality_gate_blocks_write, test_rejects_unsorted, test_write_flag_repair, test_lookup3_vector,
            test_quest_causal_pairing, test_quest_duplicate_tie_break,
            test_quest_unix_mode_takes_previous_row,
            test_quest_unix_mode_ignores_source_coverage,
            test_quest_clock_model_inverse,
            test_quest_missing_semantics, test_quest_full_group_written,
            test_quest_file_quality_gate, test_quest_coverage_gate,
            test_quest_requires_report, test_quest_cli_roundtrip,
            test_quest_hard_gate_exit_code,
        ]
        for index, test in enumerate(tests):
            work = root / f"case{index:02d}"
            work.mkdir(parents=True, exist_ok=True)
            try:
                test(work)
            except Exception as exc:  # a broken test must be reported, not fatal
                check(test.__name__, False, f"{exc.__class__.__name__}: {exc}")
    finally:
        shutil.rmtree(root, ignore_errors=True)

    failed = [name for name, ok, _ in CHECKS if not ok]
    print("")
    print(f"{len(CHECKS) - len(failed)}/{len(CHECKS)} 项通过")
    if failed:
        print("失败：" + "，".join(failed))
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
