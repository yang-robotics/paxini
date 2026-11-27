# `data.h5` 结构说明

**这个文件只存原始数据，不做任何对齐。** 

| 组               | 行轴  | 行数             |
| --------------- | --- | -------------- |
| `cap/`          | `N` | 一帧手套数据         |
| `quest/`        | `Q` | 一次 SDK 轮询拿到的样本 |
| `cameras/<id>/` | `M` | 一个视频帧          |

`N`、`Q`、`M` 三者**没有对应关系**

---

## 1. 完整目录树

```
data.h5
├── cap/                                  手套帧，行轴 = N
│   ├── timestamp_unix_ns                 (N,)            int64
│   ├── timestamp_monotonic_ns            (N,)            int64
│   ├── encoder_raw                       (N,4)           int32
│   ├── joint_angles_deg                  (N,4)           float64
│   ├── resultant_force                   (N,10,3)        int16
│   ├── distribute_force                  (N,10,119,3)    int16   gzip-1
│   └── distribute_counts                 (N,10)          uint32
├── quest/                                Quest 采样，行轴 = Q
│   ├── sdk_timestamp_ns                  (Q,)            int64
│   ├── body_timestamp_ns                 (Q,)            int64
│   ├── motion_timestamp_ns               (Q,)            int64
│   ├── body_joints_timestamp_ns          (Q,24)          int64
│   ├── sequence                          (Q,)            int32
│   ├── pose_6d                           (Q,7)           float64
│   ├── tracked                           (Q,)            bool
│   ├── head/
│   │   ├── pose_xyzw                     (Q,7)           float64
│   │   ├── tracked                       (Q,)            bool
│   │   └── available                     (Q,)            bool
│   ├── controller_left/
│   │   ├── pose_xyzw                     (Q,7)           float64
│   │   ├── grip                          (Q,)            float64
│   │   ├── trigger                       (Q,)            float64
│   │   └── axis                          (Q,2)           float64
│   ├── controller_right/                 同上 4 个
│   ├── hand_left/
│   │   ├── joints_pose_xyzw              (Q,26,7)        float64
│   │   └── joint_count                   (Q,)            int32
│   ├── hand_right/                       同上 2 个
│   └── buttons/
│       ├── A / B / X / Y                 (Q,)            bool
│       ├── left_menu / right_menu        (Q,)            bool
│       └── left_axis_click / right_axis_click (Q,)       bool
├── cameras/
│   ├── right_cam0/
│   │   └── timestamp_unix_ns             (M,)            int64
│   └── right_cam1/                       同上 1 个（M 各自独立）
└── meta/                                 组，只有属性，无数据集
```

---

## 2. 时间戳：只记各 SDK 自己给的

> 三个来源的时钟域、换算方式、坑，统一整理在 [`时间戳说明.md`](时间戳说明.md)。
> 本节只回答"哪些字段是时间戳、谁打的"。

手套用 pxhandsdk 帧结构里的两个读数，Quest 用 xrobotoolkit SDK 的四个读数，相机用 V4L2/ffmpeg 打的主机墙钟。

![手套帧的时间戳来自 SDK](figures/timestamp-frame.svg)

手套每帧同时带两个时钟面：

|        | CLOCK_REALTIME  | CLOCK_MONOTONIC |
| ------ | --------------- | --------------- |
| 起点     | 1970-01-01 UTC  | 开机 / VM 启动那一刻   |
| 会不会跳   | 会被校时 step，秒级跳变  | 只增不减，永不跳        |
| 绝对意义   | 能对外（视频时间戳、文件时间） | 只有相对意义，跨重启不可比   |
| 本文件里用来 | 行主轴、与视频时间戳同一把尺  | 帧间隔、实际频率        |

图：同一批帧在两个钟上的表现。

![同一批帧在墙钟与单调时钟上的间隔](figures/timestamp-clocks.svg)

**单调钟更可信，但它不是"更准"，是"更连续"。** 它不知道现在几点，只知道过了多久。

### 2.1 怎么自己判断墙钟有没有跳

采集端**不检测也不修正**，判断留给下游。逐行算偏移即可：

```python
import h5py, numpy as np

with h5py.File("data.h5") as f:
    offset = f["cap/timestamp_unix_ns"][:] - f["cap/timestamp_monotonic_ns"][:]

jumps = np.flatnonzero(np.abs(np.diff(offset)) > 5_000_000)   # >5 ms 视为跳变
print("跳变次数:", jumps.size, "位置(行号):", jumps[:5].tolist())
```

正常情况 `offset` 只有 µs 级漂移；出现 ms 级以上突变就是被校时 step 过。
跨那个位置之后的多源对齐结果不可信，**但原始数据仍然是对的**。

---

## 3. `cap/` 组：手套

| 数据集                      | 形状           | 类型      | 单位    | 含义                                         |
| ------------------------ | ------------ | ------- | ----- | ------------------------------------------ |
| `timestamp_unix_ns`      | (N,)         | int64   | ns    | pxhandsdk 帧结构的 `timestamp_unix_ns`；**行主轴** |
| `timestamp_monotonic_ns` | (N,)         | int64   | ns    | 同一帧的 `timestamp_monotonic_ns`；算帧间隔和频率用它    |
| `encoder_raw`            | (N,4)        | int32   | —     | 4 路磁编码器展开编码值，允许为负                          |
| `joint_angles_deg`       | (N,4)        | float64 | 度     | 4 路关节角                                     |
| `resultant_force`        | (N,10,3)     | int16   | 0.1 N | 10 路合力，每路 `x,y,z`                          |
| `distribute_force`       | (N,10,119,3) | int16   | 0.1 N | 10 路分布力，每路补齐到 119 点，gzip                   |
| `distribute_counts`      | (N,10)       | uint32  | 点     | 每槽位实际有效点数                                  |

磁编码器：

| 序号  | 字段                | 产品命名 | 说明   |
| --- | ----------------- | ---- | ---- |
| 1   | `index_proximal`  | FF2  | 食指近节 |
| 2   | `index_distal`    | FF1  | 食指远节 |
| 3   | `middle_proximal` | MF2  | 中指近节 |
| 4   | `middle_distal`   | MF1  | 中指远节 |

触觉传感器：

| 索引  | 槽位             | 有效点数    |
| --- | -------------- | ------- |
| 0   | `thumb_one`    | 49      |
| 1   | `thumb_two`    | 20      |
| 2   | `index_zero`   | 49      |
| 3   | `index_one`    | 20      |
| 4   | `middle_zero`  | 49      |
| 5   | `middle_one`   | 20      |
| 6   | `middle_two`   | 49      |
| 7   | `middle_three` | 20      |
| 8   | `palm_zero`    | 119     |
| 9   | `palm_one`     | 49      |
|     | **合计**         | **444** |

力的分量：`x`/`y` 有符号（int8 原值 -128..127），`z` 无符号（uint8 原值 0..255），
统一存 int16 以免丢符号。

**`distribute_counts` 等于实际写进数组的点数**（10 个槽位合计 444，上限 119）。
索引 ≥ `distribute_counts[n, i]` 的那些点全是补齐出来的 0，**不是测点**：
读的时候只取前 `count` 个，既不会越界，也不会把 0 当成"没有受力"。

配置参数：`--omit-distribution` 时 `distribute_force` **整个数据集不存在**，`distribute_counts` 仍然写。

---

## 4. `quest/` 组：Quest 3

**一次 SDK 轮询 = 一行。** 没采到就是没有这一行，不会补 NaN 占位。

| 数据集                                  | 形状       | 类型      | 含义                                                              |
| ------------------------------------ | -------- | ------- | --------------------------------------------------------------- |
| `sdk_timestamp_ns`                   | (Q,)     | int64   | `get_time_stamp_ns()`；没给则 -1                                    |
| `body_timestamp_ns`                  | (Q,)     | int64   | `get_body_timestamp_ns()`；没给则 -1                                |
| `motion_timestamp_ns`                | (Q,)     | int64   | `get_motion_timestamp_ns()`；没给则 -1                              |
| `body_joints_timestamp_ns`           | (Q,24)   | int64   | `get_body_joints_timestamp()`，24 个关节各一个；缺失补 -1                  |
| `sequence`                           | (Q,)     | int32   | 轮询序号，连续；跳号说明丢样本                                                 |
| `pose_6d`                            | (Q,7)    | float64 | 选定的主位姿 `x,y,z,qx,qy,qz,qw`                                      |
| `tracked`                            | (Q,)     | bool    | 任一路是否被追踪                                                        |
| `head/pose_xyzw`                     | (Q,7)    | float64 | 头显位姿                                                            |
| `head/tracked` / `head/available`    | (Q,)     | bool    | 是否被追踪 / SDK 是否提供该接口                                             |
| `controller_{left,right}/pose_xyzw`  | (Q,7)    | float64 | 手柄位姿                                                            |
| `controller_{left,right}/grip`       | (Q,)     | float64 | 握合 0..1                                                         |
| `controller_{left,right}/trigger`    | (Q,)     | float64 | 扳机 0..1                                                         |
| `controller_{left,right}/axis`       | (Q,2)    | float64 | 摇杆 -1..1                                                        |
| `hand_{left,right}/joints_pose_xyzw` | (Q,26,7) | float64 | 手部关节，固定 26 位，缺失 NaN                                             |
| `hand_{left,right}/joint_count`      | (Q,)     | int32   | 真实有效关节数，最大 26                                                   |
| `buttons/<name>`                     | (Q,)     | bool    | `A B X Y left_menu right_menu left_axis_click right_axis_click` |

位姿约定：位移米、四元数 `xyzw`、坐标系是 XRoboToolkit 的 Unity tracking-origin
（`+X` 右、`+Y` 上、`+Z` 前，左手系）。时间对齐不等于空间外参已标定——**标定功能目前
不存在**，`tracking-origin` 到相机/手套的变换还没有，见 [`标定.md`](标定.md) §2。

**四列时间戳都是 SDK 给的，我们不再追加主机侧读数。** 代价是：官方 docstring 只有
一句 "Get the timestamp in nanoseconds."，**没写起点和基准**，所以在这四个值的时钟域
被证实之前，不能拿它们直接和手套的单调钟相减。验证方法见
[`待验证清单.md`](待验证清单.md)。

---

## 5. `cameras/` 组：视频

**视频像素不在 HDF5 里。** 

只存每帧时间戳；原始帧在输出目录的 `cameras/<id>.mkv`
（MJPEG 直通，`-c:v copy`）：

```text
Recordings/data_20261111_1/          # 在 Paxini-Cap 根下，不在 Cap_Quest 里
├── data.h5                  ← 本文档描述的就是它
├── manifest.json
└── cameras/
    ├── right_cam0.mkv       ← 像素在这里
    └── right_cam1.mkv
```

| 数据集 | 形状 | 类型 | 含义 |
|---|---|---|---|
| `timestamp_unix_ns` | (M,) | int64 | 该路全部视频帧的绝对墙钟，严格递增。就是 ffmpeg 写进容器的 pts |
| `timestamp_monotonic_ns` | (M,) | int64 | **当前不创建。** 2026-11-17 及更早的文件里可能有这一列（当时相机走另一条采集路径），读的时候用 `"timestamp_monotonic_ns" in group` 判断 |

现在只写第一列。**没有 `frame_index`、没有 `skew_ns`**——配对在采集端已经全部移除。

这一列的来源是主机侧（V4L2 驱动 / ffmpeg），
**不是某个 SDK**：它没法由 SDK 提供，删了视频就没有任何时间轴可用。这是"只记 SDK
时间戳"规则唯一的例外，写在这里以免误解。

### 5.1 为什么相机时间戳只能是墙钟

V4L2 驱动在每次出帧的 `v4l2_buffer.flags` 里声明时钟类型（`TIMESTAMP_MONOTONIC` /
`_MONOTONIC_RAW` / `_BOOTTIME`），用户态不能指定；ffmpeg 又把这个标记丢了，mkv 里
只剩一个整数。所以"用驱动时间戳去和手套的单调钟对齐"只能靠数值量级推断，无法确证
——一旦驱动给的是相对时间，所有帧会整体偏一个未知常量，对齐"看起来成立"其实全错。
这种错比报错危险，因此本工程不用它。

现在只有一种做法：ffmpeg 用 `-use_wallclock_as_timestamps 1` 给每帧打主机墙钟，
并校验首帧 pts 必须是墙钟量级（≤ 1.5e9 直接报错），不做推断、不留选项。

---

## 6. `meta/` 组：属性

属性挂在**这个组上**（`f["meta"].attrs`），**一个文件只有一份**。每个值都是 JSON
字符串（`json.loads` 读回），与 `manifest.json` 同源：

| 属性                             | 内容                                                                            |
| ------------------------------ | ----------------------------------------------------------------------------- |
| `format_version`               | 结构版本，当前 `4`                                                                   |
| `started_at` / `ended_at`      | **按键开始**的本地时区 ISO 时间 / manifest 写出时刻                                   |
| `session`                      | 按键时刻 `started_at` / `started_unix_ns` / `started_monotonic_ns`、`ready_wait_s`、`key_wait_s`、`auto_start`、待机丢弃数 |
| `rows` / `quest_rows`          | `N` / `Q`（都不含待机期）                                                             |
| `actual_rate_hz`               | 按首末帧单调钟跨度算的实际频率                                                               |
| `duration_s` / `sample_span_s` | 从按键算起的墙钟耗时 / 手套首末帧跨度                                                        |
| `alignment`                    | 固定为 `none (raw per-source timestamps only)`                                   |
| `pose`                         | 主位姿来源、分量顺序、坐标系说明                                                              |
| `hand`                         | 单只手套手别 `side`、Quest 请求手别 `pose_side_requested`、串口、SN、SDK/固件版本、故障、回调耗时、队列溢出、`standby_frames_discarded` |
| `quest`                        | `enabled`、`errors`、`timestamp_source`、`timestamp_fields`、`rows`、`valid_rows`（已写入行里的有效行）、`first_sequence` / `last_sequence`、`standby_samples_discarded`、`selected_pose`（判据、待机/运行期变化、`stale_max_s`、`stale_warnings`）、`samples_dropped`、`poller_alive`、轮询次数与 `poller_overruns` |
| `cameras`                      | `backend`（固定 `linux`）、每路设备节点、帧数、分辨率、实际 fps、时间窗、错误、`first_frame_s`（`camera.start()`→首帧）、`pre_roll_s`（首帧→按键） |
| `runtime`                      | `abort_reason`（中止原因，正常结束为 null）、`configured_sources`、检查频率与停滞阈值（含 `quest_valid_*`、`quest_pose_*`）、`camera_stalls`、`glove_stalls` |
| `verify`                       | 行数自检结果（`row_mismatch` 必须为空）                                                   |
| `env`                          | python / numpy / h5py / ffmpeg 版本                                             |

---

## 7. 三条硬约定

1. **缺失语义统一**：float → `NaN`，bool → `False`，计数 → `0`，SDK 没给的时间戳 → `-1`。
2. **`Q` 不等于 `N`**，别拿手套行号去索引 Quest 数据集。
3. **分布力按 `distribute_counts` 读**，不要遍历 119。

---

## 8. 读法示例

```python
import h5py

with h5py.File("data.h5") as f:
    cap_ts = f["cap/timestamp_unix_ns"][:]        # (N,) 墙钟
    cap_mono = f["cap/timestamp_monotonic_ns"][:] # (N,) 单调钟
    counts = f["cap/distribute_counts"][:]        # (N,10)
    force = f["cap/distribute_force"]             # 惰性，7 KB/行

    row = 0
    points = [force[row, i, :counts[row, i]] * 0.1 for i in range(10)]  # 转 N

    quest_ts = f["quest/sdk_timestamp_ns"][:]     # (Q,) SDK 给的，域未知
    body_joints = f["quest/body_joints_timestamp_ns"][:]  # (Q,24)
    cam_ts = f["cameras/right_cam0/timestamp_unix_ns"][:]  # (M,) 墙钟
```

下游要做对齐的话：手套 ↔ 视频可以直接用墙钟（`cap/timestamp_unix_ns` 对
`cameras/*/timestamp_unix_ns`）。手套 ↔ Quest 必须先确认 `sdk_timestamp_ns` 的时钟域，
否则只能比增量、不能比绝对值。

### 8.1 一行长什么样

下面这组数字是**真实生成后 dump 出来的**（合成样本，时间线按"Quest 先采 → 手套后开"
排列）：

```text
cap                     row 0（手套第 1 帧）
  timestamp_unix_ns          1700000000100000000
  timestamp_monotonic_ns           6300100000000     ← 开机 6300 秒（≈1.75 小时）处
  encoder_raw              [101, 102, 103, 104]
  joint_angles_deg         [12.5, 3.25, -1.5, 0.0]   ← 度
  resultant_force[0]       [0, 0, 0]                 ← 第 0 路合力，0.1 N
  distribute_counts        [49,20,49,20,49,20,49,20,119,49]   ← 合计 444

quest                   row 0（第一次轮询）
  sdk_timestamp_ns           1700000000000000000
  body_timestamp_ns          1700000000000000010
  motion_timestamp_ns        1700000000000000020
  body_joints_timestamp_ns   [1700000000000000100, … 共 24 项]
  sequence                   1
  pose_6d                  [0.3, 1.2, 0.5, 0.0, 0.0, 0.0, 1.0]

cameras/right_cam0      row 0
  timestamp_unix_ns          1700000000100000000     ← 示例里与手套同起点
```

三条要记住的量级：

- 所有时间戳都是**纳秒**。墙钟 ≈ `1.7e18`，单调钟 ≈ `几e12`（开机秒数 × 1e9）。
- `unix − monotonic` 是常数偏移（示例里 `1699993700000000000`）；这个差值突变就是校时。
- 力是 **0.1 N**，角度是**度**，`distribute_counts` 合计应该是 444。

---

## 9. 写入策略与体积

- 两条行轴各自缓冲，每 100 行 flush 一次；关闭时缩到精确行数。
- 相机时间戳在停止后一次性写：先 `probe()` 拿 pts，再校验严格递增。
- 每行原始约 **7.3 KB**，其中分布力 7.14 KB。120 Hz 原始约 3.2 GB/小时；gzip 后
  取决于数据熵，合成数据实测 3.6 KB/行 ≈ 1.57 GB/小时。
- 短录音有约 900 KB 的 chunk 预留，不是数据。

---

## 10. 自检

`manifest.json` 的 `verify.row_mismatch` 必须为空。它由 `Hdf5Recorder.verify_rows()`
产生：`cap/*` 的首维必须等于 `N`，`quest/*` 的首维必须等于 `Q`，逐条列出不符的
数据集和期望值。`cameras/*` 是 `(M,)`，不参与这项检查。

---

## 11. 版本变更

| 版本 | 变化 |
|---|---|
| **4**（当前） | `cameras/<id>/` 曾新增可选的 `timestamp_monotonic_ns`（与 `cap/timestamp_monotonic_ns` 同域的单调钟列）；现在的采集端不再写它，但 2026-11-17 及更早的文件里可能存在。`meta.cameras.per_camera.<id>` 同步多了 `monotonic_available` |
| 3 | 时间戳只保留 SDK 给的：`cap/` 删掉 `received_monotonic_ns`；`quest/` 的三列主机时间戳（`host_poll_start_ns`/`host_poll_end_ns`/`host_timestamp_ns`）换成四列 SDK 时间戳（`sdk_`/`body_`/`motion_`/`body_joints_timestamp_ns`） |
| 2 | 删除采集端全部对齐：`cameras/*/frame_index`、`cameras/*/skew_ns`、`quest/available`、`quest/skew_ns`；`quest/` 改成自己的行轴 `Q` |
| 1（废弃） | 采集时就做最近邻配对，`quest` 与 `cap` 同为 N 行 |

读旧文件的老脚本在新文件上会直接 `KeyError`，不会有静默错误。

对齐逻辑本身没有删除，只是搬到下游处理工程（现有 `Paxini-Align/`）。那边需要的
输入这里都有：`cap` 的两个 SDK 时间戳、`quest` 的四个 SDK 时间戳、`cameras` 的墙钟。

下游**默认就对 Quest 做对齐**（`--quest-align unix`，把 SDK 戳当主机墙钟用，不要求
C8 报告）；只有 `--quest-align sdk_unix` 这条路才需要 C8 报告与文件级时间戳质量双重
门控。没有 Quest 数据时自动跳过。
