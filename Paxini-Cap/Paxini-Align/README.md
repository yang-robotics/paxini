# Paxini-Align

把手套（`cap/`）与可选 Quest 数据按相机帧时刻生成训练样本。

采集端**刻意不做对齐**（`Paxini-Cap/README.md` §0.1 第 1 条），三条行轴各写各的；
这个目录就是接住那件事的下游层。它只读 `data.h5`，不改原文件。

```text
data.h5（cap N 帧 @120Hz + cameras M 帧 @30Hz + quest Q 行）
        │
        ├─ 相机时间戳 − 延迟Δ  →  采样网格（K 个时刻）
        ├─ cap 按网格线性插值
        └─ Quest 在夹住目标的两条之间插值（可选，--quest-align）
                                      ↓
                         aligned_<cam>.h5（K 行，一行一个训练样本）
```

## 为什么相机总比手套长一段

> 下表是 2026-11-16 的旧采集（串行启动：相机连续 3 帧出图后才起手套，停止时先停手套）
> 下的实测。2026-11-20 起采集端改成三源各自启停，**这组数字只作历史参考**，
> 新采集的窗口关系不再必然满足「手套 ⊂ 相机」。

| 采集 | 相机早开 | 相机晚停 | 减 Δ 后落在手套外 |
|---|---|---|---|
| `data_20261116_1` | +1.49 s | +0.34 s | 56 帧 |
| `data_20261113_5` | +1.46 s | +0.22 s | 51（cam1 66） |
| `data_20261113_11` | +1.61 s | +0.22 s | 56（cam1 72） |
| `data_20261113_2` | +1.54 s | +0.43 s | 60（cam1 75） |

那些帧没有手套数据可插，只能丢。`manifest` 里 `dropped_outside_overlap` 记的就是它；
采集端自己也会在日志里打一行「首帧与手套首帧相差 +x.x 秒」。

## 公共有效窗口（2026-11-20 起）

三源各自启停之后，单看「相机 vs 手套」已经不够：第二路相机可能比参考相机晚开或早停，
Quest 的有效区间也可能比相机窄。所以主轴按**四路窗口的交集**过滤：

| 来源 | 窗口 |
|---|---|
| 参考相机与其它每路相机 | `[首帧 − Δ, 末帧 − Δ]`（**相机戳是到达时刻，边界必须减同一个 Δ**） |
| 手套 | `[cap/timestamp_unix_ns[0], [−1]]` |
| Quest（`--quest-align` 非 none 时） | 有效行（时间戳 > 0）映射到主机 Unix 后的首末 |

窗口外的整行丢弃，`--keep-outside` 只影响是否真的裁掉，计数照算。`summary` 里按来源分开记账：

```json
"dropped_outside_overlap": 214,
"dropped_by_glove": 60,
"dropped_by_camera": {"right_cam0": 154, "right_cam1": 0},
"dropped_by_quest": 0,
"kept_rows": 681,
"valid_windows": {"glove": {...}, "camera:right_cam0": {...}, "camera:right_cam1": {...}}
```

计数是增量式的（手套 → 各相机 → Quest），一行只算给第一个排除它的来源，所以各项相加
正好等于总丢弃数。四路完全不重叠时直接报错，不会产出 0 行的文件。

## 对齐策略（2026-11-17 定）

| 项 | 规则 |
|---|---|
| 时间基准 | **相机帧时刻**为准，一行输出对一帧图像 |
| 手套时间戳 | 当作**真实时间戳**，是插值的自变量，不信任何第三方时钟 |
| 时间对齐方式 | 相机时刻落在两帧手套之间时**线性插值**；落在覆盖范围外则丢弃 |
| 延迟补偿 | 相机戳是"帧到达主机"的时刻，减去实测 Δ（默认读台账，两路共用同一值）；**台账读不到时 Δ=0 并告警，不再套用历史默认值** |
| Quest | 默认不处理。`--quest-align unix` 直接当主机墙钟用（免报告）；`sdk_unix` 另需 C8 报告 |
| 时钟域 | 自动判定，**没有 CLI 开关**：只有参考相机、其他相机、手套都带 `timestamp_monotonic_ns` 时才用单调钟。当前 V4L2 路径的相机只有墙钟列（`write_camera_timestamps` 的 `monotonic_ns` 恒为 `None`），所以**实际总是墙钟**，靠 `clock_jumps` 查跳变兜底 |

Quest 的 Unix 目标时刻始终来自参考相机的 `timestamp_unix_ns − Δ`。即使主时间基落在单调钟上，
Quest 也只用这台主机的 Unix 轴换算，不会混用 SDK 的四个时间列。

## 快速开始

```bash
cd Paxini-Cap/Paxini-Align

# 单个采集目录
python3 align_capture.py ../Recordings/data_20261116_1

# 整个 Recordings 批量（子目录里没有 data.h5 的自动跳过）
python3 align_capture.py ../Recordings

# 只看统计不写文件
python3 align_capture.py <目录> --dry-run
```

只需要 Python 3.9+ 与 `h5py`、`numpy`，**代码里没有任何平台分支**，哪个解释器装了这两个
包装得对就用哪个（`python3 -c "import h5py, numpy"` 能过即可）。Ubuntu 上按
`Paxini-Cap/requirements.txt` 装完就能直接跑。

产物：`<采集目录>/aligned/aligned_<参考相机>.h5`。

### 带 Quest 对齐

SDK 时间戳已确认就是主机墙钟，**不需要任何前置验证**：

```bash
python3 align_capture.py <采集目录> --quest-align unix
```

`unix` 是宽松路径：把 `sdk_timestamp_ns` 直接当主机 Unix 用，与相机目标时刻比较、
在相邻两条之间插值。时间戳重复率高或覆盖率低时只警告，不作废产物——手套和相机是
主体，Quest 是可选增强。summary 里 `quest.clock_domain_verified=false` 记录这条路
没做过时钟域验证。

若以后要让时钟域有证据，跑 C8 探针并把报告传给更严格的 `sdk_unix`：

```bash
python3.10 ../Cap_Quest/diagnostics/quest_clock_probe.py --duration 60 --start-pc-service

python3.10 align_capture.py <采集目录> \
  --quest-align sdk_unix \
  --quest-clock-report ../Cap_Quest/clock_probe/run_YYYYMMDD_HHMMSS/report.json
```

`sdk_unix` 是硬门控：报告必须 `passed=true` 且 `clock_domain=unix_epoch`；当前文件的
有效时间戳比例、倒退次数、同一时间戳最长重复必须达到报告阈值；采集期间手套墙钟若发生
跳变直接失败；条件不满足时不写输出文件。报告是同一台主机、同一时钟状态下的**会话级
证据**，输出记录它的 SHA-256 与换算模型。

### Quest 的插值策略

每个相机帧时刻都去 Quest 序列里找**夹住它的两条**（左邻 ≤ 目标 < 右邻），按字段分别处理：

| 字段 | 处理 | 为什么 |
|---|---|---|
| `pose_6d`、`head/pose_xyzw`、`controller_*/pose_xyzw`、`hand_*/joints_pose_xyzw` | 前 3 位（xyz）线性，后 4 位（四元数）**nlerp** | 四元数线性插值会脱离单位球；nlerp 归一化并走短弧（q 与 -q 同一旋转） |
| `controller_*/grip`、`trigger`、`axis` | 线性 | 都是连续量 |
| `tracked`、`available`、`buttons/*` | **取左邻** | 布尔状态不能插出“半个 true” |
| `sequence`、`joint_count`、四个时间戳列 | **取左邻** | 序号/计数/时间戳不是信号 |

与图像的区别要记住：**相机取前驱（不用未来帧），Quest 插值必然用到右邻。** 这是数值
重采样的固有要求，不是因果性退步——任何一行都不会造出括号外的值，`quest/span_ms`
就是括号宽度。跨度大的行插值在硬猜，下游应按它过滤。

`--quest-max-age-ms` 默认 `25`：左邻距目标超过这个值就判该行无效。没有有效配对时相机
行仍然保留，`quest/valid=false`，整组数据按统一缺失语义写 NaN / False / -1。

`--quest-min-valid-fraction` 默认 `0.95`：只对 `sdk_unix` 生效（覆盖率不达标即不写文件）；
`unix` 模式下覆盖率低只警告，因为这份产物还有手套和相机。

当前 `data_20261118_1` / `_2` 的有效 SDK 时间戳只有 90.44% / 86.38%，最长连续重复
5472 / 3128 次——`unix` 模式下仍会产出，但 `quest/valid` 会缺一大片，用前先看
`summary.quest.valid_fraction`。另外 C8 只确认时钟域，**没有测“设备采样 → SDK 打戳”
的链路延迟**，所以不能把 SDK 时间称作曝光真值。

### 实际跑出来的数

| 采集 | 手套帧 | 相机帧 | 对齐行 | 丢弃 | 插值跨度中位 / 最大 | 耗时 |
|---|---|---|---|---|---|---|
| `data_20261116_1` | 1181 | 356（单路） | 300 | 56 | 8.41 / 32.3 ms | <1 s |
| `data_20261113_5` | 4529 | 1239（双路） | 1188 | 51 | 8.38 / 33.8 ms | 1 s |
| `data_20261113_11` | 10394 | 2756（双路） | 2700 | 56 | 8.38 / **130.9 ms** | 1 s |
| `data_20261113_2` | 27904 | 7260（双路） | 7200 | 60 | 8.39 / 35.6 ms | 4 s |

## 输出结构

`aligned_<cam>.h5`，行轴 `K` = 保留的相机帧数。

| 路径 | 形状 | 类型 | 含义 |
|---|---|---|---|
| `timestamp_ns` | (K,) | int64 | 主时间轴 = 相机戳 − Δ，**严格递增**；一行一个样本 |
| `timestamp_cam_raw_ns` | (K,) | int64 | 相机原始戳（未减 Δ），留作溯源 |
| `frame_index` | (K,) | int32 | 参考相机的 mkv 帧号，直接用于取图 |
| `glove/left_index` / `right_index` | (K,) | int32 | 插值所跨的两帧手套下标 |
| `glove/span_ms` | (K,) | float64 | 两帧间距；**这是插值可信度的唯一指标** |
| `glove/offset_ms` | (K,) | float64 | 距左邻帧多少毫秒 |
| `glove/interpolated` | (K,) | bool | 是否真的插值（正好落在手套帧上时为 False） |
| `glove/clock_offset_ns` | (K,) | int64 | 该行左邻手套帧的**墙钟 − 单调钟**。正常是常数；出现台阶就是采集中途被 NTP 校过时，台阶之后的行不可与之前混比 |
| `state/joint_angles_deg` | (K,4) | float64 | 4 路关节角，**线性插值** |
| `state/encoder_raw` | (K,4) | int32 | 4 路编码器，**最近邻**（计数器不是信号，不能平均） |
| `state/resultant_force` | (K,10,3) | int16 | 10 路合力（0.1 N），线性插值后取整 |
| `state/distribute_force` | (K,10,119,3) | int16 | 分布力，线性插值，gzip-1 |
| `state/distribute_counts` | (K,10) | uint32 | 每槽位有效点数，最近邻，合计恒 444 |
| `cameras/<id>/frame_index` | (K,) | int32 | **该路**在主轴时刻之前最近的一帧；多视角训练靠它取另一张图 |
| `cameras/<id>/dt_ms` | (K,) | float32 | 该帧时刻 − 主轴时刻，**恒 ≤ 0**（因果取帧，不会用未来的画面）；唯一例外是主轴早于该路首帧的边界行 |
| `cameras/<id>/timestamp_ns` | (K,) | int64 | 该帧的真实时刻 |
| `quest/index` | (K,) | int64 | 插值**左邻**在源文件里的行号；无配对时为 `-1` |
| `quest/right_index` | (K,) | int64 | 右邻行号；目标晚于最后一条时等于左邻 |
| `quest/weight` | (K,) | float64 | 落在左右邻之间的比例，0 = 正好在左邻上 |
| `quest/span_ms` | (K,) | float64 | 左右邻间距，**这一行插值可信度的唯一指标** |
| `quest/valid` | (K,) | bool | 是否存在年龄 ≤ `--quest-max-age-ms` 的左邻样本 |
| `quest/age_ms` | (K,) | float64 | 目标时刻减**左邻**的主机 Unix 时刻；无候选时为 NaN |
| `quest/timestamp_sdk_ns` | (K,) | int64 | 所选样本的原始 SDK 时间戳；无配对时为 `-1` |
| `quest/timestamp_host_unix_ns` | (K,) | int64 | C8 模型换算后的主机 Unix 时刻；无配对时为 `-1` |
| `quest/data/...` | (K,...) | 原类型 | 完整 `quest/` 数据组**插值后**重建，保留嵌套组名与属性 |

`attrs["summary"]` 是一份 JSON 摘要（行数、`kept_rows`、按来源的丢弃计数、公共有效窗口、
跨度分位、延迟值与 `camera_delay_source`、时钟跳变数、`clock_offset_ms{spread_ms,
max_step_ms, steps_over_5ms}`）。Quest 启用时还会包含 C8 报告 SHA-256、有效配对比例和
年龄分位。`clock_offset_ms.spread_ms > 0` 就说明采集期间墙钟被调过，具体哪一行翻的看
`glove/clock_offset_ns` 自己求差分。`--delay-ms` 没给、台账也读不到时，
`camera_delay_source` 会写成「未标定 Δ=0（台账不可用，未做延迟补偿）」——看到这句就说明
这份产物**没有做延迟补偿**，别拿去和带 Δ 的产物混训。
`attrs["format_version"]` = **2**，与采集端的 `format_version` 4 是两套编号。

`quest/data/` 的无效行按项目统一缺失语义补齐：float → NaN，bool → False，
计数 → 0，SDK 时间戳 → -1。`quest/` 组自身还带 `alignment`、选样规则、
`max_age_ms`、报告哈希和 `host_unix` 模型属性，便于审计。

多视角注意：`cam0` 与 `cam1` 是两路自由运行的 30 fps 相机，**互相并不同步**，
cam1 用的是「主轴时刻之前最近的那一帧」，所以 `cameras/right_cam1/dt_ms` 恒为负，
中位滞后约半帧（一整帧是 33 ms），最小 0、最大接近一整帧，取决于两路相位。
**这个滞后是有偏的**：cam1 永远比 cam0 晚一点。要严格同帧就得用 cam1 自己做参考
再跑一次（`--reference right_cam1`），两份文件行数不同，代价是两份。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `path` | `.` | 采集目录，或 Recordings 根目录（批量；标准位置是 `../Recordings`） |
| `--output` | `<采集目录>/aligned` | 输出目录 |
| `--reference` | 第一个相机 | 作为采样网格的相机槽位 |
| `--delay-ms` | 台账 | 延迟补偿，默认 `pick_latest(..., camera=0)`；**读不到按未标定处理，Δ=0 并告警**（`camera_delay_source` 会写进 summary） |
| `--no-delay` | 关 | 不减延迟，只用于对照调试 |
| `--omit-distribution` | 关 | 不输出 10×119×3，体积降一个量级 |
| `--nearest-distribution` | 关 | 分布力只取最近帧，省一半读取 |
| `--keep-outside` | 关 | 保留手套覆盖范围外的相机帧（默认丢弃） |
| `--dry-run` | 关 | 只统计，不写文件 |
| `--clear-write-flag` | 关 | `data.h5` 打不开时清写标志，见下 |
| `--quest-align` | `none` | `none` / `unix`（直接当主机墙钟，免报告）/ `sdk_unix`（需 C8 报告） |
| `--quest-clock-report` | — | `quest_clock_probe.py` 生成的通过报告 |
| `--quest-max-age-ms` | `25` | 左邻样本距目标时刻允许的最大年龄 |
| `--quest-min-valid-fraction` | `0.95` | 最小有效配对比例，未达标即不写文件 |

## 判据

以下六类判据在**写文件之前**执行，任何一条不过就**不写输出文件**、记入失败计数、
进程最终以非 0 退出（不再有 `--strict` 开关，不合格就是不合格）：

1. 主时间轴严格递增
2. `joint_angles_deg` 无 NaN
3. 每行 `distribute_counts` 合计 = 444
4. 对齐后至少 100 行
5. **跨度 >25 ms 的行不超过 5%** —— 超过说明相机的 33 ms 落在了手套的空洞里
   （手套 120 Hz 名义间隔 8.3 ms，跨 3 帧以上就是插值硬猜），先查 B8 的帧间隔空洞；
   **跨度 >50 ms 的行不超过 1%** —— 这类行等于用一个 130 ms 的空隙两端硬连一条直线，
   数值没有任何依据（`data_20261113_11` 就有 4 行落在 131 ms 的空洞里）
6. Quest 启用且为 `sdk_unix` 时，有效配对不少于 `--quest-min-valid-fraction`（默认 95%）。
   `unix` 模式不查这条——覆盖率低只影响 `quest/valid`，不该作废好手套数据

第 5 条是软的边界，不是"过或不过"：`data_20261113_2` 实测 200/7200 行（2.8%）超过
25 ms，来源是那批 20~35 ms 的单帧空洞，属预期、可通过。训练脚本仍应按
`glove/span_ms` 自己过滤（例如 `span_ms < 25`），别整份照单全收。

Quest 的 C8 报告、源时间戳覆盖率、倒退和冻结重复是**更早一层**的硬门控，在
`align_recording` 内部就抛错，连统计都做不出来。两层的共同点是：**不合格就没有产物**。

## 坑

**13 份采集中 8 份打不开——整份作废，不是"丢最后几秒"。** 这些是采集中途被强杀的
文件：超级块带着"仍处于写打开"的一致性标志，h5py 拒绝只读打开
（`file is already open for write`）。`--clear-write-flag` 会清掉标志**并重算超级块
校验和**（这一步不能省，只翻字节会造成 `incorrect metadata checksum`，看起来像文件坏了，
其实是修复动作自己弄坏的），但那 8 份清完仍然读不出根组对象头（`bad object header
version number`）：`data.h5` 的第 48 字节起全是 0，**HDF5 的元数据树从未落盘**。

原因是采集端的写入策略：`Hdf5Recorder` 每 100 行 resize 一次但 `file.flush()` 原本只在
`close()` 里调，进程被强杀时文件里只有超级块，没有任何目录结构。**这个坑 2026-11-17 已修**
（每批都 flush，强杀最多丢最后不满一批的行；真实环境强杀修复后仍能读出 500/550 行，
修复前只剩 48 字节）。但那 8 份老文件是修复前产生的，救不回来。

另外 `data_20261113_1` 能打开但没录相机。确认可用的是 `data_20261113_2 / _5 / _11`
和 `data_20261116_1`，共 4 份。

**`data_20261113_1` 没有相机。** 只有手套，无法确定采样网格，程序直接报错退出——
这是对的，没有相机就没有"一行一个样本"这件事。

**延迟 Δ 两路共用 cam0 的值。** 两路都已在原生 V4L2 路径上实测（2026-11-21：
cam0 **84.7 ms**、cam1 **87.6 ms**，IQR 7.3 / 8.3 ms，均 `ok: true`），台账在
`Cap_Quest/latency/records/ifnot-ubuntu.yaml`；程序按 `pick_latest(..., camera=0)` 取
cam0 那条，两路共用。差 2.9 ms，相对半帧（33 ms）可忽略，所以没有改成逐路取。
要严格逐路补偿就显式 `--delay-ms` 单独跑一份。

**不要中途读正在写的 `data.h5`。** 采集端攒 100 行才落盘，看到的是 0.83 s 一跳的批数据。
本程序只处理已经采完的目录。

**不要拿 sequence 或重复时间戳代替 C8。** `quest/sequence` 只说明轮询序号，不是物理时间；
`sdk_timestamp_ns` 数值看起来像 Unix 也不构成时钟域证据。本程序要求通过报告，并在
`data.h5` 文件级质量门控失败时拒绝执行。

**Quest 对齐不等于 Quest 延迟标定。** C8 解决的是时钟域与换算模型；目前没有测
“设备采样 → SDK 时间戳”这一段。`quest/age_ms` 只表示所选 SDK 时刻距相机目标时刻多远，
不能据此宣称姿态与像素曝光在同一物理瞬间。

## 自检

```bash
python3 selftest.py     # 30 项，不需要硬件，用合成数据精确校验插值与 Quest 门控
```

覆盖：线性信号零误差还原、Δ 逐行精确相减、边界丢弃计数、`--keep-outside`、
整数走最近邻、counts 合计 444、`--omit-distribution`、时间基自动判定、
Quest unix 模式免报告插值、四元数 nlerp 走短弧、布尔/整数取左邻、
cam1 因果取帧（dt_ms 始终 ≤ 0）、输出可重开、质量判据不合格时不写文件、乱序时间戳必须报错，
以及 Quest 选样 tie-break、缺失语义、完整字段复制、C8 时钟模型反算、
文件质量硬门控、有效配对判据和缺报告拒绝。
CLI 往返和硬门控非 0 退出码也在自检内。
