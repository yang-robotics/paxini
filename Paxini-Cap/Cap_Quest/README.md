# PXCap III + Quest 3 + 双路相机采集

一只 PXCap III 数据手套有一个串口、没有电机。程序连接手套、读取设备信息，然后启动
SDK 连续采集回调；同时用 ffmpeg 直采两路 UVC 相机，并把 Quest 3 的样本一起写进
**单个 HDF5**。

**这个工程只负责采集，不做任何对齐。** 三个来源各写各的、各带自己的时间戳，
谁跟谁配、按什么阈值配，留给下游处理工程（见 `../DOC/待验证清单.md` 与待建的
`Paxini-Align/`）。这样采集端不会替下游做决定，也不会因为推断错而污染原始数据。

**采集端也不碰像素、不做标定**：没有 OpenCV，没有内参/外参，`meta/` 里没有标定出处。
**相机标定已决定不做**（2026-11-17，理由与边界见 [`../DOC/待验证清单.md`](../DOC/待验证清单.md)
E8）：下游扩散模型把图像直接当观测输入，不解算位姿、不做三维重建，所以不需要内参/外参。
代价是这套数据能做"随时间变化"的分析，做不了任何"距离/位置/三维"的判断。设计留档见
[`../DOC/标定.md`](../DOC/标定.md) §2。

**手套零点标定是独立工具，不在采集主循环里**：`hand_calib.py` 走 SDK 的
`set_encoder_calibration` / `set_sensor_calibration`，把零点写进手套存储（设备端持久化，
不是主机侧修正），带静止/空载闸门与标零后复读验证。用法与判据见
[`../DOC/标定.md`](../DOC/标定.md) §1。

相机不用 OpenCV：`cap.read()` 只能拿到"调用返回时刻"，帧在驱动队列里等了多久未知；
ffmpeg 让每帧 pts 直接取主机墙钟并原样写进容器，停止后用 `ffprobe` 回读。

> 接手这个仓库（人或 AI）先看根目录 [`../README.md`](../README.md) 的 **§0 接手必读**：
> 里面有代码地图、不能违反的约定、环境坑和提交前必过的三关。本文件讲怎么用，
> 根 README §0 讲怎么改。

## 0. 目录结构

2026-11-16 按用途拆过目录。**下面所有命令都在 `Cap_Quest/` 根目录下执行**，
文件名带目录前缀。

| 目录 | 内容 | 入口 |
|---|---|---|
| `capture/` | 采集主程序与设备驱动层 | `collect_cap.py`（主）、`camera_ffmpeg.py`、`quest3.py` |
| `calib/` | 手套零点标定（写设备存储；报告落 `calib/hand/`，不入库） | `hand_calib.py` |
| `latency/` | 相机延迟测量 | `measure_camera_latency.py`（V4L2）；结果记在 `records/` |
| `viewer/` | 可视化 | `visualize.py`（回放）、`quest_view.py`（Quest payload）、`live_view.py`（实时） |
| `diagnostics/` | 时钟域诊断 | `quest_clock_probe.py`（C8 探针；产物落 `clock_probe/`） |
| `selftest/` | 不接硬件的自检 | `selftest.py` |
| `cap_config.yaml` | 采集配置，采集 / 延时测量 / 标定三处共用 | — |
| `QUICKSTART.md` | 当前机器的 XRoboToolKit 联合采集快速上手 | — |

采集输出**不在本目录**：默认写到 `<Paxini-Cap>/Recordings/data_<日期>_<序号>/`，
采集端与 `Paxini-Align/` 读同一批目录（见 §4）。

跨目录依赖靠各脚本开头的 `sys.path` 注入解决：`viewer/`、`selftest/`、`latency/`
都会把 `capture/` 加进搜索路径。`calib/` 只依赖 SDK，不引用采集模块，跟 `capture/`
没有耦合。所以所有命令都在本目录（`Cap_Quest/`）下执行，不需要 `cd` 到子目录。

## 1. 快速开始

```bash
cd Paxini-Cap/Cap_Quest
python3.10 capture/collect_cap.py --list-devices
```

ffmpeg 需要 5.0 以上（要 `-fps_mode`）。当前 Ubuntu 机器已经把静态版装在
`/usr/local/bin`，`--list-devices` 会直接打印 `7.0.2-static`，**不用再 export
`FFMPEG`**。换一台机器时才按 [`../README.md` §1.1](../README.md) 换静态版，或者
`export FFMPEG=/path/to/ffmpeg-5-or-newer` 指过去。

`--list-devices` 打印串口（含 USB VID:PID 与 by-id）、视频设备和 ffmpeg 版本。
先确认两件事：

- **串口那行末尾是 `PXCap`（`USB=28e9:018a`）**。没有串口就先查线缆、供电和
  `dialout` 权限（`usermod -aG dialout $USER` 后要重新登录）。
- **V4L2 节点数量对得上**。两路 MASTER Camera 应各有一个 `index 0` 采集节点；
  一个都没有说明相机没被内核识别，先 `v4l2-ctl --list-devices` 看设备在不在。

确认无误后再运行：

```bash
python3.10 capture/collect_cap.py --cap-side right --pose-source controller --pose-side right
```

常见变体：

```bash
python3.10 capture/collect_cap.py --duration 60        # 采 60 秒后自动停
python3.10 capture/collect_cap.py --camera-mode cam0   # 只录一路，排除带宽问题
python3.10 capture/collect_cap.py --no-quest           # 没有头显时只采手套 + 相机
python3.10 capture/collect_cap.py --camera-mode none   # 不录视频
```

**三个来源各自启动，互不等待**（2026-11-20 起；2026-11-23 删掉就绪期）。
**配置了就算必需**：哪一路停了，整场录制就中止，不再有「缺一路照样录完」的降级。

```text
手套       始终必需。连接/启动失败 → 按启动失败退出，不生成 data.h5
相机       由 --camera-mode 决定。启动失败 → 启动失败退出（只录一路用 --camera-mode cam0）
Quest      由 --no-quest 决定。装配失败（PC Service / ADB / SDK）→ 启动失败退出，不生成 data.h5
```

不戴头显就别靠“配了但没连”蒙混过去：显式加 `--no-quest`，否则程序按「配置了就得在场」
处理，直接判启动失败。

启动阶段没有就绪检测、也不等待。检查全部发生在采集循环里（`RuntimeMonitor`）：

| 检查 | 频率 | 判据 | 处置 |
|---|---|---|---|
| 相机进程 | 1 Hz | `process.poll()` 非空 | 中止整场 |
| 相机出帧 | **0.5 Hz** | 启动宽限（`--camera-start-timeout`，15 s）过后 4 s 进度不增长；从没出过帧也走这一条 | 中止整场 |
| Quest 轮询 | 1 Hz | `failure` 非空（线程内异常，`QuestPoller` 自己也会中止）；或 5 s 内 `samples` 不增长（线程卡在 SDK 调用里） | 中止整场 |
| 手套出帧 | 每帧记时刻 | 连续 3 s 无新帧（没有宽限期：手套要么立刻出数，要么就是坏了） | 中止整场 |

中止不是崩溃：走 `abort_recording()` → `STOP_EVENT` → 正常收尾（排空队列、停相机、
回填视频时间戳、写 manifest），`manifest.error` 写明原因，退出码 2。所以「中止」的
产物是可读的，不是半份垃圾。

`cameras.first_frame_s` 是「第一次采样看到进度增长」的时刻，受 0.5 Hz 采样限制，
精度是 2 s 的整数倍，别拿它当精确的首帧延迟。

**没配置的源不检查**：`--camera-mode none` 时相机列表为空，一条相机检查都不会跑；
`--no-quest` 时压根没有轮询线程。三路的起止时刻必然不同，manifest 分别记
`hand.first/last_timestamp_unix_ns`、`cameras.per_camera.<id>.first/last_timestamp_unix_ns`、
`quest.first/last_sdk_timestamp_ns`。

`--skip-adb` 可跳过 ADB，但必须自己先打开 XRoboToolkit 并开启 Tracking 和 Send。

单只 PXCap 通过 `--cap-side left|right` 选择手别。Quest 默认由
`--pose-side auto` 跟随手套手别；需要“左手套 + 右手柄”等特殊组合时，可显式传
`--pose-side left|right` 覆盖。选择手别只影响 provenance 和相机命名，手套原始字段
原样保存，不做镜像或符号转换。

启动在创建 `data.h5` 之前失败时，程序保留本次目录并写
`startup_failure.json`，同时保留 `cameras/*.log`、完整 ffmpeg 命令和日志末尾；
不会生成伪造 HDF5。若 `--output-dir` 原本已存在，诊断写到旁边的新目录，不污染旧数据。

#### 没有一键脚本（2026-11-20 起）

原来有一份 `scripts/run_capture_noquest.sh`，把「找 ffmpeg → 查串口 → 关 NTP → 起采集 →
读 manifest 核对行数」串成一条命令。2026-11-20 删除：它维护的东西（ffmpeg 位置、设备节点、
参数拼装）与 `collect_cap.py --help` 完全重复，而且三源解耦之后启动条件不再是「先查齐再跑」，
脚本那套前置检查反而会挡住合法的部分采集（比如只接一路相机）。

现在就是两条命令，顺序别改（ffmpeg 已装在 `/usr/local/bin`，换机器才需要
`export FFMPEG=...`）：

```bash
sudo timedatectl set-ntp false
python3.10 capture/collect_cap.py --no-quest          # 没有头显时
```

要采固定时长加 `--duration 60`，要边采边看加 `--view`，其余参数一律以 `--help` 为准。
采完怎么判合格（行数、`row_mismatch`、每路帧数、视频体积）见 §4 与 `../QUICKSTART.md` 的
「合格检查」。

#### 换一台机器要改什么

代码里的路径都相对**脚本所在目录**解析（`cap_config.yaml`、`../Recordings/`），
所以只要仓库结构不变就不用改代码。真正会挡住别人的是三件事：

| 项 | 本机现状 | 别人怎么办 |
|---|---|---|
| Python | 系统 `python3` 是 3.12，`PxCap3` 只在 `python3.10` 里 | 用 `python3.10`（或 `PYTHON=` 指定） |
| 手套 SDK | `pxhandsdk 1.0.0.0` 已 `dpkg` 装 | 装 SDK + `setup-serial-access.sh` + 重登让 `dialout` 生效 |
| ffmpeg | 静态版 7.0.2 在 `/usr/local/bin` | 5.0 以上，需带 `v4l2` 输入与 `-fps_mode`，装法见 [`../README.md` §1.1](../README.md) |

相机走 `/dev/videoN`，节点序号可能随机（两路同型号尤其明显），所以默认 `auto` 每次
自己枚举；要钉死就写 `--camera-0 /dev/video2 --camera-1 /dev/video0`。

## 2. 时间戳

三个来源各自打点，**互不配对**：

| 来源 | 打的什么 | 存在哪 |
|---|---|---|
| 手套 | SDK 每帧给 `timestamp_unix_ns` + `timestamp_monotonic_ns` | `cap/` 两列 |
| Quest | SDK 给四个：`sdk_` / `body_` / `motion_` / `body_joints_timestamp_ns` | `quest/` 四列 |
| 视频 | ffmpeg `-use_wallclock_as_timestamps 1` + `-copyts`，pts 本身就是墙钟 | `cameras/<id>/timestamp_unix_ns` |

- 手套 120 Hz（`--hand-rate`，上限 300），Quest 轮询 120 Hz（`--quest-rate`）。
- 相机默认 30 fps；**每路帧数 M 与手套帧数 N、Quest 行数 Q 互不相等**。
- 相机时间戳只能是墙钟：V4L2 的时钟类型由驱动声明、ffmpeg 丢掉该标记，无法指定
  也无法确证，所以不做推断（细节见 `../DOC/HDF5结构说明.md` §5.1）。

时钟域、换算公式、四个坑，全部整理在 [`../DOC/时间戳说明.md`](../DOC/时间戳说明.md)。

### 墙钟跳变（务必看）

手套的 `timestamp_unix_ns` 和 ffmpeg 的 pts 都是墙钟，会被校时 step 改写——NTP
守护进程、虚拟化层的宿主时间同步都做这件事，开机、休眠唤醒、偏移超阈值时都可能
触发。本机实测采集期间出现过 **2~5 秒**的多次跳变。Quest 那四个值不受影响，因为
还不知道它们属于哪种钟（见 §2 与待验证清单 C8）。

采集端**不再检测也不再修正**：原始时间戳原样落盘，跳没跳由下游判断（对比同一台
机器的 `timestamp_unix_ns` 与 `timestamp_monotonic_ns` 即可看出）。想少踩这个坑：

```bash
sudo timedatectl set-ntp false      # 采完再 set-ntp true
```

手套 120 Hz 与相机 30 fps 各跑各的，空洞由 §4.3 的跨度判据暴露。

## 3. Quest 数据

`quest/pose_6d` 是**选定**的主位姿，顺序 `x, y, z, qx, qy, qz, qw`，米 + 四元数
`xyzw`，坐标系是 XRoboToolkit 的 Unity tracking-origin（`+X` 右、`+Y` 上、`+Z` 前，
左手系）。默认 `--pose-source controller --pose-side right`。

其余字段把整个样本都存下来，不受 `--pose-source` 影响。**一次轮询 = 一行**，
没采到就是没有这一行。

四列时间戳都是 SDK 给的，我们不再追加主机侧读数。因为 `sdk_timestamp_ns` 的
时钟域官方没写，所以不能直接和手套的单调钟相减——先验证域，再谈对齐。

四个值连同位姿、按键一起在 `quest3.py` 的 `XrtReader.collect_sample()` 里取好并
返回；`collect_cap.py` 的 `QuestPoller` 只入队、只写行，不调 SDK getter。
**要加/改 `quest/` 的时间列，改 `quest3.py`。**

## 4. 输出

```text
../Recordings/data_<YYYYMMDD>_<当天序号>/     # 相对 Cap_Quest；实际在 <Paxini-Cap>/Recordings/
├── data.h5
├── manifest.json
└── cameras/
    ├── <cap_side>_cam0.mkv  <cap_side>_cam0.log  <cap_side>_cam0.progress
    └── <cap_side>_cam1.mkv  <cap_side>_cam1.log  <cap_side>_cam1.progress
```

`.log` 是 ffmpeg 的 stderr，`.progress` 是 `-progress` 输出，都用来看相机是否真的
在出帧，可以删。启动失败时同目录还会有 `startup_failure.json`，且不会创建
`data.h5`；看到该文件表示本次不是成功采集。

### 4.1 `data.h5`

三条独立行轴：`cap`（N）、`quest`（Q）、`cameras/*`（M）。视频像素在 mkv 里，
HDF5 只存索引性质的时间戳。

> 完整目录树、逐字段含义（单位/顺序/缺失语义）、读法示例见
> [`../DOC/HDF5结构说明.md`](../DOC/HDF5结构说明.md)。

### 4.2 `manifest.json`

关键字段：

| 字段 | 内容 |
|---|---|
| `rows` / `quest_rows` | 手套帧数 N / Quest 样本数 Q |
| `actual_rate_hz` | 按首末帧**单调钟**跨度算的实际频率 |
| `duration_s` / `sample_span_s` | 墙钟耗时 / 手套首末帧跨度 |
| `alignment` | 固定 `none (raw per-source timestamps only)` |
| `hand` | 手别 `side`、Quest 请求手别 `pose_side_requested`、串口、SN、SDK/固件版本、故障、回调耗时、队列溢出、**首末帧时刻** `first/last_timestamp_unix_ns` 与 `first/last_timestamp_monotonic_ns` |
| `quest` | enabled、rows、samples_dropped、轮询次数与超时（`poller_overruns`）、`poller_alive`、**`errors`**（被 `XrtReader.call` 吞掉的 getter 异常，按接口名各记一条）、**首末 SDK 时间戳** `first/last_sdk_timestamp_ns`（域未验证，只作增量参考） |
| `cameras` | 每路设备、帧数、分辨率、实际 fps、时间窗、错误、`first_frame_s`；汇总 `first_frame_s` |
| `runtime` | `abort_reason`（中止原因，正常结束为 null）、`configured_sources`、出帧检查频率与两个停滞阈值、`camera_stalls` / `glove_stalls` |
| `verify` | 行数自检（`row_mismatch` 必须为空） |
| `env` | python / numpy / h5py / ffmpeg 版本 |

`verify.row_mismatch` 非空说明某组数据集首维和自己的行数不一致，这种文件能打开，
但下游按行取数会整体错位。

### 4.3 回放

`visualize.py` 直接读采集目录，不需要重新解码视频：

```bash
python3.10 viewer/visualize.py ../Recordings/data_<日期>_<序号>          # Tk 窗口
python3.10 viewer/visualize.py ../Recordings/data_<日期>_<序号> --html   # 导出 viewer.html
python3.10 viewer/visualize.py ../Recordings/data_<日期>_<序号> --save-png run.png
```

- 窗口里三张曲线（关节角 / 合力模长 1~5 路 / 合力模长 6~10 路）+ 右侧一张
  **10 路合力数值表**，共用一条时间轴；空格播放，`←` `→` 逐帧，点曲线跳转，底部滑块可拖
- 浏览器版另有一个倍速按钮，在 1.0× → 2.0× → 0.50× → 0.25× 之间循环；播到末尾自动
  停在最后一帧，再按空格从头开始
- 浏览器版把鼠标移到曲线上会出虚线游标 + 数值提示框（该时刻的时间、帧号和该图前 6 条
  曲线的值），不用再靠滑块一点点拖去找某个尖峰
- 顶部信息条除了时长/帧数/速率，还给出**保峰抽稀比例**与**时钟漂移**（`unix − monotonic`
  的首末差、>10 ms 的跳变次数）；有跳变时同时弹一条警告
- `--html` 导出 `viewer.html`，手套/Quest 数据内嵌，双击就能看，方便发给别人；视频除外
  —— 转码后的 mp4 放在同级的 `preview/` 下，拷给别人时要连目录一起拷
- `--save-png` 用 Agg 渲染首帧，无显示器也能出图

#### 分布力空间散点图

浏览器版在曲线和 10 路合力表下方增加一张**分布力散点图**卡片，读取
`cap/distribute_force` 的 `(F, 10, 119, 3)` 数据：

- 每个槽位的测点形状优先来自本地 PXCap Studio 1.0.0 的 49/20/119 点模型
  （`viewer/sensor_layout.py`，不入库）；没有这个文件时自动回退到
  `viewer/sensor_layout_schematic.py` 的规则网格；
- 回退网格的 49/20/119 纵横比和点序号排列与真实模型一致，因此
  119 点掌心长条长度与颜色图案方向不变；但点距是均匀示意；
- 10 块贴片按 PXCap III 参考图的布局摆放，**没有手形轮廓，
  贴片位置是示意图，不是标定坐标**；
- 右手布局：左列中指3/中指2，中列中指1/中指0，右列食指1/食指0；
  中间长条掌心0（119 点横向），左下掌心1，右下拇指1/拇指2；左手镜像 x；
- 分布力卡片使用参考图编号（拇指1/2、食指0/1、中指0~3、掌心0/1），
  与上方 10 路合力表的旧名称一一对应；
- 每块贴片按 `sensor_layout.PATCH_INSET`（默认 0.10）向内留白，
  相邻槽位不会连在一起；想加大间距就调大这个值；
- 每个槽位只画 `distribute_counts` 之内的有效点，补齐区的 0 不会当成测点；
- 点按 `|F|` 着色和大小变化，右侧列出各槽位当前帧峰值；颜色标尺固定为
  0~25 N（传感器单点受力上限），超过 25 N 的值显示为红色；
  颜色和点径都用 `γ=0.5` 的同一个幂函数（`DIST_COLOR_GAMMA`）提升低力段；
  需要改上限时调整 `visualize.py` 的 `DIST_POINT_FORCE_MAX_N`；
- 每个测点还可显示切向合力箭头：只用 `Fx/Fy`（`Fz` 是法向压力），
  长度按切向模长、方向为 `atan2(Fy, Fx)`，随贴片旋转和左右手镜像；
  默认开启，卡片内可关闭；切向模长低于 0.1 N 的点不画箭头；
- 真机复测（在各槽位点阵翻转后）原始 `Fx/Fy` 已与 2D 贴片图方向一致，
  当前 `TANGENT_SIGN = 1`；若以后再出现整体 180° 反向，
  把这个常量改成 `-1` 即可；
- 真机核对后，食指/中指/掌心0 的点阵做“中心对称 + 纵轴镜像”，
  合并后等价于横轴镜像（只翻 y）；拇指/掌心1 做纵轴镜像（只翻 x）；
  切向箭头随同一变换翻转，见 `sensor_layout.SLOT_FLIPS`；
- `--no-dist-arrows` 不打包切向数据，HTML 约少 1.9 MB/600 帧；
- 悬停显示槽位、点号和该点 `|F|`；播放、逐帧、滑块与合力表共用
  `--dist-frames` 采样帧，因此散点图和数值表始终同一帧；
- 采集时用了 `--omit-distribution` 的话，卡片显示“本次采集没有分布力数据”，
  不会报错；
- 体积代价：默认 600 帧约增加 1.9 MB HTML；需要更小就调低 `--dist-frames`；
- **授权提醒**：测点坐标来自厂商 Studio 的 `UNLICENSED` 数据；公开再分发前
  请确认厂商授权。

#### Quest 3 面板

浏览器版会自动检测 `quest/`。有数据时增加 Quest 面板，无数据时自动隐藏；`--no-quest`
可强制不导出。面板根据 manifest 的 `pose.side` / `hand.side` 自动选择本次采集手
对应的手柄，包含：

- 该手柄的位置 XYZ 曲线（米）和姿态四元数 xyzw 曲线；
- 该手柄的 3D XYZ 轨迹，同时显示头显和左右手柄当前位置；
- 当前 sequence、SDK 相对时间、主位姿与四元数、追踪状态、手柄输入和手部关节数；
- 缺失 SDK 时间戳保留断点；整列不可用时降级为轮询 sequence，并明确提示。

**Quest 与手套/相机未做跨源时间对齐。** 页面共用播放/暂停和倍速控件，但 Quest
使用自身归一化进度；页面顶部、Quest 分区和 Tk 摘要都会显示这一限制。原因是 C8
尚未确认 `sdk_timestamp_ns` 的时钟域，不能按真实同一时刻配对。

默认 `none` 模式把 Quest 位置/四元数曲线映射到 `0%~100%` 归一化横轴，使图宽和
游标与手套主图对齐，并复用主网格列宽和绘图边距，使横轴左右边界与“合力模长 B”
完全一致；这不代表物理时间。`sdk_unix` 模式改用与手套相同的相对 Unix 秒轴。
3D 轨迹固定放在右侧。

`pose_6d` 是选定主位姿的紧凑表示：`x,y,z` 是米制位置，`qx,qy,qz,qw` 是姿态
四元数；它不是手部 26 关节骨架，也不代表已经做过相机/手套空间外参标定。

#### C8 时钟探针与可选对齐

先运行独立探针。它只写 `clock_probe/run_<时间>/samples.jsonl` 和 `report.json`，
不会修改 `data.h5`：

```bash
python3.10 diagnostics/quest_clock_probe.py \
  --duration 60 \
  --start-pc-service
```

报告检查有效时间戳覆盖率、探针有效时长、时间戳倒退、长时间重复冻结、相对主机
Unix 时钟的残差和时钟斜率。默认阈值为有效覆盖 ≥95%、有效时长 ≥目标的 95%、
同一时间戳连续重复 ≤10 个轮询周期、残差中位数 ≤10 ms、p99 ≤30 ms、
斜率误差 ≤100 ppm。

现有 `data_20261118_1` 不满足文件级门控：有效时间戳比例 90.4%，最长同一时间戳
连续重复 5472 次，因此即使提供一份通过的时钟探针报告，查看器也会拒绝对齐该文件。

只有报告为 `passed=true` 且 `clock_domain=unix_epoch`，才允许显式启用对齐：

```bash
python3.10 viewer/visualize.py ../Recordings/data_YYYYMMDD_N \
  --html \
  --quest-align sdk_unix \
  --quest-clock-report clock_probe/run_YYYYMMDD_HHMMSS/report.json
```

对齐后 Quest 时间换算到手套首帧 Unix 时间轴。游标取目标时刻之前最近的 Quest 样本，
不插值；同一时间戳重复时当前读数取最后一条，轨迹仍保留全部有效点。默认
`--quest-align none`，未通过报告时 `sdk_unix` 会直接报错，不会静默对齐。

#### 视频面板（只做浏览器版）

`--html` 默认把每路相机 mkv 转成 `preview/<cam>.mp4`，挂在曲线下方，和手套时间轴
联动；`--no-video` 关掉，`--video-width`（默认 640）、`--video-crf`（默认 26）和
`--ffmpeg` 可调。

- **必须转码**：Chrome/Edge 不认 Matroska，mkv 里又是 MJPEG，只有 h264 的 mp4 才进得
  了 `<video>`。命令里带 `-fps_mode passthrough`（保住原始帧间隔，掉帧不会被 CFR 抹掉）
  和 `-g 15`（每半秒一个关键帧，拖动才不卡）
- **对齐怎么算**：相机首帧的 `timestamp_unix_ns` 减手套首帧 unix 得到 `offset_s`
  （面板上显示成「首帧 −1.48 s」），视频时间 = 手套相对时间 − offset。每个手套帧 seek
  到**最近一帧的中点**，误差因此不超过半帧（30 fps 即 ±17 ms）。映射按**时间**而不是
  帧序号，所以相机掉帧也不会让后面的对齐越飘越远
- **联动**：播放/暂停/倍速/逐帧/点曲线跳转都会同步；暂停时视频一起停住，不会自己往下跑
- **实测成本**：15 s 双路 720p（mkv 各 31 MB）→ 两个 mp4 共 1.1 MB，转码 + 出 html
  合计 2.5 s。默认 640 宽、crf 26，够看清手指动作
- **Tk 窗口没有视频**，只有浏览器版有；实时视图另算（见 4.4）

#### 布局与滚动

页面是「左列三张曲线、右列当前帧 + 合力表、视频条在最下」，三块都不写死像素：

- **分布力有效点数那张图删掉了。** 它和合力模长讲的几乎是同一件事（点数非零即受力），
  十几条曲线挤在一张 150 px 高的图里谁也认不出来，所以拿掉，位置让给下面那条
- **合力模长拆成两张**：`合力模长 A` 画 1~5 路、`合力模长 B` 画 6~10 路，各自 5 条线
  一套图例。5 条比 10 条稀一半，图例也能一眼对上曲线
- **10 路合力表在右列**（约 470 px 宽，占两行高），六列：槽位 / Fx / Fy / Fz / |F| /
  点数；`table-layout: fixed` + `height: 100%`，11 行（10 路 + 合计）均分整个面板高度，
  不会在表格下面留一条空带。数值右对齐、等宽数字，合计行用琥珀色并加分隔线。力值按
  **N** 显示两位小数（文件里存的 0.1 N 整数 ÷10，等值无损）；整数显示会把 `|F|` 的
  1.73 舍成 2，小力段误差能到 15%，也看不出 0.2 N
- **当前帧卡片**：4 路关节角的小表（关节 / deg）+ 一行**合力合计**（Fx/Fy/Fz/|F|/点数）
  + 帧号、时间与速率
- **视频条自适应**：`repeat(auto-fit, minmax(320px, 1fr))`，宽屏两路并排、窄屏自动落成
  一列；每路 `width: 100%`、`object-fit: contain`，单图最宽 900 px，既不会拉伸变形，
  也不会在宽屏上只占中间一小块
- **页面可以滚动**：`#grid` 最小高度 `max(520px, 100vh − 108px)`，一屏够高时正好撑满、
  不够高时才出滚动条；底栏 `position: sticky` 常驻底部，`main` 底部留 70 px 空白，
  滚到底也不会被播放条压住

2026-11-13 实测（1024~1920 三档视口）全程无横向溢出、无遮挡，逐档尺寸见
[`../DOC/待验证清单.md`](../DOC/待验证清单.md) F9。

> 把采集目录用 `python -m http.server` 挂出去看时，浏览器拿不到 Range 响应，`<video>`
> 会变成**不可 seek**（`seekable` 为空），只能从头播。直接双击 `viewer.html`，或者用
> 支持 Range 的服务器。

> **手套没戴、手指没施力时合力表整张是 0。** 2026-11-13 实测：空负载下
> `distribute_force` 最大 0、非零占比 **0.0%**，`joint_angles_deg` 只在 −0.04~0.13°，
> `resultant_force` 三轴全 0。此时曲线是噪声底、数值表全是 0，**这是数据本身为零，
> 不是渲染 bug**。正式采集务必让手指真的受力，否则回放看不出任何信息。

横轴默认用 `cap/timestamp_monotonic_ns`，没有就退回墙钟并提示倒退处数。曲线超过
`--max-points`（默认 4000）会**保峰抽稀**：每段的极小值和极大值都保留，所以单帧尖峰
不会因为抽稀而消失（12000 帧 → 2563 帧，13 个单帧尖峰一个不丢）；合力表默认最多留
600 帧（`--dist-frames`），按等间隔取，播放时取时间上最近的那一帧。
**Quest 与手套仍是两条轴**，这里不做配对，只是各自显示。

### 4.4 实时视图

采集时加 `--view`，边录边看：

```bash
python3.10 capture/collect_cap.py --cap-side right --view                    # auto：按系统自己选
python3.10 capture/collect_cap.py --cap-side right --view --view-mode html   # 强制浏览器版
python3.10 capture/collect_cap.py --cap-side right --view --view-mode tk     # 强制 matplotlib 窗口
```

**`--view-mode auto`（默认）按显示会话落点：**

| 环境 | 落点 | 原因 |
|---|---|---|
| 有图形会话（X11 / Wayland） | `tk` | 有正常 WM，窗口该多大就多大、任务栏能点击还原，窗口版更省事，不用切浏览器 |
| 无显示会话（SSH、纯 tty、没有合成器的容器） | `html` | 没有窗口管理器时 Tk 窗口会被摆到屏幕外或压在别的东西后面，浏览器版不受影响 |

判断方式是 `DISPLAY` / `WAYLAND_DISPLAY` 是否存在（无显示会话时退回 `html`，判定见
`live_view.has_display_session()`）。两种模式的内容完全一样，只是壳不同。

**`html` 模式**：启动时打印 `http://127.0.0.1:8765/`（端口被占自动顺延），页面每 150 ms
拉一次 `/state`，两路相机走 MJPEG（`/cam/<id>`），`age_s` 超过 2 s 会显示"数据停了"。
可以拖到第二块屏、随便缩放，关掉页面不影响录制。

**`tk` 模式**：matplotlib 独立窗口，按屏幕尺寸自算位置与大小；关窗或按 `q` 只关视图。

**两种模式里都没有曲线，只有数字**：左边 4 路磁编码器关节角（30 pt 大字，一眼看出戴没戴正），
右边 **10 路合力数值表**（槽位 / Fx / Fy / Fz / |F| / 点数 + 合计，两位小数 N），开了相机
的话再加一排**相机关键帧**。采集时人要盯的是"姿势对不对、力施加了没有"，一个滚动 6 秒
就跑完的曲线不如一个大数字直接；曲线留给回放页慢慢看。

**`tk` 模式跑在一个 fork 出来的子进程里。** 采集主循环每帧只做一次 `norm` 加一次
`queue.put_nowait`，实测 **130 µs**（p99 262 µs），占 120 Hz 下 8.3 ms 预算的 1.6%。
绘制、Tk 事件循环、字体渲染全在子进程：

- 子进程跟不上时丢帧，队列上限 240 帧（约 2 秒），`dropped` 会记下来，结束时打印
- 关窗或按 `q` 只关视图，录制继续
- 视图崩溃不影响录制，`push` 发现子进程已死就直接返回
- 子进程忽略 SIGINT，Ctrl+C 只作用于采集端

`html` 模式同样 fork 一个子进程：父进程每帧只做一个 `put_nowait`（小元组 + 最多两路 JPEG），
HTTP 服务、页面、JPEG 缓冲、MJPEG 连接全在子进程里。两处细节：

- **端口由子进程绑、再用 pipe 报回父进程**，所以启动日志里的 URL 一定是真实端口
  （8765 被占时自动顺延，不会出现"日志写 8765、实际 8766"）
- 子进程挂了或被杀，`push` 发现进程不在就直接返回，录制照常（实测 kill -9 后手套仍 114.5 Hz）

不读 `data.h5`。录制器是攒 100 行才落盘的，读文件会看到一跳一跳的批数据；而且
非 SWMR 的 HDF5 同时读写在 chunk 边界上不安全。


#### 实时相机关键帧

`--view` 打开时，每路相机都会在窗口底部给一帧缩略图（`--view-video-fps` 默认 5、
`--view-video-width` 默认 360）。没有单独的开关：不看视图就没有缩略图，
开了视图就一定有——省掉一个只影响显示的布尔参数。

- 实现是给**录制那条 ffmpeg 加第二个输出**：`-map 0:v -vf fps=5,scale=360:-2 -c:v mjpeg
  -f image2pipe pipe:1`，主进程读 stdout、按 SOI/EOI 切出完整 JPEG，只保留最新一帧。
  另起一条 ffmpeg 抢同一路相机是不行的——V4L2 设备被占用时第二路直接打不开
- 抽到 5 fps 是刻意的：窗口本来 20 Hz 刷新，全帧率既看不出来又白烧 CPU。实测预览
  **5.0 fps、1.0~1.2 KB/帧**，两路合计约 12 KB/s
- 读管道跑在独立线程里不断排水，否则管道写满会把 ffmpeg 卡住，**录制跟着停**
- 子进程里用 PIL 解 JPEG，再 `imshow`；缩略图按原始比例显示（不拉伸）
- 实测对实时性**没有可测影响**（三组对照的数字见 [`../DOC/待验证清单.md`](../DOC/待验证清单.md) F8）

也可以写进配置文件，省得每次敲：

```yaml
view: true
view_fps: 20
view_video_fps: 5
view_video_width: 360
```

`view` 改成 `true` 后默认开窗口，临时不想看就加 `--no-view`。

## 5. 停止

正常停止只按一次 `Ctrl+C`，等待程序停手套、停相机、回填视频时间戳、写 manifest。
正常 2~3 秒完成，最坏给 30 秒（相机卡住时每路最多等 6 秒）。

**清理超时或被再按一次，都会先落盘再退出**（2026-11-17 起）：

| 触发 | 动作 | 拿到什么 |
|---|---|---|
| 30 s 看门狗到点 | 抢写入锁 → `emergency_save()` → 退出 130 | `data.h5` 完整（含未满一批的余数），**没有 manifest**，写标志可能未清 |
| 第二次 `Ctrl+C` | 同上，最多等 5 秒 | 同上 |
| 主线程正卡在 HDF5 里（抢不到锁） | 不强写（并发写会弄坏文件），直接退出 | 数据到最后一个已 flush 的批 |

抢不到锁是刻意的：`emergency_save` 用带超时的 `Lock`（**不能是 `RLock`**，原因见
[`agent/pitfalls.md`](../../agent/pitfalls.md) 里 Ctrl+C 重入那段），拿不到说明主线程正在
HDF5 里做 resize/flush，这时候从另一个线程插进去写会让文件结构损坏。宁可少几行。

退出码 130 之后，`data.h5` 可能带"写打开"标志，读之前先
`python ../Paxini-Align/align_capture.py <目录> --clear-write-flag`。

## 6. 参数与配置

### 6.1 配置文件

`Cap_Quest/cap_config.yaml` **存在就自动加载**，命令行参数优先于文件：

```yaml
camera_mode: both        # both / cam0 / cam1 / none
hand_rate: 120           # 连续采集请求频率 Hz
view: false              # 采集时开实时窗口
```

- 键名就是长参数名，`-` 写成 `_`（`camera_mode`、`omit_distribution`）。
- **注释是真的注释**：`#` 之后到行尾都不参与解析。字段级的代价与坑写在文件里的行内
  注释中，改配置前先看，不要只照抄参数表。
- 换一份：`python3.10 capture/collect_cap.py --config /path/to/other.yaml`。**旧的 `.json`
  也能直接用**——YAML 是 JSON 的超集，解析器不看扩展名。
- 未知键、非法选项值、布尔写成 `"yes"` 都会直接报错退出（退出码 2），不静默忽略。
- 改完想确认最终生效值：`--help` 里显示的 `(default: ...)` 就是配置文件改写后的默认值。

### 6.2 参数表

下表覆盖 `cap_config.yaml` 里的全部 21 个字段（键名把 `-` 换成 `_`），另列了一批只在命令行
用的参数——那些是一次性的，不写进配置文件。字段的取值含义、代价与坑写在该文件的行内注释里。

| 参数 | 默认 | 说明 |
|---|---|---|
| `--hand-port` | `auto` | 手套串口；auto 按 USB `28e9:018a` 发现 |
| `--hand-rate` | `120` | 连续采集目标频率，范围 1..300 |
| `--list-devices` | — | 打印串口/视频节点/ffmpeg 版本后退出 |
| `--cap-side` | `right` | 单只手套手别，决定相机命名和 Quest 默认手别 |
| `--pose-source` | `controller` | `controller` / `hand` / `head`，只影响 `pose_6d` |
| `--pose-side` | `auto` | `auto` 跟随 `--cap-side`；`left` / `right` 可显式覆盖 |
| `--quest-rate` | `120` | Quest 轮询频率 |
| `--camera-mode` | `both` | `both` / `cam0` / `cam1` / `none` |
| `--no-cameras` | 关 | 等价 `--camera-mode none` |
| `--camera-0` / `--camera-1` | `auto` | 指定 `/dev/videoN`；auto 按 sysfs 设备路径排序 |
| `--camera-width` / `--camera-height` | `1280` / `720` | 请求分辨率 |
| `--camera-fps` | `30` | 请求帧率 |
| `--camera-start-timeout` | `15` | 每路相机的启动宽限；宽限后 4 s 无新帧即中止整场 |
| `--video-codec` | `copy` | `copy`（MJPEG 直通）/`h264`/`hevc`/`mjpeg` |
| `--duration` | `0` | 采集秒数，0 表示到 Ctrl+C |
| `--output-dir` | — | 默认 `<Paxini-Cap>/Recordings/data_<日期>_<序号>` |
| `--omit-distribution` | 关 | 不写 444 点分布力 |
| `--view` / `--no-view` | `false` | 采集时开实时视图；配置文件可设 `view: true` |
| `--view-mode` | `auto` | `auto` = 有显示会话用 `tk`、没有则用 `html`；`html` = 本地页面 `http://127.0.0.1:8765/`（fork 子进程，端口由子进程绑定后回报）；`tk` = matplotlib 独立子进程窗口 |
| `--view-fps` | `20` | 实时窗口刷新率（`tk` 模式） |
| `--view-video-fps` | `5` | 相机缩略图的抽帧帧率（`--view` 打开时生效） |
| `--view-video-width` | `360` | 相机缩略图宽度，高度按比例 |
| `--no-quest` | 关 | 不采 Quest，跳过 PC Service/ADB 与 pose 等待 |
| `--skip-adb` | 关 | 不装/不启动 Quest APK |
| `--allow-usb-only` | 关 | 兼容旧参数；有 USB ADB 时本来就保留 USB |
| `--apk` / `--package` / `--adb` / `--serial` / `--quest-ip` / `--adb-port` | — | Quest 连接参数 |
| `--start-pc-service` / `--pc-service-script` / `--no-pc-service-check` | — | PC Service 管理 |
| `--config` | `cap_config.yaml` | 配置文件路径（YAML；旧的 JSON 也能解析） |

ffmpeg/ffprobe 从 `PATH` 找，也认环境变量 `FFMPEG`。只是要 5.0 以上——apt 的
22.04 版是 4.4.2，没有 `-fps_mode`，相机那一路会在启动时就失败。装法见
[`../README.md` §1.1](../README.md)。

### 6.3 相机：只有 V4L2

一条路径：ffmpeg 直接读 `/dev/videoN`，加 `-use_wallclock_as_timestamps 1`，容器里的
pts 本身就是墙钟纳秒，不需要事后锚定。设备写 `/dev/video0`，停止用 SIGINT。

**延迟 Δ 必须在本机、本参数下实测。** 任何旧的、别人机器上的数值都不能沿用 —— 拿错了
时间轴会整体平移一百多毫秒，而行数、间隔、单调性全都正常，看不出错。

测法：

```bash
python3.10 latency/measure_camera_latency.py      # 默认逐路测 cam0 与 cam1
```

结果自动写进 `latency/records/<电脑用户>-ubuntu.yaml`，下游读最近一次合格值，见 §6.4。

### 6.4 测量结果台账

测量脚本跑完都会往 `latency/records/<电脑用户>-ubuntu.yaml` 追加一条，
**一台机器一个文件**，键是测量序号：

```yaml
machine: ifnot-ubuntu
platform: ubuntu
backend: linux
measurements:
  1:
    at: '2026-11-20T20:00:00+08:00'
    camera: 0
    median_ms: 42.7         # 修正量：时间轴减掉这么多
    iqr_ms: 3.1
    samples: 240
    ok: true                # 样本 ≥ 30、IQR ≤ 40 ms、漂移 ≤ 10 ms，直接用
```

下游读法：

```python
import sys
sys.path.insert(0, "<repo>/Paxini-Cap/Cap_Quest/latency")
from latency_records import load_records, pick_latest

entry = pick_latest(load_records(), camera=0)   # 本机最近一次合格的 cam0
median = entry["median_ms"]
```

- `pick_latest` 默认跳过 `ok: false` 的记录；两路相机延迟不同，`camera` 必须指明。
- **默认逐路测 cam0 和 cam1**，不用加参数。第 3 路起（笔记本自带的
  `USB2.0 HD UVC WebCam`）不参与采集，脚本直接跳过；只测一路用 `--camera 0` / `1`。
- 换分辨率、帧率、codec、USB 口都要重测，脚本会把参数一并记下，`delta_prev_ms`
  给出与上一条同参数记录的差值，超过 20 ms 会当场提醒。
- 每个 `latency_measure/<时间戳>_*/` 目录仍是本次的完整产物（逐帧 CSV、直方图、
  录制的 mkv），台账里 `output_dir` 指回去；这些产物不入库，`output_dir` 只在本机有效。

## 7. 常见问题

```bash
ps -eo pid,ppid,stat,etime,cmd | grep "[c]ollect_cap.py"
v4l2-ctl --list-devices
ls -l /dev/serial/by-id/ /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

| 现象 | 原因与处理 |
|---|---|
| `未发现可用的串口` | 线缆松动、供电不足、不在 `dialout` 组；`als -l /dev/ttyACM*` 先确认节点本身在 |
| `检测到多个串口` | 有别的 CDC 设备；用 `--hand-port` 指定 |
| 相机节点为 0 | 内核没识别到 UVC；`v4l2-ctl --list-devices` 与 `dmesg | tail` 对照 |
| `等待 PXCap camX 摄像头画面超时` | 分辨率/帧率不被支持，先降到 640x480 / 15 fps |
| `时间戳不是墙钟时间` | ffmpeg 太老或命令被改；确认 `-use_wallclock_as_timestamps 1` |
| 一直"尚未收到 PXCap 连续采集帧" | 串口占用、回调异常；看 manifest 的 `snapshot_errors` |
| 相机 `actual_fps` 明显低于 30 | USB 带宽不够（两路 720p30 挂在同一个 Hub 上很常见）；换端口或降分辨率 |
| 采集频率低于目标 | 看 `hand.callback_max_ms` 与 `queue_overflows`；降 `--hand-rate` |

## 8. 已知限制

- 只支持一只手套、一个串口、一个 `PxCap3` handle。
- 采集端不做对齐，也不做跨源质量判断；采完当场看不出"相机和手套是否真的覆盖同一
  时间窗"，只有一条 5 秒的粗检告警。
- 采集中途断电或强杀进程，`manifest.json` 会缺失。**数据只到最后一个满批**（每 100 行
  一批，批后即 flush，2026-11-17 起）；文件此时带"写打开"标志，直接只读打不开，要先
  `python ../Paxini-Align/align_capture.py <目录> --clear-write-flag`。2026-11-13 那批
  老采集没有这个保护，被强杀的 8 份只剩 48 字节超级块，全废。
- Matroska 时间基是 1 ms，视频时间戳量化误差约 ±0.5 ms。
- mkv 里存的是绝对 epoch 时间戳，个别播放器会因时间轴起点过大显示异常。
- 主机墙钟一旦被校时，跨跳变的数据就不可信；采集端的对策是**原样落盘、不修正**。
- Quest 的空间外参没有标定，`pose_6d` 只是头显 tracking-origin 下的位姿。
- **相机与手套都没有内参/外参**：单目算不出深度，4 路关节角也变不出指尖三维位置。
  相机标定**已决定不做**（E8），不是待补的缺口；需要那类结论时先启封
  [`../DOC/标定.md`](../DOC/标定.md) §2。

## 9. 与 `Dex_Quest` 的差异

| 项 | Dex（执行器） | Cap（手套） |
|---|---|---|
| 数据源 | `pxdex3` 轮询/回调，有电机 | `pxcap3` 连续采集回调，无电机 |
| 相机实现 | OpenCV `VideoCapture` 读帧打点 | ffmpeg 直采 V4L2，pts 即墙钟 |
| 输出 | `aligned.jsonl` + `hand_pose.csv` + 逐帧 JPG | 单文件 `data.h5` |
| **对齐** | **采集时做完最近邻对齐** | **完全不做，交给下游工程** |
| 字段 | 四关节 + 电流 + 温度 + 444 点 | 四路编码器 + 关节角 + 10 路合力 + 444 点 |

另有三处必要调整：

- `--hand-rate` 默认 `120` 而不是 `30`：Dex 的 30 Hz 受其关节编码器 78 Hz 限制。
- `--camera-jpeg-quality` 换成 `--video-codec`：我们录 mkv，不写 JPG。
- 新增 `--no-quest`：手套常单独用，没有头显时也要能采。

## 10. 验证状态

### 10.1 已执行

| 项 | 结果 |
|---|---|
| `py_compile` / `pyflakes` | 全过，0 告警 |
| `selftest.py` | 150 项，通过 149（在原有覆盖之外增加 C8 时钟探针、sdk_unix 报告与文件质量门控、Quest 独立时间轴/sequence 降级、失败工件、三源运行期检查 8 项）。唯一失败项是「启动失败工件」，本机缺 `ffprobe` 导致，装好 ffmpeg 的机器上应全过 |
| `--list-devices` | 正常输出串口 / V4L2 节点 / ffmpeg |
| 无设备时报错 | 退出码 **2**，保留 `startup_failure.json`，不创建 `data.h5` |
| 相机延迟测量 | **两路都尚未实测**；任何旧路径的数都已经作废，不能沿用 |
| 假 SDK 驱动真实主循环（python3.10，5 秒） | 602 行、**120.0 Hz**、`verify` 为空、`alignment: none` |
| 回调耗时（60 秒合成帧） | 均值 0.21 ms、最大 0.62 ms |
| 文件体积（合成数据） | 3.6 KB/行 ≈ 1.57 GB/小时（120 Hz，444 点全量） |
| ffmpeg 链路（lavfi 合成源） | 首帧 pts 1794388000，间隔 33/34 ms，SIGINT 后 ffprobe 能读全帧 |
| ffmpeg 7.0.2 + V4L2 真采（2026-11-20，Ubuntu 22.04，笔记本 UVC 720p30 `copy`，4 s） | 120 帧 / 1280×720 / 30 fps，pts 是墙钟；单次 SIGINT **0.4 s** 收尾，容器完整（ffprobe 无 `File ended prematurely`），日志无 warning。开预览的第二路输出同样验过（4 s 出 18 帧 JPEG ≈ 5 fps）。**前提是命令里没有 `-nostdin`** |
| `visualize.py`（合成 data.h5，7200 帧 / 60 s） | 窗口与 `--html` 都能出图；HTML 1.45 MB；实测回放 1.01× 实时；`--save-png` 首帧正常 |
| `live_view.py`（合成 1200 帧 @120 Hz） | 子进程正常起停；父进程 `push` 均值 **130 µs**、p99 262 µs；240 帧队列零丢弃；`close` 30 ms |
| Quest 左手真机联调（2026-11-18） | XRoboToolKit `controller.left` 连续有效；PXCap 1200 帧 / 120.0 Hz、Quest 2400 样本、两路相机各 245 / 300 帧 / 15 fps；37 个数据集行数自检通过 |
| **真机（手套 + 双相机 + 回放 + 实时窗口）** | 手套 115.4~118.3 Hz（其中一次 20 s 内 34 处帧间隔空洞，疑 USB 通路，B8 待复测）、两路相机恒 30.0 fps、37 个数据集零不一致；`copy` / `h264` 双档均不丢帧；关 NTP 后时钟零跳变。回放交互、视频对齐（误差 ≤ 半帧）、实时窗口与相机关键帧全部通过。**逐项实测与取证在 [`../DOC/待验证清单.md`](../DOC/待验证清单.md) A / B / D / F 组**，这里不再重复罗列 |

### 10.2 未执行

- **Quest 跨源时间域**：XRoboToolKit 真机采集已跑通，但 C8「`sdk_timestamp_ns`
  属于哪个时钟」仍未确认，因此 Quest 与手套/相机的绝对时间对齐仍不能成立。
- **相机标定不做**（不是"没验证"，也不是"待实现"）：2026-11-17 决定划出范围，下游扩散
  模型只吃图像、不算位姿，四组量（内参/双相机外参/手套→相机/Quest→相机）都没有使用场景，
  见 [`../DOC/待验证清单.md`](../DOC/待验证清单.md) E8。代价是指尖三维位置、深度、
  跨源空间对齐都不成立，汇报里不许写。
- **相机延迟 Δ 尚未实测**：`measure_camera_latency.py` 只过了编译、`--list` 与合成
  视频验证（设定 42.7 ms 还原 42.7 ms）。旧采集路径的数值已作废，不能拿来用。
- **手套零点标定的写操作没有真机记录**：`hand_calib.py` 写完但只在离线下验证（`selftest`
  7 项）。真机只跑过只读的 `info`（2026-11-16 21:02，SN `DCG04R48500093`）：4 路编码器
  0.011 / 0.253 / 0.137 / 0.352°，10 路合力最大 0.2 N，故障 0 项——**零点本身是准的**。
  `encoder` / `sensor` / `magnet` 三个写操作仍未执行；`info` 只读单帧给不出噪声底，
  要复核阈值得跑 `--dry-run`——判据见 [`../DOC/待验证清单.md`](../DOC/待验证清单.md) B9。

其余待测项与判据在 [`../DOC/待验证清单.md`](../DOC/待验证清单.md)。
