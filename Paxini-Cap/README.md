# Paxini-Cap

PXCap III 数据手套 + 双路相机 + Quest 3 的同步采集工程，对应
`Paxini-Dex`（PXDex 执行器）那一套。区别是这里没有电机、不产生任何运动，
输出是单个 HDF5。

```text
Paxini-Cap/
├── Cap_Quest/          # 主程序（对应 Dex_Quest）；内部结构与命令见其 README
│   ├── capture/            # 采集主程序与设备驱动层
│   ├── calib/              # 手套零点标定（hand/ 放标零报告，不入库）
│   ├── latency/            # 延迟测量：两个脚本 + records/ 台账（一台机器一个文件）
│   ├── viewer/             # 回放与实时视图
│   ├── diagnostics/        # Quest 时间戳时钟域探针（不写 data.h5）
│   ├── selftest/           # 不接硬件的自检（项数以运行输出为准）
│   └── cap_config.yaml     # 可选配置（存在即加载，命令行优先）
├── Recordings/         # 采集输出（data_<日期>_<序号>/），采集端与 Paxini-Align 共用
├── Paxini-Align/       # 下游：手套 + 可选 Quest 按相机帧时刻对齐（→ README）
│   ├── align_capture.py    # 主程序：读 data.h5，输出 aligned_<cam>.h5
│   └── selftest.py         # 合成数据自检 24 项，不接硬件
├── DOC/                # 文档（格式说明、时钟说明、待验证清单、标定）
│   ├── README.md           # 文档索引 + 阅读顺序 + 维护约定
│   ├── HDF5结构说明.md     # 逐字段说明 + 完整目录树（data.h5 的唯一权威文档）
│   ├── 时间戳说明.md       # 三个来源的时钟域、换算公式、三个坑（先读这份）
│   ├── 待验证清单.md       # 真机/Quest 待验证项 + 已决定不做的理由 + 标定（G 组）
│   ├── 标定.md             # 两类标定：手套零点（已实现）+ 相机标定（未实现）
│   └── figures/            # 时间戳图解（SVG，供 HDF5结构说明 / 时间戳说明 引用）
├── py_demo/            # 独立小脚本，不参与主流程，用来确认「连得上、采得到」
│   ├── test.py             # 最小连通性：SDK 版本、SN、故障、一次读数
│   ├── stream_demo.py      # 连续采集，打印帧率与帧间隔分布
│   └── camera.py           # 单路相机录 N 秒，ffprobe 回读；复用 camera_ffmpeg.py
├── sdk/                # deb 包 + SHA256SUMS + 安装说明（二进制不入库）
│   ├── xrobotoolkit/       # Quest 端 APK
├── tools/              # 数据侧工具（不参与采集）
│   └── upload_modelscope.py   # 打包 data_* 并上传 ModelScope 数据集仓
├── requirements.txt
└── README.md
```

## 0. 接手必读

**当前状态**：真机（手套 + 双路相机 + Quest XRoboToolKit）已跑通；Quest 的 SDK
时间戳已确认是**主机墙钟**，所以下游**默认就对 Quest 做对齐**（没有书面证据，报告里
标 `clock_domain_verified=false`；要让时钟域有实证仍可跑 C8 走 `sdk_unix`）。
相机标定**已决定不做**（E8，下游扩散模型只吃图像、不解算位姿）；标定不做，任何
"距离 / 位置 / 三维"的结论都不成立。逐项状态见 §6 与
[`DOC/待验证清单.md`](DOC/待验证清单.md)。
`selftest.py` 只覆盖纯逻辑（帧解析、行轴、配置、视图挂钩、分布力散点/示意回退）；
**下游配对已有代码**：`Paxini-Align/` 把手套流按相机帧时刻重采样，Quest 取目标时刻
之前最近的一条整行复制（见 §5）。

**数据现状（2026-11-20 核对）**：`Recordings/` 里有 4 份，逐份字段见
[`Recordings/README.md`](Recordings/README.md)。三份 `data_20261117_{1,2,3}` 是从 Release
样例包里补回来的，`data_20261118_1` 是本地现采。**四份的 `manifest.quest_rows` 全是 0、
`quest.enabled` 全是 false**，即仓库里没有任何一份带 Quest 行的采集，C8 与 Quest 对齐
在当前工作区无从复查（默认对齐遇到这种文件会自动跳过，不会报错）；四份的
`env.platform` 也都是 WSL2，**没有一份来自原生 Ubuntu/V4L2 路径**，那条路只有代码，
没有留存的产物。

历史上的 13 份采集（2026-11-17 核对，原件已不在本工作区）里只有 4 份可用
（`data_20261113_2 / _5 / _11`、`data_20261116_1`）；另外 8 份是采集中途被强杀的
文件，写的时候 `file.flush()` 还只在 `close()` 里调，强杀时 HDF5 元数据树从未落盘，
`data.h5` 只剩 **48 字节超级块**，整份作废。这个坑 2026-11-17 已修（每批落盘，见
`agent/pitfalls.md`），修好之后强杀最多丢最后不满 100 行的那一批。`data_20261113_1`
能打开但没录相机。

> 同名的坑还有一处：`DOC/待验证清单.md` C8 与 A12 引用的 `data_20261118_1` 是 Quest
> 左手、640×480@15、`left_cam0` 245 帧那份，与现存这份同名不同内容。那两条判定当前
> **没有原始文件可复查**。代码与文档里其余已知错处见 [`agent/known-issues.md`](../agent/known-issues.md)，
> 动相关代码前先看那份。

### 0.1 不能违反的约定

1. **采集端不做对齐。** 三条行轴 `cap/`(N) / `quest/`(Q) / `cameras/*`(M) 互相独立、
   行数永不相等，谁都不能当索引去切另一条。配对属于下游 `Paxini-Align/`。曾经有过
   一版对齐代码，已整体删除，别加回来。
2. **只记 SDK 给的时间戳。** `cap/` 的 unix + monotonic 由 pxhandsdk 帧结构自带；
   `quest/` 的四个由 `quest3.collect_sample` 取。不要补主机侧读数。
3. **分布力的 119 是坑位数，不是点数。** 每槽位按 `distribute_counts` 读，
   该值等于真正写进数组的点数。
4. **回调线程只拷贝。** `snapshot_cap_frame` 跑在 SDK 线程里，只做 numpy 拷贝入队；
   写盘全部在主线程 `pump_and_record`。
5. **`quest3.py` 已与 `Dex_Quest/quest3.py` 分叉。** Cap 侧另有无线 ADB 配对/发现、
   `pc_service_command()` 等逻辑，Dex 侧没有；不要再追求两份一致。
6. **`DOC/HDF5结构说明.md` 被用户手动编辑过**，只能定点修改，禁止整份重写。
7. **改 `cap/` 或 `quest/` 的布局必须升 `format_version`**（当前 `4`），并在
   `HDF5结构说明.md` 的版本变更表里写清改了什么。下游按版本号判断怎么读。
8. **相机只有 V4L2 一条路。** 不要把相机经 USB/IP 转发给虚拟机或容器：UVC 的等时包
   会被丢，720p 从 30 fps 掉到 5 fps；同理也不能让两路 720p30 挤在同一个 Hub 上。
   要在 Windows 上采集只能回 `archive/wsl-backend` 分支，不要往 main 上加回现场。

### 0.2 代码地图

| 想改什么 | 改哪里 |
|---|---|
| 增删 HDF5 字段 | `collect_cap.py` 的 `CAP_DATASETS` / `_quest_layout()`，再同步文档和 selftest |
| 加命令行参数 | `build_parser()` + `cap_config.yaml` + `Cap_Quest/README.md` 参数表 |
| 回放窗口 / HTML | `visualize.py`（`collect_payload` 读+抽稀，`run_window`/`write_viewer` 两个出口） |
| 采集时实时视图 | `live_view.py`：`HtmlLiveView`（fork 子进程 + 队列，无显示会话时用）／`LiveView`（fork 子进程的 Tk 窗口，默认） |
| 视频采集与 pts 回填 | `camera_ffmpeg.py` |
| Quest 连接与样本读取 | `quest3.py` |
| 采集主循环节拍 | `collect_cap.py` 的 `pump_and_record` |
| 手套零点标定 / 静态磁检测 | `hand_calib.py`（闸门在 `check_*_gate`，判据见 `DOC/标定.md` §1） |

数据写入是批量的：`Hdf5Recorder.write_cap` 攒满 `FLUSH_ROWS = 100`（120 Hz 下约
0.83 s）才 `resize` + 批量赋值，**每批结束 `file.flush()`**（2026-11-17 起；之前只在
`close()` 里 flush，强杀会连元数据一起丢）。所以采集中途读 `data.h5` 会看到一跳一跳的
数据，实时视图因此只读内存、不读文件。

### 0.3 环境坑（都踩过）

- **`sudo` 要密码**，本机没配免密；批量命令用 `echo <pw> | sudo -S`，或先配免密。
- **matplotlib 只认 `Noto Sans CJK JP`**，没有 Mono 版。等宽中文会整片变方框，
  表格类布局别用 `family="monospace"`。
- **Write 工具产出 CRLF**，仓库其余文件都是 LF，提交前统一一次。
- 没有 `python3.10-tk` 时开不了窗口，用 `visualize.py --html`。
- **ffmpeg 7 加 `-nostdin` 会吞掉第一次 SIGINT**，相机容器收不了尾；相机命令里不要
  再加这个参数（`Popen` 已经给 `stdin=DEVNULL`），详见 [`agent/pitfalls.md`](../agent/pitfalls.md)。
- `opencv-python` 需要系统的 `libGL.so.1`（`apt install libgl1`），缺了 `import cv2`
  直接报错；测延迟才会用到。

### 0.4 改完必须过的三关

```bash
cd Cap_Quest
source ../.venv/bin/activate             # 在 Paxini-Cap/.venv 里跑
python3.10 -m py_compile capture/*.py viewer/*.py selftest/*.py latency/*.py calib/*.py diagnostics/*.py
python3.10 -m pyflakes capture/*.py viewer/*.py selftest/*.py latency/*.py calib/*.py diagnostics/*.py
python3.10 selftest/selftest.py          # 应为全过
```

- 改数据结构或行为，**必须加 selftest 用例**，不能只靠手跑
- 改字段，三处同步：`DOC/HDF5结构说明.md`、`Cap_Quest/README.md`、本文件
- 临时脚本用 `_tmp_` 前缀，任务结束删掉
- 用户文档与提交信息用中文；代码注释和 docstring 用英文（与 Dex 保持一致）

## 1. 环境要求

| 项 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04 / 24.04（原生安装） |
| Python | **`python3.10`**。`python3` 在装了 conda 的机器上可能指向 3.12/3.14，此时 `import pxhandsdk` 能过但 `hasattr(pxhandsdk,"PxCap3")` 为 False；一律显式写 `python3.10` |
| 手套 SDK | `pxhandsdk 1.0.0.0`，`dpkg -i` 装到 `/usr/lib/python3/pxhandsdk` |
| Python 包 | `numpy>=1.24` / `h5py>=3.8` / `pyyaml>=6` / `matplotlib>=3.7` / `xrobotoolkit-sdk`，装在 `Paxini-Cap/.venv` 里（建法见下） |
| 延迟测量额外包 | `opencv-python` / `qrcode[pil]`（生成并解码二维码，只有 `latency/` 用；已在 `requirements.txt` 里） |
| 回放 | `matplotlib`（随 requirements 装）；窗口模式还要 `python3.10-tk`，`--html` 不需要 |
| 系统工具 | `v4l-utils`（`v4l2-ctl`，排查相机）、`libgl1`（`opencv-python` 依赖）、`adb`（Quest） |
| ffmpeg | **5.0 以上**，需带 `v4l2` 输入、`-fps_mode`、`ffprobe`。apt 的 22.04 版是 4.4.2，**没有 `-fps_mode`，跑采集会在相机启动那一步失败**，必须换静态版，装法见 §1.1 |

### 1.0 Python 环境：`Paxini-Cap/.venv`（只做一次）

```bash
# 从仓库根目录执行
cd Paxini-Cap
# 用系统解释器建，避免 conda/旧 venv 的 python3.10 抢先
/usr/bin/python3.10 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install --ignore-installed -r requirements.txt
python3.10 -c "import pxhandsdk; print('PxCap3:', hasattr(pxhandsdk,'PxCap3'))"   # 必须 True
```

- **必须 `--system-site-packages`**：`pxhandsdk` 是 deb 装到
  `/usr/lib/python3/pxhandsdk/dist-packages`，纯 venv / conda 都看不到它，手套会起不来。
- **必须 `--ignore-installed`**：否则 pip 会复用 `~/.local` 里的 numpy，回放 matplotlib
  又会被其他工程带崩（`_ARRAY_API not found`）。
- 之后每个新终端先 `source Paxini-Cap/.venv/bin/activate`；`Cap_Quest/`、`Paxini-Align/`、
  `py_demo/` 共用这一个环境。`Paxini-Dex/.venv` 是另一套，别混用。
- `PxCap3` 不是 `True` 时不用重建，软链一次：
  `ln -sfn /usr/lib/python3/pxhandsdk/dist-packages/pxhandsdk .venv/lib/python3.10/site-packages/`。

### 1.1 ffmpeg：换静态版（必做一次）

`-fps_mode` 是 5.0 才有的选项，`camera_ffmpeg.py` 的相机命令里硬用着它。apt 源里
22.04 最高只有 4.4.2，所以必须换静态版。下面这套在 Ubuntu 22.04 上实测通过：

```bash
cd /tmp
wget -q -O ffmpeg-static.tar.xz \
  https://johnvansickle.com/ffmpeg/releases/ffmpeg-release-amd64-static.tar.xz
mkdir -p ff_static_x && tar -xf ffmpeg-static.tar.xz -C ff_static_x

sudo mkdir -p /opt/ffmpeg-7.0.2
sudo cp ff_static_x/ffmpeg-*-amd64-static/{ffmpeg,ffprobe} /opt/ffmpeg-7.0.2/
sudo chmod 755 /opt/ffmpeg-7.0.2/ffmpeg /opt/ffmpeg-7.0.2/ffprobe
# /usr/local/bin 在 /usr/bin 之前，装在这里就不用每次 export FFMPEG
sudo ln -sf /opt/ffmpeg-7.0.2/ffmpeg  /usr/local/bin/ffmpeg
sudo ln -sf /opt/ffmpeg-7.0.2/ffprobe /usr/local/bin/ffprobe

hash -r && ffmpeg -version | head -1     # 应为 7.x-static
ffmpeg -hide_banner -devices | grep v4l2          # 有 v4l2 输入
ffmpeg -hide_banner -h full | grep -c fps_mode    # 非 0
```

三条都要过。只用 apt 的 4.4.2 也能启动 ffmpeg，但 `-fps_mode` 会直接报
`Unrecognized option 'fps_mode'`，表现为相机一路起不来、`data.h5` 里没有
`cameras/`，很容易误判成 USB 带宽问题。

机器上已有别的软件自带的 5.0+ 静态版时，`wget` 这一步可以省掉：把它的 `ffmpeg` 和
`ffprobe` **两个一起**复制到 `/opt/ffmpeg-7.0.2/`，再建同样的软链。两个必须同版本，
用 `ffmpeg -version` / `ffprobe -version` 核对。

装完由 `PATH` 生效；也可以不改系统目录，改用 `--ffmpeg /opt/ffmpeg-7.0.2/ffmpeg` 或
`export FFMPEG=...` 指给单个程序（采集、回放、延迟测量三个入口都认）。

**不要用 `-nostdin`。** ffmpeg 7 带 `-nostdin` 时会吞掉第一次 SIGINT，容器收不了尾
（详见 [`agent/pitfalls.md`](../agent/pitfalls.md)）；代码里已经去掉，靠
`stdin=DEVNULL` 达到同样效果。

### 1.2 运行 XRoboToolKit 的前置条件

Quest 数据链路不是只装一个 Python 包就能工作。开始采集前必须同时满足以下条件：

| 项 | 要求 |
|---|---|
| Quest 3 | 已开启开发者模式和 USB 调试；首次连接使用支持数据传输的 USB 线，并在头显内允许 RSA 调试授权 |
| ADB | 主机已安装 Android platform-tools，`adb devices -l` 能执行；`adb` 不在 `PATH` 时用 `--adb` 指定 |
| PC Service | 已安装 `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb`，`RoboticsServiceProcess` 常驻运行 |
| Python SDK | `xrobotoolkit-sdk` 必须装在采集程序使用的同一个 `python3.10` 环境中 |
| Quest APK | APK 位于 `sdk/xrobotoolkit/XRoboToolkit-Quest-1.0.1.apk`，包名为 `com.xrobotoolkit.client.quest`；二进制未入库，下载与校验见 [`sdk/README.md`](sdk/README.md) |
| 网络 | Quest 与运行 PC Service 的主机位于可互通的局域网；无线 ADB 端口由 mDNS 发现（旧式 `adb tcpip 5555` 仅作兼容） |
| 头显设置 | 进入 XRoboToolKit 后保持 APK 运行，并开启 **Tracking** 和 **Send**；手柄需要唤醒。**采集期间必须戴上并解锁头显**，否则 APK 暂停、Quest 数据全零，10 s 后会中止 |
| 无线 ADB（可选） | 头显里打开 **Wireless debugging**；采集加 `--quest-wireless`，配对码按提示输入，端口由 mDNS 发现，失败回退 USB |

PC Service 不会开机自启。两种起法，二选一：

**A. 手动起**（任意 shell 都行；脚本没有 shebang，bash 会兜底）：

```bash
cd /opt/apps/roboticsservice
bash ./runService.sh
ps -eo pid,cmd | grep '[R]oboticsServiceProcess'
```

**B. 让采集程序代起**：在采集命令里加 `--start-pc-service`，它会用 `bash` 跑同一个脚本
（完整命令见 [`Cap_Quest/QUICKSTART.md`](Cap_Quest/QUICKSTART.md)）。

`runService.sh` 自己设置动态库、Qt 插件和 QML 路径，并以后台进程启动
`RoboticsServiceProcess`。不要只运行 `RoboticsServiceProcess`，也不要运行 `run3D.sh`
（没有显示环境时会失败）。

**这个脚本没有 shebang**（首行是 `DIR=$(...)`，且用了 bash 的 `BASH_SOURCE`），所以不能
被 `execve` 直接执行：shell 里 `./runService.sh` 能跑是因为 bash 自己兜底，而 Python 的
`subprocess.Popen([script])` 会报 `Exec format error`。`--start-pc-service` 因此固定用
`bash` 调用它（`quest3.pc_service_command()`），别再改回直接 exec。

PC Service 监听 `*:63901` 和 `127.0.0.1:60061`；
Quest 跟踪数据通过局域网进入前者，所以头显和主机必须在同一个可达网段，主机防火墙
要放行 63901。

安装并确认 Python SDK 可用（装进 `Paxini-Cap/.venv`，见 §1.0；在仓库根目录执行）：

```bash
cd Paxini-Cap
source .venv/bin/activate
pip install xrobotoolkit-sdk
python3.10 -c "import xrobotoolkit_sdk; print('xrobotoolkit_sdk OK')"
```

首次连接 Quest 时保持 USB 线连接，然后可先用一个短任务验证整条链路：

```bash
cd Paxini-Cap/Cap_Quest
source ../.venv/bin/activate
python3.10 capture/quest3.py --duration 10
```

正常结果应包含 XRoboToolkit Python SDK 初始化成功、Quest 无线 ADB 已连接，以及
`已通过 PC Service 收到 Quest 跟踪数据`。若只准备 APK 和 ADB、不读取数据，可加
`--prepare-only`；若不想传 USB 线，也可直接使用 `--quest-ip <Quest 的局域网 IPv4>`
连接已开启无线 ADB 的 Quest。看到 SDK 已初始化但迟迟没有有效 pose 时，依次检查
PC Service、Quest Wi-Fi、APK 的 Tracking/Send 开关和手柄唤醒状态。

## 2. 硬件接入

**所有设备都直接接在 Ubuntu 主机上**：两路相机走 UVC（`/dev/video*`），手套走 USB CDC
串口（`/dev/ttyACM*`）。这是唯一一条采集路径；2026-11-20 之前的 Windows 采集实现冻结在
`archive/wsl-backend` 分支，只在必须回到那套硬件时才 checkout。

- 手套按 **USB VID:PID `28e9:018a`** 识别，不依赖 `/dev/ttyACM0` 编号（重插会变）。
  程序同时扫 `ttyACM*`、`ttyUSB*` 和 `/dev/serial/by-id`。
- 相机按 `/sys/class/video4linux/video*` 的 `index == 0` 节点发现（UVC 还会多一个
  metadata 节点），按 `device` 的 realpath 去重、排序后映射 cam0/cam1。两路同型号相机
  的节点序号可能互换，要求稳定就显式写 `--camera-0 /dev/video2 --camera-1 /dev/video0`。
- 两路 UVC 同时采 720p30 约 20 MB/s，是整条链路最容易出问题的地方：先确认它们不要
  挂在同一个 USB 3.0 Hub 上（2026-11-18 遇到过整个 Hub 在取流后被复位），再用
  `--camera-fps 15` 或 640×480 做首轮验证。
- 串口没权限是 `dialout` 组的问题，装完 SDK 跑一次
  `sudo /usr/share/pxhandsdk/scripts/setup-serial-access.sh` 并重新登录。

## 3. 安装（只做一次）

```bash
# 在仓库根目录执行
cd Paxini-Cap

# 系统依赖
sudo apt install -y python3.10-tk v4l-utils libgl1

# 手套 SDK + 串口权限
sudo dpkg -i sdk/pxhandsdk_1.0.0.0.ubantu2204_amd64.deb
sudo /usr/share/pxhandsdk/scripts/setup-serial-access.sh
sudo usermod -aG dialout "$USER"
# 重新登录让 dialout 生效

# Python 依赖（含延迟测量用的 opencv / qrcode）——先按 §1.0 建好并使用 venv
source .venv/bin/activate          # 已在 Paxini-Cap/ 下
pip install --ignore-installed -r requirements.txt

# ffmpeg 5.0+（apt 的 4.4.2 不够），见 §1.1
```

装完先跑 §0.4 的三关：`selftest` 全过说明环境没问题，此时不需要插硬件。

**验证设备与日常运行不在本文档**：`--list-devices`、采集、`--view`、回放、
参数表全在 [`Cap_Quest/README.md` §1](Cap_Quest/README.md)。采前关 NTP、
确认 ffmpeg 是 5.0 以上、没有头显加 `--no-quest` 这三条纪律，在该文件的
【快速开始】与【§2 时间戳】里各有完整说明，这里不再重列。

## 4. 验证状态

**结论：采集链路可用，跨源分析不可用。**

真机（手套 + 双路相机）2026-11-13 已跑通，逐帧无丢帧、关 NTP 后时钟零跳变；
2026-11-18 Quest XRoboToolKit 左手联合采集也已跑通。不接硬件时 `selftest`
全过。但下面三处边界仍决定哪些结论成立：

| 缺口 | 状态 | 后果 |
|---|---|---|
| Quest 时间域 | 已确认是主机墙钟（**无书面证据**，C8 仍未跑） | 下游默认按 Unix 对齐，summary 标 `clock_domain_verified=false`；要实证就跑 C8 走 `sdk_unix` |
| 相机标定 | **已划出范围**（E8） | 下游扩散模型只吃图像、不算位姿，因此不做。代价是不做任何"距离 / 位置 / 三维"结论 |
| 手套零点标定 | 代码已有，无真机记录 | 关节角零点可能是偏的，默认阈值待 `info` 噪声底复核 |

逐项实测值、判据、以及 7 条"决定不做"的理由（E 组）全在
[`DOC/待验证清单.md`](DOC/待验证清单.md)。使用侧的明细表见
[`Cap_Quest/README.md` §10](Cap_Quest/README.md)。

## 5. 后续

1. **`Paxini-Align/`（已实现，2026-11-17）**：相机时间戳为采样网格，减去实测 Δ 后
   把手套流线性插值到该时刻，一行一个训练样本；输出 `aligned_<cam>.h5`，含取图用的
   `frame_index` 与插值跨度质量列。用法与字段见 [`Paxini-Align/README.md`](Paxini-Align/README.md)。
   Quest **默认参与对齐**：`--quest-align unix` 把 SDK 戳直接当主机墙钟，取目标时刻
   之前最近的一条整行复制，滞后逐行记在 `quest/age_ms`；没有 Quest 数据的采集自动跳过。
   要时钟域实证就跑 C8 再走 `sdk_unix`。C8 解决时钟域，**不等于 Quest 采样链路延迟
   已标定**。两路相机不同步，cam1 取前驱会有 0~一帧的滞后，要严格同帧就各跑一次。
2. **相机延迟 Δ 现场实测**（不是标定，别混）：改分辨率、帧率、codec、USB 口、机器之后
   重跑 `latency/`，下游从台账取最近一次合格值。扩散模型对观测-动作错帧比人眼敏感得多，
   便宜但漏了要重采。
3. **可视化**：`Cap_Quest/visualize.py` 已经能回放——关节角、10 路合力数值表、
   共享时间轴、空格播放/逐帧/点曲线跳转/滑块/倍速，浏览器版还有悬停读数、保峰抽稀、
   时钟漂移统计、空负载与跳变警告、**双路相机视频面板（按时间轴精确同步）**；
   采集时 `--view` 实时出 4 路关节角与 10 路合力表（纯数字，不画曲线）加**相机关键帧**。
   还缺两块：
   - Quest 轨迹叠加（要 C 组 + Quest 外参，即 E8 启封后的 G4）
   - 时间质量**曲线图**：现在只有数值（漂移 + >10 ms 跳变次数），画成
     `unix − monotonic` 偏移曲线更直观
