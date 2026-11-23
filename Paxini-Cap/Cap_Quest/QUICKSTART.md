# PXCap III + Quest 3 + 双路相机快速数采

只走 XRoboToolKit 路线。数据采集入口是 `capture/collect_cap.py`，可视化入口是
`viewer/visualize.py`。完整约定见 [`README.md`](README.md)。

## 快速流程

### 1. 一次性环境检查

```bash
# 从仓库根目录（包含 Paxini-Cap/ 的目录）执行
python3.10 -m pip install --user -r Paxini-Cap/requirements.txt
python3.10 -c "import pxhandsdk; print('PxCap3:', hasattr(pxhandsdk, 'PxCap3'))"
python3.10 -c "import xrobotoolkit_sdk; print('XRoboToolKit SDK OK')"
```

输出必须是 `PxCap3: True` 和 `XRoboToolKit SDK OK`。

### 2. 每个新终端确认 ffmpeg

从仓库根目录进入采集工程：

```bash
cd Paxini-Cap/Cap_Quest
command -v ffmpeg
ffmpeg -hide_banner -version | head -1
```

采集要求 ffmpeg 5.0 以上（要 `-fps_mode`）。apt 的 22.04 版是 4.4.2，不满足；换静态版
的步骤见 [`../README.md` §1.1](../README.md)，装好后 `/usr/local/bin/ffmpeg` 会盖在
`/usr/bin/ffmpeg` 前面，**不需要再设 `FFMPEG`**。项目仍按 `--ffmpeg` → `$FFMPEG` →
`PATH` 的顺序找。

### 3. 检查设备并采集

确认手套、两路 MASTER Camera、Quest 已连接，Quest 已开启 USB 调试且同局域网。
XRoboToolKit 中开启 **Tracking** 和 **Send**。

```bash
python3.10 capture/collect_cap.py --list-devices
```

确认能看到 PXCap、两个 MASTER Camera 和 ffmpeg 7 后，先关 NTP，再开始正式采集。

手套和相机的 Unix 时间戳都是墙钟，采集期间被校时 step 改写会让时间轴整体错位
（本机实测出现过 2~5 秒跳变，详见 [`README.md`](README.md) §2「墙钟跳变」）：

```bash
sudo timedatectl set-ntp false
```

```bash
python3.10 capture/collect_cap.py \
  --duration 60 \
  --apk ../sdk/xrobotoolkit/XRoboToolkit-Quest-1.0.1.apk \
  --start-pc-service
```

采 60 秒。需要人工结束时去掉 `--duration 60`，结束时只按一次 `Ctrl+C`。

看到下面两行才表示录制已经开始：

```text
[INFO] PXCap 连续采集已启动：120 Hz
开始采集：手套/相机/Quest 各自启动、互不等待，按 Ctrl+C 停止
```

三路不再串行等待，也没有就绪期：检查全部在采集循环里做（`RuntimeMonitor`）。看到这两行
之后日志里只会再出现「已出帧，等待 x.x 秒」，随后是每秒一条 `[采集]` 进度。**任何一路
断了都会中止整场**，日志末尾出现 `[错误] 采集中止：…`，manifest 里的 `runtime.abort_reason`
写的是同一个原因。

没有头显时**必须**显式加 `--no-quest`：不加就等于声明「这次要 Quest」，PC Service 没起、
APK 没开都会直接判启动失败，不会悄悄降级成只录手套和相机。

采完把 NTP 开回来：

```bash
sudo timedatectl set-ntp true
```

### 4. 采集后生成可视化

找到最新一份真正生成了 `data.h5` 的采集，并生成浏览器版回放：

```bash
RUN=$(find ../Recordings -maxdepth 2 -name data.h5 -printf '%T@ %h\n' \
  | sort -nr | head -1 | cut -d' ' -f2-)
python3.10 viewer/visualize.py "$RUN" --html
```

输出：

```text
../Recordings/data_YYYYMMDD_N/
├── viewer.html
└── preview/
    ├── left_cam0.mp4
    └── left_cam1.mp4
```

打开回放：

```bash
xdg-open "$RUN/viewer.html"
```

也可以直接在文件管理器中双击 `viewer.html`。空格播放/暂停，方向键逐帧，
点击曲线跳转，底部滑块拖动时间轴。

`viewer.html` 内嵌手套和 Quest 数据；视频在 `preview/` 中。分享回放时要把
`viewer.html` 和 `preview/` 一起复制。

#### Quest 3 可视化

默认自动检测 `data.h5` 中是否有 Quest 数据。有数据时，页面会增加：

- 本次采集手对应手柄的位置 XYZ 曲线和四元数 xyzw 曲线；
- 该手柄的 3D XYZ 轨迹，放在右侧；
- 头显、左右手柄当前位置；
- 当前 sequence、追踪状态、位姿、手柄输入和关节数量。

Quest 使用独立 SDK 时间轴。页面顶部和 Quest 面板都会显示：

```text
未做跨源时间对齐
```

手套、相机和 Quest 共用播放/暂停、倍速和全局进度，但 Quest 使用自身归一化进度；
这**不代表** Quest 样本与手套/相机发生在同一物理时刻。

默认未对齐模式使用 `0%~100%` 归一化横轴，让 Quest 曲线宽度和游标与手套图对齐，
但标签明确表示不是物理时间。C8 通过并使用 `sdk_unix` 后，才改为与手套相同的
相对 Unix 秒轴。

`pose_6d` 中的 xyz 是米制位置，xyzw 是姿态四元数；它不是完整手部骨架，也没有
手套/相机空间外参。

如果 `sdk_timestamp_ns` 部分为 `-1`，曲线在对应位置断开；如果整列都不可用，
Quest 横轴自动降级为轮询 `sequence`，页面会明确提示。

无 Quest 数据时，Quest 面板自动隐藏。已有 Quest 数据但不想显示时：

```bash
python3.10 viewer/visualize.py "$RUN" --html --no-quest
```

#### 仅 C8 验证后启用时间对齐

先运行时钟探针，不修改采集 `data.h5`：

```bash
python3.10 diagnostics/quest_clock_probe.py \
  --duration 60 \
  --start-pc-service
```

报告位于：

```text
clock_probe/run_YYYYMMDD_HHMMSS/report.json
```

只有报告显示 `passed=true` 和 `clock_domain=unix_epoch`，才显式启用：

```bash
python3.10 viewer/visualize.py "$RUN" \
  --html \
  --quest-align sdk_unix \
  --quest-clock-report clock_probe/run_YYYYMMDD_HHMMSS/report.json
```

对齐后 Quest 使用手套首帧 Unix 时间轴。默认仍是 `--quest-align none`；报告未通过时
查看器会拒绝 `sdk_unix`，不会静默猜测。

## 首次联调

第一次使用或更换 USB 端口后，先跑一次低负载 10 秒测试：

```bash
python3.10 capture/collect_cap.py \
  --duration 10 \
  --camera-mode both \
  --camera-width 640 \
  --camera-height 480 \
  --camera-fps 15 \
  --apk ../sdk/xrobotoolkit/XRoboToolkit-Quest-1.0.1.apk \
  --start-pc-service
```

测试成功后执行：

```bash
RUN=$(find ../Recordings -maxdepth 2 -name data.h5 -printf '%T@ %h\n' \
  | sort -nr | head -1 | cut -d' ' -f2-)
python3.10 viewer/visualize.py "$RUN" --html
```

## 左右手

当前 [`cap_config.yaml`](cap_config.yaml) 默认是：

```yaml
cap_side: left
pose_side: auto
```

即左手套并自动选择 `controller.left`。切到右手：

```bash
python3.10 capture/collect_cap.py --cap-side right
```

显式指定不同组合：

```bash
python3.10 capture/collect_cap.py --cap-side left --pose-side right
```

手别只影响 provenance 和相机命名。手套字段原样保存，不镜像、不翻转。

## 数据位置

采集输出写在 `<Paxini-Cap>/Recordings/`，不在 `Cap_Quest` 里面——采集端和
`Paxini-Align/` 读同一批目录。下面的路径相对 `Cap_Quest`：

```text
../Recordings/data_YYYYMMDD_N/
├── data.h5
├── manifest.json
└── cameras/
    ├── left_cam0.mkv
    ├── left_cam0.log
    ├── left_cam1.mkv
    └── left_cam1.log
```

`data.h5` 保存手套、Quest 和相机帧时间戳；视频像素在 MKV 中。

启动失败时目录中会出现 `startup_failure.json`，不会生成 `data.h5`。先查看其中的
`error`、`command_text` 和 `log_tail`。

## 合格检查

```bash
python3.10 - <<'PY'
import json
from pathlib import Path

run = sorted(Path("../Recordings").glob("data_*"), key=lambda p: p.stat().st_mtime)[-1]
m = json.loads((run / "manifest.json").read_text(encoding="utf-8"))
print("目录:", run)
print("手套:", m["rows"], "Quest:", m["quest_rows"])
print("手套频率:", m["actual_rate_hz"])
print("错误:", m["error"])
print("行数不一致:", m["verify"]["row_mismatch"])
for name, cam in m["cameras"]["per_camera"].items():
    print(name, cam.get("frames"), cam.get("actual_fps"), cam.get("error"))
PY
```

手套和 Quest 行数必须非零，两路相机必须有帧，`row_mismatch` 必须为空。

## 常见问题

| 现象 | 处理 |
|---|---|
| 找不到手套串口 | 检查 USB 线、`dialout` 组和 `/dev/serial/by-id/` |
| Quest 无有效 pose | 检查 PC Service、局域网、Tracking/Send 和手柄唤醒 |
| 相机不出帧 | 使用 ffmpeg 7；先加 `--camera-mode cam0` 排除带宽问题 |
| `VIDIOC_DQBUF: No such device` | USB/Hub 掉线；用 `sudo dmesg -wT` 检查 |
| 停止时 ffmpeg `-9` | 查看对应 `.log`；检查相机是否响应 SIGINT |
| 出现 `startup_failure.json` | 本次采集未启动；先看失败工件中的 ffmpeg 日志 |
