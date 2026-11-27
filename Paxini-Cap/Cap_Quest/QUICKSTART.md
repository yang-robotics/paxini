# PXCap III + Quest 3 + 双路相机快速数采

最短路径。安装细节、HDF5 字段、回放与故障排查见 [`README.md`](README.md)。

## 0. 一次性安装

```bash
cd Paxini-Cap/sdk
sudo dpkg -i pxhandsdk_*.deb
sudo /usr/share/pxhandsdk/scripts/setup-serial-access.sh   # 装完重新登录才进 dialout
sudo dpkg -i XRoboToolkit_PC_Service_*.deb
ls xrobotoolkit/XRoboToolkit-Quest-*.apk                   # 没有就从 Release 下载
```

```bash
cd /path/to/paxini
/usr/bin/python3.10 -m venv --system-site-packages Paxini-Cap/.venv
source Paxini-Cap/.venv/bin/activate
pip install --ignore-installed -r Paxini-Cap/requirements.txt
python3.10 -c "import pxhandsdk; print(hasattr(pxhandsdk, 'PxCap3'))"   # 必须是 True
```

ffmpeg 必须 5.0+（要 `-fps_mode`）：

```bash
ffmpeg -hide_banner -version | head -1
ffmpeg -hide_banner -h full | grep -c fps_mode          # 非 0
```

缺依赖、`PxCap3` 不是 True、或要装静态 ffmpeg，按 [`../sdk/README.md`](../sdk/README.md)
和 [`../README.md`](../README.md) §1.0/§1.1 处理。

## 1. 每个终端

```bash
cd Paxini-Cap/Cap_Quest
source ../.venv/bin/activate
python3.10 capture/collect_cap.py --list-devices
```

要看到：串口末尾是 `PXCap`、两路 V4L2 节点、ffmpeg 5.0+。缺哪样先看
[`README.md`](README.md) §1。

## 2. 首次连接 Quest 3S（只做一次）

1. 手机 Meta Horizon → 设备 → 头显设置 → 开发者模式；头显里再开 **Wireless debugging**。
2. USB-C 数据线连 PC，在头显里允许 USB 调试（可勾“始终允许”）。
3. `adb devices -l` 状态必须是 `device`。
4. `adb install -r ../sdk/xrobotoolkit/XRoboToolkit-Quest-*.apk`
5. 启动：`adb shell monkey -p com.xrobotoolkit.client.quest -c android.intent.category.LAUNCHER 1`
6. APK 里开 **Tracking** 和 **Send**；头显与 PC 同一 Wi-Fi。

不插 USB 线：头显先开 Wireless debugging，然后加 `--quest-wireless --start-pc-service`；
单独配对用 `python3.10 capture/quest3.py --pair`。更细的步骤见 [`README.md`](README.md) §1.1。

## 3. 采集

```bash
sudo timedatectl set-ntp false          # 采集期间不要被校时 step 打断时间轴

python3.10 capture/collect_cap.py \
  --duration 60 \
  --start-pc-service
```

终端打印「准备完毕」后**按任意一个键**开始；`data.h5` 只在按键后创建，相机在按键前就
在录（长度见 `manifest.cameras.*.pre_roll_s`，就绪后越早按越短）。`--duration` 从按键那刻算。

- 非 TTY（nohup/管道）用 `--no-wait`，就绪后自动开始。
- 不戴头显加 `--no-quest`；只录一路加 `--camera-mode cam0`；边采边看加 `--view`。
- 停止只按一次 `Ctrl+C`，等它写完 `manifest.json`；30 秒不退再按一次强制结束。
- 三路都配置后，任何一路断流都会中止整场，原因见 `manifest.runtime.abort_reason`。

采完开回 NTP：`sudo timedatectl set-ntp true`。

## 4. 检查

```bash
python3.10 - <<'PY'
import json
from pathlib import Path
m = sorted(Path("../Recordings").glob("data_*/manifest.json"),
           key=lambda p: p.stat().st_mtime)[-1]
d = json.loads(m.read_text(encoding="utf-8"))
print(m.parent, "手套", d["rows"], "Quest", d["quest_rows"],
      "有效", d["quest"].get("valid_rows"))
print("错误", d["error"], "行数不一致", d["verify"]["row_mismatch"])
for name, cam in d["cameras"]["per_camera"].items():
    print(name, cam.get("frames"), cam.get("actual_fps"), cam.get("error"))
PY
```

手套行数非零；采了 Quest 时 `quest_rows`/`valid_rows` 非零；两路相机有帧；
`row_mismatch` 为空。启动失败时诊断在输出目录父级的
`_startup_failures/<时间戳>_<原因>/startup_failure.json`（默认 `Recordings/`），
先看 `error`、`command_text`、`log_tail`。

format 5 host 列、相机 `duplicate_timestamps`、Align 去重和退出码等细节见
[`README.md`](README.md) §4.2。

## 5. 回放（可选）

```bash
RUN=$(find ../Recordings -maxdepth 2 -name data.h5 -printf '%T@ %h\n' \
  | sort -nr | head -1 | cut -d' ' -f2-)
python3.10 viewer/visualize.py "$RUN" --html
xdg-open "$RUN/viewer.html"
```

面板说明见 [`README.md`](README.md) §4.3；`xdg-open` 打不开就直接用浏览器打开该文件。

## 6. 常见问题

| 现象 | 处理 |
|---|---|
| 找不到串口 | 检查 USB 线、供电、`dialout` 组（改完重新登录） |
| `PxCap3` 不是 True | 不是 `python3.10`，或 venv 漏了 `--system-site-packages` |
| 相机 `fps_mode` 不认识 | ffmpeg < 5.0，装 5.0+ 或加 `--ffmpeg` |
| Quest 起不来 | 查 PC Service、USB 调试授权、局域网、Tracking/Send；不戴就 `--no-quest` |

更多见 [`README.md`](README.md) §7。
