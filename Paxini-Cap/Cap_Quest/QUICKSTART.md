# PXCap III + Quest 3 + 双路相机快速数采

采集入口是 `capture/collect_cap.py`，输出 `<Paxini-Cap>/Recordings/data_<日期>_<序号>/`。
参数、HDF5 字段和回放细节见 [`README.md`](README.md)。

## 0. 前置（一次性）

Ubuntu 22.04 + `python3.10` + `adb`（`sudo apt install android-tools-adb`；24.04 上
`pxhandsdk` 的 deb 依赖声明会挡安装，先按 [`../sdk/README.md`](../sdk/README.md) §3 打
py310fix 包）。deb 和 APK 都不入库，先从 Release 下载到
`Paxini-Cap/sdk/`（校验见 [`../sdk/README.md`](../sdk/README.md)），再安装：

```bash
cd Paxini-Cap/sdk
sudo dpkg -i pxhandsdk_*.deb
sudo /usr/share/pxhandsdk/scripts/setup-serial-access.sh   # 装完重新登录才进 dialout
sudo dpkg -i XRoboToolkit_PC_Service_*.deb
ls xrobotoolkit/XRoboToolkit-Quest-*.apk                   # 代码默认从这里装 APK
```

APK 不在 `xrobotoolkit/` 时按 [`../sdk/README.md`](../sdk/README.md) §3 放过去，
或在采集命令里加 `--apk ../sdk/XRoboToolkit-Quest-*.apk`。

## 1. 建 `.venv`（一次性）

系统没有 `venv` 时先 `sudo apt install -y python3.10-venv`。

```bash
cd /path/to/paxini                # 仓库根目录（含 Paxini-Cap/）
# 用系统解释器建，避免 conda/旧 venv 的 python3.10 抢先
/usr/bin/python3.10 -m venv --system-site-packages Paxini-Cap/.venv
source Paxini-Cap/.venv/bin/activate
pip install --ignore-installed -r Paxini-Cap/requirements.txt
python3.10 -c "import pxhandsdk; print('PxCap3:', hasattr(pxhandsdk, 'PxCap3'))"   # 必须是 True
```

- `--system-site-packages` 不能省：`pxhandsdk` 由 deb 装到
  `/usr/lib/python3/pxhandsdk/dist-packages`，纯 venv / conda 看不到它，`PxCap3` 会是空的。
  漏了也**不用重建**，软链一次即可：
  ```bash
  ln -sfn /usr/lib/python3/pxhandsdk/dist-packages/pxhandsdk \
          Paxini-Cap/.venv/lib/python3.10/site-packages/pxhandsdk
  ```
- `--ignore-installed` 让 numpy/h5py/matplotlib 都装进 venv，不回落系统或 `~/.local`。
- `xrobotoolkit_sdk` 由 pip 装进 venv，wheel 自带 `libPXREARobotSDK.so`，**不需要软链**。
- 每个新终端先激活：仓库根下 `source Paxini-Cap/.venv/bin/activate`，
  `Cap_Quest/` 下是 `source ../.venv/bin/activate`。

## 2. ffmpeg 5.0+（一次性，已有可跳过）

相机命令用到 `-fps_mode`（ffmpeg 5.0 起）。apt 的 22.04 版是 4.4.2，不满足；装静态版：

```bash
cd /tmp
wget -c --show-progress -O ffmpeg.tar.xz \
  https://johnvansickle.com/ffmpeg/releases/ffmpeg-release-amd64-static.tar.xz
ls -lh ffmpeg.tar.xz                      # 约 40 MB，明显偏小说明没下完，重跑上一条
tar -tJf ffmpeg.tar.xz >/dev/null && echo "archive OK"
mkdir -p ffmpeg-static && tar -xJf ffmpeg.tar.xz -C ffmpeg-static --strip-components=1
sudo cp ffmpeg-static/ffmpeg ffmpeg-static/ffprobe /usr/local/bin/
sudo chmod 755 /usr/local/bin/ffmpeg /usr/local/bin/ffprobe
hash -r
ffmpeg -hide_banner -version | head -1              # 5.0 以上
ffmpeg -hide_banner -devices | grep -q v4l2 && echo "v4l2 OK"
ffmpeg -hide_banner -h full | grep -c fps_mode      # 非 0
```

`ffmpeg` 和 `ffprobe` 必须同版本、同目录（回填视频时间戳要用 ffprobe）。机器上如果已有
别的软件自带的 5.0+ 静态版，**不用重复下载**：把这两个文件一起复制到 `/usr/local/bin/`
（或软链）即可，但两个都要，且用 `ffmpeg -version` / `ffprobe -version` 确认版本一致。

不装系统目录也行：采集时加 `--ffmpeg /path/to/ffmpeg`，或 `export FFMPEG=...`；查找顺序是
`--ffmpeg` → `$FFMPEG` → `PATH`。

## 3. 每个新终端：激活并检查设备

```bash
cd Paxini-Cap/Cap_Quest
source ../.venv/bin/activate
python3.10 capture/collect_cap.py --list-devices
```

要看到：串口行末尾是 `PXCap`（`USB=28e9:018a`）、两路相机各有一个 V4L2 节点、
ffmpeg 5.0 以上。缺哪样先按 [`README.md`](README.md) §1 排查。

默认值来自 [`cap_config.yaml`](cap_config.yaml)（自动加载，命令行优先）：右手套、
双路相机 1280×720@30、Quest 120 Hz。

### 3.1 首次连接 Quest 3S（只做一次）

第一次配对时 ADB 授权只能人在头显里点；装 APK、启动这两步之后由
`collect_cap.py --start-pc-service` 自动完成。

1. **开开发者模式**：手机上的 Meta Horizon App → 设备 → 头显设置 → 开发者模式，按提示
   重启头显。没有这个入口时先去 <https://developers.meta.com> 完成开发者账号验证。
   还要在头显里打开 **Wireless debugging（无线调试）**（设置 → 系统 → 开发者选项），
   不插线只用无线时必须开。
2. **USB 调试授权**：用支持数据传输的 USB-C 线连 PC，戴着头显，在弹出的
   **允许 USB 调试** 对话框里点允许（可勾“始终允许”）。
3. **确认连接**（在 `Cap_Quest/` 下、venv 已激活）：
   ```bash
   adb devices -l        # 状态必须是 device；unauthorized = 头显里还没点允许
   ```
4. **装 APK**（包名 `com.xrobotoolkit.client.quest`）：
   ```bash
   adb install -r ../sdk/xrobotoolkit/XRoboToolkit-Quest-*.apk
   adb shell pm path com.xrobotoolkit.client.quest   # 有 package: 输出才算装上
   ```
5. **在头显里启动**：
   ```bash
   adb shell monkey -p com.xrobotoolkit.client.quest \
     -c android.intent.category.LAUNCHER 1
   ```
   头显里应出现 XRoboToolKit 界面；首次启动按屏幕提示允许权限。
6. **开数据发送**：APK 内开启 **Tracking** 和 **Send**、唤醒手柄；Quest 连上与 PC 相同的
   Wi-Fi 网段（PC Service 监听 `63901`，防火墙要放行）。

**不插 USB 线时**：头显必须先开 Wireless debugging 并保持唤醒（配对码只在那个界面显示时
才能被发现），然后

```bash
python3.10 capture/collect_cap.py --quest-wireless --duration 60 --start-pc-service
```

程序先复用已连接的无线 ADB，再用 mDNS 找已配对的连接，失败才提示输入头显上的 6 位配对码；
端口自动发现，找不到或配对失败会回退 USB。只想单独配对：
`python3.10 capture/quest3.py --pair`。

> 装 APK、启动 APK 由 `prepare_quest()` 完成（`--start-pc-service` 只负责启动 PC Service，
> 不是它装 APK）；配对成功后直接进第 4 节。

## 4. 采集

Quest 已开 USB 调试并授权、与电脑同一局域网；头显内 XRoboToolKit 开启 **Tracking** 和
**Send**。手套与两路相机直连主机（不要经 Hub 或 USB/IP 转发）。

```bash
sudo timedatectl set-ntp false          # 墙钟被校时 step 改写会让时间轴错位

python3.10 capture/collect_cap.py \
  --duration 60 \
  --start-pc-service
```

启动后程序先进**待机闸门**：Quest 轮询和手套连续采集先跑，等到选定位姿
`--quest-ready-samples` 条连续有效且最近 0.5 秒有变化、手套真实出帧，才启动相机并等每路
首帧；然后终端打印「准备完毕」。

- **按任意一个键开始采集**（不用回车）。`data.h5` 只在按键后创建，Quest/手套的待机样本
  全部丢弃；相机在按键前已经在录，mkv 里有 1–2.5 秒预录（`manifest.cameras.*.pre_roll_s`），
  下游 Align 会按公共窗口丢掉。
- 非 TTY（nohup/管道）不能按键：加 `--no-wait`，就绪后自动开始；否则启动前直接报错。
- `--duration` 从**按键那一刻**开始计时。
- `--start-pc-service`：PC Service 没在跑时用 `bash` 启动厂商的 `runService.sh`；已在跑则复用。
- 不戴头显：加 `--no-quest`（不要与 `--start-pc-service` 同用）。
- 只录一路相机：`--camera-mode cam0`；左手套：`--cap-side left`；边采边看：`--view`。
- 第一次或换 USB 口后，先用 `--duration 10 --camera-width 640 --camera-height 480 --camera-fps 15`
  跑一次低负载测试。
- 停止：只按一次 `Ctrl+C`，等它写完 `manifest.json`；30 秒未退出再按一次强制结束
  （退出码 130，`manifest.json` 不会写）。
- 三路都配置后，任何一路断流都会中止整场：日志出现 `[错误] 采集中止：…`，
  `manifest.runtime.abort_reason` 记同一个原因。
- **采 Quest 时必须戴上并解锁头显、让 XRoboToolKit 保持前台**；锁屏/摘下会让 APK 暂停，
  5 秒没有有效选定位姿就中止（`manifest.runtime.quest_pose_stale_limit_s`）。右手柄休眠时
  头显还在更新，`quest/tracked` 仍是 true，所以判据是**选定位姿的 7 个数不再变化**，不是
  那个或标志。

采完把 NTP 开回来：

```bash
sudo timedatectl set-ntp true
```

## 5. 检查产物

```text
Recordings/data_YYYYMMDD_N/
├── data.h5
├── manifest.json
└── cameras/
    ├── <side>_cam0.mkv / .log
    └── <side>_cam1.mkv / .log
```

```bash
python3.10 - <<'PY'
import json
from pathlib import Path

manifests = sorted(Path("../Recordings").glob("data_*/manifest.json"),
                   key=lambda p: p.stat().st_mtime)
m = json.loads(manifests[-1].read_text(encoding="utf-8"))
print("目录:", manifests[-1].parent)
print("手套:", m["rows"], "Quest:", m["quest_rows"], "有效 Quest:", m["quest"].get("valid_rows"), "频率:", m["actual_rate_hz"])
print("错误:", m["error"], "行数不一致:", m["verify"]["row_mismatch"])
for name, cam in m["cameras"]["per_camera"].items():
    print(name, cam.get("frames"), cam.get("actual_fps"), cam.get("error"))
PY
```

手套行数必须非零；采了 Quest 时 `quest_rows` 和 `quest.valid_rows` 都必须非零
（`valid_rows` 才是真正带 tracking 数据的行）；两路相机有帧；`row_mismatch` 为空。

启动失败时目录里是 `startup_failure.json`（没有 `data.h5`），先看 `error`、`command_text`
和 `log_tail`。退出码：`0` 正常，`2` 启动失败或中止，`130` 第二次 `Ctrl+C` 强制结束。

## 6. 回放（可选）

```bash
RUN=$(find ../Recordings -maxdepth 2 -name data.h5 -printf '%T@ %h\n' \
  | sort -nr | head -1 | cut -d' ' -f2-)
python3.10 viewer/visualize.py "$RUN" --html
xdg-open "$RUN/viewer.html"
```

Quest 面板、跨源对齐口径和视频面板见 [`README.md`](README.md) §4。

## 7. 常见问题

| 现象 | 处理 |
|---|---|
| 找不到手套串口 | 检查 USB 线、`dialout` 组（改完要重新登录）、`/dev/serial/by-id/` |
| 找不到 `adb` | `sudo apt install android-tools-adb`，或给 `collect_cap.py` 加 `--adb <路径>` |
| `adb devices` 显示 `unauthorized` | 头显里没点允许；重插数据线，勾选“始终允许” |
| `adb devices` 显示 `offline` | `adb kill-server && adb devices`，或换线/换 USB 口 |
| `adb install` 报 `INSTALL_FAILED_*` | 先 `adb uninstall com.xrobotoolkit.client.quest` 再装；确认 APK 完整、头显空间足够 |
| `monkey` 报 `No activities found` | APK 没装成功或包名不对：`adb shell pm path com.xrobotoolkit.client.quest` |
| `PxCap3` 不是 `True` | 不是 `python3.10`；或 venv 漏了 `--system-site-packages`（见第 1 步软链） |
| 相机日志有 `Unrecognized option 'fps_mode'` | ffmpeg < 5.0，装静态版或加 `--ffmpeg` |
| Quest 起不来 | 检查 PC Service、USB 调试授权、局域网、Tracking/Send；不戴就加 `--no-quest` |
| 停止时 ffmpeg `-9` | 看对应 `cameras/*.log`，确认相机能响应 SIGINT |
| 出现 `startup_failure.json` | 本次没有启动；按其中的 `error` / `log_tail` 排查 |
