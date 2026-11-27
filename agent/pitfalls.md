# 踩坑记录

按"你会看到什么"组织。每条给现象、根因、对策。`Paxini-Cap/README.md` §0.3 有
一份精简版，这里是完整版，含 2026-11-16 新踩的。

## 一、看起来像 bug，其实不是

### 采集时读 `data.h5`，数据一跳一跳的

**现象**：程序在跑，用另一个进程读文件，只看到几百毫秒才多一块数据。

**根因**：`Hdf5Recorder.write_cap` 攒满 `FLUSH_ROWS = 100` 行才 resize+批量赋值，
落盘粒度就是这一批。120 Hz 下约 0.83 s 一块。（2026-11-17 起每批都 `flush()`，
但批内仍是全有全无，所以"一跳一跳"照样成立。）

**对策**：别中途读。实时视图只读内存不读文件，就是这个原因。

### 手套帧率 115~118 Hz，不是标称的 120

**现象**：`--hand-rate 120`，`manifest` 里的实际频率总是差一点，偶尔出现 12~35 ms
的帧间隔空洞。

**根因**：USB 调度与主机负载，与相机无关（关掉两路相机单采手套仍是 116.7 Hz）。
手套和两路相机共用总线，中间还有 CH340 集线器。

**对策**：中位间隔 8.34 ms 就是 120 Hz，看中位数别看均值。要压空洞就在 Windows 里
关掉「USB 选择性暂停」和「允许计算机关闭此设备以节约电源」。

### Tk 回放窗口点了没反应

**状态**：Tk 回放的交互**没人在真窗口里点过**，只验过 `--save-png`。浏览器版是实测
过的。要交互就用 `--html`。

## 二、环境

### `cv2.namedWindow` 报 "The function is not implemented"

**根因**：装的是 `opencv-python-headless`。它和 `opencv-python` 装在同一环境时，
后装的那个覆盖 `cv2`，`getBuildInformation()` 里 `GUI: NONE`。

**对策**：

```bash
python -m pip uninstall -y opencv-python-headless
python -m pip install --force-reinstall --no-deps opencv-python
```

装完 `GUI:` 应显示 `WIN32UI`（Windows）或 `QT5`（Linux）。

### `import pxhandsdk` 成功，但 `PxCap3` 不存在

**根因**：用成 `python3`（可能是 conda 的 3.12/3.14）了。这些版本能导入包，但拿不到
cpython-310 编译的 C 扩展。

**对策**：一律 `python3.10`。自查：
`python3.10 -c "import pxhandsdk; print(hasattr(pxhandsdk,'PxCap3'))"` 应为 `True`。

### venv 里 `import pxhandsdk` 过了，但 `PxCap3` 缺失

**现象**：`hasattr(pxhandsdk,'PxCap3')` 是 `False`，或者直接
`ModuleNotFoundError: No module named 'pxhandsdk'`；手套连不上。

**根因**：`pxhandsdk` 是 deb 装到 `/usr/lib/python3/pxhandsdk/dist-packages` 的，只有
系统 Python 的 `pxhandsdk.pth` 认识它。建 venv 时没加 `--system-site-packages`，或者
用了 conda / 别的解释器，都看不到它。

**对策**：优先重建（`python3.10 -m venv --system-site-packages Paxini-Cap/.venv`）；
不想重建就把包装软链进去：

```bash
ln -sfn /usr/lib/python3/pxhandsdk/dist-packages/pxhandsdk \
        Paxini-Cap/.venv/lib/python3.10/site-packages/pxhandsdk
```

`xrobotoolkit_sdk` 不一样：它由 pip 装进 venv，wheel 自带 `libPXREARobotSDK.so`，**不需要软链**。

### `sudo` 要密码，但脚本里没法交互

**对策**：`echo <pw> | sudo -S <cmd>`，或者先配免密。

### 中文在 Windows 控制台显示成乱码或问号

**根因**：控制台代码页是 936(GBK)。Python 强制 `reconfigure(encoding="utf-8")`
会让 PowerShell 按 GBK 解码 UTF-8 字节，整屏乱码；反过来 `✓` `✗` 这类字符在 GBK
里没有，会变成 `?`。

**对策**：不要强改编码，保持控制台原编码只放宽 `errors="replace"`；输出只用 GBK
覆盖得到的字符（`[OK]` `[WARN]` 而不是 `✓` `✗`）。

### matplotlib 中文变方框

**根因**：本机只装了 `Noto Sans CJK JP`，没有 Mono 版。

**对策**：等宽中文会整片变方框，表格类布局别用 `family="monospace"`。

### 写文件工具产出 CRLF，仓库其余是 LF

**对策**：提交前统一一次。

### apt 的 ffmpeg 是 4.4.2，没有 `-fps_mode`

**现象**：相机那一路起不来，log 里是 `Unrecognized option 'fps_mode'`；
`data.h5` 里没有 `cameras/`，手套照常 120 Hz，很容易误判成 USB 带宽不够。

**根因**：`-fps_mode` 是 5.0 才加的，Ubuntu 22.04 的 apt 源最高 4.4.2。

**对策**：换静态版装在 `/usr/local/bin`（在 `/usr/bin` 之前），步骤见
`Paxini-Cap/README.md` §1.1。2026-11-20 在本机 Ubuntu 22.04 上装的是 7.0.2-static。

### ffmpeg 7 带 `-nostdin` 时吞掉第一次 SIGINT

**现象**：`Ctrl+C` 停采集，相机那两路要等满 `STOP_TIMEOUT_S`（4 s）后被 `kill()`，
manifest 里记 `ffmpeg 退出码 -9` 或"未在 4 秒内退出"，mkv 没有 trailer。

**根因**：ffmpeg 7 配 `-nostdin` 时信号只在 `stdin_interaction` 打开的分支里被处理，
第一次 SIGINT 只计数、不触发收尾；第二个 SIGINT/SIGTERM 才走"immediate exit"，
直接返回 255 但不写容器尾部。ffmpeg 4.4.2 没这个问题。

**实测**（Ubuntu 22.04 + 7.0.2-static + 720p UVC）：

| 命令 | 一次 SIGINT 后 |
|---|---|
| 7.0.2 + `-nostdin` | 15 s 仍存活，SIGKILL 收场 |
| 7.0.2 去掉 `-nostdin`（stdin=`/dev/null`） | 0.4 s 退出，rc=255，mkv 完整（120/120 帧） |
| 4.4.2 + `-nostdin` | 0.2 s 退出 |

**对策**：代码里已经删掉 `-nostdin`，靠 `Popen(stdin=DEVNULL)` 达到同样效果；
`stop()` 把 255 当成正常中断而不是失败。**别再把它加回去**，`selftest` 有反向断言。

**另一条**：ffmpeg 被中断时退出码是 **255**，不是 0。判"正常停止"必须把 255 算进去。

### `--start-pc-service` 报 `Exec format error`

**现象**：`PC Service 启动失败：[Errno 8] Exec format error:
'/opt/apps/roboticsservice/runService.sh'`。

**根因**：厂商 1.0.0 的 `runService.sh` 没有 shebang（首行是 `DIR=$(...)`），并且用了
bash 的 `BASH_SOURCE`。交互 shell 里 `./runService.sh` 能跑，是 bash 对 `ENOEXEC` 的
兜底；Python 的 `subprocess.Popen([script])` 不做兜底。

**对策**：代码已改成 `bash runService.sh`（`quest3.pc_service_command()`）。手动起也可用
`bash /opt/apps/roboticsservice/runService.sh`。**别改回直接 exec。**

## 三、会静默毁数据

### 时间轴整体偏几百毫秒，但没有任何报错

**根因**：没做相机延迟标定，或用的是别人机器上的默认值。

**对策**：跑 `latency/measure_camera_latency.py`（原生 V4L2 唯一路径），数从
`latency/records/` 里取最近一次的合格值
（`pick_latest(load_records(), camera=0)`），别抄文档里的数字。改了分辨率、帧率、
codec、USB 口、机器都要重测。

### 用了几个月前的 Δ，时间轴整体偏 100 ms

**根因**：同一台机器、同一套参数，读数也会整体跳一档。连测三次能出现
189.7 → 287.2 → 192.3 ms 这种差异，而三次 IQR 都只有 5~6 ms。变的不是抖动，是
驱动缓冲深度（那一次多了约 3 帧）。把某一次的读数当常量写死，隔几个月就对不上。

**识别**：和「偏几百毫秒」一样 —— 行数、间隔、单调性全正常，只有对齐错。

**对策**：下游现场读 `latency/records/<机器>.yaml` 取最近一次 `ok: true`；记录里的
`delta_prev_ms` 超过 20 ms 说明链路变过，别直接拿去修时间轴，先重测确认。

### 两台机器的时钟相减，结果每次都不一样

**根因**：两台机器的墙钟差**不是常数**。同一对机器，历史记录差 1610 ms，另一次只有
0.3 ms，取决于两边 NTP 同步到什么程度。

**对策**：一个差值里的两个时刻必须在同一侧取。看到任何 `t_a - t_b`，先确认两边同源；
必须跨机时，两个时刻都从同一台机器上取（比如都在主机侧），让系统差自动抵消。

### 采集中途校时，数据废了

**根因**：手套墙钟列和 ffmpeg pts 会被 NTP step 改写，实测采集中出现过 2~5 秒跳变。
采集端**不修正**，原样落盘。

**对策**：采前 `sudo timedatectl set-ntp false`。已经跳了的，对比 `timestamp_unix_ns`
与 `timestamp_monotonic_ns` 能看出来，数据要么剔除要么作废。

### 两路相机 + 手套同时工作时，整个 USB Hub 掉线

**现象**：ffmpeg 已写入几十帧，随后报 `ioctl(VIDIOC_DQBUF): No such device`；
`/dev/video*` 和 `/dev/ttyACM*` 一起消失。

**根因**：USB Hub 在设备取流后被复位。2026-11-18 实测两个 MASTER Camera 与 PXCap
共用的 `usb 3-1.3` Hub 整组 disconnect，几秒后重新枚举。这是供电/硬件拓扑问题，
不是分辨率不支持。

**对策**：相机和手套尽量直连电脑的不同 USB 控制器端口；必须使用 Hub 时选带独立
电源的 USB 3.0 Hub，并用 `sudo dmesg -wT` 确认没有再次整组掉线。

### stream-copy 的 `.progress` 没有 `frame=`，被误判为 0 帧

**现象**：MKV 已经在增长，但相机出帧检查显示 0 帧（2026-11-23 前叫「就绪检查」）。

**根因**：ffmpeg 7 的 stream-copy 可能只输出 `out_time_us` 和 `total_size`。
只解析 `frame` 会误判。

**对策**：用 `FfmpegCamera.progress_signal()`，依次兼容 `frame`、`out_time_us × fps`
和文件大小；不要恢复成只读 `frame`。

### 强杀进程 → 整份采集作废（2026-11-17 已修）

**根因**：`Hdf5Recorder` 每 100 行 `resize` 一次，但 `file.flush()` 原本**只在 `close()`
里调** —— HDF5 的元数据树（根组对象头、数据集 B 树）一直留在写缓存里，进程一死就没了。

**后果（修复前）**：`data.h5` 只剩 48 字节的超级块。表现为 `file is already open for
write`（写标志没清），清掉标志、重算校验和之后仍然报 `truncated file: eof = 48`。
**2026-11-13 那批 8 份被强杀的采集全部如此，0 份可救。**

**已修**：`flush_cap()` / `flush_quest()` 末尾加 `self.file.flush()`，每批都落盘。
实测代价 **0.07 ms / 100 行批**（占这批写入 10 ms 的 0.8%）。真实环境（采集中途）强杀对照：

| 版本 | 强杀后文件 | 清标志后 |
|---|---|---|
| 修复前 | **48 字节**（只有超级块） | 打不开，`eof = 48` |
| 修复后 | 797 697 字节 | **读出 500 行**（5 个满批；最后 50 行未满批的丢失） |

**恢复步骤**：强杀后的 `data.h5` 一定带写标志，直接用只读方式打不开。先
`python Paxini-Align/align_capture.py <目录> --clear-write-flag`（改 1 字节 + 重算超级块
校验和，等价 `h5clear -s`），再读。**只翻字节不重算校验和会把好文件弄坏**，报
`incorrect metadata checksum`。

**注意 flush ≠ fsync**：进程被杀能保住，整机断电仍可能丢 page cache 里未落盘的部分。
写盘策略接受这个风险，不做 fsync（每 0.83 s 一次 fsync 代价太高）。

**已修（第二层保护）**：30 s 看门狗到点和第二次 `Ctrl+C` 不再直接 `os._exit`，
而是先调 `Hdf5Recorder.emergency_save()`（带超时抢锁 → flush 余数 → resize 到精确行数 →
close），拿到文件后再退出 130。抢不到锁（主线程正卡在 HDF5 里）就放弃落盘，因为并发写
会把文件结构弄坏。

锁是**普通 `Lock` 不是 `RLock`**，这是刻意的：Python 的信号处理器跑在主线程上，如果
Ctrl+C 正好落在一次批量写中间，`RLock` 会让同一线程重入成功、在半截写入里再 `close()`
一次，把数据集 resize 成混合行数。普通 `Lock` 在这种情况下超时失败，宁可不落盘。
代价是 HDF5 的写操作全都套这把锁，且内部调用只能用 `_flush_*_locked` / `_close_locked`
这类"已持锁"版本，不能在锁内再调公共方法。

**对策**：Ctrl+C 只按一次，等它自己收尾（有 30 s 看门狗，超时也会落盘）。

### quest 组开头总多一段 `-1`/`tracked=false`

**现象**：`data_20261127_4` 前 371 行（3.09 s）`sdk_timestamp_ns=-1`，`_2` 前 958 行
（7.98 s）；`_1`/`_3` 整份都是。

**根因**：`QuestPoller` 在 ADB/SDK 之后立刻启动，`TimedStream` 是 `deque(maxlen=600)`；
采集循环第一次 `drain()` 把待机期样本全部写进 HDF5，最多 5 s。不是「采集结束后还在写」——
最后一条 Quest 换算后和最后一帧手套只差 0.09 ms。

**已修（2026-11-27）**：`StartGate` 在待机期持续排空并丢弃 Quest/手套缓冲，按键后才创建
`data.h5`；丢弃数量记 `manifest.quest.standby_samples_discarded`。
**不要**再用「poller 一启动就写」来省一个按键。

### quest 末尾一长段水平线，但 `tracked` 仍是 true

**现象**：`data_20261127_4` 从第 1935 行起，`quest/pose_6d` 的 7 个数逐位不变，持续
1670 行 / 13.9 s；同一时间头显 `head/pose_xyzw` 和左手一直更新。

**根因**：右手柄休眠/丢追踪，APK 继续把最后一帧 pose 发上来。`quest/tracked` 是
「头显 OR 手柄 OR 手部」的或，头显还在，所以整段都是 true；SDK 没有
`get_right_controller_is_active` 之类的接口，`pose_info.tracked` 只看四元数模长，
冻结的四元数模长仍是 1。**值本身也无法可靠区分**：静止的手和休眠的手柄都可能是逐位
相同，SDK 又没有 per-controller tracked 接口，所以不要拿它做硬中止判据。

**已修（2026-11-27）**：`SelectedPoseWatch` 看选定位姿的 7 个数，但只用于诊断：就绪不再
要求位姿变化，录制期 1 s 未变化只 warn、不中止（静止是合法数据；SDK 没有
per-controller tracked 接口）。manifest 记 `quest.selected_pose.stale_max_s` /
`stale_warnings`。

### Quest 时间戳整体比主机 Unix 快约 2.55 s

**现象**：`data_20261127_4` / `_2` 的 `quest/sdk_timestamp_ns` 与主机 Unix 的差恒定
**+2.55 / +2.56 s**，30 s 内不漂。老文件的 `--quest-align unix` 把 sdk 当主机墙钟，于是
Quest 整体放晚 2.55 s：开头 2.55 s 相机没有 Quest，末尾多出一截。

**根因**：SDK 给的是头显时钟，不是这台主机的墙钟；C8 从未跑过。这不是「多一段待机
数据」，起点闸门也修不了它。

**对策**：**format 5 起新采集自带 `quest/host_unix_ns`/`host_monotonic_ns`**，Align
默认 `--quest-align auto` 直接用主机接收时刻，不再有这个问题。老文件没有这两列，才跑
`diagnostics/quest_clock_probe.py`（重复判据按持续时长，120 Hz 轮询不再误杀）拿通过
的报告走 `--quest-align sdk_unix`。只看原始 `data.h5` 的 sdk 列，偏移永远在；要么接受，
要么走 C8。

## 四、重构相关的坑（2026-11-16 目录拆分后）

### 移动文件后 `--help` 仍通过，但某个功能炸了

**根因**：`collect_cap.py` 里的 `from live_view import ...` 是**函数内延迟导入**，
只有 `--view` 时才执行。`--help` 测不出来。

**对策**：改了目录结构，除了 `--help`，还要跑一遍会用到的功能路径和
`python3.10 selftest/selftest.py`（全过说明 import 链没问题）。

### 外面的脚本引用了里面的模块

**根因**：`Paxini-Cap/py_demo/camera.py` 通过 `sys.path.insert` 复用
`Cap_Quest/capture/camera_ffmpeg.py`。Cap_Quest 内部一拆目录，这种外部引用就断。

**对策**：改结构前先 `grep -rn "camera_ffmpeg\|Cap_Quest" --include=*.py ..`，
把仓内所有跨目录引用列出来。
