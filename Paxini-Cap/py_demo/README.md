# PXCap III Python SDK 快速上手

三个脚本，覆盖从"确认连得上"到"连续采集 + 相机录一段"的全过程。
手套没有电机，**没有运动控制脚本**，不可能把设备动坏。

| 文件 | 作用 | 依赖 |
|---|---|---|
| `test.py` | 最小连通性：连接、SDK 版本、固件、SN、故障、一次关节角与 10 路合力。单次读取，不启动采集 | `pxhandsdk` |
| `stream_demo.py` | 连续采集，每 0.5 秒打印帧数/关节角/拇指合力，结束给实际频率与帧间隔分布 | `pxhandsdk` |
| `camera.py` | 枚举 V4L2 节点，ffmpeg 录 N 秒，ffprobe 回读帧数/分辨率/实际 fps/帧间隔/首末帧墙钟 | `pxhandsdk` 之外的 ffmpeg/ffprobe |

`camera.py` 直接复用 `../Cap_Quest/capture/camera_ffmpeg.py`，和主程序用同一套采集逻辑，
避免两份代码对时间戳的处理不一致。

## 1. 前置条件

在 `Paxini-Cap/` 下先激活环境（建法见 [`../README.md` §1.0](../README.md)）：

```bash
cd Paxini-Cap
source .venv/bin/activate
python3.10 -c "import pxhandsdk; print(hasattr(pxhandsdk, 'PxCap3'))"   # 必须是 True
ls -l /dev/ttyACM*                                                       # 手套串口
command -v ffmpeg                                                       # 5.0 以上
```

系统默认 `python3` 是 3.12 时 `PxCap3` 加载不出来，一律用 `python3.10`。

## 2. 先只读

```bash
python3.10 test.py
```

预期看到 SDK 版本、主控固件、SN、故障计数、四路关节角和 10 路合力。串口不是
`/dev/ttyACM0` 就改 `test.py` 顶部的 `PORT`。

## 3. 再看连续采集

```bash
python3.10 stream_demo.py 5      # 采 5 秒
```

结束会打印实际频率和帧间隔的最小/平均/中位/p95/最大值。120 Hz 下帧间隔应集中在
8.3 ms 附近；p95 明显偏大说明回调被阻塞或串口不稳。

## 4. 再验证相机

```bash
python3.10 camera.py 3           # 每路录 3 秒
```

输出每路的帧数、分辨率、实际 fps、帧间隔和首末帧墙钟时间。首帧墙钟是
`1794388000.xxxxxx` 这种 epoch 秒——如果看到 0.x 或 1.x，说明
`-use_wallclock_as_timestamps` 没生效，主程序会直接报错而不是容忍。

## 5. 常见问题

| 现象 | 处理 |
|---|---|
| `No module named 'pxhandsdk'` / `PxCap3` 缺失 | 用了 3.12，或 venv 没带 `--system-site-packages`；按 [`../README.md` §1.0](../README.md) 建 venv（漏了可软链 `pxhandsdk`） |
| 连接失败 `code=101` | 不在 `dialout` 组；跑 `setup-serial-access.sh` 后重新登录 |
| 连接失败 `code=102` | 串口被占用；关掉 PXCap Studio 或旧进程 |
| 连续采集接口返回 `4000` | 采集进行中，除 stop/disconnect/is_connected 外都不能调 |
| `camera.py` 报找不到 ffmpeg | 静态版放 `~/bin`，并 `export PATH="$HOME/bin:$PATH"` |
