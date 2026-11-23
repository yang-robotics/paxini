# agent/

给 AI 助手用的项目认知文档。目标不是"读完变专家"，是**动手前 3 分钟内知道该改哪个
文件、不能碰什么、改完怎么验**。

人类读者也欢迎，但正式说明在 [`../README.md`](../README.md) 和
[`../Paxini-Cap/README.md`](../Paxini-Cap/README.md)，那两份讲得更完整。

## 这套文档解决什么

AI 在新会话里最容易犯的四个错，各有一份对应：

| 文件 | 治什么 |
|---|---|
| [`code-map.md`](code-map.md) | 不知道改哪个文件，凭文件名瞎猜 |
| [`constraints.md`](constraints.md) | 不知道哪些"优化"会破坏下游，好心办坏事 |
| [`playbook.md`](playbook.md) | 知道要改什么，但步骤顺序错，跑一半卡住 |
| [`pitfalls.md`](pitfalls.md) | 重复踩已经踩过的坑，浪费一整轮 |
| [`known-issues.md`](known-issues.md) | 在已经错了的代码/文档上继续盖东西（当前 1 条 P2，无 P0/P1） |

## 项目 30 秒

`paxini` 是 PXCap III 数据手套与 PXDex III 灵巧手的开发工作区，两条独立的线：

```text
Paxini-Cap/   只记录、不运动
              PXCap III 手套(串口 28e9:018a, 120Hz)
            + 2× UVC 相机(1280x720@30)
            + Quest 3(ADB + XRoboToolkit, 120Hz 轮询)
            → 单个 data.h5，三条独立行轴，各自时间戳，不做对齐

Paxini-Dex/   会运动
              PXDex III 灵巧手 + 导纳控制 + Dex_Quest 采集
              输出 aligned.jsonl + hand_pose.csv + 逐帧 JPG（采集时就对齐）
```

两条线的代码骨架同源：`Cap_Quest` 对应 `Dex_Quest`。但 `quest3.py` 已经分叉，
差异在 `collect_sample` 和模块 docstring，**不要为了"一致"去同步它们**。

## 现在什么能用

| 能力 | 状态 |
|---|---|
| Cap 采集（手套 + 双路相机） | 可用，2026-11-13 真机验证 |
| Cap 回放（Tk / HTML） | 可用；HTML 含独立时间轴的 Quest 3 面板 |
| Cap 实时视图（`--view`） | 可用 |
| 手套零点标定 | 代码有，**无真机记录** |
| 相机延迟测量 | 工具可用，**两路已在原生 V4L2 实测**（2026-11-21：cam0 84.7 ms / cam1 87.6 ms，台账 `latency/records/ifnot-ubuntu.yaml`）；改硬件或参数后要重测 |
| Quest 采集 | XRoboToolKit 真机已跑通；SDK 时间戳跨源时钟域未验证 |
| 相机内参/外参 | **不做**（2026-11-17 决定划出范围，下游扩散模型只吃图像、不算位姿，见 `DOC/待验证清单.md` E8） |
| 跨源对齐 | **手套已实现；Quest 可选**：`Paxini-Align/align_capture.py` 相机时刻为主轴，手套插值；Quest 只在通过 C8 报告且文件时间戳质量达标时取“之前最近”样本 |

一句话：**能采、能按 C8 门控做时序配对；不能算三维，也没有 Quest 采样链路延迟标定。**

## 文档权威矩阵

同一个问题往往有好几份文档写过，按这张表找**唯一权威**，别读三份再自己猜：

| 想知道 | 看这份 | 其他文档的角色 |
|---|---|---|
| `data.h5` 里某个字段是什么 | `Paxini-Cap/DOC/HDF5结构说明.md` | 其他都是摘要 |
| 某列时间戳是什么时钟、怎么换算 | `Paxini-Cap/DOC/时间戳说明.md` | 其他都是摘要 |
| 某项测过没有、实测值多少 | `Paxini-Cap/DOC/待验证清单.md` | README 只给结论 |
| 相机延迟该用哪个数 | `Paxini-Cap/Cap_Quest/latency/records/<机器>.yaml` | README 只给量级，脚本负责写入 |
| 手套零点标定要不要跑、跑过没有 | `Paxini-Cap/DOC/待验证清单.md` B9 | 报告在 `calib/hand/`（不入库，只在本机） |
| 相机标定为什么不做 | `Paxini-Cap/DOC/待验证清单.md` E8；设计留档 `DOC/标定.md` §2 | 别当任务排期 |
| 某个参数什么意思 | `Paxini-Cap/Cap_Quest/README.md` §6.2 | `--help` 也是权威 |
| 怎么开发、不能违反什么 | `Paxini-Cap/README.md` §0 | 本目录是补充 |
| 怎么跑起来 | `Paxini-Cap/Cap_Quest/README.md` §1 | — |

改代码前先查这张表，能省掉一半的翻找。

## 三个最常迷路的地方

**1. 以为有个"主程序"能一把跑完。**
没有。`capture/collect_cap.py` 只负责录制，回放在 `viewer/visualize.py`，标定在
`calib/hand_calib.py`，延迟测量在 `latency/`。四个入口互不调用。

**2. 以为采集时会对齐。**
不会，而且**这是刻意设计**，不是没做完。三条行轴行数永远不相等，配对属于下游。
历史上有过一版对齐代码，被整体删除了。

**3. 以为相机能经 USB/IP 转发。**
不能。UVC 的等时包经 USB/IP 转发会丢，720p 从 30 fps 掉到 5 fps。相机必须直连运行采集
的那台主机，不要转发进虚拟机或容器。
