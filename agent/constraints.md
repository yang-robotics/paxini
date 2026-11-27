# 硬约束

这些不是风格偏好，是**违反之后下游会静默出错**的规则。每条给了「为什么」和
「违反了会怎样」——后者通常是"不报错，但数据废了"，这才是最难查的。

`Paxini-Cap/README.md` §0.1 已列过一遍；这里补的是后果与自查方法。

## 数据契约

### 1. 采集端不做对齐

**规则**：三条行轴 `cap/`(N)、`quest/`(Q)、`cameras/*`(M) 各写各的，行数永不相等，
谁都不能当索引去切另一条。配对属于下游。

**为什么**：对齐策略（阈值、插值、丢帧处理）依赖具体任务，采集端替下游做了决定就
再也改不回来。

**违反了会怎样**：下游会拿到一份"看起来已经对齐"的数据，直接按行取数，整体错位
而没有任何报错。历史上有一版对齐代码，被整体删除，**别加回来**。

### 2. 设备/SDK 原值 + Quest 主机接收时刻

**规则**：`cap/` 的两个钟由 pxhandsdk 帧结构自带；`quest/` 的四个 SDK 时钟由
`quest3.collect_sample` 取；自 format 5 起另有 `quest/host_unix_ns` /
`host_monotonic_ns` 记 **PC 接收时刻**（对齐用，不冒充设备采样）。不要给手套或相机
补别的主机侧读数；主机接收时刻不能当设备采样时刻用。

**为什么**：设备的采样时刻只能由设备/SDK 给；PC 接收时刻是另一回事，只用于和
相机/手套的主机轴对齐，不能替代采样时刻。再给手套或相机补一条主机读数，就会让同一
来源出现两个不同语义的"时间"，下游无法判断该信哪个。

**违反了会怎样**：混入的读数看起来"更精确"，实际带着用户态调度延迟，且与原有列
不同源，会让时间质量统计失真。

### 3. `CAP_MAX_POINTS = 119` 是单槽位上限

**规则**：`119` 是**一个槽位**的最大点数；10 个槽位加起来才是 444。每个槽位按
`distribute_counts` 读，该值等于真正写进数组的点数。

**为什么**：SDK 的分布力是变长的，119 是硬件上限。

**违反了会怎样**：把 119 当总点数会写越界；把 444 当每槽位会分配 4 倍内存并读到
未初始化区域。

### 4. 回调线程只拷贝

**规则**：`snapshot_cap_frame` 跑在 SDK 回调线程里，只做 numpy 拷贝入队。写盘全部
在主线程 `pump_and_record`。

**为什么**：SDK 回调有硬时间预算（实测均值 0.21 ms、最大 0.62 ms），在里面做磁盘 IO
会阻塞后续帧。

**违反了会怎样**：串口回调被阻塞 → 丢帧 → `cap/` 行数变少且间隔出现空洞，`manifest`
里看不出原因，只能靠帧间隔分布发现。

### 5. 改 `cap/` 或 `quest/` 布局必须升 `format_version`

**规则**：当前 `5`（2026-11-27 起：`quest/host_*` 主机接收时刻）。改完在
`DOC/HDF5结构说明.md` 的版本变更表里写清改了什么。

**为什么**：下游按版本号决定怎么读。

**违反了会怎样**：旧下游读新文件，字段错位但形状对得上，读到的是垃圾数据却不报错。

### 6. `DOC/HDF5结构说明.md` 只能定点修改

**规则**：这份文档被用户手动编辑过，禁止整份重写。

**违反了会怎样**：手动整理过的措辞和结构会被覆盖掉，等于白做。

## 手套手别

**规则**：当前一次只采一只 PXCap。`cap_side` 选择 `left/right`，Quest 的
`pose_side` 默认是 `auto`，跟随手套手别；显式 `left/right` 可以覆盖这种联动。
手别只写入命名和 provenance，**不得镜像、翻转或改写手套原始字段**。

**违反了会怎样**：左手数据会被静默转换成另一套方向，而下游无法仅凭数值判断是否
发生过镜像。相机 ID 也必须继续跟随手别（例如 `left_cam0`）。

## 时钟

### 7. 不要跨系统时钟域做减法

**规则**：一个时间差里的两个时刻，必须在同一侧取。跨机取就都从同一台机器取。

**为什么**：两台机器的墙钟差**不是常数** —— 同一对机器，历史测到 1610 ms，另一次只有
0.3 ms，取决于两边 NTP 同步到什么程度。

**违反了会怎样**：整体偏移几毫秒到几秒不等，而且换一天再跑结果又变了，查不出规律。
自查：任何 `t_a - t_b` 的两边，确认它们是在同一台机器同一个运行时取的。

### 8. 采集前关 NTP

**规则**：`sudo timedatectl set-ntp false`，采完再开回来。

**为什么**：手套的墙钟列和 ffmpeg 的 pts 都会被校时 step 改写，实测采集期间出现过
2~5 秒的多次跳变。

**违反了会怎样**：跨跳变的数据不可信。采集端**不修正**，原样落盘，由下游判断 ——
所以下游拿到的是带跳变的数据，要么自己剔除，要么整段作废。

### 9. 相机延迟必须实测，不能用默认值

**规则**：改了分辨率、帧率、codec、USB 口、机器，都要重跑 `latency/` 下的程序。

**为什么**：那个偏移是"首帧拍摄 → mkv 首次变大"的链路延迟，与具体硬件和系统负载
绑定。同一台机器连测，整条链路会跳档，单次读数不能当常数。

**当前值**（2026-11-21，原生 V4L2，`latency/records/ifnot-ubuntu.yaml`）：cam0
**84.7 ms**、cam1 **87.6 ms**。下游取 cam0 那条给两路共用（差 2.9 ms，相对半帧忽略）。

**违反了会怎样**：时间轴整体偏几十到几百毫秒，画面和关节角错开好几帧，策略学到
"看到还没发生的动作"。

### 9a. Quest 可视化的时间轴必须标注来源

**规则**：查看器默认 `auto`：新采集按 `host_unix_ns` 主机接收时刻映射到手套 Unix 轴
（与下游 host 口径一致），老文件回退 `unix`。界面上必须区分三种来源：「已按主机接收
时刻对齐（无需 C8）」「已按 Unix 墙钟对齐（未经 C8 验证）」「已按 C8 验证的时钟对齐」，
不得让观众以为它们等价。`--quest-align none` 时保持独立时间轴，只允许归一化进度联动；
SDK 时间戳不可用时降级为 sequence。位置/四元数曲线必须跟随本次 `pose_side` 选择对应手柄。

**违反了会怎样**：观众会把并排布局误认为物理同步，后续分析和训练样本会按错误时刻取数。

### 9b. Quest 对齐有几条路，产物必须自证走了哪条

**规则**：`Paxini-Align` **默认对齐 Quest**（`--quest-align auto`）：新采集有
`quest/host_unix_ns` 时走 host（主机接收时刻，summary 标
`alignment=host`、`clock_domain_verified=false`——host 是实测主机接收轴，不是 C8
验证的 SDK 时钟域；不需要 C8）；老文件没有 host 列才回退 `unix`（把 SDK
时间戳直接当主机墙钟用，产物写 `summary.quest.clock_domain_verified=false`，界面标
「未经 C8 验证」）。要走 SDK 时钟域实证就显式 `--quest-align sdk_unix`，它要求通过的
`passed=true / clock_domain=unix_epoch` 报告，且当前 `data.h5` 的有效时间戳比例、倒退
和冻结重复达到报告阈值，否则不写文件。

相机帧是主轴，Quest 取目标时刻之前最近的一条**整行复制**（不插值）；超过
`--quest-max-age-ms` 时保留该行并写 `quest/valid=false`。**不做四元数插值**：姿态是
离散测量，nlerp 换来的半帧收益小于它掩盖的抖动。

**为什么**：数值接近 Unix 纳秒、sequence 连续都不等于时钟域已验证。host 是 PC 接收
时刻，不等于设备采样真值；unix 是约定；只有 sdk_unix（C8）验证了 SDK 四列的时钟域。
**必须让产物和界面说清当前是哪条路**，否则三个月后没人分得清哪份数据经过验证。

**违反了会怎样**：输出行数、类型和单调性都正常，但 Quest 与图像/手套静默错配，而且
事后无从判断这份对齐是否可信。另外，C8 只证明时钟域和换算模型，**不证明 Quest 采样
链路延迟已标定**。

## 代码结构

### 10. `quest3.py` 两边已分叉

**规则**：`Cap_Quest/capture/quest3.py` 与 `Dex_Quest/quest3.py` 已分叉：Cap 侧多了
无线 ADB 配对/发现（`pair_and_connect`、`parse_mdns_services`）和
`pc_service_command()`，Dex 侧没有。不要再追求两份一致。

**违反了会怎样**：为了"统一"改动其中一份，另一条线的采集行为被悄悄改变。

### 11. Dex 在采集时对齐，Cap 不对齐

**规则**：两边设计相反，不要互相"修正"。Dex 输出 `aligned.jsonl`（采集时最近邻对齐），
Cap 输出原始 `data.h5`。

**违反了会怎样**：拿 Cap 的"不对齐"去改 Dex，会破坏它已有的下游；拿 Dex 的对齐去改
Cap，违反约束 1。

## 运行期策略

### 12. 配置了就算必需：缺一路就中止整场

**规则**：手套始终必需；相机由 `--camera-mode` 决定、Quest 由 `--no-quest` 决定。
只要配置了，启动失败或运行期**来源停摆**（poller 不增长、valid 归零、相机/手套没有新帧）
都**中止整场**（`abort_recording()`），不降级成"少录一路照样录完"。具体判据：
`valid_samples` 启动 10 s 宽限后仍为 0，或出过有效数据后 5 s 不增长；poller 样本 5 s
不增长；相机/手套没有新帧。未配置的源不建对象、
不检查。**例外**：选定位姿的数值冻结不是来源停摆，只按 §12.1 warn，不中止。

**为什么**：训练样本要求手套 + 相机 + Quest 同时在场，缺一路的采集在训练侧等于整份
作废。让它安静录完，比让它当场失败更贵 —— 后者只损失几分钟。

**违反了会怎样**：有人为了"宽容一点"把某一路改回只告警，产物看起来完整、行数正常，
但那一列全是 NaN，事后只能整份重采。

### 12.1 按键是唯一的录制起点（2026-11-27）

**规则**：`data.h5` 只在「准备完毕 + 按任意键」之后创建；Quest 和手套的待机期样本
全部丢弃，相机保留按键前的预录（`manifest.cameras.*.pre_roll_s`，下游 Align 按公共
窗口丢）。就绪判据是**选定位姿**连续 `--quest-ready-samples` 条有效；位姿静止不撤回就绪
（静止是合法数据），不是 `quest/tracked`（它是头显/手柄/手部的或，手柄休眠时仍为 true），
也不是采样计数。录制期选定位姿 1 s 未变化只 warn、不中止——SDK 没有 per-controller
tracked 接口，逐位相同既可能是静止也可能是休眠，不能拿来中止整场。真正的中止判据是
poller 卡死、`valid_samples` 宽限期后仍为 0/出过有效后 5 s 不增长、相机/手套没有新帧（§12）。

**为什么**：Quest poller 比手套/相机先起，待机缓冲直接落盘会在 quest 组开头留一段
按键前的数据；起点由按键定义，待机样本全部丢弃。选定位姿的“静止 vs 休眠”无法可靠
区分，只保留 warn 和 `stale_max_s` 供事后判断。

**违反了会怎样**：quest 组开头多一段按键前的数据；或者把静止的手柄误判成丢失而中止
一场本来正常的采集。

### 13. 中止必须走 `abort_recording()`，不许直接 raise

**规则**：运行期故障用 `abort_recording(reason)` —— 它只写 `ABORT_REASON` + 置
`STOP_EVENT`，让主线程走正常收尾。

**为什么**：抛异常会跳过排空队列、停相机、回填视频时间戳、写 manifest，留下一个
"能打开但没法归因"的残件。

**违反了会怎样**：`data.h5` 里 `cameras/` 是空组、没有 `manifest.json`，你还得猜是
哪一路出的问题。

## 环境

### 14. 只能用 `python3.10`

**规则**：一律显式用 `python3.10`（venv 也用 3.10 建）。`python3` 在装了 conda 的机器上
可能指向 3.12/3.14，此时 `import pxhandsdk` 能过但 `hasattr(pxhandsdk, "PxCap3")` 为 `False`。

**违反了会怎样**：不是 ImportError，是后续 attribute 访问才炸，报错位置离真因很远。

### 15. Python 环境：`Paxini-Cap/.venv`（必须 `--system-site-packages`）

**规则**：在 `Paxini-Cap/` 下用**系统** Python 建 venv（写绝对路径 `/usr/bin/python3.10`，
否则 conda 或旧 venv 的 `python3.10` 会抢先，见下面「违反了会怎样」），依赖装进 venv：

```bash
cd Paxini-Cap
/usr/bin/python3.10 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install --ignore-installed -r requirements.txt
```

每个新终端先 `source Paxini-Cap/.venv/bin/activate`。`Paxini-Cap/` 下三个代码目录
（`Cap_Quest/`、`Paxini-Align/`、`py_demo/`）共用这一个环境；`Paxini-Dex/` 的 `.venv`
是另一套，不要混用。

**为什么**：`pxhandsdk` 随 deb 装到 `/usr/lib/python3/pxhandsdk/dist-packages`，只有
`--system-site-packages` 的 venv 才看得见（纯 venv、conda 都是 `PxCap3` 缺失）。
`--ignore-installed` 让 pip 把 numpy/h5py/matplotlib 装进 venv，而不是复用 `~/.local`
的共享副本——否则别的工程一升级 numpy，回放就会 `_ARRAY_API not found`。

**违反了会怎样**：
- 用成 conda / 旧 venv 的 `python3.10` 建：venv 的 `home` 会指向那个解释器，`pxhandsdk`
  仍然看不见；`cat Paxini-Cap/.venv/pyvenv.cfg` 里 `home` 必须是 `/usr/bin`。
- 漏了 `--system-site-packages`：`import pxhandsdk` 可能过、但 `PxCap3` 缺失，手套起不来。
  不用重建，软链一次即可：
  `ln -sfn /usr/lib/python3/pxhandsdk/dist-packages/pxhandsdk .venv/lib/python3.10/site-packages/`
- 漏了 `--ignore-installed`：numpy 仍来自 `~/.local`，回放随时被其他工程带崩。
- 混用 `Paxini-Dex/.venv`：Dex 的依赖集与 Cap 不同，装包会互相覆盖。

> 2026-11-26 修订：原规则“不要用 venv / ensurepip 会失败”来自 2026-11-16 的 WSL 时期，
> 在原生 Ubuntu 22.04 上已不成立（`python3.10-venv` 已装，venv 内 pip 22.0.2 正常）。

### 16. 相机只有 V4L2 一条路（2026-11-20 起）

**规则**：不能把 UVC 经 USB/IP 转发给虚拟机或容器——等时包会丢，720p 从 30 fps 掉到
5 fps。采集必须由直接插着相机的那台 Linux 主机自己做。

**违反了会怎样**：720p 从 30 fps 掉到 5 fps，驱动日志刷
`Dropping payload (out of sync)`。与曝光、分辨率、厂商软件都无关，调参数无效。

## 写作约定

- 用户文档与提交信息用**中文**，代码注释和 docstring 用**英文**（与 Dex 保持一致）。
- 临时脚本用 `_tmp_` 前缀，任务结束删掉。
