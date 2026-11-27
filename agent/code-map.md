# 代码地图

回答两件事：**改这个功能动哪个文件**、**改完怎么证明没改坏**。

`Paxini-Cap/README.md` §0.2 有一份粗粒度的同名表格，这里补的是函数级定位和验证手段。

## Cap 侧：从任务到代码

| 任务 | 文件 | 具体位置 | 怎么验 |
|---|---|---|---|
| 增删 HDF5 数据集 | `capture/collect_cap.py` | `CAP_DATASETS`(:547) | `selftest` + 同步 `DOC/HDF5结构说明.md` |
| 改 `quest/` 的列 | `capture/collect_cap.py` | `_quest_layout()`(:643) | `selftest` 的 "Quest 行" 组 |
| 加命令行参数 | `capture/collect_cap.py` | `build_parser()`(:1344) | `--help` 对照 + 补 `cap_config.yaml` |
| 改配置加载/优先级 | `capture/collect_cap.py` | `load_config_file()`(:1727)、`apply_config()`(:1756) | `selftest` 的 "配置" 组 |
| 改采集输出位置 | `capture/collect_cap.py` | `DEFAULT_OUTPUT_ROOT`(:91) | `--help` 看默认值，跑一次采集确认目录落在新位置；`Paxini-Align` 的批量默认路径是 `../Recordings`，挪了要一起说 |
| 单只手套左右手选择 | `capture/collect_cap.py` | `HAND_SIDES`、`build_parser()`、`resolve_pose_side()`(:1622)、`main()` | `selftest` 的 "手别选择" 组 |
| 启动失败诊断 | `capture/collect_cap.py` | `write_startup_failure()`(:2313)、`main()` 的 `finally` | `selftest` 的 "启动失败工件" 组 |
| 采集主循环节拍 | `capture/collect_cap.py` | `pump_and_record()`(:2073) | 跑一次采集看 `manifest.row_mismatch` |
| **启动闸门（待机 + 按键）** | `capture/collect_cap.py` | `StartGate`、`SelectedPoseWatch`、`read_start_key()`、`wait_for_camera_first_frames()`、`main()` 启动段 | `selftest` 的「启动闸门与位姿冻结」；真机看 `manifest.session` / `quest.standby_samples_discarded` |
| **三源起停与运行期检查** | `capture/collect_cap.py` | `RuntimeMonitor`、`abort_recording()`、`main()` 启动段 | `selftest` 的「三源运行期检查」+「启动闸门与位姿冻结」；真机看 manifest `runtime.abort_reason` |
| 相机出帧检查（0.5 Hz） | `capture/collect_cap.py` + `camera_ffmpeg.py` | `RuntimeMonitor._check_frames()`、`progress_signal()`(:487) | `selftest` 首帧宽限 / 停滞 / 未配置三组 |
| 回调线程入队 | `capture/collect_cap.py` | `snapshot_cap_frame()`(:316) | 只做拷贝，别在这里写盘 |
| 批量落盘策略 | `capture/collect_cap.py` | `Hdf5Recorder.write_cap()`(:702)、`flush_cap()`(:724) / `_flush_cap_locked()`、`FLUSH_ROWS`(:160) | flush 时机影响中途读文件的结果 |
| 终止时落盘 | 同上 | `emergency_save()`(:999)、`emergency_save_and_exit()`(:207) | `selftest` 的"收尾/紧急落盘"三项 |

**并发模型（改这里之前必读）**：`Hdf5Recorder` 用一把**非重入** `Lock` 串行化所有 HDF5
写操作，因为看门狗与信号处理器会在主线程写入的过程中试图落盘。因此：

- 公共方法（`flush_cap` / `close` / `verify_rows` / `manifest` / `write_meta` /
  `write_camera_timestamps`）自己拿锁，锁内只能调 `_flush_cap_locked` / `_flush_quest_locked`
  / `_close_locked` / `_manifest_locked` 这些"已持锁"版本，否则死锁
- `flush_cap` 先把这一批的数组全部构造完再统一 `resize`：中途抛异常时数据集都停在旧行数，
  不会出现"前 4 个数据集多 100 行、后 3 个没有"的错位
| 相机完整的 ffmpeg 命令 | `capture/camera_ffmpeg.py` | `FfmpegCamera.command()`(:344) | V4L2 下跑一次 `--list-devices` |
| 相机出帧进度 | `capture/camera_ffmpeg.py` | `progress_signal()`(:487) | 兼容 `frame` / `out_time_us` / `total_size` |
| 每帧时间戳回填 | `capture/camera_ffmpeg.py` | `probe()`(:607) → `probe_frames()`(:178) | 采集后看 `cameras/*/timestamp_unix_ns` 是否单调 |
| 设备枚举 | `capture/camera_ffmpeg.py` | `discover_video_nodes()`(:74) | `--list-devices` |
| ffmpeg 路径解析 | `capture/collect_cap.py` + `camera_ffmpeg.py` | `resolve_ffmpeg()`、`find_ffmpeg()` | `--list-devices --ffmpeg <路径>` 看打印的版本 |
| PC Service 启动 | `capture/quest3.py` | `pc_service_command()`、`maybe_start_pc_service()` | `selftest` 杂项；真机看 `RoboticsServiceProcess` 是否在跑 |
| Quest 连接与样本 | `capture/quest3.py` | `XrtReader.collect_sample()` | `selftest` 的 "Quest 采集" 组 |
| Quest 数据有效性 | `capture/collect_cap.py` | `quest_sample_is_valid()`、`SelectedPoseWatch`、`RuntimeMonitor._check_health()` | `selftest` 的三源运行期检查 + 启动闸门；看 `manifest.quest.valid_rows` / `manifest.quest.selected_pose` |
| Quest 无线 ADB | `capture/quest3.py` | `parse_mdns_services()`、`pair_and_connect()`、`--pair/--wireless` | `selftest` 的 "无线 ADB" 组；真机 `quest3.py --pair` |
| Quest C8 时钟探针 | `diagnostics/quest_clock_probe.py` | `run_probe()`、`evaluate_clock_samples()` | `selftest` 的 "C8 探针" 组；重复判据按持续时长（默认 1 s），产物落 `clock_probe/` |
| 回放窗口 | `viewer/visualize.py` | `collect_payload()`(:380) → `run_window()`(:549) / `write_viewer()`(:966) | `--save-png` 出一张 |
| Quest 回放 payload | `viewer/quest_view.py` | `load_quest_view()`(:206)、`load_clock_report()`(:160)、`take_evenly()`(:71) | `selftest` 的 "Quest 可视化"；默认 auto（新采集 host，老文件 unix），`none` 才独立轴 |
| 回放视频预览 | `viewer/visualize.py` | `build_preview()`(:302) | `selftest` 的 "视频" 组 |
| 分布力空间散点图 | `viewer/visualize.py` + `viewer/sensor_layout_schematic.py` | `build_slot_force()` 的 `dist`、`drawDistribution()`；无本地 Studio 模型时用示意网格，切向箭头用 `dist.tangent` | `selftest` 的 "可视化分布力" / "示意布局" 组 |
| 实时视图 | `viewer/live_view.py` | `HtmlLiveView`(:773) / `LiveView`(:920) | `--view` 起一次 |
| 手套零点标定 | `calib/hand_calib.py` | `check_*_gate`、子命令 `encoder`/`sensor`/`magnet` | `calib/hand_calib.py selftest`（7 项） |
| 相机延迟测量（V4L2） | `latency/measure_camera_latency.py` | 整份，复用 `FfmpegCamera`；默认逐路测 cam0/cam1 | 跑一次看 IQR 与前后半段漂移 |
| 延迟测量台账 | `latency/latency_records.py` | 纯标准库 + PyYAML | 改字段要同步 `records/*.yaml` 与下游读法 |

## Align 侧：下游重采样

| 任务 | 文件 | 具体位置 | 怎么验 |
|---|---|---|---|
| 改采样网格 / 参考相机 | `Paxini-Align/align_capture.py` | `align_recording()`(:566)、`--reference` | `selftest.py` 的"多相机最近帧"项 |
| **改公共有效窗口** | 同上 | `align_recording()` 里的 `windows` / `keep` 段（延迟补偿之后）、`quest_valid_window()`(:543) | `selftest.py` 的五项窗口用例（晚开始/早结束/边界减 Δ/summary/不重叠报错） |
| 改插值方式 | 同上 | `lerp_rows()`(:376) / `nearest_rows()`(:405) | 合成线性信号零误差（selftest 第 1 项） |
| 改延迟补偿 | 同上 | `resolve_delay()`(:316) | `--no-delay` 与默认值对照，差值应等于 Δ；**默认 0 = 未标定** |
| 改输出字段 | 同上 | `write_aligned()`(:951) | `selftest` 的"输出可重开"项 + 同步 `Paxini-Align/README.md` 字段表 |
| 改质量判据 | 同上 | `gates()`(:1120)、`report()`(:1056) | 跑一次真采集；判据不过时不会产出文件，退出码 1 |
| 改 Quest 对齐 | `Paxini-Align/align_capture.py` | `load_quest_clock_report()`(:88)、`align_quest_to_targets()`(:470)、`write_quest_alignment()`(:1007) | `selftest.py` 的默认对齐、整行取前驱、滞后范围、无数据跳过、选样 tie-break、C8 硬门控；同步 Align README |

对齐层**只读** `data.h5`，绝不回写；改这里不会影响采集端，但改了输出布局要升
`ALIGNED_FORMAT_VERSION`（当前 2，与采集端的 5 无关）。

## 关键常量（写代码前先看这几个）

| 常量 | 值 | 含义 |
|---|---|---|
| `CAP_MAX_POINTS` | `119` | **单个槽位**的最大点数，不是总点数 |
| `CAP_SLOT_POINTS` | 49/20/49/20/49/20/49/20/119/49 | 10 个槽位各自点数，合计 **444** |
| `CAP_SLOTS` | 10 个名字 | `thumb_one` … `palm_one`，顺序与 SDK 结构体一致，不能改 |
| `ANGLE_FIELDS` | 4 个 | `index_proximal`/`index_distal`/`middle_proximal`/`middle_distal` |
| `CAMERA_SLOTS` | `("cam0", "cam1")` | 两路相机 |
| `HAND_SIDES` | `("left", "right")` | 单只手套手别；Quest 默认跟随 |
| `FLUSH_ROWS` | `100` | 攒够 100 行才 resize+赋值，120 Hz 下约 0.83 s |
| `SESSION_START_TOLERANCE_S` | `5.0` | 三源时间窗粗检阈值，只告警不纠正 |
| `format_version` | `5` | 改 `cap/` 或 `quest/` 布局必须升 |

**最容易记错的一条**：444 是全部槽位加起来，`CAP_MAX_POINTS` 是单个槽位的上限 119。
把 119 当成总点数会写越界；把 444 当成每槽位会分配 4 倍内存。

## Dex 侧

| 任务 | 文件 |
|---|---|
| 采集（对应 `Cap_Quest`） | `Paxini-Dex/Dex_Quest/collect_hand_pose.py` |
| Quest 封装（已与 Cap 分叉） | `Paxini-Dex/Dex_Quest/quest3.py` |
| 相机源（OpenCV，与 Cap 不同） | `Paxini-Dex/Dex_Quest/camera_source.py` |
| 导纳控制 | `Paxini-Dex/Dex_Admittance/dex_admittance/` |

Dex 的采集**在采集时就把最近邻对齐做完了**，输出 `aligned.jsonl`。两边设计相反，
不要拿一方的做法去"修"另一方。

## 数据流一句话

```text
PXCap3 回调(120Hz) → snapshot_cap_frame 拷贝入队 → pump_and_record 主循环
                                                        ↓
                              Hdf5Recorder.write_cap 攒 100 行批量写
                                                        ↓
ffmpeg -f v4l2(30fps, 墙钟 pts) → mkv → probe() 回填逐帧时间戳
                                                        ↓
                                            data.h5 + manifest.json
```

写盘全在主线程，回调线程只做 numpy 拷贝。所以采集中途读 `data.h5` 会看到一跳
一跳的数据（每 0.83 s 一块），实时视图因此只读内存、不读文件。