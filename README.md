# Paxini

PXCap III 数据手套与 PXDex III 灵巧手的开发工作区。两条线，代码骨架同源，用途完全不同：

- **`Paxini-Cap/`** —— 只记录、不运动。手套 + 双路相机 + Quest 3 同步采集，输出单个 HDF5。
- **`Paxini-Dex/`** —— 会运动。PXDex III 灵巧手、导纳控制、`Dex_Quest` 遥操作采集。

`Cap_Quest` 对应 `Dex_Quest`，差别是 Cap 没有电机、不产生任何动作。两条线的
`quest3.py` 已经分叉，不要再追求两份一致。

## 仓库结构

| 路径 | 内容 |
|---|---|
| `Paxini-Cap/` | Cap 采集上位机 → [工程说明](Paxini-Cap/README.md) |
| `Paxini-Cap/Paxini-Align/` | 下游重采样：手套流插值、Quest 默认按墙钟取前驱 → `aligned_*.h5` → [说明](Paxini-Cap/Paxini-Align/README.md) |
| `Paxini-Dex/` | Dex 执行器（SDK 封装、`Dex_Quest`、`Dex_Admittance`）→ [说明](Paxini-Dex/README.md) |
| `agent/` | 给 AI 助手的项目认知文档 → [入口](agent/README.md) |
| `厂商资料/` | 厂商 PDF 说明书、SDK 手册、PXCap / PXDex 的 URDF 与 STL |
| `厂商数据采集/` | 厂商上位机产出的原始录制数据（**不入库**，仅本地保留） |
| `Quest3_功能与接口清单.xlsx` | Quest 3 功能与接口梳理 |
| `帕西尼PXHandSDK功能清单_amd64_1.0.0.0.xlsx` | PXHand SDK 接口清单 |
| `厂商上位机判分标准.txt` | 上位机能力对照标准 |
| `启动PXCap.bat` / `PXCap Studio.lnk` | 本地启动入口 |

## 文档怎么读

四层，各答一个问题，不要重复读同一件事：

| 层级 | 文件 | 回答什么 |
|---|---|---|
| 项目 | 本文件 | 有哪些子项目、东西在哪、二进制从哪来 |
| 工程 | [`Paxini-Cap/README.md`](Paxini-Cap/README.md) | 怎么开发：架构、不能违反的约定、环境坑、改完怎么验证 |
| 使用 | [`Paxini-Cap/Cap_Quest/README.md`](Paxini-Cap/Cap_Quest/README.md) | 怎么用：命令、参数表、输出格式、FAQ、已知限制 |
| 细节 | [`Paxini-Cap/DOC/`](Paxini-Cap/DOC/README.md) | HDF5 逐字段、时钟域换算、待验证清单、标定设计 |

[`agent/`](agent/README.md) 是给 AI 助手的另一套：任务到文件的映射、硬约束及其理由、
任务手册、踩坑记录。人也可以看，但它是为「快速动手」而不是「系统学习」写的。

## 安装包与二进制

单个文件 8–171 MB，不纳入 Git，统一放在 [Releases](https://github.com/yang-robotics/paxini/releases) 的 `vendor-binaries-v1.0`。克隆后按下表放回：

| 文件 | 放回位置 |
|---|---|
| `pxhandsdk_*.deb` | `Paxini-Cap/sdk/` |
| `XRoboToolkit_PC_Service_*.deb` | `Paxini-Cap/sdk/` |
| `XRoboToolkit-Quest-*.apk` | `Paxini-Cap/sdk/xrobotoolkit/` |
| `pxcap-studio_*.deb` | 本机安装即可，不必放回仓库 |

原始 deb 的校验值见 `Paxini-Cap/sdk/SHA256SUMS`。

## 环境

Python 3.10 + Ubuntu 22.04/24.04（原生安装）。手套 SDK 只能用 `python3.10`，
系统自带的 3.12 加载不出 `PxCap3`。Cap 侧的 Python 包统一装在 `Paxini-Cap/.venv`
（`--system-site-packages`，建法见 [`Paxini-Cap/README.md` §1.0](Paxini-Cap/README.md)），
每个新终端先 `source Paxini-Cap/.venv/bin/activate`。

版本要求、`dpkg` 安装顺序、串口权限、ffmpeg 从哪来 —— 全在
[`Paxini-Cap/README.md` §1–§2](Paxini-Cap/README.md)。

## 版本库说明

本仓库 2026-11-16 由三个来源合并而成，历史不连续：

- `Paxini-Cap/` 原仓库 `ifnotandy/Paxini-Cap`（冻结在 72c6058）
- `Paxini-Dex/` 原仓库 `Dongqiao6/Paxini`（同步至 fb224f2）
- `厂商资料/` 与根目录文档随仓库保留；录像、HDF5、APK、DEB、缓存和采集产物默认不入库

原两个 `.git` 目录备份在 `C:\Users\Andy\Desktop\paxini-git备份\`，需要拆回独立仓库时可用。
