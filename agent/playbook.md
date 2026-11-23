# 任务手册

按"我要干成什么"组织，直接照做。命令都在 `Paxini-Cap/Cap_Quest/` 下执行。

## 1. 跑一次采集（从零）

```bash
# 0) 环境（只做一次，已装可跳）
sudo dpkg -i ../sdk/pxhandsdk_1.0.0.0.ubantu2204_amd64.deb
python3.10 -m pip install --user -r ../requirements.txt

# 1) 确认设备在
python3.10 capture/collect_cap.py --list-devices    # 串口末尾应是 PXCap，ffmpeg 应 ≥ 5.0

# 2) 不接硬件先自检
python3.10 selftest/selftest.py                     # 全过

# 3) 关 NTP（必须）
sudo timedatectl set-ntp false

# 4) 采
python3.10 capture/collect_cap.py --cap-side right --no-quest
```

**成功判据**：结束时会打印 `verify.row_mismatch` 为空。非空说明某组数据集首维与
自己的行数不一致，这个文件能打开但按行取数会整体错位。

产物在 `Paxini-Cap/Recordings/data_<YYYYMMDD>_<n>/`（在 `Cap_Quest` 下看是 `../Recordings/`）。

## 2. 采集起不来，按现象查

| 现象 | 先查这个 |
|---|---|
| `SERIAL_BUSY(102)` | 串口被占：关掉 PXCap Studio、`hand_calib.py`、旧进程 |
| 找不到串口 | `ls /dev/ttyACM*`；空就查线缆、供电与 `dialout` 组（改了组要重新登录） |
| `import pxhandsdk` 成功但没 `PxCap3` | 用成 3.12 了，换 `python3.10` |
| 相机 0 fps / 5 fps | 先 `--list-devices` 看节点，再查分辨率/帧率是否被设备支持；5 fps 多半是带宽或 Hub 问题 |
| 相机 20 秒不出图 | 先 `--camera-mode cam0` 排除带宽；再验分辨率是否被支持。宽限期后仍无帧会直接中止整场（`runtime.abort_reason`） |
| Quest 装配失败（PC Service / ADB / SDK） | 直接判启动失败，不再降级。没头显就显式加 `--no-quest` |
| Quest 有行但全是 -1/NaN | PC Service 中途掉线，getter 异常被吞：查 `manifest.quest.errors` |
| 采完没有 `manifest.json` | 被强杀了。数据落在最后一个满批之前（每批已 flush），先 `--clear-write-flag` 再读；2026-11-13 那批老文件没这个保护，全废 |

## 3. 改 HDF5 字段

三步，一步不能少：

1. 改 `capture/collect_cap.py` 的 `CAP_DATASETS`（手套侧）或 `_quest_layout()`
   （Quest 侧），同时把 `format_version` 从 4 加一
2. 改 `DOC/HDF5结构说明.md` 的字段表 + 版本变更表（**定点修改，不要整份重写**）
3. 在 `selftest/selftest.py` 加一条用例，跑通 102→103 项

漏第 3 步的后果：下次别人改坏这个字段，没人发现。

## 4. 加一个命令行参数

1. `capture/collect_cap.py` 的 `build_parser()` 加 `add_argument`
2. `cap_config.yaml` 加同名字段 + 一行注释
3. `load_config_file()` / `apply_config()` 确认这个键会被读取（命令行优先于文件）
4. `Cap_Quest/README.md` §6.2 参数表加一行
5. `selftest` 的"配置"组加用例

验证：`python3.10 capture/collect_cap.py --help` 能看到，且
`python3.10 -c "import yaml;print(yaml.safe_load(open('cap_config.yaml')))"` 不报错。

## 5. 测相机延迟

只有一条采集路径，一个脚本：

| 脚本 | 帧时间戳的构成 | 实测量级 |
|---|---|---|
| `latency/measure_camera_latency.py` | 墙钟 pts 本身，无锚点 | 几十毫秒（本机尚未实测） |

Ubuntu：

```bash
cd <repo>/Paxini-Cap/Cap_Quest
sudo timedatectl set-ntp false          # pts 走 CLOCK_REALTIME，测完记得开回来
python3.10 latency/measure_camera_latency.py
```

ffmpeg 必须是 5.0 以上（静态版装法见 `Paxini-Cap/README.md` §1.1）。还要有显示会话
——脚本用 `cv2.imshow` 弹二维码窗口，SSH 里没有 `DISPLAY` 会被 `check_display()` 直接拦下。

**默认逐路测 cam0 和 cam1、每路录 20 秒**，所以上面两条命令都不带参数。枚举到的第 3 路
（笔记本自带的 `USB2.0 HD UVC WebCam`）不参与采集，脚本会自动跳过；只想测一路加
`--camera 0`，想改时长加 `--seconds N`。

两路是不同 USB 口，延迟也不同，**各用各的，不要取平均**。跑完会打一张各路汇总表；
某一路没有有效样本时，说明那一路没拍到二维码窗口，得调整相机朝向或单独重测。

把弹出的二维码窗口移到两路都能看到的屏幕位置，占各自画面 1/4 以上。

**成功判据**：解码率 > 30%、样本 ≥ 100、IQR < 20 ms。Ubuntu 版还要看「前后漂移」
< 10 ms —— 超了说明测的时候系统时钟被步进过，或者相机在自动调曝光，结果不能用。

**测完自动记账，不用手抄**：`latency/records/<电脑用户>-<win|ubuntu>.yaml`，
一台机器一个文件，键是测量序号；日志末尾会打印「测量记录 … 第 N 次（合格）」。
`--no-record` 可以跳过，用来调试。

同一台机器不同时间的读数会**整体跳一档**：本机 2026-11-16 连测三次是
189.7 → 287.2 → 192.3 ms，而 IQR 一直只有 5~6 ms。所以下游取数只许从台账读
最近一次的合格值，别抄某个固定数字：

```python
import sys; sys.path.insert(0, "Paxini-Cap/Cap_Quest/latency")
from latency_records import load_records, pick_latest

entry = pick_latest(load_records(), camera=0)   # 跳过 ok: false，两路相机要分开取
median = entry["median_ms"]
```

台账里每条带 `delta_prev_ms`（与上一条同参数的差），超过 20 ms 时测量当场警告，
说明链路变了 —— 先重测确认，别直接拿去修时间轴。

结果怎么用：

- Windows：`timestamp_unix_ns` 和 `timestamp_monotonic_ns` **两列都减**
- Ubuntu：只有 `timestamp_unix_ns` 一列，减它

## 6. 看采到的数据

```bash
python3.10 viewer/visualize.py ../Recordings/data_20261113_9            # Tk 窗口
python3.10 viewer/visualize.py ../Recordings/data_20261113_9 --html     # 导出浏览器版
python3.10 viewer/visualize.py ../Recordings/data_20261113_9 --save-png a.png
```

没有 `python3.10-tk` 时用 `--html`。浏览器版支持悬停读数、保峰抽稀、双路视频面板。

## 7. 提交前

```bash
cd Paxini-Cap/Cap_Quest
python3.10 -m py_compile capture/*.py viewer/*.py selftest/*.py latency/*.py calib/*.py
python3.10 -m pyflakes  capture/*.py viewer/*.py selftest/*.py latency/*.py calib/*.py
python3.10 selftest/selftest.py                      # 全过（项数以运行输出为准）
```

三关都过再提交。改过行为或数据结构，`selftest` 必须加用例，不能只靠手跑过一遍。
进 main 的流程（PR、冲突用 merge 而不是 rebase）见 §9。

## 8. 判断一个改动安不安全

按顺序问自己：

1. **它会不会让 `cap/` `quest/` `cameras/` 的某一维变长或变短？** → 会，就要升
   `format_version` 并改文档
2. **它是不是在采集端做了配对或插值？** → 是，违反约束 1，停下
3. **它有没有引入一个新的时钟读数？** → 有，确认和原有列同源，否则违反约束 2
4. **它会不会让回调线程做 IO？** → 会，挪到主线程
5. **它改的是不是 `DOC/HDF5结构说明.md`？** → 整份重写会覆盖用户手动编辑的内容

五条都过了，再动手。

## 9. 改动怎么进 main（PR 与冲突处理）

**默认走 PR，不直接推 main。** 只有一两行的紧急修补（版本号、错别字、注释）才直接推。

```bash
git switch main && git pull                     # 起点必须是当前 main
git switch -c feature/<短描述>
# 改代码、按 §7 过三关、commit
git push -u origin feature/<短描述>              # 推的是分支
gh pr create --fill                              # 用提交信息当标题正文
gh pr view --json mergeable,mergeStateStatus     # 看能不能合
gh pr merge --squash --delete-branch             # 服务端合并
git switch main && git pull                      # 本地同步
```

### 冲突怎么解：把 main 合进你的分支，不要 rebase

PR 显示 `CONFLICTING` / merge 按钮变灰时：

```bash
git fetch origin
git switch feature/<短描述>
git merge origin/main            # 冲突在这一步出现
# 打开冲突文件，改掉 <<<<<<< ======= >>>>>>> 三段
git add <改过的文件>
git commit                       # merge 会自己带默认信息，直接用
git push                         # 普通 push，不需要 force
```

PR 页面会自动重算，状态条变回可合。

**为什么不用 `rebase origin/main`：**

1. **冲突要解多次**。rebase 逐个提交重放，5 个提交碰同一个文件就可能撞 5 次；merge 只在合并点撞一次。
2. **force-push 会让 review 记录失效**。评论绑在 commit + 行号上，改写后变成 outdated。
3. **别人基于你分支干活会崩**。旧哈希不存在了，他们得手工修。

唯一可以用 rebase 的场景：分支只有 1~2 个提交、没人 review、也没人基于它开发。真要 rebase，推的时候用
`git push --force-with-lease`（远端有新提交时会拒绝），**永远不要用 `--force`**。

### 三个坑

- **squash 合并后别复用旧分支。** 你的提交被压成一个新提交进 main，旧分支的历史和 main 对不上，继续在上面开发会带出一堆重复改动。所以 `--delete-branch`，下次从新 main 切。
- **`gh pr merge` 报 `not mergeable` 不等于“落后”。** 落后本身不阻止合并，有文本冲突才阻止。先看 `mergeStateStatus`。
- **语义冲突没有任何工具能拦。** main 删了某个函数、你分支还在调用它——git 认为不冲突，合完才炸。合并前跑一次 `git diff origin/main...HEAD --stat`，看自己碰的文件 main 最近有没有动过。

## 10. 标签与分支的命名和删除

### 打标签：必须显式指定提交，必须用附注标签

```bash
git tag -a <名字> <commit> -m "<一句话：这批东西是什么、对应哪个提交>"
git push origin <名字>
gh release create <名字> --verify-tag --title "..." --notes-file ... <资产...>
```

- **`-a` 不能省。** 轻量标签只是一个 ref，没有打标人、日期、说明，`git tag -f` 能无声挪走；附注标签是独立对象，出问题时能查是谁、什么时候、为什么。
- **Release 的标签不能靠默认行为。** `gh release create` 不带 `--target` 时，会在**当时的默认分支 HEAD** 上建标签——那个提交和你发布的内容通常毫无关系。要么先 `git tag -a` 再 `--verify-tag`，要么明确写 `--target <sha>`。

> 2026-11-20 踩过：`data-sample-20261117` 发布时没带 `--target`，标签落在 `3c42ec5`（PR #8 的 Quest 合并提交），而真正的数据在 `52f63ce`。Release 页面「Source code」下载到的是代码树，一份样例都没有。

### 移动 Release 的标签会把它打回草稿

Release 按标签名关联，所以"改指向"只能删标签再重建。**删标签那一刻 Release 会变成 `draft=true`**，
重建标签也不会自动恢复——资产对外不可见，`gh release list` 里显示 `Draft`。

```bash
gh release edit <tag> --draft=false            # 重新发布
gh release edit <tag> --latest                 # 需要占 Latest 位置时再补一句
```

> 2026-11-20：修 `data-sample-20261117` 的指向后两个 Release 都成了草稿，
> `--draft=false` 才恢复。改完必须 `gh release view <tag> --json isDraft,assets` 复核。

### 名字不要跨命名空间

标签和分支别共用前缀：`refs/tags/archive/xxx` 与 `refs/heads/archive/yyy` 在 `git log --decorate` 里混成一片，看图和补全都容易选错。标签用不带斜杠的平铺名，例如 `wsl-backend-archive-20261120`、`data-sample-20261117`、`vendor-binaries-v1.0`。

### 删分支前先确认 tip 有标签

```bash
git branch -a --contains <sha>     # 除了待删分支，还有谁指向它
git tag --contains <sha>           # 标签能不能接管溯源
```

两条都只剩待删的那个分支时，**先打标签再删**，否则提交变不可达，gc 之后连 `git cat-file` 都取不回来。

> 2026-11-20：`52f63ce` 当时只被 `data/sample-20261117` 一个分支指向。先修标签、再删分支，顺序反了那 251 MB 样例就再也无法从 git 复现。

### 归档分支的处置

- 冻结快照分支（如 `archive/wsl-backend`）**只加一个说明文件，不删任何代码**，checkout 出来必须能直接跑。
- 它不参与合并。合并只会带进"相对分叉点的增量"，main 上的删除不受影响——想要回某段代码就显式 `git checkout <分支> -- <文件>`。
- 说明文档只写在归档分支上（根目录 `ARCHIVED.md`），**main 不放副本**——两份必然不同步。main 要指路就只写分支名，别写文件名。
