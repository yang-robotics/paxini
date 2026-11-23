# Recordings

采集输出目录。**这些数据不入库**（`.gitignore` 里只有本文件被放行），全部由
`Cap_Quest/capture/collect_cap.py` 写入，默认位置就是这里。

一份采集是一个 `data_<YYYYMMDD>_<序号>/` 目录，结构见
[`../DOC/HDF5结构说明.md`](../DOC/HDF5结构说明.md)。

## 当前内容

| 目录 | 手别 | 相机 | 手套 | 时长 | Quest | 来历 |
|---|---|---|---|---|---|---|
| `data_20261117_1` | 右手 | 双路 1280×720@30 | 2379 行 / 114.2 Hz | 24.5 s | 无（`quest_rows=0`） | 样例，来自标签 `data-sample-20261117` @ `52f63ce`（源分支已删，标签保留溯源） |
| `data_20261117_2` | 右手 | 双路 1280×720@30 | 2587 行 / 115.1 Hz | 26.3 s | 无（`quest_rows=0`） | 同上 |
| `data_20261117_3` | 右手 | 双路 1280×720@30 | 2620 行 / 117.1 Hz | 26.3 s | 无（`quest_rows=0`） | 同上 |
| `data_20261118_1` | 右手 | 双路 1280×720@30 | 2396 行 / 115.9 Hz | 24.3 s | 无（`quest_rows=0`） | 本机现采 |

四份的行数、频率、时长与各自的 `manifest.json` 逐项核对一致（2026-11-20，按
`manifest.rows` / `actual_rate_hz` / `duration_s` 读，不按目录名猜）；`data.h5` 均为
`format_version=4`，可直接喂 [`../Paxini-Align/align_capture.py`](../Paxini-Align/align_capture.py)。

**四份都没有 Quest 行**（`quest_rows=0`、`quest.enabled=false`）。表格早先给三份
20261117 样例标过「有」，那是因为 `data.h5` 里有空的 `quest/` 组——组在、行是 0，
不等于采到了数据；C8 与 Quest 对齐在这批数据上无从验证（下游默认对齐遇到这类文件会
自动跳过，不会报错）。

## 已知限制

**这四份都是旧采集路径（WSL2，相机经 Windows）下产生的**（`manifest.env.platform` 全为
`...microsoft-standard-WSL2`，相机 device 是 Windows 的 `@device_pnp_\\?\usb#vid_...` 串），
当前 `main` 只有原生 Ubuntu（V4L2）一条路。读这批数据时注意：

- 相机 ID 是 `right_cam0` / `right_cam1`（当时前缀跟手别走），当前代码写 `<cap_side>_camN`；
- 对齐这批数据要用当时的 Δ **192.3 ms**（1280×720@30 `copy`；原始台账在
  `archive/wsl-backend` 分支的 `Cap_Quest/latency/records/andy-win.yaml`）。**当前路径必须
  重测，不能沿用它**；2026-11-20 起 Align 的默认 Δ 是 0（未标定），所以对齐这批老数据
  必须显式给 `--delay-ms 192.3`，否则拿到的是一份没做延迟补偿的产物；
- 四份的 `data.h5` 里都有 `quest/` 组但 0 行，C8 时钟域从未验证；`--quest-align sdk_unix`
  没有可对齐的样本，会被门控拒绝。这是预期行为，不是数据损坏。

另外 `data_20261118_1` 与 `DOC/待验证清单.md` C8/A12 引用的同名文件**不是同一份**
（那份是 Quest 左手、640×480@15、`left_cam0` 245 帧，原始文件已不在本工作区）。

## 下载

数据本体不进 git，通过下面两处获取（2026-11-20 起）：

- ModelScope 数据集（2026-11-20 已上传，**当前仍是私有仓**）：
  <https://www.modelscope.cn/datasets/yanglab/paxini>

  内容：`data_20261117_{1,2,3}.zip` + `data_20261118_1.zip` + `SHA256SUMS.txt` +
  `README.md`（数据集卡片）。四份 zip 已回读校验：`sha256sum -c SHA256SUMS.txt`
  四条全 `OK`，且与本地原件逐字节相同。

  这是**内部专用**仓（2026-11-20 决定）：私有是刻意设置，不是没配好。组织成员登录后
  可下载，匿名访问返回「不存在的数据集」属预期。将来要对外分发，先改公开
  （`update_dataset("yanglab/paxini", visibility=5)`），再把这一段的措辞一起改掉——
  这批数据是旧采集路径产物，对外部用户还得额外交代 Δ 的来源。
- GitHub Release（只放 20261117 三份样例的 zip，tag 指向数据源提交 `52f63ce`）：
  <https://github.com/yang-robotics/paxini/releases/tag/data-sample-20261117>

两处都只对组织成员开放，没有对外分发渠道——这是当前的口径，不是遗留缺陷。

## 上传（`tools/upload_modelscope.py`）

上传走脚本，不要手工拖网页：

```bash
cd Paxini-Cap
python tools/upload_modelscope.py                         # 演练：只列门禁，不联网
python tools/upload_modelscope.py --only data_20261118_1   # 只处理一份
python tools/upload_modelscope.py --upload                 # 真传
python tools/upload_modelscope.py --zip --upload            # 需要打包时才加 --zip
```

- **默认不打包**：staging 里用硬链接镜像（不额外占盘），远端就是
  `data_x/data.h5`、`data_x/cameras/*.mkv` 这几个文件，下载时可以只取需要的那个。
- **默认不联网**，`--upload` 才动网；确认无误再加。
- `--zip` 改成每份一个 `<目录名>.zip`（内部根节点是目录名本身），适合想一次拿全的人。
  zip 用 STORED 不压缩——里面是 mkv 和 h5，deflate 只省 2% 却要全程 CPU。
  已有 zip 比目录新就复用，`--rebuild` 强制重打。
- 已上传的四份老数据是 zip，新采集默认传目录，两种形态共存，
  `SHA256SUMS.txt` 两种条目都记。
- 门禁读的是 `manifest.json`：`verify.row_mismatch` 必须为空、手套行数 > 0、
  实际频率在 50~200 Hz、每路相机 `error` 为空且帧数 > 0、`data.h5` 与
  `cameras/<id>.mkv` 都在。不合格默认跳过并打印原因，`--allow-broken` 才强传。
- 产物在 `<recordings>/.upload_staging/`（目录树或 zip）+ 一份 `SHA256SUMS.txt`
  （逐文件 `sha256sum -c` 格式）。
- `SHA256SUMS.txt` 描述的是**整个仓库**：`--upload` 时先读远端已有文件，
  和本批合并后再写，所以分几次上传也不会把早先的条目冲掉。
- 令牌走 `--token` 或环境变量 `MODELSCOPE_API_TOKEN`
  （<https://modelscope.cn/my/myaccesstoken>），**两者都没有会直接报错退出**。
  `~/.modelscope/credentials` 里只有 cookies / session / 截断的 `git_token`，
  `login()` 不接受，别指望它兜底。**令牌不要写进仓库**。
- 上传只覆盖同名文件，不会删远端已有的 `README.md`（数据集卡片仍手工维护）。
- 脚本会自己建 staging 目录，不用手工建；`.upload_staging/` 与 `*.zip` 都在
  `.gitignore` 里。

`modelscope` 只有 `--upload` 那条路需要（`--dry-run` 是纯标准库）。没装就
`python3.10 -m pip install --user modelscope`，装完记得确认 numpy / h5py 没被连带
改版本。

## 新增采集

不需要手动建目录：`collect_cap.py` 会按当天序号自动递增。要指定位置用
`--output-dir`，但注意 `align_capture.py` 的默认批量路径是 `../Recordings`。
