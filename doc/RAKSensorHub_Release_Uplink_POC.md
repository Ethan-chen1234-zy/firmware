# RAK SensorHub — 上行 POC 固件发布流程（不破坏本地修改）

本文说明如何在**不切换当前分支、不 stash 工作区**的前提下，从已打好的 Git tag 检出固定源码、构建 **RAK2560** UF2，并将**流程与校验信息**归档；二进制通过 **GitHub Release 附件**分发（推荐），避免把大文件直接塞进主分支历史。

---

## 1. 适用场景

- **Git tag**：例如 `v2.7.21-raksensorhub-poc-uplink`（附注 tag 的说明对象为该次「上行解析」相关提交）。
- **本地分支**（如 `feature/add_RAKSensorHub`）有未提交修改：不要用 `git checkout tag` 直接覆盖工作区。

---

## 2. 推荐做法：`git worktree`（第二目录、 detached HEAD）

在主仓库目录执行：

```powershell
cd D:\Git_code\Meshtastic\firmware
git fetch --tags origin

# 若曾创建过同名目录，先删除 worktree（路径按你本机调整）
# git worktree remove D:\Git_code\Meshtastic\firmware-wt-uplink --force

git worktree add D:\Git_code\Meshtastic\firmware-wt-uplink v2.7.21-raksensorhub-poc-uplink
```

说明：

- 主目录 `D:\Git_code\Meshtastic\firmware` 的**分支与工作区不变**。
- 新目录里是 **detached HEAD**，指向 tag 对应提交（例如 `bc893370c`）。

构建：

```powershell
cd D:\Git_code\Meshtastic\firmware-wt-uplink
pio run -e rak2560
```

产物路径以 PlatformIO 输出为准，一般为：

`.pio\build\rak2560\firmware-rak2560-*.uf2`

将 UF2 **复制**到主仓库的发行目录（便于与文档一起管理，例如）：

`releases\uplink-poc\v2.7.21-raksensorhub-poc-uplink\`

计算校验和（写入 `README-RELEASE.md`）：

```powershell
Get-FileHash .\firmware-rak2560-*.uf2 -Algorithm SHA256
```

用完后可删除 worktree 释放磁盘：

```powershell
cd D:\Git_code\Meshtastic\firmware
git worktree remove D:\Git_code\Meshtastic\firmware-wt-uplink
```

---

## 3. 备选：`git stash`（无额外目录时）

若不能使用 worktree：

```powershell
git stash push -u -m "wip: before uplink release build"
git switch --detach v2.7.21-raksensorhub-poc-uplink
pio run -e rak2560
# 复制 UF2 到安全路径后
git switch feature/add_RAKSensorHub
git stash pop
```

注意：`stash -u` 会包含未跟踪文件，弹出时可能有冲突，需自行处理。

---

## 4. 上传到 Git（推荐分工）

| 内容 | 建议位置 |
|------|-----------|
| **Tag**（已存在则跳过） | `git push origin v2.7.21-raksensorhub-poc-uplink` |
| **UF2 二进制** | **GitHub Release**「Assets」附件，不要强推大文件进默认分支（除非已配 Git LFS） |
| **流程与 SHA256** | 本文件 + `releases/uplink-poc/.../README-RELEASE.md`（可提交进仓库） |

在浏览器中：**GitHub 仓库 → Releases → 已有 tag 的 Release 或 Draft → Upload assets**，上传 UF2（及可选 zip）。

若已安装 [GitHub CLI](https://cli.github.com/) 且已 `gh auth login`：

```powershell
cd D:\Git_code\Meshtastic\firmware\releases\uplink-poc\v2.7.21-raksensorhub-poc-uplink
Compress-Archive -Path .\firmware-rak2560-*.uf2 -DestinationPath .\rak2560-uplink-poc-v2.7.21.zip
gh release upload v2.7.21-raksensorhub-poc-uplink .\rak2560-uplink-poc-v2.7.21.zip --clobber
```

（若该 tag 尚无 Release，需先在网页上 **Draft release** 并选对 tag，或使用 `gh release create`。）

---

## 5. 本 tag 构建基线说明（`v2.7.21-raksensorhub-poc-uplink`）

- **Tag 对象**：`v2.7.21-raksensorhub-poc-uplink` → 提交 **`bc893370c`**（消息示例：`RAKSensorHub：完善上行解析`）。
- **该提交上 `rak2560` 的 `platformio.ini`**：含 **`HAS_RAKHUB=1`**，**未**打开 `RAK_SENSORHUB_DOWNLINK_POC` / `RAK_SENSORHUB_USB_PROFILE`（即**上行 POC / 解析与遥测路径**，非后续 USB 下行 POC）。
- **PROGNAME / 文件名**：以构建时版本脚本为准（例如 `firmware-rak2560-2.7.22.bc89337.uf2`）；发布物可复制为带 tag 名的稳定文件名，见同目录 `README-RELEASE.md`。

---

## 6. 与「下行 / USB RAKHUB」版本的区别

- **上行 POC tag**（本文）：侧重 **Probe → Hub → Meshtastic** 解析与遥测导出；无 USB `RAKHUB` 文本下行配置。
- **当前特性分支后续提交**：可能包含 `RAK_SENSORHUB_DOWNLINK_POC`、`RAK_SENSORHUB_USB_PROFILE` 等，**不得**用本 tag 的 Release 去指代那些二进制。

更新 changelog / README 时，请写清 **tag 名 + commit + 构建 env**。
