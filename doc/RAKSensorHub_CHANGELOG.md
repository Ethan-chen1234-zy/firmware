# RAK SensorHub 分支版本更新说明

**入口文档**：开发者请先阅读仓库根目录 **`README(Add_RAKSensorHub).md`**（主文件索引、构建宏、各能力用法）；本文件仅保留 **Git 提交级** 变更表。

**分支**：`feature/add_RAKSensorHub`  
**生成方式**：根据 `git log` 整理；未包含工作区内尚未提交的改动。

---

## 〇、发行物与 tag（非「新提交」，仅追溯二进制）

| Tag | Commit | 说明 |
|-----|--------|------|
| `v2.7.21-raksensorhub-poc-uplink` | `bc893370c` | **上行 POC**：`rak2560` 构建产物与 SHA256 见 `releases/uplink-poc/v2.7.21-raksensorhub-poc-uplink/README-RELEASE.md`；发布流程见 `doc/RAKSensorHub_Release_Uplink_POC.md`。UF2 通过 **GitHub Release 附件**分发（`releases/**/*.uf2` 已 `.gitignore`）。 |

---

## 一、本分支功能演进（新 → 旧）

| 提交 | 日期 | 说明 |
|------|------|------|
| `d79f2d62a` | 2026-05-13 | USB CDC `RAKHUB …` 运行时配置 POC（`RAK_SENSORHUB_USB_PROFILE`）；RS485 多任务 `slot=`；`bin/rakhub_usb_poc.py`；`RAKSensorHubProfile.h` / `xmodem.cpp` 钩子；根目录 `README(Add_RAKSensorHub).md` 入口与下行 POC / 架构 / 上行列表等文档同步。 |
| `ecac05a5e` | 2026-05-12 | 补充文档（Add docs …） |
| `9cf699d45` | 2026-05-12 | `platformio.ini`：库路径与版本说明澄清 |
| `e94e21094` | 2026-05-11 | 合并上游 `meshtastic/develop` 至本特性分支 |
| `f12f0819a` | 2026-05-11 | AIC（4–20mA）下行：`IO_DECODE` 配置支持 |
| `3e93e8fc8` | 2026-05-09 | RAKSensorHub：下行 IOC POC（`IO_ADDPOLLEX` 等） |
| `fe0171fb0` | 2026-04-21 | RAKSensorHub：上行解析完善 |
| `63a9fadc4` … `05ceb706c` | 2026-03 | 空气质量/太阳辐射/CO₂ 与 Meshtastic 连接后数据响应等修复 |
| `0b34689d1` | 2026-03-05 | 更新 Readme |
| `7efae8b61` | 2026-03-05 | 增加 RAKSensorHub 驱动与文档 |
| `b77d9c94d` | 2026-03-05 | 为 RAK 传感器增加 SensorHub 支持（基础引入） |

---

## 二、合并自 `develop` 的公共变更（节选）

`e94e21094` 合并带入的上游提交中，与本分支同期可见例如：

- **NodeDB**：收缩与结构调整（`94bb21ecc`）
- **Mesh**：用户可见通知 `sprintf` 边界处理（`1bcabb893`）
- **显示**：SH1107 几何更新修复（`fefd42490`）
- 以及版本号自动递增、RadioLib、GPS、MQTT 等常规维护类提交。

查看合并第二父完整列表：

```bash
git log e94e21094^2 -20 --oneline
```

---

## 三、版本语义小结（便于沟通）

1. **基础**：`b77d9c94d`、`7efae8b61` — SensorHub / RAKSensorHub 引入与文档。  
2. **上行增强**：`fe0171fb0` — 上行数据解析完善。  
3. **下行 IOC POC**：`3e93e8fc8` — OneWire 下行 IOC（含 `IO_ADDPOLLEX`）POC。  
4. **AIC 下行**：`f12f0819a` — 4–20mA AIC 的 `IO_DECODE` 下发。  
5. **工程与文档**：`9cf699d45`、`ecac05a5e` — `rak2560` 等构建说明与文档补充。  
6. **同步上游**：`e94e21094` — 与 `develop` 对齐，便于后续合入主线。  
7. **USB 文本 POC（本批）**：`RAKHUB …` + `rakhub_usb_poc.py` + `slot=` 多任务 + Profile 桩 — 见 § 一 表格首行（提交后补 hash）。

---

## 四、更新本文件时使用的命令

```bash
cd D:\Git_code\Meshtastic\firmware
git log --oneline -30
git log --format="%h %ad %s" --date=short -30
```

将新提交按「一、本分支功能演进」表格追加或合并说明即可。

---

## 变更记录（维护本 md 文件本身）

| 日期 | 说明 |
|------|------|
| 2026-05-13 | 增加 USB POC 提交占位行、§ 三 条目 7。 |
| 2026-05-12 | 初稿；入口 README 指向本文件。 |
