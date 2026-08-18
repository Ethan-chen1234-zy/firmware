# RAK SensorHub 分支版本更新说明

**入口文档**：开发者请先阅读仓库根目录 **`README(Add_RAKSensorHub).md`**（主文件索引、构建宏、各能力用法）；本文件仅保留 **Git 提交级** 变更表。

**分支**：`feature/add_RAKSensorHub`  
**生成方式**：根据 `git log` 整理；§〇-B 为 2026-08-17 **GE JSON 下行**（RAK9154 + RK300-03），已提交 `bcd835654`。

---

## 〇、发行物与 tag（非「新提交」，仅追溯二进制）

| Tag | Commit | 说明 |
|-----|--------|------|
| `v2.7.21-raksensorhub-poc-uplink` | `bc893370c` | **上行 POC**：`rak2560` 构建产物与 SHA256 见 `releases/uplink-poc/v2.7.21-raksensorhub-poc-uplink/README-RELEASE.md`；发布流程见 `doc/RAKSensorHub_Release_Uplink_POC.md`。UF2 通过 **GitHub Release 附件**分发（`releases/**/*.uf2` 已 `.gitignore`）。 |

---

## 〇-B、GE JSON 下行（2026-08-17，commit `bcd835654`）

共用基础设施（USB `RAKHUB`、`DOWNLINK_AUTO=0`、`rakhubUsbFeedByte`）见下表。两条传感器都是 ProbeIO **GE** 轮询表，不是 core 内置驱动。

### 1. RAK9154（battery_lite）

验收：`bin/battery_lite_core-1.2.27.json` → 从站 **`0x6E`**，四路 `IO_ADDPOLL`：

| 任务 | hex | IPSO | 含义 | Hub 处理 |
|------|-----|------|------|----------|
| 1 | `6e0360000001` | 186 (`0xBA`) dtype=6 scale=0.01 | 电压 | EnvironmentMetrics，日志 `Battery voltage: 12.88 V` / `10.78 V` |
| 2 | `6e0360010001` | 185 (`0xB9`) dtype=4 scale=0.01 | 电流 | EnvironmentMetrics |
| 3 | `6e0360020001` | 184 (`0xB8`) dtype=6 scale=1 | SOC | 本地 `HubPower.percent`（HAS_RAKHUB 不走 DeviceMetrics） |
| 4 | `6e0360090001` | 103 (`0x67`) dtype=4 scale=1 | 温度 | EnvironmentMetrics，约 `26°C` |

注意：这与 `app_start.c` 里 `dtype=='RS'` 的 **内置 `do_RAK9154_*`** 不是同一条路径。JSON 是把 9154 当通用 Modbus 设备配进 GE。`IO_PSM` / `SNSR_CONF` / `PRB_DEL` 由脚本忽略，Hub APPLY 只发 `IO_CFG` + `IO_ADDPOLLEX` + `IO_ENABLEPOLL`。9154 本体不用再重启；ProbeIO 软重启是 APPLY 清表流程的一部分。

### 2. RK300-03 CO2

验收：`bin/rk300_03_core-1.2.27.json` → 从站 **`01`**，`010300000001`，IPSO 125 (`0x7D`) → `7D D9 03` = **985 ppm**（小端）。走 AirQuality，不是 Environment。从站 `02` 读不到。

### 应保留（有用）

| 文件 | 改动 | 为什么留 |
|------|------|----------|
| `variants/nrf52840/rak2560/platformio.ini` | `DOWNLINK_POC=1` `TEMPLATE=0` **`DOWNLINK_AUTO=0`** `USB_PROFILE=1` | Hub 重启不再自动 clear ProbeIO EEPROM；配置只走 USB `RAKHUB APPLY` |
| `RAKSensorHub.cpp` `scheduleDownlinkPoc()` | `AUTO=0` 时跳过 join 自动调度 | 与上配套；APPLY 的 clear→reboot→config 仍会走 |
| `StreamAPI.cpp` + `RAKSensorHubProfile.h` | 非 protobuf START1 字节转 `rakhubUsbFeedByte()` | SerialConsole 原先吃掉 `RAKHUB` 行，JSON APPLY 经常只写到 slot 0 |
| `RAKSensorHub.cpp` USB | 抽出 `rakhubUsbFeedByte` | 给 StreamAPI 喂字节 |
| `bin/rakhub_usb_poc.py` | 单任务 JSON 也发 `slot=0` | 省略 slot 会 auto-append；上次 `DEV_ADDR=02` 会变成 `tasks=2` |
| `bin/rk300_03_core-1.2.27.json` | GE 模板，从站 `01`，IPSO 125 | RK300-03 已验证 985 ppm |
| `bin/battery_lite_core-1.2.27.json` | GE 模板，从站 `0x6E`，IPSO 186/185/184/103 | RAK9154 已验证电压/SOC/温度 |
| `parseCo2Ipso()` | SDATA/REPORT 共用；数值变化才 `LOG_INFO` | 985 ppm 验收点；避免 get.data 每秒刷屏 |
| REQ/RSP、Status 改 `LOG_DEBUG`；CO2 不再打 `+EVT:IPSO[7d]` | 降噪 | 与「取消无关日志」一致 |

### 调试过程中已删掉（不要再加回来）

- 每帧 `SDATA IPSO[7d] raw(len=3): 7D xx xx`
- 空槽 `CO2 … raw=0 (ProbeIO empty …)`
- `--subst DEV_ADDR=02` 试验（传感器从站是 **01**）

### 可选 / 偏防御（RK300 实际用不上）

- `parseCo2Ipso` 大端回退：本次成功帧是小端 `D9 03`=985，BE 分支未触发。可留作其它 ProbeIO 打包容错。

### 操作结论（不是代码）

- 9154 与 RK300-03 在 core-1.2.27 都走 ProbeIO **GE** + EEPROM 轮询，不是 `app_start.c` 内置 `do_RAK9154_*` / 无 RK300 驱动。
- APPLY 先清再写；写完等 `queued all` 后再等一个 `period`（60s）。
- Hub 闪存里的 USB override 掉电丢失；ProbeIO 任务在 EEPROM，`AUTO=0` 时 Hub 重启不会擦。

---

## 一、本分支功能演进（新 → 旧）

| 提交 | 日期 | 说明 |
|------|------|------|
| `bcd835654` | 2026-08-17 | GE JSON 下行：`DOWNLINK_AUTO=0`、`StreamAPI`→`rakhubUsbFeedByte`、`battery_lite` / `rk300_03` JSON、CO₂ 解析与日志降噪。详见 §〇-B。 |
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
| 2026-08-18 | P0–P2 文档同步：架构 / 传感器列表 / 任务计划 / Sprint 2 验证 / 上行发布差异。§〇-B 对应 commit `bcd835654`。 |
| 2026-08-17 | RAK9154 GE + RK300-03 GE 验收；`DOWNLINK_AUTO`；USB `rakhubUsbFeedByte`。见 §〇-B。 |
| 2026-05-13 | 增加 USB POC 提交占位行、§ 三 条目 7。 |
| 2026-05-12 | 初稿；入口 README 指向本文件。 |
