# RAKSensorHub Sprint 1 (P0) — 验证记录

**基线**：`v2.7.25-raksensorhub-poc-monolith-pre-split` 分支单体 `RAKSensorHub.cpp`  
**日期**：2026-05-22 建立；**2026-06-02 台架验收通过**

## 1. 代码变更摘要

| 任务 | 文件 | 说明 |
|------|------|------|
| D1-4 | `protobufs/meshtastic/telemetry.proto` | `soil_conductivity`(24), `soil_ph`(25), `salinity`(26) |
| D1-4 | `src/mesh/generated/meshtastic/telemetry.pb.h` | 手工同步 nanopb 字段（Windows 无 `nanopb-0.4.9` 目录时用） |
| D1-1 | `RAKSensorHub.cpp` | `getMetrics()` 导出上述三字段 |
| D1-5 | `RAKSensorHub_Uplink_Sensor_Support_List.en.md` | §4 映射表 |
| D2-5 | `RAKSensorHub.cpp` | 下行 IOC 等待 RSP 超时 5 s，超时时不 `advanceDataPollPid()` |
| P0-5 | `bin/rakhub_usb_poc.py` | `json --apply` 前 `--hub-ready-sec`（默认 20 s） |

## 2. 自动化自测（本机执行）

| 项 | 命令 | 结果 |
|----|------|------|
| 脚本语法 | `python -m py_compile bin/rakhub_usb_poc.py` | **PASS** |
| 固件编译 | `pio run -e rak2560` | **PASS**（92 s；`firmware-rak2560-2.8.0.003fe5c.uf2`） |

## 3. 台架手测（2026-06-02 ✅ PASS）

### 3.1 测试环境
- Hub：RAK2560（节点 4487），接 JXBS-3001-EC / SDSIN，烧录含 proto 24–26 的 fork 固件
- 接收端：RAK4631（同一 fork 固件），同信道

### 3.2 Hub 发送侧日志（关键行）
```
Send: soil_conductivity=0.033000 mS/cm
Send: soil_ph=4.200000
Send: temperature=25.700001
Send packet to mesh
```

### 3.3 接收侧（4631）日志（关键行）
```
(Received from 4487): temperature=25.700001
(Received from 4487): soil_conductivity=0.033000 mS/cm
(Received from 4487): soil_ph=4.200000
```

### 3.4 验收结论

| 验收项 | 结论 |
|--------|------|
| IPSO[67] → temperature → mesh | ✅ 两端一致 25.7 °C |
| IPSO[C0] EC → soil_conductivity(24) → mesh | ✅ 两端一致 0.033 mS/cm |
| IPSO[C2] pH → soil_ph(25) → mesh | ✅ 两端一致 pH 4.20 |
| salinity(26) | ⏸ 本次无 IPSO[13] 数据，未验证 |
| D2-5 SEQUCE 改善 | ⏸ 待单独台架验证 |

## 4. 与 Task Plan 差异说明

- **D2-5** 标为 🔄：已有 IOC 步间 400 ms gap + `IOA_RSP` 释放 `awaiting_rsp`；本次仅加强超时策略，需台架确认 SEQUCE 率。
- **D1-4 regen**：`./bin/regen-protos.sh` 需仓库根目录 `nanopb-0.4.9`；当前以 **proto + pb.h 同步** 通过编译。

## 5. Protobuf 变更的标准化流程（推荐）

目标：避免“手工改 `telemetry.pb.h` 然后两台都刷同一份”的不可复现做法。标准化的**唯一真相来源**是 `protobufs/meshtastic/*.proto`，各端从同一份 `.proto` 生成代码。

### 5.1 目录与职责

| 位置 | 内容 | 备注 |
|------|------|------|
| `firmware/protobufs/` | **submodule**（上游 `meshtastic/protobufs`） | `.proto` 的来源（唯一真相） |
| `firmware/src/mesh/generated/meshtastic/*.pb.h/*.pb.cpp` | nanopb 生成产物 | **不要手改**；由 regen 生成 |

### 5.2 一次性准备（让 `regen-protos.sh` 可用）

`./bin/regen-protos.sh` 要求在 **firmware 根目录**存在 `nanopb-0.4.9/`：

```
firmware/
  nanopb-0.4.9/
    generator-bin/
      protoc
  protobufs/
  bin/regen-protos.sh
```

说明：Windows 下如果没有这套 bundle，常见替代是手工 patch `telemetry.pb.h`（POC 可行，但不推荐长期使用）。

### 5.3 标准化操作步骤（每次改 proto 都按此执行）

1. **只改 `.proto`**：`protobufs/meshtastic/telemetry.proto`（例如新增 field 24+）。
2. 更新 submodule（如有需要）：
   - `git submodule update --init protobufs`
3. 在 firmware 根目录执行生成脚本：
   - `./bin/regen-protos.sh`
4. 重新编译固件验证：
   - `pio run -e rak2560`
   - `pio run -e rak4631`
5. 台架验证：发送端/接收端日志同时出现新增字段。

### 5.4 客户端/接收端同步原则（避免“收到了但看不见”）

- **要解码并显示/缓存新增字段**的接收端固件，也必须基于**同一份 `telemetry.proto`** 生成对应的 `telemetry.pb.h`。
- 官方旧固件对未知 field 会丢弃：能收包，但不会在 `EnvironmentMetrics` 结构体里体现新增字段。

---

## 6. Sprint 2 — GE JSON 下行台架（2026-08-17 ✅）

**基线**：特性分支 `feature/add_RAKSensorHub`，commit `bcd835654`。ProbeIO **core-1.2.27**，**GE** 模式（不是 `dtype=RS` 内置驱动）。Hub：`DOWNLINK_POC=1` `TEMPLATE=0` **`DOWNLINK_AUTO=0`** `USB_PROFILE=1`。

### 6.1 代码 / 工具

| 项 | 说明 |
|----|------|
| `DOWNLINK_AUTO=0` | Hub 重启不自动 clear ProbeIO EEPROM |
| `rakhubUsbFeedByte` + `StreamAPI.cpp` | protobuf `START1` 狩猎时转发 ASCII，`RAKHUB APPLY` 不被吃掉 |
| `bin/rakhub_usb_poc.py` | JSON 每条任务都带 `slot=i`（从 0 起），覆盖 RAM 不追加 |
| `bin/battery_lite_core-1.2.27.json` | RAK9154，从站 `0x6E`，IPSO 0xBA/0xB9/0xB8/0x67 |
| `bin/rk300_03_core-1.2.27.json` | RK300-03 CO₂，从站 `01`，IPSO 0x7D |
| `parseCo2Ipso()` | 小端 ppm；仅数值变化 `LOG_INFO` |

### 6.2 命令（COM 口按本机改）

```text
python bin/rakhub_usb_poc.py --port COMx --apply json --file bin/battery_lite_core-1.2.27.json --hub-ready-sec 0 --post-apply-lines 400
python bin/rakhub_usb_poc.py --port COMx --apply json --file bin/rk300_03_core-1.2.27.json --hub-ready-sec 0 --post-apply-lines 400
```

APPLY 后：看 `IOC RSP: func=IO_ADDPOLLEX` / `IO_ENABLEPOLL`；`queued all` 后再等一个 poll period（~60 s）。`+ERR:SEQUCE` 常见，不算失败。

### 6.3 验收

| 验收项 | 结论 |
|--------|------|
| RAK9154 电压 / 电流 / SOC / 温度 | ✅ 例：`Battery voltage: 12.88 V` / `10.78 V`，温度约 26 °C |
| RK300-03 CO₂ → `AirQualityMetrics` | ✅ `7D D9 03` = **985 ppm**（小端） |
| 从站写错 `02` | ❌ `7D 00 00` 空槽；改回 `01` 后恢复 |
| Hub 重启不擦 ProbeIO 任务 | ✅ `DOWNLINK_AUTO=0` |

细节与「有用 / 可省略」文件表：`doc/RAKSensorHub_CHANGELOG.md` §〇-B。
