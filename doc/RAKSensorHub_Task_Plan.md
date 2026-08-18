# RAKSensorHub 任务计划

> **英文完整版**：[RAKSensorHub_Task_Plan.en.md](RAKSensorHub_Task_Plan.en.md)（1.0，2026-08-18）  
> **开发基线**：单体 monolith @ `v2.7.25-raksensorhub-poc-monolith-pre-split`  
> **拆分留档**：本地 tag `v2.8.0-raksensorhub-poc-split`（暂不合并主线）

## 优先级（当前）

| 优先级 | 任务 | 状态 |
|--------|------|------|
| **P0** | D1-5 映射表 | ✅ 见上行列表 §4 |
| **P0** | D1-4 proto 24–26 | ✅ |
| **P0** | D1-1 getMetrics 土壤三要素 | ✅ |
| **P0** | D2-5 IOC 等 RSP | 🔄 已改超时逻辑；GE APPLY 台架仍见 `SEQUCE`，以 `IOC RSP` 为准 |
| **P0** | GE JSON 下行 + 9154/CO2 台架 | ✅ `battery_lite` / `rk300_03`；`DOWNLINK_AUTO=0`；`rakhubUsbFeedByte` |
| **P0** | `rakhub_usb_poc.py` hub-ready | ✅ `--hub-ready-sec` |
| **P0** | D1-7 DI/DO 上行导出（mesh） | ✅ 已导出并有日志（见 `EnvironmentTelemetry` Send/Received） |
| P1 | D1-2 / D1-6 更多 IPSO | 未开始 |
| P1 | D2-4 业务 JSON 库 | 🔄 已有 9154 / RK300-03 GE JSON；水质/NPK 仍缺 |
| P2 | D2-6 / D2-7 / CLI 集成 | 未开始 |

## 变更记录

| 版本 | 日期 | 说明 |
|------|------|------|
| **1.0** | 2026-08-18 | P0 文档同步：GE JSON 下行 + RAK9154 / RK300-03 台架；见 CHANGELOG §〇-B、Sprint 2 验证 |
| **0.9** | 2026-05-22 | P0 代码+文档；验证见 [RAKSensorHub_POC_Sprint1_Verification.md](RAKSensorHub_POC_Sprint1_Verification.md) |
| 0.8 | 2026-05-19 | 切回 v2.7.25 monolith；v2.8.0 仅本地 tag |
| 0.7 | 2026-05-19 | 多文件拆分完成 |
| 0.6 | 2026-05-15 | DI reboot、部分 D2-5 |

任务 ID：**D1-\*** 数据层上行，**D2-\*** 配置层下行。
