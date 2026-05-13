# Release: `v2.7.21-raksensorhub-poc-uplink` — RAK2560 上行 POC

| 项 | 值 |
|----|-----|
| **Git tag** | `v2.7.21-raksensorhub-poc-uplink` |
| **Commit** | `bc893370c1e92a04b302d46d9b72d88117bd7e99` |
| **PlatformIO env** | `rak2560` |
| **构建方式** | `git worktree` 在 tag 上 detached 构建（见 `doc/RAKSensorHub_Release_Uplink_POC.md`） |
| **构建日期** | 2026-05-13（本地） |

## 二进制文件

| 本地稳定文件名 | 说明 |
|----------------|------|
| `firmware-rak2560-v2.7.21-raksensorhub-poc-uplink-bc89337.uf2` | 自 `firmware-rak2560-2.7.22.bc89337.uf2` 复制重命名（PROGNAME 以构建输出为准） |

**SHA256**（复制后文件）：

```
C483CE55A508D54A2DDCA3169B5C387892A21115FFE30459F08BE7E6817BCB66
```

**MD5**（PlatformIO 构建输出记录）：

```
38ffb11105d86a8dfd8cef54c7f28eb8
```

## 功能范围（该 tag）

- `HAS_RAKHUB=1`，OneWire / RAKSensorHub 上行路径。
- **不含** `RAK_SENSORHUB_DOWNLINK_POC`、**不含** `RAK_SENSORHUB_USB_PROFILE`（无 Hub→Probe USB `RAKHUB` 下行配置 POC）。

## 分发

- 推荐：将本目录下 **UF2**（或 zip）作为 **GitHub Release** 附件上传到 tag `v2.7.21-raksensorhub-poc-uplink` 对应 Release。
- 本仓库默认 **不** 将 `.uf2` 强制加入 Git 索引；若需纳入版本库，请使用 **Git LFS** 或团队约定目录。
