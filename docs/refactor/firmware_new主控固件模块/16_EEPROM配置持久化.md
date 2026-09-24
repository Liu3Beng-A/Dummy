# 模块 16 — EEPROM 配置持久化（节点 ID + 灯效配置 + `_persist` 可选）

> **范围**：主控侧非易失存储：节点 ID / 灯效配置 / 可选持久化的电机参数
> **关联文件**：
> - `Bsp/memory/emulated_eeprom.cpp`（EEPROM 模拟层）
> - `UserApp/protocols/ascii_protocol.cpp`（调用 `SaveConfig()`）
> - `Robot/actuators/ctrl_step/ctrl_step.cpp` (`_persist` 变体)

---

## 模块概述

主控侧需要持久化的数据：

| 数据 | 何时写入 | 持久化机制 |
|---|---|---|
| 节点 ID | 第一次启动 / `SetNodeID` | `SetNodeID()` 走 CAN 0x11 |
| 灯效配置（3 状态码 + 亮度 + 模式 + 颜色） | `!RGB_*` 命令 | `SaveConfig()` |
| `_persist(true)` 电机参数（电流限幅 / 加速度） | `#I_LIMIT_J ... &` / `#ACC_J ... &` | `SetCurrentLimit_persist`/`SetAcceleration_persist` |
| 电机自身参数 | 0x12/0x13/0x14 by `&` 时 | 电机端 EEPROM |

EEPROM 写入有磨损风险，需谨慎触发（默认不持久化；`&` 才是持久化）。

---

## 三状态对比

| 维度 | OLD | firmware | 当前 firmware_new |
|---|---|---|---|
| 节点 ID 写入 | 固定写 EEPROM | 同 | 同 |
| 灯效配置持久化 | N/A | `SaveConfig()` + magic=0x12345679 | **已生效** |
| 加速/电流 默认写 EEPROM | 固定写（测试频繁写） | 默认不写；`&` 才写 | **已生效**（详见模块 09） |
| `_persist(bool)` 可选接口 | 无 | `SetCurrentLimit_persist(float, bool)` | **已生效** |
| EEPROM magic | 无 | `0x12345679`（区分灯效配置区） | 保留 |
| 老化策略 | 无 | 测试时不频繁写（默认 RAM only） | 保留 |

---

## 决策

| # | 决策 | 理由 | 风险 |
|---|---|---|---|
| 1 | **保留灯效配置 `SaveConfig()` 机制** | 用户配置偏好不能丢 | 无 |
| 2 | **保留 `_persist(bool)` 可选接口** | 减少测试磨损 | 无 |
| 3 | **EEPROM magic 保留**（`0x12345679`） | 区分灯效配置和节点 ID 区 | 无 |
| 4 | **默认 RAM only + `&` 才持久化** | 平衡使用便利 + 减少磨损 | 无 |

---

## 实施

### 实施项：当前代码已生效，无需修改

**关键代码位置**：

| 文件 | 行 | 内容 |
|---|---|---|
| `Bsp/memory/emulated_eeprom.cpp` | — | EEPROM 模拟层，magic=0x12345679 |
| `ascii_protocol.cpp`（`!RGB_*` 命令） | 多处 | `SaveConfig()` 调用 |
| `ctrl_step.cpp` | `SetCurrentLimit_persist`/`SetAcceleration_persist` | 可选持久化 |

**禁止事项**：
- ❌ 不要把默认行为改为固定写 EEPROM（增加磨损）
- ❌ 不要去掉 `&` 可选持久化标志
- ❌ 不要去掉 magic=0x12345679 区分

---

## 验证

| 测试 | 期望 | 方法 |
|---|---|---|
| 灯效配置持久化 | `!RGB_SET_START 2` + 重启 → 状态保持 | 重启验证 |
| 加速配置不持久化 | `#ACC_J 1 50` + 重启 → 失效 | 重启验证 |
| 加速配置持久化 | `#ACC_J 1 50 &` + 重启 → 保持 50 | 重启验证 |
| 节点 ID 持久化 | 改 ID 后重启保持 | `SetNodeID` |
| EEPROM magic 校验 | 写入 magic=0x12345679 | 仿真测试 |
| 磨损风险 | 测试命令不触发 EEPROM 写入 | 多次发 `#ACC_J 1 50`（不带 `&`） |

---

## 状态

- [x] 决策已记录（2026-09-24）
- [x] 灯效配置 `SaveConfig()` 已生效
- [x] `_persist(bool)` 可选接口已生效
- [x] magic=0x12345679 已生效
- [x] 默认 RAM only + `&` 持久化 已生效
- 无需代码修改

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [07a - 灯效配置](firmware_new主控固件模块07a_灯效配置.md) | `!RGB_*` 调 SaveConfig |
| [07b - RGB 状态机](firmware_new主控固件模块07b_RGB状态机.md) | 3 状态码字段需 LoadConfig 初始化 |
| [09 - 电机参数配置](firmware_new主控固件模块09_电机参数配置.md) | `_persist` 配套 |
| [12 - 校准与归零](firmware_new主控固件模块12_校准与归零.md) | 0x15 ApplyPositionAsHome 同步需电机端 |
