# 模块 07b — RGB 状态机（3 个灯效预设状态码 + 旧 `SetRGBEnable`）

> **范围**：机械臂状态 ↔ 灯效映射（启动默认 / 使能 / 失能 三态码）
> **关联文件**：
> - `Robot/instances/dummy_robot.h` (`rgbStateStart/Enable/Disable` 字段)
> - `Robot/instances/dummy_robot.cpp` (`SetRGBMode` / 旧 `SetRGBEnable`)
> - `UserApp/protocols/ascii_protocol.cpp` (`!RGB_SET_*` 命令入口)
> - `Bsp/gpio/rgb.cpp/.hpp` (RGB LED 硬件驱动)

> **说明**：本模块只关注"用哪个灯效码"；**怎么配置灯效码**见模块 [07a - 灯效配置](firmware_new主控固件模块07a_灯效配置.md)，**怎么持久化**见模块 [16 - EEPROM](firmware_new主控固件模块16_EEPROM配置持久化.md)。

---

## 模块概述

机械臂有 3 个抽象"灯效状态码"，用户可通过 `!RGB_SET_*` 配置每个状态对应的灯效：

| 状态字段 | 触发时机 | 默认值（如未配置） |
|---|---|---|
| `rgbStateStart` | 开机时（无使能状态） | 流水/呼吸 |
| `rgbStateEnable` | `!START` 后电机使能 | 绿色（健康） |
| `rgbStateDisable` | `!DISABLE` 后电机失能 | 红色（警告） |

OLD 固件只有"RGB 开关"（`SetRGBEnable(bool)`），新固件升级为"3 个灯效预设状态码"。

---

## 三状态对比

| 维度 | OLD | firmware | 当前 firmware_new |
|---|---|---|---|
| 灯效控制机制 | `SetRGBEnable(bool)` / `GetRGBEnabled()` | `rgbStateStart/Enable/Disable` 3 状态码 | **3 状态码已生效** |
| 决策 §0.2 D3 | — | — | 建议删除旧 SetRGBEnable，**当前保留**（决策未定） |
| 用户配置入口 | 无 | `!RGB_SET_START/ENABLE/DISABLE` | 保留 |
| 字段类型 | bool（启/停） | uint8_t（灯效码 0~9） | 3 个 uint8_t 字段 |

---

## 决策

| # | 决策 | 理由 | 风险 |
|---|---|---|---|
| 1 | **保留 3 灯效预设状态码机制**（rgbStateStart/Enable/Disable） | 比单纯的 bool 开关更灵活 | 无（已生效） |
| 2 | **决策 §0.2 D3 待定**：删除旧 `SetRGBEnable(bool)` / `GetRGBEnabled()` | 当前 firmware_new 同时保留新旧两套机制，需要选一套 | 选 A 删旧；选 B 全留 |
| 3 | **`SetRGBMode(int)` 用于实时切模式** | 调试时临时切灯效，不用持久化 | 无 |

---

## 实施

### 待用户拍板（P2-2）

**当前状态**（firmware_new）：
- 3 灯效预设状态码机制已存在（`rgbStateStart/Enable/Disable`）
- 旧 `SetRGBEnable(bool)` / `GetRGBEnabled()` **也保留**
- 实质上有 2 套灯效控制 API

**选项 A**（删除旧）：
- 仅保留 3 状态码机制
- 删除 `SetRGBEnable(bool)` / `GetRGBEnabled()`
- `!START` 触发的"使能灯效"通过 `rgbStateEnable` 实现
- `!DISABLE` 触发的"失能灯效"通过 `rgbStateDisable` 实现
- 风险：需要改使能/失能代码路径

**选项 B**（保留双套）：
- 不动代码，2 套机制并存
- 风险：`!START` 时旧 bool=true 给绿色 + 新 `rgbStateEnable` 给配置的灯效，可能冲突

**选项 C**（折中）：
- 旧 `SetRGBEnable` 作为"是否启用灯效系统"的开关（不指定灯效码）
- 3 个状态码作为"用哪个灯效"
- 这样 bool 控制系统启停，码决定具体样式

**禁止事项**：
- ❌ 不要在不清楚二选一之前直接修改代码（决策未定）

---

## 验证

| 测试 | 期望 | 命令 |
|---|---|---|
| 开机默认灯效 | RGB 显示 `rgbStateStart` 对应的灯效 | 重启 + OLED 观察 |
| 使能灯效 | `!START` 后 RGB 变 `rgbStateEnable` | 配置 + `!START` |
| 失能灯效 | `!DISABLE` 后 RGB 变 `rgbStateDisable` | 同上 |
| 实时切模式 | `SetRGBMode` 调用后立即变 | `SetRGBMode(3)` |
| 旧 SetRGBEnable | 待定 | 待选 A/B/C |

---

## 状态

- [x] 决策已记录（2026-09-24）
- [x] 3 灯效预设状态码机制已生效
- [ ] P2-2 用户拍板（保留 vs 删旧 vs 折中）

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [07a - 灯效配置](firmware_new主控固件模块07a_灯效配置.md) | `!RGB_SET_*` 命令写到 3 个状态码字段 |
| [16 - EEPROM](firmware_new主控固件模块16_EEPROM配置持久化.md) | 状态码持久化 |
| [10 - PID查询](firmware_new主控固件模块10_PID查询.md) | 当前 OLED 心跳探针含 RGB tick，与本模块相关 |
