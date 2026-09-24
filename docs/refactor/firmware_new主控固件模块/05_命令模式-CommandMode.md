# 模块 05 — 命令模式（`CommandMode` 6 模式 + `#CMDMODE` + cmdModeNames）

> **范围**：命令模式枚举定义 + `#CMDMODE` ASCII 命令 + 6 模式 switch 分支
> **关联文件**：
> - `Robot/instances/dummy_robot.cpp` (`CommandMode` 枚举定义 + `SetCommandMode` 实现 + switch case)
> - `Robot/instances/dummy_robot.h` (`DEFAULT_COMMAND_MODE` + `commandMode` 字段)
> - `UserApp/main.cpp` (`char cmdModeNames[6][4]` 数组)
> - `UserApp/protocols/ascii_protocol.cpp` (`#CMDMODE` 分支)

---

## 模块概述

机械臂有 6 种"命令模式"（COMMAND_MODE），决定 `>` `&` `@` 进入哪个 switch 分支：

| 枚举值 | 名称 | ASCII | 行为 |
|---|---|---|---|
| 1 | SEQUENTIAL | `SEQ` | 阻塞回 ok |
| 2 | INTERRUPTABLE | `INT` | 立即回 ok，新指令覆盖 |
| 3 | CONTINUES_TRAJECTORY | `TRJ` | 阻塞回 ok（与 SEQ 复用同一阻塞） |
| 4 | MOTOR_TUNING | `TUN` | 扫频正弦激励 |
| 5 | TORQUE_CONTROL | `TRQ` | 力矩透传 |
| 6 | SERVO_J | `SRV` | 5kHz 全速随动 |

---

## 三状态对比

| 维度 | OLD | firmware | 当前 firmware_new |
|---|---|---|---|
| 模式数量 | 4（SEQ/INT/TRJ/TUN） | 6（SEQ/INT/TRJ/TUN/TRQ/SRV） | **6**（C3 已生效） |
| cmdModeNames 数组 | `["SEQ","INT","TRJ","TUN"]` | `["SEQ","INT","TRJ","TUN","TRQ","SRV"]` | 6 元素 |
| DEFAULT_COMMAND_MODE | 1 (SEQ) | 1 | 1 |
| `#CMDMODE` 命令 | 支持 | 支持 | 支持 |
| SERVO_J 实际下发 | 不支持 | 5kHz 高频环下发 | **switch case 已实现但下发逻辑未启用**（F1.4 待拍板） |
| TORQUE_CONTROL | 不支持 | `$` 指令 7 电流透传 | **switch case 已实现但下发逻辑未启用** |

---

## 决策

| # | 决策 | 理由 | 风险 |
|---|---|---|---|
| 1 | **`cmdModeNames` 保留 6 模式数组**（C3） | OLED 第 6 行 `[CMD] __*_*__` 中的 CMD 显示需要支持 6 个名称 | 数组扩到 6 元素，4 模式用户看到 TRQ/SRV 也能用 |
| 2 | **`#CMDMODE` 支持 6 个模式切换**（已生效） | cmdModeNames 数组 6 元素意味着 `#CMDMODE 6` 可切换 | 切换到 5/6 后发 `>` 行为未明（F1.4 关联） |
| 3 | **SERVO_J 5kHz 全速下发逻辑：暂不实现**（F1.4 待拍板） | 5kHz 实时环跑下发逻辑会增加代码复杂度，当前可用 #CMDMODE 切换但实际行为待定 | 用户切换到 6 后发 `>` 行为待明 |
| 4 | **TORQUE_CONTROL 力矩透传：暂不实现** | 同 SERVO_J：模式名/数组保留，行为待定 | 同 |
| 5 | **switch case 中 4 个真正执行的模式**：当前 SEQ/TRJ/INT/TUN 是主要的，TUN/TRQ/SRV case 体实现需审查 | 待审查 firmware_new 中 TUN/TRQ/SRV case 实现是否完整 | 若不完整需回退或补全 |

---

## 实施

### F1.4 — `#CMDMODE 5/6` 行为明确（关联模块 01）

**位置**：`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp` 中 `SetCommandMode` + switch case（约 line 686-）

**待用户拍板**：
- **选项 A**：保留当前 6 模式 switch 实现
  - `#CMDMODE 5` → `COMMAND_TORQUE_CONTROL`：发 `>` 走 `$` 解析
  - `#CMDMODE 6` → `COMMAND_SERVO_J`：发 `>` 走 5kHz 实时环下发
- **选项 B**：TUN 保留，TRQ/SRV 未实现
  - `#CMDMODE 5` → 回 `error: mode 5 not implemented`
  - `#CMDMODE 6` → 回 `error: mode 6 not implemented`
- **选项 C**：`#CMDMODE 5/6` 当成 SEQ 处理（无差异化）

### 现状代码（已生效部分）

**`cmdModeNames[6]` 数组**（`main.cpp:154`）：
```cpp
char cmdModeNames[6][4] = {"SEQ", "INT", "TRJ", "TUN", "TRQ", "SRV"};
```

**switch case 6 模式**（`dummy_robot.cpp:684-`）：完整支持，与 firmware 一致

**`SetCommandMode` 范围检查**（line 686-687）：
```cpp
if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL || _mode > COMMAND_TORQUE_CONTROL)
    return;
```

**禁止事项**：
- ❌ 不要把 cmdModeNames 缩回 4 元素（C3 决策）
- ❌ 不要让 `SetCommandMode` 拒绝 5/6（当前已接受 1~6）

---

## 验证

| 测试 | 期望 | 命令 |
|---|---|---|
| 6 模式 cmdModeNames 数组 | 6 行编译通过；OLED 显示 SEQ/INT/TRJ/TUN/TRQ/SRV | `grep "cmdModeNames"` |
| `#CMDMODE 1~6` 切换 | `commandMode` 字段变化 | `#CMDMODE 1` ... `#CMDMODE 6` |
| OLED 显示 6 个模式名 | 实际切换后显示对应字符串 | OLED 观察 |
| `#CMDMODE 1~4` 行为 | 已确认（SEQ 阻塞 + INT 立即 + TRJ 阻塞 + TUN 扫频） | 各个模式发 `>` 验证 |
| `#CMDMODE 5/6` 行为 | 待拍板 | 见 F1.4 |

---

## 状态

- [x] 决策已记录（2026-09-24）
- [x] cmdModeNames 6 元素已生效（C3）
- [x] switch case 6 模式已生效
- [x] `#CMDMODE` 1~6 切换已生效
- [ ] F1.4 用户拍板 5/6 行为
- 无需代码修改

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [01 - MoveJ](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md) | F1.1/F1.3 阻塞依赖 SEQ/TRJ 模式；F1.4 TRQ/SRV 行为关联 |
| [06 - OLED显示](firmware_new主控固件模块06_OLED显示.md) | cmdModeNames 是 OLED 第 6 行 CMD 字符串来源 |
| [13 - 架构常量](firmware_new主控固件模块13_架构常量.md) | `DEFAULT_COMMAND_MODE = 1` |
