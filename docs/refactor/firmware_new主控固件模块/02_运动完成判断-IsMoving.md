# 模块 02 — 运动完成判断（`IsMoving()` + `jointsStateFlag`）

> **范围**：判定"电机是否到位"的核心公式 + 状态标志更新机制
> **关联文件**：
> - `Robot/instances/dummy_robot.h` (`IsMoving()` 声明 + `jointsStateFlag` 字段 + `ALL_FINISH_FLAG` 常量)
> - `Robot/instances/dummy_robot.cpp` (`IsMoving()` 实现 + `UpdateJointAnglesCallback` 写 flag)
> - `UserApp/protocols/can_protocol.cpp` (`case 0x23` 回包 → `UpdateAngleCallback` → `UpdateJointAnglesCallback`)

---

## 模块概述

`IsMoving()` 是运动控制回路的"心脏"：阻塞回 OK（模块 01）、Homing/Resting 完成判定、`!DISABLE` 后 IsMoving 短路等都依赖它。

**核心字段**：
- `volatile uint8_t jointsStateFlag = 0b00000000;`：每 bit 对应一个轴的 FINISH 状态（bit i=1 表示 J{i+1} 已到位）
- `static constexpr uint8_t ALL_FINISH_FLAG = 0x7E;`（= `0b01111110`，数值上等同 OLD 的 `0b1111110`）：所有 6 关节 FINISH 时的 flag 值

**核心公式**（3 个候选）：
1. **OLD 公式**：`return jointsStateFlag != 0b1111110;`（等价于 `!= ALL_FINISH_FLAG`）
2. **firmware v3.2 公式**：`if (!isEnabled) return false; return jointsStateFlag != ALL_FINISH_FLAG;`
3. **OLD + flag 复位**（OLD + MoveJ 时 `flag = 0`）：纯 OLD 公式，依赖 MoveJ/ServoJ 中的 `jointsStateFlag = 0` 复位

---

## 三状态对比

| 维度 | OLD | firmware（v3.2） | 当前 firmware_new |
|---|---|---|---|
| IsMoving 公式 | `flag != 0b1111110` | `if (!isEnabled) return false; flag != ALL_FINISH_FLAG` | **v3.2 公式**（F5 待回退） |
| 公式中 mask 字面量 | `0b1111110`（bit1~6） | `ALL_FINISH_FLAG = 0x7E`（= 0b01111110） | `ALL_FINISH_FLAG = 0x7E`（数值等同） |
| `!isEnabled` 短路 | 无 | 有（避免 SEQUENTIAL 队列卡死） | 有（F5 待删除） |
| jointsStateFlag 复位 | MoveJ 时 `flag = 0` | MoveJ/ServoJ 时 `flag = 0` | 同 firmware（已实施） |
| EmergencyStop 后 | 不置 flag（依赖回包自然更新） | `flag = ALL_FINISH_FLAG` 立即短路 | 同 firmware（已实施） |
| 7 轴 vs 6 轴 | 仅 6 关节 bit1~6 | 7 轴 + 夹爪 + 地轨机制（地轨 bit0 恒 0） | 同 firmware |
| UpdateJointAnglesCallback 处理范围 | `for (i=1; i<=6; i++)` | `for (i=1; i<=6; i++)`（仍只 6 关节） | 同 firmware |

---

## 决策

| # | 决策 | 理由 | 风险 |
|---|---|---|---|
| 1 | **`IsMoving()` 公式回归 OLD `jointsStateFlag != 0b1111110`**（F5） | 用户 P0 决策 D1：不做 v3.2 的 `!isEnabled ? false` 短路 | 失能后 SEQUENTIAL 队列可能被 IsMoving=true 阻塞（需 EmergencyStop 配合） |
| 2 | **保留 `ALL_FINISH_FLAG = 0x7E` 常量**（不删除） | 公式虽不使用该常量，但 EmergencyStop() 中仍引用 | 无（数值等同 `0b1111110`） |
| 3 | **保留 MoveJ/ServoJ 中的 `jointsStateFlag = 0` 复位** | 防"电机使能后 STOP 状态把 flag 置 1"导致 IsMoving 永远 true | 无 |
| 4 | **保留 EmergencyStop 中 `jointsStateFlag = ALL_FINISH_FLAG`** | 物理抱闸 + 短路 IsMoving，避免 !STOP 后 SEQUENTIAL 队列卡死 | OLD 是依赖回包自然更新，更慢；当前更快更安全，保留 |
| 5 | **`UpdateJointAnglesCallback` 保持 `for (i=1; i<=6)`** | OLED 与 IsMoving 都只看 6 关节（地轨独立下发） | 无 |

> **范围说明**：Homing/Resting 30s 兜底超时回退已并入模块 01 §6.3（当前重构范围）。模块 02 实施章节仅覆盖 `IsMoving()` 公式本身，不重复展开。

---



## 实施

### F5 — `IsMoving()` 函数体回退

**位置**：`Robot/instances/dummy_robot.cpp:666-671`

**当前代码**（v3.2 失能短路）：
```cpp
bool DummyRobot::IsMoving()
{
    // [v3.2] 失能短路
    if (!isEnabled) return false;
    return jointsStateFlag != ALL_FINISH_FLAG;
}
```

**替换为**（OLD 严格公式）：
```cpp
bool DummyRobot::IsMoving()
{
    // [F5] 严格 OLD 公式：不做短路，字面量与原代码 1:1
    return jointsStateFlag != 0b1111110;
}
```

**配套保留**（F4 决策）：
- `ALL_FINISH_FLAG = 0x7E` 常量保留（头文件 line 281 保留）
- `EmergencyStop()` 中 `jointsStateFlag = ALL_FINISH_FLAG` 保留（line 750）
- `MoveJ / ServoJ` 中 `jointsStateFlag = 0` 复位保留（line 381, 426）

**禁止事项**：
- ❌ 不要再加 `!isEnabled` 短路（P0 明确禁止）
- ❌ 不要改 mask 字面量为 `ALL_FINISH_FLAG`（保留 `0b1111110` 字面量与原代码 1:1，可读性更好）

---

> **本节范围说明**：模块 02 仅覆盖 `IsMoving()` 公式本身。`Homing()` / `Resting()` 函数中的 30s 兜底超时回退，**已在模块 01 §6.3 中完整记录**（属于当前重构范围的一部分）。

## 验证

| 测试 | 期望 | 命令 |
|---|---|---|
| 全部到位返回 false | flag = 0b01111110 | 7 电机到位（OLED 看 6 个 `*`） |
| 任意轴未到位返回 true | flag ≠ 0b01111110 | 发 `>` 期间查 flag |
| 失能后 IsMoving 行为 | 不短路（依赖 jointsStateFlag 实际值） | `!DISABLE` 后看 IsMoving() |
| 急停后 IsMoving 立即 false | flag = 0x7E 强制短路 | `!STOP` 后看 IsMoving() |
| 阻塞循环自然退出 | 6 个 `*` 后退出 while | 发 `>` 后 OLED 观察 |

> Homing/Resting 删 30s 兜底超时后的行为验证见 [模块 01 §6.3](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md)。

---

## 状态

- [x] 决策已记录（2026-09-24）
- [x] **2026-09-27 落地**：F5 IsMoving() 公式回退 OLD 严格公式（dummy_robot.cpp:651-659）
- [x] 编译验证（2026-09-27 ninja 201/201 全通过，FLASH 36.64%, RAM 40.70%, CCMRAM 100%）
- [ ] 实测验证（OLED + 串口）

> Homing/Resting 兜底超时删除的代码改动登记在模块 01 §6.3 状态栏。

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [01 - MoveJ](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md) | F1.1/F1.3 阻塞循环依赖 IsMoving 返回 false |
| [03 - UpdateAngle](firmware_new主控固件模块03_关节角度同步-UpdateAngle.md) | jointsStateFlag 更新靠 0x23 回包 |
| [04 - EStop](firmware_new主控固件模块04_急停-EStop.md) | EmergencyStop 中强制 flag = ALL_FINISH_FLAG；**F5 决策（IsMoving 无短路）下本机制是释放 SEQUENTIAL 阻塞循环的唯一手段，不可省略（详见模块 04 决策 #3/#5）** |
| [06 - OLED显示](firmware_new主控固件模块06_OLED显示.md) | OLED 第 6 行直接读 jointsStateFlag |
