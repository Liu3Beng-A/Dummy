# TODO — Dummy 7轴机械臂控制项目

> 本文档追踪**计划开发**的功能与待优化项。
> 已修复的 P0~P3 问题见 `ISSUES.md`。

---

### [FEAT-1] 新增「回位」指令 🟡

**状态**：待开发（2026-09-01）

**功能描述**：
新增一条 `!BACKHOME` 指令（或在 UI 提供一个「回位」按钮），统一触发各轴归位行为：

| 轴 | 回位行为 |
|---|---|
| J1~J6 关节 | 回到 `!RESET` 位置（即 `MoveJ(0,0,90,0,0,0,0,10)`） |
| 地轨（ID=9） | 触发归零程序：一直向一边移动直到触发限位 → 记录当前位置为 250mm |
| 夹爪（ID=8） | 执行闭合动作 → 记录当前位置为 0 |

**设计考量**：
- 三轴独立触发，不需要互相等待（异步执行）
- UI 端发送 `!BACKHOME` 后，各轴分别发送各自的归零/归位命令
- 地轨归零：可参考 `!HOME` 现有逻辑，确认限位触发机制
- 夹爪归零：闭合到位后发 `HAND_SAVE_ZERO`（或类似）命令记录零点

**相关代码**：
- `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp` — 新增 `!BACKHOME` handler
- `串口助手.py` — UI 按钮和命令发送逻辑
- 地轨电机固件 — 归零程序（触发边界后记录位置）
- 夹爪电机固件 — 闭合 + 零点记录

---

### [BUG-2] `!HOME` / `!RESET` 执行中 `!STOP` 急停无效 🔴

**状态**：待修复（2026-09-01）

**现象**：
- 执行 `!HOME` 或 `!RESET` 时（此时 `Homing()` / `Resting()` 中的 `while(IsMoving())` 等待循环正在运行）
- 发送 `!STOP` → 急停命令到达，`EmergencyStop()` 被调用，`"ok"` 立即返回
- 但机械臂继续运动到 home/rest 姿态才停下，`!STOP` 毫无作用

**根因**：`IsMoving()` 使用**位置误差**判断运动状态，完全忽略电机真实 `state` 字段。

`DummyRobot::IsMoving()`（`dummy_robot.cpp:679`）：
```
return fabsf(motorJ[i]->angle - motorJ[i]->targetAngle) > EPSILON_DEG
```
- `targetAngle` 在 `MoveJ()` / `MoveJoints()` 中被设为**新目标**
- `angle` 是从电机 CAN 反馈的**实测角度**（存在 CAN 通信延迟）
- 当新目标与当前位置差值 < 1° 时，`IsMoving()` **立即返回 false**，退出 `while` 循环

崩溃路径（以 `Homing()` 为例）：
1. `MoveJ(0,0,90,...)` 下发新目标 → `targetAngle` 更新
2. `while(IsMoving())` 检查：`fabsf(angle - targetAngle)` → 若 < 1° → **立即退出循环**
3. `Homing()` 函数返回，机械臂还在运动中
4. 后续任何命令（`!STOP` 等）都直接执行，`ClearFifo()` 已在函数结束后调用，无效

`EmergencyStop()` 的问题更大：
```
context->MoveJ(context->currentJoints.a[0], ...);  // ← 设 targetAngle = angle（误差归零）
context->isEnabled = false;
ClearFifo();  // ← while(IsMoving()) 根本不会进入，此行在电机还在跑时就被跳过
```

**`CtrlStepMotor::state` 字段被完全忽略：**
- `UpdateAngleCallback()` 从电机 CAN 反馈中更新 `state`（`RUNNING`/`FINISH`/`STOP`）
- 但 `IsMoving()` 从不检查这个字段

**修复方向**：
1. `IsMoving()` 同时检查 `state == RUNNING`（最可靠）
2. `EmergencyStop()` 等待所有电机 `state != RUNNING` 后再 `ClearFifo()`
3. 或：`Homing()` / `Resting()` 中的 `while(IsMoving())` 改为等待 `state` 全部非 `RUNNING`

**相关代码**：
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:679` `DummyRobot::IsMoving()`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:585` `DummyRobot::Homing()`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:603` `DummyRobot::Resting()`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:760` `DummyRobot::CommandHandler::EmergencyStop()`
- `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.hpp:13` `CtrlStepMotor::State`（`RUNNING`/`FINISH`/`STOP`）
- `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` `UpdateAngleCallback()`（更新 state 字段）

---

### [BUG-3] `!STALL_STATUS` 响应硬编码、格式/通道不匹配 🔴

**状态**：待修复（2026-09-01，发现于现场实测）

**现象**：
- 发送 `!STALL_STATUS` 后，串口只看到 `[STALL_STATUS] node=X en=Y` 格式的电机回包（部分节点缺失）
- 串口助手 `串口助手.py:1348` 的 `ok STALL_STATUS` 拦截逻辑匹配不到任何响应
- 实际收到的"en=1 / lock=0"全部是**硬编码假数据**，无法反映电机真实状态
- 串口助手 UI 状态无法正确显示哪些电机堵转保护开了、哪些处于 LOCKED

**根因**：
`ascii_protocol.cpp:172-173` 硬编码返回 en/lock 值，没收集电机真实响应：

```cpp
dummy.QueryStallStatus();   // 发 14 条 CAN 0x5C 单播
Respond(_responseChannel, "ok STALL_STATUS rail_en=1 j1_en=1 ... j6_en=1 \\");
Respond(_responseChannel, "                 rail_lock=0 ... j6_lock=0");
```

而真正的电机回包（`can_protocol.cpp:197-200` 和 L313-317）走 `printf`：

```cpp
case 0x5C:
    printf("[STALL_STATUS] node=%d en=%d\r\n", id, qval);   // 走 UART4 通道
    printf("[STALL_STATUS] node=%d lock=%d\r\n", id, qval);
```

**两套路径、两套格式、两个通道**：

| 来源 | 格式 | 通道 |
|------|------|------|
| ascii_protocol.cpp 硬编码 | `ok STALL_STATUS rail_en=1 ...` | USB |
| can_protocol.cpp printf | `[STALL_STATUS] node=9 en=1` | **UART4** |

文档设计（`重构方案—堵转检测重构需求.md` L246）期望的格式是单条 `ok STALL_STATUS rail_en=... j1_en=... ... rail_lock=...`，但当前实现**没按文档**。这与 §8.1 P1 已知未修项一致："目前返回硬编码状态字符串，未收集电机实际响应"。

**修复方向**：
1. **方案 A（推荐）**：放弃 `Respond` 那两行硬编码，全部由 `can_protocol.cpp` 的 printf 输出（UART4 通道），并把格式改为用户期望的 `ok STALL_STATUS rail_en=... j1_en=... j6_en=1 \r\n                 rail_lock=0 ... j6_lock=0`
2. **方案 B**：在主控侧加缓存，`QueryStallStatus()` 等待所有 7×2=14 个回包后再用 `Respond` 输出真实状态字符串（需要 CAN 异步响应同步机制，改动大）
3. **方案 C**：临时方案——串口助手改成解析 `[STALL_STATUS] node=X en=Y` 格式

**相关代码**：
- `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp:163-174` `OnUsbAsciiCmd` 中 `!STALL_STATUS` handler
- `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp:893-904` `OnUart4AsciiCmd` 同名 handler（USB/UART 两份要同步改）
- `firmware/ref_core_f405/UserApp/protocols/can_protocol.cpp:195-201` 地轨 (id==9) 0x5C 回包处理
- `firmware/ref_core_f405/UserApp/protocols/can_protocol.cpp:312-318` J1~J6 (id 1~6) 0x5C 回包处理
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:574-584` `DummyRobot::QueryStallStatus()`
- `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp:424` `CtrlStepMotor::QueryStallStatus()`
- `串口助手.py:1348` `ok STALL_STATUS` 拦截逻辑
- `串口助手.py:1426` `_update_stall_status_from_response` 解析逻辑

---
