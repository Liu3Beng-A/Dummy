# 主控固件迁移决策（firmware → firmware_new）

> **范围**：仅主控固件（STM32F405，FreeRTOS）。
> **基线对比**：
>
> - **来源**：`firmware/ref_core_f405/` — 已实现的"新版本"
> - **目标**：`firmware_new/dummy-ref-core-fw/` — 当前待定
> - **基线**：`firmware_old/dummy-ref-core-fw/` — 旧版本（6 轴）
> **架构差异**：旧 6 轴架构 → 新 6 轴 + 地轨 + 夹爪架构 **创建日期**：2026-09-24

> **本文档是 17 个模块的索引**。每个模块的决策和实施细节请打开对应的模块文档。

---

## 文档结构

本文档**完全废弃旧"按代码层级拆分"的结构**（§一 CAN / §二 ASCII / §三 类 / §四 实时环），改为按**功能模块**组织。每个模块是一份独立文档：

```
docs/refactor/
├── firmware_new主控固件决策.md           ← 本文（索引）
└── firmware_new主控固件模块
    └──01_运动指令-MoveJ&MoveL.md
       ~ 模块16_EEPROM配置持久化.md
```

---

## 当前工作（重构 firmware_new 功能）

### 工作背景

`firmware/ref_core_f405/` 是 v3.x 主控固件基线，被弃用但保留参考。`firmware_new/dummy-ref-core-fw/` 是当前待定的目标固件，但 v2.7+ 多处改动丢失了 OLD 时代稳定可靠的语义（如 `while(IsMoving())` 阻塞、`!DISABLE` 不丢模式、Homing/Resting 信任电机端等）。

**当前任务**：把 firmware_new 中"v3.x 加的副作用"批量回退，恢复 OLD 行为；同时不破坏 v3.x 真正合理的改进（如 7 轴架构、单层 r/s² 单位、`_persist(bool)` 可选持久化等）。

### 当前进行中


| 模块              | 状态           | 范围                                                                                                                  |
| --------------- | ------------ | ------------------------------------------------------------------------------------------------------------------- |
| **模块 01**（运动指令） | **✅ 代码完成（2026-09-24）** | F1.1 SEQ/TRJ `>` `&` 阻塞回 OK、F1.2 `@` 清理 break、**F1.3 完全照搬 OLD 三件套**（`jointSpeedRatio=0.3` + per-joint BASE 加速度档位 + ParseCommand 阻塞）、F1.5 三项清理（默认 INT / SetEnable 防呆 / Homing 兜底）、F1.4 CMDMODE 5/6 不改 — 全部代码改动 + ninja 编译（201/201）已完成；待现场实测 |
| 模块 02（运动完成判断）   | ⏸️ 待实施       | 仅 `IsMoving()` 公式回退（dummy_robot.h:666-671）；Homing/Resting 兜底回退已并入模块 01 §6.3 完成                                                          |


### 已完成（不在本次范围）


| 模块              | 状态          | 范围               |
| --------------- | ----------- | ---------------- |
| 03-13 / 16      | ✅ 已生效       | 见 §4.2 v3.x 改进清单 |
| stall v3        | ✅ 已删除（§4.1） | 2026-09-24 完成    |
| 修复—阻塞回OK与OLED同步 | ⏳ 文档已就绪     | 见 §4.3 第 4 项     |


### 改造原则

1. **OLD 是默认基线**——OLD 怎么写就怎么改；firmware v3.x 偏离的部分要明确记录理由
2. **v3.x 副作用批量清理**——v3.x 加的"防呆/防卡死"补丁与 stall v3 同源，应一并回退（详见 §4.3）
3. **决策同步进索引**——任何模块改动都要在 §1 模块清单里同步登记，避免 agent 接手时不清楚工作范围

### 模块 01 决策清单（F1 系列）


| ID       | 范围                                      | 决策摘要                                                                                   | 文档位置                                           |
| -------- | --------------------------------------- | -------------------------------------------------------------------------------------- | ---------------------------------------------- |
| F1.1     | SEQ `>` `&` 阻塞回 OK                      | 恢复 OLD `while(IsMoving())` 阻塞                                                          | [§3](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md) |
| F1.2     | SEQ/TRJ `@` 清理 break                    | 移除 v2.8 "丢指令 break" 逻辑                                                                 | [§4](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md) |
| **F1.3** | TRJ `>` `&` 完全照搬 OLD 三件套（**2026-09-24 决策反转**） | 抛弃「拆解 + 仅 ratio=0.3」的中间决策；完全照搬 OLD：`jointSpeedRatio=0.3` + per-joint BASE 加速度档位 + ParseCommand 阻塞，三件套必须配套使用 | [§5](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md) |
| F1.4     | `#CMDMODE 5/6` 不改                       | TRQ/SRV 实际下发待拍板                                                                        | [§7](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md) |
| **F1.5** | 三项清理（默认 INT / SetEnable 防呆 / Homing 兜底） | 三件 v3.x 副作用全部回退到 OLD 行为                                                                | [§6](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md) |


---

## 1. 模块清单（17 个）


| #   | 模块                                        | 一句话决策                                                                                                | 状态           | 文档                                                  |
| --- | ----------------------------------------- | ---------------------------------------------------------------------------------------------------- | ------------ | --------------------------------------------------- |
| 01  | 运动指令 (`>` `&` `@`)                        | SEQ/TRJ `>` `&` **阻塞回 OK**（OLD 行为，共用 ParseCommand）；`@` 不阻塞（SEQ/INT/TRJ/INT 都立即回 ok）；F1.3 完全照搬 OLD 三件套（`jointSpeedRatio=0.3` + per-joint BASE 加速度档位 + 阻塞语义）；F1.5 默认 INT + 删 SetEnable 防呆 | **✅ 代码完成（2026-09-24）** | [模块 01](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md)   |
| 02  | 运动完成判断 (IsMoving)                         | 回退 OLD 公式 `jointsStateFlag != 0b1111110`，删除 v3.2 短路；Homing/Resting 兜底已并入模块 01 §6.3                   | ⏸️ 待实施       | [模块 02](firmware_new主控固件模块02_运动完成判断-IsMoving.md)    |
| 03  | 关节角度同步 (UpdateAngle)                      | 保留单播轮询 14Hz/电机（决策 E2）；Enable 后预发 1 帧广播                                                               | **已生效**      | [模块 03](firmware_new主控固件模块03_关节角度同步-UpdateAngle.md) |
| 04  | 急停 (EStop)                                | 0x09 广播急停 + 置 flag=ALL_FINISH_FLAG + 清 FIFO（已生效）                                                     | **已生效**      | [模块 04](firmware_new主控固件模块04_急停-EStop.md)           |
| 05  | 命令模式 (CommandMode)                        | 6 模式 cmdModeNames 保留；SERVO_J/TORQUE 实际下发待拍板                                                          | **部分生效**     | [模块 05](firmware_new主控固件模块05_命令模式-CommandMode.md)   |
| 06  | OLED 显示                                   | 删除 DIAG 心跳探针；恢复 `[CMD] __*_*__` 状态行（10x20 字号、Y=78，照抄 OLD）；`cmdModeNames` 6 元素（决策 C3）                 | **P0 待实施**   | [模块 06](firmware_new主控固件模块06_OLED显示.md)             |
| 07a | 灯效配置 (`!RGB_*`)                           | 保留全部 6 条命令 + SaveConfig 持久化                                                                          | **已生效**      | [模块 07a](firmware_new主控固件模块07a_灯效配置.md)             |
| 07b | RGB 状态机                                   | 3 个灯效预设状态码保留；旧 `SetRGBEnable(bool)` 去留待拍板                                                            | **部分生效**     | [模块 07b](firmware_new主控固件模块07b_RGB状态机.md)           |
| 08  | 夹爪控制 (`!HAND_*`)                          | 6 条命令保留；`StepHand` 继承 `CtrlStepMotor`                                                                | **已生效**      | [模块 08](firmware_new主控固件模块08_夹爪控制.md)               |
| 09  | 电机参数配置 (`#ACC_J`/`#SPEED_J`/`#I_LIMIT_J`) | 单层 r/s² 单位；`_persist()` 可选写 EEPROM                                                                   | **已生效**      | [模块 09](firmware_new主控固件模块09_电机参数配置.md)             |
| 10  | PID 查询 (`#GET_PID`/`#GETJACC`/`#GETI`)    | ISR 标志 + 主线程 poll + `PollPidPrint()` 合并打印                                                            | **已生效**      | [模块 10](firmware_new主控固件模块10_PID查询.md)              |
| 11  | 堵转协议通路 (0x5A/0x5B/0x5C)                   | 主控侧全部删除；保留 `SetEnableStallProtect` (0x1B)                                                            | **已删除**      | [模块 11](firmware_new主控固件模块11_堵转协议通路.md)             |
| 12  | 校准与归零 (`!CALIBRATION`/`#OFFSET_J`)        | 删除全轴 `!CALIBRATION`；保留单轴 `#OFFSET_J`                                                                 | **已生效**      | [模块 12](firmware_new主控固件模块12_校准与归零.md)              |
| 13  | 架构常量 (DH/减速比/REST_POSE)                   | 新机械结构 DH；统一减速比 50；REST_POSE J2=-75°                                                                  | **已生效**      | [模块 13](firmware_new主控固件模块13_架构常量.md)               |
| 14  | 队列与 CommandHandler                        | 队列容量 32×128B；USB/UART4 默认 `#` 分支二选一（C10 ⏳）                                                           | **部分生效**     | [模块 14](firmware_new主控固件模块14_队列与CommandHandler.md)  |
| 15  | 5kHz 实时环 (TIM7)                           | 保留框架 + 50Hz 节流下发；SERVO_J/TORQUE 实际启用待拍板                                                              | **部分生效**     | [模块 15](firmware_new主控固件模块15_5kHz实时环.md)            |
| 16  | EEPROM 配置持久化                              | 灯效 SaveConfig + `_persist(bool)` 可选持久化                                                               | **已生效**      | [模块 16](firmware_new主控固件模块16_EEPROM配置持久化.md)        |


---

## 2. 决策状态统计


| 状态             | 数量  | 模块                                      |
| -------------- | --- | --------------------------------------- |
| ✅ 已生效（含 2026-09-24 模块 01 代码完成 + 编译通过）| 10  | 01, 03, 04, 07a, 08, 09, 10, 11, 12, 13, 16 |
| ⚠️ 部分生效（待拍板子项） | 4   | 05, 07b, 14, 15                         |
| ❌ P0 待实施       | 2   | 02, 06                                   |


---

---

## 4. 已完成的历史决策（不可改动）

### 4.1 Stall v3 删除（2026-09-24 完成）

**范围**：主控侧 stall v3 全部删除（7 个文件，共 -292 行）

- 删除 `isStalled` 字段 + `IsStalled()` 方法
- 删除 0x5A/0x5B/0x5C/0x7C 主控侧处理
- 删除 `!STALL_*` ASCII 命令
- 删除 `SetStallMode/BroadcastUnlock/QueryStallStatus` 方法
- 删除相关 OLED 探针和 ISR 标志
- 串口助手 stall UI 同步删除（-82 行）

**保留**：0x1B `SetEnableStallProtect`（电机端独立使用的命令）

详见：模块 [11 - 堵转协议通路](firmware_new主控固件模块11_堵转协议通路.md)





### 4.2 已生效的 v3.x 改进（来自 firmware/）


| 改进                                 | 模块     |
| ---------------------------------- | ------ |
| CAN 0x09 急停广播                      | 模块 04  |
| 7 轴架构（motorJ[8]）                   | 模块 13  |
| `StepHand` 继承 `CtrlStepMotor`      | 模块 08  |
| `SetPositionWithMotorRps` 重命名      | 模块 13  |
| `_persist(bool)` 可选持久化             | 模块 09  |
| 灯效 `SaveConfig()` 持久化              | 模块 16  |
| PID/Acc/Cur 批量查询 + ISR poll + 合并打印 | 模块 10  |
| 单播轮询 14Hz/电机                       | 模块 03  |
| 50Hz 节流下发（实时环）                     | 模块 15  |
| 队列容量 32×128B                       | 模块 14  |
| cmdModeNames 6 模式数组                | 模块 05  |
| 夹爪 6 条命令                           | 模块 08  |
| 灯效 6 条命令                           | 模块 07a |
| 单层 r/s² 单位                         | 模块 09  |
| `#OFFSET_J` 单轴归零                   | 模块 12  |
| 新机械结构 DH 参数                        | 模块 13  |
| 优化紧急停止 flag 短路                     | 模块 04  |


### 4.3 firmware v3.x 副作用批量清理（2026-09-24 决策）

**范围**：与 stall v3 删除同思路，识别并清理 firmware v3.x 引入的"防呆"副作用。


| 副作用清理项                                                         | 决策 ID   | 模块                                                     | 决策                                              |
| -------------------------------------------------------------- | ------- | ------------------------------------------------------ | ----------------------------------------------- |
| `SetEnable(false)` 中 `SetCommandMode(DEFAULT_COMMAND_MODE)` 防呆 | F1.5 配套 | 01                                                     | 删。`!DISABLE` 保持当前模式，不丢用户工作状态                    |
| Homing/Resting 的 30s 兜底超时                                      | F5 配套   | [模块 01 §6.3](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md) | 删。回到 OLD `while(IsMoving())` 纯循环                |
| `SetCommandMode()` 的 `SetJointAcceleration(LOW/HIGH)` 调用       | F1.3 反转 | 01                                                     | **保留**。完全照搬 OLD 三件套：`jointSpeedRatio=0.3` + per-joint BASE 加速度档位 + ParseCommand 阻塞语义。2026-09-24 整体反转之前「拆解 + 仅 ratio=0.3」的决策 |
| OLED 第 6 行 `5x8` 字号 + Y=78                                     | OLED 配套 | 修复文档                                                   | 改 `10x20` 字号 + Y=78，照抄 OLD 写法                   |


