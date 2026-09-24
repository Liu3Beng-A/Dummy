# 模块 03 — 关节角度同步（`UpdateAngle()` 0x23 轮询策略 + `UpdateJointAnglesCallback`）

> **范围**：主控轮询电机位置（CAN 0x23），更新 `jointsStateFlag` + `currentJoints`
> **关联文件**：
> - `Robot/instances/dummy_robot.cpp` (`UpdateJointAngles()` + `UpdateJointAnglesCallback()`)
> - `UserApp/protocols/can_protocol.cpp` (`case 0x23` 回包处理)
> - `Robot/actuators/ctrl_step/ctrl_step.cpp` (`UpdateAngle()` 发送)

---

## 模块概述

主控需要实时知道每个电机的当前位置，用于：
1. 更新 `jointsStateFlag`（让 IsMoving() 能判定到位）
2. 更新 `currentJoints.a[]`（给 OLED 显示、IK 解算用）
3. 计算 `currentPose6D`（FK 缓存，给 #GETLPOS 用）

**两种轮询策略**：
- **广播触发**（OLD）：1 帧 `0x23` 触发所有电机同时回包，回包可能撞 mailbox（CAN 总线冲突）
- **单播轮询**（firmware E2）：100Hz 主控循环，每帧只发 1 个电机（currentIndex 1~7 循环），7 节点各分到 ~14Hz

**回包后处理**：
- `UpdateAngleCallback()`：主控侧更新单个电机 `angle` 字段
- `UpdateJointAnglesCallback()`：把 J1~J6 的 `currentJoints` + `jointsStateFlag` 整合

---

## 三状态对比

| 维度 | OLD | firmware | 当前 firmware_new |
|---|---|---|---|
| UpdateAngle 触发频率 | 每帧（5kHz）触发 | 100Hz 主循环触发 + 7 节点循环（14Hz/电机） | **14Hz/电机**（E2 已生效） |
| 轮询方式 | 广播（1 帧 ALL） | 单播（currentIndex 1~7 循环） | 单播 |
| `case 0x23` 处理 | 单一函数：UpdateAngleCallback + UpdateJointAnglesCallback | 按 id 分三段（id=7 同步 rail；id=1~6 关节；id=8 夹爪） | 单一函数（与 OLD 类似，回包统一处理） |
| jointsStateFlag 位分配 | 仅 bit1~6 | bit0/7 槽（地轨恒 0；bit7 用不上） | 同 OLD（仅 bit1~6） |
| Enable 后预发 | 不预发（依赖每帧广播） | 预发 1 帧广播（让电机尽快回包建立初始状态） | **预发 1 帧广播**（已生效） |
| 7 轴处理 | 无 | J7/地轨单独回包 + 同步 currentRailPos | **存在但仍按 OLD 模式处理**（UpdateJointAnglesCallback 不处理 rail） |

---

## 决策

| # | 决策 | 理由 | 风险 |
|---|---|---|---|
| 1 | **保留单播轮询 14Hz/电机**（E2） | 1 帧 7 节点广播可能撞 mailbox；单播更稳 | 响应延迟 ~70ms（旧版广播 ≤5ms），但足够 IsMoving() 判定 |
| 2 | **保留 `motorJ[ALL]->UpdateAngle()` 1 帧广播的 Enable 后预发**（line 626） | 加速首次回包建立初始状态，避免 `Enable → 发 MoveJ → 等待 IsMoving=false` 的首次延迟 | 无（仅在 enable 时发一次） |
| 3 | **`UpdateJointAnglesCallback` 保持 OLD 逻辑**（`for i=1..6`） | OLED 显示只显示 6 关节；地轨独立走 `motorJ[7]->SetPositionWithMotorRps` 路径 | 无 |
| 4 | **`case 0x23` 不做 id 分流** | 主控侧统一处理回包，简化代码 | 无 |
| 5 | **地轨 `currentRailPos` 同步机制：暂不维护** | 当前 `UpdateJointAnglesCallback` 不处理地轨回包；地轨位置由 `motorJ[7]->SetPositionWithMotorRps` 写入后不缓存 | 地轨当前位置查询 `#GETLPOS` 时无 rail 字段（如有需求需另外实现） |

---

## 实施

### 实施项：当前代码已符合决策，无需修改

**关键代码位置**（已正确）：

| 文件 | 行 | 代码 | 说明 |
|---|---|---|---|
| `dummy_robot.cpp:441` | `motorJ[currentIndex]->UpdateAngle();` | 单播轮询（100Hz 主循环） |
| `dummy_robot.cpp:626` | `motorJ[ALL]->UpdateAngle();` | Enable 后预发广播 |
| `dummy_robot.cpp:454-467` | `UpdateJointAnglesCallback` 中 `for (i=1; i<=6; i++)` 写 flag | OLD 逻辑 |
| `can_protocol.cpp` case 0x23 | `UpdateAngleCallback + UpdateJointAnglesCallback` | 与 OLD 一致 |

**禁止事项**：
- ❌ 不要把单播轮询改回全帧广播（破坏 E2 决策的稳定性）
- ❌ 不要在 `UpdateJointAnglesCallback` 加 rail 处理（保持 6 轴清爽）
- ❌ 不要修改 `jointsStateFlag` 的 bit 分配

---

## 验证

| 测试 | 期望 | 方法 |
|---|---|---|
| 单播轮询频率 | 100Hz 主循环下 7 电机各 ~14Hz | 调试串口打印 `currentIndex` 循环 |
| 6 电机回包延迟 | 每电机 ~70ms 总到位 | 发 `>` 后看 OLED 状态行更新节奏 |
| Enable 后预发广播 | enable 后第 1 次回包延迟缩短 | `!DISABLE → !START` 后立即发 `>` |
| jointsStateFlag 位正确 | 6 轴 bit1-6 全置位时 = 0x7E | 调试串口打印 `flag` |
| 地轨不参与 jointsStateFlag | 地轨回包不影响 flag bit0 | 同上 |

---

## 状态

- [x] 决策已记录（2026-09-24）
- [x] E2 决策已生效（单播轮询 14Hz/电机）
- [x] Enable 后预发广播已生效
- [x] jointsStateFlag 6 轴逻辑已生效
- 无需代码修改

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [02 - IsMoving](firmware_new主控固件模块02_运动完成判断-IsMoving.md) | jointsStateFlag 是 IsMoving 的判定依据 |
| [06 - OLED显示](firmware_new主控固件模块06_OLED显示.md) | currentJoints.a[] 是 OLED 第 3 行显示来源 |
| [10 - PID查询](firmware_new主控固件模块10_PID查询.md) | 0x23 轮询机制与 0x28~0x2D PID 轮询独立运行 |
| [15 - 5kHz实时环](firmware_new主控固件模块15_5kHz实时环.md) | 100Hz 主循环由 5kHz TIM7 节流 |
