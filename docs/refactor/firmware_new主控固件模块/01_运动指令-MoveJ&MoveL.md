# 模块 01 — 运动指令（`>`/`&`/`@` + MoveJ/MoveL/ServoJ + 阻塞回 OK）

> **范围**：主控 ASCII 运动指令（`>` `&` `@`）的解析 + 下发 + 回执语义
> **关联文件**：
> - `Robot/instances/dummy_robot.cpp` (`ParseCommand` 中 SEQ/TRJ/INT/TUN/TRQ/SRV 6 模式分支)
> - `Robot/instances/dummy_robot.cpp` (`MoveJ / MoveL / ServoJ / MoveJoints`)
> - `UserApp/protocols/ascii_protocol.cpp` (`>` `&` `@` 解析入口)

---

## 模块概述

机械臂所有运动指令（`>` `&` `@`）由 `CommandHandler::ParseCommand` 在 `ControlLoopUpdateTask` 中消费。每条运动指令的目标是**把目标位姿经 IK/MoveJ 计算后下发到电机，并在合适时机回 `ok` 给上位机**。

**三种指令格式**：

| 前缀 | 类型 | 入参数量 | 内容 |
|---|---|---|---|
| `>` | MoveJ（关节空间） | 7 关节 + speed | j1,j2,j3,j4,j5,j6,j7_rail,speed |
| `&` | MoveJ（关节空间 + speed 显式） | 7 关节 + speed | j1,j2,j3,j4,j5,j6,j7_rail,speed |
| `@` | MoveL（笛卡尔直线） | 6 维坐标 + speed | x,y,z,a,b,c,speed |

**两种回执时机**：
- **阻塞回 ok**（OLD 行为）：发起指令后等待电机全部到位才回 `ok`（SEQ/TRJ 模式 + `>` `&`）
- **非阻塞回 ok**（OLD 行为）：发起指令后立即回 `ok`（INT 模式、`@` 任意模式）

---

## 三状态对比

| 维度 | OLD（firmware_old） | firmware（firmware/ref_core_f405） | 当前 firmware_new |
|---|---|---|---|
| 参数格式 | `>j1,j2,j3,j4,j5,j6,speed`（6关节） | `>j1,j2,j3,j4,j5,j6,j7_rail,speed`（7关节） | **7 关节**（B3 已落地） |
| `&` 与 `>` 区别 | `&` 必带 speed，`>` speed 可选 | 同 OLD，speed 必填 | 同 B3 |
| `@` 格式 | `@x,y,z,a,b,c,speed` | 同 OLD | 同 OLD |
| **SEQ `>` `&` 阻塞** | **阻塞 `while(IsMoving) osDelay(5)` 后回 ok** | **非阻塞**（v2.8 改为 `if(IsMoving) break` + 立即回 ok） | **非阻塞**（F1.1 待实施） |
| **TRJ `>` `&` 阻塞** | **阻塞** | **非阻塞**（`MoveJoints` 后立即回 ok） | **非阻塞**（F1.3 待实施） |
| SEQ `@` | 非阻塞 | 非阻塞（v2.8 同上 break） | 非阻塞（F1.2 待清理 break） |
| TRJ `@` | 非阻塞 | 非阻塞 | 非阻塞 |
| INT 任意 | 非阻塞 | 非阻塞 | 非阻塞 |
| TUN/TRQ/SRV | TUN 仅扫描正弦；无 TRQ/SRV | TUN/TRQ/SRV 全部支持 | 6 模式都写好但实际下发逻辑部分未启 |
| 回执通道 | USB + UART4 双通道 | USB + UART4 双通道 | 同 |
| 7 轴计算后地轨下发 | 无地轨 | `motorJ[7]->SetPositionWithMotorRps(...)` 单独下发 | 同 firmware |

---

## 决策

| # | 决策 | 理由 | 风险 |
|---|---|---|---|
| 1 | **`>` `&` 参数格式保留 7 关节 + speed**（B3） | 7 轴架构已落地，地轨参与 MoveJ 是必然需求 | 无（已生效） |
| 2 | **SEQ `>` `&` 阻塞回 OK**（OLD 行为） | 用户 P0 明确诉求：上一条做完才做下一条 + 全部到位才回 ok | 阻塞期间 FIFO 不消费（OLD 也有此行为，可接受） |
| 3 | **TRJ `>` `&` 同样阻塞回 OK**（OLD 行为） | TRJ 与 SEQ 复用同一阻塞语义，便于上层状态机判断 | TRJ 设计意图是"不减速衔接"，与阻塞有微妙冲突，但 OLD 就是这样 |
| 4 | **SEQ/TRJ `@` 不阻塞**（OLD 行为） | `@` 用 IK 计算周期长，本就不阻塞 | 无 |
| 5 | **INT 任意指令非阻塞**（OLD 行为） | INT 设计意图就是"丢新指令立即回 ok" | 无 |
| 6 | **SERVO_J 模式：命令模式数组保留 6 模式（TRQ/SRV 也存在）但 `>` `&` 不进入 SERVO_J 分支** | 决策 C3 保留 cmdModeNames 6 模式，但 SERVO_J 的 5kHz 全速下发逻辑暂不实现 | 用户切换到 #CMDMODE 6 后发 `>` 会走哪条分支需明确 |
| 7 | **TORQUE_CONTROL 模式：保留入口但力矩透传实际下发逻辑不实现** | 与 SERVO_J 同：模式名/数组保留透传 `$` 指令解析未启 | 同 |
| 8 | **`#CMDMODE 5/6` 行为**：待明确是回 `error` 还是走对应分支 | 当前 firmware_new 已经实现 6 模式 switch case | 待定 |

---

## 实施

### F1.1 — SEQ `>` `&` 子块改回阻塞

**位置**：`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`（约 820-840 行，对应 SEQ 模式 switch case）

**当前代码**（v2.8 非阻塞）：
```cpp
if (context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed))
{
    // [v2.8 修复] ← 要删除
    if (context->IsMoving() && context->IsEnabled())
    {
        Respond(*usbStreamOutputPtr,  "ok");
        Respond(*uart4StreamOutputPtr, "ok");
        break;
    }
    context->MoveJoints(context->targetJoints);
    context->motorJ[7]->SetPositionWithMotorRps(...);
    Respond(*usbStreamOutputPtr,  "ok");
    Respond(*uart4StreamOutputPtr, "ok");
}
```

**替换为**（OLD 阻塞语义）：
```cpp
if (context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed))
{
    // [F1.1] 恢复 OLD 阻塞回 OK 语义
    //   1) 立即下发一次 MoveJoints，防止 IsMoving() 因 jointsStateFlag 残留误判
    //   2) 阻塞等待 bit1-6 全置位（jointsStateFlag = 0x7E）
    //   3) USB+UART4 双通道同时回 ok
    context->MoveJoints(context->targetJoints);
    context->motorJ[7]->SetPositionWithMotorRps(
        context->targetRailPos / 5.0f, context->railSpeedRps);

    while (context->IsMoving() && context->IsEnabled())
        osDelay(5);

    Respond(*usbStreamOutputPtr,  "ok");
    Respond(*uart4StreamOutputPtr, "ok");
}
```

**禁止事项**：
- ❌ 保留 v2.8 的 `if (IsMoving) break` 丢指令逻辑（破坏阻塞语义）
- ❌ 在 while 循环内加超时（如 Homing 那种 30s 兜底）—— 运动指令不应有上限

### F1.2 — SEQ `@` 子块清理 v2.8 break

**位置**：同一文件中 SEQ case 的 `@` 分支（约 850-870 行）

**当前代码**：`if (IsMoving && IsEnabled) { ... break; }` —— 阻断后续 MoveJoints

**替换为**（仅删除 break，保留非阻塞语义）：
```cpp
if (context->MoveL(...))
{
    context->MoveJoints(context->targetJoints);
    context->motorJ[7]->SetPositionWithMotorRps(...);
    Respond(*usbStreamOutputPtr,  "ok");
    Respond(*uart4StreamOutputPtr, "ok");
}
```

### F1.3 — TRJ `>` `&` 子块加阻塞

**位置**：TRJ case 的 `>` `&` 分支（约 880-910 行）

**当前代码**：下发后立即回 ok（不阻塞）

**替换为**：与 F1.1 一致，加 `while (IsMoving) osDelay(5)`

### F1.4 — `#CMDMODE 5/6` 行为明确

**待用户拍板**：
- **选项 A**：保留当前 6 模式 switch 实现，发 `>` 走 SERVO_J 全速下发（需启动 5kHz 实时环下发逻辑）
- **选项 B**：在 `#CMDMODE 5/6` 时，发 `>` `&` 回 `error: mode not implemented`
- **选项 C**：TRQ/SRV 模式完全等同 SEQ 处理（不下发任何额外逻辑）

---

## 验证

| 测试 | 期望 | 命令 |
|---|---|---|
| SEQ `>` 阻塞回 OK | 发指令后等 6 个 `*` 才回 ok | `#CMDMODE 1; >0,-73,180,0,0,0,0,30` |
| TRJ `>` 阻塞回 OK | 同 SEQ | `#CMDMODE 3; >0,-73,180,0,0,0,0,30` |
| INT 立即回 OK | 发指令后立即回 ok | `#CMDMODE 2; >0,0,0,0,0,0,0,30` |
| `@` 立即回 OK | 不等电机 | `#CMDMODE 1; @0.3,0,0.2,0,0,0,30` |
| 地轨单独下发 | 7 个关节值都到 | `>0,-73,180,0,0,0,50,30` |
| 阻塞期间 FIFO 积压 | 第二条指令延迟到第一条完成后才消费 | 发两条 `>` 紧挨着 |

---

## 状态

- [x] 决策已记录（2026-09-24）
- [ ] F1.1/F1.2/F1.3 代码已修改
- [ ] F1.4 用户拍板
- [ ] 编译验证（ninja）
- [ ] 实测验证（6 电机到位后回 ok）

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [02 - IsMoving](firmware_new主控固件模块02_运动完成判断-IsMoving.md) | `while (IsMoving)` 的公式必须用 OLD 公式（F5 配套） |
| [03 - UpdateAngle](firmware_new主控固件模块03_关节角度同步-UpdateAngle.md) | jointsStateFlag 更新靠 0x23 回包 → 影响 IsMoving() 返回时机 |
| [04 - EStop](firmware_new主控固件模块04_急停-EStop.md) | 急停后会清 FIFO + flag=0x7E → 阻塞循环自然退出 |
| [05 - CommandMode](firmware_new主控固件模块05_命令模式-CommandMode.md) | 6 模式 cmdModeNames + Switch case 决策 |
| [13 - 架构常量](firmware_new主控固件模块13_架构常量.md) | REST_POSE 默认值 + 7 关节减速比 |
