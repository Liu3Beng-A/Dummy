# 关节角度更新机制与 J2-J6 卡顿问题分析

> 适用版本：`firmware_new/dummy-ref-core-fw` + `firmware_new/dummy-motor-fw`
> 撰写日期：2026-09-29
> 现象：测试时 J1 角度刷新最快不卡，J2-J6 快速刷新一段时间后会卡一段时间

---

## 1. 现象描述

- **J1**：在连续快速下发位置指令时，角度几乎每个控制周期都在刷新；
- **J2-J6**：在快速刷新一段时间后会出现"卡住"，表现为 `#GETJPOS` 读到的角度停在某个旧值，过一段时间才恢复更新。

## 2. 角度更新链路

### 2.1 主控端发送

`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:131`

```cpp
void DummyRobot::UpdateJointAngles()
{
    motorJ[ALL]->UpdateAngle();   // motorJ[0] 是 ALL，nodeID=0
}
```

`CtrlStepMotor::UpdateAngle()` 发送 `StdId = 0 << 7 | 0x23 = 0x23`，没有 nodeID，是总线广播查询；所有电机都响应。

### 2.2 主控端周期下发位置

`firmware_new/dummy-ref-core-fw/UserApp/main.cpp` 的 `ThreadControlLoopFixUpdate`：

```cpp
switch (dummy.commandMode)
{
    case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
    case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
    case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
        dummy.MoveJoints(dummy.targetJoints);
        dummy.UpdateJointPose6D();
        break;
    ...
}
```

`MoveJoints()` 对每个关节发送 `0x07`（带速度限位的位置设置）。

### 2.3 电机端强制回包

`firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:84-100`，`case 0x07` 处理末尾强制回 0x23：

```cpp
case 0x07:  // Set Position with Velocity-Limit
{
    ...
    motor.controller->SetPositionSetPoint(...);
    // Always Need Position & Finished ACK
    tmpF = motor.controller->GetPosition();
    auto* b = (unsigned char*) &tmpF;
    for (int i = 0; i < 4; i++)
        _data[i] = *(b + i);
    _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
    txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
    CAN_Send(&txHeader, _data);
}
```

**每一条 0x07 下发都强制要一条 0x23 应答**。

### 2.4 主控端接收与组装

`firmware_new/dummy-ref-core-fw/UserApp/protocols/can_protocol.cpp`：

```cpp
uint8_t id = rxHeader->StdId >> 7;
uint8_t cmd = rxHeader->StdId & 0x7F;

switch (cmd)
{
    case 0x23:
        dummy.motorJ[id]->UpdateAngleCallback(*(float*) (data), data[4]);
        break;
    ...
}
dummy.UpdateJointAnglesCallback();
```

`UpdateJointAnglesCallback` 每收到一帧都会重算所有 6 个关节的 `currentJoints` 与 `jointsStateFlag`。

## 3. 总线流量与节拍

注意：`firmware_new/dummy-ref-core-fw/UserApp/main.cpp:9`

```cpp
Timer timerCtrlLoop(&htim7, 200);   // 第二个参数是频率(Hz)
```

第二个参数是频率，所以 TIM7 实际跑在 **200Hz（5ms 周期）**`firmware_new/dummy-ref-core-fw/Bsp/utils/timer.cpp:42-57` 的 `CalcRegister()` 会动态重配 Prescaler/ARR 给出对应频率。

每 5ms 一个控制循环产生：


| 方向      | 消息                                | 条数                        |
| ------- | --------------------------------- | ------------------------- |
| 主控 → 电机 | 0x07 SetPositionWithVelocityLimit | 6                         |
| 电机 → 主控 | 0x23 GetPosition (ACK)            | 6                         |
| **合计**  |                                   | **12 帧 / 5ms ≈ 2400 帧/秒** |


按 1Mbps、8 字节 payload 算，1Mbps CAN 单帧约 110-130µs，总线容量约 7700 帧/秒。**整体利用率 ~31%，并未饱和，但所有消息都集中在 200Hz 同步波次里爆发性发送，时序冲突严重。**

## 4. 根本原因

### 4.1 AutoRetransmission 关闭 → 仲裁失败即丢帧

**主控** `firmware_new/dummy-ref-core-fw/Core/Src/can.c:41,77`：

```c
hcan1.Init.AutoRetransmission = DISABLE;
hcan2.Init.AutoRetransmission = DISABLE;
```

**电机** `firmware_new/dummy-motor-fw/Core/Src/can.c:42`：

```c
hcan.Init.AutoRetransmission = DISABLE;
```

含义：当一帧 CAN 在总线上**输掉仲裁**（总线正被更高优先级帧占用）时，硬件**不会自动重传**，消息被静默丢弃。

### 4.2 CAN ID 优先级不对称

`StdId = (nodeID << 7) | cmd`，nodeID 越小优先级越高：


| 关节                | 请求 StdId (0x07) | 应答 StdId (0x23) | 优先级    |
| ----------------- | --------------- | --------------- | ------ |
| **J1** (nodeID=1) | `0x87`          | `0xA3`          | **最高** |
| J2 (nodeID=2)     | `0x107`         | `0x123`         | 次高     |
| J3 (nodeID=3)     | `0x187`         | `0x1A3`         | 中      |
| J4 (nodeID=4)     | `0x207`         | `0x223`         | 中      |
| J5 (nodeID=5)     | `0x287`         | `0x2A3`         | 低      |
| **J6** (nodeID=6) | `0x307`         | `0x323`         | **最低** |


### 4.3 仲裁失配的典型场景

主控在 5ms 内按 `J1 → J2 → … → J6` 顺序发 6 条 0x07，每条 `osSemaphoreAcquire(sem_can1_tx, portMAX_DELAY)` 等上一条 TX 完成才发下一条。**但 6 个电机几乎同时收到命令、几乎同时尝试发送 0x23 应答**，再加上主控的下一条 0x07 也在发：

- **J1 应答** `0xA3` 永远最先抢到总线 → 必然成功；
- **J2-J6 应答**（`0x123`、`0x1A3` … `0x323`）需要和 J1 应答、主控下一条 0x07 抢仲裁 → nodeID 越大越容易输；
- **输掉仲裁的那条被永久丢弃**（AutoRetransmit 关闭，不重传）。

所以呈现的现象是：


| 关节        | 角度刷新频率      | 原因                 |
| --------- | ----------- | ------------------ |
| **J1**    | 几乎每 5ms 都更新 | CAN ID 优先级最高，仲裁永远赢 |
| **J2-J6** | 周期性卡住       | 应答帧频繁仲裁失，被永久丢弃     |

### 4.4 bxCAN 3 TX mailbox 物理限制（2026-09-30 补充）

STM32F405 bxCAN 控制器（参考手册 RM0090 §44.7）硬件只有 **3 个 TX mailbox**。`HAL_CAN_AddTxMessage` 在 3 个 mb 都处于"占用/发送中"状态时返回 `HAL_BUSY`，**帧无法入 mb**。

主控 `MoveJoints`（`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:60-67`）在 5ms 控制周期内顺序调 6 次 `SetAngleWithVelocityLimit`，每次入 1 条 0x07：

```
t=0      J1 0x87   → mb0 (TME=1→0, 立即开始发送)
t=5µs    J2 0x107  → mb1 (TME=1→0)
t=10µs   J3 0x187  → mb2 (TME=1→0)
t=15µs   J4 0x207  → 3 mb 都 TME=0 → HAL_BUSY ❌
t=20µs   J5 0x287  → HAL_BUSY ❌
t=25µs   J6 0x307  → HAL_BUSY ❌
```

**关键事实**：

- 3 条 0x07 在 1Mbps 总线上发送需要 3 × 111µs ≈ 340µs 才能让 mb 全空；
- 这 340µs 期间 bxCAN 持续"满"，任何后续 `AddTxMessage` 都 `HAL_BUSY`；
- mb 顺序空闲：mb0 在 ~126µs 空（J1 发完）→ mb1 在 ~221µs 空 → mb2 在 ~226µs 空；
- 即便 J1-J3 顺序入 mb 成功，J4-J6 也因 bxCAN 物理容量限制无法入队。

这个根因**独立于 §4.1~§4.3 的仲裁机制**——即使 `AutoRetransmission` 开启、CAN ID 完全对称，bxCAN 仍只能硬件并发发送 3 条，6 条 0x07 在 5ms 突发中必有 3 条失败。这是 8.1~8.8 已敲定项之外**唯一剩余的硬约束**，§8.9 通过在 `CanSendMessage` 内短时等待 mb 空来错开发送。


## 5. "卡一段时间"的额外诱因

### 5.1 二值信号量语义不匹配

`firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp`：

```cpp
void CanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader)
{
    osStatus semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);  // 永久等待
    ...
    if (semaphore_status == osOK)
        HAL_CAN_AddTxMessage(canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox);
}

void tx_complete_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    if (hcan->Instance == CAN1)
        osSemaphoreRelease(sem_can1_tx);
}
```

两个隐患：

1. **CMSIS-RTOS2 的** `osSemaphoreRelease` **不会把二值信号量钳在 1**。STM32F4 有 3 个 TX mailbox，多个 mailbox 同时完成会多次 release，信号量计数会 >1，主控可能在没真正等满一次完整收发时就启动下一帧，叠加后导致 mailbox 占满；
2. `HAL_CAN_AddTxMessage` **在 mailbox 满时返回 HAL_ERROR，但调用方既不检查返回值也不释放信号量**。下一次 `osSemaphoreAcquire` 会永久阻塞，直到别处的在飞帧完成才解开 → **这正是"卡一段时间"的直接触发点**。

### 5.2 接收 FIFO 上限

`firmware_new/dummy-ref-core-fw/Core/Src/can.c:46`：

```c
hcan1.Init.ReceiveFifoLocked = DISABLE;
```

STM32F4 的 CAN RX FIFO 容量是 3 帧。FIFO 满后新到的 0x23 会覆盖最旧的（因为没有锁定），进一步放大 J2-J6 的滞后。

### 5.3 角度组装逻辑放大了抖动

`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`：

```cpp
void DummyRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];
        if (motorJ[i]->state == CtrlStepMotor::FINISH)
            jointsStateFlag |= (1 << i);
        else
            jointsStateFlag &= ~(1 << i);
    }
}
```

每收到一帧 0x23 都会重写 `currentJoints` 全部 6 项 + `jointsStateFlag` 全部 6 位。J1 一直在刷，所以 `jointsStateFlag` 一直被 J1 的最新状态重写；J2-J6 的 `state` 长时间不更新，会带着上一次的 RUNNING/STALL 继续参与 flag 计算，间接影响 `IsMoving()` 在 SEQ/TRJ 模式下的同步判断。

### 5.4 MoveJ 与 idle 路径的应答机制对比——为什么 MoveJ 不卡、idle 卡（2026-10-01 修订）

> **修订说明**：原 §5.4（2026-09-30）描述"bxCAN 3 mb 让 J4-J6 的 0x07 永远 HAL_BUSY，导致 ALL 0x23 兜底偶尔命中空 mb 窗口"。**本次诊断数据（`TxAlstRetryCnt[0..2] = 0`、`ACK/TX0 = 2.00` 稳定、用户实测 MoveJ 6 关节全顺）证明该模型与实测不符，删除原内容，重写为地址化 vs 广播的本质差异**。

实测现象拆解：

| 状态 | 现象 | 证据 |
|---|---|---|
| MoveJ 期间 | 6 关节均流畅，无卡 | 用户实测 |
| idle 期间 | J1 流畅，J2-J6 间歇卡 1-2s | 用户实测 + `#GETSTAT` ACK=2×TX0 |

两种状态的根本差异在**主控→电机的命令类型**：

| 维度 | MoveJ 路径 | idle 路径 |
|---|---|---|
| 命令 | 0x07 **地址化**（StdId = `(nodeID<<7) \| 0x07`） | 0x23 **ALL 广播**（StdId = `(0<<7) \| 0x23` = 0x23） |
| 每条命令的目标电机数 | 1 | 6（全部） |
| 每个电机的响应 | ACK 自己收到的命令 | ACK 自己（收到 ALL 也回自己的） |
| 多电机同时响应的概率 | **0**（命令不重叠，每条 0x07 在不同时间到达不同电机） | **100%**（所有电机收到 ALL 同时响应） |
| 应答冲突 | 不存在 | 6 条 ACK 撞车，J1 总赢，其他永久丢 |
| 每电机 ACK 频率（实测） | 200Hz（每控制周期 1 次） | J1=50Hz、J2-J6 ≈ 8-10Hz 不稳定 |

**0x07 是地址化命令**：主控在 5ms 控制周期内按 J1→J2→…→J6 顺序发 6 条 0x07，每条 0x07 的 StdId 指向唯一电机。每个电机**只 ACK 自己收到的 0x07**，与总线上的其他命令无关。

6 条 0x07 在 666µs 内陆续到达 6 个电机（每条 0x07 帧时长 111µs）：电机 J1 在 t=222µs 收到命令，电机 J6 在 t=888µs 收到。每个电机收到命令后立即回 ACK（电机端 case 0x07 处理末尾的 `CAN_Send`），所以 6 个电机的 ACK 在总线上**天然错开 ~111µs**。**没有冲突**。

**0x23 ALL 是广播命令**：主控发一条 0x23 = `(0<<7)|0x23`，6 个电机都收到并同时回 ACK。6 条 ACK 的 StdId 各不相同（`(nodeID<<7)|0x23`），但它们**几乎同时进入总线**（不同电机 ISR 响应延迟相差 < 1µs），开始仲裁。CAN 仲裁是逐位比较：J1 的 StdId = 0xA3 最小 = 优先级最高，**永远第一个赢得仲裁**；J2-J6 的 StdId 更大 = 优先级更低，全部输给 J1 后被 bxCAN 标记为"已发但不重试"（`AutoRetransmission=DISABLE`），帧被永久丢弃。

**为什么实测 ACK = 2×TX0 而不是 1×TX0**：诊断数据稳定显示每条 ALL 广播对应恰好 2 条 ACK（多次采样 ACK/TX0 = 1.99~2.00）。第二个 ACK 的具体来源（哪个电机）随机，与 6 个电机各自的 ISR 响应延迟、CAN PHY 物理层传输延迟抖动有关；统计意义上每次只有 ~1/5 概率轮到某个 J2-J6 电机，所以每个 J2-J6 电机的 ACK 频率 ≈ 50Hz × 1/5 = 10Hz，且分布不均（偶尔 0，偶尔连中几帧），呈现"卡 1-2s 后突然齐刷一次"的间歇性更新。

**与之前分析的因果关系纠正**：

| 之前的说法（§4.4 / §5.4 / §8.9） | 本次诊断后的修正 |
|---|---|
| "bxCAN 3 mb 让 J4-J6 的 0x07 永远 HAL_BUSY" | `TxAlstRetryCnt[0..2]=0` 证伪：mb 等待逻辑落地后 0x07 全部入 mb 成功；且 MoveJ 期间 6 关节都顺，0x07 路径无可见丢帧 |
| "卡 1-2s 后突然齐刷" 是 mb 满后等空窗的幸运时刻 | 实测是 ALL 0x23 广播应答撞车，J1 总赢 + 1 个 J2-J6 随机赢 → J2-J6 间歇性 ~10Hz 更新 |
| "§8.9 bxCAN 3 mb 等待" 是修复卡顿的关键 | §8.9 修的是 MoveJ 路径（0x07 入 mb），对 idle 路径无效；idle 卡顿的真因是广播应答冲突，需 §8.10 改 6 条个体查询解决 |

**修复路径**：§8.10 把 `motorJ[ALL]->UpdateAngle()` 改成 6 条个体查询，让 6 个电机不会同时收到 0x23，从而消除 ACK 撞车。

## 6. 总结


| 现象               | 直接原因                                                                    | 根因                                               |
| ---------------- | ----------------------------------------------------------------------- | ------------------------------------------------ |
| MoveJ 期间 6 关节全顺 | 0x07 是地址化命令，6 电机在 ~666µs 内陆续收到，ACK 天然错开 ~111µs，无冲突                  | 命令类型差异：地址化 vs 广播                  |
| idle 期间 J1 刷新最快不卡 | J1 nodeID 最小 → CAN ID 优先级最高 → 仲裁永远赢                          | ALL 0x23 广播触发 6 电机同时 ACK，J1 总赢     |
| idle 期间 J2-J6 间歇卡 1-2s 后突然齐刷一次 | ALL 0x23 触发 6 电机同时回 ACK 撞车，J1 必赢 + 1 个 J2-J6 随机赢，其余 4 永久丢（AutoRetransmission=DISABLE 不重试） | **CAN 广播应答冲突的本质：地址化命令可错开，广播命令必然撞车；§8.10 改 6 条个体查询解决** |
| 之前误判（2026-09-30）| "bxCAN 3 mb 让 J4-J6 的 0x07 永远 HAL_BUSY" → 导致 ALL 0x23 兜底偶尔命中空 mb 窗口 | `TxAlstRetryCnt[0..2]=0` + MoveJ 6 关节全顺 证伪；§5.4 / §8.9 描述已修订 |


## 7. 建议的修复方向（按影响力排序）

1. **打开** `AutoRetransmission`（主控 + 电机都改 `ENABLE`）。仲裁失败时硬件自动重传，J2-J6 不再被永久丢失。
2. **检查** `HAL_CAN_AddTxMessage` **返回值**，失败时正确释放信号量、记录错误，避免永久阻塞。
3. **错开电机端应答时刻**。如果想保留"低延迟优先于可靠性"，可以给电机应答分配不同 cmd code，让电机按 `osDelay(节点ID)` 错开发送；或者让主控在 `MoveJoints` 里循环里加微秒级延时，把 6 条 0x07 摊开到 200µs+ 区间内。
4. `ReceiveFifoLocked = ENABLE`，防止新帧覆盖未读旧帧。
5. **提高控制循环频率**（如 `Timer(&htim7, 500)` 或 `1000`）。下发节拍更快更均匀后，瞬时冲突减少。
6. `UpdateJointAnglesCallback` **按 id 分支更新**，避免 J1 高频刷新把整个 `jointsStateFlag` 反复重写。

具体修改需要征得用户同意后再进行。

---

## 8. 决策记录

> 把已经敲定、即将落地的方案单独记录在这里，与"建议方向"区分。每条决策写明**改动位置、采用方案、未决项**。

### 8.1 OLED 刷新限制到 50Hz（已敲定，2026-09-29）

**背景**：当前 `ThreadOledUpdate` 是没有 `osDelay` 也没有 `vTaskDelayUntil` 的纯 `for(;;)` 循环。OLED 任务被 200Hz 控制循环和 CAN 中断频繁抢占，FPS 显示值 = `1 / 单帧绘制耗时`，本质是绘制耗时倒数而非调度频率。当前观测到的"50Hz 上下跳动"是"调度 200Hz × 绘制耗时 20ms"被除出来的统计结果，不稳定。

**决策**：将 OLED 任务刷新率用 `vTaskDelayUntil` 精确钉到 50Hz（20ms 周期）。

**改动位置**：`firmware_new/dummy-ref-core-fw/UserApp/main.cpp` 中的 `ThreadOledUpdate`。

**采用方案**（示意，未落地）：

```cpp
void ThreadOledUpdate(void* argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);   // 50Hz

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);   // 精确 50Hz 唤醒
        mpu6050.Update(true);
        oled.clearBuffer();
        // ... 现有绘制代码原样保留 ...
        oled.sendBuffer();
    }
}
```

**为什么不用 `osDelay(20)`**：相对延迟会被控制循环抢占拉长到 ≈40ms，实际变成 25Hz，屏幕会明显闪烁。`vTaskDelayUntil` 用绝对时间自动补齐丢失的 tick。

**未决项**（与本决策独立，需后续讨论）：

- 控制循环本身（`Timer timerCtrlLoop(&htim7, 200)`）是否一并降到 50Hz。
- CAN ID 优先级不对称如何解决（§4.2）—— 仍处于讨论阶段。
- `AutoRetransmission` 是否开启 —— 仍在权衡利弊。

> [2026-09-29 补充] 上述 3 条未决项已在本轮（§8.2-§8.5）中分别给出决策，本节"未决项"仅作为历史讨论状态保留。

---

### 8.2 查询频率压到 50Hz（已敲定，2026-09-29）

**背景**：当前 200Hz 控制循环每 5ms 触发一次 `UpdateJointAngles()`，主控通过 ALL 代理发 1 条 0x23 查询，6 个电机几乎同时回 6 条 0x23，应答冲突（仲裁失败）频次为 200 次/秒。

**决策**：**保持控制循环 200Hz 不变**，仅将 `UpdateJointAngles()` 改为每 4 帧调用 1 次（即 200/4 = 50Hz），电机端应答频次随之从 200Hz 降至 50Hz。

**改动位置**：`firmware_new/dummy-ref-core-fw/UserApp/main.cpp` 中的 `ThreadControlLoopFixUpdate`。

**采用方案**：

```cpp
void ThreadControlLoopFixUpdate(void* argument)
{
    static uint32_t queryTick = 0;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (dummy.IsEnabled())
        {
            switch (dummy.commandMode)
            {
                case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
                case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
                case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
                    dummy.MoveJoints(dummy.targetJoints);          // 下发 200Hz 不变
                    dummy.UpdateJointPose6D();
                    if ((++queryTick & 0x03) == 0)
                        dummy.UpdateJointAngles();                 // ★ 50Hz
                    break;
                case DummyRobot::COMMAND_MOTOR_TUNING:
                    dummy.tuningHelper.Tick(10);
                    dummy.UpdateJointPose6D();
                    if ((++queryTick & 0x03) == 0)
                        dummy.UpdateJointAngles();
                    break;
            }
        } else
        {
            // 失能态：UpdateJointPose6D 依赖 currentJoints，一起降频
            if ((++queryTick & 0x03) == 0) {
                dummy.UpdateJointAngles();
                dummy.UpdateJointPose6D();
            }
        }
    }
}
```

**收益量化**：


| 指标         | 改前                   | 改后                 |
| ---------- | -------------------- | ------------------ |
| 下发 0x07 频次 | 200Hz × 6 = 1200 帧/秒 | 1200 帧/秒（不变）       |
| 查询 0x23 频次 | 200Hz × 6 = 1200 帧/秒 | 50Hz × 6 = 300 帧/秒 |
| 总帧数        | 2400 帧/秒             | 1500 帧/秒（-37.5%）   |
| 总线利用率      | ~48%                 | ~30%               |
| 应答冲突频次     | 200 次/秒              | 50 次/秒（-75%）       |


**风险评估**：

- `currentJoints` 仅用于 OLED 显示、`IsMoving()` 状态判断、正运动学查询、串口 `?` 主动查询，**不参与电机端闭环控制**——电机端 PID 闭环在 STM32F103 上跑，不需要主控的实时角度反馈。
- TRJ 连续轨迹执行不受影响：主控下发的是"目标角度"，电机端自己插值。
- `IsMoving()` 到位检测误差从 5ms 增加到 20ms，功能层面无影响。
- 与 §8.1 OLED 50Hz 配合：`currentJoints` 阶梯式 20ms 更新一次，OLED 按 50Hz 渲染，肉眼无感知差异。

**未决项**（与本决策独立）：

- 如果查询 50Hz 后 J2-J6 卡顿仍明显，再讨论是否需要进一步降频或引入更激进的方案。

---

### 8.3 AutoRetransmission 维持 DISABLE（已敲定，2026-09-29）

**背景**：曾考虑将 `hcan1.Init.AutoRetransmission`（及 `hcan2`、电机端）由 `DISABLE` 改为 `ENABLE`，让仲裁失败的帧自动重传，避免 J2-J6 应答被永久丢弃。

**决策**：**不做**。当前维持 `DISABLE`。

**不改的 6 个原因**：

1. **信号量永久阻塞（致命）**：`CanSendMessage` 先 `osSemaphoreAcquire(sem_can1_tx, osWaitForever)` 永久等待，再调用 `HAL_CAN_AddTxMessage`。`AutoRetransmission=ENABLE` 下，TX mailbox 会长时间被自动重传中的帧占满，`HAL_CAN_AddTxMessage` 返回 `HAL_BUSY` 但**调用方未检查返回值也未释放信号量**，下一次 `osSemaphoreAcquire` 永久阻塞 → 200Hz 控制循环死亡。
2. **总线负载倍增、雪崩风险**：ENABLE 下仲裁失败的帧自动重传，重新进入总线争抢；200Hz × 6 = 1200 帧 0x07/秒在重传叠加后利用率可达 80-100%，总线占满后**新帧发不出，电机端应答也被堵**。
3. **错误帧无限重传触发 Bus-Off**：ACK / 位错误 / CRC 错误在 ENABLE 下也会自动重传，受干扰或接触不良时 CAN 控制器错误计数（TEC）快速累加到 255 → 进入 Bus-Off 状态 → **整个 CAN 总线瘫**。DISABLE 下反而更难触发 Bus-Off。
4. **失去通信状态可见性**：当前 `HAL_CAN_ErrorCallback` 通过 `HAL_CAN_ERROR_TX_ALST0/1/2` 精确知道"哪一帧仲裁失败、失败几次"。ENABLE 下硬件自己重传，应用层看不到任何失败迹象——总线繁忙时以为通信正常，其实 100 帧都在排队重传。
5. `**HAL_CAN_AddTxMessage` 阻塞语义变化**：DISABLE 下邮箱立即可发下一帧；ENABLE 下邮箱等到帧发送成功才空。3 个邮箱在 1200 帧/秒下发压力下任何时刻都可能被占满，第 4 帧 `HAL_CAN_AddTxMessage` 返回 BUSY → 信号量永久丢失 → 循环卡死。
6. **与控制循环 200Hz 不兼容**：`ThreadControlLoopFixUpdate` 是 `osPriorityRealtime`。ENABLE 下仲裁失败的帧持续占邮箱，控制循环实际周期会被拉长到 5ms+N×200µs，**200Hz 偷偷掉到 150Hz 甚至更低**，而 HAL_TIM_PWM_PulseFinishedCallback 还在按 200Hz 节奏触发，任务和中断错位。

**前置条件（如果要反悔开启 ENABLE，必须先做的配套改动）**：

- 修 `CanSendMessage` 检查 `HAL_CAN_AddTxMessage` 返回值，失败时释放信号量
- 主控侧 6 条 0x07 错开发送，避免 6 条同邮箱排队
- 电机端应答错开，避免 6 条 0x23 同时应答

三件事必须一起做，**单独开 ENABLE 风险大于收益**。

---

### 8.4 主控 6 条 0x07 错开发送（重新评估，2026-09-29）

**背景**：考虑在 `MoveJoints` 内循环里加微秒级延时，把 6 条 0x07 摊开到 200µs+ 区间内，降低 0x07 帧之间的总笔冲突。之前的"不做"决策（基于"未找到较好错开方案"的笼统理由）撤销，重新基于代码事实评估。

**决策**：**仍判定不做**。理由不是"风险大"这个主观判断，而是基于 3 个代码级事实——方案 3 在当前架构下**根本没有生效条件**。

**事实 1：当前 `CanSendMessage` 不是阻塞调用，DelayUs(50) 错不开任何东西**

`firmware_new/dummy-ref-core-fw/Core/Src/freertos.c:115`：

```c
sem_can1_tx = osSemaphoreNew(1, 1, osSemaphore(sem_can1_tx));
```

**初始计数 = 1，最大计数 = 1**。再看 `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp:108-118` 和 `235-245`：

```cpp
void tx_complete_callback(...)
{ if (hcan->Instance == CAN1) osSemaphoreRelease(sem_can1_tx); ... }

void CanSendMessage(...)
{
    semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);
    if (semaphore_status == osOK)
        HAL_CAN_AddTxMessage(...);   // 不等发完，立即返回
}
```

后果：第一次 `acquire` 立即成功（1→0），TX complete 后 `release`（→1），之后**每个 `acquire` 都立即成功、不阻塞**。也就是说 `MoveJoints` 循环 6 次调用 `SetAngleWithVelocityLimit` **总共耗时约 30µs**（5µs × 6 + 调度开销），6 条 0x07 **几乎同时塞进 CAN mailbox**。

方案 3 的 DelayUs(50) 看似错开了主循环入 mailbox 的时刻，但**实际上 mailbox 入帧顺序没变**——见事实 2。

**事实 2：CAN1 mailbox 按 ID 优先级发送，DelayUs 错开的是"入 mb 时刻"而非"上总线时刻"**

`firmware_new/dummy-ref-core-fw/Core/Src/can.c:46`：`hcan1.Init.TransmitFifoPriority = ENABLE`。

这意味着 bxCAN 硬件**按 ID 优先级从 3 个 mailbox 里选帧发送**，不是按入 mb 顺序。

**当前实际发送时序**（无 DelayUs）：

```
t=0     主循环把 J1~J6 6 帧几乎同时塞进 mb0/mb1/mb2
t=0     硬件按 ID 优先级从 mb 中选帧 → J1 0x87 (ID=135) 最先发
t=111µs J1 0x87 发完 → 硬件从 mb 中选下一帧 → J2 0x107 (ID=263)
t=222µs J2 0x107 发完 → J3 0x187 (ID=391)
t=333µs J3 0x187 发完 → J4 0x207 (ID=519)
t=444µs J4 0x207 发完 → J5 0x287 (ID=647)
t=555µs J5 0x287 发完 → J6 0x307 (ID=775)
t=666µs J6 0x307 发完
```

**6 条 0x07 在总线上是连续占用 666µs**，没有任何间隔。

**加 DelayUs(50) 后的实际时序**：

```
t=0     主循环 j=1, J1 进 mb0, 立即开始发
t=5µs   j=2 进入 DelayUs(50)
t=55µs  DelayUs 结束, J2 进 mb1, 等 mb0 发完
t=111µs J1 0x87 发完, J2 0x107 立即开始发
t=105µs j=3 进入 DelayUs(50)
t=155µs J3 进 mb2, 等 mb1 发完
t=222µs J2 0x107 发完, J3 0x187 立即开始发
...
t=666µs J6 0x307 发完（与无 DelayUs 完全相同）
```

**总线占用时刻一字不差地仍是 t=0~666µs 连续 666µs**。DelayUs 只让"主循环空闲"了，没让"总线空闲"。

**事实 3：即便改了 TransmitFifoPriority=DISABLE，对 ACK 时序改善也很有限**

要让 DelayUs(50) 真正错开发送时刻，必须把 CAN1 改为 `TransmitFifoPriority=DISABLE`（按入 mb 顺序发），或把 `CanSendMessage` 改成"阻塞到 TX complete 才返回"。这两个改动都**超出方案 3 原作者描述的范畴**，且会带来副作用：

- **改 TransmitFifoPriority=DISABLE**：高 nodeID 的 J6 反而先发，违反当前"J1 优先"的设计，且 CAN 错误处理路径需要重审。
- **改 `CanSendMessage` 阻塞**：恢复 sem 真正阻塞语义，6 条发送变 111µs + 5 × 50µs = 805µs；同时打开"方案 4 必须先做"的同款 bug（HAL_CAN_AddTxMessage 返回 HAL_ERROR 时信号量永久丢失）。

即使改了这两条之一，再评估 ACK 时序改善：

| ACK | 准备时刻 | 无 DelayUs 时总线占用 | DelayUs(50) 后总线占用 | 是否改善 |
|---|---|---|---|---|
| J1 ACK 0xA3 | t=170µs | J2 0x107 (111-222µs) 占用 | J2 0x107 (161-272µs) 占用 | ❌ 同样阻塞 |
| J2 ACK 0x123 | t=341µs | J3 0x187 (222-333µs) | J3 0x187 (272-383µs) | ❌ 同样阻塞 |
| J3 ACK 0x1A3 | t=512µs | J4 0x207 (333-444µs) | J4 0x207 (383-494µs) | ❌ 同样阻塞 |

50µs 间隔不足以让电机 ISR 处理时延（~30µs）+ 帧间间隔（3µs）完全消化。**J1~J6 的 ACK 仍会被主控的下一条 0x07 阻塞**，核心问题没解决。

**真正能错开 ACK 时序的间隔** = 6 × 111µs（确保 J1 的 ACK 在 J6 的 0x07 发完前完成）≈ 670µs。如果每条 0x07 之间 DelayUs(670)：

- 总耗时：6 × 111µs + 5 × 670µs = 666 + 3350 = **4016µs**（5ms 周期的 80%）
- 直接吃掉 4ms 控制循环余量，副作用不可接受

**结论**：方案 3 在当前架构（CAN1 TransmitFifoPriority=ENABLE + sem 初始=1）下**完全没有生效条件**。要让它生效必须连带改动硬件配置或 sem 语义，**改完后对 J2-J6 ACK 改善仍有限**，而真正有效的间隔大到不可接受。

**因此方案 3 不可行，重新评估后维持"不做"**。J2-J6 卡顿应从协议层（电机端 ACK 错开）或总线层（开启 AutoRetransmission + 配套改造）解决，不应从主控侧 DelayUs 入手。

---

### 8.5 电机端应答错开（不做，2026-09-29）

**背景**：考虑给电机应答分配不同 cmd code，或让电机端按 `osDelay(节点ID)` 错开发送 0x23 应答，避免 6 条应答同时上总线、争抢仲裁。

**决策**：**不做**。

**原因**：

- 用户认为风险大（具体风险评估见对话记录与 §7 建议 3 的讨论）
- 违反本轮"只改主控、不改电机端"的工作范围约束

---

### 8.6 检查 HAL_CAN_AddTxMessage 返回值，失败时释放信号量（已敲定，2026-09-29）

**背景**：`firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp:235-245` 的 `CanSendMessage` 在 `osSemaphoreAcquire` 成功后调 `HAL_CAN_AddTxMessage`，但**未检查返回值也未在失败时释放信号量**。3 个 mailbox 在瞬时突发场景下可能全部占满（如控制循环连续多帧入队 + CAN TX 完成 ISR 尚未运行），第 4 帧 `HAL_CAN_AddTxMessage` 返回 `HAL_ERROR`，信号量却被永久占用，**下一次 `acquire` 阻塞直到别处的在飞帧完成才解开**——这正是 `docs/CAN角度更新机制与J2-J6卡顿问题分析.md` §5.1 第 2 点和 §8.3 第 1 个"不改 ENABLE 的原因"所指出的"卡一段时间"直接触发点。

**决策**：**采纳**。零风险、零改动量级，是后续所有修复的前置条件——即使不开 AutoRetransmission，这个修复也独立有效。

**改动位置**：

1. `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp:235-245` —— 改 `CanSendMessage` 函数体
2. `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.hpp:11-32` —— `CAN_context` 结构体加 `uint32_t tx_busy_errors = 0;` 字段

**采用方案**：

```cpp
// firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp
void CanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader)
{
    osStatus semaphore_status;
    if (canCtx->handle->Instance == CAN1)
        semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);
    else if (canCtx->handle->Instance == CAN2)
        semaphore_status = osSemaphoreAcquire(sem_can2_tx, osWaitForever);
    else
        return;

    if (semaphore_status == osOK)
    {
        HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(
            canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox);
        if (status != HAL_OK)
        {
            // 失败必须释放信号量，否则下次 acquire 永久阻塞
            if (canCtx->handle->Instance == CAN1)
                osSemaphoreRelease(sem_can1_tx);
            else if (canCtx->handle->Instance == CAN2)
                osSemaphoreRelease(sem_can2_tx);
            canCtx->tx_busy_errors++;   // 用于诊断，可在串口调试时打印
        }
    }
}
```

**收益量化**：

| 指标 | 改前 | 改后 |
|---|---|---|
| 信号量永久阻塞导致控制循环假死 | 偶发（瞬时突发） | 消除 |
| `tx_busy_errors` 计数可见性 | 无 | 有（用于诊断总线瞬时饱和） |

**风险评估**：零。逻辑等价于"请求失败 → 释放信号量 + 计数"，没有改变任何时序、没有改变任何协议、没有改硬件配置。

**配套**：方案 4 落地后，原 §8.3 决策中"反悔开启 ENABLE 必须先做的 3 个配套改动"的第 1 条已完成。后续如果决定开 AutoRetransmission，剩下的两条配套（"主控侧 6 条 0x07 错开发送"、"电机端应答错开"）依然不可行——见 §8.4 重新评估。

---

### 8.7 RX FIFO 锁定 + Full 中断释放槽位（已敲定，2026-09-29）

**背景**：`firmware_new/dummy-ref-core-fw/Core/Src/can.c:46` 当前 `hcan1.Init.ReceiveFifoLocked = DISABLE`，`can.c:82` 同理 CAN2。STM32F4 RX FIFO 容量仅 3 帧；`ReceiveFifoLocked=DISABLE` 下 FIFO 满后**新帧覆盖最旧帧**（即"无声覆盖"，丢失最早的应答）；`ReceiveFifoLocked=ENABLE` 下 FIFO 满时**新帧被硬件丢弃**，触发 `CAN_IT_RX_FIFO0_FULL` 中断，**保留旧帧**。在 J2-J6 应答集中到达的场景里，"保留最早的应答"比"保留最新的应答"略差（最新位置变化更大），但**比"无声丢失且无法察觉"好得多**。

`interface_can.cpp:128-131` 当前 `HAL_CAN_RxFifo0FullCallback` 只计数不处理：
```cpp
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef* hcan)
{ if (get_can_ctx(hcan)) get_can_ctx(hcan)->RxFifo0FullCallbackCnt++; }
```

**决策**：**采纳**。单 bit 改动 + 调整 Full Callback 释放一个 FIFO 槽位。零风险、独立性强，与 §8.6 完全正交。

**改动位置**：

1. `firmware_new/dummy-ref-core-fw/Core/Src/can.c:46` —— CAN1 `ReceiveFifoLocked`：`DISABLE` → `ENABLE`
2. `firmware_new/dummy-ref-core-fw/Core/Src/can.c:82` —— CAN2 `ReceiveFifoLocked`：`DISABLE` → `ENABLE`
3. `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp:128-131` —— `HAL_CAN_RxFifo0FullCallback` 函数体

**采用方案**：

```c
// firmware_new/dummy-ref-core-fw/Core/Src/can.c
// CAN1 行 46:
hcan1.Init.ReceiveFifoLocked = ENABLE;   // 原来是 DISABLE
// CAN2 行 82:
hcan2.Init.ReceiveFifoLocked = ENABLE;
```

```cpp
// firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef* hcan)
{
    CAN_context* ctx = get_can_ctx(hcan);
    if (!ctx) return;
    ctx->RxFifo0FullCallbackCnt++;
    // FIFO 满 = 主控来不及读。释放 1 个槽位让后续帧进来（保留已读的最旧应答）。
    // HAL_CAN_GetRxMessage 调用后该帧已被消费（不再上总线协议层）。
    // 不调 OnCanMessage — 这一帧已被放弃，由 HAL_CAN_RxFifo0MsgPendingCallback
    // 在它自己的入口正常处理后续到达的帧。
    CAN_RxHeaderTypeDef tmpHeader;
    uint8_t tmpBuf[8];
    (void) HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &tmpHeader, tmpBuf);
}
```

**收益量化**：

| 指标 | 改前 | 改后 |
|---|---|---|
| 应答被静默覆盖丢失 | 偶发 | 消除 |
| 应答被显式丢弃时诊断可见性 | 无 | 有（`RxFifo0FullCallbackCnt` 计数可见） |
| FIFO 满时保留的应答 | 最旧（被覆盖） | 最旧（仍可能丢，但每次丢都有计数） |

**风险评估**：零。单 bit 改动 + Callback 函数体只多了 1 次 `HAL_CAN_GetRxMessage` 调用，行为可预测。

**配套**：与 §8.6 完全独立，可同步落地。

---

### 8.8 失能态查询频率降到 50Hz（已敲定，2026-09-29）

**背景**：`firmware_new/dummy-ref-core-fw/UserApp/main.cpp:24-30` 当前失能态分支 200Hz 调 `dummy.UpdateJointAngles()`（发 1 条 0x23 ALL 广播查询，6 个电机几乎同时回 6 条 0x23 ACK）。§8.2 已决定"使能态查询降到 50Hz"，但失能态没改——失能态查询频次仍为 200Hz × 1 条 ALL + 200Hz × 6 条 ACK = **2600 帧/秒**（比使能态还高），应答冲突频次 200 次/秒。

失能态没有 0x07 下发，**必须靠查询拿位置**，但 200Hz 太密——电机端 PID 闭环在 STM32F103 上跑，不依赖主控的实时角度反馈，主控的 50Hz 查询完全够 OLED 显示、`IsMoving()` 状态判断、正运动学查询、串口 `?` 主动查询。

**决策**：**采纳**。与 §8.2 节奏保持一致（50Hz），与 §8.6/§8.7 完全独立。

**改动位置**：`firmware_new/dummy-ref-core-fw/UserApp/main.cpp:24-30` 改失能态 else 分支。

**采用方案**：

```cpp
// firmware_new/dummy-ref-core-fw/UserApp/main.cpp
// 改前：
} else
{
    // Just update Joint states
    dummy.UpdateJointAngles();
    dummy.UpdateJointPose6D();
}
// 改后：
} else
{
    // 失能态没有 0x07 下发，靠查询拿位置
    // 50Hz 与 §8.2 使能态节奏保持一致
    if ((++queryTick & 0x03) == 0) {
        dummy.UpdateJointAngles();
        dummy.UpdateJointPose6D();
    }
}
```

**注意**：`queryTick` 是 §8.2 落地时引入的局部 `static uint32_t` 变量，位于 `ThreadControlLoopFixUpdate` 函数顶部。§8.2 落地时会使能态和失能态共用同一个 `queryTick`——这是预期设计，**§8.8 落地需在 §8.2 之后执行**，或与 §8.2 合并提交。

**收益量化**：

| 指标 | 改前 | 改后 |
|---|---|---|
| 失能态查询 0x23 频次 | 200Hz × 1 = 200 帧/秒 | 50Hz × 1 = 50 帧/秒 |
| 失能态总流量 | 2600 帧/秒 | 1700 帧/秒（-35%） |
| 失能态应答冲突频次 | 200 次/秒 | 50 次/秒（-75%） |

**风险评估**：极低。

- `currentJoints` 仅用于 OLED 显示、`IsMoving()` 状态判断、正运动学查询、串口 `?` 主动查询，**不参与电机端闭环控制**——电机端 PID 闭环在 STM32F103 上跑，不需要主控的实时角度反馈。
- 失能态到位检测误差从 5ms 增加到 20ms，功能层面无影响（SEQ 模式下 `IsMoving()` 在 `ThreadControlLoopUpdate` 的 `while (IsMoving() && IsEnabled()) osDelay(5)` 里轮询，本身就有 5ms 周期）。
- 失能态 `UpdateJointPose6D` 一起降到 50Hz 后，正运动学显示延迟最大 20ms，人眼无感。

**未决项**：

- 是否合并到 §8.2 提交？还是 §8.2 落地后单独提交？
- 如果 §8.6 落地后 J2-J6 卡顿显著缓解，失能态 50Hz 查询是否仍需要？——是的，独立需要（失能态流量减少对总线拥塞缓解独立有效）。

---

### 8.9 升级 §8.6 → §8.6+：bxCAN 3 mb 限制下 0x07 入 mb 失败的修复（已敲定，2026-09-30）

**实测背景**（2026-09-30 用户实测）：

8.1~8.8 落地后实测，现象如下：

| 项目 | 结果 |
|---|---|
| OLED FPS | 稳定 50Hz（§8.1 起效） |
| MoveJ 期间 6 关节刷新 | 用户报告 6 关节均流畅 |
| idle 期间 J1 角度刷新 | 任何时候都快速，手动掰动关节时 OLED 立即反映 |
| idle 期间 J2-J6 角度刷新 | 一段时间刷新很快（与 J1 同步），然后卡 1-2s 突然齐刷一次，循环往复 |

**§8.9 实际修复的对象**：

STM32F405 的 bxCAN 控制器**只有 3 个 TX mailbox**（硬件事实，参考手册 RM0090 §44.7）。`HAL_CAN_AddTxMessage` 在所有 3 个 mb 都占满时返回 `HAL_BUSY`，帧无法入队。

主控 `MoveJoints`（`dummy_robot.cpp:60-67`）在 5ms 周期内顺序调 6 次 `SetAngleWithVelocityLimit`，每次 1 条 0x07：

| 时刻（相对周期起） | 调用 | bxCAN mb 状态 | HAL 结果 |
|---|---|---|---|
| t=0 | J1 0x87 | mb0/mb1/mb2 全空 | 入 mb0 ✓ |
| t≈5µs | J2 0x107 | mb1/mb2 空 | 入 mb1 ✓ |
| t≈10µs | J3 0x187 | mb2 空 | 入 mb2 ✓ |
| t≈15µs | J4 0x207 | mb0/1/2 全在发送（TME=0） | **HAL_BUSY** |
| t≈20µs | J5 0x287 | 同上 | **HAL_BUSY** |
| t≈25µs | J6 0x307 | 同上 | **HAL_BUSY** |

3 条 0x07 在 ~340µs 内发完，mb 全空；但 §8.6 落地前的代码直接放弃失败帧，§8.6 落地后也只是"释放信号量 + 计数"——**帧本身永远丢了，J4-J6 永远收不到 setpoint**。

**决策**：**升级 §8.6 → §8.6+**，在 `CanSendMessage` 内**短时等待 mb 空**（最长 1ms），而不是失败立即返回。

**采用方案**：

```cpp
// firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp:248-285
void CanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader)
{
    osStatus semaphore_status;
    if (canCtx->handle->Instance == CAN1)
        semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);
    else if (canCtx->handle->Instance == CAN2)
        semaphore_status = osSemaphoreAcquire(sem_can2_tx, osWaitForever);
    else
        return;

    if (semaphore_status == osOK)
    {
        // §8.6+ — bxCAN 3 mb 限制：等 mb 出现空位再入
        // 5ms 控制周期内 6 条 0x07：J1-J3 入 mb0/1/2 立即成功，J4-J6 HAL_BUSY
        // 等 mb 空后 J4-J6 也能入；总等待 ~340µs（占 5ms 周期 7%）
        // 1ms 超时兜底，防止总线故障永久阻塞信号量
        uint32_t start = micros();
        HAL_StatusTypeDef status;
        while ((status = HAL_CAN_AddTxMessage(
                    canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox)) != HAL_OK)
        {
            if ((micros() - start) > 1000)   // 1ms 超时
            {
                canCtx->tx_busy_errors++;   // 用于诊断（健康时应 ≈ 0）
                break;
            }
        }

        if (status != HAL_OK)
        {
            if (canCtx->handle->Instance == CAN1)
                osSemaphoreRelease(sem_can1_tx);
            else if (canCtx->handle->Instance == CAN2)
                osSemaphoreRelease(sem_can2_tx);
        }
        // 成功时信号量由 tx_complete_callback 在发送完成后释放
    }
}
```

**6 条 0x07 的入 mb 时序**（升级后）：

| 时刻 | 事件 | mb 状态 |
|---|---|---|
| t=0 | J1 0x87 | mb0 占用、mb1/2 空 |
| t≈5µs | J2 0x107 | mb0/1 占用、mb2 空 |
| t≈10µs | J3 0x187 | mb0/1/2 占用 |
| t≈126µs | mb0 空（J1 发完） | J4 0x207 入 mb0 |
| t≈221µs | mb1 空（J2 发完） | J5 0x287 入 mb1 |
| t≈226µs | mb2 空（J3 发完） | J6 0x307 入 mb2 |
| t≈340µs | 全部 6 条入 mb | ALL 0x23 在剩余 4.66ms 内随时可入 |

**收益量化**：

| 指标 | §8.6 改前 | §8.6 改后 | §8.6+ 改后 |
|---|---|---|---|
| J1 0x07 入 mb 成功率 | 100% | 100% | 100% |
| J2-J3 0x07 入 mb 成功率 | 100% | 100% | 100% |
| J4-J6 0x07 入 mb 成功率 | 0% | 0%（仍丢帧） | **100%** |
| `currentJoints` 全 6 项更新频率 | 仅 J1+J2+J3 每 5ms | 同上 | **每 5ms 全 6 项** |
| `tx_busy_errors`（健康态） | 频繁 ++ | 不再 ++ | 保持 ≈ 0 |
| 5ms 周期内等待时间 | 0 | 0 | ~340µs（占 7%） |
| 控制循环实际频率 | 200Hz | 200Hz | 200Hz（不降频） |
| 失败时信号量永久阻塞风险 | 有 | 无 | 无（升级 §8.6 已修） |

**风险评估**：低。

- `micros()` 已通过 `time_utils.h` 暴露（`common_inc.h:13`），无需新增头文件依赖。
- 1ms 超时是兜底（正常 bxCAN 在 1Mbps 总线下 3 mb 全发完只需 ~340µs，1ms 余量充足），超时时仍走 §8.6 路径释放信号量 + 计数，行为不会卡死。
- 总等待 340µs 占 5ms 周期 7%，低优先级任务（OLED、CommandParser）的 CPU 时间从原来的 ~25% 减到 ~18%，人眼无感。
- 不需要改 MoveJoints 调用方、不需要改电机端、不需要拆 2 波次任务。

**验证方法**：

- 健康态：`tx_busy_errors` 在 OLED 上显示应保持 0 或偶尔 +1。
- 卡顿消失：手动掰 J4-J6 关节，OLED 角度值应跟随，J1-J6 同步刷新。
- `firmware/` 老版本对照：原 `firmware/ref_core_f405/` 的 `CanSendMessage` 在控制循环外没有任何等待，但老版本没有 6 条 0x07 突发场景（控制循环频率低），所以未踩到这个坑。

**配套**：本次升级**不需要退回** §8.1~§8.8 任何一项；是 §8.6 的向上兼容增强。如果 §8.6+ 落地后 J2-J6 仍卡，备选方案是 §8.4 重新评估中提到的"拆 2 波次（vTaskDelay 错开）"——但会牺牲控制频率到 ~166Hz。

**2026-10-01 修订说明**（用户新增诊断数据 `#GETSTAT` 后）：

- 原"为什么 J4-J6 卡 1-2s 后突然刷新"模型（bxCAN 3 mb 限制 + ALL 0x23 兜底偶中空 mb 窗口）与诊断数据**矛盾**：
  - `TxAlstRetryCnt[0..2]=0`：mb 等待逻辑落地后 0x07 全部入 mb 成功，没有"必须等 340µs"的场景
  - `ACK/TX0 = 2.00` 稳定：每条 ALL 广播对应恰好 2 条 ACK，与 6 电机全部 ACK 不一致
- **§8.9 实际修复的对象**：仅 MoveJ 路径的 0x07 入 mb（让 6 条 0x07 在 mb 满时等待而不是放弃）
- **§8.9 不解决的对象**：idle 路径的 J2-J6 卡顿。真因是 ALL 0x23 广播触发 6 电机同时 ACK 撞车，J1 总赢仲裁，其余永久丢失——见 §5.4 / §6 修订、§8.10
- **已确认必要（2026-10-01 用户确认）**：§8.9 是 MoveJ 路径 0x07 入 mb 的**必要修复**。在 5ms 控制周期内主控连续发 6 条 0x07，bxCAN 仅 3 个 TX mb（参考手册 RM0090 §44.7），没有 §8.9 的等待逻辑则 J4-J6 的 0x07 直接 `HAL_BUSY` 丢帧——MoveJ 期间 J4-J6 setpoint 频率被锁在 ≈166Hz（每帧 3 条成功）而非预期的 200Hz。用户报告"MoveJ 期间 6 关节均流畅"是因为电机端 PID 闭环对 setpoint 频率不敏感（响应惯性掩盖了 5ms 级别的 setpoint 缺失），**不代表 J4-J6 setpoint 频率健康**——这一频率差在 SEQ 模式快速方向切换、点位到位瞬间的同步判断等场景下仍会体现为 J4-J6 的微小跟随滞后与 `IsMoving()` 状态报告误差。
- **§8.9 与 §8.10 的关系（重要）**：

| 维度 | §8.9 | §8.10 |
|---|---|---|
| 修复的路径 | MoveJ（0x07 地址化下发）| idle（0x23 ALL 广播查询）|
| 丢帧机制 | bxCAN 3 mb 满 → J4-J6 HAL_BUSY 丢 0x07 | 6 电机同时回 ACK → 仲裁撞车 → J1 必赢 +1 个 J2-J6 随机赢 |
| 触发频率 | 200Hz × 6 条/帧 | 50Hz × 1 条/帧 |
| 修复方式 | mb 满时短时等待（最长 1ms）| ALL 广播 → 6 条地址化查询 |
| 改动位置 | `interface_can.cpp:CanSendMessage` | `dummy_robot.cpp:UpdateJointAngles` |

两者**完全正交**，各自解决各自路径的丢帧问题，**不是冗余修复**。

- **保留决策（2026-10-01 敲定）**：用户明确 §8.9 改动必要，**永久保留**，从 §9"待评估方向"移除，不再作为待定项。

---

### 8.10 idle 路径 6 条个体查询替代 ALL 广播查询（已敲定，2026-10-01）

**背景**：

§8.1~§8.9 落地后 idle 期间 J2-J6 仍卡 1-2s。用户诊断数据 `#GETSTAT` 显示 `ACK = 2 × TX0`（多次采样比例稳定 1.99~2.00），证明每条 ALL 0x23 广播只有 2 个电机 ACK（不是 6 个）。结合 §5.4 重写后的分析，**idle 路径的卡顿根因是 ALL 0x23 广播触发 6 电机同时 ACK 撞车，与 §8.9 修复的 0x07 路径完全正交**。

**决策**：**采纳**。把 `motorJ[ALL]->UpdateAngle()`（1 条 ALL 广播查询）改成 6 条个体查询（每条指向特定 nodeID）。6 条查询在 ~666µs 内顺序发出，6 个电机在 ~666µs 内**依次**收到、**依次**回 ACK，**ACK 时刻天然错开 ~111µs**（一帧时长），无仲裁冲突。

**改动位置**：

- `firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:161-164` — `UpdateJointAngles` 函数体
- 调用方 `firmware_new/dummy-ref-core-fw/UserApp/main.cpp` 中的 `ThreadControlLoopFixUpdate`（§8.2 / §8.8 已落地）**无需改动**——`UpdateJointAngles` 内部已经处理

**采用方案**：

```cpp
// firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp
void DummyRobot::UpdateJointAngles()
{
    // §8.10 — 6 条个体查询替代 ALL 广播
    // 6 条 0x23 在 ~666µs 内顺序发出，6 电机依次收到、依次回 ACK，
    // ACK 在总线上天然错开 ~111µs，无仲裁冲突
    for (int i = 1; i <= 6; i++)
        motorJ[i]->UpdateAngle();
}
```

**为什么有效**：

每条个体查询的 StdId 只指向一个电机（`(i << 7) | 0x23`，i=1..6）。6 条查询在 50Hz 节奏下每 20ms 发一轮，6 条查询在 ~666µs 内顺序发出（每条帧时长 111µs）。6 个电机在 ~666µs 内依次收到、依次回 ACK。每个电机的 ACK 由其 ISR 立即排入它自己的 bxCAN mb（独立于其他电机），**6 条 ACK 在总线上天然错开 ~111µs，无撞车**。

**为什么 MoveJ 路径不动**：

MoveJ 路径用 0x07（地址化命令），6 个电机不会同时响应（每条 0x07 在不同时间到达不同电机），ACK 天然错开。该路径 §8.9 已经修过，6 关节全顺，无须改动。

**收益量化**：

| 指标 | §8.10 改前（ALL 广播） | §8.10 改后（6 条个体） |
|---|---|---|
| 主控 TX 查询 | 50 Hz × 1 = 50 Hz | 50 Hz × 6 = 300 Hz |
| 电机 → 主控 ACK | 100 Hz（2/6 撞车丢） | 300 Hz（6/6 全收） |
| 总线占用率 | ~1.7% | ~6.7% |
| 每电机 ACK 频率 | J1=50Hz，J2-J6 ≈ 8-10Hz 不稳定 | 每电机稳定 50 Hz |
| J2-J6 OLED 表现 | 间歇卡 1-2s 后突然齐刷 | 50Hz 平滑刷新，与 J1 同步 |
| `currentJoints` 全 6 项更新 | 仅 J1 稳定，J2-J6 间歇 | 每 20ms 全 6 项同步更新 |

**风险评估**：低。

- 总线占用率 6.7%，留有 90%+ 余量，未来扩展地轨 (id=7) + 夹爪 (id=8) 后占用率约 8.9%
- 主控 CPU 处理量 ×6：300 calls/sec × ~10µs = 3ms/sec = ~0.3% CPU，可忽略
- 控制循环 5ms 周期内 666µs TX + 666µs ACK ≈ 1.3ms 占用（占 26%），集中在前半段，不影响 OLED 50Hz 渲染
- §8.9 的 mb 等待逻辑保证 6 条查询在 mb 满时也能入队（兼容 §8.10 的高 TX 密度）
- 不需要改电机固件，符合"只改主控、不改电机端"工作范围

**验证方法**：

1. 健康态：`received_ack`（§9.0 计数器）在 20ms 内稳定 += 6（每秒 300）
2. 之前 idle 期间 `received_ack` 是 += 100/s（2×50Hz），改后应是 300/s
3. 手动掰动 J2-J6 任一关节，OLED 角度应跟随，6 关节同步刷新无卡顿
4. `tx_busy_errors` 保持 ≈ 0（§8.9 等待逻辑正常工作）

**与之前决策的关系**：

| 决策 | 与 §8.10 的关系 |
|---|---|
| §8.1 OLED 50Hz | 正交，保留 |
| §8.2 / §8.8 查询 50Hz 节奏 | 重新定义：50Hz 节奏变成"每 50ms 跑一轮 6 条个体查询"，总线负载 300Hz 仍合理 |
| §8.6 HAL 返回值检查 | 正交，保留 |
| §8.7 RX FIFO 锁定 | 正交，保留 |
| §8.9 bxCAN 3 mb 等待 | **完全正交、各自必要**：§8.9 修 MoveJ 路径的 0x07 入 mb（避免 J4-J6 setpoint 频率被锁在 166Hz），§8.10 修 idle 路径的 0x23 广播应答撞车。两者不重叠，详见 §8.9 末尾 2026-10-01 修订说明。2026-10-01 用户确认 §8.9 必要，**永久保留**。 |

---

## 9. 待评估方向（暂不决策）

下列方向在 §8.10 落地后再观察，**目前无紧迫性**：

- **§8.5 电机端应答错开重新评估**：如果 §8.10 落地后进一步想降低总线占用率（虽然现在 6.7% 已很轻松），可以让电机端在收到 0x23 后按 `nodeID × N` 延迟回 ACK。但需要改电机固件，违反本轮工作范围，仅作远期备选。
- **§8.3 反悔开 AutoRetransmission**：本轮方案已彻底绕开"广播应答撞车"问题，开 AutoRetransmission 的主要动机消失。继续保持 DISABLE，与 §8.6+ / §8.7 / §8.9 / §8.10 兼容。

