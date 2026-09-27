# 模块 04 — 急停（CAN 0x09 广播 + `!STOP` ASCII + 短路 IsMoving）

> **范围**：紧急停止机制（CAN 0x09 急停广播 + ASCII `!STOP` 命令 + 清 FIFO）
> **关联文件**：
> - `Robot/actuators/ctrl_step/ctrl_step.cpp` (`BroadcastEStop()` 实现)
> - `Robot/instances/dummy_robot.cpp` (`EmergencyStop()` 实现 + `ClearFifo()`)
> - `UserApp/protocols/ascii_protocol.cpp` (`!STOP` 分支)

---

## 模块概述

急停是机械臂安全的最后防线。需要：
1. **快速响应**：1 帧广播触发所有电机同时抱闸
2. **状态清理**：清指令队列，避免后续 SEQUENTIAL 指令被卡死
3. **短路 IsMoving**：避免 `!DISABLE` 后 IsMoving 永远 true 导致 SEQUENTIAL 队列不可消费
4. **双通道识别**：USB + UART4 都接受 `!STOP` 精确匹配

---

## 三状态对比

| 维度 | OLD（伪急停） | firmware | 当前 firmware_new |
|---|---|---|---|
| 急停命令 | 无 0x09；用 `MoveJ(currentJoints)` 伪实现 | **CAN 0x09 广播**（电机端 `SetBrake(true) + state=STOP`） | **0x09 广播**（P0-4 已生效） |
| `!STOP` ASCII 匹配 | `find("STOP")`（误命中 `!STOP_X`） | 精确 `s == "!STOP"` | 精确匹配 |
| jointsStateFlag | 不动（依赖回包自然更新到 FINISH） | **置 `= ALL_FINISH_FLAG`**（短路 IsMoving） | **置 ALL_FINISH_FLAG** |
| isEnabled | `isEnabled = false` | 同 | 同 |
| ClearFifo | 调 `ClearFifo()` | 同 | 同 |
| motorJ[ALL] 广播 | 无（OLD 无 ALL 代理） | 发 StdId=0x09（`(0<<7)\|0x09`） | 同 firmware |
| 收尾 | 异步等电机回包（很慢） | 同步短路 IsMoving | 同 firmware |

---

## 决策

| # | 决策 | 理由 | 风险 |
|---|---|---|---|
| 1 | **保留 CAN 0x09 急停广播** | 安全急停必须独立于 stall 检测，1 帧广播保证快速响应 | 无 |
| 2 | **`!STOP` 精确匹配** | 避免误命中其他以 STOP 开头的命令 | 无 |
| 3 | **`EmergencyStop()` 流程**：发 0x09 广播 → 置 `flag = ALL_FINISH_FLAG` → `isEnabled = false` → `ClearFifo()` | 4 步一体化，短路后续 SEQUENTIAL 阻塞；**F5 决策后**（IsMoving 用 OLD 严格公式无短路），第 2 步 `flag = ALL_FINISH_FLAG` 是释放 SEQUENTIAL 阻塞循环的**唯一手段**（详见决策 #5） | 物理上电机未到位但 flag 已全 1（不影响下次 MoveJ 时 0 复位）；依赖回包自然更新当前实际角度 |
| 4 | **`motorJ[ALL]` 代理作为广播发送者** | OLD 无此机制；current firmware_new 已实现 | 无 |
| 5 | **本模块第 2 步 `flag = ALL_FINISH_FLAG` 是模块 02 F5 决策的强制反向依赖** | F5 让 IsMoving 公式无短路，EmergencyStop 必须显式置 flag 才能让 IsMoving() 返回 false 释放 SEQUENTIAL 阻塞循环；若改动任一处必须同步联动 | 误移除第 2 步 → SEQUENTIAL 永久死锁；动机：v3.2 时代此步是冗余防御，F5 决策后升级为唯一兜底 |

---

## 实施

### 实施项：当前代码已正确，无需修改

**关键代码位置**：

| 文件 | 行 | 代码 |
|---|---|---|
| `ctrl_step.cpp` | `BroadcastEStop()` | StdId = `(nodeID<<7) \| 0x09`，motorJ[ALL] 调用时为 0x09 |
| `dummy_robot.cpp:744-753` | `EmergencyStop()` 4 步流程 |
| `ascii_protocol.cpp` | `!STOP` 精确匹配分支 |

**EmergencyStop 当前实现**（无需修改，参考）：
```cpp
void DummyRobot::CommandHandler::EmergencyStop()
{
    // 1. 发 0x09 广播（通过 motorJ[ALL]，StdId=(0<<7)|0x09=0x09）
    context->motorJ[ALL]->BroadcastEStop();
    // 2. 置 jointsStateFlag = ALL_FINISH_FLAG（物理已停，短路 IsMoving）
    context->jointsStateFlag = ALL_FINISH_FLAG;  // 0x7E
    // 3. 标记失能 + 清指令队列
    context->isEnabled = false;
    ClearFifo();
}
```

**禁止事项**：
- ❌ 不要改回 `MoveJ(currentJoints)` 伪急停（OLD 行为太慢不安全）
- ❌ 不要让 `!STOP` 走模糊匹配（`find("STOP")`）
- ❌ **不要省略** `jointsStateFlag = ALL_FINISH_FLAG` — F5 决策后（IsMoving 用 OLD 严格公式无短路），省略此行会导致 SEQUENTIAL 队列永久死锁（因为 `!DISABLE` / `!STOP` 后 IsMoving 不再短路返回 false，依赖本步骤显式置 flag 才能释放阻塞循环）

---

## 验证

| 测试 | 期望 | 命令 |
|---|---|---|
| !STOP 立即抱闸 | 6 电机立即停止（示波器看电流） | 发 `!STOP` 后查电机状态 |
| !STOP 后 IsMoving()=false | OLED 看状态行立即 `======` | 发 `!STOP` |
| !STOP 后 SEQUENTIAL 可消费 | 之后发 `>` 立即被处理（不卡死） | `!STOP → !START → >0,...` |
| ClearFifo 生效 | 阻塞在 while(IsMoving) 的指令清空 | 发 `>` 后立即 `!STOP` |
| 0x09 广播可见 | CAN 分析仪捕获 StdId=0x09 帧 | 挂分析仪观测 |

---

## 状态

- [x] 决策已记录（2026-09-24）
- [x] 0x09 广播急停已生效
- [x] !STOP 精确匹配已生效
- [x] EmergencyStop 4 步流程已生效
- [x] **2026-09-27 审查**：
    - EmergencyStop() 与文档 1:1 对应（dummy_robot.cpp:731-741）
    - 双通道 `!STOP` 精确匹配生效（ascii_protocol.cpp:40, 763）
    - 0x09 广播 + ALL 代理机制生效（ctrl_step.cpp:402-413）
    - 与模块 02 F5 决策的架构耦合已记录（决策 #5）
- 无需代码修改

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [02 - IsMoving](firmware_new主控固件模块02_运动完成判断-IsMoving.md) | EmergencyStop 通过 `flag = ALL_FINISH_FLAG` 短路 |
| [11 - 堵转协议通路](firmware_new主控固件模块11_堵转协议通路.md) | 0x09 不依赖 stall 检测，是独立安全机制 |
| [13 - 架构常量](firmware_new主控固件模块13_架构常量.md) | `ALL = 0`（motorJ[ALL] 代理广播） |
