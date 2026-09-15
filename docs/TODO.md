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

### [BUG-ESTOP] `!STOP` 急停指令电机无响应（合并自 [BUG-2] + [BUG-4]） 🔴

**状态**：待修复（2026-09-15 合并）

**现象 A — 空闲态发 `!STOP`**：
- 机械臂静止时（电机已 FINISH）发 `!STOP`
- `EmergencyStop()` 立即返回 `"ok"`
- 表面"急停成功"，实际电机本来就在 FINISH 状态，`!STOP` 没起任何作用
- 这种"假成功"掩盖了真实 bug

**现象 B — `!HOME` / `!RESET` 中发 `!STOP`**：
- 机械臂在长轨迹运行中（电机 RUNNING）发 `!STOP`
- `EmergencyStop()` 立即返回 `"ok"`
- 但电机物理上继续移动到 home/rest 姿态才停下
- 用户观察到"`!STOP` 完全无效"

**根因 1：`BroadcastEStop()` 发出的 CAN 帧被所有电机静默丢弃**

`CtrlStepMotor::BroadcastEStop()`（`ctrl_step.cpp:423-429`）发送 `StdId = 0x89`：

```cpp
void CtrlStepMotor::BroadcastEStop()
{
    txHeader.StdId = 0x89;
    uint8_t buf[8] = {0};
    CanSendMessage(get_can_ctx(hcan), buf, &txHeader);
}
```

电机 Rx 路由（`motor_fw_f103_35/Core/Src/can.c:189-211`，35/42/57/gripper 同构）解析：

```c
uint8_t id  = (RxHeader.StdId >> 7);   // 0x89 >> 7  = 1
uint8_t cmd = (RxHeader.StdId & 0x7F); // 0x89 & 0x7F = 0x09  ← 关键

if (cmd >= 0x50 && cmd != 0x5C) {       // cmd=0x09 < 0x50, 不进广播分支
    OnCanCmd(cmd, RxData, RxHeader.DLC);
}
else if (id == boardConfig.canNodeId) { // 只有 id==nodeID 才进单播分支
    OnCanCmd(cmd, RxData, RxHeader.DLC);
}
```

对 `StdId=0x89` 的实际处理结果：
- 实际广播区间是 `0x50~0x7F`（不是 `0x80~0xBF`），0x09 不在范围内
- J1（canNodeId=1）：id==nodeID → 调 `OnCanCmd(0x09)`，但 `case 0x09` 在 `interface_can.cpp` 不存在 → **J1 也丢弃**
- J2~J6 / 地轨 / 夹爪：id≠nodeID → **全部丢弃**
- 结果：**0 个电机收到 STOP 命令**

**根因 2：代码注释与实现自相矛盾**

`BroadcastEStop()` 注释写"落入电机端广播区间 0x80~0xBF，所有电机节点无条件响应"，但电机实际代码用 `cmd >= 0x50` 判断广播。这是编码设计错误——11-bit StdId 用 `(nodeID<<7)|cmd` 编码时 cmd 只有 7 bit（0x00~0x7F），根本不存在 0x80~0xBF 区间。

**对比参考 — `BroadcastUnlock()` 的成功路径**：

```cpp
void CtrlStepMotor::BroadcastUnlock()
{
    txHeader.StdId = 0x5B;  // (0<<7)|0x5B,cmd=0x5B ∈ 广播区间 0x50~0x7F
    // ...
}
```
0x5B 的 cmd=0x5B 真正落在广播区间，所有电机响应 `case 0x5B`，UNLOCK 能正常工作。

**根因 3：`EmergencyStop()` 后续 3 行的副作用**

```cpp
void DummyRobot::CommandHandler::EmergencyStop()  // dummy_robot.cpp:749-757
{
    context->motorJ[1]->BroadcastEStop();  // 0x89 帧（已被电机丢弃，见根因 1）
    context->jointsStateFlag = 0;          // ← 掩盖电机真实状态，IsMoving() 假返 false
    context->isEnabled = false;             // ← 停止 5kHz 实时环下发新位置，但不直接抱闸电机
    ClearFifo();                            // ← 清指令队列
}
```

- `isEnabled=false` 只让 5kHz 实时环（`main.cpp:35-58`，检查 `dummy.IsEnabled()`）不再下发新位置命令
- 但电机继续执行**最后一次下发的位置命令**——这解释了"现象 B"中机械臂仍跑到 home/rest 才停
- `jointsStateFlag=0` 让 `IsMoving()` 返回 false，掩盖电机真实运行状态

**当前 `IsMoving()` 逻辑**（`dummy_robot.cpp:675-677`）：

```cpp
bool DummyRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110;
}
```

已不是位置误差判断，而是基于 `jointsStateFlag` 位运算（v2.7 A3 修复后）。`jointsStateFlag` 在 0x23 回包中由 `UpdateAngleCallback` 维护，但 `EmergencyStop()` 手动清零后让 IsMoving 失去意义。

**相关代码**：
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:749-757` `DummyRobot::CommandHandler::EmergencyStop()`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:675-677` `DummyRobot::IsMoving()`
- `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp:423-429` `CtrlStepMotor::BroadcastEStop()`
- `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp:415-419` `CtrlStepMotor::BroadcastUnlock()`（对比参考）
- `firmware/ref_core_f405/UserApp/main.cpp:35-58` 5kHz 实时环 `IsEnabled()` 检查点
- `firmware/motor_fw_f103_35/Core/Src/can.c:189-211` 电机 Rx 路由（35/42/57/gripper 同构，文件名 `motor_fw_f103_XX/Core/Src/can.c`）
- `firmware/motor_fw_f103_35/UserApp/protocols/interface_can.cpp:358-369` 电机 `case 0x89` 急停处理（35/42/57/gripper 同构，文件名 `motor_fw_f103_XX/UserApp/protocols/interface_can.cpp`）

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

### [CLEANUP-2] 删除 `!CALIBRATION` 标定命令 🟡

**状态**：待清理（2026-09-03）

**说明**：
当前 `!CALIBRATION` 命令对 `motorJ[1..6]` 各发一次 `ApplyPositionAsHome()`（CAN 0x15）。经核实全工程无其他调用方，决定彻底删除该 ASCII handler。

**代码现状**：
- `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp:853-861` —— 唯一 handler
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp` —— 无引用
- `CtrlStepMotor::ApplyPositionAsHome()` —— **必须保留**（`!HAND_ZERO` 仍调用此方法标定夹爪零点）

**删除步骤**：
1. 删除 `ascii_protocol.cpp:853-861` 的 `else if (s.find("!CALIBRATION") == 0) { ... }` 整段
2. 同步删除 `docs/README.md` 系统命令表中 `!CALIBRATION` 一行
3. 同步删除 `.cursor/rules/PROJECT_GUIDANCE.mdc` 系统命令表中 `!CALIBRATION` 一行
4. 重新编译并验证 0 errors
5. 灰度：先注释 handler 一周观察无业务再彻底删除

**不要碰**：
- `CtrlStepMotor::ApplyPositionAsHome()`（夹爪标定需要）
- `motor_fw_f103_*` 侧的 0x15 命令解析（电机端 CAN 命令仍需保留，夹爪用）

---

### [CLEANUP-1] 删除 CAN 0x7C 旧堵转兼容代码 🟡

**状态**：待清理（2026-09-03）

**说明**：
电机固件已全面升级到 v3 堵转协议（使用 0x5A/0x5B/0x5C），主控 `can_protocol.cpp` 中残留 3 处 `case 0x7C:` 旧兼容分支不再需要，应删除。

**待删除代码**：

| 文件 | 行号 | 内容 |
|------|------|------|
| `can_protocol.cpp` | L204~208 | `case 0x7C:` 地轨 (id==9) 分支 |
| `can_protocol.cpp` | L322~326 | `case 0x7C:` 关节 (id 1~6) 分支 |
| `can_protocol.cpp` | L412~416 | `case 0x7C:` 夹爪 (id==8) 分支 |

**相关代码**：
- `firmware/ref_core_f405/UserApp/protocols/can_protocol.cpp`

---
