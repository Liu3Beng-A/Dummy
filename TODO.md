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

### [BUG-4] `!STALL_STATUS` 查询时节点 5、6 不回包 🟡

**状态**：待修复（2026-09-01，发现于现场实测）

**现象**：
- 现场发送 `!STALL_STATUS` 后，节点 9、1、2、3、4 都回了 `[STALL_STATUS] node=X en=1`
- **节点 5、6 完全没有响应**
- 重发多次仍然只收到 5 个电机回包
- 35/42/57 三套电机固件源码都已实现 `case 0x5C` 处理（已 grep 验证 `motor_fw_f103_35/42/57/UserApp/protocols/interface_can.cpp:412`），软件层不缺

**可能原因**（按概率排序）：
1. **J5、J6 电机未通电或 CAN 线松脱**（最常见，需现场确认）
2. J5、J6 电机固件烧录版本不一致或 Flash 数据损坏（可能性低）
3. CAN 总线终端电阻未接 / 总线拓扑问题导致节点 5、6 信号衰减
4. 节点 5、6 处于 `STATE_FAULT`/`MODE_STOP` 等异常状态，无法响应 0x5C

**修复方向**：
1. **现场排查（优先）**：检查 J5、J6 电机指示灯、CAN 线连接、终端电阻
2. 若硬件无问题，加固 `QueryStallStatus()`：对每个节点超时重试 2~3 次（CAN 自动重发机制已存在，但可应用层再加一层重试）
3. 加诊断命令 `#STALL_DIAG` 主动 ping 节点 5、6，确认链路是否通畅

**相关代码**：
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:574-584` `QueryStallStatus` 顺序发送 14 条 0x5C
- `firmware/motor_fw_f103_35/UserApp/protocols/interface_can.cpp:412` J5、J6 (id 5/6) 0x5C 处理
- `firmware/motor_fw_f103_35/Core/Src/can.c:204` 35 电机 `cmd >= 0x50 && cmd != 0x5C` 广播/单播分发

---

### [BUG-5] 串口 `ClearCommError failed (设备不识别此命令)` 卡死 🟡

**状态**：待修复（2026-09-01，发现于现场实测）

**现象**：
发送 `!STALL_STATUS` 后约 14 秒，串口助手出现：

```
[ERROR] 发送失败: Write timeout          × 3
[ERROR] 串口异常: ClearCommError failed (PermissionError(13, '设备不识别此命令。', None, 22))
```

之后所有串口写入失败，必须**手动断开重连**才能恢复。

**根因**：
这是 Windows USB 转串口驱动进入异常状态，errno 22 (EINVAL) 是 pyserial 调 `ClearCommError()` 失败时返回的错误。

可能诱因：
1. **UART4 接收缓冲区溢出**：`!STALL_STATUS` 触发 14 条 CAN 回包，每条回包用 printf 输出 `\r\n` 行，UART4 一瞬间被灌入大量文本，可能撑爆 DMA/缓冲区
2. **pyserial 内部状态机错乱**：`serial_receive_loop` 在 `in_waiting > 0` 读取和后续 write 之间没有同步保护（`串口助手.py:1308-1310`）
3. **USB 设备自身异常**：USB 转串口芯片（CH340/CP2102）固件 bug，需重新插拔恢复

**修复方向**：
1. **方案 A（推荐，先做）**：Python 串口助手加自动重连机制——捕获 `PermissionError`/`SerialException` 后自动 `close()` + `open()`，UI 显示"已自动重连"
2. **方案 B**：`!STALL_STATUS` 改为分批发送（7 个电机一组，间隔 50~100ms），避免 UART4 瞬间被灌爆
3. **方案 C**：主控侧加 UART4 输出限流（printf 排队，间隔 1~2ms 输出）
4. **方案 D**：硬件层换更稳定的 USB 转串口芯片（如 CP2102N 替代 CH340G）

**相关代码**：
- `串口助手.py:1288` 串口初始化（`timeout=0.5`, `write_timeout=1.0`）
- `串口助手.py:1308-1380` `serial_receive_loop` 主循环
- `串口助手.py:1382-1388` 异常处理（仅 `log` + `break`，未做自动重连）
- `firmware/ref_core_f405/UserApp/protocols/can_protocol.cpp:197-200` printf 输出路径（UART4 收发易堵）

---

### [BUG-6] slider 1~100 实际速度变化不明显；Homing/Resting 速度值不明确 🟡

**状态**：待调查（2026-09-01，现场实测）

**现象**：
1. **slider 调节不明显**：Python UI 端「关节速度」slider 从 1 调到 100，实测电机转速看不出明显差异
2. **home/reset 速度值含义不清**：`Homing()` / `Resting()` 中硬编码 `MoveJ(..., 0, 10)` 的 `10` 含义不明，与用户 slider 控件的关系也未明示

**当前代码（`dummy_robot.cpp:587-619`）**：
```cpp
void DummyRobot::Homing() {
    float lastSlider = jointSpeedRps / SLIDER_TO_RPS;
    SetJointSpeed(10);
    MoveJ(0, 0, 90, 0, 0, 0, 0, 10);  // 最后一个 10 = slider
    ...
}
```

slider 换算：`slider × SLIDER_TO_RPS = sliderSpeed`，例如：
- `slider=10` → `sliderSpeed = 3.0 r/s`（电机轴目标速度上限）
- `slider=25` → `sliderSpeed = 7.5 r/s`
- `slider=100` → `sliderSpeed = 30 r/s`（达到 AXIS_MAX_RPS 上限）

**可能根因**（待逐一排查）：
1. **ComputeSyncSpeeds 算法约束**：距离短时退化为三角曲线，最终速度被 `√(d × a)` 限制，slider 调大无效
2. **jointAccRuntime 未及时回填**：上电后 `SyncAllMotorAcceleration()` 异步查电机端 EEPROM，**回包延迟或失败**时 `jointAccRuntime[i]=0`，触发 `FALLBACK_JOINT_ACCELERATION = 10 r/s²`（电机端默认 1000 r/s²，差 100 倍）
3. **slider 上限被钳制**：`AXIS_MAX_RPS[7] = 30.0f`（关节）`/ 30.0f`（地轨）滑不到更高
4. **距离太短触发三角曲线**：梯形 → 三角退化条件 `2 × d_acc >= d`，此时 `v_max = √(d × a)` 而非 slider 决定
5. **电机端加速度 EEPROM 实际值很小**：可能被人为改小过，与主控默认假设不符

**home/reset 速度值不明确的根因**：
- `Homing()` / `Resting()` 中 `MoveJ(..., 10)` 的 `10` 是 **slider**（无量纲）
- 经 `SLIDER_TO_RPS = 0.30f` 换算成 3 r/s 后才进入速度规划
- 这个 10 与用户 UI 端 slider 没有联动，每次 Home/Reset 都用固定的 10
- 用户无法知道"Home 到底跑多快"，调 UI slider 也不影响 Home 速度

**调查方向**：
1. **串口助手发 `#SYNC_ACC`** 触发同步，观察 `jointAccRuntime[]` 是否有非零值（验证根因 2）
2. **加调试日志**：在 `ComputeSyncSpeeds` 输出每轴实际生效的 `v`、`a`、`d`（验证根因 1、4）
3. **量化对比**：固定距离（如 J3 转 1 圈、5 圈、12.5 圈），分别用 slider=10/25/50/100 测速，记录电机端实际 RPM
4. **重构 Homing/Resting 速度**：把硬编码的 `10` 改为有名字的常量（如 `HOME_SLIDER = 50`），或运行时从用户 slider 读取
5. **修复回包超时**：`SyncAllMotorAcceleration` 改为同步等待（带超时），或加 fallback 标记便于调试

**相关代码**：
- `firmware/ref_core_f405/Robot/instances/dummy_robot.h:14` `SLIDER_TO_RPS = 0.30f`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.h:21-27` `AXIS_MAX_RPS[7]`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.h:41` `FALLBACK_JOINT_ACCELERATION = 10.0f`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:38-48` `timeTrapezoid` 算法
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:62-127` `ComputeSyncSpeeds` 速度规划
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:587-619` `Homing()` / `Resting()`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:233-235` `Init()` 末尾 `SyncAllMotorAcceleration()`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:527-535` `SyncAllMotorAcceleration()` 实现

---

## 附录：本轮新增项速览

| ID | 标题 | 优先级 | 发现日期 |
|----|------|--------|----------|
| BUG-3 | `!STALL_STATUS` 响应硬编码、格式/通道不匹配 | 🔴 P0 | 2026-09-01 |
| BUG-4 | `!STALL_STATUS` 查询时节点 5、6 不回包 | 🟡 P1 | 2026-09-01 |
| BUG-5 | 串口 `ClearCommError failed` 卡死 | 🟡 P1 | 2026-09-01 |
| BUG-6 | slider 1~100 实际速度变化不明显；Homing/Resting 速度值不明确 | 🟡 P1 | 2026-09-01 |
