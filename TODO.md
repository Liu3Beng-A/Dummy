# TODO — Dummy 7轴机械臂控制项目

> 本文档追踪**计划开发**的功能与待优化项。
> 已修复的 P0~P3 问题见 `ISSUES.md`。

---

## 优化项（低风险，随时可做）

### [OPT-1] 串口 `ok` 响应处理算法优化 🟡

**状态**：待优化（2026-08-31）

**背景**：
`serial_receive_loop` 中检测到固件回 `ok`（用于 SEQ 顺序发送下一条）时，当前路径是：

```
固件回 ok → 串口 RX buffer
  ↓ 最坏等 50ms
serial_receive_loop 轮询 in_waiting
  ↓
root.after(0, ...) 切主线程
  ↓ 主线程事件循环
_send_next_position → 串口 write
```

**当前问题**：
- 50ms 轮询间隔（即使有数据也需等待）
- 双层 `after(0)` 调度（子线程→主线程→子线程再 send_cmd）
- 实测 SEQ 点位间隔 60-90ms，其中大部分是 python 端等待

**优化方案**：
1. 空闲时轮询改为 10ms（原来 50ms）；有数据时立即连续读，不 sleep
2. SEQ 模式下，检测到 `line == "ok"` 时直接在子线程调用 `_send_next_position()`，跳过 `root.after(0)` 调度
3. UI 日志和滑块同步仍走 `after(0)`（Tk 线程安全要求）

**预期收益**：
- SEQ 点位间隔：60-90ms → **5-15ms**
- 改动量小（仅 `serial_receive_loop` 内部，约 5-10 行）

**风险评估**：
- 串口 `write` 线程安全（pyserial 内部加锁）✓
- `self._pos_queue_running` 等简单布尔读写受 GIL 保护 ✓
- 日志 `_log` 内部用 `after(0)`，子线程调用安全 ✓

**相关代码**：`串口助手.py` L1304 `serial_receive_loop`

---

## 待修复 Bug

### [BUG-1] `!START` 不自动归位（需先 `!HOME` 才会摆正） 🔴

**状态**：待修复（2026-09-01）

**现象**：
- 用户执行 `SET_ORIGIN`（`ApplyPositionAsHome`）把当前机械臂姿态设为 0°
- 然后发送 `!START` 使能电机
- 机械臂**不会**主动走到 home 姿态（`MoveJ(0,0,90,0,0,0,0,10)`）
- 必须再发 `!HOME` 才会触发归位运动

**期望行为**：
- `!START` 后电机使能的同时，应自动归位到 home 姿态
- 让"启动 = 上电 + 摆正"符合直觉
- 或者至少提供一个 `!START_NOHOME` 让用户按需选择

**根因**：
- `DummyRobot::SetEnable(true)`（`dummy_robot.cpp:622`）只调用 `SetRGBMode(rgbStateEnable)`，**不**触发任何 MoveJ
- `DummyRobot::Homing()`（`dummy_robot.cpp:585`）才是负责 MoveJ 到 home 姿态的唯一入口
- UI 视角下"启动"和"归位"被拆成了两步，但用户心理模型是"启动 = 安全姿态就绪"

**复现步骤**：
1. 上电，机械臂停在非 home 姿态（例如 J1=90°）
2. 串口发送 `SET_ORIGIN_J 1` → 把当前角度设为 0°
3. 串口发送 `!START` → 电机上电，**J1 仍停在原物理角度**，不归零
4. 串口发送 `!HOME` → 立即开始 MoveJ 到归零姿态

**风险**：
- 用户以为已就绪，实际上电机带电但姿态是任意角度
- 后续 MoveJ/MoveL 目标点可能与当前角度相差很大 → 启动瞬间大电流冲击
- 误操作夹爪/抓取时容易撞限位

**修复方向**（任选其一）：
1. **简单方案**：`!START` handler 改为先 `SetEnable(true)` 再 `Homing()`（`ascii_protocol.cpp:78` 改两行）
2. **进阶方案**：增加 `!START_NOHOME` / `!START_HOME` 两条命令，让用户显式选择
3. **可配置方案**：在 Flash 存一个 `auto_home_on_start` 配置位（默认 ON），`!START` 据此决定是否归位
4. **REST_POSE 触发**：`!START` 改为先 `Homing()` 再 `Resting()`，让启动直接进入待机位

**相关代码**：
- `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp:78` `!START` handler
- `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp:786` UART4 同名 handler（USB/UART 两份要同步改）
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:622` `DummyRobot::SetEnable`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:585` `DummyRobot::Homing`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:603` `DummyRobot::Resting`

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
