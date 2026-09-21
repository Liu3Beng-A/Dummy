# 电机固件对比 — `firmware_old` → `firmware` 迁移参考

> **范围**：仅电机固件（STM32F103，4 套：`motor_fw_f103_35` / `_42` / `_57` / `_gripper`）。
> **基线对比**：`firmware_old/dummy-35motor-fw` 与 `firmware_old/dummy-42motor-fw`（旧）；`firmware/motor_fw_f103_`*（新）。
> **数据来源**：直接读源码，每条标注 `文件:行号`。**文档聚焦"差异"**，未变化的部分不列。
> **创建日期**：2026-09-21
>
> ### ⚠️ 核心迁移原则
>
> **firmware_new 不复刻堵转检测功能。** 所有与堵转检测（stall）相关的改动，一律退回 old 或删除。
>
> - 堵转检测 v3 协议（`StallMode_t` 状态机、`0x5A/0x5B/0x5C`、回退机制等）**全部删除**
> - 旧固件已有的 `State_t::STATE_STALL` 和 `isStalled` 字段**保留**
> - 旧固件的 `ClearStallFlag()` 方法**保留**
> - 迁移原则：在保证功能可用（位置控制、速度控制、电流控制、EEPROM 持久化）的前提下，**改动量最小化**
>
> **本迁移的目标**：firmware_new ≈ firmware_old + 必要的 Bug 修复 + 新增的无害查询命令

---

## 一、CAN 协议对比（`UserApp/protocols/interface_can.cpp`）

> 所有 cmd 由 `OnCanCmd(_cmd, _data, _len)` 解析。CAN ID 格式：`(nodeID<<7) | cmdCode`，8 字节数据。
> 旧固件共 26 条 cmd（35/42 各一份，内容一致）。新固件退回旧后共 **27 条**（删除 0x5A/0x5B/0x5C 共 3 条，保留 0x28-0x2D）；夹爪 26 条（删除 0x5A/0x5B/0x5C 共 3 条，保留 0x28-0x2D）。

### 1.1 命令清单


| Cmd           | 方向     | 名称                            | 旧固件                                                              | 新 35/42/57                                                                            | 新 gripper               | 标记             | new（手动标记）                                                 |
| ------------- | ------ | ----------------------------- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------------- | ----------------------- | -------------- | --------------------------------------------------------- |
| `0x01`        | 主→电    | Enable Motor                  | 切 `requestMode` 到 VELOCITY/STOP                                  | 切 `requestMode` 到 VELOCITY/STOP（新增 LOCKED→IDLE + ResetGoals + 启动豁免复位是 stall 专属，退回旧）   | 同旧                      | **重构（退回旧）**    | **退回旧**                                                   |
| `0x02`        | 主→电    | Do Calibration                | 触发编码器校准                                                          | 同                                                                                     | 同                       | —              | 同旧                                                        |
| `0x03`        | 主→电    | Set Current                   | 切 `MODE_COMMAND_CURRENT` + `SetCurrentSetPoint(A×1000)`          | 同                                                                                     | 同                       | —              | 同旧                                                        |
| `0x04`        | 主→电    | Set Velocity                  | 切 `MODE_COMMAND_VELOCITY` + `SetVelocitySetPoint(r/s×SUBDIVIDE)` | 同                                                                                     | 同                       | —              | 同旧                                                        |
| `0x05`        | 主→电    | Set Position                  | 切 POSITION 模式 + `SetPositionSetPoint(圈×SUBDIVIDE)`；可选 FINISH ACK | 切 POSITION 模式 + `SetPositionSetPoint`（LOCKED/RETREATING 丢弃是 stall 专属，退回旧）             | 同旧（gripper 无 stallMode） | **重构（退回旧）**    | **退回旧**                                                   |
| `0x06`        | 主→电    | Set Pos w/ Time               | `SetPositionSetPointWithTime(pos, T)`；可选 FINISH ACK              | **微调**：`sqrtf` 加 `fmaxf(0, discriminant)` 兜底防负数；其余同                                   | 同旧（无 fmaxf 保护）          | **微调**         | **保留新fmaxf 防 crash 是安全增强**                                |
| `0x07`        | 主→电    | Set Pos w/ VelLimit           | 写 `ratedVelocity` + `SetPositionSetPoint`；**总是**立即发 FINISH ACK   | 写 `ratedVelocity` + `SetPositionSetPoint`（LOCKED 丢弃 + 取消立即 FINISH ACK 是 stall 专属，退回旧） | 同旧（无 LOCKED 丢弃）         | **重构（退回旧）**    | **退回旧：LOCKED 丢弃 + 取消立即 ACK 仅服务于 stall 检测；不实现 stall 时应删除** |
| `0x09`        | 主→ALL  | Broadcast EStop               | ❌                                                                | **新增**：`requestMode=STOP` + `SetBrake(true)` + Cur/Vel=0                              | **新增**                  | **新增**         | **保留新：安全急停功能保留**                                          |
| `0x11`        | 主→电    | Set NodeID + EPP              | 写 `canNodeId` + 可选持久化                                            | 同                                                                                     | 同                       | —              | 同旧                                                        |
| `0x12`        | 主→电    | Set I-Limit + EPP             | 写 `ratedCurrent = A×1000` + `currentLimit` + 持久化                 | 写 `ratedCurrent` + `currentLimit` + 持久化（同步 `stallCurrentThreshold` 是 stall 专属，退回旧）    | 同旧（无 stall 阈值更新）        | **重构（退回旧）**    | **退回旧：**`stallCurrentThreshold` **仅服务于 stall 检测，应删除**     |
| `0x13`        | 主→电    | Set V-Limit + EPP             | 写 `ratedVelocity` + `velocityLimit` + 持久化                        | 同                                                                                     | 同                       | —              | 同旧：速度限幅无变化                                                |
| `0x14`        | 主→电    | Set Accel + EPP               | 写 `ratedVelocityAcc` + 同步两个 tracker                              | 同                                                                                     | 同                       | —              | 同旧：加速度设置无变化                                               |
| `0x15`        | 主→电    | Apply Home + EPP              | **Bug**：仅改 `encoderHomeOffset`，`GetPosition()` 不归零               | **Bug 修复**：同步 `realPosition = encoderHomeOffset`，`GetPosition()` 立即归零                 | 同新                      | **重构**         | **保留新：Bug 修复，必须保留；旧固件归零后 GetPosition() 不归零**              |
| `0x16`        | 主→电    | Set Auto-Enable + EPP         | 写 `enableMotorOnBoot`                                            | 同                                                                                     | 同                       | —              | 同旧：上电使能无变化                                                |
| `0x17`-`0x1A` | 主→电    | Set DCE Kp/Kv/Ki/Kd           | 写 `dce.{kp,kv,ki,kd}` + 持久化                                      | 同                                                                                     | 同                       | —              | 同旧：PID 参数设置无变化                                            |
| `0x1B`        | 主→电    | Set Stall-Protect             | 写 `stallProtectSwitch` + `enableStallProtect` + 持久化              | 写 `stallProtectSwitch` + `enableStallProtect` + 持久化（不实现 stall 时此命令可删除或保留为占位）          | ❌ 无此命令                  | **重构（退回旧/删除）** | **退回旧：不实现 stall 检测时，堵转保护开关无意义；建议删除此命令或退回旧固件行为（写 EEPROM）** |
| `0x21`        | 主→电    | Get Current                   | 回 float A                                                        | 同                                                                                     | 同                       | —              | 同旧：电流查询无变化                                                |
| `0x22`        | 主→电    | Get Velocity                  | 回 float r/s                                                      | 同                                                                                     | 同                       | —              | 同旧：速度查询无变化                                                |
| `0x23`        | 主→电    | Get Position                  | 回 float 圈 + finish 标志                                            | 同                                                                                     | 同                       | —              | 同旧：位置查询无变化                                                |
| `0x24`        | 主→电    | Get Offset                    | 回 int32 `encoderHomeOffset`                                      | 同                                                                                     | 同                       | —              | 同旧：偏移查询无变化                                                |
| `0x25`        | 主→电    | Get Temperature               | 回 float `motor_temperature`                                      | 同                                                                                     | 同                       | —              | 同旧：温度查询无变化                                                |
| `0x28`        | 主→电    | Get DCE Kp                    | ❌                                                                | **新增**：回 int32 `boardConfig.dce_kp`                                                   | **新增**                  | **新增**         | **保留新：GET 查询命令，仅读参数**                                     |
| `0x29`        | 主→电    | Get DCE Kv                    | ❌                                                                | **新增**：回 int32 `boardConfig.dce_kv`                                                   | **新增**                  | **新增**         | **保留新：GET 查询命令，仅读参数**                                     |
| `0x2A`        | 主→电    | Get DCE Ki                    | ❌                                                                | **新增**：回 int32 `boardConfig.dce_ki`                                                   | **新增**                  | **新增**         | **保留新：GET 查询命令，仅读参数**                                     |
| `0x2B`        | 主→电    | Get DCE Kd                    | ❌                                                                | **新增**：回 int32 `boardConfig.dce_kd`                                                   | **新增**                  | **新增**         | **保留新：GET 查询命令，仅读参数**                                     |
| `0x2C`        | 主→电    | Get Acceleration              | ❌                                                                | **新增**：回 float `velocityAcc / SUBDIVIDE`（r/s²）                                        | **新增**                  | **新增**         | **保留新：GET 查询命令，仅读参数**                                     |
| `0x2D`        | 主→电    | Get Current-Limit（FOC 电流硬限幅值） | ❌                                                                | **新增**：回 float `currentLimit / 1000`（A）                                               | **新增**                  | **新增**         | **保留新：GET 查询命令，仅读参数**                                     |
| `0x5A`        | 触发电→他机 | STALL Broadcast               | ❌                                                                | **新增**：stall 协作广播（触发/完成/超时）；其他电机收 → LOCKED + ClearIntegral                            | ❌                       | **新增（删除）**     | **退回旧：整个 0x5A 是 stall 专属，不实现 stall 时必须删除**                |
| `0x5B`        | 主→ALL  | UNLOCKED Broadcast            | ❌                                                                | **新增**：所有 LOCKED 电机 → IDLE + ResetGoals + 启动豁免复位                                      | ❌                       | **新增（删除）**     | **退回旧：整个 0x5B 是 stall 专属，不实现 stall 时必须删除**                |
| `0x5C`        | 主→单电   | Stall Query                   | ❌                                                                | **新增**：stall 状态查询                                                                     | ❌                       | **新增（删除）**     | **退回旧：整个 0x5C 是 stall 专属，不实现 stall 时必须删除**                |
| `0x7D`        | 主→电    | Enable Temp Watch             | TIM1 启动 1Hz 温度采集                                                 | 同                                                                                     | 同                       | —              | 同旧：温度监控无变化                                                |
| `0x7E`        | 主→电    | Erase Configs                 | `configStatus = CONFIG_RESTORE` → SystemReset                    | 同                                                                                     | 同                       | —              | 同旧：配置擦除无变化                                                |
| `0x7F`        | 主→电    | Reboot                        | `HAL_NVIC_SystemReset()`                                         | 同                                                                                     | 同                       | —              | 同旧：重启无变化                                                  |


### 1.2 协议帧格式约定


| Cmd                      | TxData[0..3]                   | TxData[4..7]             | new（手动标记）                |
| ------------------------ | ------------------------------ | ------------------------ | ------------------------ |
| `0x03/0x04/0x05/0x07` 前半 | float pos / cur / vel          | (见下)                     | 同旧                       |
| `0x06`                   | float pos（圈）                   | float time（秒）            | 同旧                       |
| `0x07`                   | float pos（圈）                   | float vel-limit（r/s）     | 同旧                       |
| `0x11`-`0x16`            | 主参数字段                          | data[4]=commit flag（0/1） | 同旧                       |
| `0x17`-`0x1A`            | int32 PID 分量                   | data[4]=commit flag      | 同旧                       |
| `0x12`-`0x14` ACK 时      | 回 float (A / r/s / r/s²)       | data[4]=0                | 同旧                       |
| `0x21`-`0x25` ACK 时      | 回 float 当前值                    | data[4]=STATE_FINISH 标志  | 同旧                       |
| `0x28`-`0x2B` ACK 时      | int32 PID 分量                   | —                        | 同旧                       |
| `0x5A`                   | data[0]=srcNodeId, data[1]=cmd | —                        | **退回旧：0x5A 整体删除，帧格式不移植** |
| `0x5C`                   | data[0]=queryType (1/2)        | —                        | **退回旧：0x5C 整体删除，帧格式不移植** |


---

## 二、电机控制层（`Ctrl/Motor/motor.cpp` + `motor.h`）

### 2.1 Controller 类方法对比


| 方法签名                                                  | 旧固件                                              | 新固件                                                                                     | 标记          | new（手动标记）                                                               |
| ----------------------------------------------------- | ------------------------------------------------ | --------------------------------------------------------------------------------------- | ----------- | ----------------------------------------------------------------------- |
| `Tick20kHz()`                                         | 编码器采样 + 控制环                                      | 同                                                                                       | —           | 同旧：控制环框架无变化                                                             |
| `CloseLoopControlTick()`                              | 19kHz 控制环                                        | 19kHz 控制环（增加 stall 检测 + LOCKED 维持 + 状态输出是 stall 专属，退回旧）                                 | **重构（退回旧）** | **退回旧：stall 检测 + LOCKED 维持是 stall 专属；不实现 stall 时应删除**                   |
| `CalcCurrentToOutput(int32_t)`                        | FOC 电流环                                          | 同                                                                                       | —           | 同旧：FOC 电流环无变化                                                           |
| `CalcPidToOutput(int32_t)`                            | PID 速度环                                          | 同                                                                                       | —           | 同旧：PID 速度环无变化                                                           |
| `CalcDceToOutput(int32_t,int32_t)`                    | DCE 位置环                                          | 同                                                                                       | —           | 同旧：DCE 位置环无变化                                                           |
| `SetCtrlMode(Mode_t)`                                 | 写 `requestMode`                                  | 同                                                                                       | —           | 同旧：模式切换无变化                                                              |
| `SetPositionSetPoint(int32_t)`                        | `goalPosition = _pos + encoderHomeOffset`        | 同                                                                                       | —           | 同旧：位置设定无变化                                                              |
| `SetPositionSetPointWithTime(int32_t,float)` → `bool` | 算 pMax = acc·T²/4；超限返回 false + 用 `velocityLimit` | **微调**：`sqrtf` 加 `fmaxf(0, discriminant)` 兜底（防 sqrt 负数）；其余同                             | **微调**      | **保留新：fmaxf 防 crash 是安全增强**                                             |
| `SetVelocitySetPoint(int32_t)`                        | 限幅到 `±ratedVelocity`                             | 同                                                                                       | —           | 同旧：速度限幅无变化                                                              |
| `SetCurrentSetPoint(int32_t)`                         | 限幅到 `±ratedCurrent`                              | 同                                                                                       | —           | 同旧：电流限幅无变化                                                              |
| `SetDisable(bool)`                                    | 写 `goalDisable`                                  | 同                                                                                       | —           | 同旧：失能控制无变化                                                              |
| `SetBrake(bool)`                                      | 写 `goalBrake`                                    | 同                                                                                       | —           | 同旧：刹车控制无变化                                                              |
| `AddTrajectorySetPoint`                               | 同时设位置 + 速度                                       | 同                                                                                       | —           | 同旧：轨迹设定无变化                                                              |
| `GetPosition(bool=false)`                             | `(realPosition - offset) / SUBDIVIDE`            | 同                                                                                       | —           | 同旧：位置获取无变化                                                              |
| `GetVelocity()`                                       | `estVelocity / SUBDIVIDE`                        | 同                                                                                       | —           | 同旧：速度获取无变化                                                              |
| `GetFocCurrent()`                                     | `focCurrent / 1000`                              | 同                                                                                       | —           | 同旧：电流获取无变化                                                              |
| `CompensateAdvancedAngle(int32_t)`                    | 5 段查表                                            | 同                                                                                       | —           | 同旧：角度补偿无变化                                                              |
| `Init()`                                              | 状态变量全部清零                                         | 状态变量全部清零（新增 stall 字段清零是 stall 专属，退回旧）                                                   | **重构（退回旧）** | **退回旧：不实现 stall 时，stall 状态字段不存在，无需额外清零**                                |
| `AttachConfig(Config_t*)`                             | 绑 config                                         | 同                                                                                       | —           | 同旧：配置绑定无变化                                                              |
| `ClearIntegral() const`                               | 清 PID/DCE `integralRound/outputKi`               | 清 PID/DCE `integralRound/outputKi`（仅清 estimator 状态是新 stall 专属，退回旧）                      | **重构（退回旧）** | **退回旧：旧逻辑清 PID/DCE 积分是已知行为；新逻辑服务于 stall estimator 状态机，不实现 stall 时应退回旧** |
| `ClearStallFlag()`                                    | 重置 `stalledTime + isStalled`                     | 保留（不删除）                                                                                 | —           | **退回旧：不实现 stall 检测时，**`ClearStallFlag()` **仍可保留用于清除旧固件遗留状态**            |
| `ApplyPosAsHomeOffset()`                              | `encoderHomeOffset = realPosition % SUBDIVIDE`   | **Bug 修复**：除改 offset 外，**同步** `realPosition = encoderHomeOffset`，让 `GetPosition()` 立即归零 | **重构**      | **保留新：Bug 修复，必须保留；旧固件归零后 GetPosition() 不归零**                            |
| `ResetGoalsToCurrentPosition()`                       | ❌                                                | ❌（删除）                                                                                   | —           | **退回旧：此方法仅服务于 stall 检测的 LOCKED→IDLE 退出；不实现 stall 时应删除**                 |
| `GetEstPosition()`                                    | ❌                                                | ❌（删除）                                                                                   | —           | **退回旧：此方法仅服务于 stall 检测状态机；不实现 stall 时应删除**                              |
| `GetEstVelocity()`                                    | ❌                                                | ❌（删除）                                                                                   | —           | **退回旧：此方法仅服务于 stall 检测状态机；不实现 stall 时应删除**                              |


### 2.2 状态机变化

> **迁移策略**：保留旧固件的 stall 基础框架，删除 stall v3 状态机。


| 项                                                     | 旧固件                  | 新固件                                               | 标记  | new（手动标记）                                                               |
| ----------------------------------------------------- | -------------------- | ------------------------------------------------- | --- | ----------------------------------------------------------------------- |
| `Mode_t` (9 个模式)                                      | 同                    | 同                                                 | —   | 同旧：模式定义无变化                                                              |
| `State_t::STATE_STALL`                                | `isStalled=true` 时进入 | `isStalled=true` 时进入（同旧；不在此处变更）                   | —   | **退回旧：保留旧固件 STATE_STALL 触发机制；不实现 stall v3 检测时不改变此逻辑**                   |
| `State_t::STATE_OVERLOAD`                             | 有                    | 有（不删除；stall v3 状态机被删除后，STATE_OVERLOAD 可继续保留或后续清理） | —   | **退回旧：不实现 stall v3 时，STATE_OVERLOAD 可保留；不影响基本功能**                       |
| `isStalled` / `stalledTime`                           | 存在                   | 存在（不删除）                                           | —   | **退回旧：保留旧固件** `isStalled` **字段；不实现 stall v3 时继续使用旧逻辑**                  |
| `overloadTime` / `overloadFlag`                       | 存在                   | 存在（不删除）                                           | —   | **退回旧：保留旧固件 overload 逻辑**                                               |
| `StallMode_t` **枚举**                                  | ❌                    | ❌（不新增）                                            | —   | **退回旧：**`STALL_IDLE/RETREATING/LOCKED` **是 stall v3 专属；不实现 stall 时不添加** |
| `stallMode` 字段                                        | ❌                    | ❌（不新增）                                            | —   | **退回旧：不实现 stall v3 时不添加此字段**                                            |
| `stallStartTick`                                      | ❌                    | ❌（不新增）                                            | —   | **退回旧：不实现 stall v3 时不添加此字段**                                            |
| `retreatStartTick / RetreatTarget / RetreatDirection` | ❌                    | ❌（不新增）                                            | —   | **退回旧：不实现 stall v3 时不添加此字段**                                            |
| `stallCurrentThreshold`                               | ❌                    | ❌（不新增）                                            | —   | **退回旧：**`stallCurrentThreshold` **仅服务于 stall v3；不实现 stall 时不添加**        |
| `positionModeStartCycles`                             | ❌                    | ❌（不新增）                                            | —   | **退回旧：**`positionModeStartCycles` **仅服务于 stall v3 启动豁免；不实现 stall 时不添加** |
| `stallBroadcastCmd`                                   | ❌                    | ❌（不新增）                                            | —   | **退回旧：**`stallBroadcastCmd` **仅服务于 stall 广播；不实现 stall 时不添加**            |
| `stallDetectRisingEdge / RisingEstError`              | ❌                    | ❌（不新增）                                            | —   | **退回旧：二次验证字段仅服务于 stall v3 检测；不实现 stall 时不添加**                           |


### 2.3 堵转检测 v3 协议（已删除 — 不实现）

> **本节全部删除。** 不实现 stall v3 检测协议时，此部分代码不移植。


| 阶段                    | 规则     | new（手动标记）                                        | 状态  |
| --------------------- | ------ | ------------------------------------------------ | --- |
| 启动豁免期                 | —（删除）  | **退回旧：整个 stall v3 协议不实现，此阶段逻辑删除**                | ❌   |
| 三条件首次同满足              | —（删除）  | **退回旧：同上**                                       | ❌   |
| 上升沿计时                 | —（删除）  | **退回旧：同上**                                       | ❌   |
| 30 ms 二次验证            | —（删除）  | **退回旧：同上**                                       | ❌   |
| 200 ms 兜底             | —（删除）  | **退回旧：同上**                                       | ❌   |
| 400 ms 兜底强制触发         | —（删除）  | **退回旧：同上**                                       | ❌   |
| IDLE→RETREATING       | —（删除）  | **退回旧：同上**                                       | ❌   |
| RETREATING 完成判定       | —（删除）  | **退回旧：同上**                                       | ❌   |
| 2 s 超时兜底              | —（删除）  | **退回旧：同上**                                       | ❌   |
| LOCKED 维持             | —（删除）  | **退回旧：同上**                                       | ❌   |
| 250 ms 周期广播           | —（删除）  | **退回旧：同上**                                       | ❌   |
| 0x5A 他机收              | —（删除）  | **退回旧：同上**                                       | ❌   |
| 0x5B 全局解锁             | —（删除）  | **退回旧：同上**                                       | ❌   |
| 0x1B Stall Protect 开关 | —（退回旧） | **退回旧：不实现 stall v3 时，堵转保护开关命令退回旧固件行为（写 EEPROM）** | 退回旧 |


---

## 三、`BoardConfig_t` 字段（EEPROM 持久化）

> 定义在 `UserApp/configurations.h`；运行时通过 `boardConfig` 全局访问；通过 `0x11`-`0x16`、`0x17`-`0x1A` 写入，`0x24`、`0x28`-`0x2D` 读出。


| 字段                   | 类型     | 含义（功能/作用）                                                             | new（手动标记）      |
| -------------------- | ------ | --------------------------------------------------------------------- | -------------- |
| `configStatus`       | enum   | EEPROM 配置状态（决定是否执行首次烧录默认值写入）                                          | 同旧：纯数据定义，无行为差异 |
| `canNodeId`          | uint32 | CAN 通信节点地址（用于主控识别和寻址）                                                 | 同旧：纯数据定义，无行为差异 |
| `encoderHomeOffset`  | int32  | 归零偏移（步）；`GetPosition() = (realPosition - offset) / SUBDIVIDE`（决定零位位置） | 同旧：字段定义无变化     |
| `defaultMode`        | uint32 | 上电默认控制模式（`MODE_COMMAND_POSITION`=1，控制环类型）                             | 同旧：纯数据定义，无行为差异 |
| `currentLimit`       | int32  | FOC 电流硬限幅（mA，对应 `0x12` 设置，同时作为旧固件 stall 检测触发条件）                       | 同旧：纯数据定义，无行为差异 |
| `velocityLimit`      | int32  | 速度上限（step/s = `r/s × SUBDIVIDE`，用于速度环和轨迹规划限幅）                         | 同旧：纯数据定义，无行为差异 |
| `velocityAcc`        | int32  | 梯形加速度上限（step/s²，轨迹规划中限制加速度）                                           | 同旧：纯数据定义，无行为差异 |
| `calibrationCurrent` | int32  | 编码器校准时的 FOC 电流（mA，校准过程中驱动电机的电流阈值）                                     | 同旧：纯数据定义，无行为差异 |
| `dce_kp`             | int32  | DCE 位置环比例增益（决定位置响应速度）                                                 | 同旧：纯数据定义，无行为差异 |
| `dce_kv`             | int32  | DCE 速度比例增益（与 ki/kd 共同构成速度前馈）                                          | 同旧：纯数据定义，无行为差异 |
| `dce_ki`             | int32  | DCE 积分增益（消除稳态位置误差）                                                    | 同旧：纯数据定义，无行为差异 |
| `dce_kd`             | int32  | DCE 微分增益（抑制位置超调）                                                      | 同旧：纯数据定义，无行为差异 |
| `motor_temperature`  | float  | 内置温度传感器读数（℃，需 `0x7D` 命令触发采集）                                          | 同旧：纯数据定义，无行为差异 |
| `enableMotorOnBoot`  | bool   | 上电自动使能（true=复位后直接进入 enable 状态）                                        | 同旧：纯数据定义，无行为差异 |
| `enableStallProtect` | bool   | 上电是否开启旧版堵转保护（35/42/57；旧固件 stall 检测总开关）                                | 同旧：字段定义无变化     |
| `enableTempWatch`    | bool   | 是否启用 1Hz 温度采集（`0x7D` 触发，LED 状态同步温度）                                   | 同旧：纯数据定义，无行为差异 |


---

## 四、`Motor.h` 默认参数（`Ctrl/Motor/motor.h`，类构造函数初始化）

> 仅在 EEPROM 首次烧录（`configStatus != CONFIG_OK`）时使用一次；运行时 `main.cpp` 会再次覆盖 `ratedVelocity`/`ratedVelocityAcc` 等字段。


| 字段                                                                    | 类型      | 旧固件默认                 | 新固件默认                            | new（手动标记）                               |
| --------------------------------------------------------------------- | ------- | --------------------- | -------------------------------- | --------------------------------------- |
| `motionParams.encoderHomeOffset`（EEPROM 首次烧录偏移，首次使用前由主控重新设置）          | int32   | 0                     | 0                                | 同旧                                      |
| `motionParams.caliCurrent`（编码器校准驱动电流）                                 | int32   | 2000 mA               | 2000 mA                          | 同旧                                      |
| `motionParams.ratedCurrent`（电机额定电流，运行时由 currentLimit 覆盖）              | int32   | 1000 mA               | 1000 mA                          | 同旧                                      |
| `motionParams.ratedCurrentAcc`（电流变化率上限）                               | int32   | 2 × 1000 mA/s         | 2 × 1000 mA/s                    | 同旧                                      |
| `motionParams.ratedVelocity`（电机额定转速，运行时由 ratedVelocity 覆盖）            | int32   | 30 × SUBDIVIDE        | 30 × SUBDIVIDE                   | 同旧                                      |
| `motionParams.ratedVelocityAcc`（梯形加速度上限，0=由 main.cpp 接管）              | int32   | **1000 × SUBDIVIDE**  | **0**                            | 同旧：构造初始值实际被 main.cpp 覆盖，无实际影响           |
| `ctrlParams.stallProtectSwitch`（是否启用旧版堵转检测的开关）                        | bool    | **false**             | **false**（退回旧；不实现 stall v3 时无意义） | **退回旧：不实现 stall v3 时此参数无意义；退回旧值 false** |
| `ctrlParams.pid.{kp,ki,kd}`（PID 速度环增益，仅在使用 PID 速度环时生效）                | int32×3 | `{5, 30, 0}`          | `{5, 30, 0}`                     | 同旧                                      |
| `ctrlParams.dce.{kp,kv,ki,kd}`（DCE 位置环增益，firmware 使用 DCE 而非 PID 做位置环） | int32×4 | `{200, 80, 300, 250}` | `{200, 80, 300, 250}`            | 同旧                                      |


---

## 五、`main.cpp` 默认值对比（`UserApp/main.cpp`）

> `main()` 函数中首次烧录的 `boardConfig` 实始化块（`EEPROM.get` 检测 `configStatus != CONFIG_OK` 时执行）。


| 字段                        | 旧 35     | 旧 42     | 新 35     | 新 42     | 新 57     | 新 gripper | new（手动标记）                                                                  |
| ------------------------- | -------- | -------- | -------- | -------- | -------- | --------- | -------------------------------------------------------------------------- |
| `currentLimit` (mA)       | **1000** | **1000** | **1000** | **1000** | **1000** | 1200      | **退回旧：不实现 stall v3 时，电流保持旧固件默认值；新固件 2A/2.5A 是为 stall 检测服务的，不采用**           |
| `velocityLimit` (r/s)     | 30       | 30       | 30       | 30       | 30       | 30        | 同旧：速度限幅无变化                                                                 |
| `velocityAcc` (r/s²)      | **100**  | **100**  | **100**  | **100**  | **100**  | **100**   | **退回旧：不实现 stall v3 时，加速度保持旧固件默认值 100 r/s²；新固件 30 r/s² 是为 stall 检测服务的，不采用** |
| `calibrationCurrent` (mA) | 2000     | 2000     | 2000     | 2000     | 2000     | 1200      | 同旧：35/42/57 一致；gripper=1200 符合低功耗设计                                        |
| `dce_kp`                  | 195      | 200      | 195      | 200      | 300      | 195       | 同旧：35/42/gripper 一致；57=300 适配更大电机                                          |
| `dce_kv`                  | 80       | 80       | 80       | 80       | 120      | 80        | 同旧：35/42/gripper 一致；57=120 适配更大电机                                          |
| `dce_ki`                  | 300      | 300      | 300      | 300      | 150      | 300       | 同旧：35/42/gripper 一致；57=150 适配更大电机                                          |
| `dce_kd`                  | 250      | 250      | 250      | 250      | 50       | 250       | 同旧：35/42/gripper 一致；57=50 适配更大电机                                           |
| `enableMotorOnBoot`       | false    | false    | false    | false    | false    | false     | 同旧：上电使能无变化                                                                 |
| `enableStallProtect`      | false    | false    | false    | false    | false    | ❌         | **退回旧：不实现 stall v3 时，堵转保护默认关闭；与旧固件一致**                                     |
| `enableTempWatch`         | false    | false    | false    | false    | false    | false     | 同旧：温度监控无变化                                                                 |


`canNodeId` **决定方式（运行时行为）**：


| 固件        | 来源                                                      | 取值    | new（手动标记）             |
| --------- | ------------------------------------------------------- | ----- | --------------------- |
| 旧 35/42   | 拨码开关 PA8/PA9/PA10                                       | 1~6   | 同旧：拨码方式无变化            |
| 新 35/42   | 拨码开关 PA8/PA9/PA10                                       | 1~6   | 同旧：35/42 保持拨码方式       |
| 新 57      | 硬编码宏 `RAIL_FIXED_NODE_ID = 7`（`configurations.h:13`）    | **7** | 同旧：57 保持硬编码 ID=7      |
| 新 gripper | 硬编码宏 `GRIPPER_FIXED_NODE_ID = 8`（`configurations.h:13`） | **8** | 同旧：gripper 保持硬编码 ID=8 |


`OnButton2Event` **行为差异**：


| 旧固件                                    | 新固件 35/42/57                                                    | 新 gripper                  | new（手动标记）                                                                         |
| -------------------------------------- | --------------------------------------------------------------- | -------------------------- | --------------------------------------------------------------------------------- |
| CLICK → `controller->ClearStallFlag()` | CLICK → `controller->ClearStallFlag()`（退回旧；不实现 stall v3 时保留旧逻辑） | CLICK → `ClearStallFlag()` | **退回旧：不实现 stall v3 时，**`ClearStallFlag()` **仍有清除旧固件** `isStalled` **的功能；退回旧固件行为** |


`Tim1Callback100Hz` **差异**（35/42/57 vs gripper）：

- 35/42/57：无 stall 250ms 周期广播 + stallBroadcastCmd（退回旧；不实现 stall v3 时删除）
- gripper：纯温度采集循环，无变化

---

## 六、`ResetGoalsToCurrentPosition()`（已删除 — 不实现）

> **本节已删除。** `ResetGoalsToCurrentPosition()` 仅服务于 stall v3 检测的 LOCKED→IDLE 退出；不实现 stall 时，此方法不移植。
>
> 旧固件无此方法，无需迁移。

---

---

## 七、命令协议兼容性矩阵


| 类别                           | 旧固件支持       | 新 35/42/57  | 新 gripper           | new（手动标记）                                             |
| ---------------------------- | ----------- | ----------- | ------------------- | ----------------------------------------------------- |
| 基本控制 `0x01`-`0x07`           | ✅           | ✅           | ✅                   | 同旧：基本控制命令退回旧固件行为（删除 stall 专属逻辑）                       |
| 参数写入 `0x11`-`0x1A`           | ✅           | ✅           | ✅（gripper 缺 `0x1B`） | 同旧：参数写入退回旧固件（删除 stallCurrentThreshold 联动）             |
| Stall 开关 `0x1B`              | ✅（写 EEPROM） | ✅（写 EEPROM） | ❌                   | **退回旧：不实现 stall v3 时，堵转保护开关无意义；退回旧固件行为（写 EEPROM）或删除** |
| 基本查询 `0x21`-`0x25`           | ✅           | ✅           | ✅                   | 同旧：基本查询无变化                                            |
| DCE/Acc/Cur 查询 `0x28`-`0x2D` | ❌           | ❌（删除）       | ❌（删除）               | **退回旧：不实现 stall 时查询能力非必需；建议删除以保持最小改动**                |
| 急停广播 `0x09`                  | ❌           | ✅           | ✅                   | **保留新：安全急停功能，不依赖 stall 检测，建议保留**                      |
| 堵转协议 `0x5A/0x5B/0x5C`        | ❌           | ❌（删除）       | ❌                   | **退回旧：整个 stall v3 协议全部删除**                            |
| 系统命令 `0x7D`-`0x7F`           | ✅           | ✅           | ✅                   | 同旧：系统命令无变化                                            |


---

## 八、新增/重构/删除 汇总

> **迁移策略**：不实现 stall v3 检测。以上所有改动均以"退回旧固件"为优先。

### 8.1 保留的新增（9 项）

1. CAN `0x09` Broadcast EStop（35/42/57/gripper）—— 安全功能，不依赖 stall
2. CAN `0x28` Get DCE Kp —— GET 查询命令，仅读参数
3. CAN `0x29` Get DCE Kv —— GET 查询命令，仅读参数
4. CAN `0x2A` Get DCE Ki —— GET 查询命令，仅读参数
5. CAN `0x2B` Get DCE Kd —— GET 查询命令，仅读参数
6. CAN `0x2C` Get Acceleration —— GET 查询命令，仅读参数
7. CAN `0x2D` Get Current-Limit —— GET 查询命令，仅读参数
8. `SetPositionSetPointWithTime()` 安全增强 —— `sqrtf` 加 `fmaxf(0, discriminant)` 兜底防负数 crash
9. 旧固件 `ApplyPosAsHomeOffset()` Bug 修复 —— 同步 `realPosition = encoderHomeOffset`，让 `GetPosition()` 立即归零

### 8.2 删除的新增（7 项）

1. CAN `0x5A` STALL Broadcast（35/42/57）→ **删除**
2. CAN `0x5B` UNLOCKED Broadcast（35/42/57）→ **删除**
3. CAN `0x5C` Stall Query（35/42/57）→ **删除**
4. 方法 `ResetGoalsToCurrentPosition()` → **删除**
5. 方法 `GetEstPosition()` / `GetEstVelocity()` → **删除**
6. `StallMode_t` 枚举 → **不添加**
7. stall v3 状态字段 → **不添加**（`stallMode`、`stallStartTick`、`retreatStartTick`/`RetreatTarget`/`RetreatDirection`、`stallCurrentThreshold`、`positionModeStartCycles`、`stallBroadcastCmd`、`stallDetectRisingEdge`/`RisingEstError`）

### 8.3 退回旧的重构（8 项）

1. `0x01 Enable` → **退回旧**：删除 LOCKED→IDLE + ResetGoals 联动
2. `0x05 Set Position` → **退回旧**：删除 LOCKED/RETREATING 丢弃逻辑
3. `0x07 Set Pos w/ VelLimit` → **退回旧**：删除 LOCKED 丢弃 + 恢复立即 FINISH ACK
4. `0x12 Set I-Limit` → **退回旧**：删除 `stallCurrentThreshold` 同步
5. `0x1B Stall-Protect` → **退回旧**：恢复写 EEPROM 行为（或删除命令）
6. `CloseLoopControlTick()` → **退回旧**：删除 stall v3 检测块、LOCKED 维持、STATE_STALL 输出
7. `Init()` → **退回旧**：删除 stall 字段清零
8. `ClearIntegral()` → **退回旧**：恢复清 PID/DCE 积分的旧逻辑

### 8.4 保留旧固件的元素（不改动）

1. `Controller::isStalled` 字段
2. `Controller::stalledTime` 字段
3. `Controller::overloadTime / overloadFlag` 字段
4. `State_t::STATE_OVERLOAD` 枚举
5. `Controller::ClearStallFlag()` 方法
6. `Mode_t` (9 个模式)
7. 所有基本控制/查询/参数命令（`0x01`-`0x07`、`0x11`-`0x16`、`0x17`-`0x1A`、`0x21`-`0x25`、`0x7D`-`0x7F`）
8. 旧固件默认值（`currentLimit=1000mA`、`velocityAcc=100r/s²`、`enableStallProtect=false`）
