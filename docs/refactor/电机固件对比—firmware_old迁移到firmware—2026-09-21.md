# 电机固件对比 — `firmware_old` → `firmware` 迁移参考

> **范围**：仅电机固件（STM32F103，4 套：`motor_fw_f103_35` / `_42` / `_57` / `_gripper`）。
> **基线对比**：`firmware_old/dummy-35motor-fw` 与 `firmware_old/dummy-42motor-fw`（旧）；`firmware/motor_fw_f103_*`（新）。
> **数据来源**：直接读源码，每条标注 `文件:行号`。**文档聚焦"差异"**，未变化的部分不列。
> **创建日期**：2026-09-21

---

## 一、CAN 协议对比（`UserApp/protocols/interface_can.cpp`）

> 所有 cmd 由 `OnCanCmd(_cmd, _data, _len)` 解析。CAN ID 格式：`(nodeID<<7) | cmdCode`，8 字节数据。
> 旧固件共 26 条 cmd（35/42 各一份，内容一致）。新固件 35/42/57 共 **32 条**；夹爪 29 条（不含堵转协议）。

### 1.1 命令清单

| Cmd | 方向 | 名称 | 旧固件 | 新 35/42/57 | 新 gripper | 标记 | new |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `0x01` | 主→电 | Enable Motor | 切 `requestMode` 到 VELOCITY/STOP | **重构**：联动 `stallMode` 状态机；LOCKED→IDLE + ResetGoals + 启动豁免复位 | 同旧 | **重构** |  |
| `0x02` | 主→电 | Do Calibration | 触发编码器校准 | 同 | 同 | — |  |
| `0x03` | 主→电 | Set Current | 切 `MODE_COMMAND_CURRENT` + `SetCurrentSetPoint(A×1000)` | 同 | 同 | — |  |
| `0x04` | 主→电 | Set Velocity | 切 `MODE_COMMAND_VELOCITY` + `SetVelocitySetPoint(r/s×SUBDIVIDE)` | 同 | 同 | — |  |
| `0x05` | 主→电 | Set Position | 切 POSITION 模式 + `SetPositionSetPoint(圈×SUBDIVIDE)`；可选 FINISH ACK | **重构**：LOCKED/RETREATING 时丢弃（防覆盖 retreatTarget）；其余同 | 同新 35/42/57 | **重构** |  |
| `0x06` | 主→电 | Set Pos w/ Time | `SetPositionSetPointWithTime(pos, T)`；可选 FINISH ACK | **微调**：`sqrtf` 加 `fmaxf(0, discriminant)` 兜底防负数；其余同 | 同新 | **微调** |  |
| `0x07` | 主→电 | Set Pos w/ VelLimit | 写 `ratedVelocity` + `SetPositionSetPoint`；**总是**立即发 FINISH ACK | **重构**：LOCKED/RETREATING 丢弃；**取消**立即 FINISH ACK（5kHz 控制循环尚未跑到，stale state 会让主控 SEQ 提前退出） | 同新 | **重构** |  |
| **`0x09`** | 主→ALL | Broadcast EStop | ❌ | **新增**：`requestMode=STOP` + `SetBrake(true)` + Cur/Vel=0；不请 stallMode | **新增** | **新增** |  |
| `0x11` | 主→电 | Set NodeID + EPP | 写 `canNodeId` + 可选持久化 | 同 | 同 | — |  |
| `0x12` | 主→电 | Set I-Limit + EPP | 写 `ratedCurrent = A×1000` + `currentLimit` + 持久化 | **重构**：同时刷新 `stallCurrentThreshold = ratedCurrent × 40%` | 同新（无 stall 阈值更新） | **重构** |  |
| `0x13` | 主→电 | Set V-Limit + EPP | 写 `ratedVelocity` + `velocityLimit` + 持久化 | 同 | 同 | — |  |
| `0x14` | 主→电 | Set Accel + EPP | 写 `ratedVelocityAcc` + 同步两个 tracker | 同 | 同 | — |  |
| `0x15` | 主→电 | Apply Home + EPP | **Bug**：仅改 `encoderHomeOffset`，`GetPosition()` 不归零 | **重构 v3.2 修复**：同步 `realPosition = encoderHomeOffset`，`GetPosition()` 立即归零 | 同新 | **重构** |  |
| `0x16` | 主→电 | Set Auto-Enable + EPP | 写 `enableMotorOnBoot` | 同 | 同 | — |  |
| `0x17`-`0x1A` | 主→电 | Set DCE Kp/Kv/Ki/Kd | 写 `dce.{kp,kv,ki,kd}` + 持久化 | 同 | 同 | — |  |
| `0x1B` | 主→电 | Set Stall-Protect | 写 `stallProtectSwitch` + `enableStallProtect` + 持久化 | **重构**：**仅 RAM 切换**（不写 EEPROM，重启后恢复默认） | ❌ 无此命令 | **重构（35/42/57）** |  |
| `0x21` | 主→电 | Get Current | 回 float A | 同 | 同 | — |  |
| `0x22` | 主→电 | Get Velocity | 回 float r/s | 同 | 同 | — |  |
| `0x23` | 主→电 | Get Position | 回 float 圈 + finish 标志 | 同 | 同 | — |  |
| `0x24` | 主→电 | Get Offset | 回 int32 `encoderHomeOffset` | 同 | 同 | — |  |
| `0x25` | 主→电 | Get Temperature | 回 float `motor_temperature` | 同 | 同 | — |  |
| **`0x28`** | 主→电 | Get DCE Kp | ❌ | **新增**：回 int32 `boardConfig.dce_kp` | **新增** | **新增** |  |
| **`0x29`** | 主→电 | Get DCE Kv | ❌ | **新增**：回 int32 `boardConfig.dce_kv` | **新增** | **新增** |  |
| **`0x2A`** | 主→电 | Get DCE Ki | ❌ | **新增**：回 int32 `boardConfig.dce_ki` | **新增** | **新增** |  |
| **`0x2B`** | 主→电 | Get DCE Kd | ❌ | **新增**：回 int32 `boardConfig.dce_kd` | **新增** | **新增** |  |
| **`0x2C`** | 主→电 | Get Acceleration | ❌ | **新增**：回 float `velocityAcc / SUBDIVIDE`（r/s²） | **新增** | **新增** |  |
| **`0x2D`** | 主→电 | Get Current-Limit | ❌ | **新增**：回 float `currentLimit / 1000`（A） | **新增** | **新增** |  |
| **`0x5A`** | 触发电→他机 | STALL Broadcast | ❌ | **新增**：data[0]=srcId, data[1]=cmd（1=TRIGGER, 2=DONE, 3=TIMEOUT）；其他电机收 → LOCKED + ClearIntegral | ❌ | **新增（35/42/57）** |  |
| **`0x5B`** | 主→ALL | UNLOCKED Broadcast | ❌ | **新增**：所有 LOCKED 电机 → IDLE + ResetGoals + 启动豁免复位 | ❌ | **新增（35/42/57）** |  |
| **`0x5C`** | 主→单电 | Stall Query | ❌ | **新增**：data[0]=1(en) 或 2(locked)；回 data[1]=value | ❌ | **新增（35/42/57）** |  |
| `0x7D` | 主→电 | Enable Temp Watch | TIM1 启动 1Hz 温度采集 | 同 | 同 | — |  |
| `0x7E` | 主→电 | Erase Configs | `configStatus = CONFIG_RESTORE` → SystemReset | 同 | 同 | — |  |
| `0x7F` | 主→电 | Reboot | `HAL_NVIC_SystemReset()` | 同 | 同 | — |  |

### 1.2 协议帧格式约定

| Cmd | TxData[0..3] | TxData[4..7] | new |
| --- | --- | --- | --- |
| `0x03/0x04/0x05/0x07` 前半 | float pos / cur / vel | (见下) |  |
| `0x06` | float pos（圈） | float time（秒） |  |
| `0x07` | float pos（圈） | float vel-limit（r/s） |  |
| `0x11`-`0x16` | 主参数字段 | data[4]=commit flag（0/1） |  |
| `0x17`-`0x1A` | int32 PID 分量 | data[4]=commit flag |  |
| `0x12`-`0x14` ACK 时 | 回 float (A / r/s / r/s²) | data[4]=0 |  |
| `0x21`-`0x25` ACK 时 | 回 float 当前值 | data[4]=STATE_FINISH 标志 |  |
| `0x28`-`0x2B` ACK 时 | int32 PID 分量 | — |  |
| `0x5A` | data[0]=srcNodeId, data[1]=cmd | — |  |
| `0x5C` | data[0]=queryType (1/2) | — |  |

---

## 二、电机控制层（`Ctrl/Motor/motor.cpp` + `motor.h`）

### 2.1 Controller 类方法对比

| 方法签名 | 旧固件 | 新固件 | 标记 |
| --- | --- | --- | --- |
| `Tick20kHz()` | 编码器采样 + 控制环 | 同 | — |
| `CloseLoopControlTick()` | 19kHz 控制环 | **重构**：增加 `STALL_LOCKED` 维持分支、stall 检测块、RETREATING 完成判定、stallMode 状态写入 `STATE_STALL` | **重构** |
| `CalcCurrentToOutput(int32_t)` | FOC 电流环 | 同 | — |
| `CalcPidToOutput(int32_t)` | PID 速度环 | 同 | — |
| `CalcDceToOutput(int32_t,int32_t)` | DCE 位置环 | 同 | — |
| `SetCtrlMode(Mode_t)` | 写 `requestMode` | 同 | — |
| `SetPositionSetPoint(int32_t)` | `goalPosition = _pos + encoderHomeOffset` | **同**（语义不变；v3.2 已不需在调用端额外处理） | — |
| `SetPositionSetPointWithTime(int32_t,float)` → `bool` | 算 pMax = acc·T²/4；超限返回 false + 用 `velocityLimit` | **微调**：判别式用 `fmaxf(0, discriminant)` 兜底（防 sqrt 负数）；其余同 | **微调** |
| `SetVelocitySetPoint(int32_t)` | 限幅到 `±ratedVelocity` | 同 | — |
| `SetCurrentSetPoint(int32_t)` | 限幅到 `±ratedCurrent` | 同 | — |
| `SetDisable(bool)` | 写 `goalDisable` | 同 | — |
| `SetBrake(bool)` | 写 `goalBrake` | 同 | — |
| `AddTrajectorySetPoint` | 同时设位置 + 速度 | 同 | — |
| `GetPosition(bool=false)` | `(realPosition - offset) / SUBDIVIDE` | 同 | — |
| `GetVelocity()` | `estVelocity / SUBDIVIDE` | 同 | — |
| `GetFocCurrent()` | `focCurrent / 1000` | 同 | — |
| `CompensateAdvancedAngle(int32_t)` | 5 段查表 | 同 | — |
| `Init()` | 状态变量全部清零 | **重构**：增加 stall 状态字段清零（stallStartTick / retreatStartTick / positionModeStartCycles / stallBroadcastCmd 等） | **重构** |
| `AttachConfig(Config_t*)` | 绑 config | 同 | — |
| `ClearIntegral() const` | 清 PID/DCE `integralRound/outputKi` | **`void ClearIntegral()` 非 const**；仅清 `estVelocityIntegral` + `estError`（PID/DCE 积分不再显式清零） | **重构** |
| `ClearStallFlag()` | 重置 `stalledTime + isStalled` | ❌ 移除（替换为内联 `stallStartTick=0`） | **删除** |
| `ApplyPosAsHomeOffset()` | `encoderHomeOffset = realPosition % SUBDIVIDE` | **v3.2 重构**：除改 offset 外，**同步** `realPosition = encoderHomeOffset`，让 `GetPosition()` 立即归零 | **重构** |
| **`ResetGoalsToCurrentPosition()`** | ❌ | **新增**：`goalPosition=estPosition`（**绕过 +offset**），清零 goalVelocity/Current，触发 softNewCurve | **新增** |
| **`GetEstPosition()`** | ❌ | **新增**：inline getter | **新增** |
| **`GetEstVelocity()`** | ❌ | **新增**：inline getter | **新增** |

### 2.2 状态机变化

| 项 | 旧固件 | 新固件 | 标记 |
| --- | --- | --- | --- |
| `Mode_t` (9 个模式) | 同 | 同 | — |
| `State_t::STATE_OVERLOAD` | 有 | **删除** | **删除** |
| `State_t::STATE_STALL` | `isStalled=true` 时进入 | **`stallMode ∈ {RETREATING, LOCKED}` 时进入**（不再用 `isStalled`） | **重构** |
| `isStalled` / `stalledTime` / `overloadTime` / `overloadFlag` | 存在 | **全部删除**（替换为 stall 状态字段） | **删除** |
| **`StallMode_t` 枚举** | ❌ | **新增**：`STALL_IDLE=0` / `STALL_RETREATING=1` / `STALL_LOCKED=2` | **新增** |
| **`stallMode`** 字段 | ❌ | **新增** | **新增** |
| **`stallStartTick`** | ❌ | **新增**（堵转持续时间 us） | **新增** |
| **`retreatStartTick / RetreatTarget / RetreatDirection`** | ❌ | **新增**（回退快照） | **新增** |
| **`stallCurrentThreshold`** | ❌ | **新增**（`= ratedCurrent × 40%`，由 `0x12` 自动维护） | **新增** |
| **`positionModeStartCycles`** | ❌ | **新增**（启动豁免 2000 cycle @ 20kHz = 100ms） | **新增** |
| **`stallBroadcastCmd`** | ❌ | **新增**（20kHz 设置，100Hz 发送） | **新增** |
| **`stallDetectRisingEdge / RisingEstError`** | ❌ | **新增**（二次验证用） | **新增** |

### 2.3 堵转检测 v3 协议（仅 35/42/57，gripper 不实现）

> 实现位置：`Ctrl/Motor/motor.cpp:171-300`（IDLE→RETREATING 判定 + RETREATING→LOCKED 完成判定）。
> 广播发送：`UserApp/main.cpp` 的 `Tim1Callback100Hz()`。

| 阶段 | 规则 |  |  |
| --- | --- | --- | --- |
| 启动豁免期 | 进入 POSITION 模式后 100 ms 内不检测（`positionModeStartCycles < 2000`） |  |  |
| 三条件首次同满足 | ` | focCurrent | > ratedCurrent × 40%` **且** ` |
| 上升沿计时 | 三条件首次同时满足 → `stallStartTick=50us`；继续满足则累加 |  |  |
| 30 ms 二次验证 | 电流仍 > 60% 阈值 **且** ` | estError | ` ≥ 12 步 **或** 误差 ≥ 上升沿时刻误差 50% |
| 200 ms 兜底 | 速度起来（` | estVelocity | ` > 150）→ 判定不是堵转，清上升沿 |
| 400 ms 兜底强制触发 | 即使 B/C 失败也强制 `STALL_RETREATING`（保险机制） |  |  |
| IDLE→RETREATING | 计算 `retreatDirection = sign(goalPosition - estPosition)`；`retreatTarget = estPosition + direction × 35556 步`（≈1/4 圈）；`ratedVelocity = 50000 step/s`，`acc = 100000 step/s²`；调 `SetPositionSetPoint(retreatTarget)`；ClearIntegral；**立即广播** `0x5A` cmd=1 | 閲嶆柊瀹炵幇鏂规 |
| RETREATING 完成判定 | `softPosition == goalPosition && softVelocity == 0` → `STALL_LOCKED + ClearIntegral + 广播 cmd=2 (DONE)` | 閲嶆柊瀹炵幇鏂规 |
| 2 s 超时兜底 | `HAL_GetTick() - retreatStartTick > 2000` → `STALL_LOCKED + 广播 cmd=3 (TIMEOUT)` |  |
| LOCKED 维持 | **主动** `CalcDceToOutput(estPosition, 0)` 维持当前位置；不响应新位置命令 | 閲嶆柊瀹炵幇鏂规 |
| 250 ms 周期广播 | RETREATING 期间每 250 ms 重复发 `0x5A` cmd=1（仅 57 注释说明主控 Bug-11 修复依赖） |  |
| 0x5A 他机收 | cmd=1 且本机 IDLE → `STALL_LOCKED + ClearIntegral` |  |
| 0x5B 全局解锁 | LOCKED→IDLE + ResetGoals + 启动豁免复位 |  |
| 0x1B Stall Protect 开关 | 35/42/57：临时 RAM 切换；gripper：无此命令 |  |

---

## 三、`BoardConfig_t` 字段（EEPROM 持久化）

> 定义在 `UserApp/configurations.h`；运行时通过 `boardConfig` 全局访问；通过 `0x11`-`0x16`、`0x17`-`0x1A` 写入，`0x24`、`0x28`-`0x2D` 读出。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `configStatus` | enum | EEPROM 状态机：`CONFIG_RESTORE/COMMIT/OK` |
| `canNodeId` | uint32 | CAN 节点 ID |
| `encoderHomeOffset` | int32 | 归零偏移（步）；`GetPosition() = (realPosition - offset) / SUBDIVIDE` |
| `defaultMode` | uint32 | 默认模式（`MODE_COMMAND_POSITION`=1） |
| `currentLimit` | int32 | FOC 电流硬限幅（mA） |
| `velocityLimit` | int32 | 速度上限（step/s） = `r/s × SUBDIVIDE` |
| `velocityAcc` | int32 | 梯形加速度上限（step/s²） |
| `calibrationCurrent` | int32 | 编码器校准时的 FOC 电流（mA） |
| `dce_kp` | int32 | DCE 位置比例增益 |
| `dce_kv` | int32 | DCE 速度比例增益 |
| `dce_ki` | int32 | DCE 积分增益 |
| `dce_kd` | int32 | DCE 微分增益 |
| `motor_temperature` | float | 内置温度传感器读数（℃） |
| `enableMotorOnBoot` | bool | 上电自动使能 |
| `enableStallProtect` | bool | 上电是否开启堵转保护（35/42/57） |
| `enableTempWatch` | bool | 是否启用 1Hz 温度采集（`0x7D` 触发） |

> **gripper 特殊**：`BoardConfig_t` **不包含** `enableStallProtect` 字段（gripper 完全无堵转检测）。

---

## 四、`Motor.h` 默认参数（`Ctrl/Motor/motor.h`，类构造函数初始化）

> 仅在 EEPROM 首次烧录（`configStatus != CONFIG_OK`）时使用一次。

| 字段 | 类型 | 旧固件默认 |
| --- | --- | --- |
| `motionParams.encoderHomeOffset` | int32 | 0 |
| `motionParams.caliCurrent` | int32 | 2000 mA |
| `motionParams.ratedCurrent` | int32 | 1000 mA |
| `motionParams.ratedCurrentAcc` | int32 | 2 × 1000 mA/s |
| `motionParams.ratedVelocity` | int32 | 30 × SUBDIVIDE |
| `motionParams.ratedVelocityAcc` | int32 | **1000 × SUBDIVIDE** |
| `ctrlParams.stallProtectSwitch` | bool | **false** |
| `ctrlParams.pid.{kp,ki,kd}` | int32×3 | `{5, 30, 0}` |
| `ctrlParams.dce.{kp,kv,ki,kd}` | int32×4 | `{200, 80, 300, 250}` |

---

## 五、`main.cpp` 默认值对比（`UserApp/main.cpp`）

> `main()` 函数中首次烧录的 `boardConfig` 实始化块（`EEPROM.get` 检测 `configStatus != CONFIG_OK` 时执行）。

| 字段 | 旧 35 | 旧 42 |
| --- | --- | --- |
| `currentLimit` (mA) | **1000** | **1000** |
| `velocityLimit` (r/s) | 30 | 30 |
| `velocityAcc` (r/s²) | **100** | **100** |
| `calibrationCurrent` (mA) | 2000 | 2000 |
| `dce_kp` | 195 | 200 |
| `dce_kv` | 80 | 80 |
| `dce_ki` | 300 | 300 |
| `dce_kd` | 250 | 250 |
| `enableMotorOnBoot` | false | false |
| `enableStallProtect` | false | false |
| `enableTempWatch` | false | false |

**`canNodeId` 决定方式（运行时行为）**：

| 固件 | 来源 | 取值 |
| --- | --- | --- |
| 旧 35/42 | 拨码开关 PA8/PA9/PA10 | 1~6 |
| 新 35/42 | 拨码开关 PA8/PA9/PA10 | 1~6 |
| 新 57 | 硬编码宏 `RAIL_FIXED_NODE_ID = 7`（`configurations.h:13`） | **7** |
| 新 gripper | 硬编码宏 `GRIPPER_FIXED_NODE_ID = 8`（`configurations.h:13`） | **8** |

**`OnButton2Event` 行为差异**：

| 旧固件 | 新固件 35/42/57 | 新 gripper |
| --- | --- | --- |
| CLICK → `controller->ClearStallFlag()` | CLICK → `controller->stallStartTick = 0`（仅清上升沿计时，不清 stallMode） | CLICK → 仅打印日志 | 閲嶆柊瀹炵幇鏂规 |

**`Tim1Callback100Hz` 差异**（35/42/57 vs gripper）：

- 35/42/57：增加 stall 250ms 周期广播 + stallBroadcastCmd 事件广播
- gripper：纯温度采集循环，无 stall 广播

---

## 六、`ResetGoalsToCurrentPosition()` 修复详情（v3.2）

> 见 `Ctrl/Motor/motor.cpp` 新版实现。

```cpp
// 直接用 estPosition 作为 goalPosition，不 + offset
goalPosition = estPosition;   // ← 关键：绕过 +encoderHomeOffset
goalVelocity = 0;
goalCurrent = 0;
softNewCurve = true;
```

**解决的问题**：
- 调用 `SetPositionSetPoint(estPosition)` 会让 `goalPosition = estPosition + encoderHomeOffset`，导致 `CalcSoftGoal()` 算出非零 `deltaPosition`，电机被驱动多走 `encoderHomeOffset` 步。
- 用于 `0x01 Enable` 退出 LOCKED、`0x5B UNLOCKED` 时，**让电机停在当前位置**而不是被驱动回原 goal。
- 不影响主控正常 `0x05/0x07` 下发位置命令（仍走 `SetPositionSetPoint`）。

---

## 七、命令协议兼容性矩阵

| 类别 | 旧固件支持 | 新 35/42/57 | 新 gripper |
| --- | --- | --- | --- |
| 基本控制 `0x01`-`0x07` | ✅ | ✅ | ✅ |
| 参数写入 `0x11`-`0x1A` | ✅ | ✅ | ✅（gripper 缺 `0x1B`） |
| Stall 开关 `0x1B` | ✅（写 EEPROM） | ✅（仅 RAM） | ❌ | **35/42/57：语义变化**；gripper：不存在 | 閲嶆柊瀹炵幇鏂规 |
| 基本查询 `0x21`-`0x25` | ✅ | ✅ | ✅ | 完全兼容 |  |
| DCE/Acc/Cur 查询 `0x28`-`0x2D` | ❌ | ✅ | ✅ | **新增 6 条** | 閲嶆柊瀹炵幇鏂规 |
| 急停广播 `0x09` | ❌ | ✅ | ✅ | **新增** |  |
| 堵转协议 `0x5A/0x5B/0x5C` | ❌ | ✅ | ❌ | **新增（35/42/57）** |  |
| 系统命令 `0x7D`-`0x7F` | ✅ | ✅ | ✅ | 完全兼容 |  |

---

## 八、新增/重构/删除 汇总

### 8.1 新增（11 项）

1. CAN `0x09` Broadcast EStop（35/42/57/gripper）
2. CAN `0x28` Get DCE Kp
3. CAN `0x29` Get DCE Kv
4. CAN `0x2A` Get DCE Ki
5. CAN `0x2B` Get DCE Kd
6. CAN `0x2C` Get Acceleration
7. CAN `0x2D` Get Current-Limit
8. CAN `0x5A` STALL Broadcast（35/42/57）
9. CAN `0x5B` UNLOCKED Broadcast（35/42/57）
10. CAN `0x5C` Stall Query（35/42/57）
11. 方法 `ResetGoalsToCurrentPosition()` + 字段 `stallMode` 状态机（35/42/57）

### 8.2 重构（10 项）

1. `0x01 Enable` 联动 stall 状态机
2. `0x05 Set Position` LOCKED/RETREATING 丢弃
3. `0x06 Set Pos w/ Time` `fmaxf` 兜底
4. `0x07 Set Pos w/ VelLimit` LOCKED 丢弃 + 取消立即 FINISH ACK
5. `0x12 Set I-Limit` 同步 stallCurrentThreshold
6. `0x15 Apply Home` v3.2 同步 realPosition
7. `0x1B Stall-Protect` 仅 RAM（35/42/57）
8. `ApplyPosAsHomeOffset()` v3.2 同步 realPosition
9. `ClearIntegral()` 重写（清 estimator 状态，非 PID/DCE）
10. `CloseLoopControlTick()` 增加 stall 检测 + LOCKED 维持 + 状态输出

### 8.3 删除（5 项）

1. `Controller::isStalled` 字段
2. `Controller::stalledTime` 字段
3. `Controller::overloadTime / overloadFlag` 字段
4. `State_t::STATE_OVERLOAD` 枚举
5. `Controller::ClearStallFlag()` 方法
