# 主控固件对比 — `firmware_old` → `firmware` 迁移参考

> **范围**：仅主控固件（STM32F405，FreeRTOS）。对比 `firmware_old/dummy-ref-core-fw`（旧）与 `firmware/ref_core_f405`（新）。
> **基线对比**：旧固件为 6 轴架构（motorJ[0..6]，含 DummyHand）；新固件为 **7 轴 + 地轨 + 夹爪** 架构（motorJ[0..7]，含 StepHand）。
> **数据来源**：直接读源码，每条标注 `文件:行号`。**文档聚焦"差异"**，未变化的部分不列。
> **创建日期**：2026-09-21

---

## 一、CAN 协议对比（`UserApp/protocols/can_protocol.cpp`）

> 主控固件不解析电机端 cmd 含义（电机端语义见电机固件对比文档），只关注"主控侧如何构造请求帧"和"如何处理回包"。

### 1.1 电机端 cmd 构造方式

> 所有 cmd 构造方式在 `Robot/actuators/ctrl_step/ctrl_step.cpp` 中实现，StdId 格式 `(nodeID << 7) | cmdCode`。

| Cmd | 方向 | 名称 | 主控侧构造（ctrl_step.cpp） | 回包处理（can_protocol.cpp） | 标记 |
|---|---|---|---|---|---|
| `0x01` Enable | 主→电机 | 使能/失能 | `SetEnable(bool)`，StdId=(nodeID<<7)\|0x01 | — | — |
| `0x03` Set Current | 主→电机 | 力矩透传 | `SetCurrentSetPoint(float A)` | — | — |
| `0x04` Set Velocity | 主→电机 | 速度模式 | `SetVelocitySetPoint(float r/s)` | — | — |
| `0x05` Set Position | 主→电机 | 位置模式（简单） | `SetPositionSetPoint(float 圈)`，data[4]=1 | — | — |
| **`0x07`** Set Position w/ VelLimit | 主→电机 | 位置+速度限幅 | **重构**：新方法名 `SetPositionWithMotorRps(float 圈, float r/s)`，旧名 `SetPositionWithVelocityLimit` | — | **重构** |
| `0x11` Set NodeID | 主→电机 | 设置节点 ID | `SetNodeID(uint32_t)` | — | — |
| `0x12` Set I-Limit | 主→电机 | 设置电流限幅 | **重构**：`SetCurrentLimit_persist(float A, bool persist)`（按参数决定是否写 EEPROM）；旧只有 `SetCurrentLimit(float)` 固定写 | — | **重构** |
| `0x13` Set V-Limit | 主→电机 | 设置速度限幅 | `SetVelocityLimit(float r/s)` | — | — |
| `0x14` Set Acceleration | 主→电机 | 设置加速度 | **重构**：`SetAcceleration_persist(float r/s², bool persist)`（按参数决定是否写 EEPROM）；旧只有 `SetAcceleration(float)` 默认不写 | — | **重构** |
| `0x15` Apply Home | 主→电机 | 归零偏移 | `ApplyPositionAsHome()` | — | — |
| `0x16` Set Enable-On-Boot | 主→电机 | 上电自启 | `SetEnableOnBoot(bool)` | — | — |
| `0x17`-`0x1A` Set DCE Kp/Kv/Ki/Kd | 主→电机 | 设置 DCE PID | `SetDceKp/Kv/Ki/Kd(int32_t)` | — | — |
| `0x1B` Set Stall-Protect | 主→电机 | 堵转开关 | `SetEnableStallProtect(bool)` | — | — |
| `0x21` Get Current | 主→电机 | 查询电流 | `GetTemp()` 发 0x25 | 读回 temperature 字段 | — |
| **`0x23`** Get Position | 主→电机 | 查询位置 | **重构**：`UpdateAngle()` 单播轮询（100Hz / 7轴 ≈ 14Hz/电机）；旧用 `motorJ[ALL]->UpdateAngle()` 广播触发 | **重构**：新固件处理 id=7（J7/地轨）、id=1~6（臂关节）、id=8（夹爪）三类回包；旧只处理 `dummy.motorJ[id]` | **重构** |
| `0x28`/`0x29`/`0x2A`/`0x2B` Get DCE Kp/Kv/Ki/Kd | 主→电机 | 查询 DCE PID | **新增**：`QueryDceKp/Kv/Ki/Kd()` | **新增**：PID 合并打印（4次回包合并为1次 printf），主线程 poll `PollPidPrint()` | **新增** |
| **`0x2C`** Get Acceleration | 主→电机 | 查询加速度 | **新增**：`QueryAcceleration()` → `SyncAllMotorAcceleration()` 批量查询 8 个电机 | **新增**：缓存 `jointAccRuntime[idx]` + `jointAccLastUpdateMs[idx]`；异步写，无 ISR 内 printf | **新增** |
| **`0x2D`** Get Current-Limit | 主→电机 | 查询电流限幅 | **新增**：`QueryCurrentLimit()` | **新增**：缓存 `jointCurrentLimitRuntime[idx]` + `jointCurrentLimitLastUpdateMs[idx]` | **新增** |
| **`0x5A`** STALL Broadcast | 电机→主 | 堵转事件上报 | — | **新增**：设置 `_stall_event_flag/node/cmd`，主线程(OledTask) poll 后 printf | **新增** |
| **`0x5B`** UNLOCKED Broadcast | 主→ALL | 全局解锁 | **新增**：`BroadcastUnlock()` | — | **新增** |
| **`0x5C`** Stall Query | 主→电机 | 查询堵转状态 | **新增**：`QueryStallStatus(uint8_t queryType)` | **新增**：设置 `_stall_status_flag/node/type/val`，主线程 poll 后 printf | **新增** |
| **`0x09`** Broadcast EStop | 主→ALL | 急停广播 | **新增**：`BroadcastEStop()`，StdId=(nodeID<<7)\|0x09 | — | **新增** |
| `0x7D` Enable Temp Watch | 主→电机 | 温度监控 | `SetEnableTemp(bool)` | — | — |
| `0x7E` Erase Configs | 主→电机 | 擦除配置 | `EraseConfigs()` | — | — |
| `0x7F` Reboot | 主→电机 | 重启电机 | `Reboot()` | — | — |
| **`0x02`** Do Calibration | 主→电机 | 编码器校准 | **删除**（旧固件有 `DoCalibration()` 方法；新固件删除了此方法） | — | **删除** |

### 1.2 `UpdateAngle()` 轮询策略变化

> `can_protocol.cpp:UpdateAngle()` 调用方式变化。

| 维度 | 旧固件 | 新固件 | 标记 |
|---|---|---|---|
| 触发方式 | `motorJ[ALL]->UpdateAngle()` 广播（1 帧触发所有电机） | 单播轮询：`dummy.motorJ[currentIndex]->UpdateAngle()`，currentIndex 在 1~7 循环 | **重构** |
| 频率 | 实时环内每帧（5kHz）触发 | 100Hz / 7轴 ≈ 14Hz/电机（`updateCounter % 50 == 25`） | **重构** |
| 地轨处理 | 无地轨 | `can_protocol.cpp:179-184`：J7/地轨回包 `0x23` 时同步 `currentRailPos = motorJ[7]->angle / 360.0f * 5.0f` | **新增** |

### 1.3 CAN 发送层防卡死（`Bsp/communication/interface_can.cpp`）

> `CanSendMessage()` 发送函数变化。

| 维度 | 旧固件 | 新固件 | 标记 |
|---|---|---|---|
| 发送等待 | `osSemaphoreAcquire(sem_can1_tx, osWaitForever)` 无限等 | **新增** mailbox 空闲检查（最多 5ms）+ 10ms 信号量超时 | **重构** |
| 失败处理 | 无 | HAL 失败时释放信号量，防止死锁 | **新增** |

---

## 二、ASCII 命令协议对比（`UserApp/protocols/ascii_protocol.cpp`）

> 新固件 USB/UART4 双通道完整实现，UART5 空壳。下表按 `!` / `#` / `>/&/@/$` 分类。

### 2.1 `!` 系统控制命令

| 命令 | 旧 ASCII 行号 | 旧实现 | 新 ASCII 行号 | 新实现 | 标记 |
|---|---|---|---|---|---|
| `!STOP` | 旧: `find("STOP")` | `EmergencyStop()` | 新: `s == "!STOP"` | `EmergencyStop()`（走 CAN 0x09 急停） | **重构** |
| `!START` | `find("START")` | `SetEnable(true)` | `s == "!START"` | `SetEnable(true)` + 解除 `isStalled` | **重构** |
| `!DISABLE` | `find("DISABLE")` | `SetEnable(false)` | `s == "!DISABLE"` | `SetEnable(false)` + 解除 `isStalled` | **重构** |
| `!HOME` | `find("HOME")` | `Homing()` | `s == "!HOME"` + `isStalled` 检查 | `Homing()` + 堵转期间拒绝 | **重构** |
| `!RESET` | `find("RESET")` | `Resting()` | `s == "!RESET"` + `isStalled` 检查 | `Resting()` + 堵转期间拒绝 | **重构** |
| `!CALIBRATION` | `find("CALIBRATION")` | `CalibrateHomeOffset()` | ❌ | **删除**（旧固件 `CalibrateHomeOffset()` 方法已移除） | **删除** |
| `!RGB_SET_START` | ❌ | — | 新增 | `SaveConfig()` + `rgbStateStart` | **新增** |
| `!RGB_SET_ENABLE` | ❌ | — | 新增 | `SaveConfig()` + `rgbStateEnable` | **新增** |
| `!RGB_SET_DISABLE` | ❌ | — | 新增 | `SaveConfig()` + `rgbStateDisable` | **新增** |
| `!RGB_BRIGHT` | ❌ | — | 新增 | 亮度查询/设置/设置并保存 | **新增** |
| `!RGB_MODE` | ❌ | — | 新增 | 切换灯效模式 0~9 | **新增** |
| `!RGB_COLOR` | ❌ | — | 新增 | 静态纯色 RGB 0~255 | **新增** |
| `!HAND_ZERO` | ❌ | — | 新增 | 夹爪标定：记录闭合位置为零点 | **新增** |
| `!HAND_EN` | ❌ | — | 新增 | 夹爪使能 | **新增** |
| `!HAND_DIS` | ❌ | — | 新增 | 夹爪失能 | **新增** |
| `!HAND_POS` | ❌ | — | 新增 | 夹爪位置控制 0~100 | **新增** |
| `!HAND_O` | ❌ | — | 新增 | 夹爪张开（施力 -1） | **新增** |
| `!HAND_C` | ❌ | — | 新增 | 夹爪闭合（施力 +1） | **新增** |
| `!STALL_EN` | ❌ | — | 新增 | `SetEnableStallProtect(true)` → J1~J7/地轨 | **新增** |
| `!STALL_DIS` | ❌ | — | 新增 | `SetEnableStallProtect(false)` → J1~J7/地轨 | **新增** |
| `!STALL_STATUS` | ❌ | — | 新增 | `QueryStallStatus()` + 500ms 限流 | **新增** |
| `!STALL_UNLOCK` | ❌ | — | 新增 | `BroadcastUnlock()` ×3 次 | **新增** |
| `!PRINTPOSE` | ❌ | — | 新增 | 打印 GETJPOS + GETLPOS | **新增** |

### 2.2 `#` 查询/配置命令

| 命令 | 旧 ASCII 行号 | 旧实现 | 新 ASCII 行号 | 新实现 | 标记 |
|---|---|---|---|---|---|
| `#GETJPOS` | 返回 6 值 | `dummy.currentJoints.a[0..5]` | 返回 7 值 | + `currentRailPos`（Rail, J1~J6） | **重构** |
| `#GETLPOS` | 调用 `UpdateJointPose6D()` | 通信线程内触发 FK | 直接读缓存 | `currentPose6D`（预计算缓存） | **重构** |
| `#SET_DCE_KP` | 仅 J1~J6 | `SetDceKp(kp)` | 扩到 J7/地轨(ID=7)、夹爪(ID=8) | `motorJ[7]/motorJ[node]/hand` | **重构** |
| `#SET_DCE_KV/KI/KD` | 仅 J1~J6 | 同上 | 同上扩域 | 同上 | **重构** |
| `#SET_PID` | ❌ | — | 新增 | 批量设置 Kp/Kv/Ki/Kd | **新增** |
| `#GET_PID` | ❌ | — | 新增 | 批量查询 Kp/Kv/Ki/Kd（回包走 `PollPidPrint()` 合并） | **新增** |
| `#REBOOT` | 仅带参数 | `#REBOOT %lu` | 无参版 + 带参版 | 无参 → ALL 代理广播 + 主控 reset；带参 → 指定节点 | **重构** |
| `#CMDMODE` | 存在 | `SetCommandMode()` | 同 | 支持新模式 COMMAND_SERVO_J/COMMAND_TORQUE_CONTROL | **重构** |
| `#OFFSET_J` | ❌ | — | 新增 | 单轴归零标定 / 批量 `all` | **新增** |
| `#ACC_J` | ❌ | — | 新增 | `SetAcceleration_persist(float, bool)`；支持 `&` 持久化标志；扩到 J7/地轨、夹爪 | **新增** |
| `#ACC_BASE_J` | 旧有（%） | `SetAcceleration(bases * %)` | ❌ | **删除**（v2.6 单位统一后已废弃） | **删除** |
| `#ACC_RAIL` | 旧有 | 单独地轨加速度 | ❌ | **删除**（走 `#ACC_J 7` 即可） | **删除** |
| `#SPEED_J` | ❌ | — | 新增 | 设置单轴速度限幅 | **新增** |
| `#I_LIMIT_J` | ❌ | — | 新增 | `SetCurrentLimit_persist(float, bool)`；扩到 J7/地轨、夹爪 | **新增** |
| `#GETJACC` | ❌ | — | 新增 | 同步轮询 `jointAccRuntime[idx]`（30ms 轮询时间戳） | **新增** |
| `#GETI` | ❌ | — | 新增 | 同步轮询 `jointCurrentLimitRuntime[idx]`（30ms 轮询时间戳） | **新增** |
| `#SYNC_ACC` | ❌ | — | 新增 | 批量触发 8 个电机 `QueryAcceleration()` | **新增** |

### 2.3 `>/&/@/$` 运动指令格式

| 维度 | 旧固件 | 新固件 | 标记 |
|---|---|---|---|
| `>/&` 参数数量 | 6 个关节 + speed（第 7 位） | **7 个关节 + 地轨 + speed**（>j1,j2,j3,j4,j5,j6,j7_rail,speed） | **重构** |
| `@` 参数数量 | 6 维坐标 + speed | 6 维坐标 + speed（不变） | — |
| `$` 力矩指令 | ❌ | **新增**：`$c1,c2,c3,c4,c5,c6,c7_rail`（7 个 float） | **新增** |
| SERVO_J 格式 | ❌ | **新增**：`>/& j1..j6,j7_rail`（无 speed，第 7 位是 j7） | **新增** |
| 队列机制 | 无独立队列线程 | **新增**：`CommandHandler` 独立队列（32 条 × 128 字节） | **新增** |
| 未使能拒绝 | 无保护 | **新增**：运动指令检查 `IsEnabled()` + `IsStalled()` | **新增** |

---

## 三、`DummyRobot` 类对比（`Robot/instances/dummy_robot.h` + `.cpp`）

### 3.1 架构变化

| 维度 | 旧固件 | 新固件 | 标记 |
|---|---|---|---|
| motorJ[] 数组大小 | `CtrlStepMotor* motorJ[7]`（[0]=ALL, [1..6]=J1~J6） | `CtrlStepMotor* motorJ[8]`（[0]=ALL广播代理, [1..6]=J1~J6, [7]=J7/地轨） | **重构** |
| hand 类 | `DummyHand* hand`（ID=7，裸 CAN 发帧） | `StepHand* hand`（ID=8，继承 `CtrlStepMotor`） | **重构** |
| DH 参数 | `{0.109, 0.035, 0.146, 0.115, 0.052, 0.072}` | `{0.165, 0.0, 0.170, 0.117, 0.0695, 0.113}`（新机械结构） | **重构** |

### 3.2 新增常量（`dummy_robot.h`）

| 常量 | 值 | 定义行 | 说明 |
|---|---|---|---|
| `ALL` | `0` | `dummy_robot.h:7` | 广播代理索引（StdId=(0<<7)\|cmd 触发所有电机） |
| `SLIDER_TO_RPS` | `0.30f` | `dummy_robot.h:13` | slider 100 → 30 r/s |
| `AXIS_MAX_RPS[8]` | `{0,30,30,30,30,30,30,30}` | `dummy_robot.h:18-22` | 各轴物理上限 r/s；[7]=J7/地轨 |
| `MOTOR_REDUCTION[8]` | `{1,50,50,50,50,50,30,1}` | `dummy_robot.h:27-29` | 各轴减速比；[7]=地轨直连 |
| `HAND_MAX_RPS` | `20.0f` | `dummy_robot.h:31` | 夹爪速度上限 |
| `FALLBACK_JOINT_ACCELERATION` | `10.0f` | `dummy_robot.h:34` | 电机加速度查询未回包时的兜底值 r/s² |
| `RAIL_STEPS_PER_MM` | `40960.0f` | `dummy_robot.h:144` | 丝杆 1605 直连：200步×1024微步/5mm |
| `ALL_FINISH_FLAG` | `0x7E` | `dummy_robot.h:183` | jointsStateFlag 全置位 = bit1~6=1, bit0=0 |
| `REST_POSE` | `{0,-75,180,0,0,0}` | `dummy_robot.h:220` | J1~J6；旧为 `{0,-73,180,0,0,0}` |
| `DEFAULT_JOINT_SPEED` | `80` | `dummy_robot.h:222` | slider 默认值；旧为 `30` |
| `DEFAULT_JOINT_ACCELERATION` | `150.0f` | `dummy_robot.h:229` | r/s²；旧为双层语义 `DEFAULT_JOINT_ACCELERATION_BASES[]` |

### 3.3 新增字段（`dummy_robot.h`）

| 字段 | 类型 | 行号 | 说明 |
|---|---|---|---|
| `currentRailPos` | `float` | 145 | 地轨当前位置 mm |
| `targetRailPos` | `float` | 148 | 地轨目标位置 mm |
| `targetRailCurrent` | `float` | 150 | 地轨目标电流 mA |
| `jointSpeedRps` | `float` | 155 | 电机轴 r/s（slider 换算后） |
| `dynamicJointSpeeds7` | `DynamicJointSpeeds7` | 160-163 | 7 轴速度 r/s（含地轨） |
| `railSpeedRps` | `float` | 176 | 地轨速度 r/s |
| `targetCurrents[6]` | `float[6]` | 178 | 力矩透传 6 轴目标电流 |
| `motorDceKps[9]` | `int32_t[9]` | 181 | DCE Kp 缓存 |
| `jointAccRuntime[9]` | `float[9]` | 233 | 加速度查询缓存 |
| `jointAccLastUpdateMs[9]` | `uint32_t[9]` | 234 | 加速度缓存时间戳 |
| `jointCurrentLimitRuntime[9]` | `float[9]` | 237 | 电流限幅缓存 |
| `jointCurrentLimitLastUpdateMs[9]` | `uint32_t[9]` | 238 | 电流限幅缓存时间戳 |
| `isStalled` | `bool` | 320 | 堵转拦截状态 |
| `lastServoTime` | `uint32_t` | 186 | ServoJ 上次时间 us |

### 3.4 新增/删除方法

| 方法 | 行为 | 标记 |
|---|---|---|
| `MoveJ(float... float _j7_mm, float _slider)` | 7 参数（含地轨）；内置梯形同步规划 `ComputeSyncSpeeds()` | **新增** |
| `ServoJ(float... float _j7_mm)` | 高频随动（含地轨）；无加减速规划 | **新增** |
| `MoveL(...)` | 笛卡尔直线（含地轨保持）；反解选最近解后调 MoveJ | **重构** |
| `SyncAllMotorAcceleration()` | 批量发 CAN 0x2C 到 8 个电机 | **新增** |
| `SetStallMode()` / `SetStallMode(int)` | 堵转时停止发指令 + 同步 targetRailPos | **新增** |
| `BroadcastUnlock()` | 连续 3 次 CAN 0x5B 广播（间隔 100ms） | **新增** |
| `QueryStallStatus()` | 查询 7 个电机 J1~J7/地轨的 en/lock 状态 | **新增** |
| `SetJointCurrents(float c0..c7)` | 力矩透传：c0=地轨，c1~c6=关节，c7=夹爪 | **新增** |
| `LoadConfig()` / `SaveConfig()` | EEPROM 灯效配置读写 | **新增** |
| `CalibrateHomeOffset()` | ❌ | **删除**（旧固件有；新固件由 `#OFFSET_J` 替代） |
| `SetRGBEnable(bool)` | ❌ | **删除**（RGB 使能由 `rgbStateStart/Enable/Disable` 替代） |
| `GetRGBEnabled()` | ❌ | **删除** |

### 3.5 `CommandHandler` 变化

| 维度 | 旧固件 | 新固件 | 标记 |
|---|---|---|---|
| FIFO 类型 | `osMessageQueueNew(16, 64)` | `osMessageQueueNew(32, 128)` | **重构** |
| Push 参数类型 | `const std::string&` | `const char*` | **重构** |
| Pop 返回类型 | `std::string` | `const char*` | **重构** |
| ParseCommand 参数类型 | `const std::string&` | `const char*` | **重构** |
| 独立消费线程 | ❌（`ParseCommand` 在 Pop 后直接调用） | **新增**：`ThreadControlLoopUpdate()` 独立消费 FIFO（`main.cpp:71-76`） | **新增** |
| EmergencyStop 行为 | `MoveJ(currentJoints)`（伪急停） | CAN 0x09 广播急停 + 置 `ALL_FINISH_FLAG` + 清 FIFO | **重构** |
| 堵转期间运动指令保护 | 无 | `ParseCommand` 入口检查 `isStalled` → 运动指令返回 ok 但不入队 | **新增** |

### 3.6 `SetCommandMode` 变化

| 模式 | 旧固件 | 新固件 | 标记 |
|---|---|---|---|
| `COMMAND_TARGET_POINT_SEQUENTIAL` (=1) | `SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW)` | 不覆盖电机端加速度；由 `SyncAllMotorAcceleration()` 读 EEPROM 值 | **重构** |
| `COMMAND_TARGET_POINT_INTERRUPTABLE` (=2) | 同上 | 同上 | **重构** |
| `COMMAND_CONTINUES_TRAJECTORY` (=3) | `SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_HIGH)` + `jointSpeedRatio=0.3` | 不覆盖加速度；`jointSpeedRatio` 不自动减半 | **重构** |
| `COMMAND_MOTOR_TUNING` (=4) | 同上 | 同上 | — |
| **`COMMAND_TORQUE_CONTROL`** (=5) | ❌ | **新增**：力矩透传模式，5kHz 实时环下发 `$` 指令 | **新增** |
| **`COMMAND_SERVO_J`** (=6) | ❌ | **新增**：高频伺服随动，`SetJointAcceleration(DEFAULT_JOINT_ACCELERATION=150)` | **新增** |

### 3.7 `IsMoving()` 变化

| 维度 | 旧固件 | 新固件 | 标记 |
|---|---|---|---|
| 旧实现 | `jointsStateFlag != 0b1111110` | — | — |
| 新实现 | — | `!isEnabled ? false : jointsStateFlag != ALL_FINISH_FLAG`（`main.cpp:208`） | **重构** |
| 变化 | bit0 地轨恒 1 | 短路：失能时直接 false | **重构** |

---

## 四、`CtrlStepMotor` CAN 接口对比（`Robot/actuators/ctrl_step.h` + `.cpp`）

### 4.1 新增方法

| 方法 | StdId | 功能 | 行号 |
|---|---|---|---|
| `SetAngleWithMotorRps(float angle, float rps)` | — | 角度+速度（用 `reduction` 换算） | `ctrl_step.cpp:74` |
| `SetCurrentLimit_persist(float, bool)` | `(nodeID<<7)\|0x12` | 按参数决定是否写 EEPROM | `ctrl_step.cpp:101` |
| `SetAcceleration_persist(float, bool)` | `(nodeID<<7)\|0x14` | 按参数决定是否写 EEPROM | `ctrl_step.cpp:127` |
| `QueryCurrentLimit()` | `(nodeID<<7)\|0x2D` | 查询电流限幅 | `ctrl_step.cpp:136` |
| `QueryAcceleration()` | `(nodeID<<7)\|0x2C` | 查询加速度 | `ctrl_step.cpp:141` |
| `QueryDceKp()` | `(nodeID<<7)\|0x28` | 查询 DCE Kp | `ctrl_step.cpp:211` |
| `QueryDceKv()` | `(nodeID<<7)\|0x29` | 查询 DCE Kv | `ctrl_step.cpp:218` |
| `QueryDceKi()` | `(nodeID<<7)\|0x2A` | 查询 DCE Ki | `ctrl_step.cpp:225` |
| `QueryDceKd()` | `(nodeID<<7)\|0x2B` | 查询 DCE Kd | `ctrl_step.cpp:232` |
| `BroadcastUnlock()` | `0x5B` | 广播 UNLOCKED | `ctrl_step.cpp:238` |
| `BroadcastEStop()` | `(nodeID<<7)\|0x09` | 广播急停（nodeID=0 → 0x09 ALL） | `ctrl_step.cpp:248` |
| `QueryStallStatus(uint8_t)` | `(nodeID<<7)\|0x5C` | 查询堵转 en/lock | `ctrl_step.cpp:260` |

### 4.2 方法签名变化

| 方法 | 旧签名 | 新签名 | 标记 |
|---|---|---|---|
| `SetPositionWithVelocityLimit` | `SetPositionWithVelocityLimit(float pos, float vel)` | **重命名**：`SetPositionWithMotorRps(float pos, float rps)` | **重构** |
| `SetAngleWithVelocityLimit` | `SetAngleWithVelocityLimit(float angle, float vel)` | **重命名**：`SetAngleWithMotorRps(float angle, float rps)` | **重构** |
| `SetEnable` | `state = _enable ? FINISH : STOP`（乐观同步） | disable 时同步 `state = STOP`；enable 时**不清**（等 0x23 回包纠正） | **重构** |
| `DoCalibration` | `DoCalibration()` | ❌ **删除** | **删除** |

### 4.3 字段变化

| 字段 | 旧固件 | 新固件 | 标记 |
|---|---|---|---|
| `State` 枚举 | `RUNNING, FINISH, STOP` | **新增** `STALL` | **新增** |
| `targetAngle` | 无 | `float targetAngle`（MoveJ/ServoJ 写入，禁用时清 0） | **新增** |
| `AllAtTarget()` | 无 | `bool AllAtTarget(float epsilon_deg)` 方法 | **新增** |

### 4.4 `SetEnable` 行为变化详解

> `ctrl_step.cpp:23-34`。

旧：`state = _enable ? FINISH : STOP`（乐观同步，enable 时立即写 FINISH）。

新：disable → `state = STOP`；enable → **不清 state**（依赖电机端首次 0x23 回包纠正）。这与 `dummy_robot.cpp` 的 `jointsStateFlag` 方案对齐：电机 STOP 时 `UpdateAngleCallback` 会将 `jointsStateFlag |= (1<<i)`，使 `IsMoving()` 返回 false。

---

## 五、FreeRTOS 任务架构对比（`UserApp/main.cpp`）

| 线程名 | 优先级 | 频率 | 旧栈大小 | 新栈大小 | 旧实现 | 新实现 | 标记 |
|---|---|---|---|---|---|---|---|
| `ControlLoopFixUpdateTask` | `osPriorityRealtime` | 5kHz (TIM7 200μs) | 2000 | 2000 | 简单下发 MoveJ/UpdateJointAngles/Tuning | **总线节流**（50Hz 下发）+ 方案C单播轮询 + 新模式处理（ServoJ/Torque/Tuning） | **重构** |
| `ControlLoopUpdateTask` | `osPriorityNormal` | 事件触发 | 2000 | 2000 | 直接 `ParseCommand(Pop())` | **独立线程**：`Pop()` → `ParseCommand()` | **重构** |
| `OledTask` | `osPriorityNormal` | ~60Hz | 2000 | 2000 | IMU 100Hz + OLED 显示 | IMU 100Hz + **堵转事件 poll** + **PID 合并打印 poll** + **调试探针** | **重构** |
| `RGBTask` | `osPriorityNormal` | 33Hz | 2000 | 2000 | `rgb.Run()` 阻塞等 DMA | **非阻塞**：`UpdateLoop()` 检查 DMA 空闲 → `Run()` → 跳过 BUSY 帧 | **重构** |
| `CommunicationTask` | `osPriorityNormal` | — | 45000 | 45000 | 相同 | 相同 | — |
| `UsbDeferredInterruptTask` | `osPriorityNormal` | — | — | — | 相同 | 相同 | — |

### 5kHz 实时环各模式行为（`main.cpp:43-119`）

| 模式 | 下发频率 | 说明 |
|---|---|---|
| SEQ / INT / TRJ | **50Hz**（`updateCounter % 100 == 0`） | 总线节流，避免 7×5kHz=35k fps 撞爆 CAN |
| SERVO_J | **5kHz 全速**（每帧下发） | 高频随动，不节流 |
| MOTOR_TUNING | 100Hz | 扫频信号发生器 |
| TORQUE_CONTROL | 100Hz | 直接下发电流 |

---

## 六、关键常量对比表

| 常量 | 旧值 | 新值 | 文件:行号 | 说明 |
|---|---|---|---|---|
| `REST_POSE` | `{0,-73,180,0,0,0}` | `{0,-75,180,0,0,0}` | `dummy_robot.h:220` vs `old:dummy_robot.h` | J2 从 -73° 改为 -75° |
| `DEFAULT_JOINT_SPEED` | `30` | `80` | `dummy_robot.h:222` vs `old:dummy_robot.h` | slider 默认值大幅提升 |
| `DEFAULT_COMMAND_MODE` | `COMMAND_TARGET_POINT_INTERRUPTABLE` | `COMMAND_TARGET_POINT_SEQUENTIAL` | `dummy_robot.h:239` vs `old:dummy_robot.h` | 默认 SEQ |
| 加速度单位 | `DEFAULT_JOINT_ACCELERATION_BASES`+`LOW`/`HIGH` 双层 | `DEFAULT_JOINT_ACCELERATION=150.0f`（统一 r/s²） | `dummy_robot.h:229` | v2.6 重构 |
| `SLIDER_TO_RPS` | ❌ 无 | `0.30f` | `dummy_robot.h:13` | **新增** |
| `RAIL_STEPS_PER_MM` | ❌ 无地轨 | `40960.0f` | `dummy_robot.h:144` | **新增** |
| DH 参数 L1 | `0.109f` | `0.165f` | `dummy_robot.cpp:163` vs `old:dummy_robot.cpp` | 新机械结构 |
| DH 参数 D1 | `0.035f` | `0.0f` | 同上 | 新机械结构 |

---

## 七、新增/重构/删除 汇总

### 7.1 新增（38 项）

**CAN 协议相关（8 项）：**
1. CAN `0x09` Broadcast EStop → `BroadcastEStop()` 方法
2. CAN `0x28` Get DCE Kp → `QueryDceKp()` 方法
3. CAN `0x29` Get DCE Kv → `QueryDceKv()` 方法
4. CAN `0x2A` Get DCE Ki → `QueryDceKi()` 方法
5. CAN `0x2B` Get DCE Kd → `QueryDceKd()` 方法
6. CAN `0x2C` Get Acceleration → `QueryAcceleration()` + 缓存 `jointAccRuntime[]`
7. CAN `0x2D` Get Current-Limit → `QueryCurrentLimit()` + 缓存 `jointCurrentLimitRuntime[]`
8. CAN `0x5A/0x5B/0x5C` 堵转协议 → ISR 标志 + 主线程 poll 机制

**ASCII 命令（10 项）：**
9. `!STALL_EN/!STALL_DIS/!STALL_STATUS/!STALL_UNLOCK` 堵转控制命令
10. `!HAND_ZERO/!HAND_EN/!HAND_DIS/!HAND_POS/!HAND_O/!HAND_C` 夹爪命令
11. `!RGB_SET_START/!RGB_SET_ENABLE/!RGB_SET_DISABLE/!RGB_BRIGHT/!RGB_MODE/!RGB_COLOR` 灯效命令
12. `!PRINTPOSE` 打印位姿
13. `#SET_PID/#GET_PID` 批量 PID
14. `#OFFSET_J` 归零标定
15. `#ACC_J/#SPEED_J/#I_LIMIT_J/#GETJACC/#GETI/#SYNC_ACC` 参数查询
16. `$` 力矩透传指令格式
17. `>/&` 运动指令 7 轴（含地轨）格式
18. `COMMAND_TORQUE_CONTROL` 模式（=5）
19. `COMMAND_SERVO_J` 模式（=6）

**DummyRobot 类（12 项）：**
20. `StepHand` 类（替代 `DummyHand`）
21. `EepromConfig` 结构体
22. `LoadConfig()` / `SaveConfig()` 方法
23. `SyncAllMotorAcceleration()` 方法
24. `SetStallMode()` / `BroadcastUnlock()` / `QueryStallStatus()` 方法
25. `SetJointCurrents()` 方法
26. `ServoJ()` 方法
27. `ComputeSyncSpeeds()` 同步速度规划函数（梯形曲线模型）
28. 地轨相关字段（`currentRailPos` / `targetRailPos` / `railSpeedRps` 等）
29. `isStalled` 字段 + 堵转拦截逻辑
30. 独立队列消费线程 `ThreadControlLoopUpdate()`
31. `EepromConfig` / `EEPROM_MAGIC`

**CtrlStepMotor 类（5 项）：**
32. `SetAngleWithMotorRps()` 方法
33. `SetCurrentLimit_persist()` / `SetAcceleration_persist()` 方法
34. `BroadcastUnlock()` / `BroadcastEStop()` / `QueryStallStatus()` 方法
35. `State::STALL` 枚举值
36. `targetAngle` 字段 + `AllAtTarget()` 方法

**其他（4 项）：**
37. `comm_pose_uart.h/cpp` 占位文件
38. `pose_sender_task.cpp` 占位文件

### 7.2 重构（18 项）

1. `UpdateAngle()` 从 ALL 广播 → 方案C 单播轮询（`dummy_robot.cpp:318-323`）
2. `CanSendMessage()` 增加 mailbox 检查和超时（`interface_can.cpp:195-210`）
3. `!STOP` EmergencyStop 从伪急停 → CAN 0x09 广播抱闸
4. `!START/!DISABLE` 增加 `isStalled` 解除逻辑
5. `!HOME/!RESET` 增加堵转期间拒绝逻辑
6. `#GETJPOS` 返回 6 值 → 7 值（含地轨）
7. `#GETLPOS` 从通信线程内 FK → 直接读预计算缓存
8. `#REBOOT` 无参版 → ALL 代理广播 + 主控 reset
9. `MoveJ()` 6 轴 → 7 轴（含地轨 + 梯形同步规划）
10. `MoveL()` 包含地轨保持
11. `SetCommandMode` 不覆盖电机端加速度，改为 SyncAllMotorAcceleration 读 EEPROM
12. `IsMoving()` 增加失能短路 + `ALL_FINISH_FLAG` 方案
13. `SetEnable()` 禁用乐观同步，等 0x23 回包纠正
14. `SetPositionWithVelocityLimit` → `SetPositionWithMotorRps`（重命名 + 语义）
15. `SetAngleWithVelocityLimit` → `SetAngleWithMotorRps`
16. `SetCurrentLimit` → `SetCurrentLimit_persist`（按参数决定 EEPROM）
17. `SetAcceleration` → `SetAcceleration_persist`（按参数决定 EEPROM）
18. RGB DMA 非阻塞机制（`UpdateLoop()` + `dmaBusy` + `dmaEvent`）

### 7.3 删除（7 项）

1. `DoCalibration()` 方法（`CtrlStepMotor`）
2. `CalibrateHomeOffset()` 方法（`DummyRobot`）
3. `SetRGBEnable()` / `GetRGBEnable()` 方法
4. `#ACC_BASE_J` 命令（v2.6 单位统一后废弃）
5. `#ACC_RAIL` 命令（v2.6 统一到 `#ACC_J 7`）
6. `DummyHand` 类（旧夹爪，ID=7）
7. `DummyHand::SetAngle()` / `SetMaxCurrent()`（被 `StepHand` 替代）

---

## 八、未变化文件清单

以下文件在新旧固件中完全一致：

- `Bsp/communication/communication.cpp`
- `Bsp/communication/interface_can.cpp`（底层框架；仅 `CanSendMessage()` 有加固变化，见 §一.3）
- `Bsp/memory/emulated_eeprom.cpp` + `.h`
- `Bsp/gpio/encoder.cpp` + `.hpp`
- `Bsp/gpio/pwm.cpp` + `.hpp`
- `Bsp/gpio/analog.cpp` + `.hpp`
- `Bsp/utils/timer.cpp` + `.hpp`
- `Bsp/utils/time_utils.c` + `.h`
- `Robot/algorithms/kinematic/6dof_kinematic.h` + `.cpp`（bit-identical）
- `UserApp/protocols/cmd_protocol.cpp`
- `Bsp/communication/ascii_processor.cpp`（仅增加 DIAG 心跳计数器）
