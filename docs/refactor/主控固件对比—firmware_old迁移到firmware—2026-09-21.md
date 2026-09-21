# 主控固件对比 — `firmware_old` → `firmware` 迁移参考

> **范围**：仅主控固件（STM32F405，FreeRTOS）。对比 `firmware_old/dummy-ref-core-fw`（旧）与 `firmware/ref_core_f405`（新）。
> **基线对比**：旧固件为 6 轴架构（motorJ[0..6]，含 DummyHand）；新固件为 **7 轴 + 地轨 + 夹爪** 架构（motorJ[0..7]，含 StepHand）。
> **数据来源**：直接读源码，每条标注 `文件:行号`。**文档聚焦"差异"**，未变化的部分不列。
> **创建日期**：2026-09-21
>
> ### 📋 agent 标记说明
>
> 本文档每个表格均新增 `new（手动标记）` 列，由 agent 根据实际固件代码分析填写，**内容均以** `(agent)` **视角给出建议**（如"保留新：xxx"、"保留现状：xxx"、"退回旧：xxx"等）。
>
> **审核流程**：
>
> 1. 逐条审查 `new（手动标记）` 列内容是否合理
> 2. 审查通过后，从文档中**删除所有** `(agent)` **标记前缀**（"保留新：xxx" → "xxx"，"退回旧：xxx" → "xxx"），保留建议文字
> 3. 如不同意建议，可整条删除该列或改为更准确的描述
>
> ### ⚠️ 与电机固件对比的差异
>
> 本文档与 `电机固件对比—firmware_old迁移到firmware—2026-09-21.md` 的策略不同：
>
> - **电机固件**：明确"不实现 stall v3"，大量"退回旧"
> - **主控固件**：保留大量新增功能（堵转协议 ISR 标志 + 主线程 poll、新命令模式 SERVO_J/TORQUE、夹爪、灯效等）
>
> **原因**：主控侧不实现 stall v3 检测算法，但保留所有上层命令通路（`!STALL_`*、`#GET_PID`、堵转事件 poll），是否启用取决于电机端固件端的决定。

---

## 一、CAN 协议对比（`UserApp/protocols/can_protocol.cpp`）

> 主控固件不解析电机端 cmd 含义（电机端语义见电机固件对比文档），只关注"主控侧如何构造请求帧"和"如何处理回包"。

### 1.1 电机端 cmd 构造方式

> 所有 cmd 构造方式在 `Robot/actuators/ctrl_step/ctrl_step.cpp` 中实现，StdId 格式 `(nodeID << 7) | cmdCode`。


| Cmd                                             | 方向    | 名称         | 主控侧构造（ctrl_step.cpp）                                                                                          | 回包处理（can_protocol.cpp）                                                            | 标记     | new（手动标记）                                                                                                                                                                                                         |
| ----------------------------------------------- | ----- | ---------- | ------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------- | ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `0x01` Enable                                   | 主→电机  | 使能/失能      | `SetEnable(bool)`，StdId=(nodeID<<7)                                                                           | 0x01                                                                              | —      | **保留现状：基础命令无变化**                                                                                                                                                                                                  |
| `0x03` Set Current                              | 主→电机  | 力矩透传       | `SetCurrentSetPoint(float A)`                                                                                 | —                                                                                 | —      | **保留现状：基础命令无变化**                                                                                                                                                                                                  |
| `0x04` Set Velocity                             | 主→电机  | 速度模式       | `SetVelocitySetPoint(float r/s)`                                                                              | —                                                                                 | —      | **保留现状：基础命令无变化**                                                                                                                                                                                                  |
| `0x05` Set Position                             | 主→电机  | 位置模式（简单）   | `SetPositionSetPoint(float 圈)`，data[4]=1                                                                      | —                                                                                 | —      | **保留现状：基础命令无变化**                                                                                                                                                                                                  |
| `0x07` Set Position w/ VelLimit                 | 主→电机  | 位置+速度限幅    | **重构**：新方法名 `SetPositionWithMotorRps(float 圈, float r/s)`，旧名 `SetPositionWithVelocityLimit`                   | —                                                                                 | **重构** | **保留新：重命名 + 参数语义（rps）更清晰；旧固件方法名** `SetPositionWithVelocityLimit` **已废弃，建议保留新名**                                                                                                                                   |
| `0x11` Set NodeID                               | 主→电机  | 设置节点 ID    | `SetNodeID(uint32_t)`                                                                                         | —                                                                                 | —      | **保留现状：基础命令无变化**                                                                                                                                                                                                  |
| `0x12` Set I-Limit                              | 主→电机  | 设置电流限幅     | **重构**：`SetCurrentLimit_persist(float A, bool persist)`（按参数决定是否写 EEPROM）；旧只有 `SetCurrentLimit(float)` 固定写     | —                                                                                 | **重构** | **保留新：**`SetCurrentLimit_persist()` **比固定写 EEPROM 灵活；旧固件默认** `canBuf[4]=1`**（固定写 EEPROM）有时不需要，如需测试时反复调整电流限幅会频繁写 EEPROM；建议保留新版可选持久化**                                                                              |
| `0x13` Set V-Limit                              | 主→电机  | 设置速度限幅     | `SetVelocityLimit(float r/s)`                                                                                 | —                                                                                 | —      | **保留现状：基础命令无变化**                                                                                                                                                                                                  |
| `0x14` Set Acceleration                         | 主→电机  | 设置加速度      | **重构**：`SetAcceleration_persist(float r/s², bool persist)`（按参数决定是否写 EEPROM）；旧只有 `SetAcceleration(float)` 默认不写 | —                                                                                 | **重构** | **(agent) 保留新：**`SetAcceleration_persist()` **与电流限幅对称，可选持久化；建议保留**                                                                                                                                                |
| `0x15` Apply Home                               | 主→电机  | 归零偏移       | `ApplyPositionAsHome()`                                                                                       | —                                                                                 | —      | **(agent) 保留现状：方法名保留；但电机端 0x15 命令的实现可能有 Bug（旧固件** `GetPosition()` **归零后不归零），需关注电机端修复**                                                                                                                            |
| `0x16` Set Enable-On-Boot                       | 主→电机  | 上电自启       | `SetEnableOnBoot(bool)`                                                                                       | —                                                                                 | —      | **(agent) 保留现状：基础命令无变化**                                                                                                                                                                                          |
| `0x17`-`0x1A` Set DCE Kp/Kv/Ki/Kd               | 主→电机  | 设置 DCE PID | `SetDceKp/Kv/Ki/Kd(int32_t)`                                                                                  | —                                                                                 | —      | **(agent) 保留现状：4 个方法签名无变化**                                                                                                                                                                                       |
| `0x1B` Set Stall-Protect                        | 主→电机  | 堵转开关       | `SetEnableStallProtect(bool)`                                                                                 | —                                                                                 | —      | **(agent) 保留现状：基础命令无变化；但固件端是否启用堵转检测应另行决定**                                                                                                                                                                        |
| `0x21` Get Current                              | 主→电机  | 查询电流       | ❌ 主控侧**未实现** `GetCurrent()` 方法（`ctrl_step.cpp/h` 中无此方法）                                                       | —                                                                                 | **缺失** | **(agent) 保留现状：主控从未实现 GetCurrent 命令；电机端** `0x21` **是 Get Current 命令（**`firmware_old/dummy-42motor-fw/.../interface_can.cpp:183`**）；如需查询电流回环，应在 ctrl_step.cpp 中新增** `GetCurrent(float)` **方法发 0x21 + memcpy 回包处理** |
| `0x23` Get Position                             | 主→电机  | 查询位置       | **重构**：`UpdateAngle()` 单播轮询（100Hz / 7轴 ≈ 14Hz/电机）；旧用 `motorJ[ALL]->UpdateAngle()` 广播触发                        | **重构**：新固件处理 id=7（J7/地轨）、id=1~6（臂关节）、id=8（夹爪）三类回包；旧只处理 `dummy.motorJ[id]`         | **重构** | **(agent) 保留新：单播轮询避免总线拥堵；地轨 ID=7 单独处理 + 同步 currentRailPos；夹爪 ID=8 由 hand 类接管；建议保留**                                                                                                                               |
| `0x25` Get Temperature                          | 主→电机  | 查询温度       | `GetTemp()`（`ctrl_step.cpp:215`）                                                                              | memcpy 到 `motorJ[id]->temperature`（`can_protocol.cpp:108/148/189`）                | —      | **(agent) 保留现状：基础命令无变化**                                                                                                                                                                                          |
| `0x28`/`0x29`/`0x2A`/`0x2B` Get DCE Kp/Kv/Ki/Kd | 主→电机  | 查询 DCE PID | **新增**：`QueryDceKp/Kv/Ki/Kd()`                                                                                | **新增**：PID 合并打印（4次回包合并为1次 printf），主线程 poll `PollPidPrint()`                       | **新增** | **(agent) 保留新：GET 查询命令不修改电机端参数；PID 合并打印采用 ISR 内只设标志 + 主线程 poll，避免 ISR 内 printf 与 ASCII 任务 UART TX 竞争；建议保留**                                                                                                       |
| `0x2C` Get Acceleration                         | 主→电机  | 查询加速度      | **新增**：`QueryAcceleration()` → `SyncAllMotorAcceleration()` 批量查询 8 个电机                                        | **新增**：缓存 `jointAccRuntime[idx]` + `jointAccLastUpdateMs[idx]`；异步写，无 ISR 内 printf | **新增** | **(agent) 保留新：缓存 + 时间戳机制避免了 ISR 内 printf；**`#GETJACC` **通过 30ms 轮询时间戳实现同步查询；建议保留**                                                                                                                                |
| `0x2D` Get Current-Limit                        | 主→电机  | 查询电流限幅     | **新增**：`QueryCurrentLimit()`                                                                                  | **新增**：缓存 `jointCurrentLimitRuntime[idx]` + `jointCurrentLimitLastUpdateMs[idx]`  | **新增** | **(agent) 保留新：与 0x2C 对称设计；建议保留**                                                                                                                                                                                  |
| `0x5A` STALL Broadcast                          | 电机→主  | 堵转事件上报     | —                                                                                                             | **新增**：设置 `_stall_event_flag/node/cmd`，主线程(OledTask) poll 后 printf                | **新增** | **(agent) 保留新：ISR 内只设标志 + 主线程 poll 模式避免 ISR 内 printf 阻塞 CAN RX；建议保留（是否使用取决于是否启用堵转检测）**                                                                                                                            |
| `0x5B` UNLOCKED Broadcast                       | 主→ALL | 全局解锁       | **新增**：`BroadcastUnlock()`                                                                                    | —                                                                                 | **新增** | **(agent) 保留新：与 0x5A 配套；建议保留**                                                                                                                                                                                    |
| `0x5C` Stall Query                              | 主→电机  | 查询堵转状态     | **新增**：`QueryStallStatus(uint8_t queryType)`                                                                  | **新增**：设置 `_stall_status_flag/node/type/val`，主线程 poll 后 printf                    | **新增** | **(agent) 保留新：500ms 节流防总线拥塞；建议保留**                                                                                                                                                                                |
| `0x09` Broadcast EStop                          | 主→ALL | 急停广播       | **新增**：`BroadcastEStop()`，StdId=(nodeID<<7)                                                                   | 0x09                                                                              | —      | **新增**                                                                                                                                                                                                            |
| `0x7D` Enable Temp Watch                        | 主→电机  | 温度监控       | `SetEnableTemp(bool)`                                                                                         | —                                                                                 | —      | **(agent) 保留现状：基础命令无变化**                                                                                                                                                                                          |
| `0x7E` Erase Configs                            | 主→电机  | 擦除配置       | `EraseConfigs()`                                                                                              | —                                                                                 | —      | **(agent) 保留现状：基础命令无变化**                                                                                                                                                                                          |
| `0x7F` Reboot                                   | 主→电机  | 重启电机       | `Reboot()`                                                                                                    | —                                                                                 | —      | **(agent) 保留现状：基础命令无变化**                                                                                                                                                                                          |
| `0x02` Do Calibration                           | 主→电机  | 编码器校准      | **删除**（旧固件有 `DoCalibration()` 方法；新固件删除了此方法）                                                                   | —                                                                                 | **删除** | **(agent) 保留删除：编码器校准流程不通过主控驱动；如需校准应在电机端本地完成；建议保留删除**                                                                                                                                                              |


### 1.2 `UpdateAngle()` 轮询策略变化

> `can_protocol.cpp:UpdateAngle()` 调用方式变化。


| 维度   | 旧固件                                        | 新固件                                                                                               | 标记     | new（手动标记）                                                                            |
| ---- | ------------------------------------------ | ------------------------------------------------------------------------------------------------- | ------ | ------------------------------------------------------------------------------------ |
| 触发方式 | `motorJ[ALL]->UpdateAngle()` 广播（1 帧触发所有电机） | 单播轮询：`dummy.motorJ[currentIndex]->UpdateAngle()`，currentIndex 在 1~7 循环                            | **重构** | **(agent) 保留新：单播轮询避免总线冲突；电机端 ALL 广播 vs 单播性能差异在 7 轴时已较明显（同时 7 节点回包可能撞 mailbox），建议保留** |
| 频率   | 实时环内每帧（5kHz）触发                             | 100Hz / 7轴 ≈ 14Hz/电机（`updateCounter % 50 == 25`）                                                  | **重构** | **(agent) 保留新：14Hz/电机足够** `IsMoving()` **状态判定（jitter ≤70ms），对实时性影响可忽略；建议保留**         |
| 地轨处理 | 无地轨                                        | `can_protocol.cpp:179-184`：J7/地轨回包 `0x23` 时同步 `currentRailPos = motorJ[7]->angle / 360.0f * 5.0f` | **新增** | **(agent) 保留新：地轨位置实时维护于** `currentRailPos`**；`**#GETJPOS` **直接读该缓存；建议保留**            |


### 1.3 CAN 发送层防卡死（`Bsp/communication/interface_can.cpp`）

> `CanSendMessage()` 发送函数变化。


| 维度   | 旧固件                                                  | 新固件                                                                  | 标记     | new（手动标记）                                                                                  |
| ---- | ---------------------------------------------------- | -------------------------------------------------------------------- | ------ | ------------------------------------------------------------------------------------------ |
| 发送等待 | `osSemaphoreAcquire(sem_can1_tx, osWaitForever)` 无限等 | **新增** mailbox 空闲检查（最多 5ms）+ 10ms 信号量超时（`interface_can.cpp:248/250`） | **重构** | **(agent) 保留新：mailbox 空闲预检 + 10ms 超时彻底消除三重死锁风险（mailbox 满 + 无限等 + HAL 失败不释放）；F4 修复关键，必须保留** |
| 失败处理 | 无                                                    | HAL 失败时释放信号量，防止死锁（`interface_can.cpp:255-263`）                       | **新增** | **(agent) 保留新：失败补偿释放信号量，避免计数器失衡；必须保留**                                                     |


---

## 二、ASCII 命令协议对比（`UserApp/protocols/ascii_protocol.cpp`）

> 新固件 USB/UART4 双通道完整实现，UART5 空壳。下表按 `!` / `#` / `>/&/@/$` 分类。

### 2.1 `!` 系统控制命令


| 命令                 | 旧 ASCII 行号            | 旧实现                     | 新 ASCII 行号                               | 新实现                                                        | 标记     | new（手动标记）                                                                                                |
| ------------------ | --------------------- | ----------------------- | ---------------------------------------- | ---------------------------------------------------------- | ------ | -------------------------------------------------------------------------------------------------------- |
| `!STOP`            | 旧: `find("STOP")`     | `EmergencyStop()`       | 新: `s == "!STOP"`                        | `EmergencyStop()`（走 CAN 0x09 急停）                           | **重构** | **(agent) 保留新：精确匹配避免误中（如** `!STOP_X`**）；CAN 0x09 急停优于旧版伪急停（MoveJ currentJoints）；建议保留**                   |
| `!START`           | `find("START")`       | `SetEnable(true)`       | `s == "!START"`                          | `SetEnable(true)` + 解除 `isStalled`                         | **重构** | **(agent) 保留新：堵转期间 START 解除 isStalled 是必要的（用户确认后解除堵转）；建议保留**                                             |
| `!DISABLE`         | `find("DISABLE")`     | `SetEnable(false)`      | `s == "!DISABLE"`                        | `SetEnable(false)` + 解除 `isStalled`                        | **重构** | **(agent) 保留新：与 START 对称；建议保留**                                                                          |
| `!HOME`            | `find("HOME")`        | `Homing()`              | `s == "!HOME"` + `isStalled` 检查          | `Homing()` + 堵转期间拒绝                                        | **重构** | **(agent) 保留新：堵转期间拒绝 Homing 避免位置误锁死；建议保留**                                                               |
| `!RESET`           | `find("RESET")`       | `Resting()`             | `s == "!RESET"` + `isStalled` 检查         | `Resting()` + 堵转期间拒绝                                       | **重构** | **(agent) 保留新：与 HOME 对称；建议保留**                                                                           |
| `!CALIBRATION`     | `find("CALIBRATION")` | `CalibrateHomeOffset()` | ❌                                        | **删除**（旧固件 `CalibrateHomeOffset()` 方法已移除）                  | **删除** | **(agent) 保留删除：归零流程已由** `#OFFSET_J` **替代（**`dummy_robot.cpp` **§3.4），新版更精准（单轴 or all）；建议保留删除**           |
| `!RGB_SET_START`   | ❌                     | —                       | 新增（USB:50/241；UART4:818/1018；实现有冗余但功能正确） | `SaveConfig()` + `rgbStateStart`                           | **新增** | **(agent) 保留新：开机默认灯效可配置；建议保留**；**注意：USB 实现两次（顺序中前置匹配 + 后续精确匹配），存在冗余，建议清理一处**                             |
| `!RGB_SET_ENABLE`  | ❌                     | —                       | 新增（USB:64/255；UART4:832/1032）            | `SaveConfig()` + `rgbStateEnable`                          | **新增** | **(agent) 保留新：使能时灯效可配置；建议保留**；**注意：与** `!RGB_SET_START` **类似的 USB 双重匹配冗余**                               |
| `!RGB_SET_DISABLE` | ❌                     | —                       | 新增（USB:78/269；UART4:846/1046）            | `SaveConfig()` + `rgbStateDisable`                         | **新增** | **(agent) 保留新：失能时灯效可配置；建议保留**；**注意：与** `!RGB_SET_START` **类似的 USB 双重匹配冗余**                               |
| `!RGB_BRIGHT`      | ❌                     | —                       | 新增（USB:184；UART4:961）                    | 亮度查询/设置/设置并保存（3 种用法）                                       | **新增** | **(agent) 保留新：3 种用法（无参查询/带参设置/带参设置+& 持久化）比固定命令灵活；建议保留**                                                  |
| `!RGB_MODE`        | ❌                     | —                       | 新增（USB:205；UART4:982）                    | 切换灯效模式 0~9                                                 | **新增** | **(agent) 保留新：基础灯效控制；建议保留**                                                                              |
| `!RGB_COLOR`       | ❌                     | —                       | 新增（USB:218；UART4:995）                    | 静态纯色 RGB 0~255（3 组可配置）                                     | **新增** | **(agent) 保留新：3 组可配置静态纯色 + SaveConfig 持久化；建议保留**                                                         |
| `!HAND_ZERO`       | ❌                     | —                       | 新增（USB:114；UART4:877）                    | 夹爪标定：等待 `AllAtTarget(2.0f)` ≤10s 后 `ApplyPositionAsHome()` | **新增** | **(agent) 保留新：夹爪标定流程；等待 ≤10s 是合理的（夹爪速度慢）；建议保留**                                                          |
| `!HAND_EN`         | ❌                     | —                       | 新增（USB:284；UART4:889）                    | 夹爪使能                                                       | **新增** | **(agent) 保留新：夹爪独立使能；注意 USB 版多回一行 isEnabled 状态；建议保留**                                                    |
| `!HAND_DIS`        | ❌                     | —                       | 新增（USB:290；UART4:894）                    | 夹爪失能                                                       | **新增** | **(agent) 保留新：与 EN 对称；建议保留**                                                                             |
| `!HAND_POS`        | ❌                     | —                       | 新增（USB:296；UART4:899）                    | 夹爪位置控制 0~100                                               | **新增** | **(agent) 保留新：开度指令（0=闭合, 100=张开）；参数越界回 error；建议保留**                                                      |
| `!HAND_O`          | ❌                     | —                       | 新增（USB:125；UART4:919）                    | 夹爪张开（施力 -1）                                                | **新增** | **(agent) 保留新：力矩透传张开；建议保留**                                                                              |
| `!HAND_C`          | ❌                     | —                       | 新增（USB:131；UART4:924）                    | 夹爪闭合（施力 +1）                                                | **新增** | **(agent) 保留新：力矩透传闭合；建议保留**                                                                              |
| `!STALL_EN`        | ❌                     | —                       | 新增（USB:162；UART4:947）                    | `SetEnableStallProtect(true)` → J1~J7/地轨                   | **新增** | **(agent) 保留新：批量开启堵转保护（覆盖 J1~J6 + J7/地轨，跳过 motorJ[0] ALL 代理）；建议保留**                                      |
| `!STALL_DIS`       | ❌                     | —                       | 新增（USB:169；UART4:954）                    | `SetEnableStallProtect(false)` → J1~J7/地轨                  | **新增** | **(agent) 保留新：与 EN 对称；建议保留**                                                                             |
| `!STALL_STATUS`    | ❌                     | —                       | 新增（USB:144；UART4:929）                    | `QueryStallStatus()` + 500ms 限流                            | **新增** | **(agent) 保留新：500ms 节流（**`_stall_status_last_tick` **+** `STALL_STATUS_INTERVAL_MS`**）防总线/UART 拥塞；建议保留** |
| `!STALL_UNLOCK`    | ❌                     | —                       | 新增（USB:157；UART4:942）                    | `BroadcastUnlock()` ×3 次                                   | **新增** | **(agent) 保留新：3 次连续广播（间隔 100ms）确保电机端接收；建议保留**                                                            |
| `!PRINTPOSE`       | ❌                     | —                       | 新增（USB:320；UART4:1060）                   | 打印 GETJPOS + GETLPOS                                       | **新增** | **(agent) 保留新：一键打印位姿；建议保留**                                                                              |


### 2.2 `#` 查询/配置命令


| 命令                  | 旧 ASCII 行号                         | 旧实现                                                       | 新 ASCII 行号                  | 新实现                                                                                           | 标记     | new（手动标记）                                                                             |
| ------------------- | ---------------------------------- | --------------------------------------------------------- | --------------------------- | --------------------------------------------------------------------------------------------- | ------ | ------------------------------------------------------------------------------------- |
| `#GETJPOS`          | 返回 6 值                             | `dummy.currentJoints.a[0..5]`                             | 返回 7 值                      | + `currentRailPos`（Rail, J1~J6）                                                               | **重构** | **(agent) 保留新：地轨纳入查询是必然需求；建议保留**                                                      |
| `#GETLPOS`          | 调用 `UpdateJointPose6D()`           | 通信线程内触发 FK                                                | 直接读缓存                       | `currentPose6D`（预计算缓存）                                                                        | **重构** | **(agent) 保留新：5kHz 控制环已维护缓存，避免通信线程做 FK 与 ASCII 任务竞争；建议保留**                            |
| `#SET_DCE_KP`       | 仅 J1~J6                            | `SetDceKp(kp)`                                            | 扩到 J7/地轨(ID=7)、夹爪(ID=8)     | `motorJ[7]/motorJ[node]/hand`                                                                 | **重构** | **(agent) 保留新：扩域是必然需求（地轨+夹爪也要 DCE）；建议保留**                                             |
| `#SET_DCE_KV/KI/KD` | 仅 J1~J6（KV 在旧版中**不存在**，仅 KP/KI/KD） | 同上                                                        | 同上扩域                        | 同上                                                                                            | **重构** | **(agent) 保留新：**`#SET_DCE_KV` **旧版不存在，是新增项；KV 是 DCE 速度前馈分量，建议保留**                     |
| `#SET_PID`          | ❌                                  | —                                                         | 新增（USB:439；UART4:1199）      | 批量设置 Kp/Kv/Ki/Kd                                                                              | **新增** | **(agent) 保留新：批量设置 4 参数比逐个发送高效；建议保留**                                                 |
| `#GET_PID`          | ❌                                  | —                                                         | 新增（USB:465；UART4:1225）      | 批量查询 Kp/Kv/Ki/Kd（回包走 `PollPidPrint()` 合并）                                                     | **新增** | **(agent) 保留新：合并打印机制避免 4 次回包产生 4 行散乱输出；建议保留**                                         |
| `#REBOOT`           | 仅带参数                               | `#REBOOT %lu`                                             | 无参版 + 带参版                   | 无参 → ALL 代理广播 + 主控 reset；带参 → 指定节点（1~7，不含 8）                                                  | **重构** | **(agent) 保留新：无参版一键全重启（v3.2 ALL 代理兼容）；带参版节点范围扩到 1~7；夹爪 ID=8 无 Reboot 命令（旧版也没有）；建议保留** |
| `#CMDMODE`          | 存在                                 | `SetCommandMode()`                                        | 同                           | 支持新模式 COMMAND_SERVO_J/COMMAND_TORQUE_CONTROL（USB 回执加 "ok " 前缀，UART4 不变）                       | **重构** | **(agent) 保留新：扩模式到 6（SEQ/INT/TRJ/TUN/TRQ/SRV）是新功能需求；USB/UART4 回执差异是已知小问题，建议保留新功能**    |
| `#OFFSET_J`         | ❌                                  | —                                                         | 新增（USB:531；UART4:1284）      | 单轴归零标定 / 批量 `all`；**v3.2 修复**：删除主控端乐观同步                                                       | **新增** | **(agent) 保留新：单轴归零比全轴归零更灵活；v3.2 修复乐观同步避免与 0x23 回包竞争；建议保留**                            |
| `#ACC_J`            | ❌                                  | —                                                         | 新增（USB:581；UART4:1324）      | `SetAcceleration_persist(float, bool)`；支持 `&` 持久化标志；扩到 J7/地轨、夹爪；clamp 0~5000                  | **新增** | **(agent) 保留新：单位统一为 r/s²（v2.6 重构）后唯一入口；clamp 0~5000 防止非法值；建议保留**                      |
| `#ACC_BASE_J`       | 旧有（%）                              | `SetAcceleration(bases * %)`                              | ❌                           | **删除**（v2.6 单位统一后已废弃；UART4 中保留 "已删除"提示分支）                                                     | **删除** | **(agent) 保留删除：单位统一后旧版双层语义（基准 × 百分比）无意义；建议保留删除**                                      |
| `#ACC_RAIL`         | 旧有                                 | 单独地轨加速度                                                   | ❌                           | **删除**（走 `#ACC_J 7` 即可；UART4 中保留 "已删除"提示分支）                                                   | **删除** | **(agent) 保留删除：统一到** `#ACC_J 7 <r/s²> [&]`**；建议保留删除**                                 |
| `#SPEED_J`          | ❌                                  | —                                                         | 新增（USB:618；UART4:1366）      | 设置单轴速度限幅（仅 J1~J6，不含 7/8）                                                                      | **新增** | **(agent) 保留新：与** `#ACC_J` **对称；注意 7/8 不支持（旧版就没有）；建议保留**                              |
| `#I_LIMIT_J`        | ❌                                  | —                                                         | 新增（USB:634；UART4:1381）      | `SetCurrentLimit_persist(float, bool)`；扩到 J7/地轨、夹爪                                            | **新增** | **(agent) 保留新：与** `#ACC_J` **对称；建议保留**                                                |
| `#GETJACC`          | ❌                                  | —                                                         | 新增（USB:669）                 | 同步轮询 `jointAccRuntime[idx]`（30ms 轮询时间戳）                                                       | **新增** | **(agent) 保留新：30ms 超时保护 + 时间戳比对判断回包到达（避免 ISR 内 printf 阻塞 ASCII 任务 UART TX）；建议保留**     |
| `#GETI`             | ❌                                  | —                                                         | 新增（USB:721）                 | 同步轮询 `jointCurrentLimitRuntime[idx]`（30ms 轮询时间戳）                                              | **新增** | **(agent) 保留新：与** `#GETJACC` **对称；建议保留**                                              |
| `#SYNC_ACC`         | ❌                                  | —                                                         | 新增（USB:574；UART4:1112/1361） | 批量触发 8 个电机 `QueryAcceleration()`                                                              | **新增** | **(agent) 保留新：与** `#GETJACC` **配套（缓存先行 + 后续同步查询）；建议保留**                               |
| 默认 `#` 分支           | 回 `"ok"`                           | USB：转发到 `commandHandler.Push(_cmd)`；UART4：回 `"ok"`（保持原行为） | **重构**                      | **保留新（USB）：**透传未知 `#` 命令到队列可让命令线程接管，行为更灵活；**保留旧（UART4）：**简单回执；USB 与 UART4 行为不一致是已知小差异，建议二选一统一 |        |                                                                                       |


### 2.3 `>/&/@/$` 运动指令格式


| 维度         | 旧固件                  | 新固件                                                      | 标记     | new（手动标记）                                                                |
| ---------- | -------------------- | -------------------------------------------------------- | ------ | ------------------------------------------------------------------------ |
| `>/&` 参数数量 | 6 个关节 + speed（第 7 位） | **7 个关节 + 地轨 + speed**（>j1,j2,j3,j4,j5,j6,j7_rail,speed） | **重构** | **(agent) 保留新：地轨参与 MoveJ/ServoJ 是新增架构需求；建议保留**                           |
| `@` 参数数量   | 6 维坐标 + speed        | 6 维坐标 + speed（不变）                                        | —      | **(agent) 保留现状：6 维 FK 不含地轨是合理的（笛卡尔空间不包含地轨自由度）；建议保留**                     |
| `$` 力矩指令   | ❌                    | **新增**：`$c1,c2,c3,c4,c5,c6,c7_rail`（7 个 float）           | **新增** | **(agent) 保留新：力矩透传是 5kHz 实时环的新需求（**`COMMAND_TORQUE_CONTROL` **模式）；建议保留** |
| SERVO_J 格式 | ❌                    | **新增**：`>/& j1..j6,j7_rail`（无 speed，第 7 位是 j7）           | **新增** | **(agent) 保留新：SERVO_J 是 5kHz 全速下发模式（不节流），无 speed 是合理的（直接随动）；建议保留**       |
| 队列机制       | 无独立队列线程              | `CommandHandler` 队列（32 条 × 128 字节，旧版 16×64）              | **重构** | **(agent) 保留新：队列容量翻倍可缓冲更多指令；建议保留**                                       |
| 未使能拒绝      | 无保护                  | **新增**：运动指令检查 `IsEnabled()` + `IsStalled()`              | **新增** | **(agent) 保留新：未使能时拒绝运动指令避免电机乱跑；堵转期间拒绝避免死锁；建议保留**                         |


---

## 三、`DummyRobot` 类对比（`Robot/instances/dummy_robot.h` + `.cpp`）

### 3.1 架构变化


| 维度            | 旧固件                                               | 新固件                                                              | 标记     | new（手动标记）                                                                                                              |
| ------------- | ------------------------------------------------- | ---------------------------------------------------------------- | ------ | ---------------------------------------------------------------------------------------------------------------------- |
| motorJ[] 数组大小 | `CtrlStepMotor* motorJ[7]`（[0]=ALL, [1..6]=J1~J6） | `CtrlStepMotor* motorJ[8]`（[0]=ALL广播代理, [1..6]=J1~J6, [7]=J7/地轨） | **重构** | **(agent) 保留新：8 槽位是 7 轴 + 地轨的必要扩展；ALL 代理（nodeID=0）确保广播机制兼容；建议保留**                                                      |
| hand 类        | `DummyHand* hand`（ID=7，裸 CAN 发帧）                  | `StepHand* hand`（ID=8，继承 `CtrlStepMotor`）                        | **重构** | **(agent) 保留新：继承** `CtrlStepMotor` **让 hand 复用全部 CtrlStep 接口（如 SetEnable/QueryAcceleration）；ID=8 避开 J7/地轨的 ID=7；建议保留** |
| DH 参数         | `{0.109, 0.035, 0.146, 0.115, 0.052, 0.072}`      | `{0.165, 0.0, 0.170, 0.117, 0.0695, 0.113}`（新机械结构）               | **重构** | **(agent) 保留新：DH 参数是真实机械结构决定的，迁移需重新标定；建议保留**                                                                           |


### 3.2 新增常量（`dummy_robot.h`）


| 常量                            | 值                          | 定义行                   | 说明                                               | new（手动标记）                                                                                                          |
| ----------------------------- | -------------------------- | --------------------- | ------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------ |
| `ALL`                         | `0`                        | `dummy_robot.h:7`     | 广播代理索引（StdId=(0<<7)                              | cmd 触发所有电机）                                                                                                        |
| `SLIDER_TO_RPS`               | `0.30f`                    | `dummy_robot.h:13`    | slider 100 → 30 r/s                              | **(agent) 保留新：slider 物理换算系数；建议保留**                                                                                 |
| `AXIS_MAX_RPS[8]`             | `{0,30,30,30,30,30,30,30}` | `dummy_robot.h:18-22` | 各轴物理上限 r/s；[7]=J7/地轨                             | **(agent) 保留新：各轴物理上限防止失步/堵转；建议保留**                                                                                 |
| `MOTOR_REDUCTION[8]`          | `{1,50,50,50,50,50,30,1}`  | `dummy_robot.h:27-29` | 各轴减速比；[7]=地轨直连                                   | **(agent) 保留新：与电机实际减速比匹配；注意旧版 motorJ[1..6] 减速比是** `{30,30,30,24,30,50}`**（详见 §3 备注），新版统一改 50 是硬件升级；建议保留**          |
| `HAND_MAX_RPS`                | `20.0f`                    | `dummy_robot.h:31`    | 夹爪速度上限                                           | **(agent) 保留新：夹爪使用 35 电机 reduction=16，输出轴速度 = 20/16 ≈ 1.25 r/s；建议保留**                                              |
| `FALLBACK_JOINT_ACCELERATION` | `10.0f`                    | `dummy_robot.h:34`    | 电机加速度查询未回包时的兜底值 r/s²                             | **(agent) 保留新：兜底值防止** `jointAccRuntime[i]==0` **时 ComputeSyncSpeeds 算崩；建议保留**                                      |
| `RAIL_STEPS_PER_MM`           | `40960.0f`                 | `dummy_robot.h:144`   | 丝杆 1605 直连：200步×1024微步/5mm                       | **(agent) 保留新：地轨换算常数；丝杆 1605 直连，200 步/圈 × 1024 微步 / 5mm/圈 = 40960 步/mm；建议保留**                                      |
| `ALL_FINISH_FLAG`             | `0x7E`                     | `dummy_robot.h:183`   | jointsStateFlag 全置位 = bit1~6=1, bit0=0           | **(agent) 保留新：v3.2 修复** `IsMoving()` **短路条件所需；bit0（地轨位）恒为 0 是合理的（地轨不在** `UpdateJointAnglesCallback` **处理范围）；建议保留** |
| `REST_POSE`                   | `{0,-75,180,0,0,0}`        | `dummy_robot.h:220`   | J1~J6；旧为 `{0,-73,180,0,0,0}`                     | **(agent) 保留新：J2 从 -73° 改为 -75° 是机械结构调整；建议保留**                                                                     |
| `DEFAULT_JOINT_SPEED`         | `80`                       | `dummy_robot.h:222`   | slider 默认值；旧为 `30`                               | **(agent) 保留新：slider 默认值从 30 提升到 80（实际 r/s = 80 × SLIDER_TO_RPS = 24 r/s）；建议保留**                                   |
| `DEFAULT_JOINT_ACCELERATION`  | `150.0f`                   | `dummy_robot.h:229`   | r/s²；旧为双层语义 `DEFAULT_JOINT_ACCELERATION_BASES[]` | **(agent) 保留新：v2.6 加速度单位统一为 r/s²（替代旧版** `BASES × %` **双层语义）；建议保留**                                                 |


### 3.3 新增字段（`dummy_robot.h`）


| 字段                                 | 类型                    | 行号      | 说明                  | new（手动标记）                                                                                        |
| ---------------------------------- | --------------------- | ------- | ------------------- | ------------------------------------------------------------------------------------------------ |
| `currentRailPos`                   | `float`               | 145     | 地轨当前位置 mm           | **(agent) 保留新：地轨位置由 0x23 回包维护（**`can_protocol.cpp:179-184`**）；**`#GETJPOS` **直接读该字段；建议保留**       |
| `targetRailPos`                    | `float`               | 148     | 地轨目标位置 mm           | **(agent) 保留新：MoveJ/ServoJ 写入；SetStallMode 同步；建议保留**                                             |
| `targetRailCurrent`                | `float`               | 150     | 地轨目标电流 mA           | **(agent) 保留新：力矩透传时** `SetJointCurrents(c0, ...)` **写入；建议保留**                                    |
| `jointSpeedRps`                    | `float`               | 155     | 电机轴 r/s（slider 换算后） | **(agent) 保留新：v2.6 重构为电机轴 r/s（替代旧版 °/s 错误语义）；建议保留**                                              |
| `dynamicJointSpeeds7`              | `DynamicJointSpeeds7` | 160-163 | 7 轴速度 r/s（含地轨）      | **(agent) 保留新：7 轴速度规划输出；建议保留**                                                                   |
| `railSpeedRps`                     | `float`               | 176     | 地轨速度 r/s            | **(agent) 保留新：硬编码 30 r/s 非 MoveJ 路径（Homing/Resting/EmergencyStop）的兜底速度；建议保留**                    |
| `targetCurrents[6]`                | `float[6]`            | 178     | 力矩透传 6 轴目标电流        | **(agent) 保留新：5kHz 实时环 TORQUE_CONTROL 模式写入；建议保留**                                                |
| `motorDceKps[9]`                   | `int32_t[9]`          | 181     | DCE Kp 缓存           | **(agent) 保留新：索引 [0]=ALL空闲, [1..7]=J1~J7/地轨, [8]=夹爪；目前实际未使用（**`#GET_PID` **走 ISR 临时缓存），建议清理或启用** |
| `jointAccRuntime[9]`               | `float[9]`            | 233     | 加速度查询缓存             | **(agent) 保留新：**`#GETJACC` **同步查询所需；建议保留**                                                       |
| `jointAccLastUpdateMs[9]`          | `uint32_t[9]`         | 234     | 加速度缓存时间戳            | **(agent) 保留新：30ms 超时保护 + 时间戳比对判断回包到达；v3.0 修复 UART TX 竞争关键；建议保留**                                |
| `jointCurrentLimitRuntime[9]`      | `float[9]`            | 237     | 电流限幅缓存              | **(agent) 保留新：与** `jointAccRuntime` **对称；**`#GETI` **同步查询所需；建议保留**                               |
| `jointCurrentLimitLastUpdateMs[9]` | `uint32_t[9]`         | 238     | 电流限幅缓存时间戳           | **(agent) 保留新：与** `jointAccLastUpdateMs` **对称；建议保留**                                             |
| `isStalled`                        | `bool`                | 320     | 堵转拦截状态              | **(agent) 保留新：**`ParseCommand`**/**`!HOME`**/**`!RESET` **入口检查；建议保留**                            |
| `lastServoTime`                    | `uint32_t`            | 186     | ServoJ 上次时间 us      | **(agent) 保留新：ServoJ 计算 dt 所需；建议保留**                                                             |


### 3.4 新增/删除方法


| 方法                                            | 行为                                       | 标记                                                | new（手动标记）                                                                                              |
| --------------------------------------------- | ---------------------------------------- | ------------------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| `MoveJ(float... float _j7_mm, float _slider)` | 7 参数（含地轨）；内置梯形同步规划 `ComputeSyncSpeeds()` | **新增**                                            | **(agent) 保留新：7 参数 + 梯形同步规划是 v2.7 重构核心；地轨限位检查 + 臂关节逐轴检查；建议保留**                                         |
| `ServoJ(float... float _j7_mm)`               | 高频随动（含地轨）；无加减速规划                         | **新增**                                            | **(agent) 保留新：高频伺服随动（5kHz 全速下发）是新需求；建议保留**                                                             |
| `MoveL(...)`                                  | 笛卡尔直线（含地轨保持）；反解选最近解后调 MoveJ              | **重构**                                            | **(agent) 保留新：地轨保持（**`MoveJ(..., currentRailPos, ...)`**）是新增需求；最近解选取算法重写（**`minDist=1e9f` **初值）；建议保留** |
| `SyncAllMotorAcceleration()`                  | 批量发 CAN 0x2C 到 8 个电机                     | **新增**                                            | **(agent) 保留新：8 个电机批量触发查询；初始化时调用一次回填** `jointAccRuntime[]`**；建议保留**                                    |
| `SetStallMode()` / `SetStallMode(int)`        | 堵转时停止发指令 + 同步 targetRailPos              | **新增**                                            | **(agent) 保留新：堵转模式切换 + 同步 targetRailPos；建议保留（是否使用取决于是否启用堵转检测）**                                        |
| `BroadcastUnlock()`                           | 连续 3 次 CAN 0x5B 广播（间隔 100ms）             | **新增**                                            | **(agent) 保留新：与** `!STALL_UNLOCK` **配套；建议保留**                                                          |
| `QueryStallStatus()`                          | 查询 7 个电机 J1~J7/地轨的 en/lock 状态            | **新增**                                            | **(agent) 保留新：与** `!STALL_STATUS` **配套；建议保留**                                                          |
| `SetJointCurrents(float c0..c7)`              | 力矩透传：c0=地轨，c1~c6=关节，c7=夹爪                | **新增**                                            | **(agent) 保留新：**`$` **指令解析后调用；建议保留**                                                                   |
| `LoadConfig()` / `SaveConfig()`               | EEPROM 灯效配置读写                            | **新增**                                            | **(agent) 保留新：灯效配置持久化（EepromConfig + magic=0x12345679）；建议保留**                                          |
| `CalibrateHomeOffset()`                       | ❌                                        | **删除**（旧固件有；新固件由 `#OFFSET_J` 替代）                  | **(agent) 保留删除：**`#OFFSET_J` **提供更精准的单轴归零（**`dummy_robot.cpp` **行为验证后已移除旧实现）；建议保留删除**                  |
| `SetRGBEnable(bool)`                          | ❌                                        | **删除**（RGB 使能由 `rgbStateStart/Enable/Disable` 替代） | **(agent) 保留删除：RGB 使能由 3 个预设状态码管理（**`rgbStateStart/Enable/Disable`**）替代布尔开关，更灵活；建议保留删除**               |
| `GetRGBEnabled()`                             | ❌                                        | **删除**                                            | **(agent) 保留删除：与** `SetRGBEnable` **配套删除；建议保留删除**                                                      |


### 3.5 `CommandHandler` 变化


| 维度                  | 旧固件                                | 新固件                                                                            | 标记     | new（手动标记）                                                                                                              |
| ------------------- | ---------------------------------- | ------------------------------------------------------------------------------ | ------ | ---------------------------------------------------------------------------------------------------------------------- |
| FIFO 类型             | `osMessageQueueNew(16, 64)`        | `osMessageQueueNew(32, 128)`                                                   | **重构** | **(agent) 保留新：队列容量（32 vs 16）和条目大小（128 vs 64）翻倍，可缓冲更多指令；建议保留**                                                          |
| Push 参数类型           | `const std::string&`               | `const char*`                                                                  | **重构** | **(agent) 保留新：避免每次入队构造/析构** `std::string`**；C 字符串零拷贝；建议保留**                                                            |
| Pop 返回类型            | `std::string`                      | `const char*`                                                                  | **重构** | **(agent) 保留新：与 Push 对称；建议保留**                                                                                         |
| ParseCommand 参数类型   | `const std::string&`               | `const char*`                                                                  | **重构** | **(agent) 保留新：与 Push/Pop 一致；建议保留**                                                                                     |
| 消费线程 Pop nullptr 防御 | 无防御                                | `if (_cmd != nullptr) ParseCommand(_cmd)`（`main.cpp:138`）                      | **重构** | **(agent) 保留新：null 检查防止 Pop 超时返回 nullptr 后崩溃，是稳健性增强；建议保留**                                                             |
| 独立消费线程              | 无（任务名均为 `ThreadControlLoopUpdate`） | 无（任务名仍为 `ThreadControlLoopUpdate`，是 `ControlLoopUpdateTask` 的入口函数；**未新增独立线程**） | —      | **(agent) 同旧：**`ThreadControlLoopUpdate` **是两版同名任务入口函数；新版只在 Pop→Parse 路径加了 nullptr 防御，并未拆分独立线程；原文档"独立消费线程"行属描述误差**     |
| EmergencyStop 行为    | `MoveJ(currentJoints)`（伪急停）        | CAN 0x09 广播急停 + 置 `ALL_FINISH_FLAG` + 清 FIFO                                   | **重构** | **(agent) 保留新：CAN 0x09 广播抱闸（电机端** `SetBrake(true)`**）比伪急停（MoveJ currentJoints）响应快；置 ALL_FINISH_FLAG 短路 IsMoving；建议保留** |
| 堵转期间运动指令保护          | 无                                  | `ParseCommand` 入口检查 `isStalled` → 运动指令返回 ok 但不入队                               | **新增** | **(agent) 保留新：堵转期间丢弃运动指令（仍可放行** `!START/!DISABLE`**）避免电机状态紊乱；建议保留**                                                    |


### 3.6 `SetCommandMode` 变化


| 模式                                        | 旧固件                                                                             | 新固件                                                                  | 标记     | new（手动标记）                                                                                                 |
| ----------------------------------------- | ------------------------------------------------------------------------------- | -------------------------------------------------------------------- | ------ | --------------------------------------------------------------------------------------------------------- |
| `COMMAND_TARGET_POINT_SEQUENTIAL` (=1)    | `SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW)`                          | 不覆盖电机端加速度；由 `SyncAllMotorAcceleration()` 读 EEPROM 值                  | **重构** | **(agent) 保留新：v2.6 重构后 SEQ/INT 不再覆盖电机端加速度，避免与** `#ACC_J` **设置冲突；建议保留**                                    |
| `COMMAND_TARGET_POINT_INTERRUPTABLE` (=2) | 同上                                                                              | 同上                                                                   | **重构** | **(agent) 保留新：与 SEQ 对称；建议保留**                                                                             |
| `COMMAND_CONTINUES_TRAJECTORY` (=3)       | `SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_HIGH)` + `jointSpeedRatio=0.3` | 不覆盖加速度；`jointSpeedRatio` 不自动减半                                       | **重构** | **(agent) 保留新：取消** `jointSpeedRatio=0.3` **自动减半（slider × SLIDER_TO_RPS 由** `SetJointSpeed` **直接换算）；建议保留** |
| `COMMAND_MOTOR_TUNING` (=4)               | 同上                                                                              | 同上                                                                   | —      | **(agent) 保留现状：扫频模式不变**                                                                                   |
| `COMMAND_TORQUE_CONTROL` (=5)             | ❌                                                                               | **新增**：力矩透传模式，5kHz 实时环下发 `$` 指令                                      | **新增** | **(agent) 保留新：100Hz 下发电流（**`%50==0`**）；**`SetJointCurrents(c0..c7)` **写入；建议保留**                           |
| `COMMAND_SERVO_J` (=6)                    | ❌                                                                               | **新增**：高频伺服随动，`SetJointAcceleration(DEFAULT_JOINT_ACCELERATION=150)` | **新增** | **(agent) 保留新：5kHz 全速下发** `ServoJ`**；建议保留**                                                               |


### 3.7 `IsMoving()` 变化


| 维度  | 旧固件                            | 新固件                                                                       | 标记     | new（手动标记）                                                                                                                        |
| --- | ------------------------------ | ------------------------------------------------------------------------- | ------ | -------------------------------------------------------------------------------------------------------------------------------- |
| 旧实现 | `jointsStateFlag != 0b1111110` | —                                                                         | —      | —                                                                                                                                |
| 新实现 | —                              | `!isEnabled ? false : jointsStateFlag != ALL_FINISH_FLAG`（`main.cpp:208`） | **重构** | **(agent) 保留新：v3.2 短路** `!isEnabled ? false` **防止 SEQUENTIAL 队列被堵死（旧版失能后** `IsMoving()` **仍返回 true，导致下一条指令被 while 阻塞循环吞掉）；必须保留** |
| 变化  | bit0 地轨恒 1                     | 短路：失能时直接 false                                                            | **重构** | **(agent) 保留新：与 §3.4 中** `SetEnable` **改动配套（disable 时不再写** `targetJoints=currentJoints`**）；建议保留**                                |


---

## 四、`CtrlStepMotor` CAN 接口对比（`Robot/actuators/ctrl_step.h` + `.cpp`）

### 4.1 新增方法


| 方法                                             | StdId        | 功能                      | 行号                        | new（手动标记）                                                                        |
| ---------------------------------------------- | ------------ | ----------------------- | ------------------------- | -------------------------------------------------------------------------------- |
| `SetAngleWithMotorRps(float angle, float rps)` | —            | 角度+速度（用 `reduction` 换算） | `ctrl_step.cpp:74`        | **(agent) 保留新：v2.6 重命名（替代** `SetAngleWithVelocityLimit`**）；reduction 换算正确；建议保留** |
| `SetCurrentLimit_persist(float, bool)`         | `(nodeID<<7) | 0x12`                   | 按参数决定是否写 EEPROM           | `ctrl_step.cpp:101`                                                              |
| `SetAcceleration_persist(float, bool)`         | `(nodeID<<7) | 0x14`                   | 按参数决定是否写 EEPROM           | `ctrl_step.cpp:127`                                                              |
| `QueryCurrentLimit()`                          | `(nodeID<<7) | 0x2D`                   | 查询电流限幅                    | `ctrl_step.cpp:136`                                                              |
| `QueryAcceleration()`                          | `(nodeID<<7) | 0x2C`                   | 查询加速度                     | `ctrl_step.cpp:141`                                                              |
| `QueryDceKp()`                                 | `(nodeID<<7) | 0x28`                   | 查询 DCE Kp                 | `ctrl_step.cpp:211`                                                              |
| `QueryDceKv()`                                 | `(nodeID<<7) | 0x29`                   | 查询 DCE Kv                 | `ctrl_step.cpp:218`                                                              |
| `QueryDceKi()`                                 | `(nodeID<<7) | 0x2A`                   | 查询 DCE Ki                 | `ctrl_step.cpp:225`                                                              |
| `QueryDceKd()`                                 | `(nodeID<<7) | 0x2B`                   | 查询 DCE Kd                 | `ctrl_step.cpp:232`                                                              |
| `BroadcastUnlock()`                            | `0x5B`       | 广播 UNLOCKED             | `ctrl_step.cpp:238`       | **(agent) 保留新：与电机端 0x5B UNLOCKED 配套；建议保留（取决于是否启用堵转检测）**                          |
| `BroadcastEStop()`                             | `(nodeID<<7) | 0x09`                   | 广播急停（nodeID=0 → 0x09 ALL） | `ctrl_step.cpp:248`                                                              |
| `QueryStallStatus(uint8_t)`                    | `(nodeID<<7) | 0x5C`                   | 查询堵转 en/lock              | `ctrl_step.cpp:260`                                                              |


### 4.2 方法签名变化


| 方法                             | 旧签名                                                  | 新签名                                                     | 标记     | new（手动标记）                                                                 |
| ------------------------------ | ---------------------------------------------------- | ------------------------------------------------------- | ------ | ------------------------------------------------------------------------- |
| `SetPositionWithVelocityLimit` | `SetPositionWithVelocityLimit(float pos, float vel)` | **重命名**：`SetPositionWithMotorRps(float pos, float rps)` | **重构** | **(agent) 保留新：参数语义从** `vel`**（速度幅值，无明确单位）改为** `rps`**（电机轴 r/s），更明确；建议保留** |
| `SetAngleWithVelocityLimit`    | `SetAngleWithVelocityLimit(float angle, float vel)`  | **重命名**：`SetAngleWithMotorRps(float angle, float rps)`  | **重构** | **(agent) 保留新：同上语义；建议保留**                                                 |
| `SetEnable`                    | `state = _enable ? FINISH : STOP`（乐观同步）              | disable 时同步 `state = STOP`；enable 时**不清**（等 0x23 回包纠正）  | **重构** | **(agent) 保留新：F1 修复 + A3 修复配套，禁用乐观同步避免与 0x23 回包竞争；建议保留**                  |
| `DoCalibration`                | `DoCalibration()`                                    | ❌ **删除**                                                | **删除** | **(agent) 保留删除：编码器校准流程应在电机端本地完成；建议保留删除**                                  |


### 4.3 字段变化


| 字段              | 旧固件                     | 新固件                                         | 标记     | new（手动标记）                                                               |
| --------------- | ----------------------- | ------------------------------------------- | ------ | ----------------------------------------------------------------------- |
| `State` 枚举      | `RUNNING, FINISH, STOP` | **新增** `STALL`                              | **新增** | **(agent) 保留新：**`SetStallMode()` **写入** `state=STALL`**；建议保留**          |
| `targetAngle`   | 无                       | `float targetAngle`（MoveJ/ServoJ 写入，禁用时清 0） | **新增** | **(agent) 保留新：A3 修复后实际不再使用（依赖** `jointsStateFlag`**）；如长期不用建议清理以减少维护负担** |
| `AllAtTarget()` | 无                       | `bool AllAtTarget(float epsilon_deg)` 方法    | **新增** | **(agent) 保留新：**`!HAND_ZERO` **等待夹爪到位所需；建议保留**                          |


### 4.4 `SetEnable` 行为变化详解

> `ctrl_step.cpp:23-34`。

旧：`state = _enable ? FINISH : STOP`（乐观同步，enable 时立即写 FINISH）。

新：disable → `state = STOP`；enable → **不清 state**（依赖电机端首次 0x23 回包纠正）。这与 `dummy_robot.cpp` 的 `jointsStateFlag` 方案对齐：电机 STOP 时 `UpdateAngleCallback` 会将 `jointsStateFlag |= (1<<i)`，使 `IsMoving()` 返回 false。

---

## 五、FreeRTOS 任务架构对比（`UserApp/main.cpp`）


| 线程名                        | 优先级                  | 频率                | 旧栈大小  | 新栈大小  | 旧实现                                 | 新实现                                                                                                           | 标记     | new（手动标记）                                                                                   |
| -------------------------- | -------------------- | ----------------- | ----- | ----- | ----------------------------------- | ------------------------------------------------------------------------------------------------------------- | ------ | ------------------------------------------------------------------------------------------- |
| `ControlLoopFixUpdateTask` | `osPriorityRealtime` | 5kHz (TIM7 200μs) | 2000  | 2000  | 简单下发 MoveJ/UpdateJointAngles/Tuning | **总线节流**（50Hz 下发）+ 方案C单播轮询 + 新模式处理（ServoJ/Torque/Tuning）                                                      | **重构** | **(agent) 保留新：50Hz 节流避免 7×5kHz=35k fps 撞爆 1Mbps CAN 总线；新增 SRV/TRQ 模式覆盖；J7/地轨单播下发；建议保留**     |
| `ControlLoopUpdateTask`    | `osPriorityNormal`   | 事件触发              | 2000  | 2000  | 直接 `ParseCommand(Pop())`            | **重构**：拆分两行 + 增加 nullptr 防御（`_cmd != nullptr`，`main.cpp:136-139`）；任务名仍为 `ThreadControlLoopUpdate`，**未拆出独立线程** | **重构** | **(agent) 保留新：null 防御是稳健性增强（Pop 超时返回 nullptr 后不再崩溃）；建议保留；原文档"独立线程"描述不准确，需修正**               |
| `OledTask`                 | `osPriorityNormal`   | ~60Hz             | 2000  | 2000  | IMU 100Hz + OLED 显示                 | IMU 100Hz + **堵转事件 poll** + **PID 合并打印 poll** + **调试探针**（7 个心跳计数器 + 5x8 三行面板）                                 | **重构** | **(agent) 保留新：v3.0 修复 ISR 内 printf 阻塞 CAN RX 关键（F4 修复）；ISR 内只设标志 + 主线程 poll 是必须保留的设计；建议保留** |
| `RGBTask`                  | `osPriorityNormal`   | 33Hz              | 2000  | 2000  | `rgb.Run()` 阻塞等 DMA                 | **非阻塞**：`UpdateLoop()` 检查 DMA 空闲 → `Run()` → 跳过 BUSY 帧                                                        | **重构** | **(agent) 保留新：非阻塞机制避免 DMA 竞争；**`rgb.InitSync()` **初始化事件标志；建议保留**                            |
| `CommunicationTask`        | `osPriorityNormal`   | —                 | 45000 | 45000 | 相同                                  | 相同                                                                                                            | —      | **(agent) 保留现状：未变化**                                                                        |
| `UsbDeferredInterruptTask` | `osPriorityNormal`   | —                 | —     | —     | 相同                                  | 相同                                                                                                            | —      | **(agent) 保留现状：未变化**                                                                        |


