# 主控固件 — `firmware` → `firmware_new` 迁移决策

> **范围**：仅主控固件（STM32F405，FreeRTOS）。
> **基线对比**：
>   - **来源（参考功能集）**：`firmware/ref_core_f405/` — 已实现的"新版本"，含 7 轴、地轨、夹爪、5 kHz 实时环、灯效等所有 v3.x 功能
>   - **目标（迁移产物）**：`firmware_new/dummy-ref-core-fw/` — 起始为 `firmware_old` 副本，待按本决策增量实现
>   - **基线参考**：`firmware_old/dummy-ref-core-fw/` — 旧版本（6 轴，无地轨/夹爪/灯效），用于明确"退回旧"决策
> **架构差异**：旧固件 6 轴架构（`motorJ[0..6]`，`DummyHand` ID=7）；`firmware` 为 **7 轴 + 地轨 + 夹爪** 架构（`motorJ[0..7]`，`StepHand` ID=8）
> **数据来源**：直接读源码，每条标注 `文件:行号`。**文档聚焦"差异"**，未变化的部分不列。
> **创建日期**：2026-09-23（v0.1 开头重构）

> ### ⚠️ 与电机固件迁移策略的差异
>
> | 维度 | 电机固件（[`firmware_new电机固件决策.md`](./firmware_new电机固件决策.md)） | 主控固件（本文档） |
> |---|---|---|
> | 核心原则 | 明确"不实现 stall v3"，**绝大多数功能"退回旧"** | **逐项决策**：每个 v3.x 新功能评估"保留新 / 退回旧 / 删除" |
> | 决策粒度 | 总体策略 + 个别例外 | 每个新功能一项决策 |
> | 决策范围 | CAN 协议 / 控制算法 / EEPROM | CAN 协议 / ASCII 命令 / 实时环 / 7 轴架构 / 夹爪 / 灯效 / 力矩透传 / 堵转通路 |
>
> **原因**：主控固件最近几个月加入了大量新功能（7 轴架构、夹爪、5 kHz 实时环、灯效配置、堵转协议上层通路、SERVO_J/TORQUE 模式等），无法像电机固件那样"大段删除"。本文档采用**逐项决策**模式：每个新功能给出建议（✅/⏳）+ 理由 + 风险点，**由用户最终拍板**。

---

## 0. 核心迁移原则

### 0.1 已确定的决策（不可改动）

| # | 决策 | 原因 | 对应章节 |
|---|---|---|---|
| 1 | **不实现 stall v3 检测算法** | 沿用电机固件决策：电机端已删除 LOCKED/RETREATING 状态机；主控侧只保留上层命令通路，不实现具体算法 | §一.1（0x5A/0x5B/0x5C）、§二.1、§三.4 |
| 2 | **保留 CAN 0x09 急停广播** | 安全急停功能不依赖 stall 检测，必须保留；替代旧版"伪急停"（MoveJ currentJoints） | §一.1 `0x09`、§二.1 `!STOP` |
| 3 | **`!STOP` 走 CAN 0x09 急停** | 比旧版"伪急停"（MoveJ currentJoints）更安全；`firmware` 已实现 | §二.1 `!STOP` |
| 4 | **CAN 发送层防卡死重写** | F4 修复关键：消除 mailbox 满 + 无限等 + HAL 失败不释放三重死锁风险（`interface_can.cpp:248-263`） | §一.3 |
| 5 | **保留"堵转协议上层通路"** | 不实现检测算法，但保留 `!STALL_*` ASCII 命令 + `0x5A/0x5B/0x5C` CAN 命令通路 + `isStalled` 拦截开关；启用与否取决于电机端是否启用堵转检测 | §一.1、§二.1、§三.3、§三.4 |
| 6 | **`0x01` SetEnable 不写 FINISH + `IsMoving()` 短路 isEnabled** | v3.2 修复关键 bug：避免 SEQUENTIAL 队列被 `while(!IsMoving())` 阻塞吞指令；`disable` 后队列可正常推进 | §四.3、§四.4 |
| 7 | **电机端 `ApplyPosAsHomeOffset()` 同步 `realPosition`** | Bug 修复：让 `GetPosition()` 立即归零（旧固件归零后 GetPosition 不归零）。详见电机固件决策文档 | （电机端） |

### 0.2 删除决策（已确认从 `firmware_new` 中移除）

| # | 删除项 | 替代方案 | 对应章节 |
|---|---|---|---|
| D1 | `!CALIBRATION` ASCII 命令（`CalibrateHomeOffset()` 方法） | `#OFFSET_J` 单轴归零更精准 | §二.1、§三.4 |
| D2 | `#ACC_BASE_J` / `#ACC_RAIL` 双层语义命令 | v2.6 单位统一为 r/s² 后无意义；走 `#ACC_J 7 <r/s²> [&]` | §二.2 |
| D3 | `SetRGBEnable(bool)` / `GetRGBEnabled()` 旧 RGB 开关 | 由 `rgbStateStart/Enable/Disable` 3 状态码替代，更灵活 | §三.4 |
| D4 | `0x02 DoCalibration()` 主控驱动方法 | 校准应在电机端本地完成，不通过主控驱动 | §一.1、§四.2 |
| D5 | `SetPositionWithVelocityLimit` 旧名 | 重命名为 `SetPositionWithMotorRps`（参数语义 rps 更精确） | §一.1、§四.2 |

### 0.3 待决策项（按功能分组）

> **决策流程**：每项给出"保留新 / 退回旧 / 删除"三选一建议 + 理由 + 风险点；agent 在后续详细章节给出具体行号分析，**用户最终拍板**。
>
> **状态标识**：✅ 建议保留 / ⏳ 待决策 / ❌ 建议删除

#### A. 架构层（详见 §三）

| # | 功能点 | 旧固件来源 | `firmware` 实现 | 建议 | 状态 |
|---|---|---|---|---|---|
| A1 | 7 轴架构 + `motorJ[8]` | `motorJ[7]`（6轴） | `motorJ[8]`（7轴+地轨） | 保留新（7 轴 + 地轨是产品形态） | ✅ |
| A2 | 地轨（J7/rail）集成 | 无 | `motorJ[7]` + 丝杆换算 | 保留新 | ✅ |
| A3 | `StepHand` 继承 `CtrlStepMotor` | `DummyHand` 裸 CAN 发帧 | 继承 CtrlStepMotor | 保留新（接口统一） | ✅ |
| A4 | DH 参数（新机械结构） | 旧 DH | 新 DH | 保留新（机械结构决定） | ✅ |
| A5 | 新增常量（`ALL` / `SLIDER_TO_RPS` / `AXIS_MAX_RPS` / `MOTOR_REDUCTION` / `HAND_MAX_RPS` / `RAIL_STEPS_PER_MM` 等 11 项） | 旧固件无 | 新固件定义 | 待细化 | ⏳ |

#### B. 实时控制层（详见 §四）

| # | 功能点 | 旧固件来源 | `firmware` 实现 | 建议 | 状态 |
|---|---|---|---|---|---|
| B1 | 5 kHz 实时环（TIM7） | 无 TIM7 | TIM7 200 μs 中断（`ControlLoopFixUpdateTask`） | 待决策（高频环是 SERVO_J/TORQUE 的基础） | ⏳ |
| B2 | COMMAND_SERVO_J 模式 | 无 | 5 kHz 高频随动（无加减速规划） | 待决策（与 §C9 配套） | ⏳ |
| B3 | COMMAND_TORQUE_CONTROL 模式 | 无 | 100 Hz 力矩透传 | 待决策（与 §E5 配套） | ⏳ |
| B4 | 50 Hz 降频下发（SEQ/INT/TRJ） | 5 kHz 全速下发 | 50 Hz 降频下发避免总线拥堵 | 保留新 | ✅ |
| B5 | `ComputeSyncSpeeds` 梯形同步规划 | 无 | `dummy_robot.cpp` 中 | 待决策（与 B1 配套） | ⏳ |
| B6 | `currentPose6D` 预计算缓存 | 通信线程内 FK | 5 kHz 控制环维护缓存 | 待决策 | ⏳ |
| B7 | `#GETLPOS` 改读缓存 | 通信线程触发 FK | 直接读 `currentPose6D` | 保留新（避免通信线程做 FK 与 ASCII 任务竞争） | ✅ |

#### C. ASCII 命令层（详见 §二）

| # | 功能点 | 旧固件来源 | `firmware` 实现 | 建议 | 状态 |
|---|---|---|---|---|---|
| C1 | `!RGB_SET_START/ENABLE/DISABLE/BRIGHT/MODE/COLOR` | 无 | 6 条 RGB 配置（含 EEPROM 持久化） | 待决策（灯效配置是否纳入 `firmware_new`） | ⏳ |
| C2 | `!HAND_ZERO/EN/DIS/POS/O/C` | 无 | 6 条夹爪控制 | 待决策（夹爪是否纳入 `firmware_new`） | ⏳ |
| C3 | `!STALL_EN/DIS/STATUS/UNLOCK` | 无 | 4 条堵转控制 | 保留（与决策 §0.1 #5 一致） | ✅ |
| C4 | `!PRINTPOSE` | 无 | 一键打印 `GETJPOS + GETLPOS` | 待决策 | ⏳ |
| C5 | `#OFFSET_J` 单轴归零 | 无 | 单轴归零（替代 `!CALIBRATION`） | 待决策（单轴归零更灵活） | ⏳ |
| C6 | `#ACC_J` / `#SPEED_J` / `#I_LIMIT_J` 单轴配置 | 无 | 单轴配置（v2.6 单位统一） | 待决策（v2.6 单位统一是改进） | ⏳ |
| C7 | `#GET_PID` / `#GETJACC` / `#GETI` 批量查询 | 无 | 批量查询 + ISR poll + 合并打印 | 待决策（v3.0 UART TX 竞争修复） | ⏳ |
| C8 | `#REBOOT` 无参版（ALL 代理） | 仅带参版 | 无参 → ALL 广播 + 主控 reset | 待决策 | ⏳ |
| C9 | `#CMDMODE` SERVO_J/TORQUE | 不支持 | 扩展到 6 模式（SEQ/INT/TRJ/TUN/TRQ/SRV） | 待决策（与 B2/B3 配套） | ⏳ |
| C10 | USB/UART4 默认 `#` 分支不一致 | 回 "ok" | USB 转发队列；UART4 回 "ok" | 待决策（建议二选一统一） | ⏳ |
| C11 | `!START` / `!DISABLE` / `!HOME` / `!RESET` 解 isStalled + 堵转拒绝 | 简单调用 | 加 isStalled 检查 | 保留新（堵转期间拒绝避免死锁） | ✅ |

#### D. CAN 协议层（详见 §一）

| # | 功能点 | 旧固件来源 | `firmware` 实现 | 建议 | 状态 |
|---|---|---|---|---|---|
| D1 | 0x07 重命名 `SetPositionWithMotorRps` | `SetPositionWithVelocityLimit` | 参数名 rps | 保留新（参数名更精确） | ✅ |
| D2 | 0x12/0x14 可选持久化（`_persist` 方法） | 固定写 EEPROM | 旧方法 + 新 `_persist` 方法 | 保留新（避免测试频繁写 EEPROM；旧方法保留兼容） | ✅ |
| D3 | 0x23 单播轮询 | 广播触发所有电机回包 | 单播轮询 14 Hz/电机 | 保留新（避免总线冲突；jitter ≤70 ms 足够 `IsMoving()`） | ✅ |
| D4 | 0x28~0x2D PID/Acc/Cur 查询 | 无 | ISR poll + 合并打印 | 待决策（与 C7 配套） | ⏳ |
| D5 | 0x5A/0x5B/0x5C 堵转协议通路 | 无 | ISR 设标志 + 主线程 poll | 保留（与决策 §0.1 #5 一致） | ✅ |
| D6 | 0x7C 旧堵转兼容 | 无 | 兼容旧固件电机端 0x7C | 待决策（兼容成本低） | ⏳ |
| D7 | 0x09 急停广播 | 无 | ALL 代理广播 | 保留（与决策 §0.1 #2 一致） | ✅ |
| D8 | `UpdateAngle` 100 Hz / 7 轴 ≈ 14 Hz/电机 | 5 kHz 实时环每帧触发 | 100 Hz 节流 | 保留新（与 D3 配套） | ✅ |

#### E. 工具方法（详见 §三.4、§四）

| # | 功能点 | 旧固件来源 | `firmware` 实现 | 建议 | 状态 |
|---|---|---|---|---|---|
| E1 | `MoveJ` 7 参数 + 梯形同步规划 | 6 参数 | 7 参数 + `ComputeSyncSpeeds` | 保留新（7 轴必要） | ✅ |
| E2 | `ServoJ` 高频随动 | 无 | 5 kHz 全速下发 | 待决策（与 B2 配套） | ⏳ |
| E3 | `MoveL` 地轨保持 + 最近解选取 | 无 | 笛卡尔直线 + `currentRailPos` 保持 | 待决策 | ⏳ |
| E4 | `SetStallMode` / `BroadcastUnlock` / `QueryStallStatus` | 无 | 堵转协议上层方法 | 保留（与决策 §0.1 #5 一致） | ✅ |
| E5 | `SetJointCurrents` 力矩透传 | 无 | 7 电流 float 透传 | 待决策（与 B3 配套） | ⏳ |
| E6 | `LoadConfig` / `SaveConfig`（EEPROM 灯效配置） | 无 | EepromConfig + magic=0x12345679 | 待决策（与 C1 配套） | ⏳ |
| E7 | `CommandHandler` 队列扩到 32×128 | 16×64 | 32×128 | 保留新（队列容量翻倍） | ✅ |
| E8 | `SyncAllMotorAcceleration` 批量查询 | 无 | 批量发 0x2C 到 8 个电机 | 待决策（与 C7 配套） | ⏳ |

### 0.4 当前进度统计（v0.1）

| 状态 | 数量 | 说明 |
|---|---|---|
| ✅ 已决策（保留新） | 14 | 详见 §0.3 各分组的 ✅ 标记项 |
| ❌ 已决策（删除） | 5 | 详见 §0.2 表格 |
| ⏳ 待决策 | ~22 | 详见 §0.3 各分组的 ⏳ 标记项 |
| **合计** | **~41** | 涵盖架构 / 实时环 / ASCII / CAN / 工具方法 5 大类 |

---

## 一、CAN 协议对比（`UserApp/protocols/can_protocol.cpp`）

> 主控固件不解析电机端 cmd 含义（电机端语义见电机固件对比文档），只关注"主控侧如何构造请求帧"和"如何处理回包"。

### 1.1 电机端 cmd 构造方式

> 参考电机固件对比文档格式，`名称` 并入 `Cmd` 列（括号注释）。以下"旧固件 / 新固件"列均先写**主控侧发送行为**，后以 `/` 分隔**回包处理行为**；无回包时写 `/ —`；无发送时写 `— /`。


| Cmd（名称）           | 方向    | 旧固件（发送 / 回包）                                                                          | 新固件（发送 / 回包）                                                                                                                              | 标记                                                                                                                                                                                                   | new（手动标记）                                                                                                                                                                      |
| ----------------- | ----- | ------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `0x01`（使能/失能）     | 主→电机  | `SetEnable(bool)` → `state = _enable ? FINISH : STOP`；发 `(nodeID<<7)                  | 0x01` + val(0/1) 4B                                                                                                                       | `SetEnable(bool)`：enable 时**不再写** `state = FINISH`（等待 0x23 回包纠正）；disable 时显式写 `state = STOP`；其余同旧                                                                                                    | 同旧                                                                                                                                                                             |
| `0x02`（编码器校准）     | 主→电机  | `DoCalibration()` → 发 `(nodeID<<7)                                                    | 0x02`（无 payload）/ —                                                                                                                       | ❌ 旧方法已删除；新固件无此方法                                                                                                                                                                                     | **删除**                                                                                                                                                                         |
| `0x03`（力矩透传）      | 主→电机  | `SetCurrentSetPoint(float A)` → `state = RUNNING`；发 0x03 + float 4B / —               | 同旧：`state = RUNNING`；发 0x03 + float 4B / —                                                                                                | —                                                                                                                                                                                                    | **保留现状：无变化**                                                                                                                                                                   |
| `0x04`（速度模式）      | 主→电机  | `SetVelocitySetPoint(float r/s)` → `state = RUNNING`；发 0x04 + float 4B / —            | 同旧：发 0x04 + float 4B / —                                                                                                                  | —                                                                                                                                                                                                    | **保留现状：无变化**                                                                                                                                                                   |
| `0x05`（位置模式（简单））  | 主→电机  | `SetPositionSetPoint(float 圈)` → 发 0x05 + float 4B + `canBuf[4]=1`（需 ACK）/ —          | 同旧：发 0x05 + float 4B + `canBuf[4]=1` / —                                                                                                  | —                                                                                                                                                                                                    | **保留现状：无变化**                                                                                                                                                                   |
| `0x07`（位置+速度限幅）   | 主→电机  | `SetPositionWithVelocityLimit(float 圈, float vel)` → 发 0x07 + pos 4B + vel 4B / —     | **重命名**：`SetPositionWithMotorRps(float 圈, float rps)`；参数语义改为 rps；其余帧格式不变 / —                                                              | **重构**                                                                                                                                                                                               | **(agent) 保留新：参数名** `rps` **比** `vel` **更精确；建议保留新名**                                                                                                                           |
| `0x09`（急停广播）      | 主→ALL | ❌ 不存在                                                                                 | `BroadcastEStop()` → 发 `(nodeID<<7)                                                                                                       | 0x09`（ALL 代理` nodeID=0` → StdId=0x09）；payload 空 / —                                                                                                                                                  | **新增**                                                                                                                                                                         |
| `0x11`（设置节点 ID）   | 主→电机  | `SetNodeID(uint32_t)` → 发 0x11 + id 4B + `canBuf[4]=1`（固定写 EEPROM）/ —                 | 同旧：发 0x11 + id 4B + `canBuf[4]=1` / —                                                                                                     | —                                                                                                                                                                                                    | **(agent) 保留现状：无变化**                                                                                                                                                           |
| `0x12`（设置电流限幅）    | 主→电机  | `SetCurrentLimit(float A)` → 发 0x12 + float 4B + `canBuf[4]=1`（**固定写 EEPROM**）/ —     | `SetCurrentLimit_persist(float A, bool persist)`：按 persist 参数决定 `canBuf[4]=0/1`；**同时保留旧方法** `SetCurrentLimit`（固定写 EEPROM，供 fiber 协议调用）/ — | **重构**                                                                                                                                                                                               | **(agent) 保留新：可选持久化避免测试时频繁写 EEPROM；旧方法保留兼容；建议保留**                                                                                                                              |
| `0x13`（设置速度限幅）    | 主→电机  | `SetVelocityLimit(float r/s)` → 发 0x13 + float 4B + `canBuf[4]=1` / —                 | 同旧：发 0x13 + float 4B + `canBuf[4]=1` / —                                                                                                  | —                                                                                                                                                                                                    | **(agent) 保留现状：无变化**                                                                                                                                                           |
| `0x14`（设置加速度）     | 主→电机  | `SetAcceleration(float r/s²)` → 发 0x14 + float 4B + `canBuf[4]=0`（**默认不写 EEPROM**）/ — | `SetAcceleration_persist(float r/s², bool persist)`：按 persist 参数决定 `canBuf[4]=0/1`；**同时保留旧方法** `SetAcceleration`（默认不写 EEPROM）/ —          | **重构**                                                                                                                                                                                               | **(agent) 保留新：可选持久化与 0x12 对称；建议保留**                                                                                                                                            |
| `0x15`（归零偏移）      | 主→电机  | `ApplyPositionAsHome()` → 发 0x15（无 payload）/ —                                        | 同旧：发 0x15（无 payload）/ —；注意：电机端旧固件 `GetPosition()` 归零后不归零，需关注电机端修复                                                                         | —                                                                                                                                                                                                    | **(agent) 保留现状：主控侧方法不变；电机端 0x15 可能有 Bug，见电机固件对比文档**                                                                                                                            |
| `0x16`（上电自启）      | 主→电机  | `SetEnableOnBoot(bool)` → 发 0x16 + val(0/1) 4B + `canBuf[4]=1` / —                    | 同旧：发 0x16 + val(0/1) 4B + `canBuf[4]=1` / —                                                                                               | —                                                                                                                                                                                                    | **(agent) 保留现状：无变化**                                                                                                                                                           |
| `0x17`（设置 DCE Kp） | 主→电机  | `SetDceKp(int32_t)` → 发 0x17 + int32 4B + `canBuf[4]=1` / —                           | 同旧：发 0x17 + int32 4B + `canBuf[4]=1` / —                                                                                                  | —                                                                                                                                                                                                    | **(agent) 保留现状：无变化**                                                                                                                                                           |
| `0x18`（设置 DCE Kv） | 主→电机  | `SetDceKv(int32_t)` → 发 0x18 + int32 4B + `canBuf[4]=1` / —                           | 同旧：发 0x18 + int32 4B + `canBuf[4]=1` / —                                                                                                  | —                                                                                                                                                                                                    | **(agent) 保留现状：无变化**                                                                                                                                                           |
| `0x19`（设置 DCE Ki） | 主→电机  | `SetDceKi(int32_t)` → 发 0x19 + int32 4B + `canBuf[4]=1` / —                           | 同旧：发 0x19 + int32 4B + `canBuf[4]=1` / —                                                                                                  | —                                                                                                                                                                                                    | **(agent) 保留现状：无变化**                                                                                                                                                           |
| `0x1A`（设置 DCE Kd） | 主→电机  | `SetDceKd(int32_t)` → 发 0x1A + int32 4B + `canBuf[4]=1` / —                           | 同旧：发 0x1A + int32 4B + `canBuf[4]=1` / —                                                                                                  | —                                                                                                                                                                                                    | **(agent) 保留现状：无变化**                                                                                                                                                           |
| `0x1B`（堵转保护开关）    | 主→电机  | `SetEnableStallProtect(bool)` → 发 0x1B + val(0/1) 4B + `canBuf[4]=1` / —              | 同旧：发 0x1B + val(0/1) 4B + `canBuf[4]=1` / —                                                                                               | —                                                                                                                                                                                                    | **(agent) 保留现状：无变化；但是否启用堵转检测取决于电机端固件决定**                                                                                                                                       |
| `0x21`（查询电流）      | 主→电机  | ❌ 主控侧未实现 `GetCurrent()` 方法（`ctrl_step.cpp/h` 中无此方法） / —                               | ❌ 同旧                                                                                                                                      | **缺失**                                                                                                                                                                                               | **(agent) 保留现状：主控从未实现 0x21 查询；电机端有** `0x21` **Get Current 命令（firmware_old/dummy-42motor-fw/.../interface_can.cpp:183）；如需此功能，应新增** `GetCurrent(float)` **方法发 0x21 + memcpy 回包** |
| `0x23`（查询位置）      | 主→电机  | `UpdateAngle()` → 发 `(nodeID<<7)                                                      | 0x23`（nodeID=0 广播） /` case 0x23: dummy.motorJ[id]->UpdateAngleCallback(*(float*)(data), data[4])`；`motorJ[ALL]->UpdateAngle()` 广播触发所有电机回包 | `UpdateAngle()`：改为**单播**轮询（currentIndex 在 1~7 循环）/ **重构**：新固件分三段处理——id=7 → 更新 `motorJ[7]->angle` + 同步 `currentRailPos`；id=1~6 → `motorJ[id]->UpdateAngleCallback`；id=8 → `hand->UpdateAngleCallback` | **重构**                                                                                                                                                                         |
| `0x25`（查询温度）      | 主→电机  | `GetTemp()` → 发 `(nodeID<<7)                                                          | 0x25`/`memcpy(&motorJ[id]->temperature, data, 4)`                                                                                         | 同旧：发 0x25 / `memcpy(&motorJ[id]->temperature, data, 4)`；新固件对 id=1~6/7/8 分别处理（代码对称）                                                                                                                   | —                                                                                                                                                                              |
| `0x28`（查询 DCE Kp） | 主→电机  | ❌ 不存在 / —                                                                             | `QueryDceKp()` → 发 `(nodeID<<7)                                                                                                           | 0x28`（payload 空）/ **新增**：ISR 内累加` _pid_rcv_cnt[idx]`，4次回包到齐后设` _pid_print_pending[idx]=1`，主线程` PollPidPrint()` 读缓存并 printf                                                                           | **新增**                                                                                                                                                                         |
| `0x29`（查询 DCE Kv） | 主→电机  | ❌ 不存在 / —                                                                             | `QueryDceKv()` → 发 `(nodeID<<7)                                                                                                           | 0x29` / 同上（4次回包合并为 1 次 printf）                                                                                                                                                                       | **新增**                                                                                                                                                                         |
| `0x2A`（查询 DCE Ki） | 主→电机  | ❌ 不存在 / —                                                                             | `QueryDceKi()` → 发 `(nodeID<<7)                                                                                                           | 0x2A` / 同上                                                                                                                                                                                           | **新增**                                                                                                                                                                         |
| `0x2B`（查询 DCE Kd） | 主→电机  | ❌ 不存在 / —                                                                             | `QueryDceKd()` → 发 `(nodeID<<7)                                                                                                           | 0x2B` / 同上                                                                                                                                                                                           | **新增**                                                                                                                                                                         |
| `0x2C`（查询加速度）     | 主→电机  | ❌ 不存在 / —                                                                             | `QueryAcceleration()` → 发 `(nodeID<<7)                                                                                                    | 0x2C `/ **新增**：`*(float*)data`缓存到`jointAccRuntime[idx]`+`jointAccLastUpdateMs[idx]=HAL_GetTick()`                                                                                                    | **新增**                                                                                                                                                                         |
| `0x2D`（查询电流限幅）    | 主→电机  | ❌ 不存在 / —                                                                             | `QueryCurrentLimit()` → 发 `(nodeID<<7)                                                                                                    | 0x2D `/ **新增**：`*(float*)data`缓存到`jointCurrentLimitRuntime[idx]` + 时间戳                                                                                                                               | **新增**                                                                                                                                                                         |
| `0x5A`（堵转事件上报）    | 电机→主  | — / ❌ 不存在（无处理）                                                                        | — / **新增**：收到 0x5A → ISR 内设 `_stall_event_flag=1/_node/_cmd`，主线程(OledTask) poll 后 printf                                                  | **新增**                                                                                                                                                                                               | **(agent) 保留新：ISR 只设标志 + 主线程 poll 模式避免 ISR 内 printf 阻塞 CAN RX；是否使用取决于电机端是否启用堵转检测**                                                                                             |
| `0x5B`（全局解锁广播）    | 主→ALL | ❌ 不存在                                                                                 | `BroadcastUnlock()` → 发 `StdId=0x5B`（ALL 广播，payload 空，电机端收到→IDLE + ResetGoals）/ —                                                         | **新增**                                                                                                                                                                                               | **(agent) 保留新：与 0x5A 配套**                                                                                                                                                      |
| `0x5C`（堵转状态查询）    | 主→电机  | ❌ 不存在 / —                                                                             | `QueryStallStatus(uint8_t queryType)` → 发 `(nodeID<<7)                                                                                    | 0x5C`+ queryType 1B / **新增**：ISR 内设`_stall_status_flag/_node/_type/_val`，主线程 poll 后 printf                                                                                                           | **新增**                                                                                                                                                                         |
| `0x7C`（旧堵转上报，兼容）  | 电机→主  | — / ❌ 不存在                                                                             | — / **新增**：收到 0x7C → `if(data[1]==1) dummy.SetStallMode(id)`（兼容旧固件电机）                                                                     | **新增**                                                                                                                                                                                               | **(agent) 保留新：兼容旧固件电机端 0x7C；建议保留**                                                                                                                                             |
| `0x7D`（温度监控）      | 主→电机  | `SetEnableTemp(bool)` → 发 `(nodeID<<7)                                                | 0x7D` + val(0/1) 4B / —                                                                                                                   | 同旧：发 0x7D + val(0/1) 4B / —                                                                                                                                                                          | —                                                                                                                                                                              |
| `0x7E`（擦除配置）      | 主→电机  | `EraseConfigs()` → 发 `(nodeID<<7)                                                     | 0x7E`（无 payload）/ —                                                                                                                       | 同旧：发 0x7E（无 payload）/ —                                                                                                                                                                              | —                                                                                                                                                                              |
| `0x7F`（重启电机）      | 主→电机  | `Reboot()` → 发 `(nodeID<<7)                                                           | 0x7F`（无 payload）/ —                                                                                                                       | 同旧：发 0x7F（无 payload）/ —                                                                                                                                                                              | —                                                                                                                                                                              |


### 1.2 `UpdateAngle()` 轮询策略变化

> `can_protocol.cpp:UpdateAngle()` 调用方式变化。


| 维度   | 旧固件                                       | 新固件                                                                                                         | 标记     | new（手动标记）                                                                 |
| ---- | ----------------------------------------- | ----------------------------------------------------------------------------------------------------------- | ------ | ------------------------------------------------------------------------- |
| 触发方式 | `motorJ[ALL]->UpdateAngle()` 广播（1帧触发所有电机） | **单播轮询**：`dummy.motorJ[currentIndex]->UpdateAngle()`，currentIndex 在 1~7 循环                                  | **重构** | **(agent) 保留新：单播避免总线冲突；7轴时同时 7 节点回包可能撞 mailbox；建议保留**                     |
| 频率   | 实时环内每帧（5kHz）触发                            | 100Hz / 7轴 ≈ 14Hz/电机（`updateCounter % 50 == 25`）                                                            | **重构** | **(agent) 保留新：14Hz/电机 足够** `IsMoving()` **状态判定（jitter ≤70ms）；建议保留**       |
| 地轨处理 | 无地轨                                       | J7/地轨（nodeID=7）回包 `0x23` 时 → 更新 `motorJ[7]->angle` + 同步 `currentRailPos = motorJ[7]->angle / 360.0f * 5.0f` | **新增** | **(agent) 保留新：地轨位置实时维护于** `currentRailPos`**；`**#GETJPOS` **直接读该缓存；建议保留** |


### 1.3 CAN 发送层防卡死（`Bsp/communication/interface_can.cpp`）

> `CanSendMessage()` 发送函数变化。


| 维度   | 旧固件                                                  | 新固件                                                                  | 标记     | new（手动标记）                                                            |
| ---- | ---------------------------------------------------- | -------------------------------------------------------------------- | ------ | -------------------------------------------------------------------- |
| 发送等待 | `osSemaphoreAcquire(sem_can1_tx, osWaitForever)` 无限等 | **新增**：mailbox 空闲预检（最多 5ms）+ 10ms 信号量超时（`interface_can.cpp:248/250`） | **重构** | **(agent) 保留新：彻底消除三重死锁风险（mailbox 满 + 无限等 + HAL 失败不释放）；F4 修复关键，必须保留** |
| 失败补偿 | 无                                                    | HAL 失败时释放信号量，防止死锁（`interface_can.cpp:255-263`）                       | **新增** | **(agent) 保留新：失败释放信号量避免计数器失衡；必须保留**                                  |


---

## 二、ASCII 命令协议对比（`UserApp/protocols/ascii_protocol.cpp`）

> 新固件 USB/UART4 双通道完整实现，UART5 空壳。下表按 `!` / `#` / `>/&/@/$` 分类。

### 2.1 `!` 系统控制命令


| 命令                 | 旧 ASCII 行号            | 旧实现                     | 新 ASCII 行号                               | 新实现                                                        | 标记     | new（手动标记）                                                                                                |
| ------------------ | --------------------- | ----------------------- | ---------------------------------------- | ---------------------------------------------------------- | ------ | -------------------------------------------------------------------------------------------------------- |
| `!STOP`            | 旧: `find("STOP")`     | `EmergencyStop()`       | 新: `s == "!STOP"`                        | `EmergencyStop()`（走 CAN 0x09 急停）                           | **重构** | **(agent) 保留新：精确匹配避免误中（如** `!STOP_X`**）；CAN 0x09 急停优于旧版伪急停（MoveJ currentJoints）；建议保留**                   |
| `!START`           | `find("START")`       | `SetEnable(true)`       | `s == "!START"`                          | `SetEnable(true)` + 解除 `isStalled`                         | **重构** | **(agent) 保留新：堵转期间 START 解除 isStalled 是必要的；建议保留**                                                        |
| `!DISABLE`         | `find("DISABLE")`     | `SetEnable(false)`      | `s == "!DISABLE"`                        | `SetEnable(false)` + 解除 `isStalled`                        | **重构** | **(agent) 保留新：与 START 对称；建议保留**                                                                          |
| `!HOME`            | `find("HOME")`        | `Homing()`              | `s == "!HOME"` + `isStalled` 检查          | `Homing()` + 堵转期间拒绝                                        | **重构** | **(agent) 保留新：堵转期间拒绝 Homing 避免位置误锁死；建议保留**                                                               |
| `!RESET`           | `find("RESET")`       | `Resting()`             | `s == "!RESET"` + `isStalled` 检查         | `Resting()` + 堵转期间拒绝                                       | **重构** | **(agent) 保留新：与 HOME 对称；建议保留**                                                                           |
| `!CALIBRATION`     | `find("CALIBRATION")` | `CalibrateHomeOffset()` | ❌                                        | **删除**（旧固件 `CalibrateHomeOffset()` 方法已移除）                  | **删除** | **(agent) 保留删除：归零流程已由** `#OFFSET_J` **替代；建议保留删除**                                                        |
| `!RGB_SET_START`   | ❌                     | —                       | 新增（USB:50/241；UART4:818/1018；实现有冗余但功能正确） | `SaveConfig()` + `rgbStateStart`                           | **新增** | **(agent) 保留新：开机默认灯效可配置；建议保留**；**注意：USB 实现两次（顺序中前置匹配 + 后续精确匹配），存在冗余，建议清理一处**                             |
| `!RGB_SET_ENABLE`  | ❌                     | —                       | 新增（USB:64/255；UART4:832/1032）            | `SaveConfig()` + `rgbStateEnable`                          | **新增** | **(agent) 保留新：使能时灯效可配置；建议保留**；**注意：与** `!RGB_SET_START` **类似的 USB 双重匹配冗余**                               |
| `!RGB_SET_DISABLE` | ❌                     | —                       | 新增（USB:78/269；UART4:846/1046）            | `SaveConfig()` + `rgbStateDisable`                         | **新增** | **(agent) 保留新：失能时灯效可配置；建议保留**；**注意：与** `!RGB_SET_START` **类似的 USB 双重匹配冗余**                               |
| `!RGB_BRIGHT`      | ❌                     | —                       | 新增（USB:184；UART4:961）                    | 亮度查询/设置/设置并保存（3种用法）                                        | **新增** | **(agent) 保留新：3种用法比固定命令灵活；建议保留**                                                                         |
| `!RGB_MODE`        | ❌                     | —                       | 新增（USB:205；UART4:982）                    | 切换灯效模式 0~9                                                 | **新增** | **(agent) 保留新：基础灯效控制；建议保留**                                                                              |
| `!RGB_COLOR`       | ❌                     | —                       | 新增（USB:218；UART4:995）                    | 静态纯色 RGB 0~255（3组可配置）                                      | **新增** | **(agent) 保留新：3组可配置静态纯色 + SaveConfig 持久化；建议保留**                                                          |
| `!HAND_ZERO`       | ❌                     | —                       | 新增（USB:114；UART4:877）                    | 夹爪标定：等待 `AllAtTarget(2.0f)` ≤10s 后 `ApplyPositionAsHome()` | **新增** | **(agent) 保留新：夹爪标定流程；等待 ≤10s 是合理的（夹爪速度慢）；建议保留**                                                          |
| `!HAND_EN`         | ❌                     | —                       | 新增（USB:284；UART4:889）                    | 夹爪使能                                                       | **新增** | **(agent) 保留新：夹爪独立使能；注意 USB 版多回一行 isEnabled 状态；建议保留**                                                    |
| `!HAND_DIS`        | ❌                     | —                       | 新增（USB:290；UART4:894）                    | 夹爪失能                                                       | **新增** | **(agent) 保留新：与 EN 对称；建议保留**                                                                             |
| `!HAND_POS`        | ❌                     | —                       | 新增（USB:296；UART4:899）                    | 夹爪位置控制 0~100                                               | **新增** | **(agent) 保留新：开度指令（0=闭合, 100=张开）；参数越界回 error；建议保留**                                                      |
| `!HAND_O`          | ❌                     | —                       | 新增（USB:125；UART4:919）                    | 夹爪张开（施力 -1）                                                | **新增** | **(agent) 保留新：力矩透传张开；建议保留**                                                                              |
| `!HAND_C`          | ❌                     | —                       | 新增（USB:131；UART4:924）                    | 夹爪闭合（施力 +1）                                                | **新增** | **(agent) 保留新：力矩透传闭合；建议保留**                                                                              |
| `!STALL_EN`        | ❌                     | —                       | 新增（USB:162；UART4:947）                    | `SetEnableStallProtect(true)` → J1~J7/地轨                   | **新增** | **(agent) 保留新：批量开启堵转保护（覆盖 J1~J6 + J7/地轨，跳过 motorJ[0] ALL 代理）；建议保留**                                      |
| `!STALL_DIS`       | ❌                     | —                       | 新增（USB:169；UART4:954）                    | `SetEnableStallProtect(false)` → J1~J7/地轨                  | **新增** | **(agent) 保留新：与 EN 对称；建议保留**                                                                             |
| `!STALL_STATUS`    | ❌                     | —                       | 新增（USB:144；UART4:929）                    | `QueryStallStatus()` + 500ms 限流                            | **新增** | **(agent) 保留新：500ms 节流（**`_stall_status_last_tick` **+** `STALL_STATUS_INTERVAL_MS`**）防总线/UART 拥塞；建议保留** |
| `!STALL_UNLOCK`    | ❌                     | —                       | 新增（USB:157；UART4:942）                    | `BroadcastUnlock()` ×3 次                                   | **新增** | **(agent) 保留新：3次连续广播（间隔 100ms）确保电机端接收；建议保留**                                                             |
| `!PRINTPOSE`       | ❌                     | —                       | 新增（USB:320；UART4:1060）                   | 打印 GETJPOS + GETLPOS                                       | **新增** | **(agent) 保留新：一键打印位姿；建议保留**                                                                              |


### 2.2 `#` 查询/配置命令


| 命令                  | 旧 ASCII 行号                         | 旧实现                                                       | 新 ASCII 行号                  | 新实现                                                                                                   | 标记     | new（手动标记）                                                                             |
| ------------------- | ---------------------------------- | --------------------------------------------------------- | --------------------------- | ----------------------------------------------------------------------------------------------------- | ------ | ------------------------------------------------------------------------------------- |
| `#GETJPOS`          | 返回 6 值                             | `dummy.currentJoints.a[0..5]`                             | 返回 7 值                      | + `currentRailPos`（Rail, J1~J6）                                                                       | **重构** | **(agent) 保留新：地轨纳入查询是必然需求；建议保留**                                                      |
| `#GETLPOS`          | 调用 `UpdateJointPose6D()`           | 通信线程内触发 FK                                                | 直接读缓存                       | `currentPose6D`（预计算缓存）                                                                                | **重构** | **(agent) 保留新：5kHz 控制环已维护缓存，避免通信线程做 FK 与 ASCII 任务竞争；建议保留**                            |
| `#SET_DCE_KP`       | 仅 J1~J6                            | `SetDceKp(kp)`                                            | 扩到 J7/地轨(ID=7)、夹爪(ID=8)     | `motorJ[7]/motorJ[node]/hand`                                                                         | **重构** | **(agent) 保留新：扩域是必然需求（地轨+夹爪也要 DCE）；建议保留**                                             |
| `#SET_DCE_KV/KI/KD` | 仅 J1~J6（KV 在旧版中**不存在**，仅 KP/KI/KD） | 同上                                                        | 同上扩域                        | 同上                                                                                                    | **重构** | **(agent) 保留新：**`#SET_DCE_KV` **旧版不存在，是新增项；KV 是 DCE 速度前馈分量，建议保留**                     |
| `#SET_PID`          | ❌                                  | —                                                         | 新增（USB:439；UART4:1199）      | 批量设置 Kp/Kv/Ki/Kd                                                                                      | **新增** | **(agent) 保留新：批量设置 4 参数比逐个发送高效；建议保留**                                                 |
| `#GET_PID`          | ❌                                  | —                                                         | 新增（USB:465；UART4:1225）      | 批量查询 Kp/Kv/Ki/Kd（回包走 `PollPidPrint()` 合并）                                                             | **新增** | **(agent) 保留新：合并打印机制避免 4 次回包产生 4 行散乱输出；建议保留**                                         |
| `#REBOOT`           | 仅带参数                               | `#REBOOT %lu`                                             | 无参版 + 带参版                   | 无参 → ALL 代理广播 + 主控 reset；带参 → 指定节点（1~7，不含 8）                                                          | **重构** | **(agent) 保留新：无参版一键全重启（v3.2 ALL 代理兼容）；带参版节点范围扩到 1~7；夹爪 ID=8 无 Reboot 命令（旧版也没有）；建议保留** |
| `#CMDMODE`          | 存在                                 | `SetCommandMode()`                                        | 同                           | 支持新模式 COMMAND_SERVO_J/COMMAND_TORQUE_CONTROL（USB 回执加 "ok " 前缀，UART4 不变）                               | **重构** | **(agent) 保留新：扩模式到 6（SEQ/INT/TRJ/TUN/TRQ/SRV）是新功能需求；USB/UART4 回执差异是已知小问题，建议保留新功能**    |
| `#OFFSET_J`         | ❌                                  | —                                                         | 新增（USB:531；UART4:1284）      | 单轴归零标定 / 批量 `all`；**v3.2 修复**：删除主控端乐观同步                                                               | **新增** | **(agent) 保留新：单轴归零比全轴归零更灵活；v3.2 修复乐观同步避免与 0x23 回包竞争；建议保留**                            |
| `#ACC_J`            | ❌                                  | —                                                         | 新增（USB:581；UART4:1324）      | `SetAcceleration_persist(float, bool)`；支持 `&` 持久化标志；扩到 J7/地轨、夹爪；clamp 0~5000                          | **新增** | **(agent) 保留新：单位统一为 r/s²（v2.6 重构）后唯一入口；clamp 0~5000 防止非法值；建议保留**                      |
| `#ACC_BASE_J`       | 旧有（%）                              | `SetAcceleration(bases * %)`                              | ❌                           | **删除**（v2.6 单位统一后已废弃；UART4 中保留 "已删除"提示分支）                                                             | **删除** | **(agent) 保留删除：单位统一后旧版双层语义（基准 × 百分比）无意义；建议保留删除**                                      |
| `#ACC_RAIL`         | 旧有                                 | 单独地轨加速度                                                   | ❌                           | **删除**（走 `#ACC_J 7` 即可；UART4 中保留 "已删除"提示分支）                                                           | **删除** | **(agent) 保留删除：统一到** `#ACC_J 7 <r/s²> [&]`；建议保留删除**                                   |
| `#SPEED_J`          | ❌                                  | —                                                         | 新增（USB:618；UART4:1366）      | 设置单轴速度限幅（仅 J1~J6，不含 7/8）                                                                              | **新增** | **(agent) 保留新：与** `#ACC_J` **对称；注意 7/8 不支持（旧版就没有）；建议保留**                              |
| `#I_LIMIT_J`        | ❌                                  | —                                                         | 新增（USB:634；UART4:1381）      | `SetCurrentLimit_persist(float, bool)`；扩到 J7/地轨、夹爪                                                    | **新增** | **(agent) 保留新：与** `#ACC_J` **对称；建议保留**                                                |
| `#GETJACC`          | ❌                                  | —                                                         | 新增（USB:669）                 | 同步轮询 `jointAccRuntime[idx]`（30ms 轮询时间戳）                                                               | **新增** | **(agent) 保留新：30ms 超时保护 + 时间戳比对判断回包到达（避免 ISR 内 printf 阻塞 ASCII 任务 UART TX）；建议保留**     |
| `#GETI`             | ❌                                  | —                                                         | 新增（USB:721）                 | 同步轮询 `jointCurrentLimitRuntime[idx]`（30ms 轮询时间戳）                                                      | **新增** | **(agent) 保留新：与** `#GETJACC` **对称；建议保留**                                              |
| `#SYNC_ACC`         | ❌                                  | —                                                         | 新增（USB:574；UART4:1112/1361） | 批量触发 8 个电机 `QueryAcceleration()`                                                                      | **新增** | **(agent) 保留新：与** `#GETJACC` **配套（缓存先行 + 后续同步查询）；建议保留**                               |
| 默认 `#` 分支           | 回 `"ok"`                           | USB：转发到 `commandHandler.Push(_cmd)`；UART4：回 `"ok"`（保持原行为） | **重构**                      | **(agent) 保留新（USB）：**透传未知 `#` 命令到队列可让命令线程接管，行为更灵活；**保留旧（UART4）：**简单回执；USB 与 UART4 行为不一致是已知小差异，建议二选一统一 |        |                                                                                       |


### 2.3 `>/&/@/$` 运动指令格式


| 维度         | 旧固件               | 新固件                                                     | 标记     | new（手动标记）                                                                |
| ---------- | ----------------- | ------------------------------------------------------- | ------ | ------------------------------------------------------------------------ |
| `>/&` 参数数量 | 6个关节 + speed（第7位） | **7个关节 + 地轨 + speed**（>j1,j2,j3,j4,j5,j6,j7_rail,speed） | **重构** | **(agent) 保留新：地轨参与 MoveJ/ServoJ 是新增架构需求；建议保留**                           |
| `@` 参数数量   | 6维坐标 + speed      | 6维坐标 + speed（不变）                                        | —      | **(agent) 保留现状：6维 FK 不含地轨是合理的（笛卡尔空间不包含地轨自由度）；建议保留**                      |
| `$` 力矩指令   | ❌                 | **新增**：`$c1,c2,c3,c4,c5,c6,c7_rail`（7个 float）           | **新增** | **(agent) 保留新：力矩透传是 5kHz 实时环的新需求（**`COMMAND_TORQUE_CONTROL` **模式）；建议保留** |
| SERVO_J 格式 | ❌                 | **新增**：`>/& j1..j6,j7_rail`（无 speed，第7位是 j7）            | **新增** | **(agent) 保留新：SERVO_J 是 5kHz 全速下发模式（不节流），无 speed 是合理的；建议保留**             |
| 队列机制       | 无独立队列线程           | `CommandHandler` 队列（32条 × 128字节，旧版 16×64）               | **重构** | **(agent) 保留新：队列容量翻倍可缓冲更多指令；建议保留**                                       |
| 未使能拒绝      | 无保护               | **新增**：运动指令检查 `IsEnabled()` + `IsStalled()`             | **新增** | **(agent) 保留新：未使能时拒绝运动指令避免电机乱跑；堵转期间拒绝避免死锁；建议保留**                         |


---

## 三、`DummyRobot` 类对比（`Robot/instances/dummy_robot.h` + `.cpp`）

### 3.1 架构变化


| 维度            | 旧固件                                               | 新固件                                                              | 标记     | new（手动标记）                                                                               |
| ------------- | ------------------------------------------------- | ---------------------------------------------------------------- | ------ | --------------------------------------------------------------------------------------- |
| motorJ[] 数组大小 | `CtrlStepMotor* motorJ[7]`（[0]=ALL, [1..6]=J1~J6） | `CtrlStepMotor* motorJ[8]`（[0]=ALL广播代理, [1..6]=J1~J6, [7]=J7/地轨） | **重构** | **(agent) 保留新：8 槽位是 7轴 + 地轨的必要扩展；ALL 代理（nodeID=0）确保广播机制兼容；建议保留**                        |
| hand 类        | `DummyHand* hand`（ID=7，裸 CAN 发帧）                  | `StepHand* hand`（ID=8，继承 `CtrlStepMotor`）                        | **重构** | **(agent) 保留新：继承** `CtrlStepMotor` **让 hand 复用全部 CtrlStep 接口；ID=8 避开 J7/地轨的 ID=7；建议保留** |
| DH 参数         | `{0.109, 0.035, 0.146, 0.115, 0.052, 0.072}`      | `{0.165, 0.0, 0.170, 0.117, 0.0695, 0.113}`（新机械结构）               | **重构** | **(agent) 保留新：DH 参数是真实机械结构决定的，迁移需重新标定；建议保留**                                            |


### 3.2 新增常量（`dummy_robot.h`）


| 常量                            | 值                          | 定义行                   | 说明                                               | new（手动标记）                                                                                                  |
| ----------------------------- | -------------------------- | --------------------- | ------------------------------------------------ | ---------------------------------------------------------------------------------------------------------- |
| `ALL`                         | `0`                        | `dummy_robot.h:7`     | 广播代理索引（StdId=(0<<7)                              | cmd 触发所有电机）                                                                                                |
| `SLIDER_TO_RPS`               | `0.30f`                    | `dummy_robot.h:13`    | slider 100 → 30 r/s                              | **保留新：slider 物理换算系数；建议保留**                                                                                 |
| `AXIS_MAX_RPS[8]`             | `{0,30,30,30,30,30,30,30}` | `dummy_robot.h:18-22` | 各轴物理上限 r/s；[7]=J7/地轨                             | **保留新：各轴物理上限防止失步/堵转；建议保留**                                                                                 |
| `MOTOR_REDUCTION[8]`          | `{1,50,50,50,50,50,30,1}`  | `dummy_robot.h:27-29` | 各轴减速比；[7]=地轨直连                                   | **保留新：与电机实际减速比匹配；注意旧版 motorJ[1..6] 减速比是** `{30,30,30,24,30,50}`**（详见 §3 备注），新版统一改 50 是硬件升级；建议保留**          |
| `HAND_MAX_RPS`                | `20.0f`                    | `dummy_robot.h:31`    | 夹爪速度上限                                           | **保留新：夹爪使用 35 电机 reduction=16，输出轴速度 = 20/16 ≈ 1.25 r/s；建议保留**                                              |
| `FALLBACK_JOINT_ACCELERATION` | `10.0f`                    | `dummy_robot.h:34`    | 电机加速度查询未回包时的兜底值 r/s²                             | **保留新：兜底值防止** `jointAccRuntime[i]==0` **时 ComputeSyncSpeeds 算崩；建议保留**                                      |
| `RAIL_STEPS_PER_MM`           | `40960.0f`                 | `dummy_robot.h:144`   | 丝杆 1605 直连：200步×1024微步/5mm                       | **保留新：地轨换算常数；丝杆 1605 直连，200 步/圈 × 1024 微步 / 5mm/圈 = 40960 步/mm；建议保留**                                      |
| `ALL_FINISH_FLAG`             | `0x7E`                     | `dummy_robot.h:183`   | jointsStateFlag 全置位 = bit1~6=1, bit0=0           | **保留新：v3.2 修复** `IsMoving()` **短路条件所需；bit0（地轨位）恒为 0 是合理的（地轨不在** `UpdateJointAnglesCallback` **处理范围）；建议保留** |
| `REST_POSE`                   | `{0,-75,180,0,0,0}`        | `dummy_robot.h:220`   | J1~J6；旧为 `{0,-73,180,0,0,0}`                     | **保留新：J2 从 -73° 改为 -75° 是机械结构调整；建议保留**                                                                     |
| `DEFAULT_JOINT_SPEED`         | `80`                       | `dummy_robot.h:222`   | slider 默认值；旧为 `30`                               | **保留新：slider 默认值从 30 提升到 80（实际 r/s = 80 × SLIDER_TO_RPS = 24 r/s）；建议保留**                                   |
| `DEFAULT_JOINT_ACCELERATION`  | `150.0f`                   | `dummy_robot.h:229`   | r/s²；旧为双层语义 `DEFAULT_JOINT_ACCELERATION_BASES[]` | **保留新：v2.6 加速度单位统一为 r/s²（替代旧版** `BASES × %` **双层语义）；建议保留**                                                 |


### 3.3 新增字段（`dummy_robot.h`）


| 字段                                 | 类型                    | 行号      | 说明                  | new（手动标记）                                                                                |
| ---------------------------------- | --------------------- | ------- | ------------------- | ---------------------------------------------------------------------------------------- |
| `currentRailPos`                   | `float`               | 145     | 地轨当前位置 mm           | **保留新：地轨位置由 0x23 回包维护（**`can_protocol.cpp:179-184`**）；**`#GETJPOS` **直接读该字段；建议保留**       |
| `targetRailPos`                    | `float`               | 148     | 地轨目标位置 mm           | **保留新：MoveJ/ServoJ 写入；SetStallMode 同步；建议保留**                                             |
| `targetRailCurrent`                | `float`               | 150     | 地轨目标电流 mA           | **保留新：力矩透传时** `SetJointCurrents(c0, ...)` **写入；建议保留**                                    |
| `jointSpeedRps`                    | `float`               | 155     | 电机轴 r/s（slider 换算后） | **保留新：v2.6 重构为电机轴 r/s（替代旧版 °/s 错误语义）；建议保留**                                              |
| `dynamicJointSpeeds7`              | `DynamicJointSpeeds7` | 160-163 | 7轴速度 r/s（含地轨）       | **保留新：7轴速度规划输出；建议保留**                                                                    |
| `railSpeedRps`                     | `float`               | 176     | 地轨速度 r/s            | **保留新：硬编码 30 r/s 非 MoveJ 路径（Homing/Resting/EmergencyStop）的兜底速度；建议保留**                    |
| `targetCurrents[6]`                | `float[6]`            | 178     | 力矩透传 6轴目标电流         | **保留新：5kHz 实时环 TORQUE_CONTROL 模式写入；建议保留**                                                |
| `motorDceKps[9]`                   | `int32_t[9]`          | 181     | DCE Kp 缓存           | **保留新：索引 [0]=ALL空闲, [1..7]=J1~J7/地轨, [8]=夹爪；目前实际未使用（**`#GET_PID` **走 ISR 临时缓存），建议清理或启用** |
| `jointAccRuntime[9]`               | `float[9]`            | 233     | 加速度查询缓存             | **保留新：**`#GETJACC` **同步查询所需；建议保留**                                                       |
| `jointAccLastUpdateMs[9]`          | `uint32_t[9]`         | 234     | 加速度缓存时间戳            | **保留新：30ms 超时保护 + 时间戳比对判断回包到达；v3.0 修复 UART TX 竞争关键；建议保留**                                |
| `jointCurrentLimitRuntime[9]`      | `float[9]`            | 237     | 电流限幅缓存              | **保留新：与** `jointAccRuntime` **对称；**`#GETI` **同步查询所需；建议保留**                               |
| `jointCurrentLimitLastUpdateMs[9]` | `uint32_t[9]`         | 238     | 电流限幅缓存时间戳           | **保留新：与** `jointAccLastUpdateMs` **对称；建议保留**                                             |
| `isStalled`                        | `bool`                | 320     | 堵转拦截状态              | **保留新：**`ParseCommand`**/**`!HOME`**/**`!RESET` **入口检查；建议保留**                            |
| `lastServoTime`                    | `uint32_t`            | 186     | ServoJ 上次时间 us      | **保留新：ServoJ 计算 dt 所需；建议保留**                                                             |


### 3.4 新增/删除方法


| 方法                                            | 行为                                      | 标记                                                | new（手动标记）                                                                                      |
| --------------------------------------------- | --------------------------------------- | ------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `MoveJ(float... float _j7_mm, float _slider)` | 7参数（含地轨）；内置梯形同步规划 `ComputeSyncSpeeds()` | **新增**                                            | **保留新：7参数 + 梯形同步规划是 v2.7 重构核心；地轨限位检查 + 臂关节逐轴检查；建议保留**                                          |
| `ServoJ(float... float _j7_mm)`               | 高频随动（含地轨）；无加减速规划                        | **新增**                                            | **保留新：高频伺服随动（5kHz 全速下发）是新需求；建议保留**                                                             |
| `MoveL(...)`                                  | 笛卡尔直线（含地轨保持）；反解选最近解后调 MoveJ             | **重构**                                            | **保留新：地轨保持（**`MoveJ(..., currentRailPos, ...)`**）是新增需求；最近解选取算法重写（**`minDist=1e9f` **初值）；建议保留** |
| `SyncAllMotorAcceleration()`                  | 批量发 CAN 0x2C 到 8个电机                     | **新增**                                            | **保留新：8个电机批量触发查询；初始化时调用一次回填** `jointAccRuntime[]`**；建议保留**                                     |
| `SetStallMode()` / `SetStallMode(int)`        | 堵转时停止发指令 + 同步 targetRailPos             | **新增**                                            | **保留新：堵转模式切换 + 同步 targetRailPos；建议保留（是否使用取决于是否启用堵转检测）**                                        |
| `BroadcastUnlock()`                           | 连续 3次 CAN 0x5B 广播（间隔 100ms）             | **新增**                                            | **保留新：与** `!STALL_UNLOCK` **配套；建议保留**                                                          |
| `QueryStallStatus()`                          | 查询 7个电机 J1~J7/地轨的 en/lock 状态            | **新增**                                            | **保留新：与** `!STALL_STATUS` **配套；建议保留**                                                          |
| `SetJointCurrents(float c0..c7)`              | 力矩透传：c0=地轨，c1~c6=关节，c7=夹爪               | **新增**                                            | **保留新：**`$` **指令解析后调用；建议保留**                                                                   |
| `LoadConfig()` / `SaveConfig()`               | EEPROM 灯效配置读写                           | **新增**                                            | **保留新：灯效配置持久化（EepromConfig + magic=0x12345679）；建议保留**                                          |
| `CalibrateHomeOffset()`                       | ❌                                       | **删除**（旧固件有；新固件由 `#OFFSET_J` 替代）                  | **保留删除：**`#OFFSET_J` **提供更精准的单轴归零（**`dummy_robot.cpp` **行为验证后已移除旧实现）；建议保留删除**                  |
| `SetRGBEnable(bool)`                          | ❌                                       | **删除**（RGB 使能由 `rgbStateStart/Enable/Disable` 替代） | **保留删除：RGB 使能由 3个预设状态码管理（**`rgbStateStart/Enable/Disable`**）替代布尔开关，更灵活；建议保留删除**                |
| `GetRGBEnabled()`                             | ❌                                       | **删除**                                            | **保留删除：与** `SetRGBEnable` **配套删除；建议保留删除**                                                      |


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
| `COMMAND_MOTOR_TUNING` (=4)               | 同上                                                                              | 同上                                                                   | —      | **保留现状：扫频模式不变**                                                                                           |
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


| 方法                                             | StdId        | 功能                               | 行号                  | new（手动标记）                                                                |
| ---------------------------------------------- | ------------ | -------------------------------- | ------------------- | ------------------------------------------------------------------------ |
| `SetAngleWithMotorRps(float angle, float rps)` | —            | 角度+速度（用 `reduction` 换算）          | `ctrl_step.cpp:74`  | **保留新：v2.6 重命名（替代** `SetAngleWithVelocityLimit`**）；reduction 换算正确；建议保留** |
| `SetCurrentLimit_persist(float, bool)`         | `(nodeID<<7) | 0x12`                            | 按参数决定是否写 EEPROM     | `ctrl_step.cpp:101`                                                      |
| `SetAcceleration_persist(float, bool)`         | `(nodeID<<7) | 0x14`                            | 按参数决定是否写 EEPROM     | `ctrl_step.cpp:127`                                                      |
| `QueryCurrentLimit()`                          | `(nodeID<<7) | 0x2D`                            | 查询电流限幅              | `ctrl_step.cpp:136`                                                      |
| `QueryAcceleration()`                          | `(nodeID<<7) | 0x2C`                            | 查询加速度               | `ctrl_step.cpp:141`                                                      |
| `QueryDceKp()`                                 | `(nodeID<<7) | 0x28`                            | 查询 DCE Kp           | `ctrl_step.cpp:211`                                                      |
| `QueryDceKv()`                                 | `(nodeID<<7) | 0x29`                            | 查询 DCE Kv           | `ctrl_step.cpp:218`                                                      |
| `QueryDceKi()`                                 | `(nodeID<<7) | 0x2A`                            | 查询 DCE Ki           | `ctrl_step.cpp:224`                                                      |
| `QueryDceKd()`                                 | `(nodeID<<7) | 0x2B`                            | 查询 DCE Kd           | `ctrl_step.cpp:231`                                                      |
| `BroadcastUnlock()`                            | `StdId=0x5B` | 全局解锁广播（电机端收到→IDLE）               | `ctrl_step.cpp:237` | **保留新：与** `!STALL_UNLOCK` **配套；建议保留**                                    |
| `BroadcastEStop()`                             | `(nodeID<<7) | 0x09`                            | 急停广播                | `ctrl_step.cpp:245`                                                      |
| `QueryStallStatus(uint8_t)`                    | `(nodeID<<7) | 0x5C`                            | 查询堵转状态（en/lock）     | `ctrl_step.cpp:253`                                                      |
| `SetStallMode()` / `SetStallMode(int)`         | —            | 堵转拦截状态                           | `ctrl_step.cpp:204` | **保留新：堵转时停止发指令；建议保留（是否使用取决于是否启用堵转检测）**                                   |
| `SetAngle(float)`                              | —            | 角度直接设置（内部调用 SetPositionSetPoint） | `ctrl_step.cpp:183` | **保留新：关节角度直接下发接口；建议保留**                                                  |


### 4.2 删除方法


| 方法                                           | 旧固件                                | 说明                 | new（手动标记）                                            |
| -------------------------------------------- | ---------------------------------- | ------------------ | ---------------------------------------------------- |
| `SetPositionWithVelocityLimit(float, float)` | 旧名；新版改名为 `SetPositionWithMotorRps` | 重命名；旧名已废弃          | **保留删除：旧名已废弃，建议删除；新版统一使用** `SetPositionWithMotorRps` |
| `DoCalibration()`                            | 旧有；新版删除                            | 校准在电机端本地完成，不通过主控驱动 | **保留删除：校准流程应在电机端本地完成**                               |


### 4.3 `SetEnable()` 行为变化


| 维度              | 旧固件                    | 新固件                                                          | 标记     | new（手动标记）                                                               |
| --------------- | ---------------------- | ------------------------------------------------------------ | ------ | ----------------------------------------------------------------------- |
| enable 时        | `state = FINISH`（乐观同步） | **不写** `state = FINISH`（等待电机端 0x23 回包纠正）                     | **重构** | **(agent) 保留新：乐观同步 FINISH 会与 0x23 回包竞争；建议保留**                           |
| disable 时       | `state = STOP`（有）      | 同旧：`state = STOP`                                            | —      | **保留现状：disable 显式 STOP 是电机端状态机对齐所必需；建议保留**                              |
| targetAngle     | `targetAngle` 保留       | **不写** `targetAngle`（jointsStateFlag 方案不再依赖此字段）              | **重构** | **(agent) 保留新：jointsStateFlag 方案不依赖 targetAngle；建议保留**                  |
| jointsStateFlag | 无（不操作）                 | enable 时不清；disable 时不清（由 0x23 回包更新）                          | **重构** | **(agent) 保留新：jointsStateFlag 由 0x23 回包 ISR 更新，SetEnable 不干扰；建议保留**     |
| isStalled       | 无                      | enable 时不清；disable 时不清（`isStalled` 由 `!START/!DISABLE` 单独清除） | **新增** | **(agent) 保留新：isStalled 由显式** `!START/!DISABLE` **清除，避免与 SetEnable 耦合** |
| Homing 中 enable | 有（`state = FINISH`）    | 同旧                                                           | —      | **保留现状：Homing 中 enable 触发 FINISH 是正常的**                                 |


### 4.4 `SetEnable()` 与 `IsMoving()` 联动（v3.2 修复）

> 本节说明 `SetEnable()` 修改后，`IsMoving()` 短路条件为何必须同步修改。


| 维度                | 旧固件                                               | 新固件                                                                    | 标记     | new（手动标记）                                                                               |
| ----------------- | ------------------------------------------------- | ---------------------------------------------------------------------- | ------ | --------------------------------------------------------------------------------------- |
| IsMoving() 公式     | `jointsStateFlag != 0b1111110`                    | `!isEnabled ? false : jointsStateFlag != 0x7E`（`main.cpp:208`）         | **重构** | **(agent) 保留新：v3.2 短路** `!isEnabled ? false` **防止 SEQUENTIAL 队列被 while 阻塞吞指令；必须保留**     |
| 失能后 IsMoving() 返回 | true（旧版即使 disable，IsMoving() 仍返回 true → 队列被阻塞）    | false（disable 后 IsMoving() 立即返回 false → 队列可继续处理下一条）                    | **重构** | **(agent) 保留新：v3.2 修复关键 bug；disable 后队列可正常推进；必须保留**                                     |
| 配合条件              | 无配合（SetEnable() 乐观同步 FINISH 作为补偿）                 | SetEnable() enable 时不写 FINISH，disable 时显式写 STOP；IsMoving() 靠 0x23 回包短路 | **重构** | **(agent) 保留新：两个修复配套——SetEnable 不乐观同步 FINISH + IsMoving() 短路 isEnabled；必须同时保留才能消除队列死锁** |
| SEQUENTIAL 模式影响   | 上一指令未 FINISH 时下一条被 while 阻塞（`while(!IsMoving())`） | disable 后 IsMoving() 立即 false → while 退出 → 队列可继续消费                     | **重构** | **(agent) 保留新：v3.2 修复 SEQUENTIAL 模式队列死锁问题；建议保留**                                        |


---

## 五、ASCII 命令实现位置对照（USB vs UART4）

> 新固件 USB 和 UART4 各有一份实现，部分存在重复或差异。下表标注需要关注的不一致处。


| 命令                 | USB 行号                                    | UART4 行号                                  | 一致性                 | new（手动标记）                                |
| ------------------ | ----------------------------------------- | ----------------------------------------- | ------------------- | ---------------------------------------- |
| `!STOP`            | 精确匹配                                      | 精确匹配                                      | ✅ 一致                |                                          |
| `!START`           | 精确匹配                                      | 精确匹配                                      | ✅ 一致                |                                          |
| `!DISABLE`         | 精确匹配                                      | 精确匹配                                      | ✅ 一致                |                                          |
| `!HOME`            | 精确匹配                                      | 精确匹配                                      | ✅ 一致                |                                          |
| `!RESET`           | 精确匹配                                      | 精确匹配                                      | ✅ 一致                |                                          |
| `!CALIBRATION`     | ❌                                         | ❌                                         | ✅ 一致（均删除）           |                                          |
| `!RGB_SET_START`   | `find()` 前置匹配 + `s == "!RGB_SET_START"`   | `find()` 前置匹配 + `s == "!RGB_SET_START"`   | ⚠️ **USB 存在双重匹配冗余** | **建议清理：USB 侧删除** `find()` **分支，仅保留精确匹配** |
| `!RGB_SET_ENABLE`  | `find()` 前置匹配 + `s == "!RGB_SET_ENABLE"`  | `find()` 前置匹配 + `s == "!RGB_SET_ENABLE"`  | ⚠️ **USB 存在双重匹配冗余** | 同上                                       |
| `!RGB_SET_DISABLE` | `find()` 前置匹配 + `s == "!RGB_SET_DISABLE"` | `find()` 前置匹配 + `s == "!RGB_SET_DISABLE"` | ⚠️ **USB 存在双重匹配冗余** | 同上                                       |
| `!HAND_EN`         | 精确匹配                                      | 精确匹配                                      | ✅ 一致                | **注意：USB 版多回一行 isEnabled 状态**            |
| `!STALL_STATUS`    | 精确匹配                                      | 精确匹配                                      | ✅ 一致                |                                          |
| `#GETJPOS`         | `getJointPositions()`                     | `getJointPositions()`                     | ✅ 一致                |                                          |
| `#GETLPOS`         | 直接读缓存                                     | 直接读缓存                                     | ✅ 一致                |                                          |
| `#CMDMODE`         | USB 回执加 `"ok "` 前缀                        | UART4 回执无前缀                               | ⚠️ **已知差异**         | **保留新功能（USB 加前缀）；差异是已知小问题，可接受**          |
| 默认 `#` 分支          | 转发到队列                                     | 回 `"ok"`                                  | ⚠️ **行为不一致**        | **建议二选一统一**                              |


---

## 六、新增/重构/删除 汇总

### 6.1 保留的新增（26 项）

**CAN 命令新增**（5项）：

1. `0x09` Broadcast EStop —— 安全急停，不依赖堵转检测
2. `0x28` Get DCE Kp —— GET 查询命令
3. `0x29` Get DCE Kv —— GET 查询命令
4. `0x2C` Get Acceleration —— GET 查询命令
5. `0x2D` Get Current-Limit —— GET 查询命令

**堵转协议新增**（4项）：
6. `0x5A` STALL Broadcast ISR 标志 —— 堵转事件 ISR 只设标志，主线程 poll
7. `0x5B` BroadcastUnlock —— 全局解锁广播
8. `0x5C` Stall Query —— 堵转状态查询
9. `0x7C` 旧堵转上报兼容 —— 兼容旧固件电机端 0x7C

**CtrlStepMotor 新方法**（8项）：
10. `SetAngleWithMotorRps()` —— 角度+速度（reduction 换算）
11. `SetCurrentLimit_persist()` / `SetAcceleration_persist()` —— 可选 EEPROM 持久化
12. `QueryDceKp/Kv/Ki/Kd()` —— DCE 参数查询
13. `QueryAcceleration()` / `QueryCurrentLimit()` —— 运行时参数查询
14. `BroadcastUnlock()` / `BroadcastEStop()` —— 广播命令
15. `QueryStallStatus()` —— 堵转状态查询
16. `SetStallMode()` / `SetStallMode(int)` —— 堵转拦截
17. `SetAngle()` —— 角度直接下发

**DummyRobot 新方法/行为**（9项）：
18. `MoveJ(7 参数)` —— 7轴+地轨梯形同步规划
19. `ServoJ(高频随动)` —— 5kHz 全速下发
20. `MoveL(笛卡尔直线)` —— 含地轨保持
21. `SyncAllMotorAcceleration()` —— 8轴批量加速度查询
22. `BroadcastUnlock()` / `QueryStallStatus()` —— 堵转相关
23. `SetJointCurrents()` —— 力矩透传
24. `LoadConfig()` / `SaveConfig()` —— 灯效配置持久化
25. 新字段：地轨 `currentRailPos` / `targetRailPos` / `targetRailCurrent`
26. 新字段：加速度/电流限幅缓存 + 时间戳（`jointAccRuntime[]` / `jointCurrentLimitRuntime[]`）

**5kHz 实时环新行为**（1项）：
27. SERVO_J 模式（5kHz 全速下发）
28. TORQUE_CONTROL 模式（100Hz 下发电流）
29. `$` 指令解析
30. `IsMoving()` 短路 `!isEnabled`（v3.2 修复）

### 6.2 删除的旧项（4项）

1. `DoCalibration()` —— 校准在电机端本地完成
2. `CalibrateHomeOffset()` —— 由 `#OFFSET_J` 替代
3. `SetRGBEnable(bool)` / `GetRGBEnabled()` —— 由 3 个灯效预设状态码替代
4. `SetPositionWithVelocityLimit()` —— 改名为 `SetPositionWithMotorRps`

### 6.3 重构项（11项）

1. `0x01` SetEnable —— enable 时不再乐观同步 FINISH
2. `0x07` SetPositionWithMotorRps —— 重命名（rps 语义更清晰）
3. `0x12/0x14` SetCurrentLimit/SetAcceleration —— 新增 `_persist()` 变体
4. `0x23` UpdateAngle —— 广播→单播轮询；回包分三段处理（id=7/1~6/8）
5. `CommandHandler` 队列 —— 32×128 替代 16×64
6. `EmergencyStop` 行为 —— MoveJ 伪急停 → CAN 0x09 广播急停
7. `IsMoving()` 公式 —— v3.2 短路 `!isEnabled ? false`
8. `SetCommandMode` —— 扩到 6 种模式（SEQ/INT/TRJ/TUN/TRQ/SRV）
9. `CanSendMessage()` —— mailbox 空闲预检 + 10ms 超时 + 失败释放
10. `ALL_FINISH_FLAG` —— v3.2 引入
11. 堵转期间运动指令保护 —— `ParseCommand` 入口检查 `isStalled`

### 6.4 保留旧固件的元素（不改动）

1. 所有基础控制命令（`0x03/0x04/0x05/0x11/0x13/0x15/0x16/0x17-0x1A/0x1B/0x7D/0x7E/0x7F`）
2. `GetTemp()` 查询温度
3. `UpdateAngleCallback()` 回包回调框架
4. `Mode_t` / `State_t` 枚举（主控侧不变）
5. `CommandMode` 旧 4 种模式语义（SEQ/INT/TRJ/TUN 不变）

