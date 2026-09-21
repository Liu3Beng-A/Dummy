# 新固件功能清单 — `firmware/` vs `firmware_old/` 完整对比

> **文档目的**：把当前 `firmware/`（新）相比 `firmware_old/`（旧）**新增/变更的所有功能**完整记录下来，作为后续 cherry-pick 迁移的依据。
> **对比日期**：2026-09-18
> **数据来源**：直接 grep 实际源码（`E:/Dummy-code/firmware/` 与 `E:/Dummy-code/firmware_old/`），每条附 **文件:行号** 证据。

---

## 目录

1. [CAN 节点 ID 编号约定](#一can-节点-id-编号约定)
2. [电机固件新增功能（STM32F103）](#二电机固件stm32f103新增功能)
3. [主控固件新增功能（STM32F405）](#三主控固件stm32f405新增功能)
4. [增量总览（按维度）](#四增量总览按维度)
5. [当前已发现但未修复的 BUG](#五当前已发现但未修复的-bug)
6. [cherry-pick 优先级建议](#六cherry-pick-优先级建议)
7. [待确认决策点](#七待确认决策点)

---

## 一、CAN 节点 ID 编号约定


| 节点              | CAN ID     | 来源                                | 验证位置                                                                               |
| --------------- | ---------- | --------------------------------- | ---------------------------------------------------------------------------------- |
| J1~J6（42电机）     | 1~6        | 拨码开关 PA8/PA9/PA10                 | `motor_fw_f103_42/UserApp/main.cpp:18-23`                                          |
| **J7/地轨（57电机）** | **7（硬编码）** | `#define RAIL_FIXED_NODE_ID 7`    | `motor_fw_f103_57/UserApp/configurations.h:13`；`main.cpp:18`（注释明确："node ID 9 已废弃"） |
| **夹爪（35电机）**    | **8（硬编码）** | `#define GRIPPER_FIXED_NODE_ID 8` | `motor_fw_f103_gripper/UserApp/configurations.h:13`；`main.cpp:53`                  |
| ALL 广播代理        | 0          | 主控 `motorJ[0]` 占位                 | `ref_core_f405/Robot/instances/dummy_robot.cpp:153`                                |


**结论**：迁移新功能到新基线时，**地轨 ID 必须改用 7**，不是 9。

---



## 二、电机固件（STM32F103）新增功能



### 2.1 CAN 协议完整清单

格式：`(nodeID<<7)|cmdCode`，8 字节数据。所有 cmd 实现位置：`motor_fw_f103_XX/UserApp/protocols/interface_can.cpp`


| Cmd         | 方向     | 名称                      | 数据                     | 单位    | 作用                                                           | 行号                                  |
| ----------- | ------ | ----------------------- | ---------------------- | ----- | ------------------------------------------------------------ | ----------------------------------- |
| 0x01        | 主→电    | Enable                  | uint32(1/0)            | —     | **联动 stall 状态机**（LOCKED 退出 + ResetGoals + 启动豁免复位）            | `:28-63`                            |
| 0x02        | 主→电    | Calibrate               | —                      | —     | 编码器校准                                                        | `:65`                               |
| 0x03        | 主→电    | Set Current             | float                  | A     | 切 `MODE_COMMAND_CURRENT`                                     | `:68`                               |
| 0x04        | 主→电    | Set Velocity            | float                  | r/s   | 切 `MODE_COMMAND_VELOCITY`                                    | `:73`                               |
| 0x05        | 主→电    | Set Position            | float 圈                | 圈     | LOCKED 时**丢弃**                                               | `:83`                               |
| 0x06        | 主→电    | Set Pos w/ Time         | float pos+time         | 圈,s   | 梯形自动选 vmax                                                   | `:107`                              |
| 0x07        | 主→电    | Set Pos w/ VelLimit     | float pos+vel          | 圈,r/s | LOCKED 时**丢弃**，无立即 ACK                                       | `:124`                              |
| **0x09**    | 主→ALL  | **Broadcast EStop**     | —                      | —     | 全节点 STOP + 抱闸 + Cur/Vel=0（**修复旧版 0x89 bug**）                 | `:358`                              |
| 0x11        | 主→电    | Set NodeID+EPP          | uint32+commit          | —     | 烧写 `canNodeId`                                               | `:146`                              |
| 0x12        | 主→电    | Set I-Limit+EPP         | float+commit           | A     | **同时刷新** `stallCurrentThreshold = ratedCurrent×40%`          | `:151`                              |
| 0x13        | 主→电    | Set V-Limit+EPP         | float+commit           | r/s   | ratedVelocity                                                | `:159`                              |
| 0x14        | 主→电    | Set Accel+EPP           | float+commit           | r/s²  | ratedVelocityAcc + tracker.SetVelocityAcc                    | `:167`                              |
| 0x15        | 主→电    | Apply Home              | —                      | —     | **v3.2 修复**：同步 `realPosition`                                | `:177`                              |
| 0x16        | 主→电    | Set Auto-Enable+EPP     | uint32                 | —     | `enableMotorOnBoot`                                          | `:183`                              |
| **0x17-1A** | 主→电    | **Set DCE Kp/Kv/Ki/Kd** | int32+commit           | —     | DCE PID 写入                                                   | `:188-210`                          |
| **0x1B**    | 主→电    | **Stall-Protect RAM**   | uint32                 | —     | **仅 RAM 切换**，不写 EEPROM                                       | `:212`（**35/42/57 独有，gripper 不存在**） |
| 0x21        | 主→电    | Get Current             | —                      | A     | 回 float A                                                    | `:220`                              |
| 0x22        | 主→电    | Get Velocity            | —                      | r/s   | 回 float r/s                                                  | `:232`                              |
| 0x23        | 主→电    | Get Position            | —                      | 圈     | 回 float 圈 + finish 标志                                        | `:244`                              |
| 0x24        | 主→电    | Get Offset              | —                      | —     | 回 int32                                                      | `:257`                              |
| 0x25        | 主→电    | Get Temperature         | —                      | —     | 回 float                                                      | `:268`                              |
| **0x28-2B** | 主→电    | **Get DCE Kp/Kv/Ki/Kd** | —                      | —     | 回 int32×4                                                    | `:282-320`                          |
| **0x2C**    | 主→电    | **Get Acceleration**    | —                      | r/s²  | 回 float                                                      | `:323`                              |
| **0x2D**    | 主→电    | **Get Current-Limit**   | —                      | A     | 回 float                                                      | `:338`                              |
| **0x5A**    | 触发电→他机 | **STALL Broadcast**     | data[0]=srcId, [1]=cmd | —     | cmd=1=TRIGGER / 2=DONE / 3=TIMEOUT；触发 LOCKED + ClearIntegral | `:371`                              |
| **0x5B**    | 主→ALL  | **UNLOCKED Broadcast**  | —                      | —     | LOCKED→IDLE + ResetGoals                                     | `:391`                              |
| **0x5C**    | 主→单电   | **Stall Query**         | data[0]=1(en)/2(lock)  | —     | 回 `data[1]=value`                                            | `:412`                              |
| 0x7D        | 主→电    | Enable Temp Watch       | —                      | —     | TIM1 开始 1Hz 采样                                               | `:353`                              |
| 0x7E        | 主→电    | Erase Configs           | —                      | —     | 恢复出厂 + SystemReset                                           | `:431`                              |
| 0x7F        | 主→电    | Reboot                  | —                      | —     | SystemReset                                                  | `:434`                              |


**对比旧版**：旧版只有 `0x01-0x07` + `0x11-0x16` + `0x21-0x25` + `0x7D-0x7F`。**新版新增 11 条**（高亮）。

### 2.2 物理参数默认值（35 / 42 / 57 / gripper 差异）


| 参数                      | 35           | 42         | **57（地轨）**      | **gripper（夹爪）**      | 旧 35         |
| ----------------------- | ------------ | ---------- | --------------- | -------------------- | ------------ |
| `currentLimit`          | **2 A**      | **2 A**    | **2.5 A**       | **1.2 A**            | 1 A          |
| `velocityLimit`         | 30 r/s       | 30 r/s     | 30 r/s          | 30 r/s               | 30 r/s       |
| `velocityAcc`（首次烧录）     | **30 r/s²**  | 30 r/s²    | 30 r/s²         | 30 r/s²              | **100 r/s²** |
| `ratedCurrentAcc`       | 2 000 mA/s   | 2 000 mA/s | **50 000 mA/s** | 2 000 mA/s           | 2 000 mA/s   |
| `caliCurrent`           | 2 A          | 2 A        | 2 A             | **1.2 A**            | 2 A          |
| **DCE Kp**              | 195          | **200**    | **300**         | 195                  | 195          |
| **DCE Kv**              | 80           | 80         | **120**         | 80                   | 80           |
| **DCE Ki**              | 300          | 300        | **150**         | 300                  | 300          |
| **DCE Kd**              | 250          | 250        | **50**          | 250                  | 250          |
| **PID kp/kd/ki**（电机端）   | 5/0/30       | 5/0/30     | **15/10/80**    | 5/0/30               | 5/0/30       |
| `enableStallProtect` 默认 | **true 硬强制** | true       | true            | **不存在（gripper 无堵转）** | **false**    |


数据来源：`motor_fw_f103_XX/UserApp/main.cpp:33-46`（首次烧录块）。

### 2.3 堵转检测 v3 协议（35/42/57 共用，gripper 无）

**算法位置**：`Ctrl/Motor/motor.cpp:171-300`


| 阶段                | 规则                                                               | 行号         |
| ----------------- | ---------------------------------------------------------------- | ---------- |
| 启动豁免期             | 进入 POSITION 模式后 100 ms 不检测（2 000 cycle @ 20 kHz）                 | `:181-186` |
| 电流阈值              | `|focCurrent| > ratedCurrent × 40%`                              | `:193,225` |
| 速度阈值              | `|estVelocity| < 50 step/s`                                      | `:192,224` |
| 误差阈值              | `|estError| > 15` 步                                              | `:193,224` |
| 上升沿计时             | 三条件首次同时满足 → `stallStartTick=50us`                                | `:215-220` |
| 30 ms 二次验证        | 电流仍 > 60% 阈值 + 误差仍超 12 步                                         | `:225-240` |
| **400 ms 兜底强制触发** | 即使 B/C 失败也强制 RETREATING                                          | `:268-281` |
| 三条件丢失             | 仅 `|estVelocity|>150` 才清上升沿（容忍低速抖动）                              | `:284-291` |
| **回退距离**          | 关节 **35 556 步**（≈1/4 圈 1.8°×256）；地轨 **51 200 步**（≈5 mm）          | `:243,273` |
| 回退速度              | 50 000 step/s，acc 100 000 step/s²                                | `:247-248` |
| 完成判定              | RETREATING 完成 → `STALL_LOCKED + ClearIntegral + 广播 cmd=2 (DONE)` | `:317-324` |
| 2 s 超时            | → `STALL_LOCKED + 广播 cmd=3 (TIMEOUT)`                            | `:329-334` |
| LOCKED 维持         | 主动 `CalcDceToOutput(estPosition, 0)` 维持位置（**不动**）                | `:307-313` |


**TIM1 100 Hz 堵转广播**（`UserApp/main.cpp:74-130`）：

- 每 250 ms 周期广播 RETREATING 状态
- 事件驱动广播 cmd=1/2/3



### 2.4 gripper 固件独有

- **无堵转保护**（`motor.cpp` 既无 Stall Detection 也无 RETREATING handler）
- `interface_can.cpp` 不实现 `0x1B / 0x5A / 0x5B / 0x5C`
- `CalcDceToOutput` 力矩限制（夹取力控制）：注释 "DCE 输出受 ratedCurrent 限制 … ratedCurrent 可通过 0x12 动态修改"
- `stockpile_f103cb.c` 多了 3 个函数（P0-10 测试夹具）：`Stockpile_Flash_Data_Read_Data16` / `_Verify` / `_WriteComplete`

---



## 三、主控固件（STM32F405）新增功能



### 3.1 ASCII 命令协议全清单



#### 3.1.1 系统控制（`!` 前缀）


| 命令                              | 行号                        | 实现状态                         |
| ------------------------------- | ------------------------- | ---------------------------- |
| `!START`                        | ascii_protocol.cpp:96-100 | ✅ 已实现                        |
| `!DISABLE`                      | `:121-125`                | ✅（**v3.2 失能短路 IsMoving**）    |
| `!HOME`                         | `:101-106`                | ✅ + **30s 超时兜底**             |
| `!RESET`（REST_POSE 待机）          | `:107-112`                | ✅ + **30s 超时兜底**             |
| `!STOP`                         | `:48-52`                  | ✅（**CAN 0x09 ALL 代理**）       |
| `!CALIBRATION`                  | —                         | ❌ **已删**（TODO CLEANUP-2 待清理） |
| `!HAND_EN` / `!HAND_DIS`        | `:240-251`                | ✅                            |
| `!HAND_POS <0-100>`             | `:252-270`                | ✅                            |
| `!HAND_O` / `!HAND_C`           | `:277-284`                | ✅                            |
| `!HAND_ZERO`                    | `:131-138`                | ✅                            |
| `!STALL_EN` / `!STALL_DIS`      | `:189-202`                | ✅                            |
| `!STALL_STATUS`                 | `:163-175`                | ⚠️ **半成品**：响应硬编码（BUG-3）      |
| `!STALL_UNLOCK`                 | `:176-181`                | ✅（广播 UNLOCK 3 次）             |
| `!RGB_SET_START/ENABLE/DISABLE` | `:57-86, 222-243`         | ✅（持久化）                       |
| `!RGB_BRIGHT [<0-100>] [&]`     | `:204-220`                | ✅（**查询 / 设置 / 持久化**）         |
| `!RGB_MODE <0-9>`               | `:268-279`                | ✅                            |
| `!RGB_COLOR <idx> <r> <g> <b>`  | `:280-299`                | ✅（0-2 索引静态色）                 |
| `!PRINTPOSE`                    | `:295-313`                | ✅                            |




#### 3.1.2 查询 / 参数（`#` 前缀）


| 命令                                    | 行号         | 备注                        |
| ------------------------------------- | ---------- | ------------------------- |
| `#GETJPOS`                            | `:330-340` | ✅ **7 值**（+地轨 mm）         |
| `#GETLPOS`                            | `:341-349` | ✅（**读缓存，不再 FK 重算**）       |
| `#SET_DCE_KV` / `_KP` / `_KI` / `_KD` | `:350-419` | ✅ **1~8 节点**              |
| `#SET_PID <node> kp kv ki kd`         | `:420-446` | ✅（批量）                     |
| `#GET_PID <node>`                     | `:447-475` | ✅（**合并 0x28~0x2B 回包**）    |
| `#REBOOT`                             | `:476-481` | ✅（无参 = ALL 代理 + 主控 reset） |
| `#REBOOT <1~7>`                       | `:482-498` | ✅                         |
| `#CMDMODE <1-5>`                      | `:499-506` | ✅（+TORQUE，6=SERVO_J 隐式）   |
| `#OFFSET_J <1-6|all>`                 | `:507-549` | ✅（**替代 !CALIBRATION**）    |
| `#SYNC_ACC`                           | `:550-557` | ✅（**8 电机 0x2C 查询**）       |
| `#ACC_J <node> <r/s²> [&]`            | `:558-593` | ✅（**带持久化**）               |
| `#SPEED_J <node> <r/s>`               | `:594-608` | ✅                         |
| `#I_LIMIT_J <node> <A> [&]`           | `:609-640` | ✅（**带持久化**）               |
| `#GETJACC <node>`                     | `:641-685` | ✅（**同步查询，时间戳**）           |
| `#GETI <node>`                        | `:686-732` | ✅                         |




#### 3.1.3 运动指令（`>` `&` `@` `$`）


| 命令            | 参数                                        | 行号         | 备注                            |
| ------------- | ----------------------------------------- | ---------- | ----------------------------- |
| `>` MoveJ 阻塞  | **8 字段**：j1~j6 + j7(rail mm) + speed(r/s) | `:737-756` | ✅ 含地轨                         |
| `&` MoveJ 非阻塞 | 同 `>`                                     | `:737-756` | ✅ 含地轨                         |
| `@` MoveL 直线  | 6-7 字段：x,y,z,a,b,c[,speed]                | `:757-776` | ⚠️ **地轨算法未实现**（注释："地轨位置保持不变"） |
| `$` 力矩透传      | **7 字段**：c1~c6 + c7(rail)                 | `:780-783` | ✅                             |




### 3.2 单位统一（v2.6 重构）


| 量       | 旧版                | 新版            | 转换                                                |
| ------- | ----------------- | ------------- | ------------------------------------------------- |
| 关节角度    | 度                 | **度**         | 与电机端无转换（`ctrl_step.cpp:111-114` 内做 °→圈×reduction） |
| 地轨位置    | —                 | **毫米 mm**     | 主控 ↔ 电机：mm / 5 = 圈（5mm/圈，丝杆 1605）                 |
| 夹爪开度    | —                 | **百分比 0~100** | pos0 = HAND_ZERO 时位置，pos100 = 正方向 100 圈           |
| **速度**  | °/s（关节）           | **电机轴 r/s**   | slider × `SLIDER_TO_RPS` (0.30)                   |
| **加速度** | 百分比 × `Joint6D_t` | **统一 r/s²**   | 与 CAN 0x14 入参一致                                   |
| 力矩      | —                 | **A（安培）**     | 与 CAN 0x03 / 0x12 入参一致                            |


**关键常量**（`dummy_robot.h`）：


| 常量                           | 值                         | 行号                           |
| ---------------------------- | ------------------------- | ---------------------------- |
| `RAIL_STEPS_PER_MM`          | 40960                     | `:201`（200步 × 1024 微步 / 5mm） |
| `SLIDER_TO_RPS`              | 0.30                      | `:18`                        |
| `AXIS_MAX_RPS[8]`            | 全部 30                     | `:25-33`                     |
| `MOTOR_REDUCTION[8]`         | `{1,50,50,50,50,50,30,1}` | `:38`（index 7 = 地轨直连）        |
| `REST_POSE`                  | `{0,-75,180,0,0,0}`       | `:286`                       |
| `HOME_POSE`                  | `{0,0,0,90,0,0}`          | —                            |
| `DEFAULT_JOINT_SPEED`        | 80                        | `:288`                       |
| `DEFAULT_JOINT_ACCELERATION` | 150                       | `:295`                       |
| `ALL_FINISH_FLAG`            | 0x7E                      | `:307`                       |




### 3.3 motorJ[] 8-slot 架构


| slot        | 节点          | nodeID | reduction | 限位              |
| ----------- | ----------- | ------ | --------- | --------------- |
| `motorJ[0]` | ALL 广播代理    | 0      | 1         | ±180            |
| `motorJ[1]` | J1 基座       | 1      | 50        | ±175°           |
| `motorJ[2]` | J2 肩部       | 2      | 50        | -75~90°         |
| `motorJ[3]` | J3 肘部       | 3      | 50        | 0~180°          |
| `motorJ[4]` | J4 腕 1      | 4      | 50        | ±270°           |
| `motorJ[5]` | J5 腕 2      | 5      | 50        | ±100°           |
| `motorJ[6]` | J6 腕 3      | 6      | 30        | ±180°           |
| `motorJ[7]` | **J7 / 地轨** | **7**  | **1（直连）** | **-250~250 mm** |
| `hand`      | 夹爪          | 8      | 16        | -100~100        |


位置：`dummy_robot.cpp:153-164`

### 3.4 控制算法（`dummy_robot.cpp`）


| 算法                                 | 行号                         | 实现状态                                  |
| ---------------------------------- | -------------------------- | ------------------------------------- |
| **7 轴同步抵达**（`ComputeSyncSpeeds()`） | `:36-117`                  | ✅ 已完整实现（迭代收敛梯形曲线）                     |
| MoveJ（含 7 轴同步）                     | `:282-321`                 | ✅                                     |
| MoveL（IK → MoveJ）                  | `:323-358`                 | ⚠️ 基本实现，**地轨算法未独立规划**                 |
| **SERVO_J**（5kHz 实时环随动）            | `:359-394`                 | ✅                                     |
| **TORQUE_CONTROL**（`$` 透传 7 通道电流）  | `:1011-1029`               | ✅                                     |
| TUNING（扫频正弦 `TuningHelper::Tick`）  | `:941-953`                 | ✅                                     |
| 编码器归零（`#OFFSET_J`）                 | ascii_protocol.cpp:507-549 | ✅（v3.2 删除乐观同步）                        |
| Homing / Resting 30s 超时            | `:580-655`                 | ✅                                     |
| **EmergencyStop（v3.2 修复）**         | `:748-769`                 | ✅（CAN 0x09 ALL 代理 + 抱闸 + 短路 IsMoving） |




### 3.5 FreeRTOS 任务架构


| 任务                         | 优先级          | 频率                     | 栈          | 行号                         |
| -------------------------- | ------------ | ---------------------- | ---------- | -------------------------- |
| `commTask`                 | Normal       | 1000ms 心跳              | **45 000** | communication.cpp:14-24    |
| `UsbDeferredInterruptTask` | —            | `sem_usb_irq`          | —          | communication.cpp:46-58    |
| `UsbServerTask`            | Normal       | `sem_usb_rx`           | 2000       | interface_usb.cpp:155-176  |
| `UartServerTask`           | Normal       | 1ms 轮询                 | 2000       | interface_uart.cpp:130-180 |
| `ControlLoopFixUpdateTask` | **Realtime** | **5 kHz (TIM7 200μs)** | 2000       | main.cpp:80-168            |
| `ControlLoopUpdateTask`    | Normal       | 阻塞于队列                  | 2000       | main.cpp:174-182           |
| `OledTask`                 | Normal       | 空循环                    | 2000       | main.cpp:188-256           |
| `RGBTask`                  | Normal       | 30ms osDelay           | 2000       | main.cpp:268-280           |


**5kHz 实时环各模式行为**：


| 模式                  | 下发频率                                        |
| ------------------- | ------------------------------------------- |
| SEQ/INT/TRJ (1/2/3) | **50 Hz**（`%100==0`）                        |
| SERVO_J (6)         | **5 kHz**（满频）                               |
| TUN (4)             | **100 Hz**                                  |
| TRQ (5)             | **100 Hz**                                  |
| 失能态                 | 100Hz UpdateJointAngles + UpdateJointPose6D |




### 3.6 RGB 灯效（`Bsp/gpio/rgb.cpp/.h`）


| 模式  | 名称                             | 行号            |
| --- | ------------------------------ | ------------- |
| 0   | `PURE_COLOR_0`（默认蓝 0,0,255）    | rgb.cpp:25-28 |
| 1   | `PURE_COLOR_1`（默认青 0,255,255）  | `:29-32`      |
| 2   | `PURE_COLOR_2`（默认品红 255,0,255） | `:33-36`      |
| 3   | `RAINBOW`（256 色环旋转）            | `:38-41`      |
| 4   | `BLUE_TIDE`（cos 波蓝色潮汐）         | `:42-45`      |
| 5   | `WHITE_BREATH`（exp(sin) 呼吸）    | `:46-49`      |
| 6   | `CYBER_BREATH`（青/紫呼吸）          | `:50-53`      |
| 7   | `RED_HEARTBEAT`（两段心跳）          | `:54-57`      |
| 8   | `GREEN_SPIN`（暗绿背景 + 3 灯头旋转）    | `:58-61`      |
| 9   | `BLINK`（多阶段呼吸 + 闪）             | `:62-65`      |
| 99  | `ALL_OFF`                      | `:66-72`      |


**关键能力**：


| 功能                    | 行号                          | 说明                                                        |
| --------------------- | --------------------------- | --------------------------------------------------------- |
| **静态色自定义**            | rgb.hpp:53-55               | `static_r/g/b[3]`，`!RGB_COLOR idx r g b` 配置               |
| **全局亮度查询 / 设置 / 持久化** | ascii_protocol.cpp:204-220  | `!RGB_BRIGHT [<0-100>] [&]`                               |
| **亮度渐变开关**            | rgb.cpp:84-99（`FadeStep`）   | `brightness` 当前 → `targetBrightness` 步进 `fadeSpeed=0.015` |
| **DMA 非阻塞**           | rgb.cpp:14-44（`UpdateLoop`） | `osEventFlagsId_t dmaEvent` + `dmaBusy` 标志                |
| **首次发送豁免**            | rgb.cpp:18                  | 首帧不等待 DMA                                                 |
| **WS2812 保护**         | rgb.hpp:131-138             | 上一帧 DMA 未完成则跳过本帧                                          |
| 极光（Aurora）            | —                           | ❌ **未实现**（TODO FEAT-2 待开发）                                |
| 参数化预设槽位               | —                           | ❌ **未实现**（TODO FEAT-2 待开发）                                |


---



## 四、增量总览（按维度）


| 维度           | 净增代码量                                 | 关键证据                              |
| ------------ | ------------------------------------- | --------------------------------- |
| ASCII 命令     | 154 → 927 行（+773，+500%）               | ascii_protocol.cpp                |
| CAN 处理 cmd 数 | 2 → 12（+10）                           | can_protocol.cpp                  |
| motorJ 节点数   | 7 → 8（+地轨）                            | dummy_robot.cpp:153-164           |
| FreeRTOS 任务数 | 4 → 7（+UsbServer/UartServer/CommTask） | main.cpp + interface_usb/uart.cpp |
| 电机固件 build   | 1 → 4（+57 + gripper）                  | firmware 顶层                       |
| 堵转协议         | 1s 单调累加 → 5条件 v3 协议                   | motor.cpp:171-300                 |
| RGB 灯效       | 7 种 → 10 种 + 静态色 + 亮度                 | rgb.hpp/cpp                       |
| 速度 / 加速度单位   | °/s + 百分比 → **统一 r/s / r/s²**         | v2.6 重构                           |


---



## 五、当前已发现但未修复的 BUG


| BUG                                                   | 严重度   | 现状                                |
| ----------------------------------------------------- | ----- | --------------------------------- |
| `!STALL_STATUS` **响应硬编码**（ascii_protocol.cpp:163-175） | 🔴 P0 | 串口助手匹配失败，Bug 在 [BUG-3]            |
| `!STOP` **电机无响应**（`BroadcastEStop` 0x89 编码错）          | 🔴 P0 | 已在 [BUG-ESTOP]，但实际**已改用 0x09 修复** |
| `!HOME/!RESET` **中** `!STOP` **无效**                   | 🔴 P0 | 依赖 v3.2 修复 + IsMoving 短路          |
| OLED `*`/`_` 不更新                                      | 🔴 P0 | 退化（详见 A1 修复文档）                    |
| 发送就卡死                                                 | 🔴 P0 | **根因未定位**（9-16 心跳定位方案待实施）         |
| `!BACKHOME` **统一回位**                                  | 🟡    | 未实现（TODO FEAT-1）                  |
| 极光 RGB                                                | 🟡    | 未实现（TODO FEAT-2）                  |
| CAN 0x7C 旧堵转兼容代码                                      | 🟡    | 待清理（CLEANUP-1）                    |
| `MoveL` 地轨算法                                          | 🟡    | 未实现                               |


---



## 六、cherry-pick 优先级建议

按"风险低 + 价值高"排序：


| 优先级    | 功能                                                                                                               | 风险                            | 价值      |
| ------ | ---------------------------------------------------------------------------------------------------------------- | ----------------------------- | ------- |
| **P0** | **单位统一**（v2.6 r/s, r/s²）+ `motorJ[7]` 8-slot + 地轨 ID=7                                                           | 低（纯数据结构）                      | 高       |
| **P0** | **ASCII:** `!HAND_`***、**`!HAND_ZERO`                                                                            | 低（独立子系统）                      | 高       |
| **P0** | `#OFFSET_J` 替代 `!CALIBRATION`                                                                                    | 低（接口重构）                       | 中       |
| **P0** | **CAN 0x09 BroadcastEStop**（替代 0x89）+ `EmergencyStop` 改造                                                         | 中（依赖电机固件同步改 0x09）             | 高（必修）   |
| **P1** | **电机固件 0x12/0x14/0x17-1A 写入 + 0x28-2D 查询 +** `#ACC_J`**/**`#SPEED_J`**/**`#I_LIMIT_J`**/**`#GETJACC`**/**`#GETI` | 中（电机固件同步升级）                   | 高（解锁调参） |
| **P1** | **7 轴同步抵达** `ComputeSyncSpeeds`                                                                                  | 中（算法改动）                       | 高       |
| **P1** | **SERVO_J** + **TORQUE_CONTROL** + **TUNING**                                                                    | 中（实时环依赖）                      | 中       |
| **P1** | **电机固件堵转 v3**（0x5A/0x5B/0x5C + 0x1B）                                                                             | **高**（需修 [BUG-ESTOP] 0x89 问题） | 高       |
| **P2** | **RGB 10 灯效 + 静态色 + 亮度 + 渐变 + DMA 非阻塞**                                                                          | 低                             | 中       |
| **P2** | `!RGB_SET_START/ENABLE/DISABLE` 持久化                                                                              | 低                             | 低       |
| **P2** | **57 电机独立 build**（与 35/42/gripper 算法一致，仅默认参数不同） | 中 | 中 |
| **P2** | **gripper 独立 build** | 中 | 中 |
| **P3** | **MoveL 地轨算法**                                                                                                   | 高（新算法）                        | 中       |
| **P3** | **极光 RGB** + **预设槽位**                                                                                            | 低                             | 低       |


---



## 七、待确认决策点


| #   | 问题                                                                           | 建议                                 |
| --- | ---------------------------------------------------------------------------- | ---------------------------------- |
| 1   | 地轨 ID 是 7（不是 9），文档 `README.md` 要不要同步修正？                                      | ✅ 建议改                              |
| 2   | cherry-pick 顺序按上表 P0 → P1 → P2 → P3？                                         | ✅ 建议是                              |
| 3   | 堵转 v3 协议（0x5A/0x5B/0x5C）依赖主控 + 电机同步升级，迁移前是否要先修 [BUG-ESTOP] 的 0x09/0x89 编码问题？ | ✅ 必须先修                             |
| 4   | `!STALL_STATUS` 硬编码响应（BUG-3）要不要在新基线里一并修？                                     | ✅ 建议一起修（迁移时一起改 `can_protocol.cpp`） |


---



## 八、参考文档


| 文档                                            | 用途              |
| --------------------------------------------- | --------------- |
| `docs/fixbug/主控固件退化修复—全面重构需求.md`              | A 组 7 项 P0 退化清单 |
| `docs/fixbug/63b4b16版本-5问题形式分析—2026-09-16.md` | 中间版本的 5 个问题形式   |
| `docs/fixbug/发送就卡死—心跳定位排查方案—2026-09-16.md`    | 心跳定位方案          |
| `docs/TODO.md`                                | FEAT/BUG 待开发项   |
| `docs/README.md`                              | 项目主文档           |


