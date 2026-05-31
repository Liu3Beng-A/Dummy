# Dummy 机械臂固件问题追踪与待办

> 生成时间: 2026-05-30
> 状态: P0-1, P0-11 已修复 (2026-05-30/31)，其余待修复

---

## 目录

- [P0 - 上电前必须修复 (安全/功能关键)](#p0---上电前必须修复-安全功能关键)
- [P1 - 下一迭代修复](#p1---下一迭代修复)
- [P2 - 计划中版本](#p2---计划中版本)
- [P3 - 长期改进](#p3---长期改进)

---

## P0 - 上电前必须修复 (安全/功能关键)

### P0-1: 电机驱动失步检测逻辑错误

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `Ctrl/Motor/motor.cpp` |
| **位置** | L265 |
| **严重性** | Critical |
| **问题** | 失步检测使用 `current == config.motionParams.ratedCurrent` (精确相等)，由于FOC电流持续波动，该条件几乎永远不会满足，**失步保护实际完全无效** |
| **修复方案** | ✅ 已修复: 改为 `current >= stallThreshold`，其中 `stallThreshold = ratedCurrent * 95 / 100` (int32_t整数运算) |
| **修复日期** | 2026-05-30 |

```cpp
// 修复后 (motor.cpp L265-272)
// 堵转: 电流达到额定值95%以上 + 速度极低，持续1秒
const int32_t stallThreshold = (int32_t)(config.motionParams.ratedCurrent * 95 / 100);
if (// Current Mode
    ((controller->modeRunning == MODE_COMMAND_CURRENT ||
      controller->modeRunning == MODE_PWM_CURRENT) &&
     (current != 0))
    || // Other Mode: current >= ratedCurrent * 0.95
    current >= stallThreshold)
```

> 注: `overloadFlag` 检测逻辑同样受影响，已一并修复为复用 `stallThreshold`

---

### P0-2: 主控制器力矩模式Disable后电流未清零

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/instances/dummy_robot.cpp` |
| **位置** | `SetEnable(false)` 函数 |
| **严重性** | Critical |
| **问题** | `SetEnable(false)` 设置 `isEnabled=false` 但不清零 `targetCurrents[]` 和 `targetRailCurrent`，力矩模式最后一条电流命令持续生效，存在安全隐患 |
| **修复方案** | 在 `SetEnable(false)` 中添加 `memset(targetCurrents, 0, sizeof(targetCurrents))` 和 `targetRailCurrent = 0` |

---

### P0-3: CAN中断回调与5kHz控制循环数据竞争

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `UserApp/protocols/can_protocol.cpp` |
| **位置** | `OnCanMessage` 函数 L88/L105/L122 |
| **严重性** | Critical |
| **问题** | `OnCanMessage` (CAN RX中断/DMA上下文) 调用 `UpdateJointAnglesCallback` 修改 `currentJoints[]` 和 `jointsStateFlag`，而 `ThreadControlLoopFixUpdate` (5kHz) 同时读写这些变量，**无互斥保护，数据竞争确定发生** |
| **修复方案** | CAN回调只将数据复制到FreeRTOS队列，由控制循环任务消费；或使用 `taskENTER_CRITICAL()/taskEXIT_CRITICAL()` 临界区 |

---

### P0-4: Python工具力矩命令格式错误

| 属性 | 值 |
|------|-----|
| **模块** | Python工具 |
| **文件** | `串口助手.py` |
| **位置** | L893 `send_torque()` 函数 |
| **严重性** | Critical |
| **问题** | `$` 命令发送 float 格式（如 `0.00`）但固件协议要求 **整数mA**，实际力矩只有预期的 **千分之一** |
| **修复方案** | 将 `f"{val:.2f}"` 改为 `f"{int(val * 1000)}"` (A→mA整数) |

```python
# 当前代码 (错误)
cmd = f"${torques[0]:.2f},{torques[1]:.2f},..."

# 修复为
cmd = f"${int(torques[0]*1000)},{int(torques[1]*1000)},..."
```

---

### P0-5: 电机驱动急停不制动

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `UserApp/protocols/interface_can.cpp` |
| **位置** | L250 `CAN_CMD_EMERGENCY_STOP` 处理 |
| **严重性** | Critical |
| **问题** | 广播急停命令(0x89)设置STOP模式使电机滑行(coast)而非制动(brake)，安全性不足 |
| **修复方案** | 急停时调用 `SetBrake()` 而非 `SetStop()` |

---

### P0-6: 电机驱动sqrtf负数参数

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `Ctrl/Motor/motor.cpp` |
| **位置** | L449 |
| **严重性** | Critical |
| **问题** | `sqrtf(_time * _time - 4 * deltaPos / a)` 中表达式可能产生负数，`sqrtf` 负数为未定义行为(ARM CMSIS返回NaN) |
| **修复方案** | 使用 `sqrtf(fmaxf(val, 0.0f))` clamp到非负 |

---

### P0-7: 主控制器 `ApplyPositionAsHome` 未初始化CAN缓冲区

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/actuators/ctrl_step.cpp` |
| **位置** | L178 |
| **严重性** | Critical |
| **问题** | `canBuf` 数组在发送前未初始化，会发送垃圾数据到CAN总线 |
| **修复方案** | 发送前添加 `memset(canBuf, 0, sizeof(canBuf))` |

---

### P0-8: 主控制器 `SetEnable` 状态语义错误

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/actuators/ctrl_step.cpp` |
| **位置** | L22-24 |
| **严重性** | Critical |
| **问题** | 使能时设置 `state = FINISH`（已完成），语义错误，电机并非处于"完成"状态 |
| **修复方案** | 改为 `state = IDLE` 或新增 `ENABLED` 状态 |

---

### P0-9: 电机驱动无磁铁检测无处理

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `Ctrl/Sensor/Encoder/mt6816_base.cpp` |
| **位置** | L52 |
| **严重性** | Critical |
| **问题** | `noMagFlag`（磁铁脱落标志）被检测到但未触发任何错误处理，磁铁脱落后电机继续运行在错误的绝对位置反馈上，极度危险 |
| **修复方案** | 添加 `motor->Error(MOTOR_ERROR_NO_MAG)` 并停止电机输出 |

---

### P0-10: 电机驱动Flash写入后无验证

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `Ctrl/Sensor/Encoder/encoder_calibrator_base.cpp` |
| **位置** | L291-315 |
| **严重性** | Critical |
| **问题** | 编码器校准数据写入Flash后未读取验证，写入失败时静默继续运行 |
| **修复方案** | 写入后立即读取并逐字节比较，不匹配则报错重试 |

---

### P0-11: 电机堵转后无主动通知

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_35/42/57 + ref_core_f405 |
| **文件** | `Ctrl/Motor/motor.cpp` / `UserApp/protocols/can_protocol.cpp` |
| **位置** | `CloseLoopControlTick()` L278, `OnCanMessage()` 0x7C 处理 |
| **严重性** | Critical |
| **问题** | 电机堵转后仅切换STATE_STALL+LED闪烁，主控完全不知晓；errorCode字段虽存在但主控从未读取；且堵转后电机Sleep导致无锁力 |
| **修复方案** | ✅ 已修复 (2026-05-31): |
| | 1. 电机堵转时主动发送CAN帧 `(nodeID<<7)\|0x7C`，Data[0]=nodeID, Data[1]=1 |
| | 2. 主控收到0x7C后调用 `SetStallMode()`，切换RGB为红色心跳、停发新指令、清空队列 |
| | 3. ENABLE命令(0x01, data=1)清除电机isStalled标志，恢复正常 |
| **修复日期** | 2026-05-31 |
| **改动文件** | `motor_fw_f103_35/42/57/Ctrl/Motor/motor.cpp`, `motor_fw_f103_35/42/57/UserApp/protocols/interface_can.cpp`, `ref_core_f405/UserApp/protocols/can_protocol.cpp`, `ref_core_f405/Robot/actuators/ctrl_step.hpp/.cpp`, `ref_core_f405/Robot/instances/dummy_robot.hpp/.cpp` |

---

## P1 - 下一迭代修复

### P1-1: 主控制器IK求解器永远返回true

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/algorithms/kinematic/6dof_kinematic.cpp` |
| **位置** | L498 `SolveIK` |
| **严重性** | High |
| **问题** | 即使所有8组IK解都超出限位或无解，`SolveIK` 仍返回 `true`，调用方无法区分IK成功与失败 |
| **修复方案** | 当所有解均无效时返回 `false`，由 `MoveL` 处理失败情况 |

---

### P1-2: 主控制器命令队列满时返回255

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/instances/dummy_robot.cpp` |
| **位置** | `CommandHandler::Push` L592-601 |
| **严重性** | High |
| **问题** | 队列满时 `osMessageQueuePut` 返回 `osErrorResource`，函数返回255作为"剩余空间"，客户端无法区分队列满和队列有255空间的真实状态 |
| **修复方案** | 队列满时返回 `-1` 或 `0`，明确表示入队失败 |

---

### P1-3: 主控制器地轨单位混淆

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/instances/dummy_robot.cpp` |
| **位置** | `MoveRailRelative` L197-200 |
| **严重性** | High |
| **问题** | 地轨限位值(-250, 250)是毫米，但 `motorJ[0]->angleLimitMax/Min` 以电机转数(revolutions)为单位存储，比较的是不同单位 |
| **修复方案** | 在比较前统一转换为毫米：`targetRailPos * STEPS_PER_MM` 或限位值转为转数 |

---

### P1-4: 主控制器 `SetEnable` 不清零目标电流

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/instances/dummy_robot.cpp` |
| **位置** | `SetEnable` 函数 |
| **严重性** | High (与P0-2重复但更细) |
| **问题** | 同P0-2，在 CtrlStep 层也需要确保电流为零 |
| **修复方案** | 见 P0-2 |

---

### P1-5: 主控制器 `CommandHandler::ParseCommand` 忙等

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/instances/dummy_robot.cpp` |
| **位置** | L692-693 |
| **严重性** | High |
| **问题** | `while (context->IsMoving())` 内 `osDelay(5)` 轮询运动完成状态，浪费CPU且增加命令响应延迟 |
| **修复方案** | 改用 FreeRTOS 事件组(`EventGroup`)或二进制信号量，电机完成时置位，命令任务阻塞等待 |

---

### P1-6: 主控制器 MPU6050 连接检测寄存器值错误

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Bsp/imu/MPU6050.cpp` |
| **位置** | L76-77 |
| **严重性** | High |
| **问题** | `testConnection()` 检查 `getDeviceID() == 0x34`，但MPU6050的WHO_AM_I寄存器实际值为 **0x68**，`testConnection` 永远返回false |
| **修复方案** | 改为 `0x68` |

---

### P1-7: 主控制器 `SetEnable` 对 CtrlStep 层状态语义错误

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/actuators/ctrl_step.cpp` |
| **位置** | L22-24 |
| **严重性** | High (与P0-8重复但更细) |
| **问题** | 同P0-8 |
| **修复方案** | 见 P0-8 |

---

### P1-8: 主控制器 `absMaxOf6` 初始值错误

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/instances/dummy_robot.cpp` |
| **位置** | `absMaxOf6` helper函数 L18-28 |
| **严重性** | High |
| **问题** | 初始值设为 `-1`，如果所有关节角度都在 -1° 到 1° 之间，返回值为负数，导致速度缩放因子为负 |
| **修复方案** | 初始值改为 `-FLT_MAX` 或 `0.0f`，确保返回非负值 |

---

### P1-9: 电机驱动梯形轨迹除零崩溃

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `Ctrl/Motor/motion_planner.cpp` |
| **位置** | L408 |
| **严重性** | High |
| **问题** | 目标位置等于当前位置时，分母为零导致崩溃 |
| **修复方案** | 添加 `if (fabs(_goalPosition - positionNow) < 0.001f) return;` |

---

### P1-10: 电机驱动速度/位置积分溢出

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `Ctrl/Motor/motion_planner.cpp` |
| **位置** | L336, L370-372 |
| **严重性** | High |
| **问题** | `positionIntegral` 和 `estPositionIntegral` 无界累积，长时间运行后溢出 |
| **修复方案** | 添加积分限幅: `fmaxf(-MAX_INTEGRAL, fminf(value, MAX_INTEGRAL))` |

---

### P1-11: 主控制器 EepromConfig 无pack指令

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/instances/dummy_robot.h` |
| **位置** | L18-29 |
| **严重性** | High |
| **问题** | `EepromConfig` 结构体无 `#pragma pack(1)` 或 `__attribute__((packed))`，编译器可能插入填充字节，导致Flash读写出错 |
| **修复方案** | 添加 `#pragma pack(push, 1)` / `#pragma pack(pop)` 或等效属性 |

---

### P1-12: 主控制器 CAN发送使用static header非线程安全

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Bsp/interface_can.cpp` |
| **位置** | L51-59 |
| **严重性** | High |
| **问题** | `static txHeader` 被多个任务同时写入可能损坏，`CanSendMessage` 非可重入 |
| **修复方案** | 使用 FreeRTOS 互斥锁保护，或每个调用者提供独立的txHeader |

---

### P1-13: 主控制器 编码器校正中系统复位

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `Ctrl/Sensor/Encoder/encoder_calibrator_base.cpp` |
| **位置** | L382 |
| **严重性** | High |
| **问题** | 校准成功后立即 `HAL_NVIC_SystemReset()`，Flash写入可能未完成就复位 |
| **修复方案** | 延迟复位，等待Flash写入完成标志，或使用看门狗触发复位 |

---

### P1-14: 主控制器 CAN指针类型转换未对齐

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `UserApp/protocols/interface_can.cpp` |
| **位置** | L29-38 |
| **严重性** | High |
| **问题** | `*(uint32_t*)RxData` 和 `*(float*)RxData` 假设数据4字节对齐，ARM上不对齐访问导致HardFault |
| **修复方案** | 使用 `memcpy` 安全复制: `uint32_t v; memcpy(&v, RxData, 4);` |

---

### P1-15: 电机驱动 SPI无限等待

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `Port/mt6816_stm32.cpp` |
| **位置** | L14 |
| **严重性** | High |
| **问题** | `HAL_SPI_TransmitReceive()` 使用 `HAL_MAX_DELAY` 无限等待，20kHz控制循环中SPI挂起将导致系统完全挂死 |
| **修复方案** | 使用合理超时(如1ms)，SPI失败时跳过本次采样并记录错误 |

---

### P1-16: 主控制器 ServoJ首次调用dt计算问题

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/instances/dummy_robot.cpp` |
| **位置** | `ServoJ` 函数 L344-346 |
| **严重性** | High |
| **问题** | 首次调用时 `lastServoTime` 为0，`dt` 计算异常大或为负 |
| **修复方案** | 初始化 `lastServoTime = 0`，首次调用时跳过dt计算或设为固定值 |

---

### P1-17: 主控制器 IK奇异性阈值过小

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/algorithms/kinematic/6dof_kinematic.cpp` |
| **位置** | L239 |
| **严重性** | Medium |
| **问题** | 奇异性检测阈值 `0.000001` 在浮点运算中过小，近基点1微米误差就被判定为奇异 |
| **修复方案** | 使用基于问题规模的相对阈值: `FLT_EPSILON * max(1.0f, arm_length)` |

---

### P1-18: 电机驱动 advanced角补偿针对错误传感器

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `Ctrl/Motor/motor.cpp` |
| **位置** | L525-552 |
| **严重性** | High |
| **问题** | 代码注释明确说明需要为MT6816重新整定参数，但当前使用DPS系列传感器的速度阈值和系数，导致主动角补偿不准确 |
| **修复方案** | 通过实验数据重新测量MT6816的速度-角补偿曲线，修正阈值和系数 |

---

### P1-19: 电机驱动校正过程中直接系统复位

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **文件** | `Ctrl/Sensor/Encoder/encoder_calibrator_base.cpp` |
| **位置** | L382 |
| **严重性** | High |
| **问题** | 复位发生在控制循环中，未确保Flash写入完成，未给上位机通知的机会 |
| **修复方案** | 见 P1-13 |

---

### P1-20: Python工具 ServoJ线程不安全写串口

| 属性 | 值 |
|------|-----|
| **模块** | Python工具 |
| **文件** | `串口助手.py` |
| **位置** | L936 |
| **严重性** | Critical (与P0相关) |
| **问题** | `servoj_test_loop()` 直接写串口无锁，Tkinter串口非线程安全 |
| **修复方案** | 添加 `threading.Lock()`，在 `toggle_connection` 时获取锁写串口 |

---

### P1-21: Python工具 ServoJ命令缺少换行符

| 属性 | 值 |
|------|-----|
| **模块** | Python工具 |
| **文件** | `串口助手.py` |
| **位置** | L936 |
| **严重性** | High |
| **问题** | 直接发送 `cmd` 无 `+ "\n"`，固件ASCII解析器以换行符为命令分隔符，无法正确解析 |
| **修复方案** | 改为 `cmd + "\n"` |

---

### P1-22: Python工具 ServoJ参数硬编码不可调

| 属性 | 值 |
|------|-----|
| **模块** | Python工具 |
| **文件** | `串口助手.py` |
| **位置** | L922-929 |
| **严重性** | High |
| **问题** | 幅值、频率、测试关节全部硬编码，用户无法自定义测试参数 |
| **修复方案** | 在UI中添加 amplitude、frequency、joint_index 三个输入控件 |

---

### P1-23: Python工具关节角度无输入验证

| 属性 | 值 |
|------|-----|
| **模块** | Python工具 |
| **文件** | `串口助手.py` |
| **位置** | L875 `send_movej()` |
| **严重性** | High |
| **问题** | 用户可输入超出限位范围的值(如 J1: 500°)无任何警告或限制 |
| **修复方案** | 添加范围检查: `if not (MIN_ANGLE <= v <= MAX_ANGLE): showwarning()` |

---

### P1-24: Python工具 ServoJ停止不干净

| 属性 | 值 |
|------|-----|
| **模块** | Python工具 |
| **文件** | `串口助手.py` |
| **位置** | L661-669 `toggle_connection()` |
| **严重性** | High |
| **问题** | 断开连接时 `is_servoj_testing` 设为false但线程可能继续运行短暂时间，发送陈旧命令 |
| **修复方案** | 使用 `threading.Event` 显式等待线程结束: `self.servoj_stop_event.set()` + `join(timeout=1.0)` |

---

### P1-25: 主控制器 MPU6050过滤器状态跨禁用/启用不重置

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Bsp/imu/MPU6050.cpp` |
| **位置** | L3761-3780 |
| **严重性** | Medium |
| **问题** | 过滤器被禁用后重新启用，滤波器状态包含陈旧数据，导致重建后输出突变 |
| **修复方案** | 禁用时重置滤波器状态: `biquadFilterReset(&filter)` |

---

### P1-26: 主控制器 `IsMoving()` 地轨位检查逻辑含混

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `Robot/instances/dummy_robot.cpp` |
| **位置** | L540-543 |
| **严重性** | Medium |
| **问题** | `jointsStateFlag != 0b1111110` 的位掩码含义不清晰，bit 0(地轨)是否被纳入判断不明确 |
| **修复方案** | 明确定义位域: `#define JOINT_DONE(i) (1 << (i))` 并为地轨单独判断 |

---

### P1-27: 主控制器 SetEnable 漏掉地轨和夹爪

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | Robot/instances/dummy_robot.cpp |
| **位置** | SetEnable() 函数 |
| **严重性** | Critical |
| **问题** | 循环只处理 motorJ[1-6]，漏掉了 motorJ[0](地轨) 和 hand(夹爪)；导致 !ENABLE 无法使能夹爪，!DISABLE 无法失能地轨；!HAND_EN 直接调用 hand 所以正常 |
| **修复方案** | 已修复 (2026-05-31): 在循环后单独调用 motorJ[0]->SetEnable() 和 hand->SetEnable() |
| **修复日期** | 2026-05-31 |

---

## P2 - 计划中版本

### P2-1: 合并USB和UART4 ASCII命令解析器

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **文件** | `UserApp/protocols/ascii_protocol.cpp` |
| **严重性** | Medium |
| **问题** | `OnUsbAsciiCmd` 和 `OnUart4AsciiCmd` 各约530行几乎完全重复，维护成本极高 |
| **修复方案** | 提取公共解析逻辑为 `ParseCommandImpl(const char* cmd, OutputChannel out)`，两handler调用同一函数 |

---

### P2-2: MPU6050 IMU数据融合到控制循环

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | Medium |
| **问题** | MPU6050已正确驱动并在OLED显示，但5kHz控制循环完全未使用IMU数据 |
| **修复方案** | 集成IMU数据用于: 1) 地轨同步补偿 2) 碰撞检测基础 3) 自适应振动抑制 |

---

### P2-3: Python工具 添加轨迹录制与回放

| 属性 | 值 |
|------|-----|
| **模块** | Python工具 |
| **严重性** | Medium |
| **问题** | 当前无示教编程功能，用户无法录制关节轨迹并回放 |
| **修复方案** | 添加录制按钮，保存关节角度序列到CSV；回放时按时间戳重放 |

---

### P2-4: 主控制器 地轨同步补偿

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | Medium |
| **问题** | 机械臂移动时地轨可能因振动产生微小滑移，无同步补偿 |
| **修复方案** | 利用MPU6050数据检测机械臂姿态变化，动态补偿地轨目标位置 |

---

### P2-5: 电机驱动 PID参数自整定

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **严重性** | Medium |
| **问题** | PID参数需要手动整定，对非专业用户困难 |
| **修复方案** | 实现Ziegler-Nichols或频域自整定算法，自动计算Kv/Ki/Kp |

---

### P2-6: 主控制器 命令历史与别名

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | Low |
| **问题** | 无法保存常用命令序列为别名 |
| **修复方案** | 支持 `#ALIAS_SAVE name cmd1;cmd2;...` 保存，`#ALIAS_RUN name` 执行 |

---

### P2-7: Python工具 添加实时关节速度曲线显示

| 属性 | 值 |
|------|-----|
| **模块** | Python工具 |
| **严重性** | Low |
| **问题** | 用户无法直观看到关节速度变化 |
| **修复方案** | 使用matplotlib或canvas绘制实时速度曲线窗口 |

---

### P2-8: 主控制器 地轨行程限位开关支持

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | Medium |
| **问题** | 地轨只有软限位，硬件行程开关未使用 |
| **修复方案** | 添加GPIO中断检测行程开关，触发时立即停止地轨并报错 |

---

### P2-9: 主控制器 命令行宏支持

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | Low |
| **问题** | 无法执行预定义的运动序列 |
| **修复方案** | 支持 `#MACRO_DEF name cmd1,cmd2,...` 和 `#MACRO_RUN name` |

---

### P2-10: Python工具 添加连接状态监控

| 属性 | 值 |
|------|-----|
| **模块** | Python工具 |
| **严重性** | Medium |
| **问题** | 连接断开后无自动重连，状态指示不明确 |
| **修复方案** | 添加心跳检测，断开后自动重连(最多3次)；连接状态指示灯 |

---

### P2-11: 主控制器 CAN总线健康监测

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | Medium |
| **问题** | CAN总线错误(位填充错误、ACK错误等)无监测和报告 |
| **修复方案** | 利用STM32 CAN外设的错误中断，统计错误计数，超阈值时报警 |

---

### P2-12: 主控制器 多组位置目标队列

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | Medium |
| **问题** | 当前每次运动命令覆盖前一命令，无运动队列 |
| **修复方案** | 实现多组目标位置队列(如8组)，允许预填下一段轨迹 |

---

### P2-13: 电机驱动 增加温度保护

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **严重性** | Medium |
| **问题** | 温度数据已读取(`OnCanMessage` 0x25)但无实际保护动作 |
| **修复方案** | 添加温度阈值保护: 80°C降额运行，100°C强制停机 |

---

### P2-14: 主控制器 急停按钮双层确认

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | Medium |
| **问题** | OLED菜单中 `!STOP` 无二次确认，误触会导致运行中机械臂急停 |
| **修复方案** | `!STOP` 命令需要再次确认或长按触发 |

---

### P2-15: 电机驱动 编码器校正改善磁滞非线性

| 属性 | 值 |
|------|-----|
| **模块** | motor_fw_f103_all |
| **严重性** | Medium |
| **问题** | 当前16384-entry LUT只校正正向磁滞，未覆盖高速逆向运动 |
| **修复方案** | 实现双向速度相关校正表: LUT_Index = f(angle, velocity_direction) |

---

## P3 - 长期改进

### P3-1: ROS2 集成

| 属性 | 值 |
|------|-----|
| **模块** | 新增 |
| **严重性** | 长期 |
| **问题** | README声称支持ROS2 rviz/Gazebo仿真，但ros2_ws/目录完全为空 |
| **修复方案** | 实现ROS2硬件接口包(dummy_arm_bringup, dummy_arm_hardware, dummy_arm_description)，支持urdf/xacro描述 |

---

### P3-2: 碰撞检测与保护

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | 长期 |
| **问题** | README声称"实时碰撞检测保护机制"但代码中无任何碰撞检测逻辑 |
| **修复方案** | 实现: 1) 关节空间速度限制 2) 工作空间边界检测 3) 末端负载碰撞感应(利用电流突变) |

---

### P3-3: 夹爪力矩反馈与软停止

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 + motor_fw |
| **严重性** | 长期 |
| **问题** | 夹爪只有开闭位置命令，无力矩反馈监测，无软停止 |
| **修复方案** | 1) 夹爪电机也使用FOC驱动 2) 读取夹爪闭合电流 3) 电流突变检测来判断物体接触 4) 实现软停止 |

---

### P3-4: 示教编程与轨迹录制回放

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 + Python工具 |
| **严重性** | 长期 |
| **问题** | README声称"示教编程"但无相关代码 |
| **修复方案** | 1) 上位机录制关节序列+时间戳 2) 固件存储多个轨迹 3) 支持轨迹编辑(删除点/插入点) 4) 循环/条件执行 |

---

### P3-5: 速度前瞻与时间最优轨迹

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | 长期 |
| **问题** | 当前只有单段梯形速度规划，无多段轨迹前瞻 |
| **修复方案** | 实现S型速度曲线(S-curve)加减速+多段轨迹时间最优规划 |

---

### P3-6: 末端力矩传感器支持

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | 长期 |
| **问题** | README声称"支持外接力矩传感器"但无相关代码 |
| **修复方案** | 添加力矩传感器接口(模拟电压或CAN)，实现力控操作模式 |

---

### P3-7: 多机协作模式

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | 长期 |
| **问题** | README提及"多臂协作模式"但无多机通信或协作逻辑 |
| **修复方案** | 1) 多机CAN总线组网或Ethernet连接 2) 同步触发协议 3) 协作空间坐标共享 |

---

### P3-8: 视觉目标识别抓取

| 属性 | 值 |
|------|-----|
| **模块** | 新增(ESP32或PC) |
| **严重性** | 长期 |
| **问题** | README提及"集成视觉识别抓取"但无视觉处理代码 |
| **修复方案** | 使用ESP32-S3或PC处理摄像头图像，目标检测→深度估计→IK求解→抓取规划 |

---

### P3-9: 云端监控与OTA更新

| 属性 | 值 |
|------|-----|
| **模块** | 新增 |
| **严重性** | 长期 |
| **问题** | 无远程监控和固件OTA能力 |
| **修复方案** | 1) ESP32 WiFi连接 2) Web仪表盘显示实时状态 3) 固件版本管理+OTA差分升级 |

---

### P3-10: 碰撞后恢复运行

| 属性 | 值 |
|------|-----|
| **模块** | ref_core_f405 |
| **严重性** | 长期 |
| **问题** | README声称"碰撞检测后可恢复运行"但无碰撞恢复逻辑 |
| **修复方案** | 碰撞后记录位置，解除后可通过 `!RECOVER` 返回碰撞前姿态 |

---

---

## 问题统计

| 优先级 | Critical | High | Medium | Low | 合计 |
|--------|---------|------|--------|-----|------|
| P0 | 10 | 0 | 0 | 0 | **10** |
| P1 | 2 | 18 | 6 | 1 | **27** |
| P2 | 0 | 0 | 9 | 6 | **15** |
| P3 | 0 | 0 | 3 | 7 | **10** |
| **合计** | **12** | **18** | **18** | **14** | **61** |

---

## 修复进度

| ID | 描述 | 状态 | 修复日期 | 备注 |
|----|------|------|---------|------|
| P0-1 | 电机失步检测==改为>= | [x] | 2026-05-30 | motor_fw_f103_all/Ctrl/Motor/motor.cpp L265+288, 同时修复overloadFlag |
| P0-2 | 力矩模式Disable后清零电流 | [ ] | | |
| P0-3 | CAN中断数据竞争 | [ ] | | |
| P0-4 | Python力矩命令格式修正 | [ ] | | |
| P0-5 | 急停加制动 | [ ] | | |
| P0-6 | sqrtf负数参数 | [ ] | | |
| P0-7 | ApplyPositionAsHome未初始化 | [ ] | | |
| P0-8 | SetEnable状态语义错误 | [ ] | | |
| P0-9 | 无磁检测无处理 | [ ] | | |
| P0-10 | Flash写入无验证 | [ ] | | |
| P0-11 | 电机堵转主动通知+ENABLE清除 | [x] | 2026-05-31 | motor_fw_f103_35/42/57 motor.cpp+interface_can.cpp, ref_core_f405 can_protocol.cpp+ctrl_step+dummy_robot |
| P1-1 | IK永远返回true | [ ] | | |
| P1-2 | 命令队列满返回255 | [ ] | | |
| P1-3 | 地轨单位混淆 | [ ] | | |
| P1-4 | SetEnable不清零电流 | [ ] | | |
| P1-5 | ParseCommand忙等 | [ ] | | |
| P1-6 | MPU6050 ID错误 | [ ] | | |
| P1-7 | SetEnable状态语义错误(CtrlStep) | [ ] | | |
| P1-8 | absMaxOf6初始值-1 | [ ] | | |
| P1-9 | 梯形轨迹除零 | [ ] | | |
| P1-10 | 速度/位置积分溢出 | [ ] | | |
| P1-11 | EepromConfig无pack | [ ] | | |
| P1-12 | CAN发送非线程安全 | [ ] | | |
| P1-13 | 校正中系统复位 | [ ] | | |
| P1-14 | CAN指针转换未对齐 | [ ] | | |
| P1-15 | SPI无限等待 | [ ] | | |
| P1-16 | ServoJ首次dt异常 | [ ] | | |
| P1-17 | IK奇异性阈值过小 | [ ] | | |
| P1-18 | advanced角补偿参数错误 | [ ] | | |
| P1-19 | 校正中直接复位 | [ ] | | |
| P1-20 | ServoJ线程不安全 | [ ] | | |
| P1-21 | ServoJ无换行符 | [ ] | | |
| P1-22 | ServoJ参数硬编码 | [ ] | | |
| P1-23 | 关节角度无验证 | [ ] | | |
| P1-24 | ServoJ停止不干净 | [ ] | | |
| P1-25 | MPU6050过滤器状态不重置 | [ ] | | |
| P1-26 | IsMoving地轨位检查含混 | [ ] |
| P1-27 | SetEnable漏掉地轨夹爪 | [x] | 2026-05-31 | dummy_robot.cpp SetEnable() | | |
