# PROJECT_CONTEXT - Dummy 7轴机械臂控制系统

> **自动生成时间**: 2026-08-24
> **项目根目录**: `e:/Dummy-code`
> **生成方式**: 根据固件代码实际分析生成

---

## 项目简介

这是一个基于 **STM32F405 (主控) + 8×STM32F103 (电机驱动)** 的 **7轴机械臂 + 地轨 + 夹爪** 控制系统，运行 FreeRTOS 实时操作系统。

### 已实现功能

- 7轴关节空间运动 (MoveJ, blocking / non-blocking)
- 笛卡尔空间运动 (MoveL via 6-DOF FK/IK)
- 力矩控制模式 ($ 命令, mA电流)
- ServoJ 实时伺服模式
- 地轨线性滑轨控制 (mm单位)
- 夹爪控制 (开/闭/位置/力矩)
- 梯形速度规划
- 软限位保护 (每轴min/max)
- 双定时器FOC控制 (20kHz电流环)
- MT6816编码器自动校准 (16384-entry LUT)
- OLED状态显示 + RGB灯效
- Python调试工具 (串口助手.py)

### 系统组成

| 组件 | 芯片 | 说明 |
|------|------|------|
| 主控制器 | STM32F405RG | Cortex-M4, 168MHz, FreeRTOS |
| 电机驱动板 | STM32F103CBT6 | Cortex-M3, 72MHz, FOC步进 |
| 地轨电机 | 丝杆1605, 57步进 | 直连, 行程 -250~250mm, CAN ID=9 |
| 臂关节电机 | 42/35步进 | 50:1减速, CAN ID 1~6 |
| 夹爪电机 | 35步进 | 16:1减速, CAN ID=8 |

---

## 代码仓库结构

```
e:/Dummy-code/
├── firmware/
│   ├── ref_core_f405/              # 主控制器固件 (STM32F405, FreeRTOS)
│   │   ├── UserApp/
│   │   │   ├── main.cpp           # 入口, FreeRTOS任务
│   │   │   └── protocols/        # ASCII/CAN协议
│   │   │       ├── ascii_protocol.cpp  # USB/UART4 ASCII命令
│   │   │       └── can_protocol.cpp   # CAN响应路由
│   │   ├── Robot/
│   │   │   ├── instances/dummy_robot.cpp/.h  # 7轴机器人
│   │   │   ├── algorithms/kinematic/  # FK/IK求解器
│   │   │   └── actuators/ctrl_step/  # 关节电机CAN接口
│   │   └── Bsp/
│   │       ├── communication/interface_can.cpp  # CAN发送
│   │       └── emulated_eeprom.cpp  # Flash EEPROM
│   │
│   ├── motor_fw_f103_35/          # 35电机固件
│   ├── motor_fw_f103_42/          # 42电机固件
│   ├── motor_fw_f103_57/          # 57电机固件 (地轨)
│   └── motor_fw_f103_gripper/     # 夹爪固件
│       ├── Ctrl/Motor/motor.cpp/.h   # FOC + 堵转检测
│       ├── Ctrl/Sensor/Encoder/   # MT6816 + 标定
│       └── UserApp/protocols/interface_can.cpp  # CAN命令解析
│
├── 串口助手.py                    # Python调试工具
└── README.md
```

---

## 系统架构

### 软件层次

```
PC / Python串口助手
       │
       │ USB CDC / UART4 (115200bps)
       ▼
┌─────────────────────┐
│   ref_core_f405     │
│   (FreeRTOS, 168MHz)│
└────────┬────────────┘
         │ CAN1 (500kbps)
         ▼
┌─────────────────────────────────────────────────────┐
│           STM32F103 电机固件 (×8独立运行)              │
│  TIM1(100Hz): 按钮/LED/温度                           │
│  TIM4(20kHz): 电机控制环 (FOC)                        │
│                                                      │
│  Motor(Tick20kHz)                                   │
│    ├── encoder->UpdateAngle()  [MT6816 SPI]          │
│    ├── CloseLoopControlTick()                        │
│    │     ├── 估算位置/速度                           │
│    │     ├── DCE/PID/FOC 控制                       │
│    │     └── 堵转检测 (电流>=95%阈值 + 速度<阈值 + 1秒) │
│    └── 状态机                                        │
└─────────────────────────────────────────────────────┘
```

---

## 主控固件 (ref_core_f405)

### FreeRTOS线程架构

| 线程名 | 优先级 | 频率 | 职责 |
|--------|--------|------|------|
| `ControlLoopFixUpdateTask` | osPriorityRealtime | 5kHz (TIM7 200μs) | 实时下发电机指令 |
| `ControlLoopUpdateTask` | osPriorityNormal | 事件触发 | 解析并执行ASCII命令 |
| `OledTask` | osPriorityNormal | ~60Hz | OLED显示刷新 |
| `RGBTask` | osPriorityNormal | 33Hz | RGB LED灯效 |

### 5kHz 实时环 (TIM7)

控制环根据不同模式有不同行为：

| 模式 | 位置下发频率 | 说明 |
|------|-------------|------|
| SEQ/INT/TRJ | **50Hz** | 降频下发避免总线拥堵 |
| ServoJ | 5kHz | 高频全速下发 |
| Torque | 100Hz | 直接下发电流 |
| Tuning | 100Hz | 扫频信号发生器 |

### 电机对象初始化

```cpp
motorJ[0] = new CtrlStepMotor(hcan, 9, false, 1, -250, 250);  // 地轨
motorJ[1] = new CtrlStepMotor(hcan, 1, false, 50, -175, 175); // J1
motorJ[2] = new CtrlStepMotor(hcan, 2, true,  50,  -75,  90); // J2
motorJ[3] = new CtrlStepMotor(hcan, 3, true,  50,    0, 180); // J3
motorJ[4] = new CtrlStepMotor(hcan, 4, true,  50, -270, 270); // J4
motorJ[5] = new CtrlStepMotor(hcan, 5, true,  50, -100, 100); // J5
motorJ[6] = new CtrlStepMotor(hcan, 6, true,  30, -180, 180); // J6
hand     = new StepHand(hcan, 8);                               // 夹爪
```

### 关键常量

| 常量 | 值 | 说明 |
|------|-----|------|
| `RAIL_STEPS_PER_MM` | **40960** | 地轨步数当量 (丝杆1605直连, 200步×256微步/5mm) |
| `DEFAULT_JOINT_SPEED` | 80.0f | 关节默认速度 (°/s) |
| `REST_POSE` | `{0, -75, 180, 0, 0, 0}` | 待机姿态 |
| `HOME_POSE` | `{0, 0, 90, 0, 0, 0}` | 归零姿态 |

### IsMoving() 实现

使用纯位置误差判定（文档曾描述依赖 `jointsStateFlag`，已改为新实现）：

```cpp
bool DummyRobot::IsMoving() {
    static constexpr float EPSILON_DEG = 1.0f;
    for (int i = 1; i <= 6; i++) {
        if (fabsf(motorJ[i]->angle - motorJ[i]->targetAngle) > EPSILON_DEG)
            return true;
    }
    return false;
}
```

---

## 电机固件 (motor_fw_f103_*)

### 双定时器架构

| 定时器 | 频率 | 职责 |
|--------|------|------|
| TIM1 | 100Hz | 按钮去抖、LED状态、温度采集 |
| TIM4 | 20kHz | 电机FOC控制环 |

### 堵转检测机制

**触发条件（三个条件必须同时满足）**：

1. **电流条件**：`current >= ratedCurrent * 95%`
2. **速度条件**：`|estVelocity| < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 5` (即 < 10240 步/s)
3. **持续时间**：以上条件同时满足 **1秒 (1000ms)**

```cpp
// motor.cpp L270-302
const int32_t stallThreshold = (int32_t)(config.motionParams.ratedCurrent * 95 / 100);

if (controller->config->stallProtectSwitch) {
    if ((current >= stallThreshold) &&
        (abs(controller->estVelocity) < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 5)) {
        if (controller->stalledTime >= 1000 * 1000) {
            controller->isStalled = true;
            // 主动上报堵转: CAN 0x7C
            CAN_TxHeaderTypeDef txHdr = {};
            txHdr.StdId = (boardConfig.canNodeId << 7) | 0x7C;
            uint8_t txData[8] = { (uint8_t)boardConfig.canNodeId, 1, 0, 0, 0, 0, 0, 0 };
            CAN_Send(&txHdr, txData);
        } else {
            controller->stalledTime += motionPlanner.CONTROL_PERIOD;
        }
    } else {
        controller->stalledTime = 0;  // 条件不满足则清零
    }
}
```

### 主控堵转处理

接收到 `0x7C` 堵转通知后：

```cpp
// can_protocol.cpp L167-171, L251-255, L334-338
case 0x7C:
    if (data[1] == 1)
        dummy.SetStallMode((int)id);
    break;

// dummy_robot.cpp L425-442
void DummyRobot::SetStallMode(int motorIndex) {
    SetRGBMode(RGB::RED_HEARTBEAT);  // RGB变红
    targetJoints = currentJoints;     // 锁定目标
    for (int j = 1; j <= 6; j++) {
        motorJ[j]->targetAngle = currentJoints.a[j - 1] - initPose.a[j - 1];
    }
    commandHandler.ClearFifo();       // 清空命令队列
    (void)motorIndex;  // 未使用，未来可用于区分
}
```

### 急停机制 (0x89)

**电机固件端**（已实现）：

```cpp
// motor_fw_f103_*/interface_can.cpp L319-328
case 0x89:  // Broadcast Emergency Stop
    motor.controller->requestMode = Motor::MODE_STOP;
    motor.controller->SetBrake(true);  // 制动而非滑行
    motor.controller->SetVelocitySetPoint(0);
    motor.controller->SetCurrentSetPoint(0);
    break;
```

**主控端**（**未实现发送**）：
- `!STOP` 命令仅修改内存状态，调用 `EmergencyStop()`
- 未向 CAN 总线发送 0x89 急停帧
- 电机固件的 0x89 处理代码存在但从未被触发

---

## CAN 通信协议

### CAN ID 格式

```
StdId = (nodeID << 7) | cmdCode
  nodeID: 7 bits (实际使用 1~6, 8, 9)
  cmdCode: 7 bits (0x00~0x7F 普通, 0x80~0xBF 广播)
```

### 命令总表

| 命令码 | 方向 | 功能 | 数据格式 |
|--------|------|------|---------|
| 0x01 | TX | 使能/失能电机 | uint32_t (0/1) |
| 0x02 | TX | 触发编码器标定 | - |
| 0x03 | TX | 设置电流 (力矩模式) | float A |
| 0x05 | TX | 设置位置 (梯形规划) | float rotations |
| 0x07 | TX | 位置+速度限制 | float pos + float vel |
| 0x0B | TX | 设置堵转保护开关 | uint32_t (0/1) |
| 0x21~0x25 | RX | 查询电流/速度/位置/偏移/温度 | → 见各响应格式 |
| **0x7C** | RX | **堵转主动上报** | Data[0]=nodeID, Data[1]=1(stall) |
| 0x89 | TX | **广播急停** (电机端已实现，主控未发送) | 8B全0 |
| 0xA3 | TX | 广播查询 | - |

### 0x23 响应格式 (8字节)

```
Byte 0-3: float position   // 物理位置 (rotations)
Byte 4-5: int16_t current  // FOC相电流 × 1000 (mA)
Byte 6:    uint8_t errorCode // 0=OK, 1=Stall, 4=EStop
Byte 7:    uint8_t isFinished // 0=运动中, 1=到位
```

---

## ASCII 命令协议

### 系统命令 (`!` 前缀)

| 命令 | 功能 |
|------|------|
| `!START` | 使能机器人 |
| `!DISABLE` | 失能机器人 |
| `!STOP` | 急停（仅修改内存，**未发 CAN 0x89**） |
| `!HOME` | 归零姿态 |
| `!RESET` | 待机姿态 |
| `!CALIBRATION` | 标定零点偏移 |
| `!STALL_EN` | 开启堵转检测 |
| `!STALL_DIS` | 关闭堵转检测 |
| `!HAND_*` | 夹爪控制命令 |

### 运动命令

| 命令格式 | 功能 | 说明 |
|----------|------|------|
| `>j1,j2,j3,j4,j5,j6,j7,speed` | 阻塞MoveJ | 等待到位后返回"ok" |
| `&j1,j2,j3,j4,j5,j6,j7,speed` | 非阻塞MoveJ | 立即返回"ok" |
| `@x,y,z,a,b,c,speed` | MoveL | 笛卡尔空间运动 |
| `$c0,c1,c2,c3,c4,c5,c6` | 力矩控制 | 电流值 (A) |

---

## 已知问题与待改进

> 注：原 ISSUES.md 已删除，以下为根据实际代码分析发现的问题

### 安全相关

| 优先级 | 问题 | 状态 | 说明 |
|--------|------|------|------|
| **高** | EmergencyStop 未发 CAN 0x89 | 待修复 | `!STOP` 只改内存，电机固件有 0x89 处理但从未触发 |
| **中** | 多处 CAN 发送 canBuf 未清零 | 待修复 | `ApplyPositionAsHome()`, `Reboot()`, `UpdateAngle()` 等 |
| **中** | CtrlStepMotor::SetEnable 状态语义 | 待修复 | enable→FINISH, disable→STOP，语义不清 |

### 并发相关

| 优先级 | 问题 | 状态 | 说明 |
|--------|------|------|------|
| **中** | CAN 中断直接处理共享状态 | 待评估 | 5kHz 环与 CAN 回调数据竞争风险 |
| **中** | CanSendMessage 使用 osWaitForever | 待修复 | 无限等待可能阻塞实时环 |

### 功能缺失

| 优先级 | 问题 | 状态 | 说明 |
|--------|------|------|------|
| **中** | 堵转时未下发 CAN 制动命令 | 待修复 | 只做 RGB 提示和清队列，未强制停机 |
| **中** | ServoJ 1ms 限制 | 待评估 | `dummy_robot.cpp` L336 有 `dt <= 0.001f` 限制 |

---

## 固件编译

### 主控固件 (ref_core_f405)

```bash
cd firmware/ref_core_f405/build
ninja
```

### 电机驱动固件 (motor_fw_f103_*)

```bash
cd firmware/motor_fw_f103_35/build  # 或 42/57
ninja
```

---

## 关键文件索引

| 功能 | 文件路径 |
|------|----------|
| 主控入口 | `firmware/ref_core_f405/UserApp/main.cpp` |
| 7轴机器人类 | `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp` |
| 关节CAN接口 | `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` |
| CAN协议 | `firmware/ref_core_f405/UserApp/protocols/can_protocol.cpp` |
| ASCII协议 | `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp` |
| CAN发送 | `firmware/ref_core_f405/Bsp/communication/interface_can.cpp` |
| 电机控制核心 | `firmware/motor_fw_f103_35/Ctrl/Motor/motor.cpp` |
| 电机CAN解析 | `firmware/motor_fw_f103_35/UserApp/protocols/interface_can.cpp` |
