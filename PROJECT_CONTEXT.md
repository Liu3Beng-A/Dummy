# PROJECT_CONTEXT - Dummy Robotics Arm

> **自动生成时间**: 2026-05-23
> **项目根目录**: `e:/Dummy-code`
> **生成方式**: 每次 new chat 时自动读取所有源码重新生成

---

## 项目简介

这是一个基于 **STM32F405 (主控) + 8×STM32F103 (电机驱动)** 的 **7轴机械臂 + 地轨 + 夹爪** 控制系统。

### 系统组成

| 组件 | 芯片 | 数量 | 说明 |
|------|------|------|------|
| 主控制器 | STM32F405RG | ×1 | Cortex-M4, 168MHz, FreeRTOS |
| 电机驱动板 | STM32F103CBT6 | ×8 | Cortex-M3, 72MHz, FOC步进 |
| 地轨电机 | 丝杆1605 | ×1 | 直连(1:1), 行程 -250~250mm, ID=0 |
| 臂关节电机 | 42/35步进 | ×6 | 50:1减速, CAN ID 1~6 |
| 夹爪电机 | 35步进 | ×1 | 16:1减速, CAN ID=7 |
| 通信总线 | CAN1 | 500kbps | 连接主控与所有电机 |
| 调试串口 | UART4 | 115200bps | 主控命令接口 |
| USB | USB_OTG_FS | CDC/VCP | 备用命令接口 |
| OLED | SSD1306 | 128×64 | 板载显示 |
| IMU | MPU6050 | ×1 | 板载惯性测量 |

### 项目目标

- [x] 7轴关节空间运动 (MoveJ)
- [x] 笛卡尔空间运动 (MoveL via FK/IK)
- [x] 力矩控制模式 ($ 命令)
- [x] ServoJ 实时伺服模式
- [x] 地轨线性滑轨控制
- [x] 夹爪控制
- [ ] ROS2 / MoveIt2 接入
- [ ] 视觉抓取集成
- [ ] EtherCAT 升级（规划中）

### 代码仓库结构

```
e:/Dummy-code/
├── firmware/
│   ├── ref_core_f405/         # 主控固件 (STM32F405, FreeRTOS)
│   │   ├── UserApp/           # 应用层 (main.cpp, protocols/)
│   │   ├── Robot/             # 机器人抽象层
│   │   │   ├── instances/     # DummyRobot 7轴实现
│   │   │   ├── algorithms/    # 运动学 (FK/IK), 轨迹规划
│   │   │   └── actuators/     # 关节电机抽象 (CtrlStep)
│   │   ├── Bsp/              # 板级驱动 (CAN/UART/USB/I2C/SPI)
│   │   ├── 3rdParty/         # 第三方库 (Fibre, U8G2)
│   │   └── Core/             # STM32 HAL 初始化
│   │
│   └── motor_fw_f103/         # 电机驱动固件 (STM32F103, 8板)
│       ├── UserApp/           # main.cpp, CAN/UART 协议
│       ├── Ctrl/             # 电机控制逻辑
│       │   ├── Motor/        # 电机类 + 运动规划器
│       │   ├── Driver/        # TB67H450 FOC 驱动基类
│       │   ├── Sensor/        # MT6816 磁编 + 标定器
│       │   └── Signal/        # 按钮 / LED
│       ├── Port/             # STM32 硬件端口实现
│       │   ├── tb67h450_stm32.cpp/h
│       │   ├── mt6816_stm32.cpp/h
│       │   ├── button_stm32.cpp/h
│       │   ├── led_stm32.cpp/h
│       │   ├── encoder_calibrator_stm32.cpp/h
│       │   └── Platform/     # EEPROM, Flash, 硬件工具
│       └── Core/             # STM32 HAL 初始化
│
├── tools/
│   └── 串口助手.py             # Python 调试工具 (Tkinter GUI)
├── 串口助手.py                 # 根目录副本
└── review.md                  # 系统级代码审查报告 (C/H/M/L问题)
```

---

## 系统架构

### 硬件连接

```
 PC / Python串口助手
       │
       │ USB CDC / UART4 (115200bps)
       ▼
 ┌─────────────────────┐
 │   STM32F405 主控     │
 │   (FreeRTOS, 168MHz) │
 │                      │
 │  CAN1 (500kbps) ─────┼──── CAN ─────► CAN收发器
 └─────────────────────┘                    │
       ▲                                   │
       │ CAN RX 中断                       │
       │ (StdId = (id<<7) | cmd)           ▼
       │                         ┌──────────────────────────┐
       │                         │  STM32F103 电机驱动板 ×8  │
       │                         │  (72MHz, FOC, 20kHz环)   │
       │                         │                           │
       │                         │  ID=0: 地轨 (0~500mm)     │
       │                         │  ID=1~6: 臂关节 (J1~J6)  │
       │                         │  ID=7: 夹爪              │
       │                         └──────────────────────────┘
       │
 OLED 128×64 (I2C软)
 MPU6050 (I2C硬)
 RGB WS2812 (TIM2 DMA)
```

### 软件层次

```
┌─────────────────────────────────────────────────┐
│            PC / Python / ROS2                   │
└──────────────────────┬──────────────────────────┘
                       │ ASCII 协议 (>/@/$/#)
┌──────────────────────▼──────────────────────────┐
│           UserApp/main.cpp                      │
│  命令解析 → 命令队列 → 命令处理器               │
└──────────────────────┬──────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────┐
│           DummyRobot (7轴机器人类)               │
│                                                  │
│  motorJ[0]: 地轨 (mm, 单独管理)                 │
│  motorJ[1~6]: 臂关节 (° → step)                 │
│  hand: 夹爪 (0~100%)                            │
│  dof6Solver: 6-DOF FK/IK                        │
└──────────────────────┬──────────────────────────┘
                       │ CAN (StdId = id<<7 | cmd)
┌──────────────────────▼──────────────────────────┐
│           CtrlStepMotor (关节抽象层)             │
│  SetAngle / SetVelocity / SetCurrent           │
│  UpdateAngleCallback (CAN响应路由)               │
└──────────────────────┬──────────────────────────┘
                       │ CAN1 (500kbps)
┌──────────────────────▼──────────────────────────┐
│        STM32F103 电机固件 (×8独立运行)           │
│                                                  │
│  TIM1(100Hz): 按钮/LED/温度采集                  │
│  TIM4(20kHz): 电机控制环 (FOC)                   │
│                                                  │
│  Motor(Tick20kHz)                               │
│    ├── encoder->UpdateAngle()  [MT6816 SPI]     │
│    ├── CloseLoopControlTick()                   │
│    │     ├── 估算位置/速度                      │
│    │     ├── DCE/PID/FOC 控制                  │
│    │     └── driver->SetFocCurrent()           │
│    │           └── [TB67H450 PWM+DMA]          │
│    └── 状态机 / 堵转检测                         │
└─────────────────────────────────────────────────┘
```

---

## 主控固件 (ref_core_f405)

### 线程架构

| 线程名 | 优先级 | 栈大小 | 频率 | 职责 |
|--------|--------|--------|------|------|
| `ControlLoopFixUpdateTask` | `osPriorityRealtime` | 2000 | 5kHz (TIM7 200μs) | 实时下发电机指令 |
| `KinematicsTask` | `osPriorityHigh` | 2048 | 1kHz | FK正向运动学计算 |
| `MotorStateMonitorTask` | `osPriorityNormal` | 2048 | 100Hz | 广播查询所有电机状态 |
| `ControlLoopUpdateTask` | `osPriorityNormal` | 2000 | 事件触发 | 解析并执行ASCII命令 |
| `OledTask` | `osPriorityNormal` | 2000 | ~60Hz | OLED显示刷新 |
| `RGBTask` | `osPriorityNormal` | 2000 | 33Hz | RGB LED灯效 |

### 5kHz 实时环 (TIM7)

TIM7 定时器 (周期200μs) 触发中断，唤醒 `ControlLoopFixUpdateTask`，根据当前控制模式执行：

**SEQ / INT / TRJ 模式：**
```cpp
dummy.MoveJoints(dummy.targetJoints);   // 臂关节下发 (°→step)
dummy.MoveRail(dummy.targetRailPos);     // 地轨下发 (mm→step)
```

**Torque 模式 ($ 命令)：**
```cpp
for (int i = 1; i <= 6; i++)
    dummy.motorJ[i]->SetCurrentSetPoint(dummy.targetCurrents[i-1]);
dummy.motorJ[0]->SetCurrentSetPoint(dummy.targetRailCurrent);  // 地轨电流
```

**ServoJ 模式：**
```cpp
dummy.MoveJoints(dummy.targetJoints);     // 高频关节角度
dummy.MoveRail(dummy.targetRailPos);
```

**Tuning 模式：**
```cpp
dummy.tuningHelper.Tick(10);            // 扫频调试
```

### 电机对象初始化 (DummyRobot 构造函数)

```cpp
motorJ[0] = new CtrlStepMotor(hcan, 0, false, 50, 0, 500);     // 地轨: 丝杆1605, 0~500mm
motorJ[1] = new CtrlStepMotor(hcan, 1, false, 50, -175, 175); // J1 底座
motorJ[2] = new CtrlStepMotor(hcan, 2, true,  50,  -75,  90); // J2 肩部
motorJ[3] = new CtrlStepMotor(hcan, 3, true,  50,    0, 180); // J3 肘部
motorJ[4] = new CtrlStepMotor(hcan, 4, true,  50, -270, 270); // J4 腕部旋转
motorJ[5] = new CtrlStepMotor(hcan, 5, true,  50, -100, 100); // J5 腕部俯仰
motorJ[6] = new CtrlStepMotor(hcan, 6, true,  30, -180, 180); // J6 腕部偏转 (减速比30)
hand     = new StepHand(hcan, 7);                              // 夹爪 (减速比16)
```

### 6-DOF 运动学参数 (DH参数)

```cpp
dof6Solver = new DOF6Kinematic(
    0.165f,  // L_BASE    (base height, m)
    0.0f,    // D_BASE    (base offset, m)
    0.170f,  // L_ARM     (upper arm length, m)
    0.117f,  // L_FOREARM (forearm length, m)
    0.0695f, // D_ELBOW   (elbow offset, m)
    0.113f   // L_WRIST   (wrist length, m)
);
```

### 控制模式枚举

| 枚举值 | 名称 | 说明 |
|--------|------|------|
| `COMMAND_TARGET_POINT_SEQUENTIAL` = 1 | SEQ | 顺序点动 (阻塞) |
| `COMMAND_TARGET_POINT_INTERRUPTABLE` = 2 | INT | 可打断点动 (默认) |
| `COMMAND_CONTINUES_TRAJECTORY` = 3 | TRJ | 连续轨迹 |
| `COMMAND_MOTOR_TUNING` = 4 | TUN | 电机参数扫频调谐 |
| `COMMAND_TORQUE_CONTROL` = 5 | TRQ | 力矩/电流控制 |
| `COMMAND_SERVO_J` = 6 | SRV | 高频关节伺服 |

### 关键常量

| 常量 | 值 | 说明 |
|------|-----|------|
| `RAIL_STEPS_PER_MM` | 40960 | 地轨步数当量 (丝杆1605直连, 256微步) |
| `RAIL_DEFAULT_SPEED_MM_S` | 20.0f | 地轨默认速度 mm/s |
| `DEFAULT_JOINT_SPEED` | 80.0f | 关节默认速度 |
| `DEFAULT_JOINT_ACCELERATION_LOW` | 5.0f | 低加速度 |
| `DEFAULT_JOINT_ACCELERATION_HIGH` | 100.0f | 高加速度 |
| `REST_POSE` | {0, -75, 180, 0, 0, 0} | 待机姿态 |
| `HOME_POSE` | {0, 0, 90, 0, 0, 0} | 归零姿态 |

---

## 电机固件 (motor_fw_f103)

### 双定时器架构

| 定时器 | 频率 | 回调 | 职责 |
|--------|------|------|------|
| TIM1 | 100Hz (10ms) | `Tim1Callback100Hz()` | 按钮去抖、LED状态、温度采集 |
| TIM4 | 20kHz (50μs) | `Tim4Callback20kHz()` | 电机FOC控制环 / 编码器标定 |

### 电机控制模式

| 模式 | CMD | 控制方法 | 备注 |
|------|-----|----------|------|
| `MODE_STOP` | - | 失能 | |
| `MODE_COMMAND_POSITION` | 0x05/0x07 | DCE + 梯形规划 | 带速度限制 |
| `MODE_COMMAND_VELOCITY` | 0x04 | PID 速度环 | |
| `MODE_COMMAND_CURRENT` | 0x03 | 直接电流给定 | |
| `MODE_COMMAND_Trajectory` | - | 轨迹跟踪 | 外部给轨迹点 |
| `MODE_PWM_*` | - | PWM开环 | 调试用 |
| `MODE_STEP_DIR` | - | STEP/DIR 模式 | |

### 电机状态

| 状态 | 枚举值 | 说明 |
|------|--------|------|
| `STATE_STOP` | 0 | 停止 |
| `STATE_FINISH` | 1 | 到位完成 |
| `STATE_RUNNING` | 2 | 运动中 |
| `STATE_OVERLOAD` | 3 | 过载 |
| `STATE_STALL` | 4 | 堵转 |
| `STATE_NO_CALIB` | 5 | 未标定 |

### 全局错误码

| 值 | 含义 | 触发条件 |
|----|------|----------|
| 0 | 正常 | |
| 1 | 堵转 (Stall) | 堵转保护触发后标记 |
| 4 | 急停 (EmergencyStop) | 收到0x89广播急停 |

### 步进参数

| 参数 | 值 |
|------|-----|
| `MOTOR_ONE_CIRCLE_HARD_STEPS` | 200 (1.8°步距角) |
| `SOFT_DIVIDE_NUM` | 256 (微步细分) |
| `MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS` | 51200 (200×256) |
| FOC Sin表大小 | 1025 entries (sin[0] ~ sin[2π]) |
| 编码器分辨率 | 14-bit (MT6816, 16384 cpr) |

### 堵转保护机制

- 触发条件：电流饱和 (`|output| >= ratedCurrent`) 且速度极低 (`|estVelocity| < 1 step/s`) 持续 1 秒
- 动作：保持使能，原地锁死 (`goalPosition = realPosition`)，errorCode=1
- 恢复：按 KEY2 清除标志

### 编码器标定流程

1. **触发条件**：KEY1+KEY2同时长按，或CAN命令0x02
2. **正向测量**：慢速转动1圈，16次采样/步，记录所有编码器值
3. **反向测量**：反向转动，去除回差
4. **数据处理**：对正向/反向数据取平均，生成16384项查找表
5. **Flash存储**：烧录到 `0x08017C00` (32KB)，掉电不丢失

### 35/42电机切换

编译前修改 `configurations.h`：

```cpp
// 默认: 35电机 (注释掉下一行)
// #define MOTOR_TYPE_42
#ifdef MOTOR_TYPE_42
    #define DEFAULT_DCE_KP      200
    #define MAX_CURRENT_LIMIT   (2 * 1000)   // 42电机: 2000mA
#else
    #define DEFAULT_DCE_KP      195
    #define MAX_CURRENT_LIMIT   (int32_t)(1.2f * 1000)  // 35电机: 1200mA
#endif
```

### 电机固件硬件引脚

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA8 | ID0 | 拨码开关 bit0 |
| PA9 | ID1 | 拨码开关 bit1 |
| PA10 | ID2 | 拨码开关 bit2 |
| PA2 | HW_ELEC_BM | TB67H450 INBM |
| PA3 | HW_ELEC_BP | TB67H450 INBP |
| PA4 | HW_ELEC_AM | TB67H450 INAM |
| PA5 | HW_ELEC_AP | TB67H450 INAP |
| PA7 | SIGNAL_COUNT_DIR | 方向信号 |
| PB0 | SIGNAL_COUNT_EN | 使能信号 |
| PB1 | SIGNAL_ALERT | 报警信号 |
| PB2 | BUTTON2 | 按键2 |
| PB10 | HW_ELEC_BPWM | B相PWM |
| PB11 | HW_ELEC_APWM | A相PWM |
| PB12 | BUTTON1 | 按键1 |
| PA15 | SPI1_CS | MT6816 片选 |
| PC13 | LED1 | 状态灯1 |
| PC14 | LED2 | 状态灯2 |
| PB8 | CAN1_RX | CAN接收 |
| PB9 | CAN1_TX | CAN发送 |
| TIM2 CH3/CH4 | DAC输出 | TB67H450 VREF |

### Flash 存储布局 (STM32F103CBT6, 128KB)

| 地址 | 大小 | 用途 |
|------|------|------|
| 0x08000000 | 47KB | 应用程序 |
| 0x0800BC00 | 1KB | DAPLink配置 |
| 0x08017C00 | 32KB | 编码器标定数据 |
| 0x0801FC00 | 1KB | 用户配置/EEPROM |

---

## CAN 通信协议

### CAN ID 格式

```
StdId = (nodeID << 7) | cmdCode

nodeID: 3 bits (0~7)
cmdCode: 7 bits

普通命令: cmdCode 0x00~0x7F (发往特定节点)
广播命令: cmdCode 0x80~0xBF (所有节点无条件响应)
```

### 命令分类总表

| 命令码 | 方向 | 功能 | 数据格式 |
|--------|------|------|----------|
| **0x01** | TX | 使能/失能电机 | uint32_t (0/1) |
| **0x02** | TX | 触发编码器标定 | - |
| **0x03** | TX | 设置电流 (力矩模式) | float A |
| **0x04** | TX | 设置速度 | float r/s |
| **0x05** | TX | 设置位置 (梯形规划) | float rotations |
| **0x06** | TX | 位置+时间 | float pos + float time |
| **0x07** | TX | 位置+速度限制 | float pos + float vel |
| **0x08** | TX | 直通位置 (绕规划器) | float rotations |
| **0x11** | TX | 设置CAN节点ID | uint32_t + store flag |
| **0x12** | TX | 设置电流限制 | float A + store flag |
| **0x13** | TX | 设置速度限制 | float r/s + store flag |
| **0x14** | TX | 设置加速度 | float r/s² + store flag |
| **0x15** | TX | 应用零点偏移 | - |
| **0x16** | TX | 设置启动自动使能 | uint32_t + store flag |
| **0x17** | TX | 设置DCE Kp | int32_t + store flag |
| **0x18** | TX | 设置DCE Kv | int32_t + store flag |
| **0x19** | TX | 设置DCE Ki | int32_t + store flag |
| **0x1A** | TX | 设置DCE Kd | int32_t + store flag |
| **0x1B** | TX | 设置堵转保护 | uint32_t + store flag |
| **0x21** | RX | 查询电流 | → 4B float + 1B state |
| **0x22** | RX | 查询速度 | → 4B float + 1B state |
| **0x23** | RX | 查询位置+状态 | → 8B (见下方) |
| **0x24** | RX | 查询零点偏移 | → 4B int32_t |
| **0x25** | RX | 查询温度 | → 4B float |
| **0x7D** | TX | 启用温度监控 | - |
| **0x7E** | TX | 恢复出厂设置 | - |
| **0x7F** | TX | 软件重启 | - |
| **0x89** | TX | **广播急停** (所有节点) | 8B全0 |
| **0xA3** | TX | **广播查询** (所有节点) | - |

### 0x23 响应格式 (8字节)

```
Byte 0-3: float position   // 物理位置 (rotations)
Byte 4-5: int16_t current   // FOC相电流 × 1000 (mA)
Byte 6:    uint8_t errorCode // 0=OK, 1=Stall, 4=EStop
Byte 7:    uint8_t isFinished // 0=运动中, 1=到位
```

### CAN 急停广播机制

```
主控发送: StdId = 0x89 (cmd=0x89, 节点ID任意)
数据: 8字节全0
结果: 所有电机驱动板同时执行急停
  - requestMode = MODE_STOP
  - velocity = 0, current = 0
  - errorCode = 4
```

**急停条件判断 (motor_fw_f103/Core/Src/can.c)：**
```cpp
// StdId == 0x00: 立即急停，不进入协议解析
if (RxHeader.StdId == 0x00) { EmergencyStopFromCan(); return; }

// cmd 0x80~0xBF: 广播命令，所有节点无条件响应
if (cmd >= 0x80 && cmd <= 0xBF) { OnCanCmd(cmd, RxData, DLC); }
else if (id == boardConfig.canNodeId) { OnCanCmd(cmd, RxData, DLC); }
```

---

## ASCII 命令协议 (主控 → 主控内部)

### 系统命令 (`!` 前缀)

| 命令 | 功能 |
|------|------|
| `!START` | 使能机器人 |
| `!DISABLE` | 失能机器人 |
| `!STOP` | 广播急停 (StdId=0x89) |
| `!HOME` | 归零姿态 |
| `!RESET` | 待机姿态 |
| `!CALIBRATION` | 标定零点偏移 |
| `!HAND_O` | 打开夹爪 |
| `!HAND_C` | 关闭夹爪 |
| `!HAND_EN` | 使能夹爪 |
| `!HAND_DIS` | 失能夹爪 |
| `!HAND_ZERO` | 夹爪标定 |
| `!HAND_POS <0-100>` | 夹爪位置控制 |
| `!HAND_I <0-2>` | 夹爪电流档位 |

### 查询命令 (`#` 前缀)

| 命令 | 功能 |
|------|------|
| `#GETJPOS` | 获取关节角度 |
| `#GETLPOS` | 获取末端位姿 |
| `#SET_DCE_KV/KP/KI/KD <node> <val>` | 设置电机PID |
| `#REBOOT <node>` | 重启电机 |
| `#CMDMODE <1-6>` | 设置控制模式 |
| `#OFFSET_J <node>` | 应用零点 |
| `#ACC_J <node> <val>` | 设置加速度 |
| `#SPEED_J <node> <val>` | 设置速度 |

### 运动命令

| 格式 | 功能 | 说明 |
|------|------|------|
| `>j1,j2,j3,j4,j5,j6,j7,speed` | 阻塞MoveJ | 等待到位返回"ok" |
| `&j1,j2,j3,j4,j5,j6,j7,speed` | 非阻塞MoveJ | 立即返回"ok" |
| `@x,y,z,a,b,c,speed` | MoveL | 笛卡尔空间运动 |
| `$c1,c2,c3,c4,c5,c6,c7` | 力矩控制 | 7轴电流(mA) |

### 夹爪命令 (通过 `!` 和 `#`)

夹爪通过 `StepHand` 类控制 (继承 `CtrlStepMotor`, ID=7)：
- `HAND_O`: `SetAngleWithSpeedLimit(100.0f)` (全开)
- `HAND_C`: `SetAngleWithSpeedLimit(0.0f)` (全闭)
- `HAND_POS <0-100>`: 按比例设置角度

---

## 关键算法

### DCE 控制算法 (位置+速度复合环)

```
FOC_current = DCE_Kp × position_error
            + DCE_Kv × velocity_error
            + DCE_Ki × ∫position_error
            + DCE_Kd × d(position_error)/dt
```

默认参数: Kp=195/200, Kv=80, Ki=300, Kd=250

### 梯形速度规划 (PositionTracker)

```
速度
  ^
  │    ╱╲
  │   ╱  ╲
  │  ╱    ╲
  │ ╱      ╲
  └──────────────→ 位置
  加速段  匀速  减速段
```

### 地轨步数换算

```
1mm = (步数/圈 × 微步数) / 丝杆导程
     = (200 × 256) / 5
     = 40960 步/mm
```

---

## 已知问题 (来自 review.md)

系统级代码审查报告 `review.md` 记录了以下等级的问题：

### Critical (需立即修复)

- **C-1**: IK/FK 在 5kHz 实时环中执行 → 高计算负载可能超时
- **C-2**: CAN 带宽不匹配 → 9条TX消息/200μs超过500kbps物理极限
- **C-3**: `motorJ[ALL]` 仅查询地轨电机 → 臂关节状态监控缺失

### High (高优先级)

- **H-1**: 无超时保护 → ServoJ 丢帧后机器人继续运动
- **H-2**: ServoJ 周期 ≤1ms 时性能下降
- **H-3**: CAN 类型转换安全隐患
- **H-4**: 电机使能逻辑错误
- **H-5**: CAN 过滤器未配置
- **H-6**: ServoJ 无到位判断

### Medium (中优先级)

- **M-1**: USB 栈溢出风险
- **M-2**: 动态内存分配风险
- **M-3**: MoveJ 速度规划不合理
- **M-4**: CAN 过滤器配置
- **M-5**: `std::abs` 歧义
- **M-6**: 堵转保护无CAN上报
- **M-7**: 地轨与MoveL联动
- **M-8**: 命令队列阻塞
- **M-9**: OLED线程安全

### Low (低优先级)

- **L-1**: YAML解析错误处理
- **L-2**: PWM占空比限制
- **L-3**: 主循环栈深度

详见 `e:/Dummy-code/review.md`。

---

## 串口助手 (Python)

`tools/串口助手.py` / `串口助手.py` (根目录副本) 是一个 Tkinter GUI 工具：

| 功能 | 说明 |
|------|------|
| 串口连接 | 自动刷新、115200bps |
| MoveJ | 7滑块控制 + 速度滑块 |
| MoveL | XYZ + ABC + 速度 |
| 力矩控制 | 7轴电流滑块 |
| 系统命令 | !START/DISABLE/STOP/HOME/RESET |
| 夹爪控制 | 开/关/位置 |
| ServoJ测试 | 正弦波扫频 |
| 模式切换 | SEQ/INT/TRJ/Torque/Servo |
| 查询 | 关节角度、末端位姿 |
| 波特率 | 9600 / 115200 / 1000000 |

---

## ROS2 / MoveIt2 接入路径 (规划中)

当前协议为 ASCII 文本，ROS2 接入需要：

1. **协议层**: 将 ASCII 协议替换为 Fibre/CAN 或自定义二进制协议
2. **桥接节点**: Python/C++ 节点在 PC 端解析 ROS2 指令 → CAN 帧
3. **MoveIt2 配置**: URDF + SRDF + 6-DOF 运动学插件
4. **控制器**: JointTrajectoryController 替代当前直接位置控制
5. **状态反馈**: 将 CAN 0x23 回读数据发布为 `/joint_states`

详见 `review.md` 第11节。

---

## 快速参考

### 地轨扩展关键改动 (2026-05-22)

| 文件 | 改动 |
|------|------|
| `dummy_robot.cpp` | motorJ[0]=地轨, 7轴MoveJ/ServoJ/$, 地轨单独管理 |
| `dummy_robot.h` | +COMMAND_TORQUE_CONTROL, +COMMAND_SERVO_J, targetRailCurrent |
| `main.cpp` (F405) | 控制环支持7轴力矩/ServoJ, OLED显示地轨 |
| `can.c` (F103) | cmd 0x80~0xBF 广播判断 |
| `interface_can.cpp` (F103) | 0x89广播急停, 0xA3广播查询 |
| `串口助手.py` | J7地轨滑块, 7轴力矩, ServoJ 7轴 |

### CAN ID 快速查表

| 电机 | CAN ID | 广播命令响应 |
|------|--------|------------|
| 地轨 | 0 | 普通命令 (cmd<0x80) |
| J1~J6 | 1~6 | 普通命令 (cmd<0x80) |
| 夹爪 | 7 | 普通命令 (cmd<0x80) |
| 全部 | 任意 | 广播急停 0x89, 广播查询 0xA3 |

### 运动学末端链路

```
Link0 (Base) → J1 (旋转) → J2 (肩) → J3 (肘) → J4 (腕旋) → J5 (腕俯) → J6 (腕偏) → Tool0
L_BASE=0.165m   L_ARM=0.170m  L_FORE=0.117m  L_WRIST=0.113m
```
