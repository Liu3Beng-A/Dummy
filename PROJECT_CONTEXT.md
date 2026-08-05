# PROJECT_CONTEXT - Dummy 7轴机械臂控制系统

> **自动生成时间**: 2026-05-30
> **项目根目录**: `e:/Dummy-code`
> **生成方式**: 每次 new chat 时自动读取所有源码重新生成

> **重要文档索引:**
> - `README.md` — 项目主文档，包含完整功能说明和用户指南
> - `ISSUES.md` — 问题追踪，P0/P1/P2/P3 共 61 个问题及修复状态
> - `TODO.md` — 功能路线图，17 个计划开发功能及 6 个阶段规划

---

## 项目简介

这是一个基于 **STM32F405 (主控) + 8×STM32F103 (电机驱动)** 的 **7轴机械臂 + 地轨 + 夹爪** 控制系统，运行 FreeRTOS 实时操作系统，支持关节空间运动、笛卡尔空间运动、力矩控制和实时伺服模式。

### 系统组成

| 组件 | 芯片 | 数量 | 说明 |
|------|------|------|------|
| 主控制器 | STM32F405RG | ×1 | Cortex-M4, 168MHz, FreeRTOS |
| 电机驱动板 | STM32F103CBT6 | ×8 | Cortex-M3, 72MHz, FOC步进 |
| 地轨电机 | 丝杆1605 | ×1 | 直连(1:1), 行程 -250~250mm, CAN ID=9 (固定) |
| 臂关节电机 | 42/35步进 | ×6 | 50:1减速, CAN ID 1~6 |
| 夹爪电机 | 35步进 | ×1 | 16:1减速, CAN ID=8 (固定) |
| 通信总线 | CAN1 | 500kbps | 连接主控与所有电机 |
| 调试串口 | UART4 | 115200bps | 主控命令接口 |
| USB | USB_OTG_FS | CDC/VCP | 备用命令接口 |
| OLED | SSD1306 | 128×64 | 板载显示 |
| IMU | MPU6050 | ×1 | 板载惯性测量单元 |

### 项目目标

**已实现:**

- [x] 7轴关节空间运动 (MoveJ, blocking / non-blocking)
- [x] 笛卡尔空间运动 (MoveL via 6-DOF FK/IK)
- [x] 力矩控制模式 ($ 命令, mA电流)
- [x] ServoJ 实时伺服模式
- [x] 地轨线性滑轨控制 (J7, mm单位)
- [x] 夹爪控制 (开/闭/位置/力矩档位)
- [x] 梯形速度规划
- [x] 软限位保护 (每轴min/max)
- [x] 双定时器FOC控制 (20kHz电流环)
- [x] MT6816编码器自动校准 (16384-entry LUT)
- [x] OLED状态显示 + RGB灯效
- [x] Python调试工具 (串口助手.py)

**规划中 (详见 `TODO.md`):**

- [ ] ROS2 / MoveIt2 接入
- [ ] 碰撞检测与保护
- [ ] 示教编程与轨迹录制回放
- [ ] 夹爪力矩反馈与软停止
- [ ] 视觉抓取集成
- [ ] 多机协作模式
- [ ] 速度前瞻与S型速度曲线

---

## 代码仓库结构

```
e:/Dummy-code/
├── firmware/                        # 所有固件代码
│   ├── ref_core_f405/               # 主控制器固件 (STM32F405, FreeRTOS)
│   │   ├── UserApp/                 # 应用层
│   │   │   ├── main.cpp             # 入口, FreeRTOS任务创建
│   │   │   ├── protocols/           # 协议层
│   │   │   │   ├── ascii_protocol.cpp  # USB/UART4 ASCII命令解析
│   │   │   │   ├── can_protocol.cpp   # CAN响应路由
│   │   │   │   ├── cmd_protocol.cpp   # (预留)二进制协议
│   │   │   │   └── comm_pose_uart.cpp # (预留)位姿UART发送
│   │   │   ├── tasks/               # (预留)独立任务
│   │   │   └── pose_sender_task.cpp  # (预留)位姿发送任务
│   │   ├── Robot/                   # 机器人抽象层
│   │   │   ├── instances/
│   │   │   │   └── dummy_robot.cpp/.h  # 7轴机器人实现
│   │   │   ├── algorithms/
│   │   │   │   └── kinematic/
│   │   │   │       └── 6dof_kinematic.cpp/.h  # FK/IK求解器
│   │   │   └── actuators/
│   │   │       └── ctrl_step.cpp/.h  # 关节电机CAN接口
│   │   ├── Bsp/                     # 板级驱动
│   │   │   ├── interface_can.cpp/.h  # CAN初始化和发送
│   │   │   ├── interface_uart.cpp/.h # UART4/USB CDC
│   │   │   ├── oled.cpp/.h          # SSD1306 OLED
│   │   │   ├── imu/                 # MPU6050 IMU + Biquad滤波
│   │   │   ├── pwm.cpp/.h           # PWM输出
│   │   │   ├── rgb.cpp/.h           # WS2812 RGB LED
│   │   │   ├── encoder.cpp/.h       # 编码器输入
│   │   │   └── emulated_eeprom.cpp/.h # Flash EEPROM模拟
│   │   ├── Core/                    # STM32 HAL初始化
│   │   ├── 3rdParty/               # 第三方库 (Fibre, U8G2)
│   │   └── doc/
│   │       └── rail_sync_plan_B.md  # 地轨同步方案B设计文档
│   │
│   └── motor_fw_f103_*/              # 电机驱动固件 (STM32F103, 4种变体)
│       ├── UserApp/                 # main.cpp, CAN协议
│       │   └── protocols/
│       │       └── interface_can.cpp  # CAN命令解析
│       ├── Ctrl/                    # 电机控制逻辑
│       │   ├── Motor/
│       │   │   ├── motor.cpp/.h      # FOC电机控制核心
│       │   │   └── motion_planner.cpp/.h  # 梯形/S型轨迹规划
│       │   ├── Driver/
│       │   │   └── tb67h450_base.cpp/.h  # TB67H450 H桥驱动抽象
│       │   └── Sensor/Encoder/
│       │       ├── mt6816_base.cpp/.h  # MT6816磁编驱动
│       │       └── encoder_calibrator_base.cpp/.h  # 编码器校准
│       ├── Port/                    # STM32硬件端口实现
│       │   ├── tb67h450_stm32.cpp/.h # PWM输出到TB67H450
│       │   ├── mt6816_stm32.cpp/.h  # SPI读取MT6816
│       │   ├── button_stm32.cpp/.h  # 按键输入
│       │   ├── led_stm32.cpp/.h     # LED输出
│       │   └── encoder_calibrator_stm32.cpp/.h
│       └── Core/                    # STM32 HAL初始化
│
├── esp32/                           # ESP32参考示例 (非生产代码)
│   └── firmware/examples/           # 1995个Mongoose网络示例
├── tools/                           # 工具脚本
│   └── 串口助手.py                  # Python调试工具
├── 串口助手.py                      # 根目录副本 (同tools/)
├── README.md                        # 项目主文档
├── ISSUES.md                        # 问题追踪 (61个问题)
└── TODO.md                          # 功能路线图 (17个计划功能)
```

> **电机固件变体说明:** `motor_fw_f103_35/` (35mm步进)、`motor_fw_f103_42/` (42mm步进)、`motor_fw_f103_57/` (57mm步进)、`motor_fw_f103_all/` (通用版, 推荐使用) 均共用同一套代码，通过 `configurations.h` 中的 `MOTOR_TYPE_42` 宏切换参数。

---

## 系统架构

### 硬件连接

```
 PC / Python串口助手
        │
        │ USB CDC 或 UART4 (115200bps)
        ▼
 ┌─────────────────────┐
 │   STM32F405 主控     │
 │   (FreeRTOS, 168MHz)│
 │                      │
 │  CAN1 (500kbps) ─────┼──── CAN总线 ────► CAN收发器
 └─────────────────────┘                     │
        ▲                                   │
        │ CAN RX 中断                       │
        │ (StdId = id<<7 | cmd)            ▼
        │                          ┌──────────────────────────┐
        │                          │  STM32F103 电机驱动板 ×8  │
        │                          │  (72MHz, FOC, 20kHz)    │
        │                          │                          │
 OLED   │                          │  CAN ID=9: 地轨 (mm)     │
 MPU6050│                          │  CAN ID=1~6: 臂关节 (°) │
 RGB WS │                          │  CAN ID=8: 夹爪         │
        │                          └──────────────────────────┘
```

### 软件层次

```
┌──────────────────────────────────────────────────────┐
│           PC / Python 调试工具 / (未来: ROS2)          │
└───────────────────────┬──────────────────────────────┘
                        │ ASCII 协议 (>/@/$/#)
┌───────────────────────▼──────────────────────────────┐
│              UserApp/main.cpp                          │
│  USB/UART4 DMA接收 → 命令解析 → 命令队列 → 命令执行     │
└───────────────────────┬──────────────────────────────┘
                        │
┌───────────────────────▼──────────────────────────────┐
│              DummyRobot (7轴机器人类)                   │
│                                                        │
│  motorJ[0]: 地轨 (mm, 单独管理)                        │
│  motorJ[1~6]: 臂关节 (° → step换算)                   │
│  hand: 夹爪 (0~100%)                                   │
│  dof6Solver: 6-DOF FK/IK                              │
│  CommandHandler: 命令队列管理                          │
└───────────────────────┬──────────────────────────────┘
                        │ CAN (StdId = id<<7 | cmd)
┌───────────────────────▼──────────────────────────────┐
│            CtrlStepMotor (关节CAN接口)                  │
│  SetAngle / SetVelocity / SetCurrent                 │
│  UpdateAngleCallback (CAN响应路由)                     │
└───────────────────────┬──────────────────────────────┘
                        │ CAN1 (500kbps)
┌───────────────────────▼──────────────────────────────┐
│         STM32F103 电机固件 (×8 独立运行)               │
│                                                        │
│  TIM1 (100Hz): 按钮/LED/温度采集                       │
│  TIM4 (20kHz): 电机FOC控制环 / 编码器标定              │
│                                                        │
│  Motor(Tick20kHz)                                     │
│    ├── encoder->UpdateAngle()  [MT6816 SPI]            │
│    ├── CloseLoopControlTick()                          │
│    │     ├── 估算位置/速度                            │
│    │     ├── DCE/PID/FOC 控制                        │
│    │     └── driver->SetFocCurrent()                  │
│    │           └── [TB67H450 PWM+DAC]                │
│    └── 状态机 / 堵转检测                               │
└──────────────────────────────────────────────────────┘
```

---

## 主控固件 (ref_core_f405)

### FreeRTOS线程架构

| 线程名 | 优先级 | 栈大小 | 频率 | 职责 |
|--------|--------|--------|------|------|
| `ControlLoopFixUpdateTask` | `osPriorityRealtime` | 2000 | 5kHz (TIM7 200μs) | 实时下发电机指令 |
| `KinematicsTask` | `osPriorityHigh` | 2048 | 1kHz | FK正向运动学计算 |
| `MotorStateMonitorTask` | `osPriorityNormal` | 2048 | 100Hz | 广播查询所有电机状态 |
| `ControlLoopUpdateTask` | `osPriorityNormal` | 2000 | 事件触发 | 解析并执行ASCII命令 |
| `OledTask` | `osPriorityNormal` | 2000 | ~60Hz | OLED显示刷新 |
| `RGBTask` | `osPriorityNormal` | 2000 | 33Hz | RGB LED灯效 |

### 控制模式

| 枚举值 | 名称 | 说明 | 5kHz环行为 |
|--------|------|------|-----------|
| `COMMAND_TARGET_POINT_SEQUENTIAL` = 1 | SEQ | 顺序点动 (阻塞) | 下发 targetJoints |
| `COMMAND_TARGET_POINT_INTERRUPTABLE` = 2 | INT | 可打断点动 (默认) | 下发 targetJoints |
| `COMMAND_CONTINUES_TRAJECTORY` = 3 | TRJ | 连续轨迹 | 下发 targetJoints |
| `COMMAND_MOTOR_TUNING` = 4 | TUN | 电机扫频调谐 | 调用 tuningHelper.Tick() |
| `COMMAND_TORQUE_CONTROL` = 5 | TRQ | 力矩/电流控制 | 下发 targetCurrents[] |
| `COMMAND_SERVO_J` = 6 | SRV | 高频关节伺服 | 下发高频 targetJoints |

### 电机对象初始化

```cpp
motorJ[0] = new CtrlStepMotor(hcan, 9, false, 1, -250, 250);     // 地轨: 丝杆1605, 直连, -250~250mm, CAN ID=9（固定）
motorJ[1] = new CtrlStepMotor(hcan, 1, false, 50, -175, 175); // J1 底座
motorJ[2] = new CtrlStepMotor(hcan, 2, true,  50,  -75,  90); // J2 肩部
motorJ[3] = new CtrlStepMotor(hcan, 3, true,  50,    0, 180); // J3 肘部
motorJ[4] = new CtrlStepMotor(hcan, 4, true,  50, -270, 270); // J4 腕部旋转
motorJ[5] = new CtrlStepMotor(hcan, 5, true,  50, -100, 100); // J5 腕部俯仰
motorJ[6] = new CtrlStepMotor(hcan, 6, true,  30, -180, 180); // J6 腕部偏转 (减速比30)
hand     = new StepHand(hcan, 8);                              // 夹爪 (减速比16), CAN ID=8（固定）
```

> `inverseDirection=true` 表示该关节电机方向反转。J2-J6 默认反转以匹配右手坐标系。

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

**运动链:** `Link0 (Base) → J1 → J2 → J3 → J4 → J5 → J6 → Tool0`

### 关键常量

| 常量 | 值 | 说明 |
|------|-----|------|
| `RAIL_STEPS_PER_MM` | 40960 | 地轨步数当量 (丝杆1605直连, 200步×256微步/5mm导程) |
| `RAIL_DEFAULT_SPEED_MM_S` | 20.0f | 地轨默认速度 mm/s |
| `DEFAULT_JOINT_SPEED` | 80.0f | 关节默认速度 (°/s) |
| `DEFAULT_JOINT_ACCELERATION_LOW` | 5.0f | 低加速度 (°/s²) |
| `DEFAULT_JOINT_ACCELERATION_HIGH` | 100.0f | 高加速度 (°/s²) |
| `REST_POSE` | `{0, -75, 180, 0, 0, 0}` | 待机姿态 |
| `HOME_POSE` | `{0, 0, 90, 0, 0, 0}` | 归零姿态 |

### 5kHz 实时环 (TIM7)

TIM7 定时器 (周期200μs) 触发中断，唤醒 `ControlLoopFixUpdateTask`，根据当前控制模式执行不同行为:

**SEQ / INT / TRJ 模式:**

```cpp
dummy.MoveJoints(dummy.targetJoints);   // 臂关节下发 (°→step)
dummy.MoveRail(dummy.targetRailPos);    // 地轨下发 (mm→step)
```

**Torque 模式 ($ 命令):**

```cpp
for (int i = 1; i <= 6; i++)
    dummy.motorJ[i]->SetCurrentSetPoint(dummy.targetCurrents[i-1]);
dummy.motorJ[0]->SetCurrentSetPoint(dummy.targetRailCurrent);  // 地轨电流
```

**ServoJ 模式:**

```cpp
dummy.MoveJoints(dummy.targetJoints);    // 高频关节角度
dummy.MoveRail(dummy.targetRailPos);
```

**Tuning 模式:**

```cpp
dummy.tuningHelper.Tick(10);             // 扫频调试
```

---

## 电机固件 (motor_fw_f103_*)

> **推荐使用 `motor_fw_f103_all/` 通用版。** 35/42/57三种变体通过 `configurations.h` 中的 `MOTOR_TYPE_42` 宏切换。

### 双定时器架构

| 定时器 | 频率 | 回调 | 职责 |
|--------|------|------|------|
| TIM1 | 100Hz (10ms) | `Tim1Callback100Hz()` | 按钮去抖、LED状态、温度采集 |
| TIM4 | 20kHz (50μs) | `Tim4Callback20kHz()` | 电机FOC控制环 / 编码器标定 |

### 电机控制模式

| 模式 | CMD | 控制方法 |
|------|-----|---------|
| `MODE_STOP` | - | 失能 |
| `MODE_COMMAND_POSITION` | 0x05/0x07 | DCE + 梯形规划 |
| `MODE_COMMAND_VELOCITY` | 0x04 | PID 速度环 |
| `MODE_COMMAND_CURRENT` | 0x03 | 直接电流给定 |
| `MODE_COMMAND_Trajectory` | - | 外部轨迹点跟踪 |
| `MODE_PWM_*` | - | PWM开环 (调试) |
| `MODE_STEP_DIR` | - | STEP/DIR 模式 |

### 电机状态

| 状态 | 枚举值 |
|------|--------|
| `STATE_STOP` | 0 |
| `STATE_FINISH` | 1 (到位完成) |
| `STATE_RUNNING` | 2 (运动中) |
| `STATE_OVERLOAD` | 3 (过载) |
| `STATE_STALL` | 4 (堵转) |
| `STATE_NO_CALIB` | 5 (未标定) |

### 全局错误码

| 值 | 含义 | 触发条件 |
|----|------|---------|
| 0 | 正常 | - |
| 1 | 堵转 (Stall) | 堵转保护触发 |
| 4 | 急停 (EmergencyStop) | 收到0x89广播急停 |

### 步进参数

| 参数 | 值 |
|------|-----|
| `MOTOR_ONE_CIRCLE_HARD_STEPS` | 200 (1.8°步距角) |
| `SOFT_DIVIDE_NUM` | 256 (微步细分) |
| `MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS` | 51200 (200×256) |
| FOC Sin表大小 | 1025 entries (sin[0] ~ sin[2π]) |
| 编码器分辨率 | 14-bit MT6816 (16384 cpr) |

### 堵转保护机制

- **触发条件:** 电流饱和 (`|output| >= ratedCurrent`) 且速度极低 (`|estVelocity| < 1 step/s`) 持续1秒
- **动作:** 保持使能，原地锁死 (`goalPosition = realPosition`)，`errorCode = 1`
- **恢复:** 按 KEY2 清除标志

### 编码器标定流程

1. **触发:** KEY1+KEY2同时长按，或CAN命令0x02
2. **正向测量:** 慢速转动1圈，16次采样/步，记录所有编码器值
3. **反向测量:** 反向转动，去除回差
4. **数据处理:** 正向/反向数据取平均，生成16384项查找表
5. **Flash存储:** 烧录到 `0x08017C00` (32KB)，掉电不丢失

### 35/42电机参数切换

```cpp
// firmware/motor_fw_f103_42/UserApp/configurations.h
// 42电机固件专用配置：额定电流上限 2300mA = 2.3A

#ifdef MOTOR_TYPE_42
    #define DEFAULT_DCE_KP      200
    #define MAX_CURRENT_LIMIT   (2.3f * 1000)   // 42电机: 2300mA
#else
    #define DEFAULT_DCE_KP      195
    #define MAX_CURRENT_LIMIT   (2.0f * 1000)   // 35电机: 2000mA
#endif
```

### 电机固件硬件引脚

| 引脚 | 功能 |
|------|------|
| PA8/PA9/PA10 | ID0/ID1/ID2 拨码开关 |
| PA2/PA3/PA4/PA5 | TB67H450 INBM/INBP/INAM/INAP |
| PA7 | SIGNAL_COUNT_DIR (方向信号) |
| PB0 | SIGNAL_COUNT_EN (使能信号) |
| PB1 | SIGNAL_ALERT (报警信号) |
| PB2 | BUTTON2 |
| PB10/PB11 | HW_ELEC_BPWM/APWM (A/B相PWM) |
| PB12 | BUTTON1 |
| PA15 | SPI1_CS (MT6816片选) |
| PC13/PC14 | LED1/LED2 |
| PB8/PB9 | CAN1_RX/CAN1_TX |
| TIM2 CH3/CH4 | DAC输出 (TB67H450 VREF) |

### Flash存储布局 (STM32F103CBT6, 128KB)

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
  nodeID: 7 bits (0~127，实际使用 1~6, 8, 9)
  cmdCode: 7 bits

普通命令: cmdCode 0x00~0x7F (发往特定节点)
广播命令: cmdCode 0x80~0xBF (所有节点无条件响应)
```

### 命令分类总表

| 命令码 | 方向 | 功能 | 数据格式 |
|--------|------|------|---------|
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

- 主控发送: `StdId = 0x89` (`cmd=0x89`, 节点ID任意)
- 数据: 8字节全0
- 结果: 所有电机驱动板同时执行急停 (`requestMode = MODE_STOP`, `velocity = 0`, `current = 0`, `errorCode = 4`)

### CAN ID 快速查表

| 电机 | CAN ID | 说明 |
|------|--------|------|
| 地轨 | 9 | 普通命令（固定） |
| J1~J6 | 1~6 | 普通命令 |
| 夹爪 | 8 | 普通命令（固定） |
| 全部 | 广播 | 0x89急停, 0xA3查询 |

---

## ASCII 命令协议

### 系统命令 (`!` 前缀)

| 命令 | 功能 |
|------|------|
| `!START` | 使能机器人 |
| `!DISABLE` | 失能机器人 |
| `!STOP` | 广播急停 (StdId=0x89) |
| `!HOME` | 归零姿态 `{0, 0, 90, 0, 0, 0}` |
| `!RESET` | 待机姿态 `{0, -75, 180, 0, 0, 0}` |
| `!CALIBRATION` | 标定零点偏移 |
| `!HAND_O` | 打开夹爪 |
| `!HAND_C` | 关闭夹爪 |
| `!HAND_EN` | 使能夹爪 |
| `!HAND_DIS` | 失能夹爪 |
| `!HAND_ZERO` | 夹爪标定 |
| `!HAND_POS <0-100>` | 夹爪位置控制 |

### 查询命令 (`#` 前缀)

| 命令 | 功能 |
|------|------|
| `#GETJPOS` | 获取7个关节角度 (°) |
| `#GETLPOS` | 获取末端位姿 (x,y,z,a,b,c) |
| `#SET_DCE_KV/KP/KI/KD <node> <val>` | 设置电机PID参数 |
| `#REBOOT <node>` | 重启指定电机 |
| `#CMDMODE <1-6>` | 设置控制模式 |
| `#OFFSET_J <node>` | 应用零点偏移 |
| `#ACC_J <node> <val>` | 设置关节加速度 |
| `#SPEED_J <node> <val>` | 设置关节速度 |

### 运动命令

| 格式 | 功能 | 说明 |
|------|------|------|
| `>j1,j2,j3,j4,j5,j6,j7,speed` | 阻塞MoveJ | 等待到位后返回"ok" |
| `&j1,j2,j3,j4,j5,j6,j7,speed` | 非阻塞MoveJ | 立即返回"ok" |
| `@x,y,z,a,b,c,speed` | MoveL | 笛卡尔空间运动 (IK求解) |
| `$c1,c2,c3,c4,c5,c6,c7` | 力矩控制 | 7轴电流, **整数mA** |

> **注意:** `$` 命令中电流值为整数毫安(mA)，非安培。例如 `$0,0,0,0,0,0,200` 表示夹爪通电200mA。

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

## 已知问题

> 完整问题列表和修复状态见 `ISSUES.md`。

### P0 - 上电前必须修复 (共10个)

| ID | 问题 | 模块 |
|----|------|------|
| P0-1 | 失步检测使用`==`而非`>=`，堵转保护实际无效 | 电机驱动 |
| P0-2 | 力矩模式Disable后电流未清零 | 主控制器 |
| P0-3 | CAN中断回调与5kHz控制循环数据竞争 | 主控制器 |
| P0-4 | Python工具力矩命令格式错误(float vs int mA) | Python工具 |
| P0-5 | 急停不制动，电机滑行 | 电机驱动 |
| P0-6 | sqrtf负数参数导致未定义行为 | 电机驱动 |
| P0-7 | ApplyPositionAsHome未初始化CAN缓冲区 | 主控制器 |
| P0-8 | SetEnable设置错误状态值 | 主控制器 |
| P0-9 | 磁铁脱落检测到但无处理 | 电机驱动 |
| P0-10 | Flash写入后无验证 | 电机驱动 |

详见 `ISSUES.md`。

---

## 串口助手 (Python调试工具)

### 功能一览

| 分类 | 功能 |
|------|------|
| **连接** | 串口选择、115200bps、自动刷新 |
| **MoveJ** | 7滑块(°)+速度滑块、阻塞/非阻塞 |
| **MoveL** | XYZ(直线mm)+ABC(旋转°)+速度 |
| **力矩控制** | 7轴电流滑块 (mA) |
| **系统命令** | !START/DISABLE/STOP/HOME/RESET |
| **夹爪** | 开/闭/位置(0-100%) |
| **ServoJ测试** | 正弦波扫频 (50Hz) |
| **模式切换** | SEQ/INT/TRJ/Torque/Servo |
| **RGB LED** | 10种灯效、自定义颜色 |
| **查询** | 关节角度、末端位姿 |
| **地轨控制** | J7速度/加速度/电流 |

### 波特率支持

9600 / 115200 / 1000000 (1M)

---

## ESP32 模块说明

`esp32/` 目录包含 **1995个** Mongoose 网络编程示例，非生产代码:

- HTTP服务器/客户端、WebSocket
- MQTT (AWS IoT等)
- lwIP/Ethernet networking
- STM32 baremetal/FreeRTOS示例
- NXP/RP2040/Zephyr参考

> 如需WiFi监控或OTA升级，可参考 `esp32/firmware/examples/` 中相关示例。

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

### 运动学末端链路

```
Link0 (Base) → J1 (旋转) → J2 (肩) → J3 (肘) → J4 (腕旋) → J5 (腕俯) → J6 (腕偏) → Tool0
L_BASE=0.165m   L_ARM=0.170m  L_FORE=0.117m  L_WRIST=0.113m
```
