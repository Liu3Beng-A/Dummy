# Dummy 7-Axis Robotic Arm Control System

## 项目概述

本项目是一个基于 **STM32F405 (主控制器) + 8×STM32F103 (电机驱动板)** 的 **7轴机械臂 + 地轨 + 夹爪** 控制系统，运行 FreeRTOS 实时操作系统，支持关节空间运动、笛卡尔空间运动、力矩控制和实时伺服模式。

### 硬件组成

| 组件 | 芯片型号 | 数量 | 说明 |
| --- | --- | --- | --- |
| 主控制器 | STM32F405RG | ×1 | Cortex-M4, 168MHz, FreeRTOS |
| 电机驱动板 | STM32F103CBT6 | ×8 | Cortex-M3, 72MHz, FOC步进 |
| 地轨电机 | 57步进、丝杆1605 | ×1 | 直连, 行程 -250~250mm, CAN ID=9 (固定) |
| 臂关节电机 | 42/35步进 | ×6 | 50:1减速, CAN ID 1~6 |
| 夹爪电机 | 35步进 | ×1 | 16:1减速, CAN ID=8 (固定) |
| 通信总线 | CAN1 | 500kbps | 主控与所有电机通信 |
| 调试串口 | UART4 | 115200bps | 主控命令接口 |
| USB | USB_OTG_FS | CDC/VCP | 备用命令接口 |
| OLED | SSD1306 | 128×64 | 板载显示 |
| IMU | MPU6050 | ×1 | 惯性测量 |

### 机器人轴配置

| 索引 | CAN ID | 名称 | 类型 | 减速比 | 运动范围 |
| --- | --- | --- | --- | --- | --- |
| 0 | **9** | Rail | 线性(mm) | 直连(1:1) | -250 ~ 250mm |
| 1 | 1 | J1 | 旋转(°) | 50:1 | ±175° |
| 2 | 2 | J2 | 旋转(°) | 50:1 | -75° ~ +90° |
| 3 | 3 | J3 | 旋转(°) | 50:1 | 0° ~ 180° |
| 4 | 4 | J4 | 旋转(°) | 50:1 | ±270° |
| 5 | 5 | J5 | 旋转(°) | 50:1 | ±100° |
| 6 | 6 | J6 | 旋转(°) | 30:1 | ±180° |
| 7 | **8** | Hand | 夹爪(%) | 16:1 | 0% ~ 100% |

### DH 运动学参数

| 参数 | 值 | 说明 |
| --- | --- | --- |
| L_BASE | 0.165m | 基座高度 |
| D_BASE | 0.0m | 基座偏移 |
| L_ARM | 0.170m | 大臂长度 |
| L_FOREARM | 0.117m | 小臂长度 |
| D_ELBOW | 0.0695m | 肘部偏移 |
| L_WRIST | 0.113m | 腕部长度 |

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
       │ (StdId = (id<<7) | cmd)          ▼
       │                         ┌──────────────────────────┐
       │                         │  STM32F103 电机驱动板 ×8  │
       │                         │  (72MHz, FOC, 20kHz环)   │
       │                         │                           │
       │                         │  ID=9: 地轨 (-250~250mm)   │
       │                         │  ID=1~6: 臂关节 (J1~J6)  │
       │                         │  ID=8: 夹爪              │
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
│  motorJ[1~6]: 臂关节 (° → step)                │
│  hand: 夹爪 (0~100%)                            │
│  dof6Solver: 6-DOF FK/IK                        │
└──────────────────────┬──────────────────────────┘
                       │ CAN (StdId = id<<7 | cmd)
┌──────────────────────▼──────────────────────────┐
│           CtrlStepMotor (关节抽象层)             │
│  SetAngle / SetVelocity / SetCurrent           │
│  UpdateAngleCallback (CAN响应路由)              │
└──────────────────────┬──────────────────────────┘
                       │ CAN1 (500kbps)
┌──────────────────────▼──────────────────────────┐
│        STM32F103 电机固件 (×8独立运行)           │
│                                                  │
│  TIM1(100Hz): 按钮/LED/温度采集                 │
│  TIM4(20kHz): 电机控制环 (FOC)                  │
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

## 目录结构

```
e:/Dummy-code/
├── firmware/
│   ├── ref_core_f405/          # 主控固件 (STM32F405, FreeRTOS)
│   │   ├── UserApp/           # 应用层 (main.cpp, protocols/)
│   │   ├── Robot/             # 机器人抽象层
│   │   │   ├── instances/    # DummyRobot 7轴实现
│   │   │   ├── algorithms/    # 运动学 (FK/IK), 轨迹规划
│   │   │   └── actuators/     # 关节电机抽象 (CtrlStep)
│   │   ├── Bsp/              # 板级驱动 (CAN/UART/USB/I2C/SPI)
│   │   ├── 3rdParty/         # 第三方库 (Fibre, U8G2)
│   │   └── Core/             # STM32 HAL 初始化
│   │
│   └── motor_fw_f103/          # 电机驱动固件 (STM32F103, 8板)
│       ├── UserApp/           # main.cpp, CAN/UART 协议
│       ├── Ctrl/             # 电机控制逻辑
│       │   ├── Motor/        # 电机类 + 运动规划器
│       │   ├── Driver/       # TB67H450 FOC 驱动基类
│       │   ├── Sensor/       # MT6816 磁编 + 标定器
│       │   └── Signal/       # 按钮 / LED
│       ├── Port/             # STM32 硬件端口实现
│       └── Core/             # STM32 HAL 初始化
│
├── tools/
│   └── 串口助手.py             # Python 调试工具 (Tkinter GUI)
├── 串口助手.py                  # 根目录副本
├── PROJECT_CONTEXT.md          # 自动生成的项目上下文
└── review.md                   # 系统级代码审查报告
```

---

## 快速开始

### 1. 硬件准备

1. **主控制器 (STM32F405RG)**
  - 使用 ST-Link 或 J-Link 烧录固件
  - 连接 USB 到 PC（CDC/VCP 模式）
  - 或使用 UART4 (115200bps) 连接 PC
2. **电机驱动板 (STM32F103CBT6 ×8)**
  - 地轨电机固件固定 CAN ID=9（不依赖拨码开关）
  - 臂关节电机通过拨码开关设置 CAN ID (PA8-PA10)
  - ID=1~6: J1~~J6; ID=8: 夹爪（固定）
  - CAN 总线终端需要 120Ω 终端电阻
3. **CAN 总线连接**
  - 主控 CAN1_H/L → 所有电机驱动板 CAN_H/CAN_L
  - 建议使用屏蔽双绞线，500kbps 波特率

### 2. 固件烧录

**主控固件 (ref_core_f405)**

```bash
# 使用 OpenOCD 或 ST-Link 工具烧录
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c "program build/Core-STM32F4-fw.elf verify reset exit"
```

**电机驱动固件 (motor_fw_f103)**

```bash
# 为臂关节电机烧录固件（CAN ID 由拨码开关决定：ID=1~6）；夹爪固定 ID=8；地轨固定 ID=9
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
    -c "program motor_fw_f103/build/Ctrl-Step-STM32-fw.elf verify reset exit"
```

### 3. 电机编码器标定

每个电机驱动板首次使用前需要标定磁编：

**方法一：手动触发**

- 同时长按 KEY1 + KEY2 进入标定模式
- 慢速手动转动电机一圈
- 标定数据自动保存到 Flash

**方法二：CAN 命令触发**

```bash
# 发送 CAN 命令 0x02 触发标定
```

### 4. 电机类型切换 (35/42 步进)

编译前修改 `motor_fw_f103/Ctrl/Config/configurations.h`：

```cpp
// 默认: 35电机
// 如需使用42电机，取消注释下一行：
// #define MOTOR_TYPE_42

#ifdef MOTOR_TYPE_42
    #define DEFAULT_DCE_KP      200
    #define MAX_CURRENT_LIMIT   (2 * 1000)   // 42电机: 2000mA
#else
    #define DEFAULT_DCE_KP      195
    #define MAX_CURRENT_LIMIT   (int32_t)(1.2f * 1000)  // 35电机: 1200mA
#endif
```

---

## 使用方法

### 方法一：Python 串口助手 (推荐)

运行 `串口助手.py`（根目录或 tools 目录下）：

```bash
python 串口助手.py
```

功能包括：

- **MoveJ**: 7个滑块控制关节角度 + 速度滑块
- **MoveL**: XYZ + ABC + 速度（笛卡尔空间运动）
- **力矩控制**: 7轴电流滑块（mA）
- **系统命令**: !START / !DISABLE / !STOP / !HOME / !RESET
- **夹爪控制**: 开/关/位置 (0-100)
- **ServoJ 测试**: 正弦波扫频
- **模式切换**: SEQ / INT / TRJ / Torque / Servo
- **状态查询**: 关节角度、末端位姿

### 方法二：串口直接发送命令

通过 UART4 或 USB CDC 直接发送 ASCII 命令：

#### 系统命令 (`!` 前缀)

| 命令 | 功能 |
| --- | --- |
| `!START` | 使能机器人 |
| `!DISABLE` | 失能机器人 |
| `!STOP` | 广播急停 (StdId=0x89) |
| `!HOME` | 归零姿态 (0, 0, 90, 0, 0, 0, 0mm) |
| `!RESET` | 待机姿态 (0, -75, 180, 0, 0, 0, 0mm) |
| `!CALIBRATION` | 标定零点偏移 |
| `!HAND_O` | 打开夹爪 |
| `!HAND_C` | 关闭夹爪 |
| `!HAND_EN` | 使能夹爪 |
| `!HAND_DIS` | 失能夹爪 |
| `!HAND_ZERO` | 夹爪标定 |
| `!HAND_POS <0-100>` | 夹爪位置控制 |

#### 运动命令

| 命令格式 | 功能 | 说明 |
| --- | --- | --- |
| `>j1,j2,j3,j4,j5,j6,j7,speed` | 阻塞 MoveJ | 等待到位后返回 "ok" |
| `&j1,j2,j3,j4,j5,j6,j7,speed` | 非阻塞 MoveJ | 立即返回 "ok" |
| `@x,y,z,a,b,c,speed` | MoveL | 笛卡尔空间运动 (IK求解) |
| `$c1,c2,c3,c4,c5,c6,c7` | 力矩控制 | 7轴电流值 (mA) |

**参数说明**：

- `j1~j6`: 关节角度 (°)，`j7`: 地轨位置 (mm)
- `speed`: 运动速度 (°/s 或 mm/s)
- `x,y,z`: 末端位置 (mm)
- `a,b,c`: 末端姿态 (°)
- `c1~c7`: 电机电流 (mA)

**示例**：

```bash
# 使能机器人
!START

# MoveJ 到指定姿态 (阻塞)
>0,-45,90,0,0,0,100,50

# MoveJ 非阻塞
&0,-45,90,0,0,0,100,50

# 笛卡尔空间运动
@200,0,300,0,0,0,50

# 力矩控制
$500,500,500,200,200,100,200

# 待机姿态
!RESET

# 急停
!STOP
```

#### 查询命令 (`#` 前缀)

| 命令 | 功能 |
| --- | --- |
| `#GETJPOS` | 获取关节角度 |
| `#GETLPOS` | 获取末端位姿 |
| `#SET_DCE_KV <node> <val>` | 设置电机 DCE_Kv |
| `#SET_DCE_KP <node> <val>` | 设置电机 DCE_Kp |
| `#SET_DCE_KI <node> <val>` | 设置电机 DCE_Ki |
| `#SET_DCE_KD <node> <val>` | 设置电机 DCE_Kd |
| `#REBOOT <node>` | 重启电机 |
| `#CMDMODE <1-6>` | 设置控制模式 |
| `#OFFSET_J <node>` | 应用零点偏移 |
| `#ACC_J <node> <val>` | 设置关节加速度 |
| `#SPEED_J <node> <val>` | 设置关节速度 |

#### 控制模式切换

| 模式 | 命令 | 说明 |
| --- | --- | --- |
| SEQ | `>xxx` 自动切换 | 顺序点动 (阻塞) |
| INT | `&xxx` 自动切换 | 可打断点动 (默认) |
| TRJ | `#CMDMODE 3` | 连续轨迹模式 |
| TUN | `#CMDMODE 4` | 电机参数扫频调谐 |
| TRQ | `$xxx` 自动切换 | 力矩/电流控制 |
| SRV | `#CMDMODE 6` | 高频关节伺服 |

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

### 命令总表

| 命令码 | 方向 | 功能 | 数据格式 |
| --- | --- | --- | --- |
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
Byte 4-5: int16_t current  // FOC相电流 × 1000 (mA)
Byte 6:    uint8_t errorCode // 0=OK, 1=Stall, 4=EStop
Byte 7:    uint8_t isFinished // 0=运动中, 1=到位
```

### CAN 急停广播

```
主控发送: StdId = 0x89 (cmd=0x89, 节点ID任意)
数据: 8字节全0
结果: 所有电机驱动板同时执行急停
  - requestMode = MODE_STOP
  - velocity = 0, current = 0
  - errorCode = 4
```

---

## 主控固件架构

### FreeRTOS 线程

| 线程名 | 优先级 | 栈大小 | 频率 | 职责 |
| --- | --- | --- | --- | --- |
| `ControlLoopFixUpdateTask` | `osPriorityRealtime` | 2000B | 5kHz (TIM7 200μs) | 实时下发电机指令 |
| `KinematicsTask` | `osPriorityHigh` | 2048B | 1kHz | FK正向运动学计算 |
| `MotorStateMonitorTask` | `osPriorityNormal` | 2048B | 100Hz | 广播查询所有电机状态 |
| `ControlLoopUpdateTask` | `osPriorityNormal` | 2000B | 事件触发 | 解析并执行ASCII命令 |
| `OledTask` | `osPriorityNormal` | 2000B | ~60Hz | OLED显示刷新 |
| `RGBTask` | `osPriorityNormal` | 2000B | 33Hz | RGB LED灯效 |

### 步进电机关键参数

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `MOTOR_ONE_CIRCLE_HARD_STEPS` | 200 | 1.8° 步距角 |
| `SOFT_DIVIDE_NUM` | 256 | 微步细分 |
| `MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS` | 51200 | 200×256 |
| `RAIL_STEPS_PER_MM` | 40960 | 地轨步数当量 (丝杆1605直连, 256微步) |
| FOC Sin表大小 | 1025 entries | sin[0] ~ sin[2π] |
| 编码器分辨率 | 14-bit (MT6816) | 16384 cpr |

### 地轨步数换算

```
1mm = (步数/圈 × 微步数) / 丝杆导程
     = (200 × 256) / 5
     = 40960 步/mm
```

---

## 电机驱动固件架构

### 双定时器架构

| 定时器 | 频率 | 回调 | 职责 |
| --- | --- | --- | --- |
| TIM1 | 100Hz (10ms) | `Tim1Callback100Hz()` | 按钮去抖、LED状态、温度采集 |
| TIM4 | 20kHz (50μs) | `Tim4Callback20kHz()` | 电机FOC控制环 / 编码器标定 |

### 电机控制模式

| 模式 | CMD | 控制方法 | 备注 |
| --- | --- | --- | --- |
| `MODE_STOP` | - | 失能 | |
| `MODE_COMMAND_POSITION` | 0x05/0x07 | DCE + 梯形规划 | 带速度限制 |
| `MODE_COMMAND_VELOCITY` | 0x04 | PID 速度环 | |
| `MODE_COMMAND_CURRENT` | 0x03 | 直接电流给定 | |
| `MODE_COMMAND_Trajectory` | - | 轨迹跟踪 | 外部给轨迹点 |
| `MODE_PWM_*` | - | PWM开环 | 调试用 |
| `MODE_STEP_DIR` | - | STEP/DIR 模式 | |

### 电机状态

| 状态 | 枚举值 | 说明 |
| --- | --- | --- |
| `STATE_STOP` | 0 | 停止 |
| `STATE_FINISH` | 1 | 到位完成 |
| `STATE_RUNNING` | 2 | 运动中 |
| `STATE_OVERLOAD` | 3 | 过载 |
| `STATE_STALL` | 4 | 堵转 |
| `STATE_NO_CALIB` | 5 | 未标定 |

### 堵转保护机制

- **触发条件**: 电流饱和 (`|output| >= ratedCurrent`) 且速度极低 (`|estVelocity| < 1 step/s`) 持续 1 秒
- **动作**: 保持使能，原地锁死 (`goalPosition = realPosition`)，`errorCode=1`
- **恢复**: 按 KEY2 清除标志

### Flash 存储布局 (STM32F103CBT6, 128KB)

| 地址 | 大小 | 用途 |
| --- | --- | --- |
| 0x08000000 | 47KB | 应用程序 |
| 0x0800BC00 | 1KB | DAPLink配置 |
| 0x08017C00 | 32KB | 编码器标定数据 |
| 0x0801FC00 | 1KB | 用户配置/EEPROM |

### 编码器标定流程

1. **触发条件**: KEY1+KEY2同时长按，或CAN命令0x02
2. **正向测量**: 慢速转动1圈，16次采样/步，记录所有编码器值
3. **反向测量**: 反向转动，去除回差
4. **数据处理**: 对正向/反向数据取平均，生成16384项查找表
5. **Flash存储**: 烧录到 `0x08017C00`，掉电不丢失

---

## 控制算法

### DCE 控制算法 (位置+速度复合环)

```
FOC_current = DCE_Kp × position_error
            + DCE_Kv × velocity_error
            + DCE_Ki × ∫position_error
            + DCE_Kd × d(position_error)/dt
```

**默认参数**: Kp=195/200, Kv=80, Ki=300, Kd=250

### 梯形速度规划

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

---

## 关键常量参考

| 常量 | 值 | 说明 |
| --- | --- | --- |
| `RAIL_STEPS_PER_MM` | 40960 | 地轨步数当量 (直连) |
| `RAIL_DEFAULT_SPEED_MM_S` | 20.0f | 地轨默认速度 mm/s |
| `DEFAULT_JOINT_SPEED` | 80.0f | 关节默认速度 °/s |
| `DEFAULT_JOINT_ACCELERATION_LOW` | 5.0f | 低加速度 °/s² |
| `DEFAULT_JOINT_ACCELERATION_HIGH` | 100.0f | 高加速度 °/s² |
| `REST_POSE` | {0, -75, 180, 0, 0, 0} | 待机姿态 |
| `HOME_POSE` | {0, 0, 90, 0, 0, 0} | 归零姿态 |

---

## ROS2 / MoveIt2 接入 (规划中)

当前协议为 ASCII 文本，ROS2 接入需要：

1. **协议层**: 将 ASCII 协议替换为 Fibre/CAN 或自定义二进制协议
2. **桥接节点**: Python/C++ 节点在 PC 端解析 ROS2 指令 → CAN 帧
3. **MoveIt2 配置**: URDF + SRDF + 6-DOF 运动学插件
4. **控制器**: JointTrajectoryController 替代当前直接位置控制
5. **状态反馈**: 将 CAN 0x23 回读数据发布为 `/joint_states`

详见 `review.md` 第11节。

---

## 已知问题

系统级代码审查报告 `review.md` 记录了以下等级的问题：

### Critical (需立即修复)

| ID | 问题名称 | 位置 |
| --- | --- | --- |
| C-1 | IK/FK 在 5kHz 实时环中执行 | `main.cpp:266` |
| C-2 | CAN 轮询模式带宽不匹配 | `dummy_robot.cpp:252` |
| C-3 | `motorJ[ALL]` 只查询地轨电机 | `dummy_robot.cpp:252` |

### High (高优先级)

| ID | 问题名称 | 位置 |
| --- | --- | --- |
| H-1 | 控制循环无超时保护 | `main.cpp:254` |
| H-2 | CAN 发送无超时保护 | `interface_can.cpp:241` |
| H-3 | ServoJ 模式 1ms 时间限制 | `dummy_robot.cpp:228` |
| H-4 | CAN 类型转换安全隐患 | `can_protocol.cpp:33-36` |
| H-5 | 电机使能逻辑错误 | `interface_can.cpp:33-43` |
| H-6 | CAN TX 信号量无限期等待 | `interface_can.cpp:241` |

详见 `review.md`。

---

## 项目目标

- 7轴关节空间运动 (MoveJ)
- 笛卡尔空间运动 (MoveL via FK/IK)
- 力矩控制模式 ($ 命令)
- ServoJ 实时伺服模式
- 地轨线性滑轨控制
- 夹爪控制
- ROS2 / MoveIt2 接入
- 视觉抓取集成
- EtherCAT 升级（规划中）

---

## 编译说明

### 主控固件

```bash
cd firmware/ref_core_f405
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi.cmake ..
make -j$(nproc)
```

### 电机驱动固件

```bash
cd firmware/motor_fw_f103
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi.cmake ..
make -j$(nproc)
```

---

## 硬件引脚参考

### 电机驱动板引脚

| 引脚 | 功能 | 说明 |
| --- | --- | --- |
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
