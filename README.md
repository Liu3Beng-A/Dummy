# Dummy 7-Axis Robotic Arm Control System

## 项目概述

本项目是一个基于 **STM32F405 (主控制器) + 8×STM32F103 (电机驱动板)** 的 **7轴机械臂 + 地轨 + 夹爪** 控制系统，运行 FreeRTOS 实时操作系统。

### 硬件组成

| 组件 | 芯片型号 | 数量 | 说明 |
| --- | --- | --- | --- |
| 主控制器 | STM32F405RG | ×1 | Cortex-M4, 168MHz, FreeRTOS |
| 电机驱动板 | STM32F103CBT6 | ×8 | Cortex-M3, 72MHz, FOC步进 |
| 地轨电机 | 丝杆1605, 57步进 | ×1 | 直连, 行程 -250~250mm, CAN ID=9 |
| 臂关节电机 | 42/35步进 | ×6 | 50:1减速, CAN ID 1~6 |
| 夹爪电机 | 35步进 | ×1 | 16:1减速, CAN ID=8 |
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

---

## 快速开始

### 1. 硬件准备

1. **主控制器 (STM32F405RG)** - 使用 ST-Link 烧录固件
2. **电机驱动板 (STM32F103CBT6 ×8)**
   - 地轨电机固件固定 CAN ID=9
   - 臂关节电机通过拨码开关设置 CAN ID (ID=1~6)
   - 夹爪固定 CAN ID=8
3. **CAN 总线连接** - 主控 CAN1 → 所有电机驱动板

### 2. 固件烧录

**主控固件 (ref_core_f405)**

```bash
cd firmware/ref_core_f405/build
ninja
# 烧录 build/*.bin 或 *.hex
```

**电机驱动固件 (motor_fw_f103_*)**

```bash
cd firmware/motor_fw_f103_35/build  # 35电机
# 或 motor_fw_f103_42/ 或 motor_fw_f103_57/
ninja
```

### 3. 电机编码器标定

**方法一：手动触发**
- 同时长按 KEY1 + KEY2 进入标定模式
- 慢速手动转动电机一圈
- 标定数据自动保存到 Flash

**方法二：CAN 命令触发**
- 发送 CAN 命令 0x02 触发标定

### 4. 电机类型切换 (35/42 步进)

修改 `firmware/motor_fw_f103_*/UserApp/configurations.h`：

```cpp
// 默认: 35电机
// 如需使用42电机，取消注释下一行：
// #define MOTOR_TYPE_42
```

---

## 使用方法

### Python 串口助手 (推荐)

```bash
python 串口助手.py
```

功能包括：MoveJ、MoveL、力矩控制、系统命令、夹爪控制、ServoJ 测试、RGB LED、状态查询。

### 串口直接发送命令

#### 系统命令 (`!` 前缀)

| 命令 | 功能 |
| --- | --- |
| `!START` | 使能机器人 |
| `!DISABLE` | 失能机器人 |
| `!STOP` | 急停 |
| `!HOME` | 归零姿态 |
| `!RESET` | 待机姿态 |
| `!CALIBRATION` | 标定零点偏移 |
| `!STALL_EN` | 开启堵转检测 |
| `!STALL_DIS` | 关闭堵转检测 |
| `!HAND_O` | 打开夹爪 |
| `!HAND_C` | 关闭夹爪 |
| `!HAND_POS <0-100>` | 夹爪位置控制 |

#### 运动命令

| 命令格式 | 功能 | 说明 |
| --- | --- | --- |
| `>j1,j2,j3,j4,j5,j6,j7,speed` | 阻塞 MoveJ | 等待到位后返回 "ok" |
| `&j1,j2,j3,j4,j5,j6,j7,speed` | 非阻塞 MoveJ | 立即返回 "ok" |
| `@x,y,z,a,b,c,speed` | MoveL | 笛卡尔空间运动 |
| `$c0,c1,c2,c3,c4,c5,c6` | 力矩控制 | 电流值 (A) |

**示例**：

```bash
!START              # 使能机器人
>0,-45,90,0,0,0,0,50   # MoveJ 到指定姿态
@200,0,300,0,0,0,50    # 笛卡尔空间运动
$0.5,0.5,0.5,0.2,0.2,0.1,0  # 力矩控制
!STOP               # 急停
```

#### 查询命令 (`#` 前缀)

| 命令 | 功能 |
| --- | --- |
| `#GETJPOS` | 获取关节角度 |
| `#GETLPOS` | 获取末端位姿 |
| `#SET_DCE_KP <node> <val>` | 设置电机 DCE_Kp |
| `#SET_PID <node> <kp> <kv> <ki> <kd>` | 一次性设置 4 个 PID 参数 |
| `#SPEED_RAIL <val>` | 设置地轨速度 |
| `#SPEED_J <node> <val>` | 设置关节速度 |
| `#ACC_J <node> <val>` | 设置关节加速度 |
| `#I_LIMIT_J <node> <val>` | 设置电流限幅 |

---

## CAN 通信协议

### CAN ID 格式

```
StdId = (nodeID << 7) | cmdCode

nodeID: 实际使用 1~6 (关节), 8 (夹爪), 9 (地轨)
cmdCode: 0x00~0x7F 普通命令, 0x80~0xBF 广播命令
```

### 命令总表

| 命令码 | 方向 | 功能 | 数据格式 |
| --- | --- | --- | --- |
| **0x01** | TX | 使能/失能电机 | uint32_t (0/1) |
| **0x02** | TX | 触发编码器标定 | - |
| **0x03** | TX | 设置电流 (力矩模式) | float A |
| **0x05** | TX | 设置位置 (梯形规划) | float rotations |
| **0x07** | TX | 位置+速度限制 | float pos + float vel |
| **0x0B** | TX | 设置堵转保护开关 | uint32_t (0/1) |
| **0x21~0x25** | RX | 查询电流/速度/位置/偏移/温度 | 见响应格式 |
| **0x7C** | RX | **堵转主动上报** | Data[0]=nodeID, Data[1]=1 |
| **0x89** | TX | **广播急停** | 8B全0 |

### 0x23 响应格式 (8字节)

```
Byte 0-3: float position   // 物理位置 (rotations)
Byte 4-5: int16_t current // FOC相电流 × 1000 (mA)
Byte 6:    uint8_t errorCode // 0=OK, 1=Stall, 4=EStop
Byte 7:    uint8_t isFinished // 0=运动中, 1=到位
```

---

## 主控固件架构

### FreeRTOS 线程

| 线程名 | 优先级 | 频率 | 职责 |
| --- | --- | --- | --- |
| `ControlLoopFixUpdateTask` | osPriorityRealtime | 5kHz (TIM7 200μs) | 实时下发电机指令 |
| `ControlLoopUpdateTask` | osPriorityNormal | 事件触发 | 解析并执行ASCII命令 |
| `OledTask` | osPriorityNormal | ~60Hz | OLED显示刷新 |
| `RGBTask` | osPriorityNormal | 33Hz | RGB LED灯效 |

### 步进电机关键参数

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `MOTOR_ONE_CIRCLE_HARD_STEPS` | 200 | 1.8° 步距角 |
| `SOFT_DIVIDE_NUM` | 256 | 微步细分 |
| `MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS` | 51200 | 200×256 |
| `RAIL_STEPS_PER_MM` | **40960** | 地轨步数当量 (丝杆1605直连) |

### 地轨步数换算

```
1mm = (步数/圈 × 微步数) / 丝杆导程
     = (200 × 256) / 5
     = 40960 步/mm
```

---

## 电机驱动固件架构

### 双定时器架构

| 定时器 | 频率 | 职责 |
| --- | --- | --- |
| TIM1 | 100Hz | 按钮去抖、LED状态、温度采集 |
| TIM4 | 20kHz | 电机FOC控制环 |

### 堵转检测机制

**触发条件**：
1. 电流 >= 额定电流 × 95%
2. 速度 < 10240 步/s (即 < 1/5 圈)
3. 以上条件同时满足持续 **1秒**

**动作**：
- 电机上报 0x7C 堵转通知
- 主控 RGB 切换红色心跳
- 锁定当前位置，清空命令队列

### Flash 存储布局 (STM32F103CBT6, 128KB)

| 地址 | 大小 | 用途 |
| --- | --- | --- |
| 0x08000000 | 47KB | 应用程序 |
| 0x08017C00 | 32KB | 编码器标定数据 |
| 0x0801FC00 | 1KB | 用户配置/EEPROM |

---

## 硬件引脚参考

### 电机驱动板引脚

| 引脚 | 功能 | 说明 |
| --- | --- | --- |
| PA8~PA10 | ID0~ID2 | 拨码开关设置 CAN ID |
| PA2~PA5 | INBM/INBP/INAM/INAP | TB67H450 控制 |
| PB10/PB11 | BPWM/APWM | A/B相PWM |
| PA15 | SPI1_CS | MT6816 片选 |
| PB8/PB9 | CAN1_RX/TX | CAN通信 |
| PC13/PC14 | LED1/LED2 | 状态灯 |

---

## 编译说明

### 主控固件

```bash
cd firmware/ref_core_f405
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi.cmake ..
ninja
```

### 电机驱动固件

```bash
cd firmware/motor_fw_f103_35  # 或 42/57/gripper
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../cmake/arm-none-eabi.cmake ..
ninja
```

---

## 关键文档

| 文档 | 说明 |
| --- | --- |
| `PROJECT_CONTEXT.md` | 自动生成的项目上下文（代码分析） |
| `README.md` | 本文档 |
| `TODO.md` | 功能路线图 |

---

## 项目目标

- [x] 7轴关节空间运动 (MoveJ)
- [x] 笛卡尔空间运动 (MoveL via FK/IK)
- [x] 力矩控制模式 ($ 命令)
- [x] ServoJ 实时伺服模式
- [x] 地轨线性滑轨控制
- [x] 夹爪控制
- [ ] ROS2 / MoveIt2 接入
- [ ] 碰撞检测与保护
- [ ] 视觉抓取集成
