# 地轨扩展与 CAN 广播协议修改说明

> 本次修改为项目添加了地轨（线性滑轨）支持，并将 CAN 广播地址从 ID=0 改为命令码区间广播，彻底解决了 ID=0 无法同时作为地轨电机地址和广播地址的冲突问题。

**文档版本：v2（2026-05-22，反映项目重构后状态）**

---

## 目录

- [1. 修改背景](#1-修改背景)
- [2. 7轴系统架构](#2-7轴系统架构)
- [3. 修改内容](#3-修改内容)
- [4. 详细改动说明](#4-详细改动说明)
- [5. 35 / 42 电机固件切换](#5-35--42-电机固件切换)
- [6. 使用方法](#6-使用方法)
- [7. CAN 通信协议](#7-can-通信协议)
- [8. 修改文件清单](#8-修改文件清单)

---

## 1. 修改背景

### 1.1 原有架构的问题

原有 CAN 广播机制使用 `nodeID=0` 作为广播地址，所有电机固件监听 `id == 0` 实现急停广播。但系统中 `motorJ[0]` 规划为地轨（线性滑轨）电机，需要使用 `nodeID=0`。两者冲突，无法同时支持。

### 1.2 解决方案

使用 **命令码高 bit 区间**（`cmd >= 0x80 && cmd <= 0x8F`）作为广播命令标识，所有电机固件对落入此区间的命令码无条件响应，ID=0 得以释放给地轨电机使用。

### 1.3 原有 Bug 修复

本次修改同时修复了以下原有代码中的 bug：

- `**$` 力矩命令**：sscanf 缓冲区 `float cur[6]` 越界（读 7 个值），改为 `float cur[7]`
- `**HOME_POSE` / `REST_POSE`**：宏未定义，`Homing()` 和 `Resting()` 调用 `MoveJ` 少传 1 个参数（j7 地轨）
- `**ServoJ` 测试循环**：命令末尾多余 `\n`（`send_cmd` 已自动加 `\n`）

---

## 2. 7轴系统架构

### 2.1 电机 ID 分配


| ID    | 类型     | 名称  | 说明                               |
| ----- | ------ | --- | -------------------------------- |
| **0** | **地轨** | J7  | 丝杆 1605，1圈=5mm，减速比 50，行程 0~500mm |
| 1     | 42 电机  | J1  | 底座旋转，±175°                       |
| 2     | 42 电机  | J2  | 肩部，-75°~90°                      |
| 3     | 42 电机  | J3  | 肘部，0°~180°                       |
| 4     | 42 电机  | J4  | 腕部旋转，±270°                       |
| 5     | 42 电机  | J5  | 腕部俯仰，±100°                       |
| 6     | 42 电机  | J6  | 腕部偏转，±180°，减速比 30                |
| 7     | 35 电机  | 夹爪  | 减速比 16                           |


### 2.2 地轨管理策略

地轨（J7）**不纳入 6-DOF 运动学求解器**，单独管理：

- `Joint6D_t.a[0-5]` → 臂关节 J1-J6（由 `DOF6Kinematic` 处理）
- `currentRailPos` / `targetRailPos` → 地轨 J7 位置（单位 mm，单独管理）
- `MoveJoints()` → 下发臂关节指令（200us周期，由电机固件内部做运动规划）
- `MoveRail()` → 下发地轨指令（mm → step）
- `MoveL` → 不控制地轨（保持 rail 位置不变，逆解由外部计算）
- `MoveJ` / `ServoJ` / `$` 力矩 → 可同时控制地轨

**运动控制架构（无轨迹规划器）**：

```
控制环(5kHz, 200us周期)
    │
    ▼
MoveJ / ServoJ / $ → 解析命令，设置 targetJoints / targetRailPos
    │
    ▼
每200us: MoveJoints(targetJoints) + MoveRail(targetRailPos)
    │
    ▼
CAN总线 → 各电机固件内部执行速度/加速度规划
    │
    ▼
UpdateJointAnglesCallback() → 读回实际位置，更新 jointsStateFlag
```

### 2.3 地轨步进当量

```
减速比: 50:1
丝杆: 1605 → 1圈 = 5mm
微步细分: 256 (1/256 步进)

1mm = 50/5 × 200 × 256 = 50 × 51200 / 5 = 512000 步
```

---

## 3. 修改内容

### 3.1 主控固件 (`dummy-ref-core-fw`)


| 文件                                | 修改内容                                                                                                                                                                                                                                                      |
| --------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Robot\instances\dummy_robot.h`   | 新增 `targetRailCurrent`、`currentRailPos`/`targetRailPos`；`MoveJ`/`ServoJ`/`SetJointCurrents` 改为 7 参数；新增 `COMMAND_TORQUE_CONTROL`(=5) 和 `COMMAND_SERVO_J`(=6) 模式；`MoveRail()` 声明                                                                            |
| `Robot\instances\dummy_robot.cpp` | `motorJ[0]` 改为地轨电机(ID=0, reduction=50, 0~500mm)；`MoveJ`/`ServoJ` 新增 j7 参数和地轨控制；`MoveRail()` 下发地轨(mm→step)；`ParseCommand` sscanf 全部改为 7 参数；`SetJointCurrents` 支持 7 轴；`Homing()`/`Resting()` 传递 7 参数(地轨=0mm)；`UpdateJointAnglesCallback` 更新地轨位置和bits[0]状态标志 |
| `UserApp\main.cpp`                | 控制环(5kHz)：`$` 力矩模式和 ServoJ 模式支持 7 轴（地轨+臂关节）；OLED 显示地轨位置和 7 轴状态                                                                                                                                                                                            |


### 3.2 电机固件 (`dummy-motor-fw`)


| 文件                                    | 修改内容                                            |
| ------------------------------------- | ----------------------------------------------- |
| `Core\Src\can.c`                      | CAN 接收回调：广播判断改为 `cmd >= 0x80 && cmd <= 0x8F`    |
| `UserApp\protocols\interface_can.cpp` | 新增 `case 0x89:` 广播急停处理；修复多处 `break;` 缺少分号       |
| `UserApp\main.cpp`                    | 默认参数改用 `MAX_CURRENT_LIMIT` 和 `DEFAULT_DCE_KP` 宏 |


### 3.3 Python 串口助手 (`串口助手.py`)


| 修改内容                                                     |
| -------------------------------------------------------- |
| MoveJ 标题改为 `>j1~j6, j7(地轨mm), speed`，新增 J7 地轨滑块（0~500mm） |
| `send_movej()` 参数从 6 轴改为 7 轴（末尾加 j7）                     |
| 力矩控制从 6 轴扩展到 7 轴                                         |
| ServoJ 测试基准位姿从 6 轴改为 7 轴                                 |


---

## 4. 详细改动说明

### 4.1 主控固件 - 7 轴电机初始化

**文件：** `dummy_robot.cpp` 构造函数

```cpp
DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) : hcan(_hcan)
{
    // motorJ[0]: 地轨（线性滑轨）
    // 丝杆1605，转1圈=5mm，减速比50，行程0~500mm
    motorJ[0] = new CtrlStepMotor(_hcan, 0, false, 50, 0, 500);

    // motorJ[1-6]: 臂关节
    motorJ[1] = new CtrlStepMotor(_hcan, 1, false, 50, -175, 175);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, true,  50,  -75,  90);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, true,  50,    0, 180);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, true,  50, -270, 270);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, true,  50, -100, 100);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, true,  30, -180, 180);

    hand = new StepHand(_hcan, 7);

    // 地轨位置初始化（mm）
    currentRailPos = 0.0f;
    targetRailPos  = 0.0f;
    lastCmdRailPos = 0.0f;

    // 挂载轨迹规划器
    trajPlanner.SetKinematicSolver(dof6Solver);
}
```

### 4.2 主控固件 - 7 轴 MoveJ

**文件：** `dummy_robot.cpp`

```cpp
bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6, float _j7_mm)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);

    // 地轨限位检查
    if (_j7_mm > motorJ[0]->angleLimitMax || _j7_mm < motorJ[0]->angleLimitMin)
        return false;

    // 臂关节限位检查
    for (int j = 1; j <= 6; j++)
        if (targetJointsTmp.a[j-1] > motorJ[j]->angleLimitMax || ...)
            return false;

    // 计算各轴速度（保证所有关节同时到达）
    DOF6Kinematic::Joint6D_t deltaJoints = targetJointsTmp - currentJoints;
    uint8_t index;
    float maxAngle = AbsMaxOf6(deltaJoints, index);
    float time = maxAngle * (float)(motorJ[index + 1]->reduction) / jointSpeed;
    for (int j = 1; j <= 6; j++)
        dynamicJointSpeeds.a[j - 1] =
            fabsf(deltaJoints.a[j - 1] * (float)(motorJ[j]->reduction) / time * 0.1f);

    jointsStateFlag = 0;
    targetJoints = targetJointsTmp;
    targetRailPos = _j7_mm;  // 存储地轨目标位置
    return true;
}
```

### 4.3 主控固件 - MoveRail（下发地轨）

**文件：** `dummy_robot.cpp`

```cpp
void DummyRobot::MoveRail(float _railPos_mm)
{
    // 地轨：mm → step，直接下发位置（由电机固件内部做速度规划）
    float rail_steps = _railPos_mm * RAIL_STEPS_PER_MM;
    motorJ[0]->SetAngleWithVelocityLimit(rail_steps, 20.0f);
}
```

### 4.4 主控固件 - 7 轴力矩控制

**文件：** `dummy_robot.cpp`

```cpp
void DummyRobot::SetJointCurrents(float c1, float c2, float c3, float c4, float c5, float c6, float c7)
{
    targetCurrents[0] = c1;  // J1
    targetCurrents[1] = c2;  // J2
    targetCurrents[2] = c3;  // J3
    targetCurrents[3] = c4;  // J4
    targetCurrents[4] = c5;  // J5
    targetCurrents[5] = c6;  // J6
    targetRailCurrent = c7;  // J7 地轨电流

    if (!isEnabled) SetEnable(true);

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetCurrentSetPoint(targetCurrents[i - 1]);
    motorJ[0]->SetCurrentSetPoint(targetRailCurrent);  // 地轨电流单独下发
}
```

### 4.5 主控固件 - ParseCommand sscanf 7 参数

**文件：** `dummy_robot.cpp`

```cpp
// $ 力矩控制: $c1,c2,c3,c4,c5,c6,c7(地轨)
if (_cmd[0] == '$') {
    float cur[7];  // c1~c6: 臂关节, c7: 地轨电流
    argNum = sscanf(_cmd, "$%f,%f,%f,%f,%f,%f,%f",
                    &cur[0], &cur[1], &cur[2], &cur[3], &cur[4], &cur[5], &cur[6]);
    if (argNum == 7)
        context->SetJointCurrents(cur[0], cur[1], cur[2], cur[3], cur[4], cur[5], cur[6]);
    return osMessageQueueGetSpace(commandFifo);
}

// > MoveJ: >j1,j2,j3,j4,j5,j6,j7(地轨mm),speed
float joints[6];
float j7 = 0.0f;
float speed = 0.0f;
argNum = sscanf(_cmd, ">%f,%f,%f,%f,%f,%f,%f,%f",
                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &j7, &speed);
if (argNum >= 7) {
    if (argNum == 8) context->SetJointSpeed(speed);
    context->MoveJ(joints[0], joints[1], joints[2],
                   joints[3], joints[4], joints[5], j7);
    // ...
}
```

### 4.6 主控固件 - 控制环线程

**文件：** `main.cpp` - `ThreadControlLoopFixUpdate`

```cpp
// 轨迹: 2ms 推进一次 | 关节角度: 50ms 更新 | 位姿: 100ms 更新
case DummyRobot::COMMAND_TORQUE_CONTROL:
    // 臂关节 J1-J6
    for (int i = 1; i <= 6; i++)
        dummy.motorJ[i]->SetCurrentSetPoint(dummy.targetCurrents[i - 1]);
    // 地轨 J7
    dummy.motorJ[0]->SetCurrentSetPoint(dummy.targetRailCurrent);
    // ...
    break;

case DummyRobot::COMMAND_SERVO_J:
    dummy.MoveJoints(dummy.targetJoints);
    // 地轨单独下发（mm → step）
    float rail_steps = dummy.targetRailPos * DummyRobot::RAIL_STEPS_PER_MM;
    dummy.motorJ[0]->SetPositionDirect(rail_steps);
    // ...
    break;
```

### 4.7 电机固件 CAN 接收判断逻辑

**文件：** `dummy-motor-fw\Core\Src\can.c`

```cpp
uint8_t id  = (RxHeader.StdId >> 7);   // 高位 = 节点 ID
uint8_t cmd = (RxHeader.StdId & 0xFF);  // 低位 = 命令码

// ── 广播命令（cmd 属于 0x80-0x8F 区间）：所有节点无条件响应 ──
if (cmd >= 0x80 && cmd <= 0x8F) {
    OnCanCmd(cmd, RxData, RxHeader.DLC);  // 所有电机均执行
}
else if (id == boardConfig.canNodeId) {   // 否则仅本节点响应
    OnCanCmd(cmd, RxData, RxHeader.DLC);
}
```

### 4.8 电机固件 - 广播急停处理

**文件：** `dummy-motor-fw\UserApp\protocols\interface_can.cpp`

```cpp
case 0x89:  // 广播急停命令码
{
    extern Motor motor;
    motor.controller->requestMode = Motor::MODE_STOP;
    motor.controller->SetVelocitySetPoint(0);
    motor.controller->SetCurrentSetPoint(0);
    printf("[CAN BROADCAST] Emergency Stop Received!\r\n");
}
    break;
```

---

## 5. 35 / 42 电机固件切换

### 5.1 切换方法

在 `firmware\motor_fw_f103\UserApp\configurations.h` 中，通过宏定义切换：

```cpp
// 一套代码兼容 35/42 电机：编译 42 电机时取消注释下一行
//#define MOTOR_TYPE_42
#ifdef MOTOR_TYPE_42
    #define DEFAULT_DCE_KP    200      // 42 电机
    #define MAX_CURRENT_LIMIT (2*1000)  // 42 电机: 2000mA
#else
    #define DEFAULT_DCE_KP    195      // 35 电机
    #define MAX_CURRENT_LIMIT (int32_t)(1.2f*1000)  // 35 电机: 1200mA
#endif
```


| 电机类型      | 宏定义                        | 电流限制    | DCE Kp |
| --------- | -------------------------- | ------- | ------ |
| **42 电机** | 定义 `#define MOTOR_TYPE_42` | 2000 mA | 200    |
| **35 电机** | 注释掉（默认）                    | 1200 mA | 195    |


---

## 6. 使用方法

### 6.1 烧录固件

**所有 8 个电机固件都需要重新烧录**，因为：

- CAN 接收判断逻辑变更（`can.c`）
- 广播急停命令处理新增（`interface_can.cpp`）
- 默认参数改用宏定义（`main.cpp`）

烧录步骤：

1. 根据电机类型修改 `configurations.h` 中的 `#define MOTOR_TYPE_42`
2. 编译固件
3. 通过 SWD 接口烧录到对应电机板

### 6.2 地轨硬件配置

- **拨码开关**：设置为 `000`（ID = 0）
- **丝杆型号**：1605（转 1 圈 = 5mm）
- **减速比**：50:1
- **行程**：0 ~ 500mm
- **步进当量**：`1mm = 512000 步`

### 6.3 急停功能验证

烧录后测试急停功能：

1. 连接所有电机到 CAN 总线
2. 通过串口助手发送 `!STOP` 命令
3. 观察所有电机（无论拨码 ID 是多少）是否同时停止

---

## 7. CAN 通信协议

### 7.1 CAN ID 格式

```
StdId = (nodeID << 7) | cmdCode

- nodeID: 7 位（实际使用 0-7）
- cmdCode: 7 位（0x00-0x7F 为普通命令，0x80-0x8F 为广播命令）
```

### 7.2 命令码分类


| 命令码区间         | 类型       | 说明                           |
| ------------- | -------- | ---------------------------- |
| `0x01 - 0x07` | 普通命令     | 0x01 使能、0x05 位置控制等，发往特定节点    |
| `0x11 - 0x1B` | 配置命令     | 0x11 设置 ID、0x12 电流限制等，发往特定节点 |
| `0x21 - 0x25` | 查询命令     | 0x23 位置回传等，由电机发回主控           |
| `0x7D - 0x7F` | 系统命令     | 0x7F 重启，发往特定节点               |
| `**0x89**`    | **广播命令** | **急停，所有节点无条件响应**             |


### 7.3 广播急停帧格式

```
StdId: 0x89（cmdCode=0x89，nodeID 任意）
DLC:   8 字节
Data:  全 0
```

### 7.4 地轨控制命令

```
# 设置地轨位置（MoveJ 协议）
>j1,j2,j3,j4,j5,j6,j7(地轨mm),speed

# 单独设置地轨电流（力矩模式）
$c1,c2,c3,c4,c5,c6,c7(地轨)
```

---

## 8. 修改文件清单

### 8.1 主控固件

```
dummy-ref-core-fw/
├── Robot\instances\dummy_robot.h
│   ├── currentRailPos / targetRailPos
│   ├── targetRailCurrent
│   ├── RAIL_STEPS_PER_MM (constexpr float = 512000)
│   ├── MoveRail(float railPos_mm) 声明
│   ├── MoveJ / ServoJ → 7 参数（+j7_mm）
│   ├── SetJointCurrents → 7 参数（+c7）
│   └── COMMAND_TORQUE_CONTROL(=5) / COMMAND_SERVO_J(=6) 模式枚举
│
├── Robot\instances\dummy_robot.cpp
│   ├── motorJ[0] → 地轨电机 (ID=0, reduction=50, 0~500mm)
│   ├── MoveRail() → 下发地轨 (mm→step)
│   ├── MoveJ → 7参数，计算各轴速度（保证同时到达）
│   ├── ServoJ → 7参数，地轨+臂关节同步控制
│   ├── MoveJoints → 下发臂关节（°→step，由电机固件内部规划）
│   ├── UpdateJointAnglesCallback → 更新地轨位置和bits[0]状态标志
│   ├── SetJointCurrents → 7轴电流 + 地轨电流
│   ├── Homing() / Resting() → 7参数，地轨=0mm
│   └── ParseCommand → sscanf 7参数 ($/>/& 命令)
│
└── UserApp\main.cpp
    ├── 控制环(5kHz)：$ 力矩模式/ServoJ 模式支持 7 轴
    └── OLED 显示地轨位置 + 7轴状态
```

### 8.2 电机固件（需全部重新烧录）

```
dummy-motor-fw/
├── Core\Src\can.c
│   └── HAL_CAN_RxFifo0MsgPendingCallback()：广播判断改为 cmd>=0x80 && cmd<=0x8F
│
├── UserApp\protocols\interface_can.cpp
│   └── OnCanCmd()：新增 case 0x89 广播急停处理；修复 break; 缺少分号
│
└── UserApp\main.cpp
    └── 默认参数使用 MAX_CURRENT_LIMIT 和 DEFAULT_DCE_KP 宏
```

### 8.3 Python 串口助手

```
串口助手.py
├── MoveJ 控制面板：新增 J7 地轨滑块（0~500mm）
├── send_movej()：发送 7 轴 + 速度（>j1,...,j6,j7,speed）
├── 力矩控制：6→7 轴 ($c1,...,c6,c7)
├── ServoJ 测试：基准位姿 6→7 轴
└── 修复 ServoJ 测试命令多余 \n
```

---

## 附录：完整通信流程示例

### 急停流程

```
主控发送: StdId = 0x89, DLC = 8, Data = {0}
    │
    ▼
CAN 总线广播（所有电机均接收）
    │
    ├── 电机 ID=0（地轨）：cmd = 0x89 → >= 0x80 → 执行急停 ✓
    ├── 电机 ID=1（J1）  ：cmd = 0x89 → >= 0x80 → 执行急停 ✓
    ├── 电机 ID=2（J2）  ：cmd = 0x89 → >= 0x80 → 执行急停 ✓
    ├── ...
    └── 电机 ID=7（夹爪） ：cmd = 0x89 → >= 0x80 → 执行急停 ✓
```

### 地轨 MoveJ 控制流程

```
串口发送: >0,-75,180,0,0,0,0,80
    │
    ▼
ParseCommand: sscanf 解析 7 轴 + 速度
    │
    ▼
MoveJ(j1=0, j2=-75, j3=180, j4=0, j5=0, j6=0, j7=0mm)
    │
    ├── 限位检查（臂关节+地轨）
    ├── 计算各轴速度 dynamicJointSpeeds（保证同时到达）
    └── targetJoints / targetRailPos = 0mm
    │
    ▼
控制环(5kHz): MoveJoints(targetJoints) + MoveRail(targetRailPos)
    │
    ▼
CAN → 电机固件内部执行速度/加速度规划 → 各轴到位
```

### 地轨位置回传流程

```
电机 ID=0（地轨）发送: StdId = (0 << 7) | 0x23 = 0x23
    │
    ▼
主控接收: id = 0x23 >> 7 = 0, cmd = 0x23
    │
    ▼
can_protocol.cpp: dummy.motorJ[0]->UpdateAngleCallback() ✓
    │
    ▼
UpdateJointAnglesCallback: currentRailPos = motorJ[0]->angle ✓
```

