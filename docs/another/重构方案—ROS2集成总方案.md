# 总架构重构 — ROS2 集成总方案

> 文档目的：把"ROS2 MoveIt 集成 + 二进制 servoj"作为后续编码的事实基础。  
> 文档状态：议题 1~5 已全部确认（5.2 详细算法见 `7DOF冗余机械臂逆解方案.md`），议题 6~12 继续细化中。  
> 文档变更：2026-08-19 删除原"议题 2:地轨适配"章节，相关内容已收敛到 `7DOF冗余机械臂逆解方案.md` 的最终方案。  
>
> 配套文档：  
> - `7DOF冗余机械臂逆解方案.md`（地轨适配 + IK 评分函数，最终方案）  
> - `重构方案—参数重构需求.md` / `重构方案—参数重构经验总结.md`（已完成，加速度/电流重构）  
> - `ISSUES.md`（P0/P1 bug 修复）  
> - `TODO.md`（P2-P5 功能路线图）

---

## 1. 文档定位与边界

### 1.1 这是什么

本文件是 **ROS2 集成方案** —— 把"ROS2 MoveIt 集成 + 二进制 servoj 帧"合并为**一个**重构计划。

### 1.2 不是目标

- 不是"地轨适配"（那是 `7DOF冗余机械臂逆解方案.md` 的范围）
- 不是"加速度/电流重构"（那是 `重构方案—参数重构需求.md` 的范围）
- 不是"bug 修复"（那是 `ISSUES.md` 的范围）
- 不是"功能路线图"（那是 `TODO.md` 的范围）

### 1.3 决策清单（议题 1~5 已锁定）

| 议题 | 决策 | 状态 |
|---|---|---|
| **1.1** | 走 ROS2 MoveIt 路线 + 主控适配高频 servoj 模式 | ✅ 锁定 |
| **1.2** | 主仓库迁 Ubuntu 22.04，Windows 留兼容 | ✅ 锁定 |
| **1.3** | 新增二进制 servoj 帧，必须完整实现 + 文档 + 测试 | ✅ 锁定 |
| **2.3** | 地轨到位检测：**改用位置超时**（200ms 内位置不变化即认为到位），**不用堵转** | ✅ 锁定 |
| **4.1** | 路线：**ROS2 MoveIt + 主控内置 IK 双轨**（方案 Y） | ✅ 锁定 |
| **4.2** | 主控侧允许修改 ~150-200 行 | ✅ 锁定 |
| **4.3** | 二进制 servoj 必须做完整适配 | ✅ 锁定 |
| **5.1** | 6-DOF IK = 解析解（已读 `6dof_kinematic.cpp` 确认） | ✅ 锁定 |
| **5.3** | IK 失败：**返回错误码 + 电机不动** | ✅ 锁定 |
| **5.4** | MoveL **同步阻塞**（5ms 上限），超时当失败 | ✅ 锁定 |
| **5.2** | rail 适配策略 = 动态基圆 + 评分函数（详见 `7DOF冗余机械臂逆解方案.md`） | ✅ 锁定 |

### 1.4 还在讨论中的议题

| 议题 | 内容 | 状态 |
|---|---|---|
| **6** | MoveJ j7 vs $ c0 命名规范 | ⏳ 待讨论 |
| **7** | IsMoving 完整性 | ⏳ 待讨论 |
| **8** | ASCII 协议去重 | ⏳ 待讨论 |
| **9** | Fibre 协议取舍（45KB 栈） | ⏳ 待讨论 |
| **10** | 状态机 IDLE（P0-8） | ⏳ 待讨论 |
| **11** | ROS 端 URDF/SRDF/moveit_config | ⏳ 待讨论 |
| **12** | ROS 桥接节点（PC 端 Python） | ⏳ 待讨论 |

---

## 2. 议题 1：ROS2 MoveIt + 高频 servoj 集成

### 2.1 总体架构

```
┌─────────────────────────────────────────────────────────────────┐
│ Linux PC (Ubuntu 22.04)                                          │
│                                                                  │
│  ┌────────────────┐    ┌────────────────┐    ┌────────────────┐  │
│  │  ROS2 Humble   │    │  MoveIt 2      │    │  moveit_servo  │  │
│  │  + TF2         │◄──►│  (7-DOF IK)   │◄──►│  (50-500Hz)    │  │
│  │  + ros2_ctrl   │    │  KDL/IKFast    │    │  手柄/视觉接口 │  │
│  └────────┬───────┘    └────────┬───────┘    └────────┬───────┘  │
│           │ JointTrajectory     │ PlanningScene       │ Twist   │
│           ▼                     ▼                     ▼         │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  ros2_dummy_bridge (Python ~300 行)                       │   │
│  │  - 订阅 /joint_trajectory (JointState[7])                │   │
│  │  - 解析成二进制帧                                          │   │
│  │  - 写串口                                                  │   │
│  │  - 发布 /joint_states (从主控 #GETJPOS 100Hz 回读)        │   │
│  └────────────────────────┬─────────────────────────────────┘   │
└───────────────────────────┼──────────────────────────────────────┘
                            │ 串口 (USB or UART4) 115200~921600 bps
                            │ ASCII (debug) + 二进制 (servo)
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│ STM32F405 主控 (ref_core_f405)                                  │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  ASCII 协议 (legacy)  ←  调试 / 手动控制                  │   │
│  │  - `>j1..j6,j7,speed` MoveJ (含 rail)                    │   │
│  │  - `@x,y,z,a,b,c,speed` MoveL (NEW: 含 rail 适配)        │   │
│  │  - `!START/STOP/HOME/RESET/RAIL_L/RAIL_R/...`            │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  二进制协议 (NEW ~250 行)                                  │   │
│  │  - 0xB0: 二进制 MoveJ servo (j1~j6 角度 + rail mm + speed)│   │
│  │  - 0xB1: 二进制 MoveL (x,y,z,a,b,c + IK mode)            │   │
│  │  - 0xB2: 力矩 (7 维 mA)                                   │   │
│  │  - 0xB3: 心跳 / 查询                                     │   │
│  │  - 特性: 无 ack 流式模式                                  │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  �──────────────────────────────────────────────────────────┐   │
│  │  DummyRobot (主控核心)                                    │   │
│  │  - 7 轴控制: motorJ[0]=rail, [1~6]=joints                │   │
│  │  - 6-DOF IK (现有, 解析解)                               │   │
│  │  - RailAdapter (NEW ~200 行)  ← 详见 7DOF 方案文档       │   │
│  │    - 动态基圆 + 评分函数                                  │   │
│  │    - 决定 rail 移动量 + 检查关节舒适度                     │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  5kHz 实时环                                              │   │
│  │  - SEQ/INT/TRJ: 50Hz 下发                                │   │
│  │  - SERVO_J: 每 200μs 下发 (无 ack)                        │   │
│  └──────────────────────────────────────────────────────────┘   │
│                              CAN1 500kbps                       │
└──────────────────────────────┼──────────────────────────────────┘
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│ 8 个 STM32F103 电机节点                                         │
│ CAN ID 1-6 (关节) / 8 (夹爪) / 9 (地轨)                          │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 ROS2 MoveIt 集成的事实基础

#### 2.2.1 MoveIt 对 7-DOF 冗余机械臂的支持

**事实**：MoveIt 默认 IK 求解器是 **KDL（ChainIkSolverPos_LMA / NR_JL）**，数值阻尼最小二乘 / Newton-Raphson，**支持任意自由度**——包括 7-DOF 冗余。

**事实**：URDF 中 `prismatic joint` 是标准类型：
```xml
<joint name="rail" type="prismatic">
  <parent link="world"/>
  <child link="base_link"/>
  <axis xyz="0 1 0"/>  <!-- 沿 Y 方向 -->
  <limit lower="-0.25" upper="0.25" effort="..." velocity="..."/>
</joint>
```

**结论**：MoveIt 能直接处理 7-DOF（1 prismatic + 6 revolute），不需要自写 IK。

#### 2.2.2 Windows 用户的兼容

**问题**：MoveIt 在 Windows 上**不官方支持**（只有 WSL2/Docker）。

**方案**：**主控内置基础 7-DOF IK**，让 Windows 用户**完全不需要 ROS2** 就能用 MoveL + rail 适配。

#### 2.2.3 7-DOF IK 在主控的可行性（议题 5.1 确认）

**事实**（已读 `6dof_kinematic.cpp`）：
- 6-DOF IK 是**解析解**，8 组解
- `SolveIK` 是**纯函数**（输入姿态 + 上次关节，输出 8 组解）
- 输入姿态单位：**mm**（X/Y/Z 除以 1000）+ 度（A/B/C）或 R 矩阵
- 输出关节单位：**度**
- `SolveIK` **总是 return true**，需要自己检查关节限位
- 函数开销：~30 次三角函数 + ~5 次 3×3 矩阵乘，**F405 上 < 100μs**

**关键洞察**：因为地轨是 **prismatic joint（纯直线，无旋转）**，冗余自由度的几何意义清晰：

```
世界目标 P_world
  ↓ 减去 base 平移 (current_rail, 0, 0) → P_base
  ↓ 调现有 6-DOF IK
6 轴关节角
  ↓ 加上 rail 决策
7 个关节输出
```

**核心计算量 = 已有 6-DOF IK + 一个简单的 rail 决策逻辑**——**不是从零写 7-DOF 数值 IK**。

**预计主控新增代码量**：

| 文件 | 改动 | 行数 |
|---|---|---|
| `RailAdapter.h` | 新建 | ~30 |
| `RailAdapter.cpp` | 新建 | ~120 |
| `dummy_robot.cpp` | 改 MoveL 调 RailAdapter | ~20 |
| `dummy_robot.h` | 加 RailAdapter 成员 | ~5 |
| `ascii_protocol.cpp` | 加 `@!` 命令（主控内置 IK 模式） | ~10 |
| `ascii_protocol.cpp` | 二进制协议 0xB0/0xB1 | ~250 |
| `communication.cpp` | 二进制协议栈 | ~50 |
| **总计** | — | **~485 行** |

### 2.3 ROS2 桥接节点（PC 端 Python）

**文件**：`ros2_dummy_bridge/dummy_bridge_node.py`（新建）

**职责**：
- 订阅 `/joint_trajectory`（MoveIt 输出）
- 订阅 `/servo_joint_trajectory`（moveit_servo 输出）
- 解析成二进制帧，写串口
- 定时（100Hz）发 `#GETJPOS`，解析后发布 `/joint_states`

**代码量**：~300 行 Python

**依赖**：`rclpy`、`pyserial`（或 `python-can` 走 USB-CAN）

**线程模型**：
- ROS2 spinning 线程（独立）
- 串口读写线程（独立）
- Tkinter mainloop（只在 Windows GUI 版本需要）

### 2.4 MoveIt 配置（PC 端）

**新建文件**：
```
moveit_config/
├── config/
│   ├── dummy.urdf.xacro           # 7-DOF URDF
│   ├── dummy.srdf                  # 自碰撞矩阵 / 虚拟关节
│   ├── kinematics.yaml             # KDL 配置
│   ├── joint_limits.yaml           # 关节限位
│   ├── controllers.yaml            # ros2_control 配置
│   └── ompl_planning.yaml          # OMPL 配置
├── launch/
│   ├── move_group.launch.py
│   ├── dummy_bridge.launch.py
│   └── servo.launch.py
└── package.xml
```

**预计代码量**：URDF ~80 行 + SRDF ~40 行 + 配置 ~150 行 = **~270 行**。

---

## 3. 议题 2：MoveIt 适配 + MoveL 集成

> **注**：原"议题 2:地轨适配"已删除。详细算法（动态基圆 + 评分函数、位置超时到位检测等）见 `7DOF冗余机械臂逆解方案.md`。

### 3.1 主控侧 0 改方案（方案 A）

**架构**：ASCII 协议 + 7 个 joint 参数（含 rail）不变，ROS 端翻译规则。

| ROS JointTrajectory | ASCII 命令 |
|---|---|
| JointState[7] = [rail_mm, j1_deg, ..., j6_deg] | `>j1,j2,...,j6,rail,speed` |

**优点**：主控代码**完全不动**。

**缺点**：
- ASCII 速率限制：~380Hz（115200 bps ÷ ~30 字节/帧）
- ROS JointTrajectory 是 7 维 `[rail, j1..j6]`，ASCII 是 `[j1..j6, rail]`——**顺序反的**
- 每次翻译都增加延迟

### 3.2 主控内置 IK 方案（方案 Y / 推荐）

**架构**：
```
MoveL_world(x,y,z,a,b,c,speed)
  ↓ ASCII: `@x,y,z,a,b,c,speed` (Windows 串口助手直接可用)
主控
  ├── 调 RailAdapter.adapt_rail(target_world, current_rail)
  ├── 计算 rail_target
  ├── base 平移到 rail_target（实际不需真动，只 IK 坐标变换）
  ├── 调 6-DOF IK(target_in_base)
  ├── 检查关节限位
  └── 返回 6 轴 + rail → 下发

MoveL_ik(x,y,z,a,b,c,speed, ik_mode="joint")
  ↓ ASCII: `@!x,y,z,a,b,c,speed` (Linux ROS 端用，主控透明)
主控
  └── 透传，调用已有 6-DOF IK（ROS 已算好）
```

**Windows 用户**：
- 走 `@x,y,z,a,b,c,speed` → 主控算 IK + rail 适配
- 不需要 ROS2，直接用现有串口助手

**Linux / ROS 用户**：
- 走 `@!x,y,z,a,b,c,speed` → 主控透明执行（ROS 已算好）
- 享受 MoveIt 的完整路径规划 / 碰撞检测 / 冗余优化

### 3.3 MoveL 失败的错误码（议题 5.3 已锁定）

**协议**：ASCII 错误码字符串
- `@x,y,z,a,b,c,speed` 成功 → `ok`
- IK 无解 → `err: IK no solution, target unreachable`
- 关节超限位 → `err: joint N out of limit (current=X, max=Y)`
- rail 适配失败 → `err: rail adapt failed (target=X, current=Y, limit=±250)`
- 计算超时 → `err: IK timeout (>5ms)`

**主控行为**：**返回错误码，电机不动**。

### 3.4 MoveL 同步语义（议题 5.4 已锁定）

**契约**：
- MoveL 命令**阻塞等结果**，最长 5ms
- 5ms 内必须给"成功 / 失败"二选一
- 超时（计算量爆炸）→ 当作失败处理，返回 `err: IK timeout`

**实现**：
- MoveL 在 `ThreadControlLoopUpdate` 任务里执行
- 该任务优先级 `osPriorityNormal`，不被 5kHz 实时环阻塞
- 但也不能无限等，加 5ms 看门狗

---

## 4. 议题 3：二进制 servoj 帧

### 4.1 为什么必须做（议题 4.3 锁定）

**ASCII servoj 瓶颈**：
- ASCII 帧 `&j1,j2,j3,j4,j5,j6,j7,speed` ≈ 30 字节
- 115200 bps / 10 bit/byte ≈ 11520 字节/秒
- 最大 11520 ÷ 30 ≈ **380 帧/秒**

**moveit_servo 需求**：
- 50~500 Hz（取决于任务）
- 高频 servo 需要 500Hz

**结论**：ASCII 不够快，必须新增二进制 servoj 帧。

### 4.2 二进制协议设计

#### 帧格式

```
┌──────┬──────┬─────────────────┬─────┐
│ 0xAA │ CMD  │     PAYLOAD     │ CRC │
│ 1B   │ 1B   │     NB          │ 1B  │
└──────┴──────┴─────────────────┴─────�
```

- **起始字节**：`0xAA`（避免和 ASCII 数据冲突）
- **CMD**：1 字节命令码
- **PAYLOAD**：变长
- **CRC**：1 字节 XOR 校验（和 CRC8 类似，简单实现）

#### 命令码清单

| CMD | 名称 | Payload | 说明 |
|---|---|---|---|
| `0xB0` | SERVO_MOVEJ | 6×int16 (j1~j6) + 1×int32 (rail×100) + 1×uint8 (speed) | 13 字节，无 ack |
| `0xB1` | SERVO_MOVEL | 6×float (x,y,z,a,b,c) + 1×uint8 (ik_mode) | 25 字节，无 ack |
| `0xB2` | SERVO_TORQUE | 7×int16 (c0~c6, mA) | 14 字节，无 ack |
| `0xB3` | HEARTBEAT | 1×uint32 (timestamp_ms) | 4 字节，无 ack |
| `0xB4` | QUERY_JOINTS | (无 payload) | 0 字节，触发回包 |
| `0xB5` | RESPONSE_JOINTS | 7×float (j1~j6 + rail) | 28 字节 |

#### 关键设计：单位

| 量 | 单位 | 编码 | 范围 | 精度 |
|---|---|---|---|---|
| j1~j6 角度 | 0.01° | int16 | ±327.67° | 0.01° |
| rail | mm×100 | int32 | ±214748.36mm | 0.01mm |
| speed | % | uint8 | 0~100% | 1% |
| x/y/z | mm | float | — | 满精度 |
| a/b/c | 度 | float | — | 满精度 |
| 电流 | mA | int16 | ±32.767A | 1mA |

#### 流式模式（不返回 ack）

**行为**：
- 主控收到 0xB0 帧 → 直接调 `ServoJ(j1..j6, rail)` → 完成
- **不返回任何响应**
- 100Hz 查询走 0xB4/0xB5 单独通道
- 心跳 0xB3 用于链路检测

**风险与缓解**：
- 串口丢帧无法检测 → ROS 端用发送计数器，丢帧超阈值报警
- 串口拥塞 → ROS 端用 500Hz 上限，避免过载

### 4.3 完整适配（议题 4.3 要求"做好适配不忘记用"）

**主控侧**：
- 协议解析 → ~150 行
- 流式接收缓冲（ring buffer）→ ~50 行
- 5kHz 环集成 → ~30 行
- 错误处理（CRC 错 / 帧不完整）→ ~30 行

**ROS 端**：
- `ros2_dummy_bridge` 解析 JointTrajectory → 二进制帧 → ~80 行
- 心跳 / 超时检测 → ~30 行
- 测试用例 → ~50 行

**测试用例**：
- 单元测试：每条命令的解析正确性
- 集成测试：高频 servo 500Hz × 60 秒，统计丢帧率
- 端到端：ROS2 MoveIt → 主控 → 电机 → 编码器回读 → ROS JointStates

**文档**：
- `PROTOCOL_BINARY.md`（新建）— 完整二进制协议说明
- `ros2_dummy_bridge/README.md` — 桥接节点使用说明

---

## 5. 主控代码改动汇总

### 5.1 文件清单

| 文件 | 改动类型 | 行数 | 说明 |
|---|---|---|---|
| `Robot/algorithms/kinematic/RailAdapter.h` | 新建 | ~30 | rail 适配器头文件（动态基圆 + 评分函数） |
| `Robot/algorithms/kinematic/RailAdapter.cpp` | 新建 | ~120 | rail 适配器实现（详见 `7DOF冗余机械臂逆解方案.md`） |
| `Robot/algorithms/rail_checker/RailPositionChecker.h` | 新建 | ~20 | 位置超时到位检测头 |
| `Robot/algorithms/rail_checker/RailPositionChecker.cpp` | 新建 | ~50 | 位置超时到位检测实现 |
| `Robot/instances/dummy_robot.h` | 改 | ~10 | 加 RailAdapter/RailPositionChecker 成员 |
| `Robot/instances/dummy_robot.cpp` | 改 | ~40 | MoveL 调 RailAdapter，MoveRail 加超时检测，IsMoving 扩展 |
| `UserApp/protocols/ascii_protocol.cpp` | 改 | ~30 | 加 `@!` 命令，去 USB/UART4 重复（议题 8） |
| `UserApp/protocols/binary_protocol.cpp` | 新建 | ~250 | 二进制协议 |
| `UserApp/protocols/binary_protocol.h` | 新建 | ~50 | 二进制协议头 |
| `Bsp/communication/communication.cpp` | 改 | ~30 | 二进制协议栈注册 |
| `Bsp/communication/serial_rx_buffer.cpp` | 新建 | ~80 | 流式 ring buffer |
| **主控总改动** | — | **~710 行** | — |

### 5.2 ASCII 协议扩展（议题 6 待讨论）

**新增命令**：
- `@!x,y,z,a,b,c,speed` — MoveL_ik 模式（ik_mode="joint"，主控透明执行）
- `S` — 进入二进制 servoj 流式模式（连续接收 0xB0 帧）
- `s` — 退出二进制 servoj 流式模式
- `#GETSTATE` — 查询综合状态（含 rail/hand/6 轴）

**去重**（议题 8）：
- 把 `OnUsbAsciiCmd` 和 `OnUart4AsciiCmd` 抽取到 `ProcessAsciiCmd(_cmd, _channel)`
- 两边只做 channel 区分

### 5.3 状态机扩展（议题 10 关联 P0-8）

- `SetEnable` 应设 `IDLE` 而不是 `FINISH`
- `CommandMode` 新增 `BINARY_SERVO` 模式
- 流式模式下 5kHz 环每帧解一帧二进制

---

## 6. ROS 端架构

### 6.1 工作空间结构

```
dummy_ws/
├── src/
│   ├── dummy_description/           # URDF / meshes
│   ├── dummy_moveit_config/         # moveit_config
│   ├── dummy_bridge/                # ros2_dummy_bridge
│   ├── dummy_servo/                 # moveit_servo 配置
│   └── dummy_joystick/              # 手柄控制节点
├── build/
├── install/
└── log/
```

### 6.2 URDF 设计

```xml
<?xml version="1.0"?>
<robot name="dummy">
  <!-- World link -->
  <link name="world"/>

  <!-- Rail (prismatic, 沿 Y 方向) -->
  <joint name="rail" type="prismatic">
    <parent link="world"/>
    <child link="rail_base"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.25" upper="0.25" effort="100" velocity="0.5"/>
  </joint>
  <link name="rail_base"/>

  <!-- 机械臂 base 固定在 rail_base 上 -->
  <joint name="arm_to_rail" type="fixed">
    <parent link="rail_base"/>
    <child link="arm_base"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>  <!-- 机械臂 base 在 rail 上的位置 -->
  </joint>
  <link name="arm_base"/>

  <!-- 6-DOF 机械臂 -->
  <joint name="joint1" type="revolute">
    <parent link="arm_base"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-180" upper="180" effort="..." velocity="..."/>
  </joint>
  ...
  <!-- joint2 ~ joint6 类似 -->
  ...
  <link name="link6"/>

  <!-- 工具 (tool0) -->
  <joint name="tool0_joint" type="fixed">
    <parent link="link6"/>
    <child link="tool0"/>
  </joint>
  <link name="tool0"/>
</robot>
```

### 6.3 moveit_servo 配置

```yaml
# servo_server.yaml
ros2_control:
  loop_hz: 500  # 500Hz servo 频率
  planning_group_name: "dummy_arm"
  moveit_servo:
    publish_joint_position_updates_topic: "/servo_joint_states"
    publish_joint_velocities_topic: "/servo_joint_velocities"
    publish_joint_accelerations_topic: "/servo_joint_accelerations"
```

---

## 7. 实施路径（路线图）

### 7.1 Phase A：基础（主控内置 IK + rail 适配）

**目标**：让 Windows 用户的串口助手**直接能用** `MoveL(x,y,z,a,b,c,speed)` 命令，**自动**调用主控 IK + rail 适配。

**任务清单**：
- A1. `RailPositionChecker` 实现 + 单元测试
- A2. `RailAdapter` 实现 + 单元测试（议题 5.2 详见 `7DOF冗余机械臂逆解方案.md`）
- A3. `MoveL` 改造，调 RailAdapter
- A4. ASCII 协议扩展 + 错误码
- A5. 串口助手 MoveL 标签页加 rail 状态显示

**预计工作量**：2~3 周

### 7.2 Phase B：二进制协议

**目标**：实现 0xB0~0xB5 二进制帧，让高频 servo 可用。

**任务清单**：
- B1. 二进制协议解析器
- B2. 流式 ring buffer
- B3. `ServoJ` 模式扩展（接二进制帧）
- B4. CRC 校验 / 错误处理
- B5. 协议文档 `PROTOCOL_BINARY.md`

**预计工作量**：2 周

### 7.3 Phase C：ROS2 桥接

**目标**：实现 `ros2_dummy_bridge`，让 ROS2 MoveIt 能驱动主控。

**任务清单**：
- C1. `dummy_bridge_node.py` 串口读写 + ROS 订阅
- C2. JointTrajectory → 二进制帧转换
- C3. JointStates 发布（从主控 `#GETJPOS`）
- C4. 心跳 / 超时检测
- C5. 测试用例

**预计工作量**：2~3 周

### 7.4 Phase D：MoveIt 配置

**目标**：URDF + moveit_config + moveit_servo 完整配置。

**任务清单**：
- D1. URDF / SRDF
- D2. KDL kinematics 配置
- D3. joint_limits.yaml
- D4. ompl_planning.yaml
- D5. moveit_servo 配置
- D6. launch files

**预计工作量**：1~2 周

### 7.5 Phase E：高级功能

**目标**：重力补偿 + 阻抗控制 + 视觉抓取。

**任务清单**：
- E1. ROS2 cartesian_controller 配置
- E2. 阻抗控制器
- E3. 视觉抓取集成（moveit_grasps / perception_pipeline）
- E4. 手柄控制节点

**预计工作量**：4~6 周

### 7.6 总工作量估计

**Phase A~D**：7~10 周（主控 + ROS 桥接 + MoveIt 配置）

**Phase E**：4~6 周（高级功能）

**总计**：3~4 个月（含调试、测试）

---

## 8. 风险与缓解

| 风险 | 概率 | 影响 | 缓解 |
|---|---|---|---|
| 主控 IK 计算超时（>5ms） | 低 | MoveL 失败 | 算法简单（复用 6-DOF + RailAdapter），实测验证 |
| 二进制协议串口丢帧 | 中 | 高频 servo 抖动 | CRC 校验 + 心跳 + 重传 |
| ROS 端机械臂描述与实际不符 | 中 | MoveIt 规划失败 | URDF 必须基于实测 DH 参数 |
| Windows 用户无法用 ROS2 | 低 | 开源社区萎缩 | 主控内置 IK，Windows 不依赖 ROS |
| 7-DOF IK 主控实现与 ROS 不一致 | 低 | 两套结果不同 | ROS 端走"透传模式"绕过 |
| 串口速率不够 | 低 | servo 频率低 | 升级到 921600 bps 或 1Mbps |
| 串口助手重构引入 bug | 中 | 现有用户回归 | 单元测试 + 现有功能逐项验证 |
| RailAdapter 评分函数误判 | 中 | rail 不必要移动 | 双目标 EWMA 预测 + 实测调参 |

---

## 9. 后续议题清单（待讨论）

| # | 议题 | 关键决策点 |
|---|---|---|
| **6** | MoveJ j7 vs $ c0 命名规范 | j7 位置统一 / c0 位置统一 |
| **7** | IsMoving 完整性 | rail + hand 状态 |
| **8** | ASCII 协议去重 | OnUsb/OnUart4 合并 |
| **9** | Fibre 协议取舍 | 保留 / 删除 / 简化为 10KB |
| **10** | 状态机 IDLE (P0-8) | SetEnable 改为 IDLE |
| **11** | ROS 端 URDF/SRDF | prismatic 命名 + rail 方向 |
| **12** | ROS 桥接节点设计 | ros2_dummy_bridge 详细架构 |

每个议题都会基于本方案的"已锁定"部分，逐步细化。

---

## 10. 总结

### 10.1 这是什么

本文件是**议题 1~5 的 ROS2 集成总方案**——明确架构、决策、工作量、风险。

### 10.2 不是目标

- 不是地轨适配细节（详见 `7DOF冗余机械臂逆解方案.md`）
- 不是具体代码（代码在每个 Phase 的任务清单里）
- 不是 ROS 端完整配置（那是 Phase D 的工作）
- 不是测试用例（每个 Phase 自己写）

### 10.3 下一步

- 议题 6~12：逐个细化
- 最后生成总编码任务清单（类似 `重构方案—参数重构经验总结.md`）

---

## 附录 A：相关文档

| 文档 | 路径 | 说明 |
|---|---|---|
| `7DOF冗余机械臂逆解方案.md` | `e:\Dummy-code\7DOF冗余机械臂逆解方案.md` | 地轨适配 + IK 评分函数最终方案 |
| `重构方案—参数重构需求.md` | `e:\Dummy-code\重构方案—参数重构需求.md` | 加速度/电流参数重构需求（已完成） |
| `重构方案—参数重构经验总结.md` | `e:\Dummy-code\重构方案—参数重构经验总结.md` | 参数重构任务经验回顾 |
| `PROJECT_CONTEXT.md` | `e:\Dummy-code\PROJECT_CONTEXT.md` | 项目架构、CAN 协议、命令格式 |
| `README.md` | `e:\Dummy-code\README.md` | 项目主文档、用户指南 |
| `ISSUES.md` | `e:\Dummy-code\ISSUES.md` | P0/P1/P2/P3 问题追踪 |
| `TODO.md` | `e:\Dummy-code\TODO.md` | 功能路线图 |

---

## 附录 B：术语表

| 术语 | 含义 |
|---|---|
| **主控** | STM32F405RG（ref_core_f405）—— 中央控制器 |
| **电机固件** | STM32F103CBT6（motor_fw_f103_35/42/57/gripper）—— 电机驱动板 |
| **5kHz 环** | 电机固件的位置控制循环（每 200μs 跑一次） |
| **CAN StdId** | 标准帧 ID 格式：`StdId = (nodeID << 7) \| cmdCode` |
| **nodeID** | 电机节点 ID（地轨=9、关节=1~6、夹爪=8） |
| **cmdCode** | CAN 命令码（0x00~0x7F 普通，0x80~0xBF 广播） |
| **RailAdapter** | 主控内 rail 适配器（动态基圆 + 评分函数） |
| **基圆** | 以 base 原点为圆心的虚拟工作空间半径 |
| **评分函数** | 从 8 组 6-DOF IK 候选解中挑最佳的加权函数 |
| **rail** | 地轨（prismatic joint，沿 Y 轴） |
| **MoveL** | 笛卡尔直线运动（输入 X/Y/Z/A/B/C，输出 7 轴关节角） |
| **MoveJ** | 关节空间运动（输入 7 个关节角） |
| **ServoJ** | 关节空间伺服（高频位置控制，500Hz） |
| **servo** | ROS2 高频伺服模式（50~500Hz） |
| **MoveIt** | ROS2 运动规划框架 |
| **moveit_servo** | MoveIt 的实时伺服模块 |
| **KDL** | Kinematics and Dynamics Library（MoveIt 默认 IK 求解器） |
| **URDF** | Unified Robot Description Format |
| **SRDF** | Semantic Robot Description Format |
| **MoveGroup** | MoveIt 的运动规划节点 |

---

**文档结束**

> AI 助手：根据用户 2026-08-13~19 对话制定。  
> 下一步：议题 6~12 逐个细化 → 生成总编码任务清单。
