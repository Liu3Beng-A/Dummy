# 模块 03 — 关节角度同步（`UpdateAngle()` 0x23 轮询策略 + `UpdateJointAnglesCallback`）

> **范围**：主控轮询电机位置（CAN 0x23），更新 `jointsStateFlag` + `currentJoints`
> **关联文件**：
> - `Robot/instances/dummy_robot.cpp` (`UpdateJointAngles()` + `UpdateJointAnglesCallback()`)
> - `UserApp/protocols/can_protocol.cpp` (`case 0x23` 回包处理)
> - `Robot/actuators/ctrl_step/ctrl_step.cpp` (`UpdateAngle()` 发送)

---

## 模块概述

主控需要实时知道每个电机的当前位置，用于：
1. 更新 `jointsStateFlag`（让 IsMoving() 能判定到位）
2. 更新 `currentJoints.a[]`（给 OLED 显示、IK 解算用）
3. 计算 `currentPose6D`（FK 缓存，给 #GETLPOS 用）

**两种轮询策略**：
- **广播触发**（OLD）：1 帧 `0x23` 触发所有电机同时回包，回包可能撞 mailbox（CAN 总线冲突）
- **单播轮询**（firmware E2）：100Hz 主控循环，每帧只发 1 个电机（currentIndex 1~7 循环），7 节点各分到 ~14Hz

**回包后处理**：
- `UpdateAngleCallback()`：主控侧更新单个电机 `angle` 字段
- `UpdateJointAnglesCallback()`：把 J1~J6 的 `currentJoints` + `jointsStateFlag` 整合

---

## 三状态对比

| 维度 | OLD | firmware | 当前 firmware_new |
|---|---|---|---|
| UpdateAngle 触发频率 | 每帧（5kHz）触发 | 100Hz 主循环触发 + 7 节点循环（14Hz/电机） | **14Hz/电机**（E2 已生效） |
| 轮询方式 | 广播（1 帧 ALL） | 单播（currentIndex 1~7 循环） | 单播 |
| `case 0x23` 处理 | 单一函数：UpdateAngleCallback + UpdateJointAnglesCallback | 按 id 分三段（id=7 同步 rail；id=1~6 关节；id=8 夹爪） | 单一函数（与 OLD 类似，回包统一处理） |
| jointsStateFlag 位分配 | 仅 bit1~6 | bit0/7 槽（地轨恒 0；bit7 用不上） | 同 OLD（仅 bit1~6） |
| Enable 后预发 | 不预发（依赖每帧广播） | 预发 1 帧广播（让电机尽快回包建立初始状态） | **预发 1 帧广播**（已生效） |
| 7 轴处理 | 无 | J7/地轨单独回包 + 同步 currentRailPos | **存在但仍按 OLD 模式处理**（UpdateJointAnglesCallback 不处理 rail） |

---

## 新旧差异详解

> 本节只罗列事实差异（代码位置 + 行为对比），不做决策评判；具体决策见下节。

### 1. `UpdateJointAngles()` 轮询策略

| 维度 | OLD `firmware_old/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:160-163` | NEW `firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:436-444` |
|---|---|---|
| 实现 | `motorJ[ALL]->UpdateAngle();` 1 帧广播（nodeID=0 → StdId=`0<<7\|0x23=0x23`） | `motorJ[currentIndex]->UpdateAngle();` 单播，`currentIndex` 是 `static` 变量 1→7 循环 |
| 调用频率 | `firmware_old/.../UserApp/main.cpp:46` 每帧无条件执行（事件驱动 `ControlLoopUpdateTask`） | `firmware_new/.../UserApp/main.cpp:72/86/97/112/122` 节流到 `updateCounter % 50 == 25`（每个分支都先判一次） |
| 总线行为 | 1 帧 → 7 节点同时回 0x23 → CAN mailbox 仲裁失败风险 | 每帧只 1 个节点回包，无冲突 |
| 单轴采样率 | 旧广播未丢包时 ≤5ms / 不可靠 | ~14Hz（7 轴均分 100Hz 触发节流） |

### 2. `UpdateJointAnglesCallback()` 内容

| 维度 | OLD `firmware_old/.../dummy_robot.cpp:166-177` | NEW `firmware_new/.../dummy_robot.cpp:449-468` |
|---|---|---|
| 6 关节处理 | `for i=1..6 { currentJoints.a[i-1] = motorJ[i]->angle + initPose.a[i-1]; jointsStateFlag 位运算 }` | 一致 |
| jointsStateFlag 位运算 | `FINISH → \|=(1<<i); else &=~(1<<i);` | 一致 |
| 地轨 (`motorJ[7]`) 处理 | **不处理**（无 `currentRailPos` 字段） | **新增**：`currentRailPos = motorJ[7]->angle / 360.0f * 5.0f;` |
| `currentJoints.a[]` 写入 | 有 | 有（语义不变） |

### 3. CAN `0x23` 回包路由（`can_protocol.cpp`）

| 维度 | OLD `firmware_old/.../can_protocol.cpp:26-38` | NEW `firmware_new/.../can_protocol.cpp:105-293` |
|---|---|---|
| 分支结构 | 单一 `switch (cmd)`，末尾统一 `dummy.UpdateJointAnglesCallback()` | **3 段** `if/else if/else if` 按 id 分流：id=7 地轨、id=1~6 臂、id=8 夹爪；每段 switch 末尾**单独**调用 `dummy.UpdateJointAnglesCallback()` |
| 地轨额外处理 | 无 | id=7 时除了 `UpdateAngleCallback`，**还单独写一次** `currentRailPos = dummy.motorJ[7]->angle / 360.0f * 5.0f;`（与 `UpdateJointAnglesCallback` 末尾那一次功能重复） |
| 夹爪处理 | `dummy.motorJ[id]->UpdateAngleCallback(...)`（id=8 时越界访问 `motorJ[8]`） | `dummy.hand->UpdateAngleCallback(...)`（`motorJ[8]` 已被改造为 `hand` 指针，类型为 `DummyHand*`） |
| 每次回包触发 `UpdateJointAnglesCallback()` 次数 | 1 次 | 1 次（三段互斥，只走一段） |

### 4. `IsMoving()` 判定公式

```160:163:firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp
bool DummyRobot::IsMoving()
{
    if (!isEnabled) return false;              // [v3.2] 失能短路（NEW 独有）
    return jointsStateFlag != ALL_FINISH_FLAG; // 0x7E = bit1~6 全置位
}
```

vs OLD `firmware_old/.../dummy_robot.cpp:296-299`：

```cpp
bool DummyRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110;       // 字面量 0x7E
}
```

差异项：

| 维度 | OLD | NEW |
|---|---|---|
| 字面量 vs 常量 | `0b1111110`（行内字面量） | `ALL_FINISH_FLAG = 0x7E`（`dummy_robot.h:304` `static constexpr`） |
| 失能短路 | 无 | `if (!isEnabled) return false;`（`dummy_robot.cpp:659`） |
| `volatile uint8_t jointsStateFlag` | `dummy_robot.h:119` | `dummy_robot.h:297`（声明、初始值、类型一致） |

### 5. `SetEnable()` 后预发广播（NEW 独有）

```608:617:firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp
// [v3.2] ALL 代理 1 帧 + 夹爪 1 帧 = 2 帧
motorJ[ALL]->SetEnable(_enable);  // StdId=(0<<7)|0x01，所有 J1~J7/地轨电机接收
hand->SetEnable(_enable);         // StdId=(8<<7)|0x01
isEnabled = _enable;

// [F1 配套] enable 后预发 UpdateAngle，避免 jointsStateFlag 在电机回包前错填
if (_enable) {
    motorJ[ALL]->UpdateAngle();  // 1 帧触发 7 电机回包，等首次 0x23 自动纠正 state
}
```

OLD `firmware_old/.../dummy_robot.cpp:262-265` 仅：

```cpp
motorJ[ALL]->SetEnable(_enable);
isEnabled = _enable;
```

无预发广播。

### 6. 主循环轮询节流（NEW 独有）

NEW 在每个 `case COMMAND_TARGET_POINT_SEQUENTIAL / _INTERRUPTABLE / _CONTINUES_TRAJECTORY / _MOTOR_TUNING / _TORQUE_CONTROL` 分支里都加了：

```cpp
if (updateCounter % 50 == 25) {
    dummy.UpdateJointAngles();
}
```

`updateCounter` 每帧自增（`main.cpp:126`）。`% 50 == 25` 表示每 50 帧触发 1 次，且触发相位偏离 `== 0`（错开节拍）。失能分支 `main.cpp:121-122` 用 `if (updateCounter % 50 == 0)`。

OLD `main.cpp:46` 只有一个无条件调用，无节流、无相位错开。

### 7. 头文件新增成员（NEW）

`firmware_new/.../dummy_robot.h:155` 新增：

```cpp
float currentRailPos = 0.0f;   // 地轨当前位置 (mm)
float targetRailPos = 0.0f;    // 地轨目标位置 (mm)
```

`firmware_new/.../dummy_robot.h:304` 新增：

```cpp
static constexpr uint8_t ALL_FINISH_FLAG = 0x7E;  // 0b01111110
```

OLD `dummy_robot.h` 既无 `currentRailPos` 也无 `ALL_FINISH_FLAG`。

---

## 决策

### 2026-09-25 决策更新（基于实测重新评估）

| # | 决策 | 理由 | 改动点 |
|---|---|---|---|
| **D-1** | **退回 OLD 广播策略**（`motorJ[ALL]->UpdateAngle()` 1 帧广播） | 1. 实测 OLD 广播无丢包、刷新很快；2. NEW 单播 14Hz/电机不足以支撑重力补偿等高频算法（≥1kHz/电机才够用） | `firmware_new/.../dummy_robot.cpp:436-444` 改回 OLD 写法（删除 `currentIndex` 静态变量）；`firmware_new/.../UserApp/main.cpp` 删除 5 处 `if (updateCounter % 50 == 25) { dummy.UpdateJointAngles(); }` 节流判断；`firmware_new/.../UserApp/main.cpp` `updateCounter` 自增逻辑（`:126`）如不再被其他模块使用，可一并删除 |
| **D-2** | **不采取 Enable 后预发广播** | 1. OLD 没有这个机制且实际运行无功能缺失；2. 预发 1 帧广播只在 enable 边缘场景下加速首次回包，价值与成本不匹配 | 删除 `firmware_new/.../dummy_robot.cpp:626` 附近的 `if (_enable) { motorJ[ALL]->UpdateAngle(); }` 整段 |

### 历史决策（保留）

| # | 原决策 | 状态 | 备注 |
|---|---|---|---|
| 3 | `UpdateJointAnglesCallback` 保持 OLD 逻辑（`for i=1..6`） | **保留** | 与 OLD `firmware_old/.../dummy_robot.cpp:166-177` 一致 |
| 4 | `case 0x23` 不做 id 分流 | **保留** | 与 OLD `firmware_old/.../can_protocol.cpp:26-38` 一致 |
| 5 | 地轨 `currentRailPos` 暂不维护 | **保留** | OLD 既无 `currentRailPos` 字段也未维护 |

### 待决策项（用户 2026-09-25 决定暂时搁置）

| # | 议题 | 状态 |
|---|---|---|
| **D-3** | 三段 CAN 分流是否回退为单段（与决策 4 矛盾——NEW 已分三段） | 待 D-1/D-2 落地后观察是否还需要回退 |
| **D-4** | `currentRailPos` 是否回退到不维护（与决策 5 矛盾——NEW 已维护） | 待 D-1/D-2 落地后观察是否需要回退 |
| **D-5** | 在 OLD 广播基础上解决 j1-j6 刷新不均 + 提升到 ≥1kHz/电机 | **已搁置**（暂时不优化，j1-j6 ~500us 不均作为已知限制接受） |

---

### D-1 改动项（退回 OLD 广播）

| 文件 | 当前 NEW 代码 | 改为 OLD 代码 |
|---|---|---|
| `firmware_new/.../dummy_robot.cpp:436-444` | `for (i=1; i<=7) { motorJ[currentIndex]->UpdateAngle(); currentIndex = ...; }` | `motorJ[ALL]->UpdateAngle();` |
| `firmware_new/.../UserApp/main.cpp` | `if (updateCounter % 50 == 25) { dummy.UpdateJointAngles(); }` 等 5 处节流 | 全部删除 |
| `firmware_new/.../UserApp/main.cpp` 失能分支 | `if (updateCounter % 50 == 0) { dummy.UpdateJointAngles(); }` | 改为 `dummy.UpdateJointAngles();` 无条件调（OLD 行为） |
| `firmware_new/.../UserApp/main.cpp:126` | `updateCounter++` 自增 | 如果其他模块不再使用 `updateCounter`，可一并删除 |

### D-2 改动项（删除 Enable 后预发）

| 文件 | 当前 NEW 代码 | 改为 |
|---|---|---|
| `firmware_new/.../dummy_robot.cpp:626` 附近 | `if (_enable) { motorJ[ALL]->UpdateAngle(); }` | 删除整段 |

### 已正确代码（无需修改）

| 文件 | 行 | 代码 | 说明 |
|---|---|---|---|
| `dummy_robot.cpp:454-467` | `UpdateJointAnglesCallback` 中 `for (i=1; i<=6; i++)` 写 flag | OLD 逻辑，保留 |
| `can_protocol.cpp` case 0x23 | `UpdateAngleCallback + UpdateJointAnglesCallback` | 与 OLD 一致 |

### 禁止事项

- ❌ 不要保留 `currentIndex` 静态变量（D-1 已退回 OLD 广播，不再需要）
- ❌ 不要在 `UpdateJointAnglesCallback` 加 rail 处理（保持 6 轴清爽）
- ❌ 不要修改 `jointsStateFlag` 的 bit 分配（bit1-6）
- ❌ 不要恢复 `motorJ[ALL]->UpdateAngle()` 在 enable 边缘的预发（D-2 已删除）

---

## 验证

| 测试 | 期望 | 方法 |
|---|---|---|
| 广播刷新频率（disable） | 5kHz 实时环每帧 1 帧广播 → 7 电机各 ~5kHz | 调试串口打印 `dummy.UpdateJointAngles()` 调用间隔 |
| enable 时被动回包 | `MoveJoints()` 顺序发 6 个 0x07 → 电机自动回 0x23 | 调试串口打印 0x23 接收中断触发频率 |
| Enable 后**无**预发广播 | enable 后首次回包延迟无缩短 | `!DISABLE → !START` 后立即发 `>`，对比 OLD 行为 |
| jointsStateFlag 位正确 | 6 轴 bit1-6 全置位时 = 0x7E | 调试串口打印 `flag` |
| 地轨不参与 jointsStateFlag | 地轨回包不影响 flag bit0 | 同上 |

**已知限制**（不优化，接受）：
- enable 时 j6 angle 更新比 j1 晚 ~500us（OLD `MoveJoints()` 顺序发 0x07 触发的回包时序错开导致）

---

## 状态

- [x] 决策已记录（2026-09-24）
- [x] **2026-09-25 决策更新**：D-1 退回 OLD 广播；D-2 删除 Enable 后预发
- [x] **2026-09-27 落地**：
    - D-1：`UpdateJointAngles()` 回退 OLD 广播 + 删除 5 处 `% 50 == 25` 节流 + 失能分支简化（dummy_robot.cpp:436-441, main.cpp:60-118）
    - D-2：`SetEnable()` 末尾预发广播已删除（dummy_robot.cpp:614）
- [x] jointsStateFlag 6 轴逻辑已生效
- [x] 编译验证（2026-09-27 ninja 201/201 全通过，FLASH 36.64%, RAM 40.70%, CCMRAM 100%）
- [ ] D-3/D-4 待决策（用户决定暂时不优化，D-5 已搁置，j1-j6 ~500us 不均作为已知限制接受）
- [ ] 实测验证（OLED 第 3 行 + 串口）

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [02 - IsMoving](firmware_new主控固件模块02_运动完成判断-IsMoving.md) | jointsStateFlag 是 IsMoving 的判定依据 |
| [06 - OLED显示](firmware_new主控固件模块06_OLED显示.md) | currentJoints.a[] 是 OLED 第 3 行显示来源 |
| [10 - PID查询](firmware_new主控固件模块10_PID查询.md) | 0x23 轮询机制与 0x28~0x2D PID 轮询独立运行 |
| [15 - 5kHz实时环](firmware_new主控固件模块15_5kHz实时环.md) | 100Hz 主循环由 5kHz TIM7 节流 |
