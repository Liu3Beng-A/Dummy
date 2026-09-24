# 模块 01 — 运动指令（`>`/`&`/`@` + MoveJ/MoveL + 阻塞回 OK）

> **范围**：主控 ASCII 运动指令（`>` `&` `@`）的解析 + 下发 + 回执语义
> **关联文件**：`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`（`ParseCommand`）
>
> - **来源**：`firmware/ref_core_f405/` — 已实现的"新版本"
> - **目标**：`firmware_new/dummy-ref-core-fw/` — 当前待定
> - **基线**：`firmware_old/dummy-ref-core-fw/` — 旧版本（6 轴） **架构差异**：旧 6 轴架构 → 新 6 轴 + 地轨 + 夹爪架构 **创建日期**：2026-09-24

---

## 1. 三模式 × 三指令的最终回执语义（按 OLD 设计）


| 指令 \ 模式  | SEQ（顺序）        | INT（可打断）                 | TRJ（轨迹衔接）             |
| -------- | -------------- | ------------------------ | --------------------- |
| `>` `&`  | ✅ 阻塞（F1.1）    | ❌ 立即回 ok               | ✅ **阻塞**（F1.3，与 SEQ 共用同一段 ParseCommand 代码） |
| `@`      | ❌ 立即回 ok（F1.2） | ❌ 立即回 ok + `ClearFifo()` | ❌ 立即回 ok              |


> 阻塞回 ok = 等 6 电机到位（`jointsStateFlag = 0x7E`）才回 ok
> 立即回 ok = 下发后立即回 ok，不等电机
>
> **关键**（OLD 一致）：SEQ 和 TRJ 共用 ParseCommand 中的同一段 case 代码（fall-through），
> 区别仅在 `SetCommandMode()` 设置的 `jointSpeedRatio` 和 per-joint BASE 加速度档位。
> 仅 INT 单独一个 case ——`>` `&` `&` `&` `@` 全部立即回 OK，`@` 还附带 `ClearFifo()`。

---

## 2. F1 决策清单（全部已敲定）


| ID       | 指令             | 改动                                           | 实施状态                         |
| -------- | -------------- | -------------------------------------------- | ---------------------------- |
| **F1.1** | SEQ `>` `&`    | ✅ 改回阻塞                                       | 待实施                          |
| **F1.2** | SEQ/TRJ `@`    | ✅ 清理 break，保留立即回 ok                          | 待实施                          |
| **F1.3** | TRJ `>` `&` | ✅ **反转旧决策**：完全照搬 OLD 三件套（`jointSpeedRatio=0.3` + per-joint BASE 加速度档位 + ParseCommand 阻塞）。实现：新增 `JOINT_ACCEL_BASES[7]` + `ApplyJointAccelTier()`，SEQ/TRJ 共用 ParseCommand case，TRJ 用 HIGH=100%（与 SEQ 共阻塞代码） | 待新增 BASE 表 + helper + SetCommandMode 重构 + ParseCommand SEQ+TRJ fall-through |
| **F1.4** | `#CMDMODE 5/6` | ✅ **不改**，保持 firmware_new 当前行为                | —                            |
| **F1.5** | 默认上电模式 + SetEnable 副作用 + Homing/Resting 兜底 | ✅ 改为 INT（OLD 行为）+ ✅ 删 `SetEnable(false)` 防呆 + ✅ 删 Homing/Resting 30s 兜底超时 | 待改 3 处（h:267 + cpp:599-624 + cpp:520-555） |


---



## 3. F1.1 — SEQ `>` `&` 改回阻塞（OLD 队列语义）



### 行为说明

SEQ 模式下运动途中发新 `>` `&`，**新指令不被抛弃**，而是进入命令队列等待上一条到位后自动执行。回执 `ok` 在上一条电机全部到位后返回。

实现机制：`ControlLoopUpdateTask`（普通优先级）调用 `Pop()` → `ParseCommand()` → `MoveJ` → `MoveJoints` → `while(IsMoving())` **阻塞** → `Respond("ok")`。阻塞期间新指令仍由 `ascii_protocol.cpp` 持续 `Push` 入队。

### 改动位置

`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`，SEQ case 的 `>` `&` 分支

### 改前（v2.8 丢指令）

```cpp
if (context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed))
{
    // [v2.8 修复] ← 删除整个 if-break 块
    if (context->IsMoving() && context->IsEnabled())
    {
        Respond(*usbStreamOutputPtr,  "ok");
        Respond(*uart4StreamOutputPtr, "ok");
        break;
    }
    context->MoveJoints(context->targetJoints);
    context->motorJ[7]->SetPositionWithMotorRps(
        context->targetRailPos / 5.0f, context->railSpeedRps);
    Respond(*usbStreamOutputPtr,  "ok");
    Respond(*uart4StreamOutputPtr, "ok");
}
```



### 改后（OLD 阻塞）

```cpp
if (context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed))
{
    // [F1.1] 恢复 OLD 阻塞语义：立即下发 → 等到位 → 回 ok
    context->MoveJoints(context->targetJoints);
    context->motorJ[7]->SetPositionWithMotorRps(
        context->targetRailPos / 5.0f, context->railSpeedRps);

    while (context->IsMoving() && context->IsEnabled())
        osDelay(5);

    Respond(*usbStreamOutputPtr,  "ok");
    Respond(*uart4StreamOutputPtr, "ok");
}
```

---



## 4. F1.2 — SEQ/TRJ `@` 清理 break



### 行为说明

`@`（MoveL）在 SEQ 和 TRJ 模式下都是**非阻塞**——IK 计算开销大，且 OLD 也不阻塞。firmware v2.8 给 `@` 加了 `if(IsMoving) break` 丢指令逻辑是错误的，要清理。

### 改动位置

`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`，SEQ case 和 TRJ case 的 `@` 分支

### 改前（SEQ @ 有 break）

```cpp
if (context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
{
    // [v2.8 修复] ← 删除整个 if-break 块
    if (context->IsMoving() && context->IsEnabled())
    {
        Respond(*usbStreamOutputPtr,  "ok");
        Respond(*uart4StreamOutputPtr, "ok");
        break;
    }
    context->MoveJoints(context->targetJoints);
    context->motorJ[7]->SetPositionWithMotorRps(
        context->targetRailPos / 5.0f, context->railSpeedRps);
    Respond(*usbStreamOutputPtr,  "ok");
    Respond(*uart4StreamOutputPtr, "ok");
}
```



### 改后（清理 break，保留立即回 ok）

```cpp
if (context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]))
{
    context->MoveJoints(context->targetJoints);
    context->motorJ[7]->SetPositionWithMotorRps(
        context->targetRailPos / 5.0f, context->railSpeedRps);
    Respond(*usbStreamOutputPtr,  "ok");
    Respond(*uart4StreamOutputPtr, "ok");
}
```

> 注：TRJ `@` 在 firmware_new 里**本来就没有** break（v2.8 没污染它），无需改。

---



## 5. F1.3 — TRJ `>` `&` 完全照搬 OLD 三件套（jointSpeedRatio + per-joint BASE 加速度档位 + ParseCommand 阻塞）

### 决策反转说明（2026-09-24 用户拍板）

**旧决策「F1.3 拆解 OLD 双管齐下，只保留 `jointSpeedRatio=0.3`，删除 `SetJointAcceleration(LOW/HIGH)` 调用」整体废弃。**

新决策：**完全照搬 OLD 三件套**——`jointSpeedRatio` + per-joint BASE 加速度档位 + ParseCommand 阻塞。

#### 旧决策的问题

1. **OLD 默认是 INT，不是 SEQ**——这意味着 OLD 时代的核心阻塞语义是「SEQ + TRJ 共享」，而非「SEQ 单独」
2. **OLD 的 TRJ 是阻塞回 OK 的**——F1.3 旧文档基于「TRJ 不阻塞，依赖电机端 DCE 平滑衔接」的描述与 OLD 实际实现不符
3. **`jointSpeedRatio=0.3` 与「不调 SetJointAcceleration」逻辑错位**——
   - `jointSpeedRatio=0.3` 的设计动机是「**为加减速预留速度余量**」，必须搭配高加速度
   - 不调 SetJointAcceleration 时用户 `#ACC_J` 决定了实际加速度
   - 用户若配低加速度（`#ACC_J 50`），电机慢慢加减速，9 r/s 用 0.18s 就能到位，`ratio=0.3` 无意义
   - 用户若配高加速度（`#ACC_J 500`），电机快速加减速，`ratio=0.3` 才发挥作用
   - 结论：**`ratio=0.3` 必须配套固定高加速度才有意义**，否则是无效设计

#### 关键发现

| 维度 | OLD 实际 | F1.3 旧决策声称 | 真相 |
|---|---|---|---|
| OLD `DEFAULT_COMMAND_MODE` | `INTERRUPTABLE`（INT） | SEQUENTIAL | ❌ F1.3 旧决策错（错读了默认值） |
| OLD `>` `&` 是否阻塞 | **SEQ + TRJ 都阻塞，INT 不阻塞** | SEQ 阻塞，TRJ 不阻塞 | ❌ F1.3 旧决策错（漏读 ParseCommand 的 case fall-through） |
| OLD `SetJointAcceleration(LOW/HIGH)` | SEQ+INT 用 LOW=30%，TRJ 用 HIGH=100% | "v2.7 决定删除调用" | ⚠️ v2.7 决定真实存在，但**应反转**——应恢复 OLD 三件套 |
| OLD TRJ 平滑衔接机制 | 三件套 + 阻塞 FIFO | "靠电机端 DCE" | ❌ F1.3 旧决策错（混了 DCE 与主控阻塞责任） |

### 行为说明（新决策）

OLD TRJ 的真正差异化设计由**三件套**组成，三者必须**配套使用**才能达到「密集点位 + 快速响应」的设计目标：

| 序号 | 设计要素 | TRJ vs (SEQ/INT) 的差异 | 实现位置 |
|---|---|---|---|
| 1 | **`jointSpeedRatio`** | TRJ: **0.3**；SEQ/INT: 1.0 | `SetCommandMode()` |
| 2 | **per-joint BASE 加速度档位** | TRJ: **HIGH=100%**；SEQ/INT: LOW=30% | `SetCommandMode()` → `ApplyJointAccelTier()` |
| 3 | **ParseCommand 阻塞 vs 非阻塞** | TRJ + SEQ: **阻塞**（`while(IsMoving)`）；**仅 INT: 非阻塞** | `ParseCommand()` case 重排 |

注：OLD 中 SEQ 和 TRJ 的 ParseCommand 是**同一段代码**（case fall-through），只有 INT 走单独分支不阻塞。

### per-joint BASE 加速度表（方案 4 实现）

#### OLD BASE 表

```cpp
// firmware_old/dummy_robot.h:84
const DOF6Kinematic::Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES
    = {150, 100, 200, 200, 200, 200};
//                        ↑
//                    J3=J4=J5=J6=200
//                J2=100（最小，因减速比 30，电机能力有限）
```

| OLD 档位 | 档位值 (%) | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|---|
| LOW  | 30% | 45 r/s² | 30 r/s² | 60 r/s² | 60 r/s² | 60 r/s² | 60 r/s² |
| HIGH | 100% | 150 r/s² | 100 r/s² | 200 r/s² | 200 r/s² | 200 r/s² | 200 r/s² |

#### firmware_new 实施方案（保留 BASE 表 + 新增 helper）

**改动目标**：
1. 保留 per-joint BASE 表（差异化各关节能力）
2. 主控侧新增 `ApplyJointAccelTier(float pct)` 私有方法，按 BASE 表换算下发 r/s²
3. ASCII 命令 `#ACC_J <node> <rps2>` 继续走「单层 r/s²」语义——通过 `#ACC_J` 直接调底层 `motorJ[i]->SetAcceleration_persist()`，不经过档位翻译
4. `SetCommandMode()` 切换 SEQ/INT/TRJ 时调档位版本，硬覆盖到对应档位

**改动文件 1**：`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.h`

```cpp
// 在 const float DEFAULT_JOINT_ACCELERATION = 150.0f 之后添加
// OLD 风格：每关节一个 BASE 加速度上限，单位 r/s²
// 在 SetCommandMode() 切换 SEQ/INT/TRJ 时按百分比下发（LOW=30% / HIGH=100%）
// [v3.4] F1.3 改造：恢复 OLD BASE 表，保留 per-joint 差异化加速能力
//   索引 [0] = ALL 代理（占位，不参与）
//   索引 [1] = J1, [2] = J2, ..., [6] = J6
//   注：J7/地轨不参与（直连丝杆，无减速比，加速由电机端自动）
const float JOINT_ACCEL_BASES[7] = {
    0,      // [0] ALL 代理
    150,    // [1] J1 大臂
    100,    // [2] J2 肘关节（减速比小，电机能力有限）
    200,    // [3] J3 上臂
    200,    // [4] J4
    200,    // [5] J5
    200,    // [6] J6
};

// OLD 档位（与 OLD 版本一致，0~100）
// [v3.4] F1.3 改造：档位百分比 → per-joint BASE × pct → 直接下发 r/s²
//   LOW  = 30% 对应 SEQ/INT 模式（保守加速，每段都希望电机平稳加减速）
//   HIGH = 100% 对应 TRJ 模式（强加速，电机能快速响应新目标位置）
const float DEFAULT_JOINT_ACCELERATION_LOW  = 30.0f;   // 单位：百分比档位（不是 r/s²）
const float DEFAULT_JOINT_ACCELERATION_HIGH = 100.0f;  // 单位：百分比档位（不是 r/s²）
```

**改动文件 2**：`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`

新增 helper（私有方法）：

```cpp
/**
 * @brief 把 OLD 风格的档位百分比（0~100）按 per-joint BASE 表翻译成 r/s² 下发
 * @param _acc_pct 档位百分比 (0~100)，LOW=30 / HIGH=100
 * @note [v3.4] F1.3 改造：仅用于 SetCommandMode() 内部，
 *       ASCII #ACC_J <node> <rps2> 命令继续走单层 r/s² 语义（直接调 SetJointAcceleration(float)）。
 *       J7/地轨不参与（直连丝杆，加速度由电机端自动）。
 */
void DummyRobot::ApplyJointAccelTier(float _acc_pct)
{
    if (_acc_pct < 0)   _acc_pct = 0;
    if (_acc_pct > 100) _acc_pct = 100;

    float ratio = _acc_pct / 100.0f;

    for (int i = 1; i <= 6; i++)  // 仅 J1~J6（all=0 占位、J7/地轨直连）
    {
        float rps2 = JOINT_ACCEL_BASES[i] * ratio;
        if (rps2 < 1.0f)    rps2 = 1.0f;
        if (rps2 > 5000.0f) rps2 = 5000.0f;
        motorJ[i]->SetAcceleration_persist(rps2, false);
    }

    // 异步回填 runtime 缓存（不阻塞，与 SetJointAcceleration() 一致）
    SyncAllMotorAcceleration();
}
```

`SetCommandMode()` 改后：

```cpp
void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_TORQUE_CONTROL)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            // [F1.3 改造 2026-09-24] 恢复 OLD 双管齐下
            //   SEQ 与 INT 的 jointSpeedRatio / 加速度档位完全相同，仅 ParseCommand 阻塞行为不同
            jointSpeedRatio = 1;
            ApplyJointAccelTier(DEFAULT_JOINT_ACCELERATION_LOW);   // LOW = 30%
            break;

        case COMMAND_CONTINUES_TRAJECTORY:
            // [F1.3 改造 2026-09-24] 恢复 OLD 三件套
            //   ratio=0.3（速度上限缩到 30%）+ HIGH 加速度档位（电机端能快速响应新目标）
            //   两者必须配套使用，单独一个都失去设计意义
            jointSpeedRatio = 0.3;
            ApplyJointAccelTier(DEFAULT_JOINT_ACCELERATION_HIGH);  // HIGH = 100%
            break;

        case COMMAND_MOTOR_TUNING:
            break;

        case COMMAND_TORQUE_CONTROL:
            break;

        case COMMAND_SERVO_J:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION);  // 150 r/s²（保留现行为）
            break;
    }
}
```

### ParseCommand 三模式 × 三指令重新组织（完全照搬 OLD）

#### OLD ParseCommand 结构回顾

```cpp
// OLD firmware_old/dummy_robot.cpp ParseCommand
switch (context->commandMode)
{
    case COMMAND_TARGET_POINT_SEQUENTIAL:        // ←
    case COMMAND_CONTINUES_TRAJECTORY:           // ← 共用同一段代码
        if (_cmd[0] == '>' || _cmd[0] == '&')
        {
            // MoveJ + MoveJoints
            // ★ 阻塞：while (IsMoving() && IsEnabled()) osDelay(5);
            // Respond "ok"
        }
        else if (_cmd[0] == '@')
        {
            // MoveL（立即回 ok，不阻塞）
        }
        break;

    case COMMAND_TARGET_POINT_INTERRUPTABLE:
        if (_cmd[0] == '>' || _cmd[0] == '&')
        {
            // MoveJ + MoveJoints（立即回 ok，不阻塞）
        }
        else if (_cmd[0] == '@')
        {
            // MoveL（立即回 ok，不阻塞）+ ClearFifo()
        }
        break;
}
```

**关键点**：
1. SEQ + TRJ **共用同一段代码**（case fall-through）——`>` `&` 阻塞回 OK，`@` 立即回 OK
2. INT 单独一个 case——`>` `&` 立即回 OK，`@` 立即回 OK + `ClearFifo()`

#### firmware_new ParseCommand 改造方案

**当前 firmware_new 结构问题**：
- SEQ case 包含 v2.8 加的 `if(IsMoving) break` 丢指令逻辑 —— F1.1 已经决定改回阻塞
- TRJ case 是单独的，且非阻塞（v2.8 行为）—— 与 OLD 不一致
- INT case 是单独的非阻塞 —— 与 OLD 一致

**改造方案**：把 SEQ + TRJ 合并成同一个 case（fall-through），与 OLD 一致：

```cpp
switch (context->commandMode)
{
    case COMMAND_TARGET_POINT_SEQUENTIAL:        // ←
    case COMMAND_CONTINUES_TRAJECTORY:           // ← 共用同一段代码（OLD 一致）
        if (_cmd[0] == '>' || _cmd[0] == '&')
        {
            // 解析 7 + 1 个参数
            // MoveJ + MoveJoints
            // [F1.1+F1.3 改造] 阻塞：while (IsMoving() && IsEnabled()) osDelay(5);
            // Respond "ok"
        }
        else if (_cmd[0] == '@')
        {
            // 解析 pose + speed
            // MoveL（立即回 ok，不阻塞）
            // Respond "ok"
        }
        break;

    case COMMAND_TARGET_POINT_INTERRUPTABLE:
        if (_cmd[0] == '>' || _cmd[0] == '&')
        {
            // 解析 + MoveJ + MoveJoints
            // 立即回 ok，不阻塞
        }
        else if (_cmd[0] == '@')
        {
            // 解析 + MoveL + ClearFifo()
            // 立即回 ok，不阻塞
        }
        break;
    // ...
}
```

**注意**：v2.x 还加了 `$` 力矩透传和 SERVO_J 处理。这些分支位置不动（不参与三件套语义）。

### F1.3 改造任务清单（最终决策 — 方案 4 完全照搬 OLD）

| # | 任务 | 决策 | 涉及文件 |
|---|---|---|---|
| 1 | 新增 `JOINT_ACCEL_BASES[7]` 表 | 添加到 `dummy_robot.h`（DEFAULT_JOINT_ACCELERATION 之后） | `dummy_robot.h` |
| 2 | 新增 `DEFAULT_JOINT_ACCELERATION_LOW/HIGH` 常量 | 添加到 `dummy_robot.h` | `dummy_robot.h` |
| 3 | 新增私有方法 `ApplyJointAccelTier(float pct)` | 添加到 `dummy_robot.cpp` | `dummy_robot.cpp` |
| 4 | `SetCommandMode()` 改造 | SEQ/INT → LOW + ratio=1；TRJ → HIGH + ratio=0.3 | `dummy_robot.cpp` |
| 5 | `ParseCommand()` 改造 | 把 SEQ case 和 TRJ case 合并成 fall-through（共用同一段代码） | `dummy_robot.cpp` |
| 6 | `ParseCommand()` SEQ+TRJ `>` `&` 阻塞回 OK | 复用 F1.1 的 `while(IsMoving && IsEnabled) osDelay(5)` 模式 | `dummy_robot.cpp` |

### F1.3 决策说明

- **完全照搬 OLD 三件套**——`jointSpeedRatio=0.3` + per-joint BASE + ParseCommand 阻塞语义，三者必须配套使用
- **恢复 OLD 的 BLOCKING ParseCommand 设计**——SEQ/TRJ 共用同一段阻塞代码，仅 INT 不阻塞，避免 v2.8「TRJ 非阻塞」的副作用
- **保留 per-joint BASE 表**——J2 减速比小、电机能力有限，BASE=100 比统一 175/200 更合理
- **ASCII `#ACC_J <node> <rps2>` 单层 r/s² 语义不变**——用户单关节调参不受影响；只是模式切换时主控按档位翻译统一覆盖
- **`ApplyJointAccelTier()` 仅用于 SetCommandMode 内部**——ASCII 命令、调参接口不经过这个 helper，保持接口清晰
- **加速度档位选择权的语义转换**——OLD 时代用户不能配 #ACC_J，是档位驱动；现在用户能用 #ACC_J 单关节调参，但模式切换时硬覆盖——这是「保留 OLD 设计哲学 + 兼容 v2.6+ #ACC_J」的折中

#### 与「模式切换硬覆盖 `#ACC_J`」相关的副作用

- 用户先发 `#ACC_J 6 500` 让 J6 用 500 r/s² 加速度
- 然后发 `#CMDMODE 1`（SEQ 模式）
- 此时主控会调 `ApplyJointAccelTier(30)` 把 J6 强制改成 60 r/s²（BASE[6]=200 × 30%）
- 用户再次发 `#ACC_J 6 500` 才能恢复
- **OLD 时代也是这个行为**——档位切换总是覆盖用户配置，这是设计哲学
- 接受这个副作用，无需额外逻辑避免（避免破坏 OLD 语义）

---



## 6. F1.5 — 默认上电模式 + SetEnable 防呆 + Homing/Resting 兜底 三项清理



### 6.1 默认上电模式改 INT

#### 改动位置

`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.h:267`

#### 改前

```cpp
const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_SEQUENTIAL;  // 1=阻塞
```

#### 改后

```cpp
const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;  // 2=非阻塞（OLD 默认）
```

#### 理由

- 与 OLD 默认行为一致，老用户无感切换
- 默认非阻塞 = 堵转时主控不会卡在 `while(IsMoving())` 里
- 需要阻塞时上位机启动后主动发 `#CMDMODE 1`（OLED 显示 `[SEQ]`）

### 6.2 配套删除 SetEnable(false) 的防呆副作用（firmware v3.x 添加）

#### 现象

当用户发 `!DISABLE`（失能）时，**期望行为**是「保持当前模式不变，下次 `!ENABLE` 后继续当前工作」。

但 firmware v3.x 在 `SetEnable(false)` 中加了：

```cpp
// firmware/dummy_robot.cpp（原 v3.x 代码）
void DummyRobot::SetEnable(bool _enable)
{
    if (!_enable)
    {
        // ...
        SetCommandMode(DEFAULT_COMMAND_MODE);   // ← firmware v3.x 防呆
        // ...
    }
}
```

**后果**：用户在工作模式 SEQ 下发 `!DISABLE`，再 `!ENABLE` → 模式被悄悄切回 INT（DEFAULT）。

#### 决策

**删除 `SetEnable(false)` 中的 `SetCommandMode(DEFAULT_COMMAND_MODE)` 调用**。

理由：

- `!DISABLE` 不应该改用户当前的工作模式（用户切到 SEQ 后正在做点位控制，`!DISABLE` 不该把模式默默改回 INT）
- 用户明确「默认 = 上电后一直是 INT，用户主动发 `#CMDMODE 1` 再切换到 SEQ」—— 默认语义只在上电那一刻生效，**不**应延伸到 `!DISABLE`
- 与 stall v3 删除同思路：v3.x 加的「防呆」副作用是 firmware 那一坨被放弃的根源，应一并删除

#### OLD 对照（`firmware_old/dummy_robot.cpp:261-265`）

```cpp
void DummyRobot::SetEnable(bool _enable)
{
    motorJ[ALL]->SetEnable(_enable);   // OLD: 没有 SetCommandMode
    isEnabled = _enable;
}
```

OLD 就没有这条防呆，`!DISABLE` 保持当前模式不变。

#### 改动位置

`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp` 的 `SetEnable(bool)` 函数（约 line 599-624）

#### 当前代码（firmware v3.x 防呆）

```cpp
void DummyRobot::SetEnable(bool _enable)
{
    if (_enable)
    {
        SetRGBMode(rgbStateEnable);
    }
    else
    {
        SetRGBMode(rgbStateDisable);

        for (int i = 0; i < 6; i++)
            targetCurrents[i] = 0.0f;

        SetCommandMode(DEFAULT_COMMAND_MODE);   // ← [F1.5] 删这行
        // [A3 修复] 不再写 targetJoints = currentJoints；
        //          jointsStateFlag 方案下，电机 STOP 时 jointsStateFlag 自动全置位
    }
    // ...
}
```

#### 改后

```cpp
void DummyRobot::SetEnable(bool _enable)
{
    if (_enable)
    {
        SetRGBMode(rgbStateEnable);
    }
    else
    {
        SetRGBMode(rgbStateDisable);

        for (int i = 0; i < 6; i++)
            targetCurrents[i] = 0.0f;

        // [F1.5] 删除 SetCommandMode(DEFAULT_COMMAND_MODE)
        //   !DISABLE 保持当前模式不变，不丢用户的工作状态
        //   默认语义只在上电那一刻生效（由 DEFAULT_COMMAND_MODE 定义）
        //   [A3 修复] 不再写 targetJoints = currentJoints；
        //            jointsStateFlag 方案下，电机 STOP 时 jointsStateFlag 自动全置位
    }
    // ...
}
```

### 6.3 配套：Homing/Resting 删 30s 兜底超时（firmware v3.x 防卡死）

#### 现象

firmware v3.x 在 `Homing()` 和 `Resting()` 函数中加了 30s 兜底超时：

```cpp
// firmware/dummy_robot.cpp（原 v3.x 代码）
void DummyRobot::Homing()
{
    // ...
    MoveJoints(targetJoints);
    uint32_t timeoutTick = osKernelGetTickCount() + 30000;   // ← v3.x 防卡死
    while (context->IsMoving())
    {
        if (osKernelGetTickCount() > timeoutTick)
            break;
        osDelay(10);
    }
}

void DummyRobot::Resting()
{
    // 同样的 30s 兜底超时
}
```

30s 兜底是 firmware v3.x 加的"防卡死"补丁，**属于 firmware 那一坨被放弃的副作用**。

#### 决策

**删除 `Homing()` 和 `Resting()` 中的 30s 兜底超时**，回到 OLD `while(IsMoving())` 纯循环。

理由：

- F5 决策（模块 02 待开启时定义）：「严格信任电机端」整体语义要求——`IsMoving()` 公式回退 OLD 后，循环也回退 OLD 行为
- OLD 时代的现场从未遇到 Homing/Resting 真卡死（电机端 DCE 闭环会自然到位）
- 与 stall v3 删除同思路：v3.x 加的「防呆/防卡死」副作用应一并回退

**`&& IsEnabled()` 短路边界说明**：

| 场景 | OLD 行为 | 当前决策 |
|---|---|---|
| `Homing()` / `Resting()` 内部 while 循环 | **无** `&& IsEnabled()` 短路 | **保持无短路**（OLD 一致） |
| SEQ `>` `&` 阻塞循环（§3 F1.1 改后代码） | **有** `&& IsEnabled()` 短路 | **保持有短路**（OLD 一致，§3 改后代码已正确） |

两类场景不一致的原因：Homing/Resting 是**内部触发**的复位动作，`!DISABLE` 期间不应该被触发；SEQ `>` `&` 是**用户主动**发指令的上下文，`!DISABLE` 时主控阻塞会卡死，所以保留 `&& IsEnabled()` 短路避免阻塞。

#### OLD 对照（`firmware_old/dummy_robot.cpp:232-258`）

```cpp
void DummyRobot::Homing()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);
    MoveJ(0, 0, 90, 0, 0, 0);
    MoveJoints(targetJoints);
    while (IsMoving())         // ← OLD: 纯 while(IsMoving())，无 30s 兜底
        osDelay(10);
    SetJointSpeed(lastSpeed);
}

void DummyRobot::Resting()
{
    // 同样的纯 while(IsMoving()) 循环
}
```

#### 改动位置

`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp` 的 `Homing()` 和 `Resting()` 函数（约 line 520-555）

#### 当前代码（firmware v3.x 防卡死）

```cpp
void DummyRobot::Homing()
{
    float lastSlider = jointSpeedRps / SLIDER_TO_RPS;
    SetJointSpeed(10);
    MoveJ(0, 0, 90, 0, 0, 0, 0, 10);
    MoveJoints(targetJoints);
    // [v3.x] 30s 兜底超时 ← 删
    uint32_t timeoutTick = osKernelGetTickCount() + 30000;
    while (context->IsMoving())
    {
        if (osKernelGetTickCount() > timeoutTick)
            break;
        osDelay(10);
    }
}

void DummyRobot::Resting()
{
    // 同样的 30s 兜底超时
}
```

#### 改后（OLD 行为）

```cpp
void DummyRobot::Homing()
{
    float lastSlider = jointSpeedRps / SLIDER_TO_RPS;
    SetJointSpeed(10);
    MoveJ(0, 0, 90, 0, 0, 0, 0, 10);
    MoveJoints(targetJoints);
    // [F1.5 配套 §6.3] 删 30s 兜底超时，回到 OLD while(IsMoving()) 纯循环
    while (context->IsMoving())
        osDelay(10);
}

void DummyRobot::Resting()
{
    float lastSlider = jointSpeedRps / SLIDER_TO_RPS;
    SetJointSpeed(10);
    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5], 0, 10);
    MoveJoints(targetJoints);
    // [F1.5 配套 §6.3] 删 30s 兜底超时
    while (context->IsMoving())
        osDelay(10);
}
```

#### 风险说明

- 若电机真的卡死（断线/损坏/物理堵转），Homing/Resting 会**永久阻塞**
- OLD 时代的现场从未遇到这种情况（旧固件稳定运行多年）
- 如果用户希望保留兜底安全网，F5 决策需要扩展（"OLD 公式 + Homing/Resting 保留 30s 兜底作为例外"），二者择一

---



## 7. F1.4 — `#CMDMODE 5/6` 保持现状（不改）



### 决策

**保持 firmware_new 当前实现**，不调整。

### 当前行为

- `#CMDMODE 5`（TORQUE_CONTROL）：`$` 指令独立分支处理 7 通道电流下发；`>` `&` `@` 在此模式下走 INT 分支
- `#CMDMODE 6`（SERVO_J）：`ServoJ + MoveJoints + 地轨下发`，无回 ok（5kHz 环内）

---



## 8. 验证清单


| #   | 测试                       | 期望                                                                | 命令                                  |
| --- | ------------------------ | ----------------------------------------------------------------- | ----------------------------------- |
| 1   | SEQ `>` 阻塞回 OK           | 发指令后等 6 电机到位才回 ok                                                 | `#CMDMODE 1; >0,-73,180,0,0,0,0,30` |
| 2   | **TRJ `>` 阻塞回 OK**（F1.3 反转） | 下发后等 6 电机到位才回 ok（与 SEQ 一致，OLD 行为）                                    | `#CMDMODE 3; >0,-73,180,0,0,0,0,30` |
| 3   | INT 立即回 OK                | 下发后立即回 ok                                                         | `#CMDMODE 2; >0,0,0,0,0,0,0,30`     |
| 4   | `@` 立即回 OK（任意模式） | 下发后立即回 ok                                                         | `#CMDMODE 1; @0.3,0,0.2,0,0,0,30`   |
| 5   | SEQ 阻塞期间 FIFO 积压 | 第二条指令延迟到第一条完成后才消费                                                 | 发两条 `>` 紧挨着                         |
| 6   | 默认上电模式                    | 上电后 OLED 显示 `[INT]`                                               | 重启主控                                |
| 7   | TRJ 模式速度比例                | 切到 `#CMDMODE 3` 后，发 `>` 观察电机端实际速度上限缩为原来的 30%（`jointSpeedRatio=0.3`） | `#CMDMODE 3; >0,-73,180,0,0,0,0,30` + 观察电机端 |
| **8** | **TRJ 加速度档位下发**（F1.3 新增） | 切到 `#CMDMODE 3` 后，用 `#GETJACC 6` 查 J6 加速度，应为 200 r/s²（BASE[6] × 100%）；切回 `#CMDMODE 1` 后再查，应为 60 r/s²（BASE[6] × 30%） | `#CMDMODE 3; #GETJACC 6` → 200；`#CMDMODE 1; #GETJACC 6` → 60 |
| **9** | **TRJ 模式下每段距离缩短**（F1.3 新增） | TRJ 模式下从一位置到另一位置，由于 `jointSpeedRatio=0.3`，单段执行时间约为 SEQ 模式的 3.3 倍（同距离同加速度），运动更慢但电机加减速更稳       | 对比 `#CMDMODE 1` vs `#CMDMODE 3` 在相同距离下的执行时间                  |


---



## 9. 状态

- [x] F1.1 / F1.2 / F1.3 / F1.4 / F1.5 用户拍板（2026-09-24）
- [x] F1.3 决策反转（2026-09-24）：完全照搬 OLD 三件套，旧"拆解 + 仅 ratio=0.3"决策整体废弃
- [ ] F1.1 代码修改（SEQ `>` `&` 改回阻塞，删除 v2.8 丢指令 break）
- [ ] F1.2 代码修改（SEQ `@` 清理 break，TRJ `@` 无需改）
- [ ] F1.3 代码修改（共 6 处）：
  - [ ] §5 任务 1：`dummy_robot.h` 新增 `JOINT_ACCEL_BASES[7]` 表
  - [ ] §5 任务 2：`dummy_robot.h` 新增 `DEFAULT_JOINT_ACCELERATION_LOW/HIGH` 常量
  - [ ] §5 任务 3：`dummy_robot.cpp` 新增私有方法 `ApplyJointAccelTier(float pct)`
  - [ ] §5 任务 4：`dummy_robot.cpp` 重构 `SetCommandMode()`（SEQ/INT 用 LOW + ratio=1；TRJ 用 HIGH + ratio=0.3）
  - [ ] §5 任务 5：`dummy_robot.cpp` 重构 `ParseCommand()`（SEQ + TRJ 合并成 fall-through）
  - [ ] §5 任务 6：`dummy_robot.cpp` 在合并后的 SEQ+TRJ `>` `&` 分支加阻塞循环（沿用 F1.1 写法）
- [ ] F1.5 三处代码修改：
  - [ ] §6.1 默认上电模式（`dummy_robot.h:267` → `INTERRUPTABLE`）
  - [ ] §6.2 `SetEnable(false)` 删除 `SetCommandMode(DEFAULT_COMMAND_MODE)`（`dummy_robot.cpp:599-624`）
  - [ ] §6.3 `Homing()` / `Resting()` 删除 30s 兜底超时（`dummy_robot.cpp:520-555`）
- [ ] 编译验证（ninja）
- [ ] 实测验证（SEQ/TRJ 阻塞回 OK / TRJ 速度比 0.3 / TRJ 加速度档位下发 / `!DISABLE` 不丢模式 / `!HOME` 不卡死）

---



## 10. 关联模块


| 模块                                                             | 关联点                                                     |
| -------------------------------------------------------------- | ------------------------------------------------------- |
| [02 - IsMoving](firmware_new主控固件模块02_运动完成判断-IsMoving.md)       | `while(IsMoving)` 必须用 OLD 公式（`jointsStateFlag != 0x7E`） |
| [03 - UpdateAngle](firmware_new主控固件模块03_关节角度同步-UpdateAngle.md) | jointsStateFlag 更新靠 0x23 回包                             |
| [04 - EStop](firmware_new主控固件模块04_急停-EStop.md)                 | 急停清 FIFO + flag=0x7E → 阻塞循环自然退出                         |
| [05 - CommandMode](firmware_new主控固件模块05_命令模式-CommandMode.md)   | 6 模式 cmdModeNames + Switch case                         |
| [13 - 架构常量](firmware_new主控固件模块13_架构常量.md)                      | REST_POSE + 7 关节减速比                                     |


