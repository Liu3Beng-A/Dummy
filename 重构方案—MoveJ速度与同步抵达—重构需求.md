# MoveJ 速度单位 + 同时抵达偏差 — 重构需求

> 文档目的：把和 AI 助手的所有沟通结论固化下来，作为后续重构任务的依据。
> 文档状态：**需求讨论中**（2026-08-23 03:20）。
> 已确认决策：D1（slider 100=200 r/s）/ D2（统一单位 r/s）/ D4（地轨参与同步抵达）/ D8（堵转之后）。
> 待用户确认：TC1（j7_mm 第 7 位）/ TC2（jointSpeedRatio 保留）/ TC3（CONTINUES_TRAJECTORY 自动减半）。
> 待决问题：P1（`SetAngleWithVelocityLimit` 换算公式）/ P2（同步抵达算法 B1/B2/B3）。
> 启动时机：本重构排在 **堵转检测重构** 完成后开始。

---

## 1. 背景

### 1.0 源码对比发现（2026-08-22 00:25）

AI 把当前固件和原始 V2 版本固件（`source code\dummy-ref-core-fw\`、`source code\dummy-42motor-fw\`）做了完整对比。**两个 bug 都是后期重构时把 `reduction` 和单位混淆丢掉了，不是原始设计的 bug**。

#### 1.0.1 原始版本 MoveJ 速度计算公式（`dummy_robot.cpp:87-91`）

```cpp
float time = maxAngle * (float) (motorJ[index + 1]->reduction) / jointSpeed;     // ← 单位：电机时间
for (int j = 1; j <= 6; j++) {
    dynamicJointSpeeds.a[j - 1] =
        abs(deltaJoints.a[j - 1] * (float) (motorJ[j]->reduction) / time * 0.1f);  // ← 单位：电机 r/s × 0.1
}
```

**关键点**：
1. `dynamicJointSpeeds` 单位是 **电机 r/s**（不是关节 °/s）
2. **× 0.1** 是从 r/s 缩到 0~10 r/s 范围（原始速度档位 0~10）
3. `reduction` 在计算中考虑——关节角度差 × reduction 转换为电机端步/秒
4. **time 用 maxAngle 关节°× reduction ÷ jointSpeed** —— jointSpeed 是电机 r/s

**原始语义链**：`jointSpeed` (电机 r/s, 0~10) → 各关节 `dynamicJointSpeeds` (电机 r/s) → CAN 0x07 发到电机 → 电机按 0x07 接收的 r/s 跑（**电机端 0x07 也是按 r/s 解释**）。

#### 1.0.2 当前版本 MoveJ 速度计算（`dummy_robot.cpp:308-318`）

```cpp
deltaAngles = targetJointsTmp - currentJoints;
maxAngle = AbsMaxOf6(deltaAngles, maxIndex);
timeSec  = maxAngle / jointSpeed;                                                  // ← 单位：关节°
for (int j = 1; j <= 6; j++)
    dynamicJointSpeeds.a[j - 1] = fabsf(deltaAngles.a[j-1]) / timeSec;             // ← 单位：关节°/s
```

**问题**：
1. `dynamicJointSpeeds` 单位变成 **关节 °/s**
2. **`reduction` 字段被完全丢弃**
3. **× 0.1 缩放没了**——但用户感知"超过 10 没变化"，可能就是因为没有这个缩放 + 默认 `velocityLimit=30 r/s` 上限
4. `jointSpeed` 默认 30（`dummy_robot.h:108 DEFAULT_JOINT_SPEED = 30`，文档注释 "degree/s"）

#### 1.0.3 还原真相：原始版本 vs 当前版本对比

| 维度 | 原始 V2 版本 | 当前版本 | 影响 |
|---|---|---|---|
| `jointSpeed` 默认值 | 不在 .h 里硬编码（`SetJointSpeed(30)` 在 `Init()` 调） | `DEFAULT_JOINT_SPEED = 30`（"degree/s"） | 同值 |
| `dynamicJointSpeeds` 单位 | 电机 r/s | 关节 °/s | **单位错位** |
| `reduction` 是否参与计算 | ✅ 参与 | ❌ 丢弃 | **关节差转电机端完全没用 reduction** |
| × 0.1 缩放 | 有（0~10 r/s 缩放） | 无 | 原始 0~10 速度档位设计丢失 |
| 默认 `velocityLimit` | 30 r/s | 30 r/s（不变） | - |
| 速度上限 | `_speed > 100 → 100` | `_speed > 100 → 100` | 不变 |

#### 1.0.4 原始版本 CAN 0x07 接收侧（`interface_can.cpp:87-107`）

```cpp
case 0x07:  // Set Position with Velocity-Limit
    motor.config.motionParams.ratedVelocity =
        (int32_t) (*(float*) (RxData + 4) * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
        //                                  ↑ 直接乘细分倍数 —— 单位是 r/s
```

电机端 `ratedVelocity` 内部单位是 `步/200us`（r/s × MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS），**0x07 接收的就是 r/s**。

→ **原始版本主控发的 `dynamicJointSpeeds` 是 r/s**，电机端接收的也是 r/s，**单位一致**。

→ **当前版本主控发的 `dynamicJointSpeeds` 是 °/s**，电机端接收按 r/s 解释，**单位错位**——这正是用户报告"速度 1-10 有变化，超过 10 一样"的根因。

#### 1.0.5 还原"超过 10 一样"现象的真实数字

| 用户 speed (slider) | `jointSpeed` (SetJointSpeed) | 当前 `dynamicJointSpeeds` (°/s) | 原始 `dynamicJointSpeeds` (r/s, 含 ×0.1) | 电机端解释 (r/s) | 超过 30 r/s 上限？ |
|---|---|---|---|---|---|
| 1 | 1 | 1°/s | 1 r/s × 0.1 = 0.1 r/s | 0.1 r/s | ❌ |
| 5 | 5 | 5°/s | 5 × 0.1 = 0.5 r/s | 0.5 r/s | ❌ |
| 10 | 10 | 10°/s | 10 × 0.1 = 1 r/s | 1 r/s | ❌ |
| 30 | 30 | 30°/s | 30 × 0.1 = 3 r/s | 3 r/s | ❌ |
| 50 | 50 | 50°/s | 50 × 0.1 = 5 r/s | 5 r/s | ❌ |
| 100 | 100 | 100°/s | 100 × 0.1 = 10 r/s | 10 r/s | ❌ |

**等等！原始版本 × 0.1 后所有速度都远低于 30 r/s 上限**，按这个表原始版本不会触发"超过 10 一样"的现象！

**那用户报告的现象到底是版本退化造成的，还是原始版本就有的？**

让我重新审视 —— 原始 `dynamicJointSpeeds.a[j-1] = abs(deltaJoints.a[j-1] * (float) (motorJ[j]->reduction) / time * 0.1f)`：

- `deltaJoints.a[j-1]` 单位**关节°**
- × `reduction`（50）→ 单位转换到电机输出圈（**注意**：reduction 是输入轴:输出轴，输入圈 = 输出圈 × reduction，但代码里直接当 stepMotorCnt = angle/360 × reduction 用，意味着 reduction 实际是"输出转 1 圈需要的电机步数倍数"——35 电机 reduction=50，意思是输出 1 圈需要电机转 50 圈）
- ÷ `time`（秒）→ 电机 r/s × reduction
- × 0.1 → 0~10 r/s 缩放

→ 原始版本 `dynamicJointSpeeds` 单位实际是 **电机 r/s × reduction × 0.1** = "电机输出轴 r/s × reduction × 0.1"——这个单位也奇怪。

**所以原始版本**也不能算"正确"。**两个版本单位都是设计有缺陷的**。但原始版本因为 `× 0.1` 把数字压小，反而不容易触顶；当前版本没有缩放，直接发大数 → 经常触顶。

**结论**：原始版本的设计就是**不严谨**，但没有用户能看到的 bug；当前版本的退化让用户能感知到单位错位的现象。**重构目标不是"恢复到原始版本"，而是"设计一个正确且清晰的单位体系"**。

---

### 1.1 当前问题

用户在 2026-08-21 23:54 报告两个 MoveJ 相关的实测现象：

| # | 现象 | 用户原话 |
|---|------|---------|
| A | "串口助手发送 MoveJ 指令时，速度 1-10 都有变化，一旦速度超过 10 往后就和 10 一样了" | "为什么" |
| B | "算法不是有同时抵达的逻辑吗，为什么我测试发现会有些偏差呢，运动角度小的关节会更早到达目标位置" | "你全面分析固件后给我回复" |

AI 在 2026-08-22 00:06 全面扫描主控 + 电机固件后，定位到 **两个独立的结构性缺陷**。

### 1.2 根因定位（AI 诊断结论）

#### 1.2.1 问题 A — 速度单位混淆

跨固件链路发现 **速度参数的"双重身份"**：

**环节 1：串口助手发送**（`串口助手.py:1874`）

```python
cmd = f">{joints[0]},{joints[1]},{joints[2]},{joints[3]},{joints[4]},{joints[5]},{j7},{speed}"
```

`speed` 由 UI 滑块传入，默认 50，可设 1-100（不限）。

**环节 2：主控 ASCII 解析**（`dummy_robot.cpp:711`）

```cpp
argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f,%f",
                joints, joints+1, joints+2, joints+3, joints+4, joints+5, &j7, &speed);
if (argNum == 8) context->SetJointSpeed(speed);
```

**环节 3：`SetJointSpeed`**（`dummy_robot.cpp:402-408`）

```cpp
void DummyRobot::SetJointSpeed(float _speed)
{
    if (_speed < 0)        _speed = 0;
    else if (_speed > 100) _speed = 100;     // ← 上限 100
    jointSpeed = _speed * jointSpeedRatio;   // jointSpeed 单位：关节 °/s（按注释）
}
```

**环节 4：`MoveJ` 同时抵达计算**（`dummy_robot.cpp:291-309`）

```cpp
deltaAngles = targetJointsTmp - currentJoints;
maxAngle = AbsMaxOf6(deltaAngles, maxIndex);
timeSec  = maxAngle / jointSpeed;                   // 假设匀速
for (int j = 1; j <= 6; j++)
    dynamicJointSpeeds.a[j-1] = fabsf(deltaAngles.a[j-1]) / timeSec;  // 关节°/s
```

**环节 5：主控 → 电机 driver**（`ctrl_step.cpp:319-324`）

```cpp
void CtrlStepMotor::SetAngleWithVelocityLimit(float _angle, float _vel)
{
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionWithVelocityLimit(stepMotorCnt, _vel);   // ← _vel 没做单位转换！
}
```

**环节 6：电机端 0x07 接收**（`interface_can.cpp:94-95`）

```cpp
motor.config.motionParams.ratedVelocity =
    (int32_t) (*(float*) (RxData + 4) * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
//                                ↑ 把它当"电机 r/s"
```

**环节 7：电机端默认上限**（`main.cpp:36/42`）

```cpp
.velocityLimit = 30 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // r/s（圈/秒）
```

**结论**：主控发的是 **关节 °/s**，电机端把它当 **r/s** 解读。当 `jointSpeed > 30°/s` 时电机端 `ratedVelocity` 超过默认 30 r/s 上限，被钳死。

| 用户 speed | 主控 `jointSpeed` | 主控下发 `_vel` (°/s) | 电机端解释为 (r/s) | 电机端 `ratedVelocity` | 是否被 30 r/s 上限截断？ |
|---|---|---|---|---|---|
| 1 | 1°/s | 1 | 1 r/s | 51200 步/200us | ❌ 不截断 |
| 5 | 5°/s | 5 | 5 r/s | 256000 步/200us | ❌ 不截断 |
| 10 | 10°/s | 10 | 10 r/s | 512000 步/200us | ❌ 不截断（接近上限）|
| 30 | 30°/s | 30 | 30 r/s | 1536000 步/200us | ❌ 刚好等于上限 |
| 50 | 50°/s | 50 | 50 r/s | 1536000（被截断） | ✅ 截断到 30 r/s |
| 80 | 80°/s | 80 | 80 r/s | 1536000（被截断） | ✅ 截断到 30 r/s |
| 100 | 100°/s | 100 | 100 r/s | 1536000（被截断） | ✅ 截断到 30 r/s |

> **根因**：主控把 `_vel` 当 `°/s` 发，电机端把它当 `r/s` 解读并限幅到 30 r/s。speed > 30 后所有关节都被钳到 30 r/s。
>
> 用户报告"超过 10 之后差不多" 比算的"30 之后一样"还要提前 —— 加速度限制和实际梯形曲线让小幅关节的实际速度提前饱和。

#### 1.2.2 问题 B — 短关节提前到达

**算法侧**（`dummy_robot.cpp:291-301`）：

```cpp
timeSec = maxAngle / jointSpeed;                                          // 匀速假设
for (int j = 1; j <= 6; j++)
    dynamicJointSpeeds.a[j-1] = fabsf(deltaAngles.a[j-1]) / timeSec;      // 按比例反推
```

**电机端实际执行**（`motion_planner.cpp:190-330`）：**梯形加减速**，不是匀速。
- 加速段：`velocityUpAcc = 100 r/s² = 36000°/s²`（关节上）
- 匀速段：达到 `ratedVelocity` 后保持
- 减速段：`velocityDownAcc` 同上

**短关节的全时间分析**（以 maxAngle=90°、jointSpeed=80°/s、短关节 5° 为例）：
- 规划 timeSec = 90/80 = 1.125s
- 短关节 dynamicJointSpeeds = 5/1.125 = 4.44°/s
- **因为速度太低，5° 关节完全没有匀速段**，几乎全程在加速段 + 减速段
- 单程时间 ≈ 2 × √(2 × 5 / 36000) ≈ 0.033s
- **实际耗时 ~0.033s << 规划的 1.125s**
- 长关节 90° 用 1.125s 完成 → 短关节提前 1.09s 到达

**主控 IsMoving 判定**（`dummy_robot.cpp:564-573`）：

```cpp
bool DummyRobot::IsMoving()
{
    static constexpr float EPSILON_DEG = 1.0f;
    for (int i = 1; i <= 6; i++)
        if (fabsf(motorJ[i]->angle - motorJ[i]->targetAngle) > EPSILON_DEG)
            return true;
    return false;
}
```

SEQ 模式下 `while (context->IsMoving() && context->IsEnabled()) osDelay(5);` —— **只有所有 6 个关节都进入 1° 容差范围才退出**。所以短关节提前到位不影响 MoveJ 阻塞，但**机械臂结构上"短关节已停"是肉眼可见的**（用户报告的现象）。

**电机端 FINISH 判定**（`motor.cpp:334-336`）：

```cpp
if ((controller->softPosition == controller->goalPosition)
    && (controller->softVelocity == 0))
    controller->state = STATE_FINISH;
```

加上 100Hz 0x23 查询的滞后 + 1° 容差 + 减速过冲，短关节实际到位时间被进一步拉前。

> **根因**：主控"同时抵达"算法用 **匀速假设**，电机端 **梯形加减速** —— 短关节没匀速段，实际总时间 << 规划 timeSec —— 提前到达。

### 1.3 关键文件位置（影响范围）

**主控**：
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp` — `MoveJ`、`SetJointSpeed`、`MoveJoints`、`MoveRail`、`IsMoving`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.h` — `DEFAULT_JOINT_SPEED` 常量
- `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` — `SetAngleWithVelocityLimit`
- `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.hpp` — 接口声明

**电机固件**（4 份同步）：
- `firmware/motor_fw_f103_35/UserApp/protocols/interface_can.cpp` — 0x07 处理
- `firmware/motor_fw_f103_42/UserApp/protocols/interface_can.cpp`
- `firmware/motor_fw_f103_57/UserApp/protocols/interface_can.cpp`
- `firmware/motor_fw_f103_gripper/UserApp/protocols/interface_can.cpp`

**运动规划**：
- `firmware/motor_fw_f103_*/Ctrl/Motor/motion_planner.cpp` — `positionTracker.CalcSoftGoal`

---

## 2. 已确认的决策（2026-08-23）

> **说明**：以下决策为本次重构的**唯一正确决策**。如与之前版本（包括本文档早期版本、`重构方案—参数重构需求.md` 3.3.1 节、聊天记录中的旧方案）有任何冲突，**以本章为准**。

### 决策 D1：slider 100 = 200 r/s = 7200°/s（电机轴）

**含义**：
- 用户在串口助手发送的 MoveJ `speed` 字段（slider，1-100）经过主控换算后，对应电机轴 200 r/s（即 7200°/s）
- 换算公式：`motorRps = slider × 2.0f`
- 1-100 之间的 slider 线性映射到 2-200 r/s

**反推依据**：
- 地轨：300mm/s → 60 r/s = 21600°/s（丝杆1605直连，1圈=5mm），但 slider 50 实际只能跑到 ~500mm/s（被固件 velocityLimit 截断），slider 100 ≈ 200 r/s 足够覆盖地轨需求
- J2：90°/2.5s = 36°/s 输出轴 × 50 = 1800°/s 电机轴 → slider 100 对应 7200°/s 完全够用
- 关节输出轴最大可达：7200°/s ÷ 50（reduction） = 144°/s

**修改位置**：
- `dummy_robot.h`：新增常量 `constexpr float JOINT_SPEED_MAX_RAD_PER_S = 7200.0f;`
- `SetJointSpeed()`：内部换算逻辑改为 `jointSpeed = slider × 72.0f`（结果存为电机°/s，最终送电机时按需再 /360 转 r/s）—— 注意 `×72` 是临时过渡，最终调用点按"统一单位 r/s"决策走
- ⚠️ **本决策与文档早期版本"slider × 72 直接 = 电机°/s"叙述有数值重合（都是 ×72），但语义不同**：
  - 旧版语义：×72 后存为"电机°/s"（隐含主控内存是°/s）
  - 新版语义：×72 后继续除以 360 转 r/s（主控内部统一存 r/s，与电机端 0x07 单位一致）

### 决策 D2：统一单位 = r/s（圈/秒）

**含义**：
- 主控内部所有速度字段（`jointSpeed`、`dynamicJointSpeeds`、`railSpeed` 等）**统一存为 r/s**
- 主控下发到电机的 `_vel` 直接是 r/s，与电机端 0x07 接收语义**完全一致**
- 关节 °/s 仅作为**临时变量**出现在"主控内部估算 + 显示"环节（如日志、调试串口）

**接口边界**（主控 vs 电机）：
```
主控 jointSpeed (r/s) ──[× reduction ÷ 360]──> 关节 °/s（仅显示用）
                  ──────────────────────────> 直接送电机 0x07 (r/s)
```

**修改位置**：
- `dummy_robot.cpp`：
  - `MoveJ()` 中 `dynamicJointSpeeds.a[j-1] = fabsf(deltaAngles.a[j-1]) / timeSec` → 改为 `dynamicJointSpeeds.a[j-1] = fabsf(deltaJoints.a[j-1] * motorJ[j]->reduction) / timeSec`（关节° → 电机 r/s）
  - `SetAngleWithVelocityLimit`（`ctrl_step.cpp`）：调用方传入的 `_vel` 直接是 r/s，不再做°/s → r/s 换算（**注意：原函数内部"÷360 × reduction"的换算保留给"上层传入°/s"的情况做兼容**——具体由后续 P1 决定）

### 决策 D4：地轨参与同步抵达逻辑（新决策，覆盖旧版本）

**含义**：
- `MoveJ` 执行时，**地轨（J0/CAN ID=9）参与"同步抵达"计算**，与 J1~J6 一起取 max timeSec
- 短地轨 / 短关节都按"规划的总时间 = 最长轴的实际耗时"运行，**统一减速到同步抵达**

**接口**：
- MoveJ 命令格式仍是 `>Rail,j1,j2,j3,j4,j5,j6,speed`（7 个值，不含夹爪）
- `MoveRail()` 内部重写：地轨也按"按剩余距离反推速度"逻辑跑，与 6 轴取 max

### 决策 D8：启动时机 = 堵转检测重构之后

**含义**：
- 本 MoveJ 重构**不立即实施**，排在 **`重构方案—堵转检测重构需求.md`** 之后
- 堵转重构涉及电机端 0x07，与本重构可能共享电机端改动点（即使本重构承诺零电机端改动，堵转重构仍需动 0x07）
- 实施顺序：先堵转重构 → 再 MoveJ 重构

## 3. 待用户确认的决策（之前对话已说过，本次未重申）

> ⚠️ 以下决策来自之前的对话，AI 不确定本次是否仍然有效。**请用户确认**：是 / 否 / 修改。

### 待确认 TC2：`jointSpeedRatio` 保留

- **之前决策**：`SetJointSpeed` 内部仍按 `jointSpeed = slider × ratio` 计算，`jointSpeedRatio` 常量保留
- **影响**：是否与 D1 的 `× 2.0f` 冲突？需要明确是"保留 ratio 但改默认值"还是"废弃 ratio 直接用 2.0f"
- **请确认**：✅ ratio 保留 / ❌ 废弃 ratio / 🔧 修改

### 待确认 TC3：`COMMAND_CONTINUES_TRAJECTORY` 自动减半

- **之前决策**：当 MoveJ 命令模式为 `COMMAND_CONTINUES_TRAJECTORY`（"连续圆滑轨迹模式"，点位间不减速），`jointSpeed` 自动 × 0.5
- **代码位置**：`dummy_robot.h:137` 定义枚举，`dummy_robot.cpp:602 / 754` 使用
- **影响**：与 D1 联动：slider 100 在 CONTINUES 模式下 = 100 r/s（不是 200 r/s）
- **请确认**：✅ 仍然减半 / ❌ 不再减半 / 🔧 修改

---

## 4. 待决问题（暂不冻结，等堵转重构完成后再讨论）

### P1：`SetAngleWithVelocityLimit` 内部 °/s → r/s 换算公式是否保留？

**问题**：当前代码 (`ctrl_step.cpp:319-324`) 内部做了 `_angle / 360 × reduction` 换算。如果上层（`MoveJ`）已经按 D2 传 r/s 进来，函数内部还要不要再做一次转换？

**候选方案**：
- (a) 保留内部换算：函数语义变为"上层传关节°/s，内部转电机端"，与当前实现一致
- (b) 取消内部换算：函数语义变为"上层传 r/s，内部直接送电机"，与 D2 单位统一
- (c) 加新参数 `_velUnit`：让调用方显式声明单位

**状态**：⏳ 待用户拍板

---

## 5. 文档进度

- 2026-08-22 00:15：AI 助手创建文档骨架，写入背景 + Q1~Q6 待讨论问题
- 2026-08-23 02:00：用户决定**本重构排在堵转检测重构之后**，**Q1~Q6 提问部分全部移除**
- 2026-08-23 03:15：用户指示把已确认决策（D1/D2/D4/D8）写入第 2 章，旧决策全部覆盖
- 2026-08-23 03:20：AI 写入 D1/D2/D4/D8，并设置"待用户确认区"（TC1/TC2/TC3）+ "待决问题区"（P1/P2）
- 2026-08-25 21:15：堵转重构 Bug-11 已修复（`SetStallMode()` 同步 `targetRailPos`），但衍生两个遗留待办登记到本文档 §6
- ⏳ 待用户确认 TC1/TC2/TC3
- ⏳ 待堵转检测重构完成后，决定 P1/P2
- ⏳ 全部决策确认完毕后，AI 助手汇总到「需求清单」章节（参考 `重构方案—参数重构需求.md` 格式）
- ⏳ 用户审阅后启动重构任务规划

- 2026-08-22 00:15：AI 助手创建文档骨架，写入背景 + Q1~Q6 待讨论问题 + 详细方案
- 2026-08-22 17:41~17:50：用户口头确认若干方向性决策（已从本文档移除，详见聊天记录）
- 2026-08-23 02:00：用户决定**本重构排在堵转检测重构之后**，**所有方向性决策暂不冻结**，文档仅保留需求背景
- ⏳ 待堵转检测重构完成后，再恢复 Q1~Q6 决策讨论
- ⏳ 全部决策确认完毕后，AI 助手汇总到「需求清单」章节（参考 `重构方案—参数重构需求.md` 格式）
- ⏳ 用户审阅后启动重构任务规划

---

## 6. 待办登记（来自堵转重构 Bug-11 衍生）

> 本章节登记"在堵转重构中已修复 Bug-11，但衍生出本重构范畴内的待办"。这些待办与 D1/D2/D4 决策相关，**应纳入本重构一并处理**，不阻塞堵转重构完成。

### T-1：`IsMoving()` 加入地轨判定

**来源**：`重构方案—堵转检测重构需求.md` Bug-11（2026-08-25）

**现状问题**：
- `dummy_robot.cpp:560-569` 的 `IsMoving()` 只判定 `motorJ[1..6]`，**漏掉地轨 `motorJ[0]`**
- MoveJ 阻塞循环（`dummy_robot.cpp:718`）在 STALL_DONE 后立即看到 J1~J6 都到位而退出 → 打印 `"ok"`
- **副作用**：主控提前返回 ok，但地轨可能还在回退中（堵转 RETREATING 状态），用户体验割裂
- 即使非堵转场景，地轨比关节晚到位时也会"主控以为到了，但用户看到地轨还在动"

**修复方向**（详细方案待本重构启动后讨论）：
```cpp
bool DummyRobot::IsMoving()
{
    static constexpr float EPSILON_DEG = 1.0f;
    static constexpr float EPSILON_MM  = 1.0f;  // 地轨 1mm 容差
    for (int i = 1; i <= 6; i++)
        if (fabsf(motorJ[i]->angle - motorJ[i]->targetAngle) > EPSILON_DEG)
            return true;
    // 新增地轨判定
    if (fabsf(currentRailPos - targetRailPos) > EPSILON_MM)
        return true;
    return false;
}
```

**前置依赖**：**T-2**（必须先有 `currentRailPos` 实时数据，否则 `IsMoving()` 用过期值判定会失真）

### T-2：`UpdateJointAngles()` 轮询地轨 `motorJ[0]`，更新 `currentRailPos`

**来源**：Bug-11 衍生（2026-08-25）

**现状问题**：
- `dummy_robot.cpp:362-385` 的 `UpdateJointAngles()` 只轮询 `motorJ[1..6]` 发 0x23 查询
- **`motorJ[0]`（地轨，CAN ID=9）从不被查询**
- `CtrlStepMotor::UpdateAngleCallback(float, bool)` 从未被调用 → `motorJ[0]->angle` 永远是 0.0f
- 即便 T-1 把地轨加进 `IsMoving()`，用 `motorJ[0]->angle` 也没意义

**修复方向**（详细方案待本重构启动后讨论）：
- 在 `UpdateJointAngles()` 的 `group = 0/1/2` 轮询中加入 `motorJ[0]->UpdateAngle()`
- 三个 group 都已满 → 改为 4 个 group（每 50ms 轮询 2 个，每 200ms 全轮一遍；CAN 总线压力可接受）
- 实现 `CtrlStepMotor::UpdateAngleCallback()` 对地轨的特殊处理：地轨无减速比（reduction=1），直接用 `position_steps / MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS` 转圈数 × 5mm → 距离 mm
- 或更简单：在 `dummy_robot` 里直接用 `currentRailPos = motorJ[0]->angle * 5.0f`（1 圈 = 5mm 直连丝杆）
- 把 `UpdateJointAnglesCallback()` 的范围从 `i=1..6` 扩展到 `i=0..6`，把 `currentRailPos` 也刷新

**与 D4 决策的关系**：
- D4 要求地轨参与"同步抵达"计算 → 计算公式 `timeSec = max(所有轴距离 / 该轴速度)` 就要看 `currentRailPos`
- 如果 `currentRailPos` 不更新，D4 的实现无法自洽（主控不知道地轨当前位置，规划速度时只能用初始 `currentRailPos=0`）

**优先级**：🔴 **高**（T-1 的前置依赖，且 D4 决策落地的必要条件）

### T-3（可选）：堵转后 `targetRailPos` 被偷偷改成 `currentRailPos` 的语义提示

**来源**：Bug-11 衍生（2026-08-25）

**现状问题**：
- Bug-11 修复让 `SetStallMode()` 把 `targetRailPos = currentRailPos`
- 这意味着用户发 `>251,...` 堵转后，主控 `targetRailPos` 偷偷变成回退后的位置（如 245mm），用户再发 `>200,...` 时 MoveJ 是从 245 出发，不是从 251 出发
- 这是 Bug-11 文档里"场景 B 抛弃原目标"的设计取舍，但**没有任何 UI/串口提示**告诉用户"原 MoveJ 已被部分抛弃"
- 文档《重构方案—堵转检测重构需求.md》§"LOCKED 状态"已说明这是有意为之，但实际运行中用户可能困惑

**可选方案**：
- (a) STALL_DONE 后主控打印 `[STALL] 原 MoveJ 已放弃，当前地轨=245mm，请重新发目标`（明确告知）
- (b) 不打印，靠文档说明
- (c) 串口助手 UI 上红色提示（`串口助手.py` 改 UI）

**优先级**：🟡 **低**（不影响功能，仅影响体验；可推迟到 UI 改进批次统一处理）
