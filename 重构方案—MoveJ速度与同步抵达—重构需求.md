# MoveJ 速度与同步抵达 — 重构需求 v2.4

> 文档目的：把 MoveJ 速度单位 + 同时抵达 + 地轨参与同步抵达 的重构方案固化下来，作为后续编码依据。
> 启动时机：本重构排在 **堵转检测重构** 完成后开始（堵转重构已 ✅ 完成）。

---

## 1. 背景与现状

### 1.1 用户报告的两个实测问题（2026-08-21）

| # | 现象 | 影响 |
|---|------|------|
| A | 串口助手发 MoveJ，speed 1-10 有变化，超过 10 后没变化 | 速度上限被钳死 |
| B | 同时抵达逻辑没生效，小角度关节提前到达 | 视觉上"不同步" |

### 1.2 根因（已诊断）

**问题 A — 速度单位混淆（跨固件链路）**：

```
用户 slider (1~100)
    → 主控 SetJointSpeed: jointSpeed = slider × ratio (默认 ratio=1)
    → MoveJ 计算: dynamicJointSpeeds = |deltaAngle| / timeSec  ← 单位：关节 °/s
    → SetAngleWithVelocityLimit(_angle, _vel) 直接透传 _vel  ← 函数内部只换算角度，速度透传
    → 电机端 0x07: ratedVelocity = _vel × MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS  ← 电机按 r/s 解读
```

主控发关节 °/s，电机端按 r/s 解读。当 slider > 30 时，电机端 `ratedVelocity` 超过默认 30 r/s 上限被钳死。

**问题 B — 短关节提前到达**：

主控用**匀速假设**计算 `timeSec = maxAngle / jointSpeed`，但电机端是**梯形加减速**。小角度关节（如 5°）可能完全没有匀速段，全程在加减速区，实际耗时 << 规划 timeSec，导致提前到达。

### 1.3 关键文件（影响范围）

| 文件 | 涉及 |
|---|---|
| `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp` | `MoveJ`, `SetJointSpeed`, `MoveJoints`, `IsMoving`, `UpdateJointAngles` |
| `firmware/ref_core_f405/Robot/instances/dummy_robot.h` | `DEFAULT_JOINT_SPEED`, `jointSpeedRatio` |
| `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` | `SetAngleWithVelocityLimit`（透传速度接口）|
| 电机固件 4 份 | `interface_can.cpp` 的 0x07 接收（无需改动，确认单位）|

### 1.4 堵转重构遗留待办（来自 Bug-11）

| ID | 待办 | 优先级 |
|---|---|---|
| T-1 | `IsMoving()` 加入地轨判定 | 🔴 高 |
| T-2 | `UpdateJointAngles()` 轮询地轨 `motorJ[0]`，更新 `currentRailPos` | 🔴 高 |

---

## 2. 重构目标

1. **统一速度单位**：主控内部与电机端 0x07 一致 = 电机轴 r/s（圈/秒）
2. **slider → r/s 线性映射**：slider 100 = **地轨 30 r/s、关节 25 r/s**（地轨和关节分开上限，D-Q4 决策）
3. **同时抵达**：用迭代收敛算法，让所有关节（含地轨）实际同步抵达
4. **地轨参与同步抵达**：作为 7 个轴之一统一参与算法
5. **解决堵转重构遗留 T-1/T-2**

---

## 3. 决策记录（已拍板）

### D-Q1 ✅ 拍板：单位体系 = 电机轴 r/s

**理由**：
- 与电机端 0x07 单位语义完全一致，链路零换算
- 地轨也是 r/s（直连丝杆 1605），天然统一 7 个轴
- 速度上限在电机端 `velocityLimit`（默认 30 r/s）直接生效
- "用户难以直觉"靠 slider 标尺解决，不需要用户心算单位

**实施影响**：
- `jointSpeed` 含义从"关节 °/s"改为"电机轴 r/s"
- `dynamicJointSpeeds` 含义同步改为"电机轴 r/s"
- `_vel` 透传时直接是 r/s，无需单位换算

### D-Q2 ✅ 拍板：slider 100 = 地轨 30 r/s、关节 25 r/s（D-Q4 拆分）

**换算公式**：

```cpp
constexpr float SLIDER_TO_RPS        = 0.30f;  // 统一换算：slider × 0.30 = 电机轴 r/s
// slider 100 → 地轨 30 r/s，关节 20 r/s（被上限钳制）
// slider 50  → 地轨 15 r/s，关节 15 r/s
// slider 33  → 地轨 10 r/s，关节 10 r/s
// slider 10  → 地轨 3 r/s，关节 3 r/s
// slider 1   → 地轨 0.3 r/s，关节 0.3 r/s
```

**物理上限覆盖检查**（slider 100 时各轴最大输出轴速度）：

| 轴 | reduction | 电机轴 r/s | 输出轴速度 | 物理上限 | 覆盖 |
|---|---|---|---|---|---|
| 地轨 | 1（直连丝杆 1605）| **30** | 150 mm/s | 200 mm/s | ✅ 75%（保守）|
| J1/J4/J5 | 50 | **20** | 144 °/s | 216 °/s | ✅ 66% |
| J2/J3 | 50 | **20** | 144 °/s | **36 °/s（带载 90°/2.5s）** | ⚠️ 带载时堵转兜底 |
| J6 | 30 | **20** | 240 °/s | 360 °/s | ✅ 66% |

**注意点**：
- 统一 `SLIDER_TO_RPS = 0.3` 简化计算（用户只需记一个公式）
- 地轨：slider × 0.3 = 30 r/s（满速，未超物理上限）
- 关节：slider × 0.3 = 上限 20 r/s（由 `AXIS_MAX_RPS[1~6]` 钳制）
- J2/J3 带载时 slider 20 r/s 可能高于实际能力 → 触发堵转保护（Bug-11 兜底）
- 算法 D-Q3 相应改为 `sliderRps[7]` 数组，各轴独立上限

### D-Q3 ✅ 拍板：迭代收敛算法（方案 A）

**算法核心**：找"最长轴的最快时间"，迭代反推+钳制，直到稳定。

**完整伪代码**：

```cpp
// 输入：deltaAngles[7] (输出轴 °/mm，含地轨), reduction[7] = {1, 50, 50, 50, 50, 50, 30}
//       sliderSpeed = 25 r/s（电机轴，所有轴共用上限）
// 输出：dynamicJointSpeeds[7] (电机轴 r/s), timeBudget (秒)

float ComputeSyncSpeeds(const float* deltaAngles, const float* reduction,
                        float sliderSpeed, float* outSpeeds) {
    // 步骤 1：各轴距离 → 电机轴转数
    float distMotor[7];
    for (int i = 0; i < 7; i++) {
        distMotor[i] = fabsf(deltaAngles[i]) * reduction[i] / 360.0f;
    }
    
    // 步骤 2：初始预算时间 = 最远轴的最快时间
    float timeBudget = 0;
    for (int i = 0; i < 7; i++) {
        if (distMotor[i] < 0.001f) continue;
        float t = distMotor[i] / sliderSpeed;
        if (t > timeBudget) timeBudget = t;
    }
    
    // 步骤 3：迭代收敛（最多 10 轮，一般 2~3 轮收敛）
    for (int iter = 0; iter < 10; iter++) {
        float newTimeBudget = 0;
        for (int i = 0; i < 7; i++) {
            if (distMotor[i] < 0.001f) continue;
            float speedNeeded = distMotor[i] / timeBudget;
            float speedActual = fminf(speedNeeded, sliderSpeed);  // 钳制上限
            float timeActual = distMotor[i] / speedActual;
            if (timeActual > newTimeBudget) newTimeBudget = timeActual;
        }
        if (fabsf(newTimeBudget - timeBudget) < 0.001f) {
            timeBudget = newTimeBudget;
            break;
        }
        timeBudget = newTimeBudget;
    }
    
    // 步骤 4：输出每轴最终速度
    for (int i = 0; i < 7; i++) {
        if (distMotor[i] < 0.001f) {
            outSpeeds[i] = 0;
        } else {
            float speedNeeded = distMotor[i] / timeBudget;
            outSpeeds[i] = fminf(speedNeeded, sliderSpeed);
        }
    }
    return timeBudget;
}
```

**用你的例子跑一遍**（J2 转 100° + 地轨 500mm）：

```
输入：
  J2: distMotor = 100 × 50 / 360 = 13.89 转
  地轨: distMotor = 500 / 5 = 100 转（reduction=1 已隐含在 mm→转 换算）
  其他 5 轴: distMotor = 0
  sliderSpeed = 25 r/s

第 1 轮（初始）：
  time[地轨] = 100/25 = 4s
  time[J2]   = 13.89/25 = 0.56s
  timeBudget = 4s ← 地轨是瓶颈

第 2 轮迭代：
  timeBudget = 4s
  地轨: speedNeeded = 100/4 = 25 → speedActual = 25 → timeActual = 4s
  J2:   speedNeeded = 13.89/4 = 3.47 → speedActual = 3.47 → timeActual = 4s
  newTimeBudget = 4s ← 收敛

输出：
  outSpeeds = {25, 3.47, 0, 0, 0, 0, 0}
  // 地轨 25 r/s（满速），J2 3.47 r/s（被规划同步降速）
  // 总时间 4s，所有轴同步抵达
```

**结论**：J2 被降到 3.47 r/s（远低于 25 上限），地轨满速跑 4s。**瓶颈轴降不下来时，调整其他轴**。

**算法特点**：
- ✅ 物理正确：考虑速度上限钳制
- ✅ 自动边界处理：距离 0 不参与
- ✅ 快速收敛：2~3 轮
- ✅ 不依赖电机端参数：纯主控内部计算
- ⚠️ 未考虑加减速：实际电机是梯形加减速，小距离时实际时间 > 估算（误差很小）

**和原算法的对比**：
```cpp
// 原算法（伪代码）：
timeSec = maxAngle / jointSpeed;  // 最远轴的最快时间
for i:
    dynamicJointSpeeds[i] = |deltaAngle| / timeSec  // 按时间反推
// 问题：dynamicJointSpeeds 可能 > jointSpeed，超出电机能力

// 新算法：
// 1. 先用同样的 timeSec 反推
// 2. 然后 fminf(反推值, sliderSpeed) 钳制
// 3. 然后用钳制后的速度重新算时间
// 4. 迭代直到稳定
```
**新算法 = 原算法 + 钳制 + 迭代**，代码改动很小。

---

## 4. 待讨论的核心问题

### D-Q4 ✅ 拍板：地轨与关节 sliderSpeed 分开设置

**sliderSpeed 上限表**（写入代码常量，明确注释）：

```cpp
// 各轴 sliderSpeed 对应的上限（电机轴 r/s，2026-08-27 修订）
// 地轨：slider 100 → 30 r/s（满物理上限）
// 关节：slider 100 → 20 r/s（保守，低于电机端默认 30 r/s）
static constexpr float AXIS_MAX_RPS[7] = {
    30.0f,   // 地轨
    20.0f,   // J1
    20.0f,   // J2
    20.0f,   // J3
    20.0f,   // J4
    20.0f,   // J5
    20.0f    // J6
};
```
    
**slider 100 时各轴物理上限覆盖检查**：

| 轴 | reduction | sliderSpeed (r/s) | 输出轴 | 物理上限 | 覆盖？ |
|---|---|---|---|---|---|
| 地轨 | 1（直连）| **30** | 150 mm/s | 200 mm/s | ✅ 75%（满速）|
| J1/J4/J5 | 50 | **20** | 144 °/s | 216 °/s | ✅ 66% |
| **J2/J3** | 50 | **20** | 144 °/s | **36 °/s（带载 90°/2.5s）** | ⚠️ slider 高于带载能力（堵转兜底）|
| J6 | 30 | **20** | 240 °/s | 360 °/s | ✅ 66% |

**注意点**：
- 地轨 30 r/s → 150 mm/s（满速，未满 200 mm/s 物理上限，保守策略）
- 关节 20 r/s → 144 °/s（保守，低于电机端默认 30 r/s）
- J2/J3 带载情况下，slider 设的 20 r/s 在带载时电机跑不动 → 触发堵转保护（已由 Bug-11 堵转重构兜底）
- 统一 `SLIDER_TO_RPS = 0.3` 后，用户只需记一个公式

**算法调整**：`ComputeSyncSpeeds(float sliderSpeed)` → `ComputeSyncSpeeds(const float axisMaxRps[7])`
- `axisMaxRps[0]` = 地轨上限（30 r/s）
- `axisMaxRps[1~6]` = 关节上限（20 r/s）

### D-Q5 ✅ 拍板：重命名为 `SetAngleWithMotorRps`（彻底重构）

**新接口签名**：
```cpp
// 旧：SetAngleWithVelocityLimit(_angle, _vel)
// 新：SetAngleWithMotorRps(_angle, _rps)
//
// @param _angle 输出轴角度（°，与 reduction 无关）
// @param _rps   电机轴速度（r/s = 圈/秒，直接对应电机端 0x07）
void CtrlStepMotor::SetAngleWithMotorRps(float _angle, float _rps);
```

**理由**：
- 单元语义改变是"破坏性变更"，改名字强制所有调用方编译失败 → 必须 review
- 命名清晰：`MotorRps` 一看就是电机轴 r/s
- 不维护双接口
- 同步修改 `SetPositionWithVelocityLimit(step, rps)`（参数语义同步）

**影响范围**：
- `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.h` 头文件改名
- `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` 函数实现改名
- 所有调用方（`dummy_robot.cpp` 内 6 个关节 + 地轨）必须同步改名
- 编译失败强制 review → 安全

### D-Q6 ✅ 拍板：保留 `jointSpeedRatio`，移除连续轨迹减半

**TC2**：`jointSpeedRatio` 保留
- 当前 ratio=1，没用但保留
- 后续可能用于"软启动"或"速度微调"接口
- 不增加代码复杂度

**TC3**：`COMMAND_CONTINUES_TRAJECTORY` 减半**移除**
- 历史原因：连续轨迹下电机响应不及时 → 减半避免丢步
- 现在：D-Q3 迭代收敛算法已经处理"短轴同步"，**短轴不再提前到**，不需要减半
- 减半让用户感觉"连续轨迹变慢了" → 体验差
- 移除后，连续轨迹和单点 MoveJ 速度一致，体感统一

**代码改动**：
- `dummy_robot.h`：`jointSpeedRatio` 字段保留
- `dummy_robot.cpp`：`MoveJ` 内 `if (isContinuousTrajectory) effectiveSpeed *= 0.5f;` 整段删除
- `SetJointSpeed` 内 `jointSpeed = slider * jointSpeedRatio;` 保留

### D-Q8 ✅ 拍板：删除 `MoveRail` API，地轨动作统一走 `MoveJ`

**理由**：
- `MoveJ(_j1..._j6, _j7_mm, slider)` 已经把地轨作为第 7 个轴纳入同步抵达算法
- "只动地轨不动关节" = `MoveJ(currentJ1, currentJ2, ..., currentJ6, targetRail, slider)`，`deltaAngles[1~6]=0` 在 `ComputeSyncSpeeds` 中自动跳过（line 154 `if (distMotor[i] < 0.001f) continue;`）
- 旧 `MoveRail` 是冗余快捷方式，没有独特价值
- 删除后 `targetRailPos` 字段写入点唯一（`MoveJ` 内 line 757），避免"两个 API 写同一个字段"的歧义

**影响范围**：
- `dummy_robot.h/cpp`：删除 `MoveRail(_railPos_mm)` 函数声明和定义
- 任何 `MoveRail(...)` 调用方改为 `MoveJ(currentJ1...currentJ6, _railPos_mm, slider)`
- 后续 7DOF 逆解：6DOF 逆解 MoveL 内部 `MoveJ(ik..., currentRailPos, slider)`；7DOF 逆解 MoveL 内部 `MoveJ(ik..., ik[6], slider)`（地轨是 7DOF 结果一部分）

**注意点**：
- 旧 `#MOVE_RAIL` 协议命令（如有）需要同步删除
- 调用方需要写 `currentJoints.a[0]...currentJoints.a[5]` 共 6 个 current 值，比 `MoveRail(_mm)` 繁琐——这是有意识的取舍，换 API 唯一性

---

---

## 5. 待决问题汇总

| ID | 问题 | 状态 |
|---|---|---|
| Q1 | 单位体系：r/s 还是 °/s？ | ✅ 已拍板：电机轴 r/s |
| Q2 | slider 100 = ? r/s？ | ✅ 已拍板：地轨 30 r/s、关节 25 r/s（见 D-Q2/D-Q4）|
| Q3 | 同时抵达算法 B1/B2/B3？ | ✅ 已拍板：迭代收敛算法 |
| Q4 | 地轨参与同步抵达具体算法？ | ✅ 已拍板：分开 sliderSpeed，地轨 30 r/s、关节 25 r/s |
| Q5 | `SetAngleWithVelocityLimit` 接口语义？ | ✅ 已拍板：重命名为 `SetAngleWithMotorRps` |
| Q6 | TC2/TC3 保留？ | ✅ 已拍板：保留 ratio，移除减半 |
| Q7 | 地轨单位如何处理？ | ✅ 已拍板：替换 mm/s 字段为 slider，统一接口语义 |
| Q8 | MoveRail 是否保留？ | ✅ 已拍板：删除 MoveRail，地轨动作统一走 MoveJ（D-Q8） |
| T-1 | `IsMoving()` 加入地轨判定 | 🔴 必须做（编码方案 6.7）|
| T-2 | `UpdateJointAngles()` 轮询地轨 | 🔴 必须做（编码方案 6.8）|

---

## 6. 详细编码方案（v2.1）

### 6.1 新增常量与数据结构（`dummy_robot.h`）

**slider → r/s 换算常量**（D-Q4 决策）：
```cpp
// =====================================================================
// MoveJ 速度重构（D-Q1/D-Q2/D-Q4 已拍板，2026-08-27 修订）
// 单位体系：电机轴 r/s（与电机端 CAN 0x07 ratedVelocity 完全一致）
// =====================================================================

// 统一换算：slider (1~100) × 0.30 = 电机轴 r/s（未钳制前）
// 钳制由 AXIS_MAX_RPS[] 负责（地轨 30 r/s，关节 20 r/s）
static constexpr float SLIDER_TO_RPS = 0.30f;

// 各轴电机轴 r/s 上限（用于钳制 slider 计算结果）
// 地轨：直连丝杆 1605，物理上限 ~40 r/s，设 30 r/s = 150 mm/s（满速）
// 关节：42/35 电机，电机端默认 30 r/s，设 20 r/s（保守）
static constexpr float AXIS_MAX_RPS[7] = {
    30.0f,   // 地轨
    20.0f,   // J1
    20.0f,   // J2
    20.0f,   // J3
    20.0f,   // J4
    20.0f,   // J5
    20.0f    // J6
};

// 电机减速比（用于距离 → 电机转数换算）
// index [0]=地轨 (reduction=1, 直连), [1~5]=J1~J5 (50), [6]=J6 (30)
static constexpr uint8_t MOTOR_REDUCTION[7] = {1, 50, 50, 50, 50, 50, 30};

// 夹爪速度上限（电机轴 r/s，与35关节电机一致）
// 夹爪 CAN ID=8，使用35电机 reduction=16，输出轴速度 = 20/16 ≈ 1.25 r/s
static constexpr float HAND_MAX_RPS = 20.0f;

// 各轴 sliderSpeed 上限（电机轴 r/s）
// 地轨：直连丝杆 1605，物理上限 ~40 r/s，设 30 r/s = 150 mm/s（满速）
// 关节：42/35 电机，电机端默认 30 r/s，设 20 r/s（保守）
static constexpr float AXIS_MAX_RPS[7] = {
    30.0f,   // 地轨
    20.0f,   // J1
    20.0f,   // J2
    20.0f,   // J3
    20.0f,   // J4
    20.0f,   // J5
    20.0f    // J6
};

// 【已废弃，2026-08-27 由 AXIS_MAX_RPS 替换】
// static constexpr float AXIS_SLIDER_RPS[7] = { ... };
```

**新增字段**（替换旧的 `jointSpeed`/`dynamicJointSpeeds`/`railSpeed_mm_s`）：
```cpp
// ===== 替换原 jointSpeed（旧的含义是关节 °/s，错误） =====
// 含义改为：电机轴 r/s（slider 换算后的目标速度上限，所有轴共用）
float jointSpeedRps = 20.0f;  // 默认值 = AXIS_MAX_RPS[1~6] 的关节上限

// ===== 替换原 dynamicJointSpeeds（旧的 6 个关节 °/s） =====
// 含义改为：7 个轴的电机轴 r/s（含地轨）
// 索引 [0]=地轨, [1~6]=关节
struct DynamicJointSpeeds7 {
    float rps[7] = {0};
};
DynamicJointSpeeds7 dynamicJointSpeeds7;

// ===== 替换原 railSpeed_mm_s =====
    // 含义改为：地轨电机轴 r/s（不再用 mm/s）
    // 【v2.4 修订】不再有独立的 SetRailSpeed 接口；地轨速度由 MoveJ/MoveL 的 slider 统一决定，
    // ComputeSyncSpeeds 计算时填入。运行时仅作为临时计算结果，不进 EEPROM。
    // 【D-Q8 修订】lastMoveSpeedRps 字段删除——MoveRail API 已删除，该字段无下游用户；
    //  地轨速度仅由 MoveJ/MoveL 路径产生（ComputeSyncSpeeds → railSpeedRps → SetPositionWithMotorRps）。
    // 硬编码 30 r/s，非 MoveJ 路径（Homing/Resting/EmergencyStop）的兜底速度。
    // 硬编码 30 r/s，非 MoveJ 路径（Homing/Resting/EmergencyStop）的兜底速度。
    // 与 `AXIS_MAX_RPS[0]=30` 无公式关联，
    // 仅作为"从未调过 MoveJ 时，地轨也有一个安全的默认速度"。
    float railSpeedRps = 30.0f;
```

---

### 6.2 `SetJointSpeed` 改动（`dummy_robot.cpp`）

**改动原则**：slider → 关节轴 r/s（关节轴上限 20 r/s，由 AXIS_MAX_RPS[1~6] 钳制）。

**【v2.4 修订】删除 `SetRailSpeed`**：地轨不再保留独立速度通道。地轨速度 = MoveJ/MoveL/MoveJoint 中的 slider 速度上限（`AXIS_MAX_RPS[0]=30`），由 `ComputeSyncSpeeds` 在 MoveJ/MoveL 时计算填入 `railSpeedRps`。**协议层删除 `#SPEED_RAIL`**（两处：`ascii_protocol.cpp` line 618 和 line 1113 整块删），**EEPROM 删除 `railSpeed_mm_s` 字段**，**fibre 协议删除 `set_rail_speed`**。

**与 42/35 关节电机的对齐**：42/35 关节电机没有 `#SPEED_J` 单独通道，速度完全由 `>...,speed` 最后一个值决定；地轨原本的 `#SPEED_RAIL` 是"特殊待遇"，v2.4 起统一。

```cpp
// 原代码（line 402-408）：
// void DummyRobot::SetJointSpeed(float _speed)
// {
//     if (_speed < 0)        _speed = 0;
//     else if (_speed > 100) _speed = 100;
//     jointSpeed = _speed * jointSpeedRatio;
// }

// 新代码：
void DummyRobot::SetJointSpeed(float _slider)
{
    if (_slider < 0)        _slider = 0;
    else if (_slider > 100) _slider = 100;

    // 关节轴上限（D-Q4 修订）：slider 100 = 20 r/s（由 AXIS_MAX_RPS[1~6] 钳制）
    jointSpeedRps = _slider * SLIDER_TO_RPS * jointSpeedRatio;
    // 例：slider=50 → 50 × 0.30 × 1.0 = 15 r/s
    //     slider=67 → 67 × 0.30 = 20 r/s（达到上限）
    //     slider=100 → 100 × 0.30 = 30 r/s → 钳制到 20 r/s

    // 【D-Q8 修订】不再同步 lastMoveSpeedRps——MoveRail API 已删除，该字段无下游用户；
    //  地轨速度完全由 MoveJ/MoveL 的 ComputeSyncSpeeds 计算并填入 railSpeedRps。
}
```

**`SetRailSpeed` 改动**：

【v2.4 修订】整段删除。下面是历史代码片段（已废弃，仅供代码考古）：

```cpp
// 历史代码（v2.3 及之前，dummy_robot.cpp line 208-213）：
// void DummyRobot::SetRailSpeed(float _speed_mm_s)
// {
//     if (_speed_mm_s < 0.5f)        _speed_mm_s = 0.5f;
//     else if (_speed_mm_s > 100.0f) _speed_mm_s = 100.0f;
//     railSpeed_mm_s = _speed_mm_s;
// }
//
// 历史代码（v2.2 试图保留的"语义改造"版，未实施，已废弃）：

// v2.4 决定：上述两个版本都不实施。SetRailSpeed 整个 API 删除。
// 替代方案：地轨速度完全由 MoveJ/MoveL/MoveJoint 中的 slider 决定，
// ComputeSyncSpeeds 计算时填入 railSpeedRps；非 MoveJ 路径直接使用 railSpeedRps 默认值（30 r/s）。
// 【D-Q8 修订】lastMoveSpeedRps 字段删除——MoveRail 已删除，Homing/Resting/EmergencyStop 路径走 railSpeedRps 默认兜底。
```

**说明**：
- 【v2.4 修订】`SetRailSpeed` 整个 API 删除——地轨速度完全由 MoveJ/MoveL 的 slider 决定，无需独立通道
- 【D-Q8 修订】`MoveRail` API 删除——地轨动作统一走 `MoveJ(_j1..._j6, _j7_mm, slider)`
- **协议层彻底删除**：`#SPEED_RAIL` 命令（两处）整块移除，`#MOVE_RAIL` 命令（如有）整块移除，串口助手 / 上位机不再支持查询或设置
- **EEPROM 字段彻底删除**：`railSpeed_mm_s` 字段直接删（**方案 B**，接受结构体向前收缩）
- **运行时填入路径**：`MoveJ(_j1, ..., _j7, _slider)` 在调用 `ComputeSyncSpeeds` 时填入 `railSpeedRps`（v2.4 之前的 6.3 节已包含），`MoveL` 同理
- **Homing/Resting/EmergencyStop 路径**：直接使用 `railSpeedRps`（默认 30 r/s），不再有 `lastMoveSpeedRps` 中间层（见 6.3、6.8 节修订）
- **没有"未初始化"风险**：`railSpeedRps` 默认为 30 r/s（line 368 字段初始值），即使从未调过 MoveJ 也安全
- **与 42/35 电机的一致性**：42/35 电机速度完全由 MoveJ 最后一个参数控制，地轨 v2.4 起同样如此

---

### 6.3 `ComputeSyncSpeeds` 算法 + 集成到 `MoveJ`

**新增独立函数**（`dummy_robot.cpp`，作为静态或成员函数均可）：

```cpp
/**
 * @brief 7 轴同步抵达速度规划（D-Q3 决策，2026-08-26）
 * @param deltaRails[7] 各轴距离（地轨=mm，关节=°）
 * @param sliderCaps[7] 各轴电机轴 r/s 上限（地轨=30，关节=20）
 * @param sliderSpeed   统一基础速度（slider × 0.25 = r/s），用于计算初始 timeBudget
 * @param outSpeeds[7]  输出每轴最终电机轴 r/s
 * @return timeBudget   同步抵达总时间 (s)
 * @note 迭代收敛算法：用 sliderSpeed 算初始 timeBudget，再对每轴钳制到 sliderCaps，重新迭代直到稳定
 */
static float ComputeSyncSpeeds(const float deltaRails[7], const float sliderCaps[7],
                                float sliderSpeed, float outSpeeds[7])
{
    // 步骤 1：各轴距离 → 电机轴转数
    float distMotor[7];
    for (int i = 0; i < 7; i++) {
        if (i == 0) {
            // 地轨：mm → 圈（直连丝杆 1605，5mm/圈，reduction=1）
            distMotor[i] = fabsf(deltaRails[i]) / 5.0f;
        } else {
            // 关节：° → 电机圈（reduction × / 360）
            distMotor[i] = fabsf(deltaRails[i]) * (float)MOTOR_REDUCTION[i] / 360.0f;
        }
    }

    // 步骤 2：初始预算时间 = 最远轴用 sliderSpeed 跑的最快时间
    float timeBudget = 0;
    for (int i = 0; i < 7; i++) {
        if (distMotor[i] < 0.001f) continue;
        float t = distMotor[i] / sliderSpeed;
        if (t > timeBudget) timeBudget = t;
    }

    if (timeBudget < 0.001f) {
        // 所有轴距离都 ≈ 0，不动
        for (int i = 0; i < 7; i++) outSpeeds[i] = 0;
        return 0.0f;
    }

    // 步骤 3：迭代收敛（最多 10 轮，一般 2~3 轮收敛）
    for (int iter = 0; iter < 10; iter++) {
        float newTimeBudget = 0;
        for (int i = 0; i < 7; i++) {
            if (distMotor[i] < 0.001f) continue;
            float speedNeeded = distMotor[i] / timeBudget;
            float speedActual = fminf(speedNeeded, sliderCaps[i]);  // 钳制到各轴 cap
            float timeActual = distMotor[i] / speedActual;
            if (timeActual > newTimeBudget) newTimeBudget = timeActual;
        }
        if (fabsf(newTimeBudget - timeBudget) < 0.001f) {
            timeBudget = newTimeBudget;
            break;
        }
        timeBudget = newTimeBudget;
    }

    // 步骤 4：输出每轴最终速度
    for (int i = 0; i < 7; i++) {
        if (distMotor[i] < 0.001f) {
            outSpeeds[i] = 0;
        } else {
            float speedNeeded = distMotor[i] / timeBudget;
            outSpeeds[i] = fminf(speedNeeded, sliderCaps[i]);
        }
    }
    return timeBudget;
}
```

**集成到 `MoveJ`**（替换原 line 291-301 的同步逻辑）：

```cpp
// 原代码（dummy_robot.cpp line 291-301）：
//   DOF6Kinematic::Joint6D_t deltaAngles = targetJointsTmp - currentJoints;
//   float maxAngle = AbsMaxOf6(deltaAngles, maxIndex);
//   float timeSec  = maxAngle / jointSpeed;
//   for (int j = 1; j <= 6; j++) {
//       dynamicJointSpeeds.a[j - 1] = fabsf(deltaAngles.a[j - 1]) / timeSec;
//       if (dynamicJointSpeeds.a[j - 1] < 0.05f)
//           dynamicJointSpeeds.a[j - 1] = 0.05f;
//   }

// 新代码（D-Q3 迭代收敛算法 + D-Q4 分轴 cap + D-Q6 移除减半）：
{
    DOF6Kinematic::Joint6D_t deltaAngles = targetJointsTmp - currentJoints;
    float deltaRail = _j7_mm - currentRailPos;

    // 构造 7 轴距离向量
    float delta7[7] = {
        deltaRail,                              // 地轨 (mm)
        deltaAngles.a[0], deltaAngles.a[1], deltaAngles.a[2],
        deltaAngles.a[3], deltaAngles.a[4], deltaAngles.a[5]
    };

    // 统一基础速度：slider × SLIDER_TO_RPS = r/s（钳制前）
    float sliderSpeed = _slider * SLIDER_TO_RPS;

    // 7 轴速度上限 cap（D-Q4 修订：地轨 30 r/s、关节 20 r/s）
    // ComputeSyncSpeeds 先用 sliderSpeed 算 timeBudget，再对每轴钳制到 cap，迭代直到稳定
    // 【已改用 AXIS_MAX_RPS[] 替代硬编码数组】
    const float* sliderCaps = AXIS_MAX_RPS;  // 复用 6.1 节的常量数组

    // 调用迭代收敛算法
    float timeSec = ComputeSyncSpeeds(delta7, sliderCaps, sliderSpeed, dynamicJointSpeeds7.rps);

    // targetRailPos 同步写入（MoveJ 参数传入的地轨目标位置）
    targetRailPos = _j7_mm;

    // 旧字段 dynamicJointSpeeds（Joint6D_t）保持同步（向后兼容其他模块）
    for (int j = 1; j <= 6; j++)
        dynamicJointSpeeds.a[j - 1] = dynamicJointSpeeds7.rps[j];

    // 调试输出（可选）
    // printf("[MoveJ] t=%.2fs, speeds: rail=%.2f J1=%.2f J2=%.2f J3=%.2f J4=%.2f J5=%.2f J6=%.2f\r\n",
    //        timeSec, dynamicJointSpeeds7.rps[0],
    //        dynamicJointSpeeds7.rps[1], dynamicJointSpeeds7.rps[2],
    //        dynamicJointSpeeds7.rps[3], dynamicJointSpeeds7.rps[4],
    //        dynamicJointSpeeds7.rps[5], dynamicJointSpeeds7.rps[6]);
}
```

**注意点**：
- `ServoJ`（line 319-357）暂不改，仍用旧的 `dynamicJointSpeeds`——`ServoJ` 是高频伺服，不参与同步抵达逻辑
- `dynamicJointSpeeds`（旧的 Joint6D_t）暂时保留并同步值，后续若 ServoJ 也重构可一并清理

---

### 6.4 `MoveJoints` 集成（`dummy_robot.cpp`）

**`MoveJoints` 改动**（调用方改函数名 D-Q5）：
```cpp
// 原代码（line 166-173）：
// void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
// {
//     for (int j = 1; j <= 6; j++)
//         motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
//                                              dynamicJointSpeeds.a[j - 1]);
// }

// 新代码（D-Q5 重命名 + D-Q1 单位 = r/s）：
void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
{
    for (int j = 1; j <= 6; j++)
        motorJ[j]->SetAngleWithMotorRps(_joints.a[j - 1] - initPose.a[j - 1],
                                        dynamicJointSpeeds.a[j - 1]);
}
```

**`MoveRail` 删除**（D-Q8 决策）：
- `MoveRail(_railPos_mm)` 函数声明和实现整段删除
- 任何调用方改为 `MoveJ(currentJoints.a[0], currentJoints.a[1], ..., currentJoints.a[5], _railPos_mm, railSpeedSlider)`，6 个关节 delta=0 时 `ComputeSyncSpeeds` 自动跳过（`distMotor[i] < 0.001f` 短路）
- 旧协议命令 `#MOVE_RAIL` 同步删除（如果存在）

---

### 6.5 接口重命名：`SetAngleWithVelocityLimit` → `SetAngleWithMotorRps`

**`ctrl_step.hpp` 改动**（line 41、48）：
```cpp
// 原：
// void SetAngleWithVelocityLimit(float _angle, float _vel);
// void SetPositionWithVelocityLimit(float _pos, float _vel);
// 改为：
void SetAngleWithMotorRps(float _angle, float _rps);
void SetPositionWithMotorRps(float _pos, float _rps);
```

**`ctrl_step.cpp` 改动**（line 102-116、296-301）：
```cpp
// 原：
// void CtrlStepMotor::SetPositionWithVelocityLimit(float _pos, float _vel) { ... }
// void CtrlStepMotor::SetAngleWithVelocityLimit(float _angle, float _vel) {
//     _angle = inverseDirection ? -_angle : _angle;
//     float stepMotorCnt = _angle / 360.0f * (float) reduction;
//     SetPositionWithVelocityLimit(stepMotorCnt, _vel);
// }
// 改为：
void CtrlStepMotor::SetPositionWithMotorRps(float _pos, float _rps) {
    uint8_t mode = 0x07;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_pos;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    b = (unsigned char*) &_rps;
    for (int i = 4; i < 8; i++)
        canBuf[i] = *(b + i - 4);

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

void CtrlStepMotor::SetAngleWithMotorRps(float _angle, float _rps) {
    _angle = inverseDirection ? -_angle : _angle;
    float stepMotorCnt = _angle / 360.0f * (float) reduction;
    SetPositionWithMotorRps(stepMotorCnt, _rps);
}
```

**fibre 协议暴露**（`ctrl_step.hpp` line 89-90）：
```cpp
// 原：
// make_protocol_function("set_position_with_time", *this,
//                        &CtrlStepMotor::SetPositionWithVelocityLimit, "pos", "time"),
// 改为：
make_protocol_function("set_position_with_time", *this,
                       &CtrlStepMotor::SetPositionWithMotorRps, "pos", "time"),
```
（外部协议名不变，只是内部实现改名）

**StepHand 影响**（`dummy_robot.h` line 50-54）：
```cpp
// 原 SetAngleWithSpeedLimit 调用了 SetAngleWithVelocityLimit，要同步改：
// 夹爪速度从 70 r/s 改为 20 r/s（与35关节电机 AXIS_MAX_RPS[4~6] 一致）
void SetAngleWithSpeedLimit(float _angle) {
    float target_angle = OpenedAngle + (ClosedAngle - OpenedAngle) * (_angle / 100.0f);
    SetAngleWithMotorRps(target_angle, HAND_MAX_RPS);  // 20 r/s（与35关节电机一致）
}
```

---

### 6.6 TC3 移除：`COMMAND_CONTINUES_TRAJECTORY` 不再减半

**改动位置**：`dummy_robot.cpp` line 602-605：
```cpp
// 原代码：
// case COMMAND_CONTINUES_TRAJECTORY:
//     SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
//     jointSpeedRatio = 0.5f;   // ← D-Q6 移除
//     break;

// 新代码（D-Q6）：
case COMMAND_CONTINUES_TRAJECTORY:
    SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
    // jointSpeedRatio 不再自动减半，由 SetJointSpeed 直接用 slider × SLIDER_TO_RPS
    break;
```

**说明**：`jointSpeedRatio` 字段仍保留，初始值 1.0f，由 `SetCommandMode` 不再自动设置；用户如有特殊需求可通过 fibre 协议修改。

---

### 6.7 T-1：`IsMoving()` 加入地轨判定（堵转重构遗留）

**改动位置**：`dummy_robot.cpp` line 564-573：
```cpp
// 原代码：
// bool DummyRobot::IsMoving() {
//     static constexpr float EPSILON_DEG = 1.0f;
//     for (int i = 1; i <= 6; i++) {
//         if (fabsf(motorJ[i]->angle - motorJ[i]->targetAngle) > EPSILON_DEG)
//             return true;
//     }
//     return false;
// }

// 新代码（T-1 加入地轨）：
bool DummyRobot::IsMoving() {
    static constexpr float EPSILON_DEG  = 1.0f;    // 关节到位容差 (°)
    static constexpr float EPSILON_MM   = 0.5f;    // 地轨到位容差 (mm)

    // 地轨判定：currentRailPos vs targetRailPos（currentRailPos 由 6.8 UpdateJointAnglesCallback 更新）
    if (fabsf(currentRailPos - targetRailPos) > EPSILON_MM)
        return true;

    // 关节判定（不变）
    for (int i = 1; i <= 6; i++) {
        if (fabsf(motorJ[i]->angle - motorJ[i]->targetAngle) > EPSILON_DEG)
            return true;
    }
    return false;
}
```

---

### 6.8 T-2：`UpdateJointAngles` 轮询地轨（堵转重构遗留）

**轮询机制说明（Issue 11 回答）**：
`UpdateJointAngles()` 是主控 5kHz 主循环中调用的异步查询函数。每 5kHz（0.2ms）调用一次时，`UpdateJointAngles()` 只查 2 个轴（分 3 组轮询，3 步一个完整周期 = 0.6ms）。
- **为什么需要轮询**：地轨电机 ID=9，编码器 MT6816 位置通过 CAN 回传给主控。主控必须主动查询才能更新 `motorJ[0]->angle`，进而更新 `currentRailPos`。如果地轨不参与轮询，`currentRailPos` 永远是 0，`IsMoving()` 无法判断地轨是否到位。
- **当前状态**：只轮询 motorJ[1-6]（6 个关节），地轨 motorJ[0] 不在轮询列表，`IsMoving()` 只检查关节漏了地轨（Bug-11 遗留 T-1）。
- **Issue 11 决策**：从 3 组扩为 4 组（每 0.8ms 一个完整周期），关节轮询频率降低 25%。但 0.8ms 仍远小于机械响应时间（~100ms），堵转检测依赖编码器增量（速度异常），频率降低不影响。
- **备选方案**：将地轨插入现有 3 组（如 case 0 时同时查 motorJ[0]），保持 3 步周期但 case 0 多一次 CAN 发送。若总线负载紧张可改用此方案。

**targetRailPos 字段声明（Issue 12）**：`targetRailPos` 已在 `dummy_robot.h` line 103 声明（`float targetRailPos = 0.0f`）。**唯一写入点**：`MoveJ` 调用时同步更新（6.3 节 line 757）。`MoveRail` API 已删除（D-Q8），不再有第二写入点。

```cpp
// 【v2.4 修订】原 SetRailSpeed 已删除（6.2 节）。
// SetJointSpeed 不再同步地轨速度字段——地轨上限由 MoveJ 的 ComputeSyncSpeeds 在调用时计算并写入 railSpeedRps。
void DummyRobot::SetJointSpeed(float _slider) {
    jointSpeedRps = _slider * SLIDER_TO_RPS * jointSpeedRatio;
    // 由 AXIS_MAX_RPS[1~6] 钳制到 20 r/s
}
```
**改动位置**：`dummy_robot.cpp` line 362-385：

```cpp
// 原代码：分 3 组轮询 6 个关节，每组 2 个
// void DummyRobot::UpdateJointAngles() {
//     static uint8_t group = 0;
//     switch (group) {
//         case 0: motorJ[1]->UpdateAngle(); motorJ[2]->UpdateAngle(); break;
//         case 1: motorJ[3]->UpdateAngle(); motorJ[4]->UpdateAngle(); break;
//         case 2: motorJ[5]->UpdateAngle(); motorJ[6]->UpdateAngle(); break;
//     }
//     group = (group + 1) % 3;
// }

// 新代码（T-2：4 组轮询 7 个轴，每组轮询 2 个，地轨加入 case 3）：
void DummyRobot::UpdateJointAngles() {
    static uint8_t group = 0;

    switch (group) {
        case 0:
            motorJ[1]->UpdateAngle();   // J1
            motorJ[2]->UpdateAngle();   // J2
            break;
        case 1:
            motorJ[3]->UpdateAngle();   // J3
            motorJ[4]->UpdateAngle();   // J4
            break;
        case 2:
            motorJ[5]->UpdateAngle();   // J5
            motorJ[6]->UpdateAngle();   // J6
            break;
        case 3:
            motorJ[0]->UpdateAngle();   // 地轨（T-2 新增）
            break;
    }
    group = (group + 1) % 4;
}
```

**配套：`UpdateJointAnglesCallback` 更新地轨位置**（line 390-397）：
```cpp
// 原代码：
// void DummyRobot::UpdateJointAnglesCallback() {
//     for (int i = 1; i <= 6; i++)
//         currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];
// }

// 新代码（T-2 配套：地轨回包解析后更新 currentRailPos）：
void DummyRobot::UpdateJointAnglesCallback() {
    for (int i = 1; i <= 6; i++)
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];

    // 地轨回包：motorJ[0]->angle 实际是"圈"（因为 motorJ[0] 用 SetPositionWithMotorRps 下发 mm→圈）
    // 但 UpdateAngleCallback 内部：angle = pos / reduction * 360，reduction=1 → angle = pos * 360 (单位: °)
    // 需要把 ° 反推回 mm：mm = angle / 360 * 5 (因为 5mm/圈)
    // —— 更简单的办法：直接读 motorJ[0]->angle 的"原始圈"含义，需要给 CtrlStepMotor 加一个新字段
    // 暂用以下方案（基于现有接口）：
    //   motorJ[0]->angle 是"°"（已乘以 360），需要反推回 mm
    //   但地轨电机实际是线性 mm，angle 字段语义被借用了——这里做特殊换算
    currentRailPos = motorJ[0]->angle / 360.0f * 5.0f;   // ° → 圈 → mm
}
```

**注意**：地轨的 `angle` 字段语义需要再确认。`motorJ[0]` 的 reduction=1，`UpdateAngleCallback` 内 `tmp = _pos / reduction * 360` → 当 `_pos` 是"圈"时，`tmp = 圈 × 360 = 度`，所以 `motorJ[0]->angle` 实际是"伪 °"（圈 × 360）。

**motorJ[0] 数据流图（重要）**：
```
主控 SetPositionWithMotorRps(rail_laps, railSpeedRps)  // rail_laps = mm / 5
    ↓ CAN StdId=(9<<7)|0x07，payload=[pos:float, rps:float]
    ↓
电机端 interface_can.cpp 0x07：
    ratedVelocity = rps × 51200  // 51200 = 200×256
    SetPositionSetPoint(pos × 51200)  // pos 是"圈"
    ↓
电机端 5kHz 闭环：梯形规划 → FOC → 驱动
    ↓ 编码器 MT6816 回传位置
motor.controller->GetPosition() → 返回 圈×51200（细分步）
    ↓
主控 CtrlStepMotor::UpdateAngleCallback：
    angle = pos / reduction × 360 = pos × 360 (reduction=1)  // 单位：伪°
    ↓
主控 UpdateJointAnglesCallback：
    currentRailPos = angle / 360 × 5 = pos × 5 = mm  ✅ 正确
```
**结论**：`currentRailPos = motorJ[0]->angle / 360.0f * 5.0f` 公式正确，无需额外字段。

---

### 6.9 EEPROM 配置兼容（D-Q7）

【v2.4 修订】`railSpeed_mm_s` 字段**直接删除**，不接受任何替换方案。

**理由**：
- 地轨速度不再有独立通道，`railSpeed_mm_s` 没有"保存"的意义（v2.4 之前是临时占位）
- `railSpeedRps` 也不写入 EEPROM（运行时由 MoveJ 的 slider 计算），EEPROM 字段本来就该删
- 接受 `EepromConfig` 结构体向前收缩（`jointAccBases[0]` 起点向前移 `sizeof(float)=4` 字节）；用户重新上电后看到的所有配置恢复为默认值（rgb/jointAccBases），无所谓丢失（所有字段对位置 0~26 字节来说都按 magic 整体重读，超出范围则拒绝）

**具体删除清单**：
| 位置 | 原内容 | 改动 |
|---|---|---|
| `dummy_robot.h` line 28 | `float railSpeed_mm_s;` 在 `EepromConfig` 内 | 删除 |
| `dummy_robot.cpp` line 103-104 | `if (config.railSpeed_mm_s >= 0.5f && config.railSpeed_mm_s <= 100.0f) railSpeed_mm_s = ...;` | 删除整块 |
| `dummy_robot.cpp` line 130 | `config.railSpeed_mm_s = railSpeed_mm_s;` | 删除 |
| `dummy_robot.h` line 104 | `float railSpeed_mm_s = 50.0f;` | 删除（成员变量）|
| `dummy_robot.h` line 206 | `void SetRailSpeed(float _speed_mm_s);` 声明 | 删除 |
| `dummy_robot.h` line 244 | `make_protocol_function("set_rail_speed", ...)` fibre 协议 | 删除 |
| `dummy_robot.cpp` line 208-213 | `SetRailSpeed` 实现 | 删除 |
| `dummy_robot.cpp` line 185 | `float speed_laps = railSpeed_mm_s / 5.0f;` | 改为 `float speed_laps = railSpeedRps;` |
| `ascii_protocol.cpp` line 618-636 | `#SPEED_RAIL` 解析分支（第一处）| 整块删除 |
| `ascii_protocol.cpp` line 1113-1131 | `#SPEED_RAIL` 解析分支（第二处）| 整块删除 |

**EEPROM 兼容性结论**：旧固件写入的 EEPROM（含 `railSpeed_mm_s` 字段）在新固件读取时，`jointAccBases[0]` 会读到旧 `railSpeed_mm_s` 的字节。范围检查（1.0 ≤ val ≤ 2000.0）会拒绝，回到默认 200 r/s²；rgb 字段正常。**不需要清除 EEPROM 即可升级**，下次调用 `SaveConfig()` 后 layout 自动对齐。

---

### 6.10 编译/烧录/测试步骤

1. **编译主控固件**：
   ```bash
   cd firmware/ref_core_f405/build
   ninja
   ```
   检查 RAM/Flash 使用率，关注 `dummy_robot.cpp` 的代码增量。

2. **同步改动检查清单**：
   - [ ] `dummy_robot.h` 新增 SLIDER_TO_RPS、MOTOR_REDUCTION、AXIS_MAX_RPS 常量
   - [ ] `dummy_robot.h` 新增 `jointSpeedRps`、`dynamicJointSpeeds7`、`railSpeedRps` 字段
   - [ ] `dummy_robot.h` 删除 `jointSpeed`、`railSpeed_mm_s`、`lastMoveSpeedRps` 成员变量
   - [ ] `dummy_robot.h` EepromConfig 删 `railSpeed_mm_s` 字段（结构体向前收缩）
   - [ ] `dummy_robot.h` 删 `SetRailSpeed()` 声明 + 删 fibre `set_rail_speed`
   - [ ] 【D-Q8】`dummy_robot.h/cpp` 删 `MoveRail()` 整段声明和实现 + 删 fibre `move_rail` 协议
   - [ ] `dummy_robot.cpp` SetJointSpeed 改动（不再同步 `lastMoveSpeedRps`，D-Q8 已删除该字段）
   - [ ] 【v2.4】`dummy_robot.cpp` 删 `SetRailSpeed()` 整段实现
   - [ ] `dummy_robot.cpp` MoveJ 内集成 ComputeSyncSpeeds（同步填入 railSpeedRps）
   - [ ] `dummy_robot.cpp` MoveJoints 调用新接口
   - [ ] `dummy_robot.cpp` IsMoving 加地轨判定
   - [ ] `dummy_robot.cpp` UpdateJointAngles 加地轨轮询
   - [ ] 【v2.4】`ascii_protocol.cpp` 两处 `#SPEED_RAIL` 协议分支整块删除
   - [ ] 【v2.4】`ascii_protocol.cpp` 两处 `else if (s.find("SPEED_RAIL") ...)` 及其内部 if-else 全部删除（line 618-636 和 1113-1131）
   - [ ] 串口助手同步更新：删除 `#SPEED_RAIL` 设置 UI 入口（用户手动做）
   - [ ] `dummy_robot.cpp` UpdateJointAnglesCallback 加地轨回包解析
   - [ ] `dummy_robot.cpp` SetCommandMode 移除 COMMAND_CONTINUES_TRAJECTORY 减半
   - [ ] `ctrl_step.hpp` 接口改名 SetAngleWithMotorRps/SetPositionWithMotorRps
   - [ ] `ctrl_step.cpp` 函数实现改名
   - [ ] `StepHand::SetAngleWithSpeedLimit` 调用方同步改名，夹爪速度改为 `HAND_MAX_RPS`（20 r/s）
   - [ ] `dummy_robot.h` 新增 `HAND_MAX_RPS` 常量（20 r/s）

3. **烧录验证顺序**：
   - 先烧主控固件 → 测试串口发 `>j1,j2,...,j6,j7,speed` 看速度变化
   - 验证：slider 10、30、50、80、100 测出 5 组不同的速度档位
   - 验证：地轨独立动 `>0,0,0,0,0,0,0,100`（D-Q8：MoveRail 已删，地轨动作统一走 MoveJ）
   - 验证：堵转场景（手扳 J2 阻转）→ 应触发红色心跳灯 + 解锁广播

4. **测试用例**：
   | 场景 | 命令 | 期望 |
   |---|---|---|
   | 速度上限 | `>10,10,10,10,10,10,10,100` | 各轴满速，时间 ≈ max_angle / 25 r/s |
   | 速度下限 | `>1,1,1,1,1,1,1,1` | 关节 0.2 r/s、地轨 0.3 r/s |
   | 同步抵达 | `>100,100,100,100,100,100,100,50` | J2 和地轨同步抵达 4s（瓶颈轴地轨）|
   | 地轨独立 | `>0,0,0,0,0,0,0,100` | 地轨单独移动，关节不动 |
   | 堵转 | 堵转场景 | 不再卡死 MoveJ 循环（Bug-11 修复）|

5. **回归测试**：
   - Homing（`>0,0,90,0,0,0,0`）：所有轴归零正常
   - Resting：待机姿态正常
   - 力矩模式：`$100,...` 力矩控制正常（SetJointCurrents 不受影响）
   - servo 模式：`&...` 高频伺服正常（ServoJ 不参与重构）

---

## 8. 57 电机固件重构范围（2026-08-27 新增）

### 8.1 现状确认

经代码核查（2026-08-27）：
- `motor_fw_f103_57` 固件和 `motor_fw_f103_42` 固件代码**完全一致**（interface_can.cpp、motion_planner、motor 类完全相同）
- 57 电机固件使用 `Motor motor`（FOC 类），**没有使用** `RailMotor` 类
- `rail_motor.h` 在 4 个固件目录中（57/42/35/gripper）均无 `.cpp` 实现，也无任何 `#include`，是**死代码**
- 57 电机固件支持 CAN 0x07（Set Position with Velocity-Limit），与 42/35 一致

### 8.2 重构决策（用户拍板：保持电机固件不动）

**结论**：不修改 57 电机固件，只改主控同步规划 + 删除死代码。

| 决策 | 内容 |
|---|---|
| 电机固件 | 不改（已和 42/35 一致） |
| 主控 MoveJ 同步 | 按第 6 节方案修改，主控已支持地轨通过 CAN 0x07 控制 |
| 死代码清理 | 删除 4 个目录下的 `rail_motor.h` |

### 8.3 删除死代码

以下文件均为死代码（`RailMotor` 类定义了但无实现、无任何引用），建议删除：

```
firmware/motor_fw_f103_57/Ctrl/Motor/rail_motor.h
firmware/motor_fw_f103_42/Ctrl/Motor/rail_motor.h
firmware/motor_fw_f103_35/Ctrl/Motor/rail_motor.h
firmware/motor_fw_f103_gripper/Ctrl/Motor/rail_motor.h
```

**删除后验证**：`grep -r "rail_motor" firmware/motor_fw_f103_*` 应无任何匹配。

### 8.4 57 电机 isMoving 判定现状

57 电机（CAN ID=9，即 motorJ[0]）通过主控 `IsMoving()` 判定：
- **当前状态**：`IsMoving()` 只检查 motorJ[1-6]，不检查 motorJ[0]（Bug-11 T-1 遗留）
- **本重构修复后**：`IsMoving()` 将检查 motorJ[0]，使用 `currentRailPos` vs `targetRailPos` 判定
- 电机固件端：`motor.controller->state == Motor::STATE_FINISH` 由 0x23 查询，主控通过 `UpdateAngleCallback` 异步更新，不需要在电机固件端额外实现 isMoving

### 8.5 后续扩展方向（记录，不在本次重构范围）

- 若未来 57 电机需要独立于主控运行（如独立速度控制），可在电机固件中扩展 velocity 模式支持
- 若需要，地轨可改用 `rail_motor.cpp` 实现纯速度闭环，但当前 FOC 类完全满足需求

---

## 7. 文档进度

- 2026-08-22：AI 创建文档骨架
- 2026-08-23：D1/D2/D4/D8 决策写入（已废除，本版本重整）
- 2026-08-25：堵转重构 Bug-11 衍生 T-1/T-2/T-3 登记
- 2026-08-26：v2 整理，按堵转文档格式重写；D-Q1/Q2/Q3 拍板写入；Q4/Q5/Q6 等待用户决策
- 2026-08-26：v2.1 — D-Q4/Q5/Q6 拍板写入
  - D-Q4：地轨 sliderSpeed = 30 r/s（× 0.30），关节 sliderSpeed = 25 r/s（× 0.25）
  - D-Q5：`SetAngleWithVelocityLimit` → `SetAngleWithMotorRps`（彻底重构）
  - D-Q6：保留 `jointSpeedRatio`，移除 `COMMAND_CONTINUES_TRAJECTORY` 减半逻辑
- 2026-08-26：v2.2 — 第 6 节完整编码方案填入
  - 6.1 常量与数据结构（SLIDER_TO_RPS、MOTOR_REDUCTION、AXIS_MAX_RPS）
  - 6.2 SetJointSpeed / SetRailSpeed 改动
  - 6.3 ComputeSyncSpeeds 完整代码 + 集成到 MoveJ
  - 6.4 MoveJoints 调用新接口
  - 6.5 接口重命名 SetAngleWithMotorRps / SetPositionWithMotorRps
  - 6.6 TC3 移除 COMMAND_CONTINUES_TRAJECTORY 减半
  - 6.7 T-1 IsMoving 加入地轨判定
  - 6.8 T-2 UpdateJointAngles 轮询地轨
  - 6.9 EEPROM 配置兼容（D-Q7）
  - 6.10 编译/烧录/测试步骤
- 2026-08-26：v2.2 — D-Q7 拍板（railSpeed_mm_s 字段替换为 railSpeedRps），文档结构定稿
- 2026-08-27：v2.3 — 文档自查修复：
  - Issue 1 重写 D-Q2（统一 slider 100 = 地轨 30 r/s、关节 25 r/s）
  - Issue 2 在 6.8 节加 motorJ[0] 数据流图
  - Issue 3 修正 Q2 状态文字
  - Issue 4 T-1 注释改为"（更新逻辑见 6.8 节）"
  - Issue 5 SetRailSpeed 协议层需同步修改（串口助手）
  - Issue 6 ServoJ 注释补全（不走 ComputeSyncSpeeds）
  - Issue 7 测试用例命令格式修正（>0,0,0,0,0,0,0,100，8 值）
  - Issue 8 StepHand 70 r/s → 20 r/s（HAND_MAX_RPS，与35关节电机统一）
  - Issue 9 EEPROM 警告扩展（所有字段丢失）
  - Issue 10 测试用例 slider 5 → slider 1
  - Issue 11 在 6.8 节加轮询机制完整解释
  - Issue 12 补充 targetRailPos 字段声明和 SetRailSpeed 同步逻辑
  - 新增第 8 节：57 电机固件重构范围（确认无需电机改动 + 死代码清理）
- 2026-08-27：v2.4 — 简化地轨速度通道（用户决策）：
  - **删除独立地轨速度通道**：`SetRailSpeed()` 整个 API 删除、`#SPEED_RAIL` 协议两处删除、`EepromConfig.railSpeed_mm_s` 字段删除、fibre `set_rail_speed` 删除
  - **目标**：地轨速度完全由 MoveJ/MoveL 的 slider 决定（与 42/35 关节电机统一）
  - 地轨速度由 `AXIS_MAX_RPS[0]=30` 决定，MoveJ 中 `ComputeSyncSpeeds` 计算后填入 `railSpeedRps`
  - 6.8 删除方案 1/2 对比，直接给"全部删除"清单（含 10 个具体删除位置）
  - 6.9 兼容结论：无需清 EEPROM，下次 `SaveConfig()` 自动对齐
  - **常量清理**：`SLIDER_TO_RPS_RAIL` 常量删除（地轨速度由 AXIS_MAX_RPS[0] 决定，不再用 slider × 系数公式）
- 2026-08-27：v2.5 — 夹爪速度修正：
  - 夹爪速度 70 r/s → 20 r/s（`HAND_MAX_RPS`），与其他35关节电机统一
  - 新增 `HAND_MAX_RPS` 常量（20 r/s），`StepHand::SetAngleWithSpeedLimit` 调用时使用
  - 夹爪 reduction=16，输出轴速度 = 20/16 ≈ 1.25 r/s

- 2026-08-27：v2.5 — 删除 MoveRail API（用户决策 D-Q8）：
  - **删除 `MoveRail()` 整个 API**——地轨动作统一走 `MoveJ(_j1..._j6, _j7_mm, slider)`
  - **理由**：MoveJ 已经把地轨作为第 7 个轴纳入同步抵达算法；"只动地轨" = `MoveJ(currentJ, targetRail, slider)`，6 个关节 delta=0 在 `ComputeSyncSpeeds` 自动跳过
  - **关联字段清理**：`lastMoveSpeedRps` 字段删除（MoveRail 没了，无下游用户）；`SetJointSpeed` 不再同步更新 `lastMoveSpeedRps`；Homing/Resting/EmergencyStop 路径改用 `railSpeedRps` 默认 30 r/s 兜底
  - **协议层清理**：`#MOVE_RAIL` 命令（如有）整段移除；fibre `move_rail` 命令删除
  - **测试用例更新**：6.10 节"地轨独立发 `MoveRail()`"改为 `>0,0,0,0,0,0,0,100`（MoveJ 入口）
  - **D-Q8 决策** 已写入 4 节

---

## 9. 下一工作预告：7DOF 冗余机械臂逆解

> 本节是 v2.5 之外的**展望性章节**，作为下一阶段工作的接口约定。详细方案将在独立文档 `@7DOF冗余机械臂逆解方案.md` 中展开，本节只列出与 v2.5 重构方案的**耦合点**。

### 9.1 背景与动机

当前 `DummyRobot::MoveL` 调用 6DOF IK（见 `dummy_robot.cpp` line 226），求解 6 个关节角（J1~J6），地轨固定为 `currentRailPos`。这导致：
- **奇异点问题**：6DOF IK 在某些姿态下无解（腕部奇异、肘部奇异）
- **轨迹规划不连续**：6DOF IK 8 组解之间切换时，关节角可能跳变
- **工作空间受限**：末端不能到达某些位姿（被关节构型锁死）

**7DOF 冗余逆解**（以地轨作为第 7 个自由度）可以：
- 7 个输入（地轨 + J1~J6），输出 7 个关节角
- 利用冗余自由度（1 个）做奇异规避、关节限位规避、能量最优
- 7DOF IK 输出 7 个值（包括地轨目标位置），**MoveL 内部直接传给 MoveJ 的 7 个参数**，无需任何手动接线

### 9.2 与 v2.5 重构方案的接口耦合

| 耦合点 | v2.5 当前状态 | 7DOF 接入后的变化 | 影响范围 |
|---|---|---|---|
| `MoveJ` 参数 | `_j1..._j6, _j7_mm, slider` 7 个关节角 + 地轨 + slider | **不变**（签名已经支持）| 无 |
| `MoveL` 调用 MoveJ | 6DOF IK → `MoveJ(ik..., currentRailPos, slider)` | 7DOF IK → `MoveJ(ik..., ik[6], slider)`（地轨是 IK 输出的一部分）| 仅 `MoveL` 函数体 |
| `currentRailPos` 字段 | 异步轮询 `motorJ[0]->angle` 更新 | **不变** | 6.8 节 `UpdateJointAnglesCallback` 保持 |
| `targetRailPos` 字段 | MoveJ 写入 | **不变** | 无 |
| `railSpeedRps` | MoveJ 计算时填入 | **不变**（7DOF IK 不会影响速度算法）| 无 |
| `dynamicJointSpeeds7` | 7 个轴的速度输出数组 | **不变**（签名已经支持 7 个轴）| 无 |

**关键洞察**：v2.5 重构（特别是 D-Q8 删除 MoveRail、统一走 MoveJ）**为 7DOF 接入铺平了道路**——MoveL → MoveJ 的调用链已经支持"地轨是 IK 输出"这种用法，无需重构。

### 9.3 下一工作分阶段建议（仅大纲）

| Phase | 内容 | 预计改动 |
|---|---|---|
| 1 | 7DOF IK 算法选型（解析法 / 数值迭代法 / 学习型），详见 `@7DOF冗余机械臂逆解方案.md` | 新增 `DOF7Kinematic` 类（独立于 `DOF6Kinematic`） |
| 2 | `MoveL` 切换到 7DOF IK，保留 6DOF IK 作为 fallback（奇异点无法收敛时回退） | `MoveL` 函数体加 IK 选路逻辑 |
| 3 | 冗余自由度利用（奇异规避 / 关节限位规避 / 自运动避障） | 在 IK 目标函数加权重项 |
| 4 | 测试与验证：与 6DOF IK 对比可达工作空间、轨迹连续性、关节跳变次数 | 新增测试用例 |

### 9.4 与 v2.5 工作的衔接

**v2.5 完成后必须做的验证**（确保 7DOF 接入前接口稳定）：
- [ ] MoveL → MoveJ 调用链测试（IK 输出 7 个值传给 MoveJ 7 参数）
- [ ] ComputeSyncSpeeds 7 轴算法验证（含地轨 bottleneck 场景）
- [ ] currentRailPos / targetRailPos 字段语义无歧义（IsMoving 判定正确）
- [ ] 旧 6DOF IK 路径"地轨固定 = currentRailPos"的保留行为正常

**v2.5 不要做但需要预留**：
- 不要在 MoveL 里硬编码 IK 类型（留 `IKSolver` 抽象接口）
- 不要在 MoveJ 里区分"地轨来自用户参数 vs 来自 IK 输出"——它就是同一个参数
- 不要在文档里写"6DOF 唯一"——v2.5 完成后 `MoveL` 已经是 IK 输出→MoveJ 通用接口

---

## 10. 引用文档

| 文档 | 内容 |
|---|---|
| `@7DOF冗余机械臂逆解方案.md`（下一工作）| 7DOF 冗余 IK 完整方案 |
| `ISSUES.md` | P0~P3 问题清单 |
| `TODO.md` | 功能路线图（含 7DOF 在 Phase 2 的位置） |
| `PROJECT_CONTEXT.md` | 项目架构基线 |
