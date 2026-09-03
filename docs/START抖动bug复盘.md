# 代码审查方法论：定位 `!START` 抖动 bug 全流程

> 文档版本：v1.0
> 编写日期：2026-09-02
> 背景：定位 `ResetGoalsToCurrentPosition` 中 `goalPosition` 被错误加 `offset` 导致每次 `!START` 后电机突跳的 bug

---

## 一、审查方法总览

```
用户症状 → 定位代码路径 → 追踪数据流 → 找出根本原因 → 验证修复方案 → 编译确认
```

---

## 二、具体步骤

### 第 1 步：从症状出发，缩小搜索范围

**用户描述**：`每次 !START 后都会"动一下"`

这不是随机的报错，而是**规律性重复**的现象，说明问题出在每次 `!START` 都会执行的代码路径中，而不是某次特定的错误操作。

**缩小范围**：从 `!START` 命令的入口开始追踪。

### 第 2 步：追踪主控 → 电机固件的完整调用链

**做法**：
1. 在主控代码中搜 `!START` 的解析入口（`ascii_protocol.cpp`）
2. 找到发 CAN 命令的代码（0x01 Enable）
3. 在电机固件中找 0x01 的处理（`interface_can.cpp`）
4. 找到 `ResetGoalsToCurrentPosition()` 被调用的位置

**关键命令**：

```bash
# 在所有固件中搜索 ResetGoalsToCurrentPosition 的调用点
rg "ResetGoalsToCurrentPosition" firmware/

# 搜索 0x01 Enable 命令处理
rg "case 0x01" firmware/
```

**结果**：`interface_can.cpp` 中 0x01、0x5B（UNLOCKED）等 3 处都调用了 `ResetGoalsToCurrentPosition()`。

### 第 3 步：精读被调函数的实现逻辑

**做法**：读 `ResetGoalsToCurrentPosition()` 的完整实现（~10 行），理解它在做什么。

```cpp
void Motor::Controller::ResetGoalsToCurrentPosition()
{
    SetPositionSetPoint(estPosition);  // ← 可疑点
    goalVelocity = 0;
    goalCurrent = 0;
    softNewCurve = true;
}
```

发现了**第一层**：调用了 `SetPositionSetPoint(estPosition)`。

### 第 4 步：追踪被调函数的下游

**做法**：找到 `SetPositionSetPoint` 的实现。

```cpp
void Motor::Controller::SetPositionSetPoint(int32_t _pos)
{
    goalPosition = _pos + encoderHomeOffset;  // ← 关键行
}
```

**发现根因**：`estPosition + encoderHomeOffset`，而不是单纯的 `estPosition`。当 `encoderHomeOffset ≠ 0`（归零后），目标位置比当前位置多了 offset 步。

### 第 5 步：验证目标偏移是否真的导致了运动

**做法**：追踪 `goalPosition` 被谁使用，以及 `encoderHomeOffset` 的含义。

1. 查 `encoderHomeOffset` 的定义：单圈内编码器位置（mod 后），归零后等于当前电机转过的累计圈数（mod 值），每次归零后都在 0~SUBDIVIDE_STEPS-1 之间。

2. 查 `ResetGoalsToCurrentPosition` 调用后的下一步：

```cpp
// interface_can.cpp:37
motor.controller->ResetGoalsToCurrentPosition();
// 紧跟着：
motor.motionPlanner.positionTracker.NewTask(GetEstPosition(), GetEstVelocity());
```

3. 读 `PositionTracker::NewTask` 和 `CalcSoftGoal`：

```cpp
void MotionPlanner::PositionTracker::NewTask(int32_t real_location, int32_t real_speed)
{
    trackPosition = real_location;   // ← trackPosition = estPosition
    ...
}

void MotionPlanner::PositionTracker::CalcSoftGoal(int32_t _goalPosition)
{
    int32_t deltaPosition = _goalPosition - trackPosition;  // ← = (estPosition+offset) - estPosition = offset
    // deltaPosition != 0 → 电机开始走 offset 步
}
```

**验证了因果链**：
- `ResetGoalsToCurrentPosition` → `SetPositionSetPoint(estPosition)` → `goalPosition = estPosition + offset`
- `NewTask(estPosition)` → `trackPosition = estPosition`
- `CalcSoftGoal(goalPosition)` → `deltaPosition = offset` → 电机走 offset 步

### 第 6 步：提出并验证修复方案

**方案**：绕过 `SetPositionSetPoint`，直接赋值 `goalPosition = estPosition`。

```cpp
// 修复后
goalPosition = estPosition;  // 不再加 offset
goalVelocity = 0;
goalCurrent = 0;
softNewCurve = true;
```

**验证修复正确性**：
- `trackPosition = estPosition`（不变）
- `goalPosition = estPosition`（修复后）
- `deltaPosition = 0` → `CalcSoftGoal` 判断 `deltaPosition == 0` → 电机保持不动 ✓
- `SetPositionSetPoint` 的 `+offset` 语义保留（给主控 MoveJ 等正常命令用），不变 ✓

### 第 7 步：确认改动范围

**做法**：grep 所有包含 `ResetGoalsToCurrentPosition` 的文件，确保 4 个电机固件副本都改了。

**发现**：gripper 固件没有此函数，无需修改。

---

## 三、经验总结

### 3.1 核心方法：追踪数据流

```
用户症状 → 入口函数 → 被调函数 → 数据成员的使用 → 找出矛盾点
```

代码中的 bug 通常不是语法错误，而是**数据在传递过程中被意外修改**。顺着数据的流向追踪，是发现这类 bug 最可靠的方法。

### 3.2 关键技巧

#### ① 先读调用点，再读被调函数

不要先从被调函数入手。先找到**所有调用位置**，理解这个函数在什么场景下被调用，再去看它的实现。这样能快速判断函数的"预期行为"。

#### ② 追踪"下游消费者"

找到 `goalPosition` 被谁用（`CalcSoftGoal`），比找谁设了它更重要。因为**设了值不等于真的产生了效果**，要看下游有没有用以及怎么用。

#### ③ 找紧跟的下游调用

`ResetGoalsToCurrentPosition` 后面紧跟 `NewTask`，这个**调用顺序**是理解 bug 的关键。两者配合才暴露了问题：单独看 `ResetGoalsToCurrentPosition` 或 `NewTask` 都无法发现矛盾。

#### ④ 区分"谁设了什么"和"谁最终用了什么"

`SetPositionSetPoint(estPosition)` 看起来是在"把当前位置设为目标"，但它的实现是 `goalPosition = _pos + offset`。这里的关键是：**理解每个函数的语义承诺 vs 实际行为**。函数的文档/注释说的是"设置当前位置"，但实现偷偷加了 offset。

#### ⑤ 多文件对比找规律

同一个 bug 模式在 4 个电机固件中重复出现。搜索所有副本确认一致修改，是避免遗漏的标准动作。

### 3.3 常见 bug 模式识别清单

| 模式 | 特征 | 排查方法 |
|------|------|----------|
| **偏移累积** | 每次操作后数值多了一点 | 追踪所有 `+` / `-` offset 的位置 |
| **语义不一致** | 函数名暗示的行为与实现不符 | 先读所有调用点，再读实现 |
| **状态机转换错误** | 状态跳到意外的值 | 找 `state =` 赋值，检查所有转换分支 |
| **竞争条件** | 偶发、无规律的错误 | 画两个执行序列，确认没有共享状态 |
| **单位/换算错误** | 计算结果差 N 倍 | 逐行打印中间变量，核对物理单位 |
| **未初始化变量** | 第一次正常，后续异常 | 首次调用 vs 后续调用的差异 |

### 3.4 审查代码时的标准动作

1. **搜函数所有调用点**：`rg "FunctionName" firmware/`
2. **读下游消费者**：`goalPosition` 被谁用？`softNewCurve` 怎么判？
3. **确认修改前后的值**：改前是什么，改后是什么，差值有没有物理意义
4. **跨文件追踪**：`ResetGoalsToCurrentPosition` → `SetPositionSetPoint` → `goalPosition` → `CalcSoftGoal` → 电机运动
5. **检查边界条件**：`offset == 0` 时正常，`offset != 0` 时才暴露问题
6. **grep 所有副本**：电机固件有 4 个版本，改一个必须改全部

---

## 四、调试工具备忘

```bash
# 搜索函数定义和所有调用点
rg "ResetGoalsToCurrentPosition" firmware/

# 搜索某个变量的所有赋值
rg "goalPosition\s*=" firmware/

# 搜索状态转换（常见 bug 模式）
rg "state\s*=\s*[A-Z_]+" firmware/

# 搜索 CAN 命令处理入口
rg "case 0x[0-9A-F]{2}" firmware/

# 跨文件追踪某个变量的产生和消费
rg "encoderHomeOffset" firmware/
```

---

## 五、本次 bug 修复记录

| 项目 | 内容 |
|------|------|
| **Bug 编号** | P1-2（原未入 TODO） |
| **用户症状** | `!START` 后电机突跳 offset 步 |
| **根本原因** | `ResetGoalsToCurrentPosition()` 调用 `SetPositionSetPoint(estPosition)`，而 `SetPositionSetPoint` 自动加 `encoderHomeOffset`，导致目标位置比实际位置多 offset 步 |
| **修复方案** | 直接 `goalPosition = estPosition`，绕过 `SetPositionSetPoint` 的 +offset 逻辑 |
| **改动文件** | `motor_fw_f103_42/Ctrl/Motor/motor.cpp`<br>`motor_fw_f103_35/Ctrl/Motor/motor.cpp`<br>`motor_fw_f103_57/Ctrl/Motor/motor.cpp` |
| **未改动** | `motor_fw_f103_gripper`（无此函数） |
| **验证** | 编译通过，RAM ≤ 40%，Flash ≤ 66% |
