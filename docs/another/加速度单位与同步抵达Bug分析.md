# 加速度单位与同步抵达 Bug 分析

> 本文档是问题诊断报告，**不是修复方案**。请人工确认后再决定是否动手改代码。

## 1. 现象复述（用户原话）

1. **j3 比 j2 提前到，且 j3 运动角度更大**——视觉上不同步
2. **保存的加速度感觉没生效**——运行时明显不响应
3. **串口助手输入 100，返回 7.5**——输入输出不一致
4. **地轨加速度单位是 mm/s²，电机不是 r/s²**——单位混乱
5. **要求**：所有电机（含地轨）加速度统一为**电机输出端 r/s²**

---

## 2. 协议栈 & 代码现状

### 2.1 加速度相关接口（ASCII 协议）

| 命令 | 实现位置 | 当前语义 | 备注 |
|------|---------|---------|------|
| `#ACC_J <node> <v> [&]` | `ascii_protocol.cpp:568` | 直接下发到电机（CAN 0x14），同步写入 `jointAccRuntime[node]` | 单位 = r/s²（电机端） |
| `#ACC_BASE_J <node> <v>` | `ascii_protocol.cpp:539 / :1091` | 写入 `jointAccBases.a[node-1]`，保存到 EEPROM，重新应用 `SetCommandMode` | **百分比的 base**，单位 = r/s² |
| `#ACC_RAIL <v> [&]` | `ascii_protocol.cpp:622 / :1119` | 直接下发到 `motorJ[0]`（地轨），同步写入 `jointAccRuntime[0]` | 注释称 mm/s²，**实际当 r/s² 用** |
| `#GETJACC [<node>]` | `ascii_protocol.cpp:674 / :1344` | 节点查询 → 转发 CAN 0x2C；无参 → 打印 `jointAccRuntime[0..6]` | 单位 = r/s² |
| `#SYNC_ACC` | `ascii_protocol.cpp:711 / :1376` | 触发 7 轴 CAN 0x2C 查询，回包自动更新 `jointAccRuntime[]` | 单位 = r/s² |

### 2.2 串口助手 UI（`串口助手.py:1675` `send_acc_base`）

```python
# 节点 1~6 → #ACC_BASE_J   (写入 base，运行时 = 5% × base = 7.5 r/s²)
# 节点 8   → #ACC_J        (直接下发)
# 节点 9   → #ACC_RAIL     (直接下发 + 缓存)
```

### 2.3 主控内部状态

```cpp
// dummy_robot.h:239
DOF6Kinematic::Joint6D_t jointAccBases = {150, 150, 150, 150, 150, 150};  // 6 关节 base
const float DEFAULT_JOINT_ACCELERATION_LOW  = 5;     // 百分比
const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;   // 百分比

// dummy_robot.h:166 附近
float jointAccRuntime[7];   // 7 轴运行时实际下发值（r/s²）
```

### 2.4 运行时下发逻辑

```cpp
// dummy_robot.cpp:532  SetJointAcceleration(_acc)
void DummyRobot::SetJointAcceleration(float _acc)
{
    if (_acc < 0)        _acc = 0;
    else if (_acc > 100) _acc = 100;                          // 钳制到 [0, 100]
    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration_persist(
            _acc / 100.0f * jointAccBases.a[i - 1], false);  // ⚠️ 百分比 × base
}
```

### 2.5 SetCommandMode 调用

```cpp
// dummy_robot.cpp:761
case COMMAND_TARGET_POINT_SEQUENTIAL:
case COMMAND_TARGET_POINT_INTERRUPTABLE:
    SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);   // 5（百分比）
case COMMAND_CONTINUES_TRAJECTORY:
    SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);   // 5（百分比）
case COMMAND_SERVO_J:
    SetJointAcceleration(100.0f);                           // 100（百分比）
```

### 2.6 MoveJ 速度规划（`dummy_robot.cpp:382`）

```cpp
specs[i].accel = jointAccRuntime[i];    // r/s² ←—— 但来源有 bug
```

### 2.7 电机端（`motor_fw_f103_57/UserApp/protocols/interface_can.cpp`）

| 命令 | 解析 | 回包 |
|------|------|------|
| CAN 0x14 (Set Acceleration) | `tmpF = *(float*)RxData * 51200` → `ratedVelocityAcc = (int32_t)tmpF`（单位 = steps/s²） | — |
| CAN 0x2C (Get Acceleration) | — | `tmpF = ratedVelocityAcc / 51200`（单位 = r/s²） |

**电机端主控→回包单位都是 r/s²，电机内部存 steps/s²，单位一致。** ✓

---

## 3. Bug 详细分析

### Bug #1：位置模式默认加速度只有 7.5 r/s²

**链路**：
1. 系统上电 → `DummyRobot::Init` → `SetCommandMode(DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_SEQUENTIAL)` → `SetJointAcceleration(5.0f)` (因 `DEFAULT_JOINT_ACCELERATION_LOW = 5`)
2. `SetJointAcceleration(5.0f)` 内部 `_acc / 100.0f * jointAccBases[0] = 5/100 * 150 = **7.5 r/s²**`
3. 所有位置模式 MoveJ 用 7.5 r/s² 起步，加速非常慢 → 视觉上"j3 先到"

**结论**：所有位置模式默认加速度被钳到 7.5 r/s²，与 `jointAccBases[]=150` 不匹配，是历史"百分比"语义遗留。

### Bug #2：`#ACC_BASE_J` 的百分比语义与"r/s²"语义矛盾

**链路**：
1. 串口助手选节点 1，填 100，点"应用"
2. 实际发送 `#ACC_BASE_J 1 100`
3. 主控 `jointAccBases[0] = 100`，但 **运行时下发 = 5/100 × 100 = 5 r/s²**
4. 用户"我设了 100 怎么没生效"——因为 5% × 100 = 5

**用户报告 "输入100返回7.5" 的真相**：
- 如果用户填 `100` 给 J1，`jointAccBases[0]=100`，`SetCommandMode` 重新应用 → `5% × 100 = 5 r/s²`
- 如果用户填 `150` 给 J1，`jointAccBases[0]=150`，`SetCommandMode` 重新应用 → `5% × 150 = **7.5 r/s²**` ← 这就是 7.5 的来源
- `#GETJACC 1` → 触发 CAN 0x2C → 电机回 `5/51200` (steps/s²) → 实际只有 5 r/s²，与"150 r/s²"差距巨大

**结论**：`ACC_BASE_J` 的 `base` 概念 + `DEFAULT_JOINT_ACCELERATION_LOW=5%` 这套百分比机制让单位"看起来是 r/s²、实际是 base"，是用户认知错乱的根源。

### Bug #3：`#ACC_RAIL` 单位标注 vs 实际不一致

**链路**：
```cpp
// ascii_protocol.cpp:629
dummy.motorJ[0]->SetAcceleration_persist(acc, persist);   // 直接下发，r/s²
dummy.jointAccRuntime[0] = acc;                           // 写入缓存，r/s²
```
**问题**：
- 命令注释/命名暗示 mm/s²（"ACC_RAIL"）
- 实际把输入值原样当 r/s² 写入 `jointAccRuntime[0]` 并下发到电机
- 串口助手 UI 提示输入"加速度(mm/s²)"也是错的

**结论**：`#ACC_RAIL` 是命名 + UI 提示的"误导"，底层是 r/s²。

---

## 7. 修复记录（v2.6，2026-08-31）

按方案 D 落地：
- **删除**：`jointAccBases[6]` 字段、`DEFAULT_JOINT_ACCELERATION_LOW/HIGH` 常量、`EepromConfig.jointAccBases[6]`、`#ACC_BASE_J`（2 处）、`#ACC_RAIL`（2 处）、`DEFAULT_JOINT_ACCELERATION_LOW` 引起的 5% × 150 = 7.5 r/s² Bug
- **新增**：`DEFAULT_JOINT_ACCELERATION = 150.0f`（r/s²）、`jointAccRuntime[8]` 缓存、`SyncAllMotorAcceleration()` 方法、`#SYNC_ACC` ASCII 命令（USB/UART4/UART5 三处全加）
- **改写**：`SetJointAcceleration(_acc)` 直发 r/s² 到电机（带 < 10 百分比兼容路径），`SetCommandMode` 切位置模式时下发 `DEFAULT_JOINT_ACCELERATION`，`can_protocol.cpp` 三处 0x2C 处理填 `jointAccRuntime[]`
- **UI**：`串口助手.py` 加速设置合成一个块（节点选择 + 输入 + 应用 + 保存 + 同步所有），独立"地轨加速度"块删除，`send_acc_base` → `send_acc` + `save_acc`，标签"mm/s²" → "r/s²"
- **EEPROM**：旧 `jointAccBases` 字段删除后 magic 校验失败 → 自动重置为默认值（rgb 灯效/亮度保留，加速度全部默认 150 r/s²）
- **编译**：`ninja` 0 errors，RAM 40.48% / FLASH 32.64%

未实现（留待 v2.7）：
- §11.8 同步抵达算法 `ComputeSyncSpeeds`（梯形曲线模型）——依赖本次 `jointAccRuntime[]` 缓存，下一步可做
- `SetJointAcceleration(_acc) < 10 兼容百分比` 的语义保留是为了兼容 COMMAND_SERVO_J 历史调用，未来如不再需要可删除


### Bug #4：地轨"加速度"的物理含义与电机不同

- 电机 `SetAcceleration` 内部单位 = **电机输出轴 r/s²**（步进电机的轴端角加速度）
- 地轨 = 丝杆直连，1 圈 = 5mm，所以电机 r/s² 与地轨 mm/s² 是**一一对应**的（不需要换算）
- 但 UI 上写"mm/s²"会让用户**按 mm/s² 输值，按 r/s² 下发**——数量级差 5×（5mm/圈 vs 1mm/圈）就会出 bug

**结论**：地轨加速度**底层就是 r/s²（电机输出端）**，UI 注释错误。

### Bug #5：j3 比 j2 提前到的根因

`MoveJ` 同步抵达算法 `ComputeSyncSpeeds`（`dummy_robot.cpp:382` 起）：
```cpp
specs[i].accel = jointAccRuntime[i];
```

**问题**：
1. **默认加速度 7.5 r/s²**（Bug #1）→ 起始加速度极小，加速段很长
2. **每轴 `jointAccRuntime[i]` 来自不同来源**：
   - i=0（地轨）：`#ACC_RAIL` 写入的"r/s²"
   - i=1~6（关节）：`#ACC_J` 或 `SyncAllMotorAcceleration` 回包（r/s²）
   - 但 `SyncAllMotorAcceleration` 有 500ms 超时，**未回包的轴保持 `jointAccBases` 旧值 = 150 r/s²**
3. 当 `#ACC_BASE_J` 设了 150 但 500ms 内 `#SYNC_ACC` 还没收到回包时，`jointAccRuntime` 用 150；之后切到位置模式（5% × 150 = 7.5）才被压回去
4. **运行中**：`specs[i].accel` 在不同轴可能差异巨大（150 vs 7.5）→ 同步曲线彻底乱

**为什么 j3 比 j2 提前到**：
- 距离相同时，**加速度小**的轴加速慢，但 `ComputeSyncSpeeds` 用 `timeBudget = max(timeActual)` 同步，所以**加速度小的轴被等，加速度大的轴被减速**
- 但 `vMax` 钳制在 `AXIS_MAX_RPS[i]` 不同（待查），如果 j3 `vMax > j2`，j3 会先达到 vMax 然后匀速段更长
- 更直接的原因：j3 的 `jointAccRuntime` 可能等于 150 r/s²（base，没被百分比压），j2 是 7.5 r/s² → j3 加速快、提前到

---

## 4. 问题根因总结

| # | 根因 | 位置 |
|---|------|------|
| 1 | `SetCommandMode` 默认加速度 = `5% × base = 7.5 r/s²` | `dummy_robot.cpp:774/778` |
| 2 | `SetJointAcceleration` 有百分比语义，钳制 [0, 100] | `dummy_robot.cpp:532-538` |
| 3 | `ACC_BASE_J` 写入 base，运行时被百分比压 | `ascii_protocol.cpp:539/1091` |
| 4 | `ACC_RAIL` 命名/UI 暗示 mm/s²，实际 r/s² | `ascii_protocol.cpp:622/1119` + 串口助手 |
| 5 | `jointAccRuntime[]` 上电初值用 `jointAccBases[]`（=150）但运行被压到 7.5 | `dummy_robot.cpp:208-210` + Bug #1 |
| 6 | `#SYNC_ACC` 500ms 超时内的轴用 `jointAccBases` 默认值，与运行值不一致 | `dummy_robot.cpp:166` |

---

## 5. 修复方向（待用户确认）

### 方案 A：完全统一为 r/s²（推荐，符合用户诉求）

1. **删除百分比机制**：
   - 删除 `DEFAULT_JOINT_ACCELERATION_LOW/HIGH`
   - `SetJointAcceleration(_acc)` 改为 `motorJ[i]->SetAcceleration_persist(_acc, false);`（直接下发）
   - `SetCommandMode` 切位置模式时 `SetJointAcceleration(150.0f)`，切 SERVO 模式时 `SetJointAcceleration(300.0f)`

2. **重命名 / 删除 `jointAccBases` 改为 `jointAccDefaults`**（仅作 EEPROM 掉电恢复 + 上电默认 r/s² 值）

3. **重命名 `ACC_RAIL` 命令**：统一用 `#ACC_J 9 <v>`（删除 `#ACC_RAIL`）

4. **统一 `ACC_BASE_J` 语义**：直接更新 `jointAccRuntime[node] + 下发`（不存 base、不做百分比）

5. **串口助手**：`send_acc_base` 改名 `send_acc`，所有节点用 `#ACC_J <node> <v>`，UI 提示"加速度 (r/s²)"

6. **统一 r/s² 单位**：
   - 电机端：r/s²（电机输出轴）
   - 地轨：r/s²（电机输出轴，因为 1 圈 = 5mm，单位也是 r/s²，**不再是 mm/s²**）
   - 显示给用户时仍是 r/s²
   - 如果用户希望"地轨实际线加速度 mm/s²"显示，可加换算 `mm/s² = r/s² × 5`，但**底层协议仍是 r/s²**

### 方案 B：保留 base 概念，但去掉百分比

- 保留 `jointAccBases[]` 作为"目标 r/s²"（用户设的就是直接生效的 r/s²）
- `SetCommandMode` 切位置模式时 `SetJointAcceleration(jointAccBases[i])`（100% 应用）
- 其他同方案 A

### 方案 C（不推荐）：只修 Bug #1 保留百分比

- 改 `DEFAULT_JOINT_ACCELERATION_LOW = 100`（百分比 100% = 满 base）
- 不动其他逻辑
- 风险：用户对"100% 是不是真的 100 r/s²"仍会困惑

---

## 6. 待用户决策

1. 选 **方案 A / B / C**？
2. 删除 `#ACC_RAIL`（统一 `#ACC_J`）？还是保留并改注释？
3. 地轨加速度显示：(a) 仅 r/s²  (b) 同时显示 r/s² 和 mm/s²
4. 是否需要兼容旧 EEPROM 配置（`jointAccBases` 字段）？还是直接用新默认值 150？
5. `COMMAND_SERVO_J` 模式的默认加速度（100% × base = 150？还是另设 300/500？）
