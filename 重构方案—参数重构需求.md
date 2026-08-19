# 关节/地轨加速度 & 电流参数重构 — 需求总结

> 文档目的：把和 AI 助手的所有沟通结论固化下来，作为后续重构任务的依据。
> 文档状态：**需求已完全确认**（2026-08-07 01:21），等待用户启动重构任务规划。
> 下一步：用户审阅本文档 → 通过对话告诉 AI 助手"可以开始规划重构任务"。

> ⚠️ **2026-08-07 01:21 用户决策更新**：
> - `#SPEED_J` 命令**完全不动**（串口助手没按钮、没人用、`B1` flash bug 标记为"已知 bug，本次不修"）
> - 主控 `jointAccBases[6]` 字段**完全不动**（D1~D4/D10/D11/M2/M10/B4/B7 全部取消删除/重构）
> - `#ACC_BASE_J X V` ASCII 命令**保留**（主控 RAM + 主控 EEPROM 存储机制保留）
> - 重构范围**最小化**：只改 M1/M3/M4/M5/M11/M12 + A1~A8 + D5~D8/D12

---

## 1. 背景

### 1.1 当前问题

主控和电机固件在"关节/地轨加速度"和"电机电流限幅"这两个参数上**部分场景双重存储**：

| 参数 | 主控存 | 电机存 | 后果 |
|---|---|---|---|
| 6 轴加速度 | ✅ **保留** `jointAccBases[6]` + 百分比（用户 2026-08-07 01:21 决策） | 电机 `eepromStorage` | 主控基础值 + 电机 EEPROM 不一致（已知，保留） |
| 地轨加速度 | ❌ 删除 `railAcc_mm_s2` | ✅ 电机 9 号 `eepromStorage` | 字段单点存储到电机 |
| 6 轴电流 | 无 | 电机 `eepromStorage` | 主控不知道 |

并且**每次 `MoveRail` 都会重新发一次 `SetAcceleration`**——CAN 总线被不必要命令占用，电机 EEPROM 也有被过度写入的风险。

**`#SPEED_J` 不动的原因**（用户 2026-08-07 01:21）：
- 串口助手**没有 `#SPEED_J` 按钮**（grep 验证：`SPEED_J` 0 处匹配）
- 没有用户在用这条 ASCII 命令
- `B1` `canBuf[4]=1` 硬编码 bug 标记为"已知 bug，本次不修"（理论影响为零）

### 1.2 用户提出的关键改进点（沟通原文摘录）

> "0.5 秒的回查可以写在串口助手里，没必要单独放进主控占用主控资源，0.5 秒后串口助手自动下发查询"

> "地轨的运动也参考 35 和 42 电机，不用每次运动都发加速度参数"

> "加速度严格来说 j1-j6 应该相同，57 因为带着整个机械臂移动应该要小一些"

> "电机 current 任务很忙会在什么情况下很忙？设置加速度和电流时一般都是失能状态电机不运动主控不计算的待机状态"

> "不用降低优先级，直接在串口助手上加一个提示，请在失能状态下设置"

---

## 2. 需求清单

### 2.1 必须删除的内容

| ID | 内容 | 位置 | 删除理由 |
|---|---|---|---|
| ❌ ~~D1~~ | ~~`jointAccBases[6]` 字段~~ | ~~`ref_core_f405/Robot/instances/dummy_robot.h:180`~~ | **取消**（2026-08-07 01:21）——主控基础加速度字段保留 |
| ❌ ~~D2~~ | ~~`jointAccBases` 在 `EepromConfig` 里的字段~~ | ~~`ref_core_f405/Robot/instances/dummy_robot.h:18-30`~~ | **取消**——保留 |
| ❌ ~~D3~~ | ~~`jointAccBases` 在 `LoadConfig` 里的读取~~ | ~~`ref_core_f405/Robot/instances/dummy_robot.cpp:97-101`~~ | **取消**——保留 |
| ❌ ~~D4~~ | ~~`jointAccBases` 在 `SaveConfig` 里的写入~~ | ~~`ref_core_f405/Robot/instances/dummy_robot.cpp:128-131`~~ | **取消**——保留 |
| D5 | `railAcc_mm_s2` 字段 | `ref_core_f405/Robot/instances/dummy_robot.h:106` | 不再用（加速度存电机 EEPROM） |
| D6 | `railAcc_mm_s2` 在 `LoadConfig` 里的读取 | `ref_core_f405/Robot/instances/dummy_robot.cpp:105-106` | 不再用 |
| D7 | `railAcc_mm_s2` 在 `SaveConfig` 里的写入 | `ref_core_f405/Robot/instances/dummy_robot.cpp:133` | 不再用 |
| D8 | `MoveRail` 里的 `SetAcceleration` 调用 | `ref_core_f405/Robot/instances/dummy_robot.cpp:192` | 电机按自己 EEPROM 跑 |
| ~~D9~~ | ~~`EepromConfig::railSpeed_mm_s` 字段~~ | ~~`ref_core_f405/Robot/instances/dummy_robot.h:28`~~ | **取消删除**（Q5.3 选 c）——`#SPEED_RAIL X &` 保留存主控 EEPROM |
| ~~D9b~~ | ~~`railSpeed_mm_s` 在 `LoadConfig`/`SaveConfig` 里的处理~~ | ~~`ref_core_f405/Robot/instances/dummy_robot.cpp:103-104, 132`~~ | **取消删除**（Q5.3 选 c）——保留现状 |
| ❌ ~~D10~~ | ~~`#ACC_BASE_J X V` ASCII 命令（2 处）~~ | ~~`ref_core_f405/UserApp/protocols/ascii_protocol.cpp:469-497, 933-961`~~ | **取消**（2026-08-07 01:21）——保留 `#ACC_BASE_J` 命令 |
| ❌ ~~D11~~ | ~~主控里所有"查表乘百分比"的下发逻辑~~ | ~~`ref_core_f405/Robot/instances/dummy_robot.cpp:430-437`（旧 `SetJointAcceleration`）~~ | **取消**（2026-08-07 01:21）——保留原逻辑 |
| D12 | `SetRailAcc` 函数 | `ref_core_f405/Robot/instances/dummy_robot.cpp:225-230` | **改为**：直接转发 CAN 0x14 给电机 9 号（`canBuf[4]=1`）——不再存 `railAcc_mm_s2` |
| D13 | `SetRailAcc` 在 `EepromConfig` 的存/读逻辑 | 同上 | 已包含在 D5/D6/D7 |
| **D14** | **不在保留的删除清单中**：地轨"同步到达"逻辑 | - | **确认不动**（Q5.4 选 A）——地轨保持固定速度 |

### 2.2 必须修改的内容

> ⚠️ **2026-08-07 01:21 用户决策**：以下项**取消**（保留原状）：
> - ~~M2~~ — `SetJointAcceleration` 改成"展开 6 个独立"  → **撤销**
> - ~~M8~~ — `SetVelocityLimit` 加 `persist` 参数  → **撤销**（`#SPEED_J` 完全不动）
> - ~~M9~~ — `SetNodeID` 加 `persist` 参数  → **撤销**（未经用户决策擅自加入）
> - ~~M10~~ — `DummyRobot::SetJointAcceleration` 硬编码基础值重构  → **撤销**
> - ~~M14~~ — `set_joint_acc` fiber 协议重构  → **撤销**

| ID | 内容 | 位置 | 改法 |
|---|---|---|---|
| M1 | `CtrlStepMotor::SetAcceleration` 函数 | `ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp:163-176` | **函数签名改**：增加 `bool persist` 参数——根据参数决定 `canBuf[4]`。调用方按需传。`#ACC_RAIL X &` → `persist=true` → `canBuf[4]=1`；`#ACC_RAIL X` → `persist=false` → `canBuf[4]=0`。**`MoveRail` 流程不调 `SetAcceleration`**（已删 line 192）——**不会触发 flash 写入**。**`#ACC_J` 必须支持 `&`**（Q5.5 选 A） |
| ~~M2~~ | ~~（已撤销）~~ | - | 见上方撤销说明 |
| M3 | `CtrlStepMotor::SetCurrentLimit` 函数 | `ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` | **函数签名改**：增加 `bool persist` 参数——和 M1 同样处理。`#I_LIMIT_J X I &` → `persist=true`；不带 `&` → `persist=false`。**`#I_LIMIT_J` 必须支持 `&`**（Q5.6 选 A） |
| M4 | `#ACC_RAIL X &` 命令处理 | `ref_core_f405/UserApp/protocols/ascii_protocol.cpp:548-566` (2 处) | **改为**：直接调 `motorJ[0]->SetAcceleration(X, true)`（`canBuf[4]=1`）——**不调 `dummy.SetRailAcc()`**——**不存主控 EEPROM**——不调 `dummy.SaveConfig()`——重构后 `SetRailAcc` 函数**删除** |
| M5 | `#ACC_RAIL X` 不带 `&` 命令处理 | 同上 | ✅ **Q5.1 选 a**——直接调 `motorJ[0]->SetAcceleration(X, false)`（`canBuf[4]=0` 临时设置，不入 EEPROM）——不存主控 EEPROM |
| M6 | `#SPEED_RAIL X &` 命令处理 | `ref_core_f405/UserApp/protocols/ascii_protocol.cpp:529-547` (2 处) | ✅ **Q5.3 选 c**——**保留现状**——调 `dummy.SetRailSpeed(X)` 更新 `railSpeed_mm_s` 字段，调 `dummy.SaveConfig()` 存主控 EEPROM |
| M7 | `#SPEED_RAIL X` 不带 `&` 命令处理 | 同上 | ✅ **Q5.2 确认**——**保持原行为**——调 `dummy.SetRailSpeed(X)` 更新运行时字段 |
| ~~M8~~ | ~~（已撤销）~~ | - | `#SPEED_J` 完全不动，参见 B1 标记 |
| ~~M9~~ | ~~（已撤销）~~ | - | 未经用户决策，撤销 |
| ~~M10~~ | ~~（已撤销）~~ | - | 主控 `jointAccBases` 保留，参见 D1~D4 撤销说明 |
| M11 | `#ACC_J X S` 命令处理 | `ref_core_f405/UserApp/protocols/ascii_protocol.cpp:498-513` | **sscanf 格式改**：`"#ACC_J %lu %f %c"`——读 `&` 后缀。**返回值判断**：`>= 2`（要求至少前两个参数）。**`persist` 标志**：根据 `saveFlag == '&'` 决定 |
| M12 | `#I_LIMIT_J X I` 命令处理 | `ref_core_f405/UserApp/protocols/ascii_protocol.cpp:567-589` | **sscanf 格式改**：`"#I_LIMIT_J %lu %f %c"`——读 `&` 后缀。**返回值判断**：`>= 2`。**`persist` 标志**：根据 `saveFlag == '&'` 决定 |
| M13 | `#ACC_J X S` 不带 `&` 时 sscanf 返回值修复 | `ref_core_f405/UserApp/protocols/ascii_protocol.cpp:502` | **当前 bug**：sscanf `"#ACC_J %lu %f"` 不检查返回值——若用户输入无效，`node` 是未初始化垃圾。**重构后改成 `>= 2`** |
| ~~M14~~ | ~~（已撤销）~~ | - | fiber 协议 `set_joint_acc` 路径保留原状 |

### 2.3 必须新增的内容

| ID | 内容 | 位置 | 用途 |
|---|---|---|---|
| A1 | `CtrlStepMotor::QueryAcceleration()` 函数 | `ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` + `.hpp` | 发送 0x2C 给电机，**不写 EEPROM** |
| A2 | `CtrlStepMotor::QueryCurrentLimit()` 函数 | 同上 | 发送 0x2D 给电机，**不写 EEPROM** |
| A3 | ASCII 命令 `#GETJACC X` | `ref_core_f405/UserApp/protocols/ascii_protocol.cpp` | 解析后调 `motorJ[X]->QueryAcceleration()`，等待电机 0x2C 响应并打印 |
| A4 | ASCII 命令 `#GETI X` | `ref_core_f405/UserApp/protocols/ascii_protocol.cpp` | 解析后调 `motorJ[X]->QueryCurrentLimit()`，等待电机 0x2D 响应并打印 |
| A5 | CAN 协议 0x2C 响应处理 | `ref_core_f405/UserApp/protocols/can_protocol.cpp` | 电机回 0x2C → 主控打印 `[ACC] MOTOR [X] = X.XX`（X.XX 是用户能看懂的"圈/s²"） |
| A6 | CAN 协议 0x2D 响应处理 | 同上 | 电机回 0x2D → 主控打印 `[I_LIMIT] MOTOR [X] = X.XX`（X.XX 是用户能看懂的"A"） |
| A7 | 电机固件 0x2C 处理 | `motor_fw_f103_35/42/57/gripper/UserApp/protocols/interface_can.cpp` × 4 份 | 收到 0x2C → 返回当前 acc 值（按"圈/s²"换算） |
| A8 | 电机固件 0x2D 处理 | 同上 × 4 份 | 收到 0x2D → 返回当前 current limit 值（按"A"换算） |

### 2.3.1 重构前必须修复的预存 Bug（2026-08-07 验证发现）

**用户 2026-08-07 00:32 要求"验证逻辑能不能跑通"——扫描代码后发现了 7 个隐藏 bug**——**必须在有效重构任务前先修复**（注：仅"未撤销"的 B 项需要修）：

> ⚠️ **2026-08-07 01:21 用户决策更新**：
> - ~~B1~~（`#SPEED_J` flash bug）→ **已知 bug，本次不修**（串口助手无按钮，理论影响为零）
> - ~~B4~~（`SetJointAcceleration` 重构后 `jointAccBases` 删除导致阻塞）→ **撤销**（保留 `jointAccBases`，B4 不存在）
> - ~~B7~~（`set_joint_acc` fiber 协议需重构）→ **撤销**（M14 撤销）

| ID | Bug | 严重性 | 修复位置 |
|---|---|---|---|
| ~~B1~~ | ~~`SetVelocityLimit` 当前 `canBuf[4]=1` 硬编码——每次 `#SPEED_J` 都触发 flash 写入~~ | ~~🔴 高~~ | **`已知 bug，本次不修`**（2026-08-07 01:21）—— `#SPEED_J` 在串口助手**无按钮**，grep 验证 `SPEED_J` 0 处匹配——实际影响为零 |
| B2 | `SetCurrentLimit` 当前 `canBuf[4]=1` 硬编码——每次 `#I_LIMIT_J` 都触发 flash 写入 | 🔴 **高** | `ctrl_step.cpp:134-147`（M3 修复） |
| B3 | 串口助手 `#ACC_BASE_J {node}` 缺第二个参数——按"查加速度"按钮触发 6 次无意义 flash 写入 | 🟡 中 | `串口助手.py:482`（UI1 实现时同步修复） |
| ~~B4~~ | ~~`SetJointAcceleration` 重构后 `jointAccBases` 删除——百分比计算失败~~ | ~~🔴 阻塞~~ | **`已撤销`**（2026-08-07 01:21）——保留 `jointAccBases`，bug 不存在 |
| B5 | `#ACC_J` sscanf 不检查返回值——`node` 是未初始化垃圾 | 🟡 中 | `ascii_protocol.cpp:502`（M11 + M13 修复） |
| B6 | `#I_LIMIT_J` sscanf 不检查返回值——`node` 是未初始化垃圾 | 🟡 中 | `ascii_protocol.cpp:571`（M12 修复） |
| ~~B7~~ | ~~`set_joint_acc` fiber 协议主控内部 4 处调用——百分比语义不能丢~~ | ~~🟡 中~~ | **`已撤销`**（2026-08-07 01:21）——M14 撤销，fiber 路径保留原状 |

**B2 是用户关切的"每次发送都存 flash"**——**本次重构统一修复**（加 `persist=false` 默认值），**但用户输入显式 `&` 仍写**——**这是用户希望的语义**。

**B3 是串口助手现存的隐藏 bug**——**重构时同步修复**——**避免用户调试时被坑**。

### 2.4 串口助手侧需求（不是主控/电机任务范围，但要在主控/电机完成后再做）

| ID | 内容 | 触发条件 | 行为 |
|---|---|---|---|
| UI1 | `#ACC_J X V` 命令触发回查 | 串口助手看到 `ok SET MOTOR [X] ACCELERATION [V]` | 0.5s 后自动发 `#GETJACC X`；收到 `[ACC] MOTOR [X] = ...` 才取消定时器；1.0s 后无响应 → "请重试" |
| UI2 | `#I_LIMIT_J X V` 命令触发回查 | 串口助手看到 `ok SET MOTOR [X] CURRENT_LIMIT [V]` | 0.5s 后自动发 `#GETI X`；收到 `[I_LIMIT] MOTOR [X] = ...` 才取消定时器；1.0s 后无响应 → "请重试" |
| UI3 | J1~J6 批量设置按钮 | 用户点击"批量设置加速度"按钮 | 串口助手自己发 6 次 `#ACC_J 1 V`、`#ACC_J 2 V`、...、`#ACC_J 6 V`；每次都触发 UI1 流程 |
| UI4 | 失能状态提示 | UI3 / `fe7970b` 现有入口触发"set"按钮 | 串口助手检查电机状态，若 ENABLE 则弹窗"请先失能再设置加速度/电流"；若 DISABLE 则继续执行 |

### 2.5 协议约定（设计时锁定）

#### 2.5.1 数据单位

| 维度 | 内部存储 | 用户输入 | 0x2C/0x2D 返回 |
|---|---|---|---|
| 加速度 | 步/s²（电机 EEPROM） | 圈/s²（用户能懂） | 圈/s² |
| 电流 | mA（电机 EEPROM） | A（用户能懂） | A |

**换算**：电机内部按细分倍数和比例换算。

#### 2.5.2 主控 echo 格式

主控**不知道**用户预期值是多少，**只 echo 电机返回的原值**，**不带 OK/MISMATCH 判断**：

```
[ACC] MOTOR [3] = 50.00
[I_LIMIT] MOTOR [3] = 1.50
```

成功/失败的比对由**串口助手**完成。

#### 2.5.3 回执格式（设置时）

主控对 `#ACC_J X V` / `#I_LIMIT_J X V` 的回执保持现有的 `ok` 格式：

```
ok SET MOTOR [3] ACCELERATION [50.00]
ok SET MOTOR [3] CURRENT_LIMIT [1.50]
```

#### 2.5.4 CAN StdId

```
StdId = (nodeID << 7) | cmdCode
```

新加的两个命令：

```
0x2C = 0x40 | 0x0C  → 0x2C 查 acc
0x2D = 0x40 | 0x0D  → 0x2D 查 i
```

### 2.6 关于 5kHz 环的判断

**用户的失能设置习惯**保证了：
- `#ACC_J X V` / `#I_LIMIT_J X V` 通常在电机**失能**状态下执行
- 失能状态下电机 5kHz 控制环处于空转，影响最小
- EEPROM 写入阻塞 5~10ms **可以接受**

**结论**：**不优化电机固件**（不把 EEPROM 写入放低优先级任务），只在串口助手 UI 加"请先失能"的提示。

#### 2.7 `canBuf[4]` 的真相——为什么"不带 `&` 必须不写 EEPROM"

**用户原话**（2026-08-07 00:27）：

> "canBuf[4] 到底控制着什么，你能详细解释一下吗，最好是都选 A，比较不可能每次发送都要存，而且存还要清除 flash"

**电机固件实现**（`firmware/motor_fw_f103_42/UserApp/protocols/interface_can.cpp:113-136`）：

```cpp
case 0x12:  // Set Current-Limit and Store to EEPROM
    motor.config.motionParams.ratedCurrent = (int32_t) (*(float*) RxData * 1000);  // ① 更新 RAM
    boardConfig.currentLimit = motor.config.motionParams.ratedCurrent;              // ② 同步 boardConfig
    if (_data[4])                                                                  // ③ canBuf[4] 判断
        boardConfig.configStatus = CONFIG_COMMIT;                                  // ④ 标记 EEPROM 待写
    break;
case 0x14:  // Set Acceleration
    tmpF = *(float*) RxData * (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
    motor.config.motionParams.ratedVelocityAcc = (int32_t) tmpF;
    motor.motionPlanner.velocityTracker.SetVelocityAcc((int32_t) tmpF);
    motor.motionPlanner.positionTracker.SetVelocityAcc((int32_t) tmpF);
    boardConfig.velocityAcc = motor.config.motionParams.ratedVelocityAcc;
    if (_data[4])
        boardConfig.configStatus = CONFIG_COMMIT;
    break;
```

**电机主循环**（`firmware/motor_fw_f103_42/UserApp/main.cpp:97-105`）：

```cpp
for (;;)
{
    encoderCalibrator.TickMainLoop();

    if (boardConfig.configStatus == CONFIG_COMMIT)
    {
        boardConfig.configStatus = CONFIG_OK;
        eeprom.put(0, boardConfig);  // ← 写 EEPROM（擦除扇区 + 写入）！
    }
    ...
}
```

**`canBuf[4]` 语义**：

| `canBuf[4]` 值 | 电机行为 |
|---|---|
| `0` | 只更新 RAM（运行时）—— **不写 EEPROM**—— 电机重启后丢失 |
| `1`（非零）| 更新 RAM + **标记 `CONFIG_COMMIT`** —— **下一次主循环 tick 触发 `eeprom.put`**—— 电机重启后保留 |

**`eeprom.put` 的代价**：

- STM32F103CBT6 内部 flash **擦写寿命 ~10,000 次**
- `eeprom.put` 内部**擦除整个扇区 + 写入**——一次操作**至少几 ms**
- 如果每次 `MoveJ` / `MoveRail` 都发 `canBuf[4]=1`——**每次运动都触发 flash 擦写**——长期使用**会损坏电机 flash**

**重构后的 `&` 规则**（基于上述真相）：

| 命令 | 不带 `&` | 带 `&` |
|---|---|---|
| `#ACC_RAIL X` | `SetAcceleration(X, false)` → `canBuf[4]=0` —— **临时** | `SetAcceleration(X, true)` → `canBuf[4]=1` —— **持久化** |
| `#ACC_J X S` | `SetAcceleration(S, false)` → `canBuf[4]=0` —— **临时** | `SetAcceleration(S, true)` → `canBuf[4]=1` —— **持久化** |
| `#I_LIMIT_J X I` | `SetCurrentLimit(I, false)` → `canBuf[4]=0` —— **临时** | `SetCurrentLimit(I, true)` → `canBuf[4]=1` —— **持久化** |
| `MoveRail` 流程 | —— | —— **不发 SetAcceleration**—— **不触发 flash 写入** |

**结论**：**所有 `&` 后缀都保留"持久化标志"语义**——**保护电机 flash**——**重构彻底解决"每次运动都擦写 flash"的问题**。

---

## 3. 关键设计决策（沟通中已达成）

### 3.0 2026-08-07 01:21 用户最新决策（优先于此节其他小节）

> 用户原话："最大速度和主控的基础加速度留着不动就好了，改完后我来检查一遍"

#### 3.0.1 `#SPEED_J` 完全不动

- **理由**：串口助手**没有 `#SPEED_J` 按钮**（grep 验证：`SPEED_J` 0 处匹配）—— 没有用户使用
- **`B1` flash bug**：标记为"已知 bug，本次不修"—— 实际影响为零
- **保留**：原有 ASCII 命令、原有 `SetVelocityLimit` 函数签名、原有 `canBuf[4]=1` 硬编码
- **`M8` 撤销**：不需要加 `persist` 参数

#### 3.0.2 主控 `jointAccBases[6]` 完全不动

- **保留**：6 个字段在 `dummy_robot.h:180` 默认值 `{150, 100, 200, 200, 200, 200}`
- **保留**：`EepromConfig::jointAccBases` 字段（主控 EEPROM 存储）
- **保留**：`LoadConfig`/`SaveConfig` 中的读取/写入逻辑（D3/D4 不删）
- **保留**：`#ACC_BASE_J X V` ASCII 命令（D10 不删）
- **保留**：`SetJointAcceleration(%)` "查表乘百分比"逻辑（D11 不改）
- **保留**：`set_joint_acc` fiber 协议原状
- **撤销**：`D1`/`D2`/`D3`/`D4`/`D10`/`D11`/`M2`/`M10`/`M14`/`B4`/`B7`

#### 3.0.3 用户决策背后的设计理解

**两个加速度系统独立存在**（已与用户沟通清晰）：

| 系统 | 存储 | 用途 | 改值方式 |
|---|---|---|---|
| **主控 `jointAccBases[6]`** | 主控 RAM + 主控 EEPROM | 内部 4 处运动模式 `SetJointAcceleration(%)` 的基础值 | `#ACC_BASE_J X V` |
| **电机 `ratedVelocityAcc`** | 电机 RAM（5kHz 用）+ 电机 EEPROM | 电机 5kHz 位置环限速斜率 | `#ACC_J X S [&]` 或主控下发 |

**实际运行时**：电机 5kHz 循环用 `ratedVelocityAcc`——谁最后写谁生效——主控下发和用户单独设值会互相覆盖。

**单位**：用户输入 `圈/秒²`（电机输出轴加速度）——电机固件乘 51200 转换为内部步/秒²。

**`jointAccBases` 为什么不一样**（开发者实测）：
- J1（基座）= 150 —— 重力负载大，**加速度小**
- J2（肩部）= 100 —— 重力负载大，**加速度更小**
- J3-6（小臂手腕）= 200 —— 惯量小，**加速度可以更大**

### 3.1 `#SET_JOINT_ACC X` 命令

**这个命令在 ASCII 协议里不存在**——我之前误以为有，**纠正后明确**：

- 主控内部 fiber 协议有 `set_joint_acc`（`SetJointAcceleration` 函数）——**保留原状**（2026-08-07 01:21 决策）
- 串口助手没有"批量设置 6 轴"的 ASCII 命令
- 原 `#ACC_BASE_J X V` 是"批量 6 轴"的 ASCII 命令 ——**保留**（2026-08-07 01:21 决策）
- 不再讨论"删除 `#ACC_BASE_J`"或"串口助手加批量 UI"——**维持现状**

### 3.2 "伪命令" 这个说法

我之前用了"伪命令"这个词，**已被纠正**——`#GETJACC X` / `#GETI X` 就是普通的 ASCII 命令，**主控 ASCII 解析层加新命令即可**——和 `#GETJPOS` / `#GETLPOS` 一样的实现路径。

### 3.3 地轨的加速度

`#ACC_RAIL X` 是 ASCII 命令，**保留**：
- 用户输入 → 主控直接发给电机 9 号（`canBuf[4]=1` 写 EEPROM）
- 主控**不再存** `railAcc_mm_s2`
- 电机 9 号按自己 EEPROM 跑
- `MoveRail` **不再发** `SetAcceleration`

**注意**（来自 Q5.1）：
- `#ACC_RAIL X &`（带 `&`）→ `canBuf[4]=1` 写 EEPROM
- `#ACC_RAIL X`（不带 `&`）→ 待 Q5.1 确认

### 3.3.1 地轨的速度（来自 Q1 + Q4 + Q5.3）

> 用户原话："速度不用管，每次发送 movej 和 movel 最后一个值不就是速度吗"
> 用户原话："地轨的速度每次重发"
> 用户原话（2026-08-07 00:06）："确认速度存主控"

**最终决定（Q5.3 选 c）**：

| 参数 | 主控运行时 RAM | 主控 EEPROM | 电机 EEPROM | `MoveRail` 时 |
|---|---|---|---|---|
| 6 轴速度 `jointSpeed` | ✅ 字段保留 | ❌ 不存 | ❌ 不存 | 0x07 重发同步速度 |
| 6 轴同步速度 `dynamicJointSpeeds` | ✅ 字段保留 | ❌ 不存 | ❌ 不存 | 0x07 重发 |
| 地轨速度 `railSpeed_mm_s` | ✅ 字段保留 | ✅ **保留存**（Q5.3 选 c） | ❌ 不存 | 0x07 重发固定速度 |
| 地轨加速度 `railAcc_mm_s2` | ❌ 字段**删除** | ❌ 删除 | ✅ 存（CAN 0x14） | 不发 |

**明确**：
- **6 轴速度**（`MoveJ`/`MoveL` 最后一个值）：**完全不动**——本来就在每次 MoveJ/MoveL 里通过 `SetAngleWithVelocityLimit` (CAN 0x07) 发到电机——电机不存 EEPROM——下次又发——**已是"每次重发"模式**
- **地轨速度**（`railSpeed_mm_s`）：**字段保留 + EEPROM 保留存**——`#SPEED_RAIL X &` **保留现状**——每次 `MoveRail` 通过 CAN 0x07 重发固定速度到电机——**不参与 6 轴"同步到达"**（Q5.4 选 A）
- **地轨加速度**：完全按 Q4 决定——`#ACC_RAIL X &` 直接转发 CAN 0x14 给电机 9 号（`canBuf[4]=1` 写 EEPROM）——`MoveRail` 不再发

### 3.4 加速度大小约定

> "加速度严格来说 j1-j6 应该相同，57 因为带着整个机械臂移动应该要小一些"

**结合 3.0.3 节，用户最新决策（2026-08-07 01:21）**：

- **`jointAccBases` 不同关节基础值不一样** ——保留（重力负载决定）
  - J1=150、J2=100、J3-6=200 —— **开发者实测** —— **保护机械结构**
- **如果用户希望 "J1~J6 加速度相同"** —— 可以通过 `#ACC_BASE_J X V` 把所有关节基础值改成同一个值（**保留 ASCII 入口** —— 用户没动之前没人用 —— 但代码保留）
- **57（地轨）加速度** 独立设置为较小值 —— 通过 `#ACC_RAIL X [&]` 直接对电机 9 号下发

### 3.5 0.5s 回查的责任划分

| 责任 | 在哪里 |
|---|---|
| 触发回查 | **串口助手**（不是主控） |
| 等 0.5s 发查询 | **串口助手** |
| 收到查询后转发给电机 | **主控** |
| 电机回 0x2C/0x2D 后 echo | **主控** |
| 比对预期值 vs 实际值 | **串口助手** |
| 显示 OK/MISMATCH | **串口助手** |

**主控完全不需要后台定时器**——这是个明确的架构约束。

---

## 4. 不动的部分（明确划界）

### 4.1 主控不动的部分

- `LoadConfig` / `SaveConfig` 里 RGB 部分
- `LoadConfig` / `SaveConfig` 里 `railSpeed_mm_s` 部分（**Q5.3 选 c 保留**）
- `LoadConfig` / `SaveConfig` 里 `jointAccBases` 部分（**2026-08-07 01:21 保留**——主控基础加速度字段保留）
- `SetEnable` / `MoveJ` / `MoveL` / `ApplyPositionAsHome`
- `SetJointSpeed` 函数（**不动**——6 轴速度机制保留）
- `SetRailSpeed` 函数（**不动**——Q5.3 选 c 保留 `#SPEED_RAIL X &` 存主控 EEPROM）
- `SetJointAcceleration` 函数**完全保留**（2026-08-07 01:21）——函数签名/接口/实现都不动
  - 内部 fiber 协议 `set_joint_acc` 在用
  - "查表乘百分比"的实现**保留**——百分比 × `jointAccBases[i]` → 实际加速度
- **`jointAccBases[6]` 字段**（2026-08-07 01:21 新增保留）—— 主控 RAM + EEPROM 存储机制**完全保留**
- **`#ACC_BASE_J X V` ASCII 命令**（2026-08-07 01:21 新增保留）—— 主控基础加速度调整入口保留
- **`SetVelocityLimit` 函数**（`#SPEED_J` 路径）（2026-08-07 01:21 新增保留）—— **不动**—— 即使 `canBuf[4]=1` 每次写 flash 也保留
- **`MoveRail` 函数**——只删 `SetAcceleration` 调用（D8）——速度机制不变
- `MoveRailRelative` 函数**保留**——使用 `railSpeed_mm_s` 固定速度
- `MoveJ` 里的"6 轴同步到达"逻辑（line 308-318）——**保留**
- `dynamicJointSpeeds` 字段——**保留**
- **不新增** `dynamicRailSpeed` 字段（**Q5.4 选 A 不实现同步到达**）
- `DummyRobot::step()` 主循环
- FreeRTOS 任务结构

### 4.2 电机固件不动的部分

- 除 0x2C/0x2D 处理外的所有内容
- 5kHz 控制环、FOC、MT6816 编码器读取
- EEPROM 写入时机（**不优化到低优先级任务**）
- CAN 接收机制

### 4.3 串口助手不动的部分

- `fe7970b` 现有的 0.5s/1.0s 兜底逻辑**保留**
- UI 整体布局
- 多线程架构

---

## 5. 开放问题

> 2026-08-07 00:06 用户最终确认 Q5.1~Q5.4；扫描代码后又发现两个隐含问题（Q5.5/Q5.6）。

| # | 问题 | 状态 |
|---|---|---|
| Q5.1 | `#ACC_RAIL X` 不带 `&` 怎么办？ | ✅ 选 a（用户已确认） |
| Q5.2 | `#SPEED_RAIL X` 不带 `&` 怎么办？ | ✅ 保持原行为（用户已确认） |
| Q5.3 | `#SPEED_RAIL X &` 怎么办？ | ✅ 选 c：保留存主控 EEPROM（用户已确认） |
| Q5.4 | 地轨要不要也实现"同步到达"？ | ✅ 选 A：不实现（用户已确认） |
| Q5.5 | `#ACC_J X S` 要不要支持 `&` 后缀？ | ✅ 选 A：支持（保护 flash） |
| Q5.6 | `#I_LIMIT_J X I` 要不要支持 `&` 后缀？ | ✅ 选 A：支持（保护 flash） |

### Q1: `railSpeed_mm_s` 字段是否一并删除？

**用户回答**：

> "速度不用管，每次发送 movej 和 movel 最后一个值不就是速度吗"

**澄清后明确**：
- 用户指的"速度"是 **6 轴速度**（`MoveJ`/`MoveL` 的最后一个值）——**用户说"不用管"是指 6 轴速度不需要重构**
- **6 轴速度本来就在每次 MoveJ/MoveL 里通过 CAN 0x07 发到电机**——电机不存 EEPROM——下次 MoveJ 又发新速度——**已经是"每次重发"模式——不需要改任何东西**
- **地轨速度**（`railSpeed_mm_s`）和 6 轴速度是**独立概念**——用户没说删

**结论（Q1 + Q5.3）**：
- ✅ **6 轴速度完全不动**——本来就通过 CAN 0x07 每次 MoveJ/MoveL 重发——电机不存 EEPROM
- ✅ **地轨速度 `railSpeed_mm_s` 字段保留**（`MoveRail` 时需要计算 `speed_laps`）
- ✅ **地轨速度从主控 EEPROM 保留存**（Q5.3 选 c）——`#SPEED_RAIL X &` 保留存主控 EEPROM 功能
- ✅ **`MoveRail` 流程只读不存**——不会触发 `SaveConfig()`——速度由 `railSpeed_mm_s` 运行时字段提供

### Q2: 0x2C/0x2D 响应主控怎么等？

**用户回答**：

> "同步等 100ms，反正设置的时候是失能状态一般不会有其他的 can 消息"

**确认**：✅ **A 同步等 100ms**——ASCII 处理线程阻塞 100ms 内等电机响应。

### Q3: 是否需要"一键查看全部加速度"UI？

**用户回答**：

> "不用加这个功能，用户自己点自然就有一个延迟的时间等主控同步等"

**确认**：✅ **不需要**——单轴 `#GETJACC X` 足够，6 个定时器并行不需要 3 秒。

### Q4: 地轨速度/加速度是配置一次长期生效，还是每次运动重发？

**用户回答**：

> "地轨的速度每次重发，加速度存在地轨电机控制板中"

**明确**：

| 参数 | 工作机制 |
|---|---|
| **地轨速度** | 每次 `MoveRail` 通过 CAN 0x07 重发到电机——电机不存——下次 `MoveRail` 又发 |
| **地轨加速度** | `#ACC_RAIL X &` 改成"主控直接转发 CAN 0x14 给电机 9 号，`canBuf[4]=1` 写 EEPROM"——`MoveRail` 不再发 |

**结论（Q4 + Q5.1）**：
- ✅ `railAcc_mm_s2` 字段删除（主控运行时不需要——电机按 EEPROM 跑）
- ✅ `MoveRail` 不再发 `SetAcceleration`
- ✅ `#ACC_RAIL X &` → 主控转发到电机 9 号（`canBuf[4]=1` 写电机 EEPROM）
- ✅ `#ACC_RAIL X`（不带 `&`）→ 主控转发到电机 9 号（`canBuf[4]=0` 临时设置，不写 EEPROM）——Q5.1 选 a

### Q5: UI3 失能检查是阻塞还是警告？

**用户回答**：

> "阻塞吧，防止出现意外"

**确认**：✅ **阻塞 + 弹窗提示**——按用户明确指示。

**弹窗内容**：

```
┌─────────────────────────────────────┐
│ ⚠ 电机处于使能状态                     │
│                                      │
│ 设置加速度/电流需要先失能电机            │
│                                      │
│ 请执行 !DISABLE 后重试                │
│                                      │
│              [ 确定 ]                │
└─────────────────────────────────────┘
```

---

### Q5.1 / Q5.2 / Q5.3 — 用户回答后浮现的隐含问题

#### Q5.1：`#ACC_RAIL X` 不带 `&` 怎么办？

**用户回答**（2026-08-07 00:19）："确认，选 a"

**结论**：✅ **选 a**——`#ACC_RAIL X` 不带 `&` → 直接调 `motorJ[0]->SetAcceleration(X, false)` + `canBuf[4]=0`（临时设置）——**不写电机 EEPROM**——重启电机丢。

**带 `&` 和不带 `&` 的统一语义**（和 Q5.3 选 c 一致）：
- 不带 `&` = 临时设置（不持久化）
- 带 `&` = 持久化（写 EEPROM）

#### Q5.2：`#SPEED_RAIL X` 不带 `&` 怎么办？

**现状**：`#SPEED_RAIL X` → `dummy.SetRailSpeed(X)` → 主控只更新 `railSpeed_mm_s` 字段（运行时 RAM）。

**重构后**：`railSpeed_mm_s` 字段**保留**（`MoveRail` 需要用）——所以 `#SPEED_RAIL X` 不带 `&` **保持原行为**——只更新运行时 RAM。

**确认**：✅ **保持原行为**——`#SPEED_RAIL X` 不带 `&` 仍然有效。

#### Q5.3：`#SPEED_RAIL X &` 这个"存主控 EEPROM"功能怎么办？

**现状**：`#SPEED_RAIL X &` → 主控存 EEPROM（`dummy.SaveConfig()`）——主控重启后保留。

**用户回答**（2026-08-07 00:06）：

> "确认速度存主控，选择 A，不需要地轨加入同步到达逻辑"

**结论**：✅ **选 c**——`#SPEED_RAIL X &` **保留存主控 EEPROM**——保持现状——不影响重构原则。

**理由**：
- 35/42 电机也是"电机 EEPROM 存 `velocityLimit` + 每次运动重发"的混合模式
- 地轨速度经常变（不同任务速度不同）——存主控 EEPROM 也合理——重启后保留常用速度
- 电机不存，每次 `MoveRail` 重发——CAN 总线压力可控
- **没有任何不妥**——符合现有 35/42 工作机制

#### Q5.4：地轨要不要也实现"同步到达"？

**用户回答**（2026-08-07 00:06）：

> "地轨的速度比较慢，不合适"

**结论**：✅ **选 A 不实现**——地轨保持固定速度 `railSpeed_mm_s`——**不参与 6 轴"同步到达"逻辑**。

**理由**：
- 地轨物理速度远低于 6 轴（mm/s vs 度/s）——参与同步到达会拖慢整个机械臂
- 地轨和 6 轴是**独立运动学链**——地轨单独控制更合理
- 6 轴"同步到达"逻辑**不动**——只动加速度相关代码

**明确不动的内容**：
- `MoveJ` 里的"同步到达"逻辑（line 308-318）——保留
- `dynamicJointSpeeds` 字段——保留
- `MoveRail` 用固定 `railSpeed_mm_s`——保留
- `MoveRailRelative` 用固定 `railSpeed_mm_s`——保留
- **不新增** `dynamicRailSpeed` 字段

---

### Q5.5：`#ACC_J X S` 要不要支持 `&` 后缀？

**用户回答**（2026-08-07 00:27）："比较不可能每次发送都要存，而且存还要清除 flash"

**结论**：✅ **选 A**——**支持 `&` 后缀**——理由：**保护电机 flash**——STM32F103 内部 flash 寿命 ~10,000 次擦写——每次 `canBuf[4]=1` 都会触发 `eeprom.put`（扇区擦除 + 写入）——如果每次 `MoveJ` 重发都写 flash——长期使用会损坏电机。

**语义**：
- `#ACC_J X S`（不带 `&`）→ 临时设置（`canBuf[4]=0`）—— 适用于"运动期间临时覆盖"
- `#ACC_J X S &`（带 `&`）→ 持久化（`canBuf[4]=1`）—— 适用于"长期保存"

**和 `#ACC_RAIL` 规则统一**——用户已经习惯"不带 `&` 是临时"。

---

### Q5.6：`#I_LIMIT_J X I` 要不要支持 `&` 后缀？

**用户回答**（2026-08-07 00:27）：同 Q5.5 推理——"比较不可能每次发送都要存"

**结论**：✅ **选 A**——**支持 `&` 后缀**——理由同 Q5.5（保护电机 flash）。

**语义**：
- `#I_LIMIT_J X I`（不带 `&`）→ 临时设置（`canBuf[4]=0`）
- `#I_LIMIT_J X I &`（带 `&`）→ 持久化（`canBuf[4]=1`）

---

## 6. 用户最终的所有决定（核对清单）

> 更新于 2026-08-07 01:21

| # | 决定 | 来源 | 状态 |
|---|---|---|---|
| 1 | `jointAccBases` **保留**，电机自己存 | 2026-08-07 01:21（撤销原删除决策）| ✅ |
| 2 | `railAcc_mm_s2` 删除，地轨加速度存电机 EEPROM | 第一次回答 + Q4 | ✅ |
| 3 | `MoveRail` 不再发 `SetAcceleration` | 第一次回答 + Q4 | ✅ |
| 4 | `#ACC_BASE_J` **保留**（原"删除"决策撤销），串口助手加批量 UI 自己发 6 次 **取消**（用户原意改变）| 2026-08-07 01:21 | ✅ |
| 5 | 主控 UART echo `[ACC] MOTOR [X] = X.XX`，无 OK/MISMATCH | 第一次回答 | ✅ |
| 6 | 单位用用户能看懂的（圈/s²、A） | 第一次回答 | ✅ |
| 7 | 电机回什么打印什么 | 第一次回答 | ✅ |
| 8 | 每次只查设置的那一个电机 | 第一次回答 | ✅ |
| 9 | 不优化电机固件（不放低优先级任务） | 第三次回答 | ✅ |
| 10 | 串口助手加"请在失能状态下设置"提示 | 第三次回答 | ✅ |
| 11 | 0.5s 回查责任在串口助手 | 第一次回答 | ✅ |
| 12 | 6 轴速度完全不动（已经在 MoveJ/MoveL 里每次重发） | Q1 回答 | ✅ |
| 13 | 0x2C/0x2D 主控同步等 100ms | Q2 回答 | ✅ |
| 14 | 不需要"一键查看全部加速度"UI | Q3 回答 | ✅ |
| 15 | 地轨速度每次重发（不存 EEPROM） | Q4 回答 | 🔄 **修正**：Q5.3 选 c——地轨速度**保留存主控 EEPROM**（带 `&` 时），`MoveRail` 时**只读不存**（不触发 `SaveConfig`） |
| 16 | 地轨加速度存地轨电机控制板 | Q4 回答 | ✅ |
| 17 | UI3 失能检查阻塞 + 弹窗 | Q5 回答 | ✅ |
| 18 | `#ACC_RAIL X` 不带 `&`：`canBuf[4]=0` 发电机不入 EEPROM | Q5.1 回答 | ✅ |
| 19 | `#SPEED_RAIL X` 不带 `&`：保持原行为 | Q5.2 回答 | ✅ |
| 20 | `#SPEED_RAIL X &`：保留存主控 EEPROM（Q5.3 选 c） | Q5.3 回答 | ✅ |
| 21 | 地轨不加入"同步到达"逻辑（Q5.4 选 A） | Q5.4 回答 | ✅ |
| **22** | **`#SPEED_J` 完全不动**（B1 flash bug 标记为"已知 bug，本次不修"） | **2026-08-07 01:21** | ✅ |
| **23** | **`jointAccBases` / `#ACC_BASE_J` / `SetJointAcceleration` 完全不动**（M2/M10/M14/D1~D4/D10/D11/B4/B7 全部撤销）| **2026-08-07 01:21** | ✅ |

### 6.1 开放问题状态

| # | 问题 | 用户最终决定 | 文档位置 |
|---|---|---|---|
| Q5.1 | `#ACC_RAIL X` 不带 `&` 怎么办？ | 选 a：`canBuf[4]=0` 发电机不入 EEPROM | 第 5 节 |
| Q5.2 | `#SPEED_RAIL X` 不带 `&` 怎么办？ | 保持原行为 | 第 5 节 |
| Q5.3 | `#SPEED_RAIL X &` 怎么办？ | 选 c：保留存主控 EEPROM | 第 5 节 |
| Q5.4 | 地轨要不要也实现"同步到达"？ | 选 A：不实现 | 第 5 节 |
| Q5.5 | `#ACC_J X S` 要不要支持 `&` 后缀？ | ✅ 选 A：支持（保护 flash） | 第 5 节 |
| Q5.6 | `#I_LIMIT_J X I` 要不要支持 `&` 后缀？ | ✅ 选 A：支持（保护 flash） | 第 5 节 |
| **Q-extra1** | `#SPEED_J` 怎么处理？ | ✅ **不动**（2026-08-07 01:21） | 第 3.0.1 节 / 附录 C |
| **Q-extra2** | `jointAccBases[6]` 字段怎么处理？ | ✅ **保留**（2026-08-07 01:21） | 第 3.0.2 节 / 附录 C |

### 6.2 全部开放问题已确认

✅ **全部 8 个开放问题已回答**——需求清单完整无歧义——**等用户审阅后启动重构**。

## 7. 涉及的固件清单

### 主控（必须改）

```
firmware/ref_core_f405/
├── Robot/instances/dummy_robot.h
├── Robot/instances/dummy_robot.cpp
├── Robot/actuators/ctrl_step/ctrl_step.cpp
├── Robot/actuators/ctrl_step/ctrl_step.hpp
├── UserApp/protocols/ascii_protocol.cpp
└── UserApp/protocols/can_protocol.cpp
```

### 电机固件（4 份同步，必须改）

```
firmware/motor_fw_f103_35/UserApp/protocols/interface_can.cpp
firmware/motor_fw_f103_42/UserApp/protocols/interface_can.cpp
firmware/motor_fw_f103_57/UserApp/protocols/interface_can.cpp
firmware/motor_fw_f103_gripper/UserApp/protocols/interface_can.cpp
```

### 串口助手（待主控/电机完成后再做）

```
e:\Dummy-code\串口助手.py
```

---

## 8. 文档结尾

本文档由 AI 助手根据用户多轮对话整理：

- 2026-08-06 23:49 起：初次需求收集
- 2026-08-07 00:06：Q5.1/Q5.3/Q5.4 用户确认
- 2026-08-07 00:32：扫描代码发现 7 个隐藏 bug（B1~B7）
- **2026-08-07 01:21：用户最终决策——`#SPEED_J` 和 `jointAccBases` 全部不动**

**用户接下来需要做的事**：
1. ✅ 审阅本文档
2. ⏳ 通过对话告诉 AI 助手"可以开始规划重构任务"

**AI 助手接下来要做的事**：
- ⏳ 用户说"可以开始规划"后，进入重构任务规划阶段
- 规划阶段产出"按文件分组的最小化修改任务清单"——每个任务可独立 review/应用

**最终重构范围（基于 2026-08-07 01:21 决策）**：

| 类别 | 必须改 | 撤销/取消 |
|---|---|---|
| **CAN 接口**（主控） | M1 / M3（加 `persist` 参数）| M2 / M8 / M9 / M10 / M14 |
| **ASCII 命令**（主控） | M4 / M5 / M6 / M7 / M11 / M12 / M13 | —— |
| **删除/重构**（主控） | D5 / D6 / D7 / D8 / D12 / D13 | D1 / D2 / D3 / D4 / D10 / D11 |
| **新增查询**（主控 + 电机） | A1 / A2 / A3 / A4 / A5 / A6 / A7 / A8 | —— |
| **Bug 修复** | B2 / B3 / B5 / B6 | B1 / B4 / B7 |

**总计改动**：

- 主控 cpp/hpp：3 文件（M1+M3+M11+M12+M13 ~6 行 × 2 = ~12 行改；新增 `QueryAcceleration`/`QueryCurrentLimit` ~30 行 ×2 = ~60 行）
- 主控 ascii_protocol.cpp：~30 行改
- 电机 cpp（4 份同步）：~10 行/份 × 4 = ~40 行改
- 串口助手：暂未做

---

## 附录 A：Q1~Q5 决策回顾表

| # | 问题 | 用户原话 | 确认结论 |
|---|---|---|---|
| Q1 | `railSpeed_mm_s` 字段是否一并删除？ | "速度不用管，每次发送 movej 和 movel 最后一个值不就是速度吗" | 6 轴速度不动；地轨速度 `railSpeed_mm_s` 字段保留，主控 EEPROM **保留存**（Q5.3 选 c） |
| Q2 | 0x2C/0x2D 响应主控怎么等？ | "同步等 100ms，反正设置的时候是失能状态一般不会有其他的 can 消息" | A 同步等 100ms |
| Q3 | 是否需要"一键查看全部加速度"UI？ | "不用加这个功能，用户自己点自然就有一个延迟的时间等主控同步等" | 不需要 |
| Q4 | 地轨速度/加速度是配置一次长期生效，还是每次运动重发？ | "地轨的速度每次重发，加速度存在地轨电机控制板中" | **速度**：每次重发 + **保留主控 EEPROM 存**（Q5.3 选 c 修正）；**加速度**：存电机 EEPROM（CAN 0x14 `canBuf[4]=1`） |
| Q5 | UI3 失能检查是阻塞还是警告？ | "阻塞吧，防止出现意外" | 阻塞 + 弹窗 |

---

## 附录 B：Q5.1~Q5.6 决策回顾表

| # | 问题 | 用户回答 | 确认结论 |
|---|---|---|---|
| Q5.1 | `#ACC_RAIL X` 不带 `&` 怎么办？ | "确认，选 a" | 选 a：`canBuf[4]=0` 发电机不入 EEPROM |
| Q5.2 | `#SPEED_RAIL X` 不带 `&` 怎么办？ | （默认保持） | 保持原行为 |
| Q5.3 | `#SPEED_RAIL X &` 怎么办？ | "确认速度存主控" | 选 c：保留存主控 EEPROM |
| Q5.4 | 地轨要不要也实现"同步到达"？ | "不需要地轨加入同步到达逻辑，地轨的速度比较慢，不合适" | 选 A：不实现——地轨保持固定速度 |
| Q5.5 | `#ACC_J X S` 要不要支持 `&` 后缀？ | "比较不可能每次发送都要存，而且存还要清除 flash，这样不就和调整之前没区别了吗" | 选 A：支持——保护电机 flash |
| Q5.6 | `#I_LIMIT_J X I` 要不要支持 `&` 后缀？ | 同 Q5.5（同一个推理） | 选 A：支持——保护电机 flash |

---

## 附录 C：2026-08-07 01:21 用户最新决策

### C.1 用户原话

> "可以，你的解释很清晰，先跟新文档吧，最大速度和主控的基础加速度留着不动就好了，改完后我来检查一遍"

### C.2 决策解读

| 项 | 用户原意 | AI 解读 |
|---|---|---|
| "最大速度" | `#SPEED_J` 涉及的最大速度相关代码（包括 ASCII 命令、`SetVelocityLimit`、`canBuf[4]=1` 硬编码）| **完全不动**——`B1` flash bug 标记为已知但不修 |
| "主控的基础加速度" | `jointAccBases[6]` 字段、`#ACC_BASE_J` 命令、`SetJointAcceleration(%)` 内部调用 | **完全不动**——保留 RAM + EEPROM 存储机制 |
| "留着不动就好了" | 任何涉及这两个系统的代码或重构项 | **D1~D4/D10/D11/M2/M10/M14/B4/B7 全部撤销** |
| "改完后我来检查一遍" | 用户希望先固化文档，再启动重构 | **重构任务暂不开始**——等用户审阅文档后再说 |

### C.3 为什么用户改变主意

- 之前的"删除 `jointAccBases`"决策是基于"主控和电机双重存储会导致不一致"的问题
- 但用户发现：
  - 这两个存储系统**实际是独立的语义层**
  - `jointAccBases` 是"安全加速度上限"（开发者实测）
  - 电机 `ratedVelocityAcc` 是"位置环限速斜率"
  - 两者用途不同，不构成冲突
- **`#SPEED_J` 在串口助手根本没按钮** —— 删除/重构无用户收益
- B1 理论 bug 无用户影响 —— 不必改
- **结论**：最小化原则 —— 只改确实能改进的地方

### C.4 已确认的全部决策（截至 2026-08-07 01:21）

- ✅ Q5.1：选 a
- ✅ Q5.2：保持原行为
- ✅ Q5.3：选 c
- ✅ Q5.4：选 A
- ✅ Q5.5：选 A
- ✅ Q5.6：选 A
- ✅ 用户最新决策 22： `#SPEED_J` 不动
- ✅ 用户最新决策 23： `jointAccBases` 不动

**所有决策已确认** —— 需求清单稳定 —— 等用户审阅后启动重构。