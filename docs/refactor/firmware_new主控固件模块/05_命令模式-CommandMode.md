# 模块 05 — 命令模式（`CommandMode` 6 模式 + `#CMDMODE` + cmdModeNames）

> **范围**：命令模式枚举定义 + `#CMDMODE` ASCII 命令 + 6 模式 switch 分支 + 5kHz 实时环分发
> **关联文件**：
> - `Robot/instances/dummy_robot.cpp`（`CommandMode` 枚举定义 + `SetCommandMode` 实现 + switch case + `ServoJ` 函数）
> - `Robot/instances/dummy_robot.h`（`DEFAULT_COMMAND_MODE` + `commandMode` 字段）
> - `UserApp/main.cpp`（`ThreadOledUpdate` 第 6 行引用 `cmdModeNames[6][4]` + 5kHz 实时环分发）
> - `UserApp/protocols/ascii_protocol.cpp`（`#CMDMODE` 分支，`OnUsbAsciiCmd` + `OnUart4AsciiCmd` 双协议路径）

---

## 模块概述

机械臂有 6 种"命令模式"（`COMMAND_MODE`），决定 `>` `&` `@` `$` 进入哪个 switch 分支：

| 枚举值 | 名称 | ASCII | 行为 | 实现状态 |
|---|---|---|---|---|
| 1 | `COMMAND_TARGET_POINT_SEQUENTIAL` | `SEQ` | 阻塞回 ok | ✅ |
| 2 | `COMMAND_TARGET_POINT_INTERRUPTABLE` | `INT` | 立即回 ok，新指令覆盖 | ✅ |
| 3 | `COMMAND_CONTINUES_TRAJECTORY` | `TRJ` | 阻塞回 ok（复用 SEQ 阻塞通路） | ✅ |
| 4 | `COMMAND_MOTOR_TUNING` | `TUN` | 扫频正弦激励 | ✅ |
| 5 | `COMMAND_TORQUE_CONTROL` | `TRQ` | 力矩透传（`$` 入口 + 100Hz 下发） | ✅ |
| 6 | `COMMAND_SERVO_J` | `SRV` | 5kHz 全速随动 | ⚠ 代码完整，入口暂不开放（见 TODO） |

> **2026-09-27 事实修订**：原文档此行写"TRQ/SRV 暂不实现"，与代码实际不符。撤销这句，统一标注为"全部代码就绪 / SRV 入口暂关"。

---

## 三状态对比

| 维度 | OLD | firmware | 当前 firmware_new |
|---|---|---|---|
| 模式数量 | 4（SEQ/INT/TRJ/TUN） | 6（SEQ/INT/TRJ/TUN/TRQ/SRV） | **6** |
| `cmdModeNames` 数组 | `["SEQ","INT","TRJ","TUN"]` | `["SEQ","INT","TRJ","TUN","TRQ","SRV"]` | **6 元素**（OLED F3 已恢复引用，见 06 文档） |
| `DEFAULT_COMMAND_MODE` | **2 (INT)** | 1 (SEQ) | **2 (INT)** — `dummy_robot.h:290` 注释明写"OLD 默认" |
| `#CMDMODE` 命令 | 支持 | 支持 | **支持**（双协议，详见实施） |
| 5kHz 实时环 — SEQ/INT/TRJ | 50Hz 下发 | 50Hz 下发 | **50Hz 下发**（`main.cpp:188-200`） |
| 5kHz 实时环 — SRV | 不支持 | 5kHz | **5kHz**（`main.cpp:202-207`） |
| 5kHz 实时环 — TUN | 100Hz | 100Hz | **100Hz**（`main.cpp:209-215`） |
| 5kHz 实时环 — TRQ | 不支持 | 100Hz | **100Hz**（`main.cpp:217-225`） |
| SERVO_J 入口 | 不支持 | `#CMDMODE 6` 可切 | **代码完整，入口暂不开放**（见 TODO `[BUG-SRV-MODE]`） |
| TORQUE_CONTROL 入口 | 不支持 | `$` 入口可切 | **`$` 入口 + `#CMDMODE 5` 双路径生效** |

> **2026-09-27 修订**：
> - 原"DEFAULT_COMMAND_MODE = 1 (SEQ)"是错的，代码是 2 (INT)。修订为 2。
> - 原"SERVO_J/TORQUE_CONTROL 未实现"是错的。代码两端均已落地，但 SERVO_J 入口（`SetCommandMode(6)`）被范围检查拦截。

---

## 决策

| # | 决策 | 理由 | 风险 |
|---|---|---|---|
| 1 | **`cmdModeNames` 保留 6 模式数组并恢复使用** | OLED F3 恢复 `[CMD] __*_*__` 状态横幅（F3 决策引 [06 文档](06_OLED显示.md)）；CMD 字符串来源需要 6 元素 | 无 |
| 2 | **`SetCommandMode` 接受范围保留 1~5（暂拒绝 6）** | SERVO_J 入口拒绝 6 是当前实际行为；与地轨逻辑调整一并处理，避免入口开放后功能不完整 | 用户发 `#CMDMODE 6` 会被静默丢弃（回 ok 但 mode 不变）—— 上位机需靠"查 commandMode"二次校验 |
| 3 | **5 模式（SEQ/INT/TRJ/TUN/TRQ）端到端工作** | switch case + 5kHz 实时环分发都到位（见实施现状代码段） | TRQ 切档时无 isEnabled / current 限幅自动重置（已知，未入待办） |
| 4 | **SERVO_J 代码完整但入口暂不开放** | 范围 bug + `ServoJ` 函数未为地轨算 reqSpeed（地轨速度沿用 MoveJ 残值），统一作为 `[BUG-SRV-MODE]` 落地 | 用户主动切 6 被静默拒，可能造成疑惑 |

---

## 实施

### 现状代码（2026-09-27 审计后，全部对齐实际实现）

#### A. 枚举与 switch（`dummy_robot.cpp:668-708`）

```cpp
void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_TORQUE_CONTROL)    // ★ 实际接受范围 1~5，6（SRV）被拒
        return;

    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            // SEQ 与 INT 仅 ParseCommand 阻塞行为不同，档位配置完全相同
            jointSpeedRatio = 1;
            ApplyJointAccelTier(DEFAULT_JOINT_ACCELERATION_LOW);   // LOW = 30%
            break;

        case COMMAND_CONTINUES_TRAJECTORY:
            // TRJ: ratio=0.3（速度上限缩到 30%）+ HIGH 加速度档位
            jointSpeedRatio = 0.3;
            ApplyJointAccelTier(DEFAULT_JOINT_ACCELERATION_HIGH);  // HIGH = 100%
            break;

        case COMMAND_MOTOR_TUNING:
            break;

        case COMMAND_TORQUE_CONTROL:
            break;

        case COMMAND_SERVO_J:
            // 注：当前入口被上面范围检查拦截，本 case 体未被实际触发
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION);  // 150 r/s²
            break;
    }
}
```

注：
- SEQ/TRJ 共用 fall-through（`dummy_robot.cpp:823` 的 `ParseCommand` case），共用阻塞回 ok 行为；但 `SetCommandMode` 切到 TRJ 时把 `jointSpeedRatio` 设为 0.3 + HIGH 加速度档——必须配套使用。
- 模式 5（TRQ）走两条进入路径：
  - `$c1,c2,...,c7` 在 `ParseCommand` 顶端直接拦截并切档（`dummy_robot.cpp:778-786`），不依赖 `#CMDMODE 5`
  - `#CMDMODE 5` 切档后，下发路径走 5kHz 实时环 100Hz tick

#### B. 5kHz 实时环分发（`main.cpp:178-225`）

```cpp
if (dummy.IsEnabled())
{
    switch (dummy.commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
        case COMMAND_CONTINUES_TRAJECTORY:
            // [总线节流] 50 Hz 下发（updateCounter % 100 == 0）
            if (updateCounter % 100 == 0) {
                dummy.MoveJoints(dummy.targetJoints);
                dummy.motorJ[7]->SetPositionWithMotorRps(
                    dummy.targetRailPos / 5.0f, dummy.railSpeedRps);
            }
            dummy.UpdateJointPose6D();
            break;

        case COMMAND_SERVO_J:
            // 5 kHz 全速下发
            dummy.MoveJoints(dummy.targetJoints);
            dummy.motorJ[7]->SetPositionWithMotorRps(
                dummy.targetRailPos / 5.0f, dummy.railSpeedRps);
            dummy.UpdateJointPose6D();
            break;

        case COMMAND_MOTOR_TUNING:
            // 100 Hz
            if (updateCounter % 50 == 0) {
                dummy.tuningHelper.Tick(10);
            }
            dummy.UpdateJointPose6D();
            break;

        case COMMAND_TORQUE_CONTROL:
            // 100 Hz
            if (updateCounter % 50 == 0) {
                for (int i = 1; i <= 6; i++)
                    dummy.motorJ[i]->SetCurrentSetPoint(dummy.targetCurrents[i-1]);
                dummy.motorJ[7]->SetCurrentSetPoint(dummy.targetRailCurrent);
            }
            dummy.UpdateJointPose6D();
            break;
    }
}
```

> **2026-09-27 修订**：原文档实施范围仅指 `SetCommandMode` + switch；5kHz 实时环分发也属于"命令模式实施"范畴，本节补入。

#### C. `#CMDMODE` 双协议路径（`ascii_protocol.cpp`）

`#CMDMODE` 不走 `ParseCommand` 通道，由 ASCII 入口函数各自独立处理：

| 入口函数 | 行号 | 通道 | 回包（2026-09-27 统一） |
|---|---|---|---|
| `OnUsbAsciiCmd` | line 479-485 | USB | `ok Set command mode to [%lu]` |
| `OnUart4AsciiCmd` | line 1011-1017 | UART4 | `ok Set command mode to [%lu]`（2026-09-27 补 `ok ` 前缀，与 USB 一致） |

代码（双协议同步）：
```cpp
else if (s.find("CMDMODE") != std::string::npos)
{
    uint32_t mode;
    sscanf(_cmd, "#CMDMODE %lu", &mode);
    dummy.SetCommandMode(mode);
    Respond(_responseChannel, "ok Set command mode to [%lu]", mode);
}
```

> **2026-09-27 修订**：
> - 原文档未列出 `#CMDMODE` 在 ascii_protocol.cpp 中实现两份的事实，本节补入
> - UART4 路径回包补 `"ok "` 前缀（消除双协议回包格式差异）—— 改动 1 行

### 关闭 F1.4（2026-09-27）

**历史**：原 F1.4（"`#CMDMODE 5/6` 行为明确"）的 A/B/C 三个选项都基于"`TRQ/SRV 未实现`"前提。

**事实修订**：
- 5（TRQ）已完整实现（`$` 拦截 + 5kHz 实时环 100Hz 下发）
- 6（SRV）已完整实现（5kHz 实时环 5kHz 下发），但**入口暂不开放**

**关闭原因**：F1.4 前提不再成立，原 A/B/C 选项对当前实现无指导意义。

**真正遗留**：SRV 范围 bug + 地轨逻辑调整，已单独立项 `docs/another/TODO.md` 的 `[BUG-SRV-MODE]`，具体调整方案后续讨论。

### SERVO_J 入口暂不开放 — 遗留

详见 `docs/another/TODO.md` 的 `[BUG-SRV-MODE]` 条目。两条遗留紧密耦合：

1. `SetCommandMode` 范围检查拒绝 6（`dummy_robot.cpp:670-671` 的 `_mode > COMMAND_TORQUE_CONTROL`）
2. `ServoJ` 函数（`dummy_robot.cpp:391-426`）当前只对 `j=1..6` 计算 `reqSpeed`，**未为地轨算 reqSpeed**，地轨速度直接沿用 `MoveJ` 残值或 `railSpeedRps` 默认值

这两条独立修改都不完整——只修范围会让 SRV 跑出"地轨速度固定值"的不一致行为；只修 ServoJ 又用不到，因为根本切不进 SRV。

**禁止事项**：
- ❌ 不要在本模块同时放开 SRV 入口和修 ServoJ 地轨逻辑——任何一处的修改必须配套另一处，否则引入不一致
- ❌ 不要把 `cmdModeNames` 缩回 4 元素——OLED 6 模式显示需要
- ❌ 不要让 `SetCommandMode` 拒绝 1~4（保持当前行为）
- ❌ 不要把 UART4 路径的 `#CMDMODE` 回包前缀与 USB 路径不一致（已统一）

### OLED 显示（F3 已恢复）

详见 [06 - OLED显示](06_OLED显示.md) 的 F3 实施。`cmdModeNames` 现在被 OLED 第 6 行 `[CMD] __*_*__` 状态横幅引用，数组从 dead code 转为实际使用。

---

## 验证

| 测试 | 期望 | 命令 / 观察 |
|---|---|---|
| 6 模式 `cmdModeNames` 数组 | 6 元素编译通过 | `grep "cmdModeNames" firmware_new/.../main.cpp` |
| `DEFAULT_COMMAND_MODE = 2` | 上电后 `commandMode == 2` | 上电后 OLED 第 6 行显示 `[INT] __*_*__` |
| `#CMDMODE 1~5` 切换 | `commandMode` 字段变化；OLED 第 6 行 `[SEQ/INT/TRJ/TUN/TRQ]` 切换 | `#CMDMODE 1` ... `#CMDMODE 5` |
| `#CMDMODE 6` 行为 | 回 `ok Set command mode to [6]`，但 `commandMode` 不变（静默拒绝） | `#CMDMODE 6` 后查 OLED / 心跳 |
| `#CMDMODE 1~4` 行为 | SEQ 阻塞 + INT 立即 + TRJ 阻塞 + TUN 扫频 | 各个模式发 `>` 验证 |
| `#CMDMODE 5` + `$` | 力矩透传 100Hz 下发 | `$1.0,1.0,...,1.0` 后观察电机电流 |
| `#CMDMODE` 双协议回包 | USB/UART4 回包均为 `ok Set command mode to [%lu]` | 分别从两通道发 `#CMDMODE 1` |
| 5kHz 实时环分发 | SEQ/INT/TRJ=50Hz、SRV=5kHz、TRQ=100Hz、TUN=100Hz | 切到对应模式观察 `g_realtime_tick` × 实时环内部分发计数 |

---

## 状态

- [x] 6 模式 `cmdModeNames` 数组（C3，2026-09-27 修订：被 OLED 引用，不再是 dead code）
- [x] switch case 6 模式
- [x] `#CMDMODE` 双协议回包格式统一（2026-09-27，UART4 路径补 `"ok "` 前缀）
- [x] OLED 状态横幅 `[CMD] __*_*__` 恢复（[06_OLED显示](06_OLED显示.md) F3 已实施）
- [x] 5 模式（SEQ/INT/TRJ/TUN/TRQ）端到端工作
- [x] F1.4 关闭（2026-09-27，基于"未实现"的错误前提）
- [ ] TODO `[BUG-SRV-MODE]`：SERVO_J 范围 bug + `ServoJ` 地轨速度规划（地轨速度规划 + 入口开放须配套处理）

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [01 - MoveJ](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md) | F1.1/F1.3 阻塞依赖 SEQ/TRJ 模式；ParseCommand 共用 fall-through |
| [06 - OLED显示](firmware_new主控固件模块06_OLED显示.md) | `cmdModeNames` 是 OLED 第 6 行 `[CMD] __*_*__` 状态横幅模式名字符串来源（F3 已恢复） |
| [13 - 架构常量](firmware_new主控固件模块13_架构常量.md) | `DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE = 2`（OLD 默认，非 SEQ） |
| TODO `[BUG-SRV-MODE]`（`docs/another/TODO.md`） | SERVO_J 入口范围 bug + `ServoJ` 地轨速度规划待调整，配套修复 |
