# 电机固件重构文档 — `firmware_old` → `firmware_new`

> **范围**：仅电机固件（STM32F103），覆盖 35 / 42 / 57 / gripper 共 4 套电机。
>
> **目标**：以 `firmware_old/dummy-35motor-fw` / `dummy-42motor-fw` 为基线，
> 把 [`重构需求—firmware_old迁移到firmware功能清单—2026-09-19.md`](./重构需求—firmware_old迁移到firmware功能清单—2026-09-19.md) 中电机部分的需求，
> 重新实现在 `firmware_new/` 下。
>
> **核心原则**：
> - 不 cherry-pick `firmware/` 代码，只看思路自己写
> - `firmware_new/` 的 35/42 电机已经是 firmware_old 的副本，以此为基础增量修改
> - 堵转检测 v3（0x5A/0x5B/0x5C）**不实现**，但其"检测→回退→锁定→解锁"的状态机思路用于 `!BACKHOME`
>
> **创建日期**：2026-09-21

---

## 一、现状盘点（firmware_new 电机固件现在有什么）

### 1.1 目录结构

```
firmware_new/
├── dummy-35motor-fw/      ← 已存在，内容是 firmware_old 的副本
├── dummy-42motor-fw/       ← 已存在，内容是 firmware_old 的副本
└── dummy-ref-core-fw/      ← 主控固件（不在本文范围）
```

> `firmware_new/` 下**目前只有 35 和 42 电机固件**，57 和 gripper 固件尚未创建。
> 57 和 gripper 的初始内容和 35 一样（都是 firmware_old 副本），后续通过 `MOTOR_TYPE` 宏切参数。

### 1.2 已经实现的功能（直接来自 firmware_old，无需修改）

| 模块 | 文件 | 状态 | 说明 |
|---|---|---|---|
| CAN 协议解析 | `UserApp/protocols/interface_can.cpp` | ✅ 已有 | 支持 0x01~0x1B, 0x21~0x25, 0x7D~0x7F |
| 电机控制核心 | `Ctrl/Motor/motor.cpp` | ✅ 已有 | FOC 电流环 + PID 速度环 + DCE 位置环 |
| 运动规划器 | `Ctrl/Motor/motion_planner.cpp` | ✅ 已有 | 梯形曲线规划 |
| 编码器 | `Ctrl/Sensor/Encoder/mt6816_base.cpp` | ✅ 已有 | MT6816 磁编驱动 |
| 电机驱动 | `Ctrl/Driver/tb67h450_base.cpp` | ✅ 已有 | TB67H450 双 H 桥 |
| 校准器 | `Ctrl/Sensor/Encoder/encoder_calibrator_base.cpp` | ✅ 已有 | 编码器归零 |
| 按钮/LED | `Port/button_stm32.cpp` / `led_stm32.cpp` | ✅ 已有 | |
| EEPROM | `Port/Platform/Memory/emulated_eeprom.cpp` | ✅ 已有 | |
| 定时器 | `Core/Src/tim.c` | ✅ 已有 | TIM1 100Hz + TIM4 20kHz |
| CAN 收发 | `Core/Src/can.c` | ✅ 已有 | 标准 CAN |

### 1.3 需要改的地方（现状 vs 需求）

| # | 现状（firmeware_new 电机） | 目标（需求） | 优先级 |
|---|---|---|---|
| 1 | **3 个独立工程**（35/42 各一套） | **合并为 1 套源码 + 4 个 build**（35/42/57/gripper） | P0 |
| 2 | **无 0x09 急停广播**（旧固件 CAN ID 0x89 是 bug） | **新增 0x09 广播急停** | P0 |
| 3 | **`ApplyPosAsHomeOffset()` 不同步 realPosition**（GetPosition 返回累计圈数） | **v3.2 修复**：同步 `realPosition = encoderHomeOffset` | P0 |
| 4 | **无 57 和 gripper 固件** | 创建两个新 build，**57 = 硬编码 ID=7，gripper = 硬编码 ID=8** | P0 |
| 5 | **4 个参数无法独立查询**（0x28~0x2D 缺失） | **新增 6 条查询命令**：0x28 DCE Kp / 0x29 Kv / 0x2A Ki / 0x2B Kd / 0x2C 加速度 / 0x2D 电流限幅 | P1 |
| 6 | **`0x07` 总是立即发 FINISH ACK**（电机控制环还没跑到就回包） | **取消立即 ACK**：LOCKED/RETREATING 状态丢弃命令；正常状态去掉立即 ACK | P1 |
| 7 | **电流默认值 1A / 加速度 100 r/s²**（与需求不符） | **按 4 个 build 修正默认值**（见 §四） | P1 |
| 8 | **`ClearIntegral()` 是 const**，清 PID/DCE 积分 | **重构为非 const**，只清 estimator 状态（estVelocityIntegral / estError） | P2 |
| 9 | **`ResetGoalsToCurrentPosition()` 不存在** | **新增**：用于 `!BACKHOME` 回退时停在当前位置 | P2 |
| 10 | **`motor.h` 中有 `isStalled` / `stalledTime` 等字段** | **删除**，替换为 BACKHOME 状态机字段 | P2 |
| 11 | **无 `SetPositionSetPointWithTime()` 带时间的位置命令** | **新增**（已有方法签名，需在 CAN 层 case 0x06 中调用） | P1 |
| 12 | **gripper 的 `configurations.h` 包含 `enableStallProtect` 字段** | **删除**（gripper 完全无堵转检测） | P1 |
| 13 | **57 和 gripper 固件尚未加入 CMake** | **修改 CMakeLists.txt**，加入 57 和 gripper 两个 build target | P0 |

---

## 二、任务拆分（按优先级）

> 每个任务说明：做什么 → 改哪些文件 → 怎么验证。

### 【P0-1】合并 4 个 build，更新 CMakeLists.txt

**做什么**：
- 电机固件最终是 1 套源码，通过 `MOTOR_TYPE` 宏切换参数，出 4 个 ELF
- 在 35 电机的 CMakeLists.txt 基础上，添加 57 和 gripper 两个 target
- 4 个 target 用同一套源文件，只在编译参数里加 `-DMOTOR_TYPE=35` / `42` / `57` / `8`

**改哪些文件**：
- `firmware_new/dummy-35motor-fw/CMakeLists.txt` — 修改，加入 4 个 target
- `firmware_new/dummy-35motor-fw/UserApp/configurations.h` — 条件编译，`MOTOR_TYPE` 决定 canNodeId 默认值

**验证**：
- `cd firmware_new/dummy-35motor-fw/build && ninja` → 产出 `motor-35.elf/.hex/.bin`
- 同样方式验证 42 / 57 / gripper 三个 build 都能编译通过

---

### 【P0-2】新建 57 和 gripper 固件目录

**做什么**：
- 57 电机：硬编码 `canNodeId = 7`，其余参数与 35 一致（PID 等后续按 §四修正）
- gripper 夹爪：硬编码 `canNodeId = 8`，`configurations.h` 中去掉 `enableStallProtect` 字段

**改哪些文件**：
- 复制 `dummy-35motor-fw/` 为 `dummy-57motor-fw/` 和 `dummy-gripper-fw/`（用户手动做）
- `dummy-gripper-fw/UserApp/configurations.h` — 删 `enableStallProtect` 字段，删对应变量
- `dummy-gripper-fw/UserApp/protocols/interface_can.cpp` — gripper 不响应 0x1B（Set Stall-Protect）

**验证**：
- 3 个新 build 都能编译通过，CAN ID 正确

---

### 【P0-3】CAN 0x09 急停广播（替代旧 0x89 bug）

**做什么**：
- 在 `interface_can.cpp` 中添加 `case 0x09`：
  - 所有电机收到后：`requestMode = STOP`，`SetBrake(true)`，`SetCurrent/VelocitySetPoint(0)`
  - **不操作堵转状态机**（0x09 是急停，纯粹制动）
- 主控侧（`firmware_new/dummy-ref-core-fw/`）的 `EmergencyStop()` 改发 0x09，不再发 0x89

**改哪些文件**：
- `firmware_new/dummy-35motor-fw/UserApp/protocols/interface_can.cpp`
- （后续主控固件任务中改 `can_protocol.cpp`）

**验证**：
- 发 0x09，所有电机立即停止，不发任何 ACK

---

### 【P0-4】`ApplyPosAsHomeOffset()` v3.2 修复

**做什么**：
- 旧版（现状）：只改 `encoderHomeOffset`，`GetPosition()` 返回累计圈数
- 修复后：改 `encoderHomeOffset` 的同时，同步 `realPosition = encoderHomeOffset`，让 `GetPosition()` 立即返回 0

**改哪些文件**：
- `firmware_new/dummy-35motor-fw/Ctrl/Motor/motor.cpp` 中的 `ApplyPosAsHomeOffset()` 方法
- 参考 firmware `motor_fw_f103_35/Ctrl/Motor/motor.cpp` 中 v3.2 的实现思路（只看不抄，自己写）

**验证**：
- 发 `!CALIBRATION` / `#OFFSET_J` 后，发 `?J` 查询，`GetPosition()` 应立即返回 0（或设定的偏移值），不再返回累计圈数

---

### 【P1-1】新增 6 条 CAN 查询命令（0x28~0x2D）

**做什么**：在 `interface_can.cpp` 的 `case 0x21` 后添加 6 个 case：

| Cmd | 行为 | 回包 |
|---|---|---|
| `0x28` Get DCE Kp | 回 `boardConfig.dce_kp` | int32 |
| `0x29` Get DCE Kv | 回 `boardConfig.dce_kv` | int32 |
| `0x2A` Get DCE Ki | 回 `boardConfig.dce_ki` | int32 |
| `0x2B` Get DCE Kd | 回 `boardConfig.dce_kd` | int32 |
| `0x2C` Get Acceleration | 回 `velocityAcc / SUBDIVIDE`（float r/s²） | float |
| `0x2D` Get Current-Limit | 回 `currentLimit / 1000`（float A） | float |

**改哪些文件**：
- `firmware_new/dummy-35motor-fw/UserApp/protocols/interface_can.cpp`

**验证**：
- 主控发 0x28~0x2D，电机回对应值

---

### 【P1-2】`0x07` 去掉立即 ACK，LOCKED 状态丢弃

**做什么**：
- 去掉 `case 0x07` 中的立即 `CAN_Send(&txHeader, 0x23, ...)`（旧版在控制环还没跑到时就回包了，导致主控 SEQ 模式提前退出）
- 正常运行时，`0x07` 和 `0x05` 一样：只下目标位置，不立即回包，等 0x23 查询时再回

**改哪些文件**：
- `firmware_new/dummy-35motor-fw/UserApp/protocols/interface_can.cpp` 的 `case 0x07`

**验证**：
- 发 `>J j1 j2 ...` 后，主控在电机到位前不应收到 FINISH ACK

---

### 【P1-3】修正 4 个 build 的默认参数

**做什么**：按 §四的表格，修正以下默认值：

- 35/42/57/gripper 的 `currentLimit`（mA）
- 35/42/57/gripper 的 DCE Kp/Kv/Ki/Kd
- 35/42/57/gripper 的 PID Kp/Ki/Kd
- 35/42/57/gripper 的 `ratedCurrentAcc`（电流爬升斜率）
- 4 个 build 共用的 `velocityAcc` 从 100 r/s² 改为 30 r/s²

**改哪些文件**：
- `firmware_new/dummy-35motor-fw/Ctrl/Motor/motor.h`（类构造函数的默认值）
- `firmware_new/dummy-35motor-fw/UserApp/main.cpp`（首次烧录的 `boardConfig` 初始化）
- `firmware_new/dummy-35motor-fw/UserApp/configurations.h`（条件编译 `MOTOR_TYPE` 决定不同数值）

**验证**：
- 4 个 build 分别编译，查看 ELF 中常量段确认数值正确

---

### 【P1-4】gripper 的 `configurations.h` 去掉 `enableStallProtect`

**做什么**：
- `BoardConfig_t` 结构体中删 `enableStallProtect` 字段
- `interface_can.cpp` 中 gripper 的 `case 0x1B` 改为 no-op 或直接 `default`

**改哪些文件**：
- `firmware_new/dummy-gripper-fw/UserApp/configurations.h`
- `firmware_new/dummy-gripper-fw/UserApp/protocols/interface_can.cpp`

**验证**：
- gripper build 编译通过，无 `enableStallProtect` 相关引用

---

### 【P2-1】`ClearIntegral()` 重构

**做什么**：
- 旧版（现状）：是 `const` 方法，清 PID/DCE 的 `integralRound` / `outputKi`
- 新版：非 `const`，只清 `estVelocityIntegral` 和 `estError`（estimator 状态）

**改哪些文件**：
- `firmware_new/dummy-35motor-fw/Ctrl/Motor/motor.cpp`
- `firmware_new/dummy-35motor-fw/Ctrl/Motor/motor.h`

**验证**：
- `!BACKHOME` 切换状态时调用，积分累积消除，电机不抖动

---

### 【P2-2】`ResetGoalsToCurrentPosition()` 新增

**做什么**：
- 新增方法（非继承自 firmware，只借思路）：把 `goalPosition = estPosition`（绕过 +offset），清零 `goalVelocity` / `goalCurrent`，触发 `softNewCurve`
- 用于 `!BACKHOME` 回退时：让电机停在当前位置，而不是被驱动回旧目标

**改哪些文件**：
- `firmware_new/dummy-35motor-fw/Ctrl/Motor/motor.cpp`
- `firmware_new/dummy-35motor-fw/Ctrl/Motor/motor.h`

**验证**：
- `!BACKHOME` 流程中机械臂卡阻回退时，电机停在当前位置

---

### 【P2-3】删除 `isStalled` / `stalledTime` 等旧字段

**做什么**：
- `motor.h` 中删除：`isStalled` / `stalledTime` / `overloadTime` / `overloadFlag` / `ClearStallFlag()`
- 删除 `State_t::STATE_OVERLOAD`
- `main.cpp` 中按钮回调的 `ClearStallFlag()` 调用改为 no-op（gripper 是打印日志）

**改哪些文件**：
- `firmware_new/dummy-35motor-fw/Ctrl/Motor/motor.h`
- `firmware_new/dummy-35motor-fw/Ctrl/Motor/motor.cpp`
- `firmware_new/dummy-35motor-fw/UserApp/main.cpp`

**验证**：
- 编译通过，无 `isStalled` 相关引用

---

### 【P2-4】`SetPositionSetPointWithTime()` 在 0x06 中调用

**做什么**：
- firmware_new 的 `motor.cpp` 中 `SetPositionSetPointWithTime()` 方法应该已经存在
- 确认 `interface_can.cpp` 的 `case 0x06` 正确调用了该方法

**改哪些文件**：
- `firmware_new/dummy-35motor-fw/UserApp/protocols/interface_can.cpp` 的 `case 0x06`

**验证**：
- 发带时间参数的位置命令，电机按时间约束运动

---

## 三、实施顺序建议

```
第 1 步（P0）：CMake + 4 个 build + 57/gripper 目录
第 2 步（P0）：0x09 急停广播
第 3 步（P0）：ApplyPosAsHomeOffset() v3.2 修复
第 4 步（P1）：6 条 CAN 查询（0x28~0x2D）
第 5 步（P1）：0x07 去掉立即 ACK
第 6 步（P1）：修正 4 个 build 默认参数
第 7 步（P1）：gripper 去掉 enableStallProtect
第 8 步（P2）：ClearIntegral() 重构 + ResetGoalsToCurrentPosition() 新增
第 9 步（P2）：删除旧 stall 字段
第 10 步（P2）：0x06 确认调用 SetPositionSetPointWithTime()
```

> **说明**：电机固件重构**不需要等主控固件**，可以先独立做完。
> CAN 协议对齐后，主控固件的 0x09 / 0x28~0x2D 调用自然生效。

---

## 四、4 个 build 的默认参数（已确认）

> 来源：[`重构需求—firmware_old迁移到firmware功能清单—2026-09-19.md`](./重构需求—firmware_old迁移到firmware功能清单—2026-09-19.md) §1.1.2 第 51-67 行
> **实施时直接抄这段，不用再查文档。**

### 4.1 差异参数（每个 build 不同）

| 参数 | 35 | 42 | 57 | gripper |
|---|---|---|---|---|
| canNodeId | 拨码 1~6 | 拨码 1~6 | **硬编码 7** | **硬编码 8** |
| currentLimit | 2500 mA | 2500 mA | 3000 mA | 2500 mA |
| calibrationCurrent | 2000 mA | 2000 mA | 2000 mA | 2000 mA |
| dce_kp | 200 | 200 | 300 | 200 |
| dce_kv | 80 | 80 | 120 | 80 |
| dce_ki | 300 | 300 | 150 | 300 |
| dce_kd | 250 | 250 | 50 | 250 |
| pid_kp | 5 | 5 | **15** | 5 |
| pid_ki | 30 | 30 | **80** | 30 |
| pid_kd | 0 | 0 | **10** | 0 |
| ratedCurrentAcc | 2×1000 mA/s | 2×1000 mA/s | **50×1000** mA/s | 2×1000 mA/s |

### 4.2 共用参数（4 个 build 一样）

| 参数 | 值 |
|---|---|
| `MOTOR_ONE_CIRCLE_HARD_STEPS` | 200（1.8°步距） |
| `SOFT_DIVIDE_NUM` | 256 |
| `MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS` | 51200 |
| velocityLimit | 30 r/s（= 30 × 51200 step/s） |
| **velocityAcc** | **30 r/s²**（旧版是 100 r/s²，改为 30） |
| encoderHomeOffset | 0 |
| defaultMode | MODE_COMMAND_POSITION |
| enableMotorOnBoot | false |
| enableTempWatch | false |

---

## 五、不做的事情（已在 firmware_new 中确认）

- ❌ **堵转检测 v3 协议**：0x5A / 0x5B / 0x5C / stallMode 状态机 / !STALL_* 命令 — **全部不做**
- ❌ **CalcSoftGoalWithRealPos**：57 固件直接用标准轨迹规划，不做失步补偿
- ❌ **gripper 夹取力限制**：gripper 与 35/42/57 共用电流限幅即可
- ❌ **Stockpile_* 测试函数**：gripper 的出厂测试函数不迁移

---

## 六、验收标准

| # | 验收条件 | 检查方法 |
|---|---|---|
| 1 | 4 个 build 都能编译通过 | `ninja` 在 4 个 build 目录下各跑一次 |
| 2 | 4 个 ELF 同时产出 .hex 和 .bin | 检查 build 目录 |
| 3 | 57 CAN ID = 7，gripper CAN ID = 8 | 看代码中硬编码的值 |
| 4 | 0x09 急停：发命令后所有电机立即停止 | 模拟测试或实机 |
| 5 | 0x15 Apply Home 后 GetPosition() 返回 0 | 发 0x15 → 发 0x23 查询 |
| 6 | 0x28~0x2D 能返回正确数值 | 主控发命令验证 |
| 7 | 4 个 build 默认参数与 §四表格一致 | 编译后读 ELF 常量段 |
| 8 | gripper 无 enableStallProtect | `grep -r enableStallProtect` 在 gripper 目录无结果 |
| 9 | `ClearIntegral()` 是非 const，只清 estimator | 读代码确认 |
| 10 | `ResetGoalsToCurrentPosition()` 存在 | `grep -r ResetGoalsToCurrentPosition` 有结果 |
| 11 | 无 isStalled / stalledTime / overloadTime | `grep -r isStalled` 在 motor.h 无结果 |
| 12 | 57 和 gripper 固件已加入 CMake | 读 CMakeLists.txt 有 4 个 target |

---

## 七、参考文档

| 文档 | 内容 |
|---|---|
| [`重构需求—firmware_old迁移到firmware功能清单—2026-09-19.md`](./重构需求—firmware_old迁移到firmware功能清单—2026-09-19.md) | 完整功能清单电机部分，§1.1 和 §1.1.1 和 §1.1.2 |
| [`电机固件对比—firmware_old迁移到firmware—2026-09-21.md`](./电机固件对比—firmware_old迁移到firmware—2026-09-21.md) | 详细的 CAN 协议 diff、motor.cpp diff、默认参数对比 |
| [`重建需求草稿—firmware_new—2026-09-21.md`](./重建需求草稿—firmware_new—2026-09-21.md) | 总体需求草稿（电机 + 主控 + 协议 + RGB） |
| `firmware_new/dummy-35motor-fw/UserApp/protocols/interface_can.cpp` | 当前 firmware_new 电机 CAN 协议实现 |
| `firmware_new/dummy-35motor-fw/Ctrl/Motor/motor.cpp` | 当前 firmware_new 电机控制实现 |
| `firmware_new/dummy-35motor-fw/Ctrl/Motor/motor.h` | 当前 firmware_new 电机类定义 |
| `firmware/motor_fw_f103_35/Ctrl/Motor/motor.cpp` | **参考**（只看不抄）：v3.2 ApplyPosAsHomeOffset() 修复、ResetGoalsToCurrentPosition() 实现 |
| `firmware/motor_fw_f103_35/UserApp/protocols/interface_can.cpp` | **参考**（只看不抄）：0x09/0x28~0x2D 实现 |
