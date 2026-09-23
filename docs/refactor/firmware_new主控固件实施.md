# 主控固件重构文档 — `firmware_old` → `firmware_new`

> **范围**：仅主控固件（STM32F405，FreeRTOS）。
>
> **目标**：以 `firmware_old/dummy-ref-core-fw/` 为稳定基线，把 [`firmware_new主控固件决策.md`](./firmware_new主控固件决策.md) 中**已决策 ✅ 的功能**和**用户明确要求保留的旧行为**重新实现在 `firmware_new/dummy-ref-core-fw/` 下。
>
> **核心原则**：
> - **不 cherry-pick `firmware/ref_core_f405/` 代码**，只读源码按决策自己写
> - `firmware_new/dummy-ref-core-fw/` 当前**已是** `firmware_old/dummy-ref-core-fw/` 的 1:1 副本
> - **运动完成判断 + OLED 同步刷新 必须严格保留旧固件语义**（用户 P0 诉求）
> - `firmware/` 中未被决策保留的功能（如 SERVO_J 5kHz、TORQUE_CONTROL、`!BACKHOME` 等）一律不实现
>
> **创建日期**：2026-09-23

---

## 一、现状盘点（firmware_new 主控固件现在有什么）

### 1.1 目录与基线

```
firmware_new/dummy-ref-core-fw/
├── UserApp/
│   ├── main.cpp                       ✅ 已存在 = firmware_old 副本
│   ├── protocols/
│   │   ├── ascii_protocol.cpp         ✅ 已存在 = firmware_old 副本
│   │   ├── can_protocol.cpp           ✅ 已存在 = firmware_old 副本（0x23 + 0x25 处理）
│   │   └── cmd_protocol.cpp           ✅ 已存在 = firmware_old 副本
│   ├── common_inc.h
│   └── freertos_inc.h
├── Robot/
│   ├── instances/
│   │   ├── dummy_robot.cpp/.h         ✅ 已存在 = firmware_old 副本（6 轴 + 阻塞 ParseCommand）
│   ├── algorithms/kinematic/6dof_kinematic.cpp/.h
│   └── actuators/
│       ├── ctrl_step/ctrl_step.cpp/.hpp    ✅ 已存在 = firmware_old 副本
│       └── mintasca/                 （夹爪/手爪备用，未使用）
├── Bsp/                               ✅ 已存在 = firmware_old 副本
└── Core/, Drivers/, Middlewares/, 3rdParty/  ✅ 标准 HAL + FreeRTOS
```

> **关键事实**：固件代码（除 STM32 HAL/FreeRTOS 中间件外）的所有业务源文件当前都是 `firmware_old/` 的副本，**没有 v3.x 新功能**。所以本实施文档的任务本质上是**"在不动旧行为的前提下，按决策添加新功能"**。

### 1.2 已经实现的功能（直接来自 firmware_old，无需修改）

| 模块 | 文件 | 状态 | 说明 |
|---|---|---|---|
| FreeRTOS 4 线程 | `UserApp/main.cpp:35-117` | ✅ 已有 | `ControlLoopFixUpdateTask`(TIM7 5kHz)、`ControlLoopUpdateTask`、`OledTask`、`RGBTask` |
| **阻塞式运动完成判断** | `Robot/instances/dummy_robot.cpp:ParseCommand` 的 `>` `&` `@` 分支 | ✅ **P0 保留** | `while (IsMoving() && IsEnabled()) osDelay(5);` 后回 `ok` 到 USB+UART4 |
| **OLED 关节角度显示** | `UserApp/main.cpp:ThreadOledUpdate` | ✅ **P0 保留** | 第 3 行：`>%3d|%3d|%3d|%3d|%3d|%3d` 显示 6 轴当前角度（整数°） |
| **OLED 运动状态 `_`/`*`** | `UserApp/main.cpp:ThreadOledUpdate` | ✅ **P0 保留** | 第 4 行：`buf[i-1] = (jointsStateFlag & (1<<i) ? '*' : '_');` 显示每轴到位状态 |
| 6 轴 DH 运动学 | `Robot/algorithms/kinematic/6dof_kinematic.cpp` | ✅ 已有 | FK/IK 求解器 |
| CtrlStepMotor CAN 接口 | `Robot/actuators/ctrl_step/ctrl_step.cpp` | ✅ 已有 | `SetAngle`/`SetAngleWithVelocityLimit`/`SetEnable`/CAN 0x07/0x23 等 |
| CAN 回包处理 | `UserApp/protocols/can_protocol.cpp:OnCanMessage` | ✅ 已有 | `case 0x23` → `UpdateAngleCallback` → `UpdateJointAnglesCallback` → 更新 `jointsStateFlag` |
| 关节状态机 | `dummy_robot.cpp:UpdateJointAnglesCallback:174-184` | ✅ 已有 | 每轴 `state == FINISH` → 置位 `jointsStateFlag` 第 i 位 |
| DummyHand（7 号手爪）| `dummy_robot.cpp:DummyHand` | ✅ 已有 | ID=7 裸 CAN 发帧（旧固件约定） |
| RGB LED | `Bsp/gpio/rgb.cpp/.hpp` | ✅ 已有 | 基础灯效 |
| OLED SSD1306 | `Bsp/communication/ascii_processor.cpp/.hpp` | ✅ 已有 | u8g2 驱动 |
| EEPROM | `Bsp/memory/emulated_eeprom.cpp` | ✅ 已有 | 节点 ID/参数持久化 |

### 1.3 P0 核心：必须保留的旧固件行为（用户明确诉求）

> **用户原话**（2026-09-23）："最主要的是主控固件的运动是否完成判断，我认为需要退回到 old，覆盖的功能包括但不限于：
> 1. 阻塞模式电机 finish 判断
> 2. 全部电机 finish 后返回 ok
> 3. OLED 屏幕同步刷新每个关节角度和运动状态，用 `_` 和 `*` 区分"

下表把"必须保留的旧行为"与"决策文档中可能冲突的项"对齐：

| # | 必须保留的旧行为 | firmware 当前位置（参考用） | 决策文档中冲突项 | 决策 |
|---|---|---|---|---|
| 1 | `ParseCommand` 中 `>` `&` `@` 命令的 `while (IsMoving() && IsEnabled()) osDelay(5);` 阻塞 + 回 "ok" | `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:887-907`：**v2.8 已改为非阻塞**（检测 IsMoving 直接丢弃新指令回 ok） | §3.5 未明确列出；§0.1 未列入；**用户 P0 明确要求保留阻塞** | ✅ **保留阻塞**（覆盖 §3.5 v2.8 修改） |
| 2 | `IsMoving()` = `jointsStateFlag != 0b1111110`（6 轴判定） | `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:713-718`：**v3.2 已加 `!isEnabled ? false` 短路** | §3.7、§四.4：v3.2 短路修复 | ⚠️ **保留旧公式**（不引入短路 `!isEnabled ? false`），因短路会破坏"全部电机 finish"语义 |
| 3 | `jointsStateFlag` 由 `UpdateAngleCallback` + `UpdateJointAnglesCallback` 同步更新 | 已存在 | §1.2：CAN 0x23 改为单播轮询 14Hz/电机 | ✅ **保留轮询更新机制**（单播/广播细节见 P1-4） |
| 4 | OLED 第 3 行显示 6 轴当前角度（整数°）`>%3d|%3d|%3d|%3d|%3d|%3d` | `firmware/ref_core_f405/UserApp/main.cpp`：**已扩展到 7 轴 + 4 位精度** | §0.3 A1/A2：保留 7 轴架构 | ✅ **保留旧 6 轴整数显示**（不引入小数精度 / 不加第 7 轴） |
| 5 | OLED 第 4 行用 `_`/`*` 显示每轴运动状态 `[SEQ] __*_**` | 同上：仍存在但扩到 8 字符 | — | ✅ **保留 `_`/`*` 语义和 6 字符长度**（不扩到地轨位） |
| 6 | OLED 第 4 行命令模式 `SEQ/INT/TRJ/TUN`（4 模式） | 同上：扩到 6 模式 `SEQ/INT/TRJ/TUN/TRQ/SRV` | §3.6：保留新加 `COMMAND_TORQUE_CONTROL`(5)、`COMMAND_SERVO_J`(6) | ⚠️ **保留旧 4 模式**（不引入 5/6 模式） |
| 7 | 命令模式 `>` `&` `@` 阻塞完成后回 `ok` 到 USB+UART4 | 已存在 | — | ✅ **保留双通道回执** |
| 8 | 6 轴到位判定 mask = `0b1111110`（bit1~6） | `firmware/.../dummy_robot.h:183`：**已改为 `ALL_FINISH_FLAG = 0x7E`**（同样 bit1~6 全 1，bit0/7 = 0） | §3.2：保留 0x7E | ✅ **数值结果相同**（0b1111110 == 0x7E）；保留原命名/字面量形式 |

### 1.4 需要新增/调整的地方（按决策文档 + 用户 P0）

| # | 现状（firmware_new 主控） | 目标（决策 + 用户诉求） | 优先级 |
|---|---|---|---|
| 1 | 6 轴架构，无地轨 | **保留 6 轴**（不引入 7 轴 + 地轨，见 §0.3 A1/A2 → 决策 ✅ 保留新 vs 用户 P0 阻塞语义冲突） | **用户拍板** |
| 2 | 4 种命令模式（SEQ/INT/TRJ/TUN） | **保留 4 模式**（不引入 SERVO_J/TORQUE_CONTROL） | **用户拍板** |
| 3 | TIM7 5kHz 实时环（已存在但未利用） | **保留 5kHz TIM7 中断 + ThreadControlLoopFixUpdate 框架**，但不引入 5kHz 全速下发语义（SEQ/INT/TRJ 维持旧版"实时环每帧 MoveJoints"的节奏） | P1 |
| 4 | `UpdateAngle()` 广播触发 | **保留广播触发**（不引入 14Hz/电机单播轮询） | P0 |
| 5 | 无 0x09 急停广播 | **新增 0x09 急停**（决策 §0.1 #2 + §0.3 D7 ✅） | P0 |
| 6 | `CalibrateHomeOffset()` 存在 | **删除**（决策 §0.2 D1） | P1 |
| 7 | `SetRGBEnable(bool)` / `GetRGBEnabled()` 存在 | **保留**（决策 §0.2 D3 建议删除，但用户未明确，先保留） | **用户拍板** |
| 8 | `SetPositionWithVelocityLimit`（旧名） | **保留旧名**（决策 §0.2 D5 + §0.3 D1 ✅ 保留新名 — 与用户 P0 阻塞语义无关，但函数签名需保持兼容） | P1 |
| 9 | CAN 发送层简单 `osSemaphoreAcquire(osWaitForever)` | **保留旧简单实现**（决策 §0.1 #4 + §1.3 ✅ 保留防卡死重写） | **用户拍板** |
| 10 | 无 `0x23` 单播轮询 | **保留广播触发**（决策 §1.2 D3 ✅ 保留新 vs 用户 P0 阻塞语义冲突） | **用户拍板** |
| 11 | `jointsStateFlag != 0b1111110` | **保留旧公式**（决策 §3.7 v3.2 短路 — 与用户 P0 冲突） | **用户拍板** |
| 12 | `ParseCommand` 阻塞 | **保留阻塞**（决策 §3.5 未明确；用户 P0 ✅） | **P0 保留** |
| 13 | OLED 6 轴 + `_`/`*` | **保留旧显示**（决策 §0.3 A1/A2/B4 — 与用户 P0 冲突） | **P0 保留** |
| 14 | 无 `isStalled` 字段 | **不引入**（决策 §0.1 #5 保留通路但不实现 — 与 §0.1 #1 不实现 stall 一致） | P1 |

---

## 二、任务拆分（按优先级）

> 每个任务说明：做什么 → 改哪些文件 → 怎么验证。

### 【P0-1】保留 OLD 阻塞式运动完成判断（**用户 P0 核心**）

**做什么**：
- `Robot/instances/dummy_robot.cpp` 的 `CommandHandler::ParseCommand` 中：
  - `COMMAND_TARGET_POINT_SEQUENTIAL` 与 `COMMAND_CONTINUES_TRAJECTORY` 的 `>` `&` 分支：保留 OLD 行为 ——
    1. 解析参数后调用 `MoveJ(...)` 写入 targetJoints
    2. **强制立即触发一次 CAN 下发**：`context->MoveJoints(context->targetJoints);`（防止 IsMoving() 因 jointsStateFlag 残留而提前返回 false）
    3. **阻塞等待**：`while (context->IsMoving() && context->IsEnabled()) osDelay(5);`
    4. **全部电机 finish 后**回 `ok` 到 USB+UART4：`Respond(*usbStreamOutputPtr, "ok"); Respond(*uart4StreamOutputPtr, "ok");`
  - `COMMAND_TARGET_POINT_INTERRUPTABLE` 的 `>` `&` 分支：**不阻塞**，解析后立即回 `ok`（保持 OLD 行为，INT 模式天然异步）
  - `>` `&` `@` 在 SEQ/TRJ 模式下的阻塞语义：参考 [`firmware_old/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:347-358`](file:///e:/Dummy-code/firmware_old/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp)
- `IsMoving()` 公式**保留** `jointsStateFlag != 0b1111110`（不引入决策 §3.7 的 `!isEnabled ? false` 短路）
- `dummy_robot.cpp:MoveJ()` 中 **`jointsStateFlag = 0`** 复位保留（每条新指令下发前清零，让 IsMoving() 进入"等待"状态）

**禁止的修改**（避免破坏 OLD 阻塞语义）：
- ❌ **不要**把 ParseCommand 改成 v2.8 非阻塞（`if (IsMoving()) { Respond ok; continue; }`）
- ❌ **不要**在 IsMoving() 中加 `!isEnabled ? false :` 短路
- ❌ **不要**在 `SetEnable(false)` 时改写 `jointsStateFlag`（保持 OLD 的"disable 后电机可能仍在回包，flag 由 0x23 自然更新"）
- ❌ **不要**把 `osDelay(5)` 改成事件驱动（保留简单轮询）

**改哪些文件**：
- `firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp` — `ParseCommand` 三处分支（SEQ/TRJ 的 `>` `&`、`@`）+ `MoveJ`/`ServoJ`/`MoveL` 中 `jointsStateFlag = 0` 复位
- `firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.h` — `IsMoving()` 函数体保持 `jointsStateFlag != 0b1111110`

**验证**：
- 发 `>0,-73,180,0,0,0` 到 USB+UART4，OLED 上能观察到每轴 `_` 逐步变 `*`
- 全部变 `*` 后（约 0.5~3 秒），USB+UART4 同时收到 `ok`
- 发 `&0,-73,180,0,0,0,30`（带 speed），阻塞行为同上
- INT 模式（`#CMDMODE 2` 后）发 `>`，**立即**回 `ok`（不等电机）
- 7 轴全部使能后发 `!DISABLE`，**不要**因 IsMoving() 短路导致后续 SEQ 指令被吞

---

### 【P0-2】保留 OLD OLED 同步刷新（**用户 P0 核心**）

**做什么**：
- `UserApp/main.cpp:ThreadOledUpdate` 中：
  - **第 3 行 6 轴角度显示保留**：`>%3d|%3d|%3d|%3d|%3d|%3d` + `currentJoints.a[0..5]`（整数°）
  - **第 4 行 `_`/`*` 运动状态保留**：`buf[i-1] = (dummy.jointsStateFlag & (1 << i) ? '*' : '_');`（i = 1..6，6 字符）
  - **命令模式名保留 4 模式**：`char cmdModeNames[4][4] = {"SEQ", "INT", "TRJ", "TUN"};`
  - **失能状态显示保留**：`oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], "======");`（disable 时显示 6 个 `=`）
- **不要**新增第 7 轴（地轨）显示（即便未来引入地轨，也保持 6 字符长度不变，避免布局破坏）
- **不要**改用 4 位小数精度（保留 `%3d` 整数°）
- **不要**扩展命令模式名数组到 6 模式

**改哪些文件**：
- `firmware_new/dummy-ref-core-fw/UserApp/main.cpp` — `ThreadOledUpdate` 全部 OLED 绘制代码（约 30 行）

**验证**：
- 启动后 OLED 第 3 行显示 6 轴当前角度（如 `>  0| -73| 180|   0|   0|   0`），刷新率 ≥30Hz
- 运动过程中第 4 行第 i 位在 `_`（运动中）和 `*`（到位）之间切换
- 失能时第 4 行显示 `======`
- 不论命令模式如何切换（SEQ/INT/TRJ/TUN），命令模式名都正确显示

---

### 【P0-3】保留 OLD `jointsStateFlag` 更新机制

**做什么**：
- `Robot/instances/dummy_robot.cpp:UpdateJointAngles()` 保留**广播触发**：`motorJ[ALL]->UpdateAngle();`（1 帧触发 6 电机回包）
- `Robot/instances/dummy_robot.cpp:UpdateJointAnglesCallback()` 保留 **6 轴 bit1~6 更新**：
  ```cpp
  for (int i = 1; i <= 6; i++) {
      currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];
      if (motorJ[i]->state == CtrlStepMotor::FINISH)
          jointsStateFlag |= (1 << i);
      else
          jointsStateFlag &= ~(1 << i);
  }
  ```
- `UserApp/protocols/can_protocol.cpp:OnCanMessage` 的 `case 0x23` 保留现有逻辑：`dummy.motorJ[id]->UpdateAngleCallback(*(float*)(data), data[4]);` + `dummy.UpdateJointAnglesCallback();`

**禁止的修改**：
- ❌ **不要**改为 14Hz/电机单播轮询（决策 §1.2 D3 ✅ 保留新 vs 用户 P0 冲突）
- ❌ **不要**修改 mask 为 `0x7E`（虽然数值上等价于 `0b1111110`，但代码可读性角度保留旧字面量）
- ❌ **不要**在 `case 0x23` 中加 ISR 标志 + 主线程 poll 模式（保留 ISR 内直接处理）

**改哪些文件**：无（现状已符合）

**验证**：
- 发 `>` 指令后，能在 5ms 内观察到 `jointsStateFlag` 各 bit 变化（通过 OLED 第 4 行实时刷新验证）
- 6 电机任一未到位时，`IsMoving()` 返回 true，ParseCommand 阻塞循环继续
- 6 电机全部到位时（CAN 0x23 回包 `data[4]=1`），`IsMoving()` 返回 false，ParseCommand 回 `ok`

---

### 【P0-4】新增 0x09 急停广播（**决策 §0.1 #2 + D7 ✅**）

**做什么**：
- `Robot/actuators/ctrl_step/ctrl_step.cpp` 新增方法 `BroadcastEStop()`：
  ```cpp
  void CtrlStepMotor::BroadcastEStop() {
      uint8_t mode = 0x09;
      txHeader.StdId = nodeID << 7 | mode;  // ALL 代理：motorJ[ALL] 调用时 StdId=0x09
      CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
  }
  ```
- `Robot/instances/dummy_robot.cpp:CommandHandler::EmergencyStop()` 替换 OLD 伪急停：
  ```cpp
  void DummyRobot::CommandHandler::EmergencyStop() {
      context->motorJ[ALL]->BroadcastEStop();  // 1 帧广播
      // 不再调用 MoveJ(currentJoints)（伪急停）
      context->isEnabled = false;
      ClearFifo();
  }
  ```
- `UserApp/protocols/ascii_protocol.cpp` 中 `!STOP` 分支调用 `commandHandler.EmergencyStop()`
- **注意**：旧固件没有 ALL 代理机制（motorJ[ALL] 是 nodeID=0 的虚电机），需要确认 motorJ[ALL] 的 StdId 计算：`0 << 7 | 0x09 = 0x09`，电机端 `id==0` 路由识别为广播

**改哪些文件**：
- `firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.hpp` — 类内声明 `BroadcastEStop()`
- `firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp` — 函数实现
- `firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp` — `EmergencyStop()` 改写
- `firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp` — `!STOP` 精确匹配调用 `commandHandler.EmergencyStop()`

**验证**：
- 发 `!STOP`，6 电机立即抱闸停止（电机端 `SetBrake(true)` + `state=STOP`）
- 发 `!STOP` 后立即发 `>0,-73,180,0,0,0`，指令被 `ClearFifo()` 清空
- 复位后发 `!START`，电机重新使能

---

### 【P0-5】保留 OLD `SetEnable()` 行为（与 P0-1 配套）

**做什么**：
- `Robot/actuators/ctrl_step/ctrl_step.cpp:SetEnable(bool)` 保留 OLD 乐观同步 FINISH：
  ```cpp
  void CtrlStepMotor::SetEnable(bool _enable) {
      state = _enable ? FINISH : STOP;  // OLD: enable 时立即 state=FINISH
      // CAN 帧同 OLD
  }
  ```
- `Robot/instances/dummy_robot.cpp:SetEnable(bool)` 保留 OLD 行为：
  ```cpp
  void DummyRobot::SetEnable(bool _enable) {
      motorJ[ALL]->SetEnable(_enable);
      isEnabled = _enable;
  }
  ```

**禁止的修改**：
- ❌ **不要**移除 `state = FINISH` 乐观同步（决策 §四.3 ✅ 保留新 vs 用户 P0 冲突 — 乐观同步让 IsMoving() 在 enable 后立即返回 false，避免阻塞）
- ❌ **不要**加 `isStalled` 字段（决策 §0.1 #5 保留通路但不引入字段；电机端 stall v3 不实现）

**改哪些文件**：无（现状已符合）

**验证**：
- 发 `!START` 后立即发 `>0,-73,180,0,0,0`，不出现阻塞（IsMoving() 因 enable 时 FINISH 而立即 false，MoveJ 复位 flag 后进入正常等待）
- 发 `!DISABLE` 后，OLED 第 4 行显示 `======`（不依赖 IsMoving）

---

### 【P1-1】删除 `CalibrateHomeOffset()`（决策 §0.2 D1）

**做什么**：
- `Robot/instances/dummy_robot.cpp` 删除 `DummyRobot::CalibrateHomeOffset()` 实现（约 30 行）
- `Robot/instances/dummy_robot.h` 删除 `CalibrateHomeOffset()` 声明
- `Robot/instances/dummy_robot.cpp` `~DummyRobot()` 中若有相关清理代码，一并删除

**改哪些文件**：
- `firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`
- `firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.h`

**验证**：
- `grep -r "CalibrateHomeOffset" firmware_new/dummy-ref-core-fw/` 无结果
- 编译通过

---

### 【P1-2】保留 5kHz 实时环框架（TIM7 + ThreadControlLoopFixUpdate）

**做什么**：
- `UserApp/main.cpp` 保留 TIM7 中断 + `ThreadControlLoopFixUpdate` 任务入口
- **不引入** 5kHz 全速下发 `ServoJ` 逻辑
- **保留**实时环每帧调 `MoveJoints`（5kHz 节拍下发位置指令，符合 OLD 行为）
- **不引入** `ComputeSyncSpeeds` 梯形同步规划（决策 §0.3 B5 ⏳ 用户拍板）

**改哪些文件**：无（现状已符合 OLD）

**验证**：
- TIM7 中断每 200μs 触发一次，`ThreadControlLoopFixUpdate` 被唤醒
- 实时环每帧调用 `MoveJoints(targetJoints)`，5kHz 节拍向电机下发位置
- 不引入 SERVO_J 模式（`#CMDMODE 6` 应回 `error` 或忽略）

---

### 【P1-3】保留 `SetPositionWithVelocityLimit` 旧名（决策 §0.3 D1 ✅）

**做什么**：
- `Robot/actuators/ctrl_step/ctrl_step.cpp` 保留旧方法名 `SetPositionWithVelocityLimit(float, float)`，CAN StdId `0x07`
- **不重命名**为 `SetPositionWithMotorRps`（决策 §0.2 D5 删除 + §0.3 D1 保留新名，但用户未要求重命名，先保留旧名降低风险）
- **如未来要重命名**，需同步更新 `dummy_robot.cpp:MoveJoints` 中所有调用点（6 处）

**改哪些文件**：无（现状已符合）

**验证**：
- `grep -r "SetPositionWithVelocityLimit" firmware_new/dummy-ref-core-fw/` 命中 ≥ 1 处
- 发 `MoveJ` 指令后，电机按 velocity limit 减速到位

---

### 【P1-4】保留 `UpdateAngle()` 广播触发（决策 §1.2 D3 ✅ 保留新 vs 用户 P0）

**做什么**：
- `Robot/actuators/ctrl_step/ctrl_step.cpp:UpdateAngle()` 保留 OLD 行为：StdId = `(nodeID<<7)|0x23`，1 帧触发所有电机回包
- `Robot/instances/dummy_robot.cpp:UpdateJointAngles()` 保留 OLD 行为：`motorJ[ALL]->UpdateAngle();`（1 帧触发 6 电机回包）

**禁止的修改**：
- ❌ **不要**改为 14Hz/电机单播轮询（firmware 的 v2.6 重构）
- ❌ **不要**在 `can_protocol.cpp:case 0x23` 加 `id` 分流处理（1~6/7/8 三段）

**改哪些文件**：无（现状已符合）

**验证**：
- 发 `>...` 后，1 帧 CAN 0x23 广播，6 电机并发回包
- 6 电机回包到齐时间 ≤20ms（典型 ≤5ms）

---

### 【P2-1】（用户拍板）CAN 发送层防卡死重写（决策 §0.1 #4 + §1.3 ✅）

**待用户决定**：
- **选项 A**：保留 OLD 简单 `osSemaphoreAcquire(osWaitForever)`（风险：mailbox 满时永久阻塞）
- **选项 B**：采纳 firmware 的 mailbox 空闲预检 + 10ms 超时 + 失败释放（决策 ✅ 保留）

**说明**：与用户 P0 阻塞语义**无直接冲突**（CAN 发送是底层细节，不影响 ParseCommand 的 `while(IsMoving)`）。可独立决策。

---

### 【P2-2】（用户拍板）`SetRGBEnable(bool)` / `GetRGBEnabled()` 是否删除（决策 §0.2 D3 ❌）

**待用户决定**：
- **选项 A**：删除（决策 ✅），改用 3 个灯效预设状态码
- **选项 B**：保留（当前实现），降低改动风险

---

### 【P2-3】（用户拍板）是否引入 7 轴架构（决策 §0.3 A1/A2 ✅）

**待用户决定**：
- **选项 A**：保持 6 轴（不引入地轨，与用户 P0 OLED 显示直接兼容）
- **选项 B**：引入 7 轴 + 地轨（需修改 OLED 显示、IsMoving mask 为 `0x7E`、MoveJ 入参等 — 与用户 P0 多个项冲突）

---

### 【P2-4】（用户拍板）是否引入 SERVO_J / TORQUE_CONTROL 模式（决策 §0.3 B2/B3/C9 ✅）

**待用户决定**：
- **选项 A**：保持 4 模式（SEQ/INT/TRJ/TUN）
- **选项 B**：扩展到 6 模式（新增 SERVO_J + TORQUE_CONTROL + `$` 力矩指令）

---

### 【P2-5】（用户拍板）`ComputeSyncSpeeds` 梯形同步规划（决策 §0.3 B5 ⏳）

**待用户决定**：
- **选项 A**：保持 OLD `MoveJ` 中简单的 `dynamicJointSpeeds` 计算
- **选项 B**：引入 firmware 的 7 轴梯形同步规划

---

## 三、实施顺序建议

```
第 1 步（P0-1）保留 OLD 阻塞式运动完成判断 — 验证：发 >，等 6 个 * 后回 ok
第 2 步（P0-2）保留 OLD OLED 同步刷新 — 验证：6 轴角度 + _/* 实时刷新
第 3 步（P0-3）保留 OLD jointsStateFlag 更新 — 验证：广播 0x23 后 5ms 内 flag 更新
第 4 步（P0-4）新增 0x09 急停广播 — 验证：!STOP 后 6 电机抱闸
第 5 步（P0-5）保留 OLD SetEnable 乐观同步 FINISH — 验证：!START 后立即可发 >
第 6 步（P1-1）删除 CalibrateHomeOffset — 验证：grep 无残留
第 7 步（P1-2）保留 5kHz 实时环框架 — 验证：TIM7 中断频率 5kHz
第 8 步（P1-3）保留 SetPositionWithVelocityLimit 旧名 — 验证：grep 命中
第 9 步（P1-4）保留 UpdateAngle 广播触发 — 验证：1 帧触发 6 电机回包
第 10 步（用户拍板）：P2-1 ~ P2-5 按用户决策逐项实施
```

> **关键纪律**：
> - **P0-1 ~ P0-3 是用户 P0 诉求**，实施时**禁止 cherry-pick** `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp` 中 v2.8/v3.2 的修改代码（v2.8 非阻塞 ParseCommand、v3.2 IsMoving 短路）
> - **P0-4 唯一允许参考** firmware 的 0x09 急停实现（决策 ✅ 保留新）
> - P1-1 ~ P1-4 全部"保留现状"，不需要写新代码（只需验证现状符合）
> - P2-1 ~ P2-5 等用户拍板后再实施

---

## 四、不做的事情（明确禁止）

- ❌ **不要**把 ParseCommand 改为 v2.8 非阻塞（破坏用户 P0 阻塞语义）
- ❌ **不要**在 IsMoving() 中加 `!isEnabled ? false` 短路（破坏用户 P0 全部电机 finish 判定）
- ❌ **不要**改用单播轮询 UpdateAngle（决策 D3 ✅ 保留新 vs 用户 P0 — 不采纳）
- ❌ **不要**扩展 OLED 到 7 轴或小数精度（破坏用户 P0 OLED 显示）
- ❌ **不要**扩展 CommandMode 到 6 模式（保留 4 模式）
- ❌ **不要**引入 SERVO_J / TORQUE_CONTROL（决策 B2/B3 ✅ 保留新 vs 用户未要求，先不引入）
- ❌ **不要**引入 `!BACKHOME` 状态机（决策 #1 不实现）
- ❌ **不要**实现堵转检测算法（决策 §0.1 #1 — 不引入 `isStalled` 字段、0x5A/0x5B/0x5C 通路）
- ❌ **不要**引入 `!STALL_*` ASCII 命令（虽然电机端通路保留，但主控 ASCII 命令先不暴露）
- ❌ **不要**在 0x21~0x2D 实现新的查询命令（主控侧查询命令保持现状，电机端命令通路见电机固件实施文档）
- ❌ **不要**修改 DummyHand 为 StepHand 继承 CtrlStepMotor（保留 OLD 裸 CAN 发帧）

---

## 五、验收标准

| # | 验收条件 | 检查方法 |
|---|---|---|
| 1 | **P0-1 阻塞完成回 ok**：发 `>0,-73,180,0,0,0`，6 电机全部到位后 USB+UART4 同时收到 `ok` | 串口观察：发指令 → OLED 第 4 行 6 个 `*` → 回 `ok` |
| 2 | **P0-1 INT 模式异步**：发 `#CMDMODE 2` 后发 `>0,0,0,0,0,0`，**立即**回 `ok`（不等电机） | 串口观察：发指令 → 立即回 `ok`（运动中） |
| 3 | **P0-2 OLED 角度**：OLED 第 3 行实时显示 6 轴整数°角度，刷新率 ≥30Hz | 视觉观察 OLED |
| 4 | **P0-2 OLED 状态**：OLED 第 4 行 6 字符 `_`/`*` 实时切换（`_`=运动中，`*`=到位） | 视觉观察 OLED |
| 5 | **P0-2 OLED 命令模式**：OLED 第 4 行显示 `SEQ`/`INT`/`TRJ`/`TUN` 之一 | `#CMDMODE 1/2/3/4` 切换观察 |
| 6 | **P0-2 OLED 失能**：发 `!DISABLE` 后 OLED 第 4 行显示 `======` | 串口+视觉 |
| 7 | **P0-3 jointsStateFlag**：发 `>...` 后 5ms 内 `jointsStateFlag` 各 bit 置位（OLED 同步反映） | OLED 观察 + 调试串口 printf flag 值 |
| 8 | **P0-4 急停**：发 `!STOP` 后所有电机立即抱闸（电机端 `SetBrake(true)`），主控 `isEnabled=false` | 实测电机响应 |
| 9 | **P0-5 SetEnable 乐观同步**：发 `!START` 后立即发 `>...`，指令不阻塞（IsMoving 因 state=FINISH 而 false） | 串口观察 `ok` 立即返回 |
| 10 | **P1-1 CalibrateHomeOffset 删除**：`grep -r "CalibrateHomeOffset" firmware_new/dummy-ref-core-fw/` 无结果 | grep 命令 |
| 11 | **P1-2 5kHz 实时环**：TIM7 中断频率 200μs，`ThreadControlLoopFixUpdate` 每帧被唤醒 | 调试串口打印实时环执行次数 |
| 12 | **P1-3 SetPositionWithVelocityLimit 旧名保留**：`grep -r "SetPositionWithVelocityLimit"` 命中 ≥ 1 处 | grep 命令 |
| 13 | **P1-4 UpdateAngle 广播触发**：`UpdateAngle()` 单帧触发 6 电机回包（实测 ≤20ms 全到位） | 示波器观察 CAN 总线 |
| 14 | **不引入项检查**：`firmware_new/dummy-ref-core-fw/` 中**不应**出现 `isStalled` / `0x5A` / `0x5B` / `0x5C` / `COMMAND_TORQUE_CONTROL` / `COMMAND_SERVO_J` / `ComputeSyncSpeeds` / `!BACKHOME` / `CalibrateHomeOffset` | grep 命令验证全无 |

---

## 六、参考文档

| 文档 | 内容 |
|---|---|
| [`firmware_new主控固件决策.md`](./firmware_new主控固件决策.md) | 主控固件所有决策的源头（§0 已决策 / §0.3 待决策） |
| [`重构需求—firmware_old迁移到firmware功能清单—2026-09-19.md`](./重构需求—firmware_old迁移到firmware功能清单—2026-09-19.md) | §1.2 主控固件需求清单（与本实施文档一一对应） |
| [`重建需求草稿—firmware_new—2026-09-21.md`](./重建需求草稿—firmware_new—2026-09-21.md) | 总体需求草稿 |
| [`firmware_new电机固件实施.md`](./firmware_new电机固件实施.md) | 电机固件实施文档（实施时同步） |
| `firmware_old/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp` | **OLD 主控源文件** — 阻塞 ParseCommand / UpdateJointAnglesCallback / IsMoving 公式 |
| `firmware_old/dummy-ref-core-fw/UserApp/main.cpp` | **OLD 主控入口** — ThreadOledUpdate OLED 6 轴 + `_`/`*` 显示 |
| `firmware_old/dummy-ref-core-fw/UserApp/protocols/can_protocol.cpp` | **OLD CAN 回包** — `case 0x23` → UpdateAngleCallback + UpdateJointAnglesCallback |
| `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp` | **参考**（只看不抄）：v2.8 非阻塞 ParseCommand（**禁止**借鉴）、v3.2 IsMoving 短路（**禁止**借鉴）、0x09 急停（**允许**借鉴） |
| `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` | **参考**（只看不抄）：`BroadcastEStop()` 实现（**允许**借鉴） |
| `firmware/ref_core_f405/UserApp/main.cpp` | **参考**（只看不抄）：扩展的 OLED 显示（**禁止**借鉴） |
