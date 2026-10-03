# handcode 夹爪实现 → firmware_new 迁移对比与实施文档

> 编制日期：2026-10-03
> 编制人：Cursor Agent（用户授权任务）
> 涉及目录：`e:\Dummy-code\handcode`（参考实现 A：功能语义）、`e:\Dummy-code\firmware\ref_core_f405`（参考实现 B：API 与默认值）、`e:\Dummy-code\firmware_new`（当前开发，目标仓）、`e:\Dummy-code\串口助手_old.py`（目标上位机）
> 文档定位：把 `handcode` + `firmware/ref_core_f405` 的夹爪功能与 `firmware_new` 当前态做逐项对比，给出把夹爪完整功能迁移到 `firmware_new` 的实施步骤 + 同步更新 `串口助手_old.py` 的方案。
> ref指firmware\ref_core_f405

---

## 修订

6. **电流方向与 ref 对齐**：`!HAND_O` → `hand->SetCurrentSetPoint(-current)`（张开方向电流，对齐 ref 的 `SetAngleWithCurrentLimit(-1)=张开`，`firmware/ref_core_f405/Robot/instances/dummy_robot.h:76-160` 的 `StepHand` 类注释"+1=闭合施力"）；`!HAND_C` → `hand->SetCurrentSetPoint(+current)`（闭合方向电流）。`StepHand` 构造 `inverse=false` → `inverse=true`（对齐 ref 第 130 行 `CtrlStepMotor(hcan, id, true, 16, -100, 100)`），基类 `SetPositionWithVelocityLimit` 自动取反。

> ⚠️ **handcode `!HAND_O/C` 物理语义反着写**：handcode 的 `!HAND_O` 调 `SetAngleWithCurrentLimit(+1)`、`!HAND_C` 调 `SetAngleWithCurrentLimit(-1)`，**与 ref "+1=闭合施力" 注释对得上但函数名错配**（ref `-1=张开`、handcode 把 `+1` 标成"open"）。handcode 实际是 bug 状态——handcode 文件 `handcode/ascii_protocol.cpp:36,42`（`!HAND_O/C` 那两段）。本期方案**不沿用** handcode 的 `!HAND_O/C` 语义，按 ref 注释"-1=张开施力、+1=闭合施力"重新定义。
7. **`!HAND_I` 上限定 2.5A**（用户 2026-10-03 决策）：先定 2.5A，后续按实机情况调整。语义为**主控电流命令值的安全上限**（防止用户误操作发超出电机安全能力的电流），**与电机端 EEPROM currentLimit 不是一回事**——前者是主控发的电流命令值上限，后者是电机堵转保护上限。
8. **UART4 配置类禁止时改回 `error forbidden on UART4`**：原方案"落到默认 ok"会让上位机卡死等待；改为 fallback 回 `error forbidden on UART4`（不让上位机超时）。
9. **reduction 纠错→对齐**：去掉"handcode 的 8 是误植"的表述，改为"handcode 的 reduction 字段不影响实际行为（handcode 跳过 reduction 归一化），本期与 ref 对齐采用 16"。
10. **【阻塞 1】`can_protocol.cpp` 回包路由必须配套改**：`firmware_new/dummy-ref-core-fw/UserApp/protocols/can_protocol.cpp:23` 当前硬写 `dummy.motorJ[id]->UpdateAngleCallback(...)`，`id=8`（夹爪）会让 `motorJ[8]`（数组越界 nullptr）触发 HardFault。本期必须分流为 `id==8 → dummy.hand->UpdateAngleCallback(...)`；否则 Step 2 的 `StepHand` 切换会让首帧 `cmd=0x23` 角度回包直接崩。
11. **【阻塞 2】夹爪端 `cmd=0x01, data=1` 后清电流 → 改为「先实测后定」**：`forceSetCurrentSetPoint(0)` 改为**先实测后定**——理论上关节电机 `velocitySetPoint` 默认 0 不动即可；夹爪电流残值（`!HAND_O` 上一帧发过）会**可能**触发"使能瞬间以 -X A 张开"但**风险依赖 gripper 端 controller.cpp 实现细节**（`MODE_COMMAND_VELOCITY` 模式下是否会 fallback 到 CURRENT 模式）。本期**不直接加补丁**，先做 Step 6 实测清单验证（`!HAND_O -1.0A` 后 `!DISABLE !START` 看夹爪是否微动）；如确有微动再加 `firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:35-38` 的 `case 0x01` `data=1` 分支用 `#if MOTOR_TYPE == 8` 包一行 `motor.controller->SetCurrentSetPoint(0)`，关节端不动。
12. **【阻塞 3】UART4 `!` 分支必须加 else fallback**：当前 `OnUart4AsciiCmd` 的 `!` 分支只看 `STOP/START/HOME/CALIBRATION/RESET/DISABLE`，其余 `!HAND_*` 不识别时无任何回包，上位机卡死。Step 3 必须明示在 `!` 分支末尾加 `else if (s.substr(0, 5) == "!HAND") Respond(_responseChannel, "error forbidden on UART4");`（注意 `!HAND_DIS` 包含 "HAND_O" 子串，必须用顺序匹配 + `s.rfind("HAND_O ") == 1 || s == "!HAND_O"` 这类带尾空格判别，避免 `!HAND_O` 被 `!HAND_DIS` 抢先匹配）。
13. **【阻塞 4】删除 `hand2` 注释残留 → 改为「误判，无需处理」**：原文档说"`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:42` 构造区附近残留 ref/handcode 的 `// hand2 = new CtrlStepMotor(_hcan, 7, false, 10, -100, 100);` 和 `// hand2 = new StepHand(_hcan, 7);` 两行注释"。**实际核实**：这两行注释只在 `handcode/dummy_robot.cpp:35,37`（handcode 的临时类命名），**firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp` 里没有**。handcode 目录不在本期迁移工作区，无需处理。**阻塞 4 误报，从必做清单移除**。
14. **【R1 决策】`MakeProtocolDefinitions` 中 `set_enable` 必须用 ref 的强转写法**：避免 fibre 库继承二义性。Step 2 的 4 条 fibre 注册里第 2 条 `set_enable` 不能写 `*this, &StepHand::SetEnable`（不 override 编译过但与 ref 偏离），而写法是 `static_cast<CtrlStepMotor&>(*this), &CtrlStepMotor::SetEnable, "enable"`（对齐 `firmware/ref_core_f405/Robot/instances/dummy_robot.h:128-130`，StepHand 的 `MakeProtocolDefinitions` 第 1 条）。
15. **【R2 决策】不 override `UpdateAngleCallback`（采纳 R1/R2 反致，2026-10-09 评审通过）**：本期决策改为 **override 仅 2 个方法**（`SetAngle` + `SetAngleWithVelocityLimit`），不 override `UpdateAngleCallback`（与 ref 一致；本期确认不查 `hand->angle`；避免增加调试面；ref 的 `StepHand` 也不 override）。`hand->angle` 字段保留为基类物理含义"电机轴角度 × 360 / reduction"，**未来真要读时**再补 override，本期不动。
16. **【R3 决策】`!HAND_ACC` 运行时不持久化，明示给使用者**：`!HAND_ACC` 走 `CtrlStepMotor::SetAcceleration`，`firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp:174-186` 写入时 `canBuf[4]=0`（**不**写 EEPROM），下次 reboot 就丢。这是与关节 `SetJointAcceleration` 一致的行为，**但使用者可能误以为"配置类 = 持久化"**，文档必须明示。Step 3 / §1 / 决策表都需补一行说明"运行时不持久化"。同理 `!HAND_VEL / !HAND_K* / !HAND_ONBOOT / !HAND_ERASE` 中，`!HAND_ONBOOT` 和 `!HAND_ERASE` 走基类 `SetEnableOnBoot/EraseConfigs`（ctrl_step.cpp 第 198 行起、第 246 行起）写 EEPROM；其余 `VEL/ACC/KP/KV/KI/KD` 不写。Step 3 表格补列"持久化？"。

> **修订段注**：原 #8 "UpdateAngleCallback 也 override" 已删除（2026-10-09 R2 评审定调 override 仅 2 个方法）；原 #9~#17 顺次前移为 #8~#16。

## 0. 文档结论先行

1. **架构改为继承式**：`firmware_new` 把 `DummyHand` 替换为 `StepHand : public CtrlStepMotor`（类声明放 `dummy_robot.h`），`hand` 指针保留命名（与 ref 一致 收敛到 `StepHand* hand`）。继承 22 个 C++ 方法免重写，但 `MakeProtocolDefinitions` **只**注册 4 条 fibre，其余 18 条由 ASCII 协议层直接调基类方法。
   - 用户 2026-10-03 验证"11 层 fibre 嵌套 boot 仍不稳定"；4 条 fibre 是已确认的安全边界
   - **必须 override 2 个方法**（R2 决策，2026-10-09 修订）：
     - `SetAngle(float _angle)` 跳过 reduction 归一化（reduction=16 物理减速比，但夹爪 `_angle=0~100` 期望直接对应电机圈数 0~100）→ 直接调基类 `SetPositionWithVelocityLimit(_angle, 30.0f)`
     - `SetAngleWithVelocityLimit(float _angle, float _vel)` 同 `SetAngle` 跳过归一化 → 直接调基类 `SetPositionWithVelocityLimit(_angle, _vel)`
     - **不 override `UpdateAngleCallback`**——ref 的 `StepHand` 也不 override（`firmware/ref_core_f405/Robot/instances/dummy_robot.h:76-160` 的 `StepHand` 类）；本期确认不查 `hand->angle`，避免增加调试面；`hand->angle` 字段保留基类物理含义（电机轴角度 × 360 / reduction）；未来真要读再补 override
   - **不 override** `SetEnable(bool)`：调基类 `CtrlStepMotor::SetEnable` → 发 `cmd=0x01` 让电机进 `MODE_COMMAND_VELOCITY` 待命；与 handcode 行为一致；使能后靠 `!HAND_O/C/POS` 主动驱动
   - **reduction=16**：真实减速比，物理意义直观；override `SetAngle` 后 `_angle=50 → 50 圈电机轴`
   - **【R1 强制要求】`MakeProtocolDefinitions` 中 `set_enable` 必须用 ref 的强转写法**：`make_protocol_function("set_enable", static_cast<CtrlStepMotor&>(*this), &CtrlStepMotor::SetEnable, "enable")`（对齐 `firmware/ref_core_f405/Robot/instances/dummy_robot.h:128-130` StepHand 的 `MakeProtocolDefinitions` 第 1 条）。直接写 `*this, &StepHand::SetEnable` 因 StepHand 不 override 该方法，编译能过但与 ref 偏离，且 fibre 库对继承类成员指针解析有路径二义性风险——已确认采用 ref 的强转写法是安全的
2. **物理参数**：reduction = **16**（与 `firmware/ref_core_f405/Robot/instances/dummy_robot.h:82-83` 对齐；handcode 的 8 是 `CtrlStepMotor` 构造形参 `handcode/dummy_robot.h:70`，与实际减速比无关——handcode 的 `SetAngleWithSpeedLimit` 走的是 `SetAngleWithVelocityLimit(ag, 70)` 归一化路径，reduction 字段被架空；**注意**：文档早期误写为"handcode 送 `SetAngleWithMotorRps`"，那是 ref 的实现）；`HAND_MAX_RPS = **30**`（统一到 30，与电机端 EEPROM velocityLimit 默认对齐）；`!HAND_I` 上限 **2.5A**（用户 2026-10-03 决策：先定 2.5A，后续按实机调整；**语义为主控电流命令值的安全上限，与 EEPROM currentLimit 是两回事**——前者是主控发的电流命令值上限，后者是电机堵转保护上限）。
3. **命令映射**：
   - **运行时（UART4 允许）**：`!HAND_O` / `!HAND_C` / `!HAND_POS`（3 条）
   - **配置（UART4 禁止）**：`!HAND_I` / `!HAND_VEL` / `!HAND_ACC` / `!HAND_KP` / `!HAND_KV` / `!HAND_KI` / `!HAND_KD` / `!HAND_ONBOOT` / `!HAND_ERASE`
   - **不暴露**：`!HAND_EN` / `!HAND_DIS` / `!HAND_ZERO`（`EN/DIS` 删除统一 `!START/!DISABLE`；`ZERO` 不识别、不占位）
4. **`!HAND_O/C` 用电流透传**（与 ref 一致；handcode 也是电流透传）。`!HAND_O` → `hand->SetCurrentSetPoint(-current)`（张开方向电流，对齐 ref 的 `SetAngleWithCurrentLimit(-1)=张开`）；`!HAND_C` → `hand->SetCurrentSetPoint(+current)`（闭合方向电流，对齐 ref 的 `SetAngleWithCurrentLimit(+1)=闭合`）。**位置控制由 `!HAND_POS 0/100` 覆盖**。`StepHand` 构造 `inverse=true`（对齐 ref），基类 `SetPositionWithVelocityLimit` 自动取反；电流透传走 `cmd=0x03`（基类 `SetCurrentSetPoint` 不取反，**电流正方向 = 电机正方向 = 闭合方向**——这是物理约定，配合 ref 注释"+1=闭合施力"）。
5. **`!HAND_I` 行为（Q1=A）**：仅改 `hand->current` 这个运行时变量，**不**发任何 CAN 帧。默认 `current = 1.0f`（**本期决策值**，不沿用 ref 1.2f `firmware/ref_core_f405/Robot/instances/dummy_robot.h:91` 也不沿用 handcode 0.7f `handcode/dummy_robot.h:65`）；用户 `!HAND_I 2.5` → `hand->current = 2.5f`（掉电丢；下次 `!HAND_O/C` 用这个值）。**上限 2.5A 是主控电流命令值的安全上限**（防止误操作），不是 EEPROM 电流上限；EEPROM 电流上限走 `CtrlStepMotor::SetCurrentLimit`（基类），由 `#I_LIMIT_J 8` ASCII 入口暴露，不在 `!HAND_I` 范围内。
6. **PID 调参范围统一 `0~65535`**（不再细分 kv/kp 范围）。
7. **`串口助手.py` 不修改**；**`串口助手_old.py` 是目标上位机，需配合本期固件改动做 UI 重构**——**当前 0 个夹爪 UI 控件，需从零新建**（开/闭 + 电流设置 + 位置设定三个面板），不只"放宽电流范围"

`handcode` 还有个 `DummyHand` 类（直接拼 CAN 帧）—— `firmware_new` 已完全替代（`DummyHand` 重写为 `cmd=0x07/0x12/0x7F` 等电机端标准命令），**不**要复刻。**`DummyHand` 类整体删除**，由 `StepHand` 继承式取代。

---

## 1. 夹爪加入功能清单

| 类别 | 命令 | 实现方法 | 备注 | 持久化？ |
|---|---|---|---|---|
| **位置** | `!HAND_O` | `hand->SetCurrentSetPoint(-current)` | 用 ref/handcode 电流透传（开=负电流，对齐 ref） | ❌ 不写 EEPROM |
| **位置** | `!HAND_C` | `hand->SetCurrentSetPoint(+current)` | 用 ref/handcode 电流透传（闭=正电流，对齐 ref） | ❌ 不写 EEPROM |
| **位置** | `!HAND_POS <0-100>` | `hand->SetAngle(pos)` 或 `hand->SetAngleWithVelocityLimit(pos, 30)`（两个 override 都跳过归一化，pos 直接 = 电机圈数）→ 内部调基类 `SetPositionWithVelocityLimit(pos, vel)` 发 `cmd=0x07`（基类按 `inverse=true` 自动取反） | 限速 30 r/s | ❌ 不写 EEPROM |
| **开关** | `!HAND_EN` | `hand->SetEnable(true)` | **本期删除**，统一到 `!START`（夹爪随 ALL 广播一起进 MODE_COMMAND_VELOCITY 待命，Q4=A） | n/a（不实现） |
| **开关** | `!HAND_DIS` | `hand->SetEnable(false)` | **本期删除**，统一到 `!DISABLE`（夹爪随 ALL 广播一起进 MODE_STOP） | n/a（不实现） |
| **配置** | `!HAND_I <A>` | **仅改运行时 `hand->current` 成员**，不发 CAN 帧（Q1=A） | 范围 0~2.5A（主控电流命令值安全上限，后续按实机调整）；掉电丢 | ❌ 不写 EEPROM（纯 RAM 变量） |
| **配置** | `!HAND_VEL <rps>` | `hand->SetVelocityLimit(_val)` → `cmd=0x13`，`canBuf[4]=1` | EEPROM 写 | ✅ 写 EEPROM |
| **配置** | `!HAND_ACC <rps²>` | `hand->SetAcceleration(_val)` → `cmd=0x14`，`canBuf[4]=0` | 运行时加速度 | ❌ **不**写 EEPROM（R3 明示，调一次丢一次） |
| **配置** | `!HAND_KP <v>` | `hand->SetDceKp(_val)` → `cmd=0x17`，`canBuf[4]=1` | EEPROM 写 | ✅ 写 EEPROM |
| **配置** | `!HAND_KV <v>` | `hand->SetDceKv(_val)` → `cmd=0x18`，`canBuf[4]=1` | EEPROM 写 | ✅ 写 EEPROM |
| **配置** | `!HAND_KI <v>` | `hand->SetDceKi(_val)` → `cmd=0x19`，`canBuf[4]=1` | EEPROM 写 | ✅ 写 EEPROM |
| **配置** | `!HAND_KD <v>` | `hand->SetDceKd(_val)` → `cmd=0x1A`，`canBuf[4]=1` | EEPROM 写 | ✅ 写 EEPROM |
| **配置** | `!HAND_ONBOOT <0/1>` | `hand->SetEnableOnBoot(_enable)` → `cmd=0x16`，`canBuf[4]=1` | EEPROM 写 | ✅ 写 EEPROM |
| **配置** | `!HAND_ERASE` | `hand->EraseConfigs()` → `cmd=0x7E` | 触发 `CONFIG_RESTORE` 恢复出厂 | ✅ 写 EEPROM（恢复出厂） |
| **配置** | `!HAND_ZERO` | **不实现、不识别、不占位**（Q11=A） | 落到 fallback 响应；统一进 `!CALIBRATION` 后再决定 | n/a（不实现） |

> **R3 持久化速查**（与 §4 决策 #22 对齐）：
> - **写 EEPROM**：`!HAND_VEL / KP / KV / KI / KD / ONBOOT / ERASE`（基类 `canBuf[4]=1` 或命令自身语义）—— 调一次持久保留
> - **不写 EEPROM**：`!HAND_O / C / POS / I / ACC`（基类 `canBuf[4]=0` 或纯 RAM 变量）—— 调一次掉电丢
> - **不实现**：`!HAND_EN / DIS / ZERO`
>
> 与关节行为对照：关节 `SetJointAcceleration` 也不写 EEPROM；`!JOINT_PID ` 走通用 `#SET_DCE_*` 才写。本期夹爪的 `!HAND_K*` 与关节 `#SET_DCE_*` 等价（都写 EEPROM）。 

## 2. 附加提示

### 2 ref 的 `#REBOOT` 不支持 node=8

**本期决策**：扩到 `node <= 8`，加 `node == 8` 分支调 `dummy.hand->Reboot()`；**串口助手old的 `#REBOOT` 节点下拉框需从 0~7 扩展为 0~8（当前状态：combobox 值列表为 `range(0,8)` 即 0~7，缺 8）**

---

## 3. 核心差异对比（handcode vs firmware/ref_core_f405 vs firmware_new）

### 3.1 夹爪的 CAN 节点 ID

| 项目 | handcode | firmware/ref_core_f405 | firmware_new |
|---|---|---|---|
| 夹爪 nodeID | **7** | **8** | **8**（`dummy_robot.h` 字段类型 Step 2 切换；当前 `firmware_new/.../dummy_robot.h:155` 字段声明 + `dummy_robot.cpp:125` 构造） |
| 地轨 | （不存在） | **7**（`motorJ[7]`） | **7**（`firmware_new/.../dummy_robot.cpp:124`，构造行；`motorJ[7] = new CtrlStepMotor(_hcan, 7, true, 1, -250, 250)`） |
| 原因 | handcode 没有地轨概念 | ref 已加入地轨 | 与 ref 对齐 |

### 3.2 夹爪的 C++ 抽象

| 项目 | handcode | firmware/ref_core_f405 | firmware_new |
|---|---|---|---|
| 类 | `StepHand`（继承 `CtrlStepMotor`） | `StepHand`（继承 `CtrlStepMotor`） | **`StepHand`（继承 `CtrlStepMotor`，打法 B）** |
| 继承链深度 | 1 层（直接继承） | 1 层 | 1 层 |
| 位置命令 | `SetAngleWithVelocityLimit(ClosedAngle + pos, 70)` | `SetAngleWithMotorRps(target, HAND_MAX_RPS=20)` | `StepHand::SetAngle(_angle)` **override** 跳过归一化 → 调基类 `CtrlStepMotor::SetPositionWithVelocityLimit(_angle, vel)` 发 `cmd=0x07` |
| 电流命令 | `SetCurrentSetPoint(inverse * current)` | `SetCurrentSetPoint(inverse * current)` | **直接调基类** `CtrlStepMotor::SetCurrentSetPoint(±current)` 发 `cmd=0x03` |
| 限速默认 | 70 r/s 硬编码 | 20 r/s（`HAND_MAX_RPS`） | **30 r/s**（v3 用户决策统一到 30） |
| reduction（构造形参） | 8 | **16** | **16**（A2 决策，物理意义直观） |
| 真实减速比 | 16 | 16 | 16 |
| 角度范围 | `_angle ∈ [-100, 100]`（电机圈数，`handcode/dummy_robot.h:70` 构造形参） | `_angle ∈ [-100, 100]`（电机圈数，`firmware/ref_core_f405/Robot/instances/dummy_robot.h:82-83` 构造形参） | `_angle ∈ [0, 100]`（开度 0~100%，override 后直接对应电机圈数 0~100） |
| PID 调参入口 | 无 | 走 `CtrlStepMotor::SetDceK*` 21 条接口 | **直调基类** `CtrlStepMotor::SetDceKp/Kv/Ki/Kd` + ASCII `!HAND_K*` + 通用 `#SET_DCE_* 8` |
| `SetEnable` 行为 | `cmd=0x01, data=1`（基类，进速度模式待命） | `cmd=0x01, data=1`（基类，进速度模式待命） | **不 override**，调基类 `CtrlStepMotor::SetEnable` 发 `cmd=0x01` 进 `MODE_COMMAND_VELOCITY` 待命；与 handcode 一致；使能后由 `!HAND_O/C/POS` 主动驱动 |
| override 方法数 | 0（仅继承） | 0（仅继承 + MakeProtocolDefinitions） | **2 个**（R2 决策）：`SetAngle` / `SetAngleWithVelocityLimit` 跳过 reduction；**不** override `UpdateAngleCallback`（与 ref 一致） |
| fibre 注册条目数 | n/a（不暴露 fibre） | n/a | **4 条**：`set_angle` / `set_enable`（**R1 强转写法** `static_cast<CtrlStepMotor&>(*this)`） / `set_current` / `set_current_limit`（全部走 StepHand override 或基类方法；不含 reboot 走 ALL） |

### 3.3 夹爪的减速比

| 来源 | 声称 reduction | 真实减速比 |
|---|---|---|
| handcode `dummy_robot.h:70` | 8 | **16**（`handcode` 跳过归一化所以不影响行为）|
| firmware/ref_core_f405 `dummy_robot.h:82-83` | **16** | 16 |
| firmware_new `dummy_robot.h` | n/a | **16**（本期约定） |

> **对齐说明**（2026-10-03 修订）：handcode 的 `CtrlStepMotor(hcan, id, false, 8, -100, 100)`（`handcode/dummy_robot.h:70`）与 ref 的 `CtrlStepMotor(hcan, id, true, 16, -100, 100)`（`firmware/ref_core_f405/Robot/instances/dummy_robot.h:82-83`）在 reduction 字段上不同，但**不影响实际行为**——handcode 的 `StepHand::SetAngleWithSpeedLimit(_angle)` 走的是 `SetAngleWithVelocityLimit(ag, 70)`（`handcode/dummy_robot.h:73-76`），其中 `ag = ClosedAngle + _angle` 是归一化路径，**不是**文档之前所写的"直接送 `SetAngleWithMotorRps`"（那是 ref 的实现 `firmware/ref_core_f405/Robot/instances/dummy_robot.h:103-107`）。本期与 ref 对齐采用 `reduction=16`（物理减速比 16:1）。
>
> 真实减速比 16 的物理含义：电机轴转 16 圈 → 夹爪直线位移满量程；速度 **30 r/s**→ 输出轴 30/16 ≈ 1.875 r/s。

### 3.4 夹爪的限速/限位语义

| 项目 | handcode | firmware/ref_core_f405 | firmware_new |
|---|---|---|---|
| 闭合角（pos0） | `ClosedAngle = -100` 圈 | `ClosedMotorAngle = 0` 圈（!HAND_ZERO 后） | `currentMotorAngle = 0` 圈（!START 后默认闭合） |
| 张开角（pos100） | `OpenedAngle = +100` 圈 | `OpenMotorAngle = 100` 圈（撞开限位） | `SetAngle(100.0f)` → 电机轴 100 圈 |
| 默认电机轴 0~100 圈 | 100 | 100 | 100 |
| reduction | 16 | 16 | 16 |
| 校准机制 | `HandCalibration()` 空函数 | `ApplyPositionAsHome()` 真实归零 |  `ApplyPositionAsHome()` |

### 3.5 夹爪的统一失能/使能策略（Q4=A：1 帧 ALL 广播）

**本期决策（2026-10-03 修订）**：夹爪使能/失能与 J1~J7（含地轨）共享 `!START` / `!DISABLE`，**不**实现独立的 `!HAND_EN` / `!HAND_DIS`，**且**全部电机（含夹爪 id=8）通过 ALL 广播**仅 1 帧**触发。

**机制**（基于代码事实，2026-10-03 核对）：
- `!START` → `dummy.SetEnable(true)` → `motorJ[ALL]->SetEnable(true)` 1 帧广播
  - `motorJ[ALL]` 是 `CtrlStepMotor(_hcan, 0, ...)`（id=0 占位对象），发 `StdId = (0<<7)|0x01 = 0x01`
  - 电机端 `firmware_new/dummy-motor-fw/Core/Src/can.c:202-204`：
    ```c
    if (id == 0 || id == boardConfig.canNodeId) {
        OnCanCmd(cmd, RxData, RxHeader.DLC);
    }
    ```
    `id==0` 帧被**所有**板子（含夹爪 id=8）接收处理
  - 接口 `interface_can.cpp:14-17` `case 0x01`：
    ```c
    motor.controller->requestMode = (*(uint32_t*)RxData == 1) ?
                                    Motor::MODE_COMMAND_VELOCITY : Motor::MODE_STOP;
    ```
    全部电机进 `MODE_COMMAND_VELOCITY`；gripper 默认 `velocitySetPoint=0`，电机静止待命
- `!DISABLE` → `dummy.SetEnable(false)` → `motorJ[ALL]->SetEnable(false)` 1 帧广播
  - 全部电机进 `MODE_STOP`，扭矩释放

**StepHand 不 override SetEnable 的关键点**（Q3=A）：
- handcode 的 `StepHand`（继承 `CtrlStepMotor`）不 override SetEnable，调基类 → `cmd=0x01` 切 `MODE_COMMAND_VELOCITY`
- 旧 `DummyHand::SetEnable(true)` 用 `cmd=0x07` 切 `MODE_COMMAND_POSITION` 是与 handcode 偏离的特殊逻辑，必须删除（迁移时一并清掉）

**本期要删除的"2 帧"旧设计**（基于错误假设"夹爪不在 ALL 代理范围"；实际 ALL 已覆盖夹爪，但代码里还在，需要本期 Step 2 删掉）：
- ❌ `dummy_robot.cpp:148-154` `Reboot()`：`motorJ[ALL]->Reboot()` + `hand->Reboot()` + `osDelay(500)` + `HAL_NVIC_SystemReset()` —— `hand->Reboot()` 冗余（ALL `cmd=0x7F` 帧已被夹爪端 `case 0x7F` 处理）；如需保留主控重启，必须保证全部电机（含夹爪）在 `osDelay` 内完成 reboot
- ❌ `dummy_robot.cpp:156-165` `RebootAllMotors()`：`motorJ[ALL]->Reboot()` + `hand->Reboot()` —— `hand->Reboot()` 同样冗余
- ❌ `dummy_robot.cpp:416-422` `SetEnable()`：`motorJ[ALL]->SetEnable(_enable)` + `hand->SetEnable(_enable)` —— `hand->SetEnable(_enable)` 冗余（ALL `cmd=0x01` 帧已被夹爪端 `case 0x01` 处理）
- 迁移时这 3 处的 `hand->...` 调用必须删除

**【阻塞 2】电机端补丁需求（**本期改"先实测后定"**）**：
- **现状（理论）**：`case 0x01, data=1` 后关节电机 `velocitySetPoint` 默认 0 即可（不会动）；但夹爪**电流模式残值**是**可能**风险——若用户上一帧发过 `!HAND_O -1.0A`（`cmd=0x03`）后未 `!DISABLE`，下次 `!START` 发 ALL `0x01` 切到 VELOCITY 模式，但若电机端 `Motor::SetCurrentSetPoint()` 残值未清且 gripper 端有切回电流模式的 fallback（实现细节，需核对 `controller.cpp`），夹爪会"使能瞬间以 -1A 张开"
- **本期决策**：**不直接加** `motor.controller->SetCurrentSetPoint(0)` 补丁；先做 Step 6 清单 1.5 实测验证（`!HAND_O -1.0A` 后 `!DISABLE !START` 看夹爪是否微动）；若确有微动再在 `firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp` `case 0x01` 用 `#if MOTOR_TYPE == 8` 包一行，关节端不动
- **关节端**：实测通过则完全不动；实测失败再评估是否需要在 `MODE_COMMAND_VELOCITY` 模式下再补 `SetCtrlMode(MODE_COMMAND_VELOCITY)` 强制锁定

**【R3 说明】`!HAND_VEL/ACC/KP/KV/KI/KD` 运行时不持久化**：
- 这 6 条走 `CtrlStepMotor::SetVelocityLimit/Acceleration/DceKp/Kv/Ki/Kd`，`ctrl_step.cpp:124-185` 分发时 `canBuf[4]=0`（**不**写 EEPROM），下次 reboot 丢失
- 与关节 `SetJointAcceleration` 行为一致；`!HAND_ONBOOT`/`!HAND_ERASE` 走基类 `SetEnableOnBoot/EraseConfigs` `canBuf[4]=1` **写** EEPROM
- 使用者务必明示：调 `!HAND_ACC` 后如果想"刷入 EEPROM"，得用通用 `#ACC_J 8 <rps²>`（ASCII `#ACC_J` 也走 `SetAcceleration` 不持久化；如需持久化目前没有 ASCII 入口，需手工用电机端调试器烧 EEPROM）
- Step 3 / §1 / 决策表都需明示这一点

**不实现 `!HAND_EN` / `!HAND_DIS` 的理由**（用户 2026-10-03 明确）：
1. 与关节电机失能/使能状态保持同步——不存在"夹爪单独使能、关节失能"的混合态
2. 减少 ASCII 命令表面积（13 条而非 15 条）
3. 堵转保护逻辑统一：`!STALL_EN` / `!STALL_DIS` 不需要夹爪特例



## 4. 决策表（v4 全部已确认）

| # | 决策项 | 用户 2026-10-03 确认 | 备注 |
|---|---|---|---|
| 1 | 架构：DummyHand 独立类 vs StepHand 继承 | ✅ **StepHand 继承 CtrlStepMotor** | 22 个 C++ 方法免重写 |
| 2 | reduction | ✅ **16**（ref 校对） | handcode 的 8 是构造形参误植 |
| 3 | HAND_MAX_RPS | ✅ **30** | 与关节对齐 |
| 4 | `!HAND_I` 上限 | ✅ **0~2.5A** | gripper EEPROM currentLimit 上限 2500mA（与关节对齐） |
| 5 | `!HAND_O/C` 控制模式 | ✅ **电流透传 `SetCurrentSetPoint(±A)`** | **不沿用 handcode 语义**（handcode `!HAND_O` 调 `SetAngleWithCurrentLimit(+1)` = 正电流，按 ref 注释"+1=闭合施力"实际触发闭合，handcode 标错，处于 bug 状态）；本期按 ref 重新定义：`!HAND_O [A]` → `SetCurrentSetPoint(-A)`，`!HAND_C [A]` → `SetCurrentSetPoint(+A)` |
| 6 | `!HAND_EN/DIS` | ✅ **删除**，统一 `!START/!DISABLE` | 1 帧 ALL 广播覆盖全部电机（含夹爪 id=8） |
| 7 | `!HAND_I` 实现 | ✅ **handcode 原版**：仅改运行时 `hand->current` 变量，不发 CAN 帧 | I = Intensity 运行时变量；掉电丢；与 handcode 一致 |
| 8 | 变量命名 `hand` vs `hand2` | ✅ **收敛**：`StepHand* hand = nullptr` | 字段直接换类型，最小改动；handcode 的 `hand2` 临时类名不引入 |
| 9 | `StepHand::SetEnable` 是否 override | ✅ **不 override**：调基类 `CtrlStepMotor::SetEnable` → `cmd=0x01` 进速度模式待命 | 与 handcode 一致；使能后靠 `!HAND_O/C/POS` 主动驱动 |
| 10 | `!START` 是 1 帧 ALL 还是 2 帧（ALL + 夹爪） | ✅ **1 帧 ALL** | 电机端 `can.c:202-204` `id==0` 已路由所有电机；旧"2 帧"基于错误假设 |
| 11 | `!HAND_ZERO` 实现 | ✅ **什么都不写**（不识别、不占位、不返回 error） | ASCII 协议层完全不识别此命令，落到默认 fallback 响应；handcode 的 `HandCalibration()` 函数体空、`firmware/ref_core_f405` 的 `ApplyPositionAsHome()` 都不复用；统一进 `!CALIBRATION` 后再决定（`!CALIBRATION` 本期不实施） |
| 12 | `!HAND_VEL/ACC/KP/KV/KI/KD/ONBOOT/ERASE` | ✅ **新增** | 8 条 ASCII 调 `CtrlStepMotor::SetVelocityLimit/Acceleration/DceKp/Kv/Ki/Kd/EnableOnBoot/EraseConfigs`，无需新 C++ 方法（直调基类） |
| 13 | `#REBOOT 8` | ✅ **扩展** | `firmware_new/.../ascii_protocol.cpp` 当前已支持 `node 1~7` + `node==0` ALL 广播（`ascii_protocol.cpp:91-104`）；**需扩到 `node <= 8`**（`ascii_protocol.cpp` 加 `else if (node == 8)` 分支调 `dummy.hand->Reboot()`）|
| 14 | `#SET_DCE_KP/KV/KI/KD 8` | ✅ **新增** `ascii_protocol.cpp` | 当前 `firmware_new/.../ascii_protocol.cpp` **仅**有 `SET_DCE_KP/KI/KD`（范围 1~6，**无 KV**，**无** `I_LIMIT_J/ACC_J`，与 `firmware/ref_core_f405` 不一致）；本期需：(1) `#SET_DCE_KP/KI/KD` 扩 `node 1~6` → `1~8`；(2) **新增** `#SET_DCE_KV` / `#I_LIMIT_J` / `#ACC_J` 范围 `1~8`；(3) 全部 `#` 命令显式拒绝 `node==0` 广播（防止 ALL 误改所有电机 PID/电流限/加速度）|
| 15 | `#I_LIMIT_J 8` / `#ACC_J 8` / `#GETJACC 8` | ✅ **新增/跳过** `ascii_protocol.cpp` | 早期文档误标为 `#I_LIMIT_J` / `#ACC_J` **已存在**——**实际不存在**（2026-10-03 二轮核对：`firmware_new/.../ascii_protocol.cpp` 现有 # 命令只有 `GETJPOS/GETRAILPOS/GETLPOS/SET_DCE_KP/KI/KD/REBOOT/CMDMODE` 7 条，无 `I_LIMIT_J/ACC_J/SET_DCE_KV`）。本期按"新增"处理，范围 1~8 + 拒绝 `node==0`；`#GETJACC 8` 暂不实现，UI 不暴露；`#GETI 8` 已在 ref 实现（`firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp:721-770`），**本期不引入 firmware_new**（用户 2026-10-03 决策）|
| 16 | **【阻塞 1】`can_protocol.cpp` 回包路由修复** | ✅ **必做**：Step 2 末尾加配套修改（详见附录 B Step 2 配套）| `id==8` 当前硬走 `motorJ[id]` 越界崩溃；必须分流到 `dummy.hand->UpdateAngleCallback` |
| 17 | **【阻塞 2】夹爪端 `cmd=0x01, data=1` 后清电流** | ⏸️ **待实测**：电机端 `interface_can.cpp` `case 0x01` **本期不直接加** `SetCurrentSetPoint(0)` 补丁；先做 Step 6 清单 1.5 实测（`!HAND_O -1.0A` 后 `!DISABLE !START` 看夹爪是否微动），若微动再按 `#if MOTOR_TYPE == 8` 包一行 | 理论上关节电机 `velocitySetPoint` 默认 0 不动即可；夹爪电流残值风险依赖 gripper 端 controller.cpp 实现细节（`MODE_COMMAND_VELOCITY` 模式下是否会 fallback 到 CURRENT 模式，未核实）。**先实测后定**，节省调试面 |
| 18 | **【阻塞 3】UART4 `!` 分支加 else fallback** | ✅ **必做**：Step 3 `OnUart4AsciiCmd` `!` 分支末尾加 `else if (s.substr(0, 5) == "!HAND") Respond(_responseChannel, "error forbidden on UART4");` | 顺序匹配避免 `!HAND_DIS` 抢先匹配 `!HAND_O`；不让上位机超时卡死 |
| 19 | **【阻塞 4】删除 `hand2` 注释残留** | ❌ **误判，无需处理**：`hand2` 注释只在 `handcode/dummy_robot.cpp:27-28`，**firmware_new 没有**；handcode 目录不在本期迁移工作区 | 决策表登记误判，Step 2 配套段已删除；与「修订」段 #13 / Step 2 现状描述一致 |
| 20 | **【R1 决策】`MakeProtocolDefinitions` 中 `set_enable` 必须用强转写法** | ✅ **采纳**：与 ref `firmware/ref_core_f405/Robot/instances/dummy_robot.h:128-130`（StepHand 的 `MakeProtocolDefinitions` 第 1 条 `set_enable` 强转写法）一致 | 避免 fibre 库继承二义性 |
| 21 | **【R2 决策】不 override `UpdateAngleCallback`** | ✅ **采纳**：override 仅 2 个方法（`SetAngle` + `SetAngleWithVelocityLimit`） | 与 ref 一致；本期不查 `hand->angle`；减少调试面 |
| 22 | **【R3 决策】`!HAND_ACC` 等运行时不持久化** | ✅ **采纳**：文档 §1 / Step 3 表格补 "持久化？" 列；明示 `VEL/ACC/KP/KV/KI/KD` 调一次丢一次，`ONBOOT/ERASE` 写 EEPROM | 避免使用者误以为"配置类 = 持久化"；与关节 `SetJointAcceleration` 行为一致 |

## 附录 A：架构对照速查表

| 项 | handcode | firmware/ref_core_f405 | firmware_new（决策） |
|---|---|---|---|
| 夹爪 nodeID | 7 | 8 | 8 |
| reduction | 8（误，handcode 跳过归一化所以不影响行为）| 16 | **16**|
| C++ 类 | `StepHand` 继承 | `StepHand` 继承 | **`StepHand` 继承 CtrlStepMotor**|
| 构造 `inverse` | `false` | `true` | `true`（2026-10-03 决策：与 ref 对齐，让基类 `SetPositionWithVelocityLimit` 自动取反）|
| `!HAND_O/C` | 电流透传（**handcode 标错**：`!HAND_O` 调 `SetAngleWithCurrentLimit(+1)` = 正电流；按 ref 注释"+1=闭合施力"实际触发闭合，处于 bug 状态）| 电流透传 | **电流透传**（`!HAND_O [A]` → `-A`，`!HAND_C [A]` → `+A`）|
| `!HAND_EN/DIS` | 有 | 有 | **删除，统一加入 !START/!DISABLE**（Q4=A 后 1 帧 ALL 广播覆盖全部电机，含夹爪 id=8） |
| `!HAND_POS` 限速 | 70 r/s | 20 r/s | 30 r/s|
| `!HAND_I` 范围 | 0~2A（`if (cu <= 2 && cu>0)`，`handcode/ascii_protocol.cpp:114`）| n/a（无 ASCII）| 0~2.5A |
| `!HAND_ZERO` | 空函数体 | `ApplyPositionAsHome()` | **不实现、不识别、不占位** |
| ASCII 命令数 | 7 | ~7（无配置类）| **12**（`!HAND_O/C/POS` 运行时 3 条 + `!HAND_I` 运行时 1 条 + `!HAND_VEL/ACC/KP/KV/KI/KD/ONBOOT/ERASE` 配置 8 条；`!HAND_EN/DIS/ZERO` 决策删除不占位）；按 R3 持久化分：写 EEPROM 7 条 + 不写 EEPROM 5 条 |
| `#SET_DCE_* 8` | n/a | 已支持 | **ascii_protocol.cpp 需扩 `node 1~6` → `1~8` + 显式拒绝 `node==0`**（防止 ALL 广播误改所有 PID）|
| `#REBOOT 8` | n/a | 不支持（1~7）| **ascii_protocol.cpp 需扩到 `node <= 8` + `node==8` 分支**（ALL 广播 `node==0` 已存在）|
| `#I_LIMIT_J 8` / `#ACC_J 8` | n/a | 已支持 | firmware_new **实际不存在**（2026-10-03 二轮核对）→ **本期新增**，范围 1~8 + 拒绝 `node==0` |
| 串口助手 UI | n/a | n/a | **`串口助手_old.py` 需新建夹爪控制面板**（开/闭 + 电流 0.05~2.5A + 位置 0~100%）；当前 0 个夹爪 UI 控件；`#REBOOT` 下拉框需扩 0~7 → 0~8 |
| 主控 PID 默认 | n/a | 200/80/300/250（gripper EEPROM）| 同 |
| 串口助手 PID 默认 | n/a | kp=200 kv=80**ki=100** kd=100 | 同；注意 ki 与 EEPROM 默认 300 差异 |
| `hand->current` 默认 | 0.7f（`handcode/dummy_robot.h:65`） | 1.2f（`firmware/ref_core_f405/Robot/instances/dummy_robot.h:91`） | **1.0f**（本期决策值；不沿用 ref/handcode 任一默认）|
| 角度范围 | `_angle ∈ [-100, 100]`（`handcode/dummy_robot.h:70` 构造形参） | `_angle ∈ [-100, 100]`（`firmware/ref_core_f405/Robot/instances/dummy_robot.h:82-83` 构造形参） | `_angle ∈ [0, 100]`（override 后直接对应电机圈数 0~100）|

---

## 附录 B：完整 Step 实施步骤清单（v4.1）

### 现状起点（2026-10-03 17:06，2026-10-03 18:36 二轮核对）

- `firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.h`（213 行）中 `DummyHand` **独立类**（不继承 CtrlStepMotor），共 7 个公开方法 + 3 条 fibre；`dummy_robot.cpp:125` `hand = new DummyHand(_hcan, 8)`；`dummy_robot.cpp:148-154`（`Reboot()`）、`156-165`（`RebootAllMotors()`）、`416-422`（`SetEnable()`）含 3 个待清理的 `hand->Reboot/SetEnable` 调用；`dummy_robot.cpp` 末尾 ~110 行含 5 个 `DummyHand::` 方法实现（构造、`SetAngle`、`SetMaxCurrent`、`SetEnable`、`SendPositionVelocityFrame`、`Reboot`），引用已删类的成员（`hcan/canBuf/txHeader/currentMotorAngle/currentVelocityLimit/isEnabled/nodeID`），**必须全部删除**（Step 2 必做项，本期 2026-10-03 实操时首次编译因此失败，PM 补完后通过）
- `firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp`（194 行），无任何 `!HAND_*` 命令；`#` 命令现有 7 条：`GETJPOS / GETRAILPOS / GETLPOS / SET_DCE_KP / SET_DCE_KI / SET_DCE_KD / REBOOT / CMDMODE`（其中 `SET_DCE_KP/KI/KD` 范围仅 `1~6`）；**`#SET_DCE_KV` / `#I_LIMIT_J` / `#ACC_J` / `#GETJACC` / `#GETI` 当前都不存在**（与 `firmware/ref_core_f405` 不一致；`#GETI` 在 ref 已实现 `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp:721-770`，本期不引入）；`#REBOOT` 范围 `1~7` + `node==0` ALL 广播分支（**已存在**）
- `串口助手_old.py` 共 968 行，**0 个夹爪 UI 控件**；`#REBOOT` 节点下拉框值 `[0..7]`
- `firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp`（M 状态）已包含 gripper 端 `0x03/0x07/0x11/0x12/0x13/0x14/0x15/0x16/0x17~0x1A/0x1B/0x7E/0x7F` 全部命令，**电机端无需改动**

### Step 1（**已完成于 2026-10-03**，参考 git status ` M firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`）

[Step 1 已修 nodeID=8 + SetAngle/SetMaxCurrent 字节类型错配修复 + SendPositionVelocityFrame 填齐 8 字节 + SetEnable(false) 改用 cmd=0x01] — 但 `DummyHand` 独立类结构保留，待 Step 2 替换为 `StepHand` 继承式

### Step 2：主控 `dummy_robot.h/cpp` 架构改造（**本期必做**）

**改 `dummy_robot.h`**：
- 删除整个 `class DummyHand { ... }` 定义（48-78 行 + MakeProtocolDefinitions + 7 个方法）
- 新增 `class StepHand : public CtrlStepMotor { ... }`：
  - 构造函数 `StepHand(CAN_HandleTypeDef* hcan, uint8_t id) : CtrlStepMotor(hcan, id, true, 16, 0, 100)` （reduction=16, **inverse=true 对齐 ref**，基类 `SetPositionWithVelocityLimit` 自动取反；角度范围 0~100）
  - 公开成员 `float current = 1.0f`（运行时电流变量，**基类无此字段**）
  - override `SetAngle(float _angle)` → 直接 `SetPositionWithVelocityLimit(_angle, 30.0f)`，跳过基类 `_angle/360*reduction` 归一化（基类按 `inverse=true` 自动取反 `_angle`，发电机反方向圈数）
  - override `SetAngleWithVelocityLimit(float _angle, float _vel)` → 直接 `SetPositionWithVelocityLimit(_angle, _vel)`，跳过归一化（同样基类按 `inverse=true` 取反）
  - **【R2 决策】不 override `UpdateAngleCallback`**（详见「修订」段 #15 与 §0 结论第 1 大点）：与 ref/StepHand 一致（`firmware/ref_core_f405/Robot/instances/dummy_robot.h:76-141` 的 `StepHand` 类也不 override 该方法，`MakeProtocolDefinitions` 在 :128-140）；本期确认**不**读 `hand->angle`；基类公式 bug（subdivision steps 被当圈数算 reduction 归一化，见 `firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp:255-262`）保留在基类统一修，bug 修好时夹爪自动跟着对；未来真要读 `hand->angle` 时再补 override（约 3 行代码）
  - override `MakeProtocolDefinitions()` → 注册 **4 条 fibre**：`set_angle`（指 `StepHand::SetAngle`）、`set_enable`（指基类 `CtrlStepMotor::SetEnable`，不 override）、`set_current`（指基类 `SetCurrentSetPoint`）、`set_current_limit`（指基类 `SetCurrentLimit`）
- 字段类型：`DummyHand* hand = nullptr` → `StepHand* hand = nullptr`

**改 `dummy_robot.cpp`**：
- 构造函数：`hand = new DummyHand(_hcan, 8)` → `hand = new StepHand(_hcan, 8)`（行 **125**）
- ~~【阻塞 4】删除 `hand2` 注释残留~~ → **误判，无需处理**：`hand2` 注释残留只在 `handcode/dummy_robot.cpp:27-28`（`// hand2 = new CtrlStepMotor(_hcan, 7, false, 10, -100, 100);` 与 `// hand2 = new StepHand(_hcan, 7);`），**firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp` 构造区无此注释**，handcode 目录不在本期迁移工作区
- `Reboot()`（行 **148-154**）：删除 `hand->Reboot();` 一行，保留 `osDelay + HAL_NVIC_SystemReset`（注：`osDelay(500)` 给电机 reboot 时间，ALL 帧已覆盖夹爪，无需单独 hand 帧）
- `RebootAllMotors()`（行 **156-165**）：删除 `hand->Reboot();` 一行
- `SetEnable()`（行 **416-422**）：删除 `hand->SetEnable(_enable);` 一行
- **【补 2026-10-03】删除 `dummy_robot.cpp` 末尾 5 个 `DummyHand::` 方法实现**（约 110 行）：
  - 构造函数 `DummyHand::DummyHand(CAN_HandleTypeDef*, uint8_t)`
  - `void DummyHand::SetAngle(float _angle)`（发 cmd=0x07 位置帧）
  - `void DummyHand::SetMaxCurrent(float _val)`（发 cmd=0x12 电流限帧）
  - `void DummyHand::SetEnable(bool _enable)`（发 cmd=0x01/0x07 帧）
  - `void DummyHand::SendPositionVelocityFrame(float _pos, float _vel)`（私有，发 0x07 8 字节完整帧）
  - `void DummyHand::Reboot()`（发 cmd=0x7F 帧）
  - 全部删除原因：`DummyHand` 类已删，这些方法引用的成员（`hcan/canBuf/txHeader/currentMotorAngle/currentVelocityLimit/isEnabled/nodeID`）已不存在；StepHand 继承 `CtrlStepMotor` 后，相同功能由基类方法承担（`SetAngle` → `SetPositionWithVelocityLimit` 走基类，cmd=0x07 由 `CtrlStepMotor::SetPositionWithVelocityLimit` 内部构造并发送；Reboot/SetEnable 走基类对应方法，ALL 帧覆盖夹爪）

**Step 2 配套修改 1（阻塞 1，必做）— `can_protocol.cpp` 回包路由修复**：

文件：`firmware_new/dummy-ref-core-fw/UserApp/protocols/can_protocol.cpp:23`

当前代码（**必崩**）：
```cpp
case 0x23:
    dummy.motorJ[id]->UpdateAngleCallback(*(float*) (data), data[4]);  // id=8 时 motorJ[8]=nullptr → HardFault
    break;
```

改成（必加 `id==8` 分流）：
```cpp
case 0x23:
    if (id == 8)
        dummy.hand->UpdateAngleCallback(*(float*) (data), data[4]);
    else
        dummy.motorJ[id]->UpdateAngleCallback(*(float*) (data), data[4]);
    break;
case 0x25:
    if (id == 8)
        memcpy(&dummy.hand->temperature, data, sizeof(uint32_t));
    else
        memcpy(&dummy.motorJ[id]->temperature, data, sizeof(uint32_t));
    break;
```

**这是阻塞 1 的最小修复**。`id==8` 路径走 `dummy.hand`（`StepHand*`）后，基类 `UpdateAngleCallback` 公式 `angle=_pos/reduction*360` 会让 `hand->angle` 偏离电机圈数——但 R2 决策明确本期不查 `hand->angle`，且这条路径与 ref 的 `firmware/ref_core_f405/Robot/instances/dummy_robot.h` 行为一致（ref 也走基类默认实现）。

### Step 3：`ascii_protocol.cpp` 新增 `!HAND_*` 命令（**本期必做**）

**在 `OnUsbAsciiCmd` 和 `OnUart4AsciiCmd` 的 `!` 分支新增 12 条命令**：
- **运行时**（UART4 允许）：`!HAND_O`/`!HAND_C`/`!HAND_POS`（3 条）
  - `!HAND_O [A]` → `dummy.hand->SetCurrentSetPoint(-(A ? A : dummy.hand->current))`（张开方向电流；**带参用参电流，缺省用 `hand->current`**）
  - `!HAND_C [A]` → `dummy.hand->SetCurrentSetPoint(+(A ? A : dummy.hand->current))`（闭合方向电流；同上）
  - `!HAND_POS [0-100]` → `dummy.hand->SetAngle(pos)`（内部发 `cmd=0x07, vel=30`）
- **运行时但不发 CAN**（UART4 允许）：`!HAND_I`（1 条）→ 仅 `dummy.hand->current = x`，**不**调任何 `CtrlStepMotor` 方法
- **配置**（UART4 禁止）：`!HAND_VEL`/`!HAND_ACC`/`!HAND_KP`/`!HAND_KV`/`!HAND_KI`/`!HAND_KD`/`!HAND_ONBOOT`/`!HAND_ERASE`（8 条）
  - 每个直接调 `dummy.hand->SetVelocityLimit / SetAcceleration / SetDceKp/Kv/Ki/Kd / SetEnableOnBoot / EraseConfigs`

**UART4 改造**（**含阻塞 3**）：当前 `OnUart4AsciiCmd` 用 `if (_cmd[0] == '!' || !dummy.IsEnabled())` 把所有 `!` 命令放行；需拆分为：

- `!HAND_O/C/POS/I` 4 条运行时 → 允许 UART4（落到主 if 内部的具体分支）
- `!HAND_VEL/ACC/KP/KV/KI/KD/ONBOOT/ERASE` 8 条配置 → **禁止** UART4；不识别时回 `error forbidden on UART4`（**不让上位机卡住等待**——现状 UART4 收到不识别命令时无响应，上位机会超时卡死）

**【阻塞 3】UART4 `!` 分支必须加 else fallback**，最小示例代码（替换原 `OnUart4AsciiCmd` 的 `!` 分支）：

```cpp
void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    if (_cmd[0] == '!' || !dummy.IsEnabled())
    {
        std::string s(_cmd);
        // —— 原 !STOP/START/HOME/CALIBRATION/RESET/DISABLE 保持不变 ——
        if (s.find("STOP") != std::string::npos) { ... }
        else if (s.find("START") != std::string::npos) { ... }
        // ... 其余原 ! 命令
        // —— 夹爪新增 4 条运行时（顺序匹配，避免 HAND_DIS 抢先）——
        else if (s == "!HAND_O" || s.rfind("!HAND_O ", 0) == 0) {
            float A = 0; sscanf(_cmd, "!HAND_O %f", &A);
            if (A <= 0) A = dummy.hand->current;
            if (A > 2.5f) A = 2.5f;
            dummy.hand->SetCurrentSetPoint(-A);
            Respond(_responseChannel, "ok hand open I=%.2f", A);
        }
        else if (s == "!HAND_C" || s.rfind("!HAND_C ", 0) == 0) {
            float A = 0; sscanf(_cmd, "!HAND_C %f", &A);
            if (A <= 0) A = dummy.hand->current;
            if (A > 2.5f) A = 2.5f;
            dummy.hand->SetCurrentSetPoint(+A);
            Respond(_responseChannel, "ok hand close I=%.2f", A);
        }
        else if (s.rfind("!HAND_POS ", 0) == 0) {  // 以 "!HAND_POS " 开头
            uint32_t pos; sscanf(_cmd, "!HAND_POS %lu", &pos);
            if (pos <= 100) dummy.hand->SetAngle(static_cast<float>(pos));
            ...
        }
        else if (s.rfind("!HAND_I ", 0) == 0) {
            float cu; sscanf(_cmd, "!HAND_I %f", &cu);
            if (cu <= 2.5f && cu > 0) dummy.hand->current = cu;
            ...
        }
        // —— 【阻塞 3 关键】其余 !HAND_* 一律 forbidden ——
        else if (s.substr(0, 5) == "!HAND")
            Respond(_responseChannel, "error forbidden on UART4");
        // —— 其它未知 ! 命令同样 forbidden ——
        else
            Respond(_responseChannel, "error forbidden on UART4");
    }
    else if (_cmd[0] == '#') { ... }   // 保持原 # 命令
    ...
}
```

**关键陷阱**：`!HAND_DIS` 包含子串 `"HAND_O"`，若用 `s.find("HAND_O") != std::string::npos` 会被 `!HAND_DIS` 抢先匹配成"张开方向电流"——必须用 `s == "!HAND_O"` 或 `s.rfind("!HAND_O ", 0) == 0` 显式判首/等长。`!HAND_EN` 包含 `"HAND_E"`，同理。

**`!HAND_ZERO` 不实现**：两个函数都不识别此命令，落到上面的 `else if (s.substr(0, 5) == "!HAND")` 分支回 `error forbidden on UART4`；USB 侧同样不识别（落到 `if (_cmd[0] == '!')` 的最末尾 else），不返回 ok 也不返回 error，保持"不识别即静默"语义（与决策 #11 对齐）。

**Step 3 配套说明 — 阻塞 2 改"先实测后定"**：电机端 `firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp` `case 0x01` 分支**本期不直接加** `SetCurrentSetPoint(0)` 补丁。先做 Step 6 清单 1.5 实测验证（`!HAND_O -1.0A` 后 `!DISABLE !START` 看夹爪是否微动）；若确有微动再按 #if MOTOR_TYPE == 8 包一行，关节端不动。**理由**：理论上关节电机 `velocitySetPoint` 默认 0 不动即可；夹爪电流残值（`!HAND_O` 上一帧发过）会**可能**触发"使能瞬间以 -X A 张开"但**风险依赖 gripper 端 controller.cpp 实现细节**（`MODE_COMMAND_VELOCITY` 模式下是否会 fallback 到 CURRENT 模式，未核实）。**先实测**而非盲加，节省调试面。

### Step 4：`ascii_protocol.cpp` 新增/扩展 `#` 命令（**本期必做**）

- `#SET_DCE_KP/KI/KD` 范围 `1~6` → `1~8`（`OnUsbAsciiCmd` 中已有 3 个 case）
- 新增 `#SET_DCE_KV <node> <kv>`（当前完全不存在）
- 新增 `#I_LIMIT_J <node> <A>` → `dummy.hand->SetCurrentLimit(_val)`
- 新增 `#ACC_J <node> <rps²>` → `dummy.hand->SetAcceleration(_val)`
- `#GETJACC <node>` → 调用 `motorJ[node]->SetAcceleration` 的查询对称接口（基类无 `GetAcceleration`，**暂不实现** 或新建 `MotorController::GetAcceleration` 透传 `cmd=0x2C`）—— 决策待补，本期先跳过 `#GETJACC` 实现，UI 不暴露
- ~~`#GETI <node>`~~ → **本期不实现**（用户 2026-10-03 决策：ref 已有 `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp:721-770` 同步查询 Current Limit，但 firmware_new 暂无对应 `QueryCurrentLimit` 基类方法，且与基线需求无关，暂不引入）
- 扩展 `#REBOOT`：范围 `1~7` → `1~8`；加 `else if (node == 8)` 分支 → `dummy.hand->Reboot()`

### Step 5：`串口助手_old.py` UI 重构（**本期必做**）

- **新建夹爪控制面板**（位置待定，建议放在右栏 Notebook 新页"夹爪控制"）：
  - `开` 按钮 → `!HAND_O`（不带参数，用 `hand->current`）
  - `闭` 按钮 → `!HAND_C`
  - 电流值 `Spinbox`（0.05~2.5，步长 0.05，默认 1.0） + `设置电流` 按钮 → `!HAND_I {val}`
  - 位置 `Scale`（0~100，水平滑块） + `设置位置` 按钮 → `!HAND_POS {val}`
  - **可选**：配置类 8 条 `!HAND_VEL/ACC/KP/KV/KI/KD/ONBOOT/ERASE` 折叠面板（本期可不暴露，等配置需求时再加）
- 扩展 `#REBOOT` 节点下拉框：值列表 `[0..7]` → `[0..8]`（行 267）
- **不修改** `串口助手.py`

### Step 6：测试验证（**本期必做**）

- 编译：`firmware_new/dummy-ref-core-fw` 用 Ninja 构建通过
- 编译：`firmware_new/dummy-motor-fw` 确认 gripper 端固件未破坏
- 实机验证清单：
  1. 上电默认夹爪闭合（`currentMotorAngle=0`）
  1.5. **`!HAND_O -1.0A` 后 `!DISABLE !START` 验证夹爪不微动**（阻塞 2 **判定项**——实测决定电机端 `cmd=0x01, data=1` 是否要补 `SetCurrentSetPoint(0)`）
  2. `!START` 后夹爪不微动（ALL 帧进 VELOCITY，velocitySetPoint=0）
  3. `!DISABLE` 后夹爪失能（ALL 帧进 STOP，扭矩释放）
  4. `!HAND_O 1.0` 夹爪张开
  5. `!HAND_C 1.0` 夹爪闭合
  6. `!HAND_POS 50` 夹爪走到 50%（电机轴 50 圈）
  7. `!HAND_POS 0` 闭合；`!HAND_POS 100` 张开撞限位
  8. `!HAND_I 2.5` 后 `!HAND_O 2.5` 用 2.5A 张开
  9. `#REBOOT 8` 单独重启夹爪（其他电机不动）
  10. `#REBOOT 0` 重启全部电机（含夹爪）
  11. **`!HAND_O` 后回包角度对 id=8 不崩**（阻塞 1 配套——`can_protocol.cpp` 分流后第一帧 `cmd=0x23` 触发 `dummy.hand->UpdateAngleCallback` 不越界）
  12. **UART4 发 `!HAND_VEL 30` 回 `error forbidden on UART4`**（阻塞 3 配套——fallback 工作且不让上位机卡死）
  13. **`!HAND_ACC 50` 后 reboot 验证加速度丢失**（R3 明示——验证文档说明与实测一致）

### 风险与后续（非阻塞）

1. **【阻塞 2 → 改"先实测后定"】** 夹爪端 `cmd=0x01, data=1` 后是否需要在电机端清电流残值，**待 Step 6 清单 1.5 实测后定**——若 `!HAND_O -1.0A` 后 `!DISABLE !START` 夹爪不微动则不加，若微动再在 `firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp` `case 0x01` 用 `#if MOTOR_TYPE == 8` 包一行 `motor.controller->SetCurrentSetPoint(0)`
2. `CtrlStepMotor` 基类缺少 `GetAcceleration` 方法，`#GETJACC 8` 暂未实现（待后续）——`#GETI 8` 已在 ref 实现（`firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp:721-770`，同步查询 Current Limit），**本期 firmware_new 不实现，从文档需求清单中移除**（用户 2026-10-03 决策）
3. `!HAND_ZERO` 校准功能彻底空白；统一进 `!CALIBRATION` 后再决定（`!CALIBRATION` 本期不实施）
4. 若实测发现 gripper 端 `controller.cpp` 在 `MODE_COMMAND_VELOCITY` 模式下因电流残值导致 fallback 切回电流模式（实现细节），需在 Step 6.1.5 之后补"切回 VELOCITY 时强制 `SetCtrlMode(MODE_COMMAND_VELOCITY)`"——目前假设不会发生
