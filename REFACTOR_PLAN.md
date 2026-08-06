# 关节/地轨加速度 & 电流参数重构 — 任务规划

> 文档目的：把 `REFACTOR_DEMANDS.md` 固化的需求转化为**可执行的工程任务清单**。
> 文档状态：**规划已确认**（2026-08-07 01:46），等待用户审批后开始执行。
> 配套文档：`REFACTOR_DEMANDS.md`（需求背景） / `ISSUES.md`（P0/P1 问题） / `TODO.md`（功能路线图）

---

## 0. 执行原则

- ✅ 每个任务**可独立编译**（最小化依赖链）
- ✅ 每个任务**可独立 review**（提交粒度对应任务编号）
- ✅ 每个任务**可独立回滚**（git revert HEAD）
- ✅ 不改用户在 `REFACTOR_DEMANDS.md` 3.0 节已确认"不动"的部分
  - `#SPEED_J` / `jointAccBases[6]` / `#ACC_BASE_J X V` / `SetVelocityLimit` / `B1` flash bug

---

## 1. 依赖关系图

```
【L0 - 基础设施】         【L1 - 业务实现】       【L2 - 集成验证】
━━━━━━━━━━━━━━━━━━━     ━━━━━━━━━━━━━━━━━━━     ━━━━━━━━━━━━━━━━━━━

电机固件 (4 份):           主控 ASCII:              串口助手:
 T1: A7 (0x2C) ──────┐    T5: A3+A4 ──────┐       T11: UI1~UI4
 T2: A8 (0x2D) ──────┤    T6: M11+M12 ────┤        (主控/电机完成后)
                      │    T7: M4+M5 ──────┤
                      │    (T8 已并入 T6)  │
                      ▼                   ▼
主控 ctrl_step:        ▼
 T3: M1 (SetAcc+persist) ──┐
 T4: M3 (SetCur+persist) ──┤
                            ▼
                       主控 can_protocol:
                            T9: A5+A6 ◄── 依赖 A7/A8 返回数据格式
                                          ◄── 依赖 M1/M3 发出查询
                                          ◄── 依赖 A3/A4 ASCII 入口

主控 dummy_robot:         主控 ascii_protocol:
 T10: D5+D6+D7+D8+D12 ───► (被 T7 引用)
```

### 1.1 关键依赖链

| 任务 | 依赖 |
|---|---|
| T3 / T4 | 无（独立） |
| T1 / T2 | 无（独立） |
| T6 / T7 | T3（T6/T7 调 `SetAcceleration` 新签名） |
| T5 | T3 / T4（Query 函数必须存在） |
| T9 | T1 / T2 / T3 / T4 / T5（电机返回 + 主控查询入口） |
| T10 | T7（ASCII 不再调 `SetRailAcc`） |
| T11 | T1~T9 全部完成 |

### 1.2 推荐执行顺序

```
Step 1: 电机固件（T1 + T2，可并行）
Step 2: 主控底层（T3 + T4，可并行）
Step 3: 主控 ASCII（T5 + T6 + T7）
Step 4: 主控 CAN 响应（T9）
Step 5: 主控删除冗余字段（T10）—— ⚠️ EEPROM 风险
Step 6: 编译验证 + 烧录 + 集成测试
Step 7: 串口助手（T11）
```

---

## 2. 任务清单

> ⚠️ **T8 已并入 T6** —— sscanf 返回值修复（`ret >= 2`）与 #ACC_J 支持 `&` 在同一处修改 —— 不单独 commit。

### 2.1 T1 —— 电机固件加 0x2C 处理（A7）

| 项 | 内容 |
|---|---|
| **改哪** | `firmware/motor_fw_f103_35/UserApp/protocols/interface_can.cpp`<br>`firmware/motor_fw_f103_42/UserApp/protocols/interface_can.cpp`<br>`firmware/motor_fw_f103_57/UserApp/protocols/interface_can.cpp`<br>`firmware/motor_fw_f103_gripper/UserApp/protocols/interface_can.cpp` |
| **做什么** | 在 `case 0x2B` 之后加 `case 0x2C` —— 返回 `boardConfig.velocityAcc / MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS`（即"圈/s²"）|
| **改动量** | 每份 ~12 行 × 4 = **~48 行** |
| **风险** | 🟢 低 —— 加 case 不动现有 |
| **依赖** | 无 |
| **回滚** | git revert HEAD（4 份独立 revert）|

**模板代码**（4 份通用）：

```cpp
case 0x2C: // Get Acceleration (Circle/s²)
{
    tmpF = (float) boardConfig.velocityAcc / (float) motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
    auto* b = (unsigned char*) &tmpF;
    for (int i = 0; i < 4; i++)
        _data[i] = *(b + i);
    _data[4] = 0;
    _data[5] = 0;
    _data[6] = 0;
    _data[7] = 0;
    txHeader.StdId = (boardConfig.canNodeId << 7) | 0x2C;
    CAN_Send(&txHeader, _data);
}
    break;
```

**编译验证**：

```bash
cd firmware/motor_fw_f103_35/build ; ninja    # 0 errors
cd firmware/motor_fw_f103_42/build ; ninja    # 0 errors
cd firmware/motor_fw_f103_57/build ; ninja    # 0 errors
cd firmware/motor_fw_f103_gripper/build ; ninja    # 0 errors
```

---

### 2.2 T2 —— 电机固件加 0x2D 处理（A8）

| 项 | 内容 |
|---|---|
| **改哪** | 同 T1（4 份 `interface_can.cpp`）|
| **做什么** | 在 `case 0x2C` 之后加 `case 0x2D` —— 返回 `boardConfig.currentLimit / 1000.0f`（即"A"）|
| **改动量** | 每份 ~12 行 × 4 = **~48 行** |
| **风险** | 🟢 低 |
| **依赖** | 无（与 T1 独立，可并行）|
| **回滚** | git revert HEAD |

**模板代码**：

```cpp
case 0x2D: // Get Current-Limit (A)
{
    tmpF = (float) boardConfig.currentLimit / 1000.0f;
    auto* b = (unsigned char*) &tmpF;
    for (int i = 0; i < 4; i++)
        _data[i] = *(b + i);
    _data[4] = 0;
    _data[5] = 0;
    _data[6] = 0;
    _data[7] = 0;
    txHeader.StdId = (boardConfig.canNodeId << 7) | 0x2D;
    CAN_Send(&txHeader, _data);
}
    break;
```

---

### 2.3 T3 —— SetAcceleration 加 persist + QueryAcceleration（M1 + A1）

| 项 | 内容 |
|---|---|
| **改哪** | `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` + `.hpp` |
| **做什么** | ① `SetAcceleration(float _val)` 改 `SetAcceleration(float _val, bool persist = true)` ② `canBuf[4] = persist ? 1 : 0` ③ 加 `QueryAcceleration()` 函数（发 0x2C，不写 EEPROM）|
| **改动量** | .cpp **~25 行** + .hpp **~3 行** |
| **风险** | 🟡 中 —— `SetAcceleration` 签名变化，所有调用方都要传 persist |
| **依赖** | 无 |
| **连带改** | `dummy_robot.cpp:436` 内部调用必须加 `false` 参数（SetJointAcceleration 路径）|

**`.hpp` 改动**：

```cpp
void SetAcceleration(float _val, bool persist = true);  // 改签名
void QueryAcceleration();                              // 新增（A1）
```

**`.cpp` 改动**：

```cpp
void CtrlStepMotor::SetAcceleration(float _val, bool persist)
{
    uint8_t mode = 0x14;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = persist ? 1 : 0;  // 改：按参数

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

void CtrlStepMotor::QueryAcceleration()  // 新增（A1）
{
    uint8_t mode = 0x2C;
    txHeader.StdId = nodeID << 7 | mode;
    for (int i = 0; i < 8; i++) canBuf[i] = 0;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}
```

**fiber 协议 `MakeProtocolDefinitions`** —— **不动** —— fiber 协议 `set_acceleration` 仍只传 1 个 float（默认 `persist=true`）。

**编译验证**：

```bash
cd firmware/ref_core_f405/build ; ninja    # 0 errors
# 注意：line 436 不改的话会编译失败（缺参数）
```

---

### 2.4 T4 —— SetCurrentLimit 加 persist + QueryCurrentLimit（M3 + A2）

| 项 | 内容 |
|---|---|
| **改哪** | 同 T3（`ctrl_step.cpp` + `.hpp`）|
| **做什么** | ① `SetCurrentLimit(float _val)` 改 `SetCurrentLimit(float _val, bool persist = true)` ② `canBuf[4] = persist ? 1 : 0`（**修复 B2 bug**：之前每次都写 EEPROM）③ 加 `QueryCurrentLimit()` 函数（发 0x2D，不写 EEPROM）|
| **改动量** | .cpp **~25 行** + .hpp **~3 行** |
| **风险** | 🟡 中 —— 修 B2 flash bug |
| **依赖** | 无（与 T3 并行）|

**`.hpp` 改动**：

```cpp
void SetCurrentLimit(float _val, bool persist = true);  // 改签名
void QueryCurrentLimit();                               // 新增（A2）
```

**`.cpp` 改动**：

```cpp
void CtrlStepMotor::SetCurrentLimit(float _val, bool persist)
{
    uint8_t mode = 0x12;
    txHeader.StdId = nodeID << 7 | mode;

    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++)
        canBuf[i] = *(b + i);
    canBuf[4] = persist ? 1 : 0;  // 改：B2 修复

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

void CtrlStepMotor::QueryCurrentLimit()  // 新增（A2）
{
    uint8_t mode = 0x2D;
    txHeader.StdId = nodeID << 7 | mode;
    for (int i = 0; i < 8; i++) canBuf[i] = 0;

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}
```

---

### 2.5 T5 —— 新增 #GETJACC 和 #GETI 命令（A3 + A4）

| 项 | 内容 |
|---|---|
| **改哪** | `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp` |
| **做什么** | 加 `#GETJACC X` → `motorJ[X]->QueryAcceleration()`；加 `#GETI X` → `motorJ[X]->QueryCurrentLimit()`（节点 9=地轨、节点 8=夹爪）|
| **改动量** | **~60 行**（两个命令，每个 ~30 行）|
| **风险** | 🟢 低 —— 新增命令，不动现有 |
| **依赖** | T3 / T4（Query 函数必须存在）|
| **位置** | 第二处 ASCII 解析块末尾（line 1170 附近）|

**模板代码**（在第二个 else 块链尾添加）：

```cpp
else if (s.find("GETJACC") != std::string::npos)
{
    uint32_t node;
    sscanf(_cmd, "#GETJACC %lu", &node);
    if (node == 9) {
        dummy.motorJ[0]->QueryAcceleration();
    } else if (node >= 1 && node <= 6) {
        dummy.motorJ[node]->QueryAcceleration();
    } else if (node == 8) {
        dummy.hand->QueryAcceleration();
    } else {
        Respond(_responseChannel, "error GET MOTOR [%lu] ACCELERATION is wrong", node);
    }
}
else if (s.find("GETI") != std::string::npos)
{
    uint32_t node;
    sscanf(_cmd, "#GETI %lu", &node);
    if (node == 9) {
        dummy.motorJ[0]->QueryCurrentLimit();
    } else if (node >= 1 && node <= 6) {
        dummy.motorJ[node]->QueryCurrentLimit();
    } else if (node == 8) {
        dummy.hand->QueryCurrentLimit();
    } else {
        Respond(_responseChannel, "error GET MOTOR [%lu] CURRENT_LIMIT is wrong", node);
    }
}
```

**注意**：ASCII 协议中 `s.find("GET") != std::string::npos` 可能误匹配 —— 必须放在 `#ACC_J` / `#I_LIMIT_J` 之后。

---

### 2.6 T6 —— #ACC_J / #I_LIMIT_J 支持 `&` + sscanf 校验（M11 + M12 + M13）

| 项 | 内容 |
|---|---|
| **改哪** | `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp` |
| **做什么** | ① `#ACC_J` 两处（line 498-513, 1114-1128）sscanf 加 `%c` 读 `&` ② sscanf 返回值判断 `ret >= 2` ③ 调 `SetAcceleration(S, saveFlag == '&')` ④ 节点 8=夹爪、节点 9=地轨 ⑤ `#I_LIMIT_J` 同理（line 567-593, 1144-1169）|
| **改动量** | **~100 行**（每处 ~25 行 × 4）|
| **风险** | 🟡 中 —— 改 sscanf 格式 |
| **依赖** | T3（SetAcceleration 新签名）+ T4（SetCurrentLimit 新签名）|
| **位置** | **四处都要改**（第一处和第二处 ASCII 解析块各 2 个）|

**模板代码**（第一处 #ACC_J）：

```cpp
else if (s.find("ACC_J") != std::string::npos)
{
    float S;
    uint32_t node;
    char saveFlag = 0;  // 新增
    int ret = sscanf(_cmd, "#ACC_J %lu %f %c", &node, &S, &saveFlag);  // 改
    if (ret >= 2) {  // 改：检查返回值（M13 修复）
        bool persist = (saveFlag == '&');
        if (node == 9) {
            dummy.motorJ[0]->SetAcceleration(S, persist);  // 改：传 persist
            Respond(_responseChannel, "ok SET MOTOR [9] ACCELERATION [%f]", S);
        } else if (node >= 1 && node <= 6) {
            dummy.motorJ[node]->SetAcceleration(S, persist);
            Respond(_responseChannel, "ok SET MOTOR [%lu] ACCELERATION [%f]", node, S);
        } else if (node == 8) {
            dummy.hand->SetAcceleration(S, persist);
            Respond(_responseChannel, "ok SET MOTOR [8] ACCELERATION [%f] (夹爪)", S);
        } else {
            Respond(_responseChannel,
                    "error SET MOTOR [%lu] ACCELERATION [%f] is wrong", node, S);
        }
    } else {
        Respond(_responseChannel, "error ACC_J parse failed");
    }
}
```

**模板代码**（第一处 #I_LIMIT_J）：

```cpp
else if (s.find("I_LIMIT_J") != std::string::npos)
{
    float I;
    uint32_t node;
    char saveFlag = 0;
    int ret = sscanf(_cmd, "#I_LIMIT_J %lu %f %c", &node, &I, &saveFlag);
    if (ret >= 2) {
        bool persist = (saveFlag == '&');
        if (node == 9) {
            dummy.motorJ[0]->SetCurrentLimit(I, persist);
            Respond(_responseChannel, "ok SET MOTOR [9] CURRENT_LIMIT [%f] (地轨)", I);
        } else if (node >= 1 && node <= 6) {
            dummy.motorJ[node]->SetCurrentLimit(I, persist);
            Respond(_responseChannel, "ok SET MOTOR [%lu] CURRENT_LIMIT [%f]", node, I);
        } else if (node == 8) {
            dummy.hand->SetCurrentLimit(I, persist);
            Respond(_responseChannel, "ok SET MOTOR [8] CURRENT_LIMIT [%f] (夹爪)", I);
        } else {
            Respond(_responseChannel,
                    "error SET MOTOR [%lu] CURRENT_LIMIT [%f] is wrong", node, I);
        }
    } else {
        Respond(_responseChannel, "error I_LIMIT_J parse failed");
    }
}
```

**注意**：第一处和第二处 ASCII 解析块**结构略有差异** —— 第一处处理节点 0 / 1~6 / 8，第二处处理节点 0 / 1~6（无 8）。代码需根据实际情况调整。

---

### 2.7 T7 —— #ACC_RAIL 改用 motorJ[0]->SetAcceleration（M4 + M5）

| 项 | 内容 |
|---|---|
| **改哪** | `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp` |
| **做什么** | 两处 `#ACC_RAIL`（line 548-566, 980-998）改：① 删 `dummy.SetRailAcc()` 调用 ② 改调 `dummy.motorJ[0]->SetAcceleration(acc, saveFlag == '&')` ③ 删 `dummy.SaveConfig()` + `[saved to EEPROM]` ④ else 块返回当前值改用"请用 #GETJACC 9" |
| **改动量** | **~40 行**（每处 ~20 行 × 2）|
| **风险** | 🟡 中 —— 改 `SetRailAcc` 调用关系 |
| **依赖** | T3（SetAcceleration 新签名）+ T10（SetRailAcc 删除）|

**模板代码**（第一处）：

```cpp
else if (s.find("ACC_RAIL") != std::string::npos)
{
    float acc;
    char saveFlag;
    if (sscanf(_cmd, "#ACC_RAIL %f %c", &acc, &saveFlag) >= 1)
    {
        bool persist = (saveFlag == '&');
        dummy.motorJ[0]->SetAcceleration(acc, persist);  // 改：直接对电机 9 号
        Respond(_responseChannel, "ok rail acc set to %.1f", acc);
    }
    else
    {
        Respond(_responseChannel, "use #GETJACC 9 to query");  // 改：不再返回 railAcc_mm_s2
    }
}
```

**注意**：第二处的 else 块（第 2 处 line 996）也要改 —— `Respond(_responseChannel, "%.1f", dummy.railAcc_mm_s2);` 改同上。

---

### 2.8 T9 —— CAN 协议处理 0x2C/0x2D 响应（A5 + A6）

| 项 | 内容 |
|---|---|
| **改哪** | `firmware/ref_core_f405/UserApp/protocols/can_protocol.cpp` |
| **做什么** | ① id==9 switch 加 `case 0x2C` → 打印 `[ACC] MOTOR [9] = X.XX` ② id==9 switch 加 `case 0x2D` → 打印 `[I_LIMIT] MOTOR [9] = X.XX` ③ id==1~6 switch 同样两 case ④ **夹爪不查**（id==8 不加 case）|
| **改动量** | **~40 行**（4 处 case，每个 ~10 行）|
| **风险** | 🟢 低 —— 加 case |
| **依赖** | T1 / T2（电机返回数据格式）|

**模板代码**（id==9）：

```cpp
case 0x2C:
    printf("[ACC] MOTOR [9] = %.2f\r\n", *(float*)data);
    break;
case 0x2D:
    printf("[I_LIMIT] MOTOR [9] = %.2f\r\n", *(float*)data);
    break;
```

**模板代码**（id==1~6）：

```cpp
case 0x2C:
    printf("[ACC] MOTOR [%d] = %.2f\r\n", id, *(float*)data);
    break;
case 0x2D:
    printf("[I_LIMIT] MOTOR [%d] = %.2f\r\n", id, *(float*)data);
    break;
```

**注意**：id 在 line 68 解析：`uint8_t id = rxHeader->StdId >> 7;`

---

### 2.9 T10 —— 删除 railAcc_mm_s2 相关字段与函数（D5 + D6 + D7 + D8 + D12 + D13）

| 项 | 内容 |
|---|---|
| **改哪** | `firmware/ref_core_f405/Robot/instances/dummy_robot.h` + `.cpp` |
| **做什么** | ① 删 `EepromConfig::railAcc_mm_s2`（line 29）② 删 `float railAcc_mm_s2 = 500.0f;`（line 106）③ 删 `LoadConfig` 里 `railAcc_mm_s2` 读取（line 105-106）④ 删 `SaveConfig` 里 `config.railAcc_mm_s2 = railAcc_mm_s2;`（line 133）⑤ `MoveRail` 删 `acc_laps` 计算（line 189）和 `SetAcceleration(acc_laps)`（line 192）⑥ 删 `SetRailAcc` 函数（line 225-230）⑦ 删 `.h` 中 `SetRailAcc` 声明（line 209）和 fiber 协议（line 246）|
| **改动量** | .h **~3 行删** + .cpp **~10 行删** |
| **风险** | 🔴 **高** —— 影响 EEPROM 布局（字段偏移变化）|
| **依赖** | T7（ASCII 不再调 `SetRailAcc`）|
| **EEPROM 影响** | 见下方 3.1 节关键风险 |

**`.h` 改动**：

```cpp
// 删除 line 29
float railAcc_mm_s2;      // 地轨加速度 (mm/s²)

// 删除 line 106
float railAcc_mm_s2 = 500.0f;  // 地轨加速度 (mm/s²)，可通过 #ACC_RAIL 修改，默认 500

// 删除 line 209
void SetRailAcc(float _acc_mm_s2);

// 删除 line 246（fiber 协议）
make_protocol_function("set_rail_acc",    *this, &DummyRobot::SetRailAcc,    "acc"),
```

**`.cpp` 改动**：

```cpp
// 删除 line 105-106
if (config.railAcc_mm_s2 >= 10.0f && config.railAcc_mm_s2 <= 5000.0f)
    railAcc_mm_s2 = config.railAcc_mm_s2;

// 删除 line 133
config.railAcc_mm_s2 = railAcc_mm_s2;

// 删除 line 189（MoveRail 内）
float acc_laps = railAcc_mm_s2 / 5.0f;     // mm/s² → 圈/s²

// 删除 line 192（MoveRail 内）
motorJ[0]->SetAcceleration(acc_laps);

// 删除 line 225-230 整个 SetRailAcc 函数
void DummyRobot::SetRailAcc(float _acc_mm_s2)
{
    if (_acc_mm_s2 < 10.0f)         _acc_mm_s2 = 10.0f;
    else if (_acc_mm_s2 > 5000.0f)  _acc_mm_s2 = 5000.0f;
    railAcc_mm_s2 = _acc_mm_s2;
}
```

---

### 2.10 T11 —— 串口助手 UI1~UI4

| 项 | 内容 |
|---|---|
| **改哪** | `e:\Dummy-code\串口助手.py` |
| **做什么** | ① UI1：`#ACC_J X V` 成功后 0.5s 后自动 `#GETJACC X` ② UI2：`#I_LIMIT_J X I` 成功后 0.5s 后自动 `#GETI X` ③ UI3：J1~J6 批量按钮（发 6 次 `#ACC_J`）④ UI4：失能状态检查 + 弹窗 |
| **改动量** | **~80 行** |
| **风险** | 🟢 低 —— 串口助手独立模块 |
| **依赖** | T1~T9 全部完成（必须）|

**UI4 弹窗内容**：

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

## 3. 关键风险与对策

### 3.1 🔴 高风险：EEPROM 字段顺序变化（T10）

**问题**：
```
旧 EEPROM 布局（EEPROM.put(0, config)）:
┌─────────────────────────────────────┐
│ magic (uint32)                       │
│ static_r/g/b[3]                      │
│ rgbBrightness + rgbStateStart/Enable/Disable │
│ jointAccBases[6]                     │
│ railSpeed_mm_s ← 偏移 X              │
│ railAcc_mm_s2 ← 偏移 Y (要删)        │
└─────────────────────────────────────┘

新 EEPROM 布局:
┌─────────────────────────────────────┐
│ magic (uint32)                       │
│ static_r/g/b[3]                      │
│ rgbBrightness + rgbStateStart/Enable/Disable │
│ jointAccBases[6]                     │
│ railSpeed_mm_s ← 偏移 X (不变)       │
│ railAcc_mm_s2 ← 删了                │
└─────────────────────────────────────┘
```

**好消息**：`railAcc_mm_s2` 是**尾部字段** —— 删它不影响前面字段偏移 —— `railSpeed_mm_s` 仍能正确读出。

**唯一影响**：`EEPROM_MAGIC = 0x12345679` 校验可能失败（如果老设备用了不同版本） —— 失败时 `LoadConfig` 会**清空整个 EEPROM**（旧 `dummy_robot.cpp:92-99` 检查 magic）。

**对策**：
1. **升级后第一次上电** —— 用户需手动调一次 `#SPEED_RAIL X &` 重存 EEPROM
2. **发版说明** —— 明确告知"升级后重新设置速度"
3. **可选优化**（不在本次重构范围）—— magic 校验宽容（如读取多字段后求校验和）

### 3.2 🟡 中风险：SetAcceleration 签名变化（T3）

**问题**：`SetAcceleration(float)` → `SetAcceleration(float, bool)` —— **所有调用方都要传 persist**。

**调用方清单**：
- `dummy_robot.cpp:436` —— `SetJointAcceleration` 内部（**必须加 `false`**）
- ASCII 命令 `#ACC_J` 两处（T6 处理）
- ASCII 命令 `#ACC_RAIL` 两处（T7 处理）

**对策**：编译验证能查出所有遗漏 —— `ninja` 失败 = 遗漏。

### 3.3 🟡 中风险：ASCII 命令分两处解析（T6 + T7）

**问题**：`ascii_protocol.cpp` 里有**两个独立的 ASCII 解析块**（line ~460-600 和 line ~960-1170），每个块都有 `#ACC_J` / `#I_LIMIT_J` / `#ACC_RAIL`。

**对策**：
- T6 / T7 模板代码明确标注"两处都改"
- 第一处和第二处的**节点处理范围不同**（第一处有 8=夹爪，第二处只有 1~6）
- 编译后人工核对"4 处 #ACC_J 是否都被改"

### 3.4 🟡 中风险：0x2C/0x2D 响应格式

**问题**：电机回包数据格式必须和主控解析对齐。

**格式约定**：
- 0x2C：`data[0..3]` = `float`（圈/s²，已除 `MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS`）
- 0x2D：`data[0..3]` = `float`（A，已除 1000）

**对策**：
- T1 / T2 模板代码使用相同结构（除数和换算）
- T9 主控直接 `*(float*)data` 读取并 printf —— 不再做二次换算

### 3.5 🟢 低风险：B1 (#SPEED_J) bug 标记为"已知 bug，本次不修"

**问题**：`SetVelocityLimit` 当前 `canBuf[4]=1` 硬编码 —— 每次 `#SPEED_J` 都写 EEPROM。

**对策**：
- **不修**（用户 2026-08-07 01:21 决策）
- 串口助手**没有 `#SPEED_J` 按钮**（grep 验证 `SPEED_J` 0 处匹配）
- 实际影响为零

### 3.6 🟢 低风险：夹爪电机不需要 0x2C/0x2D 处理（T9）

**问题**：id==8 是夹爪 —— 现状 `can_protocol.cpp` 中 `id==8` 只处理 0x23 / 0x25。

**对策**：
- T9 **不加** `id==8` 的 0x2C/0x2D 处理
- 串口助手发 `#GETJACC 8` → 电机回 0x2C → **主控忽略**（无对应 case）
- 用户层面："夹爪不需要查加速度/电流" —— 设计上不暴露

---

## 4. 编译验证清单

### 4.1 主控

```bash
cd firmware/ref_core_f405/build
ninja
# 预期：0 errors, 0 warnings（既有 warning 不算）
```

### 4.2 4 份电机固件

```bash
cd firmware/motor_fw_f103_35/build ; ninja
cd firmware/motor_fw_f103_42/build ; ninja
cd firmware/motor_fw_f103_57/build ; ninja
cd firmware/motor_fw_f103_gripper/build ; ninja
# 每个都预期 0 errors
```

### 4.3 检查 RAM/Flash 使用率

```bash
# ninja 输出末尾会显示：
# Memory region         Used Size  Region Size  %age Used
#             RAM:       12345 B       128 KB      9.62%
#            Flash:       56789 B       512 KB     10.83%
```

**注意**：T1 + T2 + T3 + T4 共增加 ~100 行代码 —— 预计 Flash 使用率增加 < 0.5%。

---

## 5. 烧录 + 集成测试

### 5.1 烧录顺序

```
1. 烧录 4 份电机固件（新 .bin/.hex）
   - motor_fw_f103_35/build/firmware.bin
   - motor_fw_f103_42/build/firmware.bin
   - motor_fw_f103_57/build/firmware.bin
   - motor_fw_f103_gripper/build/firmware.bin

2. 烧录主控
   - ref_core_f405/build/firmware.bin
```

### 5.2 集成测试清单

| 测试项 | 命令 | 期望 |
|---|---|---|
| **基本通信** | 发 `#GETJPOS` | 主控回各关节角度 |
| **临时加速度** | 发 `#ACC_J 3 100` | 回 `ok SET MOTOR [3] ACCELERATION [100.000000]` |
| **持久化加速度** | 发 `#ACC_J 3 100 &` | 回同上 + 电机 EEPROM 写入 |
| **查询加速度** | 发 `#GETJACC 3` | 主控打印 `[ACC] MOTOR [3] = 100.00` |
| **临时电流** | 发 `#I_LIMIT_J 3 1.5` | 回 `ok SET MOTOR [3] CURRENT_LIMIT [1.500000]` |
| **持久化电流** | 发 `#I_LIMIT_J 3 1.5 &` | 回同上 + 电机 EEPROM 写入 |
| **查询电流** | 发 `#GETI 3` | 主控打印 `[I_LIMIT] MOTOR [3] = 1.50` |
| **地轨加速度** | 发 `#ACC_RAIL 50` | 回 `ok rail acc set to 50.0`（电机 9 号写 RAM）|
| **地轨加速度持久** | 发 `#ACC_RAIL 50 &` | 回同上 + 电机 9 号写 EEPROM |
| **地轨速度** | 发 `#SPEED_RAIL 30` | 回 `ok rail speed set to 30.0 mm/s`（仅 RAM）|
| **地轨速度持久** | 发 `#SPEED_RAIL 30 &` | 回同上 + `[saved to EEPROM]` |
| **基础加速度保留** | 发 `#ACC_BASE_J 3 250` | 回 `ok MOTOR [3] BASE ACCELERATION [250.000000]` |
| **基础加速度读回** | 发 `#ACC_BASE_J 3`（不带值） | 回 `250.000000` |
| **电机重启测试** | 重新上电机 1~6 之一 | 持久化的 `&` 值仍生效 |

### 5.3 24 小时 stress test

```
1. 主控 + 4 份电机全部上电
2. 反复触发 50 次 #ACC_J / #I_LIMIT_J（带 &）
3. 检查电机 EEPROM 是否仍正常（电机重启后参数保留）
4. 检查主控 UART 是否仍正常（无多余 flash 写入告警）
```

---

## 6. 与 REFACTOR_DEMANDS.md 的对应关系

| 文档 ID | 任务编号 | 状态 |
|---|---|---|
| M1 | T3 | ✅ 待执行 |
| M3 | T4 | ✅ 待执行 |
| M4 | T7 (第一处 #ACC_RAIL 带&) | ✅ 待执行 |
| M5 | T7 (第一处 #ACC_RAIL 不带&) | ✅ 待执行 |
| M6 | （不动 —— 保持现状）| - |
| M7 | （不动 —— 保持现状）| - |
| M11 | T6 (两处 #ACC_J) | ✅ 待执行 |
| M12 | T6 (两处 #I_LIMIT_J) | ✅ 待执行 |
| M13 | T6 (sscanf `>= 2`) | ✅ 已并入 T6 |
| ~~M2~~ | （撤销 —— 2026-08-07 01:21）| - |
| ~~M8~~ | （撤销 —— #SPEED_J 不动）| - |
| ~~M9~~ | （撤销 —— 未经用户决策）| - |
| ~~M10~~ | （撤销 —— jointAccBases 保留）| - |
| ~~M14~~ | （撤销 —— fiber 协议原状）| - |
| A1 | T3 | ✅ 待执行 |
| A2 | T4 | ✅ 待执行 |
| A3 | T5 | ✅ 待执行 |
| A4 | T5 | ✅ 待执行 |
| A5 | T9 | ✅ 待执行 |
| A6 | T9 | ✅ 待执行 |
| A7 | T1 (×4 电机) | ✅ 待执行 |
| A8 | T2 (×4 电机) | ✅ 待执行 |
| D5 | T10 | ✅ 待执行 |
| D6 | T10 | ✅ 待执行 |
| D7 | T10 | ✅ 待执行 |
| D8 | T10 | ✅ 待执行 |
| D12 | T10 | ✅ 待执行 |
| D13 | T10 | ✅ 待执行 |
| ~~D1/D2/D3/D4/D9/D10/D11~~ | （撤销 —— 2026-08-07 01:21）| - |
| B2 | T4 修复 | ✅ 待执行 |
| B5 | T6 修复（sscanf `>= 2`）| ✅ 待执行 |
| B6 | T6 修复（sscanf `>= 2`）| ✅ 待执行 |
| ~~B1~~ | （撤销 —— 已知 bug，本次不修）| - |
| ~~B3~~ | （撤销 —— 不在本次重构范围）| - |
| ~~B4~~ | （撤销 —— jointAccBases 保留，bug 不存在）| - |
| ~~B7~~ | （撤销 —— M14 撤销）| - |
| UI1 | T11 | ✅ 待执行 |
| UI2 | T11 | ✅ 待执行 |
| UI3 | T11 | ✅ 待执行 |
| UI4 | T11 | ✅ 待执行 |

---

## 7. 总体工作量汇总

| 文件 | 改动量 | 涉及任务 |
|---|---|---|
| `motor_fw_f103_35/interface_can.cpp` | ~24 行 | T1+T2 |
| `motor_fw_f103_42/interface_can.cpp` | ~24 行 | T1+T2 |
| `motor_fw_f103_57/interface_can.cpp` | ~24 行 | T1+T2 |
| `motor_fw_f103_gripper/interface_can.cpp` | ~24 行 | T1+T2 |
| `ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` | ~50 行 | T3+T4 |
| `ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.hpp` | ~6 行 | T3+T4 |
| `ref_core_f405/UserApp/protocols/ascii_protocol.cpp` | ~200 行 | T5+T6+T7 |
| `ref_core_f405/UserApp/protocols/can_protocol.cpp` | ~40 行 | T9 |
| `ref_core_f405/Robot/instances/dummy_robot.h` | ~3 行删 | T10 |
| `ref_core_f405/Robot/instances/dummy_robot.cpp` | ~10 行删 + 1 行改（line 436）| T10 + T3 联动 |
| `串口助手.py` | ~80 行 | T11 |
| **合计** | **~430 行** | **10 个有效任务（T1~T7+T9~T11）** |

---

## 8. 决策回顾（2026-08-07 01:21 用户最终决策）

### 8.1 用户原话

> "可以，你的解释很清晰，先跟新文档吧，最大速度和主控的基础加速度留着不动就好了，改完后我来检查一遍"

> "可以，启动重构任务规划吧，规划完我来看看合不合适"

### 8.2 关键决策

| # | 决定 | 来源 | 状态 |
|---|---|---|---|
| 1 | `jointAccBases` 保留 | 2026-08-07 01:21 | ✅ 已生效 |
| 2 | `railAcc_mm_s2` 删除 | Q4 + 2026-08-07 01:21 | ✅ T10 |
| 3 | `MoveRail` 不再发 SetAcceleration | Q4 | ✅ T10 |
| 4 | `#ACC_BASE_J` 保留 | 2026-08-07 01:21 | ✅ 已生效 |
| 5 | `#SPEED_J` 完全不动 | 2026-08-07 01:21 | ✅ 已生效（B1 标记） |
| 6 | `#ACC_RAIL X [&]` 支持 `&` | Q5.1 + Q4 | ✅ T7 |
| 7 | `#ACC_J X S [&]` 支持 `&` | Q5.5 | ✅ T6 |
| 8 | `#I_LIMIT_J X I [&]` 支持 `&` | Q5.6 | ✅ T6 |
| 9 | `#SPEED_RAIL X &` 保留存主控 EEPROM | Q5.3 选 c | ✅ 已生效（不动） |
| 10 | `#GETJACC / #GETI` 查询命令 | 新增 | ✅ T5 |
| 11 | 主控同步等 100ms | Q2 | ✅ T9 |
| 12 | 单位用"圈/s²"和"A" | 第一次回答 | ✅ T1/T2/T9 |
| 13 | 失能状态检查 + 弹窗 | Q5 | ✅ T11 (UI4) |
| 14 | 0.5s 回查责任在串口助手 | 第一次回答 | ✅ T11 (UI1/UI2) |
| 15 | 不优化电机固件 | 第三次回答 | ✅ 已生效 |

### 8.3 撤销项（2026-08-07 01:21）

- ❌ D1 / D2 / D3 / D4 —— 删 `jointAccBases` 字段 —— 撤销
- ❌ D10 —— 删 `#ACC_BASE_J` 命令 —— 撤销
- ❌ D11 —— 删"查表乘百分比"逻辑 —— 撤销
- ❌ M2 / M8 / M9 / M10 / M14 —— 重构各种内部函数 —— 撤销
- ❌ B1 / B4 / B7 —— 预存 bug 修复 —— 撤销（B1 标记已知）

---

## 9. 后续步骤

### 9.1 用户接下来

1. ⏳ **审阅本文档** —— 检查任务清单、风险评估、模板代码
2. ⏳ 通过对话告诉 AI 助手**"开始 T1"** 或**"暂停再讨论"**
3. ⏳ 每个任务完成后**确认编译通过**再进入下一任务

### 9.2 AI 助手接下来

1. ⏳ 用户审批后**按 T1~T11 顺序执行**
2. ⏳ 每个任务完成后**主动运行 ninja 验证**
3. ⏳ 如果 ninja 失败 —— **立即报告 + 暂停**等用户决策
4. ⏳ 完成后输出 commit message 草稿，等用户确认

---

## 附录 A：commit 模板

每个任务的 commit message 推荐格式（参考 `git log` 现有风格）：

```
<module>: <action> [<detail>]

例:
refactor(motor-fw): 加 0x2C/0x2D 处理（查询加速度/电流）
refactor(ctrl-step): SetAcceleration/SetCurrentLimit 加 persist 参数 + Query 函数
refactor(ascii-protocol): #ACC_J/#I_LIMIT_J 支持 &，sscanf 校验返回值
refactor(ascii-protocol): #ACC_RAIL 改用 motorJ[0]->SetAcceleration
refactor(can-protocol): 加 0x2C/0x2D 响应处理（打印 [ACC]/[I_LIMIT]）
refactor(dummy-robot): 删除 railAcc_mm_s2 字段和 SetRailAcc 函数
docs(refactor): 加 REFACTOR_PLAN.md 任务规划文档
```

---

## 附录 B：相关文档

| 文档 | 路径 | 说明 |
|---|---|---|
| `REFACTOR_DEMANDS.md` | `e:\Dummy-code\REFACTOR_DEMANDS.md` | 需求背景、用户决策、开放问题 |
| `REFACTOR_PLAN.md` | `e:\Dummy-code\REFACTOR_PLAN.md` | **本文档**——任务规划 |
| `PROJECT_CONTEXT.md` | `e:\Dummy-code\PROJECT_CONTEXT.md` | 项目架构、CAN 协议、命令格式 |
| `README.md` | `e:\Dummy-code\README.md` | 项目主文档、用户指南 |
| `ISSUES.md` | `e:\Dummy-code\ISSUES.md` | P0/P1/P2/P3 问题追踪 |
| `TODO.md` | `e:\Dummy-code\TODO.md` | 功能路线图 |

---

## 附录 C：术语表

| 术语 | 含义 |
|---|---|
| **主控** | STM32F405RG（ref_core_f405）—— 中央控制器 |
| **电机固件** | STM32F103CBT6（motor_fw_f103_35/42/57/gripper）—— 电机驱动板 |
| **5kHz 环** | 电机固件的位置控制循环（每 200μs 跑一次）|
| **CAN StdId** | 标准帧 ID 格式：`StdId = (nodeID << 7) \| cmdCode` |
| **nodeID** | 电机节点 ID（地轨=9、关节=1~6、夹爪=8）|
| **cmdCode** | CAN 命令码（0x00~0x7F 普通，0x80~0xBF 广播）|
| **canBuf[4]** | 电机固件 0x12/0x14 命令的第 5 字节，标记是否写 EEPROM（0=不写，1=写）|
| **EEPROM 写入** | STM32F103 内部 flash 写入，擦写寿命 ~10,000 次 |
| **fiber 协议** | 基于 libfibre 的 RPC 协议（`set_joint_acc` 等）|
| **0x2C / 0x2D** | 新增 CAN 命令码 —— 查询加速度 / 查询电流限幅 |
| **MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS** | 每圈步数 = 200 × 256 = 51200（电机内部常数）|
| **jointAccBases** | 主控基础加速度数组 `{150, 100, 200, 200, 200, 200}`（开发者实测）|
| **railAcc_mm_s2** | 地轨加速度字段（mm/s²）—— 本次重构删除 |
| **railSpeed_mm_s** | 地轨速度字段（mm/s）—— 保留 + EEPROM 存 |

---

**文档结束**

> AI 助手：根据用户 2026-08-07 01:21 决策制定的任务规划文档。
> 下一步：用户审阅 → 审批 → 按 T1~T11 顺序执行。