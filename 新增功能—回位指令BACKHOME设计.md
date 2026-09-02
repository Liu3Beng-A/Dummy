# 新增「回位」指令 !BACKHOME

## 功能概述

`!BACKHOME` 指令用于将机械臂所有轴归位到统一的安全姿态和物理零点：

- **J1~J6 关节**：回到 `!RESET` 待机姿态
- **地轨（ID=9）**：向**负方向**移动找物理限位 → 记录限位为 `−250mm` → 向正方向运动回 `0mm`（行程中间）
- **夹爪（ID=8）**：向**闭合方向**（`0` 方向）移动找物理限位 → 记录限位为 `0` → 向张开方向运动回 `100`（完全张开）

> **重要约定**：
>
> - **地轨**：限位值 `−250mm`（负向端），目标值 `0mm`（行程中间）
> - **夹爪**：限位值 `0`（闭合端），目标值 `100`（张开端）
> - 两个电机的算法**完全对称**：都是"向负方向找物理限位 → 记录限位值 → 向正方向运动到目标位置"

核心机制：**复用地轨电机和夹爪电机的现有堵转检测逻辑**判断物理限位到达，然后执行"后退→精确定位→回目标"三段式归零流程。

### 地轨与夹爪的算法对称性

两个电机用**完全相同的算法**，只是方向参数和数值不同：


| 阶段           | 地轨                        | 夹爪                                   |
| ------------ | ------------------------- | ------------------------------------ |
| Phase2 寻限位方向 | 负方向                       | 闭合方向（`SetAngleWithCurrentLimit(-1)`） |
| 限位值（小）       | `−250mm`                  | `0`（完全闭合）                            |
| 目标值（大）       | `0mm`（行程中间）               | `100`（完全张开）                          |
| 回退方向         | 正方向                       | 张开方向                                 |
| 主控侧偏移        | `railHomeOffsetMm = −250` | 无需（夹爪固件自带 `0/100` 映射）                |
| Phase5 目标电机角 | `+50 圈`                   | `SetAngleWithSpeedLimit(100)`        |


> **代码复用建议**：Phase2~Phase5 的状态机可以用一个通用函数 `BackHomeSingleAxis(axis_id)` 实现，地轨和夹爪只是参数不同。文档中为清晰起见分别描述，实际实现可考虑抽象。

---



## 状态机设计



### 整体流程

```
Idle ──(!BACKHOME)──> Phase1_ResetJoints ──(关节到位)──>
Phase2_SeekLimits ──(地轨+夹爪同时触发堵转)──>
Phase3_Retreat ──(后退完成)──>
Phase4_Approach ──(再次到达限位)──>
Phase5_ReturnZero ──(回到0)──>
Idle (ok)
```



### 阶段详解



#### Phase1：关节归位（同步阻塞）

**目的**：J1~J6 先回到 RESET 姿态，避免机械臂在归零过程中干涉地轨/夹爪的运动空间。

**动作**：

```cpp
void DummyRobot::BackHome()
{
    // ── Phase1: 关节归位 ──
    float lastSlider = jointSpeedRps / SLIDER_TO_RPS;
    SetJointSpeed(10);
    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5], 0, 10);
    MoveJoints(targetJoints);
    // 地轨先归零位（Phase2 并行运动时不参与，避免干扰）
    motorJ[0]->SetPositionWithMotorRps(0, railSpeedRps);
    while (IsMoving())  // 等待 J1~J6 + 地轨全部到位
        osDelay(10);
    SetJointSpeed(lastSlider);

    // ── Phase2: 地轨+夹爪同步找限位 ──
    // 关节已到位，机械臂在 RESET 姿态，地轨和夹爪可以安全运动
    StartSeekLimits();
}
```

**参数**：


| 参数   | 值                      | 说明                 |
| ---- | ---------------------- | ------------------ |
| 速度   | slider=10              | 低速，确保归位安全          |
| 等待方式 | `while(IsMoving())` 阻塞 | J1~J6 + 地轨全部到位后才继续 |


---



#### Phase2：地轨+夹爪同步寻限位（非阻塞）

**目的**：地轨和夹爪**同时**向各自物理限位方向移动，利用堵转检测判定到达限位。

**动作**：

```cpp
void DummyRobot::StartSeekLimits()
{
    backHomeState = BACKHOME_SEEK_LIMITS;
    railBackHomeDone = false;
    handBackHomeDone = false;

    // 地轨：以极低恒速向负方向运动，直到堵转触发
    // 堵转阈值复用现有 stallCurrentThreshold（ratedCurrent × 40%）
    motorJ[0]->SetVelocityWithMotorRps(-0.2f);  // 0.2 r/s ≈ 1mm/s，向 −250 限位

    // 夹爪：以闭合力矩模式向 0 方向运动，直到堵转触发
    hand->SetAngleWithCurrentLimit(-1);  // -1 = 向 0 方向（闭合），与张开方向相反
}
```

**参数**：


| 参数           | 地轨（ID=9）                        | 夹爪（ID=8）                         |
| ------------ | ------------------------------- | -------------------------------- |
| 运动方式         | 速度模式（`SetVelocityWithMotorRps`） | 电流模式（`SetAngleWithCurrentLimit`） |
| 方向           | 负方向（向 `−250mm` 限位）              | 闭合方向（向 `0` 限位）                   |
| 速度/电流        | `0.2 r/s`（约 `1mm/s`）            | `1`（满电流，与 `!HAND_C` 同）           |
| 限位判定         | 堵转检测（STALL_IDLE→RETREATING）     | 堵转检测（STALL_IDLE→RETREATING）      |
| 记录限位值        | `−250mm`                        | `0`（完全闭合）                        |
| 目标位置（Phase5） | `0mm`（行程中间）                     | `100`（完全张开）                      |
| 回零方向         | 正方向（向 `+250mm`）                 | 张开方向（向 `100`）                    |


**堵转触发后的电机侧行为**（复用现有 `RETREATING` 状态机）：

```
堵转条件满足（电流大 + 速度低 + 误差大，持续 30ms）
    │
    ▼
电机侧触发 RETREATING → 自动回退 35556 步（约 5mm / 5°）
    │
    ▼
回退完成 → LOCKED（电机停在回退后位置）
    │
    ▼
主控检测到 LOCKED 状态 → 进入 Phase3
```

**主控侧检测堵转完成的逻辑**（在 FreeRTOS 任务中轮询）：

```cpp
void DummyRobot::UpdateBackHome()
{
    // ── 地轨限位检测 ──
    if (!railBackHomeDone && motorJ[0]->IsStalled()) {
        // 电机已进入 LOCKED，railAngle 缓存了回退后的角度
        railLimitAngle = motorJ[0]->angle;  // 回退后的角度
        railBackHomeDone = true;
        railBackHomeState = BACKHOME_RETREAT;
        // 通知电机端退出 LOCKED（不发 UNLOCKED，只发 enable=1 保持控制）
        motorJ[0]->SetEnable(true);
    }

    // ── 夹爪限位检测 ──
    if (!handBackHomeDone && hand->IsStalled()) {
        handLimitAngle = hand->angle;  // 回退后的角度
        handBackHomeDone = true;
        handBackHomeState = BACKHOME_RETREAT;
        hand->SetEnable(true);
    }
}
```

> **注**：`IsStalled()` 是主控侧堵转标志（电机 CAN 回包设置），电机固件已在堵转后自动执行了回退并进入 LOCKED。

---



#### Phase3：后退（Retreat）

**目的**：地轨和夹爪各自由电机固件自动回退后（已完成），主控记录回退后的位置作为精确定位的起点。

**动作**：

- 地轨：从 `railLimitAngle` 位置，以极低速度**向原方向继续移动一小段**
- 夹爪：从 `handLimitAngle` 位置，以极低速度**向原方向继续移动一小段**

```cpp
// 地轨后退：从限位角向正方向移动 0.5mm（10240 步），脱离物理接触
railBackHomeState = BACKHOME_RETREAT;
float retreatLaps = 0.5f / 5.0f;  // +0.1 圈（正方向，脱离限位）
motorJ[0]->SetPositionWithMotorRps(
    railLimitAngle + retreatLaps, 0.1f);

// 夹爪后退：从限位角（0）向张开方向移动 3 个单位（脱离完全闭合）
handBackHomeState = BACKHOME_RETREAT;
hand->SetAngleWithSpeedLimit(3);  // pos=3，张开方向
```

**后退完成后自动进入 Phase4**（等待 `IsMoving()==false`）。

---



#### Phase4：精确定位到限位（Approach）

**目的**：从 Phase3 的后退位置出发，以更低速度再次接近限位，确保精确定位到物理接触点。

**动作**：

```cpp
// 地轨：向负方向缓慢接近限位（重新碰触），速度 0.05 r/s（约 0.25mm/s）
railBackHomeState = BACKHOME_APPROACH;
motorJ[0]->SetVelocityWithMotorRps(-0.05f);  // 更慢，更精确

// 夹爪：向 0 方向（闭合）缓慢接近限位（重新夹紧）
handBackHomeState = BACKHOME_APPROACH;
hand->SetAngleWithSpeedLimit(0.5);  // 张开方向 0.5 单位速度，向闭合方向移动
```

**等待堵转再次触发** → 电机再次回退 → 记录此次 `railLimitAngle` / `handLimitAngle` 作为**精确限位角度**。

> **注**：Phase3/Phase4 的后退+精确定位是为了消除回退产生的机械间隙，确保记录的限位角是真正的物理接触点。

---



#### Phase5：设置零点偏移 → 回零 → 返回 OK

**目的**：将 Phase4 记录的精确限位角设为各自的零点偏移，然后运动到 0 位置。

**动作**：

```cpp
// ── 设置零点偏移 ──
// 地轨：Phase4 记录限位角 → ApplyPositionAsHome() 把限位角映射为 "0"
// 然后主控侧加上 −250mm 偏移，让限位处显示为 −250mm
motorJ[0]->ApplyPositionAsHome();     // 当前角 → 零点（电机角 = 0）
// 主控侧：限位处 → currentRailPos = −250mm
railHomeOffsetMm = -250.0f;           // 限位 → −250mm 的常量偏移
// 之后 currentRailPos = motorJ[0]->angle / 360 * 5 + railHomeOffsetMm

// 夹爪：Phase4 记录限位角 → ApplyPositionAsHome() 把限位角映射为 "0"
// 夹爪 setAngle(pos) 中 pos=0 对应限位角（完全闭合）→ 正方向 → 100 张开
hand->ApplyPositionAsHome();          // 当前角 → 零点

// ── 回目标位置（限位的对侧）──
railBackHomeState = BACKHOME_RETURN_ZERO;
// 地轨：向正方向运动到 0mm（限位角 +0mm 对应角 = railHomeAngle）
motorJ[0]->SetPositionWithMotorRps(
    (0.0f - railHomeOffsetMm) / 5.0f,  // 0mm 对应的电机角（限位角 + 250mm 位移）
    0.5f);                              // 慢速回正

// 夹爪：向张开方向运动到 100（限位角 + 100% 张开单位）
hand->SetAngleWithSpeedLimit(100);     // pos=100，完全张开

while (IsMoving() || motorJ[0]->IsMoving() || hand->IsMoving())
    osDelay(10);

// ── 返回 OK ──
backHomeState = BACKHOME_IDLE;
Respond(_responseChannel, "ok backhome done: rail=0mm, joints=RESET, hand=100");
```

---



## 零点偏移策略



### 核心思路

两个电机的算法**完全一致**，都是：

1. Phase4 记录**限位处的精确电机角** → 通过 `ApplyPositionAsHome()` 把这个角设为新的"电机角 0"
2. 主控侧用**软件偏移**把电机角 0 映射为"限位值"（−250mm 或 0）
3. Phase5 用 `SetPositionWithMotorRps(目标角)` 或 `SetAngleWithSpeedLimit(目标开度)` 运动到对侧目标位置



### 地轨（ID=9）

**限位值与目标值的映射**：


| 位置      | 电机角（ApplyHome 后） | currentRailPos（用户视图） |
| ------- | ---------------- | -------------------- |
| 负方向物理限位 | `0`              | `−250mm`             |
| 行程中间    | `+50 圈`          | `0mm`                |
| 正方向物理限位 | `+100 圈`         | `+250mm`             |


**实现**：

```cpp
// dummy_robot.h 新增成员
float railHomeOffsetMm = -250.0f;   // 限位 → −250mm 的常量偏移（Phase5 时赋值）

// UpdateJointAnglesCallback() 中修正 currentRailPos 计算：
// 原来：currentRailPos = motorJ[0]->angle / 360.0f * 5.0f;
// 现在：
currentRailPos = motorJ[0]->angle / 360.0f * 5.0f + railHomeOffsetMm;

// Phase4 完成后（电机已 ApplyPositionAsHome()）：
railHomeOffsetMm = -250.0f;   // 限位处 = −250mm
// 此时电机角 0 = 限位 = −250mm
// 0mm 行程中间 = 电机角 = 250/5 圈 = 50 圈 = 18000°
```

**Phase5 回 0mm 的电机目标角**：

```cpp
// 0mm 目标位置 → 电机角 = (0mm - railHomeOffsetMm) / 5 = (0 - (-250)) / 5 = 50 圈
motorJ[0]->SetPositionWithMotorRps(
    (0.0f - railHomeOffsetMm) / 5.0f,  // = 50.0 圈
    0.5f);
```



### 夹爪（ID=8）

**限位值与目标值的映射**：


| 位置     | 电机角（ApplyHome 后） | 夹爪开度（用户视图）  |
| ------ | ---------------- | ----------- |
| 闭合物理限位 | `0`              | `0`（完全闭合）   |
| 半开     | `中转角`            | `50`        |
| 张开物理限位 | `最大角`            | `100`（完全张开） |


**实现**：

```cpp
// dummy_robot.h / StepHand 不需要新增成员
// 直接复用现有 ApplyPositionAsHome() 和 SetAngleWithSpeedLimit(pos)
//
// Phase4 完成后（电机已 ApplyPositionAsHome()）：
// 限位角 = 电机角 0 = 夹爪开度 0（完全闭合）
//
// Phase5 回 100（完全张开）：
hand->SetAngleWithSpeedLimit(100);   // pos=100 = 电机角 = 张开物理限位
```

> **关键**：夹爪固件本身已经实现了 `0=闭合, 100=张开` 的角度映射，我们只需要 `ApplyPositionAsHome()` 把限位角标记为"电机角 0"，再调用 `SetAngleWithSpeedLimit(100)` 即可运动到张开限位。



### 两个电机的算法对称性总结


| 步骤                          | 地轨（ID=9）                  | 夹爪（ID=8）                         |
| --------------------------- | ------------------------- | -------------------------------- |
| Phase2 寻限位方向                | 负方向                       | 闭合方向（向 `0`）                      |
| 记录限位值                       | `−250mm`                  | `0`（完全闭合）                        |
| Phase5 目标位置                 | `0mm`                     | `100`（完全张开）                      |
| Phase5 回零方向                 | 正方向（向 `+250mm`）           | 张开方向                             |
| 目标与限位的差值                    | `+250mm`                  | `+100 开度单位`                      |
| ApplyPositionAsHome 后的电机角 0 | 限位电机角（−250mm 处）           | 限位电机角（闭合处）                       |
| 主控侧偏移                       | `railHomeOffsetMm = −250` | 无需（`SetAngleWithSpeedLimit` 已处理） |


---



## 地轨 vs 夹爪的行为差异


| 特性     | 地轨（ID=9）                                           | 夹爪（ID=8）                             |
| ------ | -------------------------------------------------- | ------------------------------------ |
| 寻限位方式  | 速度模式（恒速 −0.2r/s）                                   | 电流模式（闭合力矩）                           |
| 堵转触发条件 | 速度低+电流大+误差大                                        | 同左（电机固件统一逻辑）                         |
| 回退行为   | 电机固件自动回退 51200 步（约 5mm）                            | 同左                                   |
| 回零方式   | `SetPositionWithMotorRps((0−offset)/5, 0.5)` → 0mm | `SetAngleWithSpeedLimit(100)` → 完全张开 |
| 零点含义   | 物理限位处 = `−250mm`                                   | 完全闭合 = `0`（home 后 100 = 完全张开）        |


---



## 主控侧状态机实现



### dummy_robot.h 新增成员

```cpp
// 回位状态机
enum BackHomeState {
    BACKHOME_IDLE = 0,
    BACKHOME_RESET_JOINTS,      // Phase1: 关节归位
    BACKHOME_SEEK_LIMITS,       // Phase2: 同步寻限位
    BACKHOME_RETREAT,           // Phase3: 后退
    BACKHOME_APPROACH,          // Phase4: 精确定位
    BACKHOME_RETURN_ZERO,       // Phase5: 回零
};
BackHomeState backHomeState = BACKHOME_IDLE;
BackHomeState railBackHomeState = BACKHOME_IDLE;
BackHomeState handBackHomeState = BACKHOME_IDLE;

bool railBackHomeDone = false;
bool handBackHomeDone = false;

float railHomeAngle = 0.0f;       // 地轨物理限位电机角（−250mm 处）
float handHomeAngle = 0.0f;       // 夹爪完全闭合电机角（0 处）
float railHomeOffsetMm = 0.0f;    // 地轨主控侧软偏移：限位处 = −250mm（默认 0 表示未归位）

bool backHomeComplete = false;    // Phase5 完成标志（main 循环检测后清零）

void BackHome();                   // 入口
void StartSeekLimits();            // Phase2 触发
void UpdateBackHome();             // 状态机主更新（每个控制周期调用）
```



### dummy_robot.cpp 核心实现

```cpp
void DummyRobot::BackHome()
{
    if (backHomeState != BACKHOME_IDLE) return;  // 防重入
    backHomeState = BACKHOME_RESET_JOINTS;

    float lastSlider = jointSpeedRps / SLIDER_TO_RPS;
    SetJointSpeed(10);

    // Phase1: J1~J6 + 地轨归位
    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5], 0, 10);
    MoveJoints(targetJoints);
    motorJ[0]->SetPositionWithMotorRps(0, railSpeedRps);

    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSlider);

    // Phase2: 地轨+夹爪同步寻限位
    StartSeekLimits();
}

void DummyRobot::StartSeekLimits()
{
    backHomeState = BACKHOME_SEEK_LIMITS;
    railBackHomeDone = false;
    handBackHomeDone = false;
    railBackHomeState = BACKHOME_SEEK_LIMITS;
    handBackHomeState = BACKHOME_SEEK_LIMITS;

    // 地轨：负方向恒速（堵转触发后电机固件自动回退）
    motorJ[0]->SetVelocityWithMotorRps(-0.2f);   // 地轨：负方向恒速 → −250 限位

    // 夹爪：闭合方向电流模式（−1 = 向 0 方向，堵转触发后电机固件自动回退）
    hand->SetAngleWithCurrentLimit(-1);
}

void DummyRobot::UpdateBackHome()
{
    switch (backHomeState) {
        case BACKHOME_IDLE:
            break;

        case BACKHOME_SEEK_LIMITS: {
            // 地轨：检测堵转（电机已回退到 LOCKED）
            if (!railBackHomeDone && motorJ[0]->isStalled) {
                railLimitAngle = motorJ[0]->angle;
                railBackHomeDone = true;
                railBackHomeState = BACKHOME_RETREAT;
                motorJ[0]->SetEnable(true);  // 退出 LOCKED
            }
            // 夹爪：检测堵转
            if (!handBackHomeDone && hand->isStalled()) {
                handLimitAngle = hand->angle;
                handBackHomeDone = true;
                handBackHomeState = BACKHOME_RETREAT;
                hand->SetEnable(true);
            }
            // 两轴都触发堵转后，进入 Phase3
            if (railBackHomeDone && handBackHomeDone)
                backHomeState = BACKHOME_RETREAT;
            break;
        }

        case BACKHOME_RETREAT: {
            static bool railRetreatDone = false;
            static bool handRetreatDone = false;

            // 地轨：从限位角向正方向后退 0.5mm（脱离物理接触）
            if (!railRetreatDone) {
                motorJ[0]->SetPositionWithMotorRps(
                    railLimitAngle + (0.5f / 5.0f),  // +0.1 圈，正方向
                    0.1f);
                railRetreatDone = true;
            }
            // 夹爪：从限位角（0）向张开方向后退到 3（脱离完全闭合）
            if (!handRetreatDone) {
                hand->SetAngleWithSpeedLimit(3);  // pos=3，张开方向
                handRetreatDone = true;
            }

            // 等待后退完成
            if ((!railRetreatDone || !motorJ[0]->IsMoving()) &&
                (!handRetreatDone || !hand->IsMoving())) {
                railRetreatDone = false;
                handRetreatDone = false;
                backHomeState = BACKHOME_APPROACH;
                // 进入 Phase4：精确定位（再次向限位方向慢速移动）
                motorJ[0]->SetVelocityWithMotorRps(-0.05f);   // 地轨：负方向慢速
                hand->SetAngleWithSpeedLimit(0.5);            // 夹爪：闭合方向慢速
            }
            break;
        }

        case BACKHOME_APPROACH: {
            // 等待再次堵转触发（精确定位）
            if (!railBackHomeDone2 && motorJ[0]->isStalled) {
                railHomeAngle = motorJ[0]->angle;  // 精确限位角（−250mm 处）
                railBackHomeDone2 = true;
                motorJ[0]->SetEnable(true);
            }
            if (!handBackHomeDone2 && hand->isStalled()) {
                handHomeAngle = hand->angle;  // 精确限位角（0 = 完全闭合）
                handBackHomeDone2 = true;
                hand->SetEnable(true);
            }
            if (railBackHomeDone2 && handBackHomeDone2) {
                backHomeState = BACKHOME_RETURN_ZERO;
                // 设置零点偏移（限位角 → 电机角 0）
                motorJ[0]->ApplyPositionAsHome();  // 限位角 → 电机角 0
                hand->ApplyPositionAsHome();        // 限位角 → 电机角 0
                // 设置主控侧软偏移：限位处 = −250mm
                railHomeOffsetMm = -250.0f;
                // 回目标位置（限位的对侧）
                motorJ[0]->SetPositionWithMotorRps(
                    (0.0f - railHomeOffsetMm) / 5.0f,  // 0mm = 50 圈
                    0.5f);
                hand->SetAngleWithSpeedLimit(100);     // pos=100 = 完全张开
            }
            break;
        }

        case BACKHOME_RETURN_ZERO: {
            if (!IsMoving() && !motorJ[0]->IsMoving() && !hand->IsMoving()) {
                backHomeState = BACKHOME_IDLE;
                // 通知协议层返回 ok
                backHomeComplete = true;
            }
            break;
        }
    }
}
```

---



## ASCII 协议层



### ascii_protocol.cpp 新增 handler

```cpp
// OnUsbAsciiCmd 和 OnUart4AsciiCmd 同步添加

else if (s == "!BACKHOME")
{
    if (dummy.IsStalled()) {
        Respond(_responseChannel, "error: motor stalled, send !START or !DISABLE first");
        return;
    }
    if (dummy.backHomeState != DummyRobot::BACKHOME_IDLE) {
        Respond(_responseChannel, "error: backhome already in progress");
        return;
    }
    dummy.BackHome();
    Respond(_responseChannel, "ok backhome started");
}
```



### 返回值说明


| 时机              | 返回                                                                             |
| --------------- | ------------------------------------------------------------------------------ |
| `!BACKHOME` 发送时 | `ok backhome started`（Phase1 启动即返回，不阻塞）                                        |
| Phase5 完成时      | `ok backhome done: rail=0mm, joints=RESET, hand=100`（在 UpdateBackHome 中通过串口发送） |


---



## 主循环集成

在 `main.cpp` 的 FreeRTOS 控制任务中（500ms 周期）：

```cpp
// 在控制任务中调用
dummy.UpdateBackHome();

// Phase5 完成后通过串口发送完成消息
if (dummy.backHomeComplete) {
    dummy.backHomeComplete = false;
    printf("ok backhome done: rail=0mm, joints=RESET, hand=100\r\n");
}
```

---



## 串口助手 UI 按钮

在「系统控制」区的 `sys_row1` 中（`!HOME` / `!RESET` 旁边）：

```python
# 在 sys_row1 的4个按钮之后添加「回位」按钮
tk.Button(sys_row1, text="回位", font=("Arial", 10), bg="#fa5252", fg="white",
          relief=tk.FLAT, pady=6,
          command=lambda: self.send_cmd("!BACKHOME")).pack(
              side=tk.LEFT, expand=True, fill=tk.BOTH, padx=1)
```

按钮样式：`bg="#fa5252"`（红色，与急停呼应，提示安全相关动作）。

---



## 风险与待确认事项



### R-1：电机固件堵转后自动回退是否触发

地轨电机固件 `motor_fw_f103_57` 中，堵转检测触发后自动进入 `RETREATING` → 回退 → `LOCKED`。主控侧需要检测 `LOCKED` 状态。

**待确认**：

- 电机固件 `motor_fw_f103_57` 中堵转检测是否已实现并启用？
- 电机固件回退完成后是否进入 `LOCKED` 状态并保持？



### R-2：夹爪固件的堵转检测

夹爪电机固件 `motor_fw_f103_gripper` 使用电流模式（`SetAngleWithCurrentLimit`），堵转检测在电流模式下是否工作？

**待确认**：

- `MODE_COMMAND_CURRENT` 模式下是否执行堵转检测？
- 如果电流模式不触发堵转，需要改用速度模式（`SetVelocityWithMotorRps`）寻限位。



### R-3：夹爪开度映射方向

**用户澄清（2026-09-02）**：

- **夹爪开度** `0` **= 完全闭合**（物理限位端）
- **夹爪开度** `100` **= 完全张开**（目标位置端）

调用 `SetAngleWithCurrentLimit(-1)` 是向"闭合方向"（`0`）运动，`+1` 是向"张开方向"（`100`）运动。

**待确认**：

- `motor_fw_f103_gripper` 固件中 `SetAngleWithCurrentLimit(_direction)` 的方向参数定义（`-1` 是闭合还是张开）
- `StepHand::SetAngleWithSpeedLimit(pos)` 中 `pos` 是否按 `0=闭合, 100=张开` 映射

> 若方向相反，Phase2 / Phase5 中需要把 `-1` ↔ `+1`、`100` ↔ `0` 互换。



### R-4：Phase3 后退距离

后退 `0.5mm` / `3` 步进单位是否为合理值（足够消除回退间隙但不过量）？**待实测调整**。

---



## 涉及的文件


| 文件                                                            | 修改内容                                                          |
| ------------------------------------------------------------- | ------------------------------------------------------------- |
| `firmware/ref_core_f405/Robot/instances/dummy_robot.h`        | 新增 `BackHomeState` 枚举、成员变量、`BackHome()`/`UpdateBackHome()` 声明 |
| `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp`      | 实现 `BackHome()` 5 阶段状态机                                       |
| `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp` | USB 和 UART4 各加 `!BACKHOME` handler                            |
| `firmware/ref_core_f405/UserApp/main.cpp`                     | 主循环调用 `UpdateBackHome()`                                      |
| `串口助手.py`                                                     | UI「回位」红色按钮                                                    |


---



## 实现顺序建议

1. **Phase1**（关节归位）— 最简单，验证框架
2. **Phase2**（寻限位）— 需要确认电机固件堵转检测已启用
3. **Phase5**（回零 + 返回 OK）— 收尾
4. **Phase3+Phase4**（后退 + 精确定位）— 精度优化

建议先实现 Phase1+2+5，跑通后再补 Phase3+4。