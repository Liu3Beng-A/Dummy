# 新增「回位」指令 !BACKHOME

> 状态：设计稿（2026-09-03 重写）
> 之前的状态机方案过复杂，按本稿的精简方案实施。

---

## 1. 指令

```
!BACKHOME
```

启动后立即回 `ok backhome started`，全部动作完成回 `ok backhome done`，异常时回 `error backhome failed: <reason>`。

---

## 2. 触发的三组动作

执行时同时启动以下三组动作，**全部完成**才返回 `ok backhome done`。

### 2.1 J1~J6 关节

- 直接内部调用一次 `DummyRobot::Resting()`（即等价于内部发一次 `!RESET`）
- 所有关节回到 `REST_POSE`

### 2.2 地轨（ID=9，地轨电机 57 步进）

1. 电机沿**正方向**移动，直到碰到物理限位
2. 到达限位后，记录当前位置为 **+250mm**（地轨正方向物理限位）
3. 反向移动 **250mm**，回到行程中间的 **0mm** 位置
4. 寻限位完成后调用 `ApplyPositionAsHome()`（CAN 0x15）把当前 home offset 写入电机 EEPROM

### 2.3 夹爪（ID=8，夹爪电机 35 步进）

1. 电机向**开度 0 方向**移动，直到碰到物理限位（完全闭合方向撞限位）
2. 到达限位后，记录当前位置为**开度 0**（完全闭合 = 主控端 pos0）
3. 回到开度 **100**（完全张开）
4. 寻限位完成后调用 `ApplyPositionAsHome()`（CAN 0x15）写入 EEPROM

### 2.4 时序关系

- **J1~J6 关节归位**：先于地轨+夹爪**串行**进行（保证机械臂不会在臂伸开时撞到夹爪/地轨）
- **地轨寻限位**与**夹爪寻限位**：可**并行**
- 三组动作全部完成 → 返回 `ok backhome done`

---

## 3. 寻限位的两段式精度策略

每条轴（地轨、夹爪）都按两段式寻限位：

### 第一段：粗寻
- 用**较大速度**向物理限位方向移动：地轨 **10 r/s**，夹爪 **2 r/s**
- 电机固件报告堵转（stalled）时认为到达限位
- 寻限位期间**禁用堵转保护**（不影响 `isStalled` 标志判断）

### 第二段：精寻
- 第一次到达限位后，**回退一段距离**：地轨 **5mm**（1 圈电机）；夹爪 **电机轴侧 3 圈**
- 切换到**慢速**再次向限位方向移动：地轨 **1 r/s**，夹爪 **1.5 r/s**
- 再次堵转时，记录当前位置为**最终限位角**

### 写入 EEPROM
- 精寻完成后调用 `CtrlStepMotor::ApplyPositionAsHome()` 发 CAN 0x15
- 电机固件把这个角度作为 home offset 写入 EEPROM
- 下次上电后电机以这个位置作为 home

---

## 4. 返回值

| 时机 | 返回 |
|---|---|
| 启动 | `ok backhome started` |
| 全部完成 | `ok backhome done: joints=REST_POSE, rail=0mm, hand=100` |
| 异常失败 | `error backhome failed: <reason>` |

---

## 5. 附带改动

### 5.1 57 电机（地轨）方向取反

当前 `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp` L138：

```cpp
motorJ[0] = new CtrlStepMotor(_hcan, 9, false, 1, -250, 250);
//                              ^^^^^
//                              第 3 参数 false = 不取反（当前行为）
```

需要改为：

```cpp
motorJ[0] = new CtrlStepMotor(_hcan, 9, true,  1, -250, 250);
//                              ^^^^
//                              第 3 参数 true = 方向取反（期望方向）
```

**说明**：当前 57 电机正方向与期望相反（用户实拍），正方向应改为负方向、负方向应改为正方向。改这一个参数即可统一全系统的地轨方向语义。

### 5.2 串口助手 UI

MoveJ 页 + MoveL 页各加一个红底白字「回位」按钮（与 MoveJ/MoveL 输入区同一行）：
- 按下后发送 `!BACKHOME`
- 显示同「START/DISABLE」按钮的颜色风格（按下变色）

### 5.3 旧指令变动

| 指令 | 状态 | 说明 |
|---|---|---|
| `!HOME` | 保留（回 HOME_POSE） | — |
| `!RESET` | 保留（回 REST_POSE） | — |
| `!HAND_ZERO` | 保留（手动标定夹爪零点） | — |
| `#OFFSET_J` | **删除**（旧版本遗留，主控侧关节 offset 不再需要） | 从 ascii_protocol.cpp 移除 handler |
| `#CMDMODE` | 保留 | 控制阻塞/非阻塞模式 |

`!BACKHOME` 是新增的**完整回位 + 自动标定**，不替代上述指令。

### 5.4 打断机制

`!BACKHOME` 执行期间可被以下指令**同步打断**（逻辑同堵转检测）：

| 打断指令 | 返回值 |
|---|---|
| `!DISABLE` | `error backhome failed: user disabled` |
| `!STOP` | `error backhome failed: user stopped` |
| `!START`（重发） | `error backhome failed: user started` |

打断后：
- 已触发的 `ApplyPositionAsHome()` 写入 EEPROM 的数据**保留**（不会回滚）
- 状态机重置为 `BH_IDLE`

---

## 6. 实现细节（代码参考）

### 6.1 状态机（极简 4 状态）

```cpp
enum BackHomeState {
    BH_IDLE,                  // 未启动
    BH_RESETTING_JOINTS,      // 阶段 A：J1~J6 → REST_POSE（同步阻塞）
    BH_SEEKING_LIMITS,        // 阶段 B+C：地轨 + 夹爪 寻限位（并行）
    BH_RETURNING_HOME,        // 阶段 D：地轨回 0mm + 夹爪回 100 开度
    BH_DONE                   // 全部完成
};
```

### 6.2 文件改动清单

| 文件 | 改动 |
|---|---|
| `firmware/ref_core_f405/Robot/instances/dummy_robot.h` | 加 `BackHomeState` 枚举 + `backHomeState` 字段 + `BackHome()` + `UpdateBackHome()` |
| `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp` | 实现 `BackHome()`（~60 行）+ `UpdateBackHome()`（~40 行）；L138 第 3 参数 `false → true` |
| `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp` | 加 `!BACKHOME` handler（~10 行） |
| `firmware/ref_core_f405/UserApp/main.cpp` | 控制循环里调 `dummy.UpdateBackHome()` 一行 |
| `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` | **修 P0-7**：`ApplyPositionAsHome()` 把 `canBuf` 初始化为 `{0}` 数组 |
| `串口助手.py` | MoveJ 页 + MoveL 页各加「回位」按钮 |

### 6.3 寻限位调用流程（伪代码）

```cpp
// 地轨寻限位（粗寻 + 精寻）
void SeekLimit_Rail() {
    // 粗寻：正方向 10 r/s，直到堵转
    motorJ[0]->SetPositionWithMotorRps(+1e6, 10);  // 大目标值
    while (!motorJ[0]->isStalled) osDelay(10);
    
    // 回退 5mm (1 圈)
    float retract_angle = motorJ[0]->angle - 720;  // 1圈 = 720°
    motorJ[0]->SetPositionWithMotorRps(retract_angle, 10);
    while (!motorJ[0]->AllAtTarget(2.0f)) osDelay(10);
    
    // 精寻：正方向 1 r/s，直到再次堵转
    motorJ[0]->SetPositionWithMotorRps(+1e6, 1);
    while (!motorJ[0]->isStalled) osDelay(10);
    
    // 记录当前位置为 +250mm
    railLimitPos = +250.0f;
    motorJ[0]->ApplyPositionAsHome();
    
    // 反向 250mm 回 0mm (速度 10 r/s)
    float retract_mm = motorJ[0]->angle - 18000;  // 250mm = 50圈 = 18000°
    motorJ[0]->SetPositionWithMotorRps(retract_mm, 10);
    while (!motorJ[0]->AllAtTarget(2.0f)) osDelay(10);
}

// 夹爪寻限位（粗寻 + 精寻）
void SeekLimit_Hand() {
    // 粗寻：开度 0 方向（电机负方向）2 r/s
    hand->SetAngleWithMotorRps(-1e6, 2);
    while (!hand->isStalled) osDelay(10);
    
    // 回退 3 圈（电机轴侧）
    hand->SetAngleWithMotorRps(hand->angle + 2160, 2);  // 3圈 = 2160°
    while (!hand->AllAtTarget(2.0f)) osDelay(10);
    
    // 精寻：开度 0 方向 1.5 r/s
    hand->SetAngleWithMotorRps(-1e6, 1.5f);
    while (!hand->isStalled) osDelay(10);
    
    // 记录当前位置为开度 0
    hand->ApplyPositionAsHome();
    
    // 回开度 100
    hand->SetAngleWithSpeedLimit(100);
    while (!hand->AllAtTarget(2.0f)) osDelay(10);
}
```

---

## 7. 用户使用流程

### 7.1 首次上电或机械搬运后

```
!DISABLE
!RAIL_L 500           ← 把地轨开到中间偏左（避免寻限位时撞到意外方向）
                      ← 把夹爪手动掰到行程中间（避免寻限位时反向撞限位）
!START
!BACKHOME              ← 自动寻限位 + 回中
                       ← ok backhome started
                       ← (J1~J6 → REST_POSE)
                       ← (地轨找 +250mm 限位 → 回 0mm)
                       ← (夹爪找 0 开度限位 → 回 100 开度)
                       ← ok backhome done
```

### 7.2 日常使用

```
>0,0,90,0,0,0,0,30         ← MoveJ 到 HOME_POSE
!HAND_POS 50               ← 夹爪半开
>30,-45,90,...             ← MoveJ 到工作位
```

---

## 8. 安全注意事项

- 寻限位前必须 `!START`（电机使能），且确认无物理障碍
- 寻限位过程电机以堵转电流运行，注意散热（57 电机堵转电流较大）
- 寻限位前应确认目标方向没有意外卡死物体
- 一旦 `!BACKHOME` 启动后中途打断（`!DISABLE` / `!STOP` / 重发 `!START`），已写入 EEPROM 的 home 仍生效，下次上电仍按该 home 起算
- 寻限位期间**禁用堵转保护**（`isStalled` 仍正常判断，只是不触发急停/告警），防止误触发影响寻限位流程

## 9. 文件改动清单

| 文件 | 改动 |
|---|---|
| `firmware/ref_core_f405/Robot/instances/dummy_robot.h` | 加 `BackHomeState` 枚举 + `backHomeState` 字段 + `BackHome()` + `UpdateBackHome()` |
| `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp` | 实现 `BackHome()`（~60 行）+ `UpdateBackHome()`（~40 行）；L138 第 3 参数 `false → true` |
| `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp` | 加 `!BACKHOME` handler（~15 行）；**删除** `#OFFSET_J` handler |
| `firmware/ref_core_f405/UserApp/main.cpp` | 控制循环里调 `dummy.UpdateBackHome()` 一行 |
| `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp` | **修 P0-7**：`ApplyPositionAsHome()` 把 `canBuf` 初始化为 `{0}` 数组 |
| `串口助手.py` | MoveJ 页 + MoveL 页各加「回位」按钮 |
