# 问题 A：四形式并存与旧固件正常的根本原因

## 一、关键代码差异定位

我在 `source code/`（旧）与 `firmware/`（新）之间对四个核心模块做了对比。差异最大的三处：

| 差异点 | 旧固件（form A 正常） | 新固件（form B/C/D 都异常） | 影响 |
|---|---|---|---|
| 电机 `0x01` 使能响应 | 切到 `MODE_COMMAND_VELOCITY`，velocity=0 → state=`STATE_FINISH` | `requestMode` 不变，保持 `MODE_STOP` → state=`STATE_STOP` | 致命 |
| 电机 `0x07` 立即回包 | 收到位置命令后立刻回 `0x23` + `data[4]=state==FINISH?1:0` | 不再回 `0x23`，强制依赖主控 100Hz 轮询拿真值 | 致命 |
| 电机 `0x01` 字段语义 | data 是 float（maxCurrent） | data 必须是 `uint32_t == 1` 才算 enable | 隐式陷阱 |

具体证据（`firmware/motor_fw_f103_35/UserApp/protocols/interface_can.cpp:32-90` vs `source code/dummy-35motor-fw/UserApp/protocols/interface_can.cpp:32-38`）：

```cpp
// 旧电机 0x01：硬切模式 → 立即可推 FINISH
case 0x01:
    motor.controller->requestMode = (*(uint32_t*) (RxData) == 1) ?
                                    Motor::MODE_COMMAND_VELOCITY : Motor::MODE_STOP;
    break;
```

```cpp
// 新电机 0x01：不改 requestMode → 仍在 MODE_STOP
case 0x01:
    if (*(uint32_t*) (RxData) == 1) {
        motor.controller->stallMode = Motor::STALL_IDLE;
        motor.controller->ResetGoalsToCurrentPosition();
        // 注意：不改 requestMode，保持原模式等待新命令
    }
```

```cpp
// 旧电机 0x07：立即回 FINISH ACK（哪怕 state 是 stale）
_data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
CAN_Send(&txHeader, _data);
```

```cpp
// 新电机 0x07：注释明确写"应答时 controller->state 是 stale 值，主控会误判到位"
break;
```

而主控侧 `firmware/ref_core_f405/Robot/actuators/ctrl_step/ctrl_step.cpp:235-241`：

```cpp
void CtrlStepMotor::UpdateAngleCallback(float _pos, bool _isFinished)
{
    state = _isFinished ? FINISH : RUNNING;   // ← 关键：只有 _isFinished=1 才置 FINISH
    ...
}
```

`jointsStateFlag` 的写入规则（`dummy_robot.cpp:432-447`）：

```cpp
if (motorJ[i]->state == CtrlStepMotor::FINISH)
    jointsStateFlag |= (1 << i);
else
    jointsStateFlag &= ~(1 << i);   // ← RUNNING/STOP/STALL 都清位
```

这就构成了"新电机 + 新主控"的死结：电机 `state == STATE_STOP`（不是 FINISH），主控 `UpdateAngleCallback` 把 `motorJ[i]->state` 写成 RUNNING，`jointsStateFlag` 永远到不了 `0x7E`，`IsMoving()` 永远 true。

## 二、四个形式各自的死结路径

### 形式 A：全旧固件 ✅ 正常

主控 old + 电机 old。

`source code/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp` 的 `UpdateJointAngles()` 是单帧 ALL 广播：

```cpp
void DummyRobot::UpdateJointAngles()
{
    motorJ[ALL]->UpdateAngle();   // 1 帧 StdId=0x23，6 电机 + 1 手同时回
}
```

而旧电机 `interface_can.cpp` 在 `0x01` 把 mode 切成 VELOCITY(v=0)，state=STATE_FINISH，于是：

1. `!START` → 主控发 1 帧 ALL 0x01
2. 旧电机进入 `MODE_COMMAND_VELOCITY`，goalVelocity=0（Init 默认）
3. `motorJ[ALL]->UpdateAngle()` → 6 个电机回 0x23，全部 `data[4]=1`（FINISH）
4. 主控 `jointsStateFlag` 在一帧之内就 `|= (1<<1)|...|(1<<6) = 0x7E`
5. `IsMoving()` 立即 false，`!HOME` 的 `while(IsMoving())` 立刻退出
6. 旧 `CanSendMessage` 是 `osWaitForever`，永远不会"静默丢帧"

这是唯一一组状态机语义自洽的搭配。

### 形式 B：旧主控 + 新电机 ❌ 立即卡死

致命三连：

1. **电机不切模式**：旧主控发 `motorJ[ALL]->SetEnable(true)`，新电机 0x01 不切 `requestMode`，全部 6 电机仍停在 `MODE_STOP`，state=STATE_STOP。
2. **旧主控的 ALL 轮询一去不回**：旧 `UpdateJointAngles` 用 `motorJ[ALL]->UpdateAngle()` 一次性查 7 电机，但新电机 0x07 的响应已被"作废"（不立即回 FINISH ACK），需要靠单播轮询 0x23 才能拿到真 state。
3. **旧主控的实时环 5kHz 全量下发**：`source code/dummy-ref-core-fw/UserApp/main.cpp`：

    ```cpp
    case COMMAND_TARGET_POINT_SEQUENTIAL:
    case COMMAND_TARGET_POINT_INTERRUPTABLE:
    case COMMAND_CONTINUES_TRAJECTORY:
        dummy.MoveJoints(dummy.targetJoints);   // 5kHz 全量下发 6 × 0x07
        dummy.UpdateJointPose6D();
        break;
    ```

    没有 50Hz 节流，每 200μs 发 6 帧 0x07。新电机 0x07 又要进 POSITION 模式 → 触发 100ms 启动豁免 → 触发 v3 堵转检测 → 7 个电机同时广播 0x5A → CAN 总线彻底拥塞。

**后果**：新电机进入 LOCKED/RETREATING 后，旧主控轮询的 state 永远是 RUNNING 或 STALL，`jointsStateFlag` 永远是 0。`!START` 之后 `isEnabled=true`，没有命令在等待 IsMoving，但任何 `!HOME` / `!RESET` / `>x,x,x...` 都会进 `while(IsMoving()) osDelay(10)` 死循环。

而且旧 `CanSendMessage` 是 `osWaitForever`，CAN 总线拥塞后直接死锁整个 ASCII 线程（重看 `9-16-23-30问题分析.md` §10.1 关于 `HAL_CAN_AddTxMessage` 返回 `HAL_ERROR` 时信号量永久泄漏的分析）。

### 形式 C：新主控 + 旧电机 ❌ !HOME 卡死

新主控 + 旧电机（不带地轨）。

旧电机被切到 `MODE_COMMAND_VELOCITY(v=0)`、state=FINISH——这点对。但 `!HOME` 的链路里有两个特定于新主控的破坏点：

1. **`motorJ[7]` 错位**：新主控把地轨当成 `motorJ[7]`（nodeID=7，`dummy_robot.cpp:84`），但旧硬件根本没有 rail，hand 是 id=7（`source code/dummy-ref-core-fw/Robot/instances/dummy_robot.h:11`）。所以：
   - `dummy.Homing()` 里的 `motorJ[7]->SetPositionWithMotorRps(0, railSpeedRps)` 实际上是给旧夹爪发 0x07（StdId=(7<<7)|0x07=0x387），夹爪被切到 POSITION 模式去追一个根本不属于它的目标位置；
   - `dummy.UpdateJointAngles()` 轮询 `motorJ[7]` 时收的是夹爪的 0x23，新主控把它当成 rail 角度写到 `currentRailPos`，`dummy_robot.cpp:441-447` 还顺手把这次响应拿去跑 `UpdateJointAnglesCallback()`，里面检查的是 `motorJ[1..6]` 的 state——夹爪的状态搅进来，但 `jointsStateFlag` 不变；
   - 旧夹爪收到 0x07 会真的物理驱动（即使没到位也会触发堵转），开始广播 0x5A——可新主控的 `can_protocol.cpp` 没有针对 id=7 的 0x5A 屏蔽（`firmware/ref_core_f405/UserApp/protocols/can_protocol.cpp:135-149` 的 case 0x5A 里有 `dummy.SetStallMode(stallNodeId)`，把 `isStalled=true` 一次性挂到所有电机）。

2. **`SetStallMode` 触发的连锁**：

    ```cpp
    void DummyRobot::SetStallMode(int motorIndex) {
        SetRGBMode(RGB::RED_HEARTBEAT);
        isStalled = true;
        targetJoints = currentJoints;
        ...
    }
    ```

    一旦夹爪广播 0x5A，新主控立即 `isStalled=true`，RGB 切红心跳。后续 ASCII 命令里虽然 `ParseCommand` 入口有"堵转期间运动指令丢"的精确拦截，但**控制指令（`!DISABLE` / `!START`）放行**，不解决根本死结。

3. **`jointsStateFlag=0` 复位**：
   新主控 `MoveJ()` 里 `dummy_robot.cpp:269` 末尾有 `jointsStateFlag = 0;`（A3 修复保留）。`!HOME` 调 `MoveJ(0,0,90,0,0,0,0,10)` 把 flag 全清零。然后 `MoveJoints` 把 6 个 0x07 发出去。旧电机立即回 0x23（**state 字段是 stale 的 VELOCITY-FINISH**，因为 0x07 是同步处理、`modeRunning` 还没切换到 POSITION 的那一拍），主控把 6 位都置 1。

   之后 50ms 内（5kHz 实时环的第一个 50Hz 槽），实时环发第二轮 6 × 0x07。旧电机这一次跑完了 20kHz 周期，进入了 `MODE_COMMAND_POSITION`，state 切到 `STATE_RUNNING`（因为 `softPosition != goalPosition`）。**主控 `UpdateJointAngles` 100Hz 轮询（每个电机 14Hz）陆续拿到 RUNNING，把 flag 各位清掉**。

   旧电机到位后 state 回到 `FINISH`，主控轮询再把位置设上。但这里有个**致命细节**：

4. **`#GET_PID` / `#GETJACC` 一类查询命令的"假响应循环"**：`can_protocol.cpp` 收到 0x2C / 0x2D 时也会跑 `UpdateJointAnglesCallback()`，相当于每个查询命令都触发一次 flag 维护。如果旧电机在等 0x23 回包时**主控先收到了 0x07 的 stale 回包**，而 `motorJ[i]->state` 一旦是 RUNNING，flag 就被清——而此时电机其实**还没开始转**（20kHz 还没来得及切到 POSITION），等到电机真的到了 FINISH，下一次轮询才会把它推回 flag。

   这些时序上的错位导致 `IsMoving()` 在 `!HOME` 期间可能**永远卡在 true**。

### 形式 D：全新固件 ❌ !START 后 RGB 立即卡死

**新主控 + 新电机，地轨 id=7、夹爪 id=8，与新主控的电机表完全对得上。**

但依然有问题，证据就在 `9-16-23-30问题分析.md` 用户实测的 `B34 L4 I4 O3`：

| 计数 | 值 | 含义 |
|---|---|---|
| B | 34 | ASCII 字节接收正常 |
| L | 4 | 4 条命令完整收到 |
| I | 4 | 4 条命令进入 handler |
| O | 3 | **3 条完成，第 4 条卡在 handler 里** |
| RT | 在涨 | 5kHz 实时环活着 |
| OL | 在涨 | OLED 任务活着 |
| RGB | **卡死** | RGB 任务被饿死 |

新主控 `dummy_robot.cpp:611-633` 的 `SetEnable(true)`：

```cpp
motorJ[ALL]->SetEnable(_enable);   // 1 帧 StdId=(0<<7)|0x01
hand->SetEnable(_enable);          // 1 帧 StdId=(8<<7)|0x01
isEnabled = _enable;
if (_enable) {
    motorJ[ALL]->UpdateAngle();    // 1 帧 StdId=(0<<7)|0x23，触发 7 电机回包
}
```

`!START` 立刻返回 "ok" 没问题。但之后：

1. `isEnabled=true`，5kHz 实时环开始 MoveJoints（50Hz，7 帧）和 UpdateJointAngles（100Hz，1 帧），每秒 350+100=450 帧。
2. 新电机 0x01 不切模式——电机仍 MODE_STOP，state=STATE_STOP。主控 `UpdateAngleCallback` 拿到 `_isFinished=false`，`motorJ[i]->state = RUNNING`，`jointsStateFlag` 各位一直是 0。
3. 实时环的 `MoveJoints` 发 6 × 0x07 给 J1~J6，新电机 0x07 不立即回 FINISH ACK（v3 改的）。新电机进入 POSITION 模式后，触发 v3 堵转检测的 100ms 启动豁免。
4. 如果机械臂当前位置已经在 home 附近（这是用户描述"上电后只发 !START 就卡死"的常见场景），新电机的 goal 和 est 几乎重合，PID 输出很小。但 `modeRunning == POSITION + softPosition == goalPosition` 才算 FINISH——只要差一步，state 就 RUNNING。

   新电机 `firmware/motor_fw_f103_35/Ctrl/Motor/motor.cpp:225-241` 的状态更新里：

    ```cpp
    if (controller->modeRunning == MODE_COMMAND_POSITION) {
        if ((controller->softPosition == controller->goalPosition)
            && (controller->softVelocity == 0))
            controller->state = STATE_FINISH;
        else controller->state = STATE_RUNNING;
    }
    ```

    整数比较 `softPosition == goalPosition`，考虑 motion planner 的累加误差，可能长时间不等于 0，state 长期是 RUNNING。

   主控侧 `dummy_robot.cpp:432-447`：

    ```cpp
    if (motorJ[i]->state == CtrlStepMotor::FINISH)
        jointsStateFlag |= (1 << i);
    else
        jointsStateFlag &= ~(1 << i);
    ```

    RUNNING 把 flag 清掉，flag 永远到不了 0x7E。

   同时，新电机 0x07 之后实时环不停下发，每次都让电机重新 CalcSoftGoal。新电机的 20kHz 闭环里 softPosition 永远差 1 step，state 永远 RUNNING。

**结果**：

- `!START` 后主控看着 6 个电机全是 RUNNING → `jointsStateFlag = 0`
- `isStalled = false`（除非夹爪 0x5A 抢先触发）
- 用户发 `!HOME` → MoveJ 把 flag 清零 → MoveJoints 发 6 帧 → `while(IsMoving())` 永不退出
- RGB 任务饿死：因为 `Respond()` 在 USB / UART4 上的 TX 路径被 RGB 子任务抢，详见 `9-16-23-30问题分析.md` §1.2 未覆盖点列表
- OLED 显示 4 边界 + 3 探针 → B34 L4 I4 O3

这就是为什么 form D `!START` 后立即卡死——而且 **`!HOME` 也卡死**（双重 bug）。

## 三、为什么只有全部回退旧固件才正常

把四个形式用一张表对照"卡死根因"：

| 形式 | 主控轮询策略 | 电机 0x01 行为 | 电机 0x07 行为 | 总线负载 | 状态机闭环 |
|---|---|---|---|---|---|
| A 全旧 | ALL 单帧全量 | 切 VELOCITY → FINISH | 立即回 FINISH ACK | 30k fps（旧实时环） | ✅ flag 一帧到位 |
| B 旧主 + 新电机 | ALL 单帧全量 | 不切模式 → STOP | 不回 ACK | 30k fps | ❌ flag 永远 0 |
| C 新主 + 旧电机 | 单播轮询 14Hz/电机 | 切 VELOCITY → FINISH | 立即回 stale ACK | 450 fps | ⚠️ 旧夹爪抢 0x7 + 0x5A 触发 isStalled |
| D 全新 | 单播轮询 14Hz/电机 | 不切模式 → STOP | 不回 ACK | 450 fps | ❌ flag 永远 0 |

旧固件这一组两个关键不变量都成立：

1. 旧电机 0x01 把 mode 切到 VELOCITY(v=0)，state=FINISH——主控侧 `UpdateAngleCallback` 一收到回包就能立刻把 `jointsStateFlag` 填满；
2. 旧电机 0x07 立即同步回 0x23 + state 字段——主控不需要异步等下一次轮询。

这两个保证让 `jointsStateFlag` 在一帧之内就能达到 `0x7E`。其他任何形式都至少破坏其中一个不变量。

## 四、四形式差异的根本机制（一句话）

| 形式 | 关键错位 | 表现 |
|---|---|---|
| A | 无错位 | 正常 |
| B | 电机状态语义错位（不切模式 → 主控 flag 永远 0）+ 总线 30k fps 撞死 | `!START` 后整个 ASCII 线程在 `CanSendMessage` 里 `osWaitForever` 死锁 |
| C | 电机 ID 错位（旧手 id=7 被新主控当 rail）+ 0x5A 假堵转触发 isStalled | `!START` ok，但 `!HOME` 进 `while(IsMoving())` 死循环 |
| D | 同 B 的状态语义错位，但走的是新主控的 10ms 超时而非死等 | `!START` 后 RGB 立即卡（4 边界 I > O），`!HOME` 也卡 |

## 五、建议下一步

`9-16-23-30问题分析.md` 已经把所有"已尝试修复"列出来（F1-F4 + R10），并明确承认：

> 结论：CAN TX 信号量泄漏（9-14 文档最大嫌疑）已修，问题依旧 → 真正的卡点不在 CAN TX 层。

4 边界心跳方案只完成了"埋点"（代码里能看到 `g_realtime_tick` / `g_oled_tick` / `g_rgb_tick` / `g_byte_in` / `g_line_ready` / `g_cmd_in` / `g_cmd_out`），但用户实测的 `B34 L4 I4 O3` 没贴出 RT/OL/RGB 的具体值。如果按现在的实测记录推理，最可能的真正根因是上面 [二、四个形式各自的死结路径](#二四个形式各自的死结路径) 中形式 D 的描述：新电机 0x01 / 0x07 的状态机语义与新主控的 `jointsStateFlag` 机制不自洽。

要不要我接着帮你把这个问题做一个最小修复方案？例如：要么把新电机 0x01 改成"使能时也切到 `MODE_COMMAND_POSITION`"，要么把新主控的 `jointsStateFlag` 改成"看到 STOP/NO_CALIB 时也算 FINISH"。这两个方向影响的代码量都很小（10 行内），可以二选一先实验。
