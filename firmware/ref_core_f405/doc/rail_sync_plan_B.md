# 地轨速度同步方案 B：将地轨纳入 MoveJ 七轴同步体系

## 一、方案目标

将地轨（Rail/地轨，第7轴）接入 MoveJ 的**同步到达（Sync-to-Arrival）**逻辑，使地轨与6个臂关节在 MoveJ 运动时能够**同时到达目标位置**，而不是各自以独立速度运行。

当前 MoveJ 的同步机制仅作用于6个臂关节，地轨以固定速度独立运行，`IsMoving()` 也只监控臂关节，不等待地轨到位。方案B将解决这两个问题。

## 二、核心设计思想

### 2.1 瓶颈轴决定总时间

同步到达的核心逻辑是：**所有关节以各自不同的速度运行，但总时间相同**。

当前6臂关节的逻辑：
```
delta[i] = |target[i] - current[i]|  (deg)
maxDelta = max(delta[0..5])           (最远关节 = 瓶颈轴)
time     = maxDelta / jointSpeed       (s)
speed[i] = delta[i] / time             (deg/s)  — 距离近的走慢
```

方案B扩展为7轴：
```
delta[0] = |targetRail - currentRail|  (mm)    — 地轨单独计算
maxDelta = max(delta[0..5])             (deg)   — 仍以臂关节最远轴为基准
time     = maxDelta / jointSpeed        (s)

speed[i]   = delta[i] / time  (deg/s)   for i = 1..6
speedRail  = delta[0] / time  (mm/s)   — 地轨按臂关节同步时间缩放
```

### 2.2 地轨速度的双重约束

地轨实际速度取以下两者的**较小值**：

1. **用户设置的速度上限**：`railSpeed_mm_s`（通过 `#SPEED_RAIL` 设置，方案A已添加）
2. **同步到达所需速度**：`railDelta / timeSec`

如果同步所需速度小于用户设置的速度上限，说明臂关节运动时间充裕，地轨按用户速度运行，地轨成为瓶颈轴。

如果同步所需速度大于用户设置的速度上限，说明臂关节运动时间紧张，地轨按用户速度运行，臂关节成为瓶颈轴，地轨实际速度大于同步所需（地轨先到达但等待臂关节）。

### 2.3 到位检测扩展

当前 `IsMoving()` 只检查臂关节（bit 1~6），方案B将加入地轨到位（bit 0）：

```
原:  jointsStateFlag != 0b1111110   (bit0 忽略)
改:  (jointsStateFlag != 0b1111110) || (motorJ[0]->state != FINISH)
```

## 三、涉及的文件与改动

| 文件 | 改动类型 | 改动内容 |
|------|----------|----------|
| `Robot/instances/dummy_robot.cpp` | 修改 | `MoveJ()` 中加入地轨同步速度计算 |
| `Robot/instances/dummy_robot.cpp` | 修改 | `IsMoving()` 中加入地轨到位检测 |
| `Robot/instances/dummy_robot.h` | 修改 | `SetRailSpeed()` 声明（方案A已添加，无需改动） |
| `UserApp/main.cpp` | 无需改动 | 控制循环逻辑无需变化，`MoveRail(targetRailPos)` 已在50Hz循环中 |

## 四、详细修改说明

### 4.1 修改 `MoveJ()` — 加入地轨同步速度规划

**文件**: `Robot/instances/dummy_robot.cpp`
**位置**: `MoveJ()` 函数体，`targetJoints = targetJointsTmp;` 之前

**改动前的代码**（约 line 276-282）：

```cpp
    for (int j = 1; j <= 6; j++)
    {
        dynamicJointSpeeds.a[j - 1] = fabsf(deltaAngles.a[j - 1]) / timeSec;
        if (dynamicJointSpeeds.a[j - 1] < 0.05f)
            dynamicJointSpeeds.a[j - 1] = 0.05f;
    }

    targetJoints = targetJointsTmp;
    targetRailPos = _j7_mm;
    jointsStateFlag = 0;
```

**改动后**：

```cpp
    for (int j = 1; j <= 6; j++)
    {
        dynamicJointSpeeds.a[j - 1] = fabsf(deltaAngles.a[j - 1]) / timeSec;
        if (dynamicJointSpeeds.a[j - 1] < 0.05f)
            dynamicJointSpeeds.a[j - 1] = 0.05f;
    }

    // 地轨同步速度计算：按臂关节同步时间等比例分配地轨速度
    float railDelta = fabsf(_j7_mm - currentRailPos);
    if (railDelta > 0.1f)
    {
        float syncRailSpeed = railDelta / timeSec;
        if (syncRailSpeed < railSpeed_mm_s)
        {
            // 臂关节运动时间充裕，地轨速度由臂关节决定（地轨成为瓶颈轴）
            railSpeed_mm_s = syncRailSpeed;
        }
        // 否则使用用户设置的 railSpeed_mm_s（臂关节成为瓶颈轴）
        if (railSpeed_mm_s < 0.5f)
            railSpeed_mm_s = 0.5f;  // 地轨最小速度兜底
    }

    targetJoints = targetJointsTmp;
    targetRailPos = _j7_mm;
    jointsStateFlag = 0;
```

**改动说明**：
- `railDelta`: 地轨需要移动的距离（mm）
- `timeSec`: 臂关节同步运动的总时间（s），由 `maxAngle / jointSpeed` 计算
- `syncRailSpeed = railDelta / timeSec`: 要在臂关节同步时间内完成地轨运动所需的匀称速度
- 如果 `syncRailSpeed < railSpeed_mm_s`，说明臂关节"走得慢"，地轨应该更快但受臂关节时间约束，所以降低地轨速度以匹配臂关节
- 如果 `syncRailSpeed >= railSpeed_mm_s`，说明地轨"走得慢"，地轨按自己速度运行，臂关节按同步时间缩放速度
- `0.1f` 阈值：地轨移动距离小于0.1mm时忽略速度规划，避免除零或速度过小
- `0.5f` 最小速度：与臂关节保持一致，防止速度趋近于零

### 4.2 修改 `IsMoving()` — 加入地轨到位检测

**文件**: `Robot/instances/dummy_robot.cpp`
**位置**: `IsMoving()` 函数

**改动前的代码**（约 line 502-505）：

```cpp
bool DummyRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110;
}
```

**改动后**：

```cpp
bool DummyRobot::IsMoving()
{
    return (jointsStateFlag != 0b1111110) || (motorJ[0]->state != CtrlStepMotor::FINISH);
}
```

**改动说明**：
- `jointsStateFlag != 0b1111110`: 检查6个臂关节是否有未到位的（bit 1~6，任一为0则返回true）
- `motorJ[0]->state != CtrlStepMotor::FINISH`: 检查地轨电机状态是否为FINISH
- 用 `||` 连接：臂关节或地轨任一未到位，`IsMoving()` 就返回 true
- 这确保了 `ParseCommand()` 中的 `while (IsMoving())` 等待逻辑会同时等待臂关节和地轨到位后才返回 "ok"

### 4.3 确认 `main.cpp` 控制循环无需改动

**文件**: `UserApp/main.cpp`

当前控制循环中已经有：
```cpp
if (updateCounter % 100 == 0) {
    dummy.MoveJoints(dummy.targetJoints);
    dummy.MoveRail(dummy.targetRailPos);  // 地轨下发（使用更新后的 railSpeed_mm_s）
}
```

`MoveRail()` 已经在50Hz循环中被调用，且使用的是 `targetRailPos`（已在 `MoveJ()` 中设置）和当前 `railSpeed_mm_s`（已在 `MoveJ()` 中按同步逻辑调整）。因此控制循环**无需任何改动**。

### 4.4 确认 `MoveL()` 无需改动

**文件**: `Robot/instances/dummy_robot.cpp`

`MoveL()` 调用了 `MoveJ()`：
```cpp
return MoveJ(ikSolves.config[bestConfig].a[0],
             ikSolves.config[bestConfig].a[1],
             ...);
```

由于 `MoveJ()` 已经包含了地轨同步逻辑，`MoveL()` 无需任何改动，地轨同步逻辑会自动生效。

## 五、行为变化说明

### 5.1 场景分析

| 场景 | 臂关节时间 | 地轨距离 | syncRailSpeed vs railSpeed | 结果 |
|------|-----------|---------|--------------------------|------|
| 地轨长距离，臂关节短距离 | 短 | 200mm | syncRailSpeed >> railSpeed | 地轨按 railSpeed 运行，臂关节成为瓶颈，地轨先到等待 |
| 地轨短距离，臂关节长距离 | 长 | 10mm | syncRailSpeed << railSpeed | 地轨按 syncRailSpeed 运行（降速），地轨成为瓶颈，臂关节等 |
| 地轨短距离，臂关节短距离 | 短 | 5mm | 约等于 | 两者基本同步 |
| 地轨极短距离 | 任意 | <0.1mm | 跳过规划 | 使用 railSpeed_mm_s |

### 5.2 与方案A的配合

方案B建立在方案A的基础上：

- `railSpeed_mm_s` 作为**用户设置的地轨速度上限**
- `MoveJ()` 中动态调整 `railSpeed_mm_s`（可能降低，但不会增加）
- `#SPEED_RAIL <speed>` 命令仍然生效，用于设置地轨速度上限
- `SetRailSpeed()` 方法仍然可用

### 5.3 对 `ServoJ` 模式的影响

`ServoJ` 模式使用高频速度前馈，不走梯形规划。其 `MoveRail(targetRailPos)` 同样在SERVO_J分支的50Hz循环中被调用，使用当前 `railSpeed_mm_s`。如果需要ServoJ模式也支持同步，可以实施方案D，此处方案B聚焦于普通MoveJ/MoveL模式。

### 5.4 `Homing` 和 `Resting` 函数无需改动

`Homing()` 和 `Resting()` 调用 `MoveJ()`，会自然获得地轨同步逻辑。它们的 `while (IsMoving())` 等待也会自然包含地轨到位等待。

## 六、完整代码改动汇总

### 改动1: `dummy_robot.cpp` — `MoveJ()` 函数

在 `MoveJ()` 函数中，`targetJoints = targetJointsTmp;` 之前插入地轨同步速度计算块：

```cpp
// 查找位置：在以下代码之后：
//   for (int j = 1; j <= 6; j++)
//   {
//       dynamicJointSpeeds.a[j - 1] = ...
//   }
// 插入以下代码：

    // 地轨同步速度计算：按臂关节同步时间等比例分配地轨速度
    float railDelta = fabsf(_j7_mm - currentRailPos);
    if (railDelta > 0.1f)
    {
        float syncRailSpeed = railDelta / timeSec;
        if (syncRailSpeed < railSpeed_mm_s)
        {
            // 臂关节运动时间充裕，地轨速度由臂关节决定（地轨成为瓶颈轴）
            railSpeed_mm_s = syncRailSpeed;
        }
        // 否则使用用户设置的 railSpeed_mm_s（臂关节成为瓶颈轴）
        if (railSpeed_mm_s < 0.5f)
            railSpeed_mm_s = 0.5f;  // 地轨最小速度兜底
    }
```

### 改动2: `dummy_robot.cpp` — `IsMoving()` 函数

```cpp
// 查找：
bool DummyRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110;
}

// 替换为：
bool DummyRobot::IsMoving()
{
    return (jointsStateFlag != 0b1111110) || (motorJ[0]->state != CtrlStepMotor::FINISH);
}
```

## 七、测试验证清单

- [ ] `#SPEED_RAIL 10` 设置地轨速度（方案A）
- [ ] `#SPEED_RAIL` 查询地轨速度
- [ ] `>0,0,90,0,0,0,100,20` MoveJ 时地轨移动到100mm，验证臂关节和地轨同时到位
- [ ] `>0,0,90,0,0,0,10,20` MoveJ 时地轨移动到10mm，验证地轨降速配合臂关节
- [ ] `>0,0,90,0,0,0,0,100` 大速度MoveJ，验证地轨不超过100mm/s上限
- [ ] `!STOP` 急停后验证地轨和臂关节同时停止
- [ ] OLED显示地轨到位标记（可选：`j` 位标志显示可扩展）

## 八、已知限制

1. **地轨速度只降不升**: `MoveJ()` 只会将 `railSpeed_mm_s` 降低到满足同步所需的速度，不会超出用户设置的上限。这意味着如果用户先设置了很小的地轨速度，然后MoveJ到长距离，地轨会一直以小速度运行，不会"自动加速"。

2. **速度规划一次性**: 同步速度在 `MoveJ()` 被调用时计算一次，如果运动中途臂关节出现堵转/打滑，地轨不会自适应调整。

3. **不适用于ServoJ**: `ServoJ` 使用高频前馈模式，其地轨速度规划逻辑不同，需要单独实施方案D。

4. **地轨到位反馈依赖CAN反馈**: 地轨到位状态来自电机固件的 CAN 0x23 反馈包，如果CAN通信异常，`IsMoving()` 可能无法正确检测地轨到位。
