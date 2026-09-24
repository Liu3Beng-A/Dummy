# 主控固件迁移决策（firmware → firmware_new）

> **范围**：仅主控固件（STM32F405，FreeRTOS）。
> **基线对比**：
>
> - **来源**：`firmware/ref_core_f405/` — 已实现的"新版本"
> - **目标**：`firmware_new/dummy-ref-core-fw/` — 当前待定
> - **基线**：`firmware_old/dummy-ref-core-fw/` — 旧版本（6 轴）
> **架构差异**：旧 6 轴架构 → 新 6 轴 + 地轨 + 夹爪架构 **创建日期**：2026-09-24

> **本文档是 17 个模块的索引**。每个模块的决策和实施细节请打开对应的模块文档。

---

## 0. 文档结构

本文档**完全废弃旧"按代码层级拆分"的结构**（§一 CAN / §二 ASCII / §三 类 / §四 实时环），改为按**功能模块**组织。每个模块是一份独立文档：

```
docs/refactor/
├── firmware_new主控固件决策.md           ← 本文（索引）
└── firmware_new主控固件模块
    └──01_运动指令-MoveJ&MoveL.md
       ~ 模块16_EEPROM配置持久化.md
```

---



## 1. 模块清单（17 个）


| #   | 模块                                        | 一句话决策                                               | 状态         | 文档                                                  |
| --- | ----------------------------------------- | --------------------------------------------------- | ---------- | --------------------------------------------------- |
| 01  | 运动指令 (`>` `&` `@`)                        | SEQ/TRJ `>` `&` 阻塞回 OK；`@` 不阻塞；7 关节 + speed 格式      | **P0 待实施** | [模块 01](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md)   |
| 02  | 运动完成判断 (IsMoving)                         | 回退 OLD 公式 `jointsStateFlag != 0b1111110`，删除 v3.2 短路 | **P0 待实施** | [模块 02](firmware_new主控固件模块02_运动完成判断-IsMoving.md)    |
| 03  | 关节角度同步 (UpdateAngle)                      | 保留单播轮询 14Hz/电机（决策 E2）；Enable 后预发 1 帧广播              | **已生效**    | [模块 03](firmware_new主控固件模块03_关节角度同步-UpdateAngle.md) |
| 04  | 急停 (EStop)                                | 0x09 广播急停 + 置 flag=ALL_FINISH_FLAG + 清 FIFO（已生效）    | **已生效**    | [模块 04](firmware_new主控固件模块04_急停-EStop.md)           |
| 05  | 命令模式 (CommandMode)                        | 6 模式 cmdModeNames 保留；SERVO_J/TORQUE 实际下发待拍板         | **部分生效**   | [模块 05](firmware_new主控固件模块05_命令模式-CommandMode.md)   |
| 06  | OLED 显示                                   | 删除 DIAG 心跳探针；恢复 `[CMD] __*_*__` 状态行                 | **P0 待实施** | [模块 06](firmware_new主控固件模块06_OLED显示.md)             |
| 07a | 灯效配置 (`!RGB_*`)                           | 保留全部 6 条命令 + SaveConfig 持久化                         | **已生效**    | [模块 07a](firmware_new主控固件模块07a_灯效配置.md)             |
| 07b | RGB 状态机                                   | 3 个灯效预设状态码保留；旧 `SetRGBEnable(bool)` 去留待拍板           | **部分生效**   | [模块 07b](firmware_new主控固件模块07b_RGB状态机.md)           |
| 08  | 夹爪控制 (`!HAND_*`)                          | 6 条命令保留；`StepHand` 继承 `CtrlStepMotor`               | **已生效**    | [模块 08](firmware_new主控固件模块08_夹爪控制.md)               |
| 09  | 电机参数配置 (`#ACC_J`/`#SPEED_J`/`#I_LIMIT_J`) | 单层 r/s² 单位；`_persist()` 可选写 EEPROM                  | **已生效**    | [模块 09](firmware_new主控固件模块09_电机参数配置.md)             |
| 10  | PID 查询 (`#GET_PID`/`#GETJACC`/`#GETI`)    | ISR 标志 + 主线程 poll + `PollPidPrint()` 合并打印           | **已生效**    | [模块 10](firmware_new主控固件模块10_PID查询.md)              |
| 11  | 堵转协议通路 (0x5A/0x5B/0x5C)                   | 主控侧全部删除；保留 `SetEnableStallProtect` (0x1B)           | **已删除**    | [模块 11](firmware_new主控固件模块11_堵转协议通路.md)             |
| 12  | 校准与归零 (`!CALIBRATION`/`#OFFSET_J`)        | 删除全轴 `!CALIBRATION`；保留单轴 `#OFFSET_J`                | **已生效**    | [模块 12](firmware_new主控固件模块12_校准与归零.md)              |
| 13  | 架构常量 (DH/减速比/REST_POSE)                   | 新机械结构 DH；统一减速比 50；REST_POSE J2=-75°                 | **已生效**    | [模块 13](firmware_new主控固件模块13_架构常量.md)               |
| 14  | 队列与 CommandHandler                        | 队列容量 32×128B；USB/UART4 默认 `#` 分支二选一（C10 ⏳）          | **部分生效**   | [模块 14](firmware_new主控固件模块14_队列与CommandHandler.md)  |
| 15  | 5kHz 实时环 (TIM7)                           | 保留框架 + 50Hz 节流下发；SERVO_J/TORQUE 实际启用待拍板             | **部分生效**   | [模块 15](firmware_new主控固件模块15_5kHz实时环.md)            |
| 16  | EEPROM 配置持久化                              | 灯效 SaveConfig + `_persist(bool)` 可选持久化              | **已生效**    | [模块 16](firmware_new主控固件模块16_EEPROM配置持久化.md)        |


---



## 2. 决策状态统计


| 状态             | 数量  | 模块                                      |
| -------------- | --- | --------------------------------------- |
| ✅ 已生效          | 9   | 03, 04, 07a, 08, 09, 10, 11, 12, 13, 16 |
| ⚠️ 部分生效（待拍板子项） | 4   | 05, 07b, 14, 15                         |
| ❌ P0 待实施       | 3   | 01, 02, 06                              |


---



## 3. 待拍板事项汇总


| #   | 模块    | 待拍板项                                             | 选项                                                |
| --- | ----- | ------------------------------------------------ | ------------------------------------------------- |
| 1   | 05/15 | F1.4 — `#CMDMODE 5/6` (TRQ/SRV) 实际下发行为           | A: 启用 / B: 回 error / C: 等同 SEQ                    |
| 2   | 07b   | P2-2 — 旧 `SetRGBEnable(bool)` 去留                 | A: 删旧 / B: 保留双套 / C: 折中（bool 控制启停，码决定样式）          |
| 3   | 14    | C10 — USB/UART4 默认 `#` 分支二选一                     | A: 都透传队列 / B: 都立即回 "ok" / C: 保留双套                 |
| 4   | 15    | B5 — `ComputeSyncSpeeds` 梯形同步规划用/不用              | A: 用 firmware 梯形 / B: 保留 OLD `dynamicJointSpeeds` |
| 5   | 15    | B6 — `currentPose6D` 5kHz 环内维护 vs ASCII 线程       | A: 5kHz 环 / B: ASCII 线程                           |
| 6   | —     | C8 — `#REBOOT` 无参版去留                             | 待讨论（影响小）                                          |
| 7   | —     | 0x21 主控侧 `GetCurrent(float)` 是否实现                | 待讨论（决策文档遗漏）                                       |
| 8   | 12    | 电机端 0x15 `ApplyPositionAsHome` 同步 `realPosition` | 等待电机端固件修复（见电机固件决策）                                |


---



## 4. 已完成的历史决策（不可改动）



### 4.1 Stall v3 删除（2026-09-24 完成）

**范围**：主控侧 stall v3 全部删除（7 个文件，共 -292 行）

- 删除 `isStalled` 字段 + `IsStalled()` 方法
- 删除 0x5A/0x5B/0x5C/0x7C 主控侧处理
- 删除 `!STALL_*` ASCII 命令
- 删除 `SetStallMode/BroadcastUnlock/QueryStallStatus` 方法
- 删除相关 OLED 探针和 ISR 标志
- 串口助手 stall UI 同步删除（-82 行）

**保留**：0x1B `SetEnableStallProtect`（电机端独立使用的命令）

详见：模块 [11 - 堵转协议通路](firmware_new主控固件模块11_堵转协议通路.md)

### 4.2 已生效的 v3.x 改进（来自 firmware/）


| 改进                                 | 模块     |
| ---------------------------------- | ------ |
| CAN 0x09 急停广播                      | 模块 04  |
| 7 轴架构（motorJ[8]）                   | 模块 13  |
| `StepHand` 继承 `CtrlStepMotor`      | 模块 08  |
| `SetPositionWithMotorRps` 重命名      | 模块 13  |
| `_persist(bool)` 可选持久化             | 模块 09  |
| 灯效 `SaveConfig()` 持久化              | 模块 16  |
| PID/Acc/Cur 批量查询 + ISR poll + 合并打印 | 模块 10  |
| 单播轮询 14Hz/电机                       | 模块 03  |
| 50Hz 节流下发（实时环）                     | 模块 15  |
| 队列容量 32×128B                       | 模块 14  |
| cmdModeNames 6 模式数组                | 模块 05  |
| 夹爪 6 条命令                           | 模块 08  |
| 灯效 6 条命令                           | 模块 07a |
| 单层 r/s² 单位                         | 模块 09  |
| `#OFFSET_J` 单轴归零                   | 模块 12  |
| 新机械结构 DH 参数                        | 模块 13  |
| 优化紧急停止 flag 短路                     | 模块 04  |


---



## 5. P0 待实施项（用户明确诉求）


| #    | 模块  | 待实施内容                                                                                        | 关联文档                                                       |
| ---- | --- | -------------------------------------------------------------------------------------------- | ---------------------------------------------------------- |
| F1.1 | 01  | SEQ `>` `&` 子块：删除 v2.8 非阻塞 `if (IsMoving) break`；加 `while (IsMoving) osDelay(5)` 阻塞 + 回 `ok` | [修复文档 F1.1](firmware_new主控固件修复—阻塞回OK与OLED同步—2026-09-24.md) |
| F1.2 | 01  | SEQ `@` 子块：删除 v2.8 break，保留立即回 `ok`                                                          | 同上                                                         |
| F1.3 | 01  | TRJ `>` `&` 子块：加阻塞循环                                                                         | 同上                                                         |
| F3   | 06  | `ThreadOledUpdate`：删除 DIAG 心跳探针 (FLAG/RT/OL/RGB/B/L/I/O)；恢复第 6 行 `[CMD] __*_*__` 状态行         | 同上                                                         |
| F5   | 02  | `IsMoving()` 函数体回退 OLD 公式 `return jointsStateFlag != 0b1111110;`（删除 v3.2 `!isEnabled` 短路）    | 同上                                                         |


详见 [修复文档](firmware_new主控固件修复—阻塞回OK与OLED同步—2026-09-24.md)。

---



## 6. 历史文档归档


| 旧文档                                             | 状态                          |
| ----------------------------------------------- | --------------------------- |
| `firmware_new主控固件决策.md`（旧版，按代码层级）               | **已废弃**，由本文档 + 17 个模块文档取代   |
| `firmware_new主控固件实施.md`（旧版）                     | **已废弃**，由新版索引 + 各模块"实施"章节取代 |
| `firmware_new主控固件修复—阻塞回OK与OLED同步—2026-09-24.md` | **保留**（P0 待实施的 5 个修复项）      |


---



## 7. 参考

- `[firmware_new电机固件决策.md](./firmware_new电机固件决策.md)` — 电机固件决策（独立）
- `[firmware_new电机固件实施.md](./firmware_new电机固件实施.md)` — 电机固件实施（独立）
- `[firmware_old/dummy-ref-core-fw/](../../firmware_old/dummy-ref-core-fw/)` — 旧固件基线（6 轴）
- `[firmware/ref_core_f405/](../../firmware/ref_core_f405/)` — 参考固件（7 轴 + 地轨 + 夹爪）

