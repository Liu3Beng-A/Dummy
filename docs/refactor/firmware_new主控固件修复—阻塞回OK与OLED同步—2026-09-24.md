# 主控固件修复方案 — 阻塞回 OK + OLED 同步刷新

> **范围**：仅主控固件 `firmware_new/dummy-ref-core-fw/`。
> **基线对齐**：`firmware_new主控固件决策.md` + `firmware_new主控固件实施.md`。
> **创建日期**：2026-09-24
> **状态**：⏳ 待执行（仅文档化，未实施）
>
> ---
>
> ## 0. 任务背景
>
> 用户 P0 明确诉求（2026-09-24）：
>
> 1. **修复"阻塞返回 OK"的逻辑** —— 当前 `firmware_new` 主控固件的 `ParseCommand` 在 `COMMAND_TARGET_POINT_SEQUENTIAL` 与 `COMMAND_CONTINUES_TRAJECTORY` 模式下，**没有阻塞等待电机到位再回 `ok`**，而是被 v2.8 改动成了"电机还在动就丢弃新指令回 ok"，破坏了 OLD 固件的"上一指令到位后才执行下一条 + 全部到位才回 ok"语义。
> 2. **修复"OLED 屏幕同步刷关节角度和运动状态"的功能** —— 当前 `ThreadOledUpdate` 把 OLD 的第 4 行 `[CMD] __*_*__` 状态横幅**完全删除了**，替换成了 [DIAG-2026-09-16] 心跳探针调试面板（FLAG/RT/OL/RGB/B/L/I/O 共 4 行）。这导致 OLED 上**看不到每轴运动状态切换**，用户反馈的运动完成情况无法直观判断。
>
> 本文档把修复内容完整文档化（已通过 AskQuestion 让用户拍板），**只待用户确认后才动代码**。
>
> ---
>
> ## 1. 用户已拍板的决策（2026-09-24 AskQuestion）
>
> | # | 决策项 | 选项 | 含义 |
> |---|---|---|---|
> | 1 | OLED 心跳探针 | **"心跳探针全部删除退回，回到旧版本的那种"** | 删除 FLAG/RT/OL/RGB/B/L/I/O 4 行调试面板，让出空间给状态横幅，回到 OLD 简洁布局 |
> | 2 | `>` `&` 命令参数格式 | **B3：严格 7 关节 + speed** | 保持当前 `>j1,j2,j3,j4,j5,j6,j7,speed` 格式（含地轨 j7），不引入 6 关节兼容 |
> | 3 | `cmdModeNames` 数组 | **C3：保留 6 模式** | 保持 `{"SEQ","INT","TRJ","TUN","TRQ","SRV"}` 6 元素不变（不缩到 4） |
> | 4 | `IsMoving()` 公式 | **D1：严格 OLD 公式** | 用 `jointsStateFlag != 0b1111110` 字面量，不引入 v3.2 的 `!isEnabled ? false` 短路 |
> | 5 | `UpdateAngle()` 触发 | **E2：保留 14Hz/电机单播轮询** | 不改回 1 帧广播，保持 v3.2 单播轮询（虽然响应比广播慢，但 1 帧 7 节点同时回包可能会撞 mailbox） |
>
> ---
>
> ## 2. 修复项清单
>
> ### F1 — `dummy_robot.cpp` 的 `ParseCommand`：SEQ/TRJ 阻塞回 OK
>
> **优先级**：P0（用户明确诉求）
>
> **位置**：`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp`
>
> **OLD 行为（参考 `firmware_old/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:347-358`）**：
>
> ```cpp
> case COMMAND_TARGET_POINT_SEQUENTIAL:
> case COMMAND_CONTINUES_TRAJECTORY:
>     if (_cmd[0] == '>' || _cmd[0] == '&') {
>         /* 解析 6 关节 + speed */
>         context->MoveJ(...);
>         context->MoveJoints(context->targetJoints);           // 1. 立即下发
>
>         while (context->IsMoving() && context->IsEnabled())   // 2. 阻塞等电机到位
>             osDelay(5);
>         Respond(*usbStreamOutputPtr, "ok");                  // 3. 双通道回 ok
>         Respond(*uart4StreamOutputPtr, "ok");
>     }
> ```
>
> **修复 1.1 — SEQ `>` `&` 子块**（删除 v2.8 "丢指令 + break"）：
>
> 当前代码（约 `dummy_robot.cpp:555-572`）：
>
> ```cpp
> if (_cmd[0] == '>' || _cmd[0] == '&') {
>     /* 解析 7 关节 + speed（含 j7 地轨） */
>     if (context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed)) {
>         // [v2.8 修复] ←──── 这段要替换
>         if (context->IsMoving() && context->IsEnabled()) {
>             Respond(*usbStreamOutputPtr,  "ok");
>             Respond(*uart4StreamOutputPtr, "ok");
>             break;
>         }
>         context->MoveJoints(context->targetJoints);
>         context->motorJ[7]->SetPositionWithMotorRps(
>             context->targetRailPos / 5.0f, context->railSpeedRps);
>
>         Respond(*usbStreamOutputPtr,  "ok");
>         Respond(*uart4StreamOutputPtr, "ok");
>     }
> }
> ```
>
> **替换为**：
>
> ```cpp
> if (_cmd[0] == '>' || _cmd[0] == '&') {
>     /* 解析 7 关节 + speed（含 j7 地轨） */
>     if (context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed)) {
>         // [修复 F1] 恢复 OLD 阻塞回 OK 语义（用户 P0 核心诉求）：
>         //   1) 立即下发一次 MoveJoints，防止 IsMoving() 因 jointsStateFlag
>         //      残留旧位而提前 false；
>         //   2) 阻塞等待所有臂关节 FINISH（bit1-6 全置位 = 0b1111110）；
>         //   3) USB+UART4 双通道同时回 ok。
>         //   注：地轨位 bit0 恒为 0，IsMoving() 不会因地轨未到位而无限阻塞。
>         context->MoveJoints(context->targetJoints);
>         context->motorJ[7]->SetPositionWithMotorRps(
>             context->targetRailPos / 5.0f, context->railSpeedRps);
>
>         while (context->IsMoving() && context->IsEnabled())
>             osDelay(5);
>
>         Respond(*usbStreamOutputPtr,  "ok");
>         Respond(*uart4StreamOutputPtr, "ok");
>     }
> }
> ```
>
> **修复 1.2 — SEQ `@` 子块**（删除 v2.8 "丢指令 + break"，保持 OLD 不阻塞回 ok）：
>
> 当前代码（约 `dummy_robot.cpp:583-602`）：
>
> ```cpp
> else if (_cmd[0] == '@') {
>     /* 解析 6 维坐标 + speed */
>     if (context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])) {
>         // [v2.8 修复] ←──── 这段要替换
>         if (context->IsMoving() && context->IsEnabled()) {
>             Respond(*usbStreamOutputPtr,  "ok");
>             Respond(*uart4StreamOutputPtr, "ok");
>             break;
>         }
>         context->MoveJoints(context->targetJoints);
>         context->motorJ[7]->SetPositionWithMotorRps(
>             context->targetRailPos / 5.0f, context->railSpeedRps);
>
>         Respond(*usbStreamOutputPtr,  "ok"); Respond(*uart4StreamOutputPtr, "ok");
>     }
> }
> ```
>
> **替换为**：
>
> ```cpp
> else if (_cmd[0] == '@') {
>     /* 解析 6 维坐标 + speed */
>     if (context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])) {
>         // [修复 F1] MoveL 保持不阻塞 (OLD 行为)，
>         //   移除 v2.8 "丢指令 break" 逻辑。
>         context->MoveJoints(context->targetJoints);
>         context->motorJ[7]->SetPositionWithMotorRps(
>             context->targetRailPos / 5.0f, context->railSpeedRps);
>
>         Respond(*usbStreamOutputPtr,  "ok");
>         Respond(*uart4StreamOutputPtr, "ok");
>     }
> }
> ```
>
> **修复 1.3 — TRJ `>` `&` 子块**（添加阻塞等待）：
>
> 当前代码（约 `dummy_robot.cpp:622-642`）：
>
> ```cpp
> case COMMAND_CONTINUES_TRAJECTORY:
>     if (_cmd[0] == '>' || _cmd[0] == '&') {
>         /* 解析 7 关节 + speed */
>         if (context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed)) {
>             context->MoveJoints(context->targetJoints);     // ←── 现在就下发，没阻塞
>             context->motorJ[7]->SetPositionWithMotorRps(
>                 context->targetRailPos / 5.0f, context->railSpeedRps);
>
>             Respond(*usbStreamOutputPtr,  "ok");
>             Respond(*uart4StreamOutputPtr, "ok");
>         }
>     }
> ```
>
> **替换为**：
>
> ```cpp
> case COMMAND_CONTINUES_TRAJECTORY:
>     if (_cmd[0] == '>' || _cmd[0] == '&') {
>         /* 解析 7 关节 + speed */
>         if (context->MoveJ(j1, j2, j3, j4, j5, j6, j7, speed)) {
>             // [修复 F1] TRJ 与 SEQ 共用 OLD 阻塞回 OK 语义
>             context->MoveJoints(context->targetJoints);
>             context->motorJ[7]->SetPositionWithMotorRps(
>                 context->targetRailPos / 5.0f, context->railSpeedRps);
>
>             while (context->IsMoving() && context->IsEnabled())
>                 osDelay(5);
>
>             Respond(*usbStreamOutputPtr,  "ok");
>             Respond(*uart4StreamOutputPtr, "ok");
>         }
>     }
> ```
>
> ---
>
> ### F5 — `dummy_robot.h` 的 `IsMoving()`：回归 OLD 公式
>
> **优先级**：P0
>
> **位置**：`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.h`
>
> **OLD 行为**：
>
> ```cpp
> bool DummyRobot::IsMoving() {
>     return jointsStateFlag != 0b1111110;
> }
> ```
>
> **当前 NEW 行为**：
>
> ```cpp
> bool DummyRobot::IsMoving() {
>     if (!isEnabled) return false;          // [v3.2] 失能短路
>     return jointsStateFlag != ALL_FINISH_FLAG;  // 0x7E
> }
> ```
>
> **替换为**（仅方法体，函数签名不变）：
>
> ```cpp
> bool DummyRobot::IsMoving() {
>     // [F5] 严格 OLD 公式（P0 决策 D1）：不做短路，字面量与原代码 1:1
>     return jointsStateFlag != 0b1111110;
> }
> ```
>
> **注**：`ALL_FINISH_FLAG` 常量可保留在头文件中备用，但 IsMoving() 不再引用它。
>
> ---
>
> ### F3 — `main.cpp` 的 `ThreadOledUpdate`：恢复状态横幅 + 删除心跳探针
>
> **优先级**：P0
>
> **位置**：`firmware_new/dummy-ref-core-fw/UserApp/main.cpp`
>
> **当前代码**（约 `main.cpp:103-127`）：
>
> ```cpp
> for (;;) {
>     g_oled_tick++;
>     uint32_t currentTime = HAL_GetTick();
>
>     /* IMU 100Hz 限流 */
>     if (currentTime - lastImuUpdate >= 10) {
>         mpu6050.Update(true);
>         lastImuUpdate = currentTime;
>     }
>
>     oled.clearBuffer();
>
>     /* 第 1 行: IMU + FPS */
>     oled.setFont(u8g2_font_5x8_tr);
>     oled.setCursor(0, 10);
>     oled.printf("IMU:%.3f/%.3f", mpu6050.data.ax, mpu6050.data.ay);
>
>     uint32_t nowMicros = micros();
>     uint32_t elapsed   = nowMicros - lastMicros;
>     lastMicros = nowMicros;
>     uint32_t fps = (elapsed > 0) ? (1000000u / elapsed) : 0u;
>
>     oled.setCursor(85, 10);
>     oled.printf("| FPS:%lu", fps);
>
>     oled.drawBox(0, 15, 128, 3);
>
>     /* 第 3 行: 6 轴角度 */
>     oled.setCursor(0, 30);
>     oled.printf(">%3d|%3d|%3d|%3d|%3d|%3d",
>                 (int)roundf(dummy.currentJoints.a[0]),
>                 (int)roundf(dummy.currentJoints.a[1]),
>                 (int)roundf(dummy.currentJoints.a[2]),
>                 (int)roundf(dummy.currentJoints.a[3]),
>                 (int)roundf(dummy.currentJoints.a[4]),
>                 (int)roundf(dummy.currentJoints.a[5]));
>
>     /* 第 4-5 行: XYZ/ABC 世界坐标（保留） */
>     oled.drawBox(40, 35, 128, 24);
>     oled.setFont(u8g2_font_6x12_tr);
>     oled.setDrawColor(0);
>     oled.setCursor(42, 45);
>     oled.printf("%4d|%4d|%4d", (int)roundf(dummy.currentPose6D.X),
>                                     (int)roundf(dummy.currentPose6D.Y),
>                                     (int)roundf(dummy.currentPose6D.Z));
>     oled.setCursor(42, 56);
>     oled.printf("%4d|%4d|%4d", (int)roundf(dummy.currentPose6D.A),
>                                     (int)roundf(dummy.currentPose6D.B),
>                                     (int)roundf(dummy.currentPose6D.C));
>     oled.setDrawColor(1);
>     oled.setCursor(0, 45);  oled.printf("[XYZ]:");
>     oled.setCursor(0, 56);  oled.printf("[ABC]:");
>
>     /* ❌ 心跳探针 (DIAG) - 要删除 */
>     oled.setFont(u8g2_font_5x8_tr);
>     oled.setCursor(0, 62);
>     oled.printf("FLAG:0x%02X/%02X", dummy.jointsStateFlag, 0x7E);
>     oled.setCursor(0, 70);
>     oled.printf("RT:%lu OL:%lu RGB:%lu",
>                 g_realtime_tick, g_oled_tick, g_rgb_tick);
>     oled.setCursor(0, 78);
>     oled.printf("B:%lu L:%lu I:%lu O:%lu",
>                 g_byte_in, g_line_ready, g_cmd_in, g_cmd_out);
>     // 旧的 cmdMode/joints 横幅已由心跳探针取代
>
>     oled.sendBuffer();
>     PollPidPrint();
> }
> ```
>
> **替换为**（新增第 6 行 `[CMD] __*_*__` 状态横幅，删除心跳探针）：
>
> ```cpp
> for (;;) {
>     g_oled_tick++;
>     uint32_t currentTime = HAL_GetTick();
>
>     /* IMU 100Hz 限流 */
>     if (currentTime - lastImuUpdate >= 10) {
>         mpu6050.Update(true);
>         lastImuUpdate = currentTime;
>     }
>
>     oled.clearBuffer();
>
>     /* 第 1 行: IMU + FPS */
>     oled.setFont(u8g2_font_5x8_tr);
>     oled.setCursor(0, 10);
>     oled.printf("IMU:%.3f/%.3f", mpu6050.data.ax, mpu6050.data.ay);
>
>     uint32_t nowMicros = micros();
>     uint32_t elapsed   = nowMicros - lastMicros;
>     lastMicros = nowMicros;
>     uint32_t fps = (elapsed > 0) ? (1000000u / elapsed) : 0u;
>
>     oled.setCursor(85, 10);
>     oled.printf("| FPS:%lu", fps);
>
>     oled.drawBox(0, 15, 128, 3);
>
>     /* 第 3 行: 6 轴角度（保留） */
>     oled.setCursor(0, 30);
>     oled.printf(">%3d|%3d|%3d|%3d|%3d|%3d",
>                 (int)roundf(dummy.currentJoints.a[0]),
>                 (int)roundf(dummy.currentJoints.a[1]),
>                 (int)roundf(dummy.currentJoints.a[2]),
>                 (int)roundf(dummy.currentJoints.a[3]),
>                 (int)roundf(dummy.currentJoints.a[4]),
>                 (int)roundf(dummy.currentJoints.a[5]));
>
>     /* 第 4-5 行: XYZ/ABC 世界坐标（保留） */
>     oled.drawBox(40, 35, 128, 24);
>     oled.setFont(u8g2_font_6x12_tr);
>     oled.setDrawColor(0);
>     oled.setCursor(42, 45);
>     oled.printf("%4d|%4d|%4d", (int)roundf(dummy.currentPose6D.X),
>                                     (int)roundf(dummy.currentPose6D.Y),
>                                     (int)roundf(dummy.currentPose6D.Z));
>     oled.setCursor(42, 56);
>     oled.printf("%4d|%4d|%4d", (int)roundf(dummy.currentPose6D.A),
>                                     (int)roundf(dummy.currentPose6D.B),
>                                     (int)roundf(dummy.currentPose6D.C));
>     oled.setDrawColor(1);
>     oled.setCursor(0, 45);  oled.printf("[XYZ]:");
>     oled.setCursor(0, 56);  oled.printf("[ABC]:");
>
>     /* [F3 修复] 第 6 行 (Y=78): [CMD] __*_*__ 状态横幅 (恢复 OLD 语义)
>      * - 使能时: 显示 [SEQ] __*_*__ ，6 字符对应 J1~J6
>      *           bit i = 1 → '*' (FINISH 到位)
>      *           bit i = 0 → '_' (RUNNING/STOP)
>      * - 失能时: 显示 [SEQ] ======
>      * 字号用 5x8_tr 与第 1 行同，Y=78 + ~8px ≈ 86px ≤ OLED 80-128 高
>      */
>     oled.setFont(u8g2_font_5x8_tr);
>     oled.setCursor(0, 78);
>     if (dummy.IsEnabled()) {
>         char buf[8];
>         for (int i = 1; i <= 6; i++)
>             buf[i - 1] = (dummy.jointsStateFlag & (1 << i) ? '*' : '_');
>         buf[6] = 0;
>         // cmdModeNames[6] 保留 6 模式 (SEQ/INT/TRJ/TUN/TRQ/SRV)
>         // mode [1..6] → idx [0..5] 永远不会越界
>         oled.printf("[%s] %s",
>                     cmdModeNames[dummy.commandMode - 1],
>                     buf);
>     } else {
>         oled.printf("[%s] %s",
>                     cmdModeNames[dummy.commandMode - 1],
>                     "======");
>     }
>
>     oled.sendBuffer();
>     PollPidPrint();
> }
> ```
>
> **注意**：
> - `cmdModeNames` 声明保持当前 `char cmdModeNames[6][4] = {"SEQ", "INT", "TRJ", "TUN", "TRQ", "SRV"};`（决策 C3 不改）
> - `buf` 声明如果在循环外，可保持现有 `char buf[16];`；如果改为内联声明，建议保持内联避免冲突
>
> ---
>
> ### F1/F5/F3 改完后的最终代码摘要
>
> | 文件 | 改动定位 | 影响行数 |
> |---|---|---|
> | `Robot/instances/dummy_robot.cpp` | SEQ `>` `&` 子块 | 删除 7 行 + 加 4 行（净 +4） |
> | `Robot/instances/dummy_robot.cpp` | SEQ `@` 子块 | 删除 7 行 + 加 5 行（净 -2） |
> | `Robot/instances/dummy_robot.cpp` | TRJ `>` `&` 子块 | 加 4 行（净 +4） |
> | `Robot/instances/dummy_robot.h` | `IsMoving()` 函数体 | 减 2 行 |
> | `UserApp/main.cpp` | `ThreadOledUpdate` 末尾 | 删除 8 行 + 加 17 行（净 +9） |
> | `UserApp/main.cpp` | 顶部 `g_*` 调试计数器（如不再用） | 可保留也可删除（建议保留以备调试） |
>
> ---
>
> ## 3. 验证步骤
>
> 按用户 P0 决策完成后，应做以下验证：
>
> ### V1 — 阻塞回 OK 行为验证
>
> 1. **SEQ 模式阻塞**：`#CMDMODE 1` 后发 `>0,-73,180,0,0,0`（7 参数），USB+UART4 应在 6 电机全部到位（OLED 显示 6 个 `*`）后**同时**回 `ok`，阻塞时长通常 0.5~3 秒（取决于距离与速度）
> 2. **TRJ 模式阻塞**：`#CMDMODE 3` 后发 `>0,-73,180,0,0,0`，同样阻塞回 ok
> 3. **INT 模式非阻塞**：`#CMDMODE 2` 后发 `>0,-73,180,0,0,0`，**立即**回 `ok`（不等电机）
> 4. **`@` 不阻塞**：在任意模式下发 `@0.3,0,0.2,0,0,0,30`，**立即**回 `ok`（OLD 行为）
>
> ### V2 — OLED 状态横幅验证
>
> 1. **运行中显示下划线**：发 `>` 指令，OLED 第 6 行（Y=78）应显示 `[SEQ] __*__*_` 状态切换：未到位轴显示 `_`，到位轴显示 `*`
> 2. **全部到位显示全 `*`**：6 轴全部到位后 OLED 第 6 行应显示 `[SEQ] ******`
> 3. **失能显示等号**：发 `!DISABLE` 后 OLED 第 6 行应显示 `[SEQ] ======`
> 4. **命令模式名切换**：
>    - `#CMDMODE 1` → OLED 显示 `[SEQ] ......`
>    - `#CMDMODE 2` → OLED 显示 `[INT] ......`
>    - `#CMDMODE 3` → OLED 显示 `[TRJ] ......`
>    - `#CMDMODE 4` → OLED 显示 `[TUN] ......`
>    - `#CMDMODE 5` → OLED 显示 `[TRQ] ......`（cmdModeNames[5-1]=cmdModeNames[4]="TRQ"）
>    - `#CMDMODE 6` → OLED 显示 `[SRV] ......`（cmdModeNames[6-1]=cmdModeNames[5]="SRV"）
>
> ### V3 — 编译验证
>
> ```bash
> cd E:\Dummy-code\firmware_new\dummy-ref-core-fw\build
> ninja
> ```
>
> 应输出 201/201 ninja targets 编译成功，输出 `Core-STM32F4-fw.bin / .hex / .elf`。
>
> ---
>
> ## 4. 风险评估
>
> | 风险点 | 等级 | 说明 |
> |---|---|---|
> | ParseCommand 阻塞会延迟 ASCII 队列消费 | 低 | OLD 行为本来就是阻塞的；FIFO 队列 32×128，容量充足；用户 P0 明确要求阻塞 |
> | `while(IsMoving) osDelay(5)` 阻塞期间有新指令入队 | 极低 | FIFO 是独立队列，新指令入队后等当前指令处理完才被消费；OLD 也是同样的行为 |
> | OLED 第 6 行与现有 drawBox 在 Y=78 重叠 | 中 | 现有 drawBox(40, 35, 128, 24) 矩形只到 Y=59，与 Y=78 不重叠 |
> | OLED Y=78 用 5x8 字号超过屏高 (128x64?) | 中 | OLD 用 10x20 字号在 Y=78，依赖屏幕实际尺寸 ≥128x80；用 5x8 字号更保守（Y=78+8=86 ≤128） |
> | 删除 `!isEnabled` 短路导致 !DISABLE 后队列卡死 | 极低 | ParseCommand 用 `while(IsMoving && IsEnabled)`，IsEnabled=false 直接跳出循环，不会卡死 |
> | heart-beat 探针删除后丢失调试信息 | 中 | 调试时可改回 printf 到 UART4；本任务优先恢复 P0 OLED 状态显示 |
>
> ---
>
> ## 5. 实施 TODO（待用户确认后执行）
>
> ```text
> ☐ F1.1  改 dummy_robot.cpp SEQ `>` `&` 子块（删除 v2.8 丢指令，加阻塞循环）
> ☐ F1.2  改 dummy_robot.cpp SEQ `@` 子块（删除 v2.8 丢指令 break）
> ☐ F1.3  改 dummy_robot.cpp TRJ `>` `&` 子块（加阻塞循环）
> ☐ F5   改 dummy_robot.h IsMoving() 函数体（回归 0b1111110 字面量）
> ☐ F3   改 main.cpp ThreadOledUpdate（删除心跳探针，加 [CMD] __*_*__ 状态行）
> ☐ V1   验证阻塞回 OK 行为
> ☐ V2   验证 OLED 状态横幅
> ☐ V3   编译验证 ninja
> ```
>
> ---
>
> ## 6. 不在本次修复范围内的事项
>
> - ❌ 不修改 `firmware/ref_core_f405/`（仍为 v3.2 主控参考固件）
> - ❌ 不修改 `firmware_old/dummy-ref-core-fw/`（OLD 主控基线，仅做参考对照）
> - ❌ 不引入 7 轴 + 地轨决策调整（F2 保持 B3）
> - ❌ 不修改 cmdModeNames 数组（F4 保持 C3）
> - ❌ 不改回 1 帧广播（F6 保持 E2）
> - ❌ 不动 stall v3 相关（2026-09-24 已完成 stall 删除）
> - ❌ 不修改其他 ASCII/CAN 命令（仅 ParseCommand 与 OLED 显示相关）
>
> ---
>
> ## 7. 参考文档
>
> - [`firmware_new主控固件决策.md`](./firmware_new主控固件决策.md) — 主控固件决策源头
> - [`firmware_new主控固件实施.md`](./firmware_new主控固件实施.md) — 主控固件实施文档
> - `firmware_old/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp` — OLD ParseCommand 实现
> - `firmware_old/dummy-ref-core-fw/UserApp/main.cpp` — OLD OLED 显示实现
