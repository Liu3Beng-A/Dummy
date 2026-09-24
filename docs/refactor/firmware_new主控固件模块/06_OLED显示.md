# 模块 06 — OLED 显示（角度行 + `_/*` 状态行 + 心跳探针去留）

> **范围**：`ThreadOledUpdate` 显示布局（6 轴角度 + 命令模式 + 状态横幅 + XYZ/ABC 坐标）
> **关联文件**：
> - `UserApp/main.cpp` (`ThreadOledUpdate` 函数，约 line 70-180)
> - `Robot/instances/dummy_robot.h` (`currentJoints` + `currentPose6D` + `jointsStateFlag` + `isEnabled`)

---

## 模块概述

OLED 显示 4 行关键信息（旧版）+ 1 行 IMU/FPS：

| 行号 | Y | 内容 | 显示源 |
|---|---|---|---|
| 第 1 行 | 10 | IMU + FPS | `mpu6050.data.ax/ay` + `micros()` |
| 第 2 行 | 15 | 分隔线（drawBox） | — |
| 第 3 行 | 30 | 6 轴当前角度 `>j1|j2|j3|j4|j5|j6` | `currentJoints.a[0..5]` 整数° |
| 第 4 行 | 45/56 | XYZ/ABC 世界坐标 | `currentPose6D.{X,Y,Z,A,B,C}` |
| 第 5 行 | 35-59 | 黑底矩形 + 文字标签 | — |
| 第 6 行 | 78 | `[CMD] __*_*__` 状态横幅 | `cmdModeNames[mode-1]` + `jointsStateFlag` |

**当前 firmware_new 状态**（重要）：删除 DIAG 心跳探针（F3 待实施）+ 恢复状态横幅。

---

## 三状态对比

| 维度 | OLD | firmware | 当前 firmware_new |
|---|---|---|---|
| 第 3 行（角度） | `>j1\|j2\|j3\|j4\|j5\|j6` 整数° | 7 轴 + 4 位小数（破坏布局） | **6 轴整数°**（保留 OLD 布局） |
| 第 4 行（CMD 模式） | `[SEQ] __*_*__` 6 字符 | `[SEQ] __*_*__` 扩 8 字符含地轨 | **心跳探针取代**（F3 待恢复） |
| 心跳探针（DIAG-2026-09-16） | 无 | 有（FLAG/RT/OL/RGB/B/L/I/O 4 行调试面板） | **存在**（F3 待删除） |
| XYZ/ABC 世界坐标 | OLD 第 4-5 行 | 同 OLD | 保留 |
| IMU 行（OLD 无） | 无 | 第 1 行 IMU+FPS | 保留 |

---

## 决策

| # | 决策 | 理由 | 风险 |
|---|---|---|---|
| 1 | **第 3 行保持 OLD 6 轴整数°显示** | 用户 P0 明确诉求：保留 6 轴角度 | 不扩展到 7 轴或小数 |
| 2 | **第 6 行（Y=78）恢复 `[CMD] __*_*__` 状态横幅** | 用户 P0 明确诉求：需要看到 `_`/`*` 状态切换判断运动情况 | 与 XYZ/ABC 第 4-5 行不重叠（drawBox 只到 Y=59） |
| 3 | **删除 DIAG 心跳探针（FLAG/RT/OL/RGB/B/L/I/O 4 行）** | 用户拍板：心跳探针全部删除退回 OLD 状态横幅布局 | 丢失调试信息（可改到 UART4 printf） |
| 4 | **`cmdModeNames` 保留 6 模式**（C3） | 配合模块 05 决策 | 详见模块 05 |
| 5 | **失能时显示 `[SEQ] ======`** | OLD 行为 | 无 |

---

## 实施

### F3 — `ThreadOledUpdate` 删除心跳探针 + 恢复状态横幅

**位置**：`UserApp/main.cpp:103-180`（当前 ThreadOledUpdate 函数体）

**当前代码**（DIAG 心跳探针，约 line 130-150）：
```cpp
oled.setFont(u8g2_font_5x8_tr);
oled.setCursor(0, 62);
oled.printf("FLAG:0x%02X/%02X", dummy.jointsStateFlag, 0x7E);
oled.setCursor(0, 70);
oled.printf("RT:%lu OL:%lu RGB:%lu", g_realtime_tick, g_oled_tick, g_rgb_tick);
oled.setCursor(0, 78);
oled.printf("B:%lu L:%lu I:%lu O:%lu", g_byte_in, g_line_ready, g_cmd_in, g_cmd_out);
// 旧的 cmdMode/joints 横幅已由心跳探针取代
```

**替换为**（OLD 状态横幅）：
```cpp
/* [F3] 第 6 行 (Y=78): [CMD] __*_*__ 状态横幅
 * - 使能时: [CMD] __*_*__ ，6 字符对应 J1~J6
 * - 失能时: [CMD] ======
 * - bit i = 1 → '*' (FINISH 到位)
 * - bit i = 0 → '_' (RUNNING/STOP)
 */
oled.setFont(u8g2_font_5x8_tr);
oled.setCursor(0, 78);
if (dummy.IsEnabled()) {
    char buf[8];
    for (int i = 1; i <= 6; i++)
        buf[i - 1] = (dummy.jointsStateFlag & (1 << i) ? '*' : '_');
    buf[6] = 0;
    oled.printf("[%s] %s",
                cmdModeNames[dummy.commandMode - 1],
                buf);
} else {
    oled.printf("[%s] %s",
                cmdModeNames[dummy.commandMode - 1],
                "======");
}
```

**注意**：
- `cmdModeNames` 已经是 6 元素（`{"SEQ","INT","TRJ","TUN","TRQ","SRV"}`）
- `dummy.commandMode - 1` 索引 0~5，对应 SEQ/INT/TRJ/TUN/TRQ/SRV
- `dummy.IsEnabled()` 是固有的方法（dummy_robot.h:313）

**禁止事项**：
- ❌ 不要把状态横幅扩到 8 字符（含地轨）—— 破坏布局
- ❌ 不要保留 DIAG 心跳探针的任何行（F3 决策删除）

---

## 验证

| 测试 | 期望 | 观察 |
|---|---|---|
| 第 3 行 6 轴角度 | `>  0| -73| 180|   0|   0|   0` 等整数° | OLED 观察 |
| 第 4-5 行 XYZ/ABC | 黑底白字 + 6 维坐标 | OLED 观察 |
| 第 6 行运行中显示 `[CMD] __*_*__` | 每轴 `_`/`*` 实时切换 | 发 `>` 后 OLED |
| 第 6 行全部到位 | `[SEQ] ******` | 运动完成后 |
| 第 6 行失能显示 | `[SEQ] ======` | 发 `!DISABLE` 后 |
| 6 模式名正确显示 | `[SEQ]/[INT]/[TRJ]/[TUN]/[TRQ]/[SRV]` | `#CMDMODE 1~6` 切换观察 |
| 心跳探针已删除 | OLED 看不到 FLAG/RT/B/L 等字样 | 视觉观察 |

---

## 状态

- [x] 决策已记录（2026-09-24）
- [ ] F3 代码已修改（main.cpp 删除 DIAG + 加状态横幅）
- [ ] 编译验证（ninja）
- [ ] 实测验证（OLED 视觉）

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [02 - IsMoving](firmware_new主控固件模块02_运动完成判断-IsMoving.md) | 第 6 行 `_`/`*` 来源于 jointsStateFlag |
| [05 - CommandMode](firmware_new主控固件模块05_命令模式-CommandMode.md) | cmdModeNames 是 OLED CMD 显示来源 |
| [03 - UpdateAngle](firmware_new主控固件模块03_关节角度同步-UpdateAngle.md) | 第 3 行 6 轴角度来自 currentJoints，由 0x23 回包更新 |
