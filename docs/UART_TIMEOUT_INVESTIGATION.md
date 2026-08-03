# 串口卡死问题调查

## 背景

最近给 STM32 主控固件加了一组新功能：

1. 串口调整 PID 参数（`#PID_*` 类命令）
2. RGB 亮度调整（`!RGB_BRIGHT`）+ 缓慢关灯（亮度渐变而非瞬时为 0）
3. 修复主控直接返回 `arrive ok` 的问题（即修复 `!HOME` / MoveJ 等命令不再立即回 `ok` 而是等真正到位再回）

修完 #3 后，开始暴露新现象。

## 当前两个问题

### 问题 1：RGB / OLED 一起卡死

**触发**：点串口助手 UI 上的
- 「关灯」（`!RGB_BRIGHT 0`）
- 「查询」（`!RGB_BRIGHT` 不带参数）
- 「应用」（`!RGB_BRIGHT N`）
- 「保存」（`!RGB_BRIGHT N` + `SaveConfig`）
- 「修改 RGB 模式」（`!RGB_MODE N`）

**结果**：
- 终端仍能收到 `ok rgb ...`（UART 通道正常）
- 但 RGB 灯效**卡住不刷新**
- OLED 屏幕**卡住不刷新**

**不触发**：直接发 `!HOME` / `!RESET` / MoveJ，机械臂能正常动作（说明控制环没死）。

### 问题 2：`#HOMEOFFSET` 后整体卡死

**触发**：发 `#HOMEOFFSET` 之后

**结果**：
- 终端**也会卡死**（Python 端 Write timeout）
- 后续任何命令都收不到响应

## 已尝试但失败的方法

- 把 `WS2812_Send` 里的 `while (!data_sentflag){}` spin-wait 改成 `osDelay(1)` + 100ms 超时 → **问题 1 未解决**
- 给 CAN TX semaphore 加 abort 时 release → 改变了问题 1 现象但**未解决**（且 PR-1.1 是基于错误推测的盲改，已撤回）

## 当前状态

- 已回滚到 `b471728 backup: 卡死问题排查前的快照`（工作树干净）
- 未做新的盲改