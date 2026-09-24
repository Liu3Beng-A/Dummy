# 模块 14 — 队列与 `CommandHandler`（指令队列容量与处理架构）

> **范围**：ASCII 指令的入队 + 消费 + 处理架构
> **关联文件**：
> - `UserApp/protocols/cmd_protocol.cpp` (`CommandHandler` 实现)
> - `UserApp/protocols/ascii_protocol.cpp`（push 端）
> - `Robot/instances/dummy_robot.cpp`（消费 + ParseCommand）

---

## 模块概述

ASCII 命令流：

```
USB/UART4 接收中断
    ↓
ascii_protocol.cpp 中解析 (按行尾 '\n' 切分)
    ↓
cmd_protocol.cpp: cmdHandler.Push(_cmd)  ← 入队 (FIFO)
    ↓
ControlLoopUpdateTask 中 CommandHandler.ParseCommand()  ← 消费
    ↓
dummy_robot.cpp 中对应分支处理（运动指令 / 配置指令）
```

队列容量决定可缓冲多少未消费指令。

---

## 三状态对比

| 维度 | OLD | firmware | 当前 firmware_new |
|---|---|---|---|
| 队列容量 | 16 × 64B | 32 × 128B | **32 × 128B**（决策 §0.3 E7 ✅） |
| 字节容量 | 1024B | 4096B | 4096B（翻 4 倍） |
| 线程架构 | 1 个消费者线程 | 1 个消费者线程 | 同 |
| 默认 `#` 分支 | 回 `"ok"` | USB 转发队列；UART4 回 `"ok"` | 见模块 [15 - 决策 §0.3 C10 ⏳](../firmware_new主控固件决策.md) |
| 队列满处理 | 覆盖 | 阻塞（默认） | 待确认 |
| CAN RX 中断优先级 | N/A | 高于 UART4 | 同 |

---

## 决策

| # | 决策 | 理由 | 风险 |
|---|---|---|---|
| 1 | **队列容量 32 × 128B**（决策 §0.3 E7 ✅） | 容量翻 4 倍，缓冲更多突发指令 | 无 |
| 2 | **C10 USB/UART4 默认 `#` 分支：二选一统一**（决策 §0.3 C10 ⏳） | 当前 USB 透传到队列，UART4 回 `"ok"`，不一致 | 待拍板 |
| 3 | **保留 CommandHandler 单一消费者线程** | 与所有现有架构一致 | 无 |

---

## 实施

### 实施项：当前代码已生效（容量），仅待 C10 决策

**关键代码位置**（`cmd_protocol.cpp`）：

- `CommandHandler` 类（含 Push / ParseCommand）
- 默认 `#` 分支：USB 行为（透传队列） vs UART4 行为（回 "ok"）
- 队列大小定义（32 × 128 已生效）

### 待用户拍板：C10 二选一

**选项 A：统一为 USB 行为（透传队列）**
- 所有 `#` 命令（包括未知命令）都进队列，由消费线程处理
- 优势：扩展性好（未来加 `#XX` 命令无需改 USB 解析）
- 风险：未知 `#XX` 会被无限推迟

**选项 B：统一为 UART4 行为（立即回 "ok"）**
- 所有 `#` 命令收到后立即回 ok，不入队
- 优势：响应快
- 风险：未知 `#XX` 命令会被错误地认为成功

**选项 C：保留双套（当前行为）**
- USB 透传；UART4 立即回
- 优势：各通道按各自惯例
- 风险：行为不一致难以调试

**禁止事项**：
- ❌ 不要缩队列到 16 × 64B（决策 §0.3 E7 ✅ 保留新）
- ❌ 不要在 C10 决策前修改两边的默认行为

---

## 验证

| 测试 | 期望 | 方法 |
|---|---|---|
| 队列满缓冲 | 连发 32 条指令不被丢 | 压测 |
| `#CMDMODE 1` | 切换 SEQ 模式 | `#CMDMODE 1` |
| `#CMDMODE 6` | 切换 SERVO_J 模式 | `#CMDMODE 6` |
| USB 默认 `#` | 见 C10 选项 | 发未知 `#XX` |
| UART4 默认 `#` | 见 C10 选项 | 发未知 `#XX` |

---

## 状态

- [x] 决策已记录（2026-09-24）
- [x] 队列容量 32 × 128B 已生效（决策 §0.3 E7 ✅）
- [ ] C10 用户拍板

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [05 - CommandMode](firmware_new主控固件模块05_命令模式-CommandMode.md) | `#CMDMODE` 走 CommandHandler → SetCommandMode |
| [01 - MoveJ](firmware_new主控固件模块01_运动指令-MoveJ&MoveL.md) | `>` `&` `@` 走 ParseCommand |
| [11 - 堵转协议通路](firmware_new主控固件模块11_堵转协议通路.md) | `!STALL_*` 已删除（无队列相关内容） |
