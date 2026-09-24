# 模块 10 — PID 查询（`#GET_PID`/`#GETJACC`/`#GETI` + ISR poll + 合并打印）

> **范围**：批量查询电机运行时参数（PID + 加速度 + 电流限幅）+ 合并打印避免 UART TX 竞争
> **关联文件**：
> - `UserApp/protocols/ascii_protocol.cpp` (`#GET_PID`/`#GETJACC`/`#GETI`/`#SYNC_ACC`)
> - `UserApp/protocols/can_protocol.cpp` (`_pid_*` 变量 + `PollPidPrint()` + `case 0x28~0x2D` 回包处理)
> - `Robot/instances/dummy_robot.cpp` (`SyncAllMotorAcceleration()`)
> - `Robot/actuators/ctrl_step/ctrl_step.cpp` (`QueryDceKp/Kv/Ki/Kd/Acceleration/CurrentLimit`)

---

## 模块概述

PID 查询用 **ISR 标志 + 主线程轮询 + 4 回合包合并打印** 模式：

```
#GET_PID
↓
发 0x28/0x29/0x2A/0x2B 到指定电机
↓
ISR 收到回包，累加 _pid_rcv_cnt[idx]，4 次到齐后设 _pid_print_pending[idx]=1
↓
主线程 PollPidPrint() 检测 print_pending，读缓存 printf 一次合并输出
```

避免：4 个独立回包引发 4 次 UART TX 阻塞；ISR 内 printf 阻塞 CAN RX。

---

## 三状态对比

| 维度 | OLD | firmware | 当前 firmware_new |
|---|---|---|---|
| 查询命令 | 无 | `#GET_PID`/`#GETJACC`/`#GETI` | **已生效** |
| CAN 查询命令 | 无 | 0x28/0x29/0x2A/0x2B/0x2C/0x2D | **已生效** |
| ISR 标志模式 | 无 | `_pid_*` + `_pid_print_pending` | **已生效** |
| 合并打印机制 | 无 | `PollPidPrint()` | **已生效** |
| `#GET_PID` 命令 | 无 | 一次性查 4 参数 | 保留 |
| `#GETJACC` 超时保护 | 无 | 30ms 轮询时间戳 | 保留 |
| UART TX 竞争修复 | 无 | v3.0 修复（ISR 只设标志） | 保留 |
| 缓存字段 | 无 | `jointAccRuntime[]`/`jointCurrentLimitRuntime[]` + 时间戳 | 保留 |

---

## 决策

| # | 决策 | 理由 | 风险 |
|---|---|---|---|
| 1 | **保留全部 6 个查询命令**（0x28~0x2D + ASCII `#GET_PID`/`#GETJACC`/`#GETI`） | v3.0 UART TX 竞争修复必须保留 | 无 |
| 2 | **保留 ISR 标志 + 主线程 poll 模式** | ISR 内 printf 会阻塞 CAN RX，不能在 ISR 做 | 无 |
| 3 | **`PollPidPrint()` 合并打印机制保留** | 4 回合包合并为 1 行 printf，避免 4 行散乱输出 | 无 |
| 4 | **30ms 轮询时间戳超时保护保留** | 防止某电机永远不回包导致查询卡死 | 无 |

---

## 实施

### 实施项：当前代码已生效，无需修改

**关键代码位置**（`can_protocol.cpp`）：

| 元素 | 行 | 说明 |
|---|---|---|
| `_pid_rcv_cnt[idx]` | 78-86 | ISR 内累加计数器 |
| `_pid_print_pending[idx]` | 78-86 | 4 次到齐后置 1 |
| `PollPidPrint()` 函数 | 99 | 主线程轮询 |
| `case 0x28 (DCE_Kp)` | ISR | `QueryDceKp` 回包 |
| `case 0x29 (DCE_Kv)` | ISR | `QueryDceKv` 回包 |
| `case 0x2A (DCE_Ki)` | ISR | `QueryDceKi` 回包 |
| `case 0x2B (DCE_Kd)` | ISR | `QueryDceKd` 回包 |
| `case 0x2C (加速度)` | ISR | `QueryAcceleration` 回包 → 缓存到 `jointAccRuntime[idx]` + 时间戳 |
| `case 0x2D (电流限幅)` | ISR | `QueryCurrentLimit` 回包 → 缓存到 `jointCurrentLimitRuntime[idx]` + 时间戳 |

**关键代码位置**（`ascii_protocol.cpp`）：

| 命令 | USB 行号 | 行为 |
|---|---|---|
| `#GET_PID` | 465 | 批量查 Kp/Kv/Ki/Kd |
| `#GETJACC` | 669 | 同步轮询 `jointAccRuntime[]` 30ms 超时 |
| `#GETI` | 721 | 同步轮询 `jointCurrentLimitRuntime[]` 30ms 超时 |
| `#SYNC_ACC` | 574 | 批量触发 8 电机 `QueryAcceleration()` |

**禁止事项**：
- ❌ 不要在 ISR 内直接 printf（阻塞 CAN RX）
- ❌ 不要把 `PollPidPrint()` 移除（合并打印机制是修复 UART TX 竞争的关键）
- ❌ 不要让 `#GETJACC`/`#GETI` 无限等待某电机回包（30ms 超时保护）

---

## 验证

| 测试 | 期望 | 命令 |
|---|---|---|
| 批量查询 PID | 一次性输出 Kp/Kv/Ki/Kd | `#GET_PID 1` |
| 单轴加速度查询 | 30ms 内输出 | `#GETJACC 1` |
| 单轴电流限幅查询 | 30ms 内输出 | `#GETI 1` |
| 批量触发 8 电机查询 | 8 电机并发 0x2C | `#SYNC_ACC` |
| 超时保护 | 电机不在线 30ms 后回包兜底值 | 拔掉电机测试 |
| ISR 标志位检查 | 4 次到齐后 print_pending 触发 | 调试串口 |

---

## 状态

- [x] 决策已记录（2026-09-24）
- [x] 6 个查询命令已生效
- [x] ISR 标志 + 主线程 poll 模式已生效
- [x] `PollPidPrint()` 已生效
- [x] 30ms 超时保护已生效
- 无需代码修改

---

## 与其他模块的关联

| 关联模块 | 关联点 |
|---|---|
| [09 - 电机参数配置](firmware_new主控固件模块09_电机参数配置.md) | 查询和配置是同一组参数的读写两侧 |
| [11 - 堵转协议通路](firmware_new主控固件模块11_堵转协议通路.md) | ISR poll 模式与堵转事件 ISR poll 机制相同 |
| [15 - 5kHz实时环](firmware_new主控固件模块15_5kHz实时环.md) | 主线程 poll 时机 |
