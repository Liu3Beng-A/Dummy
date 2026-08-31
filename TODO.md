# TODO — Dummy 7轴机械臂控制项目

> 本文档追踪**计划开发**的功能与待优化项。
> 已修复的 P0~P3 问题见 `ISSUES.md`。

---

## 优化项（低风险，随时可做）

### [OPT-1] 串口 `ok` 响应处理算法优化 🟡

**状态**：待优化（2026-08-31）

**背景**：
`serial_receive_loop` 中检测到固件回 `ok`（用于 SEQ 顺序发送下一条）时，当前路径是：

```
固件回 ok → 串口 RX buffer
  ↓ 最坏等 50ms
serial_receive_loop 轮询 in_waiting
  ↓
root.after(0, ...) 切主线程
  ↓ 主线程事件循环
_send_next_position → 串口 write
```

**当前问题**：
- 50ms 轮询间隔（即使有数据也需等待）
- 双层 `after(0)` 调度（子线程→主线程→子线程再 send_cmd）
- 实测 SEQ 点位间隔 60-90ms，其中大部分是 python 端等待

**优化方案**：
1. 空闲时轮询改为 10ms（原来 50ms）；有数据时立即连续读，不 sleep
2. SEQ 模式下，检测到 `line == "ok"` 时直接在子线程调用 `_send_next_position()`，跳过 `root.after(0)` 调度
3. UI 日志和滑块同步仍走 `after(0)`（Tk 线程安全要求）

**预期收益**：
- SEQ 点位间隔：60-90ms → **5-15ms**
- 改动量小（仅 `serial_receive_loop` 内部，约 5-10 行）

**风险评估**：
- 串口 `write` 线程安全（pyserial 内部加锁）✓
- `self._pos_queue_running` 等简单布尔读写受 GIL 保护 ✓
- 日志 `_log` 内部用 `after(0)`，子线程调用安全 ✓

**相关代码**：`串口助手.py` L1304 `serial_receive_loop`
