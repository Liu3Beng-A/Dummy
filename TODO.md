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

---

## 待修复 Bug

### [BUG-1] `!START` 不自动归位（需先 `!HOME` 才会摆正） 🔴

**状态**：待修复（2026-09-01）

**现象**：
- 用户执行 `SET_ORIGIN`（`ApplyPositionAsHome`）把当前机械臂姿态设为 0°
- 然后发送 `!START` 使能电机
- 机械臂**不会**主动走到 home 姿态（`MoveJ(0,0,90,0,0,0,0,10)`）
- 必须再发 `!HOME` 才会触发归位运动

**期望行为**：
- `!START` 后电机使能的同时，应自动归位到 home 姿态
- 让"启动 = 上电 + 摆正"符合直觉
- 或者至少提供一个 `!START_NOHOME` 让用户按需选择

**根因**：
- `DummyRobot::SetEnable(true)`（`dummy_robot.cpp:622`）只调用 `SetRGBMode(rgbStateEnable)`，**不**触发任何 MoveJ
- `DummyRobot::Homing()`（`dummy_robot.cpp:585`）才是负责 MoveJ 到 home 姿态的唯一入口
- UI 视角下"启动"和"归位"被拆成了两步，但用户心理模型是"启动 = 安全姿态就绪"

**复现步骤**：
1. 上电，机械臂停在非 home 姿态（例如 J1=90°）
2. 串口发送 `SET_ORIGIN_J 1` → 把当前角度设为 0°
3. 串口发送 `!START` → 电机上电，**J1 仍停在原物理角度**，不归零
4. 串口发送 `!HOME` → 立即开始 MoveJ 到归零姿态

**风险**：
- 用户以为已就绪，实际上电机带电但姿态是任意角度
- 后续 MoveJ/MoveL 目标点可能与当前角度相差很大 → 启动瞬间大电流冲击
- 误操作夹爪/抓取时容易撞限位

**修复方向**（任选其一）：
1. **简单方案**：`!START` handler 改为先 `SetEnable(true)` 再 `Homing()`（`ascii_protocol.cpp:78` 改两行）
2. **进阶方案**：增加 `!START_NOHOME` / `!START_HOME` 两条命令，让用户显式选择
3. **可配置方案**：在 Flash 存一个 `auto_home_on_start` 配置位（默认 ON），`!START` 据此决定是否归位
4. **REST_POSE 触发**：`!START` 改为先 `Homing()` 再 `Resting()`，让启动直接进入待机位

**相关代码**：
- `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp:78` `!START` handler
- `firmware/ref_core_f405/UserApp/protocols/ascii_protocol.cpp:786` UART4 同名 handler（USB/UART 两份要同步改）
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:622` `DummyRobot::SetEnable`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:585` `DummyRobot::Homing`
- `firmware/ref_core_f405/Robot/instances/dummy_robot.cpp:603` `DummyRobot::Resting`
