# dummy-motion 0.1.2

这是 Dummy 六轴机械臂加直线地轨的纯计算包，不访问串口、不导入 Qt，也不执行机器人动作。适用于离线规划和上位机的笛卡尔运动适配。完整上位机仍默认使用主控原生 MoveL。

## 安装与单位

在 `Dummy上位机` 目录运行 `python -m pip install .`，或安装交付的 wheel。依赖 Python 3.11+、NumPy 2.x、SciPy 1.13+。

- 七轴向量依次为 J1–J6 **弧度**、J7 **米**。
- `Pose.position` 使用米，`Pose.rotation` 是右手旋转矩阵。
- `frames.from_wire_joints()` / `to_wire_joints()` 转换串口的角度、毫米。
- `frames.from_xyzabc()` / `to_xyzabc()` 转换毫米、RPY 角度；内部姿态插补不经过欧拉角。
- 默认世界坐标与地轨零点基座重合，地轨沿世界 +Y。`RobotModel(world_rail=Pose(...))` 支持外参；不使用 URDF 显示的 Z 偏移或 J4/J5 显示符号。
- 默认工具为法兰局部 +Z 114 mm，无附加转角。这只是现有夹爪中心的**名义值**，未做标定；上位机保留用户已有工具长度。`Tool` 已接受完整刚体变换。

## 最小示例

```python
from dummy_motion import RobotModel, Tool, Pose, LinePlanner, MotionLimits, segment_timing
from dummy_motion.frames import from_wire_joints
from dummy_motion.validation import movej_command

model, tool = RobotModel(), Tool()
q = from_wire_joints([0, 10, 80, 0, 90, 0, 0])
start = model.fk(q, tool)  # 世界坐标 TCP
target = Pose(start.position + [0, 0.02, 0], start.rotation)
# MoveL 35档用于TCP规划速度；不作为MoveJ协议速度传入。
limits = MotionLimits(linear_speed=.007, angular_speed=0.06108652382)
plan = LinePlanner(model).plan_line(q, target, motion_limits=limits, tool=tool)
if not plan.ok:
    raise RuntimeError((plan.status, plan.reason))
previous, previous_time = q, 0.
for sample in plan.samples[1:]:
    _, quantized_q = movej_command(model, sample.q, sample.pose, tool, 1.)
    timing = segment_timing(previous, quantized_q, sample.time-previous_time, [30.]*7)
    command, _ = movej_command(model, quantized_q, sample.pose, tool, timing.speed_percent)
    print(sample.time, command, timing)  # 离线示例只打印，不发送
    previous, previous_time = quantized_q, sample.time
# 实机适配器用新鲜反馈估计替代 previous；30 r/s²须与设备读回值核对。
```

`IKSolver.solve_pose(target, seed, ...)` 求局部连续逆解，`RailPolicy.AUTO` 自动分配六轴和地轨，`LOCKED` 可用于离线对照。地轨代价随肩、肘的关节余量连续变化；已满足目标时直接返回，不进行静止零空间调整。IK 失败只代表当前种子及预算下未找到局部解，不能证明目标全局不可达。

`LinePlanner.plan_line()` 在返回成功前预检整条路径：平移直线与最短弧姿态共用进度，关节边界、数值残差、串口量化、段内插值误差及离散速度/加速度都要通过。预检失败返回空样本，调用方不能发送部分轨迹。默认笛卡尔速度上限 20 mm/s、10 °/s，发送采样间隔至少 100 ms，步长不超过 2 mm/1°；关节默认限制为 40 °/s、80 °/s²，地轨 40 mm/s、80 mm/s²。这些约束限定**生成的参考序列**，不是对主控 MoveJ 实际插补曲线的保证。

`MotionSession.update_feedback(FeedbackSample(...))` 与 `step(dt, target, now)` 用于在线计算；`commit()` 推进规划参考，调用方必须另存最后真正写出的参考。输入快照和工作线程结果不可混用世代。在线起步及反向遵守参考速度/加速度约束；旧时间戳被丢弃，预测最多 40 ms，300 ms 未收到有效反馈视为过期。GETJPOS 没有设备采样时间戳，收到新回包不等于七轴同步新采样。

## 上位机接入

软件设置的“笛卡尔运动逆解”选择主控或上位机，顶部状态同步显示。新配置/旧配置迁移默认主控；用户主动选择后保存在本机。已连接时先核对空闲状态，切换不会发送运动，也不会因求解失败自动换后端。

旧版普通 MoveJ 模拟执行器已经从上位机移除。`LinePlanner`、`IKSolver`、`MotionSession` 等纯算法 API 保留，供离线分析和算法调用；它们不再是应用内可选择的运动执行后端。原生主控 MoveJ/MoveL 仍保留原语义。

## STREAM_JOINT v2 / 0.3.0

`StreamPlanner(model, motor_acceleration=[30]*7, motor_velocity=[30]*7).plan_line(q, target, limits, tool)` returns immutable `StreamPlan` knots at exactly 20 ms. `plan_joint(q, target, speed)` supports finite joint moves. Each `StreamKnot.encode(session)` returns the 186-character `~S` ASCII hexadecimal record, with CRC-16/CCITT-FALSE, seven signed int32 motor q/v/a triples, seed and final END. No Qt, serial port, or device is imported or opened by this package.

The planner uses high-resolution local IK, a quintic joint spline with zero velocity/acceleration at finite endpoints, global time scaling, quantized-boundary reconstruction, joint extrema, velocity/acceleration/jerk limits, firmware-equivalent Bernstein bounds and Cartesian path checks. Both motor velocity and acceleration arguments must come from fresh device readback in an executing application. The defaults of 30 rev/s and 30 rev/s² are for offline examples only. UI speed affects Cartesian timing, never an ordinary MoveJ speed field.

This API preflights complete finite trajectories before execution. It does not implement future-window feedback correction, interactive continuous jog replanning, calibration, collision avoidance, mechanical identification, or real-device validation. The UI uses live joint feedback for communication watchdogs, actual-position display, and final settling checks; it does not rewrite already planned stream knots during execution. Multi-point callers must preserve endpoint completion semantics. A transport must separately enforce capability/session matching, FIFO credits, authoritative STOP, live feedback, and completion; encoding a packet does not authorize its execution.


Application v2 seeds every finite plan from fresh DIAG7 held-reference integers, not from rounded measured position or an assumed previous endpoint. The actual position is checked against ordinary motor settling tolerance; motors recheck readiness and latch the held reference for exact SEED matching. Each new point rereads this contract before planning and confirms the reference again afterward. The packet layout is unchanged, but VERSION=2 rejects older semantics.

Offline planning executes in a disposable Windows spawn process with no Qt or device access. The application still owns capability checks, actual limits, session cleanup, FIFO credits, STOP and actual-settling checks. Fifty hertz means a 20ms curve interval, not periodic USB writes or online feedback replanning. Feedback at 30–50Hz is PC receive time, not a device encoder measurement timestamp.
