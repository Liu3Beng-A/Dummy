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

上位机模式通过 INT 和多条七轴 MoveJ 执行，点位反馈纠正与手柄规划定时器目标 50 Hz、起始串口发送上限 10 Hz，二者独立。普通 MoveL 先做整线预检，首段采用预检结果，后续段用最新反馈（最多预测40 ms）作为种子重新逆解，并检查相对实际已写参考的速度/加速度、量化与段内几何误差。重复量化目标合并；终点复用真实回执编号，绝不虚构完成。相邻发送间隔从成功写出回调开始计时，排队延迟不会引起补发突发。单个未写槽与编号回执账本分离；`accepted` 不是到位。有限点位必须得到末条编号的 `MOTION_STATUS phase=3`，并等待新关节反馈满足位置误差和至少 150 ms 稳定条件。跟随误差增大时降速/暂停，不用滞后回读重置累计目标；失去反馈、重复超时、STOP 或断连会取消本代轨迹。

混合示教序列在上位机模式由上位机管理，并保持 INT；主控模式保留原 SEQ 流程。普通手动 MoveJ 保留原行为。鼠标拖发期间只保留最新待执行目标，当前有限路径完成后再预检执行；手柄采用增量规划。实际模型仍只显示真实 GETJPOS 数据。

上位机 MoveL 的速度档位1–100 **仅参与笛卡尔规划**：100档最高20 mm/s、10°/s，35档7 mm/s、3.5°/s；线速度与角速度共用轨迹进度，速度/加速度约束和反馈降速仍可放慢轨迹。档位不再透传到生成的 MoveJ。

每段通过 `segment_timing(actual_q, quantized_target, duration, motor_acceleration)` 单独求协议速度：六轴角差乘减速比转电机圈数，地轨按5 mm/圈换算；按主控对称梯形模型反求达到该段时长所需的统一速度上限，再量化到协议0.01档。时长取当前采样时间减上一采样时间（包括合并的重复点），并应用反馈降速。模型最短时间仍大于期望时长时，延长该段的发令间隔。主控最终再求七轴各自的同步速度。

加速度默认暂用主控20261012.01的30 r/s²，并在每次任务首条指令前提示未确认状态；收到 `#GETJACC 1`～`7` 的读回或 `MOTION Jn ... a=...` 时更新对应轴。重连、修改加速度、重启电机或同步加速度后作废有关缓存；需要重新读回才标记确认。零值/非法回包不算有效模型参数。本程序不能感知外部客户端修改。

当前主控把MoveJ钳在1–100档。反算小于1档时只能用1档，并在日志报告“期望时长/模型预测时长”，可能提前到点等待。上述预测复现主控规划模型，不保证运动中覆盖目标时的真实到达时刻，也没有解决逐段速度连续问题；提高频率不能自动消除这项限制。普通手动MoveJ、示教MOVEJ点以及主控后端的原生MoveL保留原速度语义。

串口接收线程在读到数据时使用高分辨率单调时钟记录时间，Qt处理时不会把旧数据重新标记为新数据。关节限位、非有限值、时间戳倒序的拒收会记日志并阻止其同步到显示/编辑目标；日志同时包含RX到处理延迟。手柄采样和消费也使用相同类型的高分辨率时钟。

运动及示教坐标可选“基座/法兰”或“世界/TCP”。切换时转换两组编辑值并保留工具快照；连接时使用新鲜反馈地轨，离线时使用运动编辑区地轨并提示。转换超出输入范围会拒绝切换。旧点位、原始终端 `@` 均保留基座/法兰语义；执行时用入口地轨位置转换为世界 TCP。新世界 TCP 点位保存工具快照及版本，主控后端会把同一目标转换回基座法兰，保持地轨。主控原生 MoveL 仍是主控现有终点逆解行为，不承诺七轴直线插补。

## 验证与后续

运行 `python -m unittest tests.test_dummy_motion tests.test_motion_execution tests.test_motion_correction tests.test_cartesian_session -v` 验证纯计算和假串口执行；界面测试可独立运行 `python -m unittest tests.test_cartesian_ui -v`。`python tools/benchmark_motion.py --output benchmark.json` 测量完整 IK、20 ms 在线起步以及完整直线预检的 CPU 耗时，结果不包含 USB/CAN/电机延迟。

本版没有 TCP 标定、固定 TCP 移轨、碰撞检测或在线全局换解。工具实际中心、地轨正方向/零点、连续 MoveJ 跟随误差、30/50 Hz 回读与提高发令频率仍需低速实机验收。软件定时器和 Windows 线程不提供硬实时保证。
