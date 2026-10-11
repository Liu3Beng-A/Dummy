# Dummy 上位机与 firmware_new 审查建议

2026 年 10 月 10 日。此次已完成授权的手柄 STOP 调整；下列新增发现仅列建议，未修改固件或扩大上位机功能范围。优先处理逆解正确性、参数设置的实际结果、通讯解析隔离和界面阻塞，再处理性能及维护问题。

## 本次交付

修改前已提交全部未提交内容：`bc068bf36a037490fbfacd9b1a16b6faaf858b6a`，提交名为 `chore: snapshot current gamepad and desktop development before stop fixes`，32 个文件。使用仓库本地身份 Liu3Beng-A / liu3beng@icloud.com。以下新改动保留在工作区，没有追加提交或推送。

- 手柄主动 STOP 前持久记录 `GAMEPAD_STOP`，包含来源、原状态、原因、输入年龄、采集错误、按键、最近收发及在途命令；确认停止、释放后保留原因。
- 当前连续 MoveL 收到匹配编号的 `PLAN_FAILED`，退回最后受理的软件目标，等待新鲜的摇杆和扳机回中后再接受增量；不自动重发，不额外发 STOP。最后受理目标仍可能执行，软件回退不代表实机停止。
- 主窗口与本应用浮动终端之间切换不再主动释放手柄；切出应用、对话框、最小化、离开运动页等仍暂停。
- 没有恢复位置轮询、改变 0.5 秒输入超时、关闭保护或修改初始化只发一条 MoveJ 的约定。

修改文件为 [gamepad_session.py](E:/Dummy-code/Dummy上位机/app/core/gamepad_session.py)、[main_window.py](E:/Dummy-code/Dummy上位机/app/ui/main_window.py)、[gamepad_panel.py](E:/Dummy-code/Dummy上位机/app/ui/gamepad_panel.py)、[README.md](E:/Dummy-code/Dummy上位机/README.md)，以及对应的 session、UI、收发线程集成测试。完整 `python -B -m unittest discover -s tests -q` 通过 **288 项**，耗时 244.815 秒；其中针对 STOP 的首轮 75 项也全部通过。测试使用离线串口和手柄替身，没有实机运动或烧录。EXE 未重新打包，使用修改后的 Python 源码入口才能测试本轮行为。

## STOP 日志对比结论

用户附件的主控身份为 **2026101004 / 3D8460C4CBB44FA2**，电机身份为 **2026101003 / F08D62A917C34EB5**。本地对应构建清单的主控相关 349 项、电机及共享输入 139 项源码哈希核对一致。本轮基于这些实际源码分析。主控 04 相比 03 的变化是 RGB 启动状态结束处理，不能把它当作电机实时性优化版本。

| 证据 | 能确认的结论 |
| --- | --- |
| old 助手附件共 796 条 MoveL，51 次 `PLAN_FAILED`，无主动 STOP | old 助手允许目标规划失败后继续尝试；原手柄会话把这种错误升级为 STOP，行为确实不同。 |
| old 附件 21:43:41.913 的 X=187.5 目标失败，21:43:42.654 的 X=186.1 目标受理 | 失败可随目标改变而恢复，并不等于电机保护锁定。 |
| 独立上位机日志 21:28:21.656、21:32:40.141 先出现 `[TX] !STOP`，之后才是 `Stopping ok`、`STOP locked` | 这两次停止由上位机主动请求。该段此前没有 `PROTECT`，且没有 `PLAN_FAILED`，所以不能用规划失败解释这两次。 |
| 21:32 连续 12 条 MoveL 均已受理，最后一次为 21:32:36.705 | 不存在受理后仍在途、固定等待三秒就 STOP 的规则。未点初始化确认也没有单独的确认超时。 |
| 用户澄清未打开其他窗口，旧日志又未记录输入与 UI 停止来源 | 不能把这两次唯一归因于失焦；新日志用于区分停止键、采样超时、采集失败和界面暂停。 |
| 21:37:36.070 发出 MoveL 后无回执，21:37:40.086 写超时，21:38:10 断开 | 另有链路无响应和清理耗时问题；现有证据不能证明是主控死机或哪一个 Windows 调用阻塞了 30 秒。 |
| 较早日志有 J6 `PROTECTION_STATUS_TIMEOUT`，以及 J1 `STALL` 后 `REBLOCKED` | 固件真实保护确实发生过，必须与主动 STOP 分开判断，不能一概归为界面误停。 |

按小于 0.5 秒的连续发送间隔统计，old 助手中位数约 56ms，独立上位机约 77ms；当前日志不支持“Dummy 手柄比 old 更快，单纯把串口发满”的解释。主控对 `PLAN_FAILED` 不主动停止旧运动，而实际下发失败、通信健康故障、编码器故障等仍会进入保护。

日志来源：[用户附件](C:/Users/Liu3Beng/.codex/attachments/ffcf0fe1-282a-43ea-9917-2cfb8b103c5f/已粘贴的文本.txt)、[独立上位机日志](C:/Users/Liu3Beng/AppData/Local/DummyRobot/logs/dummyrobot.log)、[历史轮转日志](C:/Users/Liu3Beng/AppData/Local/DummyRobot/logs/dummyrobot.log.1)。构建身份来源：[主控清单](E:/Dummy-code/firmware_new/dummy-ref-core-fw/build/generated/fw_build_manifest.txt)、[电机清单](E:/Dummy-code/firmware_new/dummy-motor-fw/build/generated/fw_build_manifest.txt)。轮转日志和构建产物会随之后运行变化。

## 优先修正的正确性问题

以下编号供后续选项引用。P1 表示优先处理，P2 表示稳定性或功能一致性改进，P3 表示维护和体验优化。“代码确认”表示可从当前实现确定存在该路径；不代表已在你的设备上触发。

### 01 P1 不可达目标被逆解当作有效目标

**离线复现。** `SolveIK` 将超出三角形几何范围的余弦值钳到边界，随后仍将分支标为有效；`PlanMoveL` 仅检查关节角限位，没有用 FK 校验实际目标。目标法兰 `(300,0,300;180,0,180)` 会选择合法限位内的关节解，但 FK 得到约 `(235.913,0,247.021)`，位置误差 **83.15mm**。这比单纯返回 `PLAN_FAILED` 更值得优先修复，因为主控可能受理一个无法实现的目标。

建议补齐几何可达判定、有效解筛选、位置和姿态残差检查，失败时给出准确原因。[IK 几何计算](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/algorithms/kinematic/6dof_kinematic.cpp:330)、[MoveL 筛选](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:540)。

### 02 P1 逆解角度归一化改变了实际姿态

**代码确认，离线回环存在错误解。** 角度超过 ±π 时，代码加减的是 π，而不是保持旋转等价所需的 2π；会把一部分分支变成另一姿态，却仍标为有效。可达关节姿态的 FK→IK→FK 随机探针也出现明显位置和旋转误差。

建议统一周期归一化与关节限位内的等价角选择，并纳入跨 ±180°、J4 大范围、奇异位形的回归。[角度归一化](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/algorithms/kinematic/6dof_kinematic.cpp:454)。

### 03 P1 逆解连续性参考为零且角度单位契约不一致

**代码确认。** `PlanMoveL` 把全零 `lastJoint6D` 传给 IK；肩部和腕部奇异分支会直接使用它，随后才按当前关节角挑解。IK 内部把这个参考直接用于弧度三角运算，而外部 `Joint6D_t`/FK 使用度。简单把零数组换成当前关节角仍会留下单位错误。

建议明确参考角的单位，使用一致的关节快照，并建立靠近奇异点时的分支连续性规则。[调用](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:545)、[奇异分支](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/algorithms/kinematic/6dof_kinematic.cpp:411)。

### 04 P1 部分 ASCII 参数命令解析失败后仍会执行

**代码确认。** `#SET_PID` 使用未初始化的局部参数，忽略 `sscanf` 返回值；例如缺少 PID 值的命令仍可能使用未赋值变量。其他 `SET_DCE_*` 路径也有相同结构，命令分派大量依赖子串查找。上位机的部分校验还只匹配普通空格前缀，不能替代固件校验。

建议严格匹配命令名、参数数量、尾随内容和取值，任何解析失败都不产生 CAN 写入。[SET_PID](E:/Dummy-code/firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:580)、[上位机参数校验](E:/Dummy-code/Dummy上位机/app/core/controller.py:712)。

### 05 P1 USB 和 UART 共用半包解析状态

**离线复现。** `ASCII_protocol_parse_stream` 的缓冲、索引、丢弃标志是所有通道共享的静态变量。USB 输入半行 `>1,2`，UART4 随后输入 `#GETJPOS\n`，实际交给 UART4 处理的是 `>1,2#GETJPOS`。即使不发生并发抢占也会串包；双通道使用时还存在并发修改。

建议按物理输入通道隔离解析状态，并覆盖半包交错及重连清理。[解析器](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Bsp/communication/ascii_processor.cpp:48)。

### 06 P1 参数范围在上位机和电机之间不一致

**代码与离线校验确认。** 上位机允许加速度 0～5000r/s²，电机实际接受 0.1～100r/s²；上位机通用电流上限是 3A，35/42/夹爪电机接受上限为 2.5A，57 电机才是 3A。例如 `#ACC_J 1 101`、`#I_LIMIT_J 1 2.8` 能通过上位机校验，却被对应电机丢弃。部分夹爪电流入口还允许低于电机最小值 0.1A 的设置。

建议根据实际电机类型统一控件、保存文件、主控和电机的范围及单位。[上位机](E:/Dummy-code/Dummy上位机/app/core/controller.py:717)、[电机](E:/Dummy-code/firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:169)。

### 07 P1 参数设置成功回执没有证明电机实际应用或保存

**代码确认。** 主控调用 CAN 设置函数后立即输出 `ok SET ...`，带 `&` 时直接标 `saved`；没有等待电机确认应用或 Flash 完成。电机拒绝越界值、CAN 发送失败等情况下，上位机仍可能看到成功。PID 的四项设置也不是一个完整的确认事务。

建议分别表达已发送、已应用、已保存，使用电机确认或匹配回读判断成功。[主控回执](E:/Dummy-code/firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:603)、[PID 设置](E:/Dummy-code/firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:584)。

### 08 P1 普通电机参数持久化缺少统一的失能门禁

**代码确认，运行影响待实机量测。** PID 设置始终携带保存标志，普通加速度和电流也能请求保存；上位机保存门禁仅集中处理 RGB、STALL、CAL。电机主循环见到 `CONFIG_COMMIT` 就执行 EEPROM/Flash 写入，没有在此路径确认已经失能。多项 PID 连续写入还可能形成中间配置。

建议统一所有持久化入口的空闲失能条件，区分临时调参与完整保存，减少重复擦写。[上位机门禁](E:/Dummy-code/Dummy上位机/app/core/controller.py:542)、[PID 保存标志](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp:422)、[电机主循环](E:/Dummy-code/firmware_new/dummy-motor-fw/UserApp/main.cpp:173)。

### 09 P1 电机普通 CAN 命令缺少逐命令长度及数值检查

**代码确认。** 公共入口只检查 `_len >= 4`，但 0x07 会读取两个 float，多个保存/ACK 路径访问第 5 字节；部分位置、速度、电流命令直接将 float 转整数，缺少有限性和转换范围判断。DLC 为 4 的帧因此可能被当作完整的 8 字节命令处理。

建议逐命令规定载荷长度、数值范围和拒绝行为，并覆盖短帧、NaN、Inf 和溢出；不要只依赖上位机合法输入。[CAN 入口](E:/Dummy-code/firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:21)、[0x07](E:/Dummy-code/firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:135)。

### 10 P1 TUN 未经过常规关节限位并使用错误的时间步长

**代码确认。** 控制循环是 200Hz/5ms，却调用 `tuningHelper.Tick(10)`，正常调度下设置频率会以约两倍推进。TUN 直接向 `SetAngle(delta)` 发目标，绕过 `PlanMoveJ` 的逻辑关节限位与坐标转换。直接电机坐标振荡不能假定对所有关节都安全，尤其是校准参考靠近行程边缘时。

建议修正时间基准，对调试目标统一执行坐标及关节限制检查。[调用周期](E:/Dummy-code/firmware_new/dummy-ref-core-fw/UserApp/main.cpp:49)、[TUN 实现](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:1437)、[底层 SetAngle](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp:378)。

### 11 P1 串口关闭在 Qt 主线程同步执行

**代码确认，已有离线阻塞复现记录。** 串口失败信号在 Qt 主线程处理，直接进入 `disconnect_port → transport.close`，其中包含 `cancel_read`、`cancel_write`、端口关闭和线程 join。驱动返回慢时会阻塞窗口和手柄定时器。现有日志的写超时到断开约 30 秒与此路径值得联查，但不能据此确定具体阻塞函数。

建议将可能阻塞的资源清理移出界面事件循环，并明确关闭中状态和超时结果。[控制器](E:/Dummy-code/Dummy上位机/app/core/controller.py:397)、[传输清理](E:/Dummy-code/Dummy上位机/app/transport/serial_transport.py:66)。

### 12 P1 部分主控 CAN 方法仍共用可变报头和载荷

**代码确认，竞态影响需专项验证。** 常规运动新路径已使用局部帧，但 `SetPositionSetPoint`、PID 设置、部分查询仍修改对象的 `txHeader/canBuf`。TUN 控制任务可在通讯任务组装参数帧时抢占；同一电机对象的报头或数据可能被另一方法覆盖。

建议统一使用调用内独立帧或受保护的提交接口，清理新旧发送路径混用。[位置发送](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp:106)、[PID 发送](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp:422)。

## 稳定性与控制行为建议

### 13 P2 补齐运动解析任务的栈观测

**风险，未证实溢出。** 浮点 `sscanf` 和 IK 在 `ControlLoopUpdateTask` 内执行，该任务栈仍是 2000 字节；USB 解析任务已单独增到 4096 字节，不能据此推断运动解析任务也足够。`#STACK` 只报告 USB/UART 和 heap，无法观察此任务。

建议覆盖控制、命令、OLED 等任务的栈最低余量，并结合实际构建调用链确定配置。[任务栈](E:/Dummy-code/firmware_new/dummy-ref-core-fw/UserApp/main.cpp:221)、[STACK 输出](E:/Dummy-code/firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:251)。

### 14 P2 量测并压缩电机 20kHz 控制中断的最坏耗时

**风险，有历史诊断依据。** 电机控制周期预算 50µs，历史 `.03` 诊断的 `ctl_max` 达 46.83µs，且统计不包括完整中断进出开销。控制链中仍有 64 位运算、保护检测和软停统计，CAN 优先级低于 TIM4。这支持优先排查调度余量，不足以证明本次 STOP 就由中断超时造成。

建议补齐完整周期、前台最大间隔、CAN 响应延迟的同条件对照，针对量测到的热点优化；不直接放宽保护超时。[计时](E:/Dummy-code/firmware_new/dummy-motor-fw/UserApp/main.cpp:217)、[控制热点](E:/Dummy-code/firmware_new/dummy-motor-fw/Ctrl/Motor/motor.cpp:89)、[历史测量](E:/Dummy-code/docs/固件STOP异常调查交接_20261010.md:135)。

### 15 P2 保留固件异常现场而非只停在死循环

**代码确认。** 栈溢出钩子只保存任务指针，HardFault 等异常直接无限循环。设备不再应答时，串口日志难以区别异常类型、任务卡住和通讯故障。

建议保存可在下次启动读取的故障类型、关键寄存器及任务信息，明确恢复策略；不要在损坏的任务栈中直接大量打印。[栈溢出](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Core/Src/freertos.c:173)、[HardFault](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Core/Src/stm32f4xx_it.c:106)。

### 16 P2 统一 INT 和 TRJ 的实际语义

**代码确认。** 当前 `wait = mode != INT`，TRJ 与 SEQ 都等待真实完成，没有不减速衔接；INT 的新命令仍先入 FIFO，`Push` 不清除此前排队的普通目标，再逐个覆盖规划。与项目说明中的 TRJ 连续衔接、INT 清队列描述不一致。

建议先统一文案和协议契约，再按需要补真正连续轨迹或最新目标替换，避免用户根据模式名作出错误预期。[入队](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:1284)、[等待策略](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:1359)、[上位机拖发提示](E:/Dummy-code/Dummy上位机/app/core/business.py:932)。

### 17 P2 INT 重规划的同步算法缺少当前速度输入

**代码确认其假设，实际误差待实机量测。** 主控按静止出发的对称梯形公式计算各轴共同时间，只输入距离、加速度和限速；电机位置跟踪器保留运行中的速度。频繁覆盖或反向时，实际初始速度不为零，不能把该时间预算当作已经保证同步到达。

建议针对运行中重规划补齐速度状态和同步验证，区分静止点到点与连续追随场景。[主控公式](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:43)、[电机跟踪器](E:/Dummy-code/firmware_new/dummy-motor-fw/Ctrl/Motor/motion_planner.cpp:307)。

### 18 P2 MoveL 名称与实际路径不一致

**代码确认。** 当前 MoveL 是终点 IK 后调用同步关节 MoveJ，没有笛卡尔直线插补。手柄的 50mm/s 只是软件目标生成速度，途中 TCP 路径和速度由关节运动决定。

建议明确当前功能是笛卡尔目标点控制；若确实需要直线运动，再单独增加路径规划和路径有效性检查。[MoveL 调用 MoveJ](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:591)。

### 19 P2 手柄输入检查与界面工作共用同一事件循环

**代码确认，负载影响需量测。** SDL 采样与 0.5 秒输入检查都依赖 Qt 主线程；日志直接同步写磁盘，本地设置保存使用 `fsync`。界面或磁盘工作过慢既可能触发输入超时，也可能让检查本身延迟执行，不能仅靠调大超时解决。

建议记录事件循环延迟和采样间隔，隔离耗时 I/O，保留输入中断后的可靠停止与显式恢复行为。[采样器](E:/Dummy-code/Dummy上位机/app/gamepad/backend.py:29)、[输入检查](E:/Dummy-code/Dummy上位机/app/core/gamepad_session.py:695)、[日志连接](E:/Dummy-code/Dummy上位机/main.py:40)、[本地保存](E:/Dummy-code/Dummy上位机/app/storage/store.py:220)。

### 20 P2 STOP 后关键只读诊断被上位机拦截

**2026-10-11 更新：已按用户后续授权修正上位机放行表并增加终端单次诊断快捷入口、完整缓冲导出，固件未改。本项以下为原审查依据。**

**离线与代码确认。** 固件明确允许 STOP 锁定时的 `#MOTION_STATUS`、`#FW_VERSION`、`#CAN_DIAG`；上位机 `diagnostic()` 不包含它们，因而在故障现场反而不能查询。UART4 能力表也缺失固件共用入口已支持的版本/CAN 诊断。

建议按当前固件统一 USB/UART4、STOP、校准、手柄接管时的只读能力表，并补诊断页面或日志导出入口。[上位机能力表](E:/Dummy-code/Dummy上位机/app/protocol/capabilities.py:6)、[固件放行](E:/Dummy-code/firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:201)。

### 21 P2 统一普通发送与手柄的命令长度约束

**离线与代码确认。** 手柄限制运动正文 63 字节，普通 `send_cmd` 只检查 256 字节；一个合法数值但 82 字节的 MoveJ 可过普通校验，固件却以 `error motion queue full` 拒绝。ASCII 解析器对恰好 256 字节的行也会丢弃，实际完整行上限为 255。

建议统一各类指令上限、数值格式和超长错误，避免把长度问题误判为通讯堵塞。[上位机总入口](E:/Dummy-code/Dummy上位机/app/core/controller.py:490)、[固件运动队列](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:1286)、[行缓冲](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Bsp/communication/ascii_processor.cpp:57)。

### 22 P2 扩充请求身份和实际反馈的新鲜度信息

**协议局限。** 运动已有主控分配的 id，模式查询已有 req；START/STOP/夹爪和裸位置回报仍缺少完整客户端请求身份。主控位置回报是电机缓存，电脑收到回复的时间不能证明每个电机刚刚更新。当前本机世代隔离不能阻止其他物理通道同时控制。

建议后续统一请求与会话身份，并让实际位置携带节点有效性、采样年龄等信息。本项不要求恢复本轮已按用户要求取消的手柄位置轮询。[裸位置处理](E:/Dummy-code/Dummy上位机/app/core/controller.py:829)、[START/STOP 回执](E:/Dummy-code/firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:180)、[健康状态](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/instances/protection_coordinator.cpp:591)。

### 23 P2 上位机行长度限制漏掉完整超长行

**离线复现。** `LineFramer.feed` 只检查最后的未结束片段；一次输入 9000 字节加换行时会正常返回 9000 字节的一行，绕过默认 8192 限制。

建议完整行和片段执行同样的上限检查，并明确错误后的恢复边界。[LineFramer](E:/Dummy-code/Dummy上位机/app/protocol/parser.py:13)。

### 24 P2 USB 分块发送传错长度

**代码确认，当前短回执通常未触发。** `TreatPacketSinkAsStreamSink` 计算了 `chunk`，调用 `process_packet` 时却传剩余总 `length`。大于底层单包上限时直接失败，循环没有实现预期分块能力。

建议修正分块长度并覆盖跨包输出；现有 64 字节以内 Respond 回执不应被误认为已出现该故障。[USB 发送](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Bsp/communication/interface_usb.cpp:69)。

### 25 P2 UART DMA 启动失败时发送信号量未归还

**代码确认，依赖错误条件触发。** UART 发送先取信号量，`HAL_UART_Transmit_DMA` 失败就返回，没有归还。若没有随后完成回调恢复信号量，后续发送持续等待/超时。UART4、UART5 均采用此结构。

建议补齐所有失败出口的资源释放和错误计数。[UART4](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Bsp/communication/interface_uart.cpp:43)、[UART5](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Bsp/communication/interface_uart.cpp:81)。

### 26 P2 IMU 初始化失败阻挡控制和保护任务启动

**代码确认。** `Main` 在创建控制、命令和显示任务前无限重试 MPU6050 连接。外围 IMU 不可用会让主控停留在部分初始化状态，通讯已启动但后续机器人任务尚未创建。

建议明确外围设备失败时的有限重试、禁用运动和可诊断状态，并检查任务创建失败返回值。[启动顺序](E:/Dummy-code/firmware_new/dummy-ref-core-fw/UserApp/main.cpp:191)。

### 27 P2 明确手柄姿态旋转坐标系并改善奇异点行为

**代码确认其限制。** 姿态直接对三个 Euler 分量累加并归一化；这不是一般意义上的固定工具轴或世界轴角速度控制，接近 Pitch 奇异区时按键动作与直觉可能不同。矩形 XYZ 限制也不能表示真实可达空间。

建议明确界面所指的旋转坐标系，改善姿态插值、奇异点提示和可达边界提示；可达性最终仍由修正后的固件确认。[姿态积分](E:/Dummy-code/Dummy上位机/app/core/gamepad_session.py:669)、[配置范围](E:/Dummy-code/Dummy上位机/app/gamepad/config.py:27)。

## 维护与使用体验建议

### 28 P3 降低手柄状态发布与界面刷新的重复工作

每个采样和调度 tick 都可能发布完整深复制状态，界面随后刷新面板、姿态提示和全部冲突控件。建议按状态变化更新控件，将原始输入显示和运动调度分开限频；先量测，不预设这是当前卡顿根因。[会话发布](E:/Dummy-code/Dummy上位机/app/core/gamepad_session.py:686)、[界面刷新](E:/Dummy-code/Dummy上位机/app/ui/main_window.py:628)。

### 29 P3 集中机器人常量并区分坐标用途

关节限位分别出现在协议、存储、手柄配置、三维映射中；本地 FK 与固件也各持一份机械参数。法兰、TCP、地轨位移和 URDF 的 62.3mm 显示偏移需要持续保持一致。建议建立可核对的统一参数来源和单位说明，并在 MoveL 编辑区明确“法兰目标”。[手柄配置](E:/Dummy-code/Dummy上位机/app/gamepad/config.py:9)、[存储限位](E:/Dummy-code/Dummy上位机/app/storage/store.py:26)、[显示映射](E:/Dummy-code/Dummy上位机/app/visualization/mapping.py:13)、[本地 FK](E:/Dummy-code/Dummy上位机/app/gamepad/kinematics.py:18)。

### 30 P3 拆分业务继承与重复状态处理

`RobotController` 继承旧助手移植的 `RobotBusiness`，多处覆写又调用父类，双方都处理停止、模式、位置与回执；业务层还保留 `root.after` 和界面字段适配。建议逐步集中事务状态与协议解析，让 UI 订阅明确的状态事件，减少修改一个入口却遗漏另一入口的风险。[控制器回执链](E:/Dummy-code/Dummy上位机/app/core/controller.py:829)、[父类回执链](E:/Dummy-code/Dummy上位机/app/core/business.py:283)。

### 31 P3 清理误导性注释和过时状态说明

有明确可清理实例：`business.py` 注释仍称 JPOS/LPOS 都是六值且无法区分，代码实际按七值/六值区分；电机 `interface_can.cpp` 存在 3 个 U+FFFD 损坏字符及乱码注释；项目模式表将尚未实现的 TRQ/SRV 与已实现模式并列，TRJ 语义也过时。历史交接中的“04 尚不存在”等描述应保留日期并加当前状态索引。

建议按现行代码修正文案，删除失效的版本迁移备注和大段注释掉的调试代码，保留解释约束及协议兼容性的注释，不进行无目的全仓重排。[位置注释](E:/Dummy-code/Dummy上位机/app/core/business.py:334)、[乱码实例](E:/Dummy-code/firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:194)、[当前模式定义](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.h:128)。

### 32 P3 增加可追溯的软件版本和发布验收

上位机版本固定为 `0.1.0`，不区分本次手柄变更；README 明确现有 dist 是 QtCore DLL 启动失败的排查产物。建议日志和界面显示上位机提交/构建身份，配合九板固件版本导出；后续单独恢复 EXE 构建和干净环境启动验收，避免测试旧产物。[版本常量](E:/Dummy-code/Dummy上位机/main.py:35)、[打包状态](E:/Dummy-code/Dummy上位机/README.md:84)。

### 33 P3 将关键固件探针纳入长期可复现回归

此次上位机已有完整自动回归，固件逆解和跨通道问题仍靠独立离线探针暴露。建议把几何可达/不可达、FK 与 IK 回环、奇异姿态、跨通道半包、参数范围及失败回执纳入长期测试入口，记录所用固件来源哈希；避免只留下 build 目录中的一次性程序。

本轮探针位于 [stop-review](E:/Dummy-code/Dummy上位机/build/qa/stop-review)。IK 探针直接取当前 C++ 方程，仅将 CMSIS 三角函数换为宿主数学库并去除平台头；它验证算法逻辑，不测 MCU 时序。固定种子 123 的 10000 组合法关节回环中，26 组未得到限位内候选，69 组所选候选位置误差大于 1mm，55 组旋转矩阵元素误差大于 0.01；两类误差可能重叠，不能相加，数字也不是实机故障率。[IK 探针](E:/Dummy-code/Dummy上位机/build/qa/stop-review/ik_probe.cpp)、[通道探针](E:/Dummy-code/Dummy上位机/build/qa/stop-review/framer_probe.cpp)。

## 下一次现场验证重点

先重启源码版，按原流程 Start、初始化、观察到位并确认，再测试手柄；若停止，保留 `GAMEPAD_STOP` 前后的完整日志。若先出现 `PROTECT`、`STALL`、`MOTION ... fail`，按固件保护原因分析；若先有 `GAMEPAD_STOP` 和 TX STOP，则按记录的来源分析。查看本应用终端现在不应主动停止；切出应用仍会暂停。

此轮修正改善了可恢复错误处理和故障可定位性，**尚未证明已消除实际设备的所有 STOP 或通讯冻结**。优先建议下一批处理 01～03 的逆解、06～09 的参数与协议正确性、11 的串口清理，同时用新日志决定是否进一步处理采样/界面停顿或电机实时性。
