# STOP 软停实现与测试报告

日期：2026-10-05。当前状态：J8 重新烧录后 START/DISABLE 握手成功；STOP 判稳窗口优化已通过编译及主机回归，用户实机复测反馈“这次没有什么显著的问题”。未取得逐项验收日志，此反馈不代表此前 HOME/USB 独立异常已解决。

用户询问上轮主控等待诊断是否需要撤回：建议保留。详细 USB 快照仅在 STOPPING/STARTING/DISABLING 持续至少 2 秒后输出一次，重复 START 可重新触发；输出位于 OLED 任务，诊断记录不参与判稳/解锁决策。它定位未完成确认的节点及阶段，不能单凭该输出认定关节“卡死”。本轮仅同步实机反馈与核对结论，没有改固件或重新编译。

## 当前优化：消除瞬时估速跳变对 STOP 判稳的干扰

19:08～19:10 的慢 STOP 均已收到七轴 0x0A 回包且 token 正确，等待节点为 J4 或 J4/J6，phase=1（DECELERATING）。这与缺失确认包不同；提前发送 START 保留 pending，待七轴完成 STOP 后收到 `STOP locked` 和 `Started ok`，符合既定决策。用户补充肉眼观察各轴已经停止，触碰机械臂后才完成确认；尚未采集实机编码器波动，不能断言实际噪声幅度或 PID 原因。

旧判据逐个 20kHz 周期要求瞬时估计速度≤0.01 电机 r/s，持续 50ms。实际估速函数在零速时接收一次 1 内部单位的位置变化就产生 625 单位/s（0.012207 r/s），可反复打断计时。使用真实完整控制函数和模拟反馈，位置仅在 3 单位范围跳变时复现了等待 20 秒仍不确认。测试输入是软件构造的反馈，不是实机采样。

本轮仅修改电机 [motor.cpp](E:/Dummy-code/firmware_new/dummy-motor-fw/Ctrl/Motor/motor.cpp:571) 和 [motor.h](E:/Dummy-code/firmware_new/dummy-motor-fw/Ctrl/Motor/motor.h:156) 的 STOP 判稳窗口。参考速度归零后，要求 50ms 窗口的位置峰峰值≤8 内部单位，同时窗口估计速度的有符号均值绝对值≤0.01 电机 r/s。详见第 2 节。没有修改本地减速加速度、PID、普通到位、协议、主控或串口助手。

验证结果：

- 28 组真实完整 20kHz 控制函数反馈测试：静止、1/3/4/6/8 单位有界跳变及 6 单位三角波均在零速参考后的 50ms 完成确认；9/16 单位超范围波动、±0.004 r/s 持续漂移及 0.02/1 r/s 运动在 20 秒预算内均未确认。
- 新版噪声回归 2,422,094 项断言通过；真实电机 35/夹爪原有回归分别 10,629,070 / 10,629,068 项通过，含 78 组速度/加速度、STOP 重试、保持隔离与 DISABLE。测试文件及输出保存在主控 `build/analysis/stop_regression/`。
- 四种电机 Ninja 36/36 成功；主控和电机最终 `ninja -n` 均无待构建项。既有编译警告未修改，实机闭环仍需复测。

当前电机产物（2026-10-05 19:21:27，RAM8176B）：

| 目标 | BIN / FLASH 字节数 | SHA256 |
| --- | ---: | --- |
| [motor-35.bin](E:/Dummy-code/firmware_new/dummy-motor-fw/build/motor-35.bin) | 72816 | `6676AED6D07DAAFF1763C8018259126D4493F708CD92FB30C99EE58A4AEB8D90` |
| [motor-42.bin](E:/Dummy-code/firmware_new/dummy-motor-fw/build/motor-42.bin) | 72816 | `6676AED6D07DAAFF1763C8018259126D4493F708CD92FB30C99EE58A4AEB8D90` |
| [motor-57.bin](E:/Dummy-code/firmware_new/dummy-motor-fw/build/motor-57.bin) | 72808 | `9F249A76789A3BDBFF4C5F057D4E246614208A52216E8F9F3440680FE4ED74A8` |
| [motor-gripper.bin](E:/Dummy-code/firmware_new/dummy-motor-fw/build/motor-gripper.bin) | 72776 | `216231A136A1847CA9914D1D4D1BADBDBC07FAF55185165DE4DE37A99C67350D` |

本轮只需更新 J1～J7：J1～J6 按实际电机类型选 35/42，J7 选 57。主控保持现有诊断版，J8 可保持刚验证工作的版本；本轮没有改变协议，J8 也不参与 STOP 判稳。当前主控 BIN389604B，SHA256 `8BF59F47D9CFA1274A47A938A78BD8E27DB3D688293BFE9C27A8BA904004490E`。

实机复测重点：在不同方向与速度的运动中 STOP，不碰机械臂，确认可完成停稳并保持停点；减速期间发送 START 应等七轴停稳后自动解锁，保持原停点；DISABLE 应让八轴失能。若仍慢，保留 phase/token/等待节点日志。软件测试通过不代表上述实机效果已经验证。

另外，19:01 HOME 后 DISABLE 日志 `polls=0 rx=0` 表示该阶段控制任务尚未继续轮询，不能归因于本轮瞬时估速判稳问题；原因仍未确认。19:10:50 的 USB ClearCommError 也未定位。本轮仅记录，没有扩展修改 HOME、普通 CAN 发送或 USB。

## 前一阶段：实机反馈与主控诊断版（历史记录）

用户确认主控及全部八个电机均已更新。COM20 是主控 USB 虚拟串口；UART4 是暂不用的 ESP32 通道。J6 重新烧录后恢复反馈，OLED 上 J1～J7 均能刷新，夹爪不在 OLED 上。多次 START/DISABLE 仍只收到 pending，尚未取得 0x0A 的各节点状态。

串口助手的“STOP 锁定中，仅接受 !START 或 !DISABLE”警告来自用户点击 MoveJ 等其他命令后的正常拦截，不据此认定主控上电即收到 STOP。本轮没有修改串口助手拦截规则。

仅更新主控 `dummy_robot.cpp/.h`：某个 STOP/START/DISABLE 阶段持续等待超过 2 秒后，在 USB 输出一次等待快照；等待期间再次 START 可重新输出。原确认条件与电机协议不变，不以超时认定成功。

诊断字段：

- `state` 是主控阶段，`action` 是正在发送的电机协议动作。START 包括 QUERY 和 ENABLE_HOLD 两个阶段。
- `ack` 是已通过当前阶段确认的节点，`wait` 是仍未通过的节点，`rx` 是当前阶段收到过 0x0A 回包的节点。位 `1 << nodeID` 对应该节点，例如 `wait=0x100` 只表示 J8 未通过确认，不能独立证明 J8 离线。
- `badpos` 表示状态/token 匹配但位置为非有限值，因此不能计入确认。
- `polls` 是该阶段控制任务调用次数；`txbusy=1` 表示一次 SendSoftStop 发送调用在快照时尚未返回，`node` 是该调用的节点，0 代表 ALL。它不是 CAN 硬件确认成功标志。
- 未确认节点打印 `no 0x0A reply`，或打印最新 `phase/token/expect`。phase 值见下方协议表。旧 token、减速中状态和故障都不能放行。

本轮真实主控函数回归共 391 项断言通过，包含等待诊断及原有 STOP 状态行为。主控 Ninja 成功，当前 BIN 为 389604 字节、RAM55400B，SHA256 为 `8BF59F47D9CFA1274A47A938A78BD8E27DB3D688293BFE9C27A8BA904004490E`；电机四种 BIN 与初版一致。本次只需更新主控。

用户下一步：发送一次 START 后等待至少 3 秒，再发送一次 DISABLE 后等待至少 3 秒，记录全部新增 STOP 日志。当前诊断版用于取得实机证据，尚未宣称解决 pending。下面保留初版行为、软件验证及原始构建记录；当前主控产物以本节为准。

### 18:49～18:50 实机定位

用户已回传诊断：START 停在 QUERY，DISABLE 停在 DISABLE。两阶段均为 `ack=0x0FE wait=0x100 rx=0x0FE badpos=0x000`，控制任务已轮询 403 次，快照时发送调用没有阻塞，唯一缺失节点为 `J8 no 0x0A reply`。这证明主控已收到并接受七轴确认，八轴完成条件因 J8 无回包而未满足；不能把它解释为 J8 未停稳、FAULT 或 token 不符。START 此时尚未进入 ENABLE_HOLD。

核对当时源码及夹爪 ARM ELF：MOTOR_TYPE=8，Main 设置节点8，0x0A QUERY/ENABLE_HOLD/DISABLE 均有即时状态回包分支；QUERY 不依赖电机控制中断处理完请求，也不等待编码器校准。产物 SHA256 与初版一致。用户确认实际烧录路径正确并重新烧录 J8；19:01 随后多次 START/DISABLE 均成功完成 pending→ok，该次缺失 J8 确认的问题已解决。

## 1. 已实现的行为

| 场景 | 结果 |
| --- | --- |
| 收到 `!STOP` | 立即锁定主控命令入口，取消旧运动队列和后台任务，停止周期发布旧目标；向 J1～J7 发送电机本地软停请求。 |
| J1～J7 减速 | 每轴以当前估计实际速度为起点，使用该电机本地正在生效的 `ratedVelocityAcc` 减速至零，保留原有 PID、限流和保护。 |
| 单轴实际停稳 | 捕获该轴编码器累计实际位置，切换位置闭环保持；从速度闭环切换时保留当前输出力矩的积分分量。 |
| 七轴均确认停稳 | 主控进入 STOP 锁定保持态；没有 START/DISABLE 时不会自动解锁。 |
| 减速过程中收到 `!START` | 记录请求；全部七轴停稳并捕获停点后自动执行 START 握手。减速不被打断。 |
| 停稳后收到 `!START` | 保持已捕获的原停点，确认七轴状态后解锁；主控目标回填到停点，旧队列、旧目标和 TUN 激励不恢复。 |
| 收到 `!DISABLE` | 取消待执行 START，广播失能并逐轴重试；确认 J1～J8 全部失能后解除 STOP 锁定，保留失能状态。 |
| 失能后收到 `!START` | J1～J8 在各自当前实际位置使能保持，完成八轴确认后允许接收新动作。 |
| STOP 期间收到其他指令 | 丢弃，包括运动、HOME、RESET、CALIBRATION、参数查询/设置、RGB 和新夹爪指令；ASCII 返回 `error STOP_LOCKED`。 |
| J8 夹爪 | 不接收 STOP；STOP 和紧接着的 START 均不改变其原有动作。DISABLE 包括夹爪。 |

START/DISABLE 为完整指令匹配，可带首尾空白；`!START extra`、`!XDISABLE` 等不能解除锁态。STOP 期间的重复 STOP 也不会重启减速。

这里的加速度、速度均为电机轴 r/s²、r/s；软停不使用主控中可能滞后的加速度查询缓存，也不重新分配同步运动加速度。

## 2. 电机停稳判据与保持

电机在既有 20kHz 控制周期中处理软停，复用现有 `VelocityTracker`，没有修改 `motion_planner.cpp` 算法。

当前判据：只有软停速度参考为零，且随后 1000 个周期（50ms）的窗口同时满足以下条件，才捕获停点：

1. 窗口内参考速度持续为零。
2. 窗口估计实际速度的有符号均值，取绝对值后不超过 0.01 电机 r/s；不再逐周期用瞬时估速清零计时。
3. 窗口内编码器累计实际位置的最大值减最小值不超过 8 个内部细分单位。

参考速度非零或位置范围超限时立即清零窗口；50ms 末平均速度超限时重启窗口。位置峰峰值≤8 比旧版窗口起点±8 的最大允许范围更严格，速度均值用于容纳小范围来回跳变；不以等待超时作为成功。“速度参考归零”本身不等于“实际停稳”。各轴可在不同时间停稳，已停轴立即本地保持；主控等待全部 J1～J7 的确认。

捕获的是 `realPosition`，而不是旧运动目标或旧轨迹规划位置。保持期间屏蔽普通目标、模式和规划更新。START 从 HOLDING 状态解锁时保留原捕获位置，之后的微小编码器波动不重新选择停点。

STOP 不会使能原先失能的电机。无有效编码器校准、已有堵转保护或无有效本地加速度时，不假报 HOLDING；主控收到 FAULT 后保持锁定并报告节点位图。已有保护不会被 STOP 清除。

## 3. CAN 握手与异常边界

公共定义：[stop_protocol.h](E:/Dummy-code/firmware_new/stop_protocol.h)。标准 ID 仍为 `(nodeID << 7) | cmdCode`，新增命令码 `0x0A`。

| 字段 | 请求（8 字节） | 回包（8 字节） |
| --- | --- | --- |
| 0～3 | action 位于字节 0，uint16 token 位于字节 2～3 | float 电机轴圈数位置 |
| 4 | 保留 | phase |
| 5 | 保留 | 保留 |
| 6～7 | 保留 | uint16 实际 token |

action：QUERY=0、STOP=1、RELEASE=2、ENABLE_HOLD=3、DISABLE=4。主控 START 使用 ENABLE_HOLD；RELEASE 为内部协议动作，不能通过普通主控指令绕过 STOP。phase：NORMAL=0、DECELERATING=1、HOLDING=2、DISABLED=3、FAULT=4。

- 首次 STOP 逐轴 QUERY 同步仍在运行的电机 token，再为每个响应轴发送不同于它当前 token 的 STOP。无需等其他节点响应：一个节点缺失不会阻止其他节点软停。
- START 先查询参与节点的 token，再选择一个不与任何参与节点当前 token 相同的值，避免主控单独重启后重复 token 导致请求被当成重试。
- 匹配节点、阶段与 token 的回包才能计入确认；旧 token、未停稳回包和 STOP 中的 J8 回包不能放行。
- DISABLE 首次使用 ALL 广播（节点 0），后续逐节点重试，要求八轴确认。
- 缺失确认时持续重试，保持锁定，不会以超时作为成功。STOP 保持后仍查询电机软停状态，故障不会因先前完成过握手而被忽略。
- 电机 QUERY 不改目标、模式、使能或积分；夹爪 QUERY 不会中断它的电流动作。
- 软停请求/回包使用局部 CAN 报文，避免被普通周期查询共用缓冲改写。

## 4. 主控任务取消与命令入口

主要实现：[dummy_robot.cpp](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:559)。

队列项增加运动代数 epoch。STOP/DISABLE 同时清队列、增加 epoch；已出队或正在执行的旧任务也要检查 epoch，不能在 STOP→START 后重新发布旧目标、恢复旧速度或回报旧任务完成。

HOME、RESET、CALIBRATION 原来在接收命令的任务中执行并等待，导致同一串口的 STOP 无法及时处理。本次仅把它们转入既有后台命令任务，并在等待/后续发送前加入取消检查，保留原有动作内容；没有实施新的校准功能。

CAN 发送在等待信号量之后再次检查锁态、epoch 和后台任务是否已取消；被阻塞的旧控制/参数帧不会在解锁后继续发出。内部位置查询及软停握手保留。

原生 USB 在收包时记录锁态和 epoch，丢弃 STOP 期间收到或跨 STOP 留下的待处理包。原生 UART 在 epoch 改变时清除未接收完整的包，并在锁态中停止喂入原生协议。为此在 Fibre 分包器中仅增加 `discard_pending_packet()`，没有改变普通包解析流程。

两套 ASCII 入口（USB、UART4）都经过 STOP 门禁；UART5 原有业务入口在锁态中丢弃。电机 CAN、调试 UART 及板载模式/校准按键也增加锁态拦截。

状态文字由显示任务发送，控制周期不等待 USB/UART 输出。主要回执：`Stopping ok`、`STOP locked`、`STOP locked disabled`、`Start pending`、`Disable pending`、`Started ok`、`Disabled ok`、`STOP fault mask=0x...`。

## 5. 目标串口助手

只修改 [串口助手_old.py](E:/Dummy-code/串口助手_old.py)，参考 `串口助手.py` 未修改。

- STOP 按钮显示“软停”；收到完成回执后再更新使能/失能状态。
- STOP/DISABLE 清除待发点位、顺序发送和同步等待；发送队列带 generation，已出队的旧命令也会被发送线程丢弃。
- STOP 锁定期间只允许 START/DISABLE，不允许重新启动顺序点位。
- 新增 STOP 状态回执的分段接收拼接，完整行到达后再更新锁态；没有改动其他查询回包的原有解析规则。
- 所有接收引发的控件变化继续通过 `root.after()`。

## 6. 软件验证

测试产物位于 [stop_regression](E:/Dummy-code/firmware_new/dummy-ref-core-fw/build/analysis/stop_regression/generate_tests.py)，属于构建目录内可复查的测试工件。

| 测试 | 结果与范围 |
| --- | --- |
| 主控真实函数 | 336 项断言通过；提取真实状态机、队列、ASCII 门禁、HOME/RESET/CALIBRATION 取消和 CAN 发送函数。覆盖 1～4 模式、缺失/旧回包、提前 START、DISABLE、混合使能、主控重启、token 回绕、缺失冷启动节点。 |
| 电机 35 目标 | 10,629,070 项断言通过；编译真实 `motor.cpp`、`motion_planner.cpp`、CAN 处理代码。 |
| 电机夹爪目标 | 10,629,068 项断言通过；除公共控制外，验证实际 CAN 入口忽略 STOP，QUERY 保留原电流动作，DISABLE/使能有效。 |
| 电机组合 | 78 组正/负/零速度与本地加速度组合；验证速度单调下降、加速度上限、理论减速时间/距离、重复请求、参考零速但实际未稳时不捕获。覆盖完整 20kHz 控制函数的保持规划隔离与 DISABLE 的 driver Sleep 调用。 |
| 原生 UART 真实分包代码 | 1475 项断言通过；使用实际 CRC、分包器类和发送/接收函数，对 1/8/32/64/120 字节有效包的各切分位置测试旧半包取消及新包正常接收。 |
| 真实 Tkinter 界面 | 389 项断言通过；隐藏窗口创建、STOP 取消队列、发送线程拦截、状态显示，以及所有 STOP 状态文字的每种拆分位置。Python 源码语法检查通过。 |
| 补丁范围/格式 | `git diff --check` 通过；`esp32/`、`firmware/`、`firmware_old/`、`串口助手.py` 均无本轮改动。 |

主机测试替换 RTOS、CAN、编码器和驱动硬件接口；减速曲线测试采用理想跟随的反馈输入。它们验证软件状态、参考曲线与控制路径，不证明真实电机的机械制动距离、重力负载下保持、实机 PID 过渡或总线时序已合格。尚未代理连接设备、下发命令或烧录。

## 7. 初版编译与烧录产物（历史记录，当前产物见开头）

主控及电机工程均在其 `build/` 下执行 Ninja，退出码为 0；最终 `ninja -n` 均返回 `no work to do`。ARM GCC 15.2.1，沿用既有构建配置。

| 目标 | BIN 字节数 | FLASH | RAM |
| --- | ---: | ---: | ---: |
| 主控 F405 | 387380 | 387380 B | 55360 B，另有既有 CCMRAM 65536 B |
| motor-35 | 72672 | 72672 B | 8160 B |
| motor-42 | 72672 | 72672 B | 8160 B |
| motor-57 | 72664 | 72664 B | 8160 B |
| motor-gripper | 72640 | 72640 B | 8160 B |

烧录文件（同目录另有 HEX/ELF）：

- [Core-STM32F4-fw.bin](E:/Dummy-code/firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin)
- [motor-35.bin](E:/Dummy-code/firmware_new/dummy-motor-fw/build/motor-35.bin)
- [motor-42.bin](E:/Dummy-code/firmware_new/dummy-motor-fw/build/motor-42.bin)
- [motor-57.bin](E:/Dummy-code/firmware_new/dummy-motor-fw/build/motor-57.bin)
- [motor-gripper.bin](E:/Dummy-code/firmware_new/dummy-motor-fw/build/motor-gripper.bin)

需要主控和全部八个电机使用配套本轮固件；关节按实际电机类型选择 35/42，地轨选择 57，夹爪选择 gripper。夹爪虽不参与 STOP，也必须更新以支持本轮 DISABLE/START 确认协议。已有 EEPROM 布局、PID 默认参数与电机类型构建配置未改。

SHA256：

```text
Core-STM32F4-fw.bin A946452E8B4E0CAF724A11D6C1F49ED488EBCDECE8BFFCB1BB85B9E3E55BC8F7
motor-35.bin        2464B48EAE6B820DB7D538EBF9F79F94FA4C7FEB0A11BF78FFED0337CAF46306
motor-42.bin        2464B48EAE6B820DB7D538EBF9F79F94FA4C7FEB0A11BF78FFED0337CAF46306
motor-57.bin        24CD432F1FB2C45787299C9280D0FAAD582C64903B5F7E1DEB48B8BBE0E840CC
motor-gripper.bin   AEFAEAC29BD8CA2D4654F6F6213FD811C11BB533BA83C7FB35583305EAFA408C
```

## 8. 建议实机验收顺序

1. 更新配套主控与所有电机固件，重新启动目标串口助手。先 `!START`，等待 `Started ok` 后以低速进行一段七轴动作。
2. 运动中发送 `!STOP`：应先收到 `Stopping ok`，七轴逐轴减速并保持实际停点，随后收到 `STOP locked`。等待一段时间确认不会自行恢复旧目标。
3. 锁态分别发送运动、HOME、RESET、参数和夹爪命令：应被拦截。夹爪在 STOP 前已开始的动作应继续。
4. 分别测试停稳后 START、减速中立即 START：后者先显示 `Start pending`，完成软停后收到 `Started ok`，保持原停点；只有发送新运动指令才继续运动。
5. 在减速中及保持态分别发送 DISABLE：待执行 START 应取消，J1～J8 全部失能，收到 `Disabled ok`。再次 START 应在各轴当前点使能。
6. 再测试不同方向、速度及各轴本地加速度；检查 J2/J3 重力负载下速度→位置保持的过渡，确认 J7 和 J8 的范围与上述行为一致。若没有收到最终确认或出现 `STOP fault mask`，记录回执与节点，不把 pending 当作已经完成。

## 9. 发现但未修改的其他问题

以下只记录代码证据，不在本轮修复；不能据此宣称它们是任何既有偶发现象的实机根因。

| 问题 | 直接证据与影响 |
| --- | --- |
| 普通位置运动 FINISH 不检查实际到位 | [motor.cpp:325](E:/Dummy-code/firmware_new/dummy-motor-fw/Ctrl/Motor/motor.cpp:325) 仍只检查 `softPosition==goalPosition && softVelocity==0`，普通动作可能在物理跟随尚未完成时标记完成。本轮 STOP 使用独立实际停稳判据，没有修改普通 FINISH。 |
| UART5 原生接收用了 UART4 分包器 | [interface_uart.cpp](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Bsp/communication/interface_uart.cpp:170) 的第二组 DMA 接收数据仍喂入 `uart4_stream_input`。本轮只加 STOP 门禁与半包清除，没有修正路由。 |
| 多通道 ASCII 共用一套半行缓冲 | [ascii_processor.cpp:50](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Bsp/communication/ascii_processor.cpp:50) 的静态 buffer/index/read_active 被各通道共用，并发或交错半行可能混合。本轮未改解析架构，单通道门禁测试不证明并发通道解析已修复。 |
| 普通电机发送仍共用准备缓冲 | [ctrl_step.cpp:82](E:/Dummy-code/firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp:82) 等普通方法在取 CAN 信号量之前写共享 `canBuf/txHeader`，存在竞争窗口。本轮 STOP 和相关使能帧使用局部报文，未全面重构普通发送。 |
| 既有非 STOP 串口回包仍按单次读取拆分 | [串口助手_old.py](E:/Dummy-code/串口助手_old.py:819) 原 `_split_rx_lines` 不跨读取拼接普通查询回包。此次只为新 STOP 状态添加分段保护，没有改造其他回包解析。 |
| 既有编译警告 | [mt6816_base.h:42](E:/Dummy-code/firmware_new/dummy-motor-fw/Ctrl/Sensor/Encoder/mt6816_base.h:42) 空的非 void 虚函数缺少 return；链接器仍报告 LOAD segment RWX。编译成功，本轮未改这些独立问题。 |

没有 commit；由用户决定后续烧录测试与提交。

## 10. 2026-10-06 保护/校准融合后的补充

上述为原STOP阶段记录。新一轮已授权实现堵转自动回退和CALIBRATION，仍复用本地软停与固定实际停点；普通STOP七轴、DISABLE八轴、START保持当前位置的语义保留。新增内部保护联动动作5、每次START的坐标范围同步，以及校准取消时J8专用停止/重新START解锁；编码器无效的DISABLED确认独立于位置有效性。

为保护在CAN故障下不中断，新增邮箱complete/abort/TERR/ALST释放、有界等待及CAN1 BusOff恢复；普通0x23无效值不污染位置缓存。未全面重构普通运动发送缓冲、FINISH实际到位、其他UART路由和多通道解析问题。完整决策、改动与最终构建/回归见[堵转保护与自动回退及校准方案第13～14节](E:/Dummy-code/docs/堵转保护与自动回退及校准方案.md)。五目标构建及离线检查通过，实机仍待用户烧录；未commit。
