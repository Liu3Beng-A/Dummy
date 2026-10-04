# 参数设置后整机冻结：命令任务栈分析

日期：2026-10-04。分析对象：工作区 HEAD `acb8a53`，基线 `c69430cb1e55900098b043e24747b39f73627bc1`。

## 结论与验证边界

**已确认一个会越过任务栈边界的代码缺陷：USB 命令任务只有 2000 字节栈，当前 ARM 工具链下，原版浮点命令解析路径仅七层栈帧就需要 2048 字节。** 这不是根据串口超时推测 CAN 故障，而是编译器栈报告与实际链接 libc 反汇编共同给出的数值。

栈越界发生在 `sscanf` 解析浮点数时，早于参数设置的 CAN 发送。它能够破坏相邻内存、FreeRTOS 堆块或任务状态，之后可能进入 HardFault/断言死循环。因此可以解释“不带 & 也冻结”“第一条尚能回应，后续写超时”和“RGB/OLED 一起停摆”。这些现象与该缺陷吻合，但**没有连接实机获取故障现场，也没有烧录回归，不能宣称所有症状已经解决或它是唯一缺陷**。

## 可复现的证据

主控实际构建目录是 `firmware_new/dummy-ref-core-fw/build/`，不是仓库根目录的 `build/`。后者的 CMakeCache 指向 `firmware/motor_fw_f103_35`。本轮产物均在前者。

当前工具链是 Arm GNU Toolchain 15.2.Rel1，C++ 使用 `-Ofast`。实际链接的是完整 `libc.a`。CMake 中虽然定义 `COMMON_FLAGS` 包含 `nano.specs`，但没有将这个变量加入编译/链接选项，不能据此认为实际使用 newlib-nano。

对未修改的 HEAD 命令处理源文件重新编译 `-fstack-usage`，并读取链接 ELF 的 libc 函数入口：

| 在浮点解析时同时存在的栈帧 | 字节 | 来源 |
|---|---:|---|
| `UsbServerTask` | 24 | GCC `.su` |
| `ASCII_protocol_parse_stream` | 40 | GCC `.su` |
| `ASCII_protocol_process_line` | 280 | GCC `.su`；含 257 字节命令副本 |
| `OnUsbAsciiCmd`（未修改 HEAD） | 632 | GCC `.su` |
| `sscanf` | 144 | ELF 入口：寄存器入栈 20 + 局部栈 124 |
| `__ssvfscanf_r` | 768 | ELF 入口：寄存器入栈 36 + 局部栈 732 |
| `_strtod_l` | 160 | ELF 入口：通用寄存器 36 + d8 8 + 局部栈 116 |
| **合计** | **2048** | **超过 2000 字节** |

已核对调用关系：process_line 保持局部命令副本在栈上并 `bl OnUsbAsciiCmd`；`sscanf` 调 `__ssvfscanf_r`；浮点分支调 `_strtod_r`，后者尾跳转到 `_strtod_l`。这不是把互斥路径的最大值相加。

该合计还没有包含更深层 libc 调用、异常压栈或 FreeRTOS 上下文保存，因此只是**下界**。即使修改代码后编译器重新安排局部变量、让这个合计略降到 2000 以下，也不能据此认为 2000 字节安全。

复现命令：

```powershell
# 在 firmware_new/dummy-ref-core-fw/build 中执行 ninja
# 然后回到仓库根目录：
python tools/analyze_command_stack.py
```

脚本只在主控 build/analysis 下生成临时源文件、对象、`.su` 和 `command_stack.json`。它不会连接或操作硬件。

## 为什么不能简单归因于 & 或 EEPROM

1. `c69430c` 的 `SetAcceleration` 设置 payload[4]=0；HEAD 的 `SetAcceleration_persist(v, false)` 也设置为 0。相同数值时，这两条路径的有效 CAN 内容相同。
2. 从基线到 HEAD 的已提交差异中，电机端没有改动。主控 `_persist` 方法主要增加了对第 5 字节保存标志的选择。
3. 保存开关改变了主控解析/响应函数与局部变量布局，不能用“main.cpp 没改、任务栈大小没改”来排除栈问题：栈需求会随调用链、编译器和 libc 改变。
4. 将基线 `OnUsbAsciiCmd` 源文件放到当前头文件/工具链下重新编译，得到 624 字节栈帧，对应同一路径下界 2040 字节。这说明基线也没有可靠余量；**这个重编结果不等同于历史烧录二进制的实际栈需求**。尚无完整历史二进制和实测对照，不能证明 & 改动独自引入了全部故障。
5. 当前电机端 `case 0x12/0x14` 在是否保存的判断之前已更新 `boardConfig.currentLimit/velocityAcc`；`&` 只决定是否置 `CONFIG_COMMIT`。因此应用后查询可以读到内存中的新值，原文档“查询只能读 EEPROM 旧镜像”的结论不符合当前源码。若应用后仍查到旧值，应继续核实实际烧录的电机固件和报文。

此前多轮更改 CAN 超时、重试、RGB DMA 等均未处理这条解析时的栈越界路径。阻塞在 `osSemaphoreAcquire` 的任务通常会让出 CPU，也不能仅凭两个任务等待 CAN 就断定独立的 RGB/OLED 会冻结。

## 本轮修复

- USB 和 UART 命令任务使用各 4096 字节、8 字节对齐的静态 SRAM 栈及静态 TCB；UART 也经过相同的浮点解析/格式化库。
- 不直接扩大 CCM 动态任务栈：FreeRTOS 的 64 KiB 堆位于 CCM，启动时还包含 45000 字节的 commTask。静态 SRAM 方案反而移除了这两个任务原先的动态堆分配。
- 启用 `configCHECK_FOR_STACK_OVERFLOW=2`。hook 保存 `stackOverflowTask`、`stackOverflowTaskName` 供调试器查看，然后停止；不在损坏栈上调用 printf。该检查在任务切换时执行，不能保证在每次越界写之前阻止它，也不是 MPU 隔离。
- USB/UART4 都新增 `#STACK`，返回自启动以来最小剩余栈字节数及当前 FreeRTOS 堆剩余量，例如 `ok STACK usb=1800 uart=2600 heap=5000`（示例，不是实测）。数字是剩余量，越小表示曾用得越深；若持续接近 0，需要继续扩大或缩短调用链。
- 本轮不更改 CAN、EEPROM、USB 发送、RGB DMA、运动逻辑和上位机，以减少实机验证变量。

加入诊断分支后 GCC 改变了函数布局，修复版 `OnUsbAsciiCmd` 栈帧为 560 字节，当前七层下界为 1976 字节；本轮仍使用 4096 字节栈，不依赖偶然的布局缩减。

## 验证结果与下一步

- 主控 Ninja 构建 28/28 通过，仅原有 RWX LOAD segment 警告。
- FLASH 379360 B（36.18%），SRAM 55272 B（42.17%），CCM 静态堆仍为 64 KiB。
- `tools/analyze_command_stack.py` 通过：记录 HEAD 下界 2048、基线处理函数当前重编下界 2040、修复版下界 1976、已配置 USB 栈 4096。
- 无烧录、无实机验证、无 git commit。

请烧录 **`firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`** 到主控，电机端本轮不需更新。建议在同一次上电内：

1. 发 `#STACK` 保存起始结果。
2. 依次重复 `#ACC_J 1 150.0`、`#I_LIMIT_J 1 1.5`、`#ACC_J 1 150.0 &`、`#I_LIMIT_J 1 1.5 &`。这些值沿用现有复现记录；带 & 的少量验证即可，不需要为压力测试反复写 Flash。
3. 每组后发 `#STACK`，观察最小余量；同时观察 RGB/OLED、串口以及 `#GETJACC 1`、`#GETI 1` 是否还能响应。
4. 若仍冻结且有调试器，暂停后检查是否位于 `vApplicationStackOverflowHook` 或 HardFault，读取 `stackOverflowTaskName`；否则先反馈“哪条命令、有没有先收到回复、最后一次 #STACK 数字”。这比继续同时试改多个方向更能区分剩余问题。

CAN 中止/错误释放、共享报文缓冲、Respond 超长响应边界等仍有独立风险；本轮没有将这些风险未经实测地称为这次冻结的已确认根因。
