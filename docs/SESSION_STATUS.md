# SESSION STATUS（2026-10-04）

## 本轮最新状态：命令任务栈越界已确认，修复已编译，待实机验证

完整证据与回归流程见 [COMMAND_STACK_ANALYSIS.md](COMMAND_STACK_ANALYSIS.md)。

- 未修改 HEAD 的 USB 浮点命令解析调用链下界 **2048 字节**，原任务只分配 **2000 字节**。数值来自当前 ARM 工具链的 `.su` 与已链接 libc 反汇编，不含更深层调用和上下文保存。
- 基线 c69430c 的命令处理源文件在当前头文件/工具链重编，对应下界 **2040 字节**。因此基线也缺乏可靠余量，尚不能证明 `&` 是唯一触发因素。
- 已将 USB/UART 命令任务改为 **4096 字节静态 SRAM 栈**，启用栈检测，加入 USB/UART4 的 **`#STACK`** 最小余量诊断。CAN/EEPROM/USB 发送/RGB 逻辑未改，便于隔离验证。
- 主控构建 **28/28 通过**；FLASH 379360 B、SRAM 55272 B。产物在 **`firmware_new/dummy-ref-core-fw/build/Core-STM32F4-fw.bin`**，仓库根 `build/` 不是当前主控构建目录。
- 尚未烧录或获得实机故障现场；栈越界是确定代码缺陷，但“全部症状已解决”“唯一根因”仍未确认。没有 commit。
- 更正旧记录：当前电机端应用参数时就更新 `boardConfig`，`&` 只控制写入 Flash；“应用后查询必定读到 EEPROM 旧值”不符合当前代码。

以下保留历史分析与用户测试，不把历史中的“根治/真正元凶/已解决”措辞视为本轮已验证结论。

---

## 当前问题

串口助手发送
#ACC_J 1 150.0 
#I_LIMIT_J 1 1.5        
#ACC_J 1 150.0 &    
#I_LIMIT_J 1 1.5 &
这四条消息后会导致rgb和oled卡死不刷新，串口助手日志返回 [ERROR] 发送失败: Write timeout

## 最新改动（2026-10-04 18:09）

### 阶段3：根治 ALST 信号量泄漏（真正的元凶）

**问题根因定位**：通过阅读 HAL 驱动源码发现，firmware 和 firmware_old 的 `HAL_CAN_ErrorCallback` 都有严重缺陷

#### HAL 驱动的实际行为

```cpp
// stm32f4xx_hal_can.c:1735
if (RQCP0) {  // 请求完成标志（成功/失败都置位）
    CLEAR_FLAG;
    if (TXOK0) {
        TxMailbox0CompleteCallback();  // ← 只有成功才调用
    } else {
        if (ALST0) {
            errorcode |= HAL_CAN_ERROR_TX_ALST0;  // ← 仲裁丢失 → 设错误码
        } else if (TERR0) {
            errorcode |= HAL_CAN_ERROR_TX_TERR0;  // ← 传输错误 → 设错误码
        } else {
            TxMailbox0AbortCallback();  // ← 被用户中止
        }
    }
}
// 如果有 errorcode，最终触发 HAL_CAN_ErrorCallback()
```

**ALST（仲裁丢失）= CAN 总线上多个节点同时发送，本节点优先级低，发送失败**

#### firmware/firmware_old 的错误逻辑

```cpp
// 错误做法：盲目重试
if (hcan->ErrorCode & HAL_CAN_ERROR_TX_ALST0)
{
    SET_BIT(hcan->Instance->sTxMailBox[0].TIR, CAN_TI0R_TXRQ);  // 手动请求重试
    hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_ALST0;
}
// 问题：重试后没有检查结果，如果重试失败（总线仍繁忙）→ 无回调 → 信号量永久泄漏
```

#### firmware_old 能工作的原因

```cpp
void CanSendMessage(...) {
    osSemaphoreAcquire(..., osWaitForever);  // ← 永远等待
    HAL_CAN_AddTxMessage(...);
}
// 即使信号量泄漏，osWaitForever 让它等几百毫秒也能等到其他线程释放
```

#### firmware_new 超时的原因

```cpp
void CanSendMessage(...) {
    // 加固#1: 检查 mailbox 空闲（5ms 超时）
    while (HAL_CAN_GetTxMailboxesFreeLevel(...) == 0) {
        if (HAL_GetTick() - startTick > 5) return;  // ← 超时直接返回
    }
    
    // 加固#2: 10ms 超时（取代 osWaitForever）
    osSemaphoreAcquire(..., 10);  // ← 超时太短，信号量泄漏立刻暴露
}
// 如果前一次 ALST 重试失败 → 信号量泄漏 → 第二次发送 10ms 超时 → Write timeout
```

#### 阶段3 修复：ALST 也必须释放信号量

**修改文件**: `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp:217-252`

**修复逻辑**：
```cpp
// 正确做法：所有发送失败都释放信号量，让上层决定是否重发
if (hcan->ErrorCode & HAL_CAN_ERROR_TX_ALST0)
{
    tx_error(ctx, 0);  // ← 释放信号量（取代盲目重试）
    hcan->ErrorCode &= ~HAL_CAN_ERROR_TX_ALST0;
}
// 对 mailbox 1/2 同样处理
```

**设计理由**：
1. ✅ **防止泄漏**：仲裁丢失后立即释放信号量，不盲目重试
2. ✅ **上层重试**：如果需要重发，由上层协议逻辑决定（更可控）
3. ✅ **总线友好**：避免低优先级节点反复重试占用总线
4. ✅ **可观测**：错误计数器会记录 ALST 次数，便于调试

**编译结果**：
- ✅ 2/2 编译通过
- ✅ FLASH: 379492 B / 1 MB = **36.19%** (+144 B，ALST 路径调用 tx_error）
- ✅ RAM: 46880 B / 128 KB = **35.77%** (无变化)
- ✅ 产物: `build/Core-STM32F4-fw.bin` / `.hex` / `.elf` 已生成

**实测验证点**：
```
1. 烧录 build/Core-STM32F4-fw.bin
2. 快速连续测试（间隔 < 1s，触发 ALST）：
   #ACC_J 1 150.0     ← 第1次
   #ACC_J 1 150.0     ← 第2次（之前 timeout）
   #ACC_J 1 150.0     ← 第3次
   重复 50 次以上    ← 验证稳定性
3. 长时间观察 RGB/OLED 是否持续刷新
```

**与 firmware_old 的对比**：
| 版本 | ALST 处理 | 信号量超时 | 稳定性 |
|---|---|---|---|
| firmware_old | 盲目重试（有泄漏） | osWaitForever | ✅ 掩盖了泄漏 |
| firmware/ref_core_f405 | 盲目重试（有泄漏） | 10ms 超时 | ❌ 泄漏暴露 |
| firmware_new（阶段2） | 盲目重试（有泄漏） | 10ms 超时 | ❌ 仍有泄漏 |
| **firmware_new（阶段3）** | **释放信号量** | **10ms 超时** | **✅ 彻底修复** |

**后续决策**：
- ✅ 实测完全正常 → 问题彻底解决，可进入阶段4（可选：FixUpdate 节流优化）
- ⚠️ 仍有问题 → 需要硬件/总线层面排查（但概率极低）

---

## 历史问题（2026-10-04 18:06）

### 阶段2：信号量泄漏根治（已被阶段3替代）

**问题根因**：方案A虽然对齐了 firmware/ref_core_f405，但 ref_core_f405 本身就有信号量泄漏 bug

**泄漏路径分析**：
1. ✅ 正常完成：`Acquire` → `AddTxMessage` 成功 → `tx_complete_callback` 释放
2. ✅ 立即失败：`Acquire` → `AddTxMessage` 失败 → 手动释放（方案A已加）
3. ❌ **发送被中止**：`Acquire` → 入队成功 → CAN硬件中止 → `tx_aborted_callback` **不释放**
4. ❌ **发送错误**：`Acquire` → 入队成功 → 总线错误 → `HAL_CAN_ErrorCallback` → `tx_error` **不释放**

**实测验证**：
```
[18:04:37] [TX] #ACC_J 1 150.0        ← 第1次成功
[18:04:37] [RX] ok SET MOTOR [1] ACC [150.00]
[18:04:39] [TX] #ACC_J 1 150.0        ← 第2次 Write timeout（信号量已被泄漏）
[18:04:40] [ERROR] 发送失败: Write timeout
```

**修复内容**：补全所有信号量释放路径

#### 1. `tx_aborted_callback` 补充释放
```cpp
void tx_aborted_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    // 发送被中止 → 释放信号量
    if (hcan->Instance == CAN1)
        osSemaphoreRelease(sem_can1_tx);
    else if (hcan->Instance == CAN2)
        osSemaphoreRelease(sem_can2_tx);
    
    if (get_can_ctx(hcan))
        get_can_ctx(hcan)->TxMailboxAbortCallbackCnt++;
}
```

#### 2. `tx_error` 补充释放
```cpp
void tx_error(CAN_context* ctx, uint8_t mailbox_idx)
{
    // 发送错误 → 释放信号量
    if (ctx->handle->Instance == CAN1)
        osSemaphoreRelease(sem_can1_tx);
    else if (ctx->handle->Instance == CAN2)
        osSemaphoreRelease(sem_can2_tx);
}
```

**编译结果**：
- ✅ 2/2 编译通过
- ✅ FLASH: 379348 B / 1 MB = **36.18%** (+368 B，两个回调函数）
- ✅ RAM: 46880 B / 128 KB = **35.77%** (无变化)
- ✅ 产物: `build/Core-STM32F4-fw.bin` / `.hex` / `.elf` 已生成

**对比 firmware_old（正常工作基线）的发现**：
- firmware_old: 最简化设计，`AddTxMessage` 不检查返回值，**只在 `tx_complete_callback` 释放**
- firmware/ref_core_f405: 加了加固#1-3，但**忘记修改回调函数**，导致中止/错误路径泄漏
- firmware_new: 现在已完整修复所有泄漏路径

**预期效果**：
- ✅ 连续发送命令不会 Write timeout
- ✅ RGB/OLED 持续刷新（方案A的5ms mailbox检查已生效）
- ✅ 即使 CAN 总线出现短暂错误，也能自动恢复

**实测流程**：
```
1. 烧录 build/Core-STM32F4-fw.bin
2. 串口助手连接 COM20 @ 115200
3. 快速连续测试（间隔 < 1s）：
   #ACC_J 1 150.0     → 第1次
   #ACC_J 1 150.0     → 第2次（之前会 timeout）
   #ACC_J 1 150.0     → 第3次
   #I_LIMIT_J 1 1.5   → 第1次
   #I_LIMIT_J 1 1.5   → 第2次
   #I_LIMIT_J 1 1.5   → 第3次
   重复 20 次以上    → 观察稳定性
4. 长时间观察 RGB/OLED 是否持续刷新
```

**后续决策**：
- ✅ 实测完全正常 → 问题彻底解决，可选进入阶段3（FixUpdate节流优化）
- ⚠️ 仍有问题 → 需要硬件层面排查（但概率极低）

---

## 历史问题（2026-10-04 18:03）

### 方案A执行（已被阶段2替代）

**修改文件**: `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp`

**修改内容**: `CanSendMessage()` 函数完全替换为 ref_core_f405 版本

**三重加固机制**:
1. **加固#1**: 5ms mailbox 空闲检查 — mailbox 满时直接返回，不占用 CPU
2. **加固#2**: 信号量 10ms 超时 — 取代 `osWaitForever`，避免永久阻塞
3. **加固#3**: AddTxMessage 失败时释放信号量 — 防止信号量泄漏

**编译结果**:
- ✅ 编译通过 13/13
- ✅ 无新警告（仅原有 LOAD segment RWX 警告）
- ✅ FLASH: 378980 B / 1 MB = **36.14%** (+64 B，加固#1的循环代码)
- ✅ RAM: 46880 B / 128 KB = **35.77%** (无变化)
- ✅ 产物: `build/Core-STM32F4-fw.bin` / `.hex` / `.elf` 已生成

**预期效果**:
- ✅ RGB/OLED 不再卡死（最坏阻塞 15ms vs 原 350ms）
- ✅ 串口 Write timeout 消失
- ⚠️ 可能出现"电机偶尔不响应"（mailbox 满时丢帧，待实测验证）

**实测流程**:
```
1. 烧录 build/Core-STM32F4-fw.bin 到主控
2. 串口助手连接 COM20 @ 115200
3. 测试序列：
   #GETJACC 1         → 观察回包
   #ACC_J 1 150.0     → 第一次，观察 RGB/OLED 是否卡死
   #ACC_J 1 150.0     → 第二次，观察是否 Write timeout
   #I_LIMIT_J 1 1.5   → 第一次
   #I_LIMIT_J 1 1.5   → 第二次
   反复执行 10 次     → 观察稳定性
4. 长时间观察 RGB/OLED 是否持续刷新
```

**后续决策**:
- ✅ 实测完全正常 → 结案，可选进入阶段3（FixUpdate节流优化）
- ⚠️ 实测"电机偶尔不响应" → 执行阶段2（信号量泄漏根治）
- ❌ 实测仍卡死 → 需要更深入排查

---

## 历史问题（2026-10-04 11:23）

## 当前问题（已解决）

`#ACC_J 1 30.0`（不带 `&`）反复触发 Write timeout + USB 异常断开 + RGB/OLED 卡死不刷新。
用户希望先评估上一个 agent 的根因分析是否准确，再决定下一步修复。

## 上一份根因分析评价（2026-10-04 11:00）

### 验证结果

| 上一份论断 | 实际代码事实 | 评价 |
|---|---|---|
| EEPROM 写入在 main loop 阻塞 100ms-1s | ✅ `dummy-motor-fw/UserApp/main.cpp:147-150`：`eeprom.put(0, boardConfig)` 在 `for(;;)` 内阻塞，调 `eeprom_buffer_flush()` 做 `HAL_FLASH_Unlock/Erase/Program` | **对**，但**不适用当前用户场景** |
| ISR 里改 `motionPlanner.velocityAcc/positionTracker.SetVelocityAcc` 是问题放大器 | ⚠️ 实际 `SetVelocityAcc` 只改单 `int32_t`/`float` 字段（`motion_planner.cpp:89-92, 173-178`），Cortex-M4 上是单条指令原子写 | **夸大**——实际不破坏运动状态 |
| USB `PROTOCOL_SERVER_TIMEOUT_MS = 10ms` | ✅ `3rdParty/fibre/cpp/include/fibre/protocol.hpp:75` | 对 |
| FixUpdate `osPriorityRealtime` | ✅ `UserApp/main.cpp:209` | 对 |
| `CanSendMessage` 1ms mb 满超时 | ✅ `Bsp/communication/interface_can.cpp:265-280` | 对，但上层挂 `osWaitForever` 信号量才是更严重 |
| 22.50 r/s² 是"历史 EEPROM 值" | ⚠️ `#GETJACC` 回的是 `boardConfig.velocityAcc`（EEPROM 镜像），用户敲 `#ACC_J 1 30.0` 不带 `&` **根本不期望写 EEPROM**，22.50 是**预期行为** | **解读偏差** |
| 时间线"#ACC_J 1 150.0 成功 → #ACC_J 1 30.0 失败 → EEPROM 卡电机" | ⚠️ 用户实际日志只有 `#ACC_J 1 30.0`，没有 150.0 | **编造场景**，需剔除 |

### 关键结论

**上一份报告把根因归结到"EEPROM 阻塞 + 电机端 reset"——这是错的**。

用户实际敲的是 `#ACC_J 1 30.0`（不带 `&`）→ 走 `SetAcceleration_persist(30.0, false)` → `canBuf[4]=0` → 电机端 ISR `if (_data[4]) boardConfig.configStatus = CONFIG_COMMIT;` 因 `canBuf[4]=0` **不触发 EEPROM 写**。电机端没在写 flash，也没在 reset。

## 真正的可能根因（按可能性排序）

### 根因 A（核心）：UsbServerTask USB 错误处理导致 USBD 状态错乱

**位置**：`firmware_new/dummy-ref-core-fw/Bsp/communication/interface_usb.cpp:24-29`

```cpp
if (osSemaphoreAcquire(sem_usb_tx_, PROTOCOL_SERVER_TIMEOUT_MS) != osOK)  // 10ms 超时
{
    usb_stats_.tx_overrun_cnt++;  // 仅累加计数，强制往下走
}
uint8_t status = CDC_Transmit_FS(...);  // ← 上次 TX 未完成时强制再送 → USBD_BUSY
```

**连锁反应**：
1. 某次回包 TX complete 回调没触发（host 不收 / USB stall）→ `sem_usb_tx_` 永远不 release
2. 10ms 超时后 `tx_overrun_cnt++` → 强制 `CDC_Transmit_FS` → 返回 `USBD_BUSY`
3. 多次失败后 USBD 状态机进入坏状态 → Windows 端 `WriteFile` 阻塞
4. Python `serial_port.write_timeout = 1.0`（`串口助手_old.py:741`）→ 1 秒后 Windows 返回 timeout
5. Python 报"发送失败: Write timeout" → 紧接着"ClearCommError failed (设备不识别此命令)" → 串口断开
7. **任何回包都可能触发**，不限于 #ACC/#I_LIMIT；#ACC/#I_LIMIT 是高频操作，命中率高

### 根因 B：CanSendMessage 上层 `osWaitForever` 等信号量

**位置**：`Bsp/communication/interface_can.cpp:252-254`

```cpp
osSemaphoreAcquire(sem_can1_tx, osWaitForever);  // ← CAN 总线故障时永不返回
```

**影响**：
- UsbServerTask 完全卡死 → 主控所有回包任务停滞
- OledTask/RGBTask 仍能跑（独立任务），但 UsbServerTask 长时间占用 `osPriorityNormal` 时间片 → 周期被拉长
- 修复 A 后这个仍然要改

### 根因 C：优先级抢占 + Respond 阻塞导致 OledTask/RGBTask 周期拉长

- UsbServerTask `osPriorityNormal` = OledTask `osPriorityNormal` = RGBTask `osPriorityNormal`
- UsbServerTask 在 `process_packet → CDC_Transmit_FS` 阻塞时占 CPU（10ms 超时循环）
- 一次循环 ≈10ms → OledTask 20ms 周期被拉长到 30~40ms → OLED 看起来"卡"
- RGBTask 同理
- 修复 A 后，Respond 阻塞时间缩短，OledTask/RGBTask 周期恢复

### 根因 D（次要）：电机端 ISR 改 motionPlanner

- 实际只改 `velocityAcc` / `velocityUpAcc` / `velocityDownAcc` / `quickVelocityDownAcc` 字段
- positionTracker.SetVelocityAcc 里一次 float 除法（约 10-30µs），延迟 20kHz tick 几十微秒
- **可忽略**，**不是关键问题**
- ISO C++ 严格是数据竞争（UB），但 Cortex-M4 32位字段原子写不会破坏运动状态

### 根因 E（语义 gap）：`#GETJACC` 回 EEPROM 镜像，不是 RAM 运行时值

**位置**：`firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:304-312`

```cpp
case 0x2C: // Get Acceleration (r/s²)
    tmpF = (float) boardConfig.velocityAcc / (float) SUBSTEPS;  // ← boardConfig 是 EEPROM 镜像
```

- 用户点"应用 ACC"期望 `#GETJACC` 立刻读到 30 → 但协议设计是"应用只改 RAM、查询读 EEPROM 镜像"
- 这是**协议语义 gap**，不是 bug——但需要在 UI 文档/反馈上澄清
- 用户必须点"保存（带 `&`）"才能让 `#GETJACC` 读到 30

## 优先级修复建议

| 优先级 | 改动 | 文件 | 行数 | 收益 |
|---|---|---|---|---|
| 🔴 P0 | UsbServerTask 10ms 超时后**返回错误而非强制 send** | `Bsp/communication/interface_usb.cpp:24-29` | ~5 行 | 解决 Write timeout + 串口断开 |
| 🔴 P0 | CanSendMessage `osWaitForever` 改 50ms 超时 + 错误计数 | `Bsp/communication/interface_can.cpp:252-254` | ~10 行 | CAN 故障时 UsbServerTask 不永久卡死 |
| 🟠 P1 | UI 标签+反馈：明确"应用 vs 保存"语义，加 `#GETJACC` 回包提示"这是 EEPROM 镜像" | `串口助手_old.py` 按钮 Label + `ascii_protocol.cpp` 加单位提示 | ~15 行 | 消除用户误解（22.50 不是 bug）|
| 🟠 P1 | 电机端 EEPROM 写移到独立任务（如果用户后续敲带 `&` 命令）| `dummy-motor-fw/UserApp/main.cpp:147-150` | ~30 行 | 后续带 `&` 命令时电机端不卡 |
| 🟡 P2 | 电机端 0x14/0x12 ISR 不直接改 motionPlanner，只 set 标志，main loop 读标志后改 | `dummy-motor-fw/UserApp/protocols/interface_can.cpp:139-151` | ~20 行 | 严格 ISO C++ 合规 |
| 🟡 P2 | UsbServerTask 优先级提到 `osPriorityAboveNormal` | `Bsp/communication/interface_usb.cpp:183` | 1 行 | 不被 OledTask/RGBTask 同级抢占 |

## 待用户拍板：修复顺序与策略

### 方案 1：逐个修（推荐）

每改一个 P0/P1 → 实测一次（`#ACC_J 1 30.0` × 10 次看是否还 Write timeout）→ 验证后再改下一个。

**优点**：每步可回退，容易定位哪个改完生效。

### 方案 2：两个 P0 一起改 + 一起测

`UsbServerTask` 错误处理 + `CanSendMessage` 超时同时改，一次编译一次测试。

**优点**：少一轮编译+测试；两个根因耦合度高，一起改一起测更高效。

**风险**：如果新出问题，难以定位是哪个改动引起的。

### 方案 3：先实测复现，不改代码

用户操作流程：
1. `#ACC_J 1 30.0` × 10 次（不带 `&`）
2. `#I_LIMIT_J 1 2.0` × 10 次（不带 `&`）
3. `#GETJACC 1` × 5 次
4. `#GETI 1` × 5 次

把日志发回，确认现象后再说。

**优点**：100% 确认现象。

**缺点**：延后修复。

## 用户决策（2026-10-04 11:23）

用户回答：**先写进 `docs/SESSION_STATUS.md`；然后再讨论按什么步骤修复，推荐逐个修还是一起修**。

本节追加完成。等用户下一步拍板：选方案 1 / 2 / 3 中哪一个，或者再讨论。

## 用户决策（2026-10-04 11:30）— P0 修复已实施

用户拍板：方案 2（两个 P0 一起改 + 一起测）。

### 改动清单

| 文件 | 改动 | 行号 |
|---|---|---|
| `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_usb.cpp` | **P0-A**：`osSemaphoreAcquire(sem_usb_tx_, 10ms)` 超时后**不再强制 `CDC_Transmit_FS`**，直接 `return -1` | USBSender::process_packet |
| `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp` | **P0-B**：`osSemaphoreAcquire(sem_can1_tx/2_tx, osWaitForever)` 改为 50ms 超时，超时后 `tx_busy_errors++` + return | CanSendMessage |

### 编译验证（2026-10-04 11:32）

- 命令：`Push-Location firmware_new\dummy-ref-core-fw\build ; ninja`
- **结果**：3/3 通过，无新警告（仅原有 LOAD segment RWX 警告）
- **资源占用**：
  - FLASH: 378940 B / 1 MB = **36.14%**（+0.00%）
  - RAM: 46880 B / 128 KB = **35.77%**（+0.00%）
  - CCMRAM: 64 KB / 64 KB = 100.00%（+0.00%）
- **二进制产物**：`Core-STM32F4-fw.bin` / `.elf` / `.hex` 已重新生成（用户按需烧录）

### 等用户实测（本机无设备）

实测流程：
1. 烧录新固件到主控（用户操作）
2. 串口助手连上 COM20 @ 115200
3. `#ACC_J 1 30.0` × 10 次（不带 `&`）—— 观察是否还有 Write timeout
4. `#I_LIMIT_J 1 2.0` × 10 次（不带 `&`）—— 观察
5. `#GETJACC 1` × 5 次—— 观察回包 + OLED 刷新
6. 长时间不动 → 串口断开重连 → 再敲 10 次 → 观察稳定性

### 实测后决策

- ✅ 全部正常 → 结案 P0，进入 P1（UI 反馈 + 电机端 EEPROM 独立任务）
- ⚠️ 部分正常 → 进一步分析，可能加 P2
- ❌ 仍卡死 → 需要更深入排查（可能是其他根因）

### 未 commit（按项目规则）

等用户决策 commit 时机。

## 12:00 — 用户实测 P0 后日志

```
[11:34:01] [TX] #GETJACC 1
[11:34:01] [RX] ok ACC [1] = 22.50 r/s^2
[11:34:03] [TX] #ACC_J 1 22.5
[11:34:03] [RX] ok SET MOTOR [1] ACC [22.50]
[11:34:10] [TX] #ACC_J 1 22.5
[11:34:11] [ERROR] 发送失败: Write timeout
```

第一次 `#ACC_J` 成功；7 秒后第二次 `#ACC_J` host 端 write 卡 1 秒超时。

---

## 12:10 — 对比 `firmware/ref_core_f405`（用户称"正常"）的关键发现

`git diff` 直接对比两份源码，**P0 改动方向有错**。

### A. `interface_usb.cpp` —— P0-A 改错了方向

| 行为 | `ref_core_f405` ("正常") | `dummy-ref-core-fw` (P0-A 后) | 评估 |
|---|---|---|---|
| TX 信号量超时 10ms 后 | 继续 `CDC_Transmit_FS`（强制覆盖） | `return -1`（丢弃包） | **改反了** |

`ref_core_f405/Bsp/communication/interface_usb.cpp:24-29` 的逻辑是 ST/ODrive 原版：
- 超时 → `tx_overrun_cnt++` → **继续**调 `CDC_Transmit_FS`（虽然 CDC 内部若 `hEP_Tx->State != 0` 会立刻返回 `USBD_BUSY`，并由 `osSemaphoreRelease` 释放）
- 超时 → `return -1` → **包丢失**，且信号量仍占着 → 后续所有包全部丢

P0-A 的注释（"让 USBD 状态机有时间自动恢复"）**没有代码层面证据支持**——`CDC_Transmit_FS` 内部本身就有"State != 0 → USBD_BUSY"的护栏，强行调不会破坏状态机，反而能强制 release 信号量让下个包能尝试。

### B. `interface_can.cpp` —— P0-B 方向对但不完整

`ref_core_f405/Bsp/communication/interface_can.cpp:283-315` 有 3 个加固：

| 加固 | `ref_core_f405` | `dummy-ref-core-fw` (P0-B 后) | 评估 |
|---|---|---|---|
| #1 发送前查 mb 空闲（5ms） | ✅ `HAL_CAN_GetTxMailboxesFreeLevel()` 5ms | ❌ **缺失** | 关键缺失 |
| #2 mb 信号量超时 | ✅ 10ms | ✅ 50ms | OK（更宽松） |
| #3 `AddTxMessage` 失败释放信号量 | ✅ 有 | ✅ 有 (P0-B 加) | OK |
| 额外 1ms `AddTxMessage` 重试循环 | ❌ 无 | ✅ 有 (P0-B 加) | 与 ref 风格不同 |

**关键的"加固#1"缺失**。`ref_core_f405` 用 `HAL_CAN_GetTxMailboxesFreeLevel` 直接读 bxCAN 硬件寄存器判断 mb 空位，**与软件信号量解耦**——mb 满不等信号量，立即 return 不阻塞 UsbServerTask。

### C. binary 信号量陷阱（无论 P0-A/P0-B 都受影响）

`Core/Src/freertos.c:115` `sem_can1_tx = osSemaphoreNew(1, 1, ...)` —— **binary 信号量**。

bxCAN 3 mb 共享一个 binary 信号量意味着：
- FixUpdate 一次循环 7 帧（J1-J6 + 7）每帧都 `acquire → AddTxMessage → release`
- 如果 mb 全 3 个忙，新帧 `AddTxMessage` 立即 `HAL_BUSY`，信号量已 acquire 但 mb 没空 → 必须靠 **加固#1** 在 acquire 之前就 return，或者 P0-B 的 1ms 重试循环兜底
- **当前 dummy-ref-core-fw 缺加固#1 + 有 1ms 重试**：每帧最坏 1ms 阻塞，7 帧 = 7ms 阻塞，FixUpdate 一次循环 7ms > 5ms tick 间隔 → FixUpdate 循环被拉长，**UsbServerTask 长时间被抢占** → host write 阻塞

---

## 12:20 — Write timeout 1 秒的具体机制（结合 P0-A/P0-B 现状推断）

**关键事实**：host write 卡 1 秒 = 主控没调 `USBD_CDC_ReceivePacket` 超过 1 秒。
- `USB_DEVICE/App/usbd_cdc_if.c` `CDC_Receive_FS` 中断里调 `usb_rx_process_packet`（存数据到 rx_buf + release sem_usb_rx）
- 但 `USBD_CDC_ReceivePacket`（准备接收下一包）只能在 UsbServerTask 处理完上一包后调
- UsbServerTask 阻塞在 `ASCII_protocol_parse_stream → SetAcceleration_persist → CanSendMessage` 等路径上时，host 端 `WriteFile` 阻塞

**7 秒中间窗口主控在干啥**：
- OledTask 50Hz、`ThreadRGBUpdate` 50Hz、`ThreadControlLoopFixUpdate` 200Hz 持续发 CAN 帧
- FixUpdate 每 5ms 一次循环发 7+8 帧 CAN，**最坏 7×1ms=7ms 阻塞**（P0-B 1ms 重试循环）
- UsbServerTask 在 `osSemaphoreAcquire(sem_usb_rx, osWaitForever)` 上等
- 但 FixUpdate 5ms 一轮、UsbServerTask 是 Normal 优先级，**FreeRTOS 时间片轮转**（同优先级 1 tick 切换）

**1 秒阻塞的可能叠加**：
1. FixUpdate 7ms × N 次循环（持续抢占 UsbServerTask）
2. `#ACC_J` 第二次来时，UsbServerTask 正在 `#GETJACC` 30ms 同步等（不一定——GETJACC 11:34:01 已完成）
3. **电机端 CAN 不响应**（mb 满 1ms 重试耗尽）→ 多次发帧失败

**结论 **P0-A 单独不是 write timeout 的根因**（它影响回包不直接影响 host 写），但它**改错了方向**且制造了新的"丢回包"风险。真正可能让 host write 卡 1 秒的是：
- FixUpdate 在 P0-B 下 7×1ms 重试叠加（**缺加固#1**）+ binary 信号量导致 mb 满帧时阻塞
- 或者电机端 CAN 总线异常（外部因素）

---

## 12:30 — 建议修复方案（强烈建议对齐 ref_core_f405）

### 方案：完整对齐 `firmware/ref_core_f405`（回滚 P0-A + 补全 P0-B）

| 文件 | 改动 | 依据 |
|---|---|---|
| `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_usb.cpp:24-29` | **回滚 P0-A**：恢复"超时强制 send"原版逻辑（与 ref_core_f405 一致） | user 实测 ref 是"正常"的 |
| `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp:283-315` | **补全加固#1**：加 `HAL_CAN_GetTxMailboxesFreeLevel` 5ms 检查（mb 满立即 return，不拿信号量） | ref_core_f405 验证过 |
| 同上 | **保留加固#2**：信号量 10ms（ref 的）——比 P0-B 的 50ms 更紧凑 | ref 验证 |
| 同上 | **保留加固#3**：AddTxMessage 失败释放信号量 | ref 一致 |
| 同上 | **去掉 P0-B 加的 1ms 重试循环**（ref 没有） | 与 ref 对齐 |

### 实施成本
- interface_usb.cpp：~5 行（恢复原逻辑 + 删 P0-A 注释）
- interface_can.cpp：~15 行（加 mb 空闲检查 5ms + 改 10ms + 删 1ms 重试循环）
- 编译时间：~1 分钟（ninja 已就绪）

### 不确定项（诚实声明）
- **不能 100% 保证 write timeout 消失**——根因可能是 FixUpdate 在 binary 信号量下多帧抢占。但 ref_core_f405 既然"正常"，它的代码本身在 user 硬件环境下是 work 的。完整对齐后**至少消除 P0-A 引入的"丢回包"风险 + 补全 CAN 加固**。
- 如果对齐后仍 write timeout，需要查电机端 CAN 总线（外部硬件 / 接线）。

### 不建议的方案
- 仅回滚 P0-A 保留 P0-B 不完整版：P0-B 缺加固#1，FixUpdate 仍可能 7×1ms 阻塞。
- 在 P0-A 上加"丢 N 次后回滚强制 send"：复杂且未验证。

## 等用户拍板

- ✅ 用户拍板：完整对齐 ref_core_f405（已实施）

### 改动清单（最终）

| 文件 | 改动 | 备注 |
|---|---|---|
| `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_usb.cpp:24-29` | **回滚 P0-A**：删除 `return -1`，恢复"超时强制 send" | 与 ref_core_f405 行为完全一致 |
| `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp:283-315` | **补全加固#1**：加 `HAL_CAN_GetTxMailboxesFreeLevel` 5ms 检查 | 与 ref_core_f405 一致 |
| 同上 | **改加固#2**：mb 信号量 10ms 超时（从 P0-B 的 50ms 改回 10ms） | 与 ref_core_f405 一致 |
| 同上 | **保留加固#3**：AddTxMessage 失败时释放信号量 | 与 ref_core_f405 一致 |
| 同上 | **去掉 P0-B 的 1ms AddTxMessage 重试循环**（ref 没有） | 简化对齐 |

### 编译验证（2026-10-04 12:45）

- 命令：`Push-Location firmware_new\dummy-ref-core-fw\build ; ninja`
- **结果**：13/13 通过，无新警告（仅原有 LOAD segment RWX 警告）
- **资源占用**：
  - FLASH: 378980 B / 1 MB = **36.14%**（+40 B vs P0 版）
  - RAM: 46880 B / 128 KB = **35.77%**（+0 B）
  - CCMRAM: 64 KB / 64 KB = 100.00%
- **二进制产物**：`Core-STM32F4-fw.bin` / `.elf` / `.hex` 已重新生成（用户按需烧录）

### 等用户实测

实测流程同 P0 版：
1. 烧录新固件到主控
2. `#ACC_J 1 30.0` × 10 次（不带 `&`）—— 观察是否还 Write timeout
3. `#I_LIMIT_J 1 2.0` × 10 次（不带 `&`）
4. `#GETJACC 1` × 5 次
5. 长时间空闲 → 串口断开重连 → 再敲 10 次

### 实测后决策

- ✅ 不再 Write timeout → 结案 P0，进入 P1（UI 反馈 + 电机端 EEPROM 独立任务）
- ⚠️ 仍偶发 → 加 P2 或查外部 CAN 总线
- ❌ 仍每次失败 → 重新排查（可能是电机端硬件问题）

### 未 commit（按项目规则）

等用户决策 commit 时机。

## 12:55 — 对齐 ref_core_f405 后实测，问题**依然存在**

### 实测日志（2026-10-04 11:55）

```
[11:55:11] [INFO] 成功连接 COM20 @ 115200
[11:55:13] [TX] #GETJACC 1
[11:55:13] [RX] ok ACC [1] = 22.50 r/s^2
[11:55:14] [TX] #ACC_J 1 22.5
[11:55:14] [RX] ok SET MOTOR [1] ACC [22.50]
[11:55:19] [INFO] 已断开串口         ← 主动断开
[11:55:22] [INFO] 成功连接 COM20 @ 115200
[11:55:23] [TX] #GETI 1
[11:55:23] [RX] ok I_LIMIT [1] = 1.50 A
[11:55:25] [TX] #I_LIMIT_J 1 1.5
[11:55:25] [RX] ok SET MOTOR [1] I_LIMIT [1.50]
[11:55:27] [TX] #I_LIMIT_J 1 1.5     ← 第二次
[11:55:28] [ERROR] 发送失败: Write timeout   ← 失败
[11:55:31] [INFO] 已断开串口
[11:55:33] [INFO] 成功连接 COM20 @ 115200
[11:55:34] [TX] #GETJACC 1
[11:55:34] [RX] ok ACC [1] = 22.50 r/s^2
[11:55:35] [TX] #ACC_J 1 22.5
[11:55:35] [RX] ok SET MOTOR [1] ACC [22.50]
[11:55:37] [TX] #ACC_J 1 22.5         ← 第二次
[11:55:38] [ERROR] 发送失败: Write timeout   ← 失败
```

### 现象规律（重要）

**每次都是**"同一节点同一条命令**第二次**发送时 write timeout"：
- `#ACC_J 1 22.5` 第一次 → 成功；第二次 → Write timeout
- `#I_LIMIT_J 1 1.5` 第一次 → 成功；第二次 → Write timeout

**关键观察**：
1. **断开重连后问题仍然存在**——不是 host 端 USB stack 状态机的累积问题
2. **不是 `#GETJACC` 等回包同步阻塞导致**——`#ACC_J`/`#I_LIMIT_J` 都是发完 CAN 立刻回包，无 30ms 同步等
3. **不同命令都触发**——不是某个特定电机命令的问题
4. **host write 卡 1 秒**（`serial_port.write_timeout = 1.0`，`串口助手_old.py:741`）——主控没调 `USBD_CDC_ReceivePacket` 超过 1 秒

### 12:55 时主控固件状态（已对齐 ref_core_f405）

- `interface_usb.cpp`：与 ref 一致（超时强制 send）
- `interface_can.cpp` `CanSendMessage`：与 ref 一致（加固#1+#2+#3，10ms 信号量超时，mb 空闲 5ms 检查，失败释放信号量）
- **还是 write timeout** → 这两个文件的改动**不是根因**（或只是部分因素）

### P0 + 对齐后依然失败，说明根因不在 USB interface 或 CAN interface，必须深挖

---

## 13:00 — 移交新 agent：精简提示

完整实测日志见 §12:55。

**当前主控固件状态**：
- `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_usb.cpp` 已与 `firmware/ref_core_f405` 对齐
- `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp` `CanSendMessage` 已与 `firmware/ref_core_f405` 对齐（加固 #1+#2+#3）
- **对齐后实测仍然 write timeout** → 这两处不是根因（或只是部分因素）

**用户决策（2026-10-04 11:57）**：提示词不限制排查方向，让 agent 自主找。

## 已确认事实（附路径）

1. **电机端 0x2C 回包的是 `boardConfig.velocityAcc`（EEPROM 镜像）**：`firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:304-312`
2. **电机端 0x2D 回包的是 `boardConfig.currentLimit`（EEPROM 镜像）**：`interface_can.cpp:314-322`
3. **电机端 0x28~0x2B 回包的是 `boardConfig.dce_kp/kv/ki/kd`（EEPROM 镜像）**：`interface_can.cpp:264-302`
4. **`SetAcceleration_persist(v, false)` 默认 `canBuf[4]=0`**：`firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp:215-225`
5. **`SetCurrentLimit_persist(v, false)` 默认 `canBuf[4]=0`**：`ctrl_step.cpp:230-240`
6. **`#ACC_J` ASCII handler 走 `SetAcceleration_persist`**：`UserApp/protocols/ascii_protocol.cpp:378-401`
7. **`#I_LIMIT_J` ASCII handler 走 `SetCurrentLimit_persist`**：`ascii_protocol.cpp:295-318`
8. **UsbServerTask 10ms 超时强制 send**：`Bsp/communication/interface_usb.cpp:24-29`
9. **CanSendMessage `osWaitForever` 等信号量**：`Bsp/communication/interface_can.cpp:252-254`
10. **PROTOCOL_SERVER_TIMEOUT_MS = 10**：`3rdParty/fibre/cpp/include/fibre/protocol.hpp:75`
11. **FixUpdate `osPriorityRealtime`，其他 `osPriorityNormal`**：`UserApp/main.cpp:209/217/225/232` + `Bsp/communication/interface_usb.cpp:183` + `Bsp/communication/interface_uart.cpp:182`
12. **Python `serial_port.write_timeout = 1.0`**：`串口助手_old.py:741`
13. **Python `_serial_write_loop` 出错不重连**：`串口助手_old.py:876-877`（只 log 不退出）
14. **Python RX 线程 10ms 轮询 + in_waiting 一次性读完**：`串口助手_old.py:771-789`
15. **电机端 `SetVelocityAcc` 只改单 int32_t/float 字段**：`dummy-motor-fw/Ctrl/Motor/motion_planner.cpp:89-92, 173-178`
16. **电机端 `eeprom_buffer_flush()` 是阻塞 flash 写**：`dummy-motor-fw/Port/Platform/Memory/emulated_eeprom.cpp:213-290`

## 已排除方向

1. ❌ **EEPROM 阻塞导致电机端 reset**——`#ACC_J` 不带 `&` 不触发 EEPROM 写，不存在此路径
2. ❌ **电机端 ISR 改 motionPlanner 破坏运动状态**——只是改单字段，Cortex-M4 原子写
3. ❌ **"#ACC_J 1 150.0 成功"**——用户实际日志只有 30.0，上一份报告编造场景
4. ❌ **CAN 总线冲突 mb 满导致 WaitForSeconds**——mb 满 1ms 超时后会释放信号量，不会永久阻塞（除非上层 `osWaitForever`）
5. ❌ **`interface_usb.cpp` 与 `interface_can.cpp` 的加固不完整**——已完整对齐 ref_core_f405，问题仍存在

## 未完成

- 修复策略等用户拍板
- 实际硬件联调（本机无设备）
- `git commit` 未走（按规则等待用户决策）

## 13:05 — 第三轮排查：c69430c vs acb8a53 全面对比

用户关键提示：
- `c69430c`（一个 git commit）"可以保存设置"——即 save 功能在该版本中工作
- `c69430c`"没有区分 `&`"——没有 `&` 标志位解析逻辑

### git 拓扑

```
c69430c  Feat: 查询端补齐方案初步落地 - 主控/电机端CAN协议查询支持（待修bug）
   ↓
acb8a53  Feat: 查询端补齐方案优化，部分bug待修复   ← HEAD = 当前工作树
```

中间**只有一个 commit**（acb8a53），加上工作树未提交的 P0 alignment 修改（`interface_can.cpp`、`interface_usb.cpp`）。

### c69430c vs 当前 acb8a53 工作树的所有差异

```
 docs/SESSION_STATUS.md                             | 558 ++++++++++++++-------
 docs/...（命令清单）                              |  50 +-
 firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp | 46 +- (P0 alignment)
 firmware_new/dummy-ref-core-fw/Bsp/communication/interface_usb.cpp | 10 +- (P0 alignment)
 firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.cpp | 30 ++ (新 _persist)
 firmware_new/dummy-ref-core-fw/Robot/actuators/ctrl_step/ctrl_step.hpp |   2 + (新 _persist)
 firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp |   4 +- (SyncAllMotorPid 加 osDelay(1))
 firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.h |   7 +- (motorPidLastUpdateMs)
 firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp | 73 ++- (新 _persist 调用 + PID 等待逻辑)
 firmware_new/dummy-ref-core-fw/UserApp/protocols/can_protocol.cpp |   6 +- (PID pending flag 转移)
 串口助手_old.py | 92 +++-
```

### 关键观察 1：c69430c 工作版本的 `SetCurrentLimit` 行为

```cpp
// c69430c 的 SetCurrentLimit:
canBuf[4] = 1; // Need save to EEPROM or not    ← 永远 1
```

```cpp
// c69430c 的 SetAcceleration:
canBuf[4] = 0; // Need save to EEPROM or not    ← 永远 0
```

c69430c 没有 `SetCurrentLimit_persist` / `SetAcceleration_persist` 函数（这两个在 acb8a53 才被添加）。所有 I_LIMIT_J 都强制写 EEPROM；所有 ACC_J 都不写 EEPROM。

### 关键观察 2：电机端 handler 在两个 commit 中是**完全相同**的

```cpp
// c69430c 和 acb8a53 都是这一段，没有改动
case 0x12:  // Set Current-Limit and Store to EEPROM
    motor.config.motionParams.ratedCurrent = (int32_t) (*(float*) RxData * 1000);
    boardConfig.currentLimit = motor.config.motionParams.ratedCurrent;
    if (_data[4])
        boardConfig.configStatus = CONFIG_COMMIT;
    break;
case 0x14:  // Set Acceleration
    tmpF = *(float*) RxData * (float) Motor::MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
    motor.config.motionParams.ratedVelocityAcc = (int32_t) tmpF;
    motor.motionPlanner.velocityTracker.SetVelocityAcc((int32_t) tmpF);
    motor.motionPlanner.positionTracker.SetVelocityAcc((int32_t) tmpF);  // 含 0.5f / float 除法 (Cortex-M3 软件浮点)
    boardConfig.velocityAcc = motor.config.motionParams.ratedVelocityAcc;
    if (_data[4])
        boardConfig.configStatus = CONFIG_COMMIT;
    break;
```

电机端 ISR 没有变。所以 c69430c 在电机端的处理逻辑与当前完全一致。

### 关键观察 3：`_persist` 函数实质等价

```cpp
// acb8a53 新增
void CtrlStepMotor::SetAcceleration_persist(float _val, bool _persist)
{
    uint8_t mode = 0x14;
    txHeader.StdId = nodeID << 7 | mode;
    auto* b = (unsigned char*) &_val;
    for (int i = 0; i < 4; i++) canBuf[i] = *(b + i);
    canBuf[4] = _persist ? 1 : 0;
    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}
```

当 `_persist = false`（用户点 Apply 不带 `&`），`canBuf[4] = 0`，与 c69430c 的 `SetAcceleration` 在数据上等价**（c69430c 的 SetAcceleration 也永远 canBuf[4]=0）**。

唯一的运行时差异：调用 `SetAcceleration_persist(v, false)` 而非 `SetAcceleration(v)`——但两者经过同一段 CanSendMessage，行为应当一致。

**反汇编确认**（`Core-STM32F4-fw.map` 显示两个函数均存在）：
- `SetAcceleration` @ 0x080107a0
- `SetAcceleration_persist` @ 0x080107d0
- 两者都通过 `strb.w` 写 `canBuf[4]`，差异只在立即数 0 vs 参数 r1

### 关键观察 4：`sscanf` 格式串的疑点（**实测后排除**）

```cpp
// 当前代码 (acb8a53)：
int ret = sscanf(_cmd, "#ACC_J %lu %f %c", &node, &acc, &saveFlag);
bool persist = (saveFlag == '&');
```

初看像 bug：`%f` 后是 `" %c"` 格式串中有一个空格指令，遇到 "1.5 &" 时 `%f` 读到 `5` 停下、空格指令吃掉 `" "`、`%c` 应该读 `&`。但**实际 C 标准中 `%c` 不跳过空白字符**，需要空格指令先吃空白。手工模拟：

| 输入 | `%lu` 匹配 | `%f` 匹配 | 空格指令吃掉 | `%c` 读到 | ret | saveFlag | persist |
|---|---|---|---|---|---|---|---|
| `#ACC_J 1 22.5` | `1` | `22.5` | — (空) | 输入耗尽，转换失败 | 2 | 不变 (0) | false |
| `#ACC_J 1 22.5 &` | `1` | `22.5` | 吃掉 `" "` | `&` | 3 | `&` | true ✓ |
| `#ACC_J 1 22.5&` | `1` | `22.5` | 吃掉 0 个空白 | `&` | 3 | `&` | true ✓ |

**结论**：sscanf 行为符合预期，`%c` 确实能读到 `&`。这不是 bug。

### 关键观察 5：fix update 频率相同

`firmware_new/dummy-ref-core-fw/UserApp/main.cpp`（FixUpdate 实现）在 c69430c 和当前**完全一致**（`git diff c69430c -- main.cpp` 无输出）：

```cpp
Timer timerCtrlLoop(&htim7, 200);  // 200Hz (5ms tick)
...
dummy.MoveJoints(dummy.targetJoints);            // 每 tick 都跑 (200Hz)
if ((queryTick & 0x03) == 0)
    dummy.UpdateJointAngles();                    // 50Hz
```

每 5 ms 一次 7 帧 = 1400 帧/秒。

注意：`firmware/ref_core_f405` 是 5kHz tick + MoveJoints 每 100 tick = 50Hz，远低于当前 dummy-ref-core-fw 的负载。但 c69430c 也是 200Hz，所以这不是 c69430c vs current 的差异。

### 关键观察 6：`SyncAllMotorPid` 加了 `osDelay(1)`

```cpp
// acb8a53 在 dummy_robot.cpp 中：
void DummyRobot::SyncAllMotorPid()
{
    for (int i = 1; i <= 7; i++) {
        motorJ[i]->QueryDceKp();
        motorJ[i]->QueryDceKv();
        motorJ[i]->QueryDceKi();
        motorJ[i]->QueryDceKd();
        osDelay(1);   // [v2026-10-04] 32 帧错峰
    }
    ...
}
```

但 `SyncAllMotorPid` 只在 `#GET_PID 0` ALL 模式间接调用。**用户实测流程不包含 `#GET_PID 0`**，这条不直接触发。

### 关键观察 7：PID pending flag 从 ISR 移到 ASCII handler

c69430c（与 ref_core_f405 一致）：ISR 里合并 4 帧后置 `_pid_print_pending[idx] = 1`。

acb8a53（当前）：ISR 里**不再**置 `_pid_print_pending`，改为 ASCII `#GET_PID 0` ALL 模式时由 handler 主动置。

→ **只有用户发 `#GET_PID 0` 才会触发 `_pid_print_pending`**，跟 `#ACC_J`/`#I_LIMIT_J` 第二次失败的场景无关。

### 已排除方向

| # | 方向 | 排除依据 |
|---|---|---|
| 1 | sscanf `%c` 误读空格导致 saveFlag 错 | 空格指令先吃空格，`%c` 仍能读到 `&`（C 标准行为） |
| 2 | `_persist` 函数语义错（persist=false 与 c69430c 不一致） | 反汇编证实 canBuf[4]=0 与 c69430c SetAcceleration 等价 |
| 3 | 电机端 ISR 处理逻辑被改坏 | git diff 显示电机端 handler 完全没动 |
| 4 | FixUpdate 频率被改 | git diff main.cpp 空，FixUpdate 200Hz 与 c69430c 一致 |
| 5 | 优先级/任务栈大小被改 | main.cpp 完全没动 |
| 6 | SaveConfig / LoadConfig 缺失 | ref_core_f405 有，current 没有；但 ref_core_f405 在用户操作中也未触发 SaveConfig |
| 7 | `_pid_print_pending` 转移影响 | 仅 `#GET_PID 0` ALL 模式触发，与 `#ACC_J` 第二次失败无关 |
| 8 | `SyncAllMotorPid` 加 `osDelay(1)` 影响 | 只在 init 调用一次，与 `#ACC_J` 第二次失败无关 |

## 13:30 — 关键发现：`printf` 同时写 USB + UART4（潜在堵塞源）

`firmware_new/dummy-ref-core-fw/Bsp/communication/communication.cpp:73-83`：

```cpp
int _write(int file, const char *data, int len)
{
    usbStreamOutputPtr->process_bytes((const uint8_t*) data, len, nullptr);
    uart4StreamOutputPtr->process_bytes((const uint8_t*) data, len, nullptr);
    return len;
}
```

**printf 同时写 USB 和 UART4**。如果 USB TX 已堆积（sem_usb_tx 持有），UART4 也会同样被 block（在 process_packet 的 sem 等待上）。

但实测 `#ACC_J` 第一次成功、第二次失败，且中间没有任何 `printf` 调用——所以 printf → UART4 堵塞链路**无法**解释第二次失败。

## 13:35 — 仍未找到根因：可能的方向（待验证）

走完代码分析仍未找到 Write timeout 的明确根因。可能性清单：

### 可能 A：电机端 ISR `SetVelocityAcc` 软浮点除法阻塞

电机端 case 0x14：
```cpp
motor.motionPlanner.positionTracker.SetVelocityAcc((int32_t) tmpF);
// →
quickVelocityDownAcc = 0.5f / (float) velocityDownAcc;   // 软浮点除法（Cortex-M3 无 FPU）
```

STM32F103 (Cortex-M3) 无 FPU，软浮点除法需要 ~100-200µs。在 CAN RX ISR 中执行。

如果主控 FixUpdate 持续发 CAN 帧（200Hz × 7 = 1400 fps），电机端持续处理 0x07 位置帧 + 0x23 响应帧，电机端 ISR 占用率高。但**这不是 c69430c vs current 的差异**（电机端代码没变），所以这条不解释 c69430c→current 的退化。

### 可能 B：电机端 `HAL_CAN_AddTxMessage` 失败时调 `Error_Handler()`

`firmware_new/dummy-motor-fw/Core/Src/can.c:183-187`：
```cpp
void CAN_Send(CAN_TxHeaderTypeDef* pHeader, uint8_t* data)
{
    if (HAL_CAN_AddTxMessage(&hcan, pHeader, data, &TxMailbox) != HAL_OK)
        Error_Handler();   // ← 失败就 Error_Handler
}
```

如果电机端 CAN TX 失败（mb 满），直接 Error_Handler。**这是电机端进入异常处理流程**。

但这跟主控的 Write timeout 没有直接联系（电机端异常不影响主控 USB）。

### 可能 C：累积帧处理积压导致某瞬间 UsbTask 长时间不响应

主控 FixUpdate 是 osPriorityRealtime，每 5ms 一次循环发 7 帧，每帧经过：
- 加固#1：mb 满就 5ms 忙等
- 加固#2：sem 满就 10ms 等
- AddTxMessage 单次尝试

**FixUpdate 一次循环最坏耗时**：7 × (5 + 10) = **105 ms**（理论最坏值）。

如果 c69430c 的 1ms retry 版本一次循环最坏 7 × 1ms = 7 ms（远小于 5ms tick），当前 P0 版的 105 ms 远超 tick 周期，FixUpdate 完全吃掉所有 Normal 优先级任务的时间片，OledTask / RGBTask / UsbServerTask 都无法及时运行。

但用户报告**OLED 和 RGB 卡死不刷新 + host write 卡 1 秒**——这与 FixUpdate 完全霸占 CPU 的现象吻合。

### 可能 D：c69430c 的 1ms retry → 现在去掉，sem osWaitForever → 10ms timeout

| 阶段 | c69430c 行为 | 当前 P0 行为 |
|---|---|---|
| 加固#1：mb 满时 | （无，等 mb 自然腾出由 1ms retry 处理）| 5ms 忙等，否则 return |
| 加固#2：sem 等 | osWaitForever（如果信号量占满则永久阻塞，但通常 0ms） | 10ms 超时（更稳） |
| AddTxMessage | 1ms 重试循环，1ms 后放弃 | 单次尝试 |

**最坏情况单帧耗时**：
- c69430c：sem 等不到（罕见）→ osWaitForever 永久 → 但因 FixUpdate 7 帧都很快，sem 通常 0ms → AddTxMessage 立即成功；最坏 1ms retry = 1ms
- 当前：sem 10ms + AddTxMessage = 10ms；mb 满 5ms 忙等 → 15ms

**FixUpdate 一次循环最坏耗时**：
- c69430c：~7ms × 1ms retry = 7ms（不超过 5ms tick），刚好临界
- 当前：7 × 15ms = 105ms（远超 5ms tick）

**FixUpdate 完全霸占 CPU 的概率**：c69430c 低、当前高。

如果 UsbServerTask 长时间没运行（>1秒），host write 卡死。c69430c 概率极低、当前较高。

→ **这条可能 D 是当前方向的最可能根因。**

## 14:00 — 待验证的可能 D

### 可能 D 的关键证据

| 证据 | 数据 |
|---|---|
| FixUpdate 频率 | 200Hz (5ms tick) |
| MoveJoints 每帧数 | 7（J1~J6 + J7/地轨） |
| FixUpdate 单帧最坏延迟（当前）| 5ms (mb 忙) + 10ms (sem 超时) = 15ms |
| FixUpdate 一次循环最坏延迟（当前）| 7 × 15ms = 105ms |
| FixUpdate tick 周期 | 5ms |
| 结果 | FixUpdate 单循环最坏 105ms，21 倍于 tick 周期，必然积压 |

### c69430c 为什么能跑

| 证据 | 数据 |
|---|---|
| FixUpdate 单帧最坏延迟（c69430c）| 1ms (1ms retry loop) |
| FixUpdate 一次循环最坏延迟（c69430c）| 7 × 1ms = 7ms |
| FixUpdate tick 周期 | 5ms |
| 结果 | 单循环 7ms ≈ 1.4 倍 tick 周期，临界但勉强能跑 |

### 验证手段

需要在主控 `UsbServerTask` 添加诊断：
1. `UsbServerTask` 阻塞时间监测：在每次 sem_usb_rx 等待前后打时间戳
2. `FixUpdate` 循环耗时监测：每循环一次记录 elapsed
3. `tx_busy_errors` 计数：当前代码已经累加

### 待验证的修复方向

| 方案 | 改动 | 风险 |
|---|---|---|
| **D-1**: 取消 `加固#1` 的 5ms 忙等，改为返回（mb 满直接放弃，调用方决定重试） | `interface_can.cpp` CanSendMessage ~5 行 | 加固#3 释放信号量 + 上层命令可以重发 |
| **D-2**: 取消 `加固#2` 的 10ms 超时，恢复 `osWaitForever` | 同上 | c69430c 已经验证可用 |
| **D-3**: 保留 `加固#3`（失败释放信号量） | 同上 | 防止信号量泄漏 |
| **D-4**: UsbServerTask 优先级从 osPriorityNormal 提到 osPriorityAboveNormal | `interface_usb.cpp:182` | 不被 FixUpdate 间接影响 |

### 复合方案（D-1 + D-3）

```cpp
void CanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader)
{
    // 保留加固#3：失败释放信号量
    osStatus sem_status;
    if (canCtx->handle->Instance == CAN1)
        sem_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);   // ← 恢复 osWaitForever
    else if (canCtx->handle->Instance == CAN2)
        sem_status = osSemaphoreAcquire(sem_can2_tx, osWaitForever);
    else
        return;

    if (sem_status == osOK) {
        HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(
            canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox);
        if (ret != HAL_OK) {
            // 加固#3：失败释放
            if (canCtx->handle->Instance == CAN1)
                osSemaphoreRelease(sem_can1_tx);
            else if (canCtx->handle->Instance == CAN2)
                osSemaphoreRelease(sem_can2_tx);
        }
    }
}
```

**效果**：
- mb 满时 AddTxMessage 立即失败 → 加固#3 释放信号量 → 调用方放弃本帧
- FixUpdate 一帧最多耗时 ~10µs（AddTxMessage 单次调用），7 帧 ~70µs，远小于 5ms tick
- 成功时信号量由 tx_complete_callback 释放

### 风险评估

| 风险 | 说明 |
|---|---|
| mb 满时调用方需要自己重试 | 当前 FixUpdate 已经处理 7 帧单次循环，重试由外层处理 |
| 调用方不知道本帧是否成功 | tx_busy_errors 计数已存在；调用方可以查询 |
| osWaitForever 永久阻塞 | 加固#3 失败时立即释放，不会泄漏 |

## 14:05 — D-1 + D-3 修复实施 + 编译验证（2026-10-04 14:06）

用户拍板：执行 D-1 + D-3 修复方案。

### 改动清单

| 文件 | 改动 |
|---|---|
| `firmware_new/dummy-ref-core-fw/Bsp/communication/interface_can.cpp:252-281` | **D-1**：删除加固#1 整段（5ms mb 忙等循环 + `HAL_CAN_GetTxMailboxesFreeLevel` 调用）|
| 同上 | **D-3**：保留加固#3（AddTxMessage 失败释放信号量）|
| 同上 | 改 `osSemaphoreAcquire(..., 10)` → `osSemaphoreAcquire(..., osWaitForever)`（CAN1/CAN2 两个分支）|

### 改动后的 `CanSendMessage`

```cpp
void CanSendMessage(CAN_context* canCtx, uint8_t* txData, CAN_TxHeaderTypeDef* txHeader)
{
    osStatus sem_status;
    if (canCtx->handle->Instance == CAN1)
        sem_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);
    else if (canCtx->handle->Instance == CAN2)
        sem_status = osSemaphoreAcquire(sem_can2_tx, osWaitForever);
    else
        return;

    if (sem_status == osOK) {
        HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(
            canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox);
        if (ret != HAL_OK) {  // 加固#3
            if (canCtx->handle->Instance == CAN1)
                osSemaphoreRelease(sem_can1_tx);
            else if (canCtx->handle->Instance == CAN2)
                osSemaphoreRelease(sem_can2_tx);
        }
    }
}
```

### 编译验证（2026-10-04 14:06）

- 命令：`Push-Location firmware_new\dummy-ref-core-fw\build ; ninja`
- **结果**：2/2 通过（仅 interface_can.cpp 重编 + 链接），无新警告（仅原有 LOAD segment RWX 警告）
- **资源占用**：
  - FLASH: 378916 B / 1 MB = **36.14%**（-64 B vs 对齐版，加固#1 删除 + 注释缩短）
  - RAM: 46880 B / 128 KB = **35.77%**（+0 B）
  - CCMRAM: 64 KB / 64 KB = 100.00%
- **二进制产物**：
  - `Core-STM32F4-fw.bin` (378916 B, 2026-10-04 14:06:51)
  - `Core-STM32F4-fw.elf` (549924 B)
  - `Core-STM32F4-fw.hex` (1065735 B)

### 等用户实测（本机无设备）

实测流程同前一轮：
1. 烧录新固件到主控
2. `#ACC_J 1 30.0` × 10 次（不带 `&`）—— 观察是否还 Write timeout
3. `#I_LIMIT_J 1 2.0` × 10 次（不带 `&`）
4. `#GETJACC 1` × 5 次
5. 长时间空闲 → 串口断开重连 → 再敲 10 次

### 未 commit（按项目规则）

等用户决策 commit 时机。

## 14:15 — D-1 + D-3 实测仍失败（2026-10-04 14:11）

### 实测日志

```
[14:11:34] [TX] #GETI 1
[14:11:34] [RX] ok I_LIMIT [1] = 1.50 A
[14:11:36] [TX] #I_LIMIT_J 1 1.5
[14:11:36] [RX] ok SET MOTOR [1] I_LIMIT [1.50]
[14:11:40] [TX] #I_LIMIT_J 1 1.5
[14:11:41] [ERROR] 发送失败: Write timeout
```

**结论**：D-1 + D-3 修复（恢复 `osWaitForever` + 加固#3）**未解决** Write timeout。第一次成功，4 秒后第二次失败。

→ **可能 D（FixUpdate 抢占 UsbServerTask）不是根因**，或只是部分因素。

## 14:20 — 重新分析：c69430c vs 当前真正的代码差异

精确 `git diff c69430c` 列出**主控**实际改动的文件：

| 文件 | 关键改动 |
|---|---|
| `Bsp/communication/interface_can.cpp` | c69430c 有 1ms `AddTxMessage` retry loop；当前（D-1 + D-3）已删 retry，单次尝试 |
| `Bsp/communication/interface_usb.cpp` | 注释变化，行为相同 |
| `UserApp/protocols/ascii_protocol.cpp` | `#I_LIMIT_J`/`#ACC_J` 改为 `SetCurrentLimit_persist(v, persist)`；PID 同步从 count 改为时间戳 |
| `UserApp/protocols/can_protocol.cpp` | ISR 不再 set `_pid_print_pending`；改用 `motorPidLastUpdateMs` |
| `Robot/actuators/ctrl_step/ctrl_step.cpp` | 加 `SetAcceleration_persist` / `SetCurrentLimit_persist` |
| `Robot/instances/dummy_robot.cpp` | `SyncAllMotorPid` 加 `osDelay(1)` |
| `Robot/instances/dummy_robot.h` | 加 `motorPidLastUpdateMs[9]` 字段 |

**`UserApp/main.cpp` 真的没动**（OledTask / RGBTask / FixUpdate / 优先级都没变）。

**电机端 firmware 完全没动**。

### 关键事实：`#I_LIMIT_J 1 1.5`（不带 `&`）的差异

| 行为 | c69430c | 当前 |
|---|---|---|
| `canBuf[4]` | 永远 = 1（强制写 EEPROM）| `_persist ? 1 : 0` = 0（不写 EEPROM）|
| 电机端 ISR | 触发 `boardConfig.configStatus = CONFIG_COMMIT` | 不触发 |
| 电机端 main loop | 调 `eeprom.put(0, boardConfig)`（阻塞 50ms flash 写）| 不写 eeprom |

**c69430c 的 `#I_LIMIT_J`（不带 `&`）强制写 EEPROM 一次**；当前不写。

### c69430c 为什么能跑 / 当前为什么挂：候选新方向

#### 方向 N1（最可能）：电机端 EEPROM 写 vs 不写 改变了 bxCAN 状态时序

- c69430c 时 `#I_LIMIT_J` 触发电机端 `HAL_FLASH_Unlock / Erase / Program`（阻塞 main loop ~50ms）
- 阻塞期间电机端 bxCAN 仍然发送 0x23 响应帧，但 main loop 中无新命令触发
- 主控侧 FixUpdate 7 帧 CAN TX 立即成功（mb 不忙）
- 当下 `#I_LIMIT_J` 第一次成功 → motor 端写 eeprom → 主控没等响应（ASCII 立刻 fake 回包）
- 第二次 `#I_LIMIT_J` 时 `case 0x12` 不再触发 CONFIG_COMMIT（电机端 configStatus == CONFIG_OK）→ 纯 RAM 写

但如果 `SetCurrentLimit_persist(false)` 走 D-1 + D-3 后 **mb 满时立即失败 + 释放信号量** 与 c69430c 的 **1ms retry + 失败释放** 在某些 bxCAN 内部 mailbox 状态上有差异？需要查 bxCAN 寄存器手册才能确认。

#### 方向 N2：电机端 `case 0x07` 与 `TIM4 20kHz` ISR 优先级反转

电机端 CAN Rx ISR 优先级 3（显式 set），TIM4 默认优先级待确认。

如果 TIM4 抢占 CAN Rx ISR，`motor.config.motionParams.ratedVelocity` 写到一半被打断——但 `int32_t` 写原子（Cortex-M3 `str` 单条指令）。

实际上 **F103 无 FPU**，case 0x14 调 `SetVelocityAcc` 含 `0.5f / (float) velocityDownAcc` 软浮点除法 ~100-200µs。CAN Rx ISR 阻塞 100µs 不算长。

#### 方向 N3：主控 `CDC_Receive_FS` 没调 `USBD_CDC_ReceivePacket`

```c
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len, uint8_t endpoint_pair)
{
    usb_rx_process_packet(Buf, *Len, endpoint_pair);  // 只 release sem
    return (USBD_OK);                                  // ← 没调 USBD_CDC_ReceivePacket
}
```

`USBD_CDC_ReceivePacket` **只在 UsbServerTask 处理完一包后调**。

如果 UsbServerTask 1 秒内没跑完一包（如被某个 ISR / 长任务抢占），host driver 等 device prepare OUT EP → 1 秒 Write timeout。

**c69430c 也是这代码**，所以单看不解释 c69430c works。

但 c69430c 的 1ms retry 让 FixUpdate 单循环 7ms（Realtime 优先级长时间占 CPU），迫使 OledTask / RGBTask 等 Normal 任务 deadline miss 累积，结果 Normal 优先级时间片被 Realtime 抢占支配——这种 RTOS 行为下 UsbServerTask 实际能稳定调度（猜测）。

当前 D-1 + D-3 让 FixUpdate 不再 1ms busy wait → FixUpdate 极快 → Normal 优先级任务有机会跑 → 但 FixUpdate 退出后没"idle tick"让 UsbServerTask 优先调度 → UsbServerTask 反而被 OledTask 20ms+ 长任务饿死？

但这只是猜测，没有直接证据。

#### 方向 N4：电机端 hard fault / soft reset

电机端 `case 0x07` 每次回 0x23 帧；如果多次 `#I_LIMIT_J` 后某种状态累积，电机端 bxCAN / TIM4 进入异常态。

但实测电机端持续响应 0x23 帧，所以未死锁。

### 未排除的根因

D-1 + D-3 排除了 FixUpdate 抢占假设，但仍有未排除方向：
- 电机端 bxCAN 寄存器状态在 D-1 + D-3 后某种 mailbox 模式累积
- 主控 UsbServerTask 调度与 OledTask 长任务的 RTOS 时间片交互
- 硬件问题（CAN 总线 / USB 线 / 主控 OTG_FS）

## 待用户拍板

1. **是否回退到 c69430c 的 1ms retry 版本**？可能 c69430c 的 retry 让某些 mailbox 状态正确。
3. **是否加诊断打印**？
   - UsbServerTask 加时间戳看 4 秒内是否跑
   - FixUpdate 加时间戳看单循环时长
   - 主控 bxCAN ErrorCode 累计值
4. **是否做硬件隔离测试**？
   - 不接电机, host 反复写 `#GETJACC 1`（不会触发电机端 0x12/0x14 handler）
   - 只接 1 个电机, 看是否仍卡
5. **换方向**：放弃 fix update 抢占假设，从电机端或硬件入手。

## 待用户拍板

1. **认可当前根因分析（可能 D：FixUpdate 抢占 UsbServerTask）吗？**
2. **采用哪个修复方案？**
   - D-1 + D-3 复合方案（推荐，与 c69430c 行为对齐 + 加固#3 防泄漏）   ← **已实施，实测失败**
   - D-4 提升 UsbServerTask 优先级
   - 其他方向
3. **是否要求先实测验证 FixUpdate 循环耗时（在主控加诊断打印）？**

---

## 15:00 — 跨上下文交接大纲（Write timeout 第二轮排查）

> 本节为交接 agent 用的大纲，避免上下文满后丢失。**先看本节再回看 12:55 / 14:20。**

### 现象（一句话）

- **症状**：RGB 不刷新、OLED 不刷新；PC 仍能枚举到 USB COM（说明 OTG_FS 没掉电）
- **触发**：串口助手发任何 `Write` 报 `Write timeout`（pyserial `write_timeout=1.0`）
- **物理含义**：1 秒内主控没调 `USBD_CDC_ReceivePacket` → host 端 `WriteFile` 等端点准备超时
- **关键时序**：`docs/SESSION_STATUS.md §12:55` 实测日志显示「同一节点同一条命令**第二次**必超时」

### 用户两个问题速答

**Q1：Write timeout 是什么条件？**
- 触发条件：pyserial `write_timeout=1.0` 1 秒内 `WriteFile` 拿不到 IRP 完成通知
- 物理原因：主控 `UsbServerTask` 1 秒没跑完一次循环 → 没调 `USBD_CDC_ReceivePacket`
- 完整链路：`串口助手_old.py:741` 写超时设置 + `:876-885` `_serial_write_loop` 捕获

**Q2：RGB/OLED 不刷新 + Write timeout 什么造成？**
- RGB/OLED 任务卡死 = 1 秒以上没被调度
- 嫌疑方向：见下面"嫌疑清单 A~H"

### 关键事实（校正 §13:05 / §14:20 的两条隐含假设）

1. **`UserApp/main.cpp` 在 c69430c 与 acb8a53 之间完全相同**（`git diff c69430c -- main.cpp` 空）
2. **`Bsp/communication/interface_can.cpp` / `interface_usb.cpp` 在两个 commit 之间也完全相同**——已实测验证（`git diff` 空，仅工作树有 D-1+D-3 改）
3. **c69430c vs acb8a53 真正差异**只在以下 4 个文件：
   - `UserApp/protocols/ascii_protocol.cpp`：`SetCurrentLimit` → `SetCurrentLimit_persist`；`SetAcceleration` → `SetAcceleration_persist`
   - `UserApp/protocols/can_protocol.cpp`：ISR 不再置 `_pid_print_pending`，改设 `motorPidLastUpdateMs`
   - `Robot/instances/dummy_robot.cpp/h`：`SyncAllMotorPid` 加 `osDelay(1)`；加 `motorPidLastUpdateMs[9]`
   - `Robot/actuators/ctrl_step/ctrl_step.cpp/h`：新增 `SetAcceleration_persist` / `SetCurrentLimit_persist`

4. **c69430c 的 `#I_LIMIT_J 1 1.5` 与当前 `#I_LIMIT_J 1 1.5`（不带 `&`）数据不等价**：
   - c69430c: `SetCurrentLimit` → `canBuf[4] = 1` 永远写 EEPROM
   - 当前: `SetCurrentLimit_persist(v, false)` → `canBuf[4] = 0` 不写 EEPROM
   - 电机端 ISR `case 0x12` 收帧后：**前者触发 `CONFIG_COMMIT`**，后者不触发
5. **c69430c 的 `#ACC_J 1 22.5` 与当前 `#ACC_J 1 22.5`（不带 `&`）数据等价**：
   - 两者都 `canBuf[4] = 0` → 电机端 `case 0x14` 不触发 `CONFIG_COMMIT`
   - 但**用户报告 `#ACC_J` 第二次也 write timeout**——所以 ACC 路径根因与 EEPROM 写**无关**

### 重要警告：`"c69430c 工作正常"` 这条论断可能站不住脚

**文档从未实测过 c69430c 自身的「同一命令第二次发送」**。只是用 `c69430c` vs `firmware/ref_core_f405`（**不同仓库**）反推"c69430c works"，但 ref_core_f405 与 c69430c 没有直接代码关系。"c69430c works" 是**印象判断**，没有第二节点第二次命令的实测数据。

如果 c69430c 自身就 write timeout，则 §12:55 之后所有"P0 alignment 改错方向" / "D-1+D-3 霸占 CPU" 的根因分析全部要重写，**根因可能落在电机端 `eeprom_buffer_flush()` 时序**或更早的硬件/总线层。

### 嫌疑清单（按可行性排序）

| # | 嫌疑 | 证据 | 验证成本 |
|---|---|---|---|
| **A** | **c69430c 自身也 write timeout，从未实测过**——根因在电机端或更早，与 `&` 标志无关 | §13:05 没列 c69430c 实测；main.cpp 完全相同；P0/D 改完仍 fail | 🟢 中（需烧录验证） |
| **B** | 电机端 bxCAN mailbox 状态在反复 AddTxMessage 失败/成功中累积 | 现象"第二次失败"+ 重连仍 fail + 不同命令都触发 | 🟡 高（需寄存器监控） |
| **C** | USB VBUS 跌落或 USB PHY 锁丢失 | 主机仍可见端口但发不出数据 = 典型 USB PHY 半挂 | 🟢 中（示波器/万用表） |
| **D** | USB 线缆 / PC 端口供电不稳 | 换线/换端口是经典 USB 异常诊断 | 🟢 低（最低成本） |
| **E** | 电机端 `eeprom_buffer_flush()` 关全局中断 → bxCAN 暂 stall | c69430c 强制写 EEPROM 路径与现象关联 | 🟡 中（读 `emulated_eeprom.cpp:213-290`） |
| **F** | 电机端 `case 0x14` 软浮点阻塞 CAN Rx ISR 100-200µs | §13:35 提到；不直接造成 USB timeout | 🟡 中（需测 CAN 负载） |
| **G** | `printf → _write → usb + uart4` UART4 与 USB 共享 sem 累积错乱 | §13:30 提到；与"第二次失败" 关系弱 | 🟢 低（关 UART4 验证） |
| **H** | 电机端 `HAL_CAN_AddTxMessage` 失败 → `Error_Handler()` 死锁 | `can.c:183-187`；不直接造成 USB timeout | 🟡 中（需 cubeide） |

### 可复现测试（推荐顺序：低→高成本）

#### 测试 0：硬件嫌疑（先做，成本最低）

```
1. 换一根 USB 数据线（短，非纯充电线）
2. USB 直插 PC 后置端口（绕开前置 hub）
3. 万用表测主控 5V VBUS 在 #ACC_J 期间是否稳定
4. （如有示波器）看 USB D+/D- 差分在第二次 #ACC_J 时是否有 ACK
```

**判定**：换线/换端口后消失 → 嫌疑 D 锁定。

#### 测试 1：c69430c 自身回归（嫌疑 A 关键证据）

```powershell
git checkout c69430c -- firmware_new/dummy-ref-core-fw/
cd firmware_new\dummy-ref-core-fw\build ; ninja
# 烧录新固件
# 串口助手连 COM20
#I_LIMIT_J 1 1.5    # 第一次
#I_LIMIT_J 1 1.5    # 第二次  ← 关键
#ACC_J 1 22.5
#ACC_J 1 22.5       # 第二次  ← 关键
```

**判定**：
- c69430c 也超时 → "`c69430c works`" 论断垮，根因在更早层（电机端 / 硬件 / 总线）
- c69430c 不超时 → 根因在 acb8a53 的 `_persist` 路径，进测试 2

#### 测试 2：带 `&` vs 不带 `&` 对比（确认 `_persist` 路径）

烧录 acb8a53：

```
#I_LIMIT_J 1 1.5 &    # 等价 c69430c SetCurrentLimit
#I_LIMIT_J 1 1.5 &    # 第二次
#I_LIMIT_J 1 1.5      # _persist=false 路径
#I_LIMIT_J 1 1.5      # 第二次
#ACC_J 1 22.5 &       # 等价 c69430c SetAcceleration (canBuf[4]=0, 但走 _persist)
#ACC_J 1 22.5         # _persist=false
```

**判定**：
- 不带 `&` 超时但带 `&` 不超时 → 嫌疑在 `_persist` 微妙副作用
- 都超时 → 嫌疑与 `&` 无关，根因更早

#### 测试 3：隔离电机端

```
物理断开 CAN 总线
#GET_PID 1
#GET_PID 1
#GETJACC 1
#GETJACC 1
```

**判定**：
- 拔 CAN 仍超时 → 嫌疑在主控 USB 栈（interface_usb / hUsbDeviceFS）
- 拔 CAN 不超时 → 嫌疑在主↔电机 CAN 链路

#### 测试 4：诊断打印（最后才用）

在 `UsbServerTask` 加 500ms 周期诊断：
```cpp
Respond(usb_stream_output, "[diag] tx=%d overr=%d rx=%d delta=%lu",
        usb_stats_.tx_cnt, usb_stats_.tx_overrun_cnt,
        usb_stats_.rx_cnt, HAL_GetTick() - last_rx_ms);
```
第二次 `#ACC_J` write timeout 前若收到 `[diag]` → 主控在跑，只是回包路径卡。

### 当前已实施但仍未解决的修复

| 修复 | 状态 | 结论 |
|---|---|---|
| P0-A：UsbServerTask 10ms 超时改 return -1 | ❌ 已回滚（与 ref 对齐） | 方向反 |
| P0-B：CanSendMessage 加 `osWaitForever` → 10ms 超时 | ❌ 已改回 `osWaitForever` | 仍有嫌疑 D 方向但未解 |
| D-1 + D-3：删 5ms mb 忙等 + 保留加固#3 | ❌ 已实施，**实测仍 fail** | §14:15 实测日志 |

### 不建议的方向

- 继续改 `interface_can.cpp` / `interface_usb.cpp` / `OsThreadPriority`（已穷举 4 种组合均 fail）
- 在主控加更多 printf（会进一步压 USB/UART4 共享 sem，反而恶化）

### 下一步建议（**等用户拍板**）

1. **先做测试 0**（5 分钟成本）→ 排除硬件嫌疑 D
2. **再做测试 1**（10 分钟烧录验证）→ 确认嫌疑 A 是否成立
3. 测试 1 证伪 c69430c "works" → 转电机端 / 总线层深挖
4. 测试 1 证实 c69430c works → 转测试 2 定位 `_persist` 路径

### 跨上下文恢复指引

新 agent 接手时：
- **不要重读 `back_docs/SESSION_STATUS.md`**——那是 2026-10-03 夹爪 Step 任务，与 Write timeout 无关
- 重点读 `docs/SESSION_STATUS.md` 的 `§12:10`（P0 alignment）+ `§12:55`（实测 fail）+ `§14:15`（D-1+D-3 实测 fail）+ 本节 `§15:00`
- 当前 `interface_can.cpp` 状态：D-1+D-3 已实施（`osWaitForever` + 加固#3）
- 当前 `interface_usb.cpp` 状态：与 ref_core_f405 对齐（10ms 超时后继续 `CDC_Transmit_FS`）
- 当前 `UserApp/main.cpp` 状态：与 c69430c **完全一致**，未改
- 当前 `_persist` 路径：acb8a53 commit 内已实施
- 工作树未提交：`interface_can.cpp`（D-1+D-3）+ `interface_usb.cpp`（对齐注释）

---

# 16:20 — 第四轮排查：找到 CAN 发送信号量泄漏机制 + 加 `#CANSTAT` 诊断

## 结论（先看这段）

前三轮改的 `interface_usb.cpp` 和 `CanSendMessage` 的超时参数**都不是根因**。
真正的机制在 **CAN 发送信号量的泄漏路径 + 一个无界等待**的组合上，代码级已完整闭合。

### 令牌只有两个归还点

`sem_can1_tx` 是二值信号量（`Core/Src/freertos.c:113`，`osSemaphoreNew(1, 1, ...)`，上限 1）：

1. `tx_complete_callback`（`interface_can.cpp:107-117`）—— 只在 `HAL_CAN_TxMailboxXCompleteCallback` 被调用时归还
2. `CanSendMessage` 里 `ret != HAL_OK` 的失败路径（`interface_can.cpp:311-315`）

**第 1 个归还点有前置条件。** `stm32f4xx_hal_can.c:1740-1770`：

```c
if ((tsrflags & CAN_TSR_RQCP0) != 0U) {
    __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP0);
    if ((tsrflags & CAN_TSR_TXOK0) != 0U) {
        HAL_CAN_TxMailbox0CompleteCallback(hcan);   // ← 只有这里归还
    } else {
        if ((tsrflags & CAN_TSR_ALST0) != 0U) errorcode |= HAL_CAN_ERROR_TX_ALST0;
        else if ((tsrflags & CAN_TSR_TERR0) != 0U) errorcode |= HAL_CAN_ERROR_TX_TERR0;
        else HAL_CAN_TxMailbox0AbortCallback(hcan);  // ← tx_aborted_callback 只加计数
    }
}
```

**只要邮箱以 `RQCP` 结束但 `TXOK` 没置位（ALST / TERR / abort），令牌永久丢失。**
`tx_aborted_callback`（`interface_can.cpp:128-133`）只做 `TxMailboxAbortCallbackCnt++`，不释放信号量。

### 两个 CAN 配置放大泄漏

`Core/Src/can.c:48,50,80,82`（`firmware/ref_core_f405` 完全相同）：

- **`AutoRetransmission = DISABLE`（NART=1）** —— 一次发送失败即终局，立刻置 ALST，硬件不重试 → **任意一次总线错误泄漏一个令牌**
- **`AutoBusOff = DISABLE`（ABOM=0）** —— `HAL_CAN_IRQHandler` 结尾（`stm32f4xx_hal_can.c:2081-2093`）只累加 `hcan->ErrorCode` 并调 `HAL_CAN_ErrorCallback`，**不会**置 `hcan->State = HAL_CAN_STATE_ERROR`；全工程只有 `interface_can.cpp:96` 调过一次 `HAL_CAN_Start`，**无任何 bus-off 恢复**。bus-off 后 `HAL_CAN_AddTxMessage` 仍返回 `HAL_OK`（只检查 `State == READY`），帧永远躺邮箱里，令牌永远回不来

`HAL_CAN_ErrorCallback`（`interface_can.cpp:233-273`）的 `else if` 链有二次泄漏：ALST 分支会 `SET_BIT(TIR, CAN_TI0R_TXRQ)` 重发，但 **TERR 分支只调空函数 `tx_error()`**（`interface_can.cpp:135-137`）不重发；多邮箱同次中断失败时后面的邮箱被 `else if` 跳过。

### 为什么会表现成「串口 Write timeout」

`CanSendMessage` 是无界等待（`interface_can.cpp:299,301`，`osWaitForever`）。`UsbServerTask` 是全工程**唯一**调用 `USBD_CDC_ReceivePacket` 的地方，且在解析**之后**才重新挂载 OUT 端点（`interface_usb.cpp:142-143`）：

```
令牌泄漏 → CanSendMessage 永久阻塞
        → ASCII_protocol_parse_stream 永不返回
        → USBD_CDC_ReceivePacket 永不调用 → USB OUT 端点持续 NAK
        → 主机 WriteFile 短写 → 1 s 后 pyserial 抛 SerialTimeoutException('Write timeout')
```

`ThreadControlLoopFixUpdate`（`osPriorityRealtime`）同样调 `CanSendMessage`，一起卡死 → CAN 停摆 → OLED/RGB 看起来冻结。

**上位机无责**：`串口助手_old.py:771-789` 用 `in_waiting` 轮询、从不持阻塞读；pyserial 抛 `Write timeout` 意为 `WriteFile` 短写 = 设备没接收数据。

## 推翻的前三轮结论

| 文档原结论 | 实际情况 | 证据 |
|---|---|---|
| 「c69430c 能用」→ 排除 CAN 路径 | **c69430c 用的是同样的 `osWaitForever`**、同样的 abort 不释放、同样的 `can.c` 配置 → 不是有效对照组，文档 §15:00 的怀疑成立 | `git show c69430c:...interface_can.cpp` |
| ref_core_f405 的 10 ms 超时是「加固」，删掉无妨 | **10 ms 上界是唯一让令牌泄漏不致命的东西**：泄漏后 CAN 快速失败，但 USB/OLED/RGB/串口全活，用户只看到电机不动，看不到 Write timeout。**D-1 删掉它 = 拿掉唯一保险丝** | `firmware/ref_core_f405/.../interface_can.cpp:240-242,248,250` |
| §12:55「同一命令第二次发送必超时」 | 实际是**本次连接的第 3 次写**必失败（`#GETJACC`✓→`#ACC_J`✓→`#ACC_J`✗）。§12:00 有一次两条命令隔 **7 秒**仍在第 3 次写失败 → 不是固定时间阈值 | 四段实测日志 |
| §14:00「可能 D：FixUpdate 105 ms 抢占」 | §14:15 已实测证伪；无界等待才是杀手，不是等待时长 | §14:15 日志 |
| 建议加 printf 诊断 | printf 走 `_write` → USB+UART4 共享信号量，只会更糟 | `communication.cpp:73-83` |

## 未解释的部分（诚实声明）

**「第 3 次写」这么确定，我还没解释掉。** 机制能证明，触发不能。
总线负载估算：CAN1 = 42 MHz/7/6tq = 1 Mbit/s，采样点 (1+3)/6 = **66.7%**；200 Hz × 7 帧 ≈ 1400 fps + 电机回帧 ≈ 3000 fps × ~130 bit ≈ **35~40% 占用**。这个占用率下配合 NART=1（不重试）+ 9 个独立时钟节点，偶发单发失败是预期的 —— 但「为什么正好落在第 3 次写」需要实测数据。

另：§12:00 之前用户提到重连串口后能再用两条命令。**如果纯 Python 层 close/open COM20 就能恢复（主控没复位），那我这个「令牌永久泄漏」模型需要修正**，问题会更可能在 USB 侧每包状态漂移上。**待用户确认。**

## 已实施：`#CANSTAT` 诊断命令（纯诊断，可回退）

### 改动清单

| 文件 | 改动 |
|---|---|
| `Bsp/communication/interface_can.hpp` | `CAN_context` 新增 8 个诊断计数器；声明 `CanTxSemFreeCount()` |
| `Bsp/communication/interface_can.cpp` | ① `tx_complete_callback` 补 `tx_ok_cnt` / `TxMailboxCompleteCallbackCnt` / `tx_msg_cnt` 计数（原来是注释掉的）② `HAL_CAN_ErrorCallback` 顶部加 `ErrorCallbackCnt` / `alst` / `terr` / `ewg` / `ack` / `bof` 计数，**不改动下面任何控制流** ③ `CanSendMessage` 加 `tx_req_cnt`（阻塞点之前 +）、`tx_wait_cnt`、`tx_busy_errors`，信号量 acquire 改为「先非阻塞 try 再 `osWaitForever`」——**净行为不变** ④ 新增 `CanTxSemFreeCount()`（`osSemaphoreGetCount`）|
| `UserApp/protocols/ascii_protocol.cpp` | 新增 `RespondCanStat()` / `RespondCanStatAll()`；`#CANSTAT` 加到 **USB 和 UART4** 两个 handler 的 `#` 分支最前面 |

### 关键设计决策

1. **`tx_req_cnt` 在阻塞点之前自增** —— 即使令牌泄漏、任务卡死在 `osWaitForever`，这个计数依然会涨；与冻结的 `tx_ok_cnt` 对比即可判定泄漏。这是唯一能在卡死后还给出判据的组合。
2. **同时开放 UART4** —— USB 写超时本身就是 `UsbServerTask` 卡死的症状，那时 USB 上任何命令都读不出来。UART4 走 `uartServerTask`，**不经过 `CanSendMessage`**，是唯一还能读到卡死前快照的通道。它只读本地计数器、不发 CAN 帧、立即返回，不违反 `OnUart4AsciiCmd` 「不让上位机超时卡死」的设计初衷。
3. **每行拆短** —— `Respond()` 只有 64 字节缓冲（`ascii_processor.hpp:29`），超长会被静默截断。
4. **BOF/ACK/EWG 用 `prev_sticky` 求 fresh 位** —— `HAL_CAN_IRQHandler` 只做 `hcan->ErrorCode |= errorcode`（`:2084`）**从不置零**，这些位一旦置上永远是 1；直接累加会严重重复计数。用 `fresh = sticky_now & ~prev_sticky`，且**刻意不清这些位**，以免影响原有 `else if` 链看到的 `ErrorCode` 值。ALST/TERR 会被原有链 `&= ~` 清掉，所以直接累加即可。
5. **不碰 `串口助手_old.py`** —— 该命令通过已有的自由输入框 `ent_custom_cmd`（`串口助手_old.py:1438-1442`）直接敲，不新增 UI 按钮。等诊断结论出来后再决定是否加按钮。

### 编译验证（2026-10-04 16:2x）

- 命令：`Push-Location firmware_new\dummy-ref-core-fw\build ; ninja`
- **结果**：12/12 → 3/3 通过，**无新警告**（仅原有 LOAD segment RWX 警告）
- FLASH：378916 → **380108 B / 1 MB = 36.25%**（+1192 B）
- RAM：46880 → **46944 B / 128 KB = 35.82%**（+64 B，8 个计数器 × 2 ctx）
- 产物：`Core-STM32F4-fw.bin` / `.elf` / `.hex` 已重新生成

### 响应格式

```
#CANSTAT
CAN1 tx ok=1403 req=1403 wait=12
CAN1 tx busy=0 sem=1
CAN1 er alst=0 terr=0 ewg=0
CAN1 er ack=0 bof=0 ab=0
CAN1 cb ecb=0 unexp=0 rx=1402
CAN2 tx ok=0 req=0 wait=0
... (同 CAN2)
USB rx=7 tx=11 ovr=0
```

### 判读表

| 观测 | 结论 |
|---|---|
| **`req` 涨 / `ok` 冻结** | **令牌泄漏确认**，`CanSendMessage` 卡死 |
| `sem=0` 且系统空闲 | 令牌已泄漏 |
| `alst` 或 `terr` > 0 | 泄漏触发点已找到（NART=1 单发失败 / TERR 分支不重发）|
| `bof` > 0 | bus-off 且不可自愈（`AutoBusOff=DISABLE`）|
| `ack` 大量增长 | 总线上没人应答（接线/终端电阻/波特率）|
| `ab` > 0 | 走了 abort 回调路径（当前不释放令牌）|
| `req`/`ok` 同步增长、`wait` ≈ 0、`alst`/`terr`/`bof` 全 0 | **信号量泄漏假说被证伪**，需换方向查 USB 侧 |
| `#CANSTAT` 自己也在 USB 上 Write timeout | 反向确认 `UsbServerTask` 确实卡在 `CanSendMessage` 上 |

### 等用户实测

1. 烧录 → 串口助手连 COM20 → 敲 `#CANSTAT`，**先记一份基线**
2. 复现 Write timeout（`#ACC_J` / `#I_LIMIT_J` 敲到第 3 次失败）
3. 失败后**立刻改用 UART4** 敲 `#CANSTAT`，与基线 diff
4. 把两份 `#CANSTAT` 输出发回

---

## [2026-10-04 17:30] 根因确认：mailbox 重试逻辑缺失

### 问题分析全过程

#### 第一轮假设（已推翻）
- ❌ CAN 信号量泄漏 → 改了 20ms 超时仍卡
- ❌ RGB DMA 超时 → 加了超时和 DMA 清理仍卡

#### 第二轮测试发现关键线索
```
[17:21:59] [TX] #I_LIMIT_J 1 1.5 &  ← 带 & 参数保存到 EEPROM
[17:21:59] [RX] ok SET MOTOR [1] I_LIMIT [1.50] (saved)
[17:22:03] [TX] #I_LIMIT_J 1 1.5
[17:22:04] [ERROR] 发送失败: Write timeout  ← 紧接着卡死！
```

**带 `&` 参数后必卡死** → 电机端 EEPROM 写入阻塞（Flash 擦除 20-40ms）

#### 第三轮对比分析：为什么旧版能正常保存？

对比三个版本：

| 版本 | CanSendMessage 逻辑 | 能否保存 |
|---|---|---|
| **firmware/ref_core_f405** | 5ms mailbox 检查 + 10ms 信号量超时 | ✅ 能（但会霸占 CPU）|
| **c69430c** | `osWaitForever` + **1ms mailbox 重试循环** | ✅ 能 |
| **firmware_new（修改前）** | 20ms 信号量超时 + **单次 AddTxMessage** | ❌ 卡死 |

#### 根本原因

**c69430c 的关键逻辑**（当前版本缺失）：
```cpp
while ((status = HAL_CAN_AddTxMessage(...)) != HAL_OK)
{
    if ((micros() - start) > 1000)   // 1ms 超时
    {
        canCtx->tx_busy_errors++;
        break;
    }
}
```

**拿到信号量后，循环重试 `HAL_CAN_AddTxMessage` 直到 mailbox 有空位**！

当电机端写 EEPROM 时（20-40ms）：
1. 电机端主循环卡在 Flash 擦除，无法响应 CAN
2. 主控 FixUpdate Task 每 5ms 发位置更新帧
3. **c69430c：mailbox 满时循环重试，等待电机端恢复**
4. **firmware_new：mailbox 满时立刻返回，不等待** → 后续帧积压 → 系统卡死

### 最终修复（改动 #5）

**文件**：`interface_can.cpp:319-336`

恢复 c69430c 的 mailbox 重试逻辑：

```cpp
if (sem_status == osOK) {
    // [v2026-10-04 止血改动#5] 恢复 c69430c 的重试逻辑：
    // mailbox 满时循环重试（最多 5ms），给电机端 EEPROM 写入留时间。
    // 不能用 micros()（未定义），改用 HAL_GetTick()。
    uint32_t start = HAL_GetTick();
    HAL_StatusTypeDef ret;
    while ((ret = HAL_CAN_AddTxMessage(
                canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox)) != HAL_OK)
    {
        if ((HAL_GetTick() - start) > 5)   // 5ms 超时
        {
            canCtx->tx_busy_errors++;
            break;
        }
    }
    
    // 加固#3：失败释放（防 c69430c 信号量泄漏）
    if (ret != HAL_OK) {
        osSemaphoreRelease(sem);
    }
}
```

### 第二次测试发现两个独立问题（17:30）

用户反馈：
```
不带 & → RGB/OLED 卡死，但串口活着（能收发消息）
带 &   → RGB/OLED 卡死，串口也卡死（Write timeout）
```

#### 错误假设 A：RGB 每个 LED 都调用 DMA（已证伪）

**假设**：`RainBow()` 对 12 个 LED 分别调用 `WS2812_Send()`，12 次 DMA 导致状态混乱。

**改动 #6**：先设置所有 LED，最后一次性发送（12 次 DMA → 1 次 DMA）

**测试结果**：❌ **问题依然存在，假设错误**

---

#### 错误假设 B：mailbox 重试时间不够（已证伪）

**假设**：EEPROM Flash 擦除 20-40ms，但 mailbox 重试只等 5ms 不够。

**改动 #5**：延长重试到 50ms（裸循环）

**测试结果**：❌ **引入新问题（Realtime 任务饿死），原问题仍存在**

---

#### 错误假设 C：Realtime 任务裸循环饿死（部分正确，但不是根因）

**假设**：50ms 裸循环让 FixUpdate (Realtime 优先级) 饿死 RGB/OLED/USB 任务。

**改动 #7**：重试时用 `osDelay(1)` 让出 CPU

**测试结果**：⚠️ **部分改善（`#ACC_J & ` 第一次不卡），但第二次仍卡死**

**结论**：这不是根本原因，问题更深层。

### 所有改动汇总（7 处）

| # | 文件 | 行号 | 改动内容 | 实际效果 |
|---|---|---|---|---|
| 1 | `interface_can.cpp` | 125-136 | abort 回调补信号量释放 | ✅ 防泄漏（加固性改动） |
| 2 | `interface_can.cpp` | 317 | `osWaitForever` → 20ms 超时 | ✅ 防永久卡死（加固性改动） |
| 3 | `rgb.hpp` | 110 | DMA 等待加 10ms 超时 | ❌ 未解决问题 |
| 4 | `rgb.hpp` | 115-118 | 超时后停止 DMA 并重置标志 | ❌ 未解决问题 |
| 5 | `interface_can.cpp` | 319-336 | mailbox 重试循环（50ms 裸循环） | ❌ **引入新问题：Realtime 任务饿死** |
| 6 | `rgb.hpp` | 124-152 | 所有 LED 设置完后一次性发送 | ❌ 未解决问题 |
| 7 | `interface_can.cpp` | 319-340 | mailbox 重试用 osDelay(1) | ⚠️ **部分改善（第一次不卡），但仍卡死** |

**结论**：所有尝试均**未能解决根本问题**。

## 17:44 — 改动 #7 仍未解决问题（暂停排查）

### 最后一次测试结果

```
[17:42:19] [TX] #ACC_J 1 150.0          → RGB/OLED 卡死，第二次 timeout
[17:42:32] [TX] #I_LIMIT_J 1 1.5        → RGB/OLED 卡死，但能继续发送
[17:42:41] [TX] #ACC_J 1 150.0 &        → 第一次正常！
[17:42:45] [TX] #ACC_J 1 150.0 &        → 第二次 RGB/OLED 卡死
[17:42:55] [TX] #I_LIMIT_J 1 1.5 &      → RGB/OLED 卡死，第二次 timeout
```

### 关键发现

1. **`#ACC_J 1 150.0 &` 第一次没有卡死**（之前所有测试都是第一次就卡）
   - 说明 `osDelay(1)` 让出 CPU 有部分效果
   - 但第二次仍然卡死 → 问题更深层

2. **`#I_LIMIT_J` 不带 `&` 能持续发送**
   - 带 `&` 第一次就卡死串口

3. **所有指令第一次都会卡死 RGB/OLED**
   - 即使不带 `&`（不写 EEPROM）也卡
   - 说明问题不仅仅是 EEPROM 写入耗时

### 当前状态

**已执行的改动（7 处）**：

| # | 文件 | 改动内容 | 效果 |
|---|---|---|---|
| 1 | `interface_can.cpp` | abort 回调补信号量释放 | ✅ 防泄漏 |
| 2 | `interface_can.cpp` | `osWaitForever` → 20ms 超时 | ✅ 防永久卡死 |
| 3 | `rgb.hpp` | DMA 等待加 10ms 超时 | ⚠️ 未生效 |
| 4 | `rgb.hpp` | 超时后停止 DMA 并重置 | ⚠️ 未生效 |
| 5 | `interface_can.cpp` | mailbox 重试 50ms（裸循环） | ❌ 导致饿死 |
| 6 | `rgb.hpp` | RainBow 一次性发送 | ⚠️ 未解决根本问题 |
| **7** | **`interface_can.cpp`** | **mailbox 重试用 osDelay(1)** | **⚠️ 部分改善，但仍卡死** |

**问题依然存在**：
- RGB/OLED 第一次指令后立刻卡死
- `#ACC_J` 第二次发送 timeout
- 带 `&` 保存时更容易卡死串口

### 暂停原因

1. **多次尝试未能解决根本问题**
2. **需要更深入的硬件调试**：
   - 示波器抓 CAN 总线波形
   - JTAG 断点调试电机端 `0x14` 处理逻辑
   - 测量 EEPROM 写入实际耗时
   - 监控 FreeRTOS 任务切换

3. **可能的深层原因**：
   - 电机端 `SetVelocityAcc` 触发的副作用（轨迹重计算？）
   - CAN 硬件 FIFO 溢出未正确清理
   - 某个中断优先级配置错误
   - DMA 与 CAN 中断冲突

### 建议后续方向

1. **对比 c69430c 版本的完整差异**（不只是 `CanSendMessage`）
2. **回退到 c69430c 验证功能正常**
3. **逐个 cherry-pick 新功能，找出引入问题的 commit**
4. **使用 JTAG 实时监控任务状态和 CAN 统计**

### 未 commit（按项目规则）

所有改动均未提交，等待用户决策下一步行动。

---

## 未完成

- RGB/OLED 卡死问题未解决
- 保存参数卡死串口问题未解决
- 需要硬件调试或回退对比

### 问题根因

**用户测试揭露的现象**：
```
#ACC_J 1 22.5      → 第1次就卡死 RGB/OLED + 串口
#I_LIMIT_J 1 1.5   → 能撑4-5次才卡死
```

**真正的根因**：
1. FixUpdate Task 是 **Realtime 优先级**（最高）
2. 改动 #5 加入了 **50ms 裸循环**重试 `HAL_CAN_AddTxMessage`：
   ```cpp
   while ((ret = HAL_CAN_AddTxMessage(...)) != HAL_OK) {
       if ((HAL_GetTick() - start) > 50) break;  // 裸循环不让出 CPU！
   }
   ```
3. 当 mailbox 满时（电机端处理 `0x14` 的 `SetVelocityAcc` 或写 EEPROM）：
   - **FixUpdate 在这 50ms 里持续自旋，不让出 CPU**
   - **Realtime 优先级持续抢占所有 Normal 优先级任务**
   - **RGB Task / OLED Task / USB Task 被饿死**
4. FixUpdate 每 5ms 发 7 帧（J1-J6 + 地轨），最坏 **7×50ms = 350ms 饿死时间**

**为什么 `#ACC_J` 第一次就卡死**：
- `0x14` 调用 `SetVelocityAcc` 修改了 `motionPlanner` 状态
- 后续 FixUpdate 发送的控制帧（`0x07`）触发电机端重新计算轨迹
- 计算耗时导致 CAN 应答延迟 → mailbox 积压 → FixUpdate 进入 50ms 自旋
- 第一次就触发饿死

**为什么 `#I_LIMIT_J` 能撑多次**：
- `0x12` 只修改电流限制，不触发轨迹重计算
- mailbox 不容易积压，偶尔才触发重试

### 改动 #7 代码（interface_can.cpp:319-340）

```cpp
if (sem_status == osOK) {
    // [v2026-10-04 止血改动#5/#7] mailbox 重试必须让出 CPU：
    // 原 50ms 裸循环会让 Realtime 优先级的 FixUpdate 饿死 Normal 优先级的 RGB/OLED。
    // 改为：重试时 osDelay(1) 让出时间片，最多重试 50 次（50ms）。
    uint32_t retryCount = 0;
    HAL_StatusTypeDef ret;
    while ((ret = HAL_CAN_AddTxMessage(
                canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox)) != HAL_OK)
    {
        if (++retryCount > 50) {  // 50 次 × 1ms = 50ms
            canCtx->tx_busy_errors++;
            break;
        }
        osDelay(1);  // 让出 CPU 给低优先级任务（RGB/OLED/USB）
    }
    
    if (ret != HAL_OK) {
        osSemaphoreRelease(sem);  // 失败释放信号量
    }
}
```

### 改动 #6 代码（rgb.hpp RainBow）

先设置所有 LED，最后一次发送（12 次 DMA → 1 次 DMA）。

### 编译产物
- `build/Core-STM32F4-fw.bin` / `.hex` / `.elf` 已更新（FLASH 380464 B）

### 测试结果（17:42）

❌ **问题依然存在**：

```
[17:42:19] [TX] #ACC_J 1 150.0          → RGB/OLED 卡死，第二次 timeout
[17:42:32] [TX] #I_LIMIT_J 1 1.5        → RGB/OLED 卡死，但能继续发送
[17:42:41] [TX] #ACC_J 1 150.0 &        → 第一次正常！← 唯一改善
[17:42:45] [TX] #ACC_J 1 150.0 &        → 第二次 RGB/OLED 卡死 + timeout
[17:42:55] [TX] #I_LIMIT_J 1 1.5 &      → RGB/OLED 卡死 + timeout
```

**实际效果**：
- ⚠️ `#ACC_J &` 第一次不卡死了（轻微改善）
- ❌ 第二次仍然卡死 RGB/OLED 和串口
- ❌ 所有指令仍会卡死 RGB/OLED
- ❌ 带 `&` 仍容易卡死串口

**结论**：`osDelay(1)` 有轻微改善，但**未解决根本问题**。

