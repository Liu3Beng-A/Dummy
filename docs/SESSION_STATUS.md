# SESSION_STATUS — J2-J6 卡顿诊断

更新时间：2026-10-01 10:35

## 当前问题
J2-J6 在手动掰动时（idle 状态）OLED 角度刷新卡顿；MoveJ 期间正常。

## 已确认事实（基于用户提供的 `#GETSTAT` 数据 + 2026-10-01 重分析）

### 1. ACK 增速异常 — 关键证据
- 测试时间：09:30:17 ~ 09:31:48（idle，dummy disabled，UpdateJointAngles 每 20ms 一次）
- `TX0` 涨速 ≈ 50/s ✓ 完美匹配 UpdateJointAngles 50Hz 频率
- **`ACK` 涨速 ≈ 100/s** ← 异常
- 期望 ACK 增速：50Hz × 6电机 = **300/s**
- 实际 ACK 增速：**100/s = 50Hz × 2**
- **结论：6 个电机中只有 2 个在回 0x23 ACK**

### 2. 主控 TX 路径完全健康
- `RY0/1/2 = 0` — manual retry 没触发
- `BE = 0` — HAL_CAN_AddTxMessage 无 1ms 超时
- `AB = 0` — 无 mb abort
- `T07 = 0` — 测试期没发 0x07
- `FF = 0` — RX FIFO 未满
- `UE = UN = 0` — 无 unexpected / unhandled

### 3. 关键代码确认（电机端 `interface_can.cpp`）
- `case 0x23` 处理函数里电机**会**回 0x23 ACK（line:218-227）
- 但**电机端 CAN_Send 无 retry**（`Core/Src/can.c:181`）：HAL_CAN_AddTxMessage 失败立即 Error_Handler
- `HAL_CAN_RxFifo0MsgPendingCallback` 只在 `id==0 || id==boardConfig.canNodeId` 时调 OnCanCmd（can.c:194）

### 4. MoveJ 期间为什么 J2-J6 不卡（2026-10-01 重分析）
- 0x07 是**地址化命令**，每条只发给一个电机
- 主控每 5ms 按 J1→J2→…→J6 顺序发 6 条 0x07，每条 111µs，**6 条在 ~666µs 内陆续到达 6 个电机**
- 每个电机**只 ACK 自己收到的 0x07**，6 条 ACK 在总线上天然错开 ~111µs
- **没有冲突** → 每电机稳定 200Hz ACK → OLED 50Hz 渲染完全平滑

### 5. idle 期间为什么 J2-J6 卡（2026-10-01 重分析）
- ALL 0x23 是**广播命令**，6 个电机全部响应
- 6 条 ACK 在总线上**几乎同时启动仲裁**，J1 的 StdId 最小（0xA3）优先级最高，**永远赢**
- J2-J6 输仲裁后被 bxCAN 标记为"已发但不重试"（AutoRetransmission=DISABLE），**帧被永久丢弃**
- 实测 ACK = 2×TX0：J1 必赢 + 1 个 J2-J6 随机赢（与 ISR 响应延迟抖动有关），其余 4 条丢
- J2-J6 每电机 ACK 频率 ≈ 50Hz × 1/5 = 10Hz，且分布不均 → 间歇性 1-2s 刷新

## 已排除方向

### ✗ bxCAN manual retry 路径有问题
- `RY*=0` 证实 ALST retry 没触发

### ✗ 主控 RX FIFO 溢出
- `FF=0`，`UE=0`

### ✗ 主控 HAL_CAN_AddTxMessage 1ms 超时
- `BE=0`

### ✗ §8.9 bxCAN 3 mb 等待是 idle 卡顿的根因（2026-10-01 修订）
- 之前假设 "bxCAN 3 mb 让 J4-J6 的 0x07 永远 HAL_BUSY" 被证伪：
  - `TxAlstRetryCnt[0..2]=0`：mb 等待逻辑生效后 0x07 全部入 mb 成功
  - MoveJ 期间 6 关节全顺（用户实测）
- §8.9 实际只修 MoveJ 路径的 0x07 入队，**与 idle 路径无关**

### ✗ ALL 0x23 兜底"命中空 mb 窗口"模型
- `ACK=2×TX0` 稳定，无法用"偶尔命中"解释
- 实际是广播应答撞车的确定性丢帧

## 决策状态（2026-10-01）

### ✅ 已敲定
- **§8.1 OLED 50Hz**：保留
- **§8.2 / §8.8 查询 50Hz 节奏**：保留（§8.10 落地后重新定义为"每 50ms 跑一轮 6 条个体查询"）
- **§8.6 HAL 返回值检查**：保留
- **§8.7 RX FIFO 锁定**：保留
- **§8.9 bxCAN 3 mb 等待**：保留（仅作为 MoveJ 路径防御性保护，是否仍需撤回后续讨论）
- **§8.10 6 条个体查询替代 ALL 广播**：已敲定（2026-10-01），待代码落地

### 🟡 待用户确认后落地
- §8.10 代码改动（仅 `dummy_robot.cpp:161-164`）

### 🔵 后续讨论
- §8.10 落地后评估 §8.9 是否仍需保留
- §8.5 电机端应答错开是否重新启用（如果想进一步降总线占用率）

## 最新改动

### 2026-10-01 §8.10 落地
- `firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:160-163`
  - `UpdateJointAngles()`：`motorJ[ALL]->UpdateAngle()` → 6 条个体查询循环
  - 改动 4 行，编译通过 12/12，零 warning（除 STM32 标准 RWX linker 提示）
- §8.10 决策状态：从"待用户确认后落地"升级为"已落地，待硬件验证"

### 2026-10-01 §9.0 诊断计数器
- `Bsp/communication/interface_can.hpp` — 加 3 字段 + 2 extern
- `Bsp/communication/interface_can.cpp` — 3 处累加点
- `UserApp/protocols/can_protocol.cpp` — case 0x23 启用 received_ack
- `UserApp/protocols/ascii_protocol.cpp` — USB/UART4 加 #GETSTAT 命令

### 2026-10-01 文档修订
- `docs/CAN角度更新机制与J2-J6卡顿问题分析.md`：
  - §5.4 整段重写（MoveJ vs idle 路径对比）
  - §6 总结表更新（4 行）
  - §8.9 决策描述精简 + 末尾加"2026-10-01 修订说明"
  - §8.10 新决策（6 条个体查询）
  - §9 待评估方向更新

## 测试日志

[09:29:46] [INFO] 成功连接 COM20 @ 115200
[09:30:17] [TX] #GETSTAT
[09:30:17] [RX] ok TX0=2877 TX1=0 TX2=0 RY0=0 RY1=0 RY2=0 T07=0 ACK=5742
[09:30:17] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:30:29] [TX] #GETSTAT
[09:30:29] [RX] ok TX0=3483 TX1=0 TX2=0 RY0=0 RY1=0 RY2=0 T07=0 ACK=6954
[09:30:29] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:30:36] [TX] #GETSTAT
[09:30:36] [RX] ok TX0=3840 TX1=0 TX2=0 RY0=0 RY1=0 RY2=0 T07=0 ACK=7668
[09:30:36] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:31:32] [TX] #GETSTAT
[09:31:32] [RX] ok TX0=6623 TX1=0 TX2=0 RY0=0 RY1=0 RY2=0 T07=0 ACK=13234
[09:31:32] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:31:48] [TX] #GETSTAT
[09:31:48] [RX] ok TX0=7427 TX1=0 TX2=0 RY0=0 RY1=0 RY2=0 T07=0 ACK=14842
[09:31:48] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:32:20] [TX] !START
[09:32:20] [RX] Started ok
[09:32:26] [TX] !DISABLE
[09:32:26] [RX] Disabled ok
[09:32:29] [TX] !START
[09:32:29] [RX] Started ok
[09:32:41] [TX] >90,0,90,90,90,90,30
[09:32:41] [RX] ok
[09:32:49] [INFO] 已存储: #1 [90.0,0.0,90.0,90.0,90.0]
[09:33:11] [TX] >0,-73,180,0,0,0,30
[09:33:11] [RX] 15ok
[09:33:20] [INFO] 已存储: #2 [0.0,-73.0,180.0,0.0,0.0,0.0]
[09:33:24] [TX] >90,0,90,90,90,90,30
[09:33:24] [RX] ok
[09:33:32] [TX] >0,-73,180,0,0,0,30
[09:33:32] [RX] 15ok
[09:33:44] [TX] >90,0,90,90,90,90,30
[09:33:44] [RX] 15ok
[09:33:54] [TX] >0,-73,180,0,0,0,30
[09:33:54] [RX] 15ok
[09:33:55] [TX] #GETSTAT
[09:33:55] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:34:02] [TX] >90,0,90,90,90,90,30
[09:34:02] [RX] 15ok
[09:34:11] [TX] >0,-73,180,0,0,0,30
[09:34:11] [RX] 15ok
[09:34:20] [TX] !DISABLE
[09:34:20] [RX] Disabled ok


以下是日志记录
[09:29:46] [INFO] 成功连接 COM20 @ 115200
[09:30:17] [TX] #GETSTAT
[09:30:17] [RX] ok TX0=2877 TX1=0 TX2=0 RY0=0 RY1=0 RY2=0 T07=0 ACK=5742
[09:30:17] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:30:29] [TX] #GETSTAT
[09:30:29] [RX] ok TX0=3483 TX1=0 TX2=0 RY0=0 RY1=0 RY2=0 T07=0 ACK=6954
[09:30:29] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:30:36] [TX] #GETSTAT
[09:30:36] [RX] ok TX0=3840 TX1=0 TX2=0 RY0=0 RY1=0 RY2=0 T07=0 ACK=7668
[09:30:36] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:31:32] [TX] #GETSTAT
[09:31:32] [RX] ok TX0=6623 TX1=0 TX2=0 RY0=0 RY1=0 RY2=0 T07=0 ACK=13234
[09:31:32] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:31:48] [TX] #GETSTAT
[09:31:48] [RX] ok TX0=7427 TX1=0 TX2=0 RY0=0 RY1=0 RY2=0 T07=0 ACK=14842
[09:31:48] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:32:20] [TX] !START
[09:32:20] [RX] Started ok
[09:32:26] [TX] !DISABLE
[09:32:26] [RX] Disabled ok
[09:32:29] [TX] !START
[09:32:29] [RX] Started ok
[09:32:41] [TX] >90,0,90,90,90,90,30
[09:32:41] [RX] ok
[09:32:49] [INFO] 已存储: #1 [90.0,0.0,90.0,90.0,90.0,90.0]
[09:33:11] [TX] >0,-73,180,0,0,0,30
[09:33:11] [RX] 15ok
[09:33:20] [INFO] 已存储: #2 [0.0,-73.0,180.0,0.0,0.0,0.0]
[09:33:24] [TX] >90,0,90,90,90,90,30
[09:33:24] [RX] ok
[09:33:32] [TX] >0,-73,180,0,0,0,30
[09:33:32] [RX] 15ok
[09:33:44] [TX] >90,0,90,90,90,90,30
[09:33:44] [RX] 15ok
[09:33:54] [TX] >0,-73,180,0,0,0,30
[09:33:54] [RX] 15ok
[09:33:55] [TX] #GETSTAT
[09:33:55] [RX] BE=0 FF=0 AB=0 UE=0 UN=0
[09:34:02] [TX] >90,0,90,90,90,90,30
[09:34:02] [RX] 15ok
[09:34:11] [TX] >0,-73,180,0,0,0,30
[09:34:11] [RX] 15ok
[09:34:20] [TX] !DISABLE
[09:34:20] [RX] Disabled ok