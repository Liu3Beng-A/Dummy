# 重构需求 — `firmware_old` → `firmware` 新功能需求清单

> **真实重构动机（务必先看）**：`firmware/` 在多轮迭代中累积了过多 BUG（典型如 0x89 协议编码错误、OLED 屏幕退化、CAN 总线拥堵、堵转误触发、EStop 失联等），且 BUG 之间高度耦合——修一个往往暴露另一个，继续在 `firmware/` 之上修 BUG 的边际成本**远超**重新实现。因此本文档虽名为"迁移清单"，但执行路径**不是** cherry-pick `firmware/` 代码，而是 **以 `firmware_old/` 为稳定基线，把 `firmware/` 中已验证有效的新功能按本文逐项重新实现**。
>
> **文档目的**：**只**列出需要新增的功能（每个功能用一句话说清"做什么"）。
> - 详细对比、证据、文件位置 → 见 [`新功能清单—firmware与firmware_old对比—2026-09-18.md`](./新功能清单—firmware与firmware_old对比—2026-09-18.md)
> - 分阶段实施步骤 → 见 [`实施方案—重构需求—2026-09-19.md`](./实施方案—重构需求—2026-09-19.md)
>
> **创建日期**：2026-09-19

---

## 一、需求清单

### 1.1 电机固件需求（STM32F103）

| 优先级 | 功能 | 一句话说明 |
|---|---|---|
| P0 | **单一 build 系统 + 4 套产物（ELF/HEX/BIN 各 4）** | 35 / 42 / 57 / gripper **共享一套源码**，通过 `MOTOR_TYPE` 宏切换默认参数（4 套独立的 PID + 电流默认值，即使数值相同也分开定义），CMake 生成 4 个 ELF，**每个 ELF 通过 `objcopy` 同时产出对应的 .hex 和 .bin**（共 12 个文件）。**电流限幅按电机型号分档**|
| P0 | **CAN ID 硬编码** | 地轨 = **7**，夹爪 = **8**（35 / 42 由拨码开关决定 1~6） |
| P1 | **单独查询/设置加速度** | 每个电机可被独立查询/设置 r/s² 加速度 |
| P1 | **单独查询/设置 PID** | 每个电机可被独立查询/设置 DCE Kp/Kv/Ki/Kd |
| P0 | **0x15 Apply Home v3.2 修复**（电机侧） | `ApplyPosAsHomeOffset()` 内部同步 `realPosition = encoderHomeOffset`，避免 `#OFFSET_J` 后 `GetPosition()` 返回累计圈数（mod 信息丢失） |
| ❌ | **放弃堵转检测 v3 协议** | 不迁移 0x5A/0x5B/0x5C；不迁移 `!STALL_EN/DIS/STATUS/UNLOCK`；不迁移 v3 复杂状态机。**保留"检测 → 回退 → 锁定 → 解锁"的逻辑思路**，用于 `!BACKHOME` 统一回位|

### 1.1.1 电机固件新增函数清单

> 按源码中的实际定义分组。**不列** firmware_old 中已有的旧函数（如 `SetPositionSetPoint()`、`CalcDceToOutput()` 等），只列 firmware 中新增或重构了核心逻辑的函数。

#### CAN 协议层（`interface_can.cpp` 中 `OnCanCmd()` 的 case 分支）

| 优先级 | 底层函数（`OnCanCmd` 调用） | 新增/重构 | 说明 |
|---|---|---|---|
| P1 | `SetPositionSetPoint()` + `ratedVelocity` 覆盖 → case `0x07` | **重构** | 给定目标位置 + 速度上限（代替自动选速） |
| P0 | case `0x09` | **新增**（替代旧 `0x89`） | 广播急停：所有电机切 STOP + 抱闸 + 零电流/速度 |
| P1 | case `0x17`/`0x18`/`0x19`/`0x1A` | **新增** | Set DCE Kp/Kv/Ki/Kd；写入 `boardConfig` 字段；commit 时持久化 EEPROM |
| P1 | case `0x28`/`0x29`/`0x2A`/`0x2B` | **新增** | Get DCE Kp/Kv/Ki/Kd；回读 `boardConfig` 中的值（反映 EEPROM 实际值） |
| P1 | case `0x2C` | **新增** | Get Acceleration；回 float r/s² |
| P1 | case `0x2D` | **新增** | Get Current-Limit；回 float A |

#### 电机控制层（`motor.cpp` 中 Controller 类方法）

| 优先级 | 函数签名 | 新增/重构 | 说明 |
|---|---|---|---|
| P1 | `SetPositionSetPointWithTime(int32_t _pos, float _time)` | **新增** | 内部计算 `pMax = acc·T²/4`；若 `|delta| > pMax` 返回 false（时间不够）；否则算出 `vMax` 写入 `ratedVelocity`，调用 `SetPositionSetPoint(_pos)` |
| P0 | `ApplyPosAsHomeOffset()` | **重构**（v3.2 修复核心） | 归零时同步 `realPosition = encoderHomeOffset`（修复：旧版 GetPosition() 返回累计圈数而非 0） |
| P2 | `ResetGoalsToCurrentPosition()` | **新增**（**仅 BACKHOME 用**） | 把 `goalPosition = estPosition`、清零 `goalVelocity/goalCurrent`、触发 `softNewCurve`；`!BACKHOME` 在机械臂卡阻回退时调用，避免电机被再次驱动到旧目标 |
| P2 | `ClearIntegral()` | **新增**（**仅 BACKHOME 用**） | 清零 `estVelocityIntegral` 和 `estError`；`!BACKHOME` 状态切换时调用，消除积分累积 |

### 1.1.2 4 套电机固件默认参数差异（`configurations.h` + `motor.h`）

> **目的**：实施时一眼看清要切哪些 `#if MOTOR_TYPE == ...` 块。**只有这 5 处不同**，其余全部共用。

| 参数 (括号内为字段作用) | 35（机械臂小电机） | 42（机械臂大电机） | 57（地轨） | gripper（夹爪） |
|---|---|---|---|---|
| **canNodeId**（CAN 节点 ID） | 拨码 1~6 | 拨码 1~6 | 硬编码 7 | 硬编码 8 |
| **currentLimit**（FOC 最终输出硬限幅；运行时字段名 ratedCurrent，由 #I_LIMIT_J / CAN 0x12 改写） | 2500 mA | 2500 mA | 3000 mA | 2500 mA |
| **calibrationCurrent**（编码器磁编零点校准时 FOC 输出的恒定电流） | 2000 mA | 2000 mA | 2000 mA | 2000 mA（与 35/42 一致） |
| **dce_kp**（DCE 位置比例增益） | 200 | 200 | 300 | 200 |
| **dce_kv**（DCE 速度比例增益） | 80 | 80 | 120 | 80 |
| **dce_ki**（DCE 积分增益） | 300 | 300 | 150 | 300 |
| **dce_kd**（DCE 微分增益） | 250 | 250 | 50 | 250 |
| **pid_kp**（电流/速度环 PID 比例增益，与 DCE 位置环区分） | 5 | 5 | **15** | 5 |
| **pid_ki**（电流/速度环 PID 积分增益） | 30 | 30 | **80** | 30 |
| **pid_kd**（电流/速度环 PID 微分增益） | 0 | 0 | **10** | 0 |
| **ratedCurrentAcc**（电流爬升斜率上限 dI/dt） | 2 × 1000 mA/s | 2 × 1000 mA/s | 50 × 1000 mA/s（50A/s, 0→2.8A 仅需 56ms, 地轨需快速响应） | 2 × 1000 mA/s |

**全部相同**（4 个 build 共用）：
- `MOTOR_ONE_CIRCLE_HARD_STEPS = 200`（1.8°步距）
- `SOFT_DIVIDE_NUM = 256`
- `MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = 200 × 256 = 51200`
- `velocityLimit = 30 × 51200 = 1.536M` step/s（30 r/s）
- `velocityAcc = 30 × 51200 = 1.536M` step/s²（30 r/s²；旧版 100 r/s²，重构时统一到 30 r/s²）
- `encoderHomeOffset = 0`
- `defaultMode = MODE_COMMAND_POSITION`
- `enableMotorOnBoot = false`
- `enableTempWatch = false`


### 1.2 主控固件需求（STM32F405）

| 优先级 | 功能 | 一句话说明 |
|---|---|---|
| P0 | **motorJ[] 8-slot 架构** | + 地轨 J7，reduction = 1 直连 |
| P0 | **单位统一** | 速度 r/s、加速度 r/s²、地轨 mm、夹爪 0~100%、电流 A |
| P0 | **EmergencyStop 0x09** | 修复急停：全节点 STOP + 抱闸（替代旧版错误的 0x89） |
| P1 | **MoveJ 支持地轨** | 入参扩展为 8 字段（j1~j6 + j7 mm + speed） |
| P1 | **MoveL 加入地轨算法** | 直线插补时考虑地轨位置变化（具体策略实施时再设计） |
| P1 | **7 轴同步抵达** | MoveJ 时 7 轴同时抵达目标 |
| P1 | **SERVO_J** | 5kHz 实时环随动（模式 6） |
| P1 | **TORQUE_CONTROL** | 7 通道电流透传（`$` 命令） |
| P1 | **TUNING** | 扫频正弦激励（模式 4） |
| P2 | **!BACKHOME 统一回位**（新功能） | 先 `!HAND_C`（夹爪闭合）再 `!HOME`（机械臂归位）。**可借鉴堵转检测的状态机思路**处理异常情况（机械臂卡阻时回退 → 锁定 → 解锁） |
| P2 | **Homing/Resting 30s 超时** | 避免主控永久等待 |

### 1.3 协议层需求

| 优先级 | 功能 | 一句话说明 |
|---|---|---|
| P1 | **CAN 回包路由扩展** | 从 2 条 cmd 扩展到 ~12 条（参数读写 + 堵转广播） |
| P1 | **ALL 代理广播** | `motorJ[0]`（nodeID=0）展开为所有节点 |
| P1 | **ASCII 命令集扩展** | 从基础 4 条扩展为完整指令集（系统控制 + 查询/参数 + 运动指令） |
| ⚠️ | **`!STALL_STATUS` 重构** | ⚠️ 既然放弃堵转检测，**此命令不再适用**——见 §五决策点 |

### 1.4 FreeRTOS 架构需求

| 优先级 | 功能 | 一句话说明 |
|---|---|---|
| P1 | **5kHz 实时环** | TIM7 200μs 中断 + Realtime 优先级任务 |
| P2 | **新增 3 个任务** | commTask（心跳）、UsbServerTask、UartServerTask |

### 1.5 UI / RGB 需求

| 优先级 | 功能 | 一句话说明 |
|---|---|---|
| P2 | **RGB 10 灯效** | 模式 0~9 + 99 (ALL_OFF)，含 RAINBOW / BLUE_TIDE / WHITE_BREATH / CYBER_BREATH / RED_HEARTBEAT / GREEN_SPIN / BLINK |
| P2 | **RGB 静态色自定义** | `!RGB_COLOR idx r g b` 配置 0~2 纯色 |
| P2 | **RGB 亮度查询/设置/持久化** | `!RGB_BRIGHT [<0-100>] [&]` |
| P2 | **RGB 亮度渐变开关** | `brightness` → `targetBrightness` 步进淡入淡出 |
| P2 | **RGB DMA 非阻塞** | 首帧豁免 + WS2812 保护 |
| ❌ | **不实现** | 极光（Aurora）、预设槽位 |

---

## 二、放弃与保留

| 项 | 决定 | 说明 |
|---|---|---|
| **电机 build 系统** | ✅ **合并为 1 套源码 + 4 套产物**（ELF/HEX/BIN 各 4） | 35 / 42 / 57 / gripper 通过 `MOTOR_TYPE` 宏切换；**4 套完全独立的默认参数块**（PID / 电流 / CAN ID），即使 35 和 gripper 数值相同也用独立 `#if MOTOR_TYPE == ...` 段；**电流按型号分档**（35/42=2.0A、57=2.5A、gripper=1.2A，详见 §1.1.1）；无任何条件编译代码（4 个 build 算法完全一致，仅默认参数不同）；**每个 ELF 通过 objcopy 同时产出同名 .hex 和 .bin**（用户偏好烧录 hex/bin，st-flash / openocd / 串口助手均直接可用） |
| **57 电机固件**（`CalcSoftGoalWithRealPos` 失步补偿） | ❌ **不实现** | 旧版固件（`firmware_old/`）**根本没有 57 电机固件**（只有 `dummy-35motor-fw` / `dummy-42motor-fw` / `dummy-ref-core-fw`）；`firmware/motor_fw_f103_57/` 是后建的草稿，`CalcSoftGoalWithRealPos` 是误植的"反闭环"补偿，**编码器已经在 FOC 内环工作**，再加一层补偿会引入抖动并破坏闭环正确性。57 固件直接复用 35/42/gripper 的标准轨迹规划算法 |
| **gripper 力限幅**（夹取力控制） | ❌ **不保留** | 用户决定：与 35/42/57 共用电流限幅即可，无需单独的夹取力控制逻辑 |
| **gripper P0-10 测试夹具函数**（`Stockpile_Flash_Data_Read_Data16` 等） | ❌ **不保留** | 出厂测试用，新固件无对应需求 |
| 堵转检测 v3 协议 | ❌ **放弃** | 不再迁移 0x5A/0x5B/0x5C；不再迁移 v3 复杂状态机；不再迁移 `!STALL_*` 命令；**所有电机都不需要堵转检测**（包括 gripper） |
| 堵转检测的**逻辑思路** | ✅ **保留并复用** | "检测 → 回退 → 锁定 → 解锁"用于 `!BACKHOME` 统一回位 |
| `!STALL_STATUS` | ⚠️ **重新评估** | 不再有堵转检测后，此命令需要重新设计——见 §五 |
| `!STALL_EN/DIS/UNLOCK` | ⚠️ **重新评估** | 失去依赖，建议**直接删除**——见 §五 |
| MoveL 地轨算法 | ✅ **必实现** | 用户明确要求（不是放弃项），具体策略实施时再设计 |
| OLED 退化修复 | ❌ **不在本文档** | 属于 BUG 修复，不在"新增功能"范围 |
| `!CALIBRATION` 删除 | ✅ **顺带处理** | 用 `#OFFSET_J` 替代 |

---

## 三、重构原则（精简）

| 原则 | 说明 |
|---|---|
| **接口对齐优先** | CAN 协议先行对齐，再迁移上层命令 |
| **数据结构先行** | 单位、节点表、常量先迁移，算法后迁移 |
| **独立子系统独立迁移** | RGB、夹爪可单独迁移 |
| **同步升级约束** | 主控发 0x09 EStop，电机必须能解析；任何带协议变更的功能必须主控 + 电机同步改 |
| **不沿用错误设计** | 堵转 v3 协议本身设计有缺陷，重新实现时不再沿用 |

---

## 四、实施顺序（按风险 + 价值）

> **详细分阶段步骤**见 [`实施方案—重构需求—2026-09-19.md`](./实施方案—重构需求—2026-09-19.md)。

### 第一批（P0 必修）

1. 单一 build 系统 + 4 个 ELF 产物（35 / 42 / 57 / gripper 共用源码 + `MOTOR_TYPE` 宏）
2. motorJ[7] 8-slot + 单位统一
3. CAN ID 硬编码（地轨 = 7、夹爪 = 8）
4. EmergencyStop 0x09

### 第二批（P1 高价值）

5. 单独查询/设置加速度、PID
6. 7 轴同步抵达
7. MoveJ 支持地轨
8. MoveL 加入地轨算法
9. SERVO_J + TORQUE_CONTROL + TUNING
10. 5kHz 实时环
11. ALL 代理广播
12. ASCII 命令集 - 夹爪部分（`!HAND_*`）+ 编码器归零（`#OFFSET_J`）

### 第三批（P2 独立子系统）

13. ASCII 命令集 - 调参部分（`#ACC_J` / `#SPEED_J` / `#I_LIMIT_J` / `#GETJACC` / `#GETI` / `#SET_PID` / `#GET_PID`）
14. ASCII 命令集 - RGB 部分（`!RGB_*` / `!RGB_BRIGHT` / `!RGB_COLOR`）
15. RGB 10 灯效 + 静态色 + 亮度 + 渐变 + DMA
16. Homing/Resting 30s 超时
17. !BACKHOME 统一回位（**借鉴堵转检测的状态机思路**）

### 第四批（P3 不实现 / 评估）

20. ⚠️ `!STALL_STATUS` / `!STALL_EN` / `!STALL_DIS` / `!STALL_UNLOCK` — 重新评估后决定（删 / 留 / 改）
21. ⚠️ 极光 RGB、预设槽位 — **不实现**

---

## 五、待确认决策点

| # | 问题 | 建议 |
|---|---|---|
| 1 | 既然放弃堵转检测，`!STALL_EN/DIS/STATUS/UNLOCK` 命令是否**完全删除**？还是保留命令但提示"未启用"？ | 🟡 完全删除（保持接口简洁） |
| 2 | MoveL 地轨算法的具体策略（地轨与关节同时移动 / 地轨先到位再关节 / 关节先到位再地轨）？ | 🟡 实施时由 agent 再分析 |
| 3 | `!BACKHOME` 的实现细节（先 `!HAND_C` 再 `!HOME`，还是可配置）？ | 🟡 默认顺序固定，保留可选参数 |
| 4 | 5kHz 实时环是否需要支持所有 6 种 CommandMode？ | 🟡 是 |
| 5 | `!BACKHOME` 借鉴堵转检测的具体程度（只借鉴状态机？还是包括回退距离/速度参数）？ | 🟡 只借鉴状态机思路，参数独立设计 |
| 6 | ASCII 命令集是完整迁移（927 行）还是按需迁移？ | 🟡 按需迁移，参考 §四 实施顺序 |

---

## 六、参考文档

| 文档 | 用途 |
|---|---|
| [`新功能清单—firmware与firmware_old对比—2026-09-18.md`](./新功能清单—firmware与firmware_old对比—2026-09-18.md) | 详细对比、证据、文件位置（**本文不重复此文档的内容**） |
| [`实施方案—重构需求—2026-09-19.md`](./实施方案—重构需求—2026-09-19.md) | 分阶段实施步骤 |
| `docs/TODO.md` | FEAT/BUG 待开发项 |
| `docs/README.md` | 项目主文档 |
