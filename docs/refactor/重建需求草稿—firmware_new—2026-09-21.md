# `firmware_new` 重建需求草稿


> **原则**：不 cherry-pick `firmware/` 代码；`firmware/` 只作为**算法参考**，按需求重新实现。
> **创建日期**：2026-09-21

---

## 一、整体方向（已确认）

- 基线来源：`firmware_old/`（35 / 42 电机 + 主控，3 个工程）
- 不复用 `firmware/` 代码，只看思路自己写
- 工作目录：`firmware_new/`（用户自行创建）
- 节奏 / 分支：待定

---

## 二、复用边界

### 2.1 直接搬过来用（底座）

- CAN 协议：命令码 0x01~0x7F、帧格式（8 字节 data）
- ASCII 命令字符串：`!START` / `!HOME` / `>j1,j2,...`
- 单位约定：速度 r/s、加速度 r/s²、地轨 mm、夹爪 0~100%、电流 A
- STM32 HAL 驱动、FreeRTOS 内核
- 通用 Bsp 层：pwm / encoder / analog / timer
- 6DOF 正逆解算（`6dof_kinematic.cpp`）
- EEPROM 模拟层
- CubeMX 配置（`Ctrl-Step-fw.xml` / `REF-STM32F4-fw.ioc`）

### 2.2 看 `firmware/` 思路，自己写

- 7 轴同步抵达 `ComputeSyncSpeeds()`
- 5kHz 实时环架构（TIM7 + Realtime 任务）
- `ServoJ()` / `TUNING()` / `MoveL()` 等算法
- CAN 回包合并打印思路
- 详见下面的需求清单

### 2.3 不复用（重新设计 / 放弃）

- 堵转检测 v3 协议（0x5A / 0x5B / 0x5C）
- `!STALL_*` 命令（全套）
- 57 电机的 `CalcSoftGoalWithRealPos`
- gripper 夹取力限制 / P0-10 测试夹具
- 极光 RGB / 预设槽位

**借鉴**：堵转检测的**状态机思路**（检测 → 回退 → 锁定 → 解锁）搬到 `!BACKHOME`。

---

## 三、需求清单（待填）

### 3.1 电机固件需求（STM32F103）

按优先级填，**P0 = 必须先做**。

- 【P0】单一 build 系统：一套源码，`MOTOR_TYPE` 切 35 / 42 / 57 / gripper，CMake 出 4 套产物
- 【P0】CAN ID 硬编码：57=【填】、夹爪=【填】
- 【P0】电机端 `0x15 Apply Home` v3.2 修复（同步 `realPosition`）
- 【P0】电机端 `0x09` 急停广播（替代旧 0x89 bug）
- 【P1】电机端 `0x12` 电流限幅 + 持久化
- 【P1】电机端 `0x14` 加速度 + 持久化
- 【P1】电机端 `0x17` ~ `0x1A` DCE Kp/Kv/Ki/Kd + 持久化
- 【P1】电机端 `0x28` ~ `0x2B` DCE Kp/Kv/Ki/Kd 查询
- 【P1】电机端 `0x2C` 加速度查询
- 【P1】电机端 `0x2D` 电流限幅查询

### 3.2 主控固件需求（STM32F405）

- 【P0】`motorJ[7]` 8-slot（+地轨，reduction=1 直连）
- 【P0】单位统一（v2.6 重构）
- 【P0】`EmergencyStop` 走 CAN 0x09 急停
- 【P1】`MoveJ` 支持地轨（8 字段：j1~j6 + j7_mm + speed）
- 【P1】`MoveL` 含地轨算法
- 【P1】7 轴同步抵达
- 【P1】`SERVO_J`（5kHz 随动）
- 【P1】`TORQUE_CONTROL`（`$` 透传 7 通道电流）
- 【P1】`TUNING`（扫频正弦）
- 【P1】5kHz 实时环（TIM7 + Realtime 任务）
- 【P1】ALL 代理广播（`motorJ[0]`）
- 【P1】!HAND_* 夹爪命令
- 【P1】`#OFFSET_J` 替代 `!CALIBRATION`
- 【P2】`!BACKHOME` 统一回位（先 `!HAND_C` 再 `!HOME`，**借堵转检测状态机思路**）
- 【P2】Homing / Resting 30s 超时兜底
- 【填】【P?】____________________________
- 【填】【P?】____________________________

### 3.3 协议层需求

- 【P1】CAN 回包路由扩展（旧 2 条 → 新 ~12 条）
- 【P1】ASCII 基础命令：`!START` / `!DISABLE` / `!HOME` / `!RESET` / `!STOP`
- 【P1】ASCII 调参命令：`#ACC_J` / `#SPEED_J` / `#I_LIMIT_J` / `#GETJACC` / `#GETI` / `#SET_PID` / `#GET_PID`
- 【P1】ASCII 夹爪命令：`!HAND_ZERO` / `!HAND_EN` / `!HAND_DIS` / `!HAND_POS` / `!HAND_O` / `!HAND_C`
- 【P1】ASCII 归零：`#OFFSET_J`
- 【填】【P?】____________________________

### 3.4 FreeRTOS 架构需求

- 【P1】`ControlLoopFixUpdateTask`（Realtime 优先级，5kHz）
- 【P1】`ControlLoopUpdateTask`（Normal，事件触发）
- 【P1】`OledTask` / `RGBTask`
- 【P2】新增 `commTask` / `UsbServerTask` / `UartServerTask`
- 【填】【P?】____________________________

### 3.5 RGB / UI 需求

- 【P2】10 种灯效（RAINBOW / BLUE_TIDE / WHITE_BREATH 等）
- 【P2】静态色自定义 `!RGB_COLOR idx r g b`
- 【P2】亮度查询 / 设置 / 持久化 `!RGB_BRIGHT`
- 【P2】亮度渐变
- 【P2】DMA 非阻塞（首帧豁免 + WS2812 保护）
- 【填】【P?】____________________________

### 3.6 辅助 / 文档需求

- 【填】____________________________
- 【填】____________________________

---

## 四、默认数值（参考重构需求 §1.1.2 第 51-67 行）

> **填法**：把你想要的数值填进去；不要的字段可以删。

### 4.1 4 个 build 的差异参数

| 参数 | 35 | 42 | 57 | gripper | 说明（哪行参考） |
|---|---|---|---|---|---|
| `canNodeId` | 拨码 1~6 | 拨码 1~6 | 【7】 | 【8】 | CAN 节点 ID |
| `currentLimit` (mA) | 【填】 | 【填】 | 【填】 | 【填】 | FOC 输出硬限幅 |
| `calibrationCurrent` (mA) | 【填】 | 【填】 | 【填】 | 【填】 | 编码器校准电流 |
| `dce_kp` | 【填】 | 【填】 | 【填】 | 【填】 | DCE 位置比例增益 |
| `dce_kv` | 【填】 | 【填】 | 【填】 | 【填】 | DCE 速度比例增益 |
| `dce_ki` | 【填】 | 【填】 | 【填】 | 【填】 | DCE 积分增益 |
| `dce_kd` | 【填】 | 【填】 | 【填】 | 【填】 | DCE 微分增益 |
| `pid_kp` | 【填】 | 【填】 | 【填】 | 【填】 | 电流/速度环 PID Kp |
| `pid_ki` | 【填】 | 【填】 | 【填】 | 【填】 | 电流/速度环 PID Ki |
| `pid_kd` | 【填】 | 【填】 | 【填】 | 【填】 | 电流/速度环 PID Kd |
| `ratedCurrentAcc` (mA/s) | 【填】 | 【填】 | 【填】 | 【填】 | 电流爬升斜率 dI/dt |
| `velocityLimit` (r/s) | 【填】 | 【填】 | 【填】 | 【填】 | 速度上限 |
| `velocityAcc` (r/s²) | 【填】 | 【填】 | 【填】 | 【填】 | 加速度 |

### 4.2 4 个 build 共用默认

| 参数 | 值 | 说明 |
|---|---|---|
| `MOTOR_ONE_CIRCLE_HARD_STEPS` | 【填】 | 1.8° 步距 |
| `SOFT_DIVIDE_NUM` | 【填】 | 微步细分 |
| `MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS` | 【填】 | = HARD × SOFT |
| `velocityLimit` 兜底 | 【填】 | |
| `velocityAcc` 兜底 | 【填】 | |
| `encoderHomeOffset` | 【填】 | 归零偏移 |
| `defaultMode` | 【填】 | 默认上电模式 |
| `enableMotorOnBoot` | 【填】 | 上电自启 |
| `enableTempWatch` | 【填】 | 1Hz 温度采集 |

### 4.3 主控侧单位与常量

| 常量 | 值 | 说明 |
|---|---|---|
| `RAIL_STEPS_PER_MM` | 【填】 | 丝杆 1605 直连（200 步 × 微步 / 5mm） |
| `SLIDER_TO_RPS` | 【填】 | slider 100 → ? r/s |
| `AXIS_MAX_RPS[8]` | 【填】 | 8 轴物理上限 r/s |
| `MOTOR_REDUCTION[8]` | 【填】 | 8 轴减速比（J7=地轨=1 直连） |
| `REST_POSE` (J1~J6) | 【填】 | 待机位姿 |
| `HOME_POSE` (J1~J6) | 【填】 | 归零位姿 |
| `DEFAULT_JOINT_SPEED` (slider) | 【填】 | 默认关节速度 |
| `DEFAULT_JOINT_ACCELERATION` (r/s²) | 【填】 | 默认关节加速度 |
| `ALL_FINISH_FLAG` | 【填】 | jointsStateFlag 全置位掩码 |
| `DH` 参数（L1, D1, L2...） | 【填】 | 新机械臂 DH |

---

## 五、不做的事情

- ❌ 堵转检测 v3 协议（`0x5A` / `0x5B` / `0x5C`、`stallMode` 状态机）
- ❌ `!STALL_EN` / `!STALL_DIS` / `!STALL_STATUS` / `!STALL_UNLOCK`（全删 / 留 no-op？见 §六.1）
- ❌ 57 电机的 `CalcSoftGoalWithRealPos`（反闭环补偿）
- ❌ gripper 夹取力限制
- ❌ gripper P0-10 测试夹具函数（`Stockpile_*`）
- ❌ 极光（Aurora）RGB / 预设槽位
- ❌ v2.6 重构叠加出来的过度设计（如 `acceleration * 0.3` 这类经验调参）
- 【填】____________________________
- 【填】____________________________

**但要做**：
- ✅ 把堵转检测的**状态机思路**（检测 → 回退 → 锁定 → 解锁）搬到 `!BACKHOME`
  - 仅借状态机思路
  - 参数（回退距离 / 速度）独立设计

---

## 六、待确认决策

| # | 问题 | 倾向 | 备注 |
|---|---|---|---|
| 1 | `!STALL_*` 命令彻底删除，还是保留 no-op 占位？ | 【填】 | 文档建议完全删除（保持接口简洁） |
| 2 | `MoveL` 含地轨的具体策略？（地轨与关节同时 / 地轨先到位再关节 / 关节先到位再地轨） | 【填】 | 实施时再设计 |
| 3 | `!BACKHOME` 是否可配置顺序？（默认固定：先 `!HAND_C` 再 `!HOME`） | 【填】 | |
| 4 | 5kHz 实时环是否支持全部 6 种 CommandMode？ | 【填】 | 文档建议是 |
| 5 | `!BACKHOME` 借鉴堵转的程度（仅状态机 / 含回退参数）？ | 【填】 | 文档建议仅借状态机 |
| 6 | `firmware_new/` 与 `firmware_old/` 在 git 中如何处理？ | 【填】 | |
| 7 | 【填】____________________________ | | |
| 8 | 【填】____________________________ | | |

---

## 七、节奏 / 分支（待定）

| 项 | 选项 | 选择 |
|---|---|---|
| 节奏 | P0→P1→P2→P3 渐进 / 一次性大改 / 其他 | 【填】 |
| 分支 | 新建 `feat/firmware_new` / 主干直接改 / 其他 | 【填】 |
| 提交粒度 | 每个子功能 1 commit / 每个阶段 1 commit / 其他 | 【填】 |
| 验证方式 | 每次编译通过 / 实机烧录测试 / 其他 | 【填】 |

---

## 八、参考文档（已存在）

- [`重构需求—firmware_old迁移到firmware功能清单—2026-09-19.md`](./重构需求—firmware_old迁移到firmware功能清单—2026-09-19.md) — 功能清单（细）
- [`实施方案—重构需求—2026-09-19.md`](./实施方案—重构需求—2026-09-19.md) — 6 阶段路径（细）
- [`新功能清单—firmware与firmware_old对比—2026-09-18.md`](./新功能清单—firmware与firmware_old对比—2026-09-18.md) — 详细对比 + 证据
- [`主控固件对比—firmware_old迁移到firmware—2026-09-21.md`](./主控固件对比—firmware_old迁移到firmware—2026-09-21.md) — 主控侧 diff
- [`电机固件对比—firmware_old迁移到firmware—2026-09-21.md`](./电机固件对比—firmware_old迁移到firmware—2026-09-21.md) — 电机侧 diff
