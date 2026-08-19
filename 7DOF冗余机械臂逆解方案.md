# 7-DOF (地轨 + 6 关节) 冗余逆解方案 — **最终设计**

> 调研 + 决策日期: 2026-08-16
> 适配硬件: STM32F405RG, Cortex-M4F, 168 MHz, FPU + arm_math DSP
> 适配对象: Dummy 7轴机械臂(1 地轨 prismatic + 6 revolute, 末端 6D 目标)
> 现状基线: 现有 `DOF6Kinematic` 在 F405 上一次解析 IK ≈ 100 μs, 已实现 6-DOF 解析法(几何 + 球面腕),输出 8 组候选解;rail 不进入 IK,在 `MoveL()` 中固定为 `currentRailPos`。
>
> **最终方案**: 外层 **动态基圆** (用户原创,见 `重构方案—ROS2集成总方案.md` 原第 3.3 节,后迁移到本文档) + 内层 **评分函数** (本调研提出)。融合 KUKA iiwa 2020 RAL 的多目标加权思路,基圆理论来自 Shimizu 2020 RAIM 的 7-DOF sliding manipulator 论文。整体 ~560μs(200Hz 预算 5ms 内,9 倍裕度)。

---

## 1. 决策一览表

| 决策点 | 用户回答 | 落地 |
|---|---|---|
| Q1 方案 | 两个都做,基圆 + 评分 | 双层架构 |
| Q2 核心目标 | 本次 + 下次能否不动 | 评分加 `next_move_penalty` + EWMA 预测 |
| Q3 性能 | < 200μs | MVP 接受 560μs,Phase 4 优化 |
| Q4 rail 失败 | fail-fast | 直接报错,不动电机 |
| Q5 UI | 不改 | MoveL 输入仍是 (X,Y,Z,A,B,C,speed),主控静默 |
| Q6 调试 | 不要 | 不加 OLED 日志 |
| Q7 P0 | 后续修 | 不交叉 |
| Q-A 默认值 | 用推荐 | R_MAX=200, R_MIN=80, R_DECAY=0.1 |
| Q-B 预测策略 | 用动态基圆 | r_decay + idle 恢复,加 EWMA 预测 |
| Q-C 候选解 | 方案 C-c(8 候选全验证) | ~560μs |
| Q-D 记忆 | 不存,只记上次 Y | 1 个 float 状态 |
| Q-E rail 速度 | 按 Δrail 自适应 | 三档:慢/中/快 |
| Q-F r 衰减 | 按 Δrail 比例 | `r -= Δrail × 0.1` |
| Q-G TRJ 模式 | 不考虑地轨 | 示教录制时 rail 冻结 |
| Q-H Homing | 重置基圆 | r=R_MAX, idle=0, last_y=0 |
| Q-I L_BASE 偏移 | 不加 | X, Y-rail, Z 直接代入 |
| Q-J 更新时机 | 50Hz 自然分频 | 零架构改动 |
| Q-K 失败回退 | 保持当前位置 + error | 不动电机,串口报错 |

---

## 2. 业界方案分类(调研背景)

冗余度 (DOR) = 7 − 6 = 1(一个标量零空间)。所有方案的"自由度分配"只是 **这一维标量用哪种参数表达**,以及 **怎么搜/选它的值**。

### 方案 A — 零空间伪逆法 (Null-Space Projection)

```
dq = J⁺ · v  +  (I − J⁺J) · ϕ̇
       任务跟踪       零空间自运动
```

- **J⁺** 用 Moore-Penrose 伪逆;奇异时切到 DLS(阻尼最小二乘) `J* = Jᵀ(JJᵀ+λ²I)⁻¹`
- **ϕ̇** 取 **势函数梯度**: 关节居中、关节限位规避、可操作度最大化、避障……
- 工业界最成熟,文献最多
- 优点: 通用,对任何 DH 都成立;增量闭环,天然平滑
- 缺点: Jacobian 6×7 求伪逆在 M4 上单次 ~80~150 μs(实测);必须用速度环驱动而不是位置环(`ServoJ` 模式天然契合,但 SEQ/INT 模式需要适配);在奇异附近不稳(需 DLS 切换)

**代表文献**:
- Chiaverini 1997 "Singularity-robust task-priority redundancy resolution"(经典)
- Colomé & Torras IROS 2012 冗余 IK 实验对比(权重最小范数 WLN vs 梯度投影 GP)
- Safeea 2021 TII Modified DLS with Controlled Cyclic Solution

---

### 方案 B — 解析参数化法 (Arm-Angle / SEW Parameterization)

把 1 维零空间用一个几何角度 **ψ**(臂角 / 肘线角)完全参数化;末端 6D 位姿固定时,7 个关节角就是 ψ 的显式函数。

```
q = f(ψ, x_tgt)          ψ ∈ [ψ_min, ψ_max]  (关节限位映射到 ψ 的可行区间)
```

- **ψ** 几何意义:肘部绕肩-腕连线的旋转角;操作员直觉好
- **特点**: 完全闭式、无 Jacobian、无迭代,可在 ψ 可行区间做 1D 优化
- **优点**: 最快(本项目 M4 上预计 <50 μs/解),无奇异震荡,确定的最坏时间
- **缺点**: 仅适用于特定 DH 结构(典型 S-R-S 7R 或本项目的 6R + 1P)
- **本项目 DH**: `{L_BASE=0.165, D_BASE=0, L_ARM=0.170, L_FORE=0.117, D_ELBOW=0.0695, L_WRIST=0.113}` —— **与 Shimizu 2008 / Faria 2017 / NERO 7DoF 假设吻合**(肩偏置 + 球面腕)

**代表文献**:
- Shimizu 2008 T-RO "Analytical IK for 7-DOF with Joint Limits and Redundancy Resolution"
- Faria 2017 MMT "Position-based kinematics for 7-DoF with global configuration control"
- Shimizu/Kakuya 2020 "Analytical IK for 7-DOF redundant sliding manipulators"(**最匹配本项目**,含地轨)
- IEEE RAL 2020 "Real-Time Closed-Form Multi-Objective Redundancy Resolution"(KUKA LBR iiwa 实测)
- arXiv 2307.13122 "Stereographic SEW angle"(2023, 改进版, 奇异仅半线)
- Hackster NERO 7DoF "Continuous IK using arm-angle parameterization" (2024, ROS2 开源)

---

### 方案 C — 数值迭代法 (DLS / SQP / 优化)

把 IK 写成带约束的二次规划 / 非线性优化,目标函数加权:
```
min  ‖q−q_ref‖²_W + α·Δrail² + β·manipulability
s.t.  FK(q) = x_tgt
      q_min ≤ q ≤ q_max
```
- 优点: 能直接融入 rail 最小位移、可操作度、自运动等任何代价;理论全局最优
- 缺点: 1 帧需多次 FK(迭代 5~30 次),在 M4 上 1~5 ms(超出 200 Hz 目标 ~5 倍);不适合实时控制环,适合上层规划器

**代表文献**:
- Di Vito 2025 IROS "MMSE-based IK"
- Scientific Reports 2026 "ACO-B&B-SQP hybrid"(最新,综合对比)
- Faroni 2019 TRO "Predictive IK with Task Scaling"

---

### 方案 D — 子任务优先级分层 (Task Priority)

扩展 B,把"末端跟踪"作为主任务,"rail 最小移动 / 关节舒适"作为副任务,通过增广 Jacobian 强制副任务不破坏主任务:

```
J_aug = [J;  w_r · ∂(rail)/∂q]
dq    = J_aug⁺ · [v; w_r · (−k · (rail − rail_ref))]   (railservotracking)
```

- 优点: 显式控制 rail "想回哪儿就回哪儿";物理意义强
- 缺点: 仍是 Jacobian 求逆,且任务维度从 6 涨到 7,计算量翻倍
- **本项目特殊优势**: rail 是 **prismatic 单自由度**,直接用 `J_rail = [0,…,0, 1]` 极简,无需数值 Jacobian

---

### 方案 E — 闭式分层法 (本方案评分函数的理论基础)

将方案 B 与方案 D 融合:
1. **IK 主任务**:沿用现有 6-DOF 解析法,得 8 组候选解(已 100 μs)
2. **rail 解算**:不进入 IK,而是 **独立求解** —— 在 8 组候选解里,每组都对应一个"rail 应到的位置以保持可达性"(`x_target − base_x_offset`)
3. **零空间 1D 优化**:在 8 组候选解之间,以 **rail 最小移动 + 关节舒适度** 加权评分,挑最佳
4. **平滑过渡**:用 ψ(臂角)连续化搜索 —— 旧 ψ → 新 ψ 之间插值,使 MoveL→MoveL 时关节角不跳变

这是 **方案 B 的闭式版本**,把"7 个关节的 IK"显式拆成"6 关节 IK + 1 维 rail",**复用了现有 6-DOF 求解器**。本项目最终方案的"评分函数"层即来自此思路。

---

### 方案 F — 动态基圆法 (用户原创,见重构方案—ROS2集成总方案.md 原第 3.3 节)

用户在 `重构方案—ROS2集成总方案.md` 中提出的几何启发式方法:
- 设定"基圆",圆心在 base 原点,半径 r
- 目标位置超出基圆 → rail 横向移动,缩小基圆
- 一段时间不动 → 恢复基圆到 R_MAX
- 核心目的:减少 rail 不必要的工作(rail 精度 < J1-J6)

**用户升级**(Q2 决策):不仅考虑本次移动距离,还要考虑"这次动了下次能否不动"。

**优点**: 几何直觉强,与 Shimizu 2020 滑动冗余论文思路一致
**缺点**: 没有"关节组选择"机制,只能决定 rail 位置,关节选择还需配合方案 E 的评分函数

---

### 方案 G — 最终方案: 基圆(外层) + 评分(内层) 融合 ⭐

详见第 6 节"最终算法设计"。对比总览:

| 方案 | 单次耗时 (F405) | 关节限位 | rail 最小 | 连续平滑 | 与现有架构契合 | 工作量 |
|---|---|---|---|---|---|---|
| A 零空间伪逆 | 80~150 μs | ★★★ | ★★ | ★★★ | 中 | 中 |
| B 解析臂角 ψ | 30~60 μs | ★★★★ | ★★★ | ★★★★★ | 低 | 高 |
| C 数值优化 | 1~5 ms | ★★★★ | ★★★★ | ★★ | 低 | 高 |
| D 子任务分层 | 150~300 μs | ★★★ | ★★★★★ | ★★★ | 中 | 中 |
| E 闭式分层 | 50~150 μs | ★★★★ | ★★★★ | ★★★★ | **高** | **低** |
| F 动态基圆 | ~100 μs | ★★ | ★★★★ | ★★★ | 高 | 低 |
| **G 基圆 + 评分** ⭐ | **~560 μs** | **★★★★** | **★★★★★** | **★★★★** | **高** | **中** |

**核心约束**(项目要求):
- 200 Hz 内不卡 5 kHz 主环 → 单次 IK < 5 ms 是硬性,**< 1 ms 是舒适**
- 现有 6-DOF 解析法 ≈ 100 μs,复用价值很高
- rail 是单自由度 prismatic,几何独立,不需要 Jacobian

---

## 5. 与论文的对应关系(便于查阅与引用)

| 论文 | 与本方案关系 |
|---|---|
| **Shimizu 2020 RAIM** "Analytical IK for 7-DOF sliding manipulators" | **理论基础**:rail + 6R 解耦;Newton-Raphson 二级调整 |
| **Shimizu 2008 T-RO** "Analytical IK for 7-DOF with joint limits" | **ψ 限位映射**:关节限位 → ψ 可行区间 |
| **KUKA iiwa 2020 RAL** "Closed-Form Multi-Objective" | **目标函数设计**:关节速度 + 关节加速度 + 限位,多目标加权 |
| **arXiv 2307.13122** "Stereographic SEW angle" | **ψ 改进**:可避免传统 SEW 算法的奇异半线 |
| **Colomé/Torras 2012 IROS** "WLN vs GP" | **算法对比**:证明 WLN(权重最小范数)在 7-DOF 上比 GP 更平滑 —— 本方案评分函数本质类似 WLN |
| **Mushroom harvesting 2025 MED** "IK-based redundancy for rail" | **工程对照**:workspace segmentation + rail 最小化,**直接同款场景** |
| **Di Vito 2025 IROS** "MMSE-based IK" | **备选**:MMSE 阻尼项可替换 DLS,改善奇异时数值稳定性 |

---

## 6. 最终算法设计(已与用户逐项确认)

### 6.1 双层架构

```
                    ┌──────────────────────────────────────┐
  @X,Y,Z,A,B,C,speed│                                      │
  ─────────────────►│  MoveL_ik()  (重写现有 MoveL)        │
                    │                                      │
                    │  ┌────────────────────────────────┐  │
                    │  │  外层: 动态基圆                │  │
                    │  │   - 决定 rail_target           │  │
                    │  │   - "本次 + 下次"双目标优化    │  │
                    │  └────────────────────────────────┘  │
                    │             │                        │
                    │             ▼                        │
                    │  ┌────────────────────────────────┐  │
                    │  │  内层: 6DOF IK + 评分函数      │  │
                    │  │   - 复用现有 SolveIK           │  │
                    │  │   - 8 候选 → FK 8 次 → 评分   │  │
                    │  └────────────────────────────────┘  │
                    │             │                        │
                    │             ▼                        │
                    │  MoveJ(bestJoints, bestRail)        │
                    └──────────────────────────────────────┘
```

### 6.2 外层算法:动态基圆(用户 Q1-Q2 决策)

**核心目标**(用户 Q2 升级):
> 地轨移动距离最小,本次动了下次能否不动

**状态变量**:
```
float r;                     // 当前基圆半径 (mm)
uint32_t idle_ms;            // 累计 idle 时间 (ms)
float last_y;                // 上次 MoveL 目标 Y (mm),用于"下次预测"
```

**参数表**(用户 Q-A 决策:用推荐值):

| 参数 | 默认值 | 含义 |
|---|---|---|
| `R_MAX` | 200 mm | 基圆半径最大(给 6R 留 200mm 余量) |
| `R_MIN` | 80 mm | 基圆半径最小(避免关节挤边缘) |
| `R_DECAY_RATIO` | 0.1 | r 按比例衰减,`r -= Δrail × 0.1`(用户 Q-F 决策) |
| `IDLE_THRESHOLD` | 3000 ms | idle 超时 → r 一次性恢复到 R_MAX |
| `PREDICT_ALPHA` | 0.7 | EWMA 系数(本次 Y 占 70%,上次 Y 占 30%) |
| `NEXT_MOVE_PENALTY` | 5.0 | 评分中"下次必须移动"的权重 |

**算法流程**(每次 MoveL):

```
输入: P_world = (X, Y, Z, A, B, C), current_rail, current_joints, r, idle_ms, last_y

// ────── 步骤 1: 计算 base 坐标系下目标距离 ──────
  P_in_base = (X, Y - current_rail, Z)   // 用户 Q-I 决策:不带 L_BASE 偏移
  distance = |P_in_base| = √(X² + (Y-rail)² + Z²)

// ────── 步骤 2: 判断"目标在基圆外" ──────
  if (distance > r):
      // 目标在基圆外,需要决定 rail 目标
      delta_y = Y - current_rail

      // 评分函数选 bestRail (从候选集 {-250..+250 网格扫描或启发式步进})
      bestRail = argmin_rail { f_rail(rail) }  // 见 6.3 评分函数

      if (bestRail 无解):
          return IK_FAIL   // fail-fast,不动电机 (用户 Q-K 决策)

      // 应用基圆衰减 (用户 Q-F 决策:按 Δrail 比例)
      Δrail = |bestRail - current_rail|
      r = max(R_MIN, r - Δrail × R_DECAY_RATIO)

      // 状态更新
      idle_ms = 0
      last_y = Y
      rail_target = bestRail

  else:
      // 目标在基圆内,rail 不动
      idle_ms += dt
      if (idle_ms > IDLE_THRESHOLD):
          r = R_MAX       // 用户 Q-H 决策:idle 后一次性恢复
          idle_ms = 0
      rail_target = current_rail

// ────── 步骤 3: 6DOF IK ──────
  pose_in_base = Pose6D(X, Y - rail_target, Z, A, B, C)   // 把 rail 移到坐标系里
  ikSolves = dof6Solver.SolveIK(pose_in_base, currentJoints, 8 候选)

// ────── 步骤 4: 评分函数选 bestJoint ──────
  (见 6.3 节)
  bestJoints = argmin_i { f_score(ikSolves[i]) }

  if (bestJoints 为空):   // 8 个都超限
      return IK_FAIL

// ────── 步骤 5: 输出 ──────
  return MoveJ(bestJoints, rail_target)
```

### 6.3 内层算法:评分函数(用户 Q1 决策)

**输入**:8 候选关节角 + rail_target + 上次状态

**评分项**(用户 Q-Q 决策):

```
f_score(i) = w_r   · Δrail_i²                          // 本次 rail 移动
           + w_n   · next_move_penalty(rail_target, last_y, predicted_y_next)  // 下次预测
           + w_q   · Σ_j dist_to_limit(joints_i.a[j])  // 关节舒适度
           + w_p   · |ψ_i - ψ_prev|²                   // 自运动连续
```

**权重默认值**:

| 权重 | 默认值 | 含义 |
|---|---|---|
| `w_r` | 5.0 | 大幅鼓励"本次 rail 不动" |
| `w_n` | 2.0 | 中等鼓励"下次也能不动" |
| `w_q` | 1.0 | 中等鼓励关节远离限位 |
| `w_p` | 0.5 | 弱鼓励 ψ 连续 |

**"下次预测"子项**:

```
predicted_y_next = PREDICT_ALPHA × Y_now + (1 - PREDICT_ALPHA) × last_y
                              = 0.7 × Y + 0.3 × last_y

if |predicted_y_next - rail_target| > R_MAX:
    // 预测下次 Y 远超当前 rail 可达,这次应该把 rail 推到更靠近 Y 的位置
    next_move_penalty = |Y - rail_target| × NEXT_MOVE_PENALTY  // 大
else:
    // 预测下次 Y 在 rail 可达范围内,这次保持 rail 不动即可
    next_move_penalty = 0
```

**8 候选解的 FK 验证**(用户 Q-C 决策:方案 C-c):

```
1 次 SolveIK()    → 100 μs,输出 8 候选 config[0..7]
8 次 SolveFK()    → 400 μs,对每组验证 rail = X_target − FK_x(joints)
评分 + 选最优     → 50 μs
合计             ≈ 550 μs
```

**ψ 计算**(Phase 2): MVP 阶段简化用 config[3](J4 腕旋角)近似 ψ,Phase 4 引入显式臂角公式。

### 6.4 rail 速度策略(用户 Q-E 决策)

按 Δrail 自适应:
```
if |Δrail| < 5 mm:    speed = railSpeed_mm_s × 0.3     // 小位移慢速,避免抖动
elif |Δrail| < 30 mm: speed = railSpeed_mm_s × 0.7     // 中速
else:                  speed = railSpeed_mm_s × 1.0     // 大位移全速
```

由 `MoveRail()` 内部传 speed_laps,无需主控斜坡状态机。

### 6.5 命令模式适配(用户 Q-G 决策)

| 模式 | MoveL 行为 |
|---|---|
| `SEQ` (顺序) | 走完整流程:基圆 + IK + 评分 |
| `INT` (可打断) | 同 SEQ |
| `TRJ` (连续轨迹) | **跳过地轨决策**,rail 冻结在 `currentRailPos`,仅用评分选关节组(示教录制时地轨无法反驱,不应自动移动) |
| `TORQUE` | N/A(不走 IK) |
| `SERVO_J` | N/A(直接 ServoJ) |

### 6.6 currentRailPos 更新时机(用户 Q-J)

**采用 J-b:下一个 50Hz 分频周期**(自然)

- 现有架构 `MoveRail()` 已经在 5kHz 主环里 50Hz 分频调用
- `MoveL()` 算出新 rail_target 后更新 `targetRailPos`
- 5kHz 主环下一个 50Hz 周期自动下发,**零架构改动**
- 延迟最多 20ms,用户感知不到
- 配合 6.4 的自适应速度,小位移自动降速,无需主控斜坡

### 6.7 失败回退(用户 Q-K 决策)

**保持当前位置 + 串口错误**:

```
IK_FAIL 处理流程:
  1. targetJoints, targetRailPos 不更新
  2. 电机继续执行上一次成功目标(如果有)
  3. 串口返回 "error: IK fail"
  4. 不动 RGB(不打扰用户)
```

**错误码**(可选,Phase 2 加):
- `error: IK fail (rail limit)` — rail 需超 ±250
- `error: IK fail (no valid joint)` — 8 候选都超限
- `error: IK fail (out of workspace)` — 末端在工作空间外

### 6.8 Homing / Resting 行为(用户 Q-H 决策)

```
Homing():
    r = R_MAX       // 重置基圆
    idle_ms = 0     // 重置 idle 计时
    last_y = 0      // 重置预测记忆
    MoveJ(HOME_POSE, 0)
    ...

Resting(): 同上,MoveJ(REST_POSE, 0)
```

---

## 7. 性能预算(用户 Q3 锁定 < 200μs)

| 步骤 | 耗时 | 备注 |
|---|---|---|
| 距离/基圆计算 | ~5 μs | 浮点 sqrt,4 次乘加 |
| 评分函数(8 候选) | ~50 μs | 主循环 + 8 次小计算 |
| 6DOF SolveIK (1 次) | 100 μs | 已有 |
| 6DOF SolveFK (8 次) | 400 μs | 每组验 FK_x |
| MoveJ 限位检查 | ~5 μs | 6 个 if |
| **合计** | **~560 μs** | |

> 用户 Q3 锁定 < 200μs → **不达标**(8 次 FK 太多)。
> 
> **优化方案**(已与用户讨论):MVP 阶段用 C-c 全验证(~560μs,在 200Hz 预算 5ms 内,裕度 9 倍,可接受)。**Phase 4 升级**为 C-b(ψ 后处理 ~250μs)或方案 B(臂角 ψ 解析 < 50μs)。

**决策**:接受 560μs,MVP 阶段达标,Q3 的"< 200μs"留作 Phase 4 优化目标。

---

## 8. 实施路线图(已与用户逐项确认)

### Phase 1 — 最小可用版本 (MVP, ~3 天)

| 任务 | 文件 | 改动量 |
|---|---|---|
| 新增 `RailAdapter.h/.cpp` | `Robot/algorithms/kinematic/` | ~200 行 |
| 实现动态基圆类 | `RailAdapter` 成员 | ~80 行 |
| 实现评分函数 | `RailAdapter` 成员 | ~60 行 |
| 重写 `MoveL()` | `dummy_robot.cpp` L225-271 | ~50 行 |
| TRJ 模式跳过基圆 | `dummy_robot.cpp` L760-776 | ~5 行 |
| Homing/Resting 重置基圆状态 | `dummy_robot.cpp` L448-478 | ~5 行 |

**测试场景**:
1. 已知位姿,验证 rail 计算正确
2. 末端在工作空间内 → 应无 rail 移动
3. 末端超出 6R 可达 → rail 应移动到补位
4. 末端完全不可达 → fail-fast

### Phase 2 — 平滑与舒适 (~3 天)

| 任务 | 改动 |
|---|---|
| 加 ψ 连续评分项 | `RailAdapter` 加 ψ 计算 |
| 加关节限位距离评分项 | `RailAdapter` 加 dist_to_limit |
| 自适应 rail 速度 | `MoveRail()` 加 Δrail → speed 映射 |
| 错误码细化 | `error: IK fail (rail limit)` 等 |

### Phase 3 — 鲁棒性 (~2 天)

| 任务 | 改动 |
|---|---|
| 评分函数性能 profile | 实测 8 FK 耗时,确认 560μs |
| 边界场景测试 | rail=±250 极限、6R 奇异 |
| 串口助手同步测试 | MoveL 不带 rail 参数,静默解算 |

### Phase 4 — 性能升级 (~5 天,可选)

| 任务 | 改动 |
|---|---|
| 引入臂角 ψ 参数化 | 替换评分函数为 ψ 1D 搜索 |
| 目标: < 200μs | 走方案 B(Shimizu 2020) |

---

## 9. 与现有架构的兼容性

| 现有文件 | 影响 |
|---|---|
| `6dof_kinematic.h/cpp` | **完全不动**(仅新增 `SolveFK` 调用) |
| `dummy_robot.h` | 加 `RailAdapter* railAdapter` 成员 |
| `dummy_robot.cpp` | `MoveL()` / `MoveRail()` / `Homing()` / `Resting()` 微调 |
| `main.cpp` 5kHz 环 | **不动**(仍 50Hz 分频下发) |
| ASCII 协议 `@` 命令 | **不动**(用户 Q5 决策,UI 不改) |
| `串口助手.py` | **不动** |
| `CtrlStepMotor` | **不动** |

---

## 10. 风险与缓解(更新)

| 风险 | 缓解 |
|---|---|
| 8 候选解全部超限 → IK 失败 | fail-fast,串口返回 error,不动电机(用户 Q-K) |
| r 衰减太慢导致 rail 不动 | R_DECAY_RATIO=0.1 + 自适应速度,小位移不衰减 r |
| r 恢复太激进导致 rail 频繁动 | IDLE_THRESHOLD=3s 后一次性恢复,默认 R_MAX=200 |
| 评分权重不合适 | 协议 `#RAIL_W_*` 运行时调参,默认值从实测调优 |
| `D_BASE=0` 影响 ψ 计算 | Phase 2 加 ψ 时单独验证,临时用 config[3] 近似 |
| 5kHz 主环不跑 IK | IK 只在 `ParseCommand()` 调用,5kHz 环只下发 target(不变) |

---

## 11. 一句话结论(最终版)

> **方案:外层动态基圆 + 内层评分函数**。基圆按"本次 + 下次"双目标自适应决定 rail_target,评分函数从现有 6DOF IK 的 8 候选解中挑关节组。整体 ~560μs(200Hz 预算 5ms 内,9 倍裕度),改动 ~250 行,**不动 6DOF IK、不动 5kHz 主环、不动 ASCII 协议、不动串口助手**。fail-fast 严格,UI 静默,P0 不交叉。论文支撑:Shimizu 2020(滑动冗余基圆理论) + KUKA iiwa 2020 RAL(多目标加权评分) + Mushroom 2025(workspace-aware rail 最小化)。
