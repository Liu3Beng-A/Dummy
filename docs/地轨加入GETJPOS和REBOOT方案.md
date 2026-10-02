# 地轨加入 `#GETJPOS` 与 `#REBOOT` 协议方案

> 编制日期：2026-10-01
> 编制人：Cursor Agent（用户授权任务）
> 涉及模块：`firmware_new/dummy-ref-core-fw`（主控）、`串口助手_old.py`（上位机目标）
> 当前任务范围：把地轨（nodeID=7）接入串口 ASCII 协议的 `#GETJPOS` 回读与 `#REBOOT` 单节点/广播重启。

---

## 1. 背景与目标

### 1.1 现状（v2026-10-01）

`#GETJPOS` 与 `#REBOOT` 仍按 **6 关节时代** 实现，地轨未接入串口协议：

- `firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:30-34`（USB）和 `:144-148`（UART4）
  ```cpp
  Respond("ok %.2f %.2f %.2f %.2f %.2f %.2f",
   currentJoints.a[0..5]);    // 只 6 个关节，无地轨
  ```
- `firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:78-89`（仅 USB）
  ```cpp
  if (node >= 1 & node <= 6){        // 不含 0=ALL，不含 7=地轨
      mock.motorJ[node]->Reboot();
      ...
  } else {
      Respond("error REBOOT MOTOR [%lu] is wrong", node);
  }
  ```
  - 写法 `&` 顺手改为 `&&`（bool→int 巧合等价，改后无副作用；本次本来就要扩 node 范围到 7，顺手修正）。

### 1.2 底层硬件已就绪（无需改电机固件）

| 位置 | 内容 |
|---|---|
| `dummy_robot.h:7,10` | `#define ALL 0`、`#define RAIL_FIXED_NODE_ID 7` |
| `dummy_robot.h:116` | `motorJ[8]`：索引 0=ALL / 1~6=J1~J6 / **7=地轨** |
| `dummy_robot.cpp:150` | `motorJ[ALL]->Reboot()` 已存在（CAN StdId=0x7F，触发全部电机重启） |
| `interface_can.cpp:330-333` | 电机固件收到 cmd=0x7F 即 `HAL_NVIC_SystemReset()`，**所有 nodeID（含 7）均支持** |
| `dummy_robot.cpp:538-543` | `DummyHand::Reboot()` 单独发 StdId=0x47F（夹爪不在 ALL 代理范围） |
| `dummy-motor-fw/Core/Src/can.c:202-204` | **广播机制**：`if (id == 0 \|\| id == boardConfig.canNodeId)` — StdId 高 4 位为 0 时所有电机都接收该帧，所以 `motorJ[ALL]->Reboot()` 1 帧就能让 J1~J7（含地轨）全部响应 |

### 1.3 任务目标

1. `#GETJPOS` 输出扩展为 7 个值，**Rail 在第 7 位**（与 MoveJ 命令参数顺序对齐 `>j1,j2,j3,j4,j5,j6,j7,speed`）。
2. `#REBOOT <node>` 范围扩展到 `0~7`：node=0 重启全部电机 + 夹爪（**不**重启主控），node=1~7 单节点重启，UART4 保持原样（无线通道不暴露重启）。
3. 配套 `串口助手_old.py`（目标上位机）按新协议顺序解析；`串口助手.py` 仅参考，**禁止修改**。

---

## 2. 协议变更对照

### 2.1 `#GETJPOS`

| 版本 | 回包格式 | 说明 |
|---|---|---|
| 旧（v ≤ 2026-09） | `ok J1 J2 J3 J4 J5 J6` | 6 个值，无地轨 |
| 旧（v3.1 串口助手预期） | `ok Rail_mm J1 J2 J3 J4 J5 J6` | Rail 在前（已实现于 `串口助手_old.py:917-938`） |
| **新（本次）** | `ok J1 J2 J3 J4 J5 J6 Rail_mm` | Rail 在最后，与 MoveJ 顺序对齐；**约定单位**：`nums[0..5]`=度、`nums[6]`=毫米，协议不显式标注 |

> ⚠️ **协议方向调整**：v3.1 串口助手实现的是"Rail 在前"，本次反向调整为"Rail 在后"。理由：与 MoveJ 命令 `>j1,j2,...,j6,j7,speed` 顺序一致；上位机解析代码与发送代码可共用同一索引语义。`串口助手_old.py` 的 `_fill_joints` 必须按新顺序重写。

> ⚠️ **v3.1 不一致历史遗留**：v3.1 的上位机实现"在等 7 个值"，但固件从 6 关节时代起一直回 6 个值——此不一致历史地存在了至少一个迭代周期；本次一并修正为"7 个值"。

### 2.2 `#REBOOT <node>`

| node | 行为 | 备注 |
|---|---|---|
| `0` | 重启 J1~J7 + 夹爪 | **不**重启主控（避免串口断开收不到响应） |
| `1` ~ `7` | 单节点重启 | 地轨（7）走 `motorJ[7]->Reboot()`，CAN StdId = `7<<7 \| 0x7F` |
| `8`（夹爪） | `error REBOOT MOTOR [8] is wrong` | **设计选择**：夹爪 id=8 不暴露给 `#REBOOT` 单节点；如需重启夹爪，发 `#REBOOT 0`（包含夹爪）。理由：避免用户单独戳夹爪引发现场误操作 |
| 其它 | `error REBOOT MOTOR [node] is wrong` | 错误信息保持原格式 |

> 范围检查顺手把 `&` 改成 `&&`（C++ 写法规范）。

---

## 3. 主控固件改动

### 3.1 新增 `DummyRobot::RebootAllMotors()` —— 只重启电机+夹爪，不重启系统

**`dummy_robot.h`** 在 `Reboot()` 声明附近加：

```cpp
/** 重启全部电机 + 夹爪（**不**重启主控系统）。用于串口 #REBOOT 0。
 *  - J1~J7（含地轨）通过 motorJ[ALL] 广播 1 帧 StdId=0x7F
 *    （CAN 接收端 id==0 触发所有电机响应，见 dummy-motor-fw/Core/Src/can.c:202-204）
 *  - 夹爪（id=8）通过 DummyHand::Reboot() 单独发 1 帧 StdId=0x47F
 *  - 不调 HAL_NVIC_SystemReset()，避免主控重启后串口收不到 ok 回包
 *  - 不调 osDelay：两帧 CAN 在 µs 级装入 Mailbox，硬件异步发完；
 *    block 主任务 500ms 会导致 FixUpdate / OLED / 串口命令处理全部暂停
 *  - 与 Reboot() 的区别：Reboot() 含 HAL_NVIC_SystemReset()，给内部协议层 reboot 调用 */
void RebootAllMotors();
```

**`dummy_robot.cpp`** 新增实现：

```cpp
void DummyRobot::RebootAllMotors()
{
    motorJ[ALL]->Reboot();   // J1~J7（含地轨），1 帧广播
    hand->Reboot();          // 夹爪（id=8）单独 1 帧
    // 故意不 osDelay：
    //   原 Reboot() 末尾 500ms 是给 HAL_NVIC_SystemReset() 留出发帧时间，
    //   本函数不调 reset，且 CAN 帧已在 µs 级装入 Mailbox（can.c CAN_Send），
    //   无需 block 任何 RTOS 任务。
}
```

原 `Reboot()`（`dummy_robot.cpp:148-153`）保留不动，含 system reset；`#REBOOT 0` 改用 `RebootAllMotors()`，协议层 `reboot` 函数仍走原 `Reboot()`。

### 3.2 `#GETJPOS` 扩展（USB + UART4 都改）

`ascii_protocol.cpp:30-34`：

```cpp
Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
        dummy.currentJoints.a[0], dummy.currentJoints.a[1],
        dummy.currentJoints.a[2], dummy.currentJoints.a[3],
        dummy.currentJoints.a[4], dummy.currentJoints.a[5],
        dummy.currentRailPos);  // [NEW] 第 7 位，Rail (mm)
```

`ascii_protocol.cpp:144-148`（UART4）同样改。

### 3.3 `#REBOOT` 扩展（仅 USB）

`ascii_protocol.cpp:78-89`：

```cpp
} else if (s.find("REBOOT") != std::string::npos)
{
    uint32_t node;
    sscanf(_cmd, "#REBOOT %lu", &node);
    if (node >= 1 && node <= 7)                  // [FIX] && 写法 + 范围到 7
    {
        dummy.motorJ[node]->Reboot();
        Respond(_responseChannel, "ok REBOOT MOTOR [%lu]", node);
    }
    else if (node == 0)                          // [NEW] ALL 代理
    {
        Respond(_responseChannel, "ok REBOOT ALL MOTORS + HAND");  // [FIX] 先回包
        dummy.RebootAllMotors();                                   // [FIX] 再发重启帧
    }
    else
    {
        Respond(_responseChannel, "error REBOOT MOTOR [%lu] is wrong", node);
    }
}
```

顺序很关键——**先 Respond，再调 `RebootAllMotors()`**。`RebootAllMotors()` 不再含 `osDelay(500)`（详见 §3.1），主任务不会被 block；但仍建议先回包，让 USB / UART4 发送 task 立即 drain 出包，用户在串口助手上看到 `ok` 的延迟更短。

### 3.4 `#GETRAILPOS` 单独命令

**决定：保留，不废弃。** 理由：

- 与 `#GETJPOS`（7 个值）并存，提供"只读地轨"的轻量接口，单次回包 1 个值
- 一些脚本/调试场景只关心 Rail，不需要全部关节数据，可降低串口流量
- 删除需要同步修改 USB + UART4 + 串口助手 + 任何第三方脚本，影响面大于收益

`#GETRAILPOS` 在 USB / UART4 的回包保持 `ok %.2f`（mm）原样不动。

### 3.5 UART4 `#REBOOT` 维持现状

UART4（蓝牙/无线通道）**不**实现 `#REBOOT` 命令，避免无线误重启；用户规则中无线通道权限有限，保留这层保护。

---

## 4. 串口助手 `_old.py` 改动

> 规则：**`串口助手.py` 仅参考，禁止修改**；目标上位机是 `串口助手_old.py`。

### 4.1 `_fill_joints`（`串口助手_old.py:916-938`）

**当前实现（v3.1，Rail 在前）**：
```python
def _fill_joints(self, nums):
    """把 7 个数写进滑块（v3.1：nums[0]=Rail(mm), nums[1..6]=J1..J6(°)）"""
    self._suppress_drag_send = True
    try:
        for i in range(6):
            v = nums[i + 1]                      # ❌ 旧：J1~J6 = nums[1..6]
            self.scl_joints[i].set(v)
            ...
        rv = nums[0]                             # ❌ 旧：Rail = nums[0]
        self.scl_j7.set(rv)
        ...
```

**改为（新协议 Rail 在后）**：
```python
def _fill_joints(self, nums):
    """把 7 个数写进滑块（新协议：nums[0..5]=J1..J6(°), nums[6]=Rail(mm)）"""
    self._suppress_drag_send = True
    try:
        # 同步 J1~J6 (nums[0..5])
        for i in range(6):
            v = nums[i]                          # [FIX] 直接 nums[i]
            self.scl_joints[i].set(v)
            self.ent_joints[i].delete(0, tk.END)
            self.ent_joints[i].insert(0, f"{v:.2f}")
            self.lbl_joints[i].config(text=f"{v:.2f}")
        # 同步 Rail (nums[6])
        rv = nums[6]                             # [FIX] nums[6]
        self.scl_j7.set(rv)
        self.ent_j7.delete(0, tk.END)
        self.ent_j7.insert(0, f"{rv:.2f}")
        self.lbl_j7.config(text=f"{rv:.2f} mm")
```

### 4.2 RX 线程过滤回包长度（`串口助手_old.py:721-724`）

**当前**：
```python
if self._pending_sync and line.startswith("ok"):
    nums = self._parse_numbers(line[2:].strip())
    if nums is not None and len(nums) == 6:    # ❌ 旧：6 个值
        self._sync_data = nums
```

**改为**：
```python
if self._pending_sync and line.startswith("ok"):
    nums = self._parse_numbers(line[2:].strip())
    if nums is not None and len(nums) == 7:    # [FIX] 7 个值
        self._sync_data = nums
```

> `#GETJPOS` 与 `#GETLPOS` 仍然都按 7 个值过滤——但 `#GETLPOS` 是 6 个笛卡尔位姿分量（X/Y/Z/A/B/C），**不**变长。所以必须先按 kind 区分，再分别处理：j→7 个值，l→6 个值。

**改为（区分 kind）**：
```python
if self._pending_sync and line.startswith("ok"):
    nums = self._parse_numbers(line[2:].strip())
    expected = 7 if self._pending_sync == "j" else 6
    if nums is not None and len(nums) == expected:
        self._sync_data = nums
```

---

## 5. 风险点

1. **协议方向反转**：v3.1 是 Rail 在前，新协议是 Rail 在后。任何缓存的协议文档、之前的回包脚本、外部解析代码都要同步；本次仅同步 `串口助手_old.py`。
2. **`#REBOOT 0` 响应时序**：必须先 `Respond()`，再 `RebootAllMotors()`；这样 USB / UART4 发送 task 立即 drain 回包，用户视角无延迟。如果颠倒，回包会延迟到 500ms 之后（旧实现含 osDelay 的副作用），新实现无 osDelay 但仍建议先回包以维持体验一致。
3. **`RebootAllMotors()` 不写 `osDelay`**：原 `Reboot()` 末尾 500ms 是给 `HAL_NVIC_SystemReset()` 留出发帧时间；新函数不调 reset，两帧 CAN 在 µs 级装入 Mailbox（详见 `dummy-motor-fw/Core/Src/can.c CAN_Send`），无 block 必要。如果误把 `osDelay(500)` 抄过来，主控的 FixUpdate（100Hz）、OLED 刷新、串口命令处理会全部暂停 500ms，期间用户连续发的 ASCII 命令会被压在 RX queue 里。
4. **`#REBOOT 0` 与原 `Reboot()` 行为差异**：原 `Reboot()` 含 `HAL_NVIC_SystemReset()`（主控也重启），用于协议层 `reboot` 函数（内部无响应需求）；新 `RebootAllMotors()` 只重启外设，专门给串口 `#REBOOT 0` 用。两套路径都保留，分别走 ASCII 协议和内部协议层。
5. **`串口助手.py`（参考版）**：不动；其旧实现也是 Rail 在前，但仅参考用，不跟随升级。

---

## 6. 验收清单

### 6.1 功能正确性

- [ ] `#GETJPOS`（USB）回包 7 个值 `ok J1 J2 J3 J4 J5 J6 Rail`，且 `currentRailPos` 与电机端实际位置一致（注意 `currentRailPos` 仅在 `UpdateJointAnglesCallback()` 里更新，FixUpdate 100Hz 轮询 1~7，单帧时延最大约 70ms）
- [ ] `#GETJPOS`（UART4）同上
- [ ] `#GETLPOS`（USB + UART4）仍为 6 个值，不变
- [ ] `#REBOOT 1` / `#REBOOT 2` / ... / `#REBOOT 7` 单节点回包 `ok REBOOT MOTOR [n]`，对应电机确实重启
- [ ] `#REBOOT 0` 回包 `ok REBOOT ALL MOTORS + HAND`，**主控串口不卡**，J1~J7 + 夹爪均断电重启
- [ ] `#REBOOT 8` / `#REBOOT 100` 返回 `error REBOOT MOTOR [n] is wrong`
- [ ] 串口助手_old.py 滑块同步成功：移动 J1~J6 后按"读当前位姿同步"，滑块值与实际一致；移动 Rail 后滑块值与实际一致

### 6.2 时序与副作用

- [ ] `#REBOOT 0` 后立即发 `#GETJPOS`，回包延迟 < 50ms（验证 §3.1 的 osDelay 已正确删除）
- [ ] `#REBOOT 0` 期间 OLED 不黑屏、不重启（验证主控未 system reset）
- [ ] 用 CAN 分析仪监听总线，能看到 `#REBOOT 0` 触发 2 帧：StdId=0x7F（广播）+ StdId=0x47F（夹爪），两帧相隔 < 1ms
- [ ] `#REBOOT 0` 后 500ms 查电机，J1~J7 + 夹爪均已重启完成（CAN 节点不再响应新帧）

### 6.3 边界输入

> 注：`node` 是栈上 `uint32_t`，未初始化。`sscanf` 失败时 `node` 保持栈原值；具体行为依赖编译器/调用栈深度，需真机或仿真验证。

- [ ] `#REBOOT 0.5` → sscanf 读 0 遇到 `.` 停止 → `node=0` → 走 ALL 代理分支（**不会**走 error 分支）
- [ ] `#REBOOT abc` → sscanf 解析失败 → `node` 保持栈原值（可能为 0 或其它任意值），行为未定义，需现场记录实际现象
- [ ] `#REBOOT`（缺参数） → 同上，`node` 未被赋值，行为未定义
- [ ] `#REBOOT 8` → 走 error 分支，返回 `error REBOOT MOTOR [8] is wrong`（设计选择：夹爪不暴露给单节点重启）

> **未来改进建议**：将 `uint32_t node;` 改为 `uint32_t node = UINT32_MAX;`，并在 `sscanf` 失败检查后增加 fallback 处理，把"未知/非法 node"统一收口到 error 分支。**本次任务不实现**，记入 follow-up。

---

## 7. 同类问题 follow-up（不在本任务范围）

本次任务只处理 `#GETJPOS` 与 `#REBOOT`。但地轨（id=7）接入 ASCII 协议后，类似"6 关节时代残留"还有多处需要跟进：

| 命令 | 当前 node 范围 | 涉及行 | 备注 |
|---|---|---|---|
| `#SET_DCE_KP` | `1~6` | `ascii_protocol.cpp:53-67` | 地轨 DCE KP 设不了 |
| `#SET_DCE_KI` | `1~6` | `ascii_protocol.cpp:68-79` | 同上 |
| `#SET_DCE_KD` | `1~6` | `ascii_protocol.cpp:80-91` | 同上 |
| `#SET_TORQUE_LIMIT` | 未实现 | — | 协议层 `#SET_DCE_KP` 路径可能未含 |
| `#GET_TEMP` | 未实现 | — | 电机端 0x25 已支持，需主控 + 上位机串起来 |
| `#GET_CURRENT` | 未实现 | — | 电机端 0x2D 已支持，需主控 + 上位机串起来 |

> 这些都是独立任务。本次方案不修改，留作各 follow-up task 单独建文档推进。

另外，`dummy_robot.cpp:309-318` 的 `SetJointAcceleration` 只循环 `SetAcceleration(1..6)`，地轨（id=7）的加速度兜底 `JOINT_ACC_FALLBACK[0]` 没下发到电机 EEPROM——是否需要同步下发是另一个独立问题。

---

## 附录 A：参考依据文件路径

- 主控：`firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:30-34, 78-89, 144-148`
- 主控类：`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.{h,cpp}`
  - `dummy_robot.h:7,10,116`（ALL、RAIL_FIXED_NODE_ID、motorJ[8]）
  - `dummy_robot.cpp:148-153`（原 `Reboot()`，含 system reset）
  - `dummy_robot.cpp:538-543`（`DummyHand::Reboot()`，发 StdId=0x47F）
- 电机端协议：`firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:330-333`（cmd=0x7F = Reboot）
- 电机端广播机制：`firmware_new/dummy-motor-fw/Core/Src/can.c:202-204`
  （`if (id == 0 || id == boardConfig.canNodeId)` — StdId 高 4 位为 0 时所有电机都接收）
- 电机端发送：`firmware_new/dummy-motor-fw/Core/Src/can.c:131-138`（`CAN_Send` 同步装入 Mailbox，µs 级返回）
- 上位机：`串口助手_old.py:718-724, 916-938`（过滤与 `_fill_joints`）
- 协议层节点规范：`.cursor/rules/PROJECT_GUIDANCE.mdc §2.2`
- 已有方案对照：`docs/地轨电机加入方案.md`