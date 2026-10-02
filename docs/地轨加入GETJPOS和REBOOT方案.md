# 地轨加入 `#GETJPOS` 与 `#REBOOT` 协议方案

> 编制日期：2026-10-01
> 编制人：Cursor Agent（用户授权任务）
> 涉及模块：`firmware_new/dummy-ref-core-fw`（主控）、`串口助手_old.py`（上位机目标）
> 当前任务范围：把地轨（nodeID=7）接入串口 ASCII 协议的 `#GETJPOS` 回读与 `#REBOOT` 单节点/广播重启。

---

## 1. 背景与目标

### 1.1 现状（v2026-10-01）

`#GETJPOS` 与 `#REBOOT` 仍按 **6 关节时代** 实现，地轨未接入串口协议：

- `firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:42`（USB）和 `:154`（UART4）
  ```cpp
  Respond("ok %.2f %.2f %.2f %.2f %.2f %.2f",
   currentJoints.a[0..5]);    // 只 6 个关节，无地轨
  ```
- `firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:89-99`（仅 USB）
  ```cpp
  if (node >= 1 & node <= 6){        // 不含 0=ALL，不含 7=地轨
      mock.motorJ[node]->Reboot();
      ...
  } else {
      Respond("error REBOOT MOTOR [%lu] is wrong", node);
  }
  ```
  - 写法 `&` 应为 `&&`，bool→int 巧合等价；不顺手改风险更大。

### 1.2 底层硬件已就绪（无需改电机固件）

| 位置 | 内容 |
|---|---|
| `dummy_robot.h:7,10` | `#define ALL 0`、`#define RAIL_FIXED_NODE_ID 7` |
| `dummy_robot.h:129` | `motorJ[8]`：索引 0=ALL / 1~6=J1~J6 / **7=地轨** |
| `dummy_robot.cpp:58` | `motorJ[ALL]->Reboot()` 已存在（CAN id=0x7F，触发全部电机重启） |
| `interface_can.cpp:331` | 电机固件收到 cmd=0x7F 即 `HAL_NVIC_SystemReset()`，**所有 nodeID（含 7）均支持** |
| `dummy_robot.cpp:430-435` | `DummyHand::Reboot()` 单独发 0x7F（夹爪不在 ALL 代理范围） |

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
| **新（本次）** | `ok J1 J2 J3 J4 J5 J6 Rail_mm` | Rail 在最后，与 MoveJ 顺序对齐；不带单位 |

> ⚠️ **协议方向调整**：v3.1 串口助手实现的是"Rail 在前"，本次反向调整为"Rail 在后"。理由：与 MoveJ 命令 `>j1,j2,...,j6,j7,speed` 顺序一致；上位机解析代码与发送代码可共用同一索引语义。`串口助手_old.py` 的 `_fill_joints` 必须按新顺序重写。

### 2.2 `#REBOOT <node>`

| node | 行为 | 备注 |
|---|---|---|
| `0` | 重启 J1~J7 + 夹爪 | **不**重启主控（避免串口断开收不到响应） |
| `1` ~ `7` | 单节点重启 | 地轨（7）走 `motorJ[7]->Reboot()`，CAN StdId = `7<<7 | 0x7F` |
| 其它 | `error REBOOT MOTOR [node] is wrong` | 错误信息保持原格式 |

> 范围检查顺手把 `&` 改成 `&&`（C++ 写法规范）。

---

## 3. 主控固件改动

### 3.1 新增 `DummyRobot::RebootAllMotors()` —— 只重启电机+夹爪，不重启系统

**`dummy_robot.h`** 在 `Reboot()` 声明附近加：

```cpp
/** 重启全部电机 + 夹爪（**不**重启主控系统）。用于 #REBOOT 0。
 *  与 Reboot() 的区别：Reboot() 含 HAL_NVIC_SystemReset()，会给内部协议层用；
 *  本函数专给串口 #REBOOT 0 使用，保证"ok"响应能发出去。 */
void RebootAllMotors();
```

**`dummy_robot.cpp`** 新增实现：

```cpp
void DummyRobot::RebootAllMotors()
{
    motorJ[ALL]->Reboot();   // J1~J7 全部
    hand->Reboot();          // 夹爪单独发
    osDelay(500);            // 等电机帧发完
    // 注意：不调用 HAL_NVIC_SystemReset()
}
```

原 `Reboot()`（`dummy_robot.cpp:56`）保留不动，含 system reset；`#REBOOT 0` 改用 `RebootAllMotors()`，协议层 `reboot` 函数仍走原 `Reboot()`。

### 3.2 `#GETJPOS` 扩展（USB + UART4 都改）

`ascii_protocol.cpp:42`：

```cpp
Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
        dummy.currentJoints.a[0], dummy.currentJoints.a[1],
        dummy.currentJoints.a[2], dummy.currentJoints.a[3],
        dummy.currentJoints.a[4], dummy.currentJoints.a[5],
        dummy.currentRailPos);  // [NEW] 第 7 位，Rail (mm)
```

`ascii_protocol.cpp:154`（UART4）同样改。

### 3.3 `#REBOOT` 扩展（仅 USB）

`ascii_protocol.cpp:89-99`：

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
        dummy.RebootAllMotors();                 // 不重启主控
        Respond(_responseChannel, "ok REBOOT ALL MOTORS + HAND");
    }
    else
    {
        Respond(_responseChannel, "error REBOOT MOTOR [%lu] is wrong", node);
    }
}
```

顺序很关键——**先 Respond，再调 `RebootAllMotors()`**，否则重启帧发完才回包，串口已断。

### 3.4 UART4 `#REBOOT` 维持现状

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
2. **`#REBOOT 0` 响应时序**：必须先 `Respond()`，再 `RebootAllMotors()`；否则电机帧发完时主控串口已无法回包。代码顺序已按此写。
3. **`#REBOOT 0` 与原 `Reboot()` 行为差异**：原 `Reboot()` 含 `HAL_NVIC_SystemReset()`（主控也重启），用于协议层 `reboot` 函数（内部无响应需求）；新 `RebootAllMotors()` 只重启外设，专门给串口 `#REBOOT 0` 用。
4. **`串口助手.py`（参考版）**：不动；其旧实现也是 Rail 在前，但仅参考用，不跟随升级。

---

## 6. 验收清单

- [ ] `#GETJPOS`（USB）回包 7 个值 `ok J1 J2 J3 J4 J5 J6 Rail`，且 `currentRailPos` 与电机端实际位置一致
- [ ] `#GETJPOS`（UART4）同上
- [ ] `#GETLPOS`（USB + UART4）仍为 6 个值，不变
- [ ] `#REBOOT 1` / `#REBOOT 2` / ... / `#REBOOT 7` 单节点回包 `ok REBOOT MOTOR [n]`，电机确实重启
- [ ] `#REBOOT 0` 回包 `ok REBOOT ALL MOTORS + HAND`，**主控串口不断**，电机+夹爪均重启
- [ ] `#REBOOT 8` / `#REBOOT 100` 返回 `error REBOOT MOTOR [n] is wrong`
- [ ] 串口助手_old.py 滑块同步成功：移动 J1~J6 后按"读当前位姿同步"，滑块值与实际一致；移动 Rail 后滑块值与实际一致

---

## 附录 A：参考依据文件路径

- 主控：`firmware_new/dummy-ref-core-fw/UserApp/protocols/ascii_protocol.cpp:42,89,154`
- 主控类：`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.{h,cpp}`
- 电机端协议：`firmware_new/dummy-motor-fw/UserApp/protocols/interface_can.cpp:331`（0x7F = Reboot）
- 节点表常量：`firmware_new/dummy-ref-core-fw/Robot/instances/dummy_robot.h:7,10`
- 上位机：`串口助手_old.py:721-724, 916-938`（过滤与 `_fill_joints`）
- 协议层节点规范：`.cursor/rules/PROJECT_GUIDANCE.mdc §2.2`
- 已有方案对照：`docs/地轨电机加入方案.md`