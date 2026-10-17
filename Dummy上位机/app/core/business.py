"""Business rules migrated from the old assistant, with Qt field adapters.

No runtime dependency on the original script or tkinter. The controller overrides
serial dispatch and position reading; this mixin retains verified protocol semantics.
"""
import re
import math
import queue
from dataclasses import dataclass
from functools import wraps
from app.protocol.constants import *
from app.core import dialogs as messagebox
from app.storage.store import validate_points


@dataclass(frozen=True)
class TeachingPoint:
    """A sequence snapshot never shares a mutable target with the editor."""
    name: str
    kind: str
    target: tuple[float, ...]
    gripper: float
    speed: float
    frame: str = "base"
    target_kind: str = "flange"
    tool: tuple[float, ...] | None = None


def _rgb_readback_guard(method):
    """Widget signals during device fills must not be interpreted as local edits."""
    @wraps(method)
    def guarded(self, *args, **kwargs):
        previous = self._rgb_applying_readback
        self._rgb_applying_readback = True
        try:
            return method(self, *args, **kwargs)
        finally:
            self._rgb_applying_readback = previous
    return guarded


class RobotBusiness:
    def _action_problem(self, scope, operation, message, next_step="", *,
                        target="", phase="blocked"):
        """Keep local validation visible without entering a modal event loop.

        The optional hook also keeps this mixin usable by offline adapters which
        only provide the historical log and command interfaces.
        """
        self.log(message, "WARN")
        notify = getattr(self, "notify_action", None)
        if callable(notify):
            notify(scope, operation, target=target, phase=phase, message=message,
                   next_step=next_step, evidence="local")

    @staticmethod
    def _action_node(node):
        return "全部节点" if node == 0 else f"节点 {node}"

    @staticmethod
    def _rgb_values(variables, ranges):
        values = []
        for variable, (lo, hi) in zip(variables, ranges):
            text = variable.get().strip()
            if re.fullmatch(r"[0-9]+", text) is None or not lo <= int(text) <= hi:
                raise ValueError(f"RGB参数须为{lo}～{hi}的整数")
            values.append(int(text))
        return values

    def _rgb_send(self, command):
        # Setters already return MCU status/configuration; do not append a query.
        return self.send_cmd(command)

    def _rgb_apply_bright(self):
        try:
            value = self._rgb_values([self._rgb_bright], [(0, 100)])[0]
            self._rgb_send(f"!RGB_BRIGHT {value}")
        except ValueError as error:
            self._action_problem("rgb", "应用亮度", str(error), "请输入 0～100 的整数后重试。",
                                 target="亮度", phase="invalid")

    def _rgb_apply_color(self):
        try:
            values = self._rgb_values(self._rgb_color_vars, [(0,255)] * 3)
            self._rgb_send(f"!RGB_COLOR {self._rgb_color_idx.current()} " + " ".join(map(str, values)))
        except ValueError as error:
            self._action_problem("rgb", "应用预设颜色", str(error), "请检查红、绿、蓝三个颜色值后重试。",
                                 target="单色预设", phase="invalid")

    def _rgb_apply_auto_color(self):
        try:
            values = self._rgb_values(self._rgb_auto_color_vars, [(0,255)] * 3)
            self._rgb_send(f"!RGB_AUTO_COLOR {self._rgb_auto_idx.current() + 1} " + " ".join(map(str, values)))
        except ValueError as error:
            self._action_problem("rgb", "应用自动颜色", str(error), "请检查红、绿、蓝三个颜色值后重试。",
                                 target="自动颜色", phase="invalid")

    def _rgb_apply_fades(self):
        try:
            values = self._rgb_values(self._rgb_fades, [(0,10000)] * 3 + [(0,5000)])
            self._rgb_send("!RGB_FADE " + " ".join(map(str, values)))
        except ValueError as error:
            self._action_problem("rgb", "应用过渡时间", str(error), "请检查过渡时间的范围及整数格式后重试。",
                                 target="过渡时间", phase="invalid")

    def _rgb_apply_effect(self):
        try:
            first = self._rgb_values(self._rgb_effect_first, [(0,255)] * 3)
            second = self._rgb_values(self._rgb_effect_second, [(0,255)] * 3)
            period, tail, on, off = self._rgb_values(self._rgb_effect_params,
                                                    [(250,30000), (1,12), (20,30000), (20,30000)])
            values = [self._rgb_effect_mode.current() + 3] + first + second + [
                period, int(self._rgb_reverse.get()), tail, on, off]
            self._rgb_send("!RGB_EFFECT " + " ".join(map(str, values)))
        except ValueError as error:
            self._action_problem("rgb", "应用动画参数", str(error), "请检查颜色、周期和宽度等参数后重试。",
                                 target="动画参数", phase="invalid")

    def _rgb_apply_state(self):
        try:
            state = RGB_STATES[self._rgb_state.current()][0]
            command = f"!RGB_SET_STATE {state} {self._rgb_state_mode.current()}"
            source = self._rgb_state_source.current()
            if source == 1:
                command += " " + " ".join(map(str, self._rgb_values(self._rgb_state_color, [(0,255)] * 3)))
            elif source >= 2:
                command += " " + RGB_COLOR_SOURCES[source]
            self._rgb_send(command)
        except ValueError as error:
            self._action_problem("rgb", "应用状态绑定", str(error), "请检查该状态的颜色参数后重试。",
                                 target="状态绑定", phase="invalid")

    def _rgb_apply_motion(self):
        try:
            if not self._rgb_motion_supported:
                raise ValueError("设备尚未确认支持独立自动动效，请查询全部；旧固件需升级。")
            slot, kind = self._rgb_motion_idx.current(), self._rgb_motion_kind.current()
            if slot not in range(4) or kind not in range(4):
                raise ValueError("请选择自动运动或校准动效")
            period, width = self._rgb_values(self._rgb_motion_params, [(250, 30000), (1, 12)])
            self._rgb_send(f"!RGB_MOTION {slot} {kind} {period} {int(self._rgb_motion_reverse.get())} {width}")
        except ValueError as error:
            self._action_problem("rgb", "应用自动动效", str(error),
                                 "请查询设备能力，并检查所选动效、周期和宽度后重试。",
                                 target="自动运动 / 校准动效", phase="invalid")

    def _rgb_save(self):
        if not self._motor_state_known or self._motor_enabled or self._stop_locked or self._calibration_busy or self._seq_active:
            self._action_problem("rgb", "保存RGB配置", "RGB 配置未保存：当前不满足保存条件。",
                                 "请先 DISABLE 并等待失能确认，结束校准、保护或顺序任务后重试。")
            return
        self._rgb_send("!RGB_SAVE")

    @_rgb_readback_guard
    def _rgb_set_fields(self, variables, values):
        for variable, value in zip(variables, values):
            variable.set(str(value))

    @_rgb_readback_guard
    def _rgb_fill_color(self):
        values = self._rgb_cache["colors"].get(self._rgb_color_idx.current())
        if values is not None:
            self._rgb_set_fields(self._rgb_color_vars, values)

    @_rgb_readback_guard
    def _rgb_fill_auto_color(self):
        values = self._rgb_cache["auto_colors"].get(self._rgb_auto_idx.current() + 1)
        if values is not None:
            self._rgb_set_fields(self._rgb_auto_color_vars, values)

    @_rgb_readback_guard
    def _rgb_fill_effect(self):
        values = self._rgb_cache["effects"].get(self._rgb_effect_mode.current() + 3)
        if values is not None:
            self._rgb_set_fields(self._rgb_effect_first, values[:3])
            self._rgb_set_fields(self._rgb_effect_second, values[3:6])
            self._rgb_set_fields(self._rgb_effect_params, [values[6], values[8], values[9], values[10]])
            self._rgb_reverse.set(bool(values[7]))

    @_rgb_readback_guard
    def _rgb_fill_state(self):
        values = self._rgb_cache["states"].get(RGB_STATES[self._rgb_state.current()][0])
        if values is not None:
            self._rgb_state_mode.current(values[0])
            self._rgb_state_source.current(values[1])
            self._rgb_set_fields(self._rgb_state_color, values[2:])

    @_rgb_readback_guard
    def _rgb_fill_motion(self):
        slot = self._rgb_motion_idx.current()
        if slot not in range(4):
            return
        values = self._rgb_cache["motion"].get(slot)
        binding = self._rgb_cache["states"].get(RGB_MOTION_STATES[slot])
        if values is not None:
            self._rgb_set_fields(self._rgb_motion_params, (values[0], values[2]))
            self._rgb_motion_reverse.set(bool(values[1]))
        if binding is not None and 13 <= binding[0] <= 16:
            self._rgb_motion_kind.current(binding[0] - 13)

    @staticmethod
    def _rgb_decode_fields(payload, limits):
        """Reject missing, extra, duplicate or malformed wire fields atomically."""
        fields = {}
        for token in payload.split():
            match = re.fullmatch(r"([a-z_]+)=([A-Z_]+|[0-9]+)", token)
            if not match or match[1] not in limits or match[1] in fields:
                return None
            key, value = match.groups()
            limit = limits[key]
            if isinstance(limit, set):
                if value not in limit:
                    return None
            else:
                if not value.isdigit() or int(value) > limit:
                    return None
                value = int(value)
            fields[key] = value
        return fields if fields.keys() == limits.keys() else None

    @_rgb_readback_guard
    def _handle_rgb_status(self, line):
        """Validate device readback before touching the snapshot or editor fields."""
        if not self.is_connected:
            return
        if line.startswith("error RGB "):
            self._rgb_query_failed()
            return
        state_names = {state for state, _ in RGB_STATES} | {"FAULT"}
        runtime_limits = {
            "RGB_STATUS": {"on": 1, "auto": 1, "bright": 100, "current": 100, "mode": 16},
            "RGB_RUNTIME": {"state": state_names, "source": state_names | {"MANUAL", "OFF"}},
            "RGB_STORAGE": {"dirty": 1, "storage": 2, "dma": 1},
            "RGB_DMA": {"frames": 0xFFFFFFFF, "errors": 0xFFFFFFFF},
            "RGB_TIMEOUT": {"timeouts": 0xFFFFFFFF},
        }
        prefix, _, payload = line.partition(" ")
        if prefix in runtime_limits:
            if prefix == "RGB_STATUS":
                self._rgb_runtime_pending = {}
                self._rgb_runtime_parts = set()
                self._rgb_runtime_invalid = False
                self._rgb_runtime_ready_for_config = False
            fields = self._rgb_decode_fields(payload, runtime_limits[prefix])
            if fields is None:
                self._rgb_runtime_invalid = True
                self._rgb_invalid_readback()
                return
            if self._rgb_runtime_pending is None:
                return  # Never stitch an orphan tail onto a previous device status.
            self._rgb_runtime_pending.update(fields)
            self._rgb_runtime_parts.add(prefix)
            if self._rgb_runtime_parts != runtime_limits.keys() or self._rgb_runtime_invalid:
                return
            fields = self._rgb_runtime_pending
            self._rgb_runtime_pending = None
            if fields["source"] == "MANUAL" and fields["mode"] >= 13:
                self._rgb_invalid_readback()
                return
            self._rgb_runtime = dict(fields)
            storage = {0: "未保存默认", 1: "有效", 2: "无效 / 需重新保存"}[fields["storage"]]
            self._rgb_status_var.set(
                f"灯{'开' if fields['on'] else '关'} / {'自动' if fields['auto'] else '手动'}  "
                f"设定{fields['bright']}% / 当前过渡{fields['current']}%\n"
                f"状态 {fields['state']} / 来源 {fields['source']} / 灯效 {fields['mode']}  "
                f"修改{'未保存' if fields['dirty'] else '已保存'}\n"
                f"存储 {storage} / DMA错误 {fields['errors']} / 超时 {fields['timeouts']}")
            self._rgb_bright.set(str(fields["bright"]))
            self._rgb_scale.set(fields["bright"])
            self._accept_rgb_runtime(fields)
            return
        if prefix == "RGB_CAPS":
            if not self._rgb_full_active:
                self._start_rgb_config_response(seed_runtime=True)
            fields = self._rgb_decode_fields(payload, {"motion": 1})
            if fields is None:
                self._rgb_invalid_readback()
            else:
                self._accept_rgb_caps(fields)
            return
        config_limits = {
            "RGB_CONFIG": {"manual": 12, "fade_on": 10000, "fade_off": 10000},
            "RGB_TIMING": {"bright_ms": 10000, "transition": 5000},
        }
        if prefix in config_limits:
            fields = self._rgb_decode_fields(payload, config_limits[prefix])
            if fields is None:
                self._rgb_invalid_readback()
                return
            names = {"manual": "manual_mode", "fade_on": "fade_on_ms", "fade_off": "fade_off_ms",
                     "bright_ms": "brightness_ms", "transition": "transition_ms"}
            self._accept_rgb_config_fields({names[key]: value for key, value in fields.items()},
                                           start=prefix == "RGB_CONFIG")
            for index, key in enumerate(("fade_on", "fade_off", "bright_ms", "transition")):
                if key in fields:
                    self._rgb_fades[index].set(str(fields[key]))
            return
        if prefix in ("RGB_COLOR", "RGB_AUTO_COLOR", "RGB_EFFECT", "RGB_STATE", "RGB_MOTION"):
            match = re.fullmatch(r"RGB_COLOR ([0-2]) ((?:[0-9]+ ){2}[0-9]+)", line)
            kind = "colors"
            if prefix == "RGB_AUTO_COLOR":
                match = re.fullmatch(r"RGB_AUTO_COLOR ([1-3]) ((?:[0-9]+ ){2}[0-9]+)", line)
                kind = "auto_colors"
            elif prefix == "RGB_EFFECT":
                match = re.fullmatch(r"RGB_EFFECT ([3-9]) ((?:[0-9]+ ){10}[0-9]+)", line)
                kind = "effects"
            elif prefix == "RGB_STATE":
                match = re.fullmatch(r"RGB_STATE ([A-Z_]+) ((?:[0-9]+ ){4}[0-9]+)", line)
                kind = "states"
            elif prefix == "RGB_MOTION":
                match = re.fullmatch(r"RGB_MOTION ([0-3]) ((?:[0-9]+ ){2}[0-9]+)", line)
                kind = "motion"
            if not match:
                self._rgb_invalid_readback()
                return
            key = match[1] if kind == "states" else int(match[1])
            values = tuple(map(int, match[2].split()))
            valid = all(value <= 255 for value in values)
            if kind == "effects":
                valid = (all(value <= 255 for value in values[:6]) and 250 <= values[6] <= 30000 and
                         values[7] in (0, 1) and 1 <= values[8] <= 12 and
                         all(20 <= value <= 30000 for value in values[9:]))
            elif kind == "states":
                valid = (key in state_names and values[0] <= 16 and values[1] <= 4 and
                         (values[0] < 13 or key in RGB_MOTION_STATES) and
                         all(value <= 255 for value in values[2:]))
                if key == "FAULT":
                    valid = valid and values == (7, 1, 255, 0, 0)
            elif kind == "motion":
                valid = 250 <= values[0] <= 30000 and values[1] in (0, 1) and 1 <= values[2] <= 12
            if not valid:
                self._rgb_invalid_readback()
                return
            self._accept_rgb_config_table(kind, key, values)
            return
        if line == "RGB_GET end":
            self._rgb_fill_color()
            self._rgb_fill_auto_color()
            self._rgb_fill_effect()
            self._rgb_fill_state()
            self._rgb_fill_motion()
            self._finish_rgb_readback()

    @staticmethod
    def _parse_numbers(payload):
        """把一段响应文本解析成浮点列表；出现任何非数字字段就返回 None。"""
        nums = []
        for tok in payload.replace(",", " ").split():
            try:
                nums.append(float(tok))
            except ValueError:
                return None
        return nums

    def _process_rx_line(self, line):
        """在 Qt 主线程处理回执，避免从 RX 线程读取 IntVar 等 Qt 对象。"""
        # 裸数字 = 固件指令 FIFO 剩余空位：255 说明队列已满，其余仅是余量计数，静默过滤
        if line.isdigit():
            if line == "255":
                self.root.after(0, self.log,
                                "指令队列已满(FIFO 16)，该指令被固件丢弃，请降低发送速率", "WARN")
            return

        self.root.after(0, self.log, line, "RX")
        if line.startswith(("RGB_", "ok RGB ", "error RGB ")):
            self._handle_rgb_status(line)
        if line in ("ok SYNC_ACC pending", "ok SYNC_ACC finish") or line.startswith("error SYNC_ACC "):
            self.root.after(0, self._handle_acc_sync_status, line)
        if line in ("Stopping ok", "STOP locked", "STOP locked disabled", "Start pending",
                    "Disable pending", "Started ok", "Disabled ok", "error STOP_LOCKED") or line.startswith("STOP fault "):
            self._handle_stop_status(line)

        if line.startswith(("!CALIBRATION ", "CAL ", "error CALIBRATION ")):
            self._handle_calibration_status(line)
        elif line.startswith("STALL "):
            self._handle_stall_status(line)
        elif line.startswith("PROTECT ") and " failed reason=" in line:
            self._handle_communication_protection(line)
        elif line == "error START busy":
            self.log("参数 / RGB保存任务正在进行，START 未受理；完成后请重新发送，不会自动使能", "WARN")
        elif re.fullmatch(r"ok ENC_CAL \[[1-8]\] dispatched", line):
            self.log("磁编码器校准 CAN 帧已提交；协议没有完成回执，请观察电机校准及重启结果", "INFO")
        self._handle_protection_parameter_status(line)

        # Preserve the source node now. A delayed callback must never reinterpret
        # a J1 reply as the node selected after the reply arrived.
        m = re.fullmatch(r'ok PID \[([1-8])\] kp=(-?\d+) kv=(-?\d+) ki=(-?\d+) kd=(-?\d+)(?:\s+.*)?', line)
        if m:
            n = int(m.group(1))
            self._fill_pid_entries(*m.groups()[1:], node=n)
        m = re.fullmatch(r'ok ACC \[([1-8])\] = ([^\s]+) r/s\^2(?:\s+.*)?', line)
        if m:
            n = int(m.group(1))
            self._fill_acc_entry(m.group(2), node=n)
        m = re.fullmatch(r'ok I_LIMIT \[([1-8])\] = ([^\s]+) A(?:\s+.*)?', line)
        if m:
            n = int(m.group(1))
            self._fill_i_entry(m.group(2), node=n)
        # ok GET_* ALL dispatched / ok SET ... / error SET ... 等行已走 RX 日志，无需额外处理

        # #GETJPOS / #GETLPOS 回包 → 同步滑块（"ok" + 6 个数）
        # 注意：这里只落数据，不清 _pending_sync。两种回包格式完全相同，无法从内容
        # 区分来源，必须由 500ms 定时器统一收尾时再连同 kind 一起取走。
        if self._pending_sync and line.startswith("ok"):
            nums = self._parse_numbers(line[2:].strip())
            expected = 7 if self._pending_sync == "j" else 6
            if nums is not None and len(nums) == expected:
                self._sync_data = nums

        self._handle_motion_receipt(line)

    def _stop_sequence(self, reason):
        self._seq_active = self._seq_running = self._seq_wait_mode = False
        self._seq_motion_id = None
        self._seq_expected_type = None
        self._seq_session += 1
        self.log(f"顺序发送停止：{reason}", "WARN")

    def _query_command_mode(self):
        self._mode_request_counter = (self._mode_request_counter + 1) & 0xFFFFFFFF or 1
        self._mode_request = self._mode_request_counter
        if not self.send_cmd(f"#GET_CMDMODE {self._mode_request}"):
            self._mode_request = None
            if self._seq_active:
                self._stop_sequence("模式查询未发送")
            return
        self.root.after(3000, self._mode_query_timeout, self._mode_request)

    def _mode_query_timeout(self, request):
        if self._mode_request != request:
            return
        self._mode_request = None
        self.log("主控模式查询超时；未启动顺序发送", "WARN")
        if self._seq_active:
            self._stop_sequence("模式未确认")

    def _mode_set_timeout(self, mode, token):
        if self._mode_pending == mode and self._mode_set_token == token:
            self._mode_pending = None
            self.current_cmdmode = None
            self.log("模式设置回执超时，模式状态未知，请查询确认", "WARN")

    def _sequence_queue_timeout(self, session, index):
        if self._seq_active and session == self._seq_session and index == self._seq_idx and self._seq_motion_id is None:
            self._stop_sequence("未收到本条点位的 queued 编号；请检查固件版本和日志")

    def _handle_motion_receipt(self, line):
        m = re.fullmatch(r'ok CMDMODE mode=(\d+) busy=(\d+) issued=(\d+) req=(\d+)', line)
        if m:
            mode, busy, issued, request = map(int, m.groups())
            if request != self._mode_request:
                return
            self._mode_request = None
            self.current_cmdmode = mode
            if self._seq_wait_mode:
                self._seq_wait_mode = False
                if mode != 1 or busy:
                    self._stop_sequence("主控必须处于空闲 SEQ；请切换模式并等待当前运动完成")
                else:
                    self._seq_id_floor = issued
                    self.log(f"开始顺序发送 {len(self._seq_pending)} 个点位，等待各自编号完成", "INFO")
                    self._send_next_position()
            return
        m = re.fullmatch(r'ok CMDMODE mode=(\d+)', line)
        if m:
            mode = int(m.group(1))
            if mode == self._mode_pending:
                self._mode_pending = None
                self.current_cmdmode = mode
                self.log(f"主控确认模式 {mode} ({self._MODE_NAME.get(mode, '?')})", "INFO")
            return
        if line.startswith('error CMDMODE'):
            self._mode_pending = self._mode_request = None
            if self._seq_active:
                self._stop_sequence("主控拒绝模式请求")
            return
        if not self._seq_active or not self._seq_running:
            return
        m = re.fullmatch(r'queued (MOVEJ|MOVEL) id=(\d+) mode=(\w+)', line)
        if m:
            kind, motion_id, mode = m.group(1), int(m.group(2)), m.group(3)
            if kind != self._seq_expected_type:
                return
            delta = (motion_id - self._seq_id_floor) & 0xFFFFFFFF
            if self._seq_motion_id is None and 0 < delta < 0x80000000:
                if mode != 'SEQ':
                    self._stop_sequence("点位实际使用模式不是 SEQ")
                    self.send_cmd("!STOP")
                else:
                    self._seq_motion_id = motion_id
                    self._seq_id_floor = motion_id
            return
        m = re.fullmatch(r'ok (MOVEJ|MOVEL) id=(\d+)(?: ms=\d+)?', line)
        if m and m.group(1) == self._seq_expected_type and int(m.group(2)) == self._seq_motion_id:
            self._seq_running = False
            self._seq_motion_id = None
            self._seq_expected_type = None
            self._seq_idx += 1
            if self._seq_idx < len(self._seq_pending):
                session = self._seq_session
                self.root.after(0, lambda: self._send_next_position() if self._seq_active and session == self._seq_session else None)
            else:
                self._seq_active = False
                self.log(f"顺序发送完成，共 {len(self._seq_pending)} 个点位", "INFO")
            return
        m = re.fullmatch(r'error (MOVEJ|MOVEL) id=(\d+) reason=(\w+)', line)
        if m and m.group(1) == self._seq_expected_type and int(m.group(2)) == self._seq_motion_id:
            self._stop_sequence(m.group(3))
        elif line == 'error motion queue full' and self._seq_motion_id is None:
            self._stop_sequence("主控队列已满")

    def _qry_pid(self):
        self.send_cmd(f"#GET_PID {self.param_node.get()}")

    def _qry_acc(self):
        self.send_cmd(f"#GETJACC {self.param_node.get()}")

    def _qry_i(self):
        self.send_cmd(f"#GETI {self.param_node.get()}")

    def _sync_acc(self):
        """发起独立异步同步；查询按钮仍沿用 #GETJACC <node>。"""
        if self._acc_sync_waiting:
            return
        if not self.is_connected or not self.serial_port or self._stop_locked:
            self.send_cmd("#SYNC_ACC")  # 复用未连接 / STOP 锁定提示。
            return
        self._set_acc_sync_waiting(True)
        self.send_cmd("#SYNC_ACC")

    def _set_acc_sync_waiting(self, waiting):
        """只由 Qt 主线程调用；2 秒无终结回执时允许重试。"""
        if self._acc_sync_timeout_id is not None:
            self.root.after_cancel(self._acc_sync_timeout_id)
            self._acc_sync_timeout_id = None
        self._acc_sync_waiting = waiting
        self.btn_acc_sync.config(text="同步中…" if waiting else "同步加速度至主控",
                                 state="disabled" if waiting else "normal")
        if waiting:
            self._acc_sync_timeout_id = self.root.after(2000, self._acc_sync_no_reply)

    def _acc_sync_no_reply(self):
        self._acc_sync_timeout_id = None
        self._set_acc_sync_waiting(False)
        self.log("加速度同步未收到完成回执，请确认主控固件支持 #SYNC_ACC 后重试", "WARN")

    def _handle_acc_sync_status(self, line):
        if line == "ok SYNC_ACC pending":
            self._set_acc_sync_waiting(True)
        elif line == "ok SYNC_ACC finish":
            self._set_acc_sync_waiting(False)
            self.log("八节点加速度已回读并同步至主控缓存", "INFO")
        elif line == "error SYNC_ACC busy":
            # 已有同一 USB 通道的同步任务时，继续等它的终结回执。
            if self._acc_sync_waiting:
                self.log("主控已有加速度同步任务，继续等待结果", "WARN")
        elif line.startswith("error SYNC_ACC "):
            self._set_acc_sync_waiting(False)
            self.log(f"加速度同步失败：{line}", "ERROR")

    def _fill_pid_entries(self, kp, kv, ki, kd, node=None):
        """PID 回包解析后回填 Entry（仅 UI 辅助，不影响主日志）"""
        if node is not None and node != self.param_node.get():
            return
        for ent, val in [(self.ent_set_pid_kp, kp), (self.ent_set_pid_kv, kv),
                         (self.ent_set_pid_ki, ki), (self.ent_set_pid_kd, kd)]:
            ent.delete(0, "end")
            ent.insert(0, val)

    def _fill_acc_entry(self, val, node=None):
        """ACC 回包解析后回填 Entry"""
        if node is not None and node != self.param_node.get():
            return
        self.ent_set_acc.delete(0, "end")
        self.ent_set_acc.insert(0, val)

    def _fill_i_entry(self, val, node=None):
        """I_LIMIT 回包解析后回填 Entry"""
        if node is not None and node != self.param_node.get():
            return
        self.ent_set_i.delete(0, "end")
        self.ent_set_i.insert(0, val)

    def _update_write_btn_state(self):
        """节点=0 (ALL) 时禁用写入按钮，避免误写广播"""
        if hasattr(self, "select_parameter_node"):
            self.select_parameter_node(self.param_node.get())
        state = "disabled" if self.param_node.get() == 0 else "normal"
        for btn in self._write_buttons:
            btn.config(state=state)

    @staticmethod
    def _protection_default(kind, node, key, default):
        if kind == "STALL" and key == "BACK_RPS" and node == 7:
            return 0.5
        if kind == "STALL" and key == "ERROR_R":
            return 0.1 if node == 7 else 0.5 / 360 * (30 if node == 6 else 50)
        if kind == "CAL" and node == 8:
            return {"SEEK_RPS": 2, "RETURN_RPS": 2, "ACCEL": 5,
                    "CURRENT_A": 1, "CONTACT_RATIO": 0.2, "MAX_TRAVEL_R": 6,
                    "SEEK_TIMEOUT_MS": 30000, "RETURN_TIMEOUT_MS": 15000,
                    "BACKOFF_R": 0.5, "REPEAT_TOL_R": 0.1, "POSITION_TOL_R": 0.01,
                    "CONTACT_WINDOW_R": 5 / 100 * 100 / 360 * 16}.get(key, default)
        return default

    def _refresh_protection_node(self, kind):
        node = self._protection_nodes[kind].get()
        descriptors = STALL_PARAMETERS if kind == "STALL" else CAL_PARAMETERS
        cached = self._protection_values[kind].get(node, {})
        edits = getattr(self, "parameter_edits", None)
        if edits is not None:
            previous = self._selected_protection_nodes.get(kind)
            if previous is not None:
                self._capture_protection_drafts(kind, previous)
            self._selected_protection_nodes[kind] = node
        for key, _label, _unit, default, _minimum, _maximum in descriptors:
            applicable = not (kind == "STALL" and (node == 8 or
                             (key == "BACK_DEG" and node == 7) or
                             (key == "BACK_MM" and node not in (0, 7))))
            self._protection_rows[kind][key].config(state="normal" if applicable else "disabled")
            entry = self._protection_entries[kind][key]
            default = self._protection_default(kind, node, key, default)
            value = cached.get(key, f"{default:g}")
            if edits is not None:
                edits.ensure(kind, node, key, f"{default:g}")
                value = edits.snapshot(kind, node, key)["draft"]
            entry.delete(0, "end")
            entry.insert(0, value)
            if edits is not None:
                self._legacy_parameter_rendered[(kind, int(node), key)] = str(value)
        self._protection_save_buttons[kind].config(state="disabled" if kind == "STALL" and node == 8 else "normal")

    def _query_protection(self, kind):
        self.send_cmd(f"#{kind}_GET {self._protection_nodes[kind].get()}")

    @staticmethod
    def _is_stall_status_query(cmd):
        return re.fullmatch(r"#STALL_STATUS\s+[1-7](?:\s+LAST)?", cmd) is not None

    @staticmethod
    def _is_cal_status_query(cmd):
        return re.fullmatch(r"#CAL_STATUS\s+[78](?:\s+LAST)?", cmd) is not None

    def _is_protection_status_query(self, cmd):
        return (cmd in ("#RGB_STATUS", "#RGB_GET", "!RGB_BRIGHT") or
                self._is_stall_status_query(cmd) or self._is_cal_status_query(cmd))

    def _query_cal_status(self, last=False):
        node = self._protection_nodes["CAL"].get()
        if node not in (7, 8):
            self.log("校准快照仅支持单个节点 J7、J8，请切换节点", "WARN")
            return
        self.send_cmd(f"#CAL_STATUS {node}" + (" LAST" if last else ""))

    def _query_stall_status(self, last=False):
        node = self._protection_nodes["STALL"].get()
        if node not in range(1, 8):
            self.log("堵转快照仅支持单个节点 J1–J7，请切换节点", "WARN")
            return
        self.send_cmd(f"#STALL_STATUS {node}" + (" LAST" if last else ""))

    def _set_protection(self, kind, key):
        node = self._protection_nodes[kind].get()
        descriptors = STALL_PARAMETERS if kind == "STALL" else CAL_PARAMETERS
        descriptor = next(p for p in descriptors if p[0] == key)
        if kind == "STALL" and (node == 8 or (key == "BACK_DEG" and node == 7) or
                                (key == "BACK_MM" and node not in (0, 7))):
            self._action_problem("protection", "应用堵转保护参数", "该普通堵转参数不适用于所选节点。",
                                 "请选择支持该参数的节点。", target=f"{self._action_node(node)} · {descriptor[1]}")
            return
        try:
            value = float(self._protection_entries[kind][key].get())
            minimum, maximum = descriptor[4:6]
            if (not math.isfinite(value) or value < minimum or
                    (minimum == 0 and key not in ("STARTUP_MS", "ENABLE") and value == 0) or
                    (maximum is not None and value > maximum)):
                raise ValueError("超出参数范围")
            if (key.endswith("_MS") or key == "ENABLE") and not value.is_integer():
                raise ValueError("时间/开关参数必须为整数")
        except ValueError as exc:
            scope = "protection" if kind == "STALL" else "calibration"
            self._action_problem(scope, "应用堵转保护参数" if kind == "STALL" else "应用校准参数",
                                 f"{descriptor[1]}：{exc}（{descriptor[2]}）。",
                                 "请修正该参数后重试。",
                                 target=f"{self._action_node(node)} · {descriptor[1]}", phase="invalid")
            return
        self.send_cmd(f"#{kind}_SET {node} {key} {value:g}")

    def _save_protection(self, kind):
        if self._motor_state_known and self._motor_enabled:
            self._action_problem("protection" if kind == "STALL" else "calibration",
                                 "保存堵转保护参数" if kind == "STALL" else "保存校准参数",
                                 "参数未保存：电机仍处于使能状态。",
                                 "请先 DISABLE 并等待失能确认，再保存 EEPROM。",
                                 target=self._action_node(self._protection_nodes[kind].get()))
            return
        self.send_cmd(f"#{kind}_SAVE {self._protection_nodes[kind].get()}")

    def _handle_protection_parameter_status(self, line):
        match = re.fullmatch(r"ok (STALL|CAL)(?:_SET)? \[(\d+)\] ([A-Z_0-9]+)=([^\s]+)(?:\s+.*)?", line)
        if not match:
            return
        kind, node, key, value = match.group(1), int(match.group(2)), match.group(3), match.group(4)
        try:
            if not math.isfinite(float(value)):
                return
        except ValueError:
            return
        if kind == "STALL" and key == "BACK_DISTANCE_R" and 1 <= node <= 7:
            motor_r = float(value)
            key = "BACK_MM" if node == 7 else "BACK_DEG"
            value = f"{motor_r * 5 if node == 7 else motor_r * 360 / (30 if node == 6 else 50):.6g}"
        self._protection_values[kind].setdefault(node, {})[key] = value
        window = self._protection_window
        if (window is not None and window.winfo_exists() and
                self._protection_nodes[kind].get() == node and key in self._protection_entries[kind]):
            entry = self._protection_entries[kind][key]
            entry.delete(0, "end")
            entry.insert(0, value)

    def _cmd_encoder_calibration(self):
        """对电机参数区选中的单个节点触发磁编码器校准。"""
        try:
            node = self.param_node.get()
        except (ValueError, ValueError):
            self._action_problem("calibration", "磁编码器校准", "请选择单个节点 1～8 进行磁编码器校准。",
                                 "切换到具体电机节点后重试。", phase="invalid")
            return
        if node not in range(1, 9):
            self._action_problem("calibration", "磁编码器校准", "磁编码器校准仅支持单个节点 1～8，禁止 ALL 广播。",
                                 "请选择具体电机节点后重试。", target=self._action_node(node), phase="invalid")
            return
        cmd = f"#ENC_CAL {node}"
        if not self.is_connected or not self.serial_port:
            self.send_cmd(cmd)
            return
        if (self._stop_locked or self._calibration_busy or self._seq_active or self._acc_sync_waiting or
                not self._motor_state_known or self._motor_enabled):
            self._action_problem("calibration", "磁编码器校准", "磁编码器校准未开始：当前不满足校准条件。",
                                 "请结束当前任务、支撑好机械臂并失能，等待失能确认后重试。",
                                 target=self._action_node(node))
            return
        target = f"J{node}" if node <= 6 else "地轨 J7" if node == 7 else "夹爪 J8"
        if not messagebox.askyesno(
            "确认磁编码器校准",
            f"将校准 {target}（节点 {node}）的磁编码器。\n"
            "该电机会主动正反转采样、写入校准表，成功后自动重启。\n"
            "请支撑好机械臂，并确认该轴有足够运动空间。\n"
            "校准期间请勿发送其他控制命令；完成结果需观察电机。\n\n"
            "确定开始吗？",
            parent=self.root,
        ):
            return
        self.send_cmd(cmd)

    def _set_pid(self):
        """写 4 个 PID 值到指定节点（写 EEPROM，#SET_PID 一次发 4 个值）"""
        try:
            node = self.param_node.get()
            kp = int(self.ent_set_pid_kp.get())
            kv = int(self.ent_set_pid_kv.get())
            ki = int(self.ent_set_pid_ki.get())
            kd = int(self.ent_set_pid_kd.get())
        except ValueError:
            self._action_problem("motor", "保存PID", "PID 必须为整数。",
                                 "请检查所选节点及 Kp、Kv、Ki、Kd 后重试。", target="PID", phase="invalid")
            return
        self.send_cmd(f"#SET_PID {node} {kp} {kv} {ki} {kd}")

    def _set_acc(self):
        """应用节点加速度（不写 EEPROM）：发送 #ACC_J <node> <r/s²>"""
        try:
            node = self.param_node.get()
            acc = float(self.ent_set_acc.get())
        except ValueError:
            self._action_problem("motor", "应用加速度", "加速度必须为数值（r/s²）。",
                                 "请检查所选节点及加速度后重试。", target="加速度", phase="invalid")
            return
        if acc < 0:
            self._action_problem("motor", "应用加速度", "加速度不能为负数。",
                                 "请输入非负加速度后重试。", target=self._action_node(node), phase="invalid")
            return
        if acc > 5000.0:
            acc = 5000.0
        self.send_cmd(f"#ACC_J {node} {acc}")

    def _save_acc(self):
        """保存节点加速度到 EEPROM：发送 #ACC_J <node> <r/s²> &
        末尾 `&` 触发固件解析 canBuf[4]=1 走 SetAcceleration_persist 写 EEPROM（决策#2）。"""
        try:
            node = self.param_node.get()
            acc = float(self.ent_set_acc.get())
        except ValueError:
            self._action_problem("motor", "保存加速度", "加速度必须为数值（r/s²）。",
                                 "请检查所选节点及加速度后重试。", target="加速度", phase="invalid")
            return
        if acc < 0:
            self._action_problem("motor", "保存加速度", "加速度不能为负数。",
                                 "请输入非负加速度后重试。", target=self._action_node(node), phase="invalid")
            return
        if acc > 5000.0:
            acc = 5000.0
        self.send_cmd(f"#ACC_J {node} {acc} &")

    def _set_i(self):
        """应用节点电流限制（不写 EEPROM）：发送 #I_LIMIT_J <node> <A>"""
        try:
            node = self.param_node.get()
            i_limit = float(self.ent_set_i.get())
        except ValueError:
            self._action_problem("motor", "应用电流限制", "电流必须为数值（A）。",
                                 "请检查所选节点及电流后重试。", target="电流限制", phase="invalid")
            return
        if i_limit <= 0:
            self._action_problem("motor", "应用电流限制", "电流必须大于 0。",
                                 "请输入大于 0 的电流后重试。", target=self._action_node(node), phase="invalid")
            return
        if i_limit > 3.0:
            i_limit = 3.0
        self.send_cmd(f"#I_LIMIT_J {node} {i_limit}")

    def _save_i(self):
        """保存节点电流限制到 EEPROM：发送 #I_LIMIT_J <node> <A> &
        末尾 `&` 触发固件解析 canBuf[4]=1 走 SetCurrentLimit_persist 写 EEPROM（决策#2）。"""
        try:
            node = self.param_node.get()
            i_limit = float(self.ent_set_i.get())
        except ValueError:
            self._action_problem("motor", "保存电流限制", "电流必须为数值（A）。",
                                 "请检查所选节点及电流后重试。", target="电流限制", phase="invalid")
            return
        if i_limit <= 0:
            self._action_problem("motor", "保存电流限制", "电流必须大于 0。",
                                 "请输入大于 0 的电流后重试。", target=self._action_node(node), phase="invalid")
            return
        if i_limit > 3.0:
            i_limit = 3.0
        self.send_cmd(f"#I_LIMIT_J {node} {i_limit} &")


    def _cmd_hand_open(self):
        """!HAND_O — 张开方向电流（-A），电流幅值用 self.hand_current。"""
        a = self.hand_current.get()
        self.send_cmd(f"!HAND_O {a:.2f}")

    def _cmd_hand_close(self):
        """!HAND_C — 闭合方向电流（+A），电流幅值用 self.hand_current。"""
        a = self.hand_current.get()
        self.send_cmd(f"!HAND_C {a:.2f}")

    def _cmd_hand_set_i(self):
        """!HAND_I — 设置运行时电流幅值（纯 RAM，掉电丢）。"""
        try:
            a = float(self.hand_current.get())
            if not math.isfinite(a):
                raise ValueError("电流必须为有限数值")
        except (TypeError, ValueError):
            self._action_problem("gripper", "应用夹爪电流", "夹爪电流必须为有限数值（A）。",
                                 "请检查夹爪电流后重试。", target="运行电流", phase="invalid")
            return
        # 上限 2.5A / 下限 0.05A 已被 Spinbox 限定；额外夹一道防御
        if a < 0.05:
            a = 0.05
        elif a > 2.5:
            a = 2.5
        self.hand_current.set(a)
        self.send_cmd(f"!HAND_I {a:.2f}")

    def _cmd_hand_set_pos(self):
        """!HAND_POS — 设置位置 0~100（int）。"""
        p = self.hand_pos.get()
        if p < 0:
            p = 0
        elif p > 100:
            p = 100
        self.hand_pos.set(p)
        self.send_cmd(f"!HAND_POS {p}")

    def _cancel_motion_for_stop(self):
        self._set_acc_sync_waiting(False)
        self._tx_generation += 1
        self._seq_running = self._seq_active = self._seq_wait_mode = False
        self._seq_motion_id = None
        self._seq_expected_type = None
        self._seq_session += 1
        self._mode_pending = self._mode_request = None
        self._seq_pending = ()
        self._seq_idx = 0
        self._pending_sync = None
        self._sync_data = None
        # 解锁不重新开启拖发；迟到的 after 回调也受 send_cmd 门禁限制。
        self.movej_drag_enable.set(False)
        self.movel_drag_enable.set(False)
        self.hand_pos_drag_enable.set(False)
        while True:
            try:
                self._tx_queue.get_nowait()
                self._tx_queue.task_done()
            except queue.Empty:
                break

    def _schedule_motion_callback(self, callback):
        """拖发 after 回调带世代；STOP/保护取消后，即使解锁也不能复活旧回调。"""
        generation = self._tx_generation

        def run_if_current():
            if (generation == self._tx_generation and not self._stop_locked and
                    not self._calibration_busy):
                callback()

        self.root.after(1, run_if_current)

    def _handle_stop_status(self, line):
        if line == "Started ok":
            if self._calibration_busy:
                self.log("校准忙态收到 Started ok，保持运动门禁并核查主控状态", "WARN")
                return
            self._stop_locked = False
            self._update_motor_state(True)
            if self._mode_request is None and self._mode_pending is None:
                self._query_command_mode()
        elif line == "Disabled ok":
            self._stop_locked = False
            self._update_motor_state(False)
            if self._calibration_busy:
                self.lbl_motor_state.config(text="● 校准中 / 已失能，等待保存结果", fg="#ffd43b")
        else:
            if not self._stop_locked:
                self._cancel_motion_for_stop()
            self._stop_locked = True
            labels = {
                "Stopping ok": "● 软停减速中",
                "STOP locked": "● STOP 锁定 / 停点保持",
                "STOP locked disabled": "● STOP 锁定 / 电机失能",
                "Start pending": "● 等待停稳并解锁",
                "Disable pending": "● 等待八轴失能确认",
                "error STOP_LOCKED": "● STOP 锁定",
            }
            self.lbl_motor_state.config(text=labels.get(line, "● STOP 故障 / 保持锁定"),
                                        bg="#495057", fg="#ffd43b")

    def _update_motor_state(self, enabled):
        self._motor_state_known = enabled is not None
        self._motor_enabled = enabled is True
        if enabled is None:
            self.lbl_motor_state.config(text="● 电机状态未知", bg="#495057", fg="#ffd43b")
        elif enabled:
            self.lbl_motor_state.config(text="● 电机已使能", bg="#2b8a3e", fg="white")
        else:
            self.lbl_motor_state.config(text="● 电机已失能", bg="#495057", fg="#ff6b6b")

    def _cmd_start(self):
        self.send_cmd("!START")

    def _cmd_disable(self):
        self.send_cmd("!DISABLE")

    def _cmd_calibration(self):
        """失能入口检查由主控最终确认，不自动 DISABLE 或排队校准。"""
        if self._calibration_busy:
            self.send_cmd("!CALIBRATION")
            return
        if self._stop_locked or (self._motor_state_known and self._motor_enabled):
            self._warn_calibration_disabled()
            return
        if not self.is_connected or not self.serial_port:
            self.send_cmd("!CALIBRATION")
            return
        if not messagebox.askyesno(
            "确认置零",
            "J1～J6 保持失能，仅将当前位置定义为 REST_POSE。\n"
            "六轴坐标修正完成后等待1秒，J7、J8 并行自动校准。\n"
            "J8 寻闭合限位并返回开度100；J7 寻正限位并返回0。\n"
            "全部完成后自动八轴失能并保存 EEPROM，不重启。\n\n"
            "请支撑好当前六轴姿态，并留出地轨、夹爪运动空间。\n确定继续吗？",
            parent=self.root,
        ):
            return
        self.send_cmd("!CALIBRATION")

    def _warn_calibration_disabled(self):
        self._action_problem("calibration", "置零校准", "置零校准未开始：需要八轴失能且解除保护锁定。",
                             "请先点击 DISABLE 并等待失能确认，再重新点击校准。")

    def _handle_calibration_status(self, line):
        if line == "!CALIBRATION pending" or line.startswith("CAL "):
            if not self._calibration_busy:
                self._cancel_motion_for_stop()
            self._calibration_busy = True
            stage = "校准进行中" if line == "!CALIBRATION pending" else line[4:]
            self.lbl_motor_state.config(text=f"● 校准 {stage}", bg="#495057", fg="#ffd43b")
        elif line == "!CALIBRATION ok":
            self._cancel_motion_for_stop()
            self._calibration_busy = False
            self._stop_locked = False
            self._update_motor_state(False)
            self._fill_joints(JOINT_DEFAULTS + [RAIL_DEFAULT])
            self.hand_pos.set(100)
            self.lbl_hand_pos.config(text="100")
            self.log("校准、八轴失能与 EEPROM 保存完成；再次 START 将保持当前位置", "INFO")
        elif line == "error CALIBRATION busy":
            # 拒绝 START/重复指令不能结束仍在进行的校准。
            self._calibration_busy = True
        elif line.startswith("error CALIBRATION "):
            self._calibration_busy = False
            self._cancel_motion_for_stop()
            if "require DISABLE" in line or "state unavailable" in line:
                self._update_motor_state(None)
                self._warn_calibration_disabled()
            else:
                self._stop_locked = True
                self.lbl_motor_state.config(text="● 校准失败或取消 / 保护锁定", bg="#495057", fg="#ffd43b")

    def _handle_stall_status(self, line):
        if "EXCLUDED" in line or re.search(r"\bphase=(?:IDLE|0)(?:\s|$)", line):
            return
        if not self._stop_locked:
            self._cancel_motion_for_stop()
        self._calibration_busy = False
        self._stop_locked = True
        self.lbl_motor_state.config(text=f"● 堵转保护 {line[6:]}", bg="#495057", fg="#ffd43b")

    def _handle_communication_protection(self, line):
        if not self._stop_locked:
            self._cancel_motion_for_stop()
        self._calibration_busy = False
        self._stop_locked = True
        self.lbl_motor_state.config(text=f"● 通信/状态保护 {line[8:]}", bg="#495057", fg="#ffd43b")

    def _cmd_reboot(self):
        node = self.cb_node.get()
        self.send_cmd(f"#REBOOT {node}")

    def _set_cmdmode(self, mode):
        """仅主控确认后更新模式，活动任务中不切换。"""
        if self._seq_active or self._mode_pending is not None or self._mode_request is not None:
            self.log("顺序发送或模式确认中，请等待完成", "WARN")
            return
        self._mode_pending = mode
        self._mode_request_counter = (self._mode_request_counter + 1) & 0xFFFFFFFF or 1
        self._mode_set_token = self._mode_request_counter
        if not self.send_cmd(f"#CMDMODE {mode}"):
            self._mode_pending = None
            return
        self.root.after(3000, self._mode_set_timeout, mode, self._mode_set_token)

    def _warn_drag_mode(self, enabled=None):
        """仅顺序模式下提示拖发使用 INT 或 TRJ，不自动切换模式。"""
        if enabled is None:
            enabled = (self.movej_drag_enable.get() or self.movel_drag_enable.get()
                       or self.hand_pos_drag_enable.get())
        if enabled and self.current_cmdmode == 1:
            messagebox.showwarning(
                "模式提示",
                "当前为「顺序模式（SEQ，模式 1）」；拖发指令会排队执行。\n\n"
                "建议切换为「打断模式（INT，模式 2）」或「连续模式（TRJ，模式 3）」。\n"
                "请在「模式」区域选择模式，再点击「设置模式」。"
            )

    def _warn_seqsend_mode(self):
        """顺序发送按钮按下时回调。若不是「顺序模式(1)」或「连续模式(3)」则弹提示。"""
        if self.current_cmdmode not in (1, 3):
            cur = self._MODE_NAME.get(self.current_cmdmode, f"模式{self.current_cmdmode}")
            messagebox.showwarning(
                "模式提示",
                f"顺序发送建议使用「顺序模式」(模式 1)\n"
                f"当前是「{cur}」\n\n"
                f"请在「模式」区域手动点击「1:顺序」按钮切换。"
            )

    @staticmethod
    def _validated_speed(value):
        if isinstance(value, bool):
            raise ValueError("速度档位必须为数值，不能为布尔值")
        speed = float(value)
        if not math.isfinite(speed) or not 1 <= speed <= 100:
            raise ValueError("速度档位必须为 1–100 的有限数值（100 = 30 r/s）")
        return speed

    def send_movej(self):
        """>j1,j2,j3,j4,j5,j6,j7(rail),speed  （8 个值，v3.1 新协议）"""
        try:
            j = [float(ent.get()) for ent in self.ent_joints]
            j7 = float(self.ent_j7.get())
            speed = self._validated_speed(self.ent_j_speed.get())
            cmd = ">" + ",".join(f"{v:g}" for v in j) + f",{j7:g},{speed:g}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("错误", "请输入有效数字；速度档位必须为 1–100（100 = 30 r/s）")

    def send_movel(self):
        """@X,Y,Z,A,B,C,speed  （7 个值）"""
        try:
            p = [float(ent.get()) for ent in self.ent_pose]
            speed = self._validated_speed(self.ent_l_speed.get())
            cmd = "@" + ",".join(f"{v:g}" for v in p) + f",{speed:g}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("错误", "请输入有效数字；速度档位必须为 1–100（100 = 30 r/s）")

    def _fill_joints(self, nums):
        """把 7 个数写进滑块（新协议：nums[0..5]=J1..J6(°), nums[6]=Rail(mm)）"""
        self._suppress_drag_send = True
        try:
            # 同步 J1~J6 (nums[0..5])
            for i in range(6):
                v = nums[i]
                self.scl_joints[i].set(v)
                self.ent_joints[i].delete(0, "end")
                self.ent_joints[i].insert(0, f"{v:.2f}")
                self.lbl_joints[i].config(text=f"{v:.2f}")
            # 同步 Rail (nums[6])
            rv = nums[6]
            self.scl_j7.set(rv)
            self.ent_j7.delete(0, "end")
            self.ent_j7.insert(0, f"{rv:.2f}")
            self.lbl_j7.config(text=f"{rv:.2f} mm")
        finally:
            self._suppress_drag_send = False

    def _fill_pose(self, nums):
        """把 6 个位姿分量写进笛卡尔滑块 / 输入框 / 数值标签"""
        self._suppress_drag_send = True
        try:
            for i in range(6):
                self.scl_pose[i].set(nums[i])
                self.ent_pose[i].delete(0, "end")
                self.ent_pose[i].insert(0, f"{nums[i]:.2f}")
                self.lbl_pose[i].config(text=f"{nums[i]:.2f}")
        finally:
            self._suppress_drag_send = False

    def _current_joint_values(self):
        """读取 6 个关节 + 地轨（J7）的滑块值；非法输入时记录日志并返回 None"""
        try:
            jvals = [round(float(e.get()), 2) for e in self.ent_joints]
            j7 = round(float(self.ent_j7.get()), 2)
            return jvals + [j7]
        except ValueError:
            self.log("关节/地轨输入框中存在非法数字，请先修正后再操作", "ERROR")
            return None

    def read_current_position(self):
        """读取当前滑块值并打印到日志（v3.1：J1~J6 + Rail）"""
        pos = self._current_joint_values()
        if pos is None:
            return
        names = ["J1", "J2", "J3", "J4", "J5", "J6", "Rail"]
        msg = ", ".join(f"{n}={v}" for n, v in zip(names, pos))
        self.log(f"当前滑块: {msg}", "INFO")

    def save_current_position(self):
        """把当前 6 个关节滑块值存为一个点位"""
        pos = self._current_joint_values()
        if pos is None:
            return
        idx = len(self.stored_positions) + 1
        short_name = f"#{idx}"  # 列表用短名，日志用全量
        full_name = f"#{idx} [{','.join(str(v) for v in pos)}]"
        self.stored_positions.append((short_name, pos))
        self._pos_listbox.insert("end", short_name)
        self.log(f"已存储: {full_name}", "INFO")

    def _delete_position(self):
        idx = self._pos_listbox.curselection()
        if not idx:
            return
        idx = idx[0] if isinstance(idx, tuple) else idx
        self._pos_listbox.delete(idx)
        del self.stored_positions[idx]
        self.log(f"已删除第 {idx+1} 个点位", "INFO")

    def clear_all_positions(self):
        self.stored_positions.clear()
        self._pos_listbox.delete(0, "end")
        self.log("已清空所有存储点位", "INFO")

    def _on_select_position(self):
        idx = self._pos_listbox.curselection()
        if not idx:
            return
        idx = idx[0]
        try:
            point = self._normalize_teaching_point(self.stored_positions[idx])
        except (ValueError, TypeError, IndexError) as error:
            self.log(f"点位无效：{error}", "WARN")
            return
        if point.kind == "MOVEJ":
            self._fill_joints(point.target)
        else:
            self._fill_pose(point.target)
        full = f"{point.name} {point.kind} [{','.join(str(v) for v in point.target)}]"
        self.log(f"已加载: {full}", "INFO")

    @staticmethod
    def _normalize_teaching_point(record):
        if isinstance(record, (tuple, list)) and len(record) == 2:
            # Legacy old-assistant tuples carry only a name and seven joints.
            record = {"name": record[0], "type": "MoveJ", "joints": record[1],
                      "gripper": 100, "speed": DEFAULT_SPEED}
        normalized = validate_points([record])[0]
        kind = normalized["type"].upper()
        target = normalized["joints" if kind == "MOVEJ" else "pose"]
        return TeachingPoint(normalized["name"], kind, tuple(target),
                             normalized["gripper"], normalized["speed"],
                             normalized.get("frame", "base"), normalized.get("target_kind", "flange"),
                             tuple(normalized["tool"]) if "tool" in normalized else None)

    def _teaching_command(self, point, speed):
        speed = self._validated_speed(speed)
        if point.kind == "MOVEL" and point.frame == "world":
            from app.core.cartesian_backend import configured_tool, point_tool, world_target, native_command, tcp_command
            from dummy_motion import Status
            if self.cartesian.feedback.estimate(self.cartesian.clock())[0] != Status.OK:
                raise ValueError("世界 TCP 点位需要新关节反馈")
            rail = self.cartesian.feedback.sample.q[6]
            tool = point_tool(point, configured_tool(self))
            target = world_target(self.cartesian.model, point, rail, tool)
            return tcp_command(target, tool, speed) if self.cartesian.native_tcp else native_command(self.cartesian.model, target, rail, tool, speed)
        prefix = ">" if point.kind == "MOVEJ" else "@"
        command = prefix + ",".join(f"{value:g}" for value in point.target) + f",{speed:g}"
        self._validate_command(command)
        return command

    def send_typed_position(self, record, speed=None, continuous=False):
        """Explicitly send one valid MoveJ/MoveL target; gripper is preview only."""
        try:
            point = self._normalize_teaching_point(record)
            if continuous and point.kind == "MOVEL" and self.cartesian.native_tcp:
                self.log("当前固件支持发送单个笛卡尔目标；连续拖发暂不可用", "WARN")
                return False
            value = self._validated_speed(point.speed if speed is None else speed)
            command = self._teaching_command(point, value)
        except (ValueError, TypeError, OverflowError) as error:
            self.log(f"点位无效，未发送：{error}", "WARN")
            return False
        if point.kind == "MOVEL":
            return self.send_cartesian_point(point, speed)
        return self.send_cmd(command)

    def _get_pos_speed(self):
        try:
            return self._validated_speed(self.ent_pos_speed.get())
        except ValueError:
            messagebox.showerror("错误", "速度档位必须为 1–100 的有限数值（100 = 30 r/s）")
            return None

    def send_selected_position(self):
        """Explicitly send selected typed target at the teaching speed setting."""
        idx = self._pos_listbox.curselection()
        if not idx:
            self.log("请先选择一个点位", "WARN")
            return False
        idx = idx[0]
        speed = self._get_pos_speed()
        if speed is None:
            return False
        try:
            record = self.stored_positions[idx]
        except IndexError:
            self.log("点位选择已失效，请重新选择", "WARN")
            return False
        return self.send_typed_position(record, speed)

    def send_all_positions(self):
        """先确认主控 SEQ 且无任务，再逐条等待匹配编号的完成。"""
        if self._stop_locked or self._calibration_busy:
            self.log("保护锁定或校准忙，不能启动顺序发送", "WARN")
            return False
        if not self.stored_positions:
            self.log("没有存储的点位", "WARN")
            return False
        if self._seq_active or self._mode_pending is not None or self._mode_request is not None or self.cartesian.busy:
            self.log("顺序发送、诊断或模式确认正在进行中", "WARN")
            return False
        speed = self._get_pos_speed()
        if speed is None:
            return False
        try:
            snapshot = tuple(self._normalize_teaching_point(record) for record in self.stored_positions)
            for point in snapshot:
                self._teaching_command(point, speed)
        except (ValueError, TypeError, OverflowError) as error:
            self.log(f"顺序点位无效，未查询模式或发送：{error}", "WARN")
            return False
        self._seq_pending = snapshot
        self._seq_idx = 0
        self._seq_speed = speed
        self._seq_expected_type = None
        self._seq_active = self._seq_wait_mode = True
        self._seq_session += 1
        self._query_command_mode()
        return self._seq_active

    def _send_next_position(self):
        """只发当前会话的一条点位，旧 after 回调不能恢复取消的任务。"""
        if not self._seq_active or self._stop_locked or self._calibration_busy:
            return
        if self._seq_idx >= len(self._seq_pending):
            return
        point = self._seq_pending[self._seq_idx]
        self._seq_running = True
        self._seq_motion_id = None
        self._seq_expected_type = point.kind
        try:
            command = self._teaching_command(point, self._seq_speed)
        except ValueError as error:
            self._stop_sequence(str(error))
            return
        if not self.send_cmd(command, sequence=True):
            self._stop_sequence("点位发送失败")
            return
        session, index = self._seq_session, self._seq_idx
        self.root.after(3000, self._sequence_queue_timeout, session, index)

    def _send_with_history(self):
        cmd = self.ent_custom_cmd.get().strip()
        if cmd:
            if not self.cmd_history or self.cmd_history[-1] != cmd:
                self.cmd_history.append(cmd)
            self.cmd_history_idx[-1] = len(self.cmd_history) - 1
            self.send_cmd(cmd)
            self.ent_custom_cmd.delete(0, "end")

    def _history_up(self, event):
        if not self.cmd_history:
            return
        idx = self.cmd_history_idx[-1]
        if idx > 0:
            idx -= 1
            self.cmd_history_idx[-1] = idx
            self.ent_custom_cmd.delete(0, "end")
            self.ent_custom_cmd.insert(0, self.cmd_history[idx])

    def _history_down(self, event):
        if not self.cmd_history:
            return
        idx = self.cmd_history_idx[-1]
        if idx < len(self.cmd_history) - 1:
            idx += 1
            self.cmd_history_idx[-1] = idx
            self.ent_custom_cmd.delete(0, "end")
            self.ent_custom_cmd.insert(0, self.cmd_history[idx])
        else:
            self.cmd_history_idx[-1] = len(self.cmd_history)
            self.ent_custom_cmd.delete(0, "end")
