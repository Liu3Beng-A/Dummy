"""Qt robot controller: serial transport, state and migrated user operations."""
import math
import queue
import re
import time
from copy import deepcopy
import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, QTimer, Signal, Slot, Qt

from app.core.business import RobotBusiness
from app.core.fields import ValueField, ListField
from app.core.scheduler import QtScheduler
from app.core.rgb_save import RGBSaveSession
from app.core.operation_tracker import OperationTracker
from app.core.action_feedback import ActionFeedback, configuration_action
from app.core.parameter_edits import ParameterEdits
from app.core.gamepad_session import GamepadSession
from app.core.cartesian_session import CartesianSession
from app.core.cartesian_backend import configured_tool, point_tool, world_target, native_command, tcp_command
from app.protocol import capabilities
from app.protocol.constants import *
from app.protocol.parser import finite_numbers
from app.transport.serial_transport import SerialTransport


_SYSTEM_JOINT_POSES = {
    "HOME": (0, 0, 90, 0, 0, 0, 0),
    "RESET": tuple(JOINT_DEFAULTS) + (RAIL_DEFAULT,),
}


class RobotController(QObject, RobotBusiness):
    log_message = Signal(str, str)  # message, tag
    state_changed = Signal(dict)
    actual_pose_changed = Signal(list)  # J1..J6 in degrees, rail in millimetres
    actual_cartesian_changed = Signal(list)  # Device flange XYZABC; never an edit target.
    position_synced = Signal(str, list)  # Explicit successful target sync: MoveJ / MoveL.
    preset_pose_requested = Signal(str, list)  # HOME / RESET dispatched; preview only.
    preset_pose_completed = Signal(str, list)  # Matched HOME / RESET completion; target only.
    rgb_changed = Signal(dict)
    fields_changed = Signal()
    ports_changed = Signal(list)
    feedback_event = Signal(dict)  # Explicit user action feedback; never telemetry.
    parameter_edits_changed = Signal()

    def __init__(self, parent=None, serial_factory=None, scheduler=None):
        super().__init__(parent)
        self._serial_factory = serial_factory or serial.Serial
        self.root = scheduler or QtScheduler(self)
        if isinstance(self.root, QtScheduler):
            self.root.callback_finished.connect(self._publish)
            self.root.callback_failed.connect(lambda msg: self.log(msg, "ERROR"))
        self.serial_port = None
        self._transport = None
        self._connection_id = 0
        self._tx_queue = queue.Queue()
        self._tx_generation = 0
        self._control_owner = "ui"
        self.operations = OperationTracker()
        self.action_feedback = ActionFeedback(self.feedback_event.emit, clock=lambda: time.monotonic())
        self.parameter_edits = ParameterEdits(self.parameter_edits_changed.emit)
        self.feedback_event.connect(self.parameter_edits.update_feedback)
        self._wire_admitted = queue.Queue(maxsize=256)
        self.is_connected = False
        self.port_name = ""
        self.connection_type = "USB"
        self.current_cmdmode = None
        self._MODE_NAME = {1: "顺序(SEQ)", 2: "打断(INT)", 3: "连续(TRJ)", 4: "调试(TUN)", 7: "曲线透传(STREAM)"}
        self._motor_enabled = False
        self._motor_state_known = False
        self._stop_locked = False
        self._calibration_busy = False
        self._suppress_drag_send = False
        self._pending_sync = self._sync_data = None
        self._position_request = None
        self._position_token = 0
        self._parameter_request = None
        self._parameter_timeout_id = None
        self._parameter_token = 0
        self._last_position_warning = 0.0
        self.actual_pose = None
        self.last_actual_at = None
        self._actual_received_at = None
        self._actual_feedback_sequence = 0
        self.actual_cartesian_pose = None
        self.last_actual_cartesian_at = None
        self.stored_positions = []
        self._seq_running = self._seq_active = self._seq_wait_mode = False
        self._seq_motion_id = None
        self._seq_expected_type = None
        self._seq_id_floor = self._seq_session = self._seq_idx = 0
        self._seq_pending = ()
        self._seq_speed = DEFAULT_SPEED
        self.sequence_timeout_ms = 300000
        self._system_pose_active = None
        self._system_pose_awaiting = []
        self._system_pose_ambiguous = set()
        self._system_pose_last_queued_id = None
        self._mode_request_counter = 0
        self._mode_request = self._mode_pending = self._mode_set_token = None
        self._acc_sync_waiting = False
        self._acc_sync_timeout_id = None
        self._protection_window = None
        self._protection_values = {"STALL": {}, "CAL": {}}
        self._rgb_applying_readback = False
        self._rgb_monitoring = False
        self.rgb_save = RGBSaveSession(self)
        self._reset_rgb_state(emit=False)
        self.cmd_history = []
        self.cmd_history_idx = [0]
        self.last_hand_pos_send_time = self.last_movej_send_time = self.last_movel_send_time = 0
        self._default_fields()
        self._selected_parameter_node = int(self.param_node.get())
        self._selected_protection_nodes = {}
        self._legacy_parameter_rendered = {}
        self._motor_parameter_defaults = {key: str(field.get()) for key, field in self._motor_parameter_fields().items()}
        self._parameter_bound_defaults = set()
        self._poll_enabled = True
        self._poll_hz = 5
        self._poll_timer = QTimer(self)
        self._poll_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._poll_timer.timeout.connect(self._poll_position)
        self._stale_timer = QTimer(self)
        self._stale_timer.setInterval(500)
        self._stale_timer.timeout.connect(self._emit_state)
        self._stale_timer.start()
        self._rgb_monitor_timer = QTimer(self)
        self._rgb_monitor_timer.setInterval(100)
        self._rgb_monitor_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._rgb_monitor_timer.timeout.connect(self._poll_rgb)
        self.gamepad = GamepadSession(self)
        self.cartesian = CartesianSession(self)

    def _default_fields(self):
        for name, value in {
            "cb_ports": "", "cb_node": "0", "param_node": 1,
            "ent_set_pid_kp": "100", "ent_set_pid_kv": "100", "ent_set_pid_ki": "0",
            "ent_set_pid_kd": "0", "ent_set_acc": "10", "ent_set_i": "1",
            "btn_acc_sync": "同步加速度至主控", "lbl_motor_state": "电机状态未知",
            "movej_drag_enable": False, "movel_drag_enable": False, "hand_pos_drag_enable": False,
            "ent_j7": "0", "scl_j7": 0, "lbl_j7": "0 mm", "ent_j_speed": str(DEFAULT_SPEED),
            "ent_l_speed": str(DEFAULT_SPEED), "ent_pos_speed": str(DEFAULT_SPEED),
            "hand_current": 1.0, "hand_pos": 100, "lbl_hand_pos": "100", "ent_custom_cmd": "",
            "_rgb_status_var": "RGB 未回读；显示值仅供编辑。", "_rgb_bright": "100",
            "_rgb_scale": 100, "_rgb_reverse": False, "_rgb_motion_reverse": False,
        }.items():
            setattr(self, name, ValueField(value))
        for name, values in {
            "ent_joints": list(map(str, JOINT_DEFAULTS)), "scl_joints": JOINT_DEFAULTS,
            "lbl_joints": list(map(str, JOINT_DEFAULTS)), "ent_pose": list(map(str, POSE_DEFAULTS)),
            "scl_pose": POSE_DEFAULTS, "lbl_pose": list(map(str, POSE_DEFAULTS)),
            "_rgb_color_vars": ["0", "0", "255"], "_rgb_auto_color_vars": ["0", "255", "0"],
            "_rgb_fades": ["1000", "1000", "300", "300"],
            "_rgb_effect_first": ["255", "255", "255"], "_rgb_effect_second": ["0", "0", "0"],
            "_rgb_effect_params": ["4000", "3", "200", "1000"],
            "_rgb_motion_params": ["1400", "3"],
            "_rgb_state_color": ["0", "0", "255"],
        }.items():
            setattr(self, name, [ValueField(value) for value in values])
        for name in ("_rgb_color_idx", "_rgb_auto_idx", "_rgb_effect_mode", "_rgb_state",
                     "_rgb_state_mode", "_rgb_state_source", "_rgb_motion_idx", "_rgb_motion_kind"):
            setattr(self, name, ValueField())
        self._rgb_state_mode.current(3)
        self._write_buttons = []
        self._pos_listbox = ListField()
        self._protection_nodes = {"STALL": ValueField(1), "CAL": ValueField(7)}
        self._protection_entries = {}
        self._protection_rows = {}
        self._protection_save_buttons = {}
        for kind, descriptors in (("STALL", STALL_PARAMETERS), ("CAL", CAL_PARAMETERS)):
            self._protection_entries[kind] = {row[0]: ValueField(f"{row[3]:g}") for row in descriptors}
            self._protection_rows[kind] = {row[0]: ValueField() for row in descriptors}
            self._protection_save_buttons[kind] = ValueField()

    def bind_fields(self, **fields):
        """Bind explicit adapters built by MainWindow; values remain user editable."""
        for name, adapter in fields.items():
            if not hasattr(self, name):
                raise ValueError(f"未知界面字段：{name}")
            setattr(self, name, adapter)
        for key, name in {"KP": "ent_set_pid_kp", "KV": "ent_set_pid_kv", "KI": "ent_set_pid_ki",
                          "KD": "ent_set_pid_kd", "ACC": "ent_set_acc", "I": "ent_set_i"}.items():
            if name in fields and key not in self._parameter_bound_defaults:
                self._motor_parameter_defaults[key] = str(fields[name].get())
                self._parameter_bound_defaults.add(key)
        self._publish()

    def log(self, message, tag="INFO"):
        self.log_message.emit(str(message), tag)

    def notify_action(self, scope, operation, target="", phase="blocked", message="",
                      next_step="", evidence="local", task_id=None):
        return self.action_feedback.notify(scope, operation, target, phase, message,
                                           next_step, evidence, task_id)

    def feedback_snapshot(self, scope=None):
        return self.action_feedback.snapshot(scope)

    def wire_history(self):
        transport = getattr(self, '_transport', None)
        return transport.wire_log.snapshot() if transport else getattr(self, '_last_wire_history', '')

    def state(self):
        now = time.time()
        status = ""
        try:
            status = str(self.lbl_motor_state.get())
        except (AttributeError, TypeError):
            pass
        state = {
            "cartesian_backend": self.cartesian.backend.value if hasattr(self, "cartesian") else "firmware",
            "cartesian_busy": getattr(getattr(self, "cartesian", None), "busy", False), "cartesian_stage": getattr(getattr(self, "cartesian", None), "stage", "idle"),
            "cartesian_message": getattr(getattr(self, "cartesian", None), "message", "主控逆解"),
            "connected": self.is_connected, "port": self.port_name,
            "control_owner": self._control_owner,
            "connection_type": self.connection_type,
            "motor_enabled": self._motor_enabled if self._motor_state_known else None,
            "stop_locked": self._stop_locked, "calibration_busy": self._calibration_busy,
            "cmdmode": self.current_cmdmode, "sequence_active": self._seq_active,
            "sequence_index": self._seq_idx, "sequence_count": len(self._seq_pending),
            "sequence_type": self._seq_expected_type, "generation": self._tx_generation,
            "acc_sync_waiting": self._acc_sync_waiting, "status": status,
            "actual_stale": (not self.is_connected or self.last_actual_at is None or
                             now - self.last_actual_at > max(1.5, 3 / self._poll_hz)),
            "last_actual_at": self.last_actual_at,
            "actual_received_at": self._actual_received_at,
            "actual_feedback_sequence": self._actual_feedback_sequence,
            "polling_enabled": self._poll_enabled, "polling_hz": self._poll_hz,
            "polling_paused": self._position_polling_paused(),
            "actual_pose": self.actual_pose,
            "actual_cartesian_pose": deepcopy(self.actual_cartesian_pose),
            "parameter_busy": self._parameter_request is not None,
            "parameter_task": (self._parameter_request["command"] if self._parameter_request else None),
            "rgb_save_busy": self.rgb_save.active,
            "native_tcp": getattr(getattr(self, "cartesian", None), "native_tcp", False),
            "stop_confirmed": getattr(self, "_stop_confirmed", False),
            "mode_request_busy": self._mode_request is not None or self._mode_pending is not None,
            "mode_pending": self._mode_pending,
            "position_sync_busy": bool(self._position_request and self._position_request.get("source") != "telemetry"),
            "rgb_save_stage": self.rgb_save.stage,
            "rgb_save_message": self.rgb_save.message,
        }
        gamepad = self.gamepad.state() if hasattr(self, "gamepad") else {}
        state["operation"] = self.operations.snapshot({**state, "gamepad": gamepad,
            "rgb_save": {"active": self.rgb_save.active, "stage": self.rgb_save.stage,
                         "message": self.rgb_save.message, "started_at": getattr(self.rgb_save, "started_at", None)}})
        return state

    def _emit_state(self):
        self.action_feedback.expire()
        state = self.state()
        visual = {key: value for key, value in state.items()
                  if key not in ('last_actual_at', 'actual_pose', 'actual_cartesian_pose',
                                 'actual_received_at', 'actual_feedback_sequence')}
        # The dedicated 250 ms presentation timer owns elapsed-time refresh.
        # Telemetry must not turn the whole-window state signal into a clock.
        visual["operation"] = {key: value for key, value in state["operation"].items() if key != "elapsed_s"}
        if visual != getattr(self, '_last_visual_state', None):
            self._last_visual_state = deepcopy(visual)
            self.state_changed.emit(state)

    def _publish(self):
        self.fields_changed.emit()
        self._emit_state()

    def _reset_rgb_state(self, emit=True):
        """Discard readback and query clocks whenever the device identity changes."""
        self.rgb_save.reset()
        self._rgb_cache = {key: {} for key in ("colors", "auto_colors", "effects", "states", "motion")}
        self._rgb_device_config = {key: value for key, value in self._rgb_cache.items()}
        self._rgb_runtime = {}
        self._rgb_runtime_pending = None
        self._rgb_runtime_parts = set()
        self._rgb_runtime_invalid = False
        self._rgb_runtime_ready_for_config = False
        self._rgb_received_at = None
        self._rgb_complete = False
        self._rgb_full_active = self._rgb_config_started = self._rgb_full_invalid = False
        self._rgb_full_fields = set()
        self._rgb_full_caps = None
        self._rgb_motion_supported = False
        self._rgb_get_pending_until = None
        self._rgb_get_retry_at = 0.0
        self._rgb_next_status_at = 0.0
        if emit:
            self._emit_rgb()

    def rgb_snapshot(self):
        """Return device readback only; changing editable fields cannot change it."""
        return deepcopy({"config": self._rgb_device_config, "runtime": self._rgb_runtime,
                         "complete": self._rgb_complete, "connected": self.is_connected,
                         "motion_supported": self._rgb_motion_supported,
                         "save_busy": self.rgb_save.active, "save_stage": self.rgb_save.stage,
                         "save_message": self.rgb_save.message,
                         "received_at": self._rgb_received_at, "connection_id": self._connection_id})

    def _emit_rgb(self):
        self.rgb_changed.emit(self.rgb_snapshot())

    def _start_rgb_config_response(self, seed_runtime=False):
        self._rgb_cache = {key: {} for key in ("colors", "auto_colors", "effects", "states", "motion")}
        self._rgb_device_config = {key: value for key, value in self._rgb_cache.items()}
        self._rgb_full_active = True
        self._rgb_config_started = self._rgb_full_invalid = self._rgb_complete = False
        self._rgb_full_fields = set()
        self._rgb_full_caps = None
        self._rgb_motion_supported = False
        if seed_runtime and self._rgb_runtime_ready_for_config:
            values = self._rgb_runtime
            self._rgb_device_config.update(enabled=bool(values["on"]), automatic=bool(values["auto"]),
                                           brightness=values["bright"])
            self._rgb_full_fields.update(("enabled", "automatic", "brightness"))

    def _begin_rgb_get(self):
        now = time.monotonic()
        self._rgb_get_pending_until = now + 2.0
        self._rgb_get_retry_at = now + 5.0
        self._rgb_next_status_at = max(self._rgb_next_status_at, now + 0.1)
        self._rgb_runtime_ready_for_config = False
        self._start_rgb_config_response()
        self._emit_rgb()

    def _accept_rgb_runtime(self, values):
        self._rgb_received_at = time.monotonic()
        self._rgb_runtime_ready_for_config = True
        self._rgb_device_config.update(enabled=bool(values["on"]), automatic=bool(values["auto"]),
                                       brightness=values["bright"])
        # MANUAL is proof that the rendered mode is the configured manual mode.
        if values["source"] == "MANUAL":
            self._rgb_device_config["manual_mode"] = values["mode"]
        if self._rgb_full_active:
            self._rgb_full_fields.update(("enabled", "automatic", "brightness"))
        self._emit_rgb()

    def _accept_rgb_config_fields(self, fields, start=False):
        if not self._rgb_full_active:
            self._start_rgb_config_response(seed_runtime=True)
        if start:
            self._rgb_config_started = True
            if self._rgb_get_pending_until is None:
                # Structured setters return the same full block as GET. Do not
                # interleave a monitor query while that block is being received.
                now = time.monotonic()
                self._rgb_get_pending_until = now + 2.0
                self._rgb_get_retry_at = max(self._rgb_get_retry_at, now + 5.0)
        if self._rgb_full_fields.intersection(fields):
            self._rgb_full_invalid = True
        self._rgb_device_config.update(fields)
        self._rgb_full_fields.update(fields)
        self._emit_rgb()

    def _validate_rgb_extension_command(self, command):
        """The manual bank stays 0..12; new writes need confirmed capability."""
        tokens = command.split()
        if tokens[-1:] == ["&"]:
            tokens = tokens[:-1]
        if tokens[0] == "!RGB_MODE":
            if len(tokens) != 2 or not tokens[1].isdigit() or int(tokens[1]) not in range(13):
                raise ValueError("手动RGB预设仅支持0～12；自动动效在自动颜色页设置")
        elif tokens[0] == "!RGB_MOTION":
            if not self._rgb_motion_supported:
                raise ValueError("设备尚未确认支持独立自动动效，请查询全部；旧固件需升级。")
            if len(tokens) != 6 or not all(token.isdigit() for token in tokens[1:]):
                raise ValueError("自动动效格式：!RGB_MOTION 目标 形态 周期 反向 宽度")
            slot, kind, period, reverse, width = map(int, tokens[1:])
            if not (slot in range(4) and kind in range(4) and 250 <= period <= 30000 and
                    reverse in (0, 1) and 1 <= width <= 12):
                raise ValueError("自动动效参数超出范围")
        elif tokens[0] == "!RGB_SET_STATE" and len(tokens) >= 3:
            if not tokens[2].isdigit() or int(tokens[2]) not in range(17):
                raise ValueError("RGB状态动效须为0～16")
            if int(tokens[2]) >= 13:
                if tokens[1] not in RGB_MOTION_STATES:
                    raise ValueError("独立自动动效仅用于SEQ/INT/TRJ运动和校准")
                if not self._rgb_motion_supported:
                    raise ValueError("设备尚未确认支持独立自动动效，请查询全部；旧固件需升级。")

    def _accept_rgb_config_table(self, kind, key, values):
        if not self._rgb_full_active:
            self._start_rgb_config_response(seed_runtime=True)
        if key in self._rgb_cache[kind]:
            self._rgb_full_invalid = True
        self._rgb_cache[kind][key] = tuple(values)
        self._emit_rgb()

    def _accept_rgb_caps(self, fields):
        if not self._rgb_full_active:
            self._start_rgb_config_response(seed_runtime=True)
        if self._rgb_full_caps is not None:
            self._rgb_full_invalid = True
        self._rgb_full_caps = fields
        self._emit_rgb()

    def _rgb_invalid_readback(self):
        if self._rgb_full_active:
            self._rgb_full_invalid = True

    def _rgb_query_failed(self):
        self._rgb_get_pending_until = None
        self._rgb_runtime_pending = None
        self._rgb_runtime_ready_for_config = False
        if self._rgb_full_active:
            self._rgb_full_invalid = True
            self._rgb_complete = False
        self._emit_rgb()

    def _finish_rgb_readback(self):
        if not self._rgb_full_active:
            return  # An orphan terminator cannot invalidate a confirmed configuration.
        required = {"enabled", "automatic", "brightness", "manual_mode", "fade_on_ms", "fade_off_ms",
                    "brightness_ms", "transition_ms"}
        states = {state for state, _ in RGB_STATES} | {"FAULT"}
        extended = self._rgb_full_caps == {"motion": 1}
        extension_complete = (self._rgb_cache["motion"].keys() == set(range(4)) if extended else
                              not self._rgb_cache["motion"] and
                              all(binding[0] < 13 for binding in self._rgb_cache["states"].values()))
        self._rgb_complete = bool(
            self._rgb_full_active and self._rgb_config_started and not self._rgb_full_invalid and
            required <= self._rgb_full_fields and self._rgb_cache["colors"].keys() == {0, 1, 2} and
            self._rgb_cache["auto_colors"].keys() == {1, 2, 3} and
            self._rgb_cache["effects"].keys() == set(range(3, 10)) and
            self._rgb_cache["states"].keys() == states and extension_complete)
        self._rgb_motion_supported = self._rgb_complete and extended
        if self._rgb_complete and not extended:
            # Local preview may try the new shapes with a legacy device; these
            # defaults are not advertised as a firmware capability or written.
            effect = self._rgb_cache["effects"][8]
            self._rgb_cache["motion"].update({slot: (effect[6], effect[7], effect[8]) for slot in range(4)})
        self._rgb_full_active = False
        self._rgb_get_pending_until = None
        self._rgb_runtime_ready_for_config = False
        self._emit_rgb()

    def set_rgb_monitoring(self, enabled):
        """Visible ring previews share one read-only 10 Hz monitor."""
        self._rgb_monitoring = bool(enabled)
        if self._rgb_monitoring and self.is_connected:
            if not self._rgb_monitor_timer.isActive():
                self._rgb_monitor_timer.start()
                self._poll_rgb()
        else:
            self._rgb_monitor_timer.stop()

    def _poll_rgb(self):
        if (not self._rgb_monitoring or not self.is_connected or self.serial_port is None or
                self._parameter_request is not None or self._acc_sync_waiting or self.rgb_save.active):
            return
        now = time.monotonic()
        if self._rgb_get_pending_until is not None:
            if now < self._rgb_get_pending_until:
                return
            self._rgb_get_pending_until = None
            self._rgb_full_active = False
            self._rgb_full_invalid = True
            self._rgb_complete = False
            self._emit_rgb()
        if now < self._rgb_next_status_at:
            return  # GET and STATUS share the same automatic 100 ms budget.
        if not self._rgb_complete and now >= self._rgb_get_retry_at:
            self.send_cmd("#RGB_GET")
            return
        if now >= self._rgb_next_status_at:
            self.send_cmd("#RGB_STATUS")

    def set_cartesian_backend(self, backend):
        return self.cartesian.switch(backend)

    def send_cartesian_point(self, point, speed=None):
        value = point.speed if speed is None else self._validated_speed(speed)
        if point.frame == "base" and point.target_kind == "flange":
            return self.send_cmd(self._teaching_command(point, value))
        if self.cartesian.feedback.estimate(self.cartesian.clock())[0].value != "OK":
            self.log("世界 TCP 转主控 MoveL 需要新关节回读", "WARN")
            return False
        rail = self.cartesian.feedback.sample.q[6]
        tool = point_tool(point, configured_tool(self))
        target = world_target(self.cartesian.model, point, rail, tool)
        command = tcp_command(target, tool, value) if self.cartesian.native_tcp else native_command(self.cartesian.model, target, rail, tool, value)
        return self.send_cmd(command)

    def refresh_ports(self, connection_type=None):
        """Enumerate only; select the native USB controller without opening it."""
        devices = list(serial.tools.list_ports.comports())
        ports = [port.device for port in devices]
        selected = str(self.cb_ports.get()).strip()
        connection = (connection_type or self.connection_type).upper()
        if self.is_connected:
            selected = self.port_name or selected
        elif connection == "USB":
            controllers = [port.device for port in devices
                           if getattr(port, "vid", None) == 0x1209
                           and getattr(port, "pid", None) == 0x0D32]
            if selected in controllers:
                pass  # Retain an already selected controller among multiple units.
            elif len(controllers) == 1:
                selected = controllers[0]
            elif len(controllers) > 1:
                selected = ""
                self.log("检测到多个主控串口，请手动选择：" + "、".join(controllers), "WARN")
            elif selected not in ports:
                selected = ports[0] if ports else ""
        # UART4 and established connections retain their selection, including
        # temporarily missing ports. The UI's bound adapter owns one list update.
        self.cb_ports["values"] = ports
        self.cb_ports.set(selected)
        self.ports_changed.emit(ports)
        return ports

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect_port()
        else:
            self.connect_port(self.cb_ports.get(), self.connection_type)

    def connect_port(self, port, connection_type="USB"):
        if self.is_connected:
            self.disconnect_port()
        if not port:
            self.log("请选择串口", "WARN")
            return False
        if connection_type.upper() not in ("USB", "UART4"):
            raise ValueError("连接类型必须为 USB 或 UART4")
        try:
            port_object = self._serial_factory(port, BAUDRATE, timeout=0, write_timeout=1.0)
        except Exception as error:
            self.log(f"连接失败：{error}", "ERROR")
            return False
        self._connection_id += 1
        self.operations.reset(self._connection_id)
        self.action_feedback.new_connection()
        self.parameter_edits.set_connection(True, self._connection_id)
        self._reset_system_pose_tracking()
        self.connection_type = connection_type.upper()
        self.serial_port = port_object
        self.port_name = port
        self._tx_queue = queue.Queue()
        self._wire_admitted = queue.Queue(maxsize=256)
        self.is_connected = True
        self.current_cmdmode = None
        self.cartesian.feedback.sample = None
        self.cartesian.feedback.velocity[:] = 0
        self.cartesian.motor_acceleration[:] = 30.
        self.cartesian.acceleration_confirmed[:] = False
        self._motor_state_known = self._motor_enabled = False
        self._stop_locked = self._calibration_busy = False
        self._mode_request = self._mode_pending = None
        self._update_motor_state(None)
        self.actual_pose = None
        self.last_actual_at = None
        self._actual_received_at = None
        self._actual_feedback_sequence = 0
        self.actual_cartesian_pose = self.last_actual_cartesian_at = None
        # The default GET below publishes only after its in-flight ledger is
        # established; UI monitoring listeners cannot enqueue a second GET.
        self._reset_rgb_state(emit=False)
        self.cartesian.native_tcp = False
        self._transport = SerialTransport(port_object, self._tx_queue, self._can_write,
                                          self._connection_id, self)
        getattr(self._transport, "line_received_at", self._transport.line_received).connect(self._on_transport_line, Qt.ConnectionType.QueuedConnection)
        if hasattr(self._transport, "configure_joint_polling"):
            self._transport.telemetry_received.connect(self._on_telemetry_line, Qt.ConnectionType.QueuedConnection)
            self._transport.telemetry_timed_out.connect(self._on_telemetry_timeout, Qt.ConnectionType.QueuedConnection)
            self._transport.telemetry_allowed = self._can_write_telemetry
        self._transport.command_written.connect(self._on_written, Qt.ConnectionType.QueuedConnection)
        self._transport.failed.connect(self._on_transport_failure, Qt.ConnectionType.QueuedConnection)
        self._transport.wire_log.sink = getattr(self, 'wire_log_sink', None)
        # Coalesce every finite seven-axis readback for display, including
        # measured positions outside planning limits. Planning validates its
        # own feedback independently from the actual model.
        self._transport.line_observer = self.cartesian.diagnostics.receive
        self._transport.written_observer = self.cartesian.diagnostics.written
        self._transport.write_ready = self.rgb_save.ready_to_write
        self._transport.start()
        self.log(f"已连接 {port} @ {BAUDRATE} ({self.connection_type})")
        self.send_cmd("#RGB_GET")
        self._query_command_mode()
        self.send_cmd("#KIN_CONFIG")
        self.set_polling(self._poll_enabled, self._poll_hz)
        if self._rgb_monitoring:
            self.set_rgb_monitoring(True)
        self._publish()
        return True

    def disconnect_port(self):
        self.action_feedback.invalidate(self._tx_generation, self._connection_id, disconnected=True)
        self.rgb_save.feedback_disconnected()
        self.cartesian.native_tcp = False
        self._connection_id += 1  # Ignore callbacks from a previous connection.
        self.parameter_edits.set_connection(False, self._connection_id)
        self.operations.reset(self._connection_id)
        self.is_connected = False
        self.gamepad.disconnected()
        self._poll_timer.stop()
        self._rgb_monitor_timer.stop()
        self._cancel_motion_for_stop()
        self._reset_system_pose_tracking()
        transport, self._transport = self._transport, None
        if transport:
            transport.close()
            self._last_wire_history = transport.wire_log.snapshot()
            transport.deleteLater()
        self.serial_port = None
        self.actual_cartesian_pose = self.last_actual_cartesian_at = None
        self.current_cmdmode = None
        self._stop_locked = self._calibration_busy = False
        self._update_motor_state(None)
        self._rgb_status_var.set("RGB 已断开；显示值仅供编辑，需重新回读。")
        self._reset_rgb_state()
        self.log("已断开串口")
        self._publish()

    def close(self):
        self.cartesian.close()
        self.gamepad.close()
        self.set_rgb_monitoring(False)
        if self.is_connected or self._transport is not None:
            self.disconnect_port()
        self._poll_timer.stop()
        self._stale_timer.stop()
        self._reset_system_pose_tracking()
        if hasattr(self.root, "close"):
            self.root.close()

    @Slot(str, int)
    @Slot(str, int, float)
    def _on_transport_line(self, line, connection_id, received_at=None):
        if connection_id == self._connection_id and self.is_connected:
            self._process_rx_line(line, received_at)

    @Slot(str, int, float)
    def _on_telemetry_line(self, line, connection_id, received_at):
        if connection_id == self._connection_id and self.is_connected:
            self._process_rx_line(line, received_at, telemetry=True)

    @Slot(int)
    def _on_telemetry_timeout(self, connection_id):
        if connection_id == self._connection_id and self.is_connected and not self._position_polling_paused():
            now = time.monotonic()
            if now - self._last_position_warning > 5:
                self.log("位置回读超时，三维实际姿态数据已过期", "WARN")
                self._last_position_warning = now
            self._emit_state()

    def _can_write_telemetry(self, generation):
        """Worker-only admission for the single fixed read-only poll command.

        No Qt fields, control-command admission records or motion dispatch are
        touched here. Recheck the same polling pauses at the actual writer.
        """
        return bool(self.is_connected and self.serial_port is not None and
                    generation == self._tx_generation and self._position_request is None and
                    not self._position_polling_paused() and self.supports("#GETJPOS") and
                    self.rgb_save.allows_write(generation, "#GETJPOS"))

    def performance_snapshot(self):
        metrics = (self._transport.performance_snapshot() if self._transport is not None and
                   hasattr(self._transport, "performance_snapshot") else {})
        return {**metrics, "connected": self.is_connected, "requested_hz": self._poll_hz,
                "paused": self._position_polling_paused()}

    @Slot(str, int)
    def _on_written(self, command, connection_id):
        if connection_id == self._connection_id:
            self.rgb_save.activity()
            self.log(command, "TX")
            try:
                generation, admitted_command, admitted_connection = self._wire_admitted.get_nowait()
            except queue.Empty:
                return  # A notification without actual writer admission is not proof of TX.
            if (admitted_command != command or admitted_connection != connection_id or
                    generation != self._tx_generation):
                return
            self.operations.written(command, generation, connection_id)
            self.action_feedback.written(command, generation, connection_id)
            request = self._position_request
            if request and command == ("#GETJPOS" if request["kind"] == "j" else "#GETLPOS"):
                if request.get("source") == "gamepad":
                    self._discard_gamepad_position_request()
                else:
                    request["written_at"] = time.monotonic()
            self.cartesian.invalidate_acceleration(command)
            self.cartesian.command_written(command, generation, connection_id)
            self.gamepad.command_written(command, generation, connection_id)
            self._emit_state()

    @Slot(str, int)
    def _on_transport_failure(self, message, connection_id):
        if connection_id == self._connection_id:
            self.log(message, "ERROR")
            self.disconnect_port()

    def supports(self, command):
        return capabilities.supported(command, self.connection_type)

    @staticmethod
    def _is_position_command(command):
        fields = str(command).split()
        return bool(fields) and fields[0] in ("#GETJPOS", "#GETLPOS", "#GETRAILPOS")

    def _is_joint_telemetry_request(self, generation, command):
        request = self._position_request
        return bool(command == "#GETJPOS" and request and request["kind"] == "j" and
                    request.get("source") == "telemetry" and not request["sync"] and
                    request["generation"] == generation)

    def _can_write(self, generation, command):
        if not self.rgb_save.allows_write(generation, command):
            return False
        owned_diagnostic = self.cartesian.diagnostics.owns_command(generation, command)
        if owned_diagnostic is not None:
            return bool(owned_diagnostic and self.is_connected and generation == self._tx_generation and
                         (not self._stop_locked or self.cartesian.diagnostics._locked_command(command)) and not self._calibration_busy)
        # Only the diagnostic worker may authorize mode 7 or stream packets.
        # Recheck at the writer even if an obsolete caller queued them earlier.
        if (self.cartesian.native_tcp and self._control_owner == "gamepad" and command.startswith("@")):
            return False
        if command.startswith(("~S", "!STREAM_")):
            return False
        if command.startswith("#CMDMODE") and command not in ("#CMDMODE 1", "#CMDMODE 2", "#CMDMODE 3", "#CMDMODE 4"):
            return False
        if self._reject_plain_motion_in_stream_mode(command):
            return False
        diagnostic = capabilities.diagnostic(command)
        owned_motion = self.cartesian.busy and (self._is_arm_motion(command) or command.startswith("#CMDMODE"))
        allowed = (self.is_connected and generation == self._tx_generation and
                (not owned_motion or self.cartesian.can_write(generation, command)) and
                (self._control_owner != "gamepad" or not command.startswith("@") or
                 self.gamepad.can_write_stream(generation, command)) and
                (self._control_owner != "gamepad" or not self._is_position_command(command) or
                 self._is_joint_telemetry_request(generation, command)) and
                (not self._stop_locked or diagnostic or command in ("!STOP", "!START", "!DISABLE")) and
                (not self._calibration_busy or diagnostic or command in ("!CALIBRATION", "!STOP", "!DISABLE")))
        if allowed:
            if not self.rgb_save.admitted(generation, command):
                return False
            try:
                self._wire_admitted.put_nowait((generation, command, self._connection_id))
            except queue.Full:
                return False
            self.operations.admitted(command, generation, self._connection_id)
            self.action_feedback.admitted(command, generation, self._connection_id)
        return allowed

    @staticmethod
    def _is_arm_motion(command):
        return command.startswith(("@", ">", "&")) or command in ("!HOME", "!RESET")

    def _reject_plain_motion_in_stream_mode(self, command):
        # Check again at actual writer admission: a native command may have
        # waited in the queue while a reconnect/mode receipt confirmed mode 7.
        # DiagnosticSession-owned packets are admitted before this ordinary path.
        if self.current_cmdmode == 7 and self._is_arm_motion(command):
            self.log("主控实际处于模式 7，普通运动指令未发送；请显式关闭诊断会话并确认主控模式后重试", "WARN")
            return True
        return False

    def send_cmd(self, command, sequence=False, source="ui"):
        """Only command dispatch path; STOP invalidates every pending generation."""
        command = str(command).strip()
        feedback_spec = configuration_action(command) if source == "ui" else None
        feedback_id = None
        if feedback_spec is not None:
            if feedback_spec["scope"] in ("motor", "gripper"):
                self._capture_motor_drafts()
            elif feedback_spec["scope"] in ("protection", "calibration"):
                kind = "STALL" if feedback_spec["scope"] == "protection" else "CAL"
                self._capture_protection_drafts(kind, int(self._protection_nodes[kind].get()))

        def reject(message, tag="WARN", phase="blocked"):
            self.log(message, tag)
            if feedback_spec is not None:
                self.notify_action(feedback_spec["scope"], feedback_spec["operation"],
                                   feedback_spec["target"], phase, message,
                                   "请根据提示调整后重试。", task_id=feedback_id)

        if not self.is_connected or self.serial_port is None:
            reject("未连接串口", "WARN")
            return False
        if not command or any(char in command for char in ("\r", "\n", "\x00")):
            reject("一次只能发送一条非空 ASCII 指令", "WARN")
            return False
        if not command.isascii() or len(command.encode()) > 256:
            reject("指令须为 ASCII 且长度不能超过 256 字节", "WARN")
            return False
        if source not in ("ui", "gamepad", "telemetry", "cartesian"):
            reject("未知指令来源", "WARN")
            return False
        if self.rgb_save.active and command not in RGBSaveSession.SAFETY_COMMANDS:
            reject("RGB 保存事务尚未结束；后台查询已暂停，可断开连接或发送 STOP / DISABLE", "WARN")
            return False
        shared_joint_poll = (source == "telemetry" and
                             self._is_joint_telemetry_request(self._tx_generation, command))
        if (self.cartesian.busy and not shared_joint_poll and
                not capabilities.diagnostic(command) and command not in ("!STOP", "!DISABLE")):
            # Hand position remains independent during gamepad ownership.
            if not (source == "gamepad" and command.startswith("!HAND_POS ")):
                reject("诊断或模式恢复处理中，请等待完成或 STOP", "WARN")
                return False
        if self._is_position_command(command) and (source == "gamepad" or
                (self._control_owner == "gamepad" and not
                 (source == "telemetry" and self._is_joint_telemetry_request(self._tx_generation, command)))):
            reject("手柄控制中仅使用共享关节轮询更新实际模型，请勿手动同步编辑目标", "WARN")
            return False
        if source == "gamepad" and not self.gamepad.claimed and command not in ("!STOP", "!DISABLE"):
            reject("手柄尚未接管，指令未发送", "WARN")
            return False
        if self._control_owner == "gamepad" and source == "ui" and command not in ("!STOP", "!DISABLE"):
            verb = command.split()[0]
            readonly = capabilities.diagnostic(command) or verb in {"#GET_PID", "#GETJACC", "#GETI", "#STACK"}
            if not readonly:
                reject("手柄控制中，请先释放控制；STOP / DISABLE 始终可用", "WARN")
                return False
        if not self.supports(command):
            reject(f"当前 UART4 固件不支持 {command.split()[0]}；请使用 USB", "WARN")
            return False
        if (self._parameter_request is not None and command not in ("!STOP", "!DISABLE") and
                not capabilities.diagnostic(command)):
            reject("堵转/校准参数读写中，请等待完整回执；可发送 STOP 或 DISABLE", "WARN")
            return False
        if (self.connection_type == "UART4" and self._motor_state_known and not self._motor_enabled and
                command in ("#GETJPOS", "#GETLPOS", "#GETRAILPOS", "#STACK")):
            reject("当前 UART4 固件失能时不处理位置/栈查询；实际显示暂停，请使用 USB 查询", "WARN")
            return False
        try:
            self._validate_command(command)
            self._validate_rgb_extension_command(command)
        except ValueError as error:
            reject(str(error), "WARN", phase="invalid")
            return False
        if self._seq_active and not sequence and (command.startswith((">", "&", "@", "#CMDMODE")) or
                                                  command in ("!HOME", "!RESET")):
            reject("顺序发送中，请等待完成或先 STOP", "WARN")
            return False
        diagnostic = capabilities.diagnostic(command)
        if self._calibration_busy and not diagnostic and command not in ("!STOP", "!DISABLE"):
            reject("校准及保存中，只接受 STOP、DISABLE 和保护快照查询", "WARN")
            return False
        if command.startswith('#CMDMODE'):
            if command not in ('#CMDMODE 1', '#CMDMODE 2', '#CMDMODE 3', '#CMDMODE 4', '#CMDMODE 7'):
                reject('普通模式指令须为 #CMDMODE 1～4；模式 7 仅用于独立诊断', 'WARN')
                return False
            mode = int(command[-1])
            if mode == 7 or self.current_cmdmode == 7:
                return self.cartesian.select_mode(mode)
            if mode not in (1, 2, 3, 4):
                reject('普通运动仅支持模式 1～4；模式 7 仅用于独立诊断', 'WARN')
                return False
        if self._stop_locked and not diagnostic and command not in ("!STOP", "!START", "!DISABLE"):
            reject("保护锁定中，请 START 或 DISABLE；STOP 可中断保护动作", "WARN")
            return False
        if command.startswith("#ENC_CAL"):
            if re.fullmatch(r"#ENC_CAL[ \t]+[1-8]", command) is None:
                reject("磁编码器校准只支持单个节点 1～8", "WARN")
                return False
            if self._seq_active or self._acc_sync_waiting or not self._motor_state_known or self._motor_enabled:
                reject("请结束任务并失能，等待 Disabled ok 后校准磁编码器", "WARN")
                return False
        if RGBSaveSession.is_save(command) or command.startswith(("#STALL_SAVE", "#CAL_SAVE")):
            if (not self._motor_state_known or self._motor_enabled or self._seq_active or
                    self._acc_sync_waiting):
                reject("写入 EEPROM 需要结束任务并等待 Disabled ok", "WARN")
                return False
        if RGBSaveSession.is_save(command):
            if self._parameter_request is not None or self._acc_sync_waiting or self.cartesian.busy:
                reject("请等待现有参数 / 诊断任务完成后保存 RGB", "WARN")
                return False
            return self.rgb_save.begin(command)
        if command == "!CALIBRATION":
            if self._motor_state_known and self._motor_enabled:
                self._warn_calibration_disabled()
                return False
            self._cancel_motion_for_stop()
            self._calibration_busy = True
            self.lbl_motor_state.config(text="校准入口检查中")
        if command in ("!STOP", "!DISABLE"):
            wait_for_flash = not self.rgb_save.ready_to_write(self._tx_generation, command)
            if wait_for_flash:
                reject("RGB 正在写入 Flash；已记录停止 / 失能请求，设备确认后优先发送。", "WARN")
            if source != "gamepad":
                self.gamepad.external_stop(command)
            self._cancel_motion_for_stop()
            self._stop_locked = True
            if wait_for_flash:
                action = "STOP" if command == "!STOP" else "失能"
                self._stop_confirmed = False
                self.lbl_motor_state.config(text=f"● {action} 请求待发送 / 等待 RGB 保存结果")
            else:
                self._handle_stop_status("Stopping ok" if command == "!STOP" else "Disable pending")
        if self._reject_plain_motion_in_stream_mode(command):
            return False
        if command.startswith(("~S", "!STREAM_")):
            reject("透传会话与恒定曲线仅由独立诊断入口发送", "WARN")
            return False
        if feedback_spec is not None:
            feedback_id = self.action_feedback.requested(command, self._tx_generation, self._connection_id)
        parameter_spec = self._parameter_spec(command)
        if parameter_spec is not None:
            parameter_spec["feedback_id"] = feedback_id
            self._begin_parameter_request(parameter_spec)
        try:
            self._tx_queue.put((self._tx_generation, command))
        except Exception as error:
            if parameter_spec is not None:
                self._finish_parameter_request(False, "指令入队失败")
            self.action_feedback.finish(feedback_id, False, "指令入队失败")
            reject(f"指令入队失败：{error}", "ERROR")
            return False
        self.parameter_edits.record_sent(command, feedback_id, self._connection_id)
        self.operations.requested(command, self._tx_generation, self._connection_id, source)
        self._track_system_pose_command(command)
        if command in ("!HOME", "!RESET"):
            kind = command[1:]
            self.preset_pose_requested.emit(kind, list(_SYSTEM_JOINT_POSES[kind]))
        if command == "#RGB_GET":
            self._begin_rgb_get()
        elif command == "#RGB_STATUS":
            self._rgb_next_status_at = time.monotonic() + 0.1
        self._publish()
        return True

    def _discard_system_pose_request(self, request):
        timeout = request.pop("timeout", None)
        if timeout is not None:
            self.root.after_cancel(timeout)
        self._system_pose_awaiting = [item for item in self._system_pose_awaiting if item is not request]
        if self._system_pose_active is request:
            self._system_pose_active = None

    def _invalidate_system_pose(self):
        request = self._system_pose_active
        self._system_pose_active = None
        if request is not None and request["id"] is not None:
            self._discard_system_pose_request(request)
        # Keep unnumbered requests as FIFO tombstones: their delayed queued
        # receipt must never be assigned to a newer request of the same kind.

    def _reset_system_pose_tracking(self):
        requests = list(self._system_pose_awaiting)
        if self._system_pose_active is not None:
            requests.append(self._system_pose_active)
        for request in requests:
            self._discard_system_pose_request(request)
        self._system_pose_awaiting.clear()
        self._system_pose_ambiguous.clear()
        self._system_pose_last_queued_id = None

    def _mark_system_pose_ambiguous(self, kind, reason):
        requests = [item for item in self._system_pose_awaiting if item["kind"] == kind]
        if self._system_pose_active is not None and self._system_pose_active["kind"] == kind:
            requests.append(self._system_pose_active)
        for request in requests:
            self._discard_system_pose_request(request)
        if kind not in self._system_pose_ambiguous:
            self.log(f"{kind} 回执无法对应（{reason}）；本连接暂不按完成回执同步该预设，重新连接可恢复", "WARN")
        self._system_pose_ambiguous.add(kind)

    def _track_system_pose_command(self, command):
        if (command.startswith((">", "&", "@", "#CMDMODE")) or
                command in ("!HOME", "!RESET", "!STOP", "!DISABLE", "!CALIBRATION")):
            self._invalidate_system_pose()
        kind = command[1:] if command in ("!HOME", "!RESET") else None
        if kind is None or kind in self._system_pose_ambiguous:
            return
        if len(self._system_pose_awaiting) >= 32:
            self._mark_system_pose_ambiguous(kind, "未编号请求过多")
            return
        request = {"kind": kind, "id": None, "connection": self._connection_id,
                   "generation": self._tx_generation}
        self._system_pose_awaiting.append(request)
        self._system_pose_active = request
        request["timeout"] = self.root.after(self.sequence_timeout_ms, self._system_pose_timeout, request)

    def _system_pose_timeout(self, request):
        if (request["connection"] != self._connection_id or
                not any(item is request for item in self._system_pose_awaiting) and
                self._system_pose_active is not request):
            return
        if request["id"] is None:
            # queued has no client request token. Forgetting an unnumbered
            # request and binding the next queued to a new one is unsafe.
            self._mark_system_pose_ambiguous(request["kind"], "等待 queued 超时")
        else:
            self._discard_system_pose_request(request)
            self.log(f"{request['kind']} 完成回执超时；保留当前模型目标", "WARN")

    def _handle_system_pose_receipt(self, line):
        queued = re.fullmatch(r"queued (HOME|RESET|MOVEJ|MOVEL) id=([0-9]+) mode=(SEQ|INT|TRJ|TUN)", line)
        if queued:
            kind, motion_id, _mode = queued.groups()
            motion_id = int(motion_id)
            if not 1 <= motion_id <= 0xFFFFFFFF:
                return
            previous = self._system_pose_last_queued_id
            if previous is not None and not 0 < (motion_id - previous) & 0xFFFFFFFF < 0x80000000:
                return
            self._system_pose_last_queued_id = motion_id
            request = next((item for item in self._system_pose_awaiting if item["kind"] == kind), None)
            if request is None:
                if self._system_pose_active is not None and self._system_pose_active["id"] is not None:
                    self._invalidate_system_pose()
                return
            self._system_pose_awaiting = [item for item in self._system_pose_awaiting if item is not request]
            request["id"] = motion_id
            if (self._system_pose_active is not request or request["connection"] != self._connection_id or
                    request["generation"] != self._tx_generation or self._stop_locked or self._calibration_busy):
                self._discard_system_pose_request(request)
            return
        completed = re.fullmatch(r"ok (HOME|RESET) id=([0-9]+)(?: ms=([0-9]+))?", line)
        failed = re.fullmatch(r"error (HOME|RESET) id=([0-9]+) reason=[A-Z_0-9]+", line)
        request = self._system_pose_active
        if request is None:
            return
        if line == "Disabled ok":
            self._invalidate_system_pose()
            return
        if line in ("error DISABLED", "error STOP_LOCKED"):
            if request["id"] is None:
                self._mark_system_pose_ambiguous(request["kind"], "未编号请求被拒绝")
            else:
                self._discard_system_pose_request(request)
            return
        match = completed or failed
        if not match:
            return
        kind, motion_id = match.group(1), int(match.group(2))
        if not 1 <= motion_id <= 0xFFFFFFFF or kind != request["kind"]:
            return
        if completed and completed.group(3) is not None and int(completed.group(3)) > 0xFFFFFFFF:
            return
        if failed and request["id"] is None:
            self._mark_system_pose_ambiguous(kind, "错误回执缺少对应 queued")
            return
        if request["id"] != motion_id:
            return
        current = (request["connection"] == self._connection_id and
                   request["generation"] == self._tx_generation and self.is_connected and
                   not self._stop_locked and not self._calibration_busy)
        self._discard_system_pose_request(request)
        if completed and current:
            self.preset_pose_completed.emit(kind, list(_SYSTEM_JOINT_POSES[kind]))

    @staticmethod
    def _validate_command(command):
        if command.split()[0] in ("#FW_VERSION", "#CAN_DIAG"):
            if not capabilities.identity_query(command):
                raise ValueError("固件版本查询需节点 0～8，CAN 诊断需节点 1～8；可选请求号为 1～65535")
        elif command.startswith("@T"):
            tokens = command[2:].split(",")
            parsed = [finite_numbers(token, 1) for token in tokens]
            values = None if any(value is None for value in parsed) else [value[0] for value in parsed]
            if values is None or len(values) not in (8, 14) or len(command.encode("ascii")) > 159:
                raise ValueError("TCP指令须包含8或14个有限数值，最长159字节")
            if any(abs(value) > 3.4028234663852886e38 or (value != 0 and abs(value) < 1.1754943508222875e-38) for value in values):
                raise ValueError("TCP数值超出主控float32范围")
            if not 1 <= values[6] <= 100 or values[7] not in (0, 1):
                raise ValueError("TCP速度档位1～100，自动地轨标志为0或1")
            if len(values) == 14 and any(abs(value) > 500 for value in values[8:11]):
                raise ValueError("工具偏移不得超过±500mm")
        elif command[0] in (">", "&", "@"):
            count = 7 if command[0] == "@" else 8
            values = finite_numbers(command[1:], count)
            if values is None:
                raise ValueError(f"运动指令须包含 {count} 个有限数值")
            if not 1 <= values[-1] <= 100:
                raise ValueError("速度档位必须为 1～100（100 = 30 r/s）")
            if command[0] in (">", "&"):
                for index, (value, bounds) in enumerate(zip(values[:7], JOINT_RANGES + [RAIL_RANGE])):
                    if not bounds[0] <= value <= bounds[1]:
                        raise ValueError(f"J{index + 1} 超出范围 {bounds[0]}～{bounds[1]}")
        elif command.startswith("#SET_PID "):
            parts = command.split()
            if (len(parts) != 6 or parts[1] not in [str(n) for n in range(1, 9)] or
                    any(re.fullmatch(r"[0-9]+", value) is None or int(value) > 0xFFFFFFFF for value in parts[2:])):
                raise ValueError("PID 需要单节点 1～8 和 4 个无符号 32 位整数")
        elif command.startswith(("#ACC_J ", "#I_LIMIT_J ")):
            parts = command.split()
            if (len(parts) not in (3, 4) or (len(parts) == 4 and parts[3] != "&") or
                    parts[1] not in [str(n) for n in range(1, 9)]):
                raise ValueError("参数写入需要单个节点 1～8")
            values = finite_numbers(parts[2], 1)
            if values is None or (parts[0] == "#ACC_J" and not 0 <= values[0] <= 5000) or (parts[0] == "#I_LIMIT_J" and not 0 < values[0] <= 3):
                raise ValueError("电机参数超出范围或不是有限数值")
        elif command.startswith(("!HAND_O ", "!HAND_C ", "!HAND_I ")):
            values = finite_numbers(command.split(maxsplit=1)[1], 1)
            if values is None or not 0.05 <= values[0] <= 2.5:
                raise ValueError("夹爪电流需要为 0.05～2.5 A")
        elif command.startswith("!HAND_POS "):
            parts = command.split()
            if len(parts) != 2 or not parts[1].isdigit() or not 0 <= int(parts[1]) <= 100:
                raise ValueError("夹爪位置需要为 0～100 的整数")
        elif command.startswith(("!HAND_VEL ", "!HAND_ACC ")):
            verb, payload = command.split(maxsplit=1)
            values = finite_numbers(payload, 1)
            maximum = 30 if verb == "!HAND_VEL" else 5000
            if values is None or not 0 < values[0] <= maximum:
                raise ValueError(f"夹爪参数需要为大于 0 且不超过 {maximum} 的有限数值")
        elif command.startswith(("!HAND_KP ", "!HAND_KV ", "!HAND_KI ", "!HAND_KD ")):
            parts = command.split()
            # The dedicated gripper handlers use sscanf %ld (int32_t), while
            # #SET_PID uses %lu (uint32_t). Keep these protocol widths distinct.
            if (len(parts) != 2 or re.fullmatch(r"[0-9]+", parts[1]) is None or
                    int(parts[1]) > 0x7FFFFFFF):
                raise ValueError("夹爪 PID 参数需要为 0～2147483647 的整数")
        elif command == "!HAND_ONBOOT" or command.startswith("!HAND_ONBOOT "):
            parts = command.split()
            # Bare !HAND_ONBOOT is firmware's documented default value 1.
            if len(parts) > 2 or (len(parts) == 2 and parts[1] not in ("0", "1")):
                raise ValueError("夹爪上电行为需要为 0 或 1")
        elif command.startswith(("#STALL_SET ", "#CAL_SET ")):
            parts = command.split()
            if len(parts) != 4 or finite_numbers(parts[3], 1) is None:
                raise ValueError("保护参数需要为单个有限数值")

    def _cancel_motion_for_stop(self):
        self.rgb_save.cancel_unwritten()
        self._invalidate_system_pose()
        stream = getattr(getattr(self, 'cartesian', None), 'diagnostics', None)
        axis_cleanup = bool(stream is not None and stream.active and self.cartesian.kind == 'axis_test' and self.is_connected)
        if axis_cleanup:
            with stream.lock:
                stream.request_axis_stop()
                super()._cancel_motion_for_stop()
                stream.adopt_axis_generation()
        else:
            if hasattr(self, 'cartesian'):
                self.cartesian.cancel()
            super()._cancel_motion_for_stop()
        self._position_request = None
        self._position_token += 1
        self._finish_parameter_request(False, "参数任务已被停止或断连取消")
        self.action_feedback.invalidate(self._tx_generation, self._connection_id)
        if hasattr(self, "gamepad"):
            self.gamepad.controller_cancelled()

    def allocate_command_mode_request(self):
        self._mode_request_counter = (self._mode_request_counter + 1) & 0xFFFFFFFF or 1
        return self._mode_request_counter

    def _set_cmdmode(self, mode):
        if mode == 7 or self.current_cmdmode == 7:
            return self.cartesian.select_mode(mode)
        if mode not in (1, 2, 3, 4):
            self.log('普通运动仅支持模式 1～4；模式 7 仅用于独立诊断', 'WARN')
            return False
        return super()._set_cmdmode(mode)

    def _query_command_mode(self):
        if self._control_owner != "gamepad":
            super()._query_command_mode()

    def _handle_stop_status(self, line):
        if line in ('STOP locked', 'STOP locked disabled', 'Disabled ok'):
            self._stop_confirmed = True
        elif line in ('Stopping ok', 'Start pending', 'Started ok', 'Disable pending'):
            self._stop_confirmed = False
        if (line == "Start pending" and self.gamepad.claimed and self.gamepad.stage == "starting" and
                self.gamepad._system is not None and self.gamepad._system["command"] == "!START"):
            # Claim already invalidated UI motion. This is the acknowledgement
            # of our own START, so do not cancel that transaction's generation.
            self._stop_locked = True
            self.lbl_motor_state.config(text="● 等待停稳并解锁")
            return
        stream = self.cartesian.diagnostics
        if (stream.active and self.cartesian.kind == 'axis_test' and stream.recovering and
                line in ('Stopping ok', 'STOP locked', 'STOP locked disabled', 'Disabled ok', 'error STOP_LOCKED')):
            # The test has retired its curve writer. Keep only bounded read,
            # CLOSE and mode-2 recovery ownership through the STOP receipts.
            self._stop_locked = line != 'Disabled ok'
            if line == 'Disabled ok':
                self._update_motor_state(False)
            else:
                self.lbl_motor_state.config(text='● 通信测试停止 / 等待会话关闭')
            return
        super()._handle_stop_status(line)

    def _sync_acc(self):
        if self._acc_sync_waiting:
            return
        if self.send_cmd("#SYNC_ACC"):
            self._set_acc_sync_waiting(True)
            self._publish()

    @staticmethod
    def _parse_numbers(payload):
        return finite_numbers(payload)

    def _motor_parameter_fields(self):
        return dict(KP=self.ent_set_pid_kp, KV=self.ent_set_pid_kv,
                    KI=self.ent_set_pid_ki, KD=self.ent_set_pid_kd,
                    ACC=self.ent_set_acc, I=self.ent_set_i)

    def _capture_motor_drafts(self, node=None):
        """Keep the non-Qt ValueField client compatible; Qt adapters call edit."""
        node = self._selected_parameter_node if node is None else int(node)
        for key, field in self._motor_parameter_fields().items():
            if not isinstance(field, ValueField):
                continue
            self.parameter_edits.ensure("motor", node, key, self._motor_parameter_defaults[key])
            value = str(field.get())
            address = ("motor", node, key)
            observed = self._legacy_parameter_rendered.get(address, self._motor_parameter_defaults[key])
            if value != observed and value != self.parameter_edits.snapshot("motor", node, key)["draft"]:
                self.parameter_edits.edit("motor", node, key, value)
            self._legacy_parameter_rendered[address] = value

    def _render_motor_drafts(self):
        node = int(self.param_node.get())
        for key, field in self._motor_parameter_fields().items():
            self.parameter_edits.ensure("motor", node, key, self._motor_parameter_defaults[key])
            draft = self.parameter_edits.snapshot("motor", node, key)["draft"]
            field.set(draft)
            if isinstance(field, ValueField):
                self._legacy_parameter_rendered[("motor", node, key)] = draft

    def select_parameter_node(self, node):
        node = int(node)
        if node not in range(9):
            return False
        if node != self._selected_parameter_node:
            self._capture_motor_drafts(self._selected_parameter_node)
            self._selected_parameter_node = node
        if int(self.param_node.get()) != node:
            self.param_node.set(node)
        self._render_motor_drafts()
        self.parameter_edits_changed.emit()
        return True

    def _receive_motor_parameters(self, node, values):
        node = int(node)
        if node not in range(1, 9):
            return
        if int(self.param_node.get()) != self._selected_parameter_node:
            self.select_parameter_node(int(self.param_node.get()))
        self._capture_motor_drafts()
        for key, value in values.items():
            self.parameter_edits.ensure("motor", node, key, self._motor_parameter_defaults.get(key, value))
            self.parameter_edits.receive("motor", node, key, value)
        self._render_motor_drafts()

    def _fill_pid_entries(self, kp, kv, ki, kd, node=None):
        """Decode firmware's signed %ld presentation of uint32 PID caches.

        Older #GET_PID handlers print int32_t caches, whereas newer handlers
        may print uint32 directly. Accept both wire representations, but never
        wrap an out-of-range response into a plausible motor parameter.
        """
        try:
            values = [int(value) for value in (kp, kv, ki, kd)]
        except (TypeError, ValueError):
            self.log("PID 回包不是有效整数，保留当前编辑值", "WARN")
            return
        if any(not -0x80000000 <= value <= 0xFFFFFFFF for value in values):
            self.log("PID 回包超出 int32/uint32 范围，保留当前编辑值", "WARN")
            return
        normalized = [str(value & 0xFFFFFFFF) for value in values]
        self._receive_motor_parameters(int(self.param_node.get()) if node is None else node,
                                       dict(zip(("KP", "KV", "KI", "KD"), normalized)))

    def _fill_acc_entry(self, value, node=None):
        if finite_numbers(str(value), 1) is not None:
            self._receive_motor_parameters(int(self.param_node.get()) if node is None else node, {"ACC": str(value)})

    def _fill_i_entry(self, value, node=None):
        if finite_numbers(str(value), 1) is not None:
            self._receive_motor_parameters(int(self.param_node.get()) if node is None else node, {"I": str(value)})

    def _current_joint_values(self):
        values = super()._current_joint_values()
        if values is not None and any(not math.isfinite(value) for value in values):
            self.log("关节/地轨输入必须为有限数值", "ERROR")
            return None
        return values

    @staticmethod
    def _rgb_values(variables, ranges):
        values = []
        for variable, (minimum, maximum) in zip(variables, ranges):
            text = str(variable.get()).strip()
            if re.fullmatch(r"[0-9]+", text) is None or not minimum <= int(text) <= maximum:
                raise ValueError(f"RGB 参数须为 {minimum}～{maximum} 的整数")
            values.append(int(text))
        return values

    def _process_rx_line(self, line, received_at=None, *, telemetry=False):
        observed_generation = self._tx_generation
        observed_at = self.cartesian.clock()
        receipt_at = (received_at if received_at is not None and math.isfinite(received_at) and
                      received_at <= observed_at else observed_at)
        self.operations.receive(line, observed_generation, self._connection_id,
                                feedback_sequence=self._actual_feedback_sequence,
                                feedback_after=receipt_at)
        self.action_feedback.receive(line, observed_generation, self._connection_id)
        self.rgb_save.activity(line)
        self._handle_parameter_task_reply(line)
        # A seven-value pose is unambiguously joint+rail in firmware_new.
        nums = finite_numbers(line[2:].strip()) if line.startswith("ok ") else None
        if nums is None and line.startswith("ok "):
            tokens = line[2:].split()
            if len(tokens) == 7:
                try:
                    nums = [float(value) for value in tokens]
                except ValueError:
                    pass
        if nums is not None and len(nums) == 7:
            now = self.cartesian.clock()
            stamp = now if received_at is None else received_at
            age = now-stamp
            self.feedback_delivery_ms = max(0., age*1000)
            reason = None
            if not all(math.isfinite(value) for value in nums):
                reason = "七轴回读包含非有限数值"
            elif not math.isfinite(stamp):
                reason = "回读时间戳无效"
            elif age < 0:
                reason = "回读时间戳晚于当前单调时钟"
            elif age > .3:
                reason = "回读到界面时已超过300ms"
            elif self._actual_received_at is not None and stamp < self._actual_received_at:
                reason = "回读时间戳倒序"
            if reason is not None:
                self.cartesian.feedback.rejection_reason = reason
                if now-getattr(self, "_feedback_warning_at", -float("inf")) >= 1.:
                    self._feedback_warning_at = now
                    self.log(f"界面关节显示未更新：{reason}；RX到界面 {age*1000:.0f} ms", "WARN")
                # Invalid or old data cannot refresh either display or edits.
                self.log(line, "RX")
                request = self._position_request
                if not telemetry and request and request["kind"] == "j":
                    self._position_request = None
                    self._pending_sync = None
                self._emit_state()
                return
            # Actual telemetry is a measured state, not a proposed motion.
            # Preserve raw degrees/mm in the display even when planning
            # rejects it; never clip it to make a valid planning seed.
            accepted = self.cartesian.update_feedback(nums, stamp)
            self.actual_pose = list(nums)
            self._actual_received_at = stamp
            self._actual_feedback_sequence += 1
            self.last_actual_at = time.time()-age
            if self._transport is not None and hasattr(self._transport, "metrics"):
                self._transport.metrics.delivered(stamp)
            self.actual_pose_changed.emit(list(nums))
            request = self._position_request
            if not telemetry and request and request["kind"] == "j":
                self._position_request = None
                self._pending_sync = None
                if request["sync"] and request["generation"] == self._tx_generation:
                    if accepted:
                        self._fill_joints(nums)
                        self.position_synced.emit("MoveJ", list(nums))
                        self.log("实际位置已同步到编辑目标")
                        self.fields_changed.emit()
                    else:
                        self.log("实际位置已显示原始回读；不符合运动模型限位，未同步到编辑目标", "WARN")
            self.log(line, "RX")
            self._emit_state()
            return
        elif nums is not None and len(nums) == 6:
            self.actual_cartesian_pose = list(nums)
            self.last_actual_cartesian_at = time.monotonic()
            self.actual_cartesian_changed.emit(list(nums))
            request = self._position_request
            if request and request["kind"] == "l":
                self._position_request = None
                self._pending_sync = None
                if request["sync"] and request["generation"] == self._tx_generation:
                    self._fill_pose(nums)
                    self.position_synced.emit("MoveL", list(nums))
                    self.log("实际笛卡尔位姿已同步到编辑目标")
        before_id = self._seq_motion_id
        super()._process_rx_line(line)
        if line == "Start pending":
            self.operations.carry_start_generation(observed_generation, self._tx_generation)
        self.cartesian.handle_line(line)
        self.gamepad.handle_line(line)
        self._handle_system_pose_receipt(line)
        if self._seq_motion_id is not None and self._seq_motion_id != before_id:
            self.root.after(self.sequence_timeout_ms, self._motion_completion_timeout,
                            self._seq_session, self._seq_motion_id)
        if line.startswith("error") and self._position_request:
            if line in ("error DISABLED", "error STOP_LOCKED"):
                self._position_request = None
                self._pending_sync = None
        self._publish()

    @staticmethod
    def _parameter_spec(command):
        match = re.fullmatch(r"#(STALL|CAL)_(GET|SET|SAVE)\s+([0-8])(?:\s+([A-Z_0-9]+)\s+([^\s]+))?", command)
        if not match:
            return None
        kind, operation, node, key, _value = match.groups()
        node = int(node)
        if ((operation == "SET") != (key is not None) or
                (kind == "CAL" and node not in (7, 8)) or
                (kind == "STALL" and node == 8)):
            # STALL J8's EXCLUDED reply never creates a firmware parameter job.
            return None
        nodes = {node} if node else ({7} if key == "BACK_MM" else
                                    set(range(1, 7)) if key == "BACK_DEG" else set(range(1, 8)))
        expected_fields = {}
        if operation == "GET":
            if kind == "CAL":
                fields = {row[0] for row in CAL_PARAMETERS}
                expected_fields = {target: set(fields) for target in nodes}
            else:
                fields = {row[0] for row in STALL_PARAMETERS} - {"BACK_DEG", "BACK_MM"}
                fields |= {"MIN_POSITION_R", "MAX_POSITION_R", "DIRECTION_MASK"}
                expected_fields = {target: fields | {"BACK_MM" if target == 7 else "BACK_DEG"}
                                   for target in nodes}
        return {"command": command, "kind": kind, "operation": operation, "node": node,
                "key": key, "nodes": nodes, "expected_fields": expected_fields,
                "fields": {target: set() for target in nodes}, "completed_nodes": set(),
                "accepted": False, "timeout_token": None}

    def _begin_parameter_request(self, spec):
        self._parameter_request = spec
        self._arm_parameter_timeout()

    def _arm_parameter_timeout(self):
        request = self._parameter_request
        if request is None:
            return
        if self._parameter_timeout_id is not None:
            self.root.after_cancel(self._parameter_timeout_id)
        self._parameter_token += 1
        request["timeout_token"] = self._parameter_token
        # GET has at most a 1500 ms MCU wait per field; SAVE has 2500 ms per
        # node. SET comprises BEGIN / SET / COMMIT. Extend only on new matched
        # progress, so ALL queries can finish without a fixed short total cap.
        timeout = 4000 if request["operation"] == "GET" else 6000
        self._parameter_timeout_id = self.root.after(timeout, self._parameter_timeout,
                                                     request["timeout_token"])

    def _parameter_timeout(self, token):
        request = self._parameter_request
        if request is not None and request["timeout_token"] == token:
            self._parameter_timeout_id = None
            self._finish_parameter_request(False, "完整回执超时；本次读写未确认，请核查设备日志")
            self._publish()

    def _finish_parameter_request(self, success, detail):
        request, self._parameter_request = self._parameter_request, None
        if request is None:
            return
        self._parameter_token += 1
        if self._parameter_timeout_id is not None:
            self.root.after_cancel(self._parameter_timeout_id)
            self._parameter_timeout_id = None
        if success:
            self.log(f"{request['kind']}_{request['operation']} 完整回执已确认")
        else:
            self.log(detail, "WARN")
        self.action_feedback.finish(request.get("feedback_id"), success, detail,
                                    uncertain=("超时" in detail or "取消" in detail))

    def _handle_parameter_task_reply(self, line):
        request = self._parameter_request
        if request is None:
            return
        prefix = request["kind"] + "_" + request["operation"]
        if line == f"ok {prefix} pending":
            request["accepted"] = True
            self._arm_parameter_timeout()
            return
        if line == f"error {prefix}" or line.startswith(f"error {prefix} "):
            failed_node = re.match(rf"error {prefix} J([1-8]) ", line)
            if failed_node and int(failed_node[1]) not in request["nodes"]:
                return
            write_fault = (request["accepted"] and request["operation"] != "GET" and
                           failed_node is not None)
            self._finish_parameter_request(False, f"参数任务失败：{line}")
            if write_fault:
                # Firmware locks after an accepted write/save fails; reading
                # parameters alone does not create this persistent lock.
                self._cancel_motion_for_stop()
                self._stop_locked = True
                self.lbl_motor_state.config(text="参数应用/保存失败，保护锁定")
            return
        if not request["accepted"]:
            return  # A prior query's late records cannot accept a new task.
        operation = request["operation"]
        if operation == "SAVE":
            match = re.fullmatch(r"ok (STALL|CAL)_SAVE \[([1-8])\] saved", line)
            if not match or match[1] != request["kind"]:
                return
            node, key = int(match[2]), None
        else:
            match = re.fullmatch(r"ok (STALL|CAL)(_SET)? \[([1-8])\] ([A-Z_0-9]+)=([^\s]+)(?:\s+.*)?", line)
            if (not match or match[1] != request["kind"] or
                    bool(match[2]) != (operation == "SET") or finite_numbers(match[5], 1) is None):
                return
            node, key = int(match[3]), match[4]
        if node not in request["nodes"]:
            return
        progressed = False
        if operation == "GET":
            if key not in request["expected_fields"][node] or key in request["fields"][node]:
                return
            request["fields"][node].add(key)
            progressed = True
            if request["fields"][node] >= request["expected_fields"][node]:
                request["completed_nodes"].add(node)
        elif operation == "SET":
            expected_key = request["key"]
            if expected_key == "BACK_DISTANCE_R":
                expected_key = "BACK_MM" if node == 7 else "BACK_DEG"
            if key != expected_key or node in request["completed_nodes"]:
                return
            request["completed_nodes"].add(node)
            progressed = True
        elif node not in request["completed_nodes"]:
            request["completed_nodes"].add(node)
            progressed = True
        if request["completed_nodes"] == request["nodes"]:
            self._finish_parameter_request(True, "")
        elif progressed:
            self._arm_parameter_timeout()

    def _motion_completion_timeout(self, session, motion_id):
        if self._seq_active and self._seq_session == session and self._seq_motion_id == motion_id:
            self._stop_sequence(f"运动编号 {motion_id} 超过等待时间")
            self.send_cmd("!STOP")

    def _handle_protection_parameter_status(self, line):
        match = re.fullmatch(r"ok (STALL|CAL)(?:_SET)? \[(\d+)\] ([A-Z_0-9]+)=([^\s]+)(?:\s+.*)?", line)
        if not match:
            return
        kind, node, key, value = match[1], int(match[2]), match[3], match[4]
        if (finite_numbers(value, 1) is None or node not in range(1, 9) or
                (kind == "STALL" and node == 8) or (kind == "CAL" and node not in (7, 8))):
            return
        if kind == "STALL" and key == "BACK_DISTANCE_R" and 1 <= node <= 7:
            motor_r = float(value)
            key = "BACK_MM" if node == 7 else "BACK_DEG"
            value = f"{motor_r * 5 if node == 7 else motor_r * 360 / (30 if node == 6 else 50):.6g}"
        self._protection_values[kind].setdefault(node, {})[key] = value
        selected = int(self._protection_nodes[kind].get())
        if selected == node:
            self._capture_protection_drafts(kind, node)
        descriptor = next((row for row in (STALL_PARAMETERS if kind == "STALL" else CAL_PARAMETERS)
                           if row[0] == key), None)
        default = self._protection_default(kind, node, key, descriptor[3]) if descriptor else value
        self.parameter_edits.ensure(kind, node, key, f"{default:g}" if isinstance(default, (int, float)) else default)
        self.parameter_edits.receive(kind, node, key, value)
        if self._protection_nodes[kind].get() == node and key in self._protection_entries[kind]:
            entry = self._protection_entries[kind][key]
            entry.delete(0, "end")
            draft = self.parameter_edits.snapshot(kind, node, key)["draft"]
            entry.insert(0, draft)
            self._legacy_parameter_rendered[(kind, node, key)] = draft

    def _capture_protection_drafts(self, kind, node):
        descriptors = STALL_PARAMETERS if kind == "STALL" else CAL_PARAMETERS
        for key, _label, _unit, default, _minimum, _maximum in descriptors:
            entry = self._protection_entries[kind][key]
            if not isinstance(entry, ValueField):
                continue
            default = self._protection_default(kind, node, key, default)
            self.parameter_edits.ensure(kind, node, key, f"{default:g}")
            value = str(entry.get())
            address = (kind, int(node), key)
            observed = self._legacy_parameter_rendered.get(address, f"{default:g}")
            if value != observed and value != self.parameter_edits.snapshot(kind, node, key)["draft"]:
                self.parameter_edits.edit(kind, node, key, value)
            self._legacy_parameter_rendered[address] = value

    def set_polling(self, enabled, hz=5):
        hz = float(hz)
        if not math.isfinite(hz) or not 1 <= hz <= 50:
            raise ValueError("位置回读频率应为 1～50 Hz")
        self._poll_enabled, self._poll_hz = bool(enabled), hz
        if self._transport is not None and hasattr(self._transport, "configure_joint_polling"):
            self._transport.configure_joint_polling(self._poll_enabled, hz, self._tx_generation)
        if self.is_connected and self._poll_enabled:
            self._poll_timer.start(math.ceil(1000 / hz))
        else:
            self._poll_timer.stop()
        self._emit_state()

    def _position_polling_paused(self):
        return ((self.cartesian.active and self.cartesian.kind in ("mode_switch", "axis_test")) or not self._poll_enabled or not self.is_connected or self._stop_locked or
                self._calibration_busy or self._acc_sync_waiting or self._parameter_request is not None or self.rgb_save.active or
                (self.connection_type == "UART4" and self._motor_state_known and not self._motor_enabled))

    def _poll_position(self):
        if self._transport is not None and hasattr(self._transport, "configure_joint_polling"):
            # Configuration heartbeat only. The writer schedules and retires
            # real background queries without waiting for this GUI callback.
            self._transport.configure_joint_polling(self._poll_enabled, self._poll_hz, self._tx_generation)
            return
        if self._position_polling_paused() or self._position_request is not None:
            return
        self._request_position("j", False, source="telemetry")

    def _discard_gamepad_position_request(self):
        request = self._position_request
        if request is not None and request.get("source") == "gamepad":
            self._position_request = self._pending_sync = None

    def _request_position(self, kind, sync, source="ui"):
        # Check before merging with an existing request: a UI sync must not
        # turn the active controller's shared poll into an edit-target read.
        if self.cartesian.busy and not (source == "telemetry" and kind == "j" and not sync):
            self.log("上位机轨迹或后端切换中仅使用共享关节轮询，请勿同步编辑目标", "WARN")
            return False
        if source == "gamepad" or (self.gamepad.claimed and
                not (source == "telemetry" and kind == "j" and not sync)):
            self.log("手柄控制中仅使用共享关节轮询更新实际模型，请勿手动同步编辑目标", "WARN")
            return False
        if self._position_request:
            if self._position_request["kind"] == kind and source != "gamepad":
                self._position_request["sync"] |= sync
                return True
            self.log("已有位置回读请求，请等待后再读取另一种位置", "WARN")
            return False
        command = "#GETJPOS" if kind == "j" else "#GETLPOS"
        self._position_token += 1
        token = self._position_token
        # Install before enqueue: the TX worker may check admission immediately.
        request = {"kind": kind, "sync": sync, "token": token, "generation": self._tx_generation,
                   "source": source, "written_at": None}
        self._position_request = request
        self._pending_sync = kind
        if (kind == "j" and source == "ui" and not self._position_polling_paused() and
                self._transport is not None and hasattr(self._transport, "adopt_joint_poll")):
            stamp = self._transport.adopt_joint_poll()
            if stamp is not None:
                request["written_at"] = stamp
                self.root.after(1200, self._position_timeout, token)
                return True
        if not self.send_cmd(command, source=source):
            if self._position_request is request:
                self._position_request = self._pending_sync = None
            return False
        self.root.after(1200, self._position_timeout, token)
        return True

    def _position_unwritten_timeout(self, token):
        request = self._position_request
        if request and request["token"] == token and request.get("written_at") is None:
            self._position_timeout(token)

    def _position_timeout(self, token):
        if self._position_request and self._position_request["token"] == token:
            request = self._position_request
            sync = request["sync"]
            self._position_request = self._pending_sync = None
            if request.get("source") == "gamepad":
                return  # Retired query watchdog cannot stop the software-target session.
            now = time.monotonic()
            if sync or now - self._last_position_warning > 5:
                self.log("位置回读超时，三维实际姿态数据已过期", "WARN")
                self._last_position_warning = now
            self._emit_state()

    def read_and_sync_from_robot(self):
        return self._request_position("j", True)

    def read_pose_from_robot(self):
        return self._request_position("l", True)

    def _start_sync(self, kind, _cmd):
        return self._request_position(kind, True)

    def _check_sync_response(self):
        pass  # Replies fill immediately; timeout is per request, never a delayed target overwrite.

    def _open_protection_parameters(self):
        if self._protection_window is not None:
            self._protection_window.lift()

    def _rgb_pick_color(self, variables):
        from PySide6.QtGui import QColor
        from PySide6.QtWidgets import QApplication
        from app.ui.color_dialog import choose_color
        try:
            values = self._rgb_values(variables, [(0, 255)] * 3)
        except ValueError as error:
            self.log(str(error), "WARN")
            return
        color = choose_color(QColor(*values), QApplication.activeWindow(), "选择 RGB 颜色")
        if color.isValid():
            self._rgb_set_fields(variables, [color.red(), color.green(), color.blue()])
            self.fields_changed.emit()
