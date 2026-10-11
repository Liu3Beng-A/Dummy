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
from app.core.gamepad_session import GamepadSession
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
    preset_pose_completed = Signal(str, list)  # Matched HOME / RESET completion; target only.
    rgb_changed = Signal(dict)
    fields_changed = Signal()
    ports_changed = Signal(list)

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
        self._wire_admitted = queue.Queue(maxsize=256)
        self.is_connected = False
        self.port_name = ""
        self.connection_type = "USB"
        self.current_cmdmode = None
        self._MODE_NAME = {1: "顺序(SEQ)", 2: "打断(INT)", 3: "连续(TRJ)", 4: "调试(TUN)"}
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
        self._reset_rgb_state(emit=False)
        self.cmd_history = []
        self.cmd_history_idx = [0]
        self.last_hand_pos_send_time = self.last_movej_send_time = self.last_movel_send_time = 0
        self._default_fields()
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
        self._rgb_monitor_timer.setInterval(1000)
        self._rgb_monitor_timer.timeout.connect(self._poll_rgb)
        self.gamepad = GamepadSession(self)

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
            "_rgb_scale": 100, "_rgb_reverse": False,
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
            "_rgb_state_color": ["0", "0", "255"],
        }.items():
            setattr(self, name, [ValueField(value) for value in values])
        for name in ("_rgb_color_idx", "_rgb_auto_idx", "_rgb_effect_mode", "_rgb_state",
                     "_rgb_state_mode", "_rgb_state_source"):
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
        self._publish()

    def log(self, message, tag="INFO"):
        self.log_message.emit(str(message), tag)

    def state(self):
        now = time.time()
        status = ""
        try:
            status = str(self.lbl_motor_state.get())
        except (AttributeError, TypeError):
            pass
        return {
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
            "polling_enabled": self._poll_enabled, "polling_hz": self._poll_hz,
            "polling_paused": self._position_polling_paused(),
            "actual_pose": self.actual_pose,
            "actual_cartesian_pose": deepcopy(self.actual_cartesian_pose),
            "parameter_busy": self._parameter_request is not None,
            "parameter_task": (self._parameter_request["command"] if self._parameter_request else None),
        }

    def _emit_state(self):
        self.state_changed.emit(self.state())

    def _publish(self):
        self.fields_changed.emit()
        self._emit_state()

    def _reset_rgb_state(self, emit=True):
        """Discard readback and query clocks whenever the device identity changes."""
        self._rgb_cache = {"colors": {}, "auto_colors": {}, "effects": {}, "states": {}}
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
        self._rgb_get_pending_until = None
        self._rgb_get_retry_at = 0.0
        self._rgb_next_status_at = 0.0
        if emit:
            self._emit_rgb()

    def rgb_snapshot(self):
        """Return device readback only; changing editable fields cannot change it."""
        return deepcopy({"config": self._rgb_device_config, "runtime": self._rgb_runtime,
                         "complete": self._rgb_complete, "connected": self.is_connected,
                         "received_at": self._rgb_received_at, "connection_id": self._connection_id})

    def _emit_rgb(self):
        self.rgb_changed.emit(self.rgb_snapshot())

    def _start_rgb_config_response(self, seed_runtime=False):
        self._rgb_cache = {"colors": {}, "auto_colors": {}, "effects": {}, "states": {}}
        self._rgb_device_config = {key: value for key, value in self._rgb_cache.items()}
        self._rgb_full_active = True
        self._rgb_config_started = self._rgb_full_invalid = self._rgb_complete = False
        self._rgb_full_fields = set()
        if seed_runtime and self._rgb_runtime_ready_for_config:
            values = self._rgb_runtime
            self._rgb_device_config.update(enabled=bool(values["on"]), automatic=bool(values["auto"]),
                                           brightness=values["bright"])
            self._rgb_full_fields.update(("enabled", "automatic", "brightness"))

    def _begin_rgb_get(self):
        now = time.monotonic()
        self._rgb_get_pending_until = now + 2.0
        self._rgb_get_retry_at = now + 5.0
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
        if not self._rgb_full_active or (start and self._rgb_config_started):
            self._start_rgb_config_response(seed_runtime=True)
        if start:
            self._rgb_config_started = True
            if self._rgb_get_pending_until is None:
                # Structured setters return the same full block as GET. Do not
                # interleave a monitor query while that block is being received.
                now = time.monotonic()
                self._rgb_get_pending_until = now + 2.0
                self._rgb_get_retry_at = max(self._rgb_get_retry_at, now + 5.0)
        self._rgb_device_config.update(fields)
        self._rgb_full_fields.update(fields)
        self._emit_rgb()

    def _accept_rgb_config_table(self, kind, key, values):
        if not self._rgb_full_active:
            self._start_rgb_config_response(seed_runtime=True)
        self._rgb_cache[kind][key] = tuple(values)
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
        self._rgb_complete = bool(
            self._rgb_full_active and self._rgb_config_started and not self._rgb_full_invalid and
            required <= self._rgb_full_fields and self._rgb_cache["colors"].keys() == {0, 1, 2} and
            self._rgb_cache["auto_colors"].keys() == {1, 2, 3} and
            self._rgb_cache["effects"].keys() == set(range(3, 10)) and
            self._rgb_cache["states"].keys() == states)
        self._rgb_full_active = False
        self._rgb_get_pending_until = None
        self._rgb_runtime_ready_for_config = False
        self._emit_rgb()

    def set_rgb_monitoring(self, enabled):
        """The visible RGB page explicitly opts into read-only 1 Hz monitoring."""
        self._rgb_monitoring = bool(enabled)
        if self._rgb_monitoring and self.is_connected:
            if not self._rgb_monitor_timer.isActive():
                self._rgb_monitor_timer.start()
                self._poll_rgb()
        else:
            self._rgb_monitor_timer.stop()

    def _poll_rgb(self):
        if (not self._rgb_monitoring or not self.is_connected or self.serial_port is None or
                self._parameter_request is not None or self._acc_sync_waiting):
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
        if not self._rgb_complete and now >= self._rgb_get_retry_at:
            self.send_cmd("#RGB_GET")
            return
        if now >= self._rgb_next_status_at:
            self.send_cmd("#RGB_STATUS")

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.ports_changed.emit(ports)
        self.cb_ports["values"] = ports
        if ports and not self.cb_ports.get():
            self.cb_ports.current(0)
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
        self._reset_system_pose_tracking()
        self.connection_type = connection_type.upper()
        self.serial_port = port_object
        self.port_name = port
        self._tx_queue = queue.Queue()
        self._wire_admitted = queue.Queue(maxsize=256)
        self.is_connected = True
        self.current_cmdmode = None
        self._motor_state_known = self._motor_enabled = False
        self._stop_locked = self._calibration_busy = False
        self._mode_request = self._mode_pending = None
        self._update_motor_state(None)
        self.actual_pose = None
        self.last_actual_at = None
        self.actual_cartesian_pose = self.last_actual_cartesian_at = None
        # The default GET below publishes only after its in-flight ledger is
        # established; UI monitoring listeners cannot enqueue a second GET.
        self._reset_rgb_state(emit=False)
        self._transport = SerialTransport(port_object, self._tx_queue, self._can_write,
                                          self._connection_id, self)
        self._transport.line_received.connect(self._on_transport_line, Qt.ConnectionType.QueuedConnection)
        self._transport.command_written.connect(self._on_written, Qt.ConnectionType.QueuedConnection)
        self._transport.failed.connect(self._on_transport_failure, Qt.ConnectionType.QueuedConnection)
        self._transport.start()
        self.log(f"已连接 {port} @ {BAUDRATE} ({self.connection_type})")
        self.send_cmd("#RGB_GET")
        self._query_command_mode()
        self.set_polling(self._poll_enabled, self._poll_hz)
        if self._rgb_monitoring:
            self.set_rgb_monitoring(True)
        self._publish()
        return True

    def disconnect_port(self):
        self._connection_id += 1  # Ignore callbacks from a previous connection.
        self.is_connected = False
        self.gamepad.disconnected()
        self._poll_timer.stop()
        self._rgb_monitor_timer.stop()
        self._cancel_motion_for_stop()
        self._reset_system_pose_tracking()
        transport, self._transport = self._transport, None
        if transport:
            transport.close()
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
    def _on_transport_line(self, line, connection_id):
        if connection_id == self._connection_id and self.is_connected:
            self._process_rx_line(line)

    @Slot(str, int)
    def _on_written(self, command, connection_id):
        if connection_id == self._connection_id:
            self.log(command, "TX")
            try:
                generation, admitted_command, admitted_connection = self._wire_admitted.get_nowait()
            except queue.Empty:
                return  # A notification without actual writer admission is not proof of TX.
            if (admitted_command != command or admitted_connection != connection_id or
                    generation != self._tx_generation):
                return
            request = self._position_request
            if request and command == ("#GETJPOS" if request["kind"] == "j" else "#GETLPOS"):
                if request.get("source") == "gamepad":
                    self._discard_gamepad_position_request()
                else:
                    request["written_at"] = time.monotonic()
            self.gamepad.command_written(command, generation, connection_id)

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
        diagnostic = capabilities.diagnostic(command)
        allowed = (self.is_connected and generation == self._tx_generation and
                (self._control_owner != "gamepad" or not command.startswith("@") or
                 self.gamepad.can_write_stream(generation, command)) and
                (self._control_owner != "gamepad" or not self._is_position_command(command) or
                 self._is_joint_telemetry_request(generation, command)) and
                (not self._stop_locked or diagnostic or command in ("!STOP", "!START", "!DISABLE")) and
                (not self._calibration_busy or diagnostic or command in ("!CALIBRATION", "!STOP", "!DISABLE")))
        if allowed:
            try:
                self._wire_admitted.put_nowait((generation, command, self._connection_id))
            except queue.Full:
                return False
        return allowed

    def send_cmd(self, command, sequence=False, source="ui"):
        """Only command dispatch path; STOP invalidates every pending generation."""
        if not self.is_connected or self.serial_port is None:
            self.log("未连接串口", "WARN")
            return False
        command = str(command).strip()
        if not command or any(char in command for char in ("\r", "\n", "\x00")):
            self.log("一次只能发送一条非空 ASCII 指令", "WARN")
            return False
        if not command.isascii() or len(command.encode()) > 256:
            self.log("指令须为 ASCII 且长度不能超过 256 字节", "WARN")
            return False
        if source not in ("ui", "gamepad", "telemetry"):
            self.log("未知指令来源", "WARN")
            return False
        if self._is_position_command(command) and (source == "gamepad" or
                (self._control_owner == "gamepad" and not
                 (source == "telemetry" and self._is_joint_telemetry_request(self._tx_generation, command)))):
            self.log("手柄控制中仅使用共享关节轮询更新实际模型，请勿手动同步编辑目标", "WARN")
            return False
        if source == "gamepad" and not self.gamepad.claimed and command not in ("!STOP", "!DISABLE"):
            self.log("手柄尚未接管，指令未发送", "WARN")
            return False
        if self._control_owner == "gamepad" and source == "ui" and command not in ("!STOP", "!DISABLE"):
            verb = command.split()[0]
            readonly = capabilities.diagnostic(command) or verb in {"#GET_PID", "#GETJACC", "#GETI", "#STACK"}
            if not readonly:
                self.log("手柄控制中，请先释放控制；STOP / DISABLE 始终可用", "WARN")
                return False
        if not self.supports(command):
            self.log(f"当前 UART4 固件不支持 {command.split()[0]}；请使用 USB", "WARN")
            return False
        if (self._parameter_request is not None and command not in ("!STOP", "!DISABLE") and
                not capabilities.diagnostic(command)):
            self.log("堵转/校准参数读写中，请等待完整回执；可发送 STOP 或 DISABLE", "WARN")
            return False
        if (self.connection_type == "UART4" and self._motor_state_known and not self._motor_enabled and
                command in ("#GETJPOS", "#GETLPOS", "#GETRAILPOS", "#STACK")):
            self.log("当前 UART4 固件失能时不处理位置/栈查询；实际显示暂停，请使用 USB 查询", "WARN")
            return False
        try:
            self._validate_command(command)
        except ValueError as error:
            self.log(str(error), "WARN")
            return False
        if self._seq_active and not sequence and (command.startswith((">", "&", "@", "#CMDMODE")) or
                                                  command in ("!HOME", "!RESET")):
            self.log("顺序发送中，请等待完成或先 STOP", "WARN")
            return False
        diagnostic = capabilities.diagnostic(command)
        if self._calibration_busy and not diagnostic and command not in ("!STOP", "!DISABLE"):
            self.log("校准及保存中，只接受 STOP、DISABLE 和保护快照查询", "WARN")
            return False
        if self._stop_locked and not diagnostic and command not in ("!STOP", "!START", "!DISABLE"):
            self.log("保护锁定中，请 START 或 DISABLE；STOP 可中断保护动作", "WARN")
            return False
        if command.startswith("#ENC_CAL"):
            if re.fullmatch(r"#ENC_CAL[ \t]+[1-8]", command) is None:
                self.log("磁编码器校准只支持单个节点 1～8", "WARN")
                return False
            if self._seq_active or self._acc_sync_waiting or not self._motor_state_known or self._motor_enabled:
                self.log("请结束任务并失能，等待 Disabled ok 后校准磁编码器", "WARN")
                return False
        if command == "!RGB_SAVE" or command.startswith(("#STALL_SAVE", "#CAL_SAVE")):
            if (not self._motor_state_known or self._motor_enabled or self._seq_active or
                    self._acc_sync_waiting):
                self.log("写入 EEPROM 需要结束任务并等待 Disabled ok", "WARN")
                return False
        if command == "!CALIBRATION":
            if self._motor_state_known and self._motor_enabled:
                self._warn_calibration_disabled()
                return False
            self._cancel_motion_for_stop()
            self._calibration_busy = True
            self.lbl_motor_state.config(text="校准入口检查中")
        if command in ("!STOP", "!DISABLE"):
            if source != "gamepad":
                self.gamepad.external_stop(command)
            self._cancel_motion_for_stop()
            self._stop_locked = True
            self._handle_stop_status("Stopping ok" if command == "!STOP" else "Disable pending")
        parameter_spec = self._parameter_spec(command)
        if parameter_spec is not None:
            self._begin_parameter_request(parameter_spec)
        try:
            self._tx_queue.put((self._tx_generation, command))
        except Exception as error:
            if parameter_spec is not None:
                self._finish_parameter_request(False, "指令入队失败")
            self.log(f"指令入队失败：{error}", "ERROR")
            return False
        self._track_system_pose_command(command)
        if command == "#RGB_GET":
            self._begin_rgb_get()
        elif command == "#RGB_STATUS":
            self._rgb_next_status_at = time.monotonic() + 1.0
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
            self.log(f"{kind} 回执无法对应（{reason}）；本连接暂不自动同步该预设，重新连接可恢复", "WARN")
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
        self._invalidate_system_pose()
        super()._cancel_motion_for_stop()
        self._position_request = None
        self._position_token += 1
        self._finish_parameter_request(False, "参数任务已被停止或断连取消")
        if hasattr(self, "gamepad"):
            self.gamepad.controller_cancelled()

    def allocate_command_mode_request(self):
        self._mode_request_counter = (self._mode_request_counter + 1) & 0xFFFFFFFF or 1
        return self._mode_request_counter

    def _query_command_mode(self):
        if self._control_owner != "gamepad":
            super()._query_command_mode()

    def _handle_stop_status(self, line):
        if (line == "Start pending" and self.gamepad.claimed and self.gamepad.stage == "starting" and
                self.gamepad._system is not None and self.gamepad._system["command"] == "!START"):
            # Claim already invalidated UI motion. This is the acknowledgement
            # of our own START, so do not cancel that transaction's generation.
            self._stop_locked = True
            self.lbl_motor_state.config(text="● 等待停稳并解锁")
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

    def _fill_pid_entries(self, kp, kv, ki, kd):
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
        super()._fill_pid_entries(*normalized)

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

    def _process_rx_line(self, line):
        self._handle_parameter_task_reply(line)
        # A seven-value pose is unambiguously joint+rail in firmware_new.
        nums = finite_numbers(line[2:].strip()) if line.startswith("ok ") else None
        if nums is not None and len(nums) == 7:
            self.actual_pose = list(nums)
            self.last_actual_at = time.time()
            self.actual_pose_changed.emit(list(nums))
            request = self._position_request
            if request and request["kind"] == "j":
                self._position_request = None
                self._pending_sync = None
                if request["sync"] and request["generation"] == self._tx_generation:
                    self._fill_joints(nums)
                    self.position_synced.emit("MoveJ", list(nums))
                    self.log("实际位置已同步到编辑目标")
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
        if finite_numbers(value, 1) is None:
            return
        if kind == "STALL" and key == "BACK_DISTANCE_R" and 1 <= node <= 7:
            motor_r = float(value)
            key = "BACK_MM" if node == 7 else "BACK_DEG"
            value = f"{motor_r * 5 if node == 7 else motor_r * 360 / (30 if node == 6 else 50):.6g}"
        self._protection_values[kind].setdefault(node, {})[key] = value
        if self._protection_nodes[kind].get() == node and key in self._protection_entries[kind]:
            entry = self._protection_entries[kind][key]
            entry.delete(0, "end")
            entry.insert(0, value)

    def set_polling(self, enabled, hz=5):
        hz = float(hz)
        if not math.isfinite(hz) or not 1 <= hz <= 30:
            raise ValueError("位置回读频率应为 1～30 Hz")
        self._poll_enabled, self._poll_hz = bool(enabled), hz
        if self.is_connected and self._poll_enabled:
            self._poll_timer.start(math.ceil(1000 / hz))
        else:
            self._poll_timer.stop()
        self._emit_state()

    def _position_polling_paused(self):
        return (not self._poll_enabled or not self.is_connected or self._stop_locked or
                self._calibration_busy or self._acc_sync_waiting or self._parameter_request is not None or
                (self.connection_type == "UART4" and self._motor_state_known and not self._motor_enabled))

    def _poll_position(self):
        if self._position_polling_paused() or self._position_request is not None:
            return
        self._request_position("j", False, source="telemetry")

    def _discard_gamepad_position_request(self):
        request = self._position_request
        if request is not None and request.get("source") == "gamepad":
            self._position_request = self._pending_sync = None

    def _request_position(self, kind, sync, source="ui"):
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
