"""Qt gamepad controls and local configuration, without hardware access."""
from copy import deepcopy
import sys

from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtWidgets import (
    QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox, QFormLayout, QFrame, QGridLayout,
    QHBoxLayout, QLabel, QLineEdit, QMessageBox, QPlainTextEdit, QScrollArea,
    QSizePolicy, QSpinBox, QVBoxLayout, QWidget,
)

from app.gamepad.config import default_config, validate_config
from app.ui.widgets import button, hint, number


class _UncappedSpeedSpinBox(QDoubleSpinBox):
    """Use the native float range without sizing the panel for its maximum."""

    def __init__(self, value, decimals=2, step=1):
        super().__init__()
        self.setDecimals(decimals)
        self.setRange(.01, sys.float_info.max)
        self.setSingleStep(step)
        self.setValue(value)
        self.setKeyboardTracking(False)
        self.setMinimumWidth(90)

    def sizeHint(self):
        return QSize(140, super().sizeHint().height())

    def minimumSizeHint(self):
        return QSize(90, super().minimumSizeHint().height())


def _raw_text(snapshot):
    raw = snapshot.get("raw") or {}
    axes = raw.get("axes") or []
    buttons = raw.get("buttons") or []
    hats = raw.get("hats") or []
    values = "  ".join(f"{index}: {float(value):+.3f}" for index, value in enumerate(axes))
    pressed = ", ".join(str(index) for index, value in enumerate(buttons) if value) or "无"
    return f"轴：{values or '无'}\n按键：{pressed}\n方向帽：{hats or '无'}"


class GamepadPanel(QFrame):
    """Emit user intent; the window/session owns all reader and robot actions."""

    action_requested = Signal(str)
    config_requested = Signal()
    tuning_requested = Signal(dict)
    gripper_base_requested = Signal(float)

    _STAGES = {
        "idle": "未接管", "connected": "待使能", "disconnected": "设备未连接",
        "starting": "正在使能", "enabled": "已使能 · 待初始化",
        "wait_idle": "等待主控空闲",
        "joint_queued": "等待初始化受理",
        "joint_moving": "关节运动中",
        "init_confirmation": "等待确认到位",
        "int_mode": "切换 INT", "int_check": "确认 INT", "centering": "等待手柄回中",
        "ready": "已就绪 · 手柄可控制", "rested": "休息完成 · 待初始化",
        "target_rejected": "目标被拒绝 · 等待回中",
        "released": "已释放 · 实机可能仍在运动",
        "stopping": "停止确认中", "stopped": "已停止", "disabling": "失能确认中",
        "disabled": "已失能", "fault": "故障锁定", "protected": "保护锁定",
        "calibrating": "校准中",
    }

    def __init__(self, config=None, parent=None):
        super().__init__(parent)
        config = config or default_config()
        self.setObjectName("GamepadPanel")
        self._reader_running = False
        self._state = {}
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)
        self.status_label = QLabel("未接管 · 检查输入不会发送指令")
        self.status_label.setObjectName("GamepadStatus")
        outer.addWidget(self.status_label)
        row = QHBoxLayout()
        row.setSpacing(5)
        self.check_button = button("检查输入", lambda: self.action_requested.emit("check"))
        self.claim_button = button("接管控制", lambda: self.action_requested.emit("claim"), "primary")
        self.release_button = button("释放", lambda: self.action_requested.emit("release"))
        self.release_button.setToolTip("停止手柄后续发令，不发送STOP；实机可能继续执行最后目标。")
        self.settings_button = button("设置", self.config_requested.emit)
        for control in (self.check_button, self.claim_button, self.release_button, self.settings_button):
            control.setMinimumWidth(0)
            row.addWidget(control, 1)
        outer.addLayout(row)

        self.scroll = QScrollArea()
        self.scroll.setObjectName("GamepadPanelScroll")
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll.setMinimumHeight(48)
        self.scroll.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Ignored)
        body = QWidget()
        layout = QVBoxLayout(body)
        layout.setContentsMargins(2, 2, 2, 4)
        layout.setSpacing(6)
        self.device_label = hint("手柄未检查", "muted")
        self.message_label = hint("先检查输入，再主动接管。进入本页不会使能或运动。", "muted")
        layout.addWidget(self.device_label)
        layout.addWidget(self.message_label)
        self.initial_hint = hint("")
        actions = QGridLayout()
        actions.setSpacing(5)
        self.action_buttons = {}
        for index, (key, title, role) in enumerate((
                ("start", "Start", "success"), ("initialize", "初始化", "primary"),
                ("rest", "休息", "slate"),
                ("disable", "失能", "secondary"), ("stop", "STOP", "danger"))):
            control = button(title, lambda action=key: self.action_requested.emit(action), role)
            self.action_buttons[key] = control
            actions.addWidget(control, index // 3, index % 3, 1, 2 if key == "stop" else 1)
        layout.addLayout(actions)
        layout.addWidget(self.initial_hint)

        self.tuning_widget = QWidget()
        tuning = QGridLayout(self.tuning_widget)
        tuning.setContentsMargins(0, 0, 0, 0)
        tuning.setHorizontalSpacing(6)
        tuning.setVerticalSpacing(5)
        self.linear_speed_spin = _UncappedSpeedSpinBox(config["control"]["linear_speed_mm_s"])
        self.angular_speed_spin = _UncappedSpeedSpinBox(config["control"]["angular_speed_deg_s"])
        self.speed_spin = number(config["robot"]["cmd_speed"], 1, 100, 0)
        self.tool_spin = number(config["robot"]["tool_offset_z"], 0, 1000, 1, 1)
        self.linear_speed_spin.setToolTip("满杆时的软件 TCP 目标速度，默认200 mm/s；轻推按杆量比例减速，暂不设输入上限。实际运动受电机档位与加速度限制，不保证恒定线速度。")
        self.angular_speed_spin.setToolTip("摇杆每秒改变姿态目标的速率，暂不设上限；软件命令旋转步长限制仍生效。不是 MoveL 指令的速度档位。")
        self.speed_spin.setToolTip("MoveL 的主控电机速度上限：档位 35 最高 10.5 r/s，档位 100 最高 30 r/s，不是 mm/s；同步规划时各轴可能更低。初始化仍使用独立档位 10。线速度过高而档位过低时，实物可能跟不上软件目标；实际模型按软件设置轮询更新，回读不自动修正手柄目标。")
        for index, (title, control, suffix) in enumerate((
                ("线速度", self.linear_speed_spin, " mm/s"),
                ("角速度", self.angular_speed_spin, " °/s"),
                ("速度档位", self.speed_spin, ""),
                ("工具长度", self.tool_spin, " mm"))):
            control.setMinimumWidth(70)
            control.setSuffix(suffix)
            tuning.addWidget(QLabel(title), index // 2, index % 2 * 2)
            tuning.addWidget(control, index // 2, index % 2 * 2 + 1)
        self.apply_button = button("应用设置", self._apply_tuning)
        tuning.addWidget(self.apply_button, 2, 0, 1, 4)
        layout.addWidget(self.tuning_widget)
        base = QHBoxLayout()
        self.gripper_base_spin = number(100, 0, 100, 0)
        self.gripper_base_spin.setSuffix(" %")
        self.gripper_base_button = button("发送夹爪位置", lambda: self.gripper_base_requested.emit(self.gripper_base_spin.value()))
        self.gripper_base_button.setToolTip("发送此开度并设为软件目标；初始化后默认目标100%，可直接用LT减小、RT增加。无实际开度回读。")
        base.addWidget(self.gripper_base_spin)
        base.addWidget(self.gripper_base_button, 1)
        layout.addLayout(base)
        self.tcp_label = hint("TCP 软件目标：—\n夹爪目标：未建立", "muted")
        layout.addWidget(self.tcp_label)
        layout.addWidget(hint("左摇杆 X/Y · 右摇杆上下 Z · 十字键 Pitch/Yaw · LB/RB Roll。夹爪默认100%，LT减小、RT增加。Back 软停；A 初始位姿；Y 休息。"))
        self.raw_label = hint("原始输入尚未采集", "muted")
        self.raw_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.raw_label)
        layout.addStretch()
        self.scroll.setWidget(body)
        outer.addWidget(self.scroll, 1)
        self.set_config(config)
        self.set_session_state({"stage": "idle", "claimed": False})

    def set_config(self, config):
        control, robot = config["control"], config["robot"]
        self.linear_speed_spin.setValue(control["linear_speed_mm_s"])
        self.angular_speed_spin.setValue(control["angular_speed_deg_s"])
        self.speed_spin.setValue(robot["cmd_speed"])
        self.tool_spin.setValue(robot["tool_offset_z"])
        joints = " / ".join(f"{value:g}" for value in robot["init_joints"])
        self.initial_hint.setText(f"第一次 Start 仅使能；使能后按 A（按键0）或第二次 Start 发一条初始化 MoveJ，关节至 {joints}°，地轨固定 0 mm。观察实物到位后再按 Start（按键7）或点击“确认到位”，手柄回中后解锁；连续控制需预先设置 INT。初始化不查询运动状态、不自动切换模式；实际模型按软件设置共用关节位置轮询。")

    def _apply_tuning(self):
        self.tuning_requested.emit({
            "linear_speed_mm_s": self.linear_speed_spin.value(),
            "angular_speed_deg_s": self.angular_speed_spin.value(),
            "cmd_speed": self.speed_spin.value(), "tool_offset_z": self.tool_spin.value(),
        })

    def set_reader_status(self, status):
        self._reader_running = bool(status.get("running", False))
        self.check_button.setText("停止检查" if self._reader_running else "检查输入")
        name = str(status.get("name") or "未连接手柄")
        error = str(status.get("error") or "")
        self.device_label.setText(name + (f" · {error}" if error else ""))
        self.device_label.setToolTip(name + (f"\n{error}" if error else ""))
        self.set_session_state(self._state)

    def set_sample(self, snapshot):
        self.raw_label.setText(_raw_text(snapshot))
        self.raw_label.setToolTip(str(snapshot.get("error") or ""))

    def set_session_state(self, state):
        self._state = dict(state)
        claimed = bool(state.get("claimed", False))
        pending = bool(state.get("release_pending", False))
        stage = str(state.get("stage", "idle"))
        self.status_label.setText(self._STAGES.get(stage, stage) if claimed or stage == "released" else "未接管 · 仅检查输入")
        self.status_label.setProperty("ready", bool(state.get("ready", False)))
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)
        self.message_label.setText(str(state.get("message") or "先检查输入，再主动接管。"))
        self.claim_button.setEnabled(not claimed)
        self.release_button.setEnabled(claimed and not pending)
        self.check_button.setEnabled(not claimed)
        self.settings_button.setEnabled(not claimed)
        self.tuning_widget.setEnabled(not claimed)
        available = claimed and not pending
        allowed = {
            "start": (stage in ("connected", "disabled", "stopped", "protected", "enabled", "rested", "init_confirmation")
                      or stage == "fault" and bool(state.get("stop_confirmed"))),
            "initialize": stage in ("enabled", "rested", "ready", "init_confirmation"),
            "rest": bool(state.get("ready")),
            "disable": stage != "disabling",
        }
        for key, control in self.action_buttons.items():
            control.setEnabled(key == "stop" or (available and allowed[key]))
        self.action_buttons["initialize"].setText("确认到位" if stage == "init_confirmation" else "初始化")
        self.action_buttons["start"].setToolTip("第三次Start确认实物到位；只在本地解锁，不发送命令" if stage == "init_confirmation" else "第一次使能，第二次初始化，第三次确认到位；默认按键7")
        self.action_buttons["initialize"].setToolTip("请观察实物到位后确认；只在本地解锁，不发送查询或模式指令" if stage == "init_confirmation" else "只发送初始化 MoveJ")
        self.gripper_base_button.setEnabled(available and bool(state.get("ready")))
        tcp = state.get("target_tcp")
        gripper = state.get("gripper_target")
        position = " / ".join(f"{float(value):.1f}" for value in tcp) if tcp else "—"
        opening = f"{float(gripper):g}%（软件目标）" if gripper is not None else "未建立"
        if state.get("gripper_paused"):
            opening += " · 发送暂停，点击发送夹爪位置重试"
        self.tcp_label.setText(f"TCP 软件目标：{position}\n夹爪目标：{opening}")


class GamepadSettingsDialog(QDialog):
    """Edit a copied local profile; only explicit acceptance returns changes."""

    diagnostic_requested = Signal(bool)

    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.setObjectName("GamepadSettingsDialog")
        self.setWindowTitle("手柄设置与输入诊断")
        self.resize(660, 640)
        self.setMinimumSize(520, 420)
        self._config = deepcopy(config)
        self._checking = False
        self.fields = {}
        outer = QVBoxLayout(self)
        outer.addWidget(hint("设置保存在本机；应用配置不发送机器人指令。检查输入只读取手柄。"))
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        body = QWidget()
        layout = QVBoxLayout(body)
        self.device_combo = QComboBox()
        self.device_combo.addItem("按设备编号选择", "")
        self.device_combo.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        device = QFormLayout()
        device.addRow("已检测设备", self.device_combo)
        self.device_index = QSpinBox()
        self.device_index.setRange(0, 255)
        self.device_index.setValue(config["gamepad"].get("device_index", 0))
        device.addRow("手柄编号", self.device_index)
        self.device_guid = QLineEdit(str(config["gamepad"].get("device_guid", "")))
        self.device_guid.setMaxLength(128)
        self.device_guid.setPlaceholderText("留空按编号；填写后优先匹配 GUID")
        device.addRow("设备 GUID", self.device_guid)
        self.device_combo.currentIndexChanged.connect(self._device_selected)
        layout.addLayout(device)
        self._form(layout, "摇杆与扳机", (
            ("gamepad.deadzone", "摇杆死区", 0, .95, 2, .01),
            ("gamepad.trigger_deadzone", "扳机死区", 0, .95, 2, .01),
            ("gamepad.trigger_min", "扳机最小值", -1, 1, 2, .1),
            ("gamepad.trigger_max", "扳机最大值", -1, 1, 2, .1),
            ("gamepad.hat", "方向帽编号", 0, 255, 0, 1),
        ))
        self._form(layout, "轴编号映射", tuple(
            (f"gamepad.axes.{key}", title, 0, 255, 0, 1) for key, title in (
                ("left_x", "左摇杆左右 → Y"), ("left_y", "左摇杆上下 → X"),
                ("right_y", "右摇杆上下 → Z"), ("lt", "LT → 张开"), ("rt", "RT → 闭合"))))
        self._form(layout, "按键编号映射", tuple(
            (f"gamepad.buttons.{key}", title, -1 if key == "initialize" else 0, 255, 0, 1) for key, title in (
                ("start", "Start → 使能 / 初始化"), ("stop", "Back → 软停"),
                ("initialize", "A → 初始位姿"),
                ("rest", "Y → 休息"),
                ("roll_minus", "LB → Roll −"), ("roll_plus", "RB → Roll +"))))
        self.fields["gamepad.buttons.initialize"].setSpecialValueText("未绑定")
        self._form(layout, "软件目标速度与发送", (
            ("control.linear_speed_mm_s", "平移速度（mm/s）", .01, None, 2, 1),
            ("control.angular_speed_deg_s", "旋转速度（°/s）", .01, None, 2, 1),
            ("control.gripper_speed_percent_s", "夹爪速度（%/s）", .01, 150, 2, 1),
            ("robot.cmd_speed", "MoveL 速度档位", 1, 100, 0, 1),
            ("robot.init_speed", "初始化速度档位", 1, 100, 0, 1),
            ("robot.tool_offset_z", "工具长度（mm）", 0, 1000, 1, 1),
            ("control.max_position_lead_mm", "最小位置目标缓冲（mm）", .1, 50, 1, .5),
            ("control.max_rotation_lead_deg", "最小旋转目标缓冲（°）", .1, 45, 1, .5),
            ("control.send_interval_s", "运动发送间隔（秒）", .05, 1, 3, .01),
            ("control.velocity_smoothing_s", "速度平滑时间（秒，0关闭）", 0, .5, 3, .01),
            ("control.gripper_interval_s", "夹爪发送间隔（秒）", .02, 1, 3, .01),
            ("control.response_timeout_s", "指令响应超时（秒）", .1, 30, 2, .1),
            ("robot.motion_timeout_s", "关节运动超时（秒）", 1, 600, 1, 1),
        ))
        self.fields["control.linear_speed_mm_s"].setToolTip("满杆时的软件 TCP 目标速度，默认200 mm/s；轻推按杆量比例减速，暂不设输入上限。实际运动受电机档位与加速度限制，不保证恒定线速度。")
        self.fields["control.angular_speed_deg_s"].setToolTip("摇杆每秒改变姿态目标的速率，暂不设上限；软件命令旋转步长限制仍生效。不是 MoveL 指令的速度档位。")
        for key in ("control.max_position_lead_mm", "control.max_rotation_lead_deg"):
            self.fields[key].setToolTip("目标缓冲取此值和两次发送周期对应位移中的较大者，以最近已写出目标为参考；超出时截断增量，不整次丢弃、不等待受理。不能据此判断实物误差。")
        self.fields["control.send_interval_s"].setToolTip("默认100 ms发送最新目标，不等待每条accepted MOVEL；目标相同不重复发送，串口尚未写出时合并后续变化。")
        self.fields["control.velocity_smoothing_s"].setToolTip("平滑速度和换向，默认60 ms；0为线性即时响应。摇杆回中立即停止新增位移，STOP不受平滑影响。")
        self.fields["robot.cmd_speed"].setToolTip("MoveL 的主控电机速度上限：档位 35 最高 10.5 r/s，档位 100 最高 30 r/s，不是 mm/s；同步规划时各轴可能更低。")
        self.fields["robot.init_speed"].setToolTip("初始化与休息 MoveJ 的独立电机速度档位，不随 MoveL 速度档位变化。")
        layout.addWidget(hint("十字键竖向为 Pitch，横向为 Yaw。使能后按 A 或第二次 Start 发送一条初始化 MoveJ；观察实物到位后按Start确认，或点击“确认到位”，再将手柄回中。A 不代替到位确认。连续控制需预先设置 INT；左侧实际模型通过 GETJPOS 更新，轮询开关和频率使用软件设置（1～30 Hz）。回读不覆盖手柄软件目标。初始化与休息时地轨固定 0 mm。"))
        self.raw_view = QPlainTextEdit()
        self.raw_view.setObjectName("GamepadRawInput")
        self.raw_view.setReadOnly(True)
        self.raw_view.setMinimumHeight(90)
        self.raw_view.setMaximumHeight(140)
        self.raw_view.setPlainText("尚未检查输入")
        self.raw_view.setLineWrapMode(QPlainTextEdit.LineWrapMode.WidgetWidth)
        layout.addWidget(self.raw_view)
        layout.addStretch()
        scroll.setWidget(body)
        outer.addWidget(scroll, 1)
        row = QHBoxLayout()
        self.check_button = button("检查原始输入", self._toggle_check)
        row.addWidget(self.check_button)
        row.addStretch()
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Cancel)
        buttons.addButton("应用设置", QDialogButtonBox.ButtonRole.AcceptRole)
        buttons.button(QDialogButtonBox.StandardButton.Cancel).setText("取消")
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        row.addWidget(buttons)
        outer.addLayout(row)

    def _form(self, layout, title, descriptors):
        group = QFrame()
        group.setObjectName("Card")
        box = QVBoxLayout(group)
        box.addWidget(QLabel(title))
        form = QFormLayout()
        for path, text, minimum, maximum, decimals, step in descriptors:
            value = self._config
            for part in path.split("."):
                value = value[part]
            control = (_UncappedSpeedSpinBox(value, decimals, step) if maximum is None
                       else number(value, minimum, maximum, decimals, step))
            self.fields[path] = control
            form.addRow(text, control)
        box.addLayout(form)
        layout.addWidget(group)

    def _device_selected(self, index):
        guid = str(self.device_combo.itemData(index) or "")
        self.device_guid.setText(guid)
        if index:
            self.device_index.setValue(self.device_combo.itemData(index, Qt.ItemDataRole.UserRole + 1))

    def _toggle_check(self):
        self._checking = not self._checking
        self.check_button.setText("停止检查" if self._checking else "检查原始输入")
        self.diagnostic_requested.emit(self._checking)

    def set_reader_status(self, status):
        self._checking = bool(status.get("running", False))
        self.check_button.setText("停止检查" if self._checking else "检查原始输入")
        guid = self.device_guid.text()
        self.device_combo.blockSignals(True)
        self.device_combo.clear()
        self.device_combo.addItem("按设备编号选择", "")
        for device in status.get("devices") or []:
            self.device_combo.addItem(f"{device['index']} · {device['name']}", device.get("guid", ""))
            self.device_combo.setItemData(self.device_combo.count() - 1, device["index"], Qt.ItemDataRole.UserRole + 1)
        index = self.device_combo.findData(guid) if guid else 0
        self.device_combo.setCurrentIndex(max(0, index))
        self.device_combo.blockSignals(False)
        if status.get("error"):
            self.raw_view.setPlainText(str(status["error"]))

    def set_sample(self, snapshot):
        self.raw_view.setPlainText(_raw_text(snapshot))

    def result_config(self):
        result = deepcopy(self._config)
        result["gamepad"]["device_index"] = self.device_index.value()
        result["gamepad"]["device_guid"] = self.device_guid.text().strip()
        for path, control in self.fields.items():
            parts = path.split(".")
            target = result
            for key in parts[:-1]:
                target = target[key]
            value = control.value()
            target[parts[-1]] = int(value) if control.decimals() == 0 else value
        return validate_config(result)

    def accept(self):
        try:
            self.result_config()
        except ValueError as error:
            QMessageBox.warning(self, "手柄设置", str(error))
            return
        super().accept()
