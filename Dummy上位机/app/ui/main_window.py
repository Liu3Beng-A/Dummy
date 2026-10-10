"""A complete Qt control console, with separate actual and edited state."""
from __future__ import annotations

from collections import deque
from datetime import datetime
from pathlib import Path
import copy
import re
import time

from PySide6.QtCore import QByteArray, QEvent, QSignalBlocker, Qt, QTimer
from PySide6.QtGui import QDesktopServices
from PySide6.QtCore import QUrl
from PySide6.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QDialog, QDockWidget, QFileDialog, QFormLayout,
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QInputDialog, QLabel, QLineEdit,
    QListWidget, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton,
    QScrollArea, QSizePolicy, QSlider, QSpinBox, QSplitter, QStackedWidget, QTableWidget,
    QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget,
)

from app.ui.adapters import FieldAdapter, TableAdapter
from app.core.dialogs import install_dialog_appearance
from app.ui.widgets import AxisRow, ColorEditor, PointShortcutButton, button, hint, number
from app.ui.gamepad_panel import GamepadPanel, GamepadSettingsDialog
from app.gamepad.backend import GamepadReader
from app.gamepad.config import default_config as default_gamepad_config, validate_config as validate_gamepad_config
from app.ui.rgb_ring import RgbRing
from app.core.rgb_simulator import RgbSimulator, default_config
from app.visualization.robot_view import RobotView
from app.protocol.parser import finite_numbers
from app.protocol.constants import (
    CAL_PARAMETERS, DEFAULT_SPEED, JOINT_DEFAULTS, JOINT_RANGES,
    POSE_DEFAULTS, POSE_RANGES, RGB_AUTO_MODES, RGB_COLOR_SOURCES, RGB_MODES,
    RGB_STATES, STALL_PARAMETERS,
)


class MainWindow(QMainWindow):
    def __init__(self, controller, store, gamepad_reader=None):
        super().__init__()
        install_dialog_appearance()
        self.controller = controller
        self.store = store
        self.fields = {}
        self._closed = False
        self._building = True
        self._gamepad_reader_active = False
        self._gamepad_settings_dialog = None
        self._gamepad_state = {}
        self._gamepad_target = None
        self._control_owner = "ui"
        self._gamepad_conflict_states = {}
        self._gamepad_close_pending = False
        self._gamepad_close_allowed = False
        self._gamepad_close_timer = QTimer(self)
        self._gamepad_close_timer.setSingleShot(True)
        self._gamepad_close_timer.timeout.connect(self._finish_gamepad_close)
        try:
            self._gamepad_config = validate_gamepad_config(store.settings.get("gamepad_config", default_gamepad_config()))
        except ValueError as error:
            self._gamepad_config = default_gamepad_config()
            controller.log(f"手柄配置无效，使用本地默认配置：{error}", "WARN")
        self._logs = deque(maxlen=int(store.settings.get("log_limit", 5000)))
        self._last_drag = {"j": 0., "l": 0., "hand": 0., "teaching": 0.}
        self._rgb_draft = default_config()
        self._rgb_dirty_paths = set()
        self._rgb_loading_editor = False
        self._rgb_device = None
        self._rgb_follow_seeded = False
        self._rgb_simulator = RgbSimulator()
        self._rgb_render_timer = QTimer(self)
        self._rgb_render_timer.setInterval(33)
        self._rgb_render_timer.timeout.connect(self._render_rgb_preview)
        self._drag_timers = {}
        self._drag_pending = {}
        self._drag_epoch = 0
        self._teaching_sync = None
        for kind in self._last_drag:
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.timeout.connect(lambda target=kind: self._flush_drag(target))
            self._drag_timers[kind] = timer
        self.points = copy.deepcopy(store.points)
        self._point_editing = False
        self.setWindowTitle("Dummy Robot · 机械臂控制台")
        self.resize(1440, 900)
        self.setMinimumSize(1120, 690)
        self._create_shell()
        self._build_motion_page()
        self._build_teaching_page()
        self._build_motor_settings_page()
        self._build_gripper_page()
        self._build_rgb_page()
        self._build_settings_page()
        self._build_log_dock()
        controller.bind_fields(**self.fields)
        controller._update_write_btn_state()
        controller._refresh_protection_node("STALL")
        controller._refresh_protection_node("CAL")
        controller.log_message.connect(self._append_log)
        controller.state_changed.connect(self._update_state)
        controller.actual_pose_changed.connect(self._actual_pose)
        controller.fields_changed.connect(self._fields_updated)
        controller.ports_changed.connect(self._ports_updated)
        controller.rgb_changed.connect(self._rgb_device_changed)
        controller.preset_pose_completed.connect(self._preset_pose_completed)
        self.gamepad_reader = gamepad_reader if gamepad_reader is not None else GamepadReader(self)
        self.gamepad_reader.sampled.connect(self._gamepad_sampled)
        self.gamepad_reader.status_changed.connect(self._gamepad_reader_status)
        controller.gamepad.state_changed.connect(self._gamepad_state_changed)
        controller.gamepad.target_changed.connect(self._gamepad_target_changed)
        controller.gamepad.set_config(self._gamepad_config)
        self._gamepad_state_changed(controller.gamepad.state())
        if hasattr(controller, "position_synced"):
            controller.position_synced.connect(self._position_synced)
        self.connection_combo.currentIndexChanged.connect(lambda *_: self._update_state({}))
        self._restore_points()
        self._building = False
        self.apply_theme(store.settings.get("theme", "light"))
        self._rgb_device_changed(controller.rgb_snapshot())
        geometry = store.settings.get("geometry", "")
        if geometry:
            self.restoreGeometry(QByteArray.fromBase64(geometry.encode("ascii")))
        window_state = store.settings.get("window_state", "")
        if window_state:
            self.restoreState(QByteArray.fromBase64(window_state.encode("ascii")))
        # Keep the terminal hidden and separate, including older docked layouts.
        self.log_dock.setFloating(True)
        self.log_dock.hide()
        self.controller.set_polling(store.settings.get("auto_poll", True), store.settings.get("poll_hz", 5))
        self.controller.refresh_ports()
        self._update_state({})
        self._sync_viewer_mode()
        self._preview_changed()
        QApplication.instance().installEventFilter(self)
        for error in store.load_errors:
            controller.log(error, "WARN")

    def _bind(self, name, widget, cast=None):
        adapter = FieldAdapter(widget, cast)
        self.fields[name] = adapter
        return adapter

    def _call(self, method, *args):
        return getattr(self.controller, method)(*args)

    @staticmethod
    def _combo(items, index=0):
        combo = QComboBox()
        combo.addItems([str(item) for item in items])
        combo.setCurrentIndex(index)
        return combo

    def _create_shell(self):
        central = QWidget()
        central.setObjectName("Workspace")
        outer = QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        self.setCentralWidget(central)

        top = QFrame()
        top.setObjectName("TopBar")
        row = QHBoxLayout(top)
        row.setContentsMargins(16, 10, 16, 10)
        row.setSpacing(8)
        row.addWidget(QLabel("串口"))
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(110)
        self.port_combo.setMaximumWidth(150)
        self.port_combo.setEditText(self.store.settings.get("port", ""))
        self._bind("cb_ports", self.port_combo)
        row.addWidget(self.port_combo)
        row.addWidget(button("刷新", self.controller.refresh_ports))
        self.connection_combo = self._combo(("USB", "UART4"))
        self.connection_combo.setCurrentText(self.store.settings.get("connection_type", "USB"))
        self.connection_combo.setToolTip("USB 为主控 USB 虚拟串口；UART4 为 115200 串口，接口能力不同")
        self.connection_combo.setMaximumWidth(86)
        row.addWidget(self.connection_combo)
        self.connect_button = button("连接设备", self._toggle_connection, "success")
        row.addWidget(self.connect_button)
        self.connection_status = QLabel("未连接")
        self.connection_status.setObjectName("StatusPill")
        row.addWidget(self.connection_status)
        row.addStretch()
        self.mode_status = QLabel("模式 —")
        self.mode_status.setObjectName("StatusPill")
        row.addWidget(self.mode_status)
        self.motor_status = QLabel("电机状态未知")
        self.motor_status.setObjectName("StatusPill")
        # Keep the controller's detailed calibration/protection status separate
        # from the compact visible pill, which has its own formatting.
        self._bind("lbl_motor_state", QLabel("电机状态未知"))
        row.addWidget(self.motor_status)
        self.log_toggle = button("终端", self._toggle_log)
        row.addWidget(self.log_toggle)
        stop = button("STOP", lambda: self._system_command("!STOP"), "danger")
        stop.setObjectName("StopButton")
        stop.setToolTip("发送 !STOP，取消上位机任务并请求主控软停")
        row.addWidget(stop)
        outer.addWidget(top)

        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        sidebar = QFrame()
        sidebar.setObjectName("Sidebar")
        sidebar.setFixedWidth(162)
        side = QVBoxLayout(sidebar)
        side.setContentsMargins(14, 24, 14, 18)
        brand = QLabel("DUMMY")
        brand.setObjectName("Brand")
        side.addWidget(brand)
        sub = QLabel("ROBOT CONTROL")
        sub.setObjectName("BrandSub")
        side.addWidget(sub)
        side.addSpacing(24)
        self.navigation = QListWidget()
        self.navigation.setObjectName("Navigation")
        self.navigation.addItems(("运动控制", "点位示教", "电机设置", "夹爪设置", "RGB 灯效", "软件设置"))
        side.addWidget(self.navigation, 1)
        foot = QLabel("6 轴 · 地轨 · 夹爪\nDummy Robot 控制台")
        foot.setObjectName("SidebarFoot")
        side.addWidget(foot)
        body.addWidget(sidebar)
        self.pages = QStackedWidget()
        self.pages.setObjectName("Pages")
        body.addWidget(self.pages, 1)
        outer.addLayout(body, 1)
        self.navigation.currentRowChanged.connect(self.pages.setCurrentIndex)
        self.navigation.currentRowChanged.connect(self._navigation_changed)
        self.navigation.setCurrentRow(0)
        self.statusBar().showMessage("离线预览可用；连接设备后可读取实际位置。")

    def _page(self, title, subtitle, scroll=True, actions=None, tabs=None):
        page = QWidget()
        if tabs is not None:
            page.setObjectName("MotorTabPage")
        layout = QVBoxLayout(page)
        margins = (12, 12, 12, 12) if tabs is not None else (20, 18, 20, 14)
        layout.setContentsMargins(*margins)
        layout.setSpacing(12)
        if tabs is None:
            heading = QLabel(title)
            heading.setObjectName("PageTitle")
            if actions is None:
                layout.addWidget(heading)
            else:
                header = QHBoxLayout()
                header.setSpacing(16)
                header.addWidget(heading)
                header.addLayout(actions, 1)
                layout.addLayout(header)
        description = hint(subtitle, "muted")
        description.setObjectName("PageSubtitle")
        layout.addWidget(description)
        if scroll:
            area = QScrollArea()
            area.setWidgetResizable(True)
            content = QWidget()
            content_layout = QVBoxLayout(content)
            content_layout.setContentsMargins(0, 0, 0, 0)
            content_layout.setSpacing(12)
            area.setWidget(content)
            layout.addWidget(area, 1)
        if tabs is None:
            self.pages.addWidget(page)
        else:
            tabs.addTab(page, title)
        return content_layout if scroll else layout

    @staticmethod
    def _group(title):
        group = QGroupBox(title)
        layout = QVBoxLayout(group)
        layout.setSpacing(9)
        return group, layout

    def _build_motion_page(self):
        row = QHBoxLayout()
        row.setSpacing(4)
        self.motion_system_buttons = []
        for text, callback, role in (
            ("启动 START", lambda: self._system_command("!START"), "success"),
            ("失能 DISABLE", lambda: self._system_command("!DISABLE"), "secondary"),
            ("回零 HOME", lambda: self._system_command("!HOME"), "primary"),
            ("休息 REST", lambda: self._system_command("!RESET"), "slate"),
            ("置零 / 校准", self._calibration_command, "calibration"),
        ):
            control = button(text, callback, role)
            row.addWidget(control)
            self.motion_system_buttons.append(control)
        self.mode_combo = self._combo(("1 · SEQ 顺序", "2 · INT 打断", "3 · TRJ 连续", "4 · TUN 扫频"), 1)
        self.mode_combo.setToolTip("TUN 使用固件内置扫频激励；切换前确认各轴运动空间")
        row.addWidget(self.mode_combo)
        self.mode_set_button = button("设置模式", lambda: self.controller._set_cmdmode(self.mode_combo.currentIndex() + 1))
        row.addWidget(self.mode_set_button)
        self.mode_query_button = button("查询", self.controller._query_command_mode)
        row.addWidget(self.mode_query_button)
        row.addStretch()
        layout = self._page("运动控制", "编辑目标与设备实际位置分别管理；三维预览不会自动发送运动指令。", False, row)
        layout.setContentsMargins(20, 14, 20, 14)
        layout.setSpacing(8)

        self.motion_splitter = QSplitter(Qt.Orientation.Horizontal)
        scene = QFrame()
        scene.setObjectName("Card")
        scene_layout = QVBoxLayout(scene)
        scene_layout.setContentsMargins(10, 10, 10, 8)
        toolbar = QHBoxLayout()
        self._viewer_mode_preference = 1 if self.store.settings.get("viewer_mode") == "actual" else 0
        self.viewer_mode = self._combo(("目标预览", "实际跟随"), self._viewer_mode_preference)
        self.viewer_mode.currentIndexChanged.connect(self._view_mode_changed)
        toolbar.addWidget(self.viewer_mode)
        for title, view in (("等轴", "isometric"), ("正面", "front"), ("俯视", "top")):
            toolbar.addWidget(button(title, lambda value=view: self.viewer.set_camera(value)))
        toolbar.addWidget(button("复位视角", lambda: self.viewer.reset_camera()))
        self.viewer_grid_button = button("网格")
        self.viewer_grid_button.setObjectName("ViewerToggle")
        self.viewer_grid_button.setCheckable(True)
        self.viewer_grid_button.setChecked(self.store.settings.get("show_grid", True))
        self.viewer_grid_button.setToolTip("显示或隐藏三维地面网格")
        self.viewer_grid_button.toggled.connect(self._grid_changed)
        toolbar.addWidget(self.viewer_grid_button)
        self.viewer_axes_button = button("坐标轴")
        self.viewer_axes_button.setObjectName("ViewerToggle")
        self.viewer_axes_button.setCheckable(True)
        self.viewer_axes_button.setChecked(self.store.settings.get("show_axes", True))
        self.viewer_axes_button.setToolTip("显示或隐藏三维坐标轴")
        self.viewer_axes_button.toggled.connect(self._axes_changed)
        toolbar.addWidget(self.viewer_axes_button)
        toolbar.addStretch()
        scene_layout.addLayout(toolbar)
        self.viewer = RobotView()
        self.viewer.setMinimumSize(320, 260)
        self.viewer.ready.connect(self._viewer_ready)
        self.viewer.status_changed.connect(lambda text: self.scene_status.setText(text))
        scene_layout.addWidget(self.viewer, 1)
        self.scene_status = QLabel("正在加载本地 URDF…")
        self.scene_status.setProperty("role", "muted")
        self.scene_status.setWordWrap(True)
        scene_layout.addWidget(self.scene_status)
        self.actual_status = hint("实际位置未回读 · 夹爪显示目标开度", "muted")
        scene_layout.addWidget(self.actual_status)
        self.motion_splitter.addWidget(scene)

        self.motion_controls = QWidget()
        self.motion_controls.setMinimumWidth(395)
        self.motion_controls.setMaximumWidth(545)
        controls_layout = QVBoxLayout(self.motion_controls)
        controls_layout.setContentsMargins(0, 0, 0, 0)
        controls_layout.setSpacing(8)
        input_row = QHBoxLayout()
        input_row.addWidget(QLabel("输入方式"))
        self.input_mode = self._combo(("界面", "手柄"))
        self.input_mode.setObjectName("MotionInputMode")
        self.input_mode_combo = self.input_mode
        self.input_mode.setToolTip("切换输入方式不会自动使能或接管；手柄检查与接管需主动操作。")
        self.input_mode.currentIndexChanged.connect(self._input_mode_changed)
        input_row.addWidget(self.input_mode, 1)
        controls_layout.addLayout(input_row)
        self.motion_input_stack = QStackedWidget()
        self.motion_input_stack.setObjectName("MotionInputStack")
        self.motion_tabs = QTabWidget()
        self.motion_tabs.setObjectName("MotionTabs")
        self.motion_tabs.setDocumentMode(True)
        self.motion_tabs.tabBar().setExpanding(True)
        joint_tab = QWidget()
        joint_tab.setObjectName("MotionTabPage")
        joint_tab_layout = QVBoxLayout(joint_tab)
        joint_tab_layout.setContentsMargins(10, 8, 10, 8)
        joint_tab_layout.setSpacing(6)
        joint_page = QWidget()
        joint_layout = QVBoxLayout(joint_page)
        joint_layout.setContentsMargins(2, 2, 2, 4)
        joint_layout.setSpacing(8)
        joint_layout.addWidget(hint("灰色值为实际回读，输入框为编辑目标。", "muted"))
        self.joint_rows = []
        for index, (limits, value) in enumerate(zip(list(JOINT_RANGES) + [(-250, 250)], list(JOINT_DEFAULTS) + [0])):
            axis = AxisRow(f"J{index+1}" if index < 6 else "地轨", limits, value, "°" if index < 6 else "mm")
            axis.changed.connect(lambda _, kind="j": self._target_edited(kind))
            axis.spin.setObjectName(f"TargetJ{index+1}")
            axis.spin.focusInEvent = self._focus_event(axis.spin.focusInEvent, index + 1)
            self.joint_rows.append(axis)
            joint_layout.addWidget(axis)
        self.joint_spins = [axis.spin for axis in self.joint_rows]
        self.fields["ent_joints"] = [FieldAdapter(axis.spin) for axis in self.joint_rows[:6]]
        self.fields["scl_joints"] = [FieldAdapter(axis.spin) for axis in self.joint_rows[:6]]
        self.fields["lbl_joints"] = [FieldAdapter(QLabel()) for _ in range(6)]
        self._bind("ent_j7", self.joint_rows[6].spin)
        self._bind("scl_j7", self.joint_rows[6].spin)
        self._bind("lbl_j7", QLabel())
        joint_scroll = QScrollArea()
        joint_scroll.setObjectName("MoveJAxisScroll")
        joint_scroll.setWidgetResizable(True)
        joint_scroll.setMinimumHeight(50)
        joint_scroll.setSizeAdjustPolicy(QScrollArea.SizeAdjustPolicy.AdjustToContents)
        joint_scroll.setWidget(joint_page)
        joint_tab_layout.addWidget(joint_scroll, 1)
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("速度档位"))
        self.j_speed = number(DEFAULT_SPEED, 1, 100, 0)
        self._bind("ent_j_speed", self.j_speed)
        speed_row.addWidget(self.j_speed)
        self.j_drag = QCheckBox("拖动发送")
        self.j_drag.toggled.connect(lambda enabled: self._drag_toggled("j", enabled))
        self._bind("movej_drag_enable", self.j_drag)
        speed_row.addWidget(self.j_drag)
        self.j_speed.setToolTip("速度 100 = 30 r/s；地轨与六轴同步到达")
        joint_tab_layout.addLayout(speed_row)
        actions = QHBoxLayout()
        self.movej_send_button = button("发送 MoveJ", lambda: self._send_motion("j"), "primary")
        self.movej_send_button.setObjectName("SendMoveJ")
        actions.addWidget(self.movej_send_button, 1)
        self.movej_read_button = button("读取 → 同步输入", lambda: self._read_motion("j"))
        self.movej_read_button.setToolTip("读取设备实际关节与地轨位置，然后同步至编辑目标；不会发送运动")
        actions.addWidget(self.movej_read_button, 1)
        joint_tab_layout.addLayout(actions)
        joint_tab_layout.addStretch()
        self.motion_tabs.addTab(joint_tab, "MoveJ · 关节 / 地轨")

        pose_tab = QWidget()
        pose_tab.setObjectName("MotionTabPage")
        pose_tab_layout = QVBoxLayout(pose_tab)
        pose_tab_layout.setContentsMargins(10, 8, 10, 8)
        pose_tab_layout.setSpacing(6)
        pose_page = QWidget()
        pose_layout = QVBoxLayout(pose_page)
        pose_layout.setContentsMargins(2, 2, 2, 4)
        pose_layout.setSpacing(8)
        self.pose_rows = []
        for index, (name, limits, value) in enumerate(zip(("X", "Y", "Z", "A", "B", "C"), POSE_RANGES, POSE_DEFAULTS)):
            axis = AxisRow(name, limits, value, "mm" if index < 3 else "°")
            # The old entries accepted coordinates beyond slider ranges. Keep
            # those ranges as convenience controls, never truncate a readback.
            axis.spin.setRange(-10000 if index < 3 else -3600,
                               10000 if index < 3 else 3600)
            axis.changed.connect(lambda _: self._target_edited("l"))
            axis.spin.setObjectName(f"TargetPose{name}")
            self.pose_rows.append(axis)
            pose_layout.addWidget(axis)
        self.pose_spins = [axis.spin for axis in self.pose_rows]
        self.fields["ent_pose"] = [FieldAdapter(axis.spin) for axis in self.pose_rows]
        self.fields["scl_pose"] = [FieldAdapter(axis.spin) for axis in self.pose_rows]
        self.fields["lbl_pose"] = [FieldAdapter(axis.readback) for axis in self.pose_rows]
        pose_scroll = QScrollArea()
        pose_scroll.setObjectName("MoveLAxisScroll")
        pose_scroll.setWidgetResizable(True)
        pose_scroll.setMinimumHeight(50)
        pose_scroll.setSizeAdjustPolicy(QScrollArea.SizeAdjustPolicy.AdjustToContents)
        pose_scroll.setWidget(pose_page)
        pose_tab_layout.addWidget(pose_scroll, 1)
        line = QHBoxLayout()
        line.addWidget(QLabel("速度档位"))
        self.l_speed = number(DEFAULT_SPEED, 1, 100, 0)
        self._bind("ent_l_speed", self.l_speed)
        line.addWidget(self.l_speed)
        self.l_drag = QCheckBox("拖动发送")
        self._bind("movel_drag_enable", self.l_drag)
        self.l_drag.toggled.connect(lambda enabled: self._drag_toggled("l", enabled))
        line.addWidget(self.l_drag)
        self.l_speed.setToolTip("三维标记表示笛卡尔目标；完整关节姿态以设备回读为准")
        pose_tab_layout.addLayout(line)
        actions = QHBoxLayout()
        self.movel_send_button = button("发送 MoveL", lambda: self._send_motion("l"), "primary")
        self.movel_send_button.setObjectName("SendMoveL")
        actions.addWidget(self.movel_send_button, 1)
        self.movel_read_button = button("读取 → 同步输入", lambda: self._read_motion("l"))
        self.movel_read_button.setToolTip("读取设备实际 XYZABC 位姿，然后同步至编辑目标；不会发送运动")
        actions.addWidget(self.movel_read_button, 1)
        pose_tab_layout.addLayout(actions)
        pose_tab_layout.addStretch()
        self.motion_tabs.addTab(pose_tab, "MoveL")
        self.motion_tabs.currentChanged.connect(self._motion_tab_changed)
        self._motion_axis_scrolls = (joint_scroll, pose_scroll)
        self.motion_input_stack.addWidget(self.motion_tabs)
        self.gamepad_panel = GamepadPanel(self._gamepad_config)
        self.gamepad_panel.action_requested.connect(self._gamepad_action)
        self.gamepad_panel.config_requested.connect(self._open_gamepad_settings)
        self.gamepad_panel.tuning_requested.connect(self._gamepad_apply_tuning)
        self.gamepad_panel.gripper_base_requested.connect(self._gamepad_gripper_base)
        self.motion_input_stack.addWidget(self.gamepad_panel)
        controls_layout.addWidget(self.motion_input_stack, 1)
        self._build_motion_gripper(controls_layout)
        self._build_quick_points(controls_layout)
        controls_layout.addStretch()
        self.motion_controls.installEventFilter(self)
        self.motion_splitter.addWidget(self.motion_controls)
        self.motion_splitter.setStretchFactor(0, 3)
        self.motion_splitter.setStretchFactor(1, 1)
        self.motion_splitter.setSizes((760, 430))
        layout.addWidget(self.motion_splitter, 1)

    def _schedule_motion_fit(self):
        if self._closed or getattr(self, "_motion_fit_pending", False):
            return
        self._motion_fit_pending = True
        QTimer.singleShot(0, self._fit_motion_controls)

    def _fit_motion_controls(self):
        """Use content height when room permits; scroll only the axes otherwise."""
        self._motion_fit_pending = False
        if self._closed:
            return
        for area in self._motion_axis_scrolls:
            content_layout = area.widget().layout()
            content_layout.invalidate()
            width = max(1, area.viewport().width())
            height = (content_layout.heightForWidth(width) if content_layout.hasHeightForWidth()
                      else content_layout.sizeHint().height())
            area.setMaximumHeight(height + 2 * area.frameWidth())
        page_layout = self.motion_tabs.currentWidget().layout()
        page_layout.invalidate()
        # QScrollArea caches its initial size hint before the theme is applied.
        # Replace that height with the current content height for the active tab.
        area = self._motion_axis_scrolls[self.motion_tabs.currentIndex()]
        natural_height = (page_layout.sizeHint().height() - area.sizeHint().height()
                          + area.maximumHeight())
        self.motion_tabs.setMaximumHeight(natural_height
                                          + self.motion_tabs.tabBar().sizeHint().height() + 4)
        self.motion_input_stack.setMaximumHeight(self.motion_tabs.maximumHeight()
                                                 if self.input_mode.currentIndex() == 0 else 16777215)

    def _input_mode_changed(self, index):
        self._cancel_drag()
        self._teaching_sync = None
        if index == 0:
            self._suspend_gamepad("切换至界面输入，手柄控制已暂停")
        for control in self.quick_point_buttons:
            control.setVisible(index == 0)
        self.motion_input_stack.setCurrentIndex(index)
        self._schedule_motion_fit()
        self._sync_viewer_mode()
        self._preview_changed()

    def _gamepad_locked(self):
        return self._control_owner == "gamepad" or bool(self._gamepad_state.get("claimed", False))

    def _start_gamepad_reader(self, config=None):
        if self._closed:
            return False
        self._gamepad_reader_active = True
        try:
            result = self.gamepad_reader.start(config or self._gamepad_config)
        except (ValueError, RuntimeError, OSError) as error:
            self._gamepad_reader_active = False
            self.controller.log(f"手柄采集未启动：{error}", "WARN")
            return False
        if result is False:
            self._gamepad_reader_active = False
        return self._gamepad_reader_active

    def _gamepad_action(self, action):
        if self._closed or self._gamepad_close_pending:
            return False
        session = self.controller.gamepad
        if action == "check":
            if self._gamepad_locked():
                return False
            if self._gamepad_reader_active:
                self._suspend_gamepad("原始输入检查已停止")
                return True
            return self._start_gamepad_reader()
        if action == "claim":
            if (self.input_mode.currentIndex() != 1 or self.navigation.currentRow() != 0 or
                    self.isMinimized() or self._gamepad_locked()):
                return False
            self._cancel_drag()
            self._teaching_sync = None
            for control in (self.j_drag, self.l_drag, self.motion_hand_drag,
                            self.hand_drag_checkbox, self.teaching_drag):
                with QSignalBlocker(control):
                    control.setChecked(False)
            if not self._gamepad_reader_active and not self._start_gamepad_reader():
                return False
            return session.claim()
        if action == "release":
            self._suspend_gamepad("已请求释放手柄控制；等待停止确认")
            return True
        if action == "stop":
            return self._system_command("!STOP")
        if not self._gamepad_state.get("claimed") or self._gamepad_state.get("release_pending"):
            return False
        if action in ("start", "initialize", "rest", "disable"):
            return getattr(session, action)()
        return False

    def _suspend_gamepad(self, reason):
        if self._closed or not hasattr(self, "gamepad_reader"):
            return
        active = self._gamepad_reader_active
        claimed = self._gamepad_locked()
        if not active and not claimed:
            return
        self._cancel_drag()
        self._teaching_sync = None
        if claimed and not self._gamepad_state.get("release_pending", False):
            self.controller.gamepad.release()
        if active:
            self._gamepad_reader_active = False
            self.gamepad_reader.stop()
        if active or claimed:
            self.statusBar().showMessage(reason)

    def _gamepad_sampled(self, snapshot):
        if self._closed or (not self._gamepad_reader_active and snapshot.get("connected")):
            return
        self.gamepad_panel.set_sample(snapshot)
        if self._gamepad_settings_dialog is not None:
            self._gamepad_settings_dialog.set_sample(snapshot)
        # Preserve the acquisition timestamp, including samples queued before a stall.
        if self._gamepad_settings_dialog is None:
            self.controller.gamepad.handle_input(snapshot)

    def _gamepad_reader_status(self, status):
        if self._closed:
            return
        self._gamepad_reader_active = bool(status.get("running", False))
        self.gamepad_panel.set_reader_status(status)
        if self._gamepad_settings_dialog is not None:
            self._gamepad_settings_dialog.set_reader_status(status)

    def _gamepad_state_changed(self, state):
        if self._closed:
            return
        self._gamepad_state = copy.deepcopy(state)
        self.gamepad_panel.set_session_state(state)
        self._sync_feedback_display()
        self._apply_gamepad_conflicts()
        if self._gamepad_close_pending and not self._gamepad_locked():
            self._finish_gamepad_close()

    def _apply_gamepad_conflicts(self):
        if self._closed or not hasattr(self, "teaching_send_button"):
            return
        locked = self._gamepad_locked()
        controls = [self.motion_tabs, self.motion_gripper_group, self.mode_combo,
                    self.mode_set_button, self.mode_query_button, self.teaching_drag,
                    self.teaching_send_button, self.teaching_read_button,
                    self.point_send_button, self.point_sequence_button,
                    self.motor_page_content, self.protection_tabs, self.pages.widget(3),
                    *self.motion_system_buttons[:1], *self.motion_system_buttons[2:]]
        for control in controls:
            if locked:
                if control not in self._gamepad_conflict_states:
                    self._gamepad_conflict_states[control] = (control.isEnabled(), control.toolTip())
                control.setEnabled(False)
                control.setToolTip("手柄正在接管或等待停止确认；先释放控制后使用此操作。")
            elif control in self._gamepad_conflict_states:
                enabled, tooltip = self._gamepad_conflict_states.pop(control)
                control.setEnabled(enabled)
                control.setToolTip(tooltip)
        for index, control in enumerate(self.quick_point_buttons):
            control.setEnabled(not locked and index < len(self.points))
        if not locked:
            self.motor_page_content.setEnabled(self.connection_combo.currentText() == "USB")
        self.gamepad_settings_button.setEnabled(not locked)

    def _gamepad_target_changed(self, target):
        if self._closed or not target.get("flange") or len(target["flange"]) != 6:
            return
        self._cancel_drag()
        self._gamepad_target = copy.deepcopy(target)
        for row, value in zip(self.pose_rows, target["flange"]):
            row.set_value(value)
        with QSignalBlocker(self.l_speed):
            self.l_speed.setValue(self._gamepad_config["robot"]["cmd_speed"])
        with QSignalBlocker(self.motion_tabs):
            self.motion_tabs.setCurrentIndex(1)
        gripper = target.get("gripper")
        if gripper is not None:
            self.hand_row.set_value(gripper)
            self.motion_hand_row.set_value(gripper)
        # Target diagnostics never manufacture or update actual robot geometry.

    def _gamepad_gripper_base(self, value):
        if self._gamepad_state.get("ready") and not self._gamepad_state.get("release_pending"):
            return self.controller.gamepad.set_gripper_base(value)
        return False

    def _gamepad_apply_tuning(self, values):
        config = copy.deepcopy(self._gamepad_config)
        config["control"].update({key: values[key] for key in ("linear_speed_mm_s", "angular_speed_deg_s")})
        config["robot"].update({key: values[key] for key in ("cmd_speed", "tool_offset_z")})
        return self._save_gamepad_config(config)

    def _save_gamepad_config(self, config):
        if self._gamepad_locked():
            self.controller.log("手柄接管期间不能修改配置，请先释放并等待停止确认。", "WARN")
            return False
        try:
            validated = validate_gamepad_config(config)
            self._suspend_gamepad("应用手柄配置，输入检查已停止")
            self.store.update_settings(gamepad_config=validated)
            self.controller.gamepad.set_config(validated)
            self.gamepad_reader.set_config(validated)
        except (ValueError, OSError) as error:
            self.controller.log(f"手柄配置未应用：{error}", "ERROR")
            QMessageBox.warning(self, "手柄配置", str(error))
            return False
        self._gamepad_config = validated
        self.gamepad_panel.set_config(validated)
        self.controller.log("手柄本地配置已应用；未发送机器人指令。", "INFO")
        return True

    def _open_gamepad_settings(self):
        if self._gamepad_locked():
            self._suspend_gamepad("打开设置前先释放手柄控制")
            return
        self._suspend_gamepad("打开手柄设置，输入检查已停止")
        dialog = GamepadSettingsDialog(self._gamepad_config, self)
        self._gamepad_settings_dialog = dialog
        dialog.diagnostic_requested.connect(self._gamepad_dialog_diagnostic)
        try:
            accepted = dialog.exec() == QDialog.DialogCode.Accepted
            if accepted:
                self._save_gamepad_config(dialog.result_config())
        finally:
            self._suspend_gamepad("手柄设置已关闭，输入检查已停止")
            self._gamepad_settings_dialog = None
            dialog.deleteLater()

    def _gamepad_dialog_diagnostic(self, enabled):
        if enabled and self._gamepad_settings_dialog is not None and not self._gamepad_locked():
            try:
                config = self._gamepad_settings_dialog.result_config()
            except ValueError as error:
                QMessageBox.warning(self._gamepad_settings_dialog, "手柄映射", str(error))
                return
            self._start_gamepad_reader(config)
        else:
            self._suspend_gamepad("原始输入检查已停止")

    def _finish_gamepad_close(self):
        if not self._gamepad_close_pending:
            return
        self._gamepad_close_timer.stop()
        if self._gamepad_locked():
            self.controller.log("关闭等待已结束，STOP 尚未确认；不会将设备状态视为已停止。", "WARN")
        self._gamepad_close_pending = False
        self._gamepad_close_allowed = True
        QTimer.singleShot(0, self.close)

    def _build_motion_gripper(self, layout):
        """Always visible under the movement tabs; mirrors the gripper page."""
        self.motion_gripper_group, box = self._group("夹爪")
        self.motion_gripper_group.setObjectName("MotionGripper")
        self.motion_gripper_group.setProperty("compact", True)
        box.setContentsMargins(8, 6, 8, 6)
        box.setSpacing(5)
        self.motion_hand_row = AxisRow("开度", (0, 100), 100, "%")
        self.motion_hand_row.spin.setDecimals(0)
        self.motion_hand_row.readback.hide()
        self.motion_hand_spin = self.motion_hand_row.spin
        self.quick_hand_spin = self.motion_hand_spin  # Existing external UI handles.
        self.motion_hand_spin.setObjectName("MotionHandPosition")
        self.motion_hand_row.changed.connect(lambda value: self._hand_position_changed("motion", value))
        box.addWidget(self.motion_hand_row)
        row = QHBoxLayout()
        self.motion_hand_drag = QCheckBox("拖发")
        self.motion_hand_drag.setToolTip("与夹爪设置页共享：拖动开度时发送位置指令")
        self.motion_hand_drag.setObjectName("MotionHandDrag")
        self.motion_hand_drag.toggled.connect(lambda enabled: self._hand_drag_changed("motion", enabled))
        row.addWidget(self.motion_hand_drag)
        row.addWidget(button("发送位置", self._send_hand_position, "primary"), 1)
        row.addWidget(button("张开", lambda: self._send_hand_direction(True), "success"), 1)
        row.addWidget(button("闭合", lambda: self._send_hand_direction(False), "secondary"), 1)
        box.addLayout(row)
        row = QHBoxLayout()
        row.addWidget(QLabel("电流"))
        self.motion_hand_current = number(1., .05, 2.5, 2, .05)
        self.motion_hand_current.setSuffix(" A")
        self.motion_hand_current.setObjectName("MotionHandCurrent")
        self.motion_hand_current.valueChanged.connect(lambda value: self._hand_current_changed("motion", value))
        row.addWidget(self.motion_hand_current, 1)
        row.addWidget(button("设置电流", self.controller._cmd_hand_set_i))
        box.addLayout(row)
        layout.addWidget(self.motion_gripper_group)

    def _build_quick_points(self, layout):
        self.quick_points_panel = QFrame()
        self.quick_points_panel.setObjectName("Card")
        box = QVBoxLayout(self.quick_points_panel)
        box.setContentsMargins(8, 6, 8, 8)
        box.setSpacing(6)
        header = QHBoxLayout()
        title = QLabel("点位快捷加载")
        title.setToolTip("按示教列表顺序显示前 4 个点位；点击仅加载目标和预览。")
        header.addWidget(title)
        header.addStretch()
        self.quick_point_save = button("保存当前点位", self._save_motion_point)
        self.quick_point_save.setToolTip("保存运动页当前编辑目标、速度与夹爪开度；不查询或发送设备指令")
        header.addWidget(self.quick_point_save)
        self.quick_point_manage = button("管理", lambda: self.navigation.setCurrentRow(1))
        self.quick_point_manage.setToolTip("打开点位示教，保存、编辑或调整点位顺序")
        header.addWidget(self.quick_point_manage)
        box.addLayout(header)
        grid = QGridLayout()
        grid.setSpacing(6)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        self.quick_point_buttons = []
        for index in range(4):
            control = PointShortcutButton(index + 1, lambda slot=index: self._load_quick_point(slot))
            self.quick_point_buttons.append(control)
            grid.addWidget(control, index // 2, index % 2)
        box.addLayout(grid)
        layout.addWidget(self.quick_points_panel)

    def _refresh_quick_points(self):
        for index, control in enumerate(self.quick_point_buttons):
            control.set_point(self.points[index] if index < len(self.points) else None)
        self._apply_gamepad_conflicts()

    def _load_quick_point(self, index):
        if self._gamepad_locked() or not 0 <= index < min(4, len(self.points)):
            return
        self.input_mode.setCurrentIndex(0)
        record = self.points[index]
        # Selection is mirrored even when the same row was already selected.
        with QSignalBlocker(self.point_table):
            self.point_table.selectRow(index)
        self._load_point_target(record)
        self.statusBar().showMessage(f"已载入：{record['name']} · 点击发送按钮执行")

    def _focus_event(self, original, joint):
        def focus(event):
            self.viewer.focus_joint(joint)
            original(event)
        return focus

    def _build_teaching_page(self):
        layout = self._page("点位示教", "独立编辑 MoveJ 或 MoveL 点位；选中、切换和回读只更新目标，顺序执行按运动类型与编号等待完成。", False)
        row = QHBoxLayout()
        self.point_name = QLineEdit()
        self.point_name.setPlaceholderText("点位名称，例如：取料起始")
        self.point_name.setMaxLength(120)
        row.addWidget(self.point_name, 1)
        row.addWidget(button("保存当前目标", self._save_point, "primary"))
        row.addWidget(button("更新选中", self._replace_point))
        row.addWidget(button("导入 JSON", self._import_points))
        row.addWidget(button("导出 JSON", self._export_points))
        layout.addLayout(row)
        self.teaching_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.point_table = QTableWidget(0, 11)
        self.point_table.setObjectName("PointTable")
        self.point_table.setHorizontalHeaderLabels(("类型", "名称", "J1/X", "J2/Y", "J3/Z", "J4/A", "J5/B", "J6/C", "地轨", "夹爪%", "速度"))
        self.point_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.point_table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.point_table.setAlternatingRowColors(True)
        self.point_table.verticalHeader().setVisible(True)
        header = self.point_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        header.setMinimumSectionSize(48)
        self.point_table.itemSelectionChanged.connect(self._select_point)
        self.point_table.itemChanged.connect(self._point_name_changed)
        self.fields["_pos_listbox"] = TableAdapter(self.point_table)
        self.teaching_splitter.addWidget(self.point_table)
        editor = QWidget()
        editor.setMinimumWidth(395)
        editor.setMaximumWidth(545)
        editor_layout = QVBoxLayout(editor)
        editor_layout.setContentsMargins(0, 0, 0, 0)
        editor_layout.setSpacing(8)
        self.teaching_tabs = QTabWidget()
        self.teaching_tabs.setObjectName("TeachingEditorTabs")
        self.teaching_joint_rows, self.teaching_pose_rows = [], []
        for kind, title, names, limits, defaults, target_rows in (
            ("MoveJ", "MoveJ · 关节 / 地轨", ("J1", "J2", "J3", "J4", "J5", "J6", "地轨"),
             list(JOINT_RANGES) + [(-250, 250)], list(JOINT_DEFAULTS) + [0], self.teaching_joint_rows),
            ("MoveL", "MoveL · 笛卡尔", ("X", "Y", "Z", "A", "B", "C"),
             list(POSE_RANGES), list(POSE_DEFAULTS), self.teaching_pose_rows),
        ):
            page = QWidget()
            panel = QVBoxLayout(page)
            panel.setContentsMargins(12, 12, 12, 10)
            panel.setSpacing(8)
            for index, (name, bounds, value) in enumerate(zip(names, limits, defaults)):
                unit = "mm" if (kind == "MoveJ" and index == 6) or (kind == "MoveL" and index < 3) else "°"
                axis = AxisRow(name, bounds, value, unit)
                axis.readback.hide()
                if kind == "MoveL":
                    axis.spin.setRange(-10000 if index < 3 else -3600, 10000 if index < 3 else 3600)
                axis.spin.setObjectName(f"Teaching{kind}{name}")
                axis.changed.connect(lambda _: self._teaching_target_edited())
                target_rows.append(axis)
                panel.addWidget(axis)
            panel.addWidget(hint("地轨只随 MoveJ 点位保存。" if kind == "MoveJ" else "仅更新笛卡尔目标标记；不生成关节逆解。", "muted"))
            panel.addStretch()
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setWidget(page)
            self.teaching_tabs.addTab(scroll, title)
        self.teaching_joint_spins = [row.spin for row in self.teaching_joint_rows]
        self.teaching_pose_spins = [row.spin for row in self.teaching_pose_rows]
        self.teaching_tabs.currentChanged.connect(self._teaching_tab_changed)
        editor_layout.addWidget(self.teaching_tabs, 1)
        editor_group, box = self._group("当前目标")
        editor_group.setProperty("compact", True)
        box.setContentsMargins(8, 6, 8, 6)
        box.setSpacing(5)
        edit_row = QHBoxLayout()
        edit_row.addWidget(QLabel("速度"))
        self.teaching_speed = number(DEFAULT_SPEED, 1, 100, 0)
        self.teaching_speed.setObjectName("TeachingSpeed")
        edit_row.addWidget(self.teaching_speed)
        self.teaching_drag = QCheckBox("拖动发送")
        self.teaching_drag.setObjectName("TeachingDrag")
        self.teaching_drag.toggled.connect(lambda enabled: self._drag_toggled("teaching", enabled))
        edit_row.addWidget(self.teaching_drag)
        edit_row.addStretch()
        box.addLayout(edit_row)
        edit_row = QHBoxLayout()
        edit_row.addWidget(QLabel("夹爪预览"))
        self.teaching_gripper_spin = number(100, 0, 100, 0)
        self.teaching_gripper_spin.setSuffix(" %")
        self.teaching_gripper_spin.setObjectName("TeachingGripperPreview")
        self.teaching_gripper_spin.valueChanged.connect(lambda *_: self._preview_changed())
        edit_row.addWidget(self.teaching_gripper_spin, 1)
        self.teaching_send_button = button("发送目标", self._send_teaching_target, "primary")
        edit_row.addWidget(self.teaching_send_button)
        box.addLayout(edit_row)
        edit_row = QHBoxLayout()
        read = button("读取 → 同步目标", self._read_teaching)
        self.teaching_read_button = read
        read.setToolTip("读取实际位置或位姿并同步当前类型的教学编辑器；不会发送运动")
        edit_row.addWidget(read)
        edit_row.addWidget(button("从运动页载入", self._load_motion_target))
        box.addLayout(edit_row)
        editor_layout.addWidget(editor_group)
        self.teaching_splitter.addWidget(editor)
        self.teaching_splitter.setStretchFactor(0, 2)
        self.teaching_splitter.setStretchFactor(1, 1)
        self.teaching_splitter.setSizes((735, 430))
        layout.addWidget(self.teaching_splitter, 1)
        bottom = QHBoxLayout()
        bottom.addWidget(button("上移", lambda: self._move_point(-1)))
        bottom.addWidget(button("下移", lambda: self._move_point(1)))
        bottom.addWidget(button("删除选中", self._delete_point))
        bottom.addWidget(button("清空", self._clear_points))
        bottom.addStretch()
        bottom.addWidget(QLabel("顺序执行速度"))
        self.point_speed = number(DEFAULT_SPEED, 1, 100, 0)
        self._bind("ent_pos_speed", self.point_speed)
        bottom.addWidget(self.point_speed)
        self.point_send_button = button("发送选中", self._send_selected_point, "primary")
        self.point_sequence_button = button("顺序执行全部", self.controller.send_all_positions, "success")
        bottom.addWidget(self.point_send_button)
        bottom.addWidget(self.point_sequence_button)
        bottom.addWidget(button("取消后续", lambda: self.controller._stop_sequence("用户取消后续点位；当前运动需软停")))
        layout.addLayout(bottom)
        self.sequence_status = QLabel("未运行")
        layout.addWidget(self.sequence_status)
        layout.addWidget(hint("MoveJ 为角度与地轨 mm；MoveL 为 XYZ mm、ABC °。发送选中使用点位速度，顺序执行使用统一速度；夹爪开度仅作预览。"))

    def _build_motor_settings_page(self):
        layout = self._page("电机设置", "电机参数、点位校准与保护设置集中管理。按下方分类选择操作。", False)
        self.motor_tabs = QTabWidget()
        self.motor_tabs.setObjectName("MotorTabs")
        self.motor_tabs.setDocumentMode(True)
        self.motor_tabs.tabBar().setExpanding(True)
        layout.addWidget(self.motor_tabs, 1)
        self._build_motor_page()
        self._build_protection_page()

    def _build_motor_page(self):
        layout = self._page("电机参数", "本页参数查询、写入与电机维护仅支持 USB。节点 0 为 ALL 查询；运行时应用与 EEPROM 保存明确区分。", tabs=self.motor_tabs)
        self.motor_page_content = layout.parentWidget()
        node_group, box = self._group("目标节点")
        row = QHBoxLayout()
        row.addWidget(QLabel("参数节点"))
        node = self._combo(range(9), 1)
        node.setMaximumWidth(100)
        self._bind("param_node", node, int)
        node.currentIndexChanged.connect(lambda *_: self.controller._update_write_btn_state() if not self._building else None)
        row.addWidget(node)
        row.addWidget(hint("0 = ALL · 1–6 关节 · 7 地轨 · 8 夹爪"), 1)
        row.addWidget(button("读取全部参数", self._query_all_parameters))
        box.addLayout(row)
        layout.addWidget(node_group)
        pid_group, box = self._group("PID · 保存直接写 EEPROM")
        row = QHBoxLayout()
        for key in ("kp", "kv", "ki", "kd"):
            row.addWidget(QLabel(key.upper()))
            spin = number(0, 0, 4294967295, 0)
            spin.setMinimumWidth(125)
            self._bind(f"ent_set_pid_{key}", spin)
            row.addWidget(spin)
        box.addLayout(row)
        row = QHBoxLayout()
        row.addStretch()
        row.addWidget(button("读取 PID", self.controller._qry_pid))
        pid_save = button("保存 PID", self.controller._set_pid, "primary")
        row.addWidget(pid_save)
        box.addLayout(row)
        layout.addWidget(pid_group)
        writes = [pid_save]
        for title, field, value, unit, low, high, query, apply, save in (
            ("加速度", "ent_set_acc", 150, "r/s²", 0, 5000, "_qry_acc", "_set_acc", "_save_acc"),
            ("最大电流", "ent_set_i", 1.5, "A", .001, 3, "_qry_i", "_set_i", "_save_i"),
        ):
            group, box = self._group(title)
            row = QHBoxLayout()
            spin = number(value, low, high, 3, .1)
            spin.setSuffix(f" {unit}")
            self._bind(field, spin)
            row.addWidget(spin)
            row.addWidget(hint("应用：运行时 RAM；保存：写 EEPROM，掉电保持。"), 1)
            row.addWidget(button("读取", lambda name=query: self._call(name)))
            apply_btn = button("应用", lambda name=apply: self._call(name))
            save_btn = button("保存", lambda name=save: self._call(name), "primary")
            writes.extend((apply_btn, save_btn))
            row.addWidget(apply_btn)
            row.addWidget(save_btn)
            box.addLayout(row)
            layout.addWidget(group)
        sync = button("同步八节点加速度至主控", self.controller._sync_acc, "primary")
        self._bind("btn_acc_sync", sync)
        layout.addWidget(sync)
        layout.addWidget(hint("建议在失能状态下同步；收到 SYNC_ACC finish 后才表示八节点回读齐全。", "warning"))
        maintenance, box = self._group("电机维护")
        encoder = button("校准所选节点磁编码器", self.controller._cmd_encoder_calibration)
        writes.append(encoder)
        box.addWidget(encoder)
        box.addWidget(hint("失能确认后才可执行。电机会主动正反转采样，成功后自动重启。"))
        row = QHBoxLayout()
        row.addWidget(QLabel("重启节点"))
        reboot_node = self._combo(range(9))
        self._bind("cb_node", reboot_node)
        row.addWidget(reboot_node)
        row.addWidget(hint("0 = 全部八个电机"), 1)
        row.addWidget(button("电机重启", self.controller._cmd_reboot))
        box.addLayout(row)
        layout.addWidget(maintenance)
        self.fields["_write_buttons"] = [FieldAdapter(item) for item in writes]
        layout.addStretch()

    def _build_protection_page(self):
        layout = self._page("电机点位校准", "参数先读取实际节点；应用仅改变 RAM，保存需要失能。运行与上次快照可在保护锁定时查询。", False, tabs=self.motor_tabs)
        tabs = QTabWidget()
        self.protection_tabs = tabs
        nodes, entries, rows, saves = {}, {}, {}, {}
        for kind, title, descriptors, choices, default in (
            ("STALL", "普通堵转 · J1–J7", STALL_PARAMETERS, range(9), 1),
            ("CAL", "限位校准 · J7 / J8", CAL_PARAMETERS, (7, 8), 7),
        ):
            page = QWidget()
            box = QVBoxLayout(page)
            box.setContentsMargins(12, 12, 12, 12)
            node = self._combo(choices)
            node.setCurrentText(str(default))
            nodes[kind] = FieldAdapter(node, int)
            node.currentIndexChanged.connect(lambda _, value=kind: self._refresh_protection(value))
            toolbar = QHBoxLayout()
            toolbar.addWidget(QLabel("节点"))
            toolbar.addWidget(node)
            toolbar.addWidget(button("读取配置", lambda value=kind: self.controller._query_protection(value)))
            toolbar.addWidget(button("运行快照", lambda value=kind: self._query_snapshot(value, False)))
            toolbar.addWidget(button("上次快照", lambda value=kind: self._query_snapshot(value, True)))
            toolbar.addStretch()
            save = button("保存整组 EEPROM", lambda value=kind: self.controller._save_protection(value), "primary")
            saves[kind] = FieldAdapter(save)
            toolbar.addWidget(save)
            box.addLayout(toolbar)
            box.addWidget(hint("J8 接触仅用于校准，不参与普通堵转保护。" if kind == "CAL" else "J1–J6 回退使用角度；J7 回退使用毫米。ALL 展开 J1–J7。"))
            area = QScrollArea()
            area.setWidgetResizable(True)
            form = QWidget()
            grid = QGridLayout(form)
            grid.setContentsMargins(4, 4, 4, 4)
            grid.setHorizontalSpacing(16)
            grid.setVerticalSpacing(8)
            for col, text in enumerate(("参数", "单位", "编辑值", "应用", "允许范围")):
                grid.addWidget(hint(text, "muted"), 0, col)
            entries[kind], rows[kind] = {}, {}
            for index, (key, label, unit, value, low, high) in enumerate(descriptors, 1):
                text = QLabel(f"{label}\n{key}")
                text.setToolTip(key)
                grid.addWidget(text, index, 0)
                grid.addWidget(hint(unit, "muted"), index, 1)
                # Range checking remains in the original business method. A
                # line input retains values and precision from device replies.
                edit = QLineEdit(f"{value:g}")
                edit.setObjectName(f"{kind}_{key}")
                edit.setMaximumWidth(150)
                entries[kind][key] = FieldAdapter(edit)
                grid.addWidget(edit, index, 2)
                apply = button("应用", lambda mode=kind, parameter=key: self.controller._set_protection(mode, parameter))
                rows[kind][key] = FieldAdapter(apply)
                grid.addWidget(apply, index, 3)
                maximum = f"{high:g}" if high is not None else "节点约束"
                grid.addWidget(hint(f"{low:g} ～ {maximum}", "muted"), index, 4)
            grid.setColumnStretch(0, 2)
            grid.setColumnStretch(1, 1)
            grid.setRowStretch(len(descriptors) + 1, 1)
            area.setWidget(form)
            box.addWidget(area, 1)
            tabs.addTab(page, title)
        self.fields.update(_protection_nodes=nodes, _protection_entries=entries,
                           _protection_rows=rows, _protection_save_buttons=saves)
        layout.addWidget(tabs, 1)
        self.snapshot_label = hint("快照回包将显示在终端，并在此处显示最新一条。", "muted")
        self.snapshot_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.snapshot_label)

    def _build_gripper_page(self):
        layout = self._page("夹爪设置", "基础控制支持 USB 与 UART4；夹爪独立于七轴运动到位判定。三维开度目前为目标值。")
        group, box = self._group("运行时电流 · 0.05～2.5 A")
        row = QHBoxLayout()
        self.hand_current_spin = number(1., .05, 2.5, 2, .05)
        self.hand_current_spin.setSuffix(" A")
        self._bind("hand_current", self.hand_current_spin)
        self.hand_current_spin.valueChanged.connect(lambda value: self._hand_current_changed("dedicated", value))
        row.addWidget(self.hand_current_spin)
        row.addWidget(button("设置电流", self.controller._cmd_hand_set_i))
        row.addWidget(button("张开 · 电流模式", lambda: self._send_hand_direction(True), "success"))
        row.addWidget(button("闭合 · 电流模式", lambda: self._send_hand_direction(False)))
        row.addStretch()
        box.addLayout(row)
        box.addWidget(hint("电流开合持续到接触或后续指令；仅修改运行时电流，不保存 EEPROM。"))
        layout.addWidget(group)
        group, box = self._group("目标开度 · 0 闭合 / 100 张开")
        self.hand_row = AxisRow("开度", (0, 100), 100, "%")
        self.hand_row.spin.setDecimals(0)
        self.hand_row.readback.hide()
        self.hand_row.changed.connect(lambda value: self._hand_position_changed("dedicated", value))
        self._bind("hand_pos", self.hand_row.spin, lambda value: round(value))
        self._bind("lbl_hand_pos", QLabel("100"))
        box.addWidget(self.hand_row)
        row = QHBoxLayout()
        self.hand_drag_checkbox = QCheckBox("拖动发送")
        self._bind("hand_pos_drag_enable", self.hand_drag_checkbox)
        self.hand_drag_checkbox.toggled.connect(lambda enabled: self._hand_drag_changed("dedicated", enabled))
        row.addWidget(self.hand_drag_checkbox)
        row.addStretch()
        row.addWidget(button("设置夹爪位置", self._send_hand_position, "primary"))
        box.addLayout(row)
        layout.addWidget(group)

        self.hand_advanced, box = self._group("USB 高级配置 · 写电机 EEPROM")
        box.addWidget(hint("配置会写入电机持久参数。UART4 不支持；需要已确认失能再执行。夹爪独立 PID 接口采用有符号整数，上限 2147483647；电机参数页 PID 支持完整 uint32。", "warning"))
        grid = QGridLayout()
        self.hand_advanced_fields = {}
        for index, (key, title, value, low, high, decimals) in enumerate((
            ("VEL", "速度上限 r/s", 30., .001, 30., 3),
            ("ACC", "加速度 r/s²", 5., .001, 5000., 3),
            ("KP", "位置 Kp", 0, 0, 2147483647, 0),
            ("KV", "速度 Kv", 0, 0, 2147483647, 0),
            ("KI", "积分 Ki", 0, 0, 2147483647, 0),
            ("KD", "微分 Kd", 0, 0, 2147483647, 0),
        )):
            edit = number(value, low, high, decimals)
            self.hand_advanced_fields[key] = edit
            grid.addWidget(QLabel(title), index // 2, (index % 2) * 3)
            grid.addWidget(edit, index // 2, (index % 2) * 3 + 1)
            grid.addWidget(button("保存", lambda item=key: self._hand_config(item)), index // 2, (index % 2) * 3 + 2)
        box.addLayout(grid)
        row = QHBoxLayout()
        self.hand_onboot = QCheckBox("上电自动使能夹爪")
        row.addWidget(self.hand_onboot)
        row.addWidget(button("保存上电行为", lambda: self._hand_config("ONBOOT")))
        row.addStretch()
        row.addWidget(button("擦除夹爪配置", lambda: self._hand_config("ERASE")))
        box.addLayout(row)
        layout.addWidget(self.hand_advanced)
        layout.addStretch()

    def _build_rgb_page(self):
        layout = self._page("RGB 灯效", "白色散光灯环 · 预设、动画与设备状态联动。编辑不自动发送；应用改 RAM，保存整组后掉电保持。", False)
        self.rgb_page = layout.parentWidget()
        self.rgb_status = hint("尚未回读 RGB 配置。", "muted")
        # Keep complete diagnostics in the existing field and the summary tooltip.
        self.rgb_detail_label = QLabel(self.rgb_page)
        self.rgb_detail_label.hide()
        self._bind("_rgb_status_var", self.rgb_detail_label)
        layout.addWidget(self.rgb_status)
        row = QHBoxLayout()
        for text, command in (("渐开", "!RGB_ON"), ("渐关", "!RGB_OFF"),
                              ("启用自动联动", "!RGB_AUTO 1"), ("关闭自动联动", "!RGB_AUTO 0"),
                              ("查询全部", "#RGB_GET"), ("查询状态", "#RGB_STATUS")):
            row.addWidget(button(text, lambda cmd=command: self.controller.send_cmd(cmd)))
        row.addStretch()
        layout.addLayout(row)
        tabs = QTabWidget()
        self.rgb_tabs = tabs
        tabs.setMinimumWidth(530)

        page, box = self._rgb_tab(tabs, "亮度与预设")
        group, content = self._group("设定亮度")
        row = QHBoxLayout()
        bright = number(100, 0, 100, 0)
        bright.setSuffix(" %")
        self._bind("_rgb_bright", bright, lambda value: str(int(value)))
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(0, 100)
        slider.setValue(100)
        slider.valueChanged.connect(bright.setValue)
        bright.valueChanged.connect(lambda value: slider.setValue(round(value)))
        self.fields["_rgb_scale"] = FieldAdapter(slider)
        row.addWidget(slider, 1)
        row.addWidget(bright)
        row.addWidget(button("应用亮度", self.controller._rgb_apply_bright, "primary"))
        content.addLayout(row)
        content.addWidget(hint("0% 保持全黑；渐关保留设定亮度。滑块只编辑，点击应用才发送。"))
        box.addWidget(group)
        group, content = self._group("设备灯效切换")
        grid = QGridLayout()
        for index, title in enumerate(RGB_MODES[:10]):
            grid.addWidget(button(title, lambda mode=index: self._rgb_apply_mode(mode)), index // 5, index % 5)
        content.addLayout(grid)
        box.addWidget(group)
        group, content = self._group("单色预设")
        row = QHBoxLayout()
        color_index = self._combo(range(3))
        self._bind("_rgb_color_idx", color_index)
        color_index.currentIndexChanged.connect(lambda *_: self.controller._rgb_fill_color() if not self._building else None)
        row.addWidget(QLabel("预设"))
        row.addWidget(color_index)
        row.addStretch()
        row.addWidget(button("应用颜色", self.controller._rgb_apply_color))
        content.addLayout(row)
        color = ColorEditor((0, 0, 255))
        self.fields["_rgb_color_vars"] = [FieldAdapter(spin, str) for spin in color.channels]
        content.addWidget(color)
        box.addWidget(group)
        group, content = self._group("过渡时间 · ms")
        grid = QGridLayout()
        fades = []
        for index, (title, value, maximum) in enumerate((("渐开", 1000, 10000), ("渐关", 1000, 10000), ("亮度变化", 300, 10000), ("灯效切换", 300, 5000))):
            grid.addWidget(QLabel(title), 0, index)
            edit = number(value, 0, maximum, 0, 50)
            fades.append(FieldAdapter(edit, lambda value: str(int(value))))
            grid.addWidget(edit, 1, index)
        self.fields["_rgb_fades"] = fades
        content.addLayout(grid)
        content.addWidget(button("应用过渡时间", self.controller._rgb_apply_fades))
        box.addWidget(group)
        box.addStretch()

        page, box = self._rgb_tab(tabs, "自动颜色")
        group, content = self._group("SEQ / INT / TRJ 独立颜色")
        auto_index = self._combo(RGB_AUTO_MODES)
        self._bind("_rgb_auto_idx", auto_index)
        auto_index.currentIndexChanged.connect(lambda *_: self.controller._rgb_fill_auto_color() if not self._building else None)
        content.addWidget(auto_index)
        color = ColorEditor((0, 255, 0))
        self.fields["_rgb_auto_color_vars"] = [FieldAdapter(spin, str) for spin in color.channels]
        content.addWidget(color)
        row = QHBoxLayout()
        row.addWidget(button("应用自动颜色", self.controller._rgb_apply_auto_color, "primary"))
        row.addWidget(button("应用常亮灯效", lambda: self._rgb_apply_mode(10 + auto_index.currentIndex())))
        row.addWidget(button("恢复自动联动", lambda: self.controller.send_cmd("!RGB_AUTO 1")))
        content.addLayout(row)
        content.addWidget(hint("三种自动颜色分别配置，与手动预设及旋转动画颜色独立。手动预览会退出自动联动。"))
        box.addWidget(group)
        box.addStretch()

        page, box = self._rgb_tab(tabs, "动画参数")
        group, content = self._group("动画类型与配色")
        effect_mode = self._combo(RGB_MODES[3:10])
        self._bind("_rgb_effect_mode", effect_mode)
        effect_mode.currentIndexChanged.connect(lambda *_: self.controller._rgb_fill_effect() if not self._building else None)
        content.addWidget(effect_mode)
        for field, title, value in (("_rgb_effect_first", "主颜色", (255, 255, 255)), ("_rgb_effect_second", "副颜色", (0, 0, 0))):
            content.addWidget(QLabel(title))
            color = ColorEditor(value)
            self.fields[field] = [FieldAdapter(spin, str) for spin in color.channels]
            content.addWidget(color)
        grid = QGridLayout()
        effect_params = []
        for index, (title, value, low, high) in enumerate((("周期 ms", 4000, 250, 30000), ("拖尾灯数", 3, 1, 12), ("闪烁亮 ms", 200, 20, 30000), ("闪烁灭 ms", 1000, 20, 30000))):
            grid.addWidget(QLabel(title), index // 2, (index % 2) * 2)
            edit = number(value, low, high, 0)
            grid.addWidget(edit, index // 2, (index % 2) * 2 + 1)
            effect_params.append(FieldAdapter(edit, lambda value: str(int(value))))
        self.fields["_rgb_effect_params"] = effect_params
        content.addLayout(grid)
        reverse = QCheckBox("反向旋转")
        self._bind("_rgb_reverse", reverse)
        content.addWidget(reverse)
        row = QHBoxLayout()
        row.addWidget(button("应用动画参数", self.controller._rgb_apply_effect, "primary"))
        row.addWidget(button("应用该灯效", lambda: self._rgb_apply_mode(effect_mode.currentIndex()+3)))
        content.addLayout(row)
        content.addWidget(hint("副颜色用于赛博呼吸，方向和拖尾用于旋转。彩虹自行生成颜色。保护报警颜色由固件固定。"))
        box.addWidget(group)
        box.addStretch()

        page, box = self._rgb_tab(tabs, "状态绑定")
        group, content = self._group("按主控状态自动切换")
        form = QFormLayout()
        state = self._combo([title for _, title in RGB_STATES])
        self._bind("_rgb_state", state)
        state.currentIndexChanged.connect(lambda *_: self.controller._rgb_fill_state() if not self._building else None)
        mode = self._combo(RGB_MODES, 3)
        self._bind("_rgb_state_mode", mode)
        source = self._combo(RGB_COLOR_SOURCES)
        self._bind("_rgb_state_source", source)
        form.addRow("设备状态", state)
        form.addRow("绑定灯效", mode)
        form.addRow("颜色来源", source)
        content.addLayout(form)
        color = ColorEditor((0, 0, 255))
        self.fields["_rgb_state_color"] = [FieldAdapter(spin, str) for spin in color.channels]
        content.addWidget(color)
        source.currentIndexChanged.connect(lambda index: color.setEnabled(index == 1))
        color.setEnabled(False)
        content.addWidget(button("应用当前状态绑定", self.controller._rgb_apply_state, "primary"))
        content.addWidget(hint("独立颜色仅在该颜色来源被选中时使用；AUTO 来源引用自动颜色。报警不可改色，STOP / 校准期间只允许查询。"))
        box.addWidget(group)
        box.addStretch()
        columns = QHBoxLayout()
        columns.setSpacing(12)
        columns.addWidget(tabs, 3)
        self.rgb_preview_panel = QFrame()
        self.rgb_preview_panel.setObjectName("Card")
        self.rgb_preview_panel.setMinimumWidth(285)
        preview = QVBoxLayout(self.rgb_preview_panel)
        preview.setContentsMargins(10, 10, 10, 10)
        preview.setSpacing(6)
        title = QLabel("灯环模拟")
        title.setObjectName("RgbPreviewTitle")
        preview.addWidget(title)
        self.rgb_preview_source = self._combo(("本地编辑预览", "跟随设备配置"))
        self.rgb_preview_source.setToolTip("本地预览不会发送指令；设备跟随按已回读配置与状态模拟，动画相位独立。")
        preview.addWidget(self.rgb_preview_source)
        self.rgb_preview_mode = self._combo(RGB_MODES, 3)
        self.rgb_preview_mode.setToolTip("选择仅在电脑灯环中模拟的灯效；应用设备灯效使用左侧按钮。")
        preview.addWidget(self.rgb_preview_mode)
        simulation = QHBoxLayout()
        simulation.setSpacing(6)
        self.rgb_preview_state = QComboBox()
        self.rgb_preview_state.addItem("手动灯效", "MANUAL")
        for code, name in (*RGB_STATES, ("FAULT", "故障报警")):
            self.rgb_preview_state.addItem(name, code)
        self.rgb_preview_state.setToolTip("仅模拟设备状态及其灯效绑定，不改变机械臂状态。")
        simulation.addWidget(self.rgb_preview_state, 1)
        self.rgb_preview_on = QCheckBox("模拟开灯")
        self.rgb_preview_on.setChecked(True)
        simulation.addWidget(self.rgb_preview_on)
        preview.addLayout(simulation)
        self.rgb_ring = RgbRing()
        preview.addWidget(self.rgb_ring, 1)
        self.rgb_preview_status = hint("本地模拟 · 编辑不发送", "muted")
        preview.addWidget(self.rgb_preview_status)
        columns.addWidget(self.rgb_preview_panel, 2)
        layout.addLayout(columns, 1)

        for name in ("_rgb_color_idx", "_rgb_auto_idx", "_rgb_effect_mode", "_rgb_state"):
            self._rgb_load_local_editor(name)
        for name in ("_rgb_bright", "_rgb_color_idx", "_rgb_color_vars", "_rgb_auto_idx",
                     "_rgb_auto_color_vars", "_rgb_fades", "_rgb_effect_mode", "_rgb_effect_first",
                     "_rgb_effect_second", "_rgb_effect_params", "_rgb_reverse", "_rgb_state",
                     "_rgb_state_mode", "_rgb_state_source", "_rgb_state_color"):
            adapters = self.fields[name]
            for adapter in adapters if isinstance(adapters, list) else [adapters]:
                widget = adapter.widget
                signal = (widget.currentIndexChanged if isinstance(widget, QComboBox) else
                          widget.toggled if isinstance(widget, QCheckBox) else widget.valueChanged)
                signal.connect(lambda *_, field=name: self._rgb_edit_changed(field))
        tabs.currentChanged.connect(self._rgb_editor_tab_changed)
        self.rgb_preview_source.currentIndexChanged.connect(self._rgb_preview_source_changed)
        self.rgb_preview_mode.currentIndexChanged.connect(self._rgb_preview_selection_changed)
        self.rgb_preview_state.currentIndexChanged.connect(self._rgb_preview_selection_changed)
        self.rgb_preview_on.toggled.connect(self._rgb_preview_selection_changed)
        row = QHBoxLayout()
        row.addWidget(hint("未保存的配置掉电丢失；保存需要已确认失能。"), 1)
        row.addWidget(button("恢复默认（临时）", lambda: self.controller.send_cmd("!RGB_RESET")))
        row.addWidget(button("保存全部 RGB 配置", self.controller._rgb_save, "primary"))
        layout.addLayout(row)

    def _rgb_apply_mode(self, mode):
        if self.rgb_preview_source.currentIndex() == 0:
            self.rgb_preview_state.setCurrentIndex(0)
            self.rgb_preview_mode.setCurrentIndex(mode)
        self.controller.send_cmd(f"!RGB_MODE {mode}")

    def _rgb_field_values(self, name):
        return tuple(int(adapter.get()) for adapter in self.fields[name])

    def _rgb_load_local_editor(self, name):
        """Restore the selected draft bank, including edits made while offline."""
        previous = self._rgb_loading_editor
        self._rgb_loading_editor = True
        try:
            if name == "_rgb_color_idx":
                values = self._rgb_draft["colors"][self.fields[name].current()]
                groups = (("_rgb_color_vars", values),)
            elif name == "_rgb_auto_idx":
                values = self._rgb_draft["auto_colors"][self.fields[name].current() + 1]
                groups = (("_rgb_auto_color_vars", values),)
            elif name == "_rgb_effect_mode":
                effect = self._rgb_draft["effects"][self.fields[name].current() + 3]
                groups = (("_rgb_effect_first", effect[:3]), ("_rgb_effect_second", effect[3:6]),
                          ("_rgb_effect_params", (effect[6], effect[8], effect[9], effect[10])))
                self.fields["_rgb_reverse"].set(bool(effect[7]))
            elif name == "_rgb_state":
                state = RGB_STATES[self.fields[name].current()][0]
                binding = self._rgb_draft["states"][state]
                self.fields["_rgb_state_mode"].current(binding[0])
                self.fields["_rgb_state_source"].current(binding[1])
                groups = (("_rgb_state_color", binding[2:]),)
            else:
                return
            for field, values in groups:
                for adapter, value in zip(self.fields[field], values):
                    adapter.set(value)
        finally:
            self._rgb_loading_editor = previous

    def _rgb_store_draft(self, path, value):
        target = self._rgb_draft
        for key in path[:-1]:
            target = target[key]
        if target[path[-1]] != value:
            self._rgb_dirty_paths.add(path)
            target[path[-1]] = value

    def _rgb_edit_changed(self, name):
        if self._building or self._rgb_loading_editor or getattr(self.controller, "_rgb_applying_readback", False):
            return
        if name in ("_rgb_color_idx", "_rgb_auto_idx", "_rgb_effect_mode", "_rgb_state"):
            self._rgb_load_local_editor(name)
        self.rgb_preview_source.setCurrentIndex(0)
        if name.startswith("_rgb_color"):
            index = self.fields["_rgb_color_idx"].current()
            self._rgb_store_draft(("colors", index), self._rgb_field_values("_rgb_color_vars"))
            self.rgb_preview_state.setCurrentIndex(0)
            self.rgb_preview_mode.setCurrentIndex(index)
        elif name.startswith("_rgb_auto"):
            index = self.fields["_rgb_auto_idx"].current() + 1
            self._rgb_store_draft(("auto_colors", index), self._rgb_field_values("_rgb_auto_color_vars"))
            self.rgb_preview_state.setCurrentIndex(0)
            self.rgb_preview_mode.setCurrentIndex(index + 9)
        elif name.startswith("_rgb_effect") or name == "_rgb_reverse":
            index = self.fields["_rgb_effect_mode"].current() + 3
            period, tail, on, off = self._rgb_field_values("_rgb_effect_params")
            effect = (self._rgb_field_values("_rgb_effect_first") + self._rgb_field_values("_rgb_effect_second")
                      + (period, int(self.fields["_rgb_reverse"].get()), tail, on, off))
            self._rgb_store_draft(("effects", index), effect)
            self.rgb_preview_state.setCurrentIndex(0)
            self.rgb_preview_mode.setCurrentIndex(index)
        elif name.startswith("_rgb_state"):
            state = RGB_STATES[self.fields["_rgb_state"].current()][0]
            binding = (self.fields["_rgb_state_mode"].current(), self.fields["_rgb_state_source"].current())
            self._rgb_store_draft(("states", state), binding + self._rgb_field_values("_rgb_state_color"))
            self.rgb_preview_state.setCurrentIndex(self.rgb_preview_state.findData(state))
        elif name == "_rgb_bright":
            self._rgb_store_draft(("brightness",), int(self.fields[name].get()))
        elif name == "_rgb_fades":
            for key, value in zip(("fade_on_ms", "fade_off_ms", "brightness_ms", "transition_ms"),
                                  self._rgb_field_values(name)):
                self._rgb_store_draft((key,), value)
        self._render_rgb_preview()

    def _rgb_editor_tab_changed(self, index):
        if self.rgb_preview_source.currentIndex() == 0:
            self._rgb_edit_changed(("_rgb_color_idx", "_rgb_auto_idx", "_rgb_effect_mode", "_rgb_state")[index])

    def _rgb_preview_selection_changed(self, *args):
        local = self.rgb_preview_source.currentIndex() == 0
        self.rgb_preview_mode.setEnabled(local and self.rgb_preview_state.currentIndex() == 0)
        self.rgb_preview_state.setEnabled(local)
        self.rgb_preview_on.setEnabled(local)
        self._render_rgb_preview()

    def _rgb_preview_source_changed(self, index):
        self._rgb_simulator.reset()
        self._rgb_follow_seeded = False
        self._rgb_preview_selection_changed()
        self._sync_rgb_preview_activity()

    def _rgb_device_changed(self, snapshot):
        if self._closed:
            return
        previous = self._rgb_device
        self._rgb_device = snapshot
        if previous is None or previous["connection_id"] != snapshot["connection_id"]:
            self._rgb_simulator.reset()
            self._rgb_follow_seeded = False
        if snapshot["complete"]:
            incoming = copy.deepcopy(snapshot["config"])
            for path in tuple(self._rgb_dirty_paths):
                old, target = self._rgb_draft, incoming
                for key in path[:-1]:
                    old, target = old[key], target[key]
                if target[path[-1]] == old[path[-1]]:
                    self._rgb_dirty_paths.discard(path)
                else:
                    target[path[-1]] = old[path[-1]]
            self._rgb_draft = incoming
            if self.rgb_preview_source.currentIndex() == 0:
                previous_loading = self._rgb_loading_editor
                self._rgb_loading_editor = True
                try:
                    self.fields["_rgb_bright"].set(incoming["brightness"])
                    for adapter, key in zip(self.fields["_rgb_fades"],
                                            ("fade_on_ms", "fade_off_ms", "brightness_ms", "transition_ms")):
                        adapter.set(incoming[key])
                    for name in ("_rgb_color_idx", "_rgb_auto_idx", "_rgb_effect_mode", "_rgb_state"):
                        self._rgb_load_local_editor(name)
                finally:
                    self._rgb_loading_editor = previous_loading
        runtime = snapshot["runtime"]
        if not snapshot["connected"]:
            summary = "未连接设备 · 本地灯效预览可用"
        elif runtime:
            summary = (f"{'自动' if runtime['auto'] else '手动'} · 设定 {runtime['bright']}% · "
                       f"当前 {runtime['current']}% · {runtime['state']} · "
                       f"{'配置未保存' if runtime['dirty'] else '配置已保存'}")
        else:
            summary = "已连接 · 等待 RGB 配置与状态回读"
        self.rgb_status.setText(summary)
        self.rgb_status.setToolTip(str(self.fields["_rgb_status_var"].get()))
        self._render_rgb_preview()
        self._sync_rgb_preview_activity()

    def _sync_rgb_preview_activity(self):
        if not hasattr(self, "rgb_ring"):
            return
        active = (not self._closed and self.isVisible() and not self.isMinimized()
                  and self.pages.currentWidget() is self.rgb_page)
        self.rgb_ring.set_active(active)
        if active:
            if not self._rgb_render_timer.isActive():
                self._rgb_render_timer.start()
        else:
            self._rgb_render_timer.stop()
        self.controller.set_rgb_monitoring(active and self.rgb_preview_source.currentIndex() == 1)

    def _render_rgb_preview(self):
        if self._building or not hasattr(self, "rgb_ring") or self._closed:
            return
        now = int(time.monotonic() * 1000)
        if self.rgb_preview_source.currentIndex() == 0:
            config = copy.deepcopy(self._rgb_draft)
            state = self.rgb_preview_state.currentData()
            config.update(enabled=self.rgb_preview_on.isChecked(), automatic=state != "MANUAL",
                          manual_mode=self.rgb_preview_mode.currentIndex())
            source = state
            state = "DISABLED" if state == "MANUAL" else state
            status = "本地模拟 · 编辑不发送"
        else:
            snapshot = self._rgb_device or self.controller.rgb_snapshot()
            runtime = snapshot["runtime"]
            age = time.monotonic() - snapshot["received_at"] if snapshot["received_at"] is not None else None
            unavailable = (not snapshot["connected"] or not snapshot["complete"] or not runtime)
            if unavailable or age is None or age > 3:
                self._rgb_follow_seeded = False
                title = "未连接" if not snapshot["connected"] else "等待回读" if unavailable else "状态已过期"
                self.rgb_ring.set_frame([(0, 0, 0)] * 24)
                self.rgb_ring.set_caption(title, "设备配置跟随")
                self.rgb_preview_status.setText("连接并查询全部配置后显示" if unavailable else "等待下一次状态回读")
                return
            if not self._rgb_follow_seeded:
                self._rgb_simulator.reset(initial_brightness=runtime["current"])
                self._rgb_follow_seeded = True
            config, state, source = snapshot["config"], runtime["state"], runtime["source"]
            status = f"设备配置跟随 · {age:.1f}s 前回读\n{source} · 动画相位独立"
        frame = self._rgb_simulator.frame(config, state, now, source=source)
        self.rgb_ring.set_frame(frame)
        mode = RGB_MODES[self._rgb_simulator.mode].split(" ", 1)[1]
        brightness_label = "亮度" if self.rgb_preview_source.currentIndex() == 0 else "模拟"
        self.rgb_ring.set_caption(mode, f"{brightness_label} {self._rgb_simulator.current_brightness}%")
        self.rgb_preview_status.setText(status)

    def _rgb_tab(self, tabs, title):
        area = QScrollArea()
        area.setWidgetResizable(True)
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(10)
        area.setWidget(page)
        tabs.addTab(area, title)
        return page, layout

    def _build_settings_page(self):
        layout = self._page("软件设置", "本地偏好会自动保存；打开或恢复软件配置不会向设备写入参数。")
        gamepad_group, gamepad_box = self._group("手柄输入")
        gamepad_box.addWidget(hint("映射、死区、速度与设备选择保存在本机。默认不启动手柄采集；检查原始输入不接管机器人。"))
        self.gamepad_settings_button = button("手柄设置与输入诊断", self._open_gamepad_settings)
        gamepad_box.addWidget(self.gamepad_settings_button)
        layout.addWidget(gamepad_group)
        group, box = self._group("显示与刷新")
        form = QFormLayout()
        self.theme_combo = self._combo(("浅色", "深色"), 1 if self.store.settings.get("theme") == "dark" else 0)
        self.theme_combo.currentIndexChanged.connect(lambda index: self._theme_setting(index))
        form.addRow("界面主题", self.theme_combo)
        self.poll_enabled = QCheckBox("自动读取七轴实际位置")
        self.poll_enabled.setChecked(self.store.settings.get("auto_poll", True))
        self.poll_enabled.toggled.connect(lambda *_: self._save_refresh_settings())
        form.addRow("位置跟随", self.poll_enabled)
        self.poll_hz = QSpinBox()
        self.poll_hz.setRange(1, 10)
        self.poll_hz.setValue(int(self.store.settings.get("poll_hz", 5)))
        self.poll_hz.setSuffix(" Hz")
        self.poll_hz.valueChanged.connect(lambda *_: self._save_refresh_settings())
        form.addRow("位置轮询频率", self.poll_hz)
        self.grid_check = QCheckBox("显示三维地面网格")
        self.grid_check.setChecked(self.store.settings.get("show_grid", True))
        self.grid_check.toggled.connect(self._grid_changed)
        form.addRow("三维网格", self.grid_check)
        self.axes_check = QCheckBox("显示坐标轴")
        self.axes_check.setChecked(self.store.settings.get("show_axes", True))
        self.axes_check.toggled.connect(self._axes_changed)
        form.addRow("三维坐标轴", self.axes_check)
        self.log_limit_spin = QSpinBox()
        self.log_limit_spin.setRange(500, 50000)
        self.log_limit_spin.setSingleStep(500)
        self.log_limit_spin.setValue(int(self.store.settings.get("log_limit", 5000)))
        self.log_limit_spin.valueChanged.connect(self._log_limit_changed)
        form.addRow("终端保留行数", self.log_limit_spin)
        box.addLayout(form)
        box.addWidget(hint("轮询与界面渲染独立；STOP、校准、断连或已有位置请求时暂停轮询。夹爪实际开度需要固件回读支持。"))
        layout.addWidget(group)
        group, box = self._group("本地数据与程序资源")
        path_label = hint(str(self.store.base_dir), "muted")
        path_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        box.addWidget(QLabel("软件设置、点位与运行日志目录"))
        box.addWidget(path_label)
        row = QHBoxLayout()
        row.addWidget(button("打开数据目录", lambda: QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.store.base_dir)))))
        row.addWidget(button("恢复默认窗口布局", self._reset_layout))
        row.addStretch()
        box.addLayout(row)
        box.addWidget(hint("模型使用程序自带的 DummyRobot_URDF；所有三维资源在本地加载，离线可用。"))
        layout.addWidget(group)
        group, box = self._group("协议能力")
        box.addWidget(hint("当前操作页面覆盖串口助手 old 的功能以及 USB 夹爪高级配置。SEQ、INT、TRJ、TUN 为现有可确认模式；TRQ、SRV 待固件接口确认后扩展。"))
        box.addWidget(hint("UART4 使用 115200 波特率，部分电机设置及夹爪高级配置只支持 USB。操作失败会保留设备回包用于诊断。"))
        layout.addWidget(group)
        layout.addStretch()

    def _build_log_dock(self):
        self.log_dock = QDockWidget("串口终端", self)
        self.log_dock.setObjectName("TerminalDock")
        self.log_dock.setAllowedAreas(Qt.DockWidgetArea.NoDockWidgetArea)
        self.log_dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetClosable
                                  | QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        content = QWidget()
        content.setObjectName("TerminalContent")
        layout = QVBoxLayout(content)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        filters = QFrame()
        filters.setObjectName("TerminalFilterBar")
        filter_row = QHBoxLayout(filters)
        filter_row.setContentsMargins(8, 6, 8, 6)
        filter_row.setSpacing(5)
        filter_row.addWidget(QLabel("日志类型"))
        # Keep the existing filter interface for log rendering and integrations.
        self.log_filter = self._combo(("全部", "TX", "RX", "INFO", "WARN", "ERROR"))
        self.log_filter.setParent(content)
        self.log_filter.hide()
        self.log_type_group = QButtonGroup(self)
        self.log_type_group.setExclusive(True)
        self.log_type_buttons = []
        for index, title in enumerate(("全部", "发送 TX", "接收 RX", "信息", "警告", "错误")):
            control = button(title)
            control.setObjectName("LogTypeButton")
            control.setCheckable(True)
            control.setChecked(index == 0)
            self.log_type_group.addButton(control, index)
            self.log_type_buttons.append(control)
            filter_row.addWidget(control)
        self.log_type_group.idClicked.connect(self.log_filter.setCurrentIndex)
        self.log_filter.currentIndexChanged.connect(self._log_filter_changed)
        filter_row.addStretch()
        self.log_refresh_status = QLabel("实时显示")
        self.log_refresh_status.setObjectName("LogRefreshStatus")
        self.log_refresh_status.setProperty("paused", False)
        filter_row.addWidget(self.log_refresh_status)
        self.hide_joint_logs_checkbox = QCheckBox("隐藏关节回读")
        self.hide_joint_logs_checkbox.setObjectName("HideJointLogs")
        self.hide_joint_logs_checkbox.setToolTip("隐藏 #GETJPOS 发送及七轴数值回读，显示和导出均生效；完整缓冲与磁盘日志仍保留。")
        self.hide_joint_logs_checkbox.setChecked(self.store.settings.get("hide_joint_logs", True))
        self.hide_joint_logs_checkbox.toggled.connect(self._hide_joint_logs_changed)
        filter_row.addWidget(self.hide_joint_logs_checkbox)
        layout.addWidget(filters)

        toolbar = QHBoxLayout()
        toolbar.setSpacing(8)
        self.log_search = QLineEdit()
        self.log_search.setObjectName("LogSearch")
        self.log_search.setPlaceholderText("搜索日志内容…")
        self.log_search.setClearButtonEnabled(True)
        self.log_search.setMinimumWidth(160)
        self.log_search.textChanged.connect(self._render_logs)
        toolbar.addWidget(self.log_search, 1)
        self.log_pause = button("暂停显示")
        self.log_pause.setObjectName("LogPause")
        self.log_pause.setCheckable(True)
        self.log_pause.setToolTip("只暂停日志显示；串口接收和日志记录继续，恢复时显示保留的内容")
        self.log_pause.toggled.connect(self._log_pause_changed)
        toolbar.addWidget(self.log_pause)
        toolbar.addWidget(button("导出", self._export_logs))
        toolbar.addWidget(button("清空显示", self._clear_logs))
        layout.addLayout(toolbar)
        self.log_view = QPlainTextEdit()
        self.log_view.setObjectName("LogView")
        self.log_view.setReadOnly(True)
        self.log_view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.log_view.setMinimumHeight(75)
        self.log_view.setMaximumBlockCount(self._logs.maxlen)
        layout.addWidget(self.log_view, 1)
        commands = QFrame()
        commands.setObjectName("TerminalCommandBar")
        row = QHBoxLayout(commands)
        row.setContentsMargins(8, 8, 8, 8)
        row.setSpacing(8)
        row.addWidget(QLabel("指令"))
        self.custom_command = QLineEdit()
        self.custom_command.setObjectName("CustomCommand")
        self.custom_command.setPlaceholderText("输入串口指令，Enter 发送")
        self.custom_command.setToolTip("Enter 发送；↑ / ↓ 浏览指令历史")
        self.custom_command.returnPressed.connect(self.controller._send_with_history)
        self.custom_command.installEventFilter(self)
        self._bind("ent_custom_cmd", self.custom_command)
        row.addWidget(self.custom_command, 1)
        row.addWidget(button("发送", self.controller._send_with_history, "primary"))
        row.addWidget(button("运动状态", lambda: self.controller.send_cmd("#MOTION_STATUS")))
        layout.addWidget(commands)
        self.log_dock.setWidget(content)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.log_dock)
        self.log_dock.setFloating(True)
        self.log_dock.setMinimumSize(720, 340)
        self.log_dock.resize(840, 480)
        self.log_dock.hide()

    def _log_filter_changed(self, index):
        if 0 <= index < len(self.log_type_buttons):
            self.log_type_buttons[index].setChecked(True)
        self._render_logs()

    def _log_pause_changed(self, paused):
        self.log_pause.setText("继续显示" if paused else "暂停显示")
        self.log_refresh_status.setText("显示已暂停" if paused else "实时显示")
        self.log_refresh_status.setProperty("paused", paused)
        self.log_refresh_status.style().unpolish(self.log_refresh_status)
        self.log_refresh_status.style().polish(self.log_refresh_status)
        if not paused:
            self._render_logs()

    def eventFilter(self, watched, event):
        if not self._closed and not self._building:
            if watched is self and event.type() == QEvent.Type.WindowDeactivate:
                self._suspend_gamepad("窗口失焦，手柄控制已暂停")
            elif watched is QApplication.instance() and event.type() == QEvent.Type.ApplicationDeactivate:
                self._suspend_gamepad("应用失焦，手柄控制已暂停")
            elif isinstance(watched, QDialog) and event.type() == QEvent.Type.Show and watched.isWindow():
                self._suspend_gamepad("打开对话框，手柄控制已暂停")
        if watched is getattr(self, "motion_controls", None) and event.type() == QEvent.Type.Resize:
            self._schedule_motion_fit()
        if watched is getattr(self, "custom_command", None) and event.type() == QEvent.Type.KeyPress:
            if event.key() == Qt.Key.Key_Up:
                self.controller._history_up(None)
                return True
            if event.key() == Qt.Key.Key_Down:
                self.controller._history_down(None)
                return True
        return super().eventFilter(watched, event)

    def _toggle_log(self):
        if self.log_dock.isVisible():
            self.log_dock.hide()
        else:
            self._expand_log()

    def _expand_log(self):
        self._suspend_gamepad("打开终端，手柄控制已暂停")
        self.log_dock.setFloating(True)
        self.log_dock.show()
        self.log_dock.raise_()
        self.log_dock.activateWindow()

    def _toggle_connection(self):
        if self.controller.is_connected:
            self.controller.disconnect_port()
        else:
            port, connection = self.port_combo.currentText().strip(), self.connection_combo.currentText()
            self.store.update_settings(port=port, connection_type=connection)
            self.controller.connect_port(port, connection)

    def _ports_updated(self, ports):
        selected = self.port_combo.currentText()
        self.port_combo.blockSignals(True)
        self.port_combo.clear()
        self.port_combo.addItems([str(port) for port in ports])
        if selected:
            self.port_combo.setEditText(selected)
        self.port_combo.blockSignals(False)

    @staticmethod
    def _pill(widget, healthy=False, busy=False):
        widget.setProperty("healthy", healthy)
        widget.setProperty("busy", busy)
        widget.style().unpolish(widget)
        widget.style().polish(widget)

    def _update_state(self, state):
        self._control_owner = state.get("control_owner", self._control_owner)
        connected = state.get("connected", self.controller.is_connected)
        generation = state.get("generation", getattr(self.controller, "_tx_generation", 0))
        if generation != getattr(self, "_state_generation", generation):
            self._cancel_drag()
            self._teaching_sync = None
        self._state_generation = generation
        self.connect_button.setText("断开连接" if connected else "连接设备")
        self.connection_status.setText("● 已连接" if connected else "○ 未连接")
        self._pill(self.connection_status, connected)
        self.connection_combo.setEnabled(not connected)
        self.port_combo.setEnabled(not connected)
        mode = state.get("cmdmode", getattr(self.controller, "current_cmdmode", None))
        self.mode_status.setText("模式 " + {1: "SEQ", 2: "INT", 3: "TRJ", 4: "TUN", 5: "TRQ", 6: "SRV"}.get(mode, "—"))
        known = getattr(self.controller, "_motor_state_known", False)
        motor = state.get("motor_enabled", getattr(self.controller, "_motor_enabled", None) if known else None)
        locked = state.get("stop_locked", getattr(self.controller, "_stop_locked", False))
        calibrating = state.get("calibration_busy", getattr(self.controller, "_calibration_busy", False))
        if locked or calibrating or not connected:
            self._cancel_drag()
            self._teaching_sync = None
            if locked or calibrating:
                self.teaching_drag.setChecked(False)
        self.motor_status.setText("校准进行中" if calibrating else "保护 / 软停锁定" if locked else "● 已使能" if motor is True else "● 已失能" if motor is False else "电机状态未知")
        self._pill(self.motor_status, motor is True and not locked and not calibrating, locked or calibrating)
        active = state.get("sequence_active", getattr(self.controller, "_seq_active", False))
        self.sequence_status.setText(f"顺序任务 {state.get('sequence_index', 0) + 1} / {state.get('sequence_count', len(self.points))}" if active else "未运行顺序任务")
        connection = state.get("connection_type", self.connection_combo.currentText()) if connected else self.connection_combo.currentText()
        self.hand_advanced.setEnabled(connection == "USB")
        self.motor_page_content.setEnabled(connection == "USB")
        if "actual_pose" in state and state["actual_pose"] is None:
            self.viewer.set_actual_known(False)
            for row in self.joint_rows:
                row.readback.setText("—")
                row.readback.setToolTip("实际回读；未收到有效数据时显示 —")
        stale = state.get("actual_stale", True)
        self.viewer.set_stale(stale)
        status = state.get("status")
        if state.get("parameter_busy", False) and not locked and not calibrating:
            task = state.get("parameter_task") or "参数查询 / 设置"
            self.statusBar().showMessage(f"参数任务：{task} · 实际位置轮询暂停")
        elif status:
            self.statusBar().showMessage(str(status))
        self._sync_feedback_display()
        if not self._gamepad_locked() and stale:
            self.actual_status.setText("实际位置未回读 / 已过期 · 夹爪固定示意，无实际回读"
                                       if self.input_mode.currentIndex() == 1 else
                                       "实际位置未回读 / 已过期 · 夹爪显示目标开度")
        self._sync_rgb_preview_activity()
        self._apply_gamepad_conflicts()
        if self._gamepad_close_pending and not self._gamepad_locked():
            self._finish_gamepad_close()

    def _actual_pose(self, values):
        self.viewer.set_actual_pose(values, None)
        self.viewer.set_actual_known(True)
        self.viewer.set_stale(False)
        for index, (row, value) in enumerate(zip(self.joint_rows, values)):
            row.readback.setText(f"{value:.2f}")
            row.readback.setToolTip(f"J{index+1} 实际回读：{value:.2f} {'°' if index < 6 else 'mm'}")
        if self._gamepad_locked():
            self._sync_feedback_display()
        else:
            gripper_text = "夹爪固定示意，无实际回读" if self.input_mode.currentIndex() == 1 else "夹爪无实际回读"
            self.actual_status.setText(f"实际七轴已更新 · {datetime.now():%H:%M:%S} · {gripper_text}")

    def _fields_updated(self):
        self._preview_changed()

    def _viewer_ready(self):
        self._preview_changed()
        self._sync_viewer_mode()
        self._grid_changed(self.grid_check.isChecked())
        self._axes_changed(self.axes_check.isChecked())

    def _preview_changed(self):
        if not hasattr(self, "hand_row"):
            return
        if self.input_mode.currentIndex() == 1:
            self.viewer.set_cartesian_target(None)
            return
        teaching = hasattr(self, "teaching_tabs") and self.navigation.currentRow() == 1
        if teaching:
            if self.teaching_tabs.currentIndex() == 0:
                self.viewer.set_target_pose([spin.value() for spin in self.teaching_joint_spins], self.teaching_gripper_spin.value())
                self.viewer.set_cartesian_target(None)
            else:
                self.viewer.set_target_gripper(self.teaching_gripper_spin.value())
                self.viewer.set_cartesian_target([spin.value() for spin in self.teaching_pose_spins])
        elif self.motion_tabs.currentIndex() == 0:
            self.viewer.set_target_pose([spin.value() for spin in self.joint_spins], self.hand_row.spin.value())
            self.viewer.set_cartesian_target(None)
        else:
            self.viewer.set_target_gripper(self.hand_row.spin.value())
            self.viewer.set_cartesian_target([spin.value() for spin in self.pose_spins])

    def _target_edited(self, kind):
        self._preview_changed()
        if self._building or self._gamepad_locked() or getattr(self.controller, "_suppress_drag_send", False):
            return
        if not self._drag_enabled(kind):
            return
        self._queue_drag(kind)

    def _drag_enabled(self, kind):
        if kind == "teaching":
            return self.teaching_drag.isChecked()
        flag = {"j": "movej_drag_enable", "l": "movel_drag_enable", "hand": "hand_pos_drag_enable"}[kind]
        return self.fields[flag].get()

    def _queue_drag(self, kind):
        if kind == "hand":
            target = int(self.hand_row.spin.value())
        elif kind == "teaching":
            target = self._current_point(self.point_name.text().strip() or "示教目标")
        else:
            target = {"name": "运动目标", "type": "MoveJ" if kind == "j" else "MoveL",
                      "gripper": self.hand_row.spin.value(),
                      "speed": self.j_speed.value() if kind == "j" else self.l_speed.value()}
            target["joints" if kind == "j" else "pose"] = [spin.value() for spin in (self.joint_spins if kind == "j" else self.pose_spins)]
        self._drag_pending[kind] = (self._drag_epoch, getattr(self.controller, "_tx_generation", 0), copy.deepcopy(target))
        elapsed = time.monotonic() - self._last_drag[kind]
        self._drag_timers[kind].start(max(1, round((.1 - elapsed) * 1000)))

    def _flush_drag(self, kind):
        pending = self._drag_pending.pop(kind, None)
        if pending is None:
            return
        epoch, generation, target = pending
        if (self._closed or epoch != self._drag_epoch or
                self._gamepad_locked() or
                generation != getattr(self.controller, "_tx_generation", 0) or
                getattr(self.controller, "_suppress_drag_send", False) or
                getattr(self.controller, "_stop_locked", False) or
                getattr(self.controller, "_calibration_busy", False) or not self._drag_enabled(kind)):
            return
        self._last_drag[kind] = time.monotonic()
        if kind == "hand":
            self.controller.send_cmd(f"!HAND_POS {target}")
        else:
            self.controller.send_typed_position(target)

    def _cancel_drag(self, kind=None):
        if kind is None:
            self._drag_epoch += 1
            targets = tuple(self._drag_timers)
        else:
            targets = (kind,)
        for target in targets:
            self._drag_timers[target].stop()
            self._drag_pending.pop(target, None)

    def _drag_toggled(self, kind, enabled):
        if not enabled:
            self._cancel_drag(kind)
        if self._building:
            return
        if kind == "teaching":
            if enabled and self.controller.current_cmdmode != 3:
                self.controller.log("示教拖发建议使用 TRJ 连续模式；开启本身不会发送目标。", "WARN")
        else:
            self.controller._warn_drag_mode()

    def _hand_position_changed(self, source, value):
        other = getattr(self, "hand_row" if source == "motion" else "motion_hand_row", None)
        if other is not None:
            other.set_value(value)
        self._target_edited("hand")

    def _hand_current_changed(self, source, value):
        other = getattr(self, "hand_current_spin" if source == "motion" else "motion_hand_current", None)
        if other is not None:
            other.blockSignals(True)
            other.setValue(value)
            other.blockSignals(False)

    def _hand_drag_changed(self, source, enabled):
        other = getattr(self, "hand_drag_checkbox" if source == "motion" else "motion_hand_drag", None)
        if other is not None:
            other.blockSignals(True)
            other.setChecked(enabled)
            other.blockSignals(False)
        self._drag_toggled("hand", enabled)

    def _send_hand_position(self):
        self._cancel_drag("hand")
        return self.controller._cmd_hand_set_pos()

    def _send_hand_direction(self, opened):
        self._cancel_drag("hand")
        return (self.controller._cmd_hand_open if opened else self.controller._cmd_hand_close)()

    def _send_motion(self, kind):
        self._cancel_drag(kind)
        return (self.controller.send_movej if kind == "j" else self.controller.send_movel)()

    def _read_motion(self, kind):
        self._cancel_drag()
        self._teaching_sync = None
        return (self.controller.read_and_sync_from_robot if kind == "j" else self.controller.read_pose_from_robot)()

    def _system_command(self, command):
        self._cancel_drag()
        self._teaching_sync = None
        if command in ("!STOP", "!DISABLE") and hasattr(self, "teaching_drag"):
            self.teaching_drag.setChecked(False)
        if self._gamepad_locked():
            if command == "!STOP":
                return self.controller.gamepad.stop()
            if command == "!DISABLE":
                return self.controller.gamepad.disable()
            self.controller.log("手柄已接管，请先释放控制并等待停止确认。", "WARN")
            return False
        return self.controller.send_cmd(command)

    def _preset_pose_completed(self, command, values):
        """Display a completed preset without manufacturing position telemetry."""
        if self._closed or command not in ("HOME", "RESET") or len(values) != 7:
            return
        self._cancel_drag()
        self._teaching_sync = None
        previous = self.controller._suppress_drag_send
        self.controller._suppress_drag_send = True
        try:
            self.motion_tabs.setCurrentIndex(0)
            self.teaching_tabs.setCurrentIndex(0)
            for rows in (self.joint_rows, self.teaching_joint_rows):
                for row, value in zip(rows, values):
                    row.set_value(value)
        finally:
            self.controller._suppress_drag_send = previous
        self.viewer_mode.setCurrentIndex(0)
        self._preview_changed()
        self.statusBar().showMessage(f"{command} 已完成 · 模型已同步到{'回零' if command == 'HOME' else '休息'}目标")

    def _calibration_command(self):
        if self._gamepad_locked():
            self.controller.log("手柄接管期间不能校准，请先释放控制。", "WARN")
            return False
        self._cancel_drag()
        self._teaching_sync = None
        self.teaching_drag.setChecked(False)
        return self.controller._cmd_calibration()

    def _navigation_changed(self, index):
        if index != 0:
            self._suspend_gamepad("离开运动页，手柄控制已暂停")
        self._cancel_drag()
        self._teaching_sync = None
        self._preview_changed()
        self._sync_rgb_preview_activity()

    def _motion_tab_changed(self, index):
        self._schedule_motion_fit()
        self._cancel_drag("j")
        self._cancel_drag("l")
        self._preview_changed()

    def _teaching_tab_changed(self, index):
        self._cancel_drag("teaching")
        self._teaching_sync = None
        self._preview_changed()

    def _teaching_target_edited(self):
        self._preview_changed()
        if (not self._building and not self._gamepad_locked() and
                not self.controller._suppress_drag_send and self.teaching_drag.isChecked()):
            self._queue_drag("teaching")

    def _send_teaching_target(self):
        self._cancel_drag("teaching")
        return self.controller.send_typed_position(self._current_point(self.point_name.text().strip() or "示教目标"))

    def _load_motion_target(self):
        self._cancel_drag("teaching")
        self._teaching_sync = None
        source = self.joint_spins if self.teaching_tabs.currentIndex() == 0 else self.pose_spins
        rows = self.teaching_joint_rows if self.teaching_tabs.currentIndex() == 0 else self.teaching_pose_rows
        for row, spin in zip(rows, source):
            row.set_value(spin.value())
        self.teaching_speed.setValue(self.j_speed.value() if self.teaching_tabs.currentIndex() == 0 else self.l_speed.value())
        self.teaching_gripper_spin.setValue(self.hand_row.spin.value())
        self._preview_changed()

    def _read_teaching(self):
        self._cancel_drag()
        kind = "MoveJ" if self.teaching_tabs.currentIndex() == 0 else "MoveL"
        self._teaching_sync = (kind, getattr(self.controller, "_tx_generation", 0))
        method = self.controller.read_and_sync_from_robot if kind == "MoveJ" else self.controller.read_pose_from_robot
        if not method():
            self._teaching_sync = None

    def _position_synced(self, kind, values):
        self._cancel_drag()
        if self._teaching_sync != (kind, getattr(self.controller, "_tx_generation", 0)):
            return
        self._teaching_sync = None
        rows = self.teaching_joint_rows if kind == "MoveJ" else self.teaching_pose_rows
        for row, value in zip(rows, values):
            row.set_value(value)
        self._preview_changed()

    def _quick_hand_changed(self, value):
        self._hand_position_changed("motion", value)

    def _quick_hand_refresh(self, value):
        self.motion_hand_row.set_value(value)

    def _view_mode_changed(self, index):
        if not hasattr(self, "viewer"):
            return
        if self.input_mode.currentIndex() == 1:
            with QSignalBlocker(self.viewer_mode):
                self.viewer_mode.setCurrentIndex(self._viewer_mode_preference)
        else:
            self._viewer_mode_preference = index
        self._sync_viewer_mode()
        if not self._building and self.input_mode.currentIndex() == 0:
            self.store.update_settings(viewer_mode="actual" if index == 1 else "preview")

    def _sync_viewer_mode(self):
        gamepad_mode = self.input_mode.currentIndex() == 1
        self.viewer_mode.setVisible(not gamepad_mode)
        self.viewer.set_follow_actual(self._viewer_mode_preference == 1)
        self.viewer.set_actual_only(gamepad_mode)
        if gamepad_mode:
            self.viewer.set_cartesian_target(None)
            self.actual_status.setText("左侧显示最近实际姿态 · 夹爪固定示意，无实际回读")
        self._sync_feedback_display()

    def _sync_feedback_display(self):
        paused = self._gamepad_locked()
        previous = getattr(self, "_viewer_feedback_paused", None)
        if paused != previous:
            self._viewer_feedback_paused = paused
            self.viewer.set_feedback_paused(paused)
        if paused:
            self.actual_status.setText("位置回读暂停 · 保留最近实际姿态 · 夹爪无实际回读")
        elif previous:
            self.actual_status.setText("最近实际姿态 · 等待按软件设置更新 · 夹爪无实际回读")

    def _query_all_parameters(self):
        self.controller._qry_pid()
        self.controller._qry_acc()
        self.controller._qry_i()

    def _refresh_protection(self, kind):
        if not self._building:
            self.controller._refresh_protection_node(kind)

    def _query_snapshot(self, kind, last):
        (self.controller._query_stall_status if kind == "STALL" else self.controller._query_cal_status)(last)
        self.log_dock.show()

    def _hand_config(self, key):
        if self.connection_combo.currentText() != "USB":
            self.controller.log("夹爪高级配置仅支持 USB", "WARN")
            return
        if not getattr(self.controller, "_motor_state_known", False) or getattr(self.controller, "_motor_enabled", True):
            self.controller.log("请先 DISABLE 并等待已失能确认，再保存夹爪配置。", "WARN")
            return
        if key == "ERASE":
            if QMessageBox.question(self, "擦除夹爪配置", "擦除夹爪电机 EEPROM 配置并恢复默认参数？") != QMessageBox.StandardButton.Yes:
                return
            command = "!HAND_ERASE"
        elif key == "ONBOOT":
            command = f"!HAND_ONBOOT {int(self.hand_onboot.isChecked())}"
        else:
            value = self.hand_advanced_fields[key].value()
            formatted = str(int(value)) if key in ("KP", "KV", "KI", "KD") else f"{value:g}"
            command = f"!HAND_{key} {formatted}"
        self.controller.send_cmd(command)

    def _restore_points(self, selected=None):
        self._cancel_drag()
        self._point_editing = True
        self.point_table.setRowCount(len(self.points))
        for row, record in enumerate(self.points):
            kind = record.get("type", "MoveJ")
            coordinates = record["joints"][:6] if kind == "MoveJ" else record["pose"]
            rail = f"{record['joints'][6]:.2f}" if kind == "MoveJ" else "—"
            values = [kind, record["name"]] + [f"{value:.2f}" for value in coordinates] + [rail, f"{record.get('gripper',100):g}", f"{record.get('speed',35):g}"]
            for col, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                if col != 1:
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                if 2 <= col <= 7:
                    item.setToolTip("关节角 °" if kind == "MoveJ" else "XYZ mm" if col <= 4 else "ABC °")
                elif col == 8:
                    item.setToolTip("地轨 mm，仅 MoveJ 使用")
                self.point_table.setItem(row, col, item)
        self.controller.stored_positions = copy.deepcopy(self.points)
        self._point_editing = False
        if selected is not None and 0 <= selected < len(self.points):
            self.point_table.selectRow(selected)
        self._refresh_quick_points()

    def _persist_points(self, selected=None):
        try:
            self.store.save_points(self.points)
            self.points = copy.deepcopy(self.store.points)
        except (OSError, ValueError) as exc:
            self.controller.log(f"点位保存失败：{exc}", "ERROR")
        self._restore_points(selected)

    def _selected_row(self):
        selected = self.point_table.selectionModel().selectedRows()
        return selected[0].row() if selected else None

    def _current_point(self, name):
        kind = "MoveJ" if self.teaching_tabs.currentIndex() == 0 else "MoveL"
        record = {"name": name, "type": kind, "gripper": self.teaching_gripper_spin.value(),
                  "speed": self.teaching_speed.value()}
        controls = self.teaching_joint_spins if kind == "MoveJ" else self.teaching_pose_spins
        record["joints" if kind == "MoveJ" else "pose"] = [spin.value() for spin in controls]
        return record

    def _save_point(self):
        name = self.point_name.text().strip() or f"点位 {len(self.points) + 1}"
        self.points.append(self._current_point(name))
        self._persist_points(len(self.points) - 1)
        self.controller.log(f"已保存点位：{name}", "INFO")

    def _save_motion_point(self):
        self._cancel_drag()
        self._teaching_sync = None
        joint = self.motion_tabs.currentIndex() == 0
        record = {"type": "MoveJ" if joint else "MoveL",
                  "gripper": self.motion_hand_row.spin.value(),
                  "speed": (self.j_speed if joint else self.l_speed).value()}
        record["joints" if joint else "pose"] = [
            spin.value() for spin in (self.joint_spins if joint else self.pose_spins)]
        name, accepted = QInputDialog.getText(
            self, "保存当前点位",
            f"点位名称（保存当前 {record['type']} 编辑目标、速度和夹爪开度）",
            QLineEdit.EchoMode.Normal, f"点位 {len(self.points) + 1}")
        if not accepted:
            return
        name = name.strip()
        if not name or len(name) > 120:
            QMessageBox.warning(self, "点位名称", "请输入 1～120 个字符的点位名称。")
            return
        record["name"] = name
        try:
            self.store.save_points(self.points + [record])
        except (OSError, ValueError) as exc:
            self.controller.log(f"点位保存失败：{exc}", "ERROR")
            QMessageBox.warning(self, "保存失败", f"点位未保存，请重试。\n{exc}")
            return
        self.points = copy.deepcopy(self.store.points)
        with QSignalBlocker(self.point_table):
            self._restore_points(len(self.points) - 1)
        self._load_point_target(self.points[-1], preview=False)
        self.controller.log(f"已保存点位：{name}", "INFO")
        self.statusBar().showMessage(f"已保存：{name}" + (" · 快捷区显示列表前 4 个点位" if len(self.points) > 4 else ""))

    def _replace_point(self):
        index = self._selected_row()
        if index is None:
            self.controller.log("请先选择要更新的点位。", "WARN")
            return
        name = self.point_name.text().strip() or self.points[index]["name"]
        self.points[index] = self._current_point(name)
        self._persist_points(index)

    def _select_point(self):
        if self._point_editing:
            return
        row = self._selected_row()
        if row is None or row >= len(self.points):
            return
        self._load_point_target(self.points[row])

    def _load_point_target(self, record, preview=True):
        self._cancel_drag()
        self._teaching_sync = None
        self.point_name.setText(record["name"])
        # Setting a stored target must never invoke a drag-to-send handler.
        old = self.controller._suppress_drag_send
        self.controller._suppress_drag_send = True
        try:
            kind = record.get("type", "MoveJ")
            index = 0 if kind == "MoveJ" else 1
            self.teaching_tabs.setCurrentIndex(index)
            self.motion_tabs.setCurrentIndex(index)
            values = record["joints" if kind == "MoveJ" else "pose"]
            teaching_rows = self.teaching_joint_rows if kind == "MoveJ" else self.teaching_pose_rows
            motion_rows = self.joint_rows if kind == "MoveJ" else self.pose_rows
            for target, value in zip(teaching_rows, values):
                target.set_value(value)
            for target, value in zip(motion_rows, values):
                target.set_value(value)
            gripper = record.get("gripper", 100)
            self.hand_row.set_value(gripper)
            self.motion_hand_row.set_value(gripper)
            self.teaching_gripper_spin.setValue(gripper)
            speed = record.get("speed", DEFAULT_SPEED)
            self.teaching_speed.setValue(speed)
            (self.j_speed if kind == "MoveJ" else self.l_speed).setValue(speed)
        finally:
            self.controller._suppress_drag_send = old
        if preview:
            self.viewer_mode.setCurrentIndex(0)
        self._preview_changed()

    def _point_name_changed(self, item):
        if self._point_editing or item.column() != 1:
            return
        name = item.text().strip()
        if not name or len(name) > 120:
            self.controller.log("点位名称需为 1～120 个字符。", "WARN")
            self._restore_points(item.row())
            return
        self.points[item.row()]["name"] = name
        self._persist_points(item.row())

    def _delete_point(self):
        index = self._selected_row()
        if index is not None:
            del self.points[index]
            self._persist_points(min(index, len(self.points) - 1))

    def _clear_points(self):
        if self.points and QMessageBox.question(self, "清空点位", "清空所有已保存的示教点位？") == QMessageBox.StandardButton.Yes:
            self.points.clear()
            self._persist_points()

    def _move_point(self, direction):
        index = self._selected_row()
        if index is None or not 0 <= index + direction < len(self.points):
            return
        self.points[index], self.points[index + direction] = self.points[index + direction], self.points[index]
        self._persist_points(index + direction)

    def _send_selected_point(self):
        index = self._selected_row()
        if index is None:
            self.controller.log("请先选择点位。", "WARN")
            return
        self._cancel_drag()
        self.point_speed.setValue(self.points[index].get("speed", DEFAULT_SPEED))
        self.controller.send_selected_position()

    def _import_points(self):
        path, _ = QFileDialog.getOpenFileName(self, "导入示教点位", "", "点位文件 (*.json)")
        if not path:
            return
        try:
            if self.points and QMessageBox.question(self, "导入点位", "导入后将替换当前点位列表，是否继续？") != QMessageBox.StandardButton.Yes:
                return
            records = self.store.import_points(path)
            self.points = records
            self._persist_points()
            self.controller.log(f"已导入 {len(records)} 个点位。", "INFO")
        except (OSError, ValueError, TypeError) as exc:
            self.controller.log(f"点位导入失败：{exc}", "ERROR")
            QMessageBox.warning(self, "导入失败", str(exc))

    def _export_points(self):
        path, _ = QFileDialog.getSaveFileName(self, "导出示教点位", "Dummy点位.json", "点位文件 (*.json)")
        if path:
            try:
                self.store.export_points(path, self.points)
                self.controller.log(f"点位已导出：{path}", "INFO")
            except (OSError, ValueError) as exc:
                self.controller.log(f"点位导出失败：{exc}", "ERROR")

    def _append_log(self, message, tag="INFO"):
        text = f"{datetime.now():%H:%M:%S.%f}"[:-3] + f" [{tag}] {message}"
        self._logs.append((str(tag), text))
        if tag == "RX" and ("STALL_STATUS" in message or "CAL_STATUS" in message):
            self.snapshot_label.setText(message)
        if not self.log_pause.isChecked() and self._matches_log(tag, text):
            self.log_view.appendPlainText(text)

    def _matches_log(self, tag, text):
        selected = self.log_filter.currentText()
        return (not self._joint_log_hidden(tag, text) and
                (selected == "全部" or tag == selected) and
                self.log_search.text().casefold() in text.casefold())

    def _joint_log_hidden(self, tag, text):
        if not self.hide_joint_logs_checkbox.isChecked():
            return False
        prefix, separator, payload = text.partition(f" [{tag}] ")
        if not separator or re.fullmatch(r"\d{2}:\d{2}:\d{2}\.\d{3}", prefix) is None:
            payload = text
        payload = payload.strip()
        if tag == "TX":
            return payload == "#GETJPOS"
        return tag == "RX" and payload.startswith("ok ") and finite_numbers(payload[3:], 7) is not None

    def _hide_joint_logs_changed(self, hidden):
        self.store.update_settings(hide_joint_logs=hidden)
        self._render_logs()

    def _render_logs(self, *args):
        if hasattr(self, "log_view") and not self.log_pause.isChecked():
            self.log_view.setPlainText("\n".join(text for tag, text in self._logs if self._matches_log(tag, text)))
            self.log_view.verticalScrollBar().setValue(self.log_view.verticalScrollBar().maximum())

    def _clear_logs(self):
        self._logs.clear()
        self.log_view.clear()

    def _export_logs(self):
        path, _ = QFileDialog.getSaveFileName(self, "导出终端日志", "Dummy串口日志.txt", "文本日志 (*.txt)")
        if path:
            try:
                Path(path).write_text("\n".join(text for tag, text in self._logs
                                               if not self._joint_log_hidden(tag, text)) + "\n", encoding="utf-8")
                self.controller.log(f"终端日志已导出：{path}", "INFO")
            except OSError as exc:
                self.controller.log(f"日志导出失败：{exc}", "ERROR")

    def apply_theme(self, theme):
        from app.paths import resource_path
        path = resource_path(f"themes/{theme}.qss")
        stylesheet = Path(path).read_text(encoding="utf-8")
        stylesheet = stylesheet.replace("{{ICONS}}", resource_path("icons").as_posix())
        QApplication.instance().setStyleSheet(stylesheet)
        self._schedule_motion_fit()
        self.viewer.set_theme(theme)
        self.rgb_ring.set_theme(theme)

    def _theme_setting(self, index):
        if self._building:
            return
        theme = "dark" if index == 1 else "light"
        self.apply_theme(theme)
        self.store.update_settings(theme=theme)

    def _save_refresh_settings(self):
        if self._building:
            return
        enabled, hz = self.poll_enabled.isChecked(), self.poll_hz.value()
        self.controller.set_polling(enabled, hz)
        self.store.update_settings(auto_poll=enabled, poll_hz=hz)

    def _grid_changed(self, visible):
        for name in ("viewer_grid_button", "grid_check"):
            control = getattr(self, name, None)
            if control is not None and control.isChecked() != visible:
                with QSignalBlocker(control):
                    control.setChecked(visible)
        if hasattr(self.viewer, "set_grid_visible"):
            self.viewer.set_grid_visible(visible)
        elif hasattr(self.viewer, "set_show_grid"):
            self.viewer.set_show_grid(visible)
        if not self._building:
            self.store.update_settings(show_grid=visible)

    def _axes_changed(self, visible):
        for name in ("viewer_axes_button", "axes_check"):
            control = getattr(self, name, None)
            if control is not None and control.isChecked() != visible:
                with QSignalBlocker(control):
                    control.setChecked(visible)
        if hasattr(self.viewer, "set_axes_visible"):
            self.viewer.set_axes_visible(visible)
        elif hasattr(self.viewer, "set_show_axes"):
            self.viewer.set_show_axes(visible)
        if not self._building:
            self.store.update_settings(show_axes=visible)

    def _log_limit_changed(self, limit):
        if self._building:
            return
        self._logs = deque(self._logs, maxlen=limit)
        self.log_view.setMaximumBlockCount(limit)
        self.store.update_settings(log_limit=limit)
        self._render_logs()

    def _reset_layout(self):
        self.resize(1440, 900)
        self.motion_splitter.setSizes((800, 370))
        self.log_dock.hide()
        self.store.update_settings(geometry="", window_state="")

    def showEvent(self, event):
        super().showEvent(event)
        self._sync_rgb_preview_activity()

    def hideEvent(self, event):
        self._suspend_gamepad("窗口隐藏，手柄控制已暂停")
        self._sync_rgb_preview_activity()
        super().hideEvent(event)

    def changeEvent(self, event):
        super().changeEvent(event)
        if event.type() == QEvent.Type.WindowStateChange:
            if self.isMinimized():
                self._suspend_gamepad("窗口最小化，手柄控制已暂停")
            self._sync_rgb_preview_activity()

    def closeEvent(self, event):
        if (not self._closed and not self._gamepad_close_allowed and
                self._gamepad_locked() and self.controller.is_connected):
            event.ignore()
            if not self._gamepad_close_pending:
                self._gamepad_close_pending = True
                self.setEnabled(False)
                self._gamepad_close_timer.start(1000)
                self._suspend_gamepad("关闭窗口，正在等待软停确认")
            return
        if not self._closed:
            self._suspend_gamepad("关闭窗口，手柄输入已停止")
            self._closed = True
            self._gamepad_close_timer.stop()
            QApplication.instance().removeEventFilter(self)
            self.gamepad_reader.close()
            self._rgb_render_timer.stop()
            self.controller.set_rgb_monitoring(False)
            self.log_dock.hide()
            self._cancel_drag()
            self.controller.close()
            self.viewer.cleanup()
            try:
                self.store.update_settings(
                    geometry=bytes(self.saveGeometry().toBase64()).decode("ascii"),
                    window_state=bytes(self.saveState().toBase64()).decode("ascii"),
                )
            except OSError:
                pass
            self.store.close()
        event.accept()
