"""Motion-page construction and presentation only; callbacks stay on MainWindow."""
from PySide6.QtCore import QSize, Qt
from PySide6.QtWidgets import (
    QAbstractSpinBox, QBoxLayout, QCheckBox, QComboBox, QFrame, QGridLayout, QHBoxLayout, QLabel, QLayout,
    QScrollArea, QSizePolicy, QSplitter, QStackedWidget, QTabWidget, QVBoxLayout, QWidget,
)
from app.ui.adapters import FieldAdapter
from app.ui.gamepad_panel import GamepadPanel
from app.ui.shell_widgets import InlinePanel, MotionPowerAdapter, update_shell
from app.ui.widgets import AxisRow, button, hint, number
from app.protocol.constants import DEFAULT_SPEED, JOINT_DEFAULTS, JOINT_RANGES, POSE_DEFAULTS, POSE_RANGES


def build_motion_page(self, view_factory):
    system_toolbar = QWidget()
    system_toolbar.setObjectName("MotionSystemToolbar")
    row = QHBoxLayout(system_toolbar)
    row.setContentsMargins(0, 0, 0, 0)
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
        control.setToolTip(text)
        row.addWidget(control)
        self.motion_system_buttons.append(control)
    self._motion_system_labels = [control.text() for control in self.motion_system_buttons]
    mode_toolbar = QWidget()
    mode_toolbar.setObjectName("MotionModeToolbar")
    row = QHBoxLayout(mode_toolbar)
    row.setContentsMargins(0, 0, 0, 0)
    row.setSpacing(4)
    self.motion_system_buttons[1].setProperty("operationOverlaySafety", True)
    self.mode_combo = self._combo(("1 · SEQ 顺序", "2 · INT 打断", "3 · TRJ 连续", "4 · TUN 扫频"), 1)
    self.mode_combo.setToolTip("TUN 使用固件内置扫频激励；切换前确认各轴运动空间")
    row.addWidget(self.mode_combo)
    self.mode_set_button = button("设置模式", lambda: self.controller._set_cmdmode((1, 2, 3, 4)[self.mode_combo.currentIndex()]))
    row.addWidget(self.mode_set_button)
    self.mode_query_button = button("查询", self.controller._query_command_mode)
    row.addWidget(self.mode_query_button)
    row.addStretch()
    layout = self._page("运动控制", "编辑目标与设备实际位置分别管理；三维预览不会自动发送运动指令。",
                        False, (system_toolbar, mode_toolbar))
    self.motion_page = self.pages.widget(0)
    layout.setContentsMargins(20, 14, 20, 14)
    layout.setSpacing(8)

    self.motion_splitter = QSplitter(Qt.Orientation.Horizontal)
    scene = QFrame()
    scene.setObjectName("Card")
    scene_layout = QVBoxLayout(scene)
    scene_layout.setContentsMargins(10, 10, 10, 8)
    self.viewer_toolbar = QWidget()
    self.viewer_toolbar.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
    self._viewer_toolbar_grid = QGridLayout(self.viewer_toolbar)
    self._viewer_toolbar_grid.setContentsMargins(0, 0, 0, 0)
    self._viewer_toolbar_grid.setSpacing(6)
    self._viewer_toolbar_grid.setSizeConstraint(QLayout.SizeConstraint.SetNoConstraint)
    self.viewer_camera_controls = QWidget()
    camera_row = QHBoxLayout(self.viewer_camera_controls)
    camera_row.setContentsMargins(0, 0, 0, 0)
    camera_row.setSpacing(6)
    self.viewer_display_controls = QWidget()
    display_row = QHBoxLayout(self.viewer_display_controls)
    display_row.setContentsMargins(0, 0, 0, 0)
    display_row.setSpacing(6)
    self._viewer_mode_preference = 1 if self.store.settings.get("viewer_mode") == "actual" else 0
    self.viewer_mode = self._combo(("目标预览", "实际跟随"), self._viewer_mode_preference)
    self.viewer_mode.currentIndexChanged.connect(self._view_mode_changed)
    self.viewer_reset_button = button("复位视角", lambda: self.viewer.reset_camera())
    camera_row.addWidget(self.viewer_reset_button)
    camera_row.addStretch()
    self.viewer_grid_button = button("网格")
    self.viewer_grid_button.setObjectName("ViewerToggle")
    self.viewer_grid_button.setCheckable(True)
    self.viewer_grid_button.setChecked(self.store.settings.get("show_grid", True))
    self.viewer_grid_button.setToolTip("显示或隐藏三维地面网格")
    self.viewer_grid_button.toggled.connect(self._grid_changed)
    display_row.addWidget(self.viewer_grid_button)
    self.viewer_axes_button = button("坐标轴")
    self.viewer_axes_button.setObjectName("ViewerToggle")
    self.viewer_axes_button.setCheckable(True)
    self.viewer_axes_button.setChecked(self.store.settings.get("show_axes", True))
    self.viewer_axes_button.setToolTip("显示或隐藏三维坐标轴")
    self.viewer_axes_button.toggled.connect(self._axes_changed)
    display_row.addWidget(self.viewer_axes_button)
    self.viewer_target_button = button("目标虚体")
    self.viewer_target_button.setObjectName("ViewerToggle")
    self.viewer_target_button.setCheckable(True)
    self.viewer_target_button.setChecked(self.store.settings.get("show_target_ghost", True))
    self.viewer_target_button.setToolTip("目标虚体总开关；有效回读与目标一致时自动隐藏，有差异时显示。仅影响模型显示，不改变目标或发送指令。")
    self.viewer_target_button.toggled.connect(self._target_visibility_changed)
    display_row.addWidget(self.viewer_target_button)
    display_row.addStretch()
    display_row.addWidget(self.viewer_mode)
    self._viewer_toolbar_wrapped = True
    self._viewer_toolbar_grid.addWidget(self.viewer_camera_controls, 0, 0, 1, 2)
    self._viewer_toolbar_grid.addWidget(self.viewer_display_controls, 1, 0, 1, 2)
    self._viewer_toolbar_grid.setColumnStretch(0, 1)
    self.viewer_toolbar.installEventFilter(self)
    scene_layout.addWidget(self.viewer_toolbar)
    self.viewer = view_factory()
    self.viewer.setMinimumSize(320, 260)
    self.viewer.ready.connect(self._viewer_ready)
    self.viewer.status_changed.connect(lambda text: self.scene_status.setText(text))
    scene_layout.addWidget(self.viewer, 1)
    self.scene_status = QLabel("正在加载本地 URDF…")
    self.scene_status.setProperty("role", "muted")
    self.scene_status.setWordWrap(True)
    self.scene_status.setAlignment(Qt.AlignmentFlag.AlignRight)
    scene_layout.addWidget(self.scene_status)
    self.actual_status = hint("实际位置未回读 · 夹爪显示目标开度", "muted")
    self.actual_status.setAlignment(Qt.AlignmentFlag.AlignRight)
    scene_layout.addWidget(self.actual_status)
    self.motion_splitter.addWidget(scene)

    self.motion_controls = QWidget()
    self.motion_controls.setMinimumWidth(395)
    self.motion_controls.setMaximumWidth(545)
    controls_layout = QVBoxLayout(self.motion_controls)
    controls_layout.setSizeConstraint(QLayout.SizeConstraint.SetMinimumSize)
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
    self.cartesian_frame = QComboBox()
    self.cartesian_frame.setObjectName("CartesianFrame")
    self.cartesian_frame.addItem("基座 / 法兰（兼容已有点位）", "base")
    self.cartesian_frame.addItem("世界 / TCP（名义夹爪中心）", "world")
    self._cartesian_frame_value = "base"
    self.cartesian_frame.currentIndexChanged.connect(lambda *_: self._cartesian_frame_changed())
    self._point_tool_snapshot = None
    pose_tab_layout.addWidget(self.cartesian_frame)
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
    for action in ("stop", "disable"):
        self.gamepad_panel.action_buttons[action].setProperty("operationOverlaySafety", True)
    self.gamepad_panel.scroll.setMinimumHeight(140 + 2 * self.gamepad_panel.scroll.frameWidth())
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
    # Expanded status explanations must not squeeze command buttons below
    # their text height on a small display. Keep the column readable and
    # scroll it when the available height is smaller than its minimum.
    self.motion_controls_scroll = QScrollArea()
    self.motion_controls_scroll.setObjectName("MotionControlsScroll")
    self.motion_controls_scroll.setWidgetResizable(True)
    self.motion_controls_scroll.setFrameShape(QFrame.Shape.NoFrame)
    self.motion_controls_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    self.motion_controls_scroll.setMinimumWidth(395)
    self.motion_controls_scroll.setMaximumWidth(545)
    self.motion_controls_scroll.setWidget(self.motion_controls)
    self.motion_controls_scroll.installEventFilter(self)
    self.motion_splitter.addWidget(self.motion_controls_scroll)
    self.motion_splitter.setStretchFactor(0, 3)
    self.motion_splitter.setStretchFactor(1, 1)
    self.motion_splitter.setSizes((760, 430))
    layout.addWidget(self.motion_splitter, 1)
    self._schedule_header_fit()
    self.motion_layout = MotionPageLayout(self)


class MotionPageLayout:
    """The workstation layout, using the original fields and command callbacks."""
    def __init__(self, w):
        self.w = w
        self.compact = None
        self.points_expanded = False
        page_layout = w.motion_page.layout()
        page_layout.setContentsMargins(24, 18, 24, 16)
        page_layout.setSpacing(16)
        w.motion_page.setObjectName("MotionWorkspace")
        self.header, self.slot = w._page_header_slots[w.motion_page]
        self.header.layout().setDirection(QBoxLayout.Direction.LeftToRight)
        self.header.layout().setSpacing(12)
        self.slot.setMinimumWidth(0)
        self.slot.setMaximumWidth(16777215)
        self.slot.setFixedHeight(48)
        self.slot.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        grid = w._motion_header_grid
        while grid.count():
            grid.takeAt(0)
        w._motion_header_subtitle.clear()
        w._motion_header_subtitle.hide()
        w._motion_header_title.setToolTip("")
        w._motion_header_title.setAccessibleDescription("")
        system, modes = w._motion_header_actions
        system.layout().setSpacing(8)
        for control in w.motion_system_buttons:
            control.setProperty("role", "workstation")
        # Hidden originals retain their exact callbacks and gates; each visible
        # action below always dispatches to its own original command button.
        for control in w.motion_system_buttons[:2]:
            system.layout().removeWidget(control)
            control.hide()
        w.motion_enable_button = button("使能")
        w.motion_enable_button.setObjectName("MotionPowerButton")
        w.motion_enable_button.setAccessibleName("使能 START")
        w.motion_enable_button.setIconSize(QSize(18, 18))
        w.motion_enable_button.setProperty("motionAction", "enable")
        w.motion_disable_button = button("失能")
        w.motion_disable_button.setObjectName("MotionDisableButton")
        w.motion_disable_button.setAccessibleName("失能 DISABLE")
        w.motion_disable_button.setIconSize(QSize(18, 18))
        w.motion_disable_button.setProperty("motionAction", "disable")
        w.motion_disable_button.setProperty("operationOverlaySafety", True)
        system.layout().insertWidget(0, w.motion_enable_button)
        system.layout().insertWidget(1, w.motion_disable_button)
        w.motion_power_adapter = MotionPowerAdapter(w)
        w.motion_system_buttons[2].setText("回零")
        w.motion_system_buttons[2].setProperty("motionAction", "home")
        w.motion_system_buttons[3].setText("休息")
        w.motion_system_buttons[3].setProperty("motionAction", "rest")
        calibration = w.motion_system_buttons[4]
        system.layout().removeWidget(calibration)
        calibration.hide()
        w.motion_calibration_button = button("校准")
        w.motion_calibration_button.setObjectName("MotionCalibrationButton")
        w.motion_calibration_button.setProperty("motionAction", "calibrate")
        w.motion_calibration_button.clicked.connect(calibration.click)
        system.layout().addWidget(w.motion_calibration_button)
        calibration.installEventFilter(w.motion_power_adapter)
        w.motion_mode_button = button("模式：未确认 ▾")
        w.motion_mode_button.setObjectName("MotionModeButton")
        w.motion_mode_button.setAccessibleName("切换运动模式")
        system.layout().addWidget(w.motion_mode_button)
        grid.addWidget(w._motion_header_title, 0, 0)
        grid.addWidget(system, 0, 1)
        grid.setHorizontalSpacing(14)
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 0)
        grid.setColumnStretch(2, 1)

        controls = w.motion_controls.layout()
        old_input = controls.takeAt(0).layout()
        old_input.removeWidget(w.input_mode)
        while old_input.count():
            item = old_input.takeAt(0)
            if item.widget():
                item.widget().hide()
        controls.removeWidget(w.quick_points_panel)
        controls.takeAt(controls.count() - 1)  # old trailing stretch
        controls.setContentsMargins(0, 0, 0, 0)
        controls.setSpacing(0)
        controls.setAlignment(Qt.AlignmentFlag.AlignTop)
        w.motion_controls.setObjectName("TargetEditorCard")
        w.motion_controls.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        editor_header = QWidget()
        editor_header.setObjectName("TargetEditorHeader")
        row = QHBoxLayout(editor_header)
        row.setContentsMargins(16, 4, 16, 4)
        title = QLabel("目标编辑")
        title.setObjectName("TargetEditorTitle")
        row.addWidget(title, 1)
        w.input_mode.setFixedWidth(108)
        w.input_mode.setItemText(0, "界面控制")
        w.input_mode.setItemText(1, "手柄控制")
        row.addWidget(w.input_mode)
        controls.insertWidget(0, editor_header)
        self.editor_header = editor_header
        # Retain the original setting callback and its enablement adapter;
        # the visible menu dispatches one choice without an extra query/apply.
        self.mode_preference_host = QWidget(w.motion_page)
        self.mode_preference_host.hide()
        modes.setParent(self.mode_preference_host)
        w.mode_status.setParent(self.mode_preference_host)
        w.mode_panel = InlinePanel(w, w.motion_mode_button, "运动模式", w.mode_set_button, compact=True)
        w.mode_panel.setObjectName("MotionModeMenu")
        w.mode_choice_buttons = []
        self.reported_mode = None
        for index in range(w.mode_combo.count()):
            choice = button(w.mode_combo.itemText(index), lambda index=index: self.select_mode(index))
            choice.setObjectName("MotionModeItem")
            choice.setFixedHeight(36)
            w.mode_panel.box.addWidget(choice)
            w.mode_choice_buttons.append(choice)
        w.motion_mode_button.clicked.connect(self.refresh_mode_choices)
        w._shell_panels.append(w.mode_panel)

        scene = w.motion_splitter.widget(0)
        self.scene_column = QWidget()
        column = QVBoxLayout(self.scene_column)
        column.setContentsMargins(0, 0, 0, 0)
        column.setSpacing(14)
        column.addWidget(scene, 1)
        controls.addWidget(w.quick_points_panel)
        w.motion_splitter.insertWidget(0, self.scene_column)
        w.motion_splitter.setChildrenCollapsible(False)
        w.motion_splitter.setHandleWidth(18)
        scene.layout().setSpacing(0)
        scene.setProperty("motionScene", True)
        scene.layout().setContentsMargins(1, 1, 1, 0)
        w.viewer_toolbar.setObjectName("SceneToolbar")
        w.viewer_toolbar.setFixedHeight(44)
        w._viewer_toolbar_grid.setContentsMargins(10, 3, 10, 3)
        w._viewer_toolbar_grid.setSpacing(2)
        w.viewer_reset_button.setObjectName("CameraButton")
        w.viewer_reset_button.setText("")
        w.viewer_reset_button.setFixedSize(40, 38)
        w.viewer_reset_button.setIconSize(QSize(20, 20))
        w.viewer_reset_button.setToolTip("恢复画面")
        w.viewer_reset_button.setAccessibleName("恢复画面")
        for control, caption in ((w.viewer_grid_button, "网格"),
                                 (w.viewer_axes_button, "坐标轴"),
                                 (w.viewer_target_button, "显示虚体")):
            control.setText(caption)
            control.setFixedHeight(38)
            control.setIconSize(QSize(18, 18))
            control.setAccessibleName(caption)
        # Retain the existing preference adapter without a duplicate visible
        # selector. The dedicated target/axes/grid toggles remain on the toolbar.
        w.viewer_display_controls.layout().removeWidget(w.viewer_mode)
        self.viewer_preference_host = QWidget(w.motion_page)
        self.viewer_preference_host.hide()
        w.viewer_mode.setParent(self.viewer_preference_host)
        for label in (w.scene_status, w.actual_status):
            label.setWordWrap(False)
            label.setFixedHeight(20)
            label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
            label.setObjectName("MotionSceneCaption")
            scene.layout().removeWidget(label)
            label.setParent(self.viewer_preference_host)
            label.hide()
        # Captions and rounded canvas edges are composited inside WebEngine.
        # Keep the Qt labels as internal status adapters, away from its surface.
        scene.layout().removeWidget(w.viewer)
        self.scene_viewport = QWidget()
        viewport = QGridLayout(self.scene_viewport)
        viewport.setContentsMargins(0, 0, 0, 0)
        viewport.setSpacing(0)
        viewport.addWidget(w.viewer, 0, 0)
        scene.layout().addWidget(self.scene_viewport, 1)
        w.viewer.status_changed.connect(w.scene_status.setToolTip)
        w.viewer.setMinimumSize(300, 180)
        self.axis_headers = []
        for axis in w.joint_rows + w.pose_rows:
            axis.set_motion_style()
        for area in w._motion_axis_scrolls:
            layout = area.widget().layout()
            layout.setSpacing(0)
            layout.setContentsMargins(0, 0, 0, 5)
            original_caption = layout.itemAt(0).widget()
            if isinstance(original_caption, QLabel):
                layout.takeAt(0)
                original_caption.hide()
            caption = QWidget()
            caption.setObjectName("MotionAxisColumns")
            row = QHBoxLayout(caption)
            row.setContentsMargins(0, 0, 0, 0)
            row.setSpacing(8)
            for text, width in (("轴", 30), ("实际回读", 59), ("", 0), ("编辑目标", 119)):
                label = QLabel(text)
                label.setProperty("role", "muted")
                if width:
                    label.setFixedWidth(width)
                    label.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
                                       if text != "轴" else Qt.AlignmentFlag.AlignVCenter)
                    row.addWidget(label)
                else:
                    row.addWidget(label, 1)
            layout.insertWidget(0, caption)
            self.axis_headers.append(caption)
            area.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        for index in range(w.motion_tabs.count()):
            layout = w.motion_tabs.widget(index).layout()
            layout.setContentsMargins(14, 4, 14, 4)
            layout.setSpacing(5)
            # Natural content sits above the action rows; no spare stretch below.
            layout.takeAt(layout.count() - 1)
        for control in (w.movej_read_button, w.movel_read_button):
            control.setText("同步实际")
        for control in (w.j_speed, w.l_speed):
            control.setMinimumWidth(88)
            control.setFixedWidth(88)
            control.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
            control.setSuffix(" / 100")
        for index, speed in enumerate((w.j_speed, w.l_speed)):
            layout = w.motion_tabs.widget(index).layout()
            speed_row = layout.itemAt(layout.count() - 2).layout()
            speed_row.insertStretch(speed_row.indexOf(speed) + 1)
            actions = layout.itemAt(layout.count() - 1).layout()
            actions.setStretch(0, 2)
            actions.setStretch(1, 1)
        w.motion_controls.setMinimumWidth(368)
        w.motion_controls.setMaximumWidth(500)
        w.motion_controls_scroll.setMinimumWidth(368)
        w.motion_controls_scroll.setMaximumWidth(500)
        self._gripper()
        self._points()
        w.input_mode.currentIndexChanged.connect(lambda _: (self._show_points(), update_shell(w)))
        w.motion_tabs.currentChanged.connect(lambda _: w._schedule_motion_fit())
        w.motion_splitter.setSizes((760, 430))

    def _gripper(self):
        w = self.w
        group = w.motion_gripper_group
        group.setTitle("")
        group.setProperty("workstation", True)
        layout = group.layout()
        layout.removeWidget(w.motion_hand_row)
        actions = layout.takeAt(0).layout()
        current = layout.takeAt(0).layout()
        controls = [actions.itemAt(i).widget() for i in range(actions.count())]
        for control in controls:
            actions.removeWidget(control)
        w.motion_hand_row.set_motion_style()
        w.motion_hand_row.title.hide()
        w.motion_hand_row.readback.hide()
        w.motion_hand_spin.setMinimumWidth(72)
        w.motion_hand_spin.setFixedWidth(72)
        for control in controls[1:]:
            control.setProperty("role", "workstation")
            control.setFixedWidth(74 if control is controls[1] else 48)
            w.motion_hand_row.layout().addWidget(control)
        controls[1].setObjectName("GripperSendPosition")
        header = QWidget()
        row = QHBoxLayout(header)
        row.setContentsMargins(0, 0, 0, 0)
        title = QLabel("夹爪  目标开度")
        title.setObjectName("GripperTitle")
        row.addWidget(title, 1)
        w.motion_hand_drag.setText("拖动发送")
        row.addWidget(w.motion_hand_drag)
        layout.addWidget(header)
        layout.addWidget(w.motion_hand_row)
        layout.addLayout(current)
        layout.setContentsMargins(14, 5, 14, 7)
        layout.setSpacing(4)

    def _points(self):
        w = self.w
        w.quick_points_panel.setObjectName("QuickPointsPanel")
        # Spare column height belongs below this content, not between its
        # header and fixed-height point grid when the main window grows.
        w.quick_points_panel.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        w.quick_points_panel.layout().setAlignment(Qt.AlignmentFlag.AlignTop)
        w.quick_points_panel.layout().setContentsMargins(14, 10, 14, 12)
        w.quick_points_panel.layout().setSpacing(8)
        header = w.quick_points_panel.layout().itemAt(0).layout()
        header.itemAt(0).widget().setText("点位快捷")
        self.points_body = w.quick_points_scroll.widget()
        self.points_grid = self.points_body.layout()
        while self.points_grid.count():
            self.points_grid.takeAt(0)
        self.points_body.setMinimumHeight(202)
        self.points_body.setMaximumHeight(16777215)
        for index, control in enumerate(w.quick_point_buttons):
            control.set_motion_style()
            control.setCheckable(True)
            control.clicked.connect(lambda _checked=False: self.sync_point_selection())
            self.points_grid.addWidget(control, index // 2, index % 2)
        for column in range(4):
            self.points_grid.setColumnStretch(column, 1 if column < 2 else 0)
        w.quick_points_scroll.setMinimumHeight(0)
        w.quick_point_save.setText("保存")
        w.quick_point_save.setAccessibleName("保存当前点位")
        w.quick_point_save.setObjectName("PointTextAction")
        w.quick_point_manage.setObjectName("PointTextAction")
        w.quick_points_toggle = button("展开 8 个", self.toggle_points)
        w.quick_points_toggle.setCheckable(True)
        w.quick_points_toggle.setObjectName("QuickPointsToggle")
        header.addWidget(w.quick_points_toggle)

    def bind_points(self):
        self.w.point_table.itemSelectionChanged.connect(self.sync_point_selection)
        self.sync_point_selection()

    def sync_point_selection(self):
        selected = self.w.point_table.currentRow()
        for index, control in enumerate(self.w.quick_point_buttons):
            control.setChecked(index == selected and index < len(self.w.points))

    def toggle_points(self):
        self.points_expanded = self.w.quick_points_toggle.isChecked()
        self._show_points()

    def update_mode(self, mode, connected):
        """Present confirmed controller state, independent of the editor choice."""
        labels = {1: "SEQ 顺序", 2: "INT 打断", 3: "TRJ 连续", 4: "TUN 扫频",
                  5: "TRQ", 6: "SRV", 7: "7 诊断"}
        caption = labels.get(mode, "未确认") if connected else "未确认"
        self.w.motion_mode_button.setText(f"模式：{caption} ▾")
        status = self.w.mode_status.text() if connected else "未连接设备，当前模式未确认"
        description = status + "\n点击模式立即发送切换请求，以主控回执为准；点击菜单外取消。"
        self.w.motion_mode_button.setToolTip(description)
        self.w.motion_mode_button.setAccessibleDescription(description)
        self.reported_mode = mode if connected else None
        self.refresh_mode_choices()

    def refresh_mode_choices(self):
        enabled = self.w.mode_set_button.isEnabled() and self.w.mode_combo.isEnabled()
        for index, choice in enumerate(self.w.mode_choice_buttons):
            choice.setEnabled(enabled)
            current = self.reported_mode == index + 1
            if choice.property("currentMode") != current:
                choice.setProperty("currentMode", current)
                choice.style().unpolish(choice)
                choice.style().polish(choice)
                choice.update()
            choice.setAccessibleDescription("当前已确认模式" if current else "点击立即发送切换请求")

    def select_mode(self, index):
        w = self.w
        w.mode_panel.hide()
        if not w.mode_set_button.isEnabled() or not w.mode_combo.isEnabled():
            return
        w.mode_combo.setCurrentIndex(index)
        w.mode_set_button.click()

    def _show_points(self):
        expanded = not self.compact or self.points_expanded
        ui_input = self.w.input_mode.currentIndex() == 0
        self.w.quick_points_scroll.setVisible(expanded and ui_input)
        self.w.quick_points_toggle.setVisible(bool(self.compact) and ui_input)
        self.w.quick_points_toggle.setText("收起" if expanded else "展开 8 个")
        self.w._schedule_motion_fit()

    def fit(self):
        w = self.w
        compact = w.width() < 1280 or w.height() < 780
        if compact != self.compact:
            self.compact = compact
            w.motion_page.setProperty("compact", compact)
            w.motion_page.layout().setContentsMargins(*(16, 8, 16, 8) if compact else (24, 18, 24, 16))
            w.motion_page.layout().setSpacing(8 if compact else 16)
            self.header.layout().setSpacing(12)
            self.header.setFixedHeight(48)
            w._motion_header_left.setFixedHeight(48)
            self.editor_header.setFixedHeight(30 if compact else 46)
            w.motion_gripper_group.setFixedHeight(112 if compact else 128)
            w._schedule_viewer_toolbar_fit()
            for caption in self.axis_headers:
                caption.setFixedHeight(16 if compact else 24)
            for control in w.quick_point_buttons:
                control.setFixedHeight(46)
            self._show_points()
        available = w.motion_controls_scroll.viewport().height()
        # Keep the seven-axis editor readable; shorter windows scroll the point
        # grid itself, rather than spending four rows of motion-editor space.
        w.quick_points_scroll.setFixedHeight(98 if compact else max(98, min(204, available - 574)))
        # Preserve a complete last axis row when tab/font size hints settle.
        # Spare height still goes to the axes; quick points keep their own scroll.
        for area, caption, axes in zip(w._motion_axis_scrolls, self.axis_headers,
                                       (w.joint_rows, w.pose_rows)):
            area.setMinimumHeight(caption.height() + len(axes) * 25 + 5 + 2 * area.frameWidth())
        w.motion_controls.layout().activate()
        w.motion_input_stack.layout().activate()
        w.motion_tabs.currentWidget().layout().activate()
        index = w.motion_tabs.currentIndex()
        area = w._motion_axis_scrolls[index]
        axis_budget = area.viewport().height() - self.axis_headers[index].height() - 5
        count = len(w.joint_rows if index == 0 else w.pose_rows)
        row_height = max(25, min(38 if compact else 58, axis_budget // count))
        for axis in w.joint_rows + w.pose_rows:
            axis.setFixedHeight(row_height)
            axis.spin.setFixedHeight(min(30, row_height - 4) if compact else min(38, row_height - 6))
        w.motion_hand_row.setFixedHeight(32 if compact else 34)
        w.motion_hand_spin.setFixedHeight(26 if compact else 32)
        w.motion_hand_current.setFixedHeight(28 if compact else 32)
        for control in w.motion_gripper_group.findChildren(type(w.stop_button)):
            control.setFixedHeight(30 if compact else 34)
        for control in (*w.motion_system_buttons[2:4], w.motion_enable_button,
                        w.motion_disable_button, w.motion_calibration_button):
            control.setFixedHeight(30 if compact else 40)
            control.setFixedWidth(110 if compact else 118)
        w.motion_mode_button.setFixedSize(148 if compact else 164, 30 if compact else 40)
        for control in (w.movej_send_button, w.movej_read_button, w.movel_send_button, w.movel_read_button):
            control.setFixedHeight(34 if compact else 40)
        for control in (w.j_speed, w.l_speed, w.cartesian_frame, w.input_mode):
            control.setFixedHeight(26 if compact else 32)
        w.motion_power_adapter.refresh()
