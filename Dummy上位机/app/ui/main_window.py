"""A complete Qt control console, with separate actual and edited state."""
from __future__ import annotations

from collections import deque
from datetime import datetime
from pathlib import Path
import copy
import time

from PySide6.QtCore import QByteArray, QEvent, Qt, QTimer
from PySide6.QtGui import QDesktopServices
from PySide6.QtCore import QUrl
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDockWidget, QFileDialog, QFormLayout,
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit,
    QListWidget, QMainWindow, QMessageBox, QPlainTextEdit, QPushButton,
    QScrollArea, QSlider, QSpinBox, QSplitter, QStackedWidget, QTableWidget,
    QTableWidgetItem, QTabWidget, QVBoxLayout, QWidget,
)

from app.ui.adapters import FieldAdapter, TableAdapter
from app.ui.widgets import AxisRow, ColorEditor, button, hint, number
from app.visualization.robot_view import RobotView
from app.protocol.constants import (
    CAL_PARAMETERS, DEFAULT_SPEED, JOINT_DEFAULTS, JOINT_RANGES,
    POSE_DEFAULTS, POSE_RANGES, RGB_AUTO_MODES, RGB_COLOR_SOURCES, RGB_MODES,
    RGB_STATES, STALL_PARAMETERS,
)


class MainWindow(QMainWindow):
    def __init__(self, controller, store):
        super().__init__()
        self.controller = controller
        self.store = store
        self.fields = {}
        self._closed = False
        self._building = True
        self._logs = deque(maxlen=int(store.settings.get("log_limit", 5000)))
        self._last_drag = {"j": 0., "l": 0., "hand": 0.}
        self.points = copy.deepcopy(store.points)
        self._point_editing = False
        self.setWindowTitle("Dummy Robot · 机械臂控制台")
        self.resize(1440, 900)
        self.setMinimumSize(1120, 690)
        self._create_shell()
        self._build_motion_page()
        self._build_teaching_page()
        self._build_motor_page()
        self._build_protection_page()
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
        self.connection_combo.currentIndexChanged.connect(lambda *_: self._update_state({}))
        self._restore_points()
        self._building = False
        self.apply_theme(store.settings.get("theme", "light"))
        geometry = store.settings.get("geometry", "")
        if geometry:
            self.restoreGeometry(QByteArray.fromBase64(geometry.encode("ascii")))
        window_state = store.settings.get("window_state", "")
        if window_state:
            self.restoreState(QByteArray.fromBase64(window_state.encode("ascii")))
        self.controller.set_polling(store.settings.get("auto_poll", True), store.settings.get("poll_hz", 5))
        self.controller.refresh_ports()
        self._update_state({})
        self._preview_changed()
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
        stop = button("■  软停 STOP", lambda: self.controller.send_cmd("!STOP"), "danger")
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
        self.navigation.addItems(("运动控制", "点位示教", "电机参数", "保护与校准", "夹爪设置", "RGB 灯效", "软件设置"))
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
        self.navigation.setCurrentRow(0)
        self.statusBar().showMessage("离线预览可用；连接设备后可读取实际位置。")

    def _page(self, title, subtitle, scroll=True):
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(20, 18, 20, 14)
        layout.setSpacing(12)
        heading = QLabel(title)
        heading.setObjectName("PageTitle")
        layout.addWidget(heading)
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
            self.pages.addWidget(page)
            return content_layout
        self.pages.addWidget(page)
        return layout

    @staticmethod
    def _group(title):
        group = QGroupBox(title)
        layout = QVBoxLayout(group)
        layout.setSpacing(9)
        return group, layout

    def _build_motion_page(self):
        layout = self._page("运动控制", "编辑目标与设备实际位置分别管理；三维预览不会自动发送运动指令。", False)
        row = QHBoxLayout()
        for text, callback, role in (
            ("启动 START", self.controller._cmd_start, "success"),
            ("失能 DISABLE", self.controller._cmd_disable, None),
            ("回零 HOME", lambda: self.controller.send_cmd("!HOME"), None),
            ("休息 RESET", lambda: self.controller.send_cmd("!RESET"), None),
            ("置零 / 校准", self.controller._cmd_calibration, None),
        ):
            row.addWidget(button(text, callback, role))
        row.addStretch()
        self.mode_combo = self._combo(("1 · SEQ 顺序", "2 · INT 打断", "3 · TRJ 连续", "4 · TUN 扫频"), 1)
        self.mode_combo.setToolTip("TUN 使用固件内置扫频激励；切换前确认各轴运动空间")
        row.addWidget(self.mode_combo)
        row.addWidget(button("设置模式", lambda: self.controller._set_cmdmode(self.mode_combo.currentIndex() + 1)))
        row.addWidget(button("查询", self.controller._query_command_mode))
        layout.addLayout(row)

        self.motion_splitter = QSplitter(Qt.Orientation.Horizontal)
        scene = QFrame()
        scene.setObjectName("Card")
        scene_layout = QVBoxLayout(scene)
        scene_layout.setContentsMargins(10, 10, 10, 8)
        toolbar = QHBoxLayout()
        self.viewer_mode = self._combo(("目标预览", "实际跟随"), 1 if self.store.settings.get("viewer_mode") == "actual" else 0)
        self.viewer_mode.currentIndexChanged.connect(self._view_mode_changed)
        toolbar.addWidget(self.viewer_mode)
        for title, view in (("等轴", "isometric"), ("正面", "front"), ("俯视", "top")):
            toolbar.addWidget(button(title, lambda value=view: self.viewer.set_camera(value)))
        toolbar.addWidget(button("复位视角", lambda: self.viewer.reset_camera()))
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

        self.motion_tabs = QTabWidget()
        self.motion_tabs.setMinimumWidth(330)
        self.motion_tabs.setMaximumWidth(520)
        joint_page = QWidget()
        joint_layout = QVBoxLayout(joint_page)
        joint_layout.setContentsMargins(12, 11, 12, 10)
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
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("速度档位"))
        self.j_speed = number(DEFAULT_SPEED, 1, 100, 0)
        self._bind("ent_j_speed", self.j_speed)
        speed_row.addWidget(self.j_speed)
        self.j_drag = QCheckBox("拖动发送")
        self.j_drag.toggled.connect(lambda *_: self.controller._warn_drag_mode())
        self._bind("movej_drag_enable", self.j_drag)
        speed_row.addWidget(self.j_drag)
        joint_layout.addLayout(speed_row)
        joint_layout.addWidget(button("发送 MoveJ", self.controller.send_movej, "primary"))
        joint_layout.addWidget(button("读取实际位置 → 同步输入", self.controller.read_and_sync_from_robot))
        gripper_row = QHBoxLayout()
        gripper_row.addWidget(QLabel("夹爪目标"))
        self.quick_hand_spin = number(100, 0, 100, 0)
        self.quick_hand_spin.setSuffix(" %")
        self.quick_hand_spin.valueChanged.connect(self._quick_hand_changed)
        gripper_row.addWidget(self.quick_hand_spin, 1)
        gripper_row.addWidget(button("发送夹爪", self.controller._cmd_hand_set_pos))
        joint_layout.addLayout(gripper_row)
        joint_layout.addWidget(hint("速度 100 = 30 r/s；地轨与六轴同步到达。"))
        joint_layout.addStretch()
        joint_scroll = QScrollArea()
        joint_scroll.setWidgetResizable(True)
        joint_scroll.setWidget(joint_page)
        self.motion_tabs.addTab(joint_scroll, "关节 / 地轨")

        pose_page = QWidget()
        pose_layout = QVBoxLayout(pose_page)
        pose_layout.setContentsMargins(12, 11, 12, 10)
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
        line = QHBoxLayout()
        line.addWidget(QLabel("速度档位"))
        self.l_speed = number(DEFAULT_SPEED, 1, 100, 0)
        self._bind("ent_l_speed", self.l_speed)
        line.addWidget(self.l_speed)
        l_drag = QCheckBox("拖动发送")
        self._bind("movel_drag_enable", l_drag)
        l_drag.toggled.connect(lambda *_: self.controller._warn_drag_mode())
        line.addWidget(l_drag)
        pose_layout.addLayout(line)
        pose_layout.addWidget(button("发送 MoveL", self.controller.send_movel, "primary"))
        pose_layout.addWidget(button("读取实际位姿 → 同步输入", self.controller.read_pose_from_robot))
        pose_layout.addWidget(hint("三维标记表示笛卡尔目标；完整关节姿态以设备回读为准。"))
        pose_layout.addStretch()
        pose_scroll = QScrollArea()
        pose_scroll.setWidgetResizable(True)
        pose_scroll.setWidget(pose_page)
        self.motion_tabs.addTab(pose_scroll, "MoveL")
        self.motion_splitter.addWidget(self.motion_tabs)
        self.motion_splitter.setStretchFactor(0, 3)
        self.motion_splitter.setStretchFactor(1, 1)
        self.motion_splitter.setSizes((800, 370))
        layout.addWidget(self.motion_splitter, 1)

    def _focus_event(self, original, joint):
        def focus(event):
            self.viewer.focus_joint(joint)
            original(event)
        return focus

    def _build_teaching_page(self):
        layout = self._page("点位示教", "保存目标关节与地轨坐标，选择点位预览；顺序执行等待匹配运动编号的完成回执。", False)
        row = QHBoxLayout()
        self.point_name = QLineEdit()
        self.point_name.setPlaceholderText("点位名称，例如：取料起始")
        self.point_name.setMaxLength(120)
        row.addWidget(self.point_name, 1)
        row.addWidget(button("保存当前目标", self._save_point, "primary"))
        row.addWidget(button("用当前目标更新选中", self._replace_point))
        row.addWidget(button("导入 JSON", self._import_points))
        row.addWidget(button("导出 JSON", self._export_points))
        layout.addLayout(row)
        self.point_table = QTableWidget(0, 10)
        self.point_table.setObjectName("PointTable")
        self.point_table.setHorizontalHeaderLabels(("名称", "J1 °", "J2 °", "J3 °", "J4 °", "J5 °", "J6 °", "地轨 mm", "夹爪 %", "速度"))
        self.point_table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.point_table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.point_table.setAlternatingRowColors(True)
        self.point_table.verticalHeader().setVisible(True)
        header = self.point_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.Interactive)
        self.point_table.setColumnWidth(0, 175)
        self.point_table.itemSelectionChanged.connect(self._select_point)
        self.point_table.itemChanged.connect(self._point_name_changed)
        self.fields["_pos_listbox"] = TableAdapter(self.point_table)
        layout.addWidget(self.point_table, 1)
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
        bottom.addWidget(button("发送选中", self._send_selected_point, "primary"))
        bottom.addWidget(button("顺序执行全部", self.controller.send_all_positions, "success"))
        bottom.addWidget(button("取消后续点位", lambda: self.controller._stop_sequence("用户取消后续点位；当前运动需软停")))
        layout.addLayout(bottom)
        self.sequence_status = QLabel("未运行")
        layout.addWidget(self.sequence_status)
        layout.addWidget(hint("选中只载入目标输入。发送选中使用该点位速度；顺序执行采用上方统一速度。夹爪开度仅作预览记录，需单独发送夹爪指令。"))

    def _build_motor_page(self):
        layout = self._page("电机参数", "本页参数查询、写入与电机维护仅支持 USB。节点 0 为 ALL 查询；运行时应用与 EEPROM 保存明确区分。")
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
        layout = self._page("保护与校准", "参数先读取实际节点；应用仅改变 RAM，保存需要失能。运行与上次快照可在保护锁定时查询。", False)
        tabs = QTabWidget()
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
        row.addWidget(self.hand_current_spin)
        row.addWidget(button("设置电流", self.controller._cmd_hand_set_i))
        row.addWidget(button("张开 · 电流模式", self.controller._cmd_hand_open, "success"))
        row.addWidget(button("闭合 · 电流模式", self.controller._cmd_hand_close))
        row.addStretch()
        box.addLayout(row)
        box.addWidget(hint("电流开合持续到接触或后续指令；仅修改运行时电流，不保存 EEPROM。"))
        layout.addWidget(group)
        group, box = self._group("目标开度 · 0 闭合 / 100 张开")
        self.hand_row = AxisRow("开度", (0, 100), 100, "%")
        self.hand_row.spin.setDecimals(0)
        self.hand_row.changed.connect(lambda _: self._target_edited("hand"))
        self.hand_row.changed.connect(self._quick_hand_refresh)
        self._bind("hand_pos", self.hand_row.spin, lambda value: round(value))
        self._bind("lbl_hand_pos", QLabel("100"))
        box.addWidget(self.hand_row)
        row = QHBoxLayout()
        hand_drag = QCheckBox("拖动发送")
        self._bind("hand_pos_drag_enable", hand_drag)
        hand_drag.toggled.connect(lambda *_: self.controller._warn_drag_mode())
        row.addWidget(hand_drag)
        row.addStretch()
        row.addWidget(button("设置夹爪位置", self.controller._cmd_hand_set_pos, "primary"))
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
        layout = self._page("RGB 灯效", "24 灯珠圆环 · 预设、动画与设备状态联动。编辑不自动发送；应用改 RAM，保存整组后掉电保持。", False)
        self.rgb_status = hint("尚未回读 RGB 配置。", "muted")
        self._bind("_rgb_status_var", self.rgb_status)
        layout.addWidget(self.rgb_status)
        row = QHBoxLayout()
        for text, command in (("渐开", "!RGB_ON"), ("渐关", "!RGB_OFF"),
                              ("启用自动联动", "!RGB_AUTO 1"), ("关闭自动联动", "!RGB_AUTO 0"),
                              ("查询全部", "#RGB_GET"), ("查询状态", "#RGB_STATUS")):
            row.addWidget(button(text, lambda cmd=command: self.controller.send_cmd(cmd)))
        row.addStretch()
        layout.addLayout(row)
        tabs = QTabWidget()

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
        group, content = self._group("手动灯效预览")
        grid = QGridLayout()
        for index, title in enumerate(RGB_MODES[:10]):
            grid.addWidget(button(title, lambda mode=index: self.controller.send_cmd(f"!RGB_MODE {mode}")), index // 5, index % 5)
        content.addLayout(grid)
        box.addWidget(group)
        group, content = self._group("单色预设")
        row = QHBoxLayout()
        color_index = self._combo(range(3))
        self._bind("_rgb_color_idx", color_index)
        color_index.currentIndexChanged.connect(lambda *_: self.controller._rgb_fill_color() if not self._building else None)
        row.addWidget(QLabel("预设"))
        row.addWidget(color_index)
        color = ColorEditor((0, 0, 255))
        self.fields["_rgb_color_vars"] = [FieldAdapter(spin, str) for spin in color.channels]
        row.addWidget(color, 1)
        row.addWidget(button("应用颜色", self.controller._rgb_apply_color))
        content.addLayout(row)
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
        row.addWidget(button("单色常亮预览", lambda: self.controller.send_cmd(f"!RGB_MODE {10 + auto_index.currentIndex()}")))
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
        row.addWidget(button("手动预览", lambda: self.controller.send_cmd(f"!RGB_MODE {effect_mode.currentIndex()+3}")))
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
        layout.addWidget(tabs, 1)
        row = QHBoxLayout()
        row.addWidget(hint("未保存的配置掉电丢失；保存需要已确认失能。"), 1)
        row.addWidget(button("恢复默认（临时）", lambda: self.controller.send_cmd("!RGB_RESET")))
        row.addWidget(button("保存全部 RGB 配置", self.controller._rgb_save, "primary"))
        layout.addLayout(row)

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
        self.log_dock.setAllowedAreas(Qt.DockWidgetArea.BottomDockWidgetArea)
        self.log_dock.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetClosable)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(10, 6, 10, 8)
        layout.setSpacing(5)
        toolbar = QHBoxLayout()
        self.log_filter = self._combo(("全部", "TX", "RX", "INFO", "WARN", "ERROR"))
        self.log_filter.setFixedWidth(88)
        self.log_filter.currentIndexChanged.connect(self._render_logs)
        toolbar.addWidget(self.log_filter)
        self.log_search = QLineEdit()
        self.log_search.setPlaceholderText("搜索日志…")
        self.log_search.textChanged.connect(self._render_logs)
        toolbar.addWidget(self.log_search, 1)
        self.log_pause = QCheckBox("暂停刷新")
        self.log_pause.toggled.connect(lambda paused: self._render_logs() if not paused else None)
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
        row = QHBoxLayout()
        self.custom_command = QLineEdit()
        self.custom_command.setObjectName("CustomCommand")
        self.custom_command.setPlaceholderText("自定义指令 · Enter 发送 · ↑ / ↓ 历史记录")
        self.custom_command.returnPressed.connect(self.controller._send_with_history)
        self.custom_command.installEventFilter(self)
        self._bind("ent_custom_cmd", self.custom_command)
        row.addWidget(self.custom_command, 1)
        row.addWidget(button("发送", self.controller._send_with_history, "primary"))
        row.addWidget(button("运动状态", lambda: self.controller.send_cmd("#MOTION_STATUS")))
        layout.addLayout(row)
        self.log_dock.setWidget(content)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.log_dock)
        self.resizeDocks([self.log_dock], [190], Qt.Orientation.Vertical)
        self.log_dock.hide()

    def eventFilter(self, watched, event):
        if watched is getattr(self, "custom_command", None) and event.type() == QEvent.Type.KeyPress:
            if event.key() == Qt.Key.Key_Up:
                self.controller._history_up(None)
                return True
            if event.key() == Qt.Key.Key_Down:
                self.controller._history_down(None)
                return True
        return super().eventFilter(watched, event)

    def _toggle_log(self):
        self.log_dock.setVisible(not self.log_dock.isVisible())

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
        connected = state.get("connected", self.controller.is_connected)
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
        if stale:
            self.actual_status.setText("实际位置未回读 / 已过期 · 夹爪显示目标开度")

    def _actual_pose(self, values):
        self.viewer.set_actual_pose(values, None)
        self.viewer.set_actual_known(True)
        self.viewer.set_stale(False)
        for index, (row, value) in enumerate(zip(self.joint_rows, values)):
            row.readback.setText(f"{value:.2f}")
            row.readback.setToolTip(f"J{index+1} 实际回读：{value:.2f} {'°' if index < 6 else 'mm'}")
        self.actual_status.setText(f"实际七轴已更新 · {datetime.now():%H:%M:%S} · 夹爪无实际回读")

    def _fields_updated(self):
        self._preview_changed()

    def _viewer_ready(self):
        self._preview_changed()
        self._view_mode_changed(self.viewer_mode.currentIndex())
        self._grid_changed(self.grid_check.isChecked())
        self._axes_changed(self.axes_check.isChecked())

    def _preview_changed(self):
        if not hasattr(self, "hand_row"):
            return
        self.viewer.set_target_pose([spin.value() for spin in self.joint_spins], self.hand_row.spin.value())
        self.viewer.set_cartesian_target([spin.value() for spin in self.pose_spins])

    def _target_edited(self, kind):
        self._preview_changed()
        if self._building or getattr(self.controller, "_suppress_drag_send", False):
            return
        flag = {"j": "movej_drag_enable", "l": "movel_drag_enable", "hand": "hand_pos_drag_enable"}[kind]
        if not self.fields[flag].get():
            return
        now = time.monotonic()
        if now - self._last_drag[kind] < .1:
            return
        self._last_drag[kind] = now
        callback = {"j": self.controller.send_movej, "l": self.controller.send_movel, "hand": self.controller._cmd_hand_set_pos}[kind]
        self.controller._schedule_motion_callback(callback)

    def _quick_hand_changed(self, value):
        if hasattr(self, "hand_row"):
            self.hand_row.spin.setValue(value)

    def _quick_hand_refresh(self, value):
        self.quick_hand_spin.blockSignals(True)
        self.quick_hand_spin.setValue(value)
        self.quick_hand_spin.blockSignals(False)

    def _view_mode_changed(self, index):
        if not hasattr(self, "viewer"):
            return
        self.viewer.set_follow_actual(index == 1)
        if not self._building:
            self.store.update_settings(viewer_mode="actual" if index == 1 else "preview")

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
        self._point_editing = True
        self.point_table.setRowCount(len(self.points))
        for row, record in enumerate(self.points):
            values = [record["name"]] + [f"{value:.2f}" for value in record["joints"]] + [f"{record.get('gripper',100):g}", f"{record.get('speed',35):g}"]
            for col, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                if col:
                    item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.point_table.setItem(row, col, item)
        self.controller.stored_positions = [(record["name"], list(record["joints"])) for record in self.points]
        self._point_editing = False
        if selected is not None and 0 <= selected < len(self.points):
            self.point_table.selectRow(selected)

    def _persist_points(self, selected=None):
        try:
            self.store.save_points(self.points)
        except (OSError, ValueError) as exc:
            self.controller.log(f"点位保存失败：{exc}", "ERROR")
        self._restore_points(selected)

    def _selected_row(self):
        selected = self.point_table.selectionModel().selectedRows()
        return selected[0].row() if selected else None

    def _current_point(self, name):
        return {"name": name, "joints": [spin.value() for spin in self.joint_spins],
                "gripper": self.hand_row.spin.value(), "speed": self.j_speed.value()}

    def _save_point(self):
        name = self.point_name.text().strip() or f"点位 {len(self.points) + 1}"
        self.points.append(self._current_point(name))
        self._persist_points(len(self.points) - 1)
        self.controller.log(f"已保存点位：{name}", "INFO")

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
        record = self.points[row]
        self.point_name.setText(record["name"])
        self.controller._fill_joints(record["joints"])
        # Setting a stored target must never invoke a drag-to-send handler.
        old = self.controller._suppress_drag_send
        self.controller._suppress_drag_send = True
        try:
            self.hand_row.spin.setValue(record.get("gripper", 100))
            self.j_speed.setValue(record.get("speed", 35))
        finally:
            self.controller._suppress_drag_send = old
        self.viewer_mode.setCurrentIndex(0)
        self._preview_changed()

    def _point_name_changed(self, item):
        if self._point_editing or item.column() != 0:
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
        return (selected == "全部" or tag == selected) and self.log_search.text().casefold() in text.casefold()

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
                Path(path).write_text("\n".join(text for _, text in self._logs) + "\n", encoding="utf-8")
                self.controller.log(f"终端日志已导出：{path}", "INFO")
            except OSError as exc:
                self.controller.log(f"日志导出失败：{exc}", "ERROR")

    def apply_theme(self, theme):
        from app.paths import resource_path
        path = resource_path(f"themes/{theme}.qss")
        QApplication.instance().setStyleSheet(Path(path).read_text(encoding="utf-8"))
        self.viewer.set_theme(theme)

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
        if hasattr(self.viewer, "set_grid_visible"):
            self.viewer.set_grid_visible(visible)
        elif hasattr(self.viewer, "set_show_grid"):
            self.viewer.set_show_grid(visible)
        if not self._building:
            self.store.update_settings(show_grid=visible)

    def _axes_changed(self, visible):
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
        self.log_dock.show()
        self.resizeDocks([self.log_dock], [190], Qt.Orientation.Vertical)
        self.store.update_settings(geometry="", window_state="")

    def closeEvent(self, event):
        if not self._closed:
            self._closed = True
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
