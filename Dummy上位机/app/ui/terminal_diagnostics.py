"""Firmware diagnostics and bounded joint communication tests."""
from PySide6.QtCore import Signal
from PySide6.QtWidgets import QComboBox, QGridLayout, QPlainTextEdit, QWidget

from app.ui.widgets import button, hint


class TerminalDiagnostics(QWidget):
    command_requested = Signal(str)
    export_requested = Signal()
    open_test_requested = Signal(int)
    axis_test_requested = Signal(int)
    motion_axis_test_requested = Signal(int)
    test_clicked = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False
        self._stack_available = False
        self._open_test_available = False
        self._axis_test_available = False
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        self.node = QComboBox()
        for node in range(1, 9):
            name = f"J{node} 关节" if node <= 6 else "J7 地轨" if node == 7 else "J8 夹爪"
            self.node.addItem(f"诊断 / OPEN 节点 · {name}", node)
        self.node.setToolTip("CAN 支持 J1～J8；堵转支持 J1～J7；校准快照支持 J7 / J8。切换节点不发送指令。")
        layout.addWidget(self.node, 0, 0)
        self.buttons = {}
        entries = (
            ("core_version", "主控版本", "#FW_VERSION 0", 0, 1),
            ("motor_version", "节点版本", "#FW_VERSION {node}", 0, 2),
            ("can", "CAN 通信", "#CAN_DIAG {node}", 0, 3),
            ("stall", "堵转当前", "#STALL_STATUS {node}", 1, 0),
            ("stall_last", "堵转 LAST", "#STALL_STATUS {node} LAST", 1, 1),
            ("cal", "校准当前", "#CAL_STATUS {node}", 1, 2),
            ("cal_last", "校准 LAST", "#CAL_STATUS {node} LAST", 1, 3),
            ("motion", "运动状态", "#MOTION_STATUS", 2, 0),
            ("stack", "USB / UART 栈", "#STACK", 2, 1),
            ("rgb", "RGB 状态", "#RGB_STATUS", 2, 2),
        )
        for key, title, command, row, column in entries:
            control = button(title, lambda template=command: self._query(template))
            control.setToolTip(command + ("\nLAST 为电机保存的最近一次事件快照。" if key.endswith("_last") else ""))
            self.buttons[key] = control
            layout.addWidget(control, row, column)
        self.buttons["stack"].setToolTip(
            "#STACK：当前固件仅报告 USB / UART 最小剩余栈及空闲堆，不含运动任务栈。\n"
            "STOP / 校准锁定或 UART4 失能时不可用。")
        self.export_button = button("导出完整日志", self.export_requested.emit)
        self.export_button.setToolTip("导出当前保留的全部日志，包含隐藏的关节回读，不受搜索、分类和暂停显示影响。")
        layout.addWidget(self.export_button, 2, 3)
        layout.addWidget(hint("单次只读查询，请等回包后再查下一项。LAST 为最近事件；主控停刷后可能无法回复。"), 3, 0, 1, 4)
        self.open_test_button = button('单电机 OPEN → CLOSE 测试', self._request_open_test)
        self.open_test_button.setToolTip('仅 USB / J1～J7；主控自动关闭；不切模式、不调用逆解、不发送运动点。')
        layout.addWidget(self.open_test_button, 4, 0, 1, 2)
        layout.addWidget(hint('只验证握手；关闭确认和拒绝快照显示在下方。'), 4, 2, 1, 2)
        self.axis_test_selector = QComboBox()
        layout.addWidget(hint('曲线测试参与轴（独立选择，不随上方诊断节点变化）'), 5, 0, 1, 4)
        choices = [(f'J{node} · 旋转关节', 1 << (node - 1)) for node in range(1, 7)]
        choices.extend((('J7 · 地轨', 64), ('J1 + J7 · 两轴同时', 65), ('J1～J7 · 七轴同时', 127)))
        for label, mask in choices:
            self.axis_test_selector.addItem(label, mask)
        self.axis_test_selector.setToolTip('选择本轮曲线通信测试的参与轴；切换选择不会发送指令。')
        layout.addWidget(self.axis_test_selector, 6, 0, 1, 2)
        self.axis_test_button = button('2 s 原地曲线通信测试', self._request_axis_test)
        self.axis_test_button.setToolTip('保持电机已有目标，发送50 Hz零位移曲线，验证接收、缓冲、执行和关闭。 '
                                        '需要USB、已使能且静止、模式2或7；不调用逆解，不自动使能。 '
                                        '结束后确认CLOSE并恢复进入模式；故障沿用全机软停保护。')
        layout.addWidget(self.axis_test_button, 6, 2, 1, 2)
        self.motion_axis_test_button = button('J3/J5 · 0.5° 非零往返测试', self._request_motion_axis_test)
        self.motion_axis_test_button.setToolTip('会真实运动，仅支持下方所选 J3 或 J5 单轴：J3 -0.5°，J5 +0.5°；2 s 出去 + 2 s 返回。 '
                                               '新电机计时能力确认后才 OPEN；越限拒绝、不自动改方向。不调用逆解、不自动使能。 '
                                               '结束核对真实位移和返回位置，确认 CLOSE 并恢复模式；故障全机软停。')
        layout.addWidget(self.motion_axis_test_button, 7, 0, 1, 2)
        layout.addWidget(hint('真实运动：先选 J3 测通，再选 J5；其他轴不参与，默认主控模式不变。'), 7, 2, 1, 2)
        self.open_test_output = QPlainTextEdit()
        self.open_test_output.setObjectName("OpenTestResult")
        self.open_test_output.setReadOnly(True)
        self.open_test_output.setPlainText('尚未测试。OPEN 使用上方“诊断 / OPEN 节点”；原地曲线使用下方“曲线测试参与轴”。两个选择独立。')
        self.open_test_output.setMinimumHeight(110)
        self.open_test_output.setMaximumHeight(190)
        layout.addWidget(self.open_test_output, 8, 0, 1, 4)
        for column in range(4):
            layout.setColumnStretch(column, 1)
        self.node.currentIndexChanged.connect(self._refresh)
        self.axis_test_selector.currentIndexChanged.connect(self._refresh)
        self._refresh()

    def set_state(self, *, connected, locked, calibrating, connection_type, disabled, busy=False):
        self._connected = bool(connected)
        self._open_test_available = connected and connection_type == "USB" and not (locked or calibrating or busy)
        self._axis_test_available = self._open_test_available and not disabled
        self._stack_available = not (locked or calibrating or (connection_type == "UART4" and disabled))
        self._refresh()

    def _refresh(self, *_args):
        node = self.node.currentData()
        self.open_test_button.setText(f'J{node} · 单电机 OPEN → CLOSE 测试')
        self.axis_test_button.setText(self._axis_target()+' · 2 s 原地曲线通信测试')
        self.open_test_button.setEnabled(self._open_test_available and node <= 7)
        self.axis_test_button.setEnabled(self._axis_test_available)
        self.axis_test_selector.setEnabled(self._axis_test_available)
        motion_mask = self.axis_test_selector.currentData()
        self.motion_axis_test_button.setEnabled(self._axis_test_available and motion_mask in (4, 16))
        motion_node = 3 if motion_mask == 4 else 5 if motion_mask == 16 else None
        self.motion_axis_test_button.setText(f'J{motion_node} · {-.5 if motion_node == 3 else .5:+.1f}° 非零往返测试' if motion_node else 'J3/J5 · 0.5° 非零往返测试')
        for key, control in self.buttons.items():
            allowed = (node <= 7 if key.startswith("stall") else
                       node in (7, 8) if key.startswith("cal") else True)
            control.setEnabled(self._connected and allowed and (key != "stack" or self._stack_available))

    def _axis_target(self):
        mask = self.axis_test_selector.currentData()
        if mask == 127:
            return 'J1～J7（七轴）'
        if mask == 65:
            return 'J1+J7（两轴）'
        return '+'.join(f'J{node}' for node in range(1, 8) if mask & (1 << (node-1)))

    def _request_open_test(self):
        node = self.node.currentData()
        self.test_clicked.emit(f'测试点击：OPEN→CLOSE；诊断 / OPEN 节点=J{node}；请求节点={node}；'
                               f'曲线参与轴={self._axis_target()}（独立选择，本次不用）')
        self.open_test_requested.emit(node)

    def _request_axis_test(self):
        mask = self.axis_test_selector.currentData()
        self.test_clicked.emit(f'测试点击：2 s 原地曲线；参与轴={self._axis_target()}；mask={mask}；'
                               f'诊断 / OPEN 节点=J{self.node.currentData()}（独立选择，本次不用）')
        self.axis_test_requested.emit(mask)

    def _request_motion_axis_test(self):
        mask = self.axis_test_selector.currentData()
        if mask not in (4, 16):
            return
        node = 3 if mask == 4 else 5
        delta = -.5 if node == 3 else .5
        self.test_clicked.emit(f'测试点击：真实非零往返；仅 J{node}；固定 {delta:+.1f}°；2 s 出去 + 2 s 返回；mask={mask}')
        self.motion_axis_test_requested.emit(mask)

    def show_open_test(self, result):
        from app.core.open_test import format_open_test
        self.open_test_output.setPlainText(format_open_test(result))

    def show_axis_test(self, result):
        from app.core.axis_communication_test import format_axis_test
        self.open_test_output.setPlainText(format_axis_test(result))

    def _query(self, template):
        self.command_requested.emit(template.format(node=self.node.currentData()))
