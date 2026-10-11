"""Manual, read-only shortcuts for the diagnostics in firmware_new."""
from PySide6.QtCore import Signal
from PySide6.QtWidgets import QComboBox, QGridLayout, QWidget

from app.ui.widgets import button, hint


class TerminalDiagnostics(QWidget):
    command_requested = Signal(str)
    export_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False
        self._stack_available = False
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        self.node = QComboBox()
        for node in range(1, 9):
            name = f"J{node} 关节" if node <= 6 else "J7 地轨" if node == 7 else "J8 夹爪"
            self.node.addItem(f"诊断节点 · {name}", node)
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
        for column in range(4):
            layout.setColumnStretch(column, 1)
        self.node.currentIndexChanged.connect(self._refresh)
        self._refresh()

    def set_state(self, *, connected, locked, calibrating, connection_type, disabled):
        self._connected = bool(connected)
        self._stack_available = not (locked or calibrating or (connection_type == "UART4" and disabled))
        self._refresh()

    def _refresh(self, *_args):
        node = self.node.currentData()
        for key, control in self.buttons.items():
            allowed = (node <= 7 if key.startswith("stall") else
                       node in (7, 8) if key.startswith("cal") else True)
            control.setEnabled(self._connected and allowed and (key != "stack" or self._stack_available))

    def _query(self, template):
        self.command_requested.emit(template.format(node=self.node.currentData()))
