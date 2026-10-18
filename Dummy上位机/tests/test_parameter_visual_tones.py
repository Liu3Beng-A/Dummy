"""Parameter visual emphasis is scoped and never changes drafts or geometry."""
from pathlib import Path
import unittest

from PySide6.QtCore import QEvent, QObject, Signal
from PySide6.QtGui import QPalette
from PySide6.QtWidgets import QLabel, QPushButton, QVBoxLayout, QWidget

from tests.test_ui import APPLICATION
from app.core.parameter_edits import ParameterEdits
from app.ui.parameter_edits import ParameterEditsUi, parameter_tone
from app.ui.widgets import number


class DraftController(QObject):
    parameter_edits_changed = Signal()

    def __init__(self, parent):
        super().__init__(parent)
        self.parameter_edits = ParameterEdits(self.parameter_edits_changed.emit)


class ParameterVisualToneTests(unittest.TestCase):
    COLORS = {
        "light": {"muted": "#6d7d8d", "success": "#36785c", "warning": "#93691d", "error": "#b33b43"},
        "dark": {"muted": "#93a8bc", "success": "#8bc3a7", "warning": "#dfbb78", "error": "#ed969c"},
    }

    def setUp(self):
        self.previous_style = APPLICATION.styleSheet()
        self.host = QWidget()
        self.host.setProperty("parameterPanel", True)
        self.layout = QVBoxLayout(self.host)
        self.caption = QLabel("加速度")
        self.caption.setProperty("parameterCaption", True)
        self.caption.setFixedHeight(20)
        self.layout.addWidget(self.caption)
        self.spin = number(10, 0, 5000, 3)
        self.spin.setProperty("parameterInput", True)
        self.spin.setFixedHeight(36)
        self.layout.addWidget(self.spin)
        self.controller = DraftController(self.host)
        self.ui = ParameterEditsUi(self.controller, self.host)
        self.binding = self.ui.register("motor", 1, "ACC", self.spin, 10, minimum=0, maximum=5000)
        self.layout.addWidget(self.binding.readback_label)
        self.layout.addWidget(self.binding.status_label)
        self.buttons = []
        for action in ("primary", "normal", "quiet", "danger"):
            button = self.binding.restore_button if action == "quiet" else QPushButton(action)
            button.setProperty("parameterAction", action)
            if action == "danger":
                button.setProperty("role", "danger")
            button.setFixedHeight(36)
            self.layout.addWidget(button)
            self.buttons.append(button)
        self.rgb_status = QLabel("本地默认 · 未回读")
        self.rgb_status.setProperty("parameterStatus", True)
        self.layout.addWidget(self.rgb_status)
        self.unmarked_input = number(0)
        self.layout.addWidget(self.unmarked_input)
        self.unmarked_button = QPushButton("RGB 原有按钮")
        self.layout.addWidget(self.unmarked_button)
        self.host.resize(480, 600)
        self.host.show()
        self.settle()

    def tearDown(self):
        self.ui.closed = True
        self.host.close()
        self.host.deleteLater()
        APPLICATION.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        self.settle()
        APPLICATION.setStyleSheet(self.previous_style)

    @staticmethod
    def settle():
        for _ in range(6):
            APPLICATION.processEvents()

    def theme(self, name):
        root = Path(__file__).resolve().parents[1] / "resources"
        sheet = (root / "themes" / f"{name}.qss").read_text(encoding="utf-8")
        APPLICATION.setStyleSheet(sheet.replace("{{ICONS}}", (root / "icons").as_posix()))
        self.settle()

    def geometry(self):
        return tuple(widget.geometry().getRect() for widget in (
            self.host, self.caption, self.spin, self.binding.readback_label,
            self.binding.status_label, *self.buttons, self.rgb_status))

    def assert_tone(self, tone, text, theme):
        self.settle()
        self.assertEqual(self.binding.status_label.text(), text)
        self.assertEqual(self.binding.status_label.property("parameterTone"), tone)
        self.assertEqual(self.spin.property("parameterTone"), tone)
        self.assertEqual(self.binding.status_label.palette().color(QPalette.ColorRole.WindowText).name(),
                         self.COLORS[theme][tone])

    def test_tone_matrix_distinguishes_real_readbacks_from_request_receipts(self):
        clean = dict(draft="10", readback=None, fresh=False, dirty=False, sent=None)
        self.assertEqual(parameter_tone(clean), "muted")
        self.assertEqual(parameter_tone(dict(clean, queryable=False)), "muted")
        self.assertEqual(parameter_tone(dict(clean, readback="10", fresh=False)), "muted")
        self.assertEqual(parameter_tone(dict(clean, readback="10", fresh=True)), "success")
        self.assertEqual(parameter_tone(dict(clean, dirty=True)), "warning")
        self.assertEqual(parameter_tone(clean, valid=False), "error")
        for phase, expected in (("pending", "warning"), ("success", "warning"),
                                ("uncertain", "warning"), ("cancelled", "warning"),
                                ("error", "error"), ("blocked", "error"), ("invalid", "error")):
            with self.subTest(phase=phase):
                sent = dict(value="10", fresh=True, phase=phase, evidence="request_confirmed")
                self.assertEqual(parameter_tone(dict(clean, sent=sent, dirty=True)), expected)
                self.assertEqual(parameter_tone(dict(clean, sent=dict(sent, fresh=False))), "muted")
        # Preserve the existing preference for actual fresh matching readback.
        self.assertEqual(parameter_tone(dict(clean, readback="10", fresh=True,
                                            sent=dict(value="10", fresh=True, phase="pending"))), "success")

    def test_real_states_change_colors_without_moving_fields_or_changing_drafts(self):
        edits = self.controller.parameter_edits
        for theme in ("light", "dark"):
            with self.subTest(theme=theme):
                self.theme(theme)
                edits.set_connection(False, theme)
                edits.restore("motor", 1, "ACC")
                initial = self.geometry()
                self.assert_tone("muted", "本地默认 · 未回读" if self.binding.snapshot()["readback"] is None
                                 else "最近回读已过期", theme)
                edits.receive("motor", 1, "ACC", "10")
                self.assert_tone("success", "与最近回读一致", theme)
                self.assertEqual(self.binding.readback_label.property("parameterTone"), "success")
                edits.edit("motor", 1, "ACC", "12.345")
                self.assert_tone("warning", "本地修改 · 未发送", theme)
                self.assertEqual(self.binding.snapshot()["draft"], "12.345")
                self.assertEqual(self.binding.readback_label.property("parameterTone"), "muted")
                edits.edit("motor", 1, "ACC", ".")
                self.assert_tone("error", "输入未完成 / 超出范围", theme)
                self.assertEqual(self.binding.raw_text(), ".")
                edits.restore("motor", 1, "ACC")
                self.assert_tone("success", "与最近回读一致", theme)
                edits.set_connection(False, theme + "-next")
                self.assert_tone("muted", "最近回读已过期", theme)
                self.assertEqual(self.geometry(), initial)
                self.assertEqual(self.binding.status_label.height(), 20)
                self.assertEqual(self.binding.readback_label.height(), 20)

    def test_pending_confirmed_uncertain_and_failure_never_gain_false_green(self):
        self.theme("light")
        edits = self.controller.parameter_edits
        edits.set_connection(True, "fake")
        edits.receive("motor", 1, "ACC", "10")
        edits.edit("motor", 1, "ACC", "20")
        geometry = self.geometry()
        for index, phase in enumerate(("pending", "success", "uncertain", "error", "blocked", "invalid")):
            edits.record_sent("#ACC_J 1 20", index, "fake")
            if phase != "pending":
                edits.update_feedback(dict(task_id=index, phase=phase,
                                           evidence="request_confirmed" if phase == "success" else "none"))
            expected = "error" if phase in ("error", "blocked", "invalid") else "warning"
            self.assertEqual(self.binding.status_label.property("parameterTone"), expected)
            self.assertEqual(self.binding.snapshot()["draft"], "20")
            self.assertEqual(self.geometry(), geometry)
        edits.receive("motor", 1, "ACC", "20")
        self.assert_tone("success", "与最近回读一致", "light")

    def test_local_type_hierarchy_and_button_states_leave_unmarked_rgb_controls_alone(self):
        for theme in ("light", "dark"):
            with self.subTest(theme=theme):
                self.theme(theme)
                self.assertEqual(self.spin.font().pixelSize(), 16)
                self.assertEqual(self.spin.lineEdit().font().pixelSize(), 16)
                self.assertEqual(self.caption.font().pixelSize(), 14)
                self.assertGreaterEqual(self.caption.font().weight(), 600)
                self.assertEqual(self.binding.status_label.font().pixelSize(), 12)
                self.assertEqual(self.unmarked_input.font().pixelSize(), 14)
                self.assertNotEqual(self.unmarked_button.minimumHeight(), 36)
                original = self.geometry()
                for button in self.buttons:
                    self.assertEqual(button.height(), 36)
                    self.assertEqual(button.font().pixelSize(), 14)
                    button.setEnabled(True)
                    button.setFocus()
                    self.settle()
                    self.assertEqual(self.geometry(), original)
                    button.setEnabled(False)
                    self.settle()
                    self.assertEqual(self.geometry(), original)
                for tone, color in self.COLORS[theme].items():
                    self.rgb_status.setProperty("parameterTone", tone)
                    self.rgb_status.style().unpolish(self.rgb_status)
                    self.rgb_status.style().polish(self.rgb_status)
                    self.settle()
                    self.assertEqual(self.rgb_status.palette().color(QPalette.ColorRole.WindowText).name(), color)
                    self.assertEqual(self.geometry(), original)


if __name__ == "__main__":
    unittest.main()
