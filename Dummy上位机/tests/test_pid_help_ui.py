"""Inline PID descriptions stay readable without changing parameter drafts."""
import unittest
from unittest.mock import patch

from PySide6.QtWidgets import QPushButton, QWidget

from tests import test_parameter_edits_ui as parameter_ui
from app.ui.pid_help import PID_HELP


class PidInlineUiTests(unittest.TestCase):
    setUp = parameter_ui.ParameterEditsUiTests.setUp
    tearDown = parameter_ui.ParameterEditsUiTests.tearDown
    settle = staticmethod(parameter_ui.ParameterEditsUiTests.settle)
    commands = parameter_ui.ParameterEditsUiTests.commands

    def geometry(self):
        return tuple(widget.geometry().getRect() for widget in (
            self.w.pages, self.w.motor_tabs, self.w.pid_group,
            self.w.motor_pid_restore, self.w.operation_panel,
            *self.w.pid_parameter_descriptions.values(),
            *[getattr(self.c, f"ent_set_pid_{key}").widget for key in ("kp", "kv", "ki", "kd")],
        )) + ((self.w.width(), self.w.height()),)

    def test_descriptions_are_inline_and_no_help_button_or_overlay_is_created(self):
        self.c._process_rx_line("ok PID [1] kp=350 kv=320 ki=25 kd=50")
        self.c.parameter_edits.edit("motor", 1, "KP", "250")
        snapshots = {key: self.c.parameter_edits.snapshot("motor", 1, key) for key in PID_HELP}
        self.assertFalse(hasattr(self.w, "pid_help_button"))
        self.assertFalse(hasattr(self.w, "pid_help_overlay"))
        self.assertIsNone(self.w.findChild(QWidget, "PidHelpOverlay"))
        self.assertFalse(any(button.text() == "调参说明" for button in self.w.findChildren(QPushButton)))
        with patch.object(self.c, "send_cmd", side_effect=AssertionError("Descriptions must not send")):
            for key, description in PID_HELP.items():
                label = self.w.pid_parameter_descriptions[key]
                self.assertTrue(label.isVisible())
                self.assertEqual(label.text(), description["inline"])
                for caption in ("作用：", "调大：", "过大："):
                    self.assertIn(caption, label.text())
                self.assertEqual(self.w.pid_parameter_labels[key].text(), description["title"])
            self.settle()
        self.assertEqual(snapshots, {key: self.c.parameter_edits.snapshot("motor", 1, key)
                                    for key in PID_HELP})
        self.assertEqual(self.commands(), [])

    def test_two_themes_three_sizes_keep_all_descriptions_equal_height_and_unclipped(self):
        for theme in ("light", "dark"):
            self.w.apply_theme(theme)
            for width, height in ((1120, 690), (1440, 900), (2502, 1000)):
                with self.subTest(theme=theme, width=width):
                    self.w.resize(width, height)
                    self.settle()
                    before = self.geometry()
                    labels = list(self.w.pid_parameter_descriptions.values())
                    self.assertEqual(len({label.height() for label in labels}), 1)
                    required = max(label.heightForWidth(label.width()) for label in labels)
                    self.assertLessEqual(labels[0].height(), required + 4)
                    for label in labels:
                        self.assertLessEqual(label.heightForWidth(label.width()), label.height())
                    self.c.parameter_edits.edit("motor", 1, "KP", "250")
                    self.c._process_rx_line("ok PID [1] kp=350 kv=320 ki=25 kd=50")
                    self.settle()
                    self.assertEqual(self.geometry(), before)
                    self.assertEqual((self.w.width(), self.w.height()), (width, height))
        self.assertEqual(self.commands(), [])


if __name__ == "__main__":
    unittest.main()
