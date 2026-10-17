"""Stable summary layout and passive, bounded workspace-child conditions."""
import copy
import unittest

from PySide6.QtCore import QCoreApplication, QEvent, Qt
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
from app.paths import resource_path
from app.ui.operation_panel import OperationStatusPanel

APPLICATION = QApplication.instance() or QApplication([])


class _ObservedPanel(OperationStatusPanel):
    filtered_probe_events = 0

    def eventFilter(self, watched, event):
        if event.type() == QEvent.Type.User:
            self.filtered_probe_events += 1
        return super().eventFilter(watched, event)


class OperationPanelTests(unittest.TestCase):
    def setUp(self):
        self.previous_style = APPLICATION.styleSheet()
        self.window = QMainWindow()
        self.host = QWidget()
        self.window.setCentralWidget(self.host)
        layout = QVBoxLayout(self.host)
        layout.setContentsMargins(0, 0, 0, 0)
        self.ui = _ObservedPanel()
        layout.addWidget(self.ui)
        self.outside_button = QPushButton("模拟运动按钮")
        layout.addWidget(self.outside_button)
        self.safety_button = QPushButton("STOP")
        self.safety_button.setProperty("operationOverlaySafety", True)
        layout.addWidget(self.safety_button)
        layout.addStretch()
        self.window.resize(720, 580)
        self.window.show()
        APPLICATION.processEvents()

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        APPLICATION.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        APPLICATION.processEvents()
        APPLICATION.setStyleSheet(self.previous_style)

    @staticmethod
    def actions(**changes):
        result = {
            key: {"label": label, "available": True, "status": "available",
                  "reason": "条件已满足", "next_step": "使用对应控制按钮"}
            for key, label in OperationStatusPanel.ACTIONS
        }
        result.update(changes)
        return result

    def update(self, **changes):
        operation = dict(title="已使能 · 待命", phase="idle", tone="success",
                         reason="可以发送运动目标", next_step="编辑目标后发送", elapsed_s=1.5)
        operation.update(changes)
        self.ui.set_status(operation, self.actions())
        APPLICATION.processEvents()

    def assert_fixed(self):
        for height in (self.ui.height(), self.ui.minimumHeight(), self.ui.maximumHeight(),
                       self.ui.sizeHint().height(), self.ui.minimumSizeHint().height()):
            self.assertEqual(height, 80)
        self.assertTrue(self.ui.reason_label.isVisible())
        self.assertTrue(self.ui.next_step_label.isVisible())

    def test_idle_keeps_all_three_slots_and_details_discoverable(self):
        self.update()
        self.assertEqual(self.ui.title_label.text(), "已使能 · 待命")
        self.assertEqual(self.ui.reason_label.text(), "")
        self.assertEqual(self.ui.next_step_label.text(), "")
        self.assertFalse(self.ui.elapsed_label.isVisible())
        self.assertFalse(self.ui.details_overlay.isVisible())
        self.assertTrue(self.ui.details_button.isEnabled())
        self.assertEqual(self.ui.details_button.arrowType(), Qt.ArrowType.DownArrow)
        self.assertEqual(len(self.ui.action_labels), 6)
        self.assert_fixed()

    def test_all_states_long_text_and_expansion_keep_following_layout_fixed(self):
        before = self.outside_button.geometry()
        # Elapsed appears inside the heading only; reason and next-step stay put.
        slots = [label.geometry() for label in (self.ui.reason_label, self.ui.next_step_label)]
        for phase in ("idle", "queued", "waiting", "executing", "uncertain", "locked",
                      "cancelled", "completed", "observed_reached", "rejected", "disconnected"):
            for expanded in (False, True):
                with self.subTest(phase=phase, expanded=expanded):
                    self.update(phase=phase, title="长状态" * 400,
                                reason="详细原因" * 1000, next_step="下一步" * 1000)
                    self.ui.details_button.setChecked(expanded)
                    APPLICATION.processEvents()
                    self.assert_fixed()
                    self.assertEqual(self.outside_button.geometry(), before)
                    self.assertEqual([label.geometry() for label in (
                        self.ui.reason_label, self.ui.next_step_label)], slots)

    def test_waiting_and_lock_explanations_remain_readable(self):
        self.update(title="使能确认中", phase="waiting", reason="等待主控确认",
                    next_step="请等待", elapsed_s=0.35)
        self.assertEqual(self.ui.reason_label.text(), "等待主控确认")
        self.assertEqual(self.ui.next_step_label.text(), "下一步：请等待")
        self.assertEqual(self.ui.elapsed_label.text(), "已等待 0.3 秒")
        self.update(title="STOP 锁定", phase="locked", tone="warning", reason="用户请求 STOP")
        self.assertFalse(self.ui.elapsed_label.isVisible())
        self.assertEqual(self.ui.property("tone"), "warning")
        self.assert_fixed()

    def test_elapsed_invalid_terminal_and_observed_reached(self):
        self.update(phase="executing", elapsed_s=2.75)
        self.assertEqual(self.ui.elapsed_label.text(), "已用时 2.8 秒")
        for phase in ("manual_confirmation", "uncertain", "accepted", "queued", "waiting_confirmation"):
            self.update(phase=phase)
            self.assertEqual(self.ui.elapsed_label.text(), "已等待 1.5 秒")
        for value in (None, "bad", float("nan"), float("inf"), float("-inf")):
            self.update(phase="waiting", elapsed_s=value)
            self.assertFalse(self.ui.elapsed_label.isVisible())
        for phase in OperationStatusPanel.TERMINAL_PHASES:
            self.update(phase=phase, elapsed_s=99)
            self.assertFalse(self.ui.elapsed_label.isVisible())
        self.update(phase="observed_reached", title="回读位置已到目标", reason="不代替停稳确认")
        self.assertEqual(self.ui.reason_label.text(), "不代替停稳确认")
        self.update(phase="waiting", elapsed_s=-3)
        self.assertEqual(self.ui.elapsed_label.text(), "已等待 0.0 秒")

    def test_child_overlay_preserves_focus_and_expansion_survives_updates(self):
        self.outside_button.setFocus()
        APPLICATION.processEvents()
        focused = QApplication.focusWidget()
        before = self.outside_button.geometry()
        self.ui.details_button.click()
        APPLICATION.processEvents()
        self.assertTrue(self.ui.details_area.isVisible())
        self.assertIs(self.ui.details_overlay.parentWidget(), self.host)
        self.assertFalse(self.ui.details_overlay.isWindow())
        self.assertIs(self.ui.details_overlay.window(), self.window)
        self.assertIs(QApplication.focusWidget(), focused)
        self.assertEqual(self.outside_button.geometry(), before)
        self.ui.set_status({"phase": "waiting", "title": "确认中"}, self.actions(
            movej={"label": "发送 MoveJ", "available": False, "status": "blocked",
                   "reason": "尚未确认使能", "next_step": "等待 START 成功"}))
        self.assertTrue(self.ui.details_button.isChecked())
        self.assertIn("尚未确认使能", self.ui.action_labels["movej"].text())
        self.assertEqual(self.ui.action_labels["movej"].property("status"), "blocked")
        self.ui.details_button.click()
        self.assertFalse(self.ui.details_overlay.isVisible())
        self.assert_fixed()

    def test_outside_click_consumes_motion_but_next_click_works(self):
        calls = []
        self.outside_button.clicked.connect(lambda: calls.append("motion"))
        self.ui.details_button.click()
        QTest.mouseClick(self.outside_button, Qt.MouseButton.LeftButton)
        self.assertFalse(self.ui.details_button.isChecked())
        self.assertEqual(calls, [])
        QTest.mouseClick(self.outside_button, Qt.MouseButton.LeftButton)
        self.assertEqual(calls, ["motion"])

    def test_global_filter_exists_only_while_details_or_outside_release_need_it(self):
        def probe():
            before = self.ui.filtered_probe_events
            APPLICATION.sendEvent(self.outside_button, QEvent(QEvent.Type.User))
            return self.ui.filtered_probe_events - before

        self.assertFalse(self.ui._event_filter_installed)
        self.assertEqual(probe(), 0)
        self.ui.details_button.click()
        self.assertTrue(self.ui._event_filter_installed)
        self.assertGreater(probe(), 0)
        calls = []
        self.outside_button.clicked.connect(lambda: calls.append("motion"))
        QTest.mousePress(self.outside_button, Qt.MouseButton.LeftButton)
        self.assertFalse(self.ui.details_button.isChecked())
        self.assertTrue(self.ui._event_filter_installed)
        QTest.qWait(25)
        self.assertTrue(self.ui._event_filter_installed)
        self.assertGreater(probe(), 0)
        QTest.mouseRelease(self.outside_button, Qt.MouseButton.LeftButton)
        self.assertFalse(self.ui._event_filter_installed)
        self.assertIsNone(self.ui._swallow_release_button)
        self.assertEqual(probe(), 0)
        self.assertEqual(calls, [])
        self.ui.details_button.click()
        self.ui.details_button.click()
        self.assertFalse(self.ui._event_filter_installed)
        self.assertEqual(probe(), 0)

    def test_safety_click_closes_details_and_is_one_click(self):
        calls = []
        self.safety_button.clicked.connect(lambda: calls.append("stop"))
        # Safety admission is an existing host decision, independent of name.
        for name in ("STOP", "失能"):
            self.safety_button.setText(name)
            self.ui.details_button.click()
            QTest.mousePress(self.safety_button, Qt.MouseButton.LeftButton)
            self.assertFalse(self.ui.details_overlay.isVisible())
            self.assertFalse(self.ui._event_filter_installed)
            self.assertIsNone(self.ui._swallow_release_button)
            QTest.mouseRelease(self.safety_button, Qt.MouseButton.LeftButton)
        self.assertEqual(calls, ["stop", "stop"])

    def test_escape_is_consumed_without_closing_parent(self):
        for target in (self.outside_button, self.ui.details_overlay):
            self.ui.details_button.click()
            QTest.keyPress(target, Qt.Key.Key_Escape)
            self.assertTrue(self.ui._event_filter_installed)
            self.assertTrue(self.ui._swallow_escape_release)
            QTest.qWait(25)
            QTest.keyRelease(target, Qt.Key.Key_Escape)
            self.assertFalse(self.ui._event_filter_installed)
            self.assertFalse(self.ui.details_overlay.isVisible())
            self.assertFalse(self.ui.details_button.isChecked())
            self.assertTrue(self.window.isVisible())

    def test_overlay_tracks_resize_move_and_workspace_bounds(self):
        self.ui.details_button.click()
        for width, height in ((720, 580), (460, 270), (320, 210), (1000, 800)):
            self.window.resize(width, height)
            self.window.move(20 + width // 5, 20)
            APPLICATION.processEvents()
            rect = self.ui.details_overlay.geometry()
            self.assertTrue(self.host.rect().contains(rect), (rect, self.host.rect()))
            self.assertLessEqual(rect.width(), 520)
            self.assertLessEqual(rect.height(), 330)
            self.assertEqual(self.ui.details_area.horizontalScrollBar().maximum(), 0)
            self.assert_fixed()

    def test_hide_and_close_collapse_overlay(self):
        self.ui.details_button.click()
        self.window.hide()
        APPLICATION.processEvents()
        self.assertFalse(self.ui.details_button.isChecked())
        self.assertFalse(self.ui.details_overlay.isVisible())
        self.assertFalse(self.ui._event_filter_installed)
        self.window.show()
        APPLICATION.processEvents()
        self.ui.details_button.click()
        self.window.close()
        self.assertFalse(self.ui.details_button.isChecked())
        self.assertFalse(self.ui._event_filter_installed)

    def test_hidden_host_removes_filter_even_with_a_pending_release(self):
        self.ui.details_button.click()
        QTest.mousePress(self.outside_button, Qt.MouseButton.LeftButton)
        self.assertTrue(self.ui._event_filter_installed)
        self.window.hide()
        APPLICATION.processEvents()
        self.assertFalse(self.ui._event_filter_installed)
        self.assertIsNone(self.ui._swallow_release_button)

    def test_passive_updates_preserve_inputs_and_never_click_buttons(self):
        operation = {"phase": "waiting", "title": "等待完成确认", "elapsed_s": 2}
        actions = self.actions()
        original = copy.deepcopy((operation, actions))
        clicks = []
        self.ui.details_button.clicked.connect(clicks.append)
        for _ in range(10):
            self.ui.set_status(operation, actions)
        self.assertEqual(clicks, [])
        self.assertEqual((operation, actions), original)
        self.assertFalse(self.ui.details_button.isChecked())

    def test_long_plain_text_elides_and_preserves_accessible_full_text(self):
        text = "<b>设备回执</b>" + "X" * 1500
        self.ui.set_status({"phase": "rejected", "title": text, "tone": "error",
                            "reason": text, "next_step": text}, self.actions(
            rgb_save={"label": "保存 RGB", "available": False, "reason": text, "next_step": text}))
        self.ui.details_button.click()
        APPLICATION.processEvents()
        for label in (self.ui.title_label, self.ui.reason_label, self.ui.next_step_label):
            self.assertEqual(label.textFormat(), Qt.TextFormat.PlainText)
            self.assertFalse(label.wordWrap())
            self.assertIn(text, label.fullText())
            self.assertIn(text, label.accessibleDescription())
            self.assertIn("&lt;b&gt;", label.toolTip())
            self.assertNotIn("<b>", label.toolTip())
            self.assertLess(len(label.displayedText()), len(label.fullText()))
            self.assertTrue(label.displayedText().endswith("…"))
        self.assertEqual(self.ui.title_label.text(), text)
        self.assertEqual(self.ui.minimumSizeHint().width(), 0)
        self.assertEqual(self.ui.details_area.horizontalScrollBar().maximum(), 0)
        self.assertGreater(self.ui.details_area.verticalScrollBar().maximum(), 0)
        self.assert_fixed()

    def test_themes_remain_external_and_keep_slots_readable(self):
        for theme in ("light", "dark"):
            with self.subTest(theme=theme):
                stylesheet = resource_path(f"themes/{theme}.qss").read_text(encoding="utf-8")
                APPLICATION.setStyleSheet(stylesheet.replace("{{ICONS}}", resource_path("icons").as_posix()))
                self.update(phase="waiting", tone="warning")
                self.ui.details_button.setChecked(True)
                APPLICATION.processEvents()
                self.assertEqual(self.ui.styleSheet(), "")
                self.assertEqual(self.ui.reason_label.styleSheet(), "")
                self.assertFalse(self.ui.grab().toImage().isNull())
                self.assertFalse(self.ui.details_overlay.grab().toImage().isNull())
                self.assertEqual(self.ui.title_label.property("tone"), "warning")
                self.assert_fixed()

    def test_partial_snapshots_and_unknown_values_are_safe(self):
        self.ui.set_status(None, None)
        self.assertEqual(self.ui.title_label.text(), "状态待确认")
        self.assertIn("条件待确认", self.ui.action_labels["movej"].text())
        self.ui.set_status({"tone": "unknown"}, {"start": {"status": "unknown", "available": True}})
        self.assertEqual(self.ui.property("tone"), "info")
        self.assertEqual(self.ui.action_labels["start"].property("status"), "available")
        self.assert_fixed()

    def test_compact_constructor_keeps_full_width_reasons_and_workspace_details(self):
        panel = OperationStatusPanel(self.host, compact=True)
        panel.setGeometry(5, 140, 395, 80)
        panel.set_status({"title": "主控已受理", "phase": "accepted", "reason": "等待位置到达",
                          "next_step": "保持设备连接", "elapsed_s": 2}, self.actions())
        panel.show()
        APPLICATION.processEvents()
        self.assertEqual(panel.height(), 80)
        self.assertEqual(panel.sizeHint().width(), 395)
        self.assertEqual(panel.reason_label.width(), panel.body_widget.width())
        self.assertTrue(panel.reason_label.isVisible())
        self.assertFalse(panel.feedback_slot.isVisible())
        before = panel.geometry()
        panel.details_button.click()
        APPLICATION.processEvents()
        self.assertIs(panel.details_overlay.parentWidget(), self.host)
        self.assertFalse(panel.details_overlay.isWindow())
        self.assertEqual(panel.geometry(), before)
        self.assertTrue(self.host.rect().contains(panel.details_overlay.geometry()))
        panel.close_details(clear_pending=True)
        panel.deleteLater()


if __name__ == "__main__":
    unittest.main()
