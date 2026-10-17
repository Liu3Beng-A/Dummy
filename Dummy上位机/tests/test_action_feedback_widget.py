"""Non-modal feedback stays bounded and cannot expire a newer action."""
import copy
import unittest

from PySide6.QtCore import QCoreApplication, QEvent, Qt
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
from PySide6.QtGui import QEnterEvent
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QPushButton, QVBoxLayout, QWidget

from app.paths import resource_path
from app.ui.action_feedback import ActionFeedbackNotice, feedback_tone
from app.ui.operation_panel import OperationStatusPanel

APPLICATION = QApplication.instance() or QApplication([])


class _Clock:
    value = 100.0

    def __call__(self):
        return self.value


class ActionFeedbackWidgetTests(unittest.TestCase):
    def setUp(self):
        self.previous_style = APPLICATION.styleSheet()
        self.clock = _Clock()
        self.host = QWidget()
        layout = QVBoxLayout(self.host)
        self.panel = OperationStatusPanel()
        self.notice = ActionFeedbackNotice(clock=self.clock)
        self.panel.set_feedback_widget(self.notice)
        layout.addWidget(self.panel)
        self.button = QPushButton("其他操作")
        layout.addWidget(self.button)
        layout.addStretch()
        self.host.resize(1120, 400)
        self.host.show()
        QApplication.processEvents()
        # Offscreen platforms may place the initial mouse inside a widget.
        self.notice._hovered = False

    def tearDown(self):
        self.host.close()
        self.host.deleteLater()
        APPLICATION.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        APPLICATION.processEvents()
        APPLICATION.setStyleSheet(self.previous_style)

    @staticmethod
    def event(**changes):
        value = {
            "id": 1, "task_id": "rgb-1", "scope": "rgb", "operation": "save",
            "target": "RGB", "phase": "pending", "message": "正在保存 RGB 配置…",
            "next_step": "请等待设备确认", "evidence": "", "updated_at": 100.0,
        }
        value.update(changes)
        return value

    def test_fixed_slot_empty_pending_result_and_expiry_do_not_move_controls(self):
        before = (self.panel.geometry(), self.panel.reason_label.geometry(),
                  self.panel.next_step_label.geometry(), self.panel.feedback_slot.geometry(),
                  self.button.geometry())
        for phase in ("pending", "success", "blocked", "invalid", "error", "uncertain", "confirmed"):
            self.notice.set_event(self.event(id=phase, phase=phase, message="长文字" * 600))
            APPLICATION.processEvents()
            self.assertEqual((self.panel.geometry(), self.panel.reason_label.geometry(),
                              self.panel.next_step_label.geometry(), self.panel.feedback_slot.geometry(),
                              self.button.geometry()), before)
            self.assertEqual(self.panel.height(), 80)
            self.assertEqual(self.notice.height(), 44)
            self.assertTrue(self.notice.isVisible())
        self.notice.clear_event()
        self.assertTrue(self.notice.isVisible())
        self.assertEqual(self.button.geometry(), before[-1])
        self.assertEqual(self.notice.graphicsEffect().opacity(), 0)

    def test_pending_does_not_expire_or_take_focus_and_allows_other_controls(self):
        calls = []
        self.button.clicked.connect(lambda: calls.append("clicked"))
        self.button.setFocus()
        focused = QApplication.focusWidget()
        self.notice.set_event(self.event())
        self.assertFalse(self.notice._timer.isActive())
        self.clock.value += 600
        self.notice._on_timeout()
        self.assertEqual(self.notice.current_event["phase"], "pending")
        self.assertIs(QApplication.focusWidget(), focused)
        QTest.mouseClick(self.button, Qt.MouseButton.LeftButton)
        self.assertEqual(calls, ["clicked"])
        self.assertFalse(self.notice.isWindow())
        self.assertEqual(self.notice.focusPolicy(), Qt.FocusPolicy.NoFocus)
        self.assertFalse(self.panel._event_filter_installed)

    def test_duration_policy(self):
        expected = {"pending": None, "success": 3000, "confirmed": 3000,
                    "blocked": 5000, "invalid": 5000, "error": 6000,
                    "uncertain": 6000, "cancelled": 3000}
        for phase, duration in expected.items():
            self.notice.set_event(self.event(phase=phase, updated_at=self.clock.value))
            self.assertEqual(self.notice._remaining_ms, duration)
            self.assertEqual(self.notice._timer.isActive(), duration is not None)
            if duration is not None:
                self.assertEqual(self.notice._timer.interval(), duration)

    def test_success_fades_only_opacity_then_clears_event(self):
        expired = []
        self.notice.expired.connect(expired.append)
        event = self.event(phase="success", message="RGB 配置已保存", evidence="device_confirmed")
        self.notice.set_event(event)
        before = self.notice.geometry()
        self.clock.value += 3
        self.notice._on_timeout()
        QTest.qWait(self.notice.FADE_MS + 80)
        self.assertIsNone(self.notice.current_event)
        self.assertEqual(expired, [event])
        self.assertEqual(self.notice.geometry(), before)
        self.assertTrue(self.notice.isVisible())

    def test_queued_old_timer_cannot_expire_replacement(self):
        self.notice.set_event(self.event(phase="success"))
        self.clock.value += 2.9
        second = self.event(id=2, task_id="motor-2", scope="motor", phase="error",
                            message="参数请求未确认", updated_at=self.clock.value)
        self.notice.set_event(second)
        self.clock.value += 0.1
        self.notice._on_timeout()
        self.assertEqual(self.notice.current_event, second)
        self.assertIsNone(self.notice._animation)
        self.assertGreaterEqual(self.notice._timer.interval(), 5899)

    def test_old_animation_finish_cannot_clear_a_new_task_or_same_task_result(self):
        for task_id in ("rgb-1", "motor-2"):
            self.notice.set_event(self.event(phase="success", updated_at=self.clock.value))
            self.clock.value += 3
            self.notice._on_timeout()
            old_generation, old_animation = self.notice._generation, self.notice._animation
            replacement = self.event(id=2, task_id=task_id, phase="error",
                                     message="写入失败", updated_at=self.clock.value)
            self.notice.set_event(replacement)
            self.notice._finish_expiry(old_generation, old_animation)
            self.assertEqual(self.notice.current_event, replacement)
            self.assertEqual(self.notice.graphicsEffect().opacity(), 1)
            self.clock.value += 1

    def test_pending_error_update_reuses_same_card_and_message_is_not_rewritten(self):
        event = self.event()
        self.notice.set_event(event)
        original_parent = self.notice.parentWidget()
        event.update(id=2, phase="error", message="RGB 保存失败", next_step="请检查设备连接", updated_at=101)
        self.notice.set_event(event)
        self.assertIs(self.notice.parentWidget(), original_parent)
        self.assertEqual(self.notice.message_label.text(), "RGB 保存失败")
        self.assertEqual(self.notice.next_step_label.text(), "请检查设备连接")
        self.assertEqual(self.notice.property("tone"), "error")
        self.assertEqual(len(self.panel.feedback_slot.findChildren(ActionFeedbackNotice)), 1)

    def test_forwarded_save_request_is_informational_and_preserves_qualified_text(self):
        event = self.event(scope="motor", phase="success", evidence="request_confirmed",
                           message="保存请求已确认，电机写入结果未回报")
        self.notice.set_event(event)
        self.assertEqual(self.notice.message_label.text(), event["message"])
        self.assertEqual(self.notice.property("tone"), "info")
        self.assertEqual(feedback_tone(event), "info")
        event["evidence"] = "device_confirmed"
        self.assertEqual(feedback_tone(event), "success")

    def test_duplicate_and_older_update_do_not_restart_or_regress_new_result(self):
        event = self.event(phase="success", updated_at=102)
        self.notice.set_event(event)
        deadline = self.notice._deadline
        generation = self.notice._generation
        self.clock.value += 1
        self.notice.set_event(copy.deepcopy(event))
        self.assertEqual(self.notice._generation, generation)
        self.assertEqual(self.notice._deadline, deadline)
        self.notice.set_event(self.event(phase="pending", updated_at=101))
        self.assertEqual(self.notice.current_event, event)

    def test_event_input_and_current_event_are_detached(self):
        event = self.event(extra={"value": [1]})
        original = copy.deepcopy(event)
        self.notice.set_event(event)
        event["extra"]["value"].append(2)
        visible = self.notice.current_event
        visible["extra"]["value"].append(3)
        self.assertEqual(self.notice.current_event, original)

    def test_long_device_text_elides_and_keeps_escaped_full_tooltip_and_accessibility(self):
        text = "<b>设备回执</b>" + "超长内容" * 1000
        self.notice.set_event(self.event(phase="error", message=text, next_step=text))
        QApplication.processEvents()
        for label in (self.notice.message_label, self.notice.next_step_label):
            self.assertEqual(label.textFormat(), Qt.TextFormat.PlainText)
            self.assertFalse(label.wordWrap())
            self.assertEqual(label.fullText(), text)
            self.assertIn(text, label.accessibleDescription())
            self.assertIn("&lt;b&gt;", label.toolTip())
            self.assertNotIn("<b>", label.toolTip())
            self.assertLess(len(label.displayedText()), len(text))
        self.assertIn("&lt;b&gt;", self.notice.toolTip())
        self.assertNotIn("<b>", self.notice.toolTip())
        self.assertIn(text, self.notice.accessibleDescription())

    def test_hover_pauses_expiry_to_read_full_text_without_keeping_a_new_task_stale(self):
        self.notice.set_event(self.event(phase="success"))
        self.clock.value += 2
        APPLICATION.sendEvent(self.notice, QEnterEvent(self.notice.pos(), self.notice.pos(), self.notice.pos()))
        self.assertFalse(self.notice._timer.isActive())
        self.clock.value += 20
        self.notice._on_timeout()
        self.assertIsNotNone(self.notice.current_event)
        APPLICATION.sendEvent(self.notice, QEvent(QEvent.Type.Leave))
        self.assertEqual(self.notice._timer.interval(), 1000)
        self.clock.value += 1
        self.notice._on_timeout()
        QTest.qWait(self.notice.FADE_MS + 80)
        self.assertIsNone(self.notice.current_event)

    def test_clear_and_partial_events_are_safe_and_fixed(self):
        self.notice.set_event({"message": "只显示提供的消息"})
        self.assertEqual(self.notice.message_label.text(), "只显示提供的消息")
        self.assertEqual(self.notice.property("tone"), "info")
        self.notice.set_event("not a dictionary")
        self.assertEqual(self.notice.message_label.text(), "只显示提供的消息")
        self.notice.set_event(None)
        self.assertIsNone(self.notice.current_event)
        self.assertEqual(self.notice.message_label.text(), "")
        self.assertEqual(self.notice.height(), 44)

    def test_external_themes_do_not_change_widget_dimensions_or_message_semantics(self):
        before = self.button.geometry()
        for theme in ("light", "dark"):
            stylesheet = resource_path(f"themes/{theme}.qss").read_text(encoding="utf-8")
            APPLICATION.setStyleSheet(stylesheet.replace("{{ICONS}}", resource_path("icons").as_posix()))
            self.notice.set_event(self.event(phase="blocked", message="请先失能并等待确认"))
            QApplication.processEvents()
            self.assertEqual(self.notice.styleSheet(), "")
            self.assertEqual(self.panel.height(), 80)
            self.assertEqual(self.notice.height(), 44)
            self.assertFalse(self.notice.grab().toImage().isNull())
            self.assertEqual(self.notice.message_label.text(), "请先失能并等待确认")
            # Theme fonts may change button height, but feedback itself never
            # changes the panel footprint or creates another top-level window.
            self.assertEqual(self.button.geometry().top(), before.top())

    def test_event_changed_reports_updates_manual_clear_and_expiry(self):
        changed, expired = [], []
        self.notice.event_changed.connect(changed.append)
        self.notice.expired.connect(expired.append)
        self.notice.set_event(self.event())
        self.notice.set_event(self.event(phase="error", updated_at=101))
        self.notice.clear_event()
        self.assertEqual(changed, [True, True, False])
        self.assertEqual(expired, [])
        self.notice.clear_event()
        self.assertEqual(changed, [True, True, False])
        self.notice.set_event(self.event(phase="success", updated_at=102))
        self.clock.value += 3
        self.notice._on_timeout()
        QTest.qWait(self.notice.FADE_MS + 70)
        self.assertEqual(changed[-2:], [True, False])
        self.assertEqual(len(expired), 1)

    def test_compact_feedback_shares_full_width_body_without_moving_other_controls(self):
        self.panel.set_compact(True)
        self.panel.set_status({"phase": "waiting", "title": "等待确认", "reason": "原始原因",
                               "next_step": "原始操作"}, {})
        self.button.setFocus()
        for width in (417, 567):
            self.host.resize(width, 400)
            APPLICATION.processEvents()
            before = self.button.geometry()
            focus = QApplication.focusWidget()
            reason_rect = self.panel.description_widget.geometry()
            self.assertTrue(self.panel.description_widget.isVisible())
            self.assertFalse(self.panel.feedback_slot.isVisible())
            self.assertEqual(reason_rect.width(), self.panel.body_widget.width())
            self.assertEqual(self.panel.feedback_slot.geometry(), reason_rect)
            self.assertEqual(self.notice.width(), reason_rect.width())
            self.assertEqual(self.notice.height(), 44)
            self.notice.set_event(self.event(message="很长的反馈" * 500))
            APPLICATION.processEvents()
            self.assertFalse(self.panel.description_widget.isVisible())
            self.assertTrue(self.panel.feedback_slot.isVisible())
            self.assertEqual(self.panel.feedback_slot.geometry(), reason_rect)
            self.assertEqual(self.notice.width(), reason_rect.width())
            self.assertEqual(self.button.geometry(), before)
            self.assertIs(QApplication.focusWidget(), focus)
            self.assertFalse(self.panel._event_filter_installed)
            self.notice.clear_event()
            APPLICATION.processEvents()
            self.assertTrue(self.panel.description_widget.isVisible())
            self.assertEqual(self.panel.description_widget.geometry(), reason_rect)
            self.assertEqual(self.button.geometry(), before)
            self.assertEqual(self.panel.height(), 80)

    def test_compact_hidden_notice_tracks_resize_before_its_first_event(self):
        self.panel.set_compact(True)
        self.assertIsNone(self.notice.current_event)
        for width in (417, 567, 467, 417):
            self.host.resize(width, 400)
            APPLICATION.processEvents()
            expected = self.panel.body_widget.width()
            self.assertFalse(self.panel.feedback_slot.isVisible())
            self.assertEqual(self.panel.feedback_slot.width(), expected)
            self.assertEqual(self.notice.width(), expected)
            before = self.notice.geometry()
            self.notice.set_event(self.event())
            APPLICATION.processEvents()
            self.assertEqual(self.notice.geometry(), before)
            self.notice.clear_event()
            APPLICATION.processEvents()
            self.assertEqual(self.notice.geometry(), before)

    def test_compact_restores_latest_hidden_reason_after_manual_clear_and_fade(self):
        self.panel.set_compact(True)
        self.notice.set_event(self.event())
        self.panel.set_status({"phase": "locked", "title": "保护锁定", "reason": "保护原始原因",
                               "next_step": "检查保护信息"}, {})
        self.assertFalse(self.panel.description_widget.isVisible())
        self.notice.clear_event()
        self.assertTrue(self.panel.description_widget.isVisible())
        self.assertEqual(self.panel.reason_label.text(), "保护原始原因")
        self.assertIn("检查保护信息", self.panel.next_step_label.text())
        self.notice.set_event(self.event(phase="success"))
        self.clock.value += 3
        self.notice._on_timeout()
        QTest.qWait(self.notice.FADE_MS + 70)
        self.assertTrue(self.panel.description_widget.isVisible())
        self.assertFalse(self.panel.feedback_slot.isVisible())
        self.assertEqual(self.panel.reason_label.text(), "保护原始原因")

    def test_compact_switching_modes_and_replacing_notice_disconnects_old_updates(self):
        self.notice.set_event(self.event())
        self.panel.set_compact(True)
        self.assertTrue(self.panel.feedback_slot.isVisible())
        self.panel.set_compact(False)
        APPLICATION.processEvents()
        self.assertTrue(self.panel.description_widget.isVisible())
        self.assertTrue(self.panel.feedback_slot.isVisible())
        self.assertEqual(self.panel.feedback_slot.width(), 380)
        self.panel.set_compact(True)
        replacement = ActionFeedbackNotice(clock=self.clock)
        original = self.notice
        self.panel.set_feedback_widget(replacement)
        self.assertTrue(self.panel.description_widget.isVisible())
        original.clear_event()
        original.set_event(self.event(id=2, phase="error", updated_at=102))
        self.assertTrue(self.panel.description_widget.isVisible())
        replacement.set_event(self.event())
        self.assertFalse(self.panel.description_widget.isVisible())
        self.panel.set_feedback_widget(None)
        self.assertTrue(self.panel.description_widget.isVisible())
        original.deleteLater()
        replacement.deleteLater()


if __name__ == "__main__":
    unittest.main()
