"""Configuration feedback integration without serial hardware or modal dialogs."""
from copy import deepcopy
from pathlib import Path
import queue
import tempfile
import unittest
from unittest.mock import patch

from PySide6.QtCore import QEvent, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QMessageBox

from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow
from tests.test_controller import FakePort, ManualScheduler
from tests.test_gamepad_ui import FakeReader
from tests.test_ui import APPLICATION, FakeView


class ActionFeedbackUiTests(unittest.TestCase):
    def setUp(self):
        base = Path(__file__).resolve().parents[1] / "build" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=base)
        self.store = AppStore(self.directory.name)
        self.scheduler = ManualScheduler()
        self.controller = RobotController(scheduler=self.scheduler)
        self.viewer_patch = patch("app.ui.main_window.RobotView", FakeView)
        self.viewer_patch.start()
        self.window = MainWindow(self.controller, self.store, gamepad_reader=FakeReader())
        self.window.resize(1120, 760)
        self.window.show()
        self.scheduler.run_ready()
        APPLICATION.processEvents()
        self.window.action_feedback_notice._hovered = False
        self.window.rgb_feedback_notice._hovered = False
        self.popup_patches = [patch.object(QMessageBox, name, side_effect=AssertionError("feedback must not show dialogs"))
                              for name in ("information", "warning", "critical", "question")]
        for mock in self.popup_patches:
            mock.start()

    def tearDown(self):
        for mock in reversed(self.popup_patches):
            mock.stop()
        self.window._control_owner = "ui"
        self.window.close()
        self.window.deleteLater()
        APPLICATION.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        APPLICATION.processEvents()
        self.viewer_patch.stop()
        self.store.close()
        self.directory.cleanup()

    def connect_fake(self, enabled=False):
        self.controller.is_connected = True
        self.controller.serial_port = FakePort()
        self.controller._motor_state_known = True
        self.controller._motor_enabled = enabled
        self.controller._publish()
        APPLICATION.processEvents()
        # Publishing a newly connected state may schedule the existing RGB
        # configuration seed query. It is baseline behavior, not action UI work.
        baseline = self.commands()
        self.assertTrue(all(item[1] in ("#RGB_GET", "#RGB_STATUS") for item in baseline))

    def commands(self):
        values = []
        while True:
            try:
                item = self.controller._tx_queue.get_nowait()
            except queue.Empty:
                return values
            if item is not None:
                values.append(item)

    def admit_and_write(self, expected):
        commands = self.commands()
        self.assertEqual([item[1] for item in commands], [expected])
        generation, command = commands[0]
        self.assertTrue(self.controller._can_write(generation, command))
        self.controller._on_written(command, self.controller._connection_id)
        return self.controller.feedback_snapshot()

    def notify(self, scope="motor", phase="pending", message="正在应用配置…", **kwargs):
        return self.controller.notify_action(scope, kwargs.pop("operation", "应用配置"),
                                             target=kwargs.pop("target", "节点 1"),
                                             phase=phase, message=message,
                                             next_step=kwargs.pop("next_step", "请等待设备确认"), **kwargs)

    def test_same_task_pending_sent_success_updates_main_notice_and_persistent_result(self):
        task = self.notify()
        self.assertEqual(self.window.action_feedback_notice.current_event["phase"], "pending")
        self.notify(phase="pending", message="请求已发送，等待确认。", task_id=task, evidence="written")
        self.assertEqual(self.window.motor_feedback_result.text(), "请求已发送，等待确认。")
        self.notify(phase="success", message="配置已确认。", task_id=task, evidence="device_confirmed")
        current = self.window.action_feedback_notice.current_event
        self.assertEqual(current["task_id"], task)
        self.assertEqual(current["phase"], "success")
        self.assertEqual(self.window.motor_feedback_result.text(), "配置已确认。")
        self.assertEqual(self.window.motor_feedback_result.property("tone"), "success")

    def test_rgb_apply_runs_through_real_dispatch_and_receipt_without_opening_editor(self):
        self.connect_fake()
        self.assertTrue(self.controller.send_cmd("!RGB_BRIGHT 25"))
        task = self.window.action_feedback_notice.current_event["task_id"]
        self.admit_and_write("!RGB_BRIGHT 25")
        self.assertIn("已发送", self.window.rgb_save_status.text())
        self.controller._process_rx_line("ok RGB applied")
        event = self.window.action_feedback_notice.current_event
        self.assertEqual((event["task_id"], event["phase"], event["evidence"]),
                         (task, "success", "device_confirmed"))
        self.assertEqual(self.window.rgb_save_status.property("tone"), "success")
        self.assertFalse(self.window.rgb_window.isVisible())
        self.assertEqual(self.commands(), [])

    def test_motor_acknowledgement_stays_qualified_and_information_colored(self):
        self.connect_fake()
        self.assertTrue(self.controller.send_cmd("#SET_PID 1 10 20 30 40"))
        task = self.window.action_feedback_notice.current_event["task_id"]
        self.admit_and_write("#SET_PID 1 10 20 30 40")
        self.controller._process_rx_line("ok SET PID [1] kp=10 kv=20 ki=30 kd=40")
        event = self.window.action_feedback_notice.current_event
        self.assertEqual(event["task_id"], task)
        self.assertEqual(event["evidence"], "request_confirmed")
        self.assertEqual(self.window.action_feedback_notice.property("tone"), "info")
        self.assertEqual(self.window.motor_feedback_result.property("tone"), "info")
        self.assertIn("电机写入结果未回报", self.window.motor_feedback_result.text())
        self.assertNotIn("保存成功", self.window.motor_feedback_result.text())

    def test_all_scopes_keep_independent_results_and_old_tasks_cannot_overwrite_newer_task(self):
        expected = {}
        for scope in ("rgb", "motor", "protection", "calibration", "gripper"):
            text = f"{scope} 当前操作已确认。"
            old = self.notify(scope, "pending", f"{scope} 正在处理…")
            latest = self.notify(scope, "success", text, evidence="device_confirmed")
            self.notify(scope, "error", "旧任务失败", task_id=old)
            expected[scope] = (latest, text)
        for scope, (latest, text) in expected.items():
            self.assertEqual(self.window._feedback_last[scope]["task_id"], latest)
            for label in self.window._feedback_result_labels[scope]:
                self.assertEqual(label.text(), text)
                self.assertEqual(label.height(), 22)
        self.assertFalse(self.window.rgb_window.isVisible())

    def test_visible_rgb_window_routes_rgb_only_there_and_does_not_take_input_focus(self):
        self.window._open_rgb_window()
        APPLICATION.processEvents()
        self.assertTrue(self.window.rgb_window.isVisible())
        self.window.fields["_rgb_bright"].widget.setFocus()
        APPLICATION.processEvents()
        focused = QApplication.focusWidget()
        task = self.notify("rgb", "pending", "正在保存 RGB…")
        self.assertEqual(self.window.rgb_feedback_notice.current_event["task_id"], task)
        self.assertIsNone(self.window.action_feedback_notice.current_event)
        self.assertIs(QApplication.focusWidget(), focused)
        self.notify("motor", "blocked", "请先失能。")
        self.assertEqual(self.window.action_feedback_notice.current_event["scope"], "motor")
        self.assertEqual(self.window.rgb_feedback_notice.current_event["scope"], "rgb")
        self.assertIs(QApplication.focusWidget(), focused)

    def test_hidden_and_minimized_rgb_editor_never_opens_for_feedback(self):
        self.notify("rgb", "blocked", "RGB 未保存：请先失能。")
        self.assertFalse(self.window.rgb_window.isVisible())
        self.assertEqual(self.window.action_feedback_notice.current_event["scope"], "rgb")
        self.window._open_rgb_window()
        APPLICATION.processEvents()
        self.window.rgb_window.showMinimized()
        APPLICATION.processEvents()
        self.notify("rgb", "error", "保存结果未确认。")
        self.assertTrue(self.window.rgb_window.isMinimized())
        self.assertEqual(self.window.action_feedback_notice.message_label.text(), "保存结果未确认。")

    def test_opening_rgb_between_pending_and_result_removes_old_main_pending_card(self):
        task = self.notify("rgb", "pending", "正在保存 RGB…")
        self.assertEqual(self.window.action_feedback_notice.current_event["phase"], "pending")
        self.window._open_rgb_window()
        APPLICATION.processEvents()
        self.notify("rgb", "success", "RGB 配置已保存。", task_id=task, evidence="device_confirmed")
        self.assertEqual(self.window.rgb_feedback_notice.current_event["phase"], "success")
        self.assertIsNone(self.window.action_feedback_notice.current_event)
        self.assertEqual(self.window.rgb_save_status.text(), "RGB 配置已保存。")

    def test_closing_rgb_between_pending_and_result_removes_hidden_pending_card(self):
        self.window._open_rgb_window()
        APPLICATION.processEvents()
        task = self.notify("rgb", "pending", "正在保存 RGB…")
        self.assertEqual(self.window.rgb_feedback_notice.current_event["phase"], "pending")
        self.window.rgb_window.close()
        APPLICATION.processEvents()
        self.assertFalse(self.window.rgb_window.isVisible())
        self.notify("rgb", "error", "RGB 保存失败。", task_id=task, evidence="device_rejected")
        self.assertEqual(self.window.action_feedback_notice.current_event["phase"], "error")
        self.assertIsNone(self.window.rgb_feedback_notice.current_event)
        self.assertFalse(self.window.rgb_window.isVisible())

    def test_pending_result_error_and_clear_keep_all_page_and_status_geometries_fixed(self):
        for theme in ("light", "dark"):
            self.window.apply_theme(theme)
            self.window.navigation.setCurrentRow(2)
            APPLICATION.processEvents()
            labels = [label for values in self.window._feedback_result_labels.values() for label in values]
            widgets = [self.window.operation_panel, self.window.pages,
                       self.window.motor_tabs, self.window.action_feedback_notice] + labels
            before = [widget.geometry() for widget in widgets]
            for phase in ("pending", "success", "blocked", "invalid", "error", "uncertain"):
                for scope in self.window._feedback_result_labels:
                    self.notify(scope, phase, "很长的操作结果" * 300, next_step="详细操作方法" * 300)
                APPLICATION.processEvents()
                self.assertEqual([widget.geometry() for widget in widgets], before)
                self.assertEqual(self.window.operation_panel.height(), 80)
            self.window.action_feedback_notice.clear_event()
            self.window.rgb_feedback_notice.clear_event()
            APPLICATION.processEvents()
            self.assertEqual([widget.geometry() for widget in widgets], before)
            self.assertTrue(all(label.text() for label in labels))

    def test_rgb_editor_geometry_remains_fixed_for_long_feedback_updates(self):
        self.window._open_rgb_window()
        APPLICATION.processEvents()
        widgets = (self.window.rgb_window, self.window.rgb_tabs, self.window.rgb_status,
                   self.window.rgb_save_status, self.window.rgb_feedback_notice)
        before = [widget.geometry() for widget in widgets]
        for phase in ("pending", "success", "blocked", "error"):
            self.notify("rgb", phase, "长结果" * 1000, next_step="长说明" * 1000)
            APPLICATION.processEvents()
            self.assertEqual([widget.geometry() for widget in widgets], before)
        self.window.rgb_feedback_notice.clear_event()
        APPLICATION.processEvents()
        self.assertEqual([widget.geometry() for widget in widgets], before)

    def test_background_readback_logging_and_status_refresh_do_not_replace_action_result(self):
        self.connect_fake()
        self.notify("motor", "success", "保存请求已确认，电机写入结果未回报。", evidence="request_confirmed")
        original = deepcopy(self.window.action_feedback_notice.current_event)
        labels = {scope: [label.text() for label in values]
                  for scope, values in self.window._feedback_result_labels.items()}
        for line in ("ok 0 -75 180 0 0 0 0", "RGB_STATUS on=1 auto=1 bright=25 current=25 mode=3",
                     "RGB_RUNTIME state=DISABLED source=BOOT", "RGB_STORAGE dirty=0 storage=1 dma=0",
                     "ok CMDMODE mode=2 busy=0 issued=0 req=1"):
            self.controller._process_rx_line(line)
        self.controller.log("测试后台日志", "INFO")
        self.window._refresh_operation_status()
        self.window._rgb_device_changed(self.controller.rgb_snapshot())
        QApplication.processEvents()
        self.assertEqual(self.window.action_feedback_notice.current_event, original)
        self.assertEqual({scope: [label.text() for label in values]
                          for scope, values in self.window._feedback_result_labels.items()}, labels)
        self.assertEqual(self.commands(), [])

    def test_gripper_save_requires_disable_feedback_without_modal_or_dispatch(self):
        self.connect_fake(enabled=True)
        self.window.navigation.setCurrentRow(3)
        self.window.hand_advanced_fields["ACC"].setFocus()
        focused = QApplication.focusWidget()
        self.window._hand_config("ACC")
        event = self.window.action_feedback_notice.current_event
        self.assertEqual((event["scope"], event["phase"]), ("gripper", "blocked"))
        self.assertIn("失能", event["message"])
        self.assertIn("失能", event["next_step"])
        self.assertEqual(self.window.gripper_feedback_result.text(), event["message"])
        self.assertEqual(self.commands(), [])
        self.assertIs(QApplication.focusWidget(), focused)

    def test_rgb_and_protection_calibration_save_rejections_are_visible_without_dialogs(self):
        self.connect_fake(enabled=True)
        for scope, method in (("rgb", self.controller._rgb_save),
                              ("protection", lambda: self.controller._save_protection("STALL")),
                              ("calibration", lambda: self.controller._save_protection("CAL"))):
            method()
            event = self.window.action_feedback_notice.current_event
            self.assertEqual((event["scope"], event["phase"]), (scope, "blocked"))
            self.assertTrue(event["message"])
            self.assertTrue(event["next_step"])
            self.assertTrue(all(label.text() == event["message"]
                                for label in self.window._feedback_result_labels[scope]))
        self.assertEqual(self.commands(), [])

    def test_invalid_protection_field_is_marked_locally_and_editing_clears_it_without_layout_shift(self):
        self.connect_fake()
        self.window.navigation.setCurrentRow(2)
        self.window.motor_tabs.setCurrentIndex(1)
        self.window.protection_tabs.setCurrentIndex(0)
        APPLICATION.processEvents()
        entries = self.controller._protection_entries["STALL"]
        key = "CURRENT_A" if "CURRENT_A" in entries else next(iter(entries))
        edit = entries[key].widget
        other = next(adapter.widget for parameter, adapter in entries.items() if parameter != key)
        geometry = (self.window.pages.geometry(), self.window.operation_panel.geometry(),
                    edit.geometry(), other.geometry())
        edit.setText("invalid-value")
        self.controller._set_protection("STALL", key)
        event = self.window.action_feedback_notice.current_event
        self.assertEqual((event["scope"], event["phase"]), ("protection", "invalid"))
        self.assertTrue(edit.property("feedbackInvalid"))
        self.assertFalse(bool(other.property("feedbackInvalid")))
        self.assertEqual((self.window.pages.geometry(), self.window.operation_panel.geometry(),
                          edit.geometry(), other.geometry()), geometry)
        edit.setFocus()
        edit.selectAll()
        QTest.keyClicks(edit, "1")
        APPLICATION.processEvents()
        self.assertFalse(edit.property("feedbackInvalid"))
        self.assertEqual((self.window.pages.geometry(), self.window.operation_panel.geometry(),
                          edit.geometry(), other.geometry()), geometry)
        self.assertEqual(self.commands(), [])

    def test_feedback_does_not_disable_or_click_unrelated_controls(self):
        self.connect_fake()
        self.window.navigation.setCurrentRow(2)
        controls = (self.window.movej_send_button, self.window.movel_send_button,
                    self.window.stop_button, self.window.motor_tabs, self.window.navigation)
        before = [widget.isEnabled() for widget in controls]
        calls = []
        self.window.stop_button.clicked.connect(lambda: calls.append("stop"))
        self.notify("motor", "pending", "正在应用电机参数…")
        self.notify("motor", "error", "设备拒绝应用。")
        APPLICATION.processEvents()
        self.assertEqual([widget.isEnabled() for widget in controls], before)
        self.assertEqual(calls, [])
        self.window.navigation.setCurrentRow(3)
        self.assertEqual(self.window.pages.currentIndex(), 3)
        self.assertEqual(self.commands(), [])

    def test_protection_integer_time_and_strict_positive_boundaries_mark_only_rejected_input(self):
        self.connect_fake()
        self.window.navigation.setCurrentRow(2)
        self.window.motor_tabs.setCurrentIndex(1)
        for kind, key, rejected, corrected in (("STALL", "STARTUP_MS", "1.5", "2"),
                                                ("CAL", "CURRENT_A", "0", "1")):
            with self.subTest(kind=kind, key=key):
                self.window.protection_tabs.setCurrentIndex(0 if kind == "STALL" else 1)
                APPLICATION.processEvents()
                entries = self.controller._protection_entries[kind]
                edit = entries[key].widget
                other = next(adapter.widget for parameter, adapter in entries.items() if parameter != key)
                before = (self.window.operation_panel.geometry(), self.window.pages.geometry(),
                          edit.geometry(), other.geometry())
                edit.setText(rejected)
                self.controller._set_protection(kind, key)
                event = self.window.action_feedback_notice.current_event
                self.assertEqual(event["phase"], "invalid")
                self.assertEqual(event["scope"], "protection" if kind == "STALL" else "calibration")
                self.assertTrue(edit.property("feedbackInvalid"))
                self.assertFalse(bool(other.property("feedbackInvalid")))
                self.assertEqual((self.window.operation_panel.geometry(), self.window.pages.geometry(),
                                  edit.geometry(), other.geometry()), before)
                edit.setFocus()
                edit.selectAll()
                QTest.keyClicks(edit, corrected)
                QApplication.processEvents()
                self.assertFalse(edit.property("feedbackInvalid"))
                self.assertEqual((self.window.operation_panel.geometry(), self.window.pages.geometry(),
                                  edit.geometry(), other.geometry()), before)
                self.assertEqual(self.commands(), [])


if __name__ == "__main__":
    unittest.main()
