"""Exercise Qt handover and local configuration without SDL or hardware."""
from copy import deepcopy
from pathlib import Path
import queue
import tempfile
import unittest
from unittest.mock import patch

from PySide6.QtCore import QEvent, QObject, QPoint, Signal
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QDialog, QMessageBox

from app.core.controller import RobotController
from app.gamepad.config import default_config
from app.protocol.constants import POSE_DEFAULTS
from app.storage.store import AppStore
from app.ui.gamepad_panel import GamepadSettingsDialog
from app.ui.main_window import MainWindow
from tests.test_controller import FakePort, ManualScheduler
from tests.test_ui import APPLICATION, FakeView


class FakeReader(QObject):
    sampled = Signal(dict)
    status_changed = Signal(dict)

    def __init__(self):
        super().__init__()
        self.calls = []
        self.config = default_config()
        self.running = False

    def start(self, config=None):
        self.calls.append("start")
        self.config = deepcopy(config or self.config)
        self.running = True
        self.status_changed.emit(self.status())
        return True

    def status(self):
        return {"running": self.running, "connected": self.running,
                "name": "模拟手柄", "guid": "fake", "devices": [], "error": ""}

    def stop(self):
        self.calls.append("stop")
        self.running = False
        self.status_changed.emit(self.status())

    def close(self):
        self.calls.append("close")
        self.running = False

    def set_config(self, config):
        self.calls.append("set_config")
        self.config = deepcopy(config)


class FakeSession(QObject):
    state_changed = Signal(dict)
    target_changed = Signal(dict)
    initial_pose_completed = Signal(list)

    def __init__(self):
        super().__init__()
        self.calls, self.samples = [], []
        self.config = default_config()
        self.data = {"stage": "disconnected", "claimed": False, "ready": False,
                     "release_pending": False, "message": "未接管"}

    def state(self):
        return deepcopy(self.data)

    @property
    def claimed(self):
        return bool(self.data["claimed"])

    def disconnected(self):
        self.confirm_release()

    def controller_cancelled(self):
        pass

    def update(self, **state):
        self.data.update(state)
        self.state_changed.emit(self.state())

    def set_config(self, config):
        self.calls.append("set_config")
        self.config = deepcopy(config)
        return True

    def handle_input(self, snapshot):
        self.samples.append(snapshot)

    def handle_line(self, line):
        pass

    def claim(self):
        self.calls.append("claim")
        self.update(claimed=True, stage="connected", ready=False)
        return True

    def release(self):
        self.calls.append("release")
        self.update(release_pending=True, ready=False, stage="stopping")

    def confirm_release(self):
        self.update(claimed=False, release_pending=False, ready=False, stage="stopped")

    def start(self):
        self.calls.append("start")
        return True

    def initialize(self):
        self.calls.append("initialize")
        return True

    def resync(self):
        self.calls.append("resync")
        return True

    def rest(self):
        self.calls.append("rest")
        return True

    def stop(self):
        self.calls.append("stop")
        return True

    def disable(self):
        self.calls.append("disable")
        return True

    def set_gripper_base(self, value):
        self.calls.append(("base", value))
        return True

    def close(self):
        self.calls.append("close")


class GamepadUiTests(unittest.TestCase):
    def setUp(self):
        base = Path(__file__).resolve().parents[1] / "build" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=base)
        self.store = AppStore(self.directory.name)
        self.controller = RobotController(scheduler=ManualScheduler())
        self.session = FakeSession()
        self.controller.gamepad = self.session
        self.reader = FakeReader()
        self.viewer_patch = patch("app.ui.main_window.RobotView", FakeView)
        self.viewer_patch.start()
        self.window = MainWindow(self.controller, self.store, gamepad_reader=self.reader)

    def tearDown(self):
        self.session.confirm_release()
        self.window._control_owner = "ui"
        self.window.close()
        APPLICATION.processEvents()
        self.window.deleteLater()
        APPLICATION.processEvents()
        self.store.close()
        self.viewer_patch.stop()
        self.directory.cleanup()

    def commands(self):
        result = []
        while True:
            try:
                item = self.controller._tx_queue.get_nowait()
                if item:
                    result.append(item[1])
            except queue.Empty:
                return result

    def claim(self):
        self.window.input_mode.setCurrentIndex(1)
        self.assertTrue(self.window._gamepad_action("claim"))

    def test_default_and_input_switch_do_not_start_reader_or_robot(self):
        expected = default_config()
        self.assertEqual(self.window.gamepad_panel.tool_spin.value(), expected["robot"]["tool_offset_z"])
        angles = " / ".join(f"{value:g}" for value in expected["robot"]["init_joints"])
        self.assertIn(angles, self.window.gamepad_panel.initial_hint.text())
        self.assertIn("地轨固定 0 mm", self.window.gamepad_panel.initial_hint.text())
        self.assertIn("第二次 Start 只发一条 MoveJ", self.window.gamepad_panel.initial_hint.text())
        self.assertIn("观察实物到位后点击“确认到位”", self.window.gamepad_panel.initial_hint.text())
        self.assertIn("暂停位置回读", self.window.gamepad_panel.initial_hint.text())
        self.assertEqual([spin.value() for spin in self.window.pose_spins], POSE_DEFAULTS)
        self.window.navigation.setCurrentRow(1)
        self.window.teaching_tabs.setCurrentIndex(1)
        self.assertEqual([spin.value() for spin in self.window.teaching_pose_spins], POSE_DEFAULTS)
        self.assertEqual(self.commands(), [])
        self.window.navigation.setCurrentRow(0)
        dialog = GamepadSettingsDialog(self.window._gamepad_config, self.window)
        self.assertEqual(dialog.fields["robot.tool_offset_z"].value(), expected["robot"]["tool_offset_z"])
        self.assertEqual(dialog.result_config()["robot"]["init_joints"], expected["robot"]["init_joints"])
        for path in ("gamepad.buttons.manual", "control.feedback_interval_s", "control.feedback_timeout_s"):
            self.assertNotIn(path, dialog.fields)
        dialog.deleteLater()
        with patch("app.ui.main_window.GamepadReader") as reader_type:
            self.assertEqual(self.window.input_mode.currentIndex(), 0)
            self.window.input_mode.setCurrentIndex(1)
            self.assertIs(self.window.motion_input_stack.currentWidget(), self.window.gamepad_panel)
            self.assertEqual(self.reader.calls, [])
            self.assertNotIn("claim", self.session.calls)
            self.assertEqual(self.commands(), [])
            reader_type.assert_not_called()

    def test_inspection_preserves_timestamp_and_stopped_samples_are_ignored(self):
        self.window._gamepad_action("check")
        sample = {"connected": True, "timestamp": 123.4, "decoded": {"centered": True},
                  "raw": {"axes": [.1], "buttons": [False], "hats": [(0, 0)]}, "pressed": []}
        self.reader.sampled.emit(sample)
        self.assertEqual(self.session.samples[-1]["timestamp"], 123.4)
        self.assertIn("+0.100", self.window.gamepad_panel.raw_label.text())
        self.assertEqual(self.commands(), [])
        self.window._gamepad_action("check")
        count = len(self.session.samples)
        self.reader.sampled.emit(sample)
        self.assertEqual(len(self.session.samples), count)
        self.reader.sampled.emit({"connected": False})
        self.assertEqual(len(self.session.samples), count + 1)

    def test_claim_cancels_drag_and_keeps_conflicts_locked_until_confirmation(self):
        self.controller.current_cmdmode = 3
        self.window.j_drag.setChecked(True)
        self.claim()
        self.assertFalse(self.window.j_drag.isChecked())
        self.assertFalse(self.window.motion_gripper_group.isEnabled())
        self.assertFalse(self.window.mode_set_button.isEnabled())
        self.assertFalse(self.window.teaching_send_button.isEnabled())
        self.assertNotIn("start", self.session.calls)
        self.window._gamepad_action("release")
        self.assertFalse(self.window.movej_send_button.isEnabled())
        self.window._suspend_gamepad("再暂停")
        self.assertEqual(self.session.calls.count("release"), 1)
        self.session.confirm_release()
        self.assertTrue(self.window.movej_send_button.isEnabled())
        self.assertFalse(self.window.j_drag.isChecked())
        self.assertEqual(self.commands(), [])

    def test_stage_actions_and_gripper_base_require_ready(self):
        self.claim()
        panel = self.window.gamepad_panel
        self.assertNotIn("resync", panel.action_buttons)
        self.assertFalse(self.window._gamepad_action("resync"))
        self.assertNotIn("resync", self.session.calls)
        self.assertTrue(panel.action_buttons["start"].isEnabled())
        self.assertFalse(panel.action_buttons["initialize"].isEnabled())
        self.session.update(stage="enabled", ready=False)
        self.assertTrue(panel.action_buttons["initialize"].isEnabled())
        self.session.update(stage="init_confirmation", ready=False)
        self.assertEqual(panel.action_buttons["initialize"].text(), "确认到位")
        self.assertTrue(panel.action_buttons["initialize"].isEnabled())
        panel.action_buttons["initialize"].click()
        self.assertIn("initialize", self.session.calls)
        self.assertFalse(panel.action_buttons["start"].isEnabled())
        self.assertFalse(panel.gripper_base_button.isEnabled())
        for stage in ("starting", "fault", "wait_idle", "joint_queued",
                      "joint_moving", "int_mode", "int_check", "centering"):
            self.session.update(stage=stage, ready=False)
            self.assertFalse(panel.action_buttons["initialize"].isEnabled())
            self.assertFalse(panel.gripper_base_button.isEnabled())
            self.assertFalse(self.window._gamepad_gripper_base(30))
        self.session.update(stage="fault", stop_confirmed=False)
        self.assertFalse(panel.action_buttons["start"].isEnabled())
        self.session.update(stop_confirmed=True)
        self.assertTrue(panel.action_buttons["start"].isEnabled())
        self.assertFalse(panel.action_buttons["initialize"].isEnabled())
        self.session.update(stage="ready", ready=True)
        self.assertTrue(panel.action_buttons["rest"].isEnabled())
        panel.gripper_base_spin.setValue(40)
        panel.gripper_base_button.click()
        self.assertIn(("base", 40), self.session.calls)
        self.window._system_command("!STOP")
        self.assertIn("stop", self.session.calls)
        self.assertFalse(self.window._system_command("!HOME"))
        self.assertEqual(self.commands(), [])

    def test_gamepad_target_and_initial_completion_never_replace_actual_feedback(self):
        original_target = deepcopy(self.window.viewer.target)
        original_joints = [spin.value() for spin in self.window.joint_spins]
        self.window.input_mode.setCurrentIndex(1)
        target = {"flange": [100, 20, 30, 0, 10, 20], "tcp": [100, 20, 190, 0, 10, 20], "gripper": None}
        opening = self.window.hand_row.spin.value()
        self.session.target_changed.emit(target)
        self.assertEqual([spin.value() for spin in self.window.pose_spins], target["flange"])
        self.assertIsNone(self.window.viewer.cartesian)
        self.assertEqual(self.window.viewer.mode, "actual")
        self.assertEqual(self.window.viewer.target, original_target)
        self.assertEqual(self.window.hand_row.spin.value(), opening)
        self.assertIsNone(self.window.viewer.actual)
        self.session.initial_pose_completed.emit([0, 10, 80, 0, 90, 0, 0])
        self.assertEqual([spin.value() for spin in self.window.joint_spins], original_joints)
        self.window._fields_updated()
        self.assertIsNone(self.window.viewer.cartesian)
        self.assertEqual(self.window.viewer.target, original_target)
        self.assertIsNone(self.window.viewer.actual)
        actual = [2, 9, 78, 3, 87, -2, 0]
        self.controller._process_rx_line("ok " + " ".join(map(str, actual)))
        self.assertEqual(self.window.viewer.actual, (actual, None))
        self.assertTrue(self.window.viewer.actual_known)
        self.session.update(stage="ready", claimed=True, ready=True)
        self.assertTrue(self.window.viewer.feedback_paused)
        self.assertIn("位置回读暂停", self.window.actual_status.text())
        self.session.target_changed.emit(dict(target, gripper=20))
        self.session.initial_pose_completed.emit([0, 10, 80, 0, 90, 0, 0])
        self.window._update_state({"actual_pose": actual, "actual_stale": True})
        self.assertEqual(self.window.hand_row.spin.value(), 20)
        self.assertEqual(self.window.viewer.target, original_target)
        self.assertEqual(self.window.viewer.actual, (actual, None))
        self.assertIsNone(self.window.viewer.cartesian)
        self.assertTrue(self.window.viewer.stale)
        self.session.confirm_release()
        self.assertFalse(self.window.viewer.feedback_paused)
        self.assertNotIn("回读暂停", self.window.actual_status.text())
        self.assertEqual(self.commands(), [])

    def test_gamepad_actual_only_is_temporary_even_before_claim(self):
        for preference in (0, 1):
            self.window.viewer_mode.setCurrentIndex(preference)
            expected = "actual" if preference else "preview"
            self.store.update_settings(viewer_mode=expected)
            self.window.input_mode.setCurrentIndex(1)
            self.assertFalse(self.session.claimed)
            self.assertTrue(self.window.viewer_mode.isHidden())
            self.assertTrue(self.window.viewer.actual_only)
            self.assertFalse(self.window.viewer.feedback_paused)
            self.assertEqual(self.window.viewer.mode, "actual")
            self.assertEqual(self.store.settings["viewer_mode"], expected)
            self.window.viewer_mode.setCurrentIndex(1 - preference)
            self.window._viewer_ready()
            self.assertEqual(self.window.viewer_mode.currentIndex(), preference)
            self.assertEqual(self.window.viewer.mode, "actual")
            self.assertEqual(self.store.settings["viewer_mode"], expected)
            self.window.input_mode.setCurrentIndex(0)
            self.assertFalse(self.window.viewer_mode.isHidden())
            self.assertFalse(self.window.viewer.actual_only)
            self.assertEqual(self.window.viewer.mode, expected)
            self.assertEqual(self.store.settings["viewer_mode"], expected)
        self.assertEqual(self.reader.calls, [])
        self.assertNotIn("claim", self.session.calls)
        self.assertEqual(self.commands(), [])

    def test_page_interface_hide_and_dialog_release_without_auto_resume(self):
        for action in (lambda: self.window.navigation.setCurrentRow(1),
                       lambda: self.window.input_mode.setCurrentIndex(0),
                       lambda: self.window.eventFilter(self.window, QEvent(QEvent.Type.WindowDeactivate)),
                       self.window._expand_log):
            self.window.navigation.setCurrentRow(0)
            self.claim()
            count = self.session.calls.count("release")
            action()
            self.assertEqual(self.session.calls.count("release"), count + 1)
            self.assertFalse(self.reader.running)
            self.session.confirm_release()
            self.window.log_dock.hide()
        self.claim()
        dialog = QDialog(self.window)
        dialog.show()
        APPLICATION.processEvents()
        self.assertTrue(self.session.data["release_pending"])
        dialog.close()
        self.session.confirm_release()
        self.assertFalse(self.reader.running)

    def test_config_save_failure_leaves_runtime_and_panel_unchanged(self):
        old = deepcopy(self.window._gamepad_config)
        candidate = deepcopy(old)
        candidate["robot"]["tool_offset_z"] = 175
        with patch.object(self.store, "update_settings", side_effect=OSError("磁盘写入失败")), \
                patch.object(QMessageBox, "warning"):
            self.assertFalse(self.window._save_gamepad_config(candidate))
        self.assertEqual(self.session.config, old)
        self.assertEqual(self.reader.config, old)
        self.assertEqual(self.window._gamepad_config, old)
        self.assertEqual(self.window.gamepad_panel.tool_spin.value(), old["robot"]["tool_offset_z"])
        self.assertTrue(self.window._save_gamepad_config(candidate))
        self.assertEqual(self.session.config["robot"]["tool_offset_z"], 175)
        self.assertEqual(self.store.settings["gamepad_config"]["robot"]["tool_offset_z"], 175)
        self.assertEqual(self.commands(), [])

    def test_settings_draft_diagnostics_do_not_feed_session(self):
        original = deepcopy(self.window._gamepad_config)
        dialog = GamepadSettingsDialog(self.window._gamepad_config, self.window)
        self.window._gamepad_settings_dialog = dialog
        dialog.fields["robot.tool_offset_z"].setValue(200)
        self.window._gamepad_dialog_diagnostic(True)
        self.reader.sampled.emit({"connected": True, "timestamp": 20, "raw": {"axes": [.3]}})
        self.assertIn("+0.300", dialog.raw_view.toPlainText())
        self.assertEqual(self.session.samples, [])
        self.assertEqual(self.window._gamepad_config, original)
        self.window._suspend_gamepad("关闭设置")
        self.window._gamepad_settings_dialog = None
        dialog.deleteLater()
        self.assertEqual(self.commands(), [])

    def test_close_waits_for_release_then_closes_reader_and_transport(self):
        self.controller.is_connected = True
        self.controller.serial_port = FakePort()
        self.claim()
        self.window.close()
        self.assertTrue(self.window._gamepad_close_pending)
        self.assertFalse(self.window._closed)
        self.assertTrue(self.controller.is_connected)
        self.assertFalse(self.reader.running)
        self.window.close()
        self.assertEqual(self.session.calls.count("release"), 1)
        self.session.confirm_release()
        APPLICATION.processEvents()
        self.assertTrue(self.window._closed)
        self.assertFalse(self.controller.is_connected)
        self.assertEqual(self.reader.calls.count("close"), 1)

    def test_close_timeout_reports_unconfirmed_stop(self):
        self.controller.is_connected = True
        self.controller.serial_port = FakePort()
        self.claim()
        self.window.close()
        self.window._gamepad_close_timer.start(1)
        QTest.qWait(80)
        APPLICATION.processEvents()
        self.assertTrue(self.window._closed)
        self.assertTrue(any("STOP 尚未确认" in text for _tag, text in self.window._logs))

    def test_minimum_window_keeps_fixed_controls_and_panel_scroll_separate(self):
        self.window.resize(1120, 690)
        self.window.show()
        for theme in ("light", "dark"):
            self.window.apply_theme(theme)
            for mode in (0, 1):
                self.window.input_mode.setCurrentIndex(mode)
                QTest.qWait(20)
                APPLICATION.processEvents()
                controls = self.window.motion_controls
                active = self.window.motion_input_stack
                grip = self.window.motion_gripper_group
                quick = self.window.quick_points_panel
                def bounds(widget):
                    point = widget.mapTo(controls, QPoint(0, 0))
                    return point.y(), point.y() + widget.height()
                self.assertLessEqual(bounds(active)[1], bounds(grip)[0])
                self.assertLessEqual(bounds(grip)[1], bounds(quick)[0])
                self.assertLessEqual(bounds(quick)[1], controls.height())
                if mode:
                    self.assertGreaterEqual(self.window.gamepad_panel.scroll.viewport().height(), 140)
                    self.assertTrue(all(control.isHidden() for control in self.window.quick_point_buttons))
                    start = self.window.gamepad_panel.action_buttons["start"]
                    self.assertFalse(start.visibleRegion().isEmpty())
                    bar = self.window.gamepad_panel.scroll.verticalScrollBar()
                    self.assertGreater(bar.maximum(), 0)
                else:
                    self.assertFalse(self.window.movej_send_button.visibleRegion().isEmpty())
                self.assertEqual(self.commands(), [])


if __name__ == "__main__":
    unittest.main()
