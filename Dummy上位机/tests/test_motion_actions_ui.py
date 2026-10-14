"""Real Qt actions for local saving, immediate presets and numbered completion."""
from copy import deepcopy
import json
from pathlib import Path
import queue
import tempfile
import time
import unittest
from unittest.mock import patch

from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QMessageBox, QPushButton

from tests.test_ui import APPLICATION, FakeView
from tests.test_controller import FakePort, ManualScheduler
from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow


JOINTS = [25, -40, 120, 15, 20, -10, 99.5]
POSE = [500, -400, 700, 200, -190, 250]
ACTUAL = [10, -60, 160, 5, 6, 7, 20]


class MotionActionUiTests(unittest.TestCase):
    def setUp(self):
        base = Path(__file__).resolve().parents[1] / "build" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=base)
        self.store = AppStore(self.directory.name)
        self.store.update_settings(auto_poll=False)
        self.scheduler = ManualScheduler()
        self.controller = RobotController(scheduler=self.scheduler)
        self.viewer_patch = patch("app.ui.main_window.RobotView", FakeView)
        self.viewer_patch.start()
        self.warning_patch = patch("app.ui.main_window.QMessageBox.warning",
                                   return_value=QMessageBox.StandardButton.Ok)
        self.warnings = self.warning_patch.start()
        self.window = MainWindow(self.controller, self.store)
        # Isolate motion command assertions from the independent RGB monitor.
        self.window.rgb_preview_source.setCurrentIndex(0)
        self.window.resize(1366, 768)
        self.window.show()
        APPLICATION.processEvents()

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        APPLICATION.processEvents()
        self.warning_patch.stop()
        self.viewer_patch.stop()
        self.directory.cleanup()

    def connect_fake(self):
        self.controller.is_connected = True
        self.controller.serial_port = FakePort()
        self.controller.current_cmdmode = 3

    def commands(self):
        result = []
        while True:
            try:
                item = self.controller._tx_queue.get_nowait()
                if item is not None:
                    result.append(item[1])
            except queue.Empty:
                return result

    def settle(self):
        QTest.qWait(180)
        self.scheduler.run_ready()
        APPLICATION.processEvents()

    def click_save(self):
        self.assertEqual(self.window.navigation.currentRow(), 0)
        self.assertTrue(self.window.quick_points_panel.isAncestorOf(self.window.quick_point_save))
        QTest.mouseClick(self.window.quick_point_save, Qt.MouseButton.LeftButton)

    def seed(self, count=1):
        records = [{"name": f"已有点-{index + 1}", "type": "MoveJ", "joints": JOINTS,
                    "speed": 35, "gripper": 100} for index in range(count)]
        self.store.save_points(records)
        self.window.points = deepcopy(self.store.points)
        self.window._restore_points()

    def edit_motion(self, kind, viewer_mode):
        w = self.window
        w.navigation.setCurrentRow(0)
        w.motion_tabs.setCurrentIndex(0 if kind == "MoveJ" else 1)
        # The inactive teaching editor deliberately differs from the homepage.
        w.teaching_tabs.setCurrentIndex(1 if kind == "MoveJ" else 0)
        w.teaching_joint_spins[0].setValue(-30)
        w.teaching_pose_spins[0].setValue(-500)
        w.teaching_speed.setValue(80)
        w.teaching_gripper_spin.setValue(12)
        for spin, value in zip(w.joint_spins, JOINTS):
            spin.setValue(value)
        for spin, value in zip(w.pose_spins, POSE):
            spin.setValue(value)
        w.j_speed.setValue(31)
        w.l_speed.setValue(57)
        w.motion_hand_row.spin.setValue(63 if kind == "MoveJ" else 42)
        w.viewer_mode.setCurrentIndex(viewer_mode)
        APPLICATION.processEvents()

    def target_snapshot(self):
        w = self.window
        return {
            "joints": [spin.value() for spin in w.joint_spins],
            "pose": [spin.value() for spin in w.pose_spins],
            "speed": (w.j_speed.value(), w.l_speed.value()),
            "gripper": w.motion_hand_row.spin.value(),
            "page": w.navigation.currentRow(), "tab": w.motion_tabs.currentIndex(),
            "viewer_mode": w.viewer_mode.currentIndex(), "mode": w.viewer.mode,
            "target": deepcopy(w.viewer.target), "marker": deepcopy(w.viewer.cartesian),
        }

    def assert_saved(self, kind, name, before):
        w = self.window
        record = {"name": name, "type": kind, "gripper": 63 if kind == "MoveJ" else 42,
                  "speed": 31 if kind == "MoveJ" else 57,
                  "joints" if kind == "MoveJ" else "pose": JOINTS if kind == "MoveJ" else POSE}
        self.assertEqual(w.points[-1], record)
        self.assertEqual(self.store.points, w.points)
        saved = json.loads((self.store.base_dir / "points.json").read_text(encoding="utf-8"))
        self.assertEqual(saved, {"schema_version": 3, "points": w.points})
        self.assertEqual(self.target_snapshot(), before)
        self.assertEqual(w.teaching_tabs.currentIndex(), 0 if kind == "MoveJ" else 1)
        teaching = w.teaching_joint_spins if kind == "MoveJ" else w.teaching_pose_spins
        self.assertEqual([spin.value() for spin in teaching], record.get("joints", record.get("pose")))
        self.assertEqual(w.teaching_speed.value(), record["speed"])
        self.assertEqual(w.teaching_gripper_spin.value(), record["gripper"])
        self.assertEqual(w._selected_row(), len(w.points) - 1)
        for button, point in zip(w.quick_point_buttons, w.points[:len(w.quick_point_buttons)]):
            self.assertTrue(button.isEnabled())
            self.assertIn(point["name"], button.accessibleName())
            self.assertIn(point["type"], button.accessibleName())
        self.assertEqual(self.commands(), [])
        self.settle()
        self.assertEqual(self.commands(), [])

    def test_homepage_save_movej_uses_motion_fields_and_keeps_actual_view(self):
        self.connect_fake()
        self.edit_motion("MoveJ", viewer_mode=1)
        before = self.target_snapshot()
        with patch("app.ui.main_window.QInputDialog.getText", return_value=("  取料起点  ", True)) as naming:
            self.click_save()
        self.assertEqual(naming.call_count, 1)
        self.assertIn("MoveJ", naming.call_args.args[2])
        self.assert_saved("MoveJ", "取料起点", before)
        self.warnings.assert_not_called()

    def test_homepage_save_movel_preserves_type_and_current_marker_preview(self):
        self.connect_fake()
        self.edit_motion("MoveL", viewer_mode=0)
        before = self.target_snapshot()
        with patch("app.ui.main_window.QInputDialog.getText", return_value=("末端放料", True)) as naming:
            self.click_save()
        self.assertIn("MoveL", naming.call_args.args[2])
        self.assert_saved("MoveL", "末端放料", before)
        self.assertNotIn("joints", self.window.points[-1])
        self.warnings.assert_not_called()

    def test_save_cancels_all_drag_before_opening_the_name_dialog(self):
        self.connect_fake()
        self.edit_motion("MoveL", viewer_mode=1)
        w, c = self.window, self.controller
        c.movej_drag_enable.set(True)
        c.movel_drag_enable.set(True)
        c.hand_pos_drag_enable.set(True)
        w.teaching_drag.setChecked(True)
        for kind in w._last_drag:
            w._last_drag[kind] = time.monotonic()
        w.joint_spins[0].setValue(40)
        w.pose_spins[0].setValue(510)
        w.motion_hand_row.spin.setValue(55)
        w.teaching_joint_spins[0].setValue(15)
        w._teaching_sync = ("MoveJ", c._tx_generation)
        self.assertEqual(set(w._drag_pending), {"j", "l", "hand", "teaching"})
        before = self.target_snapshot()

        def naming(*_args):
            self.assertEqual(w._drag_pending, {})
            self.assertTrue(all(not timer.isActive() for timer in w._drag_timers.values()))
            self.assertIsNone(w._teaching_sync)
            self.settle()  # A modal dialog must not let an old queued target escape.
            self.assertEqual(self.commands(), [])
            return "", False

        with patch("app.ui.main_window.QInputDialog.getText", side_effect=naming):
            self.click_save()
        self.assertEqual(w.points, [])
        self.assertEqual(self.target_snapshot(), before)
        self.assertTrue(c.movej_drag_enable.get())
        self.assertTrue(c.movel_drag_enable.get())
        self.assertTrue(c.hand_pos_drag_enable.get())
        self.assertTrue(w.teaching_drag.isChecked())
        self.assertEqual(self.commands(), [])

    def test_cancel_and_invalid_names_preserve_saved_memory_and_disk(self):
        self.seed()
        self.edit_motion("MoveJ", viewer_mode=1)
        before = deepcopy(self.window.points)
        disk_before = (self.store.base_dir / "points.json").read_bytes()
        target_before = self.target_snapshot()
        log_start = len(self.window._logs)
        for name, accepted in (("不应保存", False), (" \t ", True), ("超" * 121, True)):
            with self.subTest(name=name, accepted=accepted), \
                 patch("app.ui.main_window.QInputDialog.getText", return_value=(name, accepted)):
                self.click_save()
            self.assertEqual(self.window.points, before)
            self.assertEqual(self.store.points, before)
            self.assertEqual((self.store.base_dir / "points.json").read_bytes(), disk_before)
            self.assertEqual(self.target_snapshot(), target_before)
            self.assertEqual(self.window.point_table.rowCount(), len(before))
            self.assertEqual(self.commands(), [])
        self.assertEqual(self.warnings.call_count, 2)
        self.assertFalse(any("已保存点位" in message for _, message in list(self.window._logs)[log_start:]))

    def test_120_character_name_is_accepted_after_trimming(self):
        self.edit_motion("MoveJ", viewer_mode=0)
        before = self.target_snapshot()
        name = "边" * 120
        with patch("app.ui.main_window.QInputDialog.getText", return_value=(f"  {name}  ", True)):
            self.click_save()
        self.assert_saved("MoveJ", name, before)
        self.warnings.assert_not_called()

    def test_disk_failure_keeps_saved_points_and_reports_failure_without_success(self):
        self.connect_fake()
        self.seed()
        self.edit_motion("MoveL", viewer_mode=1)
        before = deepcopy(self.window.points)
        target_before = self.target_snapshot()
        disk_before = (self.store.base_dir / "points.json").read_bytes()
        log_start = len(self.window._logs)
        # Exercise an actual atomic-save failure before AppStore replaces either
        # the persistent file or its in-memory list.
        with patch("app.ui.main_window.QInputDialog.getText", return_value=("应失败的点", True)), \
             patch("app.storage.store.os.replace", side_effect=OSError("test disk unavailable")):
            self.click_save()
        self.assertEqual(self.window.points, before)
        self.assertEqual(self.store.points, before)
        self.assertEqual((self.store.base_dir / "points.json").read_bytes(), disk_before)
        self.assertEqual(self.target_snapshot(), target_before)
        messages = list(self.window._logs)[log_start:]
        self.assertTrue(any(tag == "ERROR" and "保存失败" in message for tag, message in messages))
        self.assertFalse(any("已保存点位" in message for _, message in messages))
        self.warnings.assert_called_once()
        self.assertEqual(self.window.point_table.rowCount(), len(before))
        self.assertEqual(self.commands(), [])
        self.settle()
        self.assertEqual(self.commands(), [])

    def test_new_fifth_point_is_selected_without_replacing_the_first_four_shortcuts(self):
        self.seed(4)
        self.edit_motion("MoveJ", viewer_mode=1)
        before = self.target_snapshot()
        names = [button.accessibleName() for button in self.window.quick_point_buttons]
        with patch("app.ui.main_window.QInputDialog.getText", return_value=("新第五点", True)):
            self.click_save()
        save_notice = self.window.statusBar().currentMessage()
        self.assert_saved("MoveJ", "新第五点", before)
        self.assertEqual(len(self.window.points), 5)
        current_names = [button.accessibleName() for button in self.window.quick_point_buttons]
        self.assertEqual(current_names[:4], names[:4])
        self.assertIn("新第五点", current_names[4])
        self.assertEqual(current_names[5:], names[5:])
        # Five saved points fit the existing eight-slot shortcut area.
        self.assertNotIn("快捷区显示列表前", save_notice)

    def test_terminal_home_reset_preview_immediately_and_cancel_old_drag(self):
        self.connect_fake()
        w, c = self.window, self.controller
        c._process_rx_line("ok " + " ".join(map(str, ACTUAL)))
        for kind, target in (("HOME", [0, 0, 90, 0, 0, 0, 0]),
                             ("RESET", [0, -75, 180, 0, 0, 0, 0])):
            with self.subTest(kind=kind):
                self.edit_motion("MoveL", viewer_mode=1)
                c.movej_drag_enable.set(True)
                w._last_drag["j"] = time.monotonic()
                w.joint_spins[0].setValue(44)
                self.assertIn("j", w._drag_pending)
                w._teaching_sync = ("MoveJ", c._tx_generation)
                # Terminal/custom commands use controller dispatch directly.
                self.assertTrue(c.send_cmd("!" + kind))
                self.assertEqual(self.commands(), ["!" + kind])
                self.assertEqual(w.viewer.target[0], target)
                self.assertEqual(w.viewer.actual, (ACTUAL, None))
                self.assertEqual(w.viewer_mode.currentIndex(), 0)
                self.assertEqual(w._drag_pending, {})
                self.assertIsNone(w._teaching_sync)
                self.assertFalse(c._suppress_drag_send)
                self.assertNotIn("已完成", w.statusBar().currentMessage())
                self.settle()
                self.assertEqual(self.commands(), [])
                c.movej_drag_enable.set(False)

    def test_locally_rejected_presets_do_not_change_the_preview(self):
        self.connect_fake()
        self.edit_motion("MoveL", viewer_mode=1)
        before = self.target_snapshot()
        for kind in ("HOME", "RESET"):
            self.controller._stop_locked = True
            self.assertFalse(self.controller.send_cmd("!" + kind))
            self.controller._stop_locked = False
            self.assertEqual(self.target_snapshot(), before)
            with patch.object(self.controller._tx_queue, "put", side_effect=IOError("fake queue failure")):
                self.assertFalse(self.controller.send_cmd("!" + kind))
            self.assertEqual(self.target_snapshot(), before)
            self.assertEqual(self.commands(), [])

    def test_home_reset_dispatch_and_completion_update_both_editors_without_fake_telemetry(self):
        self.connect_fake()
        w, c = self.window, self.controller
        c._process_rx_line("ok " + " ".join(map(str, ACTUAL)))
        actual_before = deepcopy(c.state()["actual_pose"])
        viewer_actual_before = deepcopy(w.viewer.actual)
        readbacks_before = [row.readback.text() for row in w.joint_rows]
        for command, target, page, receipt_id in (
                ("HOME", [0, 0, 90, 0, 0, 0, 0], 0, 101),
                ("RESET", [0, -75, 180, 0, 0, 0, 0], 1, 102)):
            with self.subTest(command=command):
                w.navigation.setCurrentRow(0)
                w.motion_tabs.setCurrentIndex(1)
                w.teaching_tabs.setCurrentIndex(1)
                w.joint_spins[0].setValue(30)
                w.teaching_joint_spins[0].setValue(-30)
                w.pose_spins[0].setValue(250)
                w.teaching_pose_spins[0].setValue(350)
                w.j_speed.setValue(31)
                w.l_speed.setValue(57)
                w.teaching_speed.setValue(62)
                w.motion_hand_row.spin.setValue(55)
                w.teaching_gripper_spin.setValue(88)
                w.navigation.setCurrentRow(page)
                w.viewer_mode.setCurrentIndex(1)
                self.assertIsNotNone(w.viewer.cartesian)
                system_button = next(button for button in w.findChildren(QPushButton)
                                     if button.text() == ("回零 HOME" if command == "HOME" else "休息 REST"))
                QTest.mouseClick(system_button, Qt.MouseButton.LeftButton)
                self.assertEqual(self.commands(), [f"!{command}"])
                self.assertEqual([spin.value() for spin in w.joint_spins], target)
                self.assertEqual([spin.value() for spin in w.teaching_joint_spins], target)
                self.assertEqual(w.viewer.target[0], target)
                self.assertEqual(w.viewer_mode.currentIndex(), 0)
                self.assertIsNone(w.viewer.cartesian)
                self.assertIn("等待设备执行", w.statusBar().currentMessage())
                self.assertEqual(w.viewer.actual, viewer_actual_before)
                self.assertEqual([row.readback.text() for row in w.joint_rows], readbacks_before)
                c._process_rx_line(f"queued {command} id={receipt_id} mode=TRJ")
                c._process_rx_line("Home pending" if command == "HOME" else "Reset pending")
                c._process_rx_line("ok")
                c._process_rx_line(f"ok {command} id={receipt_id - 1} ms=400")
                wrong_type = "RESET" if command == "HOME" else "HOME"
                c._process_rx_line(f"ok {wrong_type} id={receipt_id} ms=400")
                self.assertEqual([spin.value() for spin in w.joint_spins], target)
                self.assertEqual(w.viewer_mode.currentIndex(), 0)

                # Local edits queued while waiting must be cancelled when the
                # acknowledged preset becomes the display target.
                w.motion_tabs.setCurrentIndex(1)
                w.teaching_tabs.setCurrentIndex(1)
                c.movej_drag_enable.set(True)
                c.movel_drag_enable.set(True)
                c.hand_pos_drag_enable.set(True)
                w.teaching_drag.setChecked(True)
                for kind in w._last_drag:
                    w._last_drag[kind] = time.monotonic()
                w.joint_spins[0].setValue(44)
                w.pose_spins[0].setValue(260)
                w.motion_hand_row.spin.setValue(54)
                w.teaching_pose_spins[0].setValue(360)
                w._teaching_sync = ("MoveL", c._tx_generation)
                self.assertEqual(set(w._drag_pending), {"j", "l", "hand", "teaching"})
                speed_gripper_before = (w.j_speed.value(), w.l_speed.value(), w.teaching_speed.value(),
                                        w.hand_row.spin.value(), w.motion_hand_row.spin.value(),
                                        w.teaching_gripper_spin.value())
                c._process_rx_line(f"ok {command} id={receipt_id} ms=400")
                self.scheduler.run_ready()
                APPLICATION.processEvents()

                self.assertEqual([spin.value() for spin in w.joint_spins], target)
                self.assertEqual([spin.value() for spin in w.teaching_joint_spins], target)
                self.assertEqual(w.motion_tabs.currentIndex(), 0)
                self.assertEqual(w.teaching_tabs.currentIndex(), 0)
                self.assertEqual(w.navigation.currentRow(), page)
                self.assertEqual(w.viewer_mode.currentIndex(), 0)
                self.assertEqual(w.viewer.mode, "preview")
                self.assertEqual(w.viewer.target[0], target)
                self.assertEqual(w.viewer.target[1], 54 if page == 0 else 88)
                self.assertIsNone(w.viewer.cartesian)
                self.assertEqual((w.j_speed.value(), w.l_speed.value(), w.teaching_speed.value(),
                                  w.hand_row.spin.value(), w.motion_hand_row.spin.value(),
                                  w.teaching_gripper_spin.value()), speed_gripper_before)
                self.assertEqual(c.state()["actual_pose"], actual_before)
                self.assertEqual(w.viewer.actual, viewer_actual_before)
                self.assertEqual([row.readback.text() for row in w.joint_rows], readbacks_before)
                self.assertEqual(w._drag_pending, {})
                self.assertTrue(all(not timer.isActive() for timer in w._drag_timers.values()))
                self.assertIsNone(w._teaching_sync)
                self.assertFalse(c._suppress_drag_send)
                self.assertEqual(self.commands(), [])
                self.settle()
                self.assertEqual(self.commands(), [])
                c.movej_drag_enable.set(False)
                c.movel_drag_enable.set(False)
                c.hand_pos_drag_enable.set(False)
                w.teaching_drag.setChecked(False)


if __name__ == "__main__":
    unittest.main()
