"""Real Qt regression checks for loading taught targets without sending motion."""
from copy import deepcopy
import json
from pathlib import Path
import queue
import tempfile
import time
import unittest
from unittest.mock import patch

from PySide6.QtTest import QTest
from PySide6.QtWidgets import QMessageBox

from tests.test_ui import APPLICATION, FakeView
from tests.test_controller import FakePort, ManualScheduler
from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow


JOINT_POINT = {
    "name": "取料起点", "type": "MoveJ",
    "joints": [25, -40, 120, 15, 20, -10, 99.5],
    "gripper": 63, "speed": 31,
}
POSE_POINT = {
    "name": "末端放料", "type": "MoveL",
    "pose": [500, -400, 700, 200, -190, 250],
    "gripper": 42, "speed": 57,
}


class QuickPointUiTests(unittest.TestCase):
    def setUp(self):
        base = Path(__file__).resolve().parents[1] / "build" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=base)
        self.viewer_patch = patch("app.ui.main_window.RobotView", FakeView)
        self.viewer_patch.start()
        self._open_window()

    def _open_window(self):
        self.store = AppStore(self.directory.name)
        self.store.update_settings(auto_poll=False)
        self.scheduler = ManualScheduler()
        self.controller = RobotController(scheduler=self.scheduler)
        self.window = MainWindow(self.controller, self.store)
        self.window.show()
        APPLICATION.processEvents()

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        APPLICATION.processEvents()
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

    def seed(self, records):
        self.store.save_points(records)
        self.window.points = deepcopy(self.store.points)
        self.window._restore_points()

    def assert_shortcuts(self, records):
        buttons = self.window.quick_point_buttons
        self.assertEqual(len(buttons), 4)
        for index, control in enumerate(buttons):
            with self.subTest(slot=index):
                self.assertTrue(self.window.quick_points_panel.isAncestorOf(control))
                if index >= len(records):
                    self.assertFalse(control.isEnabled())
                    continue
                record = records[index]
                self.assertTrue(control.isEnabled())
                # The accessible label retains the full name when visible text
                # is elided, and lets us verify what each real button presents.
                self.assertIn(record["name"], control.accessibleName())
                self.assertIn(record["type"], control.accessibleName())

    def assert_target(self, record):
        w = self.window
        joint = record["type"] == "MoveJ"
        expected = record["joints" if joint else "pose"]
        motion = w.joint_spins if joint else w.pose_spins
        teaching = w.teaching_joint_spins if joint else w.teaching_pose_spins
        self.assertEqual([spin.value() for spin in motion], expected)
        self.assertEqual([spin.value() for spin in teaching], expected)
        self.assertEqual(w.motion_tabs.currentIndex(), 0 if joint else 1)
        self.assertEqual(w.teaching_tabs.currentIndex(), 0 if joint else 1)
        self.assertEqual((w.j_speed if joint else w.l_speed).value(), record["speed"])
        self.assertEqual(w.teaching_speed.value(), record["speed"])
        self.assertEqual(w.hand_row.spin.value(), record["gripper"])
        self.assertEqual(w.motion_hand_row.spin.value(), record["gripper"])
        self.assertEqual(w.teaching_gripper_spin.value(), record["gripper"])
        self.assertEqual(w.viewer.target[1], record["gripper"])
        self.assertEqual(w.viewer.cartesian, None if joint else expected)
        self.assertEqual(w.navigation.currentRow(), 0)
        self.assertEqual(w.viewer_mode.currentIndex(), 0)
        self.assertEqual(w.viewer.mode, "preview")

    def test_loading_cancels_all_pending_drag_and_sync_without_transmitting(self):
        self.seed([JOINT_POINT, POSE_POINT])
        self.connect_fake()
        w, c = self.window, self.controller
        c.movej_drag_enable.set(True)
        c.movel_drag_enable.set(True)
        c.hand_pos_drag_enable.set(True)
        w.teaching_drag.setChecked(True)
        for kind in w._last_drag:
            w._last_drag[kind] = time.monotonic()
        w.joint_spins[0].setValue(10)
        w.pose_spins[0].setValue(150)
        w.motion_hand_row.spin.setValue(50)
        w.teaching_joint_spins[0].setValue(15)
        self.assertEqual(set(w._drag_pending), {"j", "l", "hand", "teaching"})
        self.assertTrue(all(timer.isActive() for timer in w._drag_timers.values()))
        w._teaching_sync = ("MoveJ", c._tx_generation)

        w.quick_point_buttons[1].click()

        self.assertEqual(w._drag_pending, {})
        self.assertTrue(all(not timer.isActive() for timer in w._drag_timers.values()))
        self.assertIsNone(w._teaching_sync)
        self.assertFalse(c._suppress_drag_send)
        self.assertTrue(c.movej_drag_enable.get())
        self.assertTrue(c.movel_drag_enable.get())
        self.assertTrue(c.hand_pos_drag_enable.get())
        self.assertTrue(w.teaching_drag.isChecked())
        self.assert_target(POSE_POINT)
        self.assertEqual(self.commands(), [])
        QTest.qWait(180)
        self.scheduler.run_ready()
        self.assertEqual(self.commands(), [])

    def test_movej_movel_load_both_editors_speed_gripper_and_preserve_rail(self):
        self.seed([JOINT_POINT, POSE_POINT])
        self.connect_fake()
        self.window.viewer_mode.setCurrentIndex(1)
        self.window.quick_point_buttons[0].click()
        self.assert_target(JOINT_POINT)
        self.assertEqual(self.window._selected_row(), 0)

        self.window.viewer_mode.setCurrentIndex(1)
        self.window.quick_point_buttons[1].click()
        self.assert_target(POSE_POINT)
        self.assertEqual(self.window._selected_row(), 1)
        self.assertEqual(self.window.joint_spins[6].value(), JOINT_POINT["joints"][6])
        self.assertEqual(self.window.teaching_joint_spins[6].value(), JOINT_POINT["joints"][6])
        self.assertEqual(self.window.j_speed.value(), JOINT_POINT["speed"])
        self.assertEqual(self.commands(), [])

    def test_clicking_already_selected_point_restores_edited_targets_again(self):
        self.seed([JOINT_POINT, POSE_POINT])
        self.connect_fake()
        for index, record in enumerate((JOINT_POINT, POSE_POINT)):
            with self.subTest(kind=record["type"]):
                w = self.window
                w.quick_point_buttons[index].click()
                self.assertEqual(w._selected_row(), index)
                joint = record["type"] == "MoveJ"
                (w.joint_spins if joint else w.pose_spins)[0].setValue(100)
                (w.teaching_joint_spins if joint else w.teaching_pose_spins)[0].setValue(110)
                w.motion_hand_row.spin.setValue(10)
                w.teaching_gripper_spin.setValue(20)
                (w.j_speed if joint else w.l_speed).setValue(90)
                w.teaching_speed.setValue(95)
                w.viewer_mode.setCurrentIndex(1)
                self.assertEqual(w._selected_row(), index)

                w.quick_point_buttons[index].click()

                self.assert_target(record)
                self.assertEqual(w._selected_row(), index)
                self.assertEqual(self.commands(), [])

    def test_first_four_refresh_after_create_rename_replace_reorder_and_delete(self):
        w = self.window
        for index in range(5):
            w.point_name.setText(f"位置-{index + 1}")
            w.teaching_joint_spins[0].setValue(index * 10)
            w._save_point()
            self.assert_shortcuts(w.points)
        self.assertEqual(len(w.points), 5)
        self.assertNotIn("位置-5", " ".join(control.accessibleName() for control in w.quick_point_buttons))

        w.point_table.item(1, 1).setText("改名后的第二点")
        self.assertEqual(w.points[1]["name"], "改名后的第二点")
        self.assert_shortcuts(w.points)

        w.point_table.selectRow(0)
        w.teaching_tabs.setCurrentIndex(1)
        for spin, value in zip(w.teaching_pose_spins, POSE_POINT["pose"]):
            spin.setValue(value)
        w.teaching_speed.setValue(65)
        w.teaching_gripper_spin.setValue(20)
        w.point_name.setText("更新为末端点")
        w._replace_point()
        self.assertEqual(w.points[0]["type"], "MoveL")
        self.assert_shortcuts(w.points)
        w.quick_point_buttons[0].click()
        self.assert_target(w.points[0])

        w.point_table.selectRow(4)
        w._move_point(-1)
        self.assertEqual([point["name"] for point in w.points[:4]],
                         ["更新为末端点", "改名后的第二点", "位置-3", "位置-5"])
        self.assert_shortcuts(w.points)
        w.point_table.selectRow(1)
        w._delete_point()
        self.assertEqual([point["name"] for point in w.points],
                         ["更新为末端点", "位置-3", "位置-5", "位置-4"])
        self.assert_shortcuts(w.points)
        self.assertEqual(self.store.points, w.points)
        self.assertEqual(self.commands(), [])

    def test_import_and_restart_restore_first_four_from_existing_point_schema(self):
        self.seed([JOINT_POINT])
        records = []
        for index in range(6):
            point = deepcopy(JOINT_POINT if index % 2 == 0 else POSE_POINT)
            point["name"] = f"导入位置-{index + 1}"
            records.append(point)
        incoming = Path(self.directory.name) / "incoming.json"
        self.store.export_points(incoming, records)
        with patch("app.ui.main_window.QFileDialog.getOpenFileName", return_value=(str(incoming), "")), \
             patch("app.ui.main_window.QMessageBox.question", return_value=QMessageBox.StandardButton.Yes):
            self.window._import_points()
        self.assertEqual(self.window.points, records)
        self.assert_shortcuts(records)
        saved = json.loads((self.store.base_dir / "points.json").read_text(encoding="utf-8"))
        self.assertEqual(set(saved), {"schema_version", "points"})
        self.assertEqual(saved["schema_version"], 3)
        self.assertEqual(saved["points"], records)

        self.window.close()
        self.window.deleteLater()
        APPLICATION.processEvents()
        self._open_window()

        self.assertFalse(self.controller.is_connected)
        self.assertEqual(self.window.points, records)
        self.assert_shortcuts(records)
        self.window.quick_point_buttons[3].click()
        self.assert_target(records[3])
        self.assertEqual(self.commands(), [])

    def test_empty_slots_and_clear_are_disabled_and_management_only_navigates(self):
        w = self.window
        self.connect_fake()
        before = [spin.value() for spin in w.joint_spins]
        self.assert_shortcuts([])
        for control in w.quick_point_buttons:
            control.click()
        self.assertEqual([spin.value() for spin in w.joint_spins], before)
        self.assertEqual(self.commands(), [])
        self.seed([JOINT_POINT, POSE_POINT])
        self.assert_shortcuts(w.points)
        w.quick_point_manage.click()
        self.assertEqual(w.navigation.currentRow(), 1)
        self.assertEqual(self.commands(), [])
        with patch("app.ui.main_window.QMessageBox.question", return_value=QMessageBox.StandardButton.Yes):
            w._clear_points()
        self.assertEqual(w.points, [])
        self.assertEqual(self.store.points, [])
        self.assert_shortcuts([])
        w.navigation.setCurrentRow(0)
        for control in w.quick_point_buttons:
            control.click()
        QTest.qWait(150)
        self.scheduler.run_ready()
        self.assertEqual(self.commands(), [])


if __name__ == "__main__":
    unittest.main()
