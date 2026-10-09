"""Verify real Qt fields against controller behavior without opening hardware."""
from pathlib import Path
import queue
import tempfile
import unittest
from unittest.mock import patch

from PySide6.QtCore import QCoreApplication, Signal, Qt
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
from PySide6.QtWidgets import QApplication, QMessageBox, QWidget

# unittest imports every test module before executing them. Creating QApplication
# here also lets controller tests reuse it instead of a non-GUI QCoreApplication.
APPLICATION = QApplication.instance() or QApplication([])

from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow
from app.protocol.constants import CAL_PARAMETERS
from tests.test_controller import FakePort, ManualScheduler


class FakeView(QWidget):
    ready = Signal()
    status_changed = Signal(str)

    def __init__(self):
        super().__init__()
        self.target = self.actual = self.cartesian = None
        self.mode = "preview"
        self.closed = False

    def set_target_pose(self, values, gripper=100):
        self.target = (list(values), gripper)

    def set_actual_pose(self, values, gripper=None):
        self.actual = (list(values), gripper)

    def set_cartesian_target(self, values):
        self.cartesian = list(values)

    def set_follow_actual(self, follow):
        self.mode = "actual" if follow else "preview"

    def set_theme(self, theme):
        self.theme = theme

    def set_stale(self, value):
        self.stale = value

    def set_actual_known(self, value):
        self.actual_known = value

    def set_grid_visible(self, value):
        self.grid = value

    def set_axes_visible(self, value):
        self.axes = value

    def set_camera(self, _value):
        pass

    def reset_camera(self):
        pass

    def focus_joint(self, _value):
        pass

    def cleanup(self):
        self.closed = True


class UiIntegrationTests(unittest.TestCase):
    def setUp(self):
        base = Path(__file__).resolve().parents[1] / "build" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=base)
        self.store = AppStore(self.directory.name)
        self.scheduler = ManualScheduler()
        self.controller = RobotController(scheduler=self.scheduler)
        self.viewer_patch = patch("app.ui.main_window.RobotView", FakeView)
        self.viewer_patch.start()
        self.window = MainWindow(self.controller, self.store)

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        APPLICATION.processEvents()
        self.viewer_patch.stop()
        self.directory.cleanup()

    def connect_fake(self):
        self.controller.is_connected = True
        self.controller.serial_port = FakePort()

    def commands(self):
        commands = []
        while True:
            try:
                item = self.controller._tx_queue.get_nowait()
                if item is not None:
                    commands.append(item[1])
            except queue.Empty:
                return commands

    def test_launch_is_offline_and_pages_have_all_parameter_fields(self):
        self.assertFalse(self.controller.is_connected)
        self.assertIsNone(self.controller.serial_port)
        self.assertEqual(self.window.pages.count(), 7)
        self.assertEqual(len(self.window.joint_spins), 7)
        self.assertEqual(len(self.window.pose_spins), 6)
        self.assertEqual(len(self.controller._protection_entries["STALL"]), 18)
        self.assertEqual(len(self.controller._protection_entries["CAL"]), 18)
        self.assertEqual(self.commands(), [])
        for page in range(7):
            self.window.navigation.setCurrentRow(page)
            self.assertEqual(self.window.pages.currentIndex(), page)

    def test_readback_keeps_target_until_explicit_sync_even_with_drag_enabled(self):
        self.connect_fake()
        self.controller.current_cmdmode = 3
        self.controller.movej_drag_enable.set(True)
        original = [spin.value() for spin in self.window.joint_spins]
        actual = [10, -60, 160, 5, 6, 7, 20]
        self.controller._process_rx_line("ok " + " ".join(map(str, actual)))
        self.assertEqual([spin.value() for spin in self.window.joint_spins], original)
        self.assertEqual(self.window.viewer.actual, (actual, None))
        self.assertEqual(self.commands(), [])
        self.controller.read_and_sync_from_robot()
        self.assertEqual(self.commands(), ["#GETJPOS"])
        self.controller._process_rx_line("ok " + " ".join(map(str, actual)))
        self.scheduler.run_ready()
        self.assertEqual([spin.value() for spin in self.window.joint_spins], actual)
        self.assertEqual(self.commands(), [])

    def test_uint32_pid_readback_and_all_node_write_disable(self):
        self.connect_fake()
        self.controller._process_rx_line("ok PID [1] kp=4294967295 kv=2147483648 ki=0 kd=10")
        self.scheduler.run_ready()
        self.assertEqual(self.controller.ent_set_pid_kp.get(), 4294967295)
        self.assertEqual(self.controller.ent_set_pid_kv.get(), 2147483648)
        self.controller._process_rx_line("ok PID [1] kp=-1 kv=-2147483648 ki=0 kd=10")
        self.scheduler.run_ready()
        self.assertEqual(self.controller.ent_set_pid_kp.get(), 4294967295)
        self.assertEqual(self.controller.ent_set_pid_kv.get(), 2147483648)
        self.controller._set_pid()
        self.assertEqual(self.commands(), ["#SET_PID 1 4294967295 2147483648 0 10"])
        self.controller.param_node.set(0)
        self.assertTrue(all(not field.widget.isEnabled() for field in self.controller._write_buttons))

    def test_cartesian_sync_retains_values_beyond_old_slider_bounds(self):
        self.connect_fake()
        self.controller.current_cmdmode = 3
        self.controller.movel_drag_enable.set(True)
        actual = [500, -400, 700, 200, -190, 250]
        self.controller.read_pose_from_robot()
        self.assertEqual(self.commands(), ["#GETLPOS"])
        self.controller._process_rx_line("ok " + " ".join(map(str, actual)))
        self.scheduler.run_ready()
        self.assertEqual([spin.value() for spin in self.window.pose_spins], actual)
        self.assertEqual(self.commands(), [])

    def test_new_connection_does_not_show_previous_devices_actual_pose(self):
        self.controller._process_rx_line("ok 10 -60 160 5 6 7 20")
        self.assertTrue(self.window.viewer.actual_known)
        self.assertNotEqual(self.window.joint_rows[0].readback.text(), "—")
        self.window._update_state({"connected": True, "actual_pose": None, "actual_stale": True})
        self.assertFalse(self.window.viewer.actual_known)
        self.assertTrue(all(row.readback.text() == "—" for row in self.window.joint_rows))

    def test_protection_query_fills_ui_without_polling_or_false_stop(self):
        self.connect_fake()
        self.controller._query_protection("CAL")
        self.assertEqual(self.commands(), ["#CAL_GET 7"])
        self.assertTrue(self.controller.state()["parameter_busy"])
        self.controller._poll_position()
        self.assertEqual(self.commands(), [])
        self.controller._process_rx_line("ok CAL_GET pending")
        for key, _label, _unit, default, _low, _high in CAL_PARAMETERS:
            value = 0.75 if key == "SEEK_RPS" else default
            self.controller._process_rx_line(f"ok CAL [7] {key}={value:g}")
        self.assertEqual(self.controller._protection_entries["CAL"]["SEEK_RPS"].get(), "0.75")
        self.assertFalse(self.controller.state()["parameter_busy"])
        self.assertFalse(self.controller._stop_locked)
        self.controller._poll_position()
        self.assertEqual(self.commands(), ["#GETJPOS"])

    def test_selecting_stored_point_does_not_send_and_order_persists(self):
        self.connect_fake()
        self.controller.current_cmdmode = 3
        self.window.point_name.setText("起始")
        self.window._save_point()
        self.window.joint_spins[0].setValue(20)
        self.window.point_name.setText("取料")
        self.window._save_point()
        self.controller.movej_drag_enable.set(True)
        self.controller.hand_pos_drag_enable.set(True)
        self.window.point_table.selectRow(0)
        self.assertEqual(self.window.joint_spins[0].value(), 0)
        self.assertEqual(self.commands(), [])
        self.window._move_point(1)
        self.assertEqual([point["name"] for point in self.store.points], ["取料", "起始"])
        reloaded = AppStore(self.directory.name)
        try:
            self.assertEqual(reloaded.points, self.store.points)
        finally:
            reloaded.close()

    def test_cancel_import_preserves_memory_and_disk(self):
        self.window.point_name.setText("保留点")
        self.window._save_point()
        before = (self.store.base_dir / "points.json").read_bytes()
        imported = Path(self.directory.name) / "incoming.json"
        self.store.export_points(imported, [{"name": "新点", "joints": [1,-75,180,0,0,0,0], "gripper": 50, "speed": 35}])
        with patch("app.ui.main_window.QFileDialog.getOpenFileName", return_value=(str(imported), "")), patch("app.ui.main_window.QMessageBox.question", return_value=QMessageBox.StandardButton.No):
            self.window._import_points()
        self.assertEqual(self.window.points[0]["name"], "保留点")
        self.assertEqual((self.store.base_dir / "points.json").read_bytes(), before)

    def test_hand_integer_format_and_rgb_query_fill(self):
        self.connect_fake()
        self.controller._update_motor_state(False)
        self.window.hand_advanced_fields["KP"].setValue(2147483647)
        self.window._hand_config("KP")
        self.assertEqual(self.commands(), ["!HAND_KP 2147483647"])
        self.controller._process_rx_line("RGB_COLOR 0 12 34 56")
        self.controller._process_rx_line("RGB_GET end")
        self.assertEqual([int(field.get()) for field in self.controller._rgb_color_vars], [12,34,56])
        self.controller._rgb_apply_color()
        self.assertEqual(self.commands(), ["!RGB_COLOR 0 12 34 56"])

    def test_display_and_poll_settings_do_not_dispatch_commands(self):
        self.window.theme_combo.setCurrentIndex(1)
        self.window.poll_hz.setValue(10)
        self.window.poll_enabled.setChecked(False)
        self.window.grid_check.setChecked(False)
        self.window.axes_check.setChecked(False)
        self.assertEqual(self.store.settings["theme"], "dark")
        self.assertEqual(self.controller._poll_hz, 10)
        self.assertFalse(self.controller._poll_enabled)
        self.assertFalse(self.window.viewer.grid)
        self.assertFalse(self.window.viewer.axes)
        self.assertEqual(self.commands(), [])


if __name__ == "__main__":
    unittest.main()
