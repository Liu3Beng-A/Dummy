"""Verify real Qt fields against controller behavior without opening hardware."""
from pathlib import Path
import queue
import tempfile
import unittest
from unittest.mock import patch

from PySide6.QtCore import QCoreApplication, Signal, Qt
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
from PySide6.QtWidgets import QApplication, QMessageBox, QWidget
from PySide6.QtTest import QTest

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
        self.preferred_mode = "preview"
        self.actual_only = False
        self.feedback_paused = False
        self.commanded_gripper = None
        self.closed = False

    def set_target_pose(self, values, gripper=100):
        self.target = (list(values), gripper)

    def set_target_gripper(self, gripper):
        self.target = (self.target[0], gripper)

    def set_commanded_gripper(self, gripper):
        self.commanded_gripper = gripper

    def set_actual_pose(self, values, gripper=None):
        self.actual = (list(values), gripper)

    def set_cartesian_target(self, values):
        self.cartesian = list(values) if values is not None else None

    def set_follow_actual(self, follow):
        self.preferred_mode = "actual" if follow else "preview"
        self.mode = "actual" if self.actual_only else self.preferred_mode

    def set_actual_only(self, value):
        self.actual_only = bool(value)
        self.mode = "actual" if self.actual_only else self.preferred_mode

    def set_feedback_paused(self, value):
        self.feedback_paused = bool(value)

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
        self.assertEqual(self.window.pages.count(), 6)
        self.assertEqual([self.window.navigation.item(index).text() for index in range(6)],
                         ["运动控制", "点位示教", "电机设置", "夹爪设置", "RGB 灯效", "软件设置"])
        self.assertEqual(len(self.window.joint_spins), 7)
        self.assertEqual(len(self.window.pose_spins), 6)
        self.assertEqual(len(self.controller._protection_entries["STALL"]), 18)
        self.assertEqual(len(self.controller._protection_entries["CAL"]), 18)
        self.assertEqual(self.commands(), [])
        for page in range(6):
            self.window.navigation.setCurrentRow(page)
            self.assertEqual(self.window.pages.currentIndex(), page)
        self.assertIs(self.window.pages.widget(4), self.window.rgb_page)
        self.assertTrue(self.window.pages.widget(3).isAncestorOf(self.window.hand_row))
        self.assertTrue(self.window.pages.widget(5).isAncestorOf(self.window.theme_combo))
        self.window.navigation.setCurrentRow(2)
        self.assertEqual(self.window.motor_tabs.count(), 2)
        self.assertEqual([self.window.motor_tabs.tabText(index) for index in range(2)],
                         ["电机参数", "电机点位校准"])
        self.assertTrue(self.window.motor_tabs.widget(0).isAncestorOf(
            self.controller.ent_set_pid_kp.widget))
        self.assertTrue(self.window.motor_tabs.widget(1).isAncestorOf(
            self.window.protection_tabs))
        self.assertEqual(self.window.protection_tabs.count(), 2)
        for index, kind in enumerate(("STALL", "CAL")):
            self.window.motor_tabs.setCurrentIndex(1)
            self.window.protection_tabs.setCurrentIndex(index)
            self.assertTrue(self.window.protection_tabs.currentWidget().isAncestorOf(
                next(iter(self.controller._protection_entries[kind].values())).widget))
        self.window.motor_tabs.setCurrentIndex(0)
        self.assertEqual(self.commands(), [])

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
        self.window.teaching_joint_spins[0].setValue(20)
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
        self.assertEqual(self.window.poll_hz.maximum(), 30)
        self.window.poll_hz.setValue(30)
        self.window.poll_enabled.setChecked(False)
        self.window.grid_check.setChecked(False)
        self.window.axes_check.setChecked(False)
        self.assertEqual(self.store.settings["theme"], "dark")
        self.assertEqual(self.controller._poll_hz, 30)
        self.assertEqual(self.store.settings["poll_hz"], 30)
        self.assertFalse(self.controller._poll_enabled)
        self.assertFalse(self.window.viewer.grid)
        self.assertFalse(self.window.viewer.axes)
        self.assertEqual(self.commands(), [])

    def test_fixed_gripper_mirrors_page_and_drag_sends_latest_position_once(self):
        self.connect_fake()
        self.controller.current_cmdmode = 3
        self.window.navigation.setCurrentRow(0)
        self.window.resize(1366, 768)
        self.window.show()
        APPLICATION.processEvents()
        for index in (0, 1):
            self.window.motion_tabs.setCurrentIndex(index)
            APPLICATION.processEvents()
            self.assertTrue(self.window.motion_hand_row.isVisible())
            self.assertFalse(self.window.motion_tabs.isAncestorOf(self.window.motion_hand_row))
            send_button = self.window.movej_send_button if index == 0 else self.window.movel_send_button
            self.assertFalse(send_button.visibleRegion().isEmpty())
        self.window.motion_hand_current.setValue(1.25)
        self.assertEqual(self.window.hand_current_spin.value(), 1.25)
        self.assertEqual(float(self.controller.hand_current.get()), 1.25)
        self.window.motion_hand_drag.setChecked(True)
        self.assertTrue(self.window.hand_drag_checkbox.isChecked())
        self.window.motion_hand_row.spin.setValue(45)
        self.window.motion_hand_row.spin.setValue(60)
        QTest.qWait(180)
        self.scheduler.run_ready()
        commands = self.commands()
        self.assertTrue(commands)
        self.assertEqual(commands[-1], "!HAND_POS 60")
        self.assertEqual(commands.count("!HAND_POS 60"), 1)
        self.assertEqual(self.window.hand_row.spin.value(), 60)
        self.assertEqual(self.controller.hand_pos.get(), 60)
        self.assertEqual(self.window.viewer.target[1], 60)
        self.window.hand_drag_checkbox.setChecked(False)
        self.window.hand_row.spin.setValue(35)
        self.assertEqual(self.window.motion_hand_row.spin.value(), 35)
        QTest.qWait(150)
        self.scheduler.run_ready()
        self.assertEqual(self.commands(), [])

    def test_pending_gripper_drag_cannot_resume_after_stop_or_calibration(self):
        self.connect_fake()
        self.controller.current_cmdmode = 3
        for command, value in (("!STOP", 70), ("!CALIBRATION", 80)):
            with self.subTest(command=command):
                self.window.motion_hand_drag.setChecked(True)
                self.window.motion_hand_row.spin.setValue(value)
                self.controller.send_cmd(command)
                self.assertEqual(self.commands(), [command])
                self.assertFalse(self.window.motion_hand_drag.isChecked())
                QTest.qWait(150)
                self.scheduler.run_ready()
                self.assertEqual(self.commands(), [])
                self.controller._process_rx_line("Disabled ok")
                if command == "!CALIBRATION":
                    self.controller._process_rx_line("!CALIBRATION ok")

    def test_hand_open_close_and_explicit_send_replace_pending_drag(self):
        self.connect_fake()
        self.controller.current_cmdmode = 3
        self.window.motion_hand_drag.setChecked(True)
        for opened, value in ((True, 55), (False, 65)):
            self.window.motion_hand_row.spin.setValue(value)
            self.window._send_hand_direction(opened)
            QTest.qWait(150)
            self.scheduler.run_ready()
            self.assertEqual(self.commands(), ["!HAND_O 1.00" if opened else "!HAND_C 1.00"])
        self.window.motion_hand_row.spin.setValue(75)
        self.window._send_hand_position()
        QTest.qWait(150)
        self.scheduler.run_ready()
        self.assertEqual(self.commands(), ["!HAND_POS 75"])

    def test_teaching_movel_save_select_and_send_preserves_type(self):
        self.connect_fake()
        self.controller.current_cmdmode = 3
        self.window.navigation.setCurrentRow(1)
        self.window.point_name.setText("关节示教")
        self.window.teaching_joint_spins[0].setValue(25)
        self.window._save_point()
        self.window.teaching_tabs.setCurrentIndex(1)
        pose = [500, -400, 700, 200, -190, 250]
        for spin, value in zip(self.window.teaching_pose_spins, pose):
            spin.setValue(value)
        self.window.teaching_gripper_spin.setValue(55)
        self.window.teaching_speed.setValue(42)
        self.window.point_name.setText("末端示教")
        self.window._save_point()
        self.assertEqual([point["type"] for point in self.store.points], ["MoveJ", "MoveL"])
        self.assertEqual(self.store.points[1]["pose"], pose)
        self.assertNotIn("joints", self.store.points[1])
        self.assertEqual(self.commands(), [])
        self.controller.movej_drag_enable.set(True)
        self.controller.movel_drag_enable.set(True)
        self.controller.hand_pos_drag_enable.set(True)
        self.window.teaching_drag.setChecked(True)
        self.window.point_table.selectRow(0)
        self.assertEqual(self.window.teaching_tabs.currentIndex(), 0)
        self.assertEqual(self.window.teaching_joint_spins[0].value(), 25)
        self.window.point_table.selectRow(1)
        self.assertEqual(self.window.teaching_tabs.currentIndex(), 1)
        self.assertEqual([spin.value() for spin in self.window.teaching_pose_spins], pose)
        self.assertEqual(self.window.viewer.cartesian, pose)
        self.assertEqual(self.window.viewer.target[1], 55)
        QTest.qWait(150)
        self.scheduler.run_ready()
        self.assertEqual(self.commands(), [])
        self.window._send_selected_point()
        self.assertEqual(self.commands(), ["@500,-400,700,200,-190,250,42"])

    def test_teaching_drag_uses_active_type_and_disabling_cancels_tail(self):
        self.connect_fake()
        self.controller.current_cmdmode = 3
        self.window.navigation.setCurrentRow(1)
        self.window.teaching_drag.setChecked(True)
        self.window.teaching_joint_spins[0].setValue(12)
        self.window.teaching_joint_spins[0].setValue(24)
        QTest.qWait(150)
        self.scheduler.run_ready()
        commands = self.commands()
        self.assertTrue(commands)
        self.assertTrue(commands[-1].startswith(">24,"))
        self.assertFalse(any(command.startswith("@") for command in commands))
        self.window.teaching_tabs.setCurrentIndex(1)
        self.window.teaching_pose_spins[0].setValue(245)
        QTest.qWait(150)
        self.scheduler.run_ready()
        commands = self.commands()
        self.assertTrue(commands)
        self.assertTrue(commands[-1].startswith("@245,"))
        self.assertFalse(any(command.startswith(">") for command in commands))
        self.window.teaching_pose_spins[0].setValue(260)
        self.window.teaching_drag.setChecked(False)
        QTest.qWait(150)
        self.scheduler.run_ready()
        self.assertEqual(self.commands(), [])

    def test_point_selection_and_explicit_sync_cancel_pending_drag(self):
        self.connect_fake()
        self.controller.current_cmdmode = 3
        self.window.point_name.setText("安全预览点")
        self.window._save_point()
        self.window.point_table.clearSelection()
        self.controller.hand_pos_drag_enable.set(True)
        self.window.motion_hand_row.spin.setValue(60)
        self.window.point_table.selectRow(0)
        QTest.qWait(150)
        self.scheduler.run_ready()
        self.assertEqual(self.commands(), [])
        self.assertEqual(self.window.motion_hand_row.spin.value(), 100)
        self.controller.movej_drag_enable.set(True)
        self.window.joint_spins[0].setValue(20)
        self.controller.read_and_sync_from_robot()
        self.assertEqual(self.commands(), ["#GETJPOS"])
        self.controller._process_rx_line("ok 30 -60 160 5 6 7 20")
        self.scheduler.run_ready()
        QTest.qWait(150)
        self.scheduler.run_ready()
        self.assertEqual(self.window.joint_spins[0].value(), 30)
        self.assertEqual(self.commands(), [])

    def test_joint_log_hiding_preserves_actual_pose_buffer_and_disk(self):
        self.controller.log_message.connect(lambda message, tag: self.store.log(tag, message))
        self.assertTrue(self.window.hide_joint_logs_checkbox.isChecked())
        self.controller.log("#GETJPOS", "TX")
        actual = [10, -60, 160, 5, 6, 7, 20]
        response = "ok " + " ".join(map(str, actual))
        self.controller._process_rx_line(response)
        self.scheduler.run_ready()
        self.assertEqual(self.window.viewer.actual, (actual, None))
        self.assertEqual(self.controller.actual_pose, actual)
        self.assertEqual(self.window.joint_rows[0].readback.text(), "10.00")
        self.assertNotIn("#GETJPOS", self.window.log_view.toPlainText())
        self.assertNotIn(response, self.window.log_view.toPlainText())
        buffered = list(self.window._logs)
        self.assertTrue(any(response in text for _tag, text in buffered))
        disk = (self.store.log_dir / "dummyrobot.log").read_text(encoding="utf-8")
        self.assertIn("#GETJPOS", disk)
        self.assertIn(response, disk)
        self.window.hide_joint_logs_checkbox.setChecked(False)
        self.assertIn(response, self.window.log_view.toPlainText())
        self.assertEqual(list(self.window._logs), buffered)
        reloaded = AppStore(self.directory.name)
        try:
            self.assertFalse(reloaded.settings["hide_joint_logs"])
        finally:
            reloaded.close()
        self.window.hide_joint_logs_checkbox.setChecked(True)
        self.assertNotIn(response, self.window.log_view.toPlainText())
        self.assertEqual(self.commands(), [])

    def test_terminal_diagnostics_node_scope_and_stop_lock(self):
        panel = self.window.terminal_diagnostics
        self.assertTrue(panel.export_button.isEnabled())
        self.assertTrue(all(not control.isEnabled() for control in panel.buttons.values()))
        self.connect_fake()
        self.controller._stop_locked = True
        self.controller._control_owner = "gamepad"
        self.window._update_state({})
        self.assertFalse(panel.buttons["stack"].isEnabled())
        self.assertFalse(panel.buttons["cal"].isEnabled())
        self.assertTrue(panel.buttons["stall_last"].isEnabled())
        for key in ("core_version", "motor_version", "can", "stall", "stall_last", "motion", "rgb"):
            panel.buttons[key].click()
        self.assertEqual(self.commands(), ["#FW_VERSION 0", "#FW_VERSION 1", "#CAN_DIAG 1",
                                          "#STALL_STATUS 1", "#STALL_STATUS 1 LAST", "#MOTION_STATUS", "#RGB_STATUS"])
        panel.node.setCurrentIndex(7)
        self.assertFalse(panel.buttons["stall"].isEnabled())
        self.assertTrue(panel.buttons["cal_last"].isEnabled())
        for key in ("can", "cal", "cal_last"):
            panel.buttons[key].click()
        self.assertEqual(self.commands(), ["#CAN_DIAG 8", "#CAL_STATUS 8", "#CAL_STATUS 8 LAST"])
        panel.node.setCurrentIndex(6)
        self.assertTrue(panel.buttons["stall"].isEnabled())
        self.assertTrue(panel.buttons["cal"].isEnabled())
        self.assertEqual(self.commands(), [])  # Selecting a node never queries or moves.
        self.controller._control_owner = "ui"
        self.window._update_state({"control_owner": "ui"})

    def test_terminal_stack_shortcut_matches_firmware_availability(self):
        self.connect_fake()
        panel = self.window.terminal_diagnostics
        c = self.controller
        c._motor_state_known = True
        c._motor_enabled = False
        for connection, locked, calibrating, available in (
                ("USB", False, False, True), ("UART4", False, False, False),
                ("USB", True, False, False), ("USB", False, True, False)):
            c.connection_type = connection
            c._stop_locked = locked
            c._calibration_busy = calibrating
            self.window._update_state({})
            self.assertEqual(panel.buttons["stack"].isEnabled(), available)
            panel.buttons["stack"].click()
            self.assertEqual(self.commands(), ["#STACK"] if available else [])

    def test_complete_diagnostic_export_preserves_hidden_and_paused_records(self):
        entries = [("#GETJPOS", "TX"), ("ok 1 2 3 4 5 6 7", "RX"),
                   ("CAN_DIAG [1] req=7 end", "RX"), ("GAMEPAD_STOP timeout", "WARN")]
        self.window.log_pause.setChecked(True)
        self.window.log_filter.setCurrentText("ERROR")
        self.window.log_search.setText("absent")
        for message, tag in entries:
            self.controller.log(message, tag)
        before = list(self.window._logs)
        path = Path(self.directory.name) / "diagnostic.txt"
        with patch("app.ui.main_window.QFileDialog.getSaveFileName", return_value=(str(path), "")):
            self.window.terminal_diagnostics.export_button.click()
        contents = path.read_text(encoding="utf-8")
        for message, tag in entries:
            self.assertIn(f"[{tag}] {message}", contents)
        self.assertIn("当前缓冲 4 / 5000 条", contents)
        self.assertIn(str(self.store.log_dir), contents)
        self.assertEqual(list(self.window._logs)[:4], before)
        self.assertEqual(self.commands(), [])
        with patch("app.ui.main_window.QFileDialog.getSaveFileName", return_value=("", "")):
            self.window.terminal_diagnostics.export_button.click()
        with patch("app.ui.main_window.QFileDialog.getSaveFileName", return_value=(str(path), "")), \
                patch("app.ui.main_window.Path.write_text", side_effect=OSError("denied")):
            self.window.terminal_diagnostics.export_button.click()
        self.assertTrue(any("日志导出失败" in text for tag, text in self.window._logs if tag == "ERROR"))

    def test_joint_log_filter_requires_exact_command_and_seven_finite_values(self):
        hidden = [("TX", "#GETJPOS"), ("RX", "ok 1 2 3 4 5 6 7"),
                  ("RX", "ok +1.5,-2e1,3,4,5,6,7")]
        visible = [("TX", "#GETJPOS extra"), ("TX", "prefix #GETJPOS"),
                   ("INFO", "#GETJPOS"), ("RX", "ok 1 2 3 4 5 6"),
                   ("RX", "ok 1 2 3 4 5 6 7 extra"), ("RX", "ok 1 2 3 4 5 6 NaN"),
                   ("RX", "ok 1 2 3 4 5 6 inf"), ("RX", "ok 1 2 3 4 5 6 1e309"),
                   ("RX", "ok MOVEJ id=1 ms=100"), ("RX", "error GETJPOS OFFLINE"),
                   ("WARN", "位置回读超时"), ("ERROR", "串口发送失败：Write timeout"),
                   ("RX", "prefix ok 1 2 3 4 5 6 7"), ("RX", "ok 1 2 3 4 5 6 7 8")]
        for tag, message in hidden + visible:
            self.controller.log(message, tag)
        displayed = self.window.log_view.toPlainText().splitlines()
        for tag, message in hidden:
            self.assertFalse(any(line.endswith(f"[{tag}] {message}") for line in displayed), message)
        for tag, message in visible:
            self.assertTrue(any(line.endswith(f"[{tag}] {message}") for line in displayed), message)
        self.assertEqual(len(self.window._logs), len(hidden) + len(visible))

    def test_joint_log_export_obeys_only_hiding_switch_and_cancel_keeps_buffer(self):
        self.controller.log("#GETJPOS", "TX")
        response = "ok 1 2 3 4 5 6 7"
        self.controller.log(response, "RX")
        self.controller.log("ok 1 2 3 4 5 6", "RX")
        self.controller.log("ok MOVEJ id=1", "RX")
        self.controller.log("Write timeout", "ERROR")
        self.window.log_filter.setCurrentText("ERROR")
        self.window.log_search.setText("Write")
        filtered = Path(self.directory.name) / "filtered.txt"
        with patch("app.ui.main_window.QFileDialog.getSaveFileName", return_value=(str(filtered), "")):
            self.window._export_logs()
        exported = filtered.read_text(encoding="utf-8")
        self.assertNotIn("#GETJPOS", exported)
        self.assertNotIn(response, exported)
        self.assertIn("ok 1 2 3 4 5 6", exported)
        self.assertIn("ok MOVEJ id=1", exported)
        self.assertIn("Write timeout", exported)
        self.window.hide_joint_logs_checkbox.setChecked(False)
        complete = Path(self.directory.name) / "complete.txt"
        with patch("app.ui.main_window.QFileDialog.getSaveFileName", return_value=(str(complete), "")):
            self.window._export_logs()
        self.assertIn(response, complete.read_text(encoding="utf-8"))
        before = list(self.window._logs)
        with patch("app.ui.main_window.QFileDialog.getSaveFileName", return_value=("", "")):
            self.window._export_logs()
        self.assertEqual(list(self.window._logs), before)

    def test_joint_log_toggle_respects_pause_type_and_search(self):
        self.controller.log("ok 1 2 3 4 5 6 7", "RX")
        self.controller.log("MOVEJ marker", "TX")
        self.controller.log("ok MOVEJ id=2", "RX")
        self.window.log_pause.setChecked(True)
        before = self.window.log_view.toPlainText()
        self.window.hide_joint_logs_checkbox.setChecked(False)
        self.assertEqual(self.window.log_view.toPlainText(), before)
        self.window.log_pause.setChecked(False)
        self.assertIn("ok 1 2 3 4 5 6 7", self.window.log_view.toPlainText())
        self.window.log_filter.setCurrentText("RX")
        self.window.log_search.setText("MOVEJ")
        self.assertIn("ok MOVEJ id=2", self.window.log_view.toPlainText())
        self.assertNotIn("MOVEJ marker", self.window.log_view.toPlainText())
        self.assertNotIn("ok 1 2 3 4 5 6 7", self.window.log_view.toPlainText())
        self.window.log_search.clear()
        self.window.hide_joint_logs_checkbox.setChecked(True)
        self.assertEqual(self.window.log_view.document().blockCount(), 1)
        self.assertIn("ok MOVEJ id=2", self.window.log_view.toPlainText())


if __name__ == "__main__":
    unittest.main()
