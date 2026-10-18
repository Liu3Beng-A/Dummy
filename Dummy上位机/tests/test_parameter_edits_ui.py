"""Node-addressed drafts survive readbacks, focus fixup and request receipts."""
from copy import deepcopy
from pathlib import Path
import queue
import tempfile
import unittest
from unittest.mock import patch

from PySide6.QtCore import QEvent, QPoint, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QLabel, QMessageBox

from tests.test_ui import APPLICATION, FakeView
from tests.test_gamepad_ui import FakeReader
from tests.test_controller import FakePort, ManualScheduler
from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow
from app.core.rgb_simulator import default_config


class ParameterEditsUiTests(unittest.TestCase):
    def setUp(self):
        base = Path(__file__).resolve().parents[1] / "build" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=base)
        self.previous_style = APPLICATION.styleSheet()
        self.store = AppStore(self.directory.name)
        self.store.update_settings(auto_poll=False)
        self.scheduler = ManualScheduler()
        self.c = RobotController(scheduler=self.scheduler, serial_factory=FakePort)
        with patch("app.ui.main_window.RobotView", FakeView):
            self.w = MainWindow(self.c, self.store, gamepad_reader=FakeReader())
        self.scheduler.run_ready()
        self.w.rgb_preview_source.setCurrentIndex(0)
        self.w._operation_timer.stop()
        self.w._rgb_render_timer.stop()
        self.c._poll_timer.stop()
        self.w.show()
        self.w.navigation.setCurrentRow(2)
        self.settle()
        self.modal_patches = [patch.object(QMessageBox, name, side_effect=AssertionError("No modal feedback"))
                              for name in ("information", "warning", "critical", "question")]
        for context in self.modal_patches:
            context.start()

    def tearDown(self):
        for context in reversed(self.modal_patches):
            context.stop()
        self.w.close()
        self.w.deleteLater()
        APPLICATION.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        self.settle()
        APPLICATION.setStyleSheet(self.previous_style)
        self.store.close()
        self.directory.cleanup()

    @staticmethod
    def settle():
        for _ in range(6):
            APPLICATION.processEvents()

    def binding(self, key, family="motor", view=""):
        return self.w.parameter_bindings[(family, key, view)]

    def connect_fake(self):
        self.c.is_connected = True
        self.c.serial_port = FakePort()
        self.c._motor_state_known = True
        self.c._motor_enabled = False
        self.c.parameter_edits.set_connection(True, self.c._connection_id)

    def receive(self, *lines):
        self.connect_fake()
        for line in lines:
            self.c._process_rx_line(line)
        self.settle()

    def type_raw(self, binding, text):
        widget = binding.widget
        line = widget.lineEdit() if hasattr(widget, "lineEdit") else widget
        line.setFocus()
        line.selectAll()
        QTest.keyClicks(line, text)
        self.settle()

    def commands(self):
        commands = []
        while True:
            try:
                commands.append(self.c._tx_queue.get_nowait())
            except queue.Empty:
                return commands

    def test_query_keeps_invalid_raw_and_focusout_cannot_send_old_spin_value(self):
        self.receive("ok ACC [1] = 10.00 r/s^2")
        binding = self.binding("ACC")
        self.type_raw(binding, "-")
        self.assertEqual(binding.snapshot()["draft"], "-")
        self.receive("ok ACC [1] = 20.00 r/s^2")
        self.assertEqual(binding.snapshot()["draft"], "-")
        self.assertEqual(binding.snapshot()["readback"], "20.00")
        self.w.motor_pid_restore.setFocus()
        self.settle()
        self.assertEqual(binding.raw_text(), "-")
        self.assertEqual(self.c.ent_set_acc.get(), "-")
        self.c._set_acc()
        self.assertEqual(self.commands(), [])
        self.assertEqual(self.c.feedback_snapshot("motor")["phase"], "invalid")

    def test_typing_each_decimal_character_is_not_normalized_or_sent(self):
        binding = self.binding("ACC")
        line = binding.widget.lineEdit()
        for theme in ("light", "dark"):
            self.w.apply_theme(theme)
            line.setFocus()
            line.selectAll()
            current = ""
            for character in "12.345":
                QTest.keyClicks(line, character)
                current += character
                self.settle()
                self.assertEqual(binding.snapshot()["draft"], current)
                self.assertEqual(binding.raw_text(), current)
        self.assertEqual(self.commands(), [])

    def test_high_precision_spin_edits_and_hand_acc_send_keep_all_decimals(self):
        binding = self.binding("ACC")
        self.connect_fake()
        for value in (3333.333, 4999.999):
            binding.widget.setValue(value)
            self.settle()
            self.assertEqual(float(binding.snapshot()["draft"]), value)
            self.assertEqual(binding.widget.value(), value)
        self.w.navigation.setCurrentRow(3)
        hand = self.binding("ACC", view="hand")
        self.type_raw(hand, "3333.333")
        self.w._hand_config("ACC")
        self.assertEqual([command for _, command in self.commands()], ["!HAND_ACC 3333.333"])

    def test_keyboard_tracking_both_modes_keep_raw_precision_and_insert_cursor(self):
        binding = self.binding("ACC")
        line = binding.widget.lineEdit()
        for tracking in (False, True):
            with self.subTest(keyboard_tracking=tracking):
                binding.widget.setKeyboardTracking(tracking)
                self.type_raw(binding, "3333.333")
                self.assertEqual(binding.raw_text(), "3333.333")
                self.assertEqual(binding.snapshot()["draft"], "3333.333")
                self.type_raw(binding, "12.345")
                line.setCursorPosition(1)
                QTest.keyClicks(line, "0")
                self.settle()
                self.assertEqual(binding.raw_text(), "102.345")
                self.assertEqual(binding.snapshot()["draft"], "102.345")
                self.assertEqual(line.cursorPosition(), 2)
        self.assertEqual(self.commands(), [])

    def test_switching_nodes_preserves_each_draft_and_late_readback_source(self):
        self.receive("ok ACC [1] = 10.00 r/s^2")
        self.type_raw(self.binding("ACC"), "21.5")
        self.c.param_node.set(2)
        self.settle()
        self.type_raw(self.binding("ACC"), "-")
        self.receive("ok ACC [1] = 11.00 r/s^2", "ok ACC [2] = 40.00 r/s^2")
        self.assertEqual(self.binding("ACC").snapshot()["draft"], "-")
        self.assertEqual(self.binding("ACC").snapshot()["readback"], "40.00")
        self.c.param_node.set(1)
        self.settle()
        self.assertEqual(self.binding("ACC").snapshot()["draft"], "21.5")
        self.assertEqual(self.binding("ACC").snapshot()["readback"], "11.00")
        self.assertEqual(self.c.ent_set_acc.get(), 21.5)

    def test_rail_pid_default_editor_has_no_fake_readback_and_preserves_drafts(self):
        self.c.param_node.set(7)
        self.settle()
        for key, value in {"KP": 150, "KV": 180, "KI": 25, "KD": 50}.items():
            binding = self.binding(key)
            self.assertEqual(binding.widget.value(), value)
            self.assertEqual(binding.snapshot()["draft"], str(value))
            self.assertIsNone(binding.snapshot()["readback"])
            self.assertIn("未回读", binding.status_label.text())
        self.assertFalse(self.w.motor_pid_restore.isEnabled())
        self.type_raw(self.binding("KP"), "155")
        self.c.param_node.set(1)
        self.settle()
        self.assertEqual(self.binding("KP").widget.value(), 0)
        self.c.param_node.set(7)
        self.settle()
        self.assertEqual(self.binding("KP").snapshot()["draft"], "155")
        self.receive("ok PID [7] kp=160 kv=181 ki=26 kd=51 (rail)")
        self.assertEqual(self.binding("KP").snapshot()["draft"], "155")
        self.assertEqual(self.binding("KV").widget.value(), 181)
        self.assertEqual(self.commands(), [])

    def test_restore_only_changes_local_edit_and_disconnect_marks_readback_old(self):
        self.receive("ok ACC [1] = 10.00 r/s^2")
        binding = self.binding("ACC")
        self.type_raw(binding, "25")
        binding.restore_button.click()
        self.settle()
        self.assertEqual(binding.snapshot()["draft"], "10.00")
        self.assertFalse(binding.snapshot()["dirty"])
        self.assertEqual(self.commands(), [])
        self.c.disconnect_port()
        self.settle()
        self.assertFalse(binding.snapshot()["fresh"])
        self.assertIn("过期", binding.readback_label.text())
        self.assertIn("过期", binding.status_label.text())

    def test_send_snapshot_and_ack_do_not_overwrite_newer_edit(self):
        self.receive("ok ACC [1] = 10.00 r/s^2")
        binding = self.binding("ACC")
        self.type_raw(binding, "50")
        self.c._set_acc()
        generation, command = self.commands()[0]
        self.assertEqual(command, "#ACC_J 1 50.0")
        self.assertTrue(self.c._can_write(generation, command))
        self.c._on_written(command, self.c._connection_id)
        self.type_raw(binding, "60")
        self.c._process_rx_line("ok SET MOTOR [1] ACC [50.00]")
        self.settle()
        state = binding.snapshot()
        self.assertEqual(state["draft"], "60")
        self.assertEqual(state["sent"]["value"], "50.0")
        self.assertTrue(state["dirty"])
        self.assertIn("未发送", binding.status_label.text())
        self.assertEqual(self.commands(), [])

    def test_query_after_request_confirmation_can_show_readback_consistency(self):
        self.connect_fake()
        binding = self.binding("ACC")
        self.type_raw(binding, "50")
        self.c._set_acc()
        generation, command = self.commands()[0]
        self.assertTrue(self.c._can_write(generation, command))
        self.c._on_written(command, self.c._connection_id)
        self.c._process_rx_line("ok SET MOTOR [1] ACC [50.00]")
        self.receive("ok ACC [1] = 50.00 r/s^2")
        self.assertIn("一致", binding.status_label.text())
        self.assertNotIn("已保存", binding.status_label.text())

    def test_protection_query_and_node_switch_preserve_raw_field_and_restore(self):
        self.w.motor_tabs.setCurrentIndex(1)
        binding = self.binding("TRIGGER_MS", "STALL")
        self.receive("ok STALL [1] TRIGGER_MS=100")
        self.type_raw(binding, "unfinished")
        self.receive("ok STALL [1] TRIGGER_MS=120")
        self.assertEqual(binding.widget.text(), "unfinished")
        node = self.w.fields["_protection_nodes"]["STALL"]
        node.set(2)
        self.settle()
        self.assertNotEqual(binding.snapshot()["draft"], "unfinished")
        node.set(1)
        self.settle()
        self.assertEqual(binding.widget.text(), "unfinished")
        binding.restore_button.click()
        self.assertEqual(binding.widget.text(), "120")
        self.assertEqual(self.commands(), [])

    def test_visible_plain_line_change_becomes_draft_before_apply(self):
        binding = self.binding("CURRENT_A", "STALL")
        binding.widget.setText("invalid-value")
        self.assertEqual(binding.snapshot()["draft"], "invalid-value")
        self.assertEqual(self.w.fields["_protection_entries"]["STALL"]["CURRENT_A"].get(), "invalid-value")
        self.connect_fake()
        self.c._set_protection("STALL", "CURRENT_A")
        self.assertEqual(self.commands(), [])
        self.assertEqual(self.c.feedback_snapshot("protection")["phase"], "invalid")

    def test_group_restore_requires_baseline_and_inapplicable_rows_never_restore(self):
        self.assertFalse(self.w.motor_pid_restore.isEnabled())
        self.c.parameter_edits.receive("motor", 1, "KP", "123")
        self.assertTrue(self.w.motor_pid_restore.isEnabled())
        self.c.param_node.set(0)
        self.assertFalse(self.w.motor_pid_restore.isEnabled())
        self.c.param_node.set(2)
        self.assertFalse(self.w.motor_pid_restore.isEnabled())
        self.c.param_node.set(1)
        self.assertTrue(self.w.motor_pid_restore.isEnabled())
        stall_restore = next(button for button, family, _, _ in self.w.parameter_edit_ui.groups if family == "STALL")
        self.assertFalse(stall_restore.isEnabled())
        self.c.parameter_edits.receive("STALL", 1, "BACK_MM", "5")
        self.assertFalse(self.binding("BACK_MM", "STALL").restore_button.isEnabled())
        self.assertFalse(stall_restore.isEnabled())
        self.c.parameter_edits.receive("STALL", 1, "TRIGGER_MS", "100")
        self.assertTrue(stall_restore.isEnabled())
        self.w.fields["_protection_nodes"]["STALL"].set(8)
        self.settle()
        self.assertFalse(stall_restore.isEnabled())

    def test_uint32_hand_pid_is_displayed_raw_and_signed_send_is_refused(self):
        self.receive("ok PID [8] kp=-1 kv=2 ki=3 kd=4 (HAND)")
        binding = self.binding("KP", view="hand")
        self.assertEqual(binding.snapshot()["readback"], "4294967295")
        self.assertEqual(binding.raw_text(), "4294967295")
        self.assertIn("4294967295", binding.readback_label.text())
        self.w.navigation.setCurrentRow(3)
        self.w._hand_config("KP")
        self.assertEqual(self.commands(), [])
        self.assertEqual(self.c.feedback_snapshot("gripper")["phase"], "invalid")
        self.c.param_node.set(8)
        self.settle()
        self.assertEqual(self.c.ent_set_pid_kp.get(), 4294967295)

    def test_node8_views_share_pid_acc_but_not_runtime_hand_current(self):
        self.receive("ok ACC [8] = 7.00 r/s^2", "ok I_LIMIT [8] = 2.00 A")
        hand_acc = self.binding("ACC", view="hand")
        self.assertEqual(hand_acc.snapshot()["readback"], "7.00")
        self.assertEqual(self.w.hand_current_spin.value(), 1.0)
        self.type_raw(hand_acc, "9")
        self.c.param_node.set(8)
        self.settle()
        self.assertEqual(self.c.ent_set_acc.get(), 9)
        for key in ("VEL", "ONBOOT"):
            binding = self.binding(key, view="hand")
            self.assertFalse(binding.snapshot()["queryable"])
            self.assertFalse(binding.restore_button.isEnabled())
            self.assertIsNone(binding.snapshot()["readback"])

    def test_node_captions_reserve_layout_space_beside_their_selectors(self):
        for theme in ("light", "dark"):
            self.w.apply_theme(theme)
            for width, height in ((1120, 690), (1440, 900)):
                self.w.resize(width, height)
                for kind in ("motor", "STALL", "CAL"):
                    self.w.motor_tabs.setCurrentIndex(0 if kind == "motor" else 1)
                    if kind != "motor":
                        self.w.protection_tabs.setCurrentIndex(0 if kind == "STALL" else 1)
                    self.settle()
                    selector = (self.w.fields["param_node"].widget if kind == "motor" else
                                self.w.fields["_protection_nodes"][kind].widget)
                    text = "参数节点" if kind == "motor" else "节点"
                    captions = [label for label in selector.parentWidget().findChildren(QLabel)
                                if label.property("parameterCaption") and label.text() == text]
                    self.assertEqual(len(captions), 1)
                    caption = captions[0]
                    right = caption.mapTo(self.w, QPoint(caption.width(), 0)).x()
                    left = selector.mapTo(self.w, QPoint(0, 0)).x()
                    self.assertLessEqual(right + 8, left, (theme, width, kind))
                    self.assertEqual(caption.displayedText(), caption.text())
                self.assertEqual((self.w.width(), self.w.height()), (width, height))
        self.assertEqual(self.commands(), [])

    def test_fixed_labels_and_dirty_styles_do_not_change_page_header_geometry(self):
        self.w.resize(1120, 690)
        self.settle()
        binding = self.binding("ACC")
        widgets = [self.w.pages, self.w.operation_panel, binding.widget,
                   binding.readback_label, binding.status_label]
        before = [widget.geometry() for widget in widgets]
        self.type_raw(binding, "-")
        self.receive("ok ACC [1] = 20.00 r/s^2")
        self.assertEqual([widget.geometry() for widget in widgets], before)
        self.assertEqual(binding.readback_label.height(), 20)
        self.assertEqual(binding.status_label.height(), 20)

    def test_unused_parameter_controls_are_owned_and_destroyed_without_garbage_collection(self):
        names = {"ParameterRestore", "ParameterReadback", "ParameterEditStatus"}
        self.assertFalse(self.w.parameter_edit_ui.stash.isVisible())
        baseline = sum(widget.objectName() in names for widget in APPLICATION.allWidgets())
        with tempfile.TemporaryDirectory(dir=Path(self.directory.name).parent) as directory:
            store = AppStore(directory)
            store.update_settings(auto_poll=False)
            controller = RobotController(scheduler=ManualScheduler(), serial_factory=FakePort)
            with patch("app.ui.main_window.RobotView", FakeView):
                window = MainWindow(controller, store, gamepad_reader=FakeReader())
            window.show()
            self.settle()
            self.assertFalse(window.parameter_edit_ui.stash.isVisible())
            self.assertFalse(any(widget.objectName() == "ParameterRestore"
                                 for widget in APPLICATION.topLevelWidgets()))
            for widget in window.parameter_edit_ui.stash.findChildren(type(self.w.motor_pid_restore)):
                self.assertFalse(widget.isVisible())
            self.assertGreater(sum(widget.objectName() in names for widget in APPLICATION.allWidgets()), baseline)
            window.close()
            window.deleteLater()
            APPLICATION.sendPostedEvents(None, QEvent.Type.DeferredDelete)
            self.settle()
            # Keep window/controller/store Python references alive: Qt ownership,
            # rather than a later cyclic GC sweep, must release all controls.
            self.assertEqual(sum(widget.objectName() in names for widget in APPLICATION.allWidgets()), baseline)
            store.close()

    def test_rgb_edit_back_to_baseline_and_restore_are_local_without_new_queries(self):
        snapshot = deepcopy(self.c.rgb_snapshot())
        snapshot.update(connected=True, complete=True, connection_id=77, config=default_config())
        self.w._rgb_device_changed(snapshot)
        original = snapshot["config"]["brightness"]
        self.w._rgb_store_draft(("brightness",), original - 1)
        self.assertIn(("brightness",), self.w._rgb_dirty_paths)
        self.w._rgb_store_draft(("brightness",), original)
        self.assertNotIn(("brightness",), self.w._rgb_dirty_paths)
        self.w._rgb_store_draft(("brightness",), original - 1)
        before = self.w.rgb_preview_source.currentIndex()
        self.w.rgb_restore_edits.click()
        self.assertEqual(self.w._rgb_draft["brightness"], original)
        self.assertEqual(self.w._rgb_dirty_paths, set())
        self.assertEqual(self.w.rgb_preview_source.currentIndex(), before)
        snapshot.update(connected=False, complete=False)
        self.w._rgb_device_changed(snapshot)
        self.assertIn("过期", self.w.rgb_edit_status.text())
        self.assertEqual(self.commands(), [])


if __name__ == "__main__":
    unittest.main()
