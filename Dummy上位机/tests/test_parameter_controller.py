"""Parameter drafts remain independent from node-addressed readback and ACKs."""
import queue
import unittest
from unittest.mock import patch

from PySide6.QtCore import QCoreApplication

from app.core.controller import RobotController
from app.core.fields import ValueField
from tests.test_controller import FakePort, ManualScheduler


class ParameterControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.scheduler = ManualScheduler()
        self.c = RobotController(scheduler=self.scheduler)
        self.c.is_connected = True
        self.c.serial_port = FakePort()
        self.c._motor_state_known = True
        self.c._motor_enabled = False
        self.c.parameter_edits.set_connection(True, self.c._connection_id)

    def tearDown(self):
        self.c.close()

    def take(self):
        generation, command = self.c._tx_queue.get_nowait()
        self.c._tx_queue.task_done()
        self.assertTrue(self.c._can_write(generation, command))
        self.c._on_written(command, self.c._connection_id)
        return command

    def snapshot(self, node, key):
        return self.c.parameter_edits.snapshot("motor", node, key)

    def test_all_motor_replies_keep_source_node_even_when_not_selected(self):
        c = self.c
        c._process_rx_line("ok PID [1] kp=1 kv=2 ki=3 kd=4")
        c._process_rx_line("ok PID [8] kp=81 kv=82 ki=83 kd=84 (HAND)")
        c._process_rx_line("ok ACC [7] = 123.50 r/s^2 (rail)")
        c._process_rx_line("ok I_LIMIT [8] = 1.75 A (HAND)")
        self.assertEqual(c.ent_set_pid_kp.get(), "1")
        self.assertEqual(self.snapshot(8, "KP")["readback"], "81")
        self.assertEqual(self.snapshot(7, "ACC")["readback"], "123.50")
        self.assertEqual(self.snapshot(8, "I")["readback"], "1.75")
        c.select_parameter_node(8)
        self.assertEqual(c.ent_set_pid_kp.get(), "81")
        self.assertEqual(c.ent_set_i.get(), "1.75")

    def test_node_change_before_scheduler_drain_does_not_redirect_reply(self):
        c = self.c
        c._process_rx_line("ok PID [1] kp=11 kv=12 ki=13 kd=14")
        c.select_parameter_node(2)
        self.scheduler.run_ready()
        self.assertEqual(c.ent_set_pid_kp.get(), "100")
        self.assertEqual(self.snapshot(1, "KP")["readback"], "11")
        self.assertIsNone(self.snapshot(2, "KP")["readback"])

    def test_valuefield_draft_changes_are_preserved_while_untouched_pid_fields_update(self):
        c = self.c
        c._process_rx_line("ok PID [1] kp=10 kv=20 ki=30 kd=40")
        c.ent_set_pid_kp.set("-")
        c._process_rx_line("ok PID [1] kp=15 kv=25 ki=35 kd=45")
        self.assertEqual(c.ent_set_pid_kp.get(), "-")
        self.assertEqual(c.ent_set_pid_kv.get(), "25")
        self.assertEqual(self.snapshot(1, "KP")["readback"], "15")
        self.assertTrue(self.snapshot(1, "KP")["dirty"])

    def test_switch_nodes_restores_each_draft_including_invalid_raw_text(self):
        c = self.c
        c.select_parameter_node(1)
        c.ent_set_acc.set("1e")
        c.select_parameter_node(2)
        c.ent_set_acc.set("200")
        c.select_parameter_node(1)
        self.assertEqual(c.ent_set_acc.get(), "1e")
        c.select_parameter_node(2)
        self.assertEqual(c.ent_set_acc.get(), "200")

    def test_bound_ui_defaults_are_used_for_new_nodes(self):
        c = self.c
        c.bind_fields(ent_set_pid_kp=ValueField("0"), ent_set_pid_kv=ValueField("0"),
                      ent_set_acc=ValueField("150"))
        c.select_parameter_node(7)
        self.assertEqual(c.ent_set_pid_kp.get(), "150")
        self.assertEqual(c.ent_set_acc.get(), "150")
        self.assertFalse(self.snapshot(7, "ACC")["dirty"])

    def test_selecting_rail_seeds_pid_defaults_without_readback_or_automatic_commands(self):
        self.c.select_parameter_node(7)
        for key, value in {"KP": "150", "KV": "180", "KI": "25", "KD": "50"}.items():
            state = self.snapshot(7, key)
            self.assertEqual(state["draft"], value)
            self.assertIsNone(state["readback"])
            self.assertIsNone(state["sent"])
            self.assertFalse(state["dirty"])
        self.assertTrue(self.c._tx_queue.empty())
        self.c.ent_set_pid_kp.set("155")
        self.c.select_parameter_node(1)
        self.assertEqual(self.c.ent_set_pid_kp.get(), "100")
        self.c.select_parameter_node(7)
        self.assertEqual(self.c.ent_set_pid_kp.get(), "155")
        self.c._process_rx_line("ok PID [7] kp=160 kv=181 ki=26 kd=51 (rail)")
        self.assertEqual(self.c.ent_set_pid_kp.get(), "155")
        self.assertEqual(self.c.ent_set_pid_kv.get(), "181")
        self.assertEqual(self.snapshot(7, "KP")["readback"], "160")
        self.assertTrue(self.c._tx_queue.empty())

    def test_unsigned_pid_query_is_shared_with_gripper_but_not_silently_clipped(self):
        self.c._process_rx_line("ok PID [8] kp=-1 kv=0 ki=0 kd=0 (HAND)")
        self.assertEqual(self.snapshot(8, "KP")["readback"], "4294967295")
        self.c._process_rx_line("ok PID [8] kp=4294967296 kv=0 ki=0 kd=0 (HAND)")
        self.assertEqual(self.snapshot(8, "KP")["readback"], "4294967295")

    def test_nonfinite_and_non_node_parameter_replies_are_ignored(self):
        for line in ("ok ACC [0] = 1 r/s^2", "ok I_LIMIT [9] = 1 A", "ok ACC [1] = nan r/s^2",
                     "ok I_LIMIT [1] = inf A", "ok PID [0] kp=1 kv=2 ki=3 kd=4"):
            self.c._process_rx_line(line)
        self.assertIsNone(self.snapshot(1, "ACC")["readback"])
        self.assertIsNone(self.snapshot(1, "I")["readback"])

    def test_sent_snapshot_uses_actual_command_and_ack_does_not_rewrite_draft_or_readback(self):
        c = self.c
        c._process_rx_line("ok ACC [1] = 10.00 r/s^2")
        c.ent_set_acc.set("20")
        c.parameter_edits.edit("motor", 1, "ACC", "20")
        self.assertTrue(c.send_cmd("#ACC_J 1 15 &"))  # A console command differs from the editor.
        sent = self.snapshot(1, "ACC")["sent"]
        self.assertEqual(sent["value"], "15")
        self.assertEqual(sent["evidence"], "local")
        self.take()
        c.parameter_edits.edit("motor", 1, "ACC", "30")
        c._process_rx_line("ok SET MOTOR [1] ACC [15.00] (saved)")
        state = self.snapshot(1, "ACC")
        self.assertEqual((state["draft"], state["readback"], state["dirty"]), ("30", "10.00", True))
        self.assertEqual(state["sent"]["evidence"], "request_confirmed")
        self.assertIn("电机写入结果未回报", state["sent"]["message"])

    def test_valuefield_operation_captures_typed_draft_before_send_without_new_query(self):
        c = self.c
        c._process_rx_line("ok ACC [1] = 100 r/s^2")
        c.ent_set_acc.set("200")
        c._set_acc()
        state = self.snapshot(1, "ACC")
        self.assertEqual((state["draft"], state["readback"], state["dirty"]), ("200", "100", True))
        self.assertEqual(state["sent"]["value"], "200.0")
        self.assertEqual(c._tx_queue.qsize(), 1)

    def test_gripper_and_motor8_share_the_same_sent_keys_without_fake_readback(self):
        self.assertTrue(self.c.send_cmd("!HAND_KP 123"))
        self.assertEqual(self.snapshot(8, "KP")["sent"]["value"], "123")
        self.take()
        self.c._process_rx_line("ok hand set KP=123")
        self.assertIsNone(self.snapshot(8, "KP")["readback"])
        self.assertEqual(self.snapshot(8, "KP")["sent"]["evidence"], "request_confirmed")

    def test_rejected_and_failed_enqueue_do_not_create_sent_snapshot(self):
        self.c.is_connected = False
        self.assertFalse(self.c.send_cmd("#ACC_J 1 20"))
        self.assertIsNone(self.snapshot(1, "ACC")["sent"])
        self.c.is_connected = True
        with patch.object(self.c._tx_queue, "put", side_effect=RuntimeError("queue unavailable")):
            self.assertFalse(self.c.send_cmd("#ACC_J 1 20"))
        self.assertIsNone(self.snapshot(1, "ACC")["sent"])

    def test_observer_does_not_crash_or_change_preexisting_console_admission(self):
        c = self.c
        for command in ("#STALL_SET X ENABLE 1", "#CAL_SET 99 SEEK_RPS 1", "#STALL_SET 1 UNKNOWN_KEY 1"):
            accepted = c.send_cmd(command)
            self.assertTrue(accepted, command)
            self.take()
            self.assertFalse(any(item["sent"] is not None for item in c.parameter_edits._items.values()))
            c._process_rx_line("ok STALL_SET pending")  # A tokenless old receipt must also be safe.
            c._process_rx_line("RGB_RUNTIME state=DISABLED source=BOOT")
            c._cancel_motion_for_stop()
    def test_protection_drafts_remain_per_node_and_set_receipt_is_not_a_save(self):
        c = self.c
        c._refresh_protection_node("STALL")
        c._process_rx_line("ok STALL [1] ENABLE=1")
        c._protection_entries["STALL"]["ENABLE"].set("0")
        c._process_rx_line("ok STALL [1] ENABLE=1")
        self.assertEqual(c._protection_entries["STALL"]["ENABLE"].get(), "0")
        c._protection_nodes["STALL"].set(2)
        c._refresh_protection_node("STALL")
        c._process_rx_line("ok STALL_SET [1] ENABLE=0")
        self.assertIsNone(c.parameter_edits.snapshot("STALL", 2, "ENABLE")["readback"])
        c._protection_nodes["STALL"].set(1)
        c._refresh_protection_node("STALL")
        self.assertEqual(c._protection_entries["STALL"]["ENABLE"].get(), "0")
        self.assertEqual(c.parameter_edits.snapshot("STALL", 1, "ENABLE")["readback"], "0")

    def test_save_existing_ram_does_not_mark_unapplied_protection_draft_as_sent(self):
        c = self.c
        c._refresh_protection_node("STALL")
        c.parameter_edits.edit("STALL", 1, "ENABLE", "0")
        self.assertTrue(c.send_cmd("#STALL_SAVE 1"))
        self.assertIsNone(c.parameter_edits.snapshot("STALL", 1, "ENABLE")["sent"])

    def test_protection_defaults_remain_node_specific_and_all_has_no_single_readback(self):
        c = self.c
        c._protection_nodes["STALL"].set(7)
        c._refresh_protection_node("STALL")
        self.assertEqual(c.parameter_edits.snapshot("STALL", 7, "BACK_RPS")["draft"], "0.5")
        c._protection_nodes["CAL"].set(8)
        c._refresh_protection_node("CAL")
        self.assertEqual(c.parameter_edits.snapshot("CAL", 8, "CURRENT_A")["draft"], "1")
        c._protection_nodes["STALL"].set(0)
        c._refresh_protection_node("STALL")
        c._process_rx_line("ok STALL [1] ENABLE=1")
        c._process_rx_line("ok STALL [2] ENABLE=0")
        self.assertIsNone(c.parameter_edits.snapshot("STALL", 0, "ENABLE")["readback"])

    def test_disconnect_keeps_drafts_and_last_results_but_marks_history(self):
        c = self.c
        c._process_rx_line("ok ACC [1] = 10 r/s^2")
        c.parameter_edits.edit("motor", 1, "ACC", "20")
        self.assertTrue(c.send_cmd("#ACC_J 1 20"))
        c.disconnect_port()
        state = self.snapshot(1, "ACC")
        self.assertEqual(state["draft"], "20")
        self.assertFalse(state["fresh"])
        self.assertFalse(state["sent"]["fresh"])


if __name__ == "__main__":
    unittest.main()
