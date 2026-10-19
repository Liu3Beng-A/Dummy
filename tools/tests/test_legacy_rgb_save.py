"""Hidden Tk and a flash-busy fake port; never opens physical serial hardware."""
import importlib.util
from pathlib import Path
import queue
import time
import tkinter as tk
import unittest
from unittest.mock import patch


REPO = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("legacy_rgb_save_under_test", REPO / "串口助手_old.py")
LEGACY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LEGACY)


class FlashBusyPort:
    def __init__(self):
        self.is_open = True
        self.write_timeout = 1.0
        self.writes = []
        self.busy = False
        self.fail_save = False
        self.fail_safety = False

    def write(self, data):
        command = data.decode().strip()
        if self.busy and self.fail_safety and command in ("!STOP", "!DISABLE"):
            raise LEGACY.serial.SerialTimeoutException("simulated flash-busy safety write")
        if self.busy and command not in ("!STOP", "!DISABLE"):
            raise AssertionError(f"unexpected write during simulated flash save: {command}")
        self.writes.append(command)
        if LEGACY.RobotSerialAssistantOld._is_rgb_save_command(command):
            self.busy = True
            if self.fail_save:
                raise LEGACY.serial.SerialTimeoutException("simulated ambiguous save write")
        return len(data)

    def close(self):
        self.is_open = False


class LegacyRgbSaveTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.ports_patch = patch.object(LEGACY.serial.tools.list_ports, "comports", return_value=[])
        self.serial_patch = patch.object(LEGACY.serial, "Serial", side_effect=AssertionError("No physical port"))
        self.ports_patch.start()
        self.serial_mock = self.serial_patch.start()
        self.app = LEGACY.RobotSerialAssistantOld(self.root)
        self.port = FlashBusyPort()
        self.app.serial_port, self.app.is_connected = self.port, True
        self.app._motor_state_known, self.app._motor_enabled = True, False
        self.app._stop_locked = self.app._calibration_busy = self.app._seq_active = False

    def tearDown(self):
        self.app._finish_rgb_save("测试结束")
        self.app.is_connected = False
        self.root.update()
        self.root.destroy()
        self.serial_mock.assert_not_called()
        self.serial_patch.stop()
        self.ports_patch.stop()

    def drain_writer(self):
        self.app._tx_queue.put(None)
        self.app._serial_write_loop()

    def prepare_tick(self, delay=0.2):
        request = self.app._rgb_save_request
        if request["timer"] is not None:
            self.root.after_cancel(request["timer"])
            request["timer"] = None
        when = max(time.monotonic(), self.app._rgb_save_last_io) + delay
        with patch.object(LEGACY.time, "monotonic", return_value=when):
            self.app._prepare_rgb_save(request)
        return request

    def ready_and_write(self, command="!RGB_SAVE"):
        self.assertTrue(self.app.send_cmd(command))
        request = self.prepare_tick()
        self.assertEqual(request["phase"], "awaiting")
        self.drain_writer()
        self.assertTrue(request["written"])
        return request

    def reply(self, line):
        self.port.busy = False
        self.app._process_rx_line(line, self.app._kin_connection)

    def log_text(self):
        self.root.update_idletasks()
        return self.app.txt_log.get("1.0", tk.END)

    def test_existing_edits_drain_before_save_and_no_background_queries_during_flash(self):
        self.assertTrue(self.app.send_cmd("!RGB_COLOR 0 12 34 56"))
        self.assertTrue(self.app.send_cmd("#GETJPOS"))
        self.assertTrue(self.app.send_cmd("!RGB_SAVE"))
        request = self.prepare_tick()
        self.assertEqual(request["phase"], "preparing")
        for command in ("#RGB_STATUS", "#GETJPOS", "!RGB_MODE 3", "!RGB_SAVE"):
            self.assertFalse(self.app.send_cmd(command))
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!RGB_COLOR 0 12 34 56", "#GETJPOS"])
        self.prepare_tick()
        self.drain_writer()
        self.assertEqual(self.port.writes[-1], "!RGB_SAVE")
        self.assertFalse(self.app.send_cmd("#RGB_GET"))
        self.assertFalse(self.app.send_cmd("#GETJPOS"))
        self.app.read_and_sync_from_robot()
        self.assertIsNone(self.app._pending_sync)
        self.reply("ok RGB saved")
        self.assertIsNone(self.app._rgb_save_request)
        self.assertTrue(self.app.send_cmd("#RGB_STATUS"))
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!RGB_COLOR 0 12 34 56", "#GETJPOS", "!RGB_SAVE", "#RGB_STATUS"])
        self.assertEqual(self.port.write_timeout, 1.0)

    def test_rgb_setter_save_suffix_uses_same_exclusive_transaction(self):
        self.ready_and_write("!RGB_BRIGHT 50 &")
        self.assertFalse(self.app.send_cmd("!RGB_COLOR 1 1 2 3 &"))
        self.assertFalse(self.app.send_cmd("#GETLPOS"))
        self.assertEqual(self.port.writes, ["!RGB_BRIGHT 50 &"])
        self.reply("ok RGB saved")
        self.assertIsNone(self.app._rgb_save_request)
        self.assertEqual(self.port.writes, ["!RGB_BRIGHT 50 &"])

    def test_explicit_save_errors_resume_without_resend(self):
        for reason in ("save_failed", "save_requires_idle_disabled", "save_busy"):
            with self.subTest(reason=reason):
                self.ready_and_write()
                self.reply(f"error RGB {reason}")
                self.assertIsNone(self.app._rgb_save_request)
                self.assertTrue(self.app.send_cmd("#RGB_STATUS"))
                self.drain_writer()
        self.assertEqual(self.port.writes.count("!RGB_SAVE"), 3)
        self.assertEqual(self.port.writes.count("#RGB_STATUS"), 3)

    def test_reply_timeout_remains_uncertain_with_late_ack_and_safety_commands(self):
        request = self.ready_and_write()
        self.root.after_cancel(request["timer"])
        self.app._rgb_save_timeout(request)
        self.assertEqual(request["phase"], "uncertain")
        for command in ("!RGB_SAVE", "!RGB_BRIGHT 60 &", "#RGB_STATUS", "#GETJPOS"):
            self.assertFalse(self.app.send_cmd(command))
        self.assertTrue(self.app.send_cmd("!STOP"))
        self.drain_writer()
        self.assertTrue(self.app.send_cmd("!DISABLE"))
        self.drain_writer()
        self.assertIs(self.app._rgb_save_request, request)
        self.assertEqual(self.port.writes, ["!RGB_SAVE"])
        self.assertEqual(request["pending_safety"][1], "!DISABLE")
        self.assertIn("待发送", self.app.lbl_motor_state.cget("text"))
        self.assertNotIn("[TX] !STOP", self.log_text())
        self.assertIn("结果未确认", self.log_text())
        self.reply("ok RGB saved")
        self.assertIsNone(self.app._rgb_save_request)
        self.assertTrue(self.app.send_cmd("#RGB_STATUS"))
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!RGB_SAVE", "!DISABLE", "#RGB_STATUS"])
        self.assertEqual(self.port.writes.count("!RGB_SAVE"), 1)

    def test_prepare_timeout_never_sends_save_or_discards_pending_edits(self):
        self.assertTrue(self.app.send_cmd("!RGB_COLOR 0 1 2 3"))
        self.assertTrue(self.app.send_cmd("!RGB_SAVE"))
        request = self.app._rgb_save_request
        self.root.after_cancel(request["timer"])
        with patch.object(LEGACY.time, "monotonic", return_value=request["prepare_until"] + 0.01):
            self.app._prepare_rgb_save(request)
        self.assertIsNone(self.app._rgb_save_request)
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!RGB_COLOR 0 1 2 3"])
        self.assertIn("保存未发送", self.log_text())

    def test_writer_failure_does_not_retry_and_disconnect_cleans_uncertain_state(self):
        self.port.fail_save = True
        request = self.ready_and_write()
        self.root.after_cancel(request["timer"])
        self.app._rgb_save_timeout(request)
        self.assertEqual(request["phase"], "uncertain")
        self.assertFalse(self.app.send_cmd("#RGB_STATUS"))
        self.assertTrue(self.app.send_cmd("!STOP"))
        previous_connection = self.app._kin_connection
        self.app.toggle_connection()
        self.assertFalse(self.app.is_connected)
        self.assertIsNone(self.app._rgb_save_request)
        self.assertFalse(self.port.is_open)
        self.app._process_rx_line("ok RGB saved", previous_connection)
        self.assertIsNone(self.app._rgb_save_request)
        self.assertEqual(self.port.writes, ["!RGB_SAVE"])

    def test_raw_save_and_suffix_require_confirmed_idle_disabled_state(self):
        for known, enabled in ((False, False), (True, True)):
            self.app._motor_state_known, self.app._motor_enabled = known, enabled
            self.assertFalse(self.app.send_cmd("!RGB_SAVE"))
            self.assertFalse(self.app.send_cmd("!RGB_COLOR 0 1 2 3 &"))
        self.app._motor_state_known, self.app._motor_enabled = True, False
        self.app._acc_sync_waiting = True
        self.assertFalse(self.app.send_cmd("!RGB_SAVE"))
        self.app._acc_sync_waiting = False
        self.assertFalse(self.app.send_cmd("!RGB_SAVE &"))
        self.assertIsNone(self.app._rgb_save_request)
        self.assertTrue(self.app._tx_queue.empty())

    def test_safety_cancels_preparation_before_save_is_sent(self):
        self.assertTrue(self.app.send_cmd("!RGB_COLOR 0 1 2 3"))
        self.assertTrue(self.app.send_cmd("!RGB_SAVE"))
        self.assertTrue(self.app.send_cmd("!STOP"))
        self.assertIsNone(self.app._rgb_save_request)
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!STOP"])

    def test_unsent_or_previous_connection_ack_cannot_complete_transaction(self):
        self.assertTrue(self.app.send_cmd("!RGB_SAVE"))
        request = self.app._rgb_save_request
        self.app._process_rx_line("ok RGB saved", self.app._kin_connection)
        self.assertIs(self.app._rgb_save_request, request)
        self.prepare_tick()
        self.drain_writer()
        self.app._process_rx_line("ok RGB saved", self.app._kin_connection - 1)
        self.assertIs(self.app._rgb_save_request, request)
        self.reply("ok RGB saved")
        self.assertIsNone(self.app._rgb_save_request)

    def test_prepare_waits_for_existing_writer_sync_and_rgb_response(self):
        self.assertTrue(self.app.send_cmd("!RGB_SAVE"))
        request = self.app._rgb_save_request
        for attr, busy, idle in (("_tx_write_active", True, False), ("_pending_sync", "j", None),
                                 ("_rgb_motion_response_active", True, False)):
            setattr(self.app, attr, busy)
            self.prepare_tick()
            self.assertEqual(request["phase"], "preparing")
            self.assertTrue(self.app._tx_queue.empty())
            setattr(self.app, attr, idle)
        self.prepare_tick()
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!RGB_SAVE"])

    def test_preparation_requires_a_real_quiet_window_after_rx(self):
        self.assertTrue(self.app.send_cmd("!RGB_SAVE"))
        request = self.app._rgb_save_request
        self.app._process_rx_line("RGB_TIMEOUT timeouts=0", self.app._kin_connection)
        self.prepare_tick(delay=0.05)
        self.assertEqual(request["phase"], "preparing")
        self.assertTrue(self.app._tx_queue.empty())
        self.prepare_tick(delay=0.2)
        self.assertEqual(request["phase"], "awaiting")
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!RGB_SAVE"])

    def test_unwritten_save_is_not_started_late_after_confirmation_deadline(self):
        self.assertTrue(self.app.send_cmd("!RGB_SAVE"))
        request = self.prepare_tick()
        self.assertFalse(request["written"])
        self.root.after_cancel(request["timer"])
        self.app._rgb_save_timeout(request)
        self.drain_writer()
        self.assertEqual(self.port.writes, [])
        self.assertIs(self.app._rgb_save_request, request)
        self.assertFalse(self.app.send_cmd("!RGB_SAVE"))
        self.assertTrue(self.app.send_cmd("!STOP"))
        self.assertIsNone(self.app._rgb_save_request)
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!STOP"])

    def test_safety_waits_for_exact_save_ack_and_never_attempts_flash_busy_write(self):
        request = self.ready_and_write()
        self.port.fail_safety = True
        self.assertTrue(self.app.send_cmd("!STOP"))
        self.drain_writer()
        self.assertTrue(self.app.is_connected)
        self.assertIs(self.app._rgb_save_request, request)
        self.assertEqual(self.port.writes, ["!RGB_SAVE"])
        self.assertFalse(self.app._stop_locked)
        self.assertEqual(request["pending_safety"][1], "!STOP")
        self.root.after_cancel(request["timer"])
        self.app._rgb_save_timeout(request)
        self.assertEqual(request["phase"], "uncertain")
        self.assertFalse(self.app.send_cmd("!RGB_SAVE"))
        self.reply("ok RGB saved")
        self.assertIsNone(self.app._rgb_save_request)
        self.assertTrue(self.app.is_connected)
        self.assertTrue(self.app.send_cmd("#RGB_STATUS"))
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!RGB_SAVE", "!STOP", "#RGB_STATUS"])
        self.assertEqual(self.port.writes.count("!RGB_SAVE"), 1)

    def test_failed_save_reply_also_flushes_current_safety_before_queries(self):
        request = self.ready_and_write()
        self.port.fail_safety = True
        self.assertTrue(self.app.send_cmd("!DISABLE"))
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!RGB_SAVE"])
        self.reply("error RGB save_failed")
        self.assertIsNone(self.app._rgb_save_request)
        self.assertTrue(self.app.send_cmd("#RGB_STATUS"))
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!RGB_SAVE", "!DISABLE", "#RGB_STATUS"])

    def test_generation_change_cancels_deferred_old_safety(self):
        request = self.ready_and_write()
        self.assertTrue(self.app.send_cmd("!STOP"))
        generation = request["pending_safety"][0]
        self.app._cancel_motion_for_stop()
        self.assertNotEqual(generation, self.app._tx_generation)
        self.reply("ok RGB saved")
        self.assertIsNone(self.app._rgb_save_request)
        self.drain_writer()
        self.assertEqual(self.port.writes, ["!RGB_SAVE"])

    def test_rejected_saved_setter_ends_transaction_and_flushes_safety_without_timeout(self):
        for reason in ("syntax_or_range", "config_busy", "unknown"):
            with self.subTest(reason=reason):
                self.app._stop_locked = False
                command = "!RGB_BRIGHT 101 &"
                request = self.ready_and_write(command)
                self.assertTrue(self.app.send_cmd("!DISABLE"))
                self.assertIs(self.app._rgb_save_request, request)
                self.assertEqual(request["phase"], "awaiting")
                before = len(self.port.writes)
                self.reply(f"error RGB {reason}")
                self.assertIsNone(self.app._rgb_save_request)
                self.assertTrue(self.app.send_cmd("#RGB_STATUS"))
                self.drain_writer()
                self.assertEqual(self.port.writes[before:], ["!DISABLE", "#RGB_STATUS"])
        self.assertEqual(self.port.writes.count("!RGB_BRIGHT 101 &"), 3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
