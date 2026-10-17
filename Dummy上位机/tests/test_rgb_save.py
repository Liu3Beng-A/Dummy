"""Flash-save isolation and real single-writer regressions; no device access."""
import queue
import threading
import time
import unittest
from unittest.mock import patch

from PySide6.QtCore import QCoreApplication, Qt

from app.core.controller import RobotController
from app.transport.serial_transport import SerialTransport
from tests.test_controller import FakePort, ManualScheduler
from tests.test_rgb_controller import READBACK


class RGBSaveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.clock = 100.0
        self.patched = patch("time.monotonic", side_effect=lambda: self.clock)
        self.patched.start()
        self.c = RobotController(scheduler=ManualScheduler())
        self.c.is_connected = True
        self.c.serial_port = FakePort()
        self.c._motor_state_known = True
        self.c._motor_enabled = False
        self.logs = []
        self.c.log_message.connect(lambda message, tag: self.logs.append((tag, message)))

    def tearDown(self):
        self.c.close()
        self.patched.stop()

    def tick(self, dt=.2):
        self.clock += dt
        session = self.c.rgb_save
        timer = session.timer
        if timer is not None:
            self.c.root.calls.pop(timer, None)
        session._tick(session.token)

    def take(self):
        item = self.c._tx_queue.get_nowait()
        self.c._tx_queue.task_done()
        return item

    def admit(self, notify=True):
        generation, command = self.take()
        self.assertTrue(self.c._can_write(generation, command))
        if notify:
            self.c._on_written(command, self.c._connection_id)
        return command

    def begin_waiting(self, command="!RGB_SAVE", notify=True):
        self.assertTrue(self.c.send_cmd(command))
        self.tick()
        self.assertEqual(self.admit(notify), command)
        self.assertEqual(self.c.rgb_save.stage, "waiting")

    def test_waits_for_existing_edits_and_complete_joint_and_rgb_replies(self):
        self.assertTrue(self.c.send_cmd("!RGB_BRIGHT 30"))
        self.assertTrue(self.c._request_position("j", False, source="telemetry"))
        self.assertTrue(self.c.send_cmd("#RGB_GET"))
        self.assertTrue(self.c.send_cmd("!RGB_SAVE"))
        self.tick()
        self.assertEqual(self.c.rgb_save.stage, "preparing")
        self.assertEqual(self.admit(), "!RGB_BRIGHT 30")
        self.assertEqual(self.admit(), "#GETJPOS")
        self.assertEqual(self.admit(), "#RGB_GET")
        self.tick()
        self.assertTrue(self.c._tx_queue.empty())
        self.c._process_rx_line("ok 0,10,80,0,90,0,0")
        for line in READBACK:
            self.c._process_rx_line(line)
        self.tick()
        self.assertEqual(self.admit(), "!RGB_SAVE")

    def test_busy_pauses_polling_and_blocks_queued_bypass_without_ledger_side_effect(self):
        self.begin_waiting()
        self.c._rgb_monitoring = True
        self.c._poll_rgb()
        self.c._poll_position()
        self.assertTrue(self.c._position_polling_paused())
        self.assertFalse(self.c.send_cmd("#RGB_STATUS"))
        self.assertFalse(self.c.send_cmd("!RGB_SAVE"))
        self.assertFalse(self.c.send_cmd("!START"))
        self.assertTrue(self.c._tx_queue.empty())
        for command in ("#GETJPOS", "#RGB_STATUS", "!RGB_BRIGHT 40", "!RGB_SAVE"):
            self.assertFalse(self.c._can_write(self.c._tx_generation, command))
        self.assertTrue(self.c._wire_admitted.empty())

    def test_waits_for_short_status_tail_as_well_as_full_config(self):
        self.assertTrue(self.c.send_cmd("#RGB_STATUS"))
        self.admit()
        self.assertTrue(self.c.send_cmd("!RGB_SAVE"))
        for line in READBACK[:2]:
            self.c._process_rx_line(line)
        self.tick()
        self.assertEqual(self.c.rgb_save.stage, "preparing")
        self.assertTrue(self.c._tx_queue.empty())
        for line in READBACK[2:5]:
            self.c._process_rx_line(line)
        self.tick()
        self.assertEqual(self.admit(), "!RGB_SAVE")

    def test_ack_before_tx_notification_does_not_rearm_barrier(self):
        self.begin_waiting(notify=False)
        self.c._process_rx_line("ok RGB saved")
        self.assertEqual(self.c.rgb_save.stage, "settling")
        self.c._on_written("!RGB_SAVE", self.c._connection_id)
        for line in READBACK[:5]:
            self.c._process_rx_line(line)
        self.tick()
        self.assertFalse(self.c.rgb_save.active)
        self.assertIn("已保存", self.c.rgb_snapshot()["save_message"])
        self.assertTrue(self.c.send_cmd("#RGB_STATUS"))

    def test_only_explicit_save_receipt_completes_save(self):
        self.begin_waiting()
        for line in ("ok RGB applied", *READBACK[:5], "ok CMDMODE mode=2 busy=0 issued=0 req=1"):
            self.c._process_rx_line(line)
        self.tick()
        self.assertEqual(self.c.rgb_save.stage, "waiting")
        self.c._process_rx_line("error RGB save_failed")
        self.tick()
        self.assertFalse(self.c.rgb_save.active)
        self.assertIn("save_failed", self.c.rgb_save.message)

    def test_timeout_retains_connection_and_fence_and_accepts_late_ack(self):
        self.begin_waiting()
        self.tick(6.1)
        self.assertEqual(self.c.rgb_save.stage, "uncertain")
        self.assertTrue(self.c.is_connected)
        self.assertTrue(self.c.serial_port.is_open)
        self.assertTrue(self.c._tx_queue.empty())
        self.assertFalse(self.c.send_cmd("!RGB_SAVE"))
        self.c._process_rx_line("ok RGB saved")
        self.tick()
        self.assertFalse(self.c.rgb_save.active)

    def test_stop_before_dispatch_cancels_save_and_removes_queued_save(self):
        self.assertTrue(self.c.send_cmd("!RGB_SAVE"))
        self.tick()  # SAVE is queued but has not reached the writer.
        self.assertTrue(self.c.send_cmd("!STOP"))
        self.assertFalse(self.c.rgb_save.active)
        self.assertEqual(self.admit(), "!STOP")
        self.assertTrue(self.c._tx_queue.empty())

    def test_stop_after_dispatch_preserves_irreversible_save_barrier(self):
        self.begin_waiting()
        self.c._stop_confirmed = True  # A prior Disabled ok cannot confirm the new STOP.
        self.assertTrue(self.c.send_cmd("!STOP"))
        self.assertFalse(self.c._stop_confirmed)
        self.assertEqual(self.admit(), "!STOP")
        self.assertTrue(self.c.rgb_save.active)
        self.assertFalse(self.c.send_cmd("#RGB_STATUS"))
        self.c._process_rx_line("ok RGB saved")
        self.tick()
        self.assertFalse(self.c.rgb_save.active)

    def test_rgb_suffix_flash_save_uses_same_transaction_and_disabled_gate(self):
        self.c._motor_enabled = True
        self.assertFalse(self.c.send_cmd("!RGB_BRIGHT 25 &"))
        self.c._motor_enabled = False
        self.begin_waiting("!RGB_BRIGHT 25 &")
        self.assertFalse(self.c._can_write(self.c._tx_generation, "#RGB_STATUS"))
        self.c._process_rx_line("error RGB syntax_or_range")
        self.tick()
        self.assertFalse(self.c.rgb_save.active)

    def test_prepare_timeout_does_not_send_save_or_discard_existing_edits(self):
        self.assertTrue(self.c.send_cmd("!RGB_COLOR 0 1 2 3"))
        self.assertTrue(self.c.send_cmd("!RGB_SAVE"))
        self.tick(3.1)
        self.assertFalse(self.c.rgb_save.active)
        self.assertEqual(self.admit(), "!RGB_COLOR 0 1 2 3")
        self.assertTrue(self.c._tx_queue.empty())

    def test_disconnect_clears_transaction_and_stale_callbacks_are_ignored(self):
        self.begin_waiting()
        old_token = self.c.rgb_save.token
        old_connection = self.c._connection_id
        self.c.disconnect_port()
        self.c.rgb_save._tick(old_token)
        self.c._on_transport_line("ok RGB saved", old_connection)
        self.assertFalse(self.c.rgb_save.active)
        self.assertEqual(self.c.rgb_save.message, "")

    def test_prior_motor_enable_receipt_aborts_unwritten_save(self):
        self.assertTrue(self.c.send_cmd("!RGB_SAVE"))
        self.c._process_rx_line("Started ok")
        self.tick()
        self.assertFalse(self.c.rgb_save.active)
        self.assertTrue(self.c._tx_queue.empty())

    def test_stop_discarded_queue_items_do_not_poison_later_save_drain_count(self):
        c = self.c
        c._transport = SerialTransport(c.serial_port, c._tx_queue, c._can_write, c._connection_id, c)
        self.assertTrue(c.send_cmd("!RGB_BRIGHT 20"))
        self.assertTrue(c.send_cmd("#RGB_STATUS"))
        self.assertTrue(c.send_cmd("!STOP"))
        self.assertEqual(self.admit(), "!STOP")
        self.assertEqual(c._transport.pending_writes, 0)
        c._process_rx_line("Disabled ok")
        self.begin_waiting()


class SlowSavePort(FakePort):
    def __init__(self):
        super().__init__()
        self.saved = threading.Event()
        self.busy_until = 0
        self.failed_commands = []
        self.ack_timer = None

    def write(self, data):
        command = data.decode().strip()
        if time.monotonic() < self.busy_until:
            self.failed_commands.append(command)
            raise TimeoutError("simulated Flash busy")
        result = super().write(data)
        if command == "!RGB_SAVE":
            self.busy_until = time.monotonic() + .35
            self.saved.set()
            response = "ok RGB saved\n" + "\n".join(READBACK[:5]) + "\n"
            self.ack_timer = threading.Timer(.4, self.inject, (response.encode(),))
            self.ack_timer.start()
        return result


class RGBSaveWireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def wait_for(self, predicate, seconds=3):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.app.processEvents()
            if predicate():
                return
            time.sleep(.005)
        self.fail("Timed out waiting for simulated serial transaction")

    def open_controller(self):
        port = SlowSavePort()
        c = RobotController()
        c.is_connected, c.serial_port = True, port
        c._motor_state_known = True
        c._motor_enabled = False
        transport = SerialTransport(port, c._tx_queue, c._can_write, c._connection_id, c)
        c._transport = transport
        transport.write_ready = c.rgb_save.ready_to_write
        transport.command_written.connect(c._on_written, Qt.ConnectionType.QueuedConnection)
        transport.line_received_at.connect(c._on_transport_line, Qt.ConnectionType.QueuedConnection)
        transport.failed.connect(c._on_transport_failure, Qt.ConnectionType.QueuedConnection)
        transport.start()
        return c, port

    def test_real_writer_does_not_write_background_queries_during_flash_busy(self):
        c, port = self.open_controller()
        try:
            c.set_polling(True, 30)
            c._rgb_monitoring = True
            self.assertTrue(c.send_cmd("!RGB_SAVE"))
            self.wait_for(port.saved.is_set)
            # Even a caller bypassing send_cmd cannot leak old queued polling.
            c._tx_queue.put((c._tx_generation, "#GETJPOS"))
            c._tx_queue.put((c._tx_generation, "#RGB_STATUS"))
            self.wait_for(lambda: not c.rgb_save.active)
            self.assertTrue(c.is_connected)
            self.assertEqual(port.failed_commands, [])
            self.assertEqual(port.written.count(b"!RGB_SAVE\n"), 1)
            self.assertIn("已保存", c.rgb_save.message)
            c._poll_rgb()
            self.wait_for(lambda: b"#RGB_GET\n" in port.written)
        finally:
            c.close()
            if port.ack_timer:
                port.ack_timer.join(1)

    def test_stop_during_flash_busy_is_preserved_and_sent_before_polling_resumes(self):
        c, port = self.open_controller()
        try:
            self.assertTrue(c.send_cmd("!RGB_SAVE"))
            self.wait_for(port.saved.is_set)
            self.assertTrue(c.send_cmd("!STOP"))
            self.assertIn("请求待发送", c.lbl_motor_state.get())
            # Process GUI events while the TX worker holds the safety intent.
            end = time.monotonic() + .1
            while time.monotonic() < end:
                self.app.processEvents()
                time.sleep(.005)
            self.assertNotIn(b"!STOP\n", port.written)
            self.assertTrue(c.rgb_save.active)
            self.wait_for(lambda: b"!STOP\n" in port.written)
            self.assertTrue(c.is_connected)
            self.assertEqual(port.failed_commands, [])
            self.assertEqual(port.written, [b"!RGB_SAVE\n", b"!STOP\n"])
        finally:
            c.close()
            if port.ack_timer:
                port.ack_timer.join(1)

    def test_close_interrupts_safety_command_wait_without_sending_or_hanging(self):
        c, port = self.open_controller()
        try:
            self.assertTrue(c.send_cmd("!RGB_SAVE"))
            self.wait_for(port.saved.is_set)
            port.ack_timer.cancel()
            self.assertTrue(c.send_cmd("!DISABLE"))
            start = time.monotonic()
            c.close()
            self.assertLess(time.monotonic() - start, .4)
            self.assertNotIn(b"!DISABLE\n", port.written)
            self.assertEqual(port.failed_commands, [])
        finally:
            c.close()
            if port.ack_timer:
                port.ack_timer.cancel()
                port.ack_timer.join(1)


if __name__ == "__main__":
    unittest.main()
