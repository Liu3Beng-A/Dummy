"""Configuration feedback integration with fake transport and manual clocks."""
import unittest
from unittest.mock import patch

from PySide6.QtCore import QCoreApplication

from app.core.controller import RobotController
from tests.test_controller import FakePort, ManualScheduler


class FeedbackControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.now = 100.
        self.clock = patch("time.monotonic", side_effect=lambda: self.now)
        self.clock.start()
        self.c = RobotController(scheduler=ManualScheduler())
        self.c.is_connected = True
        self.c.serial_port = FakePort()
        self.c._motor_state_known = True
        self.c._motor_enabled = False
        self.events = []
        self.c.feedback_event.connect(self.events.append)

    def tearDown(self):
        self.c.close()
        self.clock.stop()

    def admit(self, notify=True):
        generation, command = self.c._tx_queue.get_nowait()
        self.c._tx_queue.task_done()
        self.assertTrue(self.c._can_write(generation, command))
        if notify:
            self.c._on_written(command, self.c._connection_id)
        return command

    def tick_rgb(self, dt=.2):
        self.now += dt
        session = self.c.rgb_save
        if session.timer is not None:
            self.c.root.calls.pop(session.timer, None)
        session._tick(session.token)

    def begin_rgb_save(self, notify=True):
        self.assertTrue(self.c.send_cmd("!RGB_SAVE"))
        self.tick_rgb()
        self.assertEqual(self.admit(notify), "!RGB_SAVE")

    def test_no_feedback_for_background_queries_and_motion(self):
        for command in ("#RGB_STATUS", "#GET_PID 1", "#GETJACC 1", ">0,10,80,0,90,0,0,35"):
            self.assertTrue(self.c.send_cmd(command))
            self.admit()
        self.assertEqual(self.events, [])

    def test_no_connection_and_disabled_requirement_are_nonmodal_blocked(self):
        self.c.is_connected = False
        self.assertFalse(self.c.send_cmd("!RGB_SAVE"))
        self.assertEqual(self.events[-1]["phase"], "blocked")
        self.assertIn("未连接", self.events[-1]["message"])
        self.c.is_connected = True
        self.c._motor_enabled = True
        self.assertFalse(self.c.send_cmd("!RGB_SAVE"))
        self.assertIn("Disabled ok", self.events[-1]["message"])
        self.assertTrue(self.c._tx_queue.empty())

    def test_validation_and_uart_rejection_dont_enqueue(self):
        self.assertFalse(self.c.send_cmd("#ACC_J 1 nan &"))
        self.assertEqual(self.events[-1]["phase"], "invalid")
        self.c.connection_type = "UART4"
        self.assertFalse(self.c.send_cmd("!HAND_ACC 15"))
        self.assertEqual(self.events[-1]["scope"], "gripper")
        self.assertEqual(self.events[-1]["phase"], "blocked")
        self.assertTrue(self.c._tx_queue.empty())

    def test_queue_write_and_matched_forward_ack_share_one_id(self):
        self.assertTrue(self.c.send_cmd("#SET_PID 7 100 100 0 0"))
        task = self.events[-1]["id"]
        self.admit()
        self.assertEqual(self.events[-1]["phase"], "pending")
        self.assertEqual(self.events[-1]["evidence"], "written")
        self.c._process_rx_line("ok SET PID [7] kp=100 kv=100 ki=0 kd=0 (rail)")
        self.assertEqual(self.events[-1]["id"], task)
        self.assertEqual(self.events[-1]["phase"], "success")
        self.assertEqual(self.events[-1]["evidence"], "request_confirmed")
        self.assertIn("电机写入结果未回报", self.events[-1]["message"])
        self.assertTrue(self.c._tx_queue.empty())

    def test_rgb_applied_receipt_does_not_need_additional_query(self):
        self.assertTrue(self.c.send_cmd("!RGB_RESET"))
        task = self.events[-1]["id"]
        self.admit(notify=False)
        self.c._process_rx_line("ok RGB applied")
        self.c._on_written("!RGB_RESET", self.c._connection_id)
        self.assertEqual(self.events[-1]["phase"], "success")
        self.assertEqual(self.events[-1]["id"], task)
        self.assertTrue(self.c._tx_queue.empty())

    def test_rgb_save_phases_update_one_task_and_real_saved_confirms(self):
        self.begin_rgb_save()
        task = self.events[-1]["id"]
        self.assertIn("后台查询已暂停", self.events[-1]["message"])
        self.assertIn("后台查询已暂停", self.events[0]["message"])
        self.c._process_rx_line("ok RGB applied")
        self.assertEqual(self.events[-1]["phase"], "pending")
        self.c._process_rx_line("ok RGB saved")
        self.tick_rgb()
        self.assertEqual(self.events[-1]["phase"], "success")
        self.assertEqual(self.events[-1]["evidence"], "device_confirmed")
        self.assertEqual({event["id"] for event in self.events}, {task})
        self.assertFalse(self.c.rgb_save.active)

    def test_rgb_save_uncertain_is_not_failure_and_late_unique_receipt_can_resolve(self):
        self.begin_rgb_save()
        task = self.events[-1]["id"]
        self.tick_rgb(6.1)
        self.assertEqual(self.events[-1]["phase"], "uncertain")
        self.assertIn("查询保持暂停", self.events[-1]["next_step"])
        self.assertIn("断开重连", self.events[-1]["next_step"])
        self.assertTrue(self.c.rgb_save.active)
        self.c._process_rx_line("ok RGB saved")
        self.tick_rgb()
        self.assertEqual(self.events[-1]["phase"], "success")
        self.assertEqual(self.events[-1]["id"], task)
        self.assertTrue(self.c._tx_queue.empty())

    def test_rgb_disconnect_and_old_transport_cannot_report_saved(self):
        self.begin_rgb_save()
        connection = self.c._connection_id
        self.c.disconnect_port()
        self.assertEqual(self.events[-1]["phase"], "uncertain")
        count = len(self.events)
        self.c._on_transport_line("ok RGB saved", connection)
        self.assertEqual(len(self.events), count)

    def test_parameter_save_requires_pending_and_every_requested_node(self):
        self.assertTrue(self.c.send_cmd("#STALL_SAVE 0"))
        task = self.events[-1]["id"]
        self.admit()
        self.c._process_rx_line("ok STALL_SAVE [1] saved")
        self.assertEqual(self.events[-1]["phase"], "pending")
        self.c._process_rx_line("ok STALL_SAVE pending")
        for node in range(1, 7):
            self.c._process_rx_line(f"ok STALL_SAVE [{node}] saved")
        self.c._process_rx_line("ok STALL_SAVE [8] saved")
        self.assertEqual(self.events[-1]["phase"], "pending")
        self.c._process_rx_line("ok STALL_SAVE [7] saved")
        self.assertEqual(self.events[-1]["phase"], "success")
        self.assertEqual(self.events[-1]["id"], task)
        self.assertEqual(self.events[-1]["evidence"], "device_confirmed")
        count = len(self.events)
        self.c._process_rx_line("ok STALL_SAVE [7] saved")
        self.assertEqual(len(self.events), count)

    def test_calibration_parameter_apply_and_wrong_kind_are_distinct(self):
        self.assertTrue(self.c.send_cmd("#CAL_SET 7 SEEK_RPS 2"))
        self.admit()
        self.c._process_rx_line("ok CAL_SET pending")
        self.c._process_rx_line("ok STALL_SET [7] SEEK_RPS=2")
        self.assertEqual(self.events[-1]["phase"], "pending")
        self.c._process_rx_line("ok CAL_SET [7] SEEK_RPS=2")
        self.assertEqual(self.events[-1]["scope"], "calibration")
        self.assertEqual(self.events[-1]["phase"], "success")

    def test_parameter_timeout_and_late_receipt_dont_become_success(self):
        self.assertTrue(self.c.send_cmd("#CAL_SAVE 7"))
        self.admit()
        self.c._process_rx_line("ok CAL_SAVE pending")
        self.c._parameter_timeout(self.c._parameter_request["timeout_token"])
        self.assertEqual(self.events[-1]["phase"], "uncertain")
        count = len(self.events)
        self.c._process_rx_line("ok CAL_SAVE [7] saved")
        self.assertEqual(len(self.events), count)

    def test_parameter_device_error_keeps_protection_behavior(self):
        self.assertTrue(self.c.send_cmd("#CAL_SAVE 7"))
        self.admit()
        self.c._process_rx_line("ok CAL_SAVE pending")
        self.c._process_rx_line("error CAL_SAVE J7 WRITE_FAILED")
        self.assertEqual(self.events[-1]["phase"], "error")
        self.assertTrue(self.c._stop_locked)

    def test_generic_timeout_uses_existing_presentation_clock_and_does_not_query(self):
        self.assertTrue(self.c.send_cmd("!HAND_ACC 10"))
        self.admit()
        self.now += 7.
        self.c._emit_state()
        self.assertEqual(self.events[-1]["phase"], "uncertain")
        self.assertTrue(self.c._tx_queue.empty())
        self.c._process_rx_line("ok hand set ACC=10.00")
        self.assertEqual(self.events[-1]["phase"], "uncertain")


if __name__ == "__main__":
    unittest.main()
