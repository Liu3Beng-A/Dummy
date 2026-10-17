"""Qt controller integration without opening any device or adding queries."""
import unittest
import time
from unittest.mock import patch

from PySide6.QtCore import QCoreApplication

from app.core.controller import RobotController
from tests.test_controller import FakePort, ManualScheduler


MOVEJ = ">0,10,80,0,90,0,0,35"


class OperationControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.c = RobotController(scheduler=ManualScheduler(), serial_factory=FakePort)
        self.c.is_connected = True
        self.c.serial_port = FakePort()
        self.c._motor_state_known = self.c._motor_enabled = True

    def tearDown(self):
        self.c.close()

    def write(self, command):
        self.assertTrue(self.c.send_cmd(command))
        self.assertEqual(self.c._tx_queue.get_nowait(), (self.c._tx_generation, command))
        self.assertTrue(self.c._can_write(self.c._tx_generation, command))
        self.c._on_written(command, self.c._connection_id)

    def accepted_movej(self, command=MOVEJ, number=1):
        self.write(command)
        self.c._process_rx_line(f"queued MOVEJ id={number} mode=INT")
        self.c._process_rx_line(f"accepted MOVEJ id={number}")

    def feedback(self, pose="0 10 80 0 90 0 0", received_at=None):
        self.c._process_rx_line("ok " + pose, received_at)

    def test_state_fields_and_read_only_observation(self):
        self.write(MOVEJ)
        self.c._process_rx_line("queued MOVEJ id=31 mode=INT")
        self.c._process_rx_line("accepted MOVEJ id=31")
        for _ in range(20):
            state = self.c.state()
            self.c._emit_state()
            self.assertEqual(state["operation"]["phase"], "accepted")
            self.assertEqual(state["operation"]["motion_id"], 31)
        self.assertTrue(self.c._tx_queue.empty())
        self.assertTrue({"native_tcp", "stop_confirmed", "mode_request_busy", "mode_pending",
                         "position_sync_busy", "rgb_save_stage", "rgb_save_message"} <= state.keys())

    def test_early_rx_does_not_get_reset_by_late_tx_signal(self):
        self.c.send_cmd(MOVEJ)
        self.c._tx_queue.get_nowait()
        self.assertTrue(self.c._can_write(self.c._tx_generation, MOVEJ))
        self.c._process_rx_line("queued MOVEJ id=32 mode=INT")
        self.c._process_rx_line("accepted MOVEJ id=32")
        self.c._on_written(MOVEJ, self.c._connection_id)
        self.assertEqual(self.c.state()["operation"]["phase"], "accepted")

    def test_received_int_position_updates_display_without_changing_device_state_or_sending(self):
        self.accepted_movej()
        generation = self.c._tx_generation
        self.feedback()
        state = self.c.state()
        self.assertEqual(state["operation"]["phase"], "observed_reached")
        self.assertEqual(self.c.operations.latest["phase"], "accepted")
        self.assertEqual(self.c._tx_generation, generation)
        self.assertTrue(self.c._motor_enabled)
        self.assertTrue(self.c._tx_queue.empty())
        self.c._process_rx_line("ok MOVEJ id=1")
        self.assertEqual(self.c.state()["operation"]["phase"], "completed")

    def test_target_is_sent_snapshot_not_current_edit_fields_or_ghost(self):
        self.accepted_movej()
        for field in self.c.ent_joints:
            field.set(0.)
        self.c.ent_j7.set(20.)
        self.feedback()
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.feedback("0 0 0 0 0 0 20")
        self.assertEqual(self.c.state()["operation"]["phase"], "accepted")

    def test_existing_matching_position_and_buffered_sample_do_not_count_for_new_request(self):
        self.feedback()
        old_stamp = self.c._actual_received_at
        self.accepted_movej()
        self.assertEqual(self.c.state()["operation"]["phase"], "accepted")
        self.feedback(received_at=old_stamp)
        self.assertEqual(self.c.state()["operation"]["phase"], "accepted")
        self.feedback()
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")

    def test_same_delivery_batch_uses_receive_time_not_later_ui_callback_time(self):
        self.write(MOVEJ)
        self.c._process_rx_line("queued MOVEJ id=1 mode=INT")
        received = self.c.cartesian.clock()-.1
        self.c._process_rx_line("accepted MOVEJ id=1", received_at=received)
        self.feedback(received_at=received+.01)
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")

    def test_position_state_withdraws_for_stale_pause_and_real_departure(self):
        self.accepted_movej()
        self.feedback()
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.c.last_actual_at = time.time()-4.
        self.assertEqual(self.c.state()["operation"]["phase"], "accepted")
        self.feedback()
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.c.set_polling(False, 5)
        self.assertEqual(self.c.state()["operation"]["phase"], "accepted")
        self.c.set_polling(True, 5)
        self.feedback("0 10 80 0 90 0 .3")
        self.assertEqual(self.c.state()["operation"]["phase"], "accepted")
        self.assertTrue(self.c._tx_queue.empty())

    def test_invalid_and_old_connection_feedback_do_not_observe_target(self):
        self.accepted_movej()
        sequence = self.c._actual_feedback_sequence
        for payload in ("0 10 80 0 90 0 nan", "0 10 80 0 90 0 inf"):
            self.feedback(payload)
        self.c._on_transport_line("ok 0 10 80 0 90 0 0", self.c._connection_id+1)
        self.assertEqual(self.c._actual_feedback_sequence, sequence)
        self.assertEqual(self.c.state()["operation"]["phase"], "accepted")
        self.feedback()
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")

    def test_each_valid_sample_does_not_emit_whole_window_state(self):
        self.accepted_movej()
        # The first sample legitimately changes actual_stale to False.
        self.feedback("0 10 80 0 90 0 3")
        observed = []
        self.c.state_changed.connect(observed.append)
        self.feedback("0 10 80 0 90 0 1")
        self.feedback("0 10 80 0 90 0 2")
        self.assertEqual(observed, [])
        self.feedback()
        self.assertEqual(len(observed), 1)
        self.assertEqual(observed[-1]["operation"]["phase"], "observed_reached")
        self.feedback(".01 10 80 0 90 0 .01")
        self.feedback()
        self.assertEqual(len(observed), 1)
        self.feedback("0 10 80 0 90 0 .21")
        self.assertEqual(len(observed), 2)
        self.assertEqual(observed[-1]["operation"]["phase"], "accepted")
        self.assertTrue(self.c._tx_queue.empty())

    def test_new_command_must_have_its_own_acceptance_and_feedback(self):
        self.accepted_movej()
        self.feedback()
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.write(MOVEJ)
        self.c._process_rx_line("queued MOVEJ id=2 mode=INT")
        self.c._process_rx_line("accepted MOVEJ id=1")
        self.feedback()
        self.assertEqual(self.c.state()["operation"]["phase"], "queued")
        self.c._process_rx_line("accepted MOVEJ id=2")
        self.assertEqual(self.c.state()["operation"]["phase"], "accepted")
        self.feedback()
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")

    def test_stop_request_prevents_old_matching_feedback_from_restoring_target_status(self):
        self.accepted_movej()
        self.feedback()
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.write("!STOP")
        self.feedback()
        self.assertEqual(self.c.state()["operation"]["kind"], "stop")
        self.assertFalse(self.c.state()["stop_confirmed"])
        self.assertTrue(self.c._tx_queue.empty())

    def test_elapsed_only_changes_do_not_emit_whole_window_state(self):
        self.write(MOVEJ)
        observed = []
        self.c.state_changed.connect(observed.append)
        start = self.c.operations.latest["since"]
        with patch.object(self.c.operations, "clock", return_value=start+1.):
            self.assertEqual(self.c.state()["operation"]["elapsed_s"], 1.)
            self.c._emit_state()
            self.c._emit_state()
            self.assertEqual(observed, [])
            self.c._process_rx_line("queued MOVEJ id=72 mode=INT")
            self.assertEqual(len(observed), 1)
            self.assertEqual(observed[-1]["operation"]["motion_id"], 72)
        with patch.object(self.c.operations, "clock", return_value=start+2.):
            self.c._emit_state()
            self.assertEqual(len(observed), 1)

    def test_start_pending_rebases_observation_only_no_new_queries(self):
        self.c._motor_enabled = False
        self.write("!START")
        generation = self.c._tx_generation
        self.c._process_rx_line("Start pending")
        self.assertGreater(self.c._tx_generation, generation)
        self.assertEqual(self.c.state()["operation"]["kind"], "start")
        self.assertTrue(self.c._tx_queue.empty())

    def test_target_submitted_while_start_pending_cannot_hide_start_after_retirement(self):
        self.c._motor_enabled = False
        self.write("!START")
        generation = self.c._tx_generation
        self.assertTrue(self.c.send_cmd(MOVEJ))
        self.assertEqual(self.c.state()["operation"]["kind"], "motion")
        self.c._process_rx_line("Start pending")
        self.assertGreater(self.c._tx_generation, generation)
        view = self.c.state()["operation"]
        self.assertEqual(view["kind"], "start")
        self.assertEqual(view["title"], "使能确认中")
        self.assertTrue(self.c._tx_queue.empty())
        # Started-ok already has a legacy mode query. Observer adds none.
        with patch.object(self.c, "_query_command_mode") as query:
            self.c._process_rx_line("Started ok")
            query.assert_called_once()
        self.assertEqual(self.c.state()["operation"]["title"], "已使能 · 待命")
        self.assertTrue(self.c._tx_queue.empty())

    def test_stop_request_and_disconnect_do_not_claim_stop_confirmation(self):
        self.write("!STOP")
        state = self.c.state()
        self.assertFalse(state["stop_confirmed"])
        self.assertNotEqual(state["operation"]["phase"], "completed")
        self.c._on_transport_failure("Write timeout", self.c._connection_id)
        view = self.c.state()["operation"]
        self.assertEqual(view["phase"], "disconnected")
        self.assertIn("无法确认", view["reason"])

    def test_explicit_protection_reason_retained_and_no_extra_stop(self):
        self.write(MOVEJ)
        self.c._process_rx_line("PROTECT J5 failed reason=PROTECTION_STATUS_TIMEOUT")
        self.c._process_rx_line("STOP locked")
        self.c._process_rx_line("error STOP_LOCKED")
        state = self.c.state()
        self.assertEqual(state["operation"]["kind"], "protection")
        self.assertIn("PROTECTION_STATUS_TIMEOUT", state["operation"]["reason"])
        self.assertTrue(self.c._tx_queue.empty())

    def test_existing_bad_generation_and_connection_guards_still_apply(self):
        self.write(MOVEJ)
        self.c._process_rx_line("queued MOVEJ id=10 mode=INT")
        self.c._on_transport_line("ok MOVEJ id=10", self.c._connection_id + 1)
        self.assertNotEqual(self.c.state()["operation"]["phase"], "completed")
        old_generation = self.c._tx_generation
        self.write("!STOP")
        self.assertFalse(self.c._can_write(old_generation, MOVEJ))
        self.c._process_rx_line("ok MOVEJ id=10")
        self.assertEqual(self.c.state()["operation"]["kind"], "stop")


if __name__ == "__main__":
    unittest.main()
