"""Offline HOME/RESET receipt identity, target-only notification and cancellation."""
import queue
import unittest
from unittest.mock import patch

from PySide6.QtCore import QCoreApplication

from app.core.controller import RobotController
from tests.test_controller import FakePort, ManualScheduler
from tests.test_rgb_controller import FakeTransport


HOME = [0, 0, 90, 0, 0, 0, 0]
RESET = [0, -75, 180, 0, 0, 0, 0]


class SystemPoseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.scheduler = ManualScheduler()
        self.c = RobotController(scheduler=self.scheduler, serial_factory=FakePort)
        self.c.is_connected = True
        self.c.serial_port = FakePort()
        self.completed = []
        self.requested = []
        self.c.preset_pose_requested.connect(lambda kind, pose: self.requested.append((kind, pose)))
        self.c.preset_pose_completed.connect(lambda kind, pose: self.completed.append((kind, pose)))

    def tearDown(self):
        self.c.close()

    def commands(self):
        result = []
        while True:
            try:
                item = self.c._tx_queue.get_nowait()
                if item:
                    result.append(item[1])
            except queue.Empty:
                return result

    def feed(self, *lines):
        for line in lines:
            self.c._process_rx_line(line)

    def test_both_presets_notify_once_only_after_numbered_completion(self):
        self.c.actual_pose = [12, -30, 120, 6, 7, 8, 300]
        self.c.last_actual_at = 123.0
        self.c.hand_pos.set(27)
        self.c.ent_j_speed.set("72")
        self.c.ent_l_speed.set("43")
        for kind, target, motion_id in (("HOME", HOME, 101), ("RESET", RESET, 102)):
            self.assertTrue(self.c.send_cmd("!" + kind))
            self.feed(kind.title() + " pending", "ok", f"ok {kind} id={motion_id}")
            self.assertEqual(len(self.completed), motion_id - 101)
            self.feed(f"queued {kind} id={motion_id} mode=TRJ", f"ok {kind} id={motion_id} ms=400")
            self.assertEqual(self.completed[-1], (kind, target))
            self.feed(f"ok {kind} id={motion_id} ms=400")
            self.assertEqual(len(self.completed), motion_id - 100)
        self.assertEqual(self.c.actual_pose, [12, -30, 120, 6, 7, 8, 300])
        self.assertEqual(self.c.last_actual_at, 123.0)
        self.assertEqual(self.c.hand_pos.get(), 27)
        self.assertEqual(self.c.ent_j_speed.get(), "72")
        self.assertEqual(self.c.ent_l_speed.get(), "43")
        # Signal consumers may edit their list without changing the next target.
        self.completed[0][1][0] = 999
        self.c.send_cmd("!HOME")
        self.feed("queued HOME id=103 mode=INT", "ok HOME id=103")
        self.assertEqual(self.completed[-1], ("HOME", HOME))

    def test_local_rejections_do_not_create_requests_or_disable_future_home(self):
        for field in ("is_connected", "_stop_locked", "_calibration_busy", "_seq_active"):
            previous = getattr(self.c, field)
            setattr(self.c, field, field != "is_connected")
            self.assertFalse(self.c.send_cmd("!HOME"), field)
            setattr(self.c, field, previous)
            self.assertEqual(self.c._system_pose_awaiting, [])
            self.assertEqual(self.c._system_pose_ambiguous, set())
            self.assertEqual(self.requested, [])
        self.assertFalse(self.c.send_cmd("!HOME\n!RESET"))
        with patch.object(self.c._tx_queue, "put", side_effect=IOError("fake queue failure")):
            self.assertFalse(self.c.send_cmd("!HOME"))
        self.assertEqual(self.requested, [])
        self.assertEqual(self.commands(), [])
        self.assertTrue(self.c.send_cmd("!HOME"))
        self.feed("queued HOME id=1 mode=SEQ", "ok HOME id=1")
        self.assertEqual(self.completed, [("HOME", HOME)])

    def test_preview_notifies_on_dispatch_before_any_receipt_without_actual_pose_changes(self):
        actual = [10, -60, 160, 5, 6, 7, 20]
        self.c.actual_pose = list(actual)
        self.c.last_actual_at = 123.
        self.c.hand_pos.set(27)
        for kind, target in (("HOME", HOME), ("RESET", RESET)):
            self.assertTrue(self.c.send_cmd("!" + kind))
            self.assertEqual(self.requested[-1], (kind, target))
            self.assertEqual(self.completed, [])
            self.assertEqual(self.c.actual_pose, actual)
            self.assertEqual(self.c.last_actual_at, 123.)
            self.assertEqual(self.c.hand_pos.get(), 27)
        self.assertEqual(self.commands(), ["!HOME", "!RESET"])

    def test_preview_does_not_depend_on_completion_tracking_and_uses_independent_lists(self):
        self.c._system_pose_ambiguous.add("HOME")
        self.assertTrue(self.c.send_cmd("!HOME"))
        self.assertEqual(self.requested, [("HOME", HOME)])
        self.assertIsNone(self.c._system_pose_active)
        self.requested[0][1][0] = 999
        self.assertTrue(self.c.send_cmd("!HOME"))
        self.assertEqual(self.requested[-1], ("HOME", HOME))
        self.assertEqual(self.completed, [])

    def test_strict_type_id_and_optional_duration_reject_malformed_receipts(self):
        self.c.send_cmd("!HOME")
        self.feed("ok HOME id=101", "queued HOME id=0 mode=TRJ",
                  "queued HOME id=4294967296 mode=TRJ", "queued HOME id=101 mode=SRV",
                  "queued HOME id=101 mode=TRJ extra=1", "queued RESET id=100 mode=TRJ")
        self.assertEqual(self.completed, [])
        self.feed("queued HOME id=101 mode=TRJ", "ok RESET id=101", "ok HOME id=100",
                  "ok HOME id=0", "ok HOME id=4294967296", "ok HOME id=101 ms=-1",
                  "ok HOME id=101 ms=4294967296", "ok HOME id=101 ms=1 extra=1")
        self.assertEqual(self.completed, [])
        self.feed("ok HOME id=101")
        self.assertEqual(self.completed, [("HOME", HOME)])

    def test_repeated_home_keeps_old_queued_and_ok_away_from_latest_request(self):
        self.c.send_cmd("!HOME")
        self.c.send_cmd("!HOME")
        self.assertEqual(self.commands(), ["!HOME", "!HOME"])
        self.feed("queued HOME id=101 mode=TRJ", "ok HOME id=101",
                  "queued HOME id=101 mode=TRJ", "ok HOME id=102")
        self.assertEqual(self.completed, [])
        self.feed("queued HOME id=102 mode=TRJ", "queued HOME id=101 mode=TRJ", "ok HOME id=101")
        self.assertEqual(self.completed, [])
        self.feed("ok HOME id=102")
        self.assertEqual(self.completed, [("HOME", HOME)])
        self.assertEqual(self.c._system_pose_awaiting, [])

    def test_new_reset_replaces_home_but_diagnostics_leave_reset_tracking_intact(self):
        self.c.send_cmd("!HOME")
        self.c.send_cmd("!RESET")
        self.feed("queued HOME id=101 mode=TRJ", "ok HOME id=101", "queued RESET id=102 mode=TUN")
        self.assertTrue(self.c.send_cmd("#GETJPOS"))
        self.assertTrue(self.c.send_cmd("#RGB_STATUS"))
        self.feed("error RGB bad", "error SET PID failed", "ok HOME id=101", "ok RESET id=102")
        self.assertEqual(self.completed, [("RESET", RESET)])

    def test_new_motion_or_mode_cancels_only_after_successful_dispatch(self):
        for index, replacement in enumerate((">0,-75,180,0,0,0,0,35", "@180,0,300,0,0,0,35", "#CMDMODE 2")):
            with self.subTest(replacement=replacement):
                self.c.send_cmd("!HOME")
                motion_id = index + 1
                self.feed(f"queued HOME id={motion_id} mode=TRJ")
                self.assertFalse(self.c.send_cmd(">0,-75,180,0,0,0,nan,35"))
                self.assertIsNotNone(self.c._system_pose_active)
                self.assertTrue(self.c.send_cmd(replacement))
                self.feed(f"ok HOME id={motion_id}")
                self.assertEqual(self.completed, [])

    def test_stop_disable_calibration_and_stall_cancel_numbered_completion(self):
        for index, cancellation in enumerate(("!STOP", "!DISABLE", "!CALIBRATION", "STALL J1 detected")):
            with self.subTest(cancellation=cancellation):
                self.c._stop_locked = self.c._calibration_busy = False
                self.c.send_cmd("!HOME")
                motion_id = index + 1
                self.feed(f"queued HOME id={motion_id} mode=TRJ")
                if cancellation.startswith("!"):
                    self.assertTrue(self.c.send_cmd(cancellation))
                else:
                    self.feed(cancellation)
                self.feed(f"ok HOME id={motion_id}")
                self.assertEqual(self.completed, [])

    def test_stop_with_late_unnumbered_request_never_assigns_old_ok_to_new_home(self):
        self.c.send_cmd("!HOME")
        self.c.send_cmd("!STOP")
        self.feed("Disabled ok")
        self.assertTrue(self.c.send_cmd("!HOME"))
        self.feed("queued HOME id=101 mode=TRJ", "ok HOME id=101")
        self.assertEqual(self.completed, [])
        self.feed("queued HOME id=102 mode=TRJ", "ok HOME id=102")
        self.assertEqual(self.completed, [("HOME", HOME)])

    def test_disconnect_reconnect_drops_old_connection_callbacks_and_recovers_tracking(self):
        self.c.send_cmd("!HOME")
        old_request = self.c._system_pose_active
        old_connection = self.c._connection_id
        self.feed("queued HOME id=101 mode=TRJ")
        self.c.disconnect_port()
        with patch("app.core.controller.SerialTransport", FakeTransport):
            self.assertTrue(self.c.connect_port("FAKE", "USB"))
        self.assertTrue(self.c.send_cmd("!RESET"))
        self.c._on_transport_line("queued RESET id=102 mode=TRJ", old_connection)
        self.c._on_transport_line("ok HOME id=101", old_connection)
        self.c._on_transport_line("ok RESET id=102", old_connection)
        self.c._system_pose_timeout(old_request)
        self.assertEqual(self.completed, [])
        self.feed("queued RESET id=1 mode=TRJ", "ok RESET id=1")
        self.assertEqual(self.completed, [("RESET", RESET)])

    def test_only_matching_motion_error_finishes_numbered_request(self):
        self.c.send_cmd("!HOME")
        self.feed("queued HOME id=1 mode=TRJ", "error RESET id=1 reason=CANCELLED",
                  "error HOME id=2 reason=CANCELLED", "error RGB bad", "error SET busy")
        self.assertIsNotNone(self.c._system_pose_active)
        self.feed("error HOME id=1 reason=CANCELLED", "ok HOME id=1")
        self.assertEqual(self.completed, [])
        self.assertIsNone(self.c._system_pose_active)
        self.assertTrue(self.c.send_cmd("!HOME"))
        self.feed("queued HOME id=2 mode=TRJ", "ok HOME id=2")
        self.assertEqual(self.completed, [("HOME", HOME)])

    def test_missing_queued_rejection_timeout_and_burst_have_bounded_lifetimes(self):
        self.c.send_cmd("!HOME")
        self.feed("error DISABLED")
        self.assertEqual(self.c._system_pose_awaiting, [])
        self.assertTrue(self.c.send_cmd("!HOME"))  # Motion remains available despite ambiguous UI sync.
        self.feed("queued HOME id=1 mode=TRJ", "ok HOME id=1")
        self.assertEqual(self.completed, [])
        self.c._reset_system_pose_tracking()
        self.c.send_cmd("!HOME")
        unnumbered = self.c._system_pose_active
        self.c._system_pose_timeout(unnumbered)
        self.assertEqual(self.c._system_pose_awaiting, [])
        self.assertEqual(self.c._system_pose_ambiguous, {"HOME"})
        self.c.send_cmd("!RESET")
        self.feed("queued RESET id=2 mode=TRJ", "ok RESET id=2")
        self.assertEqual(self.completed, [("RESET", RESET)])
        self.c._reset_system_pose_tracking()
        for _ in range(100):
            self.assertTrue(self.c.send_cmd("!HOME"))
            self.assertLessEqual(len(self.c._system_pose_awaiting), 32)
        self.assertEqual(self.c._system_pose_awaiting, [])
        self.assertIsNone(self.c._system_pose_active)
        self.assertFalse(any(delay == self.c.sequence_timeout_ms for delay, _cb, _args in self.scheduler.calls.values()))

    def test_numbered_timeout_cleans_timer_and_accepts_a_new_wraparound_id(self):
        self.c.send_cmd("!HOME")
        self.feed("queued HOME id=4294967295 mode=TRJ")
        request = self.c._system_pose_active
        token = request["timeout"]
        self.c._system_pose_timeout(request)
        self.assertNotIn(token, self.scheduler.calls)
        self.assertEqual(self.c._system_pose_ambiguous, set())
        self.c.send_cmd("!HOME")
        self.feed("queued HOME id=4294967295 mode=TRJ", "ok HOME id=4294967295")
        self.assertEqual(self.completed, [])
        self.feed("queued HOME id=1 mode=SEQ", "ok HOME id=1 ms=0")
        self.assertEqual(self.completed, [("HOME", HOME)])


if __name__ == "__main__":
    unittest.main()
