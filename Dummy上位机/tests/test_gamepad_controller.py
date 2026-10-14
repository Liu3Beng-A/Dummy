"""Shared-port ownership, generation admission and telemetry regressions."""
import unittest
from PySide6.QtCore import QCoreApplication
from tests.test_gamepad_session import GamepadFixture


class GamepadControllerTests(GamepadFixture, unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.setup_gamepad()

    def tearDown(self):
        self.c.close()

    def test_claim_discards_old_ui_queue_and_blocks_all_conflicting_sources(self):
        self.c.send_cmd(">0,-75,180,0,0,0,0,35")
        old_generation = self.c._tx_generation
        self.assertTrue(self.g.claim())
        self.assertEqual(self.take(), [])
        self.assertFalse(self.c._can_write(old_generation, ">0,-75,180,0,0,0,0,35"))
        commands = [">0,-75,180,0,0,0,0,35", "@175,187,-31,90,-45,180,10", "!HOME", "!RESET",
                    "!START", "!HAND_POS 50", "!HAND_I 1", "#CMDMODE 3", "#ENC_CAL 1",
                    "#SET_PID 1 200 80 300 250", "#ACC_J 1 10", "#I_LIMIT_J 1 2.5",
                    "!CALIBRATION", "#STALL_GET 1", "#STALL_SET 1 ENABLE 1", "!RGB_SAVE"]
        for command in commands:
            self.assertFalse(self.c.send_cmd(command), command)
        self.assertTrue(self.c.send_cmd("#RGB_STATUS"))
        self.assertTrue(self.c.send_cmd("#MOTION_STATUS"))
        self.assertEqual([command for _gen, command in self.take()], ["#RGB_STATUS", "#MOTION_STATUS"])

    def test_claim_waits_for_ui_position_and_other_transactions_without_interrupting_them(self):
        self.c.read_and_sync_from_robot()
        generation = self.c._tx_generation
        self.assertFalse(self.g.claim())
        self.assertFalse(self.g.claimed)
        self.assertEqual(self.c._tx_generation, generation)
        self.feed("ok 0 -75 180 0 0 0 10")
        self.assertTrue(self.g.claim())
        self.assertEqual(self.take(), [])

    def test_claim_cancels_passive_poll_then_resumes_shared_poll_in_new_generation(self):
        self.c.set_polling(True, 30)
        self.c._poll_position()
        queued = self.take()[0]
        token = self.c._position_request["token"]
        self.assertTrue(self.g.claim())
        self.assertIsNone(self.c._position_request)
        self.assertFalse(self.c._can_write(*queued))
        self.c._poll_position()
        current = self.c._position_request
        self.c._position_timeout(token)
        self.assertIs(self.c._position_request, current)
        self.assertEqual(self.write(), ["#GETJPOS"])
        self.feed("ok 2 9 78 3 87 -2 0")
        self.assertTrue(self.g.claimed)
        self.assertEqual(self.g.stage, "connected")
        self.assertIsNone(self.g.target_tcp)

    def test_deadline_needs_actual_admission_not_enqueue_or_spurious_written(self):
        self.g.claim()
        self.g.start()
        pending = self.take()
        self.clock.advance(4)
        self.sample()
        self.g.tick()
        self.assertEqual(self.g.stage, "starting")
        self.assertIsNone(self.g._system["deadline"])
        self.c._on_written("!START", self.c._connection_id)
        self.assertFalse(self.g._system["written"])
        self.write(pending)
        self.assertEqual(self.g._system["deadline"], self.clock() + 3)
        self.clock.advance(3.1)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["!STOP"])

    def test_old_admitted_written_cannot_install_deadline_after_stop(self):
        self.g.claim()
        self.g.start()
        old = self.take()[0]
        self.assertTrue(self.c._can_write(*old))
        self.g.stop()
        stop = self.take()
        self.c._on_written(old[1], self.c._connection_id)
        self.assertEqual(self.g.stage, "stopping")
        self.assertFalse(self.g._system["written"])
        self.assertIsNone(self.g._system["deadline"])
        self.write(stop)
        self.feed("STOP locked")
        self.assertTrue(self.g.start())
        self.c._on_written("!START", self.c._connection_id)
        self.assertFalse(self.g._system["written"])
        self.write()
        self.assertTrue(self.g._system["written"])

    def test_shared_mode_counter_and_owned_query_prevent_two_mode_transactions(self):
        self.ready_session()
        self.c._mode_request_counter = 100
        self.g.rest()
        self.assertEqual(self.g._mode_query["request"], 101)
        self.assertFalse(self.c.send_cmd("#GET_CMDMODE 101"))
        self.c._query_command_mode()
        self.assertEqual([command for _gen, command in self.take()], ["#GET_CMDMODE 101"])

    def test_claimed_shared_joint_polling_updates_actual_and_preserves_targets(self):
        self.ready_session()
        target = list(self.g.target_tcp)
        reference = list(self.g.command_reference_tcp)
        edit = [field.get() for field in self.c.ent_joints]
        synced = []
        self.c.position_synced.connect(lambda *args: synced.append(args))
        self.c.set_polling(True, 30)
        self.assertEqual(self.c._poll_timer.interval(), 34)
        for _ in range(20):
            self.clock.advance(.2)
            self.sample()
            self.g.tick()
            self.c._poll_position()
            request = self.c._position_request
            self.assertEqual(request["source"], "telemetry")
            self.assertFalse(request["sync"])
            for _ in range(30):
                self.c._poll_position()
            self.assertEqual(self.write(), ["#GETJPOS"])
            self.feed("ok 2 9 78 3 87 -2 0")
        self.assertEqual(self.c.actual_pose, [2, 9, 78, 3, 87, -2, 0])
        self.assertEqual(synced, [])
        self.assertEqual([field.get() for field in self.c.ent_joints], edit)
        for source in ("ui", "telemetry", "gamepad"):
            for command in ("#GETJPOS", "#GETLPOS", "#GETRAILPOS", "#GETJPOS\tignored"):
                # Direct, untracked queries cannot bypass the shared request path.
                self.assertFalse(self.c.send_cmd(command, source=source), (source, command))
                self.assertFalse(self.c._can_write(self.c._tx_generation, command))
            for kind in ("j", "l"):
                if source != "telemetry" or kind != "j":
                    self.assertFalse(self.c._request_position(kind, False, source=source))
            self.assertFalse(self.c._request_position("j", True, source=source))
        self.assertEqual(self.take(), [])
        self.assertIsNone(self.c._position_request)
        self.assertEqual(self.g.target_tcp, target)
        self.assertEqual(self.g.command_reference_tcp, reference)
        self.assertIsNone(self.g.actual_tcp)
        self.assertTrue(self.g.ready)

    def test_shared_poll_off_timeout_stop_and_failed_enqueue_do_not_change_session(self):
        self.ready_session()
        self.c.set_polling(False, 30)
        self.c._poll_position()
        self.assertEqual(self.take(), [])
        self.assertFalse(self.c._poll_timer.isActive())
        self.c.set_polling(True, 30)
        self.c._poll_position()
        request = self.c._position_request
        self.assertEqual(self.write(), ["#GETJPOS"])
        self.c._position_timeout(request["token"])
        self.assertTrue(self.g.ready)
        self.assertEqual(self.take(), [])
        self.assertTrue(self.c.state()["actual_stale"])
        self.c._poll_position()
        queued = self.take()[0]
        self.assertFalse(self.c.read_and_sync_from_robot())
        self.assertFalse(self.c._position_request["sync"])
        self.g.stop()
        self.assertFalse(self.c._can_write(*queued))
        self.c._position_timeout(request["token"])
        self.c._poll_position()
        self.assertEqual(self.write(), ["!STOP"])
        self.feed("STOP locked")
        self.assertFalse(self.c._request_position("j", False, source="telemetry"))
        self.assertIsNone(self.c._position_request)
        self.assertIsNone(self.c._pending_sync)

    def test_ui_stop_is_priority_and_releasing_does_not_send_a_second_stop(self):
        self.ready_session()
        self.g.set_target(self.target(1))
        self.g.tick()
        old_generation = self.c._tx_generation
        self.assertTrue(self.c.send_cmd("!STOP"))
        self.g.release()
        self.g.release()
        self.assertEqual(self.write(), ["!STOP"])
        self.assertFalse(self.c._can_write(old_generation, "@147.5,0,288.9,180,0,180,10"))
        self.feed("accepted MOVEL id=42", "ok CMDMODE mode=2", "ok MOVEJ id=41", "STOP locked")
        self.assertEqual(self.take(), [])
        self.assertFalse(self.g.claimed)
        self.assertEqual(self.c.state()["control_owner"], "ui")

    def test_device_disconnect_never_sends_to_closed_link_and_clears_caches(self):
        self.ready_session()
        old_connection = self.c._connection_id
        self.c.disconnect_port()
        self.g.release()
        self.g.tick()
        self.c._on_transport_line("ok 10 20 30 0 0 0", old_connection)
        self.assertFalse(self.g.claimed)
        self.assertIsNone(self.c.actual_cartesian_pose)
        self.assertIsNone(self.g.actual_tcp)
        self.assertEqual(self.take(), [])

    def test_diagnostic_idle_is_not_a_stall_and_real_backoff_cancels_without_extra_stop(self):
        self.ready_session()
        self.feed("STALL node=8 EXCLUDED", "STALL node=1 phase=IDLE")
        self.assertTrue(self.g.ready)
        self.feed("STALL node=6 phase=BACKOFF")
        self.assertFalse(self.g.ready)
        self.assertEqual(self.g.stage, "protected")
        self.assertTrue(self.c._stop_locked)
        self.assertEqual(self.take(), [])

    def test_delayed_writer_coalesces_motion_while_gripper_throttle_uses_real_write(self):
        self.ready_session()
        self.g.set_target(self.target(1))
        self.g.set_gripper_base(50)
        pending = self.take()
        self.clock.advance(.15)
        self.sample()
        self.write(pending)
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=42", "ok hand pos=50")
        self.g.set_target(self.target(2))
        self.g.gripper_target = 51
        self.g.tick()
        commands = self.write()
        self.assertIn("@148.5,0,331.2,180,0,180,10", commands)
        self.assertNotIn("!HAND_POS 51", commands)
        self.feed("queued MOVEL id=43 mode=INT", "accepted MOVEL id=43", "ok 0 10 80 0 90 0 37.25")
        self.clock.advance(.051)
        self.sample()
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.clock.advance(.05)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["!HAND_POS 51"])

    def test_retired_written_position_timeout_is_discarded_without_stop(self):
        self.ready_session()
        token = 88
        self.c._position_request = {"kind": "j", "sync": False, "token": token,
                                    "generation": self.c._tx_generation, "source": "gamepad",
                                    "written_at": self.clock()}
        self.c._pending_sync = "j"
        self.assertTrue(self.g.initialize())
        self.assertIsNone(self.c._position_request)
        self.assertIsNone(self.c._pending_sync)
        self.c._position_timeout(token)
        self.assertEqual(self.g.stage, "joint_queued")
        commands = self.write()
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0], ">0,10,80,0,90,0,0,10")
        self.assertNotIn("!STOP", self.writes)
        # A stray legacy deadline can clear only its own retired request.
        self.c._position_request = {"kind": "j", "sync": False, "token": token,
                                    "source": "gamepad", "written_at": self.clock()}
        self.c._position_timeout(token)
        self.assertIsNone(self.c._position_request)
        self.assertEqual(self.g.stage, "joint_queued")
        self.assertEqual(self.take(), [])

    def test_retired_unwritten_query_cannot_send_or_stop_new_generation(self):
        self.ready_session()
        token = 89
        old_generation = self.c._tx_generation
        self.c._position_request = {"kind": "j", "sync": False, "token": token,
                                    "generation": old_generation, "source": "gamepad", "written_at": None}
        self.c._pending_sync = "j"
        self.assertTrue(self.g.initialize())
        self.assertFalse(self.c._can_write(old_generation, "#GETJPOS"))
        self.c._position_unwritten_timeout(token)
        self.c._on_written("#GETJPOS", self.c._connection_id)
        self.assertIsNone(self.c._position_request)
        self.assertEqual(self.g.stage, "joint_queued")
        self.assertEqual(self.c.state()["control_owner"], "gamepad")
        self.assertFalse(any(callback == self.c._position_timeout and args == (token,)
                             for _delay, callback, args in self.c.root.calls.values()))

    def test_early_accepted_advances_only_written_immutable_stream_reference(self):
        self.ready_session()
        initial = list(self.g.command_reference_tcp)
        actual = list(self.c.actual_pose) if self.c.actual_pose is not None else None
        self.assertTrue(self.g.set_target(self.target(1)))
        self.g.tick()
        pending = self.take()
        self.assertTrue(self.g.set_target(self.target(2)))
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=42")
        self.assertEqual(self.g.command_reference_tcp, initial)
        self.assertIsNotNone(self.g._stream)
        self.write(pending)
        self.assertAlmostEqual(self.g.command_reference_tcp[0], 147.5)
        self.assertAlmostEqual(self.g.target_tcp[0], 148.520190203)
        self.assertIsNone(self.g._stream)
        self.assertIsNone(self.g.actual_tcp)
        self.assertEqual(self.c.actual_pose, actual)
        self.c._poll_position()
        self.assertEqual(self.write(), ["#GETJPOS"])
        self.feed("ok 2 9 78 3 87 -2 0")
        self.assertAlmostEqual(self.g.command_reference_tcp[0], 147.5)
        self.assertAlmostEqual(self.g.target_tcp[0], 148.520190203)
        self.clock.advance(.051)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["@148.5,0,331.2,180,0,180,10"])
        self.g.stop()
        self.feed("accepted MOVEL id=43")
        self.assertIsNone(self.g.command_reference_tcp)

    def test_manual_lpos_query_remains_available_without_gamepad_ownership(self):
        self.assertTrue(self.c.read_pose_from_robot())
        self.assertEqual(self.write(), ["#GETLPOS"])
        self.feed("ok 146.52 0 288.92 180 0 180")
        self.assertEqual(self.c.actual_cartesian_pose, [146.52, 0., 288.92, 180., 0., 180.])
        self.assertIsNone(self.g.actual_tcp)

    def test_gamepad_position_source_is_retired_even_without_ownership(self):
        for kind in ("j", "l"):
            self.assertFalse(self.c._request_position(kind, False, source="gamepad"))
        for command in ("#GETJPOS", "#GETLPOS", "#GETRAILPOS"):
            self.assertFalse(self.c.send_cmd(command, source="gamepad"))
        self.assertEqual(self.take(), [])
        self.assertIsNone(self.c._position_request)
        self.assertTrue(self.c.read_and_sync_from_robot())
        token = self.c._position_request["token"]
        self.assertFalse(self.g.claim())
        self.c._position_timeout(token)
        self.assertTrue(self.g.claim())
        self.assertNotIn("!STOP", [command for _gen, command in self.take()])


if __name__ == "__main__":
    unittest.main()
