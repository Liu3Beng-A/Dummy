"""Fake-wire integration: no real serial or device access."""
import queue
import tempfile
import unittest
import numpy as np
from PySide6.QtCore import QCoreApplication
from app.core.controller import RobotController
from app.core.business import TeachingPoint
from app.core.cartesian_backend import CartesianBackend
from app.storage.store import AppStore, validate_points
from dummy_motion import Tool, Pose
from dummy_motion.frames import from_wire_joints, to_wire_joints, to_xyzabc
from tests.test_controller import ManualScheduler, FakePort


class CartesianSessionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.now = 10.
        self.c = RobotController(scheduler=ManualScheduler())
        self.s = self.c.cartesian
        self.s.clock = lambda: self.now
        self.s._timer.stop()
        self.c._poll_timer.stop()
        self.q = from_wire_joints([0, 10, 80, 0, 90, 0, 0])
        self.s.switch("host")
        self.c.is_connected = True
        self.c.serial_port = FakePort()
        self.c._motor_state_known = self.c._motor_enabled = True
        self.c.current_cmdmode = 2
        self.s.update_feedback(to_wire_joints(self.q))
        self.logs = []
        self.c.log_message.connect(lambda message, tag: self.logs.append(message))

    def tearDown(self):
        self.c.close()

    def take(self):
        values = []
        while not self.c._tx_queue.empty():
            item = self.c._tx_queue.get_nowait()
            if item: values.append(item)
        return values

    def write(self, item):
        generation, command = item
        self.assertTrue(self.c._can_write(generation, command), command)
        self.c._on_written(command, self.c._connection_id)
        return command

    def point(self, delta=.004):
        pose = self.s.model.fk(self.q, Tool(Pose.identity()), frame="base")
        target = Pose(pose.position+[0, delta, 0], pose.rotation)
        return TeachingPoint("line", "MOVEL", tuple(to_xyzabc(target)), 100., 35.)

    def start(self, points=None, sequence=False):
        self.assertTrue(self.s.start_points(points or (self.point(),), sequence=sequence))
        command = self.write(self.take()[0])
        request = int(command.split()[-1])
        self.c._process_rx_line(f"ok CMDMODE mode=2 busy=0 issued=100 req={request}")
        self.s.future.result(timeout=10)
        self.s.tick()
        self.assertEqual(self.s.stage, "running", self.logs)

    def test_shared_poll_survives_host_motion_for_more_than_stale_window(self):
        self.start((self.point(.02),))
        actual, motion_id, motions = self.q.copy(), 100, 0
        for _ in range(40):
            self.now += .05
            self.c._poll_position()
            poll = self.take()
            self.assertEqual([cmd for _, cmd in poll], ["#GETJPOS"], self.logs)
            self.write(poll[0])
            self.c._process_rx_line("ok " + " ".join(map(str, to_wire_joints(actual))))
            self.assertIsNone(self.c._position_request)
            self.s.tick()
            if self.s.future:
                self.s.future.result(timeout=5)
                self.s.tick()
            for item in self.take():
                command = self.write(item)
                self.assertTrue(command.startswith(">"), command)
                actual = from_wire_joints([float(x) for x in command[1:].split(",")[:7]])
                motion_id += 1
                motions += 1
                self.c._process_rx_line(f"queued MOVEJ id={motion_id} mode=INT")
                self.c._process_rx_line(f"accepted MOVEJ id={motion_id}")
            self.assertTrue(self.s.active, self.logs)
        self.assertGreaterEqual(motions, 5)
        self.assertAlmostEqual(self.s.feedback.sample.received_at, self.now)
        self.assertFalse(any("处理中" in message or "300 ms" in message for message in self.logs))

    def test_pending_shared_poll_does_not_block_path_start_or_backend_switch(self):
        self.c._poll_position()
        self.assertTrue(self.s.start_points((self.point(),)), self.logs)
        self.s.cancel()
        self.take()
        self.c._position_request = None
        self.c._poll_position()
        self.assertTrue(self.s.switch("firmware"), self.logs)
        for item in self.take():
            self.write(item)

    def test_joint_reply_before_written_does_not_block_new_host_path(self):
        self.c._poll_position()
        item = self.take()[0]
        self.assertTrue(self.c._can_write(*item))
        self.c._process_rx_line("ok " + " ".join(map(str, to_wire_joints(self.q))))
        self.assertIsNone(self.c._position_request)
        self.assertFalse(self.c._wire_admitted.empty())
        self.assertTrue(self.s.start_points((self.point(),)), self.logs)
        self.c._on_written(item[1], self.c._connection_id)
        self.assertTrue(self.c._wire_admitted.empty())

    def test_shared_poll_allowed_while_switch_is_waiting(self):
        self.assertTrue(self.s.switch("firmware"))
        self.c._poll_position()
        items = self.take()
        self.assertEqual([cmd for _, cmd in items][-1], "#GETJPOS", self.logs)
        for item in items:
            self.write(item)

    def test_active_host_does_not_allow_ui_sync_to_mutate_pending_poll(self):
        self.start()
        self.c._poll_position()
        self.assertIsNotNone(self.c._position_request, self.logs)
        self.assertFalse(self.c.read_and_sync_from_robot())
        self.assertFalse(self.c._position_request["sync"])
        self.assertEqual(self.c._position_request["source"], "telemetry")
        self.assertFalse(self.c.send_cmd("#GETJPOS", source="ui"))
        self.assertFalse(self.c.send_cmd("!HAND_POS 50"))

    def test_startup_380ms_gap_then_reply_does_not_cancel_before_motion(self):
        self.assertTrue(self.s.start_points((self.point(),)))
        command = self.write(self.take()[0])
        self.now += .379
        self.s.tick()  # Same order as the log: timer, joint reply, mode reply.
        self.assertTrue(self.s.active, self.logs)
        self.assertIsNone(self.s.future)
        self.assertFalse(any(cmd.startswith(">") for _, cmd in self.take()))
        self.now += .001
        self.c._process_rx_line("ok 0 10 80 0 90 0 -0.00")
        self.now += .012
        self.c._process_rx_line(f"ok CMDMODE mode=2 busy=0 issued=100 req={command.split()[-1]}")
        self.s.future.result(timeout=5)
        self.s.tick()
        self.s.tick()
        self.assertTrue(any(cmd.startswith(">") for _, cmd in self.take()), self.logs)

    def test_mode_reply_with_old_feedback_defers_planning_until_new_sample(self):
        self.assertTrue(self.s.start_points((self.point(),)))
        command = self.write(self.take()[0])
        self.now += .4
        self.c._process_rx_line(f"ok CMDMODE mode=2 busy=0 issued=100 req={command.split()[-1]}")
        self.assertIsNone(self.s.future)
        self.assertTrue(self.s.active)
        self.c._process_rx_line("ok 0 10 80 0 90 0 -0.00")
        self.s.tick()
        self.assertIsNotNone(self.s.future)

    def test_startup_feedback_wait_has_deadline_and_late_data_does_not_resume(self):
        self.assertTrue(self.s.start_points((self.point(),)))
        command = self.write(self.take()[0])
        self.now += .4
        self.c._process_rx_line(f"ok CMDMODE mode=2 busy=0 issued=100 req={command.split()[-1]}")
        self.s.tick()
        self.assertTrue(self.s.active)
        self.now += 3.01
        self.s.tick()
        self.assertFalse(self.s.active)
        self.assertIn("启动前", self.s.message)
        self.c._process_rx_line("ok 0 10 80 0 90 0 -0.00")
        self.s.tick()
        self.assertIsNone(self.s.future)
        self.assertFalse(any(cmd.startswith(">") or cmd == "!STOP" for _, cmd in self.take()))

    def test_first_motion_writer_refuses_feedback_that_aged_while_queued(self):
        self.start()
        self.s.tick()
        item = self.take()[0]
        self.now += .31
        self.assertFalse(self.c._can_write(*item))

    def test_firmware_is_default_and_plain_movel_retains_wire_semantics(self):
        c = RobotController(scheduler=ManualScheduler())
        try:
            self.assertEqual(c.cartesian.backend, CartesianBackend.FIRMWARE)
            c.is_connected, c.serial_port = True, FakePort()
            self.assertTrue(c.send_cmd("@150,0,280,180,0,180,35"))
            self.assertEqual(c._tx_queue.get()[1], "@150,0,280,180,0,180,35")
        finally:
            c.close()

    def test_switch_waits_for_written_numbered_idle_response_and_never_moves(self):
        self.assertTrue(self.s.switch("firmware"))
        item = self.take()[0]
        request = int(item[1].split()[-1])
        self.c._process_rx_line(f"ok CMDMODE mode=2 busy=0 issued=100 req={request}")
        self.assertEqual(self.s.backend, CartesianBackend.HOST)
        self.write(item)
        self.assertEqual(self.s.backend, CartesianBackend.FIRMWARE)
        self.assertTrue(all(not cmd.startswith((">", "@")) for _, cmd in self.take()))

    def test_busy_switch_rejected_and_no_automatic_fallback(self):
        self.s.switch("firmware")
        command = self.write(self.take()[0])
        request = int(command.split()[-1])
        self.c._process_rx_line(f"ok CMDMODE mode=2 busy=1 issued=100 req={request}")
        self.assertEqual(self.s.backend, CartesianBackend.HOST)
        self.assertFalse(self.s.busy)

    def test_terminal_at_entry_is_routed_and_plan_precedes_movej(self):
        point = self.point()
        command = "@"+",".join(map(str, (*point.target, point.speed)))
        self.assertTrue(self.c.send_cmd(command))
        commands = self.take()
        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0][1].startswith("#GET_CMDMODE"))
        self.assertEqual(self.s.stage, "checking")

    def test_accepted_not_completed_and_receipt_before_written(self):
        self.start()
        self.s.tick()
        item = self.take()[0]
        self.c._process_rx_line("queued MOVEJ id=101 mode=INT")
        self.c._process_rx_line("accepted MOVEJ id=101")
        self.assertIsNone(self.s.receipts[0]["id"])
        self.write(item)
        self.assertEqual(len(self.s.receipts), 0)
        self.assertTrue(self.s.active)
        self.assertFalse(self.s.terminal_complete)

    def test_stop_cancels_worker_and_queued_generation(self):
        self.start()
        self.s.tick()
        item = self.take()[0]
        self.assertTrue(self.c.send_cmd("!STOP"))
        self.assertFalse(self.s.active)
        self.assertFalse(self.c._can_write(*item))
        self.assertEqual([cmd for _, cmd in self.take()], ["!STOP"])
        self.c._process_rx_line("queued MOVEJ id=101 mode=INT")
        self.c._process_rx_line("accepted MOVEJ id=101")
        self.s.tick()
        self.assertFalse(self.s.active)

    def test_stale_feedback_stops_after_motion_write(self):
        self.start()
        self.s.tick()
        self.write(self.take()[0])
        self.now += .31
        self.s.tick()
        self.assertFalse(self.s.active)
        self.assertTrue(any(cmd == "!STOP" for _, cmd in self.take()))

    def test_complete_mixed_sequence_requires_final_id_and_stable_actual(self):
        joint = TeachingPoint("joint", "MOVEJ", tuple(to_wire_joints(self.q)), 100., 20.)
        self.start((self.point(.002), joint, self.point(.003)), sequence=True)
        motion_id = 100
        actual = self.q.copy()
        status_reads = 0
        for _ in range(250):
            self.now += .05
            self.s.update_feedback(to_wire_joints(actual))
            self.s.tick()
            if self.s.future:
                self.s.future.result(timeout=5)
                self.s.tick()
            for item in self.take():
                cmd = self.write(item)
                if cmd.startswith(">"):
                    actual = from_wire_joints([float(value) for value in cmd[1:].split(",")[:7]])
                    motion_id += 1
                    self.c._process_rx_line(f"queued MOVEJ id={motion_id} mode=INT")
                    self.c._process_rx_line(f"accepted MOVEJ id={motion_id}")
                elif cmd == "#MOTION_STATUS":
                    status_reads += 1
                    self.c._process_rx_line(f"MOTION id={motion_id-1} kind=MOVEJ phase=3 mode=2 ms=50")
                    self.assertFalse(self.s.terminal_complete)
                    self.c._process_rx_line(f"MOTION id={motion_id} kind=MOVEJ phase=3 mode=2 ms=100")
            if not self.s.active:
                break
        self.assertEqual(self.s.stage, "complete", self.logs)
        self.assertFalse(self.c._seq_active)
        self.assertEqual(status_reads, 3)

    def test_cancel_sequence_tail_preserves_only_current_point(self):
        self.start((self.point(.002), self.point(.004)), sequence=True)
        self.c._stop_sequence("cancel tail")
        self.assertTrue(self.s.active)
        self.assertEqual(len(self.s.plans), 1)
        self.s.tick()
        self.assertTrue(any(command.startswith(">") for _, command in self.take()))
        self.assertFalse(self.c._seq_active)

    def test_cancel_sequence_before_preflight_sends_no_motion(self):
        self.assertTrue(self.s.start_points((self.point(),), sequence=True))
        item = self.take()[0]
        self.c._stop_sequence("cancel tail")
        self.assertFalse(self.s.active)
        self.assertFalse(self.s.can_write(*item))

    def test_host_world_point_bypasses_native_movel_formatter(self):
        from unittest.mock import patch
        target = self.s.model.fk(self.q)
        record = {"name": "world", "type": "MoveL", "pose": to_xyzabc(target),
                  "frame": "world", "target_kind": "tcp", "tool": [0, 0, 114, 0, 0, 0]}
        with patch.object(self.c, "_teaching_command", side_effect=AssertionError("native formatter")):
            self.assertTrue(self.c.send_typed_position(record))
        self.assertTrue(self.take()[0][1].startswith("#GET_CMDMODE"))

    def test_unreachable_path_sends_no_motion(self):
        bad = TeachingPoint("bad", "MOVEL", (900., 900., 900., 0., 0., 0.), 100., 35.)
        self.assertTrue(self.s.start_points((bad,)))
        command = self.write(self.take()[0])
        self.c._process_rx_line(f"ok CMDMODE mode=2 busy=0 issued=100 req={command.split()[-1]}")
        with self.assertRaises(ValueError): self.s.future.result(timeout=15)
        self.s.tick()
        self.assertFalse(self.s.active)
        self.assertEqual(self.take(), [])
        self.assertEqual(self.s.backend, CartesianBackend.HOST)

    def test_model_feedback_does_not_modify_edit_target(self):
        before = [field.get() for field in self.c.ent_joints]
        self.now += .02
        self.c._process_rx_line("ok 0 10 80 0 90 0 30")
        self.assertEqual([field.get() for field in self.c.ent_joints], before)
        self.assertEqual(self.c.actual_pose[-1], 30)
        self.assertAlmostEqual(self.s.feedback.sample.q[-1], .03)

    def test_disconnect_cancels_worker_and_forbids_old_write(self):
        self.start()
        self.s.tick()
        item = self.take()[0]
        self.c.disconnect_port()
        self.assertFalse(self.c._can_write(*item))
        self.assertFalse(self.s.active)

    def test_legacy_and_world_points_keep_different_semantics_and_tool_snapshots(self):
        old = {"name": "old", "type": "MoveL", "pose": list(self.point().target)}
        new = dict(old, name="world", frame="world", target_kind="tcp", tool=[0, 0, 114, 0, 0, 0])
        records = validate_points([old, new])
        self.assertNotIn("frame", records[0])
        self.assertEqual(records[1]["tool"], new["tool"])
        new["tool"][2] = 999
        self.assertEqual(records[1]["tool"][2], 114)
        with tempfile.TemporaryDirectory() as directory:
            store = AppStore(directory)
            self.assertEqual(store.settings["cartesian_backend"], "firmware")
            store.save_points(records)
            loaded = AppStore(directory)
            self.assertEqual(loaded.points, records)
            loaded.close()
            store.close()

    def test_host_gamepad_stream_uses_movej_and_bounded_lead(self):
        self._exercise_host_jog((.02,))

    def test_host_gamepad_jitter_keeps_packet_bound_and_receipt_floor(self):
        self._exercise_host_jog((.02, .05, .08, .03))

    def _exercise_host_jog(self, intervals):
        g = self.c.gamepad
        g.claimed, g.device_connected, g.stage = True, True, "ready"
        self.c._control_owner = "gamepad"
        target = self.s.model.fk(self.q)
        tcp = to_xyzabc(Pose(target.position+[0, .04, 0], target.rotation))
        self.assertTrue(self.s.jog(tcp))
        actual, motion_id, writes = self.q.copy(), 100, []
        for index in range(50):
            self.now += intervals[index % len(intervals)]
            self.s.update_feedback(to_wire_joints(actual))
            self.s.tick()
            if self.s.future:
                self.s.future.result(timeout=5)
            for item in self.take():
                cmd = self.write(item)
                self.assertTrue(cmd.startswith(">"), cmd)
                next_q = from_wire_joints([float(value) for value in cmd[1:].split(",")[:7]])
                step = np.linalg.norm(self.s.model.fk(next_q).position-self.s.model.fk(actual).position)
                self.assertLessEqual(step, .0022)
                actual = next_q
                motion_id += 1
                self.c._process_rx_line(f"queued MOVEJ id={motion_id} mode=INT")
                self.c._process_rx_line(f"accepted MOVEJ id={motion_id}")
                self.assertEqual(g._issued, motion_id)
                writes.append(self.now)
        self.assertGreater(len(writes), 2, self.logs)
        self.assertTrue(all(right-left >= .099 for left, right in zip(writes, writes[1:])))
        self.s.jog_idle()
        self.now += .02
        self.s.update_feedback(to_wire_joints(actual))
        self.s.tick()
        if self.s.future: self.s.future.result(timeout=5)
        self.now += .1
        self.s.update_feedback(to_wire_joints(actual))
        self.s.tick()
        self.assertEqual(self.take(), [])

    def test_writer_rechecks_mode_and_enable_after_planning(self):
        self.start()
        self.s.tick()
        item = self.take()[0]
        self.c.current_cmdmode = 1
        self.assertFalse(self.c._can_write(*item))
        self.c.current_cmdmode = 2
        self.c._motor_enabled = False
        self.assertFalse(self.c._can_write(*item))

    def test_stale_numbered_failure_cannot_cancel_current_path(self):
        self.start()
        self.c._process_rx_line("MOTION fail id=99 reason=STALL")
        self.assertTrue(self.s.active)

    def test_failure_of_written_path_preserves_device_protection(self):
        self.start()
        self.s.tick()
        self.write(self.take()[0])
        self.c._process_rx_line("queued MOVEJ id=101 mode=INT")
        self.c._process_rx_line("accepted MOVEJ id=101")
        self.c._process_rx_line("MOTION fail id=101 reason=STALL")
        self.assertFalse(self.s.active)
        self.assertTrue(self.c._stop_locked)
        self.assertFalse(any(cmd == "!STOP" for _, cmd in self.take()))


    def test_quantized_plateaus_are_collapsed(self):
        self.start((self.point(.04),))
        self.assertGreater(len(self.s.samples), 2)
        for a, b in zip(self.s.samples, self.s.samples[1:]):
            self.assertFalse(np.array_equal(a.q, b.q))

    def test_delayed_write_starts_spacing_after_write_not_enqueue(self):
        self.start((self.point(.02),))
        self.s.tick()
        item = self.take()[0]
        self.now += .25
        self.s.update_feedback(to_wire_joints(self.q))
        self.write(item)
        self.assertGreaterEqual(self.s.next_due, self.now+.1)
        self.c._process_rx_line("queued MOVEJ id=101 mode=INT")
        self.c._process_rx_line("accepted MOVEJ id=101")
        self.s.tick()
        if self.s.future:
            self.s.future.result(timeout=5)
            self.s.tick()
        self.assertFalse(any(command.startswith(">") for _, command in self.take()))

    def test_late_qt_delivery_does_not_make_old_rx_fresh(self):
        self.now += .5
        before = self.c.actual_pose
        self.c._process_rx_line("ok 0 10 80 0 90 0 20", received_at=self.now-.4)
        self.assertEqual(self.c.actual_pose, before)
        from dummy_motion import Status
        self.assertEqual(self.s.feedback.estimate(self.now)[0], Status.FEEDBACK_STALE)
        self.assertTrue(any("RX到处理 400 ms" in line for line in self.logs))

    def test_rejected_joints_do_not_update_view_or_sync_target(self):
        self.now += .01
        self.c._process_rx_line("ok 0 10 80 0 90 0 20")
        before = list(self.c.actual_pose)
        self.now += .01
        self.c._process_rx_line("ok 200 10 80 0 90 0 20")
        self.assertEqual(self.c.actual_pose, before)
        self.assertTrue(any("超出模型限位" in line for line in self.logs))

    def test_speed_setting_changes_preflight_time(self):
        from app.core.cartesian_backend import host_limits
        from dummy_motion import LinePlanner
        tool = Tool(Pose.identity())
        start = self.s.model.fk(self.q, tool)
        target = Pose(start.position+[0,.02,0],start.rotation)
        slow = LinePlanner(self.s.model).plan_line(self.q,target,host_limits(10),tool)
        fast = LinePlanner(self.s.model).plan_line(self.q,target,host_limits(50),tool)
        self.assertTrue(slow.ok and fast.ok)
        self.assertGreater(slow.samples[-1].time, 3*fast.samples[-1].time)
        self.assertAlmostEqual(host_limits(35).linear_speed, .007)



    def test_duplicate_terminal_reuses_real_receipt_without_second_movej(self):
        from dummy_motion import Sample
        self.start()
        self.s.tick()
        self.write(self.take()[0])
        self.c._process_rx_line("queued MOVEJ id=101 mode=INT")
        self.c._process_rx_line("accepted MOVEJ id=101")
        q = self.s.last_written_q
        self.assertTrue(self.s._write_sample(Sample(.1,q,self.s.model.fk(q,self.s.tool)),True))
        self.assertEqual(self.s.terminal_id,101)
        self.assertFalse(self.s.terminal_complete)
        self.assertEqual(self.take(),[])

    def test_nan_reply_cannot_enter_legacy_sync(self):
        self.now += .01
        before = self.s.feedback.sample
        self.c._pending_sync = "j"
        self.c._process_rx_line("ok nan 10 80 0 90 0 0")
        self.assertIs(self.s.feedback.sample,before)
        self.assertIsNone(self.c.actual_pose)
        self.assertTrue(any("无效" in line for line in self.logs))



    def _timed_sample(self, degrees=1.):
        from dummy_motion import Sample
        q = self.q.copy(); q[0] += np.deg2rad(degrees)
        return Sample(.2, q, self.s.model.fk(q,self.s.tool))

    def test_host_speed_not_forwarded_to_movej_for_identical_timed_segment(self):
        self.start()
        sample = self._timed_sample()
        speeds = []
        for ui_speed in (5.,35.,100.):
            self.s.speed_value = ui_speed
            self.assertTrue(self.s._write_sample(sample,False,.2))
            speeds.append(float(self.take()[0][1].split(',')[-1]))
        self.assertEqual(speeds,[speeds[0]]*3)
        self.assertNotIn(speeds[0],(5.,35.,100.))
        self.assertTrue(self.s._write_sample(sample,False,.4))
        self.assertLess(float(self.take()[0][1].split(',')[-1]),speeds[0])

    def test_explicit_movej_point_keeps_own_speed(self):
        p = TeachingPoint("joint","MOVEJ",tuple(to_wire_joints(self.q)),100.,35.)
        self.start((p,))
        self.s.tick()
        self.assertEqual(float(self.take()[0][1].split(',')[-1]),35.)

    def test_host_jog_does_not_forward_cmd_speed(self):
        self.start()
        self.s.kind = "jog"
        sample = self._timed_sample()
        self.s.speed_value = 100.
        self.assertTrue(self.s._write_sample(sample,False,.2))
        self.assertNotEqual(float(self.take()[0][1].split(',')[-1]),100.)

    def test_current_segment_duration_includes_collapsed_time(self):
        from dataclasses import replace
        self.start()
        a = self._timed_sample(.1)
        b = self._timed_sample(.2)
        self.s.samples = (replace(a,time=.4),replace(b,time=.5))
        self.s.point_candidate = self.s.samples[0]
        self.s.tick()
        self.assertAlmostEqual(self.s.last_segment_timing.requested_seconds,.4)
        self.assertAlmostEqual(self.s.job['dt'],.4)

    def test_motor_limit_extends_write_cadence(self):
        self.start()
        self.s.motor_acceleration[:] = .1
        self.assertTrue(self.s._write_sample(self._timed_sample(),False,.1))
        self.assertTrue(self.s.last_segment_timing.duration_limited)
        interval = self.s.job['dt']
        self.assertGreater(interval,.1)
        self.write(self.take()[0])
        self.assertAlmostEqual(self.s.next_due-self.now,interval)

    def test_acceleration_readback_validation_and_write_invalidation(self):
        for line in ('ok ACC [1] = .. r/s^2','ok ACC [1] = nan r/s^2',
                     'ok ACC [1] = 1e999 r/s^2','ok ACC [1] = 0 r/s^2'):
            self.s.handle_line(line)
        self.assertFalse(self.s.acceleration_confirmed.any())
        self.s.handle_line('ok ACC [7] = 12.50 r/s^2 (rail)')
        self.s.handle_line('MOTION J2 v=0.010 a=20.000 d=0.004')
        self.assertEqual(self.s.motor_acceleration[6],12.5)
        self.assertEqual(self.s.motor_acceleration[1],20.)
        self.assertTrue(self.c.send_cmd('#ACC_J 7 8'))
        self.write(self.take()[0])
        self.assertFalse(self.s.acceleration_confirmed[6])
        self.assertTrue(self.s.acceleration_confirmed[1])
        self.s.invalidate_acceleration('#REBOOT 0')
        self.assertFalse(self.s.acceleration_confirmed.any())


if __name__ == '__main__':
    unittest.main()
