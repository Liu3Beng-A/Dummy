"""v2 held-reference seed, spawn isolation, cancellation and recovery regressions."""
import os
import time
import unittest
import numpy as np
from PySide6.QtCore import QTimer
from PySide6.QtTest import QTest
from app.core.planning_process import PlanningProcess
from dummy_motion import RobotModel, Tool, Pose
from dummy_motion.frames import from_wire_joints, to_xyzabc
from dummy_motion.stream_protocol import decode_knot
from tests.test_ui import APPLICATION
from tests import test_stream_transport as transport_tests


class PlanningProcessTests(unittest.TestCase):
    def request(self, distance=.025):
        model=RobotModel();q=from_wire_joints([0,10,80,0,90,0,0]);pose=model.fk(q)
        point=dict(kind='MOVEL',target=to_xyzabc(Pose(pose.position+[0,distance,0],pose.rotation)),
                   tool=None,frame='world',target_kind='tcp')
        return model,point,35.,Tool(),q,np.full(7,30.),np.full(7,30.)

    def test_real_plan_runs_in_different_process_and_gui_heartbeat_continues(self):
        ticks=[];timer=QTimer();timer.setInterval(20);timer.timeout.connect(lambda:ticks.append(time.perf_counter()))
        future=PlanningProcess(self.request());timer.start()
        try:
            self.assertNotEqual(future.process.pid,os.getpid())
            deadline=time.perf_counter()+20
            while not future.done() and time.perf_counter()<deadline:QTest.qWait(5)
            plan=future.result()
            self.assertGreater(len(plan.knots),100)
            self.assertGreater(len(ticks),10)
            self.assertLess(max(np.diff(ticks)),.2)
            self.assertFalse(future.process.is_alive())
        finally:timer.stop();future.close()

    def test_cancel_reaps_spawn_process_and_does_not_deliver_late_result(self):
        future=PlanningProcess(self.request(.08))
        self.assertTrue(future.process.is_alive())
        future.cancel()
        self.assertFalse(future.process.is_alive())
        with self.assertRaises(InterruptedError):future.done()


class StreamV2LifecycleTests(unittest.TestCase):
    setUp=transport_tests.StreamTransportTests.setUp
    tearDown=transport_tests.StreamTransportTests.tearDown
    wait_for=transport_tests.StreamTransportTests.wait_for
    ready=transport_tests.StreamTransportTests.ready
    point=transport_tests.StreamTransportTests.point
    start=transport_tests.StreamTransportTests.start

    def test_v1_capability_is_rejected_before_open_or_any_curve(self):
        self.port.version=1
        self.start()
        self.wait_for(lambda:self.c.cartesian.stage=='error')
        self.assertFalse(any(cmd.startswith(('!STREAM_OPEN','~S')) for cmd in self.port.commands()))
        self.assertTrue(any('v2' in message for _,message in self.logs))

    def test_native_residual_uses_held_reference_seed_not_actual_position(self):
        held=self.port.held.copy()
        self.port.joints[0]+=.07  # 498 motor subdivisions, valid ordinary settling tolerance.
        self.start()
        self.wait_for(lambda:self.c.cartesian.stage in ('complete','error'))
        self.assertEqual(self.c.cartesian.stage,'complete',self.logs)
        knot=next(decode_knot(cmd)[1] for cmd in self.port.commands() if cmd.startswith('~S'))
        np.testing.assert_array_equal(knot.motor[:,0],held)

    def test_each_sequence_point_rechecks_held_reference_and_uses_confirmed_seed(self):
        self.ready();self.port.actual_offset[0]=.04
        self.assertTrue(self.c.cartesian.start_points((self.point(.003),self.point(.006)),sequence=True))
        self.wait_for(lambda:self.c.cartesian.stage in ('complete','error'),timeout=15.)
        self.assertEqual(self.c.cartesian.stage,'complete',self.logs)
        commands=self.port.commands()
        groups={}
        for command in commands:
            if command.startswith('~S'):
                session,knot=decode_knot(command);groups.setdefault(session,[]).append(knot)
        self.assertEqual(len(groups),2)
        first,second=list(groups.values())
        np.testing.assert_array_equal(first[-1].motor[:,0],second[0].motor[:,0])
        self.assertGreaterEqual(commands.count('#STREAM_DIAG 1 7'),4)

    def test_stopped_session_can_exit_native_without_reenabling_motors(self):
        self.start(.01)
        self.wait_for(lambda:self.c.cartesian.stage=='running')
        self.assertTrue(self.c.send_cmd('!STOP'))
        self.wait_for(lambda:getattr(self.c,'_stop_confirmed',False))
        self.wait_for(lambda:not self.c.cartesian.stream.thread.is_alive())
        previous=self.port.session
        self.assertTrue(self.c.set_cartesian_backend('firmware'))
        self.wait_for(lambda:self.c.cartesian.backend.value=='firmware' or self.c.cartesian.stage=='error')
        self.assertEqual(self.c.cartesian.backend.value,'firmware',self.logs)
        self.assertTrue(self.c._stop_locked)
        commands=self.port.commands()
        close=commands.index(f'!STREAM_CLOSE {previous}')
        switch=commands.index('#CMDMODE 2',close)
        self.assertLess(close,switch)
        self.assertNotIn('!START',commands[commands.index('!STOP')+1:])

    def test_close_precedes_diags_after_stop_start_recovery(self):
        self.start(.01);self.wait_for(lambda:self.c.cartesian.stage=='running')
        self.assertTrue(self.c.send_cmd('!STOP'));self.wait_for(lambda:getattr(self.c,'_stop_confirmed',False))
        old=self.port.session
        self.assertTrue(self.c.send_cmd('!START'))
        self.wait_for(lambda:not self.c._stop_locked and self.c._mode_request is None)
        self.wait_for(lambda:not self.c.cartesian.stream.thread.is_alive())
        self.wait_for(lambda:time.perf_counter()-self.c.cartesian.feedback.sample.received_at<.2)
        before=len(self.port.commands())
        self.assertTrue(self.c.send_cartesian_point(self.point(.003)))
        self.wait_for(lambda:self.c.cartesian.stage in ('complete','error'))
        self.assertEqual(self.c.cartesian.stage,'complete',self.logs)
        commands=self.port.commands()[before:]
        self.assertLess(commands.index(f'!STREAM_CLOSE {old}'),commands.index('#STREAM_DIAG 1 5'))

    def test_close_application_cancels_inflight_planning_and_reaps_process(self):
        self.start(.08)
        self.wait_for(lambda:self.c.cartesian.stream.planner_future is not None)
        future=self.c.cartesian.stream.planner_future
        self.c.close()
        self.assertFalse(future.process.is_alive())
        self.assertFalse(any(cmd.startswith(('~S','!STREAM_OPEN')) for cmd in self.port.commands()))

    def test_top_mode7_and_native_mode_share_backend_and_never_send_motion(self):
        self.ready()
        self.assertTrue(self.c._set_cmdmode(7))
        self.wait_for(lambda:self.c.current_cmdmode==7 and not self.c.cartesian.busy)
        self.assertEqual(self.c.cartesian.backend.value,'stream')
        self.wait_for(lambda:not self.c.cartesian.stream.thread.is_alive())
        self.assertTrue(self.c._set_cmdmode(1))
        self.wait_for(lambda:self.c.current_cmdmode==1 and not self.c.cartesian.busy)
        self.assertEqual(self.c.cartesian.backend.value,'firmware')
        self.assertFalse(any(cmd.startswith(('!STREAM_OPEN','~S','>','@')) for cmd in self.port.commands()))

    def test_busy_mode_switch_is_explicit_and_does_not_change_mode(self):
        self.ready();self.port.busy=True
        self.assertTrue(self.c._set_cmdmode(7))
        self.wait_for(lambda:self.c.cartesian.stage=='error')
        self.assertEqual(self.port.mode,2)
        self.assertTrue(any('仍在运动' in message for _,message in self.logs))
        self.assertFalse(any('Malformed' in message for _,message in self.logs))

    def test_terminal_mode_exit_uses_same_close_gate_while_stop_locked(self):
        self.start(.01);self.wait_for(lambda:self.c.cartesian.stage=='running')
        self.assertTrue(self.c.send_cmd('!STOP'))
        self.wait_for(lambda:getattr(self.c,'_stop_confirmed',False))
        self.wait_for(lambda:not self.c.cartesian.stream.thread.is_alive())
        old=self.port.session
        self.assertTrue(self.c.send_cmd('#CMDMODE 2'))
        self.wait_for(lambda:self.c.cartesian.backend.value=='firmware' or self.c.cartesian.stage=='error')
        self.assertEqual(self.c.cartesian.backend.value,'firmware',self.logs)
        self.assertIn(f'!STREAM_CLOSE {old}',self.port.commands())
        self.assertTrue(self.c._stop_locked)

    def test_reconnect_mode7_with_native_preference_never_writes_plain_or_teaching_motion(self):
        self.assertTrue(self.c.connect_port('MEMORY_NATIVE_ONLY','USB'))
        self.wait_for(lambda:self.c._mode_request is None)
        self.c.disconnect_port()
        self.port=transport_tests.StreamMemoryPort();self.port.mode=7
        self.c._serial_factory=lambda *args,**kwargs:self.port
        self.assertTrue(self.c.connect_port('MEMORY_RECONNECT_ONLY','USB'))
        self.wait_for(lambda:self.c.current_cmdmode==7)
        self.assertEqual(self.c.cartesian.backend.value,'firmware')
        for command in ('@146.5,0,288.9,180,0,180,35','>0,10,80,0,90,0,0,35','&0,10,80,0,90,0,0,35'):
            self.assertFalse(self.c.send_cmd(command))
        self.assertFalse(self.c.send_cartesian_point(self.point()))
        QTest.qWait(40)
        self.assertFalse(any(command.startswith(('@','>','&','#CMDMODE ','!STREAM_OPEN')) for command in self.port.commands()))
        self.assertEqual(self.port.mode,7)
