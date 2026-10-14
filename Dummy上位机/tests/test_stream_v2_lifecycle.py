"""v2 held-reference seed, spawn isolation, cancellation and recovery regressions."""
import os
import threading
from unittest.mock import patch
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
from tests.test_stream_transport import NativeTransportFixture


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


class StreamV2LifecycleTests(NativeTransportFixture, unittest.TestCase):
    def residual(self, state='FAULT'):
        self.port.mode=7;self.port.session=4123;self.port.state=state
        self.ready()
        self.assertEqual(self.c.current_cmdmode,7)

    def test_reconnect_mode7_does_not_auto_takeover_or_send_motion(self):
        self.residual()
        self.assertEqual(self.c.cartesian.backend.value,'firmware')
        for command in ('@146.5,0,288.9,180,0,180,35','>0,10,80,0,90,0,0,35','&0,10,80,0,90,0,0,35'):
            self.assertFalse(self.c.send_cmd(command))
        self.assertFalse(self.c.send_cartesian_point(self.point()))
        QTest.qWait(35)
        self.assertFalse(any(command.startswith(('@','>','&','#CMDMODE ','!STREAM_CLOSE ')) for command in self.port.commands()))
        self.assertEqual(self.port.mode,7)

    def test_explicit_native_recovery_closes_before_mode_set_without_enable(self):
        self.residual();marker=len(self.port.commands())
        self.assertTrue(self.c.cartesian.select_mode(2))
        self.wait_for(lambda:not self.c.cartesian.busy)
        commands=self.port.commands()[marker:]
        self.assertLess(commands.index('!STREAM_CLOSE 4123'),commands.index('#CMDMODE 2'))
        self.assertNotIn('!START',commands)
        self.assert_no_planned_motion(commands)
        self.assertEqual(self.port.mode,2)
        self.assertEqual(self.c.current_cmdmode,2)

    def test_stop_locked_recovery_keeps_lock_and_never_enables(self):
        self.residual();self.c._stop_locked=True;self.c._stop_confirmed=True
        marker=len(self.port.commands())
        self.assertFalse(self.c.cartesian.select_mode(3))
        self.assertTrue(self.c.cartesian.select_mode(2))
        self.wait_for(lambda:not self.c.cartesian.busy)
        commands=self.port.commands()[marker:]
        self.assertIn('!STREAM_CLOSE 4123',commands)
        self.assertIn('#CMDMODE 2',commands)
        self.assertNotIn('!START',commands)
        self.assertTrue(self.c._stop_locked)

    def test_unconfirmed_stop_blocks_recovery_before_query_or_close(self):
        self.residual();self.c._stop_locked=True;self.c._stop_confirmed=False
        marker=len(self.port.commands())
        self.assertFalse(self.c.cartesian.select_mode(2))
        self.assertEqual(self.port.commands()[marker:],[])

    def test_foreign_running_session_is_not_closed_or_taken_over(self):
        self.residual('RUNNING');marker=len(self.port.commands())
        self.assertTrue(self.c.cartesian.select_mode(2))
        self.wait_for(lambda:not self.c.cartesian.busy)
        self.assertEqual(self.c.cartesian.stage,'error')
        commands=self.port.commands()[marker:]
        self.assertFalse(any(command.startswith(('!STREAM_CLOSE ','#CMDMODE ')) for command in commands))
        self.assertEqual(self.port.mode,7)

    def test_close_without_confirmation_cannot_switch_or_accept_native_target(self):
        self.residual();original=self.port.write
        def missing_close(data):
            command=data.decode('ascii').strip()
            if command.startswith('!STREAM_CLOSE '):
                with self.port.lock:self.port.writes.append(command)
                return len(data)
            return original(data)
        marker=len(self.port.commands())
        with patch.object(self.port,'write',side_effect=missing_close):
            self.assertTrue(self.c.cartesian.select_mode(2))
            self.wait_for(lambda:not self.c.cartesian.busy)
        self.assertEqual(self.c.cartesian.stage,'error')
        self.assertNotIn('#CMDMODE 2',self.port.commands()[marker:])
        self.assertEqual(self.port.mode,7)
        self.assertFalse(self.c.send_cartesian_point(self.point()))

    def test_busy_mode_recovery_cannot_close_or_change_mode(self):
        self.residual();self.port.busy=True;marker=len(self.port.commands())
        self.assertTrue(self.c.cartesian.select_mode(2))
        self.wait_for(lambda:not self.c.cartesian.busy)
        self.assertEqual(self.c.cartesian.stage,'error')
        self.assertFalse(any(command.startswith(('!STREAM_CLOSE ','#CMDMODE ')) for command in self.port.commands()[marker:]))

    def test_stop_during_close_recovery_discards_queued_mode_tail(self):
        self.residual();worker=self.c.cartesian.diagnostics
        reached,release=threading.Event(),threading.Event();original=worker._cleanup_existing_session
        def cleanup(check_feedback):
            original(check_feedback);reached.set();release.wait(3.)
        marker=len(self.port.commands())
        with patch.object(worker,'_cleanup_existing_session',side_effect=cleanup):
            self.assertTrue(self.c.cartesian.select_mode(3))
            self.wait_for(reached.is_set)
            self.assertTrue(self.c.send_cmd('!STOP'))
            release.set();self.wait_for(lambda:not worker.thread.is_alive())
        QTest.qWait(30)
        commands=self.port.commands()[marker:]
        self.assertNotIn('#CMDMODE 3',commands)
        self.assertIn('!STOP',commands)
        self.assertTrue(self.c._stop_locked)
