"""Real writer contract: ordinary motion belongs exclusively to firmware."""
import ast
from pathlib import Path
import time
import threading
import unittest
from unittest.mock import patch
from PySide6.QtTest import QTest
from app.core.controller import RobotController
from app.core.business import TeachingPoint
from app.core.cartesian_backend import CartesianBackend
from app.core.diagnostic_session import DiagnosticSession
from dummy_motion import Tool, Pose, IKSolver, StreamPlanner
from dummy_motion.frames import from_wire_joints, to_xyzabc
from tests.stream_memory_port import StreamMemoryPort
from tests.test_ui import APPLICATION


class NativeTransportFixture:
    def setUp(self):
        self.port = StreamMemoryPort()
        self.c = RobotController(serial_factory=lambda *a, **kw: self.port)
        self.logs = []
        self.c.log_message.connect(lambda msg, tag: self.logs.append((tag, msg)))
        self.guards = [patch('app.core.controller.serial.Serial', side_effect=AssertionError('real serial forbidden')),
                       patch('dummy_motion.ik.IKSolver.solve_pose', side_effect=AssertionError('live IK forbidden')),
                       patch('dummy_motion.stream_curve.StreamPlanner.plan_line', side_effect=AssertionError('live planner forbidden')),
                       patch('dummy_motion.stream_curve.StreamPlanner.plan_joint', side_effect=AssertionError('live planner forbidden'))]
        for guard in self.guards:
            guard.start()

    def tearDown(self):
        self.c.close(); self.port.close(); APPLICATION.processEvents()
        for guard in reversed(self.guards):
            guard.stop()

    def wait_for(self, predicate, timeout=8.):
        deadline = time.perf_counter()+timeout
        while not predicate() and time.perf_counter()<deadline:
            QTest.qWait(5); APPLICATION.processEvents()
        self.assertTrue(predicate(), (self.c.cartesian.stage,self.logs[-12:],self.port.commands()[-15:]))

    def ready(self):
        self.assertEqual(self.c.cartesian.backend.value,'firmware')
        self.c.set_polling(True,50)
        self.assertTrue(self.c.connect_port('MEMORY_NATIVE_ONLY','USB'))
        self.wait_for(lambda:self.c._mode_request is None)
        self.assertTrue(self.c.send_cmd('!START'))  # Explicit simulated user enable.
        self.wait_for(lambda:self.c._motor_enabled and self.c._mode_request is None and
                      self.c.cartesian.feedback.sample is not None)

    def point(self, delta=.004):
        q=from_wire_joints(self.port.joints)
        pose=self.c.cartesian.model.fk(q,Tool(Pose.identity()),frame='base')
        target=Pose(pose.position+[0,delta,0],pose.rotation)
        return TeachingPoint('native line','MOVEL',tuple(to_xyzabc(target)),100.,35.)

    def assert_no_planned_motion(self, commands):
        self.assertFalse(any(command.startswith(('~S','!STREAM_OPEN ','!STREAM_START ')) for command in commands),commands)


class StreamTransportTests(NativeTransportFixture, unittest.TestCase):
    def test_native_movel_reaches_actual_writer_without_pc_ik(self):
        self.ready();marker=len(self.port.commands())
        self.assertTrue(self.c.send_cartesian_point(self.point()))
        self.wait_for(lambda:any(command.startswith('@') for command in self.port.commands()[marker:]))
        commands=self.port.commands()[marker:]
        self.assertEqual(sum(command.startswith('@') for command in commands),1)
        self.assertTrue(next(command for command in commands if command.startswith('@')).endswith(',35'))
        self.assert_no_planned_motion(commands)
        self.assertFalse(self.c.cartesian.busy)

    def test_native_joint_targets_are_not_intercepted_as_curves(self):
        self.ready();marker=len(self.port.commands())
        targets=('>0,10,80,0,90,0,0,35','&0,10,80,0,90,0,0,35')
        for command in targets:
            self.assertTrue(self.c.send_cmd(command))
        self.wait_for(lambda:all(command in self.port.commands()[marker:] for command in targets))
        self.assert_no_planned_motion(self.port.commands()[marker:])

    def test_legacy_execution_selection_never_changes_mode_or_writes(self):
        self.ready();marker=len(self.port.commands())
        for value in ('host','stream',CartesianBackend.STREAM):
            self.assertFalse(self.c.set_cartesian_backend(value))
        QTest.qWait(35)
        commands=self.port.commands()[marker:]
        self.assert_no_planned_motion(commands)
        self.assertFalse(any(command.startswith('#CMDMODE ') for command in commands))
        self.assertEqual(self.port.mode,2)

    def test_ordinary_ui_and_terminal_cannot_select_diagnostic_mode7(self):
        self.ready();marker=len(self.port.commands())
        self.assertFalse(self.c._set_cmdmode(7))
        self.assertFalse(self.c.send_cmd('#CMDMODE 7'))
        QTest.qWait(30)
        self.assertNotIn('#CMDMODE 7',self.port.commands()[marker:])
        self.assertEqual(self.port.mode,2)

    def test_old_capability_does_not_disable_native_movel(self):
        self.port.version=1;self.port.old_firmware=True
        self.ready();marker=len(self.port.commands())
        self.assertTrue(self.c.send_cartesian_point(self.point()))
        self.wait_for(lambda:any(command.startswith('@') for command in self.port.commands()[marker:]))
        self.assertNotIn('#STREAM',self.port.commands()[marker:])
        self.assert_no_planned_motion(self.port.commands()[marker:])

    def test_world_tcp_uses_current_rail_and_tool_conversion_without_ik(self):
        self.ready()
        q=self.c.cartesian.feedback.sample.q
        tool=Tool(Pose([0,0,.04],Pose.identity().rotation))
        native=self.c.cartesian.model.fk(q,frame='base')
        world=self.c.cartesian.model.legacy_target(native,q[6],tool)
        point=TeachingPoint('world tool','MOVEL',tuple(to_xyzabc(world)),100.,35.,
                            'world','tcp',(0.,0.,40.,0.,0.,0.))
        marker=len(self.port.commands())
        self.assertTrue(self.c.send_cartesian_point(point))
        self.wait_for(lambda:any(command.startswith('@') for command in self.port.commands()[marker:]))
        command=next(command for command in self.port.commands()[marker:] if command.startswith('@'))
        values=[float(value) for value in command[1:].split(',')]
        expected=to_xyzabc(native)
        for actual,wanted in zip(values[:6],expected):
            self.assertEqual(actual,round(wanted,1))  # Native wire XYZABC precision is 0.1.
        self.assert_no_planned_motion(self.port.commands()[marker:])

    def test_diagnostics_remain_available_but_have_no_point_api(self):
        facade=self.c.cartesian
        self.assertIs(facade.stream,facade.diagnostics)
        self.assertIsInstance(facade.diagnostics,DiagnosticSession)
        for name in ('start_points','_held_reference','_plan_point','_run','_execute'):
            self.assertFalse(hasattr(facade.diagnostics,name),name)
        self.assertFalse(hasattr(facade,'start_points'))
        self.assertTrue(callable(IKSolver.solve_pose))
        self.assertTrue(callable(StreamPlanner.plan_line))

    def test_mode_owner_cannot_authorize_motion_even_through_private_send(self):
        self.ready()
        owner=self.c.cartesian;worker=owner.diagnostics
        owner._begin('mode_switch');worker.target_mode=2;worker._prepare_worker(False)
        for command in ('!START','>0,10,80,0,90,0,0,35','@200,0,200,180,0,180,35',
                        '!STREAM_OPEN 1 127','!STREAM_START 1','#CMDMODE 7'):
            with self.subTest(command=command):
                with self.assertRaises(ValueError):worker._send(command)
        self.assertEqual(worker.authorized,{})

    def test_actual_rx_fault_discards_admitted_native_mode_tail_before_writer(self):
        self.ready()
        owner=self.c.cartesian;worker=owner.diagnostics
        owner._begin('mode_switch');worker.target_mode=3;worker._prepare_worker(False)
        worker.session=919
        reached,release=threading.Event(),threading.Event()
        allowed=self.c._transport.allowed
        def writer_gate(generation,command):
            if command=='#CMDMODE 3':
                reached.set();release.wait(3.)
            return allowed(generation,command)
        marker=len(self.port.commands())
        try:
            with patch.object(self.c._transport,'allowed',side_effect=writer_gate):
                worker._send('#CMDMODE 3')
                self.wait_for(reached.is_set)
                worker.receive('STREAM_FAULT s=919 code=9',worker.connection,time.perf_counter())
                self.assertTrue(worker.recovering)
                release.set();QTest.qWait(40)
            self.assertNotIn('#CMDMODE 3',self.port.commands()[marker:])
            self.assertEqual(self.port.mode,2)
            self.assertFalse(worker._locked_command('#CMDMODE 3'))
        finally:
            release.set();owner.cancel()

    def test_production_modules_have_no_point_planner_import_or_api(self):
        core=Path(__file__).parents[1]/'app/core'
        for filename in ('cartesian_session.py','diagnostic_session.py','stream_session.py'):
            tree=ast.parse((core/filename).read_text(encoding='utf-8'))
            methods={node.name for node in ast.walk(tree) if isinstance(node,ast.FunctionDef)}
            self.assertFalse(methods & {'start_points','_plan_point','_run','_execute'})
            for node in ast.walk(tree):
                if isinstance(node,ast.ImportFrom):
                    self.assertNotIn('planning_process',node.module or '')


if __name__ == '__main__':
    unittest.main()
