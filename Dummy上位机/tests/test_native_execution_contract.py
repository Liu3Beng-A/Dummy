"""Native-only routing through the real serial writer, without hardware."""
import sys
import subprocess
import time
import unittest
from unittest.mock import patch

import numpy as np
from PySide6.QtCore import QCoreApplication
from app.core.controller import RobotController
from dummy_motion import IKSolver, StreamPlanner, Tool
from dummy_motion.frames import from_xyzabc, to_xyzabc
from tests.test_controller import FakePort


class NativeMemoryPort(FakePort):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.mode = 2
        self.issued = 10

    def write(self, data):
        count = super().write(data)
        command = data.decode('ascii').strip()
        if command.startswith('#GET_CMDMODE '):
            reply = f'ok CMDMODE mode={self.mode} busy=0 issued={self.issued} req={command.split()[1]}'
        elif command == '#RGB_GET':
            reply = 'RGB_GET end'
        elif command == '#GETJPOS':
            reply = 'ok 0 10 80 0 90 0 37.25'
        elif command.startswith(('@', '>', '&')):
            self.issued += 1
            kind = 'MOVEL' if command.startswith('@') else 'MOVEJ'
            reply = f'queued {kind} id={self.issued} mode=INT\naccepted {kind} id={self.issued}'
        elif command == '!STOP':
            reply = 'Stopping ok\nSTOP locked'
        else:
            reply = ''
        if reply:
            self.inject((reply + '\n').encode('ascii'))
        return count

    def commands(self):
        with self.lock:
            return [item.decode('ascii').strip() for item in self.written]


class NativeExecutionContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.port = NativeMemoryPort()
        self.c = RobotController(serial_factory=lambda *args, **kwargs: self.port)
        self.assertTrue(self.c.connect_port('MEMORY_NATIVE', 'USB'))
        self.wait(lambda: self.c.current_cmdmode == 2 and self.c._mode_request is None)
        self.c._poll_timer.stop()
        self.c._poll_enabled = False
        self.c._motor_state_known = self.c._motor_enabled = True

    def tearDown(self):
        self.c.close()

    def wait(self, predicate, timeout=1.5):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            self.app.processEvents()
            if predicate():
                return
            time.sleep(.002)
        self.fail('native writer condition timed out')

    def settle(self, seconds=.08):
        deadline = time.perf_counter() + seconds
        while time.perf_counter() < deadline:
            self.app.processEvents()
            time.sleep(.002)

    def test_typed_movel_and_movej_never_call_pc_planners(self):
        with patch.object(IKSolver, 'solve_pose', side_effect=AssertionError('live IK')), \
             patch.object(StreamPlanner, 'plan_line', side_effect=AssertionError('live line planner')), \
             patch.object(StreamPlanner, 'plan_joint', side_effect=AssertionError('live joint planner')):
            self.assertTrue(self.c.send_typed_position(dict(name='L', type='MoveL',
                pose=[220, 0, 180, 180, 0, 180], speed=35, gripper=100)))
            self.wait(lambda: '@220,0,180,180,0,180,35' in self.port.commands())
            self.assertTrue(self.c.send_typed_position(dict(name='J', type='MoveJ',
                joints=[0, 10, 80, 0, 90, 0, 0], speed=27, gripper=100)))
            self.wait(lambda: '>0,10,80,0,90,0,0,27' in self.port.commands())
        self.assertFalse(any(command.startswith(('~S', '!STREAM_')) for command in self.port.commands()))

    def test_world_tcp_tool_snapshot_keeps_native_flange_semantics(self):
        flange = [220, 20, 180, 20, 30, 40]
        tool_values = [10, 20, 114, 0, 0, 0]
        tool = Tool(from_xyzabc(tool_values))
        target = self.c.cartesian.model.legacy_target(from_xyzabc(flange), .03725, tool)
        self.assertTrue(self.c.cartesian.update_feedback([0,10,80,0,90,0,37.25], time.perf_counter()))
        record = dict(name='TCP', type='MoveL', pose=to_xyzabc(target), speed=35,
                      gripper=100, frame='world', target_kind='tcp', tool=tool_values, tool_version=1)
        with patch.object(IKSolver, 'solve_pose', side_effect=AssertionError('live IK')):
            self.assertTrue(self.c.send_typed_position(record))
        self.wait(lambda: any(command.startswith('@') for command in self.port.commands()))
        command = next(command for command in self.port.commands() if command.startswith('@'))
        actual = list(map(float, command[1:].split(',')))
        np.testing.assert_allclose(actual[:6], flange, rtol=0, atol=1e-3)
        self.assertEqual(actual[6], 35)
        self.assertEqual(record['tool'], tool_values)

    def test_retired_backends_are_rejected_without_any_control_tx(self):
        before = self.port.commands()
        for backend in ('host', 'stream', 'unknown'):
            self.assertFalse(self.c.set_cartesian_backend(backend))
        self.settle()
        self.assertEqual(self.port.commands(), before)
        self.assertEqual(self.c.state()['cartesian_backend'], 'firmware')

    def test_mode7_rejects_native_motion_at_dispatch_and_final_writer(self):
        self.c.current_cmdmode = 7
        before = self.port.commands()
        for command in ('@220,0,180,180,0,180,35', '>0,10,80,0,90,0,0,35'):
            self.assertFalse(self.c.send_cmd(command))
            self.c._tx_queue.put((self.c._tx_generation, command))
        self.settle()
        self.assertEqual(self.port.commands(), before)

    def test_unowned_stream_and_mode7_packets_never_reach_writer(self):
        before = self.port.commands()
        for command in ('~S forged', '!STREAM_OPEN 1 127', '!STREAM_TEST 1 4', '#CMDMODE 7'):
            self.assertFalse(self.c.send_cmd(command, source='cartesian'))
            self.c._tx_queue.put((self.c._tx_generation, command))
        self.settle()
        self.assertEqual(self.port.commands(), before)

    def test_mode7_blocks_home_reset_for_every_source_and_late_queue(self):
        self.c.current_cmdmode = 7
        previews = []
        self.c.preset_pose_requested.connect(lambda *args: previews.append(args))
        before = self.port.commands()
        for source in ('ui', 'cartesian', 'telemetry', 'gamepad'):
            for command in ('!HOME', '!RESET'):
                self.assertFalse(self.c.send_cmd(command, source=source))
                self.c._tx_queue.put((self.c._tx_generation, command))
        self.settle()
        self.assertEqual(previews, [])
        self.assertEqual(self.port.commands(), before)

    def test_diagnostic_owner_blocks_home_reset_despite_source_label(self):
        self.c.cartesian._begin('open_test')
        previews = []
        self.c.preset_pose_requested.connect(lambda *args: previews.append(args))
        before = self.port.commands()
        for command in ('!HOME', '!RESET'):
            self.assertFalse(self.c.send_cmd(command, source='cartesian'))
            self.c._tx_queue.put((self.c._tx_generation, command))
        self.settle()
        self.assertEqual(previews, [])
        self.assertEqual(self.port.commands(), before)

    def test_noncanonical_mode_variants_are_blocked_at_both_gates(self):
        before = self.port.commands()
        for command in ('#CMDMODE 07', '#CMDMODE +7', '#CMDMODE7',
                        '#CMDMODE\t7', '#CMDMODE 0007', '#CMDMODE 7 extra'):
            self.assertFalse(self.c.send_cmd(command, source='cartesian'), command)
            self.c._tx_queue.put((self.c._tx_generation, command))
        self.settle()
        self.assertEqual(self.port.commands(), before)

    def test_mode_migration_does_not_reinterpret_actual_device_mode(self):
        self.c.current_cmdmode = 7
        before = self.port.commands()
        self.assertFalse(self.c.set_cartesian_backend('stream'))
        self.assertFalse(self.c.send_cmd('@220,0,180,180,0,180,35'))
        self.settle()
        self.assertEqual(self.c.current_cmdmode, 7)
        self.assertEqual(self.c.state()['cartesian_backend'], 'firmware')
        self.assertEqual(self.port.commands(), before)

    def test_stop_invalidates_old_native_writer_generation(self):
        old = self.c._tx_generation
        self.assertTrue(self.c.send_cmd('!STOP'))
        self.wait(lambda: self.c._stop_confirmed)
        self.c._tx_queue.put((old, '@220,0,180,180,0,180,35'))
        self.settle()
        self.assertEqual([command for command in self.port.commands() if command.startswith('@')], [])
        self.assertTrue(self.c._stop_locked)

    def test_no_point_execution_api_or_live_planning_process(self):
        self.assertFalse(hasattr(self.c.cartesian, 'start_points'))
        diagnostic = self.c.cartesian.diagnostics
        for name in ('start_points', '_plan_point', '_run', '_execute'):
            self.assertFalse(hasattr(diagnostic, name), name)
        # Offline process tests may import this module during test discovery.
        # Check the production import graph in a clean interpreter instead.
        probe = subprocess.run([sys.executable, '-B', '-c',
            "import sys; from PySide6.QtCore import QCoreApplication; "
            "app=QCoreApplication([]); from app.core.controller import RobotController; "
            "c=RobotController(); assert 'app.core.planning_process' not in sys.modules; c.close()"],
            capture_output=True, timeout=30)
        self.assertEqual(probe.returncode, 0, probe.stderr.decode('utf-8', errors='replace'))


if __name__ == '__main__':
    unittest.main()
