"""Host paths use real Qt polling and serial workers against memory firmware."""
import time
import unittest
from unittest.mock import patch
from PySide6.QtTest import QTest
from app.core.controller import RobotController
from dummy_motion import Tool, Pose
from dummy_motion.frames import from_wire_joints, to_xyzabc
from tests.test_gamepad_integration import MemoryFirmwarePort
from tests.test_ui import APPLICATION


class PollingFirmwarePort(MemoryFirmwarePort):
    def __init__(self):
        super().__init__()
        self.joints = [0., 10., 80., 0., 90., 0., 0.]
        self.drop_joint_feedback = False
        self.delay_next_mode_reply = 0.

    def write(self, data):
        if self.delay_next_mode_reply and data.decode('ascii').startswith('#GET_CMDMODE '):
            delay, self.delay_next_mode_reply = self.delay_next_mode_reply, 0.
            time.sleep(delay)
        if self.drop_joint_feedback and data.decode('ascii').strip() == '#GETJPOS':
            with self.lock:
                self.writes.append('#GETJPOS')
            return len(data)
        return super().write(data)


class CartesianTransportTests(unittest.TestCase):
    def setUp(self):
        self.port = PollingFirmwarePort()
        self.c = RobotController(serial_factory=lambda *args, **kwargs: self.port)
        self.logs = []
        self.c.log_message.connect(lambda message, tag: self.logs.append((tag, message)))
        self.real_serial_guard = patch('app.core.controller.serial.Serial', side_effect=AssertionError('real serial forbidden'))
        self.real_serial_guard.start()

    def tearDown(self):
        self.c.close()
        APPLICATION.processEvents()
        self.real_serial_guard.stop()

    def wait_for(self, predicate, timeout=6.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            QTest.qWait(10)
            APPLICATION.processEvents()
        self.assertTrue(predicate(), (self.c.cartesian.stage, self.port.commands(), self.logs))

    def connect_and_start_path(self, hz, distance=.02, startup_delay=0., speed=100):
        self.c.set_cartesian_backend('host')
        self.c.set_polling(False, hz)
        self.assertTrue(self.c.connect_port('MEMORY_ONLY', 'USB'))
        self.wait_for(lambda: self.c._mode_request is None)
        self.assertTrue(self.c.send_cmd('!START'))
        self.wait_for(lambda: self.c._motor_enabled and self.c._mode_request is None)
        self.c.set_polling(True, hz)
        self.wait_for(lambda: self.c.cartesian.feedback.sample is not None)
        q = from_wire_joints(self.port.joints)
        flange = self.c.cartesian.model.fk(q, Tool(Pose.identity()), frame='base')
        target = Pose(flange.position+[0, distance, 0], flange.rotation)
        self.port.delay_next_mode_reply = startup_delay
        self.assertTrue(self.c.send_typed_position({'name': 'polling path', 'type': 'MoveL',
            'pose': to_xyzabc(target), 'speed': speed}), self.logs)

    def test_30hz_polling_continues_through_host_line_completion(self):
        self._complete_line(30)

    def test_50hz_polling_continues_through_host_line_completion(self):
        self._complete_line(50)

    def _complete_line(self, hz):
        self.connect_and_start_path(hz)
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        commands = self.port.commands()
        first = next(i for i, command in enumerate(commands) if command.startswith('>'))
        self.assertGreaterEqual(commands[first:].count('#GETJPOS'), 10)
        self.assertGreaterEqual(sum(command.startswith('>') for command in commands), 5)
        self.assertNotIn('!STOP', commands)
        self.assertFalse(any('处理中' in message or '300 ms' in message for _, message in self.logs))

    def test_400ms_startup_query_delay_recovers_before_sending_motion(self):
        self.connect_and_start_path(30, startup_delay=.4)
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        self.assertTrue(any('启动前等待新关节回读' in message for _, message in self.logs))
        self.assertNotIn('!STOP', self.port.commands())
        self.assertGreaterEqual(sum(command.startswith('>') for command in self.port.commands()), 5)

    def test_real_feedback_loss_still_stops_active_host_path(self):
        self.connect_and_start_path(30, .04)
        self.wait_for(lambda: sum(command.startswith('>') for command in self.port.commands()) >= 3)
        self.port.drop_joint_feedback = True
        self.wait_for(lambda: '!STOP' in self.port.commands())
        self.wait_for(lambda: self.c._stop_locked)
        self.assertFalse(self.c.cartesian.active)
        self.assertTrue(any('300 ms' in message for _, message in self.logs))

    def test_low_speed_line_completes_without_repeated_motion_packets(self):
        self.connect_and_start_path(30,distance=.002,speed=5)
        self.wait_for(lambda: self.c.cartesian.stage in ('complete','error'),timeout=10.)
        self.assertEqual(self.c.cartesian.stage,'complete',self.logs)
        moves=[cmd for cmd in self.port.commands() if cmd.startswith('>')]
        self.assertGreater(len(moves),2)
        self.assertTrue(all(a!=b for a,b in zip(moves,moves[1:])))


if __name__ == '__main__':
    unittest.main()
