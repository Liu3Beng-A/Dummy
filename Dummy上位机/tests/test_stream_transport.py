"""Real RX/TX worker integration against bounded in-memory stream firmware."""
from collections import deque
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
from PySide6.QtTest import QTest
from app.core.controller import RobotController
from app.core.business import TeachingPoint
from dummy_motion import Tool, Pose
from dummy_motion.frames import from_wire_joints, to_wire_joints, to_xyzabc
from dummy_motion.stream_protocol import decode_knot, robot_coordinates, motor_coordinates
from tests.test_ui import APPLICATION


class StreamMemoryPort:
    def __init__(self):
        self.lock = threading.RLock()
        self.rx, self.writes = bytearray(), []
        self.closed = False
        self.mode = 2
        self.busy = False
        self.session = 0
        self.next = 0
        self.state = 'IDLE'
        self.mask = 127
        self.buffer = deque()
        self.running_at = None
        self.next_tick = None
        self.joints = [0., 10., 80., 0., 90., 0., 0.]
        self.held = motor_coordinates(from_wire_joints(self.joints))[:, 0].copy()
        self.actual_offset = np.zeros(7)
        self.version = 2
        self.drop_feedback = self.old_firmware = self.wrong_done = False
        self.drop_ack = set()
        self.bogus_ack = False
        self.executed = 0
        self.end = False
        self.diag_nonce = 0
        self.motor_velocity = 30.
        self.motor_acceleration = 30.
        self.maximum_buffer = 0
        self.clock_thread = threading.Thread(target=self._clock, daemon=True)
        self.clock_thread.start()

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.rx)

    def read(self, n):
        with self.lock:
            data = bytes(self.rx[:n]); del self.rx[:n]
            return data

    def inject(self, *lines):
        with self.lock:
            self.rx.extend(('\n'.join(lines)+'\n').encode('ascii'))

    def commands(self):
        with self.lock:
            return list(self.writes)

    def snapshot(self):
        return f'STREAM_STATE s={self.session} st={self.state} next={self.next} free={16-len(self.buffer)}'

    def write(self, data):
        command = data.decode('ascii').strip()
        with self.lock:
            self.writes.append(command)
            if command == '#RGB_GET':
                self.inject('RGB_GET end')
            elif command == '!START':
                self.inject('Start pending', 'Started ok')
            elif command.startswith('#GET_CMDMODE '):
                self.inject(f'ok CMDMODE mode={self.mode} busy={int(self.busy)} issued=0 req={command.split()[1]}')
            elif command.startswith('#CMDMODE '):
                self.mode = int(command.split()[1]); self.inject(f'ok CMDMODE mode={self.mode}')
            elif command == '#GETJPOS':
                if not self.drop_feedback:
                    self.inject('ok '+' '.join(map(str, self.joints)))
            elif command.startswith('#GETJACC '):
                self.inject(f'ok ACC [{command.split()[1]}] = 30 r/s^2')
            elif command.startswith('#STREAM_DIAG '):
                node, field = map(int, command.split()[1:])
                self.diag_nonce += 1
                value = int(self.held[node-1]) if field == 7 else round((self.motor_velocity if field == 5 else self.motor_acceleration)*51200)
                self.inject(f'STREAM_NODE node={node} n={self.diag_nonce} field={field} value={value}')
            elif command == '#STREAM':
                if not self.old_firmware:
                    self.inject(f'STREAM_CAP v={self.version} hz=50 mode=7 mask={self.mask}', self.snapshot(),
                                f'STREAM_DIAG c={self.next} e={self.executed} gap=5 us=100')
            elif command.startswith('!STREAM_OPEN '):
                if self.session:
                    self.inject(f'error STREAM s={self.session} code=5')
                    return len(data)
                self.session = int(command.split()[1]); self.next = 0
                self.buffer.clear(); self.end = False; self.executed = 0
                self.running_at = self.next_tick = None
                self.state = 'OPEN'; self.inject(self.snapshot())
            elif command.startswith('~S'):
                session, knot = decode_knot(command)
                if session != self.session or knot.sequence != self.next or len(self.buffer) >= 16 or self.end:
                    self.inject(f'STREAM_FAULT s={self.session} code=BAD_SEQUENCE')
                else:
                    self.next += 1; self.buffer.append(knot)
                    self.maximum_buffer = max(self.maximum_buffer, len(self.buffer))
                    self.end = bool(knot.flags & 2)
                    if self.running_at is None:
                        self.state = 'ARMED' if self.next >= 6 or self.end else 'BUFFERING'
                    if knot.sequence not in self.drop_ack:
                        ack = knot.sequence+20 if self.bogus_ack else knot.sequence
                        self.inject(f'STREAM_ACK s={session} n={ack} free={16-len(self.buffer)}')
            elif command.startswith('!STREAM_START '):
                if self.state != 'ARMED':
                    self.inject(f'STREAM_FAULT s={self.session} code=BAD_STATE')
                else:
                    self.running_at = time.perf_counter()+.1
                    self.next_tick = self.running_at
            elif command.startswith('!STREAM_CLOSE '):
                self.session = 0; self.state = 'IDLE'; self.buffer.clear()
                self.running_at = self.next_tick = None
                self.inject(self.snapshot())
            elif command == '!STOP':
                self.buffer.clear(); self.running_at = self.next_tick = None
                self.state = 'FAULT'; self.inject('Stopping ok', 'STOP locked')
            else:
                raise AssertionError('unexpected command: '+command)
        return len(data)

    def _clock(self):
        while not self.closed:
            with self.lock:
                now = time.perf_counter()
                if self.next_tick is not None and now >= self.next_tick:
                    if not self.buffer:
                        self.inject(f'STREAM_FAULT s={self.session} code=UNDERRUN')
                        self.next_tick = None
                    else:
                        knot = self.buffer.popleft()
                        self.held = knot.motor[:, 0].copy()
                        self.joints = to_wire_joints(robot_coordinates(knot.motor)[0])
                        self.joints = list(np.asarray(self.joints) + self.actual_offset)
                        self.executed = knot.sequence
                        self.state = 'RUNNING'
                        self.next_tick += .02
                        if knot.flags & 2:
                            self.state = 'IDLE'; self.next_tick = None
                            self.inject(f'STREAM_DONE s={self.session} n={knot.sequence+int(self.wrong_done)}')
            time.sleep(.002)

    def cancel_read(self):
        pass

    def cancel_write(self):
        pass

    def close(self):
        self.closed = True
        if self.clock_thread is not threading.current_thread():
            self.clock_thread.join(timeout=.2)


class StreamTransportTests(unittest.TestCase):
    def setUp(self):
        self.port = StreamMemoryPort()
        self.c = RobotController(serial_factory=lambda *a, **kw: self.port)
        self.logs = []
        self.c.log_message.connect(lambda msg, tag: self.logs.append((tag, msg)))
        self.guard = patch('app.core.controller.serial.Serial', side_effect=AssertionError('real serial forbidden'))
        self.guard.start()

    def tearDown(self):
        self.c.close(); self.port.close(); APPLICATION.processEvents(); self.guard.stop()

    def wait_for(self, predicate, timeout=8.):
        deadline = time.perf_counter()+timeout
        while not predicate() and time.perf_counter() < deadline:
            QTest.qWait(5); APPLICATION.processEvents()
        self.assertTrue(predicate(), (self.c.cartesian.stage, self.logs[-12:], self.port.commands()[-15:]))

    def ready(self):
        self.assertTrue(self.c.set_cartesian_backend('stream'))
        self.c.set_polling(True, 50)
        self.assertTrue(self.c.connect_port('MEMORY_STREAM_ONLY', 'USB'))
        self.wait_for(lambda: self.c._mode_request is None)
        self.assertTrue(self.c.send_cmd('!START'))
        self.wait_for(lambda: self.c._motor_enabled and self.c._mode_request is None and self.c.cartesian.feedback.sample is not None)

    def point(self, delta=.004):
        q = from_wire_joints(self.port.joints)
        pose = self.c.cartesian.model.fk(q, Tool(Pose.identity()), frame='base')
        return TeachingPoint('line', 'MOVEL', tuple(to_xyzabc(Pose(pose.position+[0, delta, 0], pose.rotation))), 100., 100.)

    def start(self, delta=.004):
        self.ready()
        self.assertTrue(self.c.send_cartesian_point(self.point(delta)))

    def test_complete_50hz_finite_movel_without_any_legacy_movej(self):
        self.start()
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        commands = self.port.commands()
        self.assertTrue(any(cmd.startswith('~S') for cmd in commands))
        self.assertFalse(any(cmd.startswith(('>', '@')) for cmd in commands))
        self.assertLessEqual(self.port.maximum_buffer, 16)
        self.assertNotIn('!STOP', commands)
        self.assertEqual(self.port.mode, 7)

    def test_worker_keeps_credits_and_feedback_during_600ms_gui_stall(self):
        self.start(.02)
        self.wait_for(lambda: self.c.cartesian.stage == 'running')
        before = self.port.executed
        time.sleep(.6)  # Intentionally no Qt event processing.
        self.assertGreater(self.port.executed, before+10)
        self.assertNotIn('!STOP', self.port.commands())
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)

    def test_feedback_loss_stops_from_worker_even_without_qt_events(self):
        self.start(.03)
        self.wait_for(lambda: self.c.cartesian.stage == 'running')
        self.port.drop_feedback = True
        time.sleep(.42)
        self.assertIn('!STOP', self.port.commands())
        self.wait_for(lambda: not self.c.cartesian.active)
        self.assertFalse(any(c.startswith('~S') for c in self.port.commands()[self.port.commands().index('!STOP')+1:]))

    def test_mixed_motor_capabilities_never_start(self):
        self.port.mask = 126
        self.start()
        self.wait_for(lambda: '!STOP' in self.port.commands() and any('七轴固件能力不一致' in line for tag,line in self.logs))
        self.wait_for(lambda: not self.c.cartesian.active)
        self.assertFalse(any(c.startswith('!STREAM_START') for c in self.port.commands()))
        self.assertFalse(any(c.startswith(('>', '@', '~S')) for c in self.port.commands()))

    def test_missing_ack_recovers_by_cumulative_state_without_resending(self):
        self.port.drop_ack = {2, 7, 9}
        self.start(.007)
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        seqs = [decode_knot(c)[1].sequence for c in self.port.commands() if c.startswith('~S')]
        self.assertEqual(seqs, list(range(len(seqs))))

    def test_foreign_future_ack_aborts_before_start(self):
        self.port.bogus_ack = True
        self.start()
        self.wait_for(lambda: '!STOP' in self.port.commands() and any('未发送的透传序号' in line for tag,line in self.logs))
        self.wait_for(lambda: not self.c.cartesian.active)

    def test_wrong_done_sequence_fails_and_stops(self):
        self.port.wrong_done = True
        self.start()
        self.wait_for(lambda: '!STOP' in self.port.commands() and any('完成序号不匹配' in line for tag,line in self.logs))
        self.wait_for(lambda: not self.c.cartesian.active)
        self.assertNotEqual(self.c.cartesian.stage, 'complete')

    def test_explicit_stop_invalidates_all_queued_curve_packets(self):
        self.start(.03)
        self.wait_for(lambda: self.c.cartesian.stage == 'running')
        self.assertTrue(self.c.send_cmd('!STOP'))
        self.wait_for(lambda: '!STOP' in self.port.commands())
        QTest.qWait(100)
        commands = self.port.commands()
        self.assertFalse(any(c.startswith('~S') for c in commands[commands.index('!STOP')+1:]))
        self.assertFalse(self.c.cartesian.active)

    def test_switch_back_to_default_backend_restores_native_mode(self):
        self.start()
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        self.assertTrue(self.c.set_cartesian_backend('firmware'))
        self.wait_for(lambda: self.c.cartesian.backend.value == 'firmware')
        self.assertEqual(self.port.mode, 2)

    def test_ordinary_movej_is_planned_as_timed_stream_when_selected(self):
        self.ready()
        self.assertTrue(self.c.send_cmd('>0.1,10,80,0,90,0,0,35'))
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        self.assertFalse(any(c.startswith(('>', '@')) for c in self.port.commands()))

    def test_mixed_teaching_points_are_two_closed_finite_sessions(self):
        self.ready()
        joint = TeachingPoint('joint', 'MOVEJ', (0.1, 10, 80, 0, 90, 0, 0), 100., 35.)
        self.assertTrue(self.c.cartesian.start_points((joint, self.point(.004)), sequence=True))
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'), 12.)
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        self.assertEqual(sum(cmd.startswith('!STREAM_OPEN') for cmd in self.port.commands()), 2)
        self.assertEqual(sum(cmd.startswith('!STREAM_CLOSE') for cmd in self.port.commands()), 2)
        self.assertFalse(self.c._seq_active)

    def test_cancel_remaining_preserves_current_point_and_drops_tail(self):
        self.ready()
        self.assertTrue(self.c.cartesian.start_points((self.point(.008), self.point(.014)), sequence=True))
        self.wait_for(lambda: self.c.cartesian.stage == 'running')
        self.c.cartesian.cancel_remaining()
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        self.assertEqual(sum(cmd.startswith('!STREAM_OPEN') for cmd in self.port.commands()), 1)
        self.assertNotIn('!STOP', self.port.commands())

    def test_old_firmware_has_no_motion_fallback(self):
        self.port.old_firmware = True
        self.start()
        self.wait_for(lambda: self.c.cartesian.stage == 'error')
        self.assertFalse(any(cmd.startswith(('~S', '>', '@', '!STREAM_OPEN')) for cmd in self.port.commands()))
        self.assertEqual(self.port.mode, 2)

    def test_uart_rejected_before_start_and_usb_remains_required(self):
        self.ready()
        self.c.connection_type = 'UART4'
        self.assertFalse(self.c.send_cartesian_point(self.point()))
        self.assertFalse(self.c.supports('!STREAM_OPEN 1 127'))
        self.assertFalse(any(cmd.startswith('!STREAM_OPEN') for cmd in self.port.commands()))

    def test_all_seven_actual_motor_caps_are_read_before_any_motion(self):
        self.start()
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        commands = self.port.commands()
        before = commands[:next(i for i, cmd in enumerate(commands) if cmd.startswith('!STREAM_OPEN'))]
        for node in range(1, 8):
            for field in (5, 6):
                self.assertIn(f'#STREAM_DIAG {node} {field}', before)

    def test_invalid_motor_cap_fails_before_mode_change_or_open(self):
        self.port.motor_velocity = 0
        self.start()
        self.wait_for(lambda: self.c.cartesian.stage == 'error')
        self.assertFalse(any(cmd.startswith(('!STREAM_OPEN', '~S', '#CMDMODE 7')) for cmd in self.port.commands()))
        self.assertEqual(self.port.mode, 2)

    def test_disabling_required_feedback_stops_worker_without_gui_dependency(self):
        self.start(.03)
        self.wait_for(lambda: self.c.cartesian.stage == 'running')
        self.c.set_polling(False, 50)
        time.sleep(.1)
        self.assertIn('!STOP', self.port.commands())

    def test_two_separate_completed_requests_accept_idle_retained_next_sequence(self):
        self.start()
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        self.assertGreater(self.port.next, 0)
        self.assertEqual(self.port.session, 0)
        self.wait_for(lambda: not self.c.cartesian.stream.thread.is_alive())
        self.assertTrue(self.c.send_cartesian_point(self.point(.004)))
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        self.assertEqual(sum(cmd.startswith('!STREAM_OPEN') for cmd in self.port.commands()), 2)

    def test_stop_start_then_new_motion_closes_retained_fault_session(self):
        self.start(.02)
        self.wait_for(lambda: self.c.cartesian.stage == 'running')
        self.assertTrue(self.c.send_cmd('!STOP'))
        self.wait_for(lambda: self.c._stop_locked and '!STOP' in self.port.commands())
        old_session = self.port.session
        self.assertNotEqual(old_session, 0)
        self.assertTrue(self.c.send_cmd('!START'))
        self.wait_for(lambda: not self.c._stop_locked and self.c._mode_request is None and self.c._motor_enabled)
        self.wait_for(lambda: not self.c.cartesian.stream.thread.is_alive())
        self.wait_for(lambda: time.perf_counter()-self.c.cartesian.feedback.sample.received_at < .2)
        self.assertTrue(self.c.send_cartesian_point(self.point(.004)))
        self.wait_for(lambda: self.c.cartesian.stage in ('complete', 'error'))
        self.assertEqual(self.c.cartesian.stage, 'complete', self.logs)
        commands = self.port.commands()
        self.assertIn(f'!STREAM_CLOSE {old_session}', commands)
        self.assertEqual(sum(cmd.startswith('!STREAM_OPEN') for cmd in commands), 2)

    def test_stop_start_then_native_switch_closes_fault_session(self):
        self.start(.02)
        self.wait_for(lambda: self.c.cartesian.stage == 'running')
        self.assertTrue(self.c.send_cmd('!STOP'))
        self.wait_for(lambda: '!STOP' in self.port.commands())
        old_session = self.port.session
        self.assertTrue(self.c.send_cmd('!START'))
        self.wait_for(lambda: not self.c._stop_locked and self.c._mode_request is None)
        self.wait_for(lambda: not self.c.cartesian.stream.thread.is_alive())
        self.assertTrue(self.c.set_cartesian_backend('firmware'))
        self.wait_for(lambda: self.c.cartesian.backend.value == 'firmware' or self.c.cartesian.stage == 'error')
        self.assertEqual(self.c.cartesian.backend.value, 'firmware', self.logs)
        self.assertEqual(self.port.mode, 2)
        self.assertEqual(self.port.session, 0)
        self.assertIn(f'!STREAM_CLOSE {old_session}', self.port.commands())

    def test_usb_only_gamepad_rejected_and_default_is_native(self):
        self.assertEqual(self.c.cartesian.backend.value, 'firmware')
        self.c.set_cartesian_backend('stream')
        self.assertFalse(self.c.gamepad.claim())
        self.assertFalse(self.c.supports('!STREAM_OPEN 1 127') and self.c.connection_type == 'UART4')


if __name__ == '__main__':
    unittest.main()
