"""Real host RX/TX threads with bounded in-memory selected-axis firmware."""
import threading
import time
import unittest
from unittest.mock import patch
import numpy as np
from PySide6.QtTest import QTest

from app.core.controller import RobotController
from app.core.axis_communication_test import held_test_knots, format_axis_test
from dummy_motion.stream_protocol import decode_knot, parse_stream_line, robot_coordinates
from dummy_motion.frames import to_wire_joints
from tests.test_stream_transport import StreamMemoryPort
from tests.test_ui import APPLICATION


class AxisMemoryPort(StreamMemoryPort):
    def __init__(self):
        self.axis_capability = True
        self.axis_cap_mask = 127
        self.open_refused_node = None
        self.close_missing = False
        self.drop_selected_diagnostic = False
        self.drop_diagnostic_node = None
        self.diagnostic_delay = 0.
        self.pending_diagnostic = None
        self.health_busy_once = False
        self.bad_held_after_open = False
        self.duplicate_stop_locked = False
        self.open_snapshots = {}
        super().__init__()

    def write(self, data):
        command = data.decode('ascii').strip()
        if command == '!DISABLE':
            with self.lock:
                self.writes.append(command)
                self.buffer.clear()
                self.running_at = self.next_tick = None
                self.state = 'FAULT'
                self.inject('Disabled ok')
            return len(data)
        if command == '#STREAM_AXES':
            with self.lock:
                self.writes.append(command)
                if self.axis_capability:
                    self.inject(f'STREAM_AXES v=1 hz=50 mask={self.axis_cap_mask} targeted=1')
                else:
                    self.inject('error STREAM_AXES unsupported')
            return len(data)
        if command.startswith('!STREAM_OPEN '):
            with self.lock:
                token, self.mask = map(int, command.split()[1:])
                for node in range(1, 8):
                    if self.mask & (1 << (node-1)):
                        held = int(self.held[node-1])
                        bits = 1 << 12 if node == self.open_refused_node else 0
                        self.open_snapshots[node] = {23: token, 24: bits, 25: 257, 26: 625 if bits else 0,
                                                     27: 0, 28: held, 29: held, 30: held, 31: 0, 32: 10000, 33: 1}
                if self.open_refused_node is not None:
                    self.writes.append(command)
                    self.session = token
                    self.state = 'FAULT'
                    self.inject(self.snapshot(), f'STREAM_FAULT s={token} code=9',
                                f'STREAM_FAULT_CAN s={token} node={self.open_refused_node} cmd=47 gap=1',
                                'Stopping ok', 'STOP locked')
                    return len(data)
        if command.startswith('#STREAM_DIAG '):
            node, field = map(int, command.split()[1:])
            with self.lock:
                self.writes.append(command)
                if self.pending_diagnostic is not None:
                    self.inject('error STREAM_DIAG busy')
                    return len(data)
                if self.health_busy_once and node == 1 and field == 7 and self.running_at is not None:
                    self.health_busy_once = False
                    self.inject('error STREAM_DIAG busy')
                    return len(data)
                if ((self.drop_selected_diagnostic or node == self.drop_diagnostic_node) and self.running_at is not None):
                    # Match the core's single mailbox: an unanswered CAN query
                    # owns it until its bounded one-second device timeout.
                    self.pending_diagnostic = (time.perf_counter()+1., f'error STREAM_DIAG node={node} timeout')
                    return len(data)
                self.diag_nonce += 1
                value = (int(self.held[node-1])+int(self.bad_held_after_open and self.session != 0)
                         if field == 7 else self.open_snapshots[node][field])
                reply = f'STREAM_NODE node={node} n={self.diag_nonce} field={field} value={value}'
                if self.diagnostic_delay:
                    self.pending_diagnostic = (time.perf_counter()+self.diagnostic_delay, reply)
                else:
                    self.inject(reply)
            return len(data)
        if command.startswith('!STREAM_CLOSE ') and self.close_missing:
            with self.lock:
                self.writes.append(command)
            return len(data)
        if command == '!STOP' and self.duplicate_stop_locked:
            self.inject('error STOP_LOCKED')
        return super().write(data)

    def _clock(self):
        while not self.closed:
            with self.lock:
                now = time.perf_counter()
                if self.pending_diagnostic is not None and now >= self.pending_diagnostic[0]:
                    self.inject(self.pending_diagnostic[1])
                    self.pending_diagnostic = None
                if self.next_tick is not None and now >= self.next_tick:
                    if not self.buffer:
                        self.inject(f'STREAM_FAULT s={self.session} code=UNDERRUN')
                        self.next_tick = None
                    else:
                        knot = self.buffer.popleft()
                        q = robot_coordinates(knot.motor)[0]
                        joints = to_wire_joints(q)
                        for axis in range(7):
                            if self.mask & (1 << axis):
                                self.held[axis] = knot.motor[axis, 0]
                                self.joints[axis] = joints[axis]+self.actual_offset[axis]
                        self.executed = knot.sequence
                        self.state = 'RUNNING'
                        self.next_tick += .02
                        if knot.flags & 2:
                            self.state = 'IDLE'
                            self.next_tick = None
                            self.inject(f'STREAM_DONE s={self.session} n={knot.sequence+int(self.wrong_done)}')
            time.sleep(.002)


class AxisCommunicationTests(unittest.TestCase):
    def setUp(self):
        self.port = AxisMemoryPort()
        self.c = RobotController(serial_factory=lambda *a, **kw: self.port)
        self.events = []
        self.c.cartesian.stream.axis_test_changed.connect(self.events.append)
        self.guard = patch('app.core.stream_session.PlanningProcess', side_effect=AssertionError('IK forbidden'))
        self.hardware = patch('app.core.controller.serial.Serial', side_effect=AssertionError('hardware forbidden'))
        self.guard.start(); self.hardware.start()

    def tearDown(self):
        self.c.close(); self.port.close(); APPLICATION.processEvents()
        self.guard.stop(); self.hardware.stop()

    def wait(self, predicate, seconds=9.):
        end = time.perf_counter()+seconds
        while not predicate() and time.perf_counter() < end:
            QTest.qWait(5); APPLICATION.processEvents()
        self.assertTrue(predicate(), (self.events[-2:], self.port.commands()[-15:]))

    def ready(self):
        self.c.set_polling(False, 30)
        self.assertTrue(self.c.connect_port('MEMORY_AXIS_ONLY', 'USB'))
        self.wait(lambda: self.c._mode_request is None)
        self.assertTrue(self.c.send_cmd('!START'))
        self.wait(lambda: self.c._motor_enabled and self.c._mode_request is None)
        self.initial_command_count = len(self.port.commands())

    def run_test(self, mask=1, seconds=9.):
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(mask))
        self.wait(lambda: self.events and self.events[-1].get('done'), seconds)
        self.assertFalse(self.c.cartesian.busy)
        return self.events[-1]

    def assert_constant_packets(self, mask, held):
        commands = self.port.commands()[self.initial_command_count:]
        knots = [decode_knot(command)[1] for command in commands if command.startswith('~S')]
        self.assertEqual(len(knots), 101)
        self.assertEqual([k.sequence for k in knots], list(range(101)))
        for knot in knots:
            self.assertFalse(np.any(knot.motor[:, 1:]))
            for axis in range(7):
                self.assertEqual(knot.motor[axis, 0], held[axis] if mask & (1 << axis) else 0)
        self.assertFalse(any(command.startswith(('>', '@', '&')) or command == '!START' for command in commands))
        self.assertLessEqual(self.port.maximum_buffer, 16)
        self.assertEqual(sum(command.startswith('!STREAM_START ') for command in commands), 1)
        self.assertEqual(self.c.cartesian.backend.value, 'firmware')

    def test_j1_constant_50hz_no_ik_and_restore_mode(self):
        held = self.port.held.copy()
        result = self.run_test(1)
        self.assertTrue(result['passed'], result)
        self.assertTrue(result['closed'] and result['restored'])
        self.assertEqual(result['executed'], 100)
        self.assertEqual(self.port.mode, 2)
        self.assert_constant_packets(1, held)

    def test_j7_ignores_unselected_joint_model_limits(self):
        self.port.joints[1] = -9000.
        held = self.port.held.copy()
        result = self.run_test(64)
        self.assertTrue(result['passed'], result)
        self.assert_constant_packets(64, held)
        diags = [c for c in self.port.commands() if c.startswith('#STREAM_DIAG ')]
        self.assertTrue(diags)
        self.assertTrue(all(c.split()[1] == '7' for c in diags))

    def test_two_structures_j1_j7_in_one_session(self):
        held = self.port.held.copy()
        result = self.run_test(65)
        self.assertTrue(result['passed'], result)
        self.assert_constant_packets(65, held)
        self.assertTrue(any(c.endswith(' 65') for c in self.port.commands() if c.startswith('!STREAM_OPEN')))
        self.assertIn('J1+J7', format_axis_test(result))

    def test_gui_stall_does_not_interrupt_credits_or_rx_health(self):
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(65))
        self.wait(lambda: any(c.startswith('!STREAM_START') for c in self.port.commands()))
        time.sleep(.65)
        self.assertGreater(self.port.executed, 20)
        self.assertNotIn('!STOP', self.port.commands()[self.initial_command_count:])
        self.wait(lambda: self.events[-1].get('done'))
        self.assertTrue(self.events[-1]['passed'])

    def test_open_rejection_automatically_captures_cause_then_close_restore(self):
        self.port.open_refused_node = 7
        result = self.run_test(65)
        self.assertFalse(result['passed'])
        self.assertTrue(result['closed'] and result['restored'])
        self.assertEqual(result['snapshots'][7][24], 1 << 12)
        self.assertIn('瞬时估计速度超出 ±512', format_axis_test(result))
        self.assertFalse(any(c.startswith(('~S', '!STREAM_START')) for c in self.port.commands()))
        self.assertEqual(self.port.mode, 2)
        self.assertTrue(self.c._stop_locked)

    def test_old_core_target_capability_rejected_before_mode_open(self):
        self.port.axis_capability = False
        result = self.run_test()
        self.assertFalse(result['passed'])
        self.assertFalse(any(c.startswith(('#CMDMODE', '!STREAM_OPEN', '~S')) for c in self.port.commands()))

    def test_busy_core_never_changes_mode_or_opens(self):
        self.port.busy = True
        result = self.run_test()
        self.assertFalse(result['passed'])
        self.assertIn('仍在运动', result['message'])
        self.assertFalse(any(c.startswith(('#CMDMODE', '!STREAM_OPEN', '~S')) for c in self.port.commands()))

    def test_open_reference_change_blocks_seed_and_cleans_up(self):
        self.port.bad_held_after_open = True
        result = self.run_test()
        self.assertFalse(result['passed'])
        self.assertIn('保持参考改变', result['message'])
        self.assertTrue(result['closed'] and result['restored'])
        self.assertFalse(any(c.startswith(('~S', '!STREAM_START')) for c in self.port.commands()))

    def test_raw_feedback_loss_stops_and_restores(self):
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test())
        self.wait(lambda: self.c.cartesian.stream.started)
        self.port.drop_feedback = True
        self.wait(lambda: self.events[-1].get('done'))
        result = self.events[-1]
        self.assertFalse(result['passed'])
        self.assertIn('实际 RX 超过 300', result['message'])
        self.assertTrue(result['closed'] and result['restored'])
        commands = self.port.commands()
        self.assertIn('!STOP', commands)
        self.assertFalse(any(c.startswith('~S') for c in commands[commands.index('!STOP')+1:]))

    def test_motor_can_diag_loss_stops_while_core_usb_remains_live(self):
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(65))
        self.wait(lambda: self.c.cartesian.stream.started)
        self.port.drop_selected_diagnostic = True
        self.wait(lambda: any(c == '!STOP' for c in self.port.commands()), 3.)
        self.port.drop_selected_diagnostic = False
        self.wait(lambda: self.events[-1].get('done'))
        self.assertIn('CAN 诊断回读超过 600', self.events[-1]['message'])
        self.assertFalse(self.events[-1]['passed'])

    def test_close_missing_keeps_mode7_and_reports_no_recovery(self):
        self.port.close_missing = True
        result = self.run_test()
        self.assertFalse(result['passed'])
        self.assertFalse(result['closed'] or result['restored'])
        self.assertEqual(self.port.mode, 7)
        self.assertFalse(any(c == '#CMDMODE 2' for c in self.port.commands()))

    def test_user_stop_discards_curve_tail_and_retires_in_new_generation(self):
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(65))
        self.wait(lambda: self.c.cartesian.stream.started)
        self.assertTrue(self.c.send_cmd('!STOP'))
        self.wait(lambda: self.events[-1].get('done'))
        result = self.events[-1]
        self.assertFalse(result['passed'])
        self.assertTrue(result['closed'] and result['restored'], result)
        commands = self.port.commands()
        stop = commands.index('!STOP')
        self.assertFalse(any(c.startswith(('~S', '!STREAM_START')) or c == '!START' for c in commands[stop+1:]))

    def test_disconnect_cannot_report_pass_or_send_old_tail(self):
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test())
        self.wait(lambda: self.c.cartesian.stream.started)
        self.c.disconnect_port()
        self.wait(lambda: self.events[-1].get('done'))
        self.assertFalse(self.events[-1]['passed'])
        self.assertIn('结果未知', self.events[-1]['message'])

    def test_entry_guards_and_constant_only_writer_allowlist(self):
        self.ready()
        worker = self.c.cartesian.stream
        for mask in (0, 3, 126, True):
            self.assertFalse(worker.start_axis_test(mask))
        self.assertTrue(worker.start_axis_test())
        self.wait(lambda: hasattr(worker, 'axis_knots'))
        for command in ('!START', '!HOME', '!RESET', '>0,0,0,0,0,0,0,5', '@0,0,0,0,0,0,5'):
            with self.assertRaises(ValueError):
                worker._send(command)
        worker.axis_cleanup = True
        for command in (f'!STREAM_START {worker.session}', f'!STREAM_OPEN {worker.session} 1', worker.axis_knots[0].encode(max(1, worker.session))):
            with self.assertRaises(ValueError):
                worker._send(command)

    def test_motor_int32_seed_not_roundtrip_through_joint_angles(self):
        knots = held_test_knots(65, {1: -2147483648, 7: 2147483647})
        self.assertEqual(knots[0].motor[0, 0], -2147483648)
        self.assertEqual(knots[-1].motor[6, 0], 2147483647)
        self.assertFalse(np.any(knots[25].motor[:, 1:]))

    def test_axis_capability_parser(self):
        result = parse_stream_line('STREAM_AXES v=1 hz=50 mask=65 targeted=1')
        self.assertEqual(result['kind'], 'axis_cap')
        self.assertEqual(result['mask'], 65)

    def test_existing_mode7_is_preserved_after_success(self):
        self.port.mode = 7
        result = self.run_test(64)
        self.assertTrue(result['passed'] and result['restored'])
        self.assertEqual(self.port.mode, 7)
        self.assertFalse(any(c.startswith('#CMDMODE ') for c in self.port.commands()))

    def test_entry_does_not_auto_enable(self):
        self.c.set_polling(False, 30)
        self.assertTrue(self.c.connect_port('MEMORY_DISABLED_ONLY', 'USB'))
        self.wait(lambda: self.c._mode_request is None)
        self.assertFalse(self.c.cartesian.stream.start_axis_test(1))
        self.assertFalse(any(c.startswith(('!START', '!STREAM_OPEN', '~S')) for c in self.port.commands()))

    def test_actual_position_deviation_stops_even_when_held_diag_is_constant(self):
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(64))
        self.wait(lambda: self.c.cartesian.stream.started)
        self.port.actual_offset[6] = 2.
        self.wait(lambda: self.events[-1].get('done'))
        self.assertFalse(self.events[-1]['passed'])
        self.assertIn('原地保持位置偏差超限', self.events[-1]['message'])
        self.assertTrue(self.events[-1]['closed'] and self.events[-1]['restored'])

    def test_user_disable_remains_effective_and_retires_session(self):
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(65))
        self.wait(lambda: self.c.cartesian.stream.started)
        self.assertTrue(self.c.send_cmd('!DISABLE'))
        self.wait(lambda: self.events[-1].get('done'))
        self.assertFalse(self.events[-1]['passed'])
        self.assertFalse(self.c._motor_enabled)
        self.assertTrue(self.events[-1]['closed'] and self.events[-1]['restored'], self.events[-1])
        commands = self.port.commands()
        disabled = commands.index('!DISABLE')
        self.assertFalse(any(c.startswith(('~S', '!STREAM_START')) or c == '!START' for c in commands[disabled+1:]))

    def test_actual_rx_fault_rejects_authorized_curve_at_final_writer(self):
        self.ready()
        worker = self.c.cartesian.stream
        self.assertTrue(worker.start_axis_test(65))
        self.wait(lambda: worker.started)
        command = worker.axis_knots[0].encode(worker.session)
        key = (worker.generation, command)
        with worker.lock:
            worker.authorized[key] += 1
        worker.receive(f'STREAM_FAULT s={worker.session} code=9', worker.connection, time.perf_counter())
        self.assertFalse(worker.owns_command(*key))
        self.assertTrue(worker.recovering)
        self.wait(lambda: self.events[-1].get('done'))
        self.assertFalse(self.events[-1]['passed'])

    def test_stop_between_initial_allowlist_and_new_generation_queue_rejects_old_curve(self):
        self.ready()
        worker = self.c.cartesian.stream
        self.assertTrue(worker.start_axis_test(65))
        self.wait(lambda: worker.started)
        command = worker.axis_knots[0].encode(worker.session)
        reached, release = threading.Event(), threading.Event()
        original = worker._axis_command_allowed
        def barrier(candidate):
            allowed = original(candidate)
            if candidate == command and threading.current_thread().name == 'axis-allowlist-race':
                reached.set()
                release.wait(2.)
            return allowed
        errors = []
        def enqueue_after_stop():
            try:
                worker._send(command)
            except InterruptedError:
                pass
            except Exception as exc:
                errors.append(exc)
        with patch.object(worker, '_axis_command_allowed', side_effect=barrier):
            racing = threading.Thread(target=enqueue_after_stop, name='axis-allowlist-race')
            racing.start()
            self.assertTrue(reached.wait(1.))
            self.assertTrue(self.c.send_cmd('!STOP'))
            release.set()
            racing.join(timeout=2.)
            self.assertFalse(racing.is_alive())
            self.assertFalse(errors)
            self.wait(lambda: self.events[-1].get('done'))
        self.assertEqual(self.port.commands().count(command), 1)
        self.assertFalse(self.events[-1]['passed'])
        self.assertTrue(self.events[-1]['closed'] and self.events[-1]['restored'])

    def test_duplicate_stop_locked_reply_does_not_abort_read_close_recovery(self):
        self.port.duplicate_stop_locked = True
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(65))
        self.wait(lambda: self.c.cartesian.stream.started)
        self.port.drop_feedback = True
        self.wait(lambda: self.events[-1].get('done'))
        self.assertFalse(self.events[-1]['passed'])
        self.assertTrue(self.events[-1]['closed'] and self.events[-1]['restored'], self.events[-1])
        self.assertTrue(self.c._stop_locked)

    def test_malformed_stream_reply_stops_but_still_closes_and_restores(self):
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(65))
        self.wait(lambda: self.c.cartesian.stream.started)
        self.port.inject('STREAM_INVALID_LINE broken')
        self.wait(lambda: self.events[-1].get('done'))
        self.assertFalse(self.events[-1]['passed'])
        self.assertIn('Malformed', self.events[-1]['message'])
        self.assertTrue(self.events[-1]['closed'] and self.events[-1]['restored'], self.events[-1])

    def test_wrong_done_sequence_is_failure_with_bounded_cleanup(self):
        self.port.wrong_done = True
        result = self.run_test(65)
        self.assertFalse(result['passed'])
        self.assertIn('最终序号 100 不一致', result['message'])
        self.assertTrue(result['closed'] and result['restored'])

    def _assert_added_single_node(self, node):
        mask = 1 << (node-1)
        held = self.port.held.copy()
        result = self.run_test(mask)
        self.assertTrue(result['passed'], result)
        self.assert_constant_packets(mask, held)
        diags = [command for command in self.port.commands() if command.startswith('#STREAM_DIAG ')]
        self.assertTrue(diags)
        self.assertTrue(all(int(command.split()[1]) == node for command in diags))
        self.assertIn(f'J{node}', format_axis_test(result))

    def test_j2_single_axis(self):
        self._assert_added_single_node(2)

    def test_j3_single_axis(self):
        self._assert_added_single_node(3)

    def test_j4_single_axis(self):
        self._assert_added_single_node(4)

    def test_j5_single_axis(self):
        self._assert_added_single_node(5)

    def test_j6_single_axis(self):
        self._assert_added_single_node(6)

    def test_all_seven_axes_constant_int32_group_with_one_diagnostic_mailbox(self):
        self.port.diagnostic_delay = .018
        self.port.health_busy_once = True
        held = self.port.held.copy()
        result = self.run_test(127)
        self.assertTrue(result['passed'], result)
        self.assert_constant_packets(127, held)
        self.assertEqual(set(result['held']), set(range(1, 8)))
        for node in range(1, 8):
            self.assertGreater(self.port.commands().count(f'#STREAM_DIAG {node} 7'), 5)
        self.assertTrue(result['closed'] and result['restored'])
        self.assertEqual(self.port.mode, 2)

    def test_all_axes_late_axis_can_loss_detected_without_starvation(self):
        self.port.diagnostic_delay = .018
        self.ready()
        worker = self.c.cartesian.stream
        self.assertTrue(worker.start_axis_test(127))
        self.wait(lambda: worker.started)
        self.port.drop_diagnostic_node = 7
        stopped_at = time.perf_counter()
        self.wait(lambda: any(command == '!STOP' for command in self.port.commands()), 1.5)
        self.assertLess(time.perf_counter()-stopped_at, 1.5)
        self.port.drop_diagnostic_node = None
        self.wait(lambda: self.events[-1].get('done'), 12.)
        result = self.events[-1]
        self.assertFalse(result['passed'])
        self.assertIn('J7 电机 CAN 诊断回读超过 600', result['message'])
        self.assertTrue(result['closed'] and result['restored'], result)

    def test_old_65_capability_keeps_j1_j7_group_compatible(self):
        self.port.axis_cap_mask = 65
        result = self.run_test(65)
        self.assertTrue(result['passed'], result)

    def test_old_65_capability_rejects_other_single_axis_before_mode_open(self):
        self.port.axis_cap_mask = 65
        result = self.run_test(2)
        self.assertFalse(result['passed'])
        self.assertIn('需要更新主控', result['message'])
        self.assertFalse(any(command.startswith(('#CMDMODE', '!STREAM_OPEN', '~S')) for command in self.port.commands()))

    def test_old_65_capability_rejects_all_seven_before_mode_open(self):
        self.port.axis_cap_mask = 65
        result = self.run_test(127)
        self.assertFalse(result['passed'])
        self.assertFalse(any(command.startswith(('#CMDMODE', '!STREAM_OPEN', '~S')) for command in self.port.commands()))

    def test_all_group_open_j4_refusal_reads_each_selected_snapshot(self):
        self.port.open_refused_node = 4
        result = self.run_test(127)
        self.assertFalse(result['passed'])
        self.assertEqual(set(result['snapshots']), set(range(1, 8)))
        self.assertEqual(result['snapshots'][4][24], 1 << 12)
        self.assertTrue(result['closed'] and result['restored'], result)
        self.assertIn('J4', format_axis_test(result))
        self.assertFalse(any(command.startswith(('~S', '!STREAM_START')) for command in self.port.commands()))

    def test_nonfirst_rotary_axis_units_are_degrees_and_deviation_guard_applies(self):
        text = format_axis_test(dict(mask=4, stage='通过', max_actual_deviation={3: np.deg2rad(.2)}))
        self.assertIn('0.2000 °', text)
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(4))
        self.wait(lambda: self.c.cartesian.stream.started)
        self.port.actual_offset[2] = .31
        self.wait(lambda: self.events[-1].get('done'))
        result = self.events[-1]
        self.assertFalse(result['passed'])
        self.assertIn('J3 原地保持位置偏差超限', result['message'])
        self.assertTrue(result['closed'] and result['restored'])

    def test_recovery_diagnostic_allowlist_cannot_read_unselected_axes(self):
        self.ready()
        worker = self.c.cartesian.stream
        self.assertTrue(worker.start_axis_test(8))
        self.wait(lambda: worker.started)
        for node in (1, 2, 3, 5, 6, 7):
            with self.assertRaises(ValueError):
                worker._send(f'#STREAM_DIAG {node} 23')
        worker.request_axis_stop()
        self.assertTrue(worker._axis_command_allowed('#STREAM_DIAG 4 23'))
        self.assertFalse(worker._axis_command_allowed('#STREAM_DIAG 7 23'))

    def test_final_seven_serial_diag_queries_cannot_hide_raw_actual_feedback_loss(self):
        self.port.diagnostic_delay = .018
        observed = []
        def lose_raw_feedback_in_final_check(result):
            if result.get('stage') == '核对最终保持参考' and not result.get('done'):
                observed.append(result['stage'])
                self.port.drop_feedback = True
                self.port.diagnostic_delay = .06
        self.c.cartesian.stream.axis_test_changed.connect(lose_raw_feedback_in_final_check)
        # Keep the imposed 60 ms diagnostic latency during recovery too;
        # seven latched snapshots require 91 serial fields after the STOP.
        result = self.run_test(127, seconds=15.)
        self.assertTrue(observed)
        self.assertFalse(result['passed'], result)
        self.assertIn('实际 RX 超过 300', result['message'])
        self.assertEqual(result['failure_stage'], '核对最终保持参考')
        self.assertTrue(result['closed'] and result['restored'], result)

    def test_first_activation_window_accepts_immediate_raw_rx_and_busy_receipt(self):
        self.ready()
        worker = self.c.cartesian.stream
        original_prepare = worker._prepare_worker
        receipts = []
        def activate_and_immediately_receive(require_poll):
            original_prepare(require_poll)
            # The transport can call receive immediately after active becomes
            # true, before start_axis_test returns or starts its worker thread.
            receipts.append(worker.receive('ok '+' '.join(map(str, self.port.joints)),
                                           worker.connection, time.perf_counter()))
            receipts.append(worker.receive('error STREAM_DIAG busy', worker.connection, time.perf_counter()))
        with patch.object(worker, '_prepare_worker', side_effect=activate_and_immediately_receive):
            self.assertTrue(worker.start_axis_test(8))
        self.assertEqual(receipts, [False, False])
        self.assertTrue(self.c.is_connected)
        self.wait(lambda: self.events[-1].get('done'))
        self.assertTrue(self.events[-1]['passed'], self.events[-1])
        self.assertTrue(self.c.is_connected)

    def test_common_fields_are_initialized_before_active_is_published(self):
        self.ready()
        worker = self.c.cartesian.stream
        allocate = self.c.allocate_command_mode_request
        before = []
        def receive_during_common_initialization():
            self.assertFalse(worker.active)
            for line in ('ok '+' '.join(map(str, self.port.joints)), 'STOP locked', 'error STREAM_DIAG busy'):
                before.append(worker.receive(line, self.c._connection_id, time.perf_counter()))
            return allocate()
        with patch.object(self.c, 'allocate_command_mode_request', side_effect=receive_during_common_initialization):
            self.assertTrue(worker.start_axis_test(16))
        self.assertEqual(before, [False, False, False])
        for field in ('latest_feedback', 'diagnostics', 'session', 'recovering', 'opened', 'request',
                      'axis_feedback', 'axis_health_enabled', 'axis_result'):
            self.assertTrue(hasattr(worker, field), field)
        self.wait(lambda: self.events[-1].get('done'))
        self.assertTrue(self.events[-1]['passed'], self.events[-1])

    def test_stop_in_initial_display_callback_never_sends_open_seed_or_stream_start(self):
        self.ready()
        stopped = []
        def stop_on_first_state(result):
            if result.get('stage') == '检查主控' and not stopped:
                stopped.append(self.c.send_cmd('!STOP'))
        self.c.cartesian.stream.axis_test_changed.connect(stop_on_first_state)
        self.assertTrue(self.c.cartesian.stream.start_axis_test(32))
        self.wait(lambda: self.events[-1].get('done'))
        self.assertEqual(stopped, [True])
        self.assertFalse(self.events[-1]['passed'])
        self.assertFalse(any(c.startswith(('!STREAM_OPEN ', '!STREAM_START ', '~S')) for c in self.port.commands()))

    def test_disconnect_in_initial_display_callback_never_sends_motion(self):
        self.ready()
        disconnected = []
        def disconnect_on_first_state(result):
            if result.get('stage') == '检查主控' and not disconnected:
                disconnected.append(True)
                self.c.disconnect_port()
        self.c.cartesian.stream.axis_test_changed.connect(disconnect_on_first_state)
        self.assertTrue(self.c.cartesian.stream.start_axis_test(4))
        self.wait(lambda: self.events[-1].get('done'))
        self.assertFalse(self.events[-1]['passed'])
        self.assertFalse(self.c.is_connected)
        self.assertFalse(any(c.startswith(('!STREAM_OPEN ', '!STREAM_START ', '~S')) for c in self.port.commands()))


if __name__ == '__main__':
    unittest.main()
