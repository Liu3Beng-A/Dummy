"""Exercise explicit nonzero diagnostics with real RX/TX threads and no hardware."""
import time
import threading
from types import SimpleNamespace, MethodType
import unittest
from unittest.mock import patch
import numpy as np
from PySide6.QtTest import QTest

from app.core.axis_communication_test import (
    moving_test_knots, counter_delta, format_axis_test, TIMING_SCHEMA, TIMING_FIELDS, AxisCommunicationTestMixin)
from app.core.controller import RobotController
from app.core.diagnostic_session import DiagnosticSession
from app.ui.terminal_diagnostics import TerminalDiagnostics
from dummy_motion.stream_protocol import decode_knot, robot_coordinates, parse_stream_line, MOTOR_SCALE, MOTOR_ZERO
from dummy_motion.frames import to_wire_joints, from_wire_joints
from tests.test_axis_communication_test import AxisMemoryPort
from tests.test_ui import APPLICATION


class MotionAxisMemoryPort(AxisMemoryPort):
    def __init__(self):
        self.motion_schema = TIMING_SCHEMA
        self.freeze_actual = False
        self.reverse_actual = False
        self.outside_actual = False
        self.motion_samples = []
        self.diagnostic_values = {field: 0 for field in TIMING_FIELDS}
        self.diagnostic_values[63] = 64
        self.stale_after = False
        self.stale_nonce = None
        self.late_increment = 0
        self.drop_increment = 0
        # Synthetic state-machine evidence; these values are not hardware timing.
        self.running_peak = 2000
        self.running_sample_increment = 1250
        self.curve_fault_at = None
        super().__init__()
        self.initial_joints = list(self.joints)

    def write(self, data):
        command = data.decode('ascii').strip()
        if command.startswith('~S') and self.curve_fault_at is not None:
            session, knot = decode_knot(command)
            if knot.sequence == self.curve_fault_at:
                with self.lock:
                    self.writes.append(command)
                    self.inject(f'STREAM_FAULT s={session} code=17')
                return len(data)
        if command.startswith('#STREAM_DIAG '):
            node, field = map(int, command.split()[1:])
            if field in (5, 6, 51, *TIMING_FIELDS):
                with self.lock:
                    self.writes.append(command)
                    self.diag_nonce = (self.diag_nonce+1) % 65536
                    value = (self.motion_schema if field == 51 else
                             round((self.motor_velocity if field == 5 else self.motor_acceleration)*51200)
                             if field in (5, 6) else self.diagnostic_values[field])
                    nonce = self.diag_nonce
                    if self.stale_after and field == 12 and self.motion_samples:
                        nonce = self.stale_nonce
                    elif field == 12:
                        self.stale_nonce = nonce
                    self.inject(f'STREAM_NODE node={node} n={nonce} field={field} value={value}')
                    return len(data)
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
                        joints = to_wire_joints(robot_coordinates(knot.motor)[0])
                        for axis in range(7):
                            if self.mask & (1 << axis):
                                self.held[axis] = knot.motor[axis, 0]
                                if not self.freeze_actual:
                                    actual = joints[axis]
                                    if self.reverse_actual:
                                        actual = 2*self.initial_joints[axis]-actual
                                    if self.outside_actual:
                                        actual += 2.
                                    self.joints[axis] = actual+self.actual_offset[axis]
                        self.motion_samples.append((knot.sequence, tuple(self.joints)))
                        self.executed = knot.sequence
                        self.state = 'RUNNING'
                        self.next_tick += .02
                        if knot.flags & 2:
                            self.diagnostic_values[12] = (self.diagnostic_values[12]+self.late_increment) & 0xffffffff
                            self.diagnostic_values[18] = (self.diagnostic_values[18]+self.drop_increment) & 0xffffffff
                            self.diagnostic_values[58] = self.running_peak
                            self.diagnostic_values[61] = (self.diagnostic_values[61]+self.running_sample_increment) & 0xffffffff
                            self.state = 'IDLE'; self.next_tick = None
                            self.inject(f'STREAM_DONE s={self.session} n={knot.sequence+int(self.wrong_done)}')
            time.sleep(.002)


class NonzeroAxisMotionTests(unittest.TestCase):
    def setUp(self):
        self.port = MotionAxisMemoryPort()
        self.controller = RobotController(serial_factory=lambda *a, **kw: self.port)
        self.events = []
        self.controller.cartesian.stream.axis_test_changed.connect(self.events.append)
        self.ik = patch('dummy_motion.ik.IKSolver.solve_pose', side_effect=AssertionError('IK forbidden'))
        self.hardware = patch('app.core.controller.serial.Serial', side_effect=AssertionError('hardware forbidden'))
        self.ik.start(); self.hardware.start()

    def tearDown(self):
        self.controller.close(); self.port.close(); APPLICATION.processEvents()
        self.ik.stop(); self.hardware.stop()

    def wait(self, condition, seconds=11.):
        end = time.perf_counter()+seconds
        while not condition() and time.perf_counter() < end:
            QTest.qWait(5); APPLICATION.processEvents()
        self.assertTrue(condition(), (self.events[-2:], self.port.commands()[-10:]))

    def ready(self):
        c = self.controller
        c.set_polling(False, 30)
        self.assertTrue(c.connect_port('MEMORY_NONZERO_AXIS', 'USB'))
        self.wait(lambda: c._mode_request is None)
        self.assertTrue(c.send_cmd('!START'))
        self.wait(lambda: c._motor_enabled and c._mode_request is None)
        self.initial_count = len(self.port.commands())

    def run_motion(self, mask):
        self.ready()
        self.assertTrue(self.controller.cartesian.stream.start_motion_axis_test(mask))
        self.wait(lambda: self.events and self.events[-1].get('done'))
        return self.events[-1]

    def assert_selected_motion(self, node, result):
        self.assertTrue(result['passed'], result)
        self.assertTrue(result['closed'] and result['restored'])
        self.assertGreaterEqual(result['motion_actual_peak_deg'], .15)
        self.assertEqual(result['accepted'], 200)
        self.assertEqual(result['executed'], 200)
        commands = self.port.commands()[self.initial_count:]
        self.assertFalse(any(c == '!START' or c.startswith(('>', '@', '&')) for c in commands))
        knots = [decode_knot(c)[1] for c in commands if c.startswith('~S')]
        self.assertEqual(len(knots), 201)
        self.assertEqual([k.sequence for k in knots], list(range(201)))
        self.assertEqual(knots[0].motor[node-1, 0], knots[-1].motor[node-1, 0])
        self.assertTrue(all(not np.any(k.motor[[i for i in range(7) if i != node-1]]) for k in knots))
        self.assertTrue(any(np.any(k.motor[node-1, 1:]) for k in knots))
        for other in range(7):
            if other != node-1:
                self.assertEqual(self.port.joints[other], self.port.initial_joints[other])
        self.assertAlmostEqual(self.port.joints[node-1], self.port.initial_joints[node-1], places=3)
        self.assertEqual(self.port.mode, 2)
        self.assertIn('不能相加', format_axis_test(result))

    def test_j3_negative_half_degree_real_feedback_and_return(self):
        self.assert_selected_motion(3, self.run_motion(4))
        self.assertLess(min(sample[1][2] for sample in self.port.motion_samples), 79.6)

    def test_j5_positive_half_degree_real_feedback_and_return(self):
        self.assert_selected_motion(5, self.run_motion(16))
        self.assertGreater(max(sample[1][4] for sample in self.port.motion_samples), 90.4)

    def test_stationary_initial_correct_direction_error_is_not_movement(self):
        # The memory admission deliberately accepts this initial tracking error.
        # Host motion evidence must still start at actual RX, not held reference.
        self.port.joints[2] -= .2
        self.port.freeze_actual = True
        result = self.run_motion(4)
        self.assertFalse(result['passed'])
        self.assertIn('0.15', result['message'])
        self.assertAlmostEqual(result['motion_actual_peak_deg'], 0.)
        self.assertAlmostEqual(result['motion_baseline_deg'], 79.8)
        self.assertTrue(result['closed'] and result['restored'])

    def test_no_actual_motion_does_not_pass_after_done(self):
        self.port.freeze_actual = True
        result = self.run_motion(4)
        self.assertFalse(result['passed'])
        self.assertIn('0.15', result['message'])
        self.assertTrue(result['closed'] and result['restored'])

    def test_reverse_actual_direction_does_not_pass(self):
        self.port.reverse_actual = True
        result = self.run_motion(16)
        self.assertFalse(result['passed'])
        self.assertIn('包络', result['message'])
        self.assertIn('!STOP', self.port.commands())

    def test_actual_outside_authorized_envelope_stops(self):
        self.port.outside_actual = True
        result = self.run_motion(4)
        self.assertFalse(result['passed'])
        self.assertIn('包络', result['message'])
        self.assertTrue(result['closed'] and result['restored'])

    def test_old_motor_schema_refused_before_mode_open_motion(self):
        self.port.motion_schema = 0
        result = self.run_motion(4)
        self.assertFalse(result['passed'])
        self.assertIn('新版电机计时能力', result['message'])
        self.assertFalse(any(c.startswith(('!STREAM_OPEN', '~S', '#CMDMODE')) for c in self.port.commands()))

    def test_j3_limit_refused_not_reversed(self):
        self.port.joints[2] = 0.
        self.port.held[2] = round((np.deg2rad(0)-MOTOR_ZERO[2])*MOTOR_SCALE[2])
        result = self.run_motion(4)
        self.assertFalse(result['passed'])
        self.assertIn('硬限位', result['message'])
        self.assertFalse(any(c.startswith(('!STREAM_OPEN', '~S')) for c in self.port.commands()))

    def test_all_other_masks_rejected_without_any_command(self):
        self.ready()
        start = len(self.port.commands())
        for mask in (1, 2, 8, 32, 64, 65, 127, True):
            self.assertFalse(self.controller.cartesian.stream.start_motion_axis_test(mask))
        self.assertEqual(self.port.commands()[start:], [])

    def test_stop_retires_session_and_never_auto_enables(self):
        self.ready()
        worker = self.controller.cartesian.stream
        self.assertTrue(worker.start_motion_axis_test(4))
        self.wait(lambda: worker.started)
        self.assertTrue(self.controller.send_cmd('!STOP'))
        self.wait(lambda: self.events and self.events[-1].get('done'))
        result = self.events[-1]
        self.assertFalse(result['passed'])
        self.assertTrue(result['closed'] and result['restored'])
        commands = self.port.commands()
        stopped = commands.index('!STOP')
        self.assertFalse(any(c.startswith(('~S', '!STREAM_START')) or c == '!START' for c in commands[stopped+1:]))

    def test_unconfirmed_timing_reply_is_not_reused_for_pass(self):
        self.port.stale_after = True
        result = self.run_motion(4)
        self.assertFalse(result['passed'])
        self.assertIn('计时', format_axis_test(result))
        self.assertTrue(result['closed'] and result['restored'])

    def test_late_increment_fails_performance_after_successful_motion_and_close(self):
        self.port.late_increment = 1
        result = self.run_motion(4)
        self.assertFalse(result['passed'])
        self.assertTrue(result['motion_verified'])
        self.assertFalse(result['performance_passed'])
        self.assertTrue(result['closed'] and result['restored'])
        self.assertIn('性能项未通过', result['message'])
        self.assertNotIn('!STOP', self.port.commands()[self.initial_count:])

    def test_drop_increment_fails_performance_and_uint32_wrap_is_not_hidden(self):
        self.port.diagnostic_values[12] = 0xfffffff5
        self.port.late_increment = 20
        self.port.drop_increment = 1
        result = self.run_motion(16)
        self.assertFalse(result['passed'])
        self.assertTrue(result['closed'] and result['restored'])
        self.assertIn('增量 20', result['message'])
        self.assertIn('丢弃增量 1', result['message'])

    def test_zero_running_sample_evidence_cannot_pass_performance(self):
        self.port.running_sample_increment = 0
        result = self.run_motion(4)
        self.assertFalse(result['passed'])
        self.assertTrue(result['motion_verified'])
        self.assertFalse(result['performance_passed'])
        self.assertTrue(result['closed'] and result['restored'])
        self.assertIn('RUNNING 计时证据', result['message'])

    def test_zero_dwt_running_peak_cannot_pass_performance(self):
        self.port.running_peak = 0
        result = self.run_motion(16)
        self.assertFalse(result['passed'])
        self.assertTrue(result['motion_verified'])
        self.assertFalse(result['performance_passed'])
        self.assertTrue(result['closed'] and result['restored'])
        self.assertIn('RUNNING 计时证据', result['message'])

    def test_new_boot_peak_warning_does_not_masquerade_as_this_round_wcet(self):
        self.port.running_peak = 3601
        result = self.run_motion(4)
        self.assertTrue(result['passed'])
        self.assertIn('性能待复查', format_axis_test(result))
        self.assertIn('不能相加', format_axis_test(result))

    def test_first_firmware_fault_survives_stop_and_cleanup(self):
        self.port.curve_fault_at = 25
        result = self.run_motion(4)
        self.assertFalse(result['passed'])
        self.assertEqual(result['primary_fault']['code'], '17')
        self.assertIn('17 (DEADLINE)', result['message'])
        self.assertTrue(result['closed'] and result['restored'])


class NonzeroCurveContractTests(unittest.TestCase):
    def test_exact_seed_inactive_axes_derivatives_and_end(self):
        for node, mask in ((3, 4), (5, 16)):
            for seed in (1, 7, 123456):
                knots = moving_test_knots(mask, {node: seed}, 1536000, 1536000)
                self.assertEqual(knots[0].motor[node-1, 0], seed)
                self.assertEqual(knots[-1].motor[node-1, 0], seed)
                self.assertEqual(knots[0].flags, 1)
                self.assertEqual(knots[-1].flags, 2)
                self.assertFalse(np.any(knots[0].motor[:, 1:]))
                self.assertFalse(np.any(knots[-1].motor[:, 1:]))
                self.assertGreater(max(abs(k.motor[node-1, 1]) for k in knots), 3000)
                for k in knots:
                    other = k.motor.copy(); other[node-1] = 0
                    self.assertFalse(np.any(other))
                self.assertEqual(knots[100].motor[node-1, 1], 0)
                self.assertEqual(knots[100].motor[node-1, 2], 0)

    def test_bad_mask_and_motor_limits_refuse(self):
        for mask in (1, 65, 127, True):
            with self.assertRaises(ValueError):
                moving_test_knots(mask, {3: 1000}, 1536000, 1536000)
        for velocity, acceleration in ((1, 1536000), (1536000, 1), (0, 1536000), (1536000, -1)):
            with self.assertRaises(ValueError):
                moving_test_knots(4, {3: 1000}, velocity, acceleration)

    def test_prefill_unconfirmed_and_old_rx_cannot_contribute_motion_peak(self):
        actual = from_wire_joints([0, 10, 79.8, 0, 90, 0, 0])
        state = SimpleNamespace(active=True, connection=1, lock=threading.RLock(),
                                owner=SimpleNamespace(kind='axis_test', model=SimpleNamespace(valid=lambda q: True)),
                                axis_feedback=(actual.copy(), 9.9), latest_feedback=(actual.copy(), 9.9),
                                axis_rx_gap=0., axis_motion=True, started=False, running_confirmed=False,
                                axis_cleanup=False, axis_start_actual=actual.copy(), axis_start_stamp=10.,
                                start_requested_at=10.02, axis_deviation={}, axis_actual_peak=0.,
                                axis_result=dict(held={3:711111}, motion_degrees=-.5))
        state._axis_motion_feedback_allowed = MethodType(AxisCommunicationTestMixin._axis_motion_feedback_allowed, state)
        DiagnosticSession.receive(state, 'ok 0 10 79.4 0 90 0 0', 1, 10.01)  # prefill
        self.assertEqual(state.axis_actual_peak, 0.)
        state.started = True
        DiagnosticSession.receive(state, 'ok 0 10 79.4 0 90 0 0', 1, 10.03)  # START requested, RUNNING not confirmed
        self.assertEqual(state.axis_actual_peak, 0.)
        state.running_confirmed = True
        DiagnosticSession.receive(state, 'ok 0 10 79.4 0 90 0 0', 1, 10.015)  # old timestamp
        self.assertEqual(state.axis_actual_peak, 0.)
        DiagnosticSession.receive(state, 'ok 0 10 79.62 0 90 0 0', 1, 10.05)
        self.assertAlmostEqual(np.rad2deg(state.axis_actual_peak), .18)
        self.assertAlmostEqual(np.rad2deg(state.axis_deviation[3]), .18)

    def test_counter_wrap_and_new_field_parser_bound(self):
        self.assertEqual(counter_delta(0xfffffff0, 5), 21)
        self.assertEqual(parse_stream_line(f'STREAM_NODE node=3 n=1 field=51 value={TIMING_SCHEMA}')['value'], TIMING_SCHEMA)
        self.assertEqual(parse_stream_line('STREAM_NODE node=5 n=2 field=63 value=64')['field'], 63)
        with self.assertRaises(ValueError):
            parse_stream_line('STREAM_NODE node=5 n=3 field=64 value=0')


class NonzeroAxisUiTests(unittest.TestCase):
    def test_only_j3_j5_button_is_explicit_motion_and_gated(self):
        ui = TerminalDiagnostics()
        self.addCleanup(ui.close)
        requests = []
        ui.motion_axis_test_requested.connect(requests.append)
        state = dict(connected=True, locked=False, calibrating=False, connection_type='USB', disabled=False, busy=False)
        ui.set_state(**state)
        for index, mask in enumerate((1, 2, 4, 8, 16, 32, 64, 65, 127)):
            ui.axis_test_selector.setCurrentIndex(index)
            self.assertEqual(ui.motion_axis_test_button.isEnabled(), mask in (4, 16))
            ui.motion_axis_test_button.click()
        self.assertEqual(requests, [4, 16])
        self.assertIn('会真实运动', ui.motion_axis_test_button.toolTip())
        self.assertIn('不自动使能', ui.motion_axis_test_button.toolTip())
        for changed in (dict(locked=True), dict(disabled=True), dict(busy=True), dict(connected=False)):
            ui.axis_test_selector.setCurrentIndex(2)
            ui.set_state(**(state | changed))
            self.assertFalse(ui.motion_axis_test_button.isEnabled())


if __name__ == '__main__':
    unittest.main()
