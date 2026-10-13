"""Modern entry evidence, legacy fallback, fault priority and real UI targets."""
import threading
import time
import unittest
from unittest.mock import patch

from PySide6.QtTest import QTest
from app.core.controller import RobotController
from app.core.open_test import format_open_test, feature_supported
from app.core.axis_communication_test import format_axis_test
from app.ui.terminal_diagnostics import TerminalDiagnostics
from dummy_motion.stream_protocol import parse_stream_line
from tests import test_axis_communication_test as axis_tests
from tests import test_open_test as open_tests
from tests.test_ui import APPLICATION


class ModernProbePort(open_tests.ProbePort):
    def __init__(self):
        super().__init__()
        self.feature_token_offset = 0

    def write(self, data):
        command = data.decode('ascii').strip()
        if command.startswith('#STREAM_DIAG '):
            node, field = map(int, command.split()[1:])
            if field in (45, 46, 47):
                with self.lock:
                    self.writes.append(command)
                    self.diag_nonce += 1
                    self.inject(f'STREAM_NODE node={node} n={self.diag_nonce} field={field} value={dict(zip((45,46,47),(10,80,100)))[field]}')
                return len(data)
        result = super().write(data)
        if command.startswith('!STREAM_TEST '):
            bit = 1 << (self.probe_node-1)
            self.inject(f'STREAM_FEATURES s={self.probe_session+self.feature_token_offset} steady={bit} start={bit}')
        return result


class ModernAxisPort(axis_tests.AxisMemoryPort):
    def __init__(self):
        self.modern = True
        self.fail_phase = None
        self.start_snapshots = {}
        self.fault_delay = .035
        self.manual_fault_delivery = False
        self.pending_fault = None
        self.start_delay = .1
        self.start_bits = 1 << 12
        self.feature_token_offset = 0
        self.snapshot_token_offset = 0
        self.hold_close = False
        self.held_close_command = None
        self.timers = []
        super().__init__()

    def features(self):
        mask = self.mask if self.modern else 0
        return f'STREAM_FEATURES s={self.session+self.feature_token_offset} steady={mask} start={mask}'

    def latch_entries(self, token, mask):
        for node in range(1, 8):
            if mask & (1 << (node-1)):
                held = int(self.held[node-1])
                self.open_snapshots[node] = {23: token, 24: (1 << 12) if self.fail_phase == 'OPEN' else 0,
                    25: 257, 26: 857, 27: 0, 28: held, 29: held, 30: held, 31: 0, 32: 60000, 33: 1,
                    45: 20, 46: 80, 47: 0}
                self.start_snapshots[node] = {34: token+self.snapshot_token_offset, 35: self.start_bits if self.fail_phase == 'START' else 0,
                    36: 257, 37: 857, 38: 0, 39: held, 40: held, 41: held, 42: -8458, 43: 65000, 44: 1,
                    48: 0xffffffff if self.fail_phase == 'START' else 10,
                    49: 0xffffffff if self.fail_phase == 'START' else 50, 50: 0}

    def inject_fault_after_stop(self, code):
        # Deterministically reproduce the real USB ordering: a rejected poll
        # says STOP_LOCKED before the periodic numeric STREAM_FAULT arrives.
        token = self.session
        self.inject('error STOP_LOCKED')
        if self.manual_fault_delivery:
            self.pending_fault = (token, code)
            return
        def deliver():
            self._deliver_fault(token, code, with_stop=True)
        timer = threading.Timer(self.fault_delay, deliver)
        timer.daemon = True
        self.timers.append(timer)
        timer.start()

    def _deliver_fault(self, token, code, with_stop=False):
        if not self.closed:
            lines = [f'STREAM_FAULT s={token} code={code}',
                     f'STREAM_FAULT_CAN s={token} node=4 cmd=47 gap=0']
            if with_stop:
                lines.extend(('Stopping ok', 'STOP locked'))
            self.inject(*lines)

    def deliver_pending_fault(self):
        if self.pending_fault is None:
            raise AssertionError('No held numeric fault is available')
        token, code = self.pending_fault
        self.pending_fault = None
        self._deliver_fault(token, code)

    def write(self, data):
        command = data.decode('ascii').strip()
        if command.startswith('!STREAM_CLOSE ') and self.hold_close:
            with self.lock:
                self.writes.append(command)
                self.held_close_command = command
            return len(data)
        if command.startswith('!STREAM_OPEN '):
            token, mask = map(int, command.split()[1:])
            self.mask = mask
            self.latch_entries(token, mask)
            if self.fail_phase == 'OPEN':
                with self.lock:
                    self.writes.append(command)
                    self.session = token
                    self.state = 'FAULT'
                    self.inject(self.features())
                    self.inject_fault_after_stop(9)
                return len(data)
            result = super().write(data)
            # AxisMemoryPort creates the old OPEN shape. Restore modern fields.
            self.latch_entries(token, mask)
            self.inject(self.features())
            return result
        if command.startswith('!STREAM_START ') and self.fail_phase == 'START':
            with self.lock:
                self.writes.append(command)
                self.state = 'FAULT'
                self.inject_fault_after_stop(15)
            return len(data)
        if command.startswith('#STREAM_DIAG '):
            node, field = map(int, command.split()[1:])
            if field >= 34:
                with self.lock:
                    self.writes.append(command)
                    self.diag_nonce += 1
                    value = (self.open_snapshots[node][field] if 45 <= field <= 47 else
                             self.start_snapshots[node][field])
                    self.inject(f'STREAM_NODE node={node} n={self.diag_nonce} field={field} value={value}')
                return len(data)
        result = super().write(data)
        if command.startswith('!STREAM_START ') and self.start_delay != .1:
            with self.lock:
                self.running_at = self.next_tick = time.perf_counter()+self.start_delay
        return result

    def close(self):
        for timer in self.timers:
            timer.cancel()
        super().close()


class EntryAxisTests(unittest.TestCase):
    tearDown = axis_tests.AxisCommunicationTests.tearDown
    ready = axis_tests.AxisCommunicationTests.ready
    wait = axis_tests.AxisCommunicationTests.wait
    run_test = axis_tests.AxisCommunicationTests.run_test

    def setUp(self):
        self.port = ModernAxisPort()
        self.c = RobotController(serial_factory=lambda *a, **kw: self.port)
        self.events = []
        self.c.cartesian.stream.axis_test_changed.connect(self.events.append)
        self.guard = patch('app.core.stream_session.PlanningProcess', side_effect=AssertionError('IK forbidden'))
        self.hardware = patch('app.core.controller.serial.Serial', side_effect=AssertionError('hardware forbidden'))
        self.guard.start(); self.hardware.start()

    def test_new_cap_success_retains_default_mode(self):
        result = self.run_test(8)
        self.assertTrue(result['passed'] and result['closed'] and result['restored'], result)
        self.assertEqual(result['steady_nodes'], (4,))
        self.assertEqual(self.port.mode, 2)

    def test_open_numeric_fault_overrides_earlier_stop_generic(self):
        self.port.fail_phase = 'OPEN'
        result = self.run_test(8)
        self.assertEqual(result['code'], '9')
        self.assertIn('9 (BLOCKED)', result['message'])
        self.assertEqual(result['failure_stage'], '等待 OPEN')
        self.assertTrue(result['closed'] and result['restored'])
        text = format_axis_test(result)
        self.assertIn('新鲜 50 ms 稳态证据不足或过期', text)
        self.assertIn('瞬时速度 857', text)
        self.assertNotIn('瞬时估计速度超出 ±512', text)
        self.assertFalse(result['start_snapshots'])
        self.assertIn('尚未发送 STREAM_START', result['start_snapshot_errors'][4])

    def test_start_fault_reports_waiting_start_and_independent_snapshot(self):
        self.port.fail_phase = 'START'
        result = self.run_test(8)
        self.assertIn('15 (BAD_SEED)', result['message'])
        self.assertEqual(result['failure_stage'], '等待同步启动')
        self.assertFalse(any(event.get('stage') == '执行原地曲线' for event in self.events))
        self.assertEqual(result['snapshots'][4][24], 0)
        self.assertEqual(result['start_snapshots'][4][35], 1 << 12)
        self.assertEqual(result['start_snapshots'][4][42], -8458)
        self.assertIn('START 拒绝位', format_axis_test(result))
        self.assertTrue(result['closed'] and result['restored'])

    def test_start_binding_bit_is_distinct_from_open(self):
        self.port.fail_phase = 'START'
        self.port.start_bits = 1 << 19
        result = self.run_test(8)
        self.assertIn('目标或零点与 OPEN 锁存参考不匹配', format_axis_test(result))

    def test_legacy_start_failure_never_queries_new_fields(self):
        self.port.modern = False
        self.port.fail_phase = 'START'
        result = self.run_test(8)
        self.assertIn('15 (BAD_SEED)', result['message'])
        self.assertIn('旧固件', result['start_snapshot_errors'][4])
        self.assertFalse(any(int(command.split()[2]) >= 34 for command in self.port.commands()
                             if command.startswith('#STREAM_DIAG ')))

    def test_wrong_feature_session_is_not_used(self):
        self.port.feature_token_offset = -1
        self.port.fail_phase = 'START'
        result = self.run_test(8)
        self.assertFalse(result.get('steady_nodes'))
        self.assertFalse(result['start_snapshots'])
        self.assertFalse(any(int(command.split()[2]) >= 34 for command in self.port.commands()
                             if command.startswith('#STREAM_DIAG ')))

    def test_start_snapshot_session_mismatch_is_explicit(self):
        self.port.fail_phase = 'START'
        self.port.snapshot_token_offset = -1
        result = self.run_test(8)
        self.assertFalse(result['start_snapshots'])
        self.assertIn('START 快照会话', result['start_snapshot_errors'][4])
        self.assertIn('不匹配', result['start_snapshot_errors'][4])
        self.assertEqual(result['snapshots'][4][23], result['session'])

    def test_late_running_confirmation_keeps_waiting_stage(self):
        self.port.start_delay = .4
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(8))
        self.wait(lambda: self.c.cartesian.stream.started)
        QTest.qWait(120); APPLICATION.processEvents()
        self.assertEqual(self.events[-1]['stage'], '等待同步启动')
        self.assertFalse(self.c.cartesian.stream.running_confirmed)
        self.wait(lambda: self.events[-1].get('done'))
        stages = [event.get('stage') for event in self.events]
        self.assertIn('执行原地曲线', stages)
        self.assertTrue(self.events[-1]['passed'])

    def external_stop_then_numeric_fault_during_cleanup(self, command):
        self.port.fail_phase = 'START'
        self.port.manual_fault_delivery = True
        self.ready()
        worker = self.c.cartesian.stream
        original_snapshot = worker._axis_snapshot
        cleanup_entered, cleanup_release = threading.Event(), threading.Event()
        def hold_snapshot(node, phase='OPEN'):
            if phase == 'OPEN' and not cleanup_entered.is_set():
                cleanup_entered.set()
                if not cleanup_release.wait(3.):
                    raise AssertionError('Test did not release the cleanup barrier')
            return original_snapshot(node, phase)
        try:
            with patch.object(worker, '_axis_snapshot', side_effect=hold_snapshot):
                self.assertTrue(worker.start_axis_test(8))
                self.wait(lambda: worker.started)
                self.wait(lambda: self.port.pending_fault is not None)
                self.assertTrue(self.c.send_cmd(command))
                self.wait(cleanup_entered.is_set)
                self.assertIn(command, self.port.commands())
                self.assertTrue(worker.active and worker.axis_cleanup and worker.axis_abort)
                self.assertIsNone(worker.primary_fault)
                self.port.deliver_pending_fault()
                # Observe actual RX of the held numeric fault while cleanup is
                # still blocked. Scheduler speed cannot move it after CLOSE.
                self.wait(lambda: worker.primary_fault is not None)
                self.assertTrue(worker.active and worker.axis_cleanup)
                cleanup_release.set()
                self.wait(lambda: self.events[-1].get('done'))
        finally:
            cleanup_release.set()
        return self.events[-1]

    def test_manual_stop_keeps_interruption_over_late_primary_fault(self):
        result = self.external_stop_then_numeric_fault_during_cleanup('!STOP')
        self.assertEqual(result['stage'], '已中断')
        self.assertIn('用户', result['message'])
        self.assertEqual(result['failure_stage'], '等待同步启动')
        self.assertEqual(result['primary_fault']['stage'], '等待同步启动')
        self.assertIn('15 (BAD_SEED)', format_axis_test(result))
        commands = self.port.commands()
        stop = commands.index('!STOP')
        self.assertFalse(any(command.startswith(('~S', '!STREAM_START ', '!START')) for command in commands[stop+1:]))

    def test_manual_disable_stays_effective_over_late_fault(self):
        result = self.external_stop_then_numeric_fault_during_cleanup('!DISABLE')
        self.assertEqual(result['stage'], '已中断')
        self.assertFalse(self.c._motor_enabled)
        self.assertIn('15 (BAD_SEED)', format_axis_test(result))

    def test_numeric_fault_after_close_does_not_relabel_completed_interruption(self):
        self.port.fail_phase = 'START'
        self.port.manual_fault_delivery = True
        self.ready()
        worker = self.c.cartesian.stream
        self.assertTrue(worker.start_axis_test(8))
        self.wait(lambda: worker.started)
        self.wait(lambda: self.port.pending_fault is not None)
        self.assertTrue(self.c.send_cmd('!DISABLE'))
        self.wait(lambda: self.events[-1].get('done'))
        before = dict(self.events[-1])
        self.assertEqual(before['stage'], '已中断')
        self.assertTrue(before['closed'] and before['restored'])
        self.assertIsNone(worker.primary_fault)
        self.assertFalse(worker.active)
        delivered = threading.Event()
        original_observer = self.c._transport.line_observer
        def observe_late_fault(line, connection, received_at):
            result = original_observer(line, connection, received_at)
            if line == f"STREAM_FAULT s={before['session']} code=15":
                delivered.set()
            return result
        self.c._transport.line_observer = observe_late_fault
        try:
            self.port.deliver_pending_fault()
            self.wait(delivered.is_set)
            APPLICATION.processEvents()
            self.assertIsNone(worker.primary_fault)
            self.assertEqual(self.events[-1], before)
            self.assertFalse(self.c._motor_enabled)
        finally:
            self.c._transport.line_observer = original_observer

    def test_manual_stop_after_primary_fault_still_has_interrupted_title(self):
        self.port.fail_phase = 'START'
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(8))
        self.wait(lambda: self.c.cartesian.stream.primary_fault is not None)
        self.assertTrue(self.c.send_cmd('!STOP'))
        self.wait(lambda: self.events[-1].get('done'))
        self.assertEqual(self.events[-1]['stage'], '已中断')
        self.assertIn('15 (BAD_SEED)', format_axis_test(self.events[-1]))

    def test_manual_stop_during_successful_close_cannot_report_pass(self):
        self.port.hold_close = True
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(8))
        self.wait(lambda: self.port.held_close_command is not None)
        self.assertTrue(self.c.send_cmd('!STOP'))
        self.port.hold_close = False
        self.port.write((self.port.held_close_command+'\n').encode('ascii'))
        self.wait(lambda: self.events[-1].get('done'))
        result = self.events[-1]
        self.assertEqual(result['stage'], '已中断')
        self.assertFalse(result['passed'])
        self.assertTrue(result['closed'] and result['restored'], result)

    def test_manual_stop_without_close_confirmation_preserves_recovery_error(self):
        self.port.close_missing = True
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_axis_test(8))
        self.wait(lambda: self.c.cartesian.stream.started)
        self.assertTrue(self.c.send_cmd('!STOP'))
        self.wait(lambda: self.events[-1].get('done'), seconds=12.)
        result = self.events[-1]
        self.assertEqual(result['stage'], '已中断')
        self.assertFalse(result['passed'] or result['closed'] or result['restored'])
        self.assertIn('恢复未完成', result['message'])
        self.assertIn('CLOSE 未确认', result['message'])
        self.assertIn('CLOSE 未确认', format_axis_test(result))


class EntryOpenTests(unittest.TestCase):
    tearDown = open_tests.OpenTestTransportTests.tearDown
    ready = open_tests.OpenTestTransportTests.ready
    wait = open_tests.OpenTestTransportTests.wait
    run_probe = open_tests.OpenTestTransportTests.run_probe

    def setUp(self):
        self.port = ModernProbePort()
        self.c = RobotController(serial_factory=lambda *a, **kw: self.port)
        self.events, self.logs = [], []
        self.c.cartesian.stream.open_test_changed.connect(self.events.append)
        self.c.log_message.connect(lambda message, tag: self.logs.append(message))
        self.guards = [patch('app.core.controller.serial.Serial', side_effect=AssertionError('hardware forbidden')),
                       patch('app.core.stream_session.PlanningProcess', side_effect=AssertionError('IK forbidden'))]
        for guard in self.guards:
            guard.start()

    def test_modern_open_queries_evidence_and_changes_bit12_meaning(self):
        self.port.reject = 9
        result = self.run_probe(4)
        self.assertTrue(result['steady_supported'])
        self.assertIn(45, result['snapshot'])
        self.assertIn('新鲜 50 ms 稳态证据不足或过期', format_open_test(result))
        self.assertNotIn('瞬时估计速度超出 ±512', format_open_test(result))
        self.assertIn('位置范围 80（≤142', format_open_test(result))

    def test_modern_feature_stale_token_cannot_reinterpret_old_snapshot(self):
        self.port.feature_token_offset = -1
        self.port.reject = 9
        result = self.run_probe(4)
        self.assertFalse(result['steady_supported'])
        self.assertIn('瞬时估计速度超出 ±512', format_open_test(result))
        self.assertFalse(any(int(command.split()[2]) >= 34 for command in self.port.commands()
                             if command.startswith('#STREAM_DIAG ')))


class EntryUiAndProtocolTests(unittest.TestCase):
    def test_button_targets_and_click_audit_follow_independent_selections(self):
        ui = TerminalDiagnostics()
        opened, curves, logs = [], [], []
        ui.open_test_requested.connect(opened.append)
        ui.axis_test_requested.connect(curves.append)
        ui.test_clicked.connect(logs.append)
        ui.set_state(connected=True, locked=False, calibrating=False, connection_type='USB', disabled=False)
        ui.node.setCurrentIndex(3)
        ui.axis_test_selector.setCurrentIndex(0)
        self.assertTrue(ui.open_test_button.text().startswith('J4'))
        self.assertTrue(ui.axis_test_button.text().startswith('J1 ·'))
        ui.open_test_button.click(); ui.axis_test_button.click()
        self.assertEqual(opened, [4]); self.assertEqual(curves, [1])
        self.assertIn('请求节点=4', logs[0]); self.assertIn('mask=1', logs[1])
        ui.axis_test_selector.setCurrentIndex(8)
        self.assertIn('J1～J7（七轴）', ui.axis_test_button.text())
        self.assertEqual(ui.node.currentData(), 4)
        ui.close(); ui.deleteLater(); APPLICATION.processEvents()

    def test_entry_protocol_bounds_and_signed_home(self):
        self.assertEqual(parse_stream_line('STREAM_FEATURES s=123 steady=8 start=8')['start'], 8)
        self.assertEqual(parse_stream_line('STREAM_NODE node=4 n=1 field=42 value=-8458')['value'], -8458)
        self.assertEqual(parse_stream_line('STREAM_NODE node=4 n=1 field=48 value=4294967295')['value'], 0xffffffff)
        for line in ('STREAM_FEATURES s=65536 steady=8 start=8', 'STREAM_FEATURES s=123 steady=128 start=8',
                     'STREAM_NODE node=4 n=1 field=51 value=0', 'STREAM_NODE node=4 n=1 field=48 value=-1'):
            with self.assertRaises(ValueError):
                parse_stream_line(line)
        self.assertFalse(feature_supported(dict(session=0, steady=8), 0, 4, 'steady'))


if __name__ == '__main__':
    unittest.main()
