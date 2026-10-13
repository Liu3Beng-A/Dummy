"""Real host TX/RX workers against an OPEN-only memory device; no hardware."""
import re
import time
import unittest
from unittest.mock import patch
from PySide6.QtTest import QTest
from app.core.controller import RobotController
from app.core.open_test import format_open_test, OPEN_BLOCKS
from app.ui.terminal_diagnostics import TerminalDiagnostics
from dummy_motion.stream_protocol import parse_stream_line
from tests.test_stream_transport import StreamMemoryPort
from tests.test_ui import APPLICATION


class ProbePort(StreamMemoryPort):
    def __init__(self):
        super().__init__()
        self.probe_session = 0
        self.probe_node = 0
        self.probe_state = 'IDLE'
        self.reject = 0
        self.close_failed = False
        self.unknown = False
        self.stale_snapshot = False
        self.changing_snapshot = False
        self.snapshot_reads = 0
        self.hold = False

    def probe_reply(self):
        opened = self.probe_session != 0 and self.reject == 0
        closed = self.probe_state == 'DONE'
        code = self.reject or (17 if self.close_failed else 0)
        return f'STREAM_TEST s={self.probe_session} node={self.probe_node} st={self.probe_state} open={int(opened)} closed={int(closed)} code={code}'

    def write(self, data):
        command = data.decode('ascii').strip()
        if not (command.startswith(('!STREAM_TEST ', '#STREAM_DIAG ')) or command == '#STREAM_TEST'):
            return super().write(data)
        with self.lock:
            self.writes.append(command)
            if self.unknown:
                self.inject('error STREAM syntax')
            elif command == '#STREAM_TEST':
                self.inject(self.probe_reply())
            elif command.startswith('!STREAM_TEST '):
                self.probe_session, self.probe_node = map(int, command.split()[1:])
                self.probe_state = 'OPENING' if self.hold else 'FAULT' if self.close_failed else 'DONE'
                self.inject(self.probe_reply())
            else:
                node, field = map(int, command.split()[1:])
                self.diag_nonce += 1
                if field == 23:
                    self.snapshot_reads += 1
                snapshot = {23: self.probe_session + int(self.stale_snapshot or (self.changing_snapshot and self.snapshot_reads > 1)),
                            24: (1 << 12) if self.reject == 9 else 0, 25: 0x101,
                            26: 513 if self.reject == 9 else 0, 27: 0,
                            28: -123, 29: -123, 30: -120, 31: -1000, 32: 100000, 33: 1}
                self.inject(f'STREAM_NODE node={node} n={self.diag_nonce} field={field} value={snapshot[field]}')
        return len(data)


class OpenTestTransportTests(unittest.TestCase):
    def setUp(self):
        self.port = ProbePort()
        self.c = RobotController(serial_factory=lambda *a, **kw: self.port)
        self.events, self.logs = [], []
        self.c.cartesian.stream.open_test_changed.connect(self.events.append)
        self.c.log_message.connect(lambda msg, tag: self.logs.append(msg))
        self.guards = [patch('app.core.controller.serial.Serial', side_effect=AssertionError('hardware forbidden')),
                       patch('app.core.stream_session.PlanningProcess', side_effect=AssertionError('IK forbidden'))]
        for guard in self.guards:
            guard.start()

    def tearDown(self):
        self.c.close(); self.port.close(); APPLICATION.processEvents()
        for guard in self.guards:
            guard.stop()

    def wait(self, predicate, seconds=4.):
        end = time.perf_counter()+seconds
        while not predicate() and time.perf_counter() < end:
            QTest.qWait(5); APPLICATION.processEvents()
        self.assertTrue(predicate(), (self.events[-2:], self.logs[-3:], self.port.commands()[-12:]))

    def ready(self):
        self.c.set_polling(False, 30)  # Handshake must not depend on JPOS or GUI clocks.
        self.assertTrue(self.c.connect_port('MEMORY_OPEN_ONLY', 'USB'))
        self.wait(lambda: self.c._mode_request is None)
        self.assertEqual(self.c.cartesian.backend.value, 'firmware')

    def run_probe(self, node=4):
        self.ready()
        self.assertTrue(self.c.cartesian.stream.start_open_test(node))
        self.wait(lambda: self.events and self.events[-1].get('done'))
        self.assertFalse(self.c.cartesian.busy)
        return self.events[-1]

    def assert_no_motion(self):
        for cmd in self.port.commands():
            self.assertFalse(cmd.startswith(('>', '@', '~S', '!STREAM_OPEN', '!STREAM_START', '#CMDMODE')) or
                             cmd in ('!START', '!HOME', '!RESET', '!STOP'), cmd)
        self.assertEqual(self.port.mode, 2)

    def test_success_in_default_mode_without_ik_or_joint_poll(self):
        result = self.run_probe()
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['snapshot'][30], -120)
        self.assertEqual(result['snapshot'][23], result['session'])
        self.assertEqual(self.port.snapshot_reads, 2)
        self.assert_no_motion()
        self.assertFalse(any(x == '#GETJPOS' for x in self.port.commands()))
        self.assertEqual(sum(x.startswith('!STREAM_TEST ') for x in self.port.commands()), 1)

    def test_blocked_speed_is_named_and_close_confirmed(self):
        self.port.reject = 9
        result = self.run_probe()
        self.assertFalse(result['passed']); self.assertTrue(result['closed'])
        text = format_open_test(result)
        self.assertIn('瞬时估计速度超出 ±512', text)
        self.assertIn('瞬时速度 513', text)
        self.assertIn('到位标志 1', text)
        self.assert_no_motion()

    def test_close_timeout_not_pass_and_new_open_not_sent(self):
        self.port.close_failed = True
        result = self.run_probe()
        self.assertTrue(result['opened']); self.assertFalse(result['closed']); self.assertFalse(result['passed'])
        self.wait(lambda: not self.c.cartesian.stream.thread.is_alive())
        self.assertTrue(self.c.cartesian.stream.start_open_test(4))
        self.wait(lambda: self.events[-1].get('done'))
        self.assertIn('尚未确认关闭', self.events[-1]['message'])
        self.assertEqual(sum(x.startswith('!STREAM_TEST ') for x in self.port.commands()), 1)
        self.assert_no_motion()

    def test_old_core_rejected_before_open(self):
        self.port.unknown = True
        result = self.run_probe()
        self.assertFalse(result['passed'])
        self.assertFalse(any(x.startswith('!STREAM_TEST') for x in self.port.commands()))
        self.assert_no_motion()

    def test_old_motor_capability_error_not_stale_snapshot(self):
        self.port.reject = 13
        result = self.run_probe()
        self.assertFalse(result['passed']); self.assertNotIn('snapshot', result)
        self.assertFalse(any(x.startswith('#STREAM_DIAG') for x in self.port.commands()))
        self.assert_no_motion()

    def test_stale_snapshot_rejected(self):
        self.port.stale_snapshot = True
        result = self.run_probe()
        self.assertFalse(result['passed']); self.assertNotIn('snapshot', result)
        self.assertIn('不匹配', result['message']); self.assert_no_motion()

    def test_snapshot_change_during_read_rejected(self):
        self.port.changing_snapshot = True
        result = self.run_probe()
        self.assertFalse(result['passed']); self.assertIn('已变化', result['message']); self.assert_no_motion()

    def test_busy_motor_never_sent_open(self):
        self.port.busy = True
        result = self.run_probe()
        self.assertFalse(result['passed']); self.assertIn('运动忙', result['message']); self.assert_no_motion()

    def test_disconnect_during_open_never_reports_success(self):
        self.ready(); self.port.hold = True
        self.assertTrue(self.c.cartesian.stream.start_open_test(4))
        self.wait(lambda: any(x.startswith('!STREAM_TEST') for x in self.port.commands()))
        self.c.disconnect_port()
        self.wait(lambda: self.events[-1].get('done'))
        self.assertFalse(self.events[-1]['passed']); self.assertEqual(self.events[-1]['stage'], '已中断')
        self.assert_no_motion()

    def test_worker_allowlist_and_ui_conflict_gates(self):
        self.ready(); self.port.hold = True
        self.assertTrue(self.c.cartesian.stream.start_open_test(4))
        worker = self.c.cartesian.stream
        for command in ('!START', '#CMDMODE 7', '!STREAM_START 1', '~S123', '>0,0,0,0,0,0,0,5', '@0,0,0,0,0,0,5'):
            with self.assertRaises(ValueError):
                worker._send(command)
            self.assertFalse(self.c.send_cmd(command))
        self.assertFalse(worker.start_open_test(5))
        self.assert_no_motion()

    def test_node_and_lock_guards(self):
        self.ready()
        worker = self.c.cartesian.stream
        for node in (0, 8, True):
            self.assertFalse(worker.start_open_test(node))
        self.c._stop_locked = True
        self.assertFalse(worker.start_open_test(4))
        self.assert_no_motion()


class OpenTestViewTests(unittest.TestCase):
    def test_diagnostic_button_node_usb_busy_and_readable_snapshot(self):
        ui = TerminalDiagnostics(); selected = []
        ui.open_test_requested.connect(selected.append)
        def state(**kw):
            args = dict(connected=True, locked=False, calibrating=False, connection_type='USB', disabled=False)
            args.update(kw); ui.set_state(**args)
        state(); ui.node.setCurrentIndex(3); ui.open_test_button.click(); self.assertEqual(selected, [4])
        state(busy=True); self.assertFalse(ui.open_test_button.isEnabled())
        state(connection_type='UART4'); self.assertFalse(ui.open_test_button.isEnabled())
        state(); ui.node.setCurrentIndex(7); self.assertFalse(ui.open_test_button.isEnabled())
        ui.show_open_test(dict(node=4, stage='未通过', message='测试原因', opened=False, closed=True, code=9,
                              snapshot={23: 55, 24: (1 << 19)-1, 25: 0x101, 26: -513, 27: 0, 28: 0, 29: 0, 30: 0, 31: 0, 32: 55, 33: 1}))
        for label in OPEN_BLOCKS:
            self.assertIn(label, ui.open_test_output.toPlainText())
        self.assertTrue(ui.open_test_output.isReadOnly()); ui.close()

    def test_readonly_fields_and_usb_capability(self):
        from app.protocol.capabilities import diagnostic, supported
        self.assertTrue(diagnostic('#STREAM_TEST'))
        self.assertFalse(supported('#STREAM_TEST', 'UART4'))
        for field in range(23, 34):
            self.assertTrue(diagnostic(f'#STREAM_DIAG 4 {field}'))
        self.assertTrue(diagnostic('#STREAM_DIAG 4 50'))
        self.assertFalse(diagnostic('#STREAM_DIAG 4 51'))
        self.assertFalse(diagnostic('!STREAM_TEST 1 4'))

    def test_reply_bounds(self):
        item = parse_stream_line('STREAM_TEST s=55 node=4 st=CLOSING open=1 closed=0 code=0')
        self.assertEqual(item['code'], 0); self.assertEqual(item['session'], 55)
        for line in ('STREAM_TEST s=65536 node=4 st=DONE open=1 closed=1 code=0',
                     'STREAM_TEST s=2 node=8 st=DONE open=1 closed=1 code=0',
                     'STREAM_NODE node=1 n=1 field=51 value=0'):
            with self.assertRaises(ValueError):
                parse_stream_line(line)


if __name__ == '__main__':
    unittest.main()
