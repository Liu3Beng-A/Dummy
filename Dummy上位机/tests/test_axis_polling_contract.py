"""One selected-axis sampler using real Controller/RX/TX memory transport."""
import threading
import time
import unittest
from unittest.mock import patch

from PySide6.QtTest import QTest
from app.core.controller import RobotController
from tests.test_axis_communication_test import AxisMemoryPort
from tests.test_ui import APPLICATION


class PollTracePort(AxisMemoryPort):
    def __init__(self):
        self.controller = None
        self.trace_lock = threading.Lock()
        self.wire_writes = []
        super().__init__()

    def write(self, data):
        command = data.decode('ascii').strip()
        c = self.controller
        with self.trace_lock:
            self.wire_writes.append(dict(stamp=time.perf_counter(), command=command,
                active=c.cartesian.active if c else False,
                kind=c.cartesian.kind if c else None))
        return super().write(data)

    def actual_writes(self):
        with self.trace_lock:
            return tuple(self.wire_writes)


class AxisPollingContractTests(unittest.TestCase):
    def setUp(self):
        self.port = PollTracePort()
        self.c = RobotController(serial_factory=lambda *a, **kw: self.port)
        self.port.controller = self.c
        self.results, self.gui_queries, self.worker_queries, self.telemetry_queries = [], [], [], []
        self.c.cartesian.stream.axis_test_changed.connect(self.results.append)
        original_send = self.c.send_cmd
        original_worker_send = self.c.cartesian.stream._send
        original_telemetry_allowed = self.c._can_write_telemetry
        def send(command, *args, **kwargs):
            if command == '#GETJPOS' and kwargs.get('source') == 'telemetry':
                self.gui_queries.append(dict(stamp=time.perf_counter(), active=self.c.cartesian.active,
                                             kind=self.c.cartesian.kind))
            return original_send(command, *args, **kwargs)
        def worker_send(command):
            if command == '#GETJPOS':
                self.worker_queries.append(dict(stamp=time.perf_counter(), kind=self.c.cartesian.kind))
            return original_worker_send(command)
        def telemetry_allowed(generation):
            allowed = original_telemetry_allowed(generation)
            if allowed:
                self.telemetry_queries.append(dict(stamp=time.perf_counter(), active=self.c.cartesian.active,
                                                   kind=self.c.cartesian.kind))
            return allowed
        self.guards = [patch('app.core.controller.serial.Serial', side_effect=AssertionError('hardware forbidden')),
                       patch('dummy_motion.ik.IKSolver.solve_pose', side_effect=AssertionError('IK forbidden')),
                       patch.object(self.c,'send_cmd',side_effect=send),
                       patch.object(self.c,'_can_write_telemetry',side_effect=telemetry_allowed),
                       patch.object(self.c.cartesian.stream,'_send',side_effect=worker_send)]
        for guard in self.guards:
            guard.start()

    def tearDown(self):
        self.c.close();self.port.close();APPLICATION.processEvents()
        for guard in reversed(self.guards):
            guard.stop()

    def wait(self, predicate, seconds=9.):
        deadline = time.perf_counter()+seconds
        while not predicate() and time.perf_counter()<deadline:
            QTest.qWait(5);APPLICATION.processEvents()
        self.assertTrue(predicate(), (self.results[-1:],self.port.commands()[-12:]))

    def ready(self):
        self.c.set_polling(True,30)
        self.assertTrue(self.c.connect_port('MEMORY_AXIS_POLLING','USB'))
        self.wait(lambda:self.c._mode_request is None)
        self.assertTrue(self.c.send_cmd('!START'))
        self.wait(lambda:self.c._motor_enabled and self.c._mode_request is None)
        # Stop only the GUI configuration heartbeat, then drain its old manual
        # ledger. Transport polling is independent and remains enabled.
        self.c._poll_timer.stop()
        self.wait(lambda:self.c._position_request is None and self.c._tx_queue.empty())

    def start_hold(self):
        self.assertTrue(self.c.cartesian.stream.start_axis_test(1))
        self.c._poll_timer.start(34)  # Same configured 30 Hz GUI timer, still running.

    def assert_one_sampler_during_axis(self):
        extra = [query for query in self.gui_queries if query['active'] and query['kind']=='axis_test']
        self.assertEqual(extra, [])
        self.assertFalse(any(query['active'] and query['kind']=='axis_test' for query in self.telemetry_queries))
        actual = [entry for entry in self.port.actual_writes()
                  if entry['command']=='#GETJPOS' and entry['active'] and entry['kind']=='axis_test']
        worker = [entry for entry in self.worker_queries if entry['kind']=='axis_test']
        self.assertTrue(worker and actual)
        # A final queued read can be discarded when ownership ends; an extra
        # GUI read must never increase the actual writer count over the owner.
        self.assertLessEqual(len(actual),len(worker))
        return actual

    def assert_gui_resumes(self):
        marker = time.perf_counter()
        self.wait(lambda:any(entry['stamp']>marker and entry['command']=='#GETJPOS' and not entry['active']
                             for entry in self.port.actual_writes()),1.)
        self.assertTrue(any(query['stamp']>marker and not query['active'] for query in self.telemetry_queries))
        self.assertEqual(self.gui_queries, [])  # GUI no longer dispatches automatic queries.

    def test_completed_hold_has_worker_feedback_and_no_duplicate_gui_poll(self):
        self.ready();self.start_hold()
        self.wait(lambda:self.results and self.results[-1].get('done'))
        result = self.results[-1]
        self.assertTrue(result['passed'] and result['closed'] and result['restored'],result)
        actual = self.assert_one_sampler_during_axis()
        self.assertGreater(len(actual),80)  # Live feedback remains active through the two-second hold.
        self.assertEqual(sum(command.startswith('~S') for command in self.port.commands()),101)
        self.assertFalse(any(command.startswith(('>','@')) for command in self.port.commands()))
        self.assert_gui_resumes()

    def test_preopen_failure_releases_sampler_and_gui_poll_resumes(self):
        self.port.axis_capability=False
        self.ready();self.start_hold()
        self.wait(lambda:self.results and self.results[-1].get('done'))
        self.assertFalse(self.results[-1]['passed'])
        self.assertFalse(any(command.startswith(('!STREAM_OPEN ','~S')) for command in self.port.commands()))
        self.assert_one_sampler_during_axis()
        self.assertFalse(self.c._stop_locked)
        self.assert_gui_resumes()

    def test_user_stop_keeps_polling_locked_until_explicit_start_then_resumes(self):
        self.ready();self.start_hold()
        self.wait(lambda:self.port.running_at is not None)  # Observe real START write, not queued flag.
        self.assertTrue(self.c.send_cmd('!STOP'))
        self.wait(lambda:self.results and self.results[-1].get('done'))
        result=self.results[-1]
        self.assertFalse(result['passed'])
        self.assertTrue(result['closed'] and result['restored'],result)
        self.assert_one_sampler_during_axis()
        marker=len(self.gui_queries)
        for _ in range(3):
            self.c._poll_position()
        self.assertEqual(len(self.gui_queries),marker)
        self.assertTrue(self.c._stop_locked)
        self.assertTrue(self.c.send_cmd('!START'))  # A simulated explicit user action only.
        self.wait(lambda:not self.c._stop_locked and self.c._motor_enabled and self.c._mode_request is None)
        self.assert_gui_resumes()

    def test_diagnostic_owner_suspends_gui_and_releases_on_cancel(self):
        self.ready()
        self.c.cartesian._begin('axis_test')
        self.c._poll_timer.start(34)
        marker=len(self.gui_queries)
        for _ in range(3):
            self.c._poll_position()
        QTest.qWait(110);APPLICATION.processEvents()
        self.assertEqual(len(self.gui_queries),marker)
        self.c.cartesian.cancel()
        self.assert_gui_resumes()

    def test_gui_alone_keeps_normal_telemetry_on_real_writer(self):
        self.ready()
        marker=time.perf_counter()
        self.c._poll_timer.start(34)
        self.wait(lambda:sum(entry['stamp']>marker and entry['command']=='#GETJPOS'
                            for entry in self.port.actual_writes())>=3,1.)
        self.assertFalse(self.worker_queries)
        self.assertGreaterEqual(sum(query['stamp']>marker for query in self.telemetry_queries),3)
        self.assertEqual(self.gui_queries, [])


if __name__ == '__main__':
    unittest.main()
