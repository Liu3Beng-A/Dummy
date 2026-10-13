"""Terminal Qt receipts may not release a still-running serial worker."""
import threading
import unittest
from contextlib import contextmanager
from unittest.mock import patch
from PySide6.QtTest import QTest
from app.core.cartesian_backend import CartesianBackend
from tests import test_stream_transport as transport_tests


class StreamTerminalLifecycleTests(unittest.TestCase):
    setUp = transport_tests.StreamTransportTests.setUp
    tearDown = transport_tests.StreamTransportTests.tearDown
    wait_for = transport_tests.StreamTransportTests.wait_for
    ready = transport_tests.StreamTransportTests.ready
    point = transport_tests.StreamTransportTests.point
    start = transport_tests.StreamTransportTests.start

    @contextmanager
    def hold_complete_worker(self):
        worker = self.c.cartesian.stream
        original = worker._notify
        reached, release = threading.Event(), threading.Event()
        def notify(kind, value):
            original(kind, value)
            if kind == 'complete':
                reached.set()
                release.wait(3.)
        try:
            with patch.object(worker, '_notify', side_effect=notify):
                yield reached, release
        finally:
            release.set()

    def test_completion_is_not_published_until_worker_exit_then_switch_succeeds(self):
        worker = self.c.cartesian.stream
        with self.hold_complete_worker() as (reached, release):
            self.start()
            self.wait_for(reached.is_set)
            QTest.qWait(40)
            self.assertTrue(worker.thread.is_alive())
            self.assertTrue(self.c.cartesian.busy)
            self.assertNotEqual(self.c.cartesian.stage, 'complete')
            self.assertFalse(self.c.set_cartesian_backend('firmware'))
            release.set()
            self.wait_for(lambda: self.c.cartesian.stage == 'complete')
            self.assertFalse(worker.thread.is_alive())
            self.assertFalse(self.c.cartesian.busy)
        self.assertTrue(self.c.set_cartesian_backend('firmware'))
        self.wait_for(lambda: self.c.cartesian.backend.value == 'firmware')
        self.assertEqual(self.port.mode, 2)

    def test_stop_invalidates_deferred_completion_epoch(self):
        worker = self.c.cartesian.stream
        with self.hold_complete_worker() as (reached, release):
            self.start()
            self.wait_for(reached.is_set)
            QTest.qWait(25)
            epoch = self.c.cartesian.epoch
            self.assertTrue(self.c.send_cmd('!STOP'))
            self.assertGreater(self.c.cartesian.epoch, epoch)
            release.set()
            self.wait_for(lambda: not worker.thread.is_alive())
            QTest.qWait(30)
            self.assertNotEqual(self.c.cartesian.stage, 'complete')
            self.assertFalse(self.c.cartesian.busy)

    def test_disconnect_invalidates_deferred_completion_epoch(self):
        worker = self.c.cartesian.stream
        with self.hold_complete_worker() as (reached, release):
            self.start()
            self.wait_for(reached.is_set)
            QTest.qWait(25)
            epoch = self.c.cartesian.epoch
            self.c.disconnect_port()
            self.assertGreater(self.c.cartesian.epoch, epoch)
            release.set()
            self.wait_for(lambda: not worker.thread.is_alive())
            QTest.qWait(30)
            self.assertNotEqual(self.c.cartesian.stage, 'complete')
            self.assertFalse(self.c.is_connected)
            self.assertFalse(self.c.cartesian.busy)

    def test_every_owner_releasing_terminal_waits_for_its_original_worker(self):
        self.ready()
        worker, owner = self.c.cartesian.stream, self.c.cartesian
        values = {
            'complete': None,
            'backend': (CartesianBackend.FIRMWARE, 2),
            'fault': 'forced terminal fault',
            'open_test_done': dict(node=1, stage='通过', message='test', done=True, passed=True),
            'axis_test_done': dict(mask=1, stage='通过', message='test', done=True, passed=True),
        }
        for kind, value in values.items():
            with self.subTest(kind=kind):
                release = threading.Event()
                thread = threading.Thread(target=lambda: release.wait(2.), daemon=True)
                owner._begin('open_test' if kind == 'open_test_done' else 'axis_test' if kind == 'axis_test_done' else 'stream')
                owner._state('checking', 'terminal barrier')
                worker.thread = thread
                thread.start()
                try:
                    worker._notification(owner.epoch, kind, value)
                    QTest.qWait(20)
                    self.assertTrue(owner.busy)
                    self.assertEqual(owner.stage, 'checking')
                    release.set()
                    thread.join(timeout=.5)
                    self.wait_for(lambda: not owner.busy)
                finally:
                    release.set()
                    thread.join(timeout=.5)


if __name__ == '__main__':
    unittest.main()
