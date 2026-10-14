"""Diagnostic/mode Qt terminals cannot release a still-running worker."""
import threading
import unittest
from contextlib import contextmanager
from unittest.mock import patch
from PySide6.QtTest import QTest
from app.core.cartesian_backend import CartesianBackend
from tests.test_stream_transport import NativeTransportFixture


class StreamTerminalLifecycleTests(NativeTransportFixture,unittest.TestCase):
    @contextmanager
    def hold_mode_worker(self):
        worker=self.c.cartesian.diagnostics;original=worker._notify
        reached,release=threading.Event(),threading.Event()
        def notify(kind,value):
            original(kind,value)
            if kind=='backend':
                reached.set();release.wait(3.)
        try:
            with patch.object(worker,'_notify',side_effect=notify):
                yield reached,release
        finally:release.set()

    def test_mode_result_waits_for_worker_exit_before_next_switch(self):
        self.ready();worker=self.c.cartesian.diagnostics
        with self.hold_mode_worker() as (reached,release):
            self.assertTrue(self.c.cartesian.select_mode(3))
            self.wait_for(reached.is_set);QTest.qWait(40)
            self.assertTrue(worker.thread.is_alive())
            self.assertTrue(self.c.cartesian.busy)
            self.assertEqual(self.c.cartesian.stage,'switching')
            self.assertFalse(self.c.cartesian.select_mode(2))
            release.set();self.wait_for(lambda:not self.c.cartesian.busy)
            self.assertFalse(worker.thread.is_alive())
        self.assertTrue(self.c.cartesian.select_mode(2))
        self.wait_for(lambda:not self.c.cartesian.busy)
        self.assertEqual(self.port.mode,2)

    def test_stop_invalidates_deferred_mode_result(self):
        self.ready();worker=self.c.cartesian.diagnostics
        with self.hold_mode_worker() as (reached,release):
            self.assertTrue(self.c.cartesian.select_mode(3))
            self.wait_for(reached.is_set);QTest.qWait(25)
            epoch=self.c.cartesian.epoch
            self.assertTrue(self.c.send_cmd('!STOP'))
            self.assertGreater(self.c.cartesian.epoch,epoch)
            release.set();self.wait_for(lambda:not worker.thread.is_alive());QTest.qWait(30)
            self.assertNotEqual(self.c.cartesian.stage,'switching')
            self.assertFalse(self.c.cartesian.busy)
            self.assertTrue(self.c._stop_locked)

    def test_disconnect_invalidates_deferred_mode_result(self):
        self.ready();worker=self.c.cartesian.diagnostics
        with self.hold_mode_worker() as (reached,release):
            self.assertTrue(self.c.cartesian.select_mode(3))
            self.wait_for(reached.is_set);QTest.qWait(25)
            epoch=self.c.cartesian.epoch;self.c.disconnect_port()
            self.assertGreater(self.c.cartesian.epoch,epoch)
            release.set();self.wait_for(lambda:not worker.thread.is_alive());QTest.qWait(30)
            self.assertFalse(self.c.is_connected)
            self.assertFalse(self.c.cartesian.busy)

    def test_each_diagnostic_terminal_waits_for_original_worker(self):
        self.ready();worker,owner=self.c.cartesian.diagnostics,self.c.cartesian
        values={'backend':(CartesianBackend.FIRMWARE,2),'fault':'forced diagnostic fault',
                'open_test_done':dict(node=1,stage='通过',message='test',done=True,passed=True),
                'axis_test_done':dict(mask=1,stage='通过',message='test',done=True,passed=True)}
        for kind,value in values.items():
            with self.subTest(kind=kind):
                release=threading.Event();thread=threading.Thread(target=lambda:release.wait(2.),daemon=True)
                owner._begin('open_test' if kind=='open_test_done' else 'axis_test' if kind=='axis_test_done' else 'mode_switch')
                owner._state('checking','terminal barrier');worker.thread=thread;thread.start()
                try:
                    worker._notification(owner.epoch,kind,value);QTest.qWait(20)
                    self.assertTrue(owner.busy);self.assertEqual(owner.stage,'checking')
                    release.set();thread.join(.5);self.wait_for(lambda:not owner.busy)
                finally:release.set();thread.join(.5)
