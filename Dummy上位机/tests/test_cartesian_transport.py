"""Offline transport correctness independent of Qt event delivery."""
import queue
import threading
import time
import unittest
from PySide6.QtTest import QTest
from app.transport.serial_transport import SerialTransport
from app.transport.wire_log import WireLog
from tests.test_ui import APPLICATION
from tests.test_stream_transport import StreamMemoryPort


class CartesianTransportTests(unittest.TestCase):
    def test_latest_batch_pose_survives_gui_stall_and_all_raw_lines_are_audited(self):
        port=StreamMemoryPort();queue_out=queue.Queue();seen=[];observed=[];audit=[]
        transport=SerialTransport(port,queue_out,lambda *_: True,1)
        transport.wire_log.sink=lambda tag,line:audit.append((tag,line))
        transport.line_observer=lambda line,*_:observed.append(line) or line.startswith('STREAM_')
        transport.line_received_at.connect(lambda line,*_:seen.append(line))
        transport.start()
        try:
            port.inject('ok 1 10 80 0 90 0 0','ok 2 10 80 0 90 0 0','STREAM_FAULT s=5 code=17')
            time.sleep(.1)  # Deliberately no Qt event processing.
            self.assertEqual(len(seen),0)
            self.assertIn('STREAM_FAULT s=5 code=17',observed)
            QTest.qWait(60)
            self.assertEqual([line for line in seen if line.startswith('ok ')],['ok 2 10 80 0 90 0 0'])
            self.assertTrue(any('STREAM_FAULT s=5 code=17' in line for _,line in audit))
            self.assertTrue(any('ok 1 10 80 0 90 0 0' in line for _,line in audit))
            self.assertTrue(any('ok 2 10 80 0 90 0 0' in line for _,line in audit))
        finally:transport.close()

    def test_written_observer_cannot_swallow_wire_audit(self):
        port=StreamMemoryPort();out=queue.Queue();transport=SerialTransport(port,out,lambda *_:True,1)
        transport.written_observer=lambda *_:True
        transport.start()
        try:
            out.put((1,'#STREAM'))
            time.sleep(.08)
            history=transport.wire_log.snapshot()
            self.assertIn('[WIRE TX]',history)
            self.assertIn('#STREAM',history)
            self.assertIn('rx_tx=',history)
        finally:transport.close()

    def test_bounded_log_overflow_is_explicit_and_does_not_block_rx(self):
        gate=threading.Event();entered=threading.Event()
        def slow(*_):entered.set();gate.wait(.5)
        audit=WireLog(slow,capacity=2)
        try:
            audit.record('RX','first',1.,1.);self.assertTrue(entered.wait(.5))
            started=time.perf_counter()
            for index in range(50):audit.record('RX',str(index),float(index),1.)
            self.assertLess(time.perf_counter()-started,.1)
            self.assertGreater(audit.dropped,0)
            self.assertEqual(len(audit.history),2)
        finally:gate.set();audit.close()
