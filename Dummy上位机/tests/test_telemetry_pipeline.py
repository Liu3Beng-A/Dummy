"""Independent read polling must remain bounded and yield to control commands."""
import queue
import time
import unittest
from unittest.mock import patch
from PySide6.QtTest import QTest
from tests.test_ui import APPLICATION
from tests.stream_memory_port import StreamMemoryPort
from app.transport.serial_transport import SerialTransport
from app.transport.telemetry import JointPollSchedule, TelemetryMetrics
from app.core.controller import RobotController


class TelemetryUnitTests(unittest.TestCase):
    def test_schedule_single_flight_no_gui_wait_and_no_catch_up_burst(self):
        now=[10.]
        schedule=JointPollSchedule(clock=lambda:now[0])
        schedule.configure(True,50,7)
        self.assertEqual(schedule.take_due(),7)
        self.assertIsNone(schedule.written_at())
        schedule.written()
        self.assertIsNone(schedule.take_due())
        now[0]+=.005
        self.assertAlmostEqual(schedule.receive(now[0]),5.)
        self.assertIsNone(schedule.take_due())
        now[0]+=.020
        self.assertEqual(schedule.take_due(),7)
        schedule.written()
        now[0]+=5
        self.assertEqual(schedule.take_due(),7)
        self.assertEqual(schedule.timeouts,1)
        self.assertIsNone(schedule.take_due())

    def test_metrics_measure_host_rates_and_latency_and_expire_during_silence(self):
        now=[10.]
        metrics=TelemetryMetrics(clock=lambda:now[0])
        for index in range(101):
            now[0]=10.+index*.02
            metrics.received(now[0],5.)
            now[0]+=.004
            metrics.delivered(now[0]-.004)
        result=metrics.snapshot()
        self.assertAlmostEqual(result['rx_hz'],50,delta=.2)
        self.assertAlmostEqual(result['latency_ms'],4.)
        self.assertEqual(result['rtt_ms'],5.)
        now[0]+=3
        self.assertEqual(metrics.snapshot()['rx_hz'],0.)
        self.assertIsNone(metrics.snapshot()['latency_ms'])


class TelemetryViewTests(unittest.TestCase):
    def test_statistics_bridge_does_not_refresh_pose_camera_or_rgb(self):
        from copy import deepcopy
        from app.visualization.robot_view import RobotView
        with patch('app.visualization.robot_view.AssetServer', side_effect=RuntimeError('offline test')):
            view = RobotView()
        poses, stats, rgb = [], [], []
        view._bridge.state_changed.connect(poses.append)
        view._bridge.telemetry_changed.connect(stats.append)
        view._bridge.rgb_changed.connect(rgb.append)
        try:
            view.set_performance_metrics({'connected': True, 'rx_hz': 49.5})
            self.assertEqual(stats, [])
            view._on_ready()
            self.assertIn('49.5', stats[-1])
            before = deepcopy(view._state)
            poses.clear(); rgb.clear()
            view.set_performance_metrics({'connected': True, 'rx_hz': 50.})
            self.assertEqual(view._state, before)
            self.assertEqual((poses, rgb), ([], []))
            view._bridge.viewerPerformance('{"fps":37.5,"frame_ms_p95":30}')
            self.assertEqual(view._render_performance['fps'], 37.5)
            for invalid in ('[]', '{"fps":true}', '{"fps":-1}', '{"fps":NaN}', 'broken'):
                view._bridge.viewerPerformance(invalid)
                self.assertEqual(view._render_performance['fps'], 37.5)
        finally:
            view.close()
            view.deleteLater()
            APPLICATION.processEvents()


class TelemetryTransportTests(unittest.TestCase):
    def setUp(self):
        self.port=StreamMemoryPort()
        self.out=queue.Queue()
        self.transport=SerialTransport(self.port,self.out,lambda *_:True,1)
        self.transport.telemetry_allowed=lambda generation:generation==1
        self.received=[]
        self.transport.telemetry_received.connect(lambda line,*_:self.received.append(line))
        self.transport.configure_joint_polling(True,50,1)

    def tearDown(self):
        self.transport.close()
        QTest.qWait(10)

    def test_polling_continues_while_gui_stalls_and_only_latest_pose_is_delivered(self):
        self.transport.start()
        time.sleep(.5)  # Intentionally do not process any Qt events.
        count=self.port.commands().count('#GETJPOS')
        self.assertGreaterEqual(count,15,self.port.commands())
        self.assertLessEqual(count,28)
        self.assertEqual(self.received,[])
        self.assertGreater(self.transport.metrics.snapshot()['coalesced'],10)
        self.transport.configure_joint_polling(False,50,1)
        time.sleep(.03)
        QTest.qWait(40)
        self.assertEqual(len(self.received),1)
        self.assertEqual(self.out.unfinished_tasks,0)

    def test_missing_reply_is_single_flight_and_does_not_block_stop(self):
        self.port.drop_feedback=True
        self.transport.start()
        time.sleep(.06)
        self.out.put((1,'!STOP'))
        time.sleep(.06)
        self.assertEqual(self.port.commands().count('#GETJPOS'),1)
        self.assertIn('!STOP',self.port.commands())

    def test_manual_pose_query_yields_to_stop_while_background_reply_is_missing(self):
        self.port.drop_feedback=True
        self.transport.start()
        time.sleep(.06)
        self.out.put((1,'#GETJPOS'))
        self.out.put((1,'!STOP'))
        time.sleep(.06)
        self.assertIn('!STOP',self.port.commands())
        self.assertEqual(self.port.commands().count('#GETJPOS'),1)

    def test_writer_rechecks_gate_generation_and_flash_readiness(self):
        self.transport.telemetry_allowed=lambda _:False
        self.transport.start()
        time.sleep(.06)
        self.assertEqual(self.port.commands(),[])
        self.transport.telemetry_allowed=lambda generation:generation==2
        self.transport.configure_joint_polling(True,50,1)
        time.sleep(.04)
        self.assertEqual(self.port.commands(),[])
        self.transport.configure_joint_polling(True,50,2)
        self.transport.write_ready=lambda *_:False
        time.sleep(.06)
        self.assertEqual(self.port.commands(),[])
        self.transport.write_ready=lambda *_:True
        time.sleep(.08)
        self.assertGreater(len(self.port.commands()),1)

    def test_adopted_manual_sync_is_reliable_even_before_gui_flush(self):
        self.port.drop_feedback=True
        direct=[]
        self.transport.line_received_at.connect(lambda line,*_:direct.append(line))
        self.transport.start()
        time.sleep(.05)
        self.assertIsNotNone(self.transport.adopt_joint_poll())
        self.transport.configure_joint_polling(False,50,1)
        self.port.inject('ok 3 10 80 0 90 0 0')
        time.sleep(.03)
        QTest.qWait(40)
        self.assertEqual(direct,['ok 3 10 80 0 90 0 0'])
        self.assertEqual(self.received,[])


class TelemetryControllerTests(unittest.TestCase):
    def setUp(self):
        self.port = StreamMemoryPort()
        self.guard = patch('app.core.controller.serial.Serial', side_effect=AssertionError('real serial forbidden'))
        self.guard.start()
        self.c = RobotController(serial_factory=lambda *a, **kw: self.port)
        self.c.set_polling(True, 50)
        self.assertTrue(self.c.connect_port('MEMORY_TELEMETRY_ONLY', 'USB'))
        self.wait_for(lambda: self.c.actual_pose is not None and self.c._mode_request is None)

    def tearDown(self):
        self.c.close()
        self.port.close()
        APPLICATION.processEvents()
        self.guard.stop()

    def wait_for(self, predicate):
        deadline = time.perf_counter() + 2
        while not predicate() and time.perf_counter() < deadline:
            QTest.qWait(5)
        self.assertTrue(predicate(), self.port.commands())

    def test_gui_stall_keeps_queries_running_without_motion_or_admission_backlog(self):
        c = self.c
        target = [entry.get() for entry in c.ent_joints]
        before = self.port.commands().count('#GETJPOS')
        with self.port.lock:
            self.port.joints[0] = 4.
        time.sleep(.35)  # Deliberately do not pump Qt events.
        self.assertGreaterEqual(self.port.commands().count('#GETJPOS') - before, 10)
        self.assertTrue(c._wire_admitted.empty())
        self.wait_for(lambda: c.actual_pose[0] == 4.)
        self.assertEqual([entry.get() for entry in c.ent_joints], target)
        self.assertGreater(c.performance_snapshot()['coalesced'], 5)
        self.assertTrue(all(command.startswith('#') for command in self.port.commands()))

    def test_cached_background_pose_cannot_complete_manual_sync(self):
        c = self.c
        c.set_polling(False, 50)
        QTest.qWait(40)
        self.port.drop_feedback = True
        self.assertTrue(c.read_and_sync_from_robot())
        self.wait_for(lambda: c._position_request.get('written_at') is not None)
        request = c._position_request
        target = [entry.get() for entry in c.ent_joints]
        c._on_telemetry_line('ok 2 10 80 0 90 0 0', c._connection_id, time.perf_counter())
        self.assertIs(c._position_request, request)
        self.assertEqual([entry.get() for entry in c.ent_joints], target)
        self.port.inject('ok 4 10 80 0 90 0 0')
        self.wait_for(lambda: c._position_request is None)
        self.assertEqual(float(c.ent_joints[0].get()), 4.)

    def test_manual_sync_adopts_only_an_outstanding_written_poll(self):
        c = self.c
        self.port.drop_feedback = True
        self.wait_for(lambda: c._transport.joint_poll.written_at() is not None)
        before = self.port.commands().count('#GETJPOS')
        self.assertTrue(c.read_and_sync_from_robot())
        self.assertIsNotNone(c._position_request['written_at'])
        QTest.qWait(30)
        self.assertEqual(self.port.commands().count('#GETJPOS'), before)
        self.port.inject('ok 6 10 80 0 90 0 0')
        self.wait_for(lambda: c._position_request is None)
        self.assertEqual(float(c.ent_joints[0].get()), 6.)

    def test_poll_admission_preserves_pause_generation_and_uart_gates(self):
        c = self.c
        self.assertTrue(c._can_write_telemetry(c._tx_generation))
        self.assertFalse(c._can_write_telemetry(c._tx_generation - 1))
        for name, value in [('_stop_locked', True), ('_calibration_busy', True),
                            ('_acc_sync_waiting', True), ('_parameter_request', {}),
                            ('_position_request', {})]:
            previous = getattr(c, name)
            setattr(c, name, value)
            self.assertFalse(c._can_write_telemetry(c._tx_generation), name)
            setattr(c, name, previous)
        c.connection_type = 'UART4'
        c._motor_state_known, c._motor_enabled = True, False
        self.assertFalse(c._can_write_telemetry(c._tx_generation))
        c.connection_type = 'USB'
