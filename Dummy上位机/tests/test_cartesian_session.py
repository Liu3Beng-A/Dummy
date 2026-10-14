"""Current public backend contract; ordinary MoveJ approximation was retired."""
import unittest
import time
import tempfile
from app.core.controller import RobotController
from app.storage.store import AppStore
from dummy_motion import FeedbackSample
from dummy_motion.frames import from_wire_joints
from tests.test_ui import APPLICATION
from tests.test_controller import ManualScheduler, FakePort


class CartesianSessionTests(unittest.TestCase):
    def setUp(self):
        self.c = RobotController(scheduler=ManualScheduler())
        self.s = self.c.cartesian

    def tearDown(self):
        self.c.close()

    def test_default_is_firmware_and_removed_backend_has_no_fallback(self):
        self.assertEqual(self.s.backend.value, 'firmware')
        self.assertFalse(self.s.switch('host'))
        self.assertEqual(self.s.backend.value, 'firmware')
        self.assertTrue(self.c._tx_queue.empty())

    def test_native_backend_cannot_run_pc_trajectory(self):
        self.assertFalse(hasattr(self.s, 'start_points'))
        self.assertFalse(hasattr(self.s.diagnostics, 'start_points'))
        self.assertFalse(self.s.active)
        self.assertTrue(self.c._tx_queue.empty())

    def test_reserved_and_diagnostic_modes_are_not_selectable_for_ordinary_motion(self):
        self.assertFalse(self.s.select_mode(5))
        self.assertFalse(self.s.select_mode(6))
        self.assertFalse(self.s.select_mode(7))
        self.assertFalse(self.s.switch('stream'))
        self.assertEqual(self.s.backend.value, 'firmware')
        self.assertTrue(self.s.select_mode(3))
        self.assertEqual(self.s.backend.value, 'firmware')
        self.assertTrue(self.c._tx_queue.empty())

    def test_old_host_setting_migrates_to_default(self):
        with tempfile.TemporaryDirectory() as directory:
            store = AppStore(directory)
            try:
                store.update_settings(cartesian_backend='host')
                self.assertEqual(store.settings['cartesian_backend'], 'firmware')
            finally:
                store.close()

    def test_same_read_batch_keeps_last_pose_without_fake_time_or_velocity(self):
        stamp = time.perf_counter()
        self.assertTrue(self.s.update_feedback([1,10,80,0,90,0,0], stamp))
        velocity = self.s.feedback.velocity.copy()
        self.assertTrue(self.s.update_feedback([2,10,80,0,90,0,0], stamp))
        self.assertEqual(self.s.feedback.sample.received_at, stamp)
        self.assertAlmostEqual(self.s.feedback.sample.q[0], from_wire_joints([2,10,80,0,90,0,0])[0])
        self.assertTrue((velocity == self.s.feedback.velocity).all())
        self.assertFalse(self.s.update_feedback([3,10,80,0,90,0,0], stamp-.01))

    def test_late_gui_delivery_never_renews_real_rx_age(self):
        self.c.is_connected=True
        self.c._process_rx_line('ok 0 10 80 0 90 0 0', time.perf_counter()-1.)
        self.assertIsNone(self.c.actual_pose)
        self.assertEqual(self.s.feedback.estimate(time.perf_counter())[0].value,'FEEDBACK_STALE')
        self.c.is_connected=False

    def test_nonfinite_telemetry_does_not_modify_actual_pose(self):
        for value in ('nan', 'inf', '-inf'):
            self.c._process_rx_line(f'ok {value} 10 80 0 90 0 0')
        self.assertIsNone(self.c.actual_pose)

    def test_measured_j3_above_limit_updates_display_without_a_planning_seed(self):
        actual=[]
        self.c.actual_pose_changed.connect(actual.append)
        values=[0., -75., 180.62, 0., .59, 0., 0.]
        self.c._process_rx_line('ok 0.00 -75.00 180.62 0.00 0.59 0.00 -0.00')
        self.assertEqual(self.c.actual_pose, values)
        self.assertEqual(actual, [values])
        self.assertIsNone(self.s.feedback.sample)
        self.assertEqual(self.s.feedback.rejection_reason, '七轴位置超出模型限位')
        self.assertFalse(self.c._motor_enabled)
        self.assertTrue(self.c._tx_queue.empty())

    def test_all_measured_axes_can_exceed_limits_without_display_clipping(self):
        values=[190., -80., 181., 280., 101., 185., 260.]
        self.c._process_rx_line('ok ' + ' '.join(map(str, values)))
        self.assertEqual(self.c.actual_pose, values)
        self.assertIsNone(self.s.feedback.sample)

    def test_display_orders_outside_limit_samples_independently_of_planning(self):
        self.s.clock=lambda: 100.
        self.c._process_rx_line('ok 0 -75 180 0 0 0 0', 99.8)
        self.c._process_rx_line('ok 0 -75 180.62 0 0 0 0', 99.9)
        last_at=self.c.last_actual_at
        self.c._process_rx_line('ok 0 -75 179 0 0 0 0', 99.85)
        self.assertEqual(self.c.actual_pose[2], 180.62)
        self.assertEqual(self.c.last_actual_at, last_at)
        self.assertEqual(self.s.feedback.sample.received_at, 99.8)
        # The last line in one ordered read batch may share its RX timestamp.
        self.c._process_rx_line('ok 0 -75 180.65 0 0 0 0', 99.9)
        self.assertEqual(self.c.actual_pose[2], 180.65)

    def test_invalid_age_never_refreshes_an_outside_limit_display(self):
        self.s.clock=lambda: 100.
        self.c._process_rx_line('ok 0 -75 180.62 0 0 0 0', 99.9)
        last_at=self.c.last_actual_at
        for stamp in (99., 101., float('nan'), float('inf')):
            self.c._process_rx_line('ok 0 -75 180.65 0 0 0 0', stamp)
            self.assertEqual(self.c.actual_pose[2], 180.62)
            self.assertEqual(self.c.last_actual_at, last_at)
        self.assertIsNone(self.s.feedback.sample)

    def test_reconnect_resets_display_receive_order(self):
        self.c._actual_received_at=100.
        self.c._serial_factory=lambda *_args, **_kwargs: FakePort()
        self.c.set_polling(False)
        self.assertTrue(self.c.connect_port('FAKE', 'USB'))
        self.assertIsNone(self.c._actual_received_at)
        self.s.clock=lambda: 50.
        self.c._process_rx_line('ok 0 -75 180.62 0 0 0 0', 49.9)
        self.assertEqual(self.c.actual_pose[2], 180.62)

    def test_joint_telemetry_does_not_publish_edit_fields(self):
        changes=[]
        self.c.fields_changed.connect(lambda: changes.append(True))
        self.c._process_rx_line('ok 0 10 80 0 90 0 0')
        self.assertEqual(changes, [])
        self.assertEqual(self.c.actual_pose, [0.,10.,80.,0.,90.,0.,0.])
