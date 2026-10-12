"""Backend selection and world-TCP UI semantics with a fake viewer/device."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from PySide6.QtCore import QEvent
from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow
from tests.test_ui import APPLICATION, FakeView
from tests.test_controller import ManualScheduler


class CartesianUiTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.store = AppStore(self.directory.name)
        self.c = RobotController(scheduler=ManualScheduler())
        self.viewer_patch = patch('app.ui.main_window.RobotView', FakeView)
        self.viewer_patch.start()
        self.w = MainWindow(self.c, self.store)

    def tearDown(self):
        self.w.close()
        self.w.deleteLater()
        APPLICATION.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        APPLICATION.processEvents()
        self.viewer_patch.stop()
        self.store.close()
        self.directory.cleanup()

    def test_default_firmware_switch_is_persisted_without_writes_or_target_changes(self):
        self.assertEqual(self.w.ik_backend.currentData(), 'firmware')
        before = [spin.value() for spin in self.w.pose_spins]
        self.w.ik_backend.setCurrentIndex(1)
        self.assertEqual(self.c.cartesian.backend.value, 'host')
        self.assertEqual(self.store.settings['cartesian_backend'], 'host')
        self.assertEqual([spin.value() for spin in self.w.pose_spins], before)
        self.assertTrue(self.c._tx_queue.empty())
        self.assertEqual(self.w.poll_hz.maximum(), 50)

    def test_world_point_roundtrip_retains_tool_and_legacy_point_resets_frame(self):
        self.w.teaching_tabs.setCurrentIndex(1)
        self.w.cartesian_frame.setCurrentIndex(1)
        point = self.w._current_point('world')
        self.assertEqual((point['frame'], point['target_kind']), ('world', 'tcp'))
        self.assertEqual(point['tool'][2], 114.)
        point['tool'] = [10., 5., 125., 0., 3., 0.]
        self.w._load_point_target(point)
        self.assertEqual(self.w._current_point('copy')['tool'], point['tool'])
        legacy = {'name': 'old', 'type': 'MoveL', 'pose': point['pose'], 'gripper': 100., 'speed': 35.}
        self.w._load_point_target(legacy)
        self.assertEqual(self.w.cartesian_frame.currentData(), 'base')
        self.assertNotIn('frame', self.w._current_point('old-copy'))
        self.assertTrue(self.c._tx_queue.empty())

    def test_disabling_drag_clears_host_pending_retarget(self):
        self.w.l_drag.setChecked(True)
        self.c.cartesian.retarget = (object(), 35)
        self.w.l_drag.setChecked(False)
        self.assertIsNone(self.c.cartesian.retarget)

    def test_world_read_does_not_trigger_enabled_drag(self):
        self.w.cartesian_frame.setCurrentIndex(1)
        self.w.l_drag.setChecked(True)
        self.c.cartesian.update_feedback([0, 10, 80, 0, 90, 0, 30])
        self.assertTrue(self.w._world_cartesian_read())
        self.assertAlmostEqual(self.w.pose_spins[1].value(), 30., places=2)
        self.assertAlmostEqual(self.w.pose_spins[2].value(), 174.92, places=2)
        self.assertEqual(self.w._drag_pending, {})
        self.assertTrue(self.c._tx_queue.empty())

    def test_frame_switch_preserves_world_target_and_tool_without_sending(self):
        import numpy as np
        from dummy_motion import Tool
        from dummy_motion.frames import from_xyzabc, error
        self.w.joint_rows[6].set_value(40.)
        values = [146.52, 15., 288.92, 180., 0., 180.]
        for row,value in zip(self.w.pose_rows,values): row.set_value(value)
        tool = Tool(from_xyzabc([10,5,125,0,3,0]))
        self.w._point_tool_snapshot = [10,5,125,0,3,0]
        expected = self.c.cartesian.model.legacy_target(from_xyzabc(values), .04, tool)
        self.w.cartesian_frame.setCurrentIndex(1)
        actual = from_xyzabc([spin.value() for spin in self.w.pose_spins])
        residual = error(actual,expected)
        self.assertLess(np.linalg.norm(residual[:3]), .00002)
        self.assertLess(np.linalg.norm(residual[3:]), .0003)
        self.w.cartesian_frame.setCurrentIndex(0)
        np.testing.assert_allclose([spin.value() for spin in self.w.pose_spins],values,atol=.02)
        self.assertEqual(self.w._point_tool_snapshot,[10,5,125,0,3,0])
        self.assertTrue(self.c._tx_queue.empty())

    def test_connected_frame_switch_requires_fresh_rail(self):
        self.c.is_connected = True
        before = [spin.value() for spin in self.w.pose_spins]
        self.w.cartesian_frame.setCurrentIndex(1)
        self.assertEqual(self.w.cartesian_frame.currentData(), 'base')
        self.assertEqual(self.w.teaching_frame.currentData(), 'base')
        self.assertEqual([spin.value() for spin in self.w.pose_spins],before)
        self.c.is_connected = False


if __name__ == '__main__':
    unittest.main()
