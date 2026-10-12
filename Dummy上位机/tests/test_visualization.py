"""Coordinate and offline resource integrity tests; no robot is connected."""

import hashlib
import json
import math
import re
import struct
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
from pathlib import Path

from app.visualization.mapping import (MODEL_Z_OFFSET_M, REST_POSE, cartesian_marker,
                                       firmware_to_urdf, normalize_pose)
from app.visualization.asset_server import resource_path

ROOT = Path(__file__).resolve().parents[1]


class MappingTests(unittest.TestCase):
    def test_sign_units_and_finger_endpoints(self):
        values = firmware_to_urdf([10, 20, 30, 40, 50, 60, 125], 0)
        self.assertAlmostEqual(values['Joint1'], math.radians(10))
        self.assertAlmostEqual(values['Joint4'], -math.radians(40))
        self.assertAlmostEqual(values['Joint5'], -math.radians(50))
        self.assertAlmostEqual(values['Joint6'], math.radians(60))
        self.assertEqual(values['rail'], 0.125)
        self.assertEqual(values['Hand_R'], 0.026)
        self.assertEqual(values['Hand_R'], values['Hand_L'])
        opened = firmware_to_urdf(REST_POSE, 100)
        self.assertEqual(opened['Hand_R'], 0)

    def test_feedback_does_not_clip_to_rounded_urdf_limits(self):
        actual = normalize_pose([190, -80, 180, 280, 101, 180, 260], preview=False)
        self.assertEqual(actual, [190, -80, 180, 280, 101, 180, 260])
        preview = normalize_pose(actual, preview=True)
        self.assertEqual(preview, [175, -75, 180, 270, 100, 180, 250])
        self.assertEqual(firmware_to_urdf(REST_POSE)['Joint3'], math.pi)

    def test_invalid_pose_is_rejected_before_renderer(self):
        for pose in ([], [0] * 6, [0] * 6 + [float('nan')], [0] * 6 + [float('inf')]):
            with self.assertRaises(ValueError):
                firmware_to_urdf(pose)
        with self.assertRaises(ValueError):
            firmware_to_urdf(REST_POSE, float('nan'))

    def test_marker_applies_base_and_rail_offset(self):
        marker = cartesian_marker([100, 200, 300, 0, 90, 180], 150)
        self.assertEqual(marker['position'][:2], [0.1, 0.35])
        self.assertAlmostEqual(marker['position'][2], 0.3 + MODEL_Z_OFFSET_M)
        self.assertEqual(marker['orientation'], [0, math.pi / 2, math.pi])


class OfflineResourceTests(unittest.TestCase):
    def test_meshes_and_urdf_joint_names(self):
        base = ROOT / 'resources/models/DummyRobot_URDF'
        urdf = ET.parse(base / 'urdf/DummyRobot_URDF.urdf').getroot()
        joints = {joint.attrib['name'] for joint in urdf.findall('joint')}
        self.assertEqual(joints, {'rail', *(f'Joint{i}' for i in range(1, 7)), 'Hand_R', 'Hand_L'})
        meshes = urdf.findall('link/visual/geometry/mesh')
        self.assertEqual(len(meshes), 10)
        triangles = 0
        for mesh in meshes:
            prefix = 'package://DummyRobot_URDF/'
            self.assertTrue(mesh.attrib['filename'].startswith(prefix))
            path = base / mesh.attrib['filename'][len(prefix):]
            data = path.read_bytes()
            count = struct.unpack_from('<I', data, 80)[0]
            self.assertEqual(len(data), 84 + count * 50, path.name)
            triangles += count
        # Baseline 2bbf67a contains the user's replacement high-detail meshes.
        self.assertEqual(triangles, 1934314)

    def test_vendor_files_match_locked_digest(self):
        root = ROOT / 'resources/viewer'
        manifest = json.loads((root / 'vendor/manifest.json').read_text(encoding='utf-8'))
        for item in manifest['files']:
            path = root / 'vendor' / item['path']
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), item['sha256'], item['path'])
        self.assertIn('MIT License', (root / 'vendor/three/LICENSE').read_text(encoding='utf-8'))
        self.assertIn('Apache', (root / 'vendor/urdf-loader/LICENSE').read_text(encoding='utf-8'))

    def test_module_import_dependencies_are_all_bundled(self):
        root = ROOT / 'resources/viewer'
        html = (root / 'index.html').read_text(encoding='utf-8')
        import_map = json.loads(re.search(r'<script type="importmap">\s*(.*?)\s*</script>',
                                         html, re.S).group(1))['imports']
        for source in [root / 'viewer.js', *sorted((root / 'vendor').rglob('*.js'))]:
            imports = re.findall(r"\b(?:from\s+|import\s*)['\"]([^'\"]+)['\"]",
                                 source.read_text(encoding='utf-8'))
            for name in imports:
                if name.startswith('.'):
                    target = source.parent / name
                else:
                    self.assertIn(name, import_map, f'{source.name}: {name}')
                    target = root / import_map[name]
                self.assertTrue(target.is_file(), f'{source.name}: missing {target}')
                self.assertTrue(target.resolve().is_relative_to(root.resolve()))

    def test_server_cannot_serve_code_traversal_or_wrong_token(self):
        resources = ROOT / 'resources'
        self.assertEqual(resource_path(resources, '/abc/viewer/index.html', 'abc'),
                         (resources / 'viewer/index.html').resolve())
        bad_paths = ['/wrong/viewer/index.html', '/abc/viewer/../../main.py',
                     '/abc/viewer/%2e%2e/%2e%2e/main.py', '/abc/viewer',
                     '/abc/viewer/', '/abc/app/main.py', '/abc/viewer/..\\main.py']
        for path in bad_paths:
            self.assertIsNone(resource_path(resources, path, 'abc'), path)


class ActualOnlyViewTests(unittest.TestCase):
    def test_commanded_gripper_changes_fingers_without_refreshing_joint_feedback_or_interface_preview(self):
        from PySide6.QtWidgets import QApplication
        application = QApplication.instance() or QApplication([])
        from app.visualization.robot_view import RobotView
        with patch('app.visualization.robot_view.AssetServer', side_effect=RuntimeError('offline test')):
            view = RobotView()
        try:
            actual = [2, 9, 78, 3, 87, -2, 0]
            view.set_target_pose(REST_POSE, 80)
            target = dict(view._state['target'])
            view.set_actual_pose(actual)
            view.set_actual_only(True)
            view.set_stale(True)
            view.set_feedback_paused(True)
            for opening in (100, 40, 0):
                view.set_commanded_gripper(opening)
                self.assertEqual(view._state['actual'], firmware_to_urdf(actual, opening))
                self.assertEqual(view._state['commandedGripper'], opening)
                self.assertEqual(view._state['target'], target)
                self.assertTrue(view._state['stale'])
                self.assertTrue(view._state['feedbackPaused'])
                self.assertFalse(view._state['actualGripperKnown'])
            actual[0] = 7
            view.set_actual_pose(actual)
            self.assertEqual(view._state['actual'], firmware_to_urdf(actual, 0))
            with self.assertRaises(ValueError):
                view.set_commanded_gripper(float('nan'))
            self.assertEqual(view._state['commandedGripper'], 0)
            view.set_actual_only(False)
            self.assertIsNone(view._state['commandedGripper'])
            self.assertEqual(view._state['actual'], firmware_to_urdf(actual, 80))
            view.set_actual_only(True)
            self.assertEqual(view._state['actual'], firmware_to_urdf(actual, 0))
            view.set_commanded_gripper(None)
            self.assertEqual(view._state['actual'], firmware_to_urdf(actual, 100))
            self.assertIsNone(view._state['commandedGripper'])
        finally:
            view.close()
            view.deleteLater()
            application.processEvents()

    def test_world_marker_does_not_move_with_preview_rail(self):
        from PySide6.QtWidgets import QApplication
        application = QApplication.instance() or QApplication([])
        from app.visualization.robot_view import RobotView
        with patch('app.visualization.robot_view.AssetServer', side_effect=RuntimeError('offline test')):
            view = RobotView()
        try:
            view.set_world_cartesian_target([100, 200, 300, 0, 0, 0])
            before = view._state['marker']
            view.set_target_pose([0, 10, 80, 0, 90, 0, 100])
            self.assertEqual(view._state['marker'], before)
            view.set_cartesian_target([100, 200, 300, 0, 0, 0])
            self.assertAlmostEqual(view._state['marker']['position'][1], .3)
        finally:
            view.close()
            view.deleteLater()
            application.processEvents()

    def test_actual_only_hides_targets_and_restores_display_preference(self):
        from PySide6.QtCore import QCoreApplication, Qt
        from PySide6.QtWidgets import QApplication
        QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
        application = QApplication.instance() or QApplication([])
        from app.visualization.robot_view import RobotView
        # Exercise the real widget state API without starting HTTP or WebEngine.
        with patch('app.visualization.robot_view.AssetServer', side_effect=RuntimeError('offline test')):
            view = RobotView()
        try:
            view.set_target_pose([10, -45, 90, 0, 10, 20, 15], 0)
            view.set_cartesian_target([100, 20, 300, 0, 10, 20])
            self.assertIsNotNone(view._state['marker'])
            view.set_actual_only(True)
            view.set_feedback_paused(True)
            self.assertEqual(view._state['mode'], 'actual')
            self.assertTrue(view._state['feedbackPaused'])
            self.assertIsNone(view._state['marker'])
            self.assertFalse(view._state['actualKnown'])
            actual = [2, 9, 78, 3, 87, -2, 0]
            view.set_actual_pose(actual)
            self.assertTrue(view._state['actualKnown'])
            self.assertFalse(view._state['actualGripperKnown'])
            self.assertEqual(view._state['actual'], firmware_to_urdf(actual, 100))
            view.set_target_gripper(50)
            view.set_target_pose([20, -40, 100, 10, 20, 30, 0], 20)
            self.assertEqual(view._state['actual'], firmware_to_urdf(actual, 100))
            view.set_stale(True)
            self.assertTrue(view._state['stale'])
            for preference in ('preview', 'actual'):
                view.set_display_mode(preference)
                self.assertEqual(view._state['mode'], 'actual')
                view.set_actual_only(False)
                self.assertEqual(view._state['mode'], preference)
                self.assertIsNotNone(view._state['marker'])
                view.set_actual_only(True)
            view.set_actual_known(False)
            self.assertFalse(view._state['actualKnown'])
            view.set_feedback_paused(False)
            self.assertFalse(view._state['feedbackPaused'])
        finally:
            view.close()
            view.deleteLater()
            application.processEvents()


if __name__ == '__main__':
    unittest.main()
