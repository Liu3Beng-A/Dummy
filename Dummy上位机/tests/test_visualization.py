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

    def test_207_3mm_base_keeps_mathematical_marker_aligned_with_urdf(self):
        from app.gamepad.kinematics import forward_kinematics
        self.assertAlmostEqual(MODEL_Z_OFFSET_M, .0200)
        marker = cartesian_marker(forward_kinematics([0] * 6), 0)
        # The visual URDF remains 227.3mm; only its mathematical marker offset
        # changes, so the zero-pose flange stays at 627.3mm in display space.
        self.assertAlmostEqual(marker["position"][2], .6273)

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
        self.assertEqual(len(meshes), 11)
        triangles = 0
        for mesh in meshes:
            prefix = 'package://DummyRobot_URDF/'
            self.assertTrue(mesh.attrib['filename'].startswith(prefix))
            path = base / mesh.attrib['filename'][len(prefix):]
            data = path.read_bytes()
            count = struct.unpack_from('<I', data, 80)[0]
            self.assertEqual(len(data), 84 + count * 50, path.name)
            triangles += count
        # The split base/diffuser export adds ten interface facets; retain detail.
        self.assertEqual(triangles, 1934324)

    def test_diffuser_is_in_base_coordinates_and_requested_colors_are_applied(self):
        import numpy as np
        base = ROOT / 'resources/models/DummyRobot_URDF'
        urdf = ET.parse(base / 'urdf/DummyRobot_URDF.urdf').getroot()
        links = {link.get('name'): link for link in urdf.findall('link')}
        color = lambda name: tuple(float(value) for value in links[name].find('visual/material/color').get('rgba').split())
        for name in ('base_link', 'Link1', 'Link2', 'Link3', 'Link4', 'Link5'):
            self.assertEqual(color(name), (.035, .035, .035, 1.))
        for name in ('Link6', 'Hand_R', 'Hand_L'):
            self.assertEqual(color(name), (.10, .10, .10, 1.))
        diffuser = links['base_link'].find("visual[@name='rgb_diffuser']")
        self.assertIsNotNone(diffuser)
        self.assertEqual(tuple(map(float, diffuser.find('material/color').get('rgba').split())), (.85, .85, .85, 1.))
        self.assertEqual(diffuser.find('origin').attrib, {'xyz': '0 0 0', 'rpy': '0 0 0'})
        self.assertEqual(diffuser.find('geometry/mesh').get('filename'), 'package://DummyRobot_URDF/meshes/light.stl')
        data = (base / 'meshes/light.stl').read_bytes()
        dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        vertices = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3)
        self.assertTrue(np.isfinite(vertices).all())
        np.testing.assert_allclose(vertices.min(axis=0), [-.0443023667, -.0442976765, .1868000031], atol=1e-8)
        np.testing.assert_allclose(vertices.max(axis=0), [.0443039276, .0443039276, .1931343526], atol=1e-8)

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
    def test_target_visibility_is_an_independent_preference_and_cannot_bypass_actual_only(self):
        from copy import deepcopy
        from PySide6.QtWidgets import QApplication
        application = QApplication.instance() or QApplication([])
        from app.visualization.robot_view import RobotView
        with patch('app.visualization.robot_view.AssetServer', side_effect=RuntimeError('offline test')):
            view = RobotView()
        states, frames = [], []
        view._bridge.state_changed.connect(lambda text: states.append(json.loads(text)))
        view._bridge.rgb_changed.connect(lambda text: frames.append(json.loads(text)))
        try:
            self.assertIs(view._state['targetEnabled'], True)
            view.set_target_pose([10, -45, 90, 0, 10, 20, 15], 20)
            view.set_actual_pose([190, -80, 180, 280, 101, 180, 260], 45)
            view.set_world_cartesian_target([100, 200, 300, 0, 10, 20])
            view.set_stale(True)
            view.set_feedback_paused(True)
            view.set_rgb_preview(True)
            view.set_rgb_frame([(0, 128, 64)] * 24, light_on=True)
            view._on_ready()
            frames.clear()
            rgb_before = (view._rgb_frame, view._rgb_light_on)
            for preference in ('preview', 'actual'):
                view.set_display_mode(preference)
                before = deepcopy(view._state)
                before_signals = len(states)
                view.set_target_visible(False)
                expected = deepcopy(before)
                expected['targetEnabled'] = False
                self.assertEqual(view._state, expected)
                self.assertEqual(states[-1], expected)
                self.assertEqual(len(states), before_signals + 1)
                self.assertEqual(view._display_mode, preference)
                view.set_target_visible(False)
                self.assertEqual(len(states), before_signals + 1, 'An unchanged visibility preference must not resend poses')
                view.set_target_visible(True)
                self.assertEqual(view._state, before)
                self.assertEqual(states[-1], before)
                self.assertEqual(len(states), before_signals + 2)

            view.set_display_mode('preview')
            view.set_actual_only(True)
            view.set_commanded_gripper(30)
            for visible in (False, True, False):
                before = deepcopy(view._state)
                view.set_target_visible(visible)
                expected = deepcopy(before)
                expected['targetEnabled'] = visible
                self.assertEqual(view._state, expected)
                self.assertIs(view._state['actualOnly'], True)
                self.assertEqual(view._state['mode'], 'actual')
                self.assertIsNone(view._state['marker'])
                self.assertEqual(view._display_mode, 'preview')
            view.set_actual_only(False)
            self.assertIs(view._state['targetEnabled'], False, 'Leaving actual-only mode must retain the ghost preference')
            self.assertEqual(view._state['mode'], 'preview')
            self.assertIsNotNone(view._state['marker'])
            self.assertTrue(view._state['actualKnown'])
            self.assertTrue(view._state['actualGripperKnown'])
            self.assertTrue(view._state['stale'])
            self.assertTrue(view._state['feedbackPaused'])
            self.assertEqual(view._state['actual'], firmware_to_urdf([190, -80, 180, 280, 101, 180, 260], 45))
            self.assertEqual((view._rgb_frame, view._rgb_light_on), rgb_before)
            self.assertEqual(frames, [], 'The ghost preference must not resend or change the RGB frame')

            before = deepcopy(view._state)
            before_signals = len(states)
            view.cleanup()
            view.set_target_visible(True)
            self.assertEqual(view._state, before)
            self.assertEqual(len(states), before_signals, 'Closed views must ignore a new visibility preference')
        finally:
            view.close()
            view.deleteLater()
            application.processEvents()

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
            view.focus_joint('rail')
            self.assertEqual(view._state['highlight'], 'rail')
            view.set_actual_only(True)
            self.assertIsNone(view._state['highlight'])
            for focused in ('rail', 'J1', 'gripper'):
                view.focus_joint(focused)
                self.assertIsNone(view._state['highlight'])
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
                self.assertIsNone(view._state['highlight'])
                view.focus_joint('J2')
                self.assertEqual(view._state['highlight'], 'Joint2')
                self.assertIsNotNone(view._state['marker'])
                view.set_actual_only(True)
                self.assertIsNone(view._state['highlight'])
            view.set_actual_known(False)
            self.assertFalse(view._state['actualKnown'])
            view.set_feedback_paused(False)
            self.assertFalse(view._state['feedbackPaused'])
        finally:
            view.close()
            view.deleteLater()
            application.processEvents()


class ModelLightBridgeTests(unittest.TestCase):
    def setUp(self):
        from tests.test_ui import APPLICATION
        from app.visualization.robot_view import RobotView
        self.application = APPLICATION
        with patch('app.visualization.robot_view.AssetServer', side_effect=RuntimeError('offline test')):
            self.view = RobotView()
        self.frames, self.poses = [], []
        self.view._bridge.rgb_changed.connect(lambda text: self.frames.append(json.loads(text)))
        self.view._bridge.state_changed.connect(lambda text: self.poses.append(json.loads(text)))

    def tearDown(self):
        self.view.cleanup()
        self.view.close()
        self.view.deleteLater()
        self.application.processEvents()

    def test_latest_rgb_frame_is_delivered_at_ready_and_separate_from_poses(self):
        from copy import deepcopy
        view = self.view
        view.set_actual_pose([10, 20, 30, 40, 50, 60, 70], 40)
        view.set_stale(True)
        before = deepcopy(view._state)
        frame = [[index, 100, 200] for index in range(24)]
        view.set_rgb_frame(frame)
        self.assertEqual(self.frames, [])
        self.assertEqual(self.poses, [])
        frame[0][0] = 250
        self.assertEqual(view._rgb_frame[0], (0, 100, 200))
        view._on_ready()
        self.assertEqual(self.frames, [[[index, 100, 200] for index in range(24)]])
        self.assertEqual(len(self.poses), 1)
        self.assertEqual(view._state, before)
        self.poses.clear()
        view.set_rgb_frame([(0, 0, 0)] * 24)
        self.assertEqual(self.frames[-1], [[0, 0, 0]] * 24)
        self.assertEqual(self.poses, [])
        self.assertEqual(view._state, before)
        view.set_rgb_frame([(0, 0, 0)] * 24)
        self.assertEqual(len(self.frames), 2)
        view.cleanup()
        view.set_rgb_frame([(255, 0, 0)] * 24)
        self.assertEqual(len(self.frames), 2)

    def test_invalid_rgb_frames_are_atomic(self):
        view = self.view
        view._on_ready()
        view.set_rgb_frame([(255, 0, 0)] * 24)
        before = view._rgb_frame
        signal_count = len(self.frames)
        for invalid in (None, [], [(0, 0, 0)] * 23, [(0, 0)] * 24,
                        [(256, 0, 0)] * 24, [(-1, 0, 0)] * 24,
                        [(True, 0, 0)] * 24, [(1.5, 0, 0)] * 24,
                        [(float('nan'), 0, 0)] * 24):
            with self.subTest(frame=invalid):
                with self.assertRaises(ValueError):
                    view.set_rgb_frame(invalid)
                self.assertEqual(view._rgb_frame, before)
                self.assertEqual(len(self.frames), signal_count)

    def test_light_power_is_cached_and_changes_even_when_a_breath_frame_is_zero(self):
        view = self.view
        zero = [(0, 0, 0)] * 24
        view.set_rgb_frame(zero, light_on=True)
        view._on_ready()
        self.assertEqual(self.frames[-1], {"colors": [[0, 0, 0]] * 24, "light_on": True})
        self.poses.clear()
        count = len(self.frames)
        view.set_rgb_frame(zero, light_on=True)
        self.assertEqual(len(self.frames), count)
        view.set_rgb_frame(zero, light_on=False)
        self.assertEqual(self.frames[-1]["light_on"], False)
        self.assertEqual(len(self.frames), count + 1)
        self.assertEqual(self.poses, [])
        for invalid in (0, 1, "true", [], float('nan')):
            with self.assertRaises(ValueError):
                view.set_rgb_frame([(255, 0, 0)] * 24, light_on=invalid)
            self.assertEqual(view._rgb_frame, tuple(zero))
            self.assertIs(view._rgb_light_on, False)
            self.assertEqual(len(self.frames), count + 1)

    def test_effect_mode_is_atomic_cached_and_independent_of_pose(self):
        view = self.view
        zero = [(0, 0, 0)] * 24
        view.set_rgb_frame(zero, light_on=True, effect_mode=8)
        view._on_ready()
        self.assertEqual(self.frames[-1]["effect_mode"], 8)
        self.poses.clear()
        count = len(self.frames)
        view.set_rgb_frame(zero, light_on=True, effect_mode=8)
        self.assertEqual(len(self.frames), count)
        for invalid in (-1, 17, True, 8.0, "8", float('nan')):
            with self.assertRaises(ValueError):
                view.set_rgb_frame([(255, 0, 0)] * 24, light_on=False, effect_mode=invalid)
            self.assertEqual(view._rgb_effect_mode, 8)
            self.assertEqual(view._rgb_frame, tuple(zero))
            self.assertIs(view._rgb_light_on, True)
            self.assertEqual(len(self.frames), count)
        view.set_rgb_frame(zero, light_on=True, effect_mode=3)
        self.assertEqual(self.frames[-1]["effect_mode"], 3)
        self.assertEqual(len(self.frames), count + 1)
        view.set_rgb_frame(zero, light_on=True)
        self.assertNotIn("effect_mode", self.frames[-1])
        self.assertIsNone(view._rgb_effect_mode)
        self.assertEqual(self.poses, [])

    def test_independent_motion_modes_replay_and_change_without_touching_pose(self):
        view = self.view
        zero = [(0, 0, 0)] * 24
        view.set_rgb_frame(zero, light_on=True, effect_mode=16)
        view._on_ready()
        self.assertEqual(self.frames[-1]["effect_mode"], 16)
        self.poses.clear()
        for mode in range(13, 17):
            view.set_rgb_frame(zero, light_on=True, effect_mode=mode)
            self.assertEqual(self.frames[-1]["effect_mode"], mode)
            count = len(self.frames)
            view.set_rgb_frame(zero, light_on=True, effect_mode=mode)
            self.assertEqual(len(self.frames), count)
        before = self.frames[-1]
        with self.assertRaises(ValueError):
            view.set_rgb_frame([(255, 0, 0)] * 24, light_on=False, effect_mode=17)
        self.assertEqual(self.frames[-1], before)
        self.assertEqual(view._rgb_frame, tuple(zero))
        self.assertIs(view._rgb_light_on, True)
        self.assertEqual(view._rgb_effect_mode, 16)
        self.assertEqual(self.poses, [])

    def test_rgb_illustration_flag_never_claims_feedback_or_changes_pose_data(self):
        from copy import deepcopy
        view = self.view
        view._on_ready()
        self.frames.clear()
        before = deepcopy(view._state)
        view.set_rgb_preview(True)
        self.assertTrue(view._state["rgbPreview"])
        self.assertIs(view._state["actualKnown"], False)
        self.assertEqual(view._state["actual"], before["actual"])
        self.assertEqual(view._state["target"], before["target"])
        self.assertEqual(view._state["stale"], before["stale"])
        self.assertEqual(self.frames, [])
        count = len(self.poses)
        view.set_rgb_preview(True)
        self.assertEqual(len(self.poses), count)
        view.set_rgb_preview(False)
        self.assertFalse(view._state["rgbPreview"])
        self.assertEqual(view._state["actual"], before["actual"])


if __name__ == '__main__':
    unittest.main()
