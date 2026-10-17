"""Target-ghost visibility decisions use real mappings/Qt bridge, without devices."""

from copy import deepcopy
import json
import math
import unittest
from unittest.mock import patch

from app.visualization.mapping import firmware_to_urdf, target_pose_matches


POSE = [10., 10., 80., 0., 20., 0., 10.]


class TargetPoseMatchTests(unittest.TestCase):
    def test_each_joint_and_rail_uses_close_threshold_and_release_hysteresis(self):
        self.assertTrue(target_pose_matches(POSE, POSE))
        for axis in range(7):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    actual = POSE.copy()
                    actual[axis] += sign * .1
                    self.assertTrue(target_pose_matches(POSE, actual))
                    actual[axis] = POSE[axis] + sign * (.1 + 1e-8)
                    self.assertFalse(target_pose_matches(POSE, actual))
                    self.assertTrue(target_pose_matches(POSE, actual, was_matched=True))
                    actual[axis] = POSE[axis] + sign * .2
                    self.assertTrue(target_pose_matches(POSE, actual, was_matched=True))
                    actual[axis] = POSE[axis] + sign * (.2 + 1e-8)
                    self.assertFalse(target_pose_matches(POSE, actual, was_matched=True))

    def test_gripper_is_compared_only_when_both_feedback_and_target_are_known(self):
        for target, actual in ((None, None), (75, None), (None, 0)):
            self.assertTrue(target_pose_matches(POSE, POSE,
                target_gripper=target, actual_gripper=actual))
        self.assertTrue(target_pose_matches(POSE, POSE, target_gripper=75, actual_gripper=76))
        self.assertFalse(target_pose_matches(POSE, POSE, target_gripper=75, actual_gripper=76.01))
        self.assertTrue(target_pose_matches(POSE, POSE, was_matched=True,
            target_gripper=75, actual_gripper=77))
        self.assertFalse(target_pose_matches(POSE, POSE, was_matched=True,
            target_gripper=75, actual_gripper=77.01))

    def test_angles_are_raw_unwrapped_and_not_clipped_to_model_limits(self):
        outside_limits = [190., -80., 181., 280., 101., 185., 260.]
        self.assertTrue(target_pose_matches(outside_limits, outside_limits))
        clipped_target = [175., -75., 180., 270., 100., 180., 250.]
        self.assertFalse(target_pose_matches(clipped_target, outside_limits))
        positive, negative = POSE.copy(), POSE.copy()
        positive[5], negative[5] = 180., -180.
        self.assertFalse(target_pose_matches(positive, negative))

    def test_invalid_values_never_report_a_match(self):
        for bad in ([], [0.] * 6, [0.] * 8,
                    [float('nan')] + POSE[1:], [float('inf')] + POSE[1:]):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    target_pose_matches(bad, POSE)
                with self.assertRaises(ValueError):
                    target_pose_matches(POSE, bad)
        with self.assertRaises(ValueError):
            target_pose_matches(POSE, POSE, target_gripper=75, actual_gripper=math.nan)

    def test_inputs_are_not_modified(self):
        target, actual = POSE.copy(), POSE.copy()
        before = deepcopy((target, actual))
        target_pose_matches(target, actual, target_gripper=75, actual_gripper=75)
        self.assertEqual((target, actual), before)


class TargetGhostBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication
        cls.application = QApplication.instance() or QApplication([])
        from app.visualization.robot_view import RobotView
        cls.view_type = RobotView

    def setUp(self):
        # Fail before creating WebEngine pages or opening an HTTP listener.
        with patch('app.visualization.robot_view.AssetServer', side_effect=RuntimeError('offline test')):
            self.view = self.view_type()
        self.states, self.rgb_frames = [], []
        self.view._bridge.state_changed.connect(lambda payload: self.states.append(json.loads(payload)))
        self.view._bridge.rgb_changed.connect(lambda payload: self.rgb_frames.append(json.loads(payload)))
        self.view.set_target_pose(POSE, 75)
        self.view._on_ready()
        self.states.clear()
        self.rgb_frames.clear()

    def tearDown(self):
        self.view.close()
        self.view.deleteLater()
        self.application.processEvents()

    def assertMatched(self, expected):
        self.assertIs(self.view._state['targetMatched'], expected)
        self.assertIs(self.states[-1]['targetMatched'], expected)

    def test_fresh_match_hides_and_changed_actual_position_restores_target(self):
        self.assertIs(self.view._state['targetMatched'], False)
        self.view.set_actual_pose(POSE, 75)
        self.assertMatched(True)
        displaced = POSE.copy()
        displaced[6] += .3
        self.view.set_actual_pose(displaced, 75)
        self.assertMatched(False)
        self.view.set_actual_pose(POSE, 75)
        self.assertMatched(True)
        self.assertIs(self.view._state['targetEnabled'], True)

    def test_noise_uses_hysteresis_and_same_target_does_not_reset_it(self):
        self.view.set_actual_pose(POSE)
        for offset, expected in ((.15, True), (.2, True), (.201, False), (.15, False), (.1, True)):
            actual = POSE.copy()
            actual[0] += offset
            self.view.set_actual_pose(actual)
            self.assertMatched(expected)
            self.view.set_target_pose(POSE, 75)
            self.assertMatched(expected)

    def test_edited_joint_target_must_meet_close_threshold_again(self):
        self.view.set_actual_pose(POSE, 75)
        self.assertMatched(True)
        edited = POSE.copy()
        edited[0] += .15
        self.view.set_target_pose(edited, 75)
        self.assertMatched(False)
        self.view.set_actual_pose(edited, 75)
        self.assertMatched(True)

    def test_edited_gripper_must_meet_close_threshold_again(self):
        self.view.set_actual_pose(POSE, 75)
        self.view.set_actual_pose(POSE, 76.5)
        self.assertMatched(True)
        self.view.set_target_gripper(75.)
        self.assertMatched(True)  # An unchanged target retains hysteresis.
        self.view.set_target_gripper(75.1)
        self.assertMatched(False)
        self.view.set_actual_pose(POSE, 75.1)
        self.assertMatched(True)

    def test_unknown_gripper_feedback_does_not_prevent_joint_match(self):
        self.view.set_actual_pose(POSE)
        self.assertMatched(True)
        self.assertIs(self.view._state['actualGripperKnown'], False)
        self.view.set_target_gripper(20)
        self.assertMatched(True)
        self.view.set_actual_pose(POSE, 75)
        self.assertMatched(False)

    def test_unknown_stale_and_paused_feedback_cannot_hide_a_target(self):
        self.view.set_actual_pose(POSE, 75)
        self.assertMatched(True)
        self.view.set_stale(True)
        self.assertMatched(False)
        self.view.set_theme('dark')
        self.assertMatched(False)
        self.view.set_stale(False)
        self.assertMatched(True)
        self.view.set_feedback_paused(True)
        self.assertMatched(False)
        self.view.set_actual_pose(POSE, 75)
        self.assertMatched(False)
        self.view.set_feedback_paused(False)
        self.assertMatched(True)
        self.view.set_actual_known(False)
        self.assertMatched(False)
        self.assertIs(self.view._state['actualGripperKnown'], False)
        self.view.set_target_pose(POSE, 75)
        self.assertMatched(False)
        self.view.set_actual_pose(POSE)
        self.assertMatched(True)

    def test_raw_feedback_outside_model_limits_does_not_make_a_false_match(self):
        raw = [190., -80., 181., 280., 101., 185., 260.]
        self.view.set_target_pose(raw, 75)
        self.view.set_actual_pose(raw, 75)
        self.assertMatched(False)
        self.assertEqual(self.view._actual_pose, raw)
        self.assertEqual(self.view._state['actual'], firmware_to_urdf(raw, 75))
        self.assertNotEqual(self.view._state['target'], self.view._state['actual'])

    def test_manual_switch_is_independent_and_never_reenabled_by_auto_match(self):
        self.view.set_actual_pose(POSE, 75)
        self.view.set_target_visible(False)
        self.assertMatched(True)
        other = POSE.copy()
        other[0] += 1
        self.view.set_actual_pose(other, 75)
        self.assertMatched(False)
        self.assertIs(self.view._state['targetEnabled'], False)
        self.view.set_target_visible(True)
        self.assertMatched(False)
        self.assertIs(self.view._state['targetEnabled'], True)

    def test_movel_marker_and_actual_only_rules_remain_independent(self):
        self.view.set_world_cartesian_target([100, 200, 300, 0, 90, 180])
        marker = deepcopy(self.view._state['marker'])
        self.view.set_actual_pose(POSE, 75)
        self.assertMatched(True)
        self.assertEqual(self.view._state['marker'], marker)
        self.view.set_actual_only(True)
        self.assertIsNone(self.view._state['marker'])
        self.assertEqual(self.view._state['mode'], 'actual')
        self.view.set_actual_only(False)
        self.assertEqual(self.view._state['marker'], marker)
        self.assertEqual(self.view._state['mode'], 'preview')

    def test_match_and_visibility_changes_emit_only_display_state_and_preserve_rgb(self):
        frame = [(0, 128, 64)] * 24
        self.view.set_rgb_frame(frame, light_on=True, effect_mode=16)
        before_rgb = (self.view._rgb_frame, self.view._rgb_light_on, self.view._rgb_effect_mode)
        self.rgb_frames.clear()
        # These are view calls only: they expose no serial/controller send hook.
        self.view.set_actual_pose(POSE, 75)
        self.view.set_target_visible(False)
        self.view.set_target_visible(True)
        self.view.set_target_gripper(50)
        self.assertEqual((self.view._rgb_frame, self.view._rgb_light_on, self.view._rgb_effect_mode), before_rgb)
        self.assertEqual(self.rgb_frames, [])
        self.assertTrue(self.states)
        self.assertIsNone(self.view._server)
        self.assertIsNone(self.view.web_view)


if __name__ == '__main__':
    unittest.main()
