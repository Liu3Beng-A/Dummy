"""Independent geometry references and the firmware's Euler singular branch."""
import math
import unittest

from app.gamepad.kinematics import forward_kinematics
from app.core.gamepad_session import angle_difference, tool_translation


class GamepadKinematicsTests(unittest.TestCase):
    def assert_pose(self, joints, expected):
        pose = forward_kinematics(joints)
        for actual, reference in zip(pose[:3], expected[:3]):
            self.assertAlmostEqual(actual, reference, delta=.01)
        for actual, reference in zip(pose[3:], expected[3:]):
            self.assertAlmostEqual(angle_difference(actual, reference), 0., delta=.001)

    def test_factory_init_rest_and_zero_reference_poses(self):
        self.assert_pose([0, 10, 80, 0, 90, 0], [146.520190203, 0, 288.917318012, 180, 0, 180])
        self.assert_pose([0, -75, 180, 0, 0, 0], [75.943473212, 0, 216.602702221, 180, 75, 180])
        self.assert_pose([0, 0, 0, 0, 0, 0], [-69.5, 0, 565, 0, 0, 0])

    def test_nonzero_axes_keep_firmware_signs_and_rotation_order(self):
        self.assert_pose([15, -30, 120, 25, 20, -35],
                         [129.249304975, 51.541896535, 346.697084972,
                          -155.341912232, 70.057488940, -130.226882438])

    def test_exact_and_near_gimbal_use_firmware_threshold_and_yaw_zero(self):
        self.assert_pose([0, 0, 0, 0, 90, 0], [43.5, 0, 452, 0, 90, 0])
        self.assert_pose([0, 10, 80, 0, 0, 0], [259.520190203, 0, 401.917318012, 0, 90, 0])
        self.assertEqual(forward_kinematics([0, 0, 0, 0, 89.5, 0])[4:], [90., 0.])
        self.assertAlmostEqual(forward_kinematics([0, 0, 0, 0, 89, 0])[4], 89.)
        self.assertEqual(forward_kinematics([0, 0, 0, 0, -90, 0])[4:], [-90., 0.])

    def test_rail_does_not_translate_firmware_frame_or_clip_actual_joints(self):
        self.assertEqual(forward_kinematics([0, 10, 80, 0, 90, 0, 37.25]),
                         forward_kinematics([0, 10, 80, 0, 90, 0, -200]))
        self.assert_pose([360, 10, 80, 0, 90, 0], [146.520190203, 0, 288.917318012, 180, 0, 180])

    def test_tool_114_is_added_once_after_flange_fk(self):
        flange = forward_kinematics([0, 10, 80, 0, 90, 0])
        offset = tool_translation(flange, 114)
        tcp = [flange[i] + offset[i] for i in range(3)]
        for actual, expected in zip(tcp, [146.520190203, 0, 174.917318012]):
            self.assertAlmostEqual(actual, expected, delta=.01)

    def test_nonfinite_boolean_and_wrong_shape_are_rejected(self):
        for invalid in ([], [0] * 5, [0] * 8, "000000", [True] + [0] * 5,
                        [math.nan] + [0] * 5, [0] * 6 + [math.inf]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                forward_kinematics(invalid)


if __name__ == "__main__":
    unittest.main()
