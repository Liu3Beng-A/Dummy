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
        self.assert_pose([0, 10, 80, 0, 90, 0], [146.520190203, 0, 331.217318012, 180, 0, 180])
        self.assert_pose([0, -75, 180, 0, 0, 0], [75.943473212, 0, 258.902702221, 180, 75, 180])
        self.assert_pose([0, 0, 0, 0, 0, 0], [-69.5, 0, 607.3, 0, 0, 0])

    def test_nonzero_axes_keep_firmware_signs_and_rotation_order(self):
        self.assert_pose([15, -30, 120, 25, 20, -35],
                         [129.249304975, 51.541896535, 388.997084972,
                          -155.341912232, 70.057488940, -130.226882438])

    def test_exact_and_near_gimbal_preserve_pitch_with_new_firmware(self):
        self.assert_pose([0, 0, 0, 0, 90, 0], [43.5, 0, 494.3, 0, 90, 0])
        self.assert_pose([0, 10, 80, 0, 0, 0], [259.520190203, 0, 444.217318012, 0, 90, 0])
        self.assertAlmostEqual(forward_kinematics([0, 0, 0, 0, 89.5, 0])[4], 89.5)
        self.assertAlmostEqual(forward_kinematics([0, 0, 0, 0, 89, 0])[4], 89.)
        self.assertEqual(forward_kinematics([0, 0, 0, 0, -90, 0])[4:], [-90., 0.])

    def test_rail_does_not_translate_firmware_frame_or_clip_actual_joints(self):
        self.assertEqual(forward_kinematics([0, 10, 80, 0, 90, 0, 37.25]),
                         forward_kinematics([0, 10, 80, 0, 90, 0, -200]))
        self.assert_pose([360, 10, 80, 0, 90, 0], [146.520190203, 0, 331.217318012, 180, 0, 180])

    def test_tool_114_is_added_once_after_flange_fk(self):
        flange = forward_kinematics([0, 10, 80, 0, 90, 0])
        offset = tool_translation(flange, 114)
        tcp = [flange[i] + offset[i] for i in range(3)]
        for actual, expected in zip(tcp, [146.520190203, 0, 217.217318012]):
            self.assertAlmostEqual(actual, expected, delta=.01)

    def test_new_height_and_euler_matrix_match_pure_model(self):
        import numpy as np
        from dummy_motion import RobotModel, Tool, Pose
        from dummy_motion.frames import from_wire_joints, from_xyzabc, error
        model = RobotModel()
        for joints in ([23, 0, 0, 0, 90, 0, 100],
                       [-17, 0, 0, 0, -90, 0, -100],
                       [23, 0, 0, 0, 89.5, 0, 20],
                       [23, 0, 0, 0, 89.99, 0, 20],
                       [15, -30, 120, 265, 20, -35, 50]):
            flange = model.fk(from_wire_joints(joints), Tool(Pose.identity()), frame="base")
            actual = from_xyzabc(forward_kinematics(joints))
            np.testing.assert_allclose(error(actual, flange), 0, atol=1e-11)
        self.assert_pose([23, 0, 0, 0, 90, 0],
                         [43.5*math.cos(math.radians(23)),
                          43.5*math.sin(math.radians(23)), 494.3, 0, 90, 23])

    def test_nonfinite_boolean_and_wrong_shape_are_rejected(self):
        for invalid in ([], [0] * 5, [0] * 8, "000000", [True] + [0] * 5,
                        [math.nan] + [0] * 5, [0] * 6 + [math.inf]):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                forward_kinematics(invalid)


if __name__ == "__main__":
    unittest.main()
