"""Gamepad conversion and profile bounds, without Qt, SDL or hardware."""
from copy import deepcopy
import math
import unittest

from app.gamepad.config import default_config, validate_config
from app.gamepad.input import decode_gamepad, motion_delta


class GamepadInputTests(unittest.TestCase):
    def setUp(self):
        self.config = default_config()
        self.axes = [0., 0., 0., 0., -1., -1.]
        self.buttons = [0] * 8
        self.hats = [(0, 0)]

    def decode(self):
        return decode_gamepad(self.axes, self.buttons, self.hats, self.config["gamepad"])

    def test_defaults_and_partial_overrides_are_independent(self):
        self.assertEqual(self.config["robot"]["tool_offset_z"], 114)
        self.assertEqual(self.config["robot"]["init_joints"], [0, 10, 80, 0, 90, 0])
        self.assertEqual(self.config["robot"]["flange_limits"]["z"], [-150, 300])
        self.assertEqual(self.config["control"]["linear_speed_mm_s"], 200)
        self.assertEqual(self.config["control"]["send_interval_s"], .1)
        self.assertEqual(self.config["control"]["velocity_smoothing_s"], .06)
        self.assertEqual(self.config["control"]["angular_speed_deg_s"], 50)
        self.assertEqual(self.config["control"]["gripper_speed_percent_s"], 150)
        self.assertEqual(self.config["robot"]["cmd_speed"], 35)
        self.assertEqual(self.config["robot"]["init_speed"], 10)
        self.assertEqual(self.config["gamepad"]["buttons"]["initialize"], 0)
        override = {"gamepad": {"device_guid": " ABC ", "deadzone": .25},
                    "robot": {"flange_limits": {"x": (-40, 280)}}}
        result = validate_config(override)
        self.assertEqual(result["gamepad"]["device_guid"], "abc")
        self.assertEqual(result["robot"]["flange_limits"]["x"], [-40., 280.])
        self.assertEqual(result["control"], self.config["control"])
        result["robot"]["init_joints"][0] = 30
        override["gamepad"]["deadzone"] = .7
        self.assertEqual(result["gamepad"]["deadzone"], .25)
        self.assertEqual(default_config()["robot"]["init_joints"][0], 0)
        self.assertEqual(set(result), {"gamepad", "control", "robot"})

    def test_invalid_profile_values_raise_without_mutation(self):
        bad = [None, [], {"serial": {}}, {"gamepad": {"deadzone": True}},
               {"gamepad": {"device_index": -1}}, {"gamepad": {"hat": 1.0}},
               {"gamepad": {"deadzone": 1}}, {"gamepad": {"trigger_min": 1}},
               {"gamepad": {"buttons": {"start": 6}}},
               {"gamepad": {"axes": {"lt": 0}}},
               {"control": {"linear_speed_mm_s": math.nan}},
               {"control": {"send_interval_s": .001}},
               {"control": {"velocity_smoothing_s": -.01}},
               {"control": {"velocity_smoothing_s": math.inf}},
               {"control": {"feedback_timeout_s": .2}},
               {"control": {"response_timeout_s": 10 ** 1000}},
               {"robot": {"cmd_speed": 101}},
               {"robot": {"rail_limits": [-251, 250]}},
               {"robot": {"rail_limits": [1, 250]}},
               {"robot": {"rail_limits": [-250, -1]}},
               {"robot": {"init_joints": [0] * 5}},
               {"robot": {"rest_joints": [0, -76, 180, 0, 0, 0]}},
               {"robot": {"z_min_height": -151}},
               {"robot": {"flange_limits": {"x": [2, 1]}}}]
        for value in bad:
            original = deepcopy(value)
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_config(value)
            # NaN does not compare equal; its input remains untouched by type.
            self.assertEqual(repr(value), repr(original))

    def test_linear_and_angular_rates_have_no_application_ceiling(self):
        config = validate_config({"control": {"linear_speed_mm_s": 500,
                                               "angular_speed_deg_s": 360}})
        self.axes[1] = -1
        self.buttons[5] = 1
        pos, att = motion_delta(self.decode(), config["control"], .01)
        self.assertEqual(pos["x"], 5)
        self.assertAlmostEqual(att["r"], 3.6)
        for key in ("linear_speed_mm_s", "angular_speed_deg_s"):
            with self.subTest(key=key):
                self.assertEqual(validate_config({"control": {key: 1e100}})["control"][key], 1e100)
            for value in (0, -1, math.nan, math.inf, -math.inf, True, "500", 10 ** 1000):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    validate_config({"control": {key: value}})

    def test_retired_manual_binding_is_ignored_without_losing_custom_profile(self):
        legacy = {"gamepad": {"device_guid": " CUSTOM ", "deadzone": .25,
                              "axes": {"left_x": 2}, "buttons": {"start": 0, "manual": 251}},
                  "control": {"linear_speed_mm_s": 22},
                  "robot": {"cmd_speed": 17}}
        before = deepcopy(legacy)
        normalized = validate_config(legacy)
        self.assertNotIn("manual", normalized["gamepad"]["buttons"])
        self.assertEqual(normalized["gamepad"]["device_guid"], "custom")
        self.assertEqual(normalized["gamepad"]["axes"]["left_x"], 2)
        self.assertEqual(normalized["gamepad"]["buttons"]["start"], 0)
        self.assertEqual(normalized["gamepad"]["buttons"]["initialize"], -1)
        self.assertEqual(normalized["control"]["linear_speed_mm_s"], 22)
        self.assertEqual(normalized["robot"]["cmd_speed"], 17)
        self.assertEqual(legacy, before)
        legacy["gamepad"]["buttons"]["unknown_action"] = 1
        with self.assertRaises(ValueError):
            validate_config(legacy)

    def test_direct_decoder_ignores_legacy_manual_and_unassigned_b(self):
        self.config["gamepad"]["buttons"]["manual"] = 251
        before = deepcopy(self.config["gamepad"])
        self.buttons[1] = 1
        decoded = self.decode()
        self.assertNotIn("manual", decoded["buttons"])
        self.assertFalse(any(decoded["buttons"].values()))
        self.assertTrue(decoded["centered"])
        self.assertEqual(self.config["gamepad"], before)
        self.config["gamepad"]["buttons"]["unknown_action"] = 1
        with self.assertRaises(ValueError):
            self.decode()

    def test_a_initialization_binding_and_disabled_legacy_mapping(self):
        self.buttons[0] = 1
        self.assertTrue(self.decode()["buttons"]["initialize"])
        self.assertFalse(self.decode()["buttons"]["start"])
        legacy = {"buttons": {"start": 0}}
        decoded = decode_gamepad(self.axes, self.buttons, self.hats, legacy)
        self.assertTrue(decoded["buttons"]["start"])
        self.assertFalse(decoded["buttons"]["initialize"])
        self.config["gamepad"]["buttons"]["initialize"] = -1
        self.buttons[7] = 1
        self.assertFalse(self.decode()["buttons"]["initialize"])  # -1 must not select the last button.
        with self.assertRaises(ValueError):
            validate_config({"gamepad": {"buttons": {"initialize": 7}}})

    def test_narrow_joint_limits_validate_preset_poses(self):
        profile = default_config()
        profile["robot"]["joint_limits"][0] = [-40, 40]
        self.assertEqual(validate_config(profile)["robot"]["joint_limits"][0], [-40., 40.])
        profile["robot"]["init_joints"][0] = 41
        with self.assertRaises(ValueError):
            validate_config(profile)
        profile = default_config()
        profile["robot"]["joint_limits"][5] = [-181, 180]
        with self.assertRaises(ValueError):
            validate_config(profile)

    def test_neutral_and_soft_deadzone_boundary(self):
        self.assertTrue(self.decode()["centered"])
        self.axes[0] = .15
        self.assertEqual(self.decode()["lx"], 0)
        self.axes[0] = -.575
        self.assertAlmostEqual(self.decode()["lx"], -.5)
        self.assertFalse(self.decode()["centered"])
        self.axes[0] = 5
        self.assertEqual(self.decode()["lx"], 1)

    def test_triggers_and_configurable_zero_based_range(self):
        self.axes[4] = -.8
        self.assertEqual(self.decode()["lt"], 0)
        self.axes[4] = 0
        self.assertEqual(self.decode()["lt"], .5)
        self.config["gamepad"].update(trigger_min=0, trigger_max=1)
        self.axes[4:6] = [0, .75]
        result = self.decode()
        self.assertEqual(result["lt"], 0)
        self.assertEqual(result["rt"], .75)

    def test_opposing_shoulders_are_not_centered_and_actions_are_reported(self):
        self.buttons[4] = self.buttons[5] = 1
        result = self.decode()
        self.assertEqual(result["roll"], 0)
        self.assertFalse(result["centered"])
        self.buttons[4] = self.buttons[5] = 0
        self.buttons[7] = 1
        result = self.decode()
        self.assertTrue(result["centered"])
        self.assertTrue(result["buttons"]["start"])
        self.assertFalse(result["buttons"]["stop"])

    def test_custom_mapping_selects_required_inputs(self):
        self.config["gamepad"]["axes"]["left_x"] = 2
        self.config["gamepad"]["buttons"]["start"] = 0
        self.config["gamepad"]["buttons"]["initialize"] = 7
        self.config["gamepad"]["hat"] = 1
        self.axes[0], self.axes[2] = -1, 1
        self.buttons[0] = 1
        self.hats.append((1, -1))
        result = self.decode()
        self.assertEqual(result["lx"], 1)
        self.assertEqual(result["hat"], (1, -1))
        self.assertTrue(result["buttons"]["start"])

    def test_absent_or_invalid_raw_input_rejects_entire_snapshot(self):
        invalid = [([], self.buttons, self.hats), (self.axes, [], self.hats),
                   (self.axes, self.buttons, []),
                   ([math.nan] + self.axes[1:], self.buttons, self.hats),
                   ([10 ** 1000] + self.axes[1:], self.buttons, self.hats),
                   (self.axes, [2] + self.buttons[1:], self.hats),
                   (self.axes, self.buttons, [(True, 0)]),
                   (self.axes, self.buttons, [(0, 2)]),
                   (self.axes, self.buttons, [(0,)]),
                   ("123456", self.buttons, self.hats)]
        for axes, buttons, hats in invalid:
            with self.subTest(axes=axes, buttons=buttons, hats=hats), self.assertRaises(ValueError):
                decode_gamepad(axes, buttons, hats, self.config["gamepad"])

    def test_axis_signs_and_rotation_match_original_controls(self):
        self.axes[0], self.axes[1], self.axes[3] = 0, -1, 0
        self.buttons[5] = 1
        self.hats[0] = (1, -1)
        pos, att = motion_delta(self.decode(), self.config["control"], .02)
        self.assertEqual(pos, {"x": 4., "y": 0., "z": 0.})
        self.assertEqual(att, {"r": 1., "p": -1., "y": 1.})

    def test_diagonal_translation_never_exceeds_scalar_speed(self):
        self.axes[0], self.axes[1], self.axes[3] = 1, -1, 1
        pos, _ = motion_delta(self.decode(), self.config["control"], .05)
        self.assertAlmostEqual(math.sqrt(sum(v * v for v in pos.values())), 10)
        self.assertAlmostEqual(pos["x"], 10 / math.sqrt(3))
        self.assertLess(pos["y"], 0)
        self.assertLess(pos["z"], 0)

    def test_dt_clips_stalls_without_accumulated_motion(self):
        self.axes[1] = -1
        self.assertEqual(motion_delta(self.decode(), self.config["control"], 20)[0]["x"], 10)
        self.assertEqual(motion_delta(self.decode(), self.config["control"], -2)[0]["x"], 0)
        for dt in (math.inf, math.nan, True, "1"):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                motion_delta(self.decode(), self.config["control"], dt)

    def test_bad_motion_snapshot_is_rejected(self):
        for decoded in ({}, {**self.decode(), "hat": (0,)}, {**self.decode(), "lx": math.nan}):
            with self.subTest(decoded=decoded), self.assertRaises(ValueError):
                motion_delta(decoded, self.config["control"], .01)


if __name__ == "__main__":
    unittest.main()
