"""Profile persistence remains local, detached, and atomic on failure."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from app.gamepad.config import default_config, validate_config
from app.storage.store import AppStore, DEFAULT_SETTINGS


class GamepadStoreTests(unittest.TestCase):
    def setUp(self):
        test_root = Path(__file__).resolve().parent.parent / "build" / "test-data"
        test_root.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=test_root)
        self.path = Path(self.directory.name)
        self.store = AppStore(self.path)

    def tearDown(self):
        self.store.close()
        self.directory.cleanup()

    def reopen(self):
        self.store.close()
        self.store = AppStore(self.path)

    def test_new_and_legacy_settings_receive_defaults_without_writes(self):
        self.assertEqual(self.store.settings["gamepad_config"], default_config())
        self.assertFalse((self.path / "settings.json").exists())
        path = self.path / "settings.json"
        path.write_text('{"theme":"dark","poll_hz":8}', encoding="utf-8")
        original = path.read_bytes()
        self.reopen()
        self.assertEqual(self.store.settings.get("theme"), "dark")
        self.assertEqual(self.store.settings.get("gamepad_config"), default_config())
        self.assertEqual(path.read_bytes(), original)
        self.assertEqual(self.store.load_errors, [])

    def test_valid_configuration_persists_complete_normalized_copy(self):
        config = {"gamepad": {"device_guid": " ABCD ", "deadzone": .3},
                  "control": {"linear_speed_mm_s": 20}}
        expected = validate_config(config)
        self.store.update_settings(gamepad_config=config, theme="dark", poll_hz=99)
        config["gamepad"]["deadzone"] = .8
        data = json.loads((self.path / "settings.json").read_text(encoding="utf-8"))
        self.assertEqual(data["gamepad_config"], expected)
        self.assertEqual(data["poll_hz"], 10)
        self.assertEqual(data["theme"], "dark")
        self.assertEqual(data["gamepad_defaults_revision"], 4)
        self.assertEqual(self.store.settings["gamepad_config"], expected)
        self.reopen()
        self.assertEqual(self.store.settings["gamepad_config"], expected)

    def test_profile_reads_cannot_dirty_store_or_other_instances(self):
        snapshot = self.store.settings["gamepad_config"]
        snapshot["robot"]["init_joints"][0] = 30
        self.store.settings.get("gamepad_config")["gamepad"]["deadzone"] = .9
        dict(self.store.settings.items())["gamepad_config"]["robot"]["rail_limits"][0] = 0
        self.store.settings.copy()["gamepad_config"]["control"]["send_interval_s"] = 1
        self.assertEqual(self.store.settings["gamepad_config"], default_config())
        self.assertEqual(DEFAULT_SETTINGS["gamepad_config"], default_config())
        other = AppStore(self.path / "other")
        try:
            self.assertEqual(other.settings["gamepad_config"], default_config())
        finally:
            other.close()

    def test_bad_saved_profile_falls_back_with_visible_error_without_rewriting(self):
        path = self.path / "settings.json"
        for bad in (None, [], {"gamepad": {"deadzone": 1}}, {"serial": {}}):
            with self.subTest(bad=bad):
                path.write_text(json.dumps({"theme": "dark", "gamepad_config": bad}), encoding="utf-8")
                original = path.read_bytes()
                self.reopen()
                self.assertEqual(self.store.settings["gamepad_config"], default_config())
                self.assertEqual(self.store.settings["theme"], "dark")
                self.assertTrue(any("手柄配置读取失败" in error for error in self.store.load_errors))
                self.assertEqual(path.read_bytes(), original)

    def test_invalid_update_leaves_all_settings_and_disk_intact(self):
        self.store.update_settings(theme="dark")
        original = (self.path / "settings.json").read_bytes()
        before = deepcopy(self.store.settings.copy())
        with self.assertRaises(ValueError):
            self.store.update_settings(theme="light", gamepad_config={"control": {"send_interval_s": 0}})
        self.assertEqual(self.store.settings, before)
        self.assertEqual((self.path / "settings.json").read_bytes(), original)

    def test_write_failure_does_not_apply_configuration_or_other_changes(self):
        self.store.update_settings(theme="dark")
        before = deepcopy(self.store.settings.copy())
        original = (self.path / "settings.json").read_bytes()
        with patch.object(self.store, "_write", side_effect=OSError("disk full")):
            with self.assertRaises(OSError):
                self.store.update_settings(gamepad_config={"gamepad": {"deadzone": .3}}, theme="light")
        self.assertEqual(self.store.settings, before)
        self.assertEqual((self.path / "settings.json").read_bytes(), original)

    def test_existing_flat_settings_and_points_round_trip_with_profile(self):
        self.store.update_settings(port="COM9", geometry="test", show_axes=False,
                                   gamepad_config={"control": {"response_timeout_s": 4}})
        self.store.save_points([{"name": "test", "joints": [0, -45, 90, 0, 0, 0, 0]}])
        self.reopen()
        self.assertEqual(self.store.settings["port"], "COM9")
        self.assertEqual(self.store.settings.get("geometry"), "test")
        self.assertFalse(self.store.settings.get("show_axes"))
        self.assertEqual(self.store.settings["gamepad_config"]["control"]["response_timeout_s"], 4)
        self.assertEqual(self.store.points[0]["name"], "test")

    def test_legacy_factory_values_migrate_in_memory_without_losing_custom_fields(self):
        for revision in (None, 1):
            with self.subTest(revision=revision):
                profile = default_config()
                profile["robot"].update(tool_offset_z=160, init_joints=[0, -45, 90, 0, 0, 0], cmd_speed=23)
                profile["robot"]["flange_limits"]["z"] = [-150, 250]
                profile["gamepad"].update(device_guid="custom-device", deadzone=.22)
                profile["gamepad"]["axes"]["left_x"] = 2
                profile["control"]["linear_speed_mm_s"] = 17
                document = {"theme": "dark", "gamepad_config": profile}
                if revision is not None:
                    document["gamepad_defaults_revision"] = revision
                path = self.path / "settings.json"
                path.write_text(json.dumps(document), encoding="utf-8")
                before = path.read_bytes()
                self.reopen()
                migrated = self.store.settings["gamepad_config"]
                self.assertEqual(migrated["robot"]["tool_offset_z"], 114)
                self.assertEqual(migrated["robot"]["init_joints"], [0, 10, 80, 0, 90, 0])
                self.assertEqual(migrated["robot"]["flange_limits"]["z"], [-150, 300])
                self.assertEqual(migrated["robot"]["cmd_speed"], 23)
                self.assertEqual(migrated["gamepad"], profile["gamepad"])
                self.assertEqual(migrated["control"], profile["control"])
                self.assertEqual(self.store.settings["gamepad_defaults_revision"], 4)
                self.assertEqual(path.read_bytes(), before)
                self.assertEqual(self.store.load_errors, [])

    def test_revision_two_preserves_explicit_old_values_and_legacy_custom_values(self):
        for revision, tool, joints in ((2, 160, [0, -45, 90, 0, 0, 0]),
                                       (1, 175, [1, 10, 80, 0, 90, 0])):
            with self.subTest(revision=revision):
                path = self.path / "settings.json"
                path.write_text(json.dumps({"gamepad_defaults_revision": revision,
                                           "gamepad_config": {"robot": {"tool_offset_z": tool, "init_joints": joints,
                                                                         "flange_limits": {"z": [-150, 320]}}}}),
                                encoding="utf-8")
                self.reopen()
                self.assertEqual(self.store.settings["gamepad_config"]["robot"]["tool_offset_z"], tool)
                self.assertEqual(self.store.settings["gamepad_config"]["robot"]["init_joints"], joints)
                self.assertEqual(self.store.settings["gamepad_config"]["robot"]["flange_limits"]["z"], [-150, 320])

    def test_migrated_settings_later_save_revision_and_explicit_160_survives_restart(self):
        profile = default_config()
        profile["robot"].update(tool_offset_z=160, init_joints=[0, -45, 90, 0, 0, 0])
        path = self.path / "settings.json"
        path.write_text(json.dumps({"gamepad_config": profile}), encoding="utf-8")
        self.reopen()
        self.store.update_settings(theme="dark")
        self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["gamepad_defaults_revision"], 4)
        self.store.update_settings(gamepad_config={"robot": {"tool_offset_z": 160, "init_joints": [0, -45, 90, 0, 0, 0]}})
        self.reopen()
        self.assertEqual(self.store.settings["gamepad_config"]["robot"]["tool_offset_z"], 160)
        self.assertEqual(self.store.settings["gamepad_config"]["robot"]["init_joints"], [0, -45, 90, 0, 0, 0])

    def test_legacy_custom_joint_limits_keep_profile_when_new_preset_would_exceed_them(self):
        profile = default_config()
        profile["robot"].update(tool_offset_z=160, init_joints=[0, -45, 90, 0, 0, 0])
        profile["robot"]["joint_limits"][4] = [-50, 50]
        profile["gamepad"]["device_guid"] = "narrow-profile"
        path = self.path / "settings.json"
        path.write_text(json.dumps({"gamepad_config": profile}), encoding="utf-8")
        before = path.read_bytes()
        self.reopen()
        preserved = self.store.settings["gamepad_config"]
        self.assertEqual(preserved["robot"]["tool_offset_z"], 114)
        self.assertEqual(preserved["robot"]["init_joints"], profile["robot"]["init_joints"])
        self.assertEqual(preserved["robot"]["joint_limits"], profile["robot"]["joint_limits"])
        self.assertEqual(preserved["gamepad"]["device_guid"], "narrow-profile")
        self.assertTrue(any("自定义关节范围" in error for error in self.store.load_errors))
        self.assertEqual(path.read_bytes(), before)

    def test_old_default_linear_speed_migrates_once_and_custom_speed_is_preserved(self):
        path = self.path / "settings.json"
        for revision, speed, expected in ((1, 10, 50), (2, 10, 50),
                                           (2, 17, 17), (3, 10, 10)):
            with self.subTest(revision=revision, speed=speed):
                profile = default_config()
                profile["control"]["linear_speed_mm_s"] = speed
                profile["gamepad"]["device_guid"] = "retained-device"
                path.write_text(json.dumps({"gamepad_defaults_revision": revision,
                                           "gamepad_config": profile}), encoding="utf-8")
                before = path.read_bytes()
                self.reopen()
                self.assertEqual(self.store.settings["gamepad_config"]["control"]["linear_speed_mm_s"], expected)
                self.assertEqual(self.store.settings["gamepad_config"]["gamepad"]["device_guid"], "retained-device")
                self.assertEqual(self.store.settings["gamepad_defaults_revision"], 4)
                self.assertEqual(path.read_bytes(), before)
        self.store.update_settings(gamepad_config={"control": {"linear_speed_mm_s": 10}})
        self.reopen()
        self.assertEqual(self.store.settings["gamepad_config"]["control"]["linear_speed_mm_s"], 10)

    def test_old_default_command_speed_migrates_once_without_changing_init_speed(self):
        path = self.path / "settings.json"
        for revision, speed, expected in ((1, 10, 35), (2, 10, 35), (3, 10, 35),
                                         (3, 23, 23), (4, 10, 10)):
            with self.subTest(revision=revision, speed=speed):
                profile = default_config()
                profile["robot"]["cmd_speed"] = speed
                profile["robot"]["init_speed"] = 10
                profile["gamepad"]["device_guid"] = "custom-device"
                document = {"gamepad_defaults_revision": revision, "gamepad_config": profile}
                path.write_text(json.dumps(document), encoding="utf-8")
                before = path.read_bytes()
                self.reopen()
                loaded = self.store.settings["gamepad_config"]
                self.assertEqual(loaded["robot"]["cmd_speed"], expected)
                self.assertEqual(loaded["robot"]["init_speed"], 10)
                self.assertEqual(loaded["gamepad"], profile["gamepad"])
                self.assertEqual(path.read_bytes(), before)
        self.store.update_settings(gamepad_config={"robot": {"cmd_speed": 10}})
        self.reopen()
        self.assertEqual(self.store.settings["gamepad_config"]["robot"]["cmd_speed"], 10)

    def test_legacy_manual_binding_loads_without_resetting_custom_settings_or_writing(self):
        path = self.path / "settings.json"
        profile = default_config()
        profile["gamepad"]["device_guid"] = "custom-device"
        profile["gamepad"]["axes"]["left_x"] = 2
        profile["gamepad"]["buttons"].update(start=0, manual=1)
        profile["control"]["linear_speed_mm_s"] = 22
        profile["robot"]["tool_offset_z"] = 150
        document = {"gamepad_defaults_revision": 3, "gamepad_config": profile, "theme": "dark"}
        path.write_text(json.dumps(document), encoding="utf-8")
        before = path.read_bytes()
        self.reopen()
        expected = validate_config(profile)
        self.assertEqual(self.store.load_errors, [])
        self.assertEqual(self.store.settings["gamepad_config"], expected)
        self.assertEqual(self.store.settings["theme"], "dark")
        self.assertNotIn("manual", expected["gamepad"]["buttons"])
        self.assertEqual(path.read_bytes(), before)
        self.assertEqual(profile["gamepad"]["buttons"]["manual"], 1)
        self.store.update_settings(gamepad_config=self.store.settings["gamepad_config"])
        saved = json.loads(path.read_text(encoding="utf-8"))["gamepad_config"]
        self.assertEqual(saved, expected)
        self.assertNotIn("manual", saved["gamepad"]["buttons"])


if __name__ == "__main__":
    unittest.main()
