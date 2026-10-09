import json
from pathlib import Path
import tempfile
import unittest

from app.storage.store import AppStore


class StoreTests(unittest.TestCase):
    def setUp(self):
        test_root = Path(__file__).resolve().parent.parent / "build" / "test-data"
        test_root.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=test_root)
        self.store = AppStore(self.directory.name)

    def tearDown(self):
        self.store.close()
        self.directory.cleanup()

    def test_roundtrip_chinese_points_and_preferences(self):
        points = [{"name": "休息点位", "joints": [0, -75, 180, 0, 0, 0, 12.5],
                   "gripper": 100, "speed": 35}]
        self.store.save_points(points)
        self.store.update_settings(theme="dark", poll_hz=7)
        exported = Path(self.directory.name) / "导出点位.json"
        self.store.export_points(exported)
        self.assertEqual(self.store.import_points(exported), self.store.points)
        second = AppStore(self.directory.name)
        try:
            self.assertEqual(second.points, self.store.points)
            self.assertEqual(second.settings["theme"], "dark")
            self.assertEqual(second.settings["poll_hz"], 7)
        finally:
            second.close()

    def test_invalid_import_does_not_replace_existing_points(self):
        self.store.save_points([{"name": "有效", "joints": [0, 0, 0, 0, 0, 0, 0]}])
        previous = list(self.store.points)
        path = Path(self.directory.name) / "bad.json"
        path.write_text(json.dumps({"points": [{"name": "无效", "joints": [0] * 6 + [999]}]}), encoding="utf-8")
        with self.assertRaises(ValueError):
            self.store.import_points(path)
        self.assertEqual(self.store.points, previous)

    def test_corrupt_json_is_reported_and_does_not_crash_startup(self):
        self.store.close()
        (Path(self.directory.name) / "settings.json").write_text("broken", encoding="utf-8")
        self.store = AppStore(self.directory.name)
        self.assertTrue(self.store.load_errors)
        self.assertEqual(self.store.settings["poll_hz"], 5)

    def test_nan_or_incomplete_pose_rejected(self):
        for joints in ([0] * 6, [0] * 6 + [float("nan")]):
            with self.assertRaises(ValueError):
                self.store.save_points([{"name": "bad", "joints": joints}])


if __name__ == "__main__":
    unittest.main()
