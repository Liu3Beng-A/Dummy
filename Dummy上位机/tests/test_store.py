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

    def test_mixed_types_roundtrip_without_inventing_movel_joints(self):
        points = [
            {"name": "关节点", "type": "MoveJ", "joints": [0, -75, 180, 0, 0, 0, 10]},
            {"name": "末端点", "type": "MoveL", "pose": [500, -400, 700, 200, -190, 250],
             "gripper": 65, "speed": 42},
        ]
        self.store.save_points(points)
        exported = Path(self.directory.name) / "mixed.json"
        self.store.export_points(exported)
        document = json.loads(exported.read_text(encoding="utf-8"))
        self.assertEqual(document["schema_version"], 3)
        self.assertEqual(self.store.import_points(exported), self.store.points)
        self.assertNotIn("joints", self.store.points[1])
        second = AppStore(self.directory.name)
        try:
            self.assertEqual(second.points, self.store.points)
            self.assertEqual(second.points[1]["pose"], points[1]["pose"])
        finally:
            second.close()

    def test_v1_and_unversioned_points_are_imported_as_movej(self):
        legacy = {"name": "旧点位", "joints": [0, -75, 180, 0, 0, 0, 0]}
        path = Path(self.directory.name) / "legacy.json"
        for document in ([legacy], {"points": [legacy]}, {"schema_version": 1, "points": [legacy]}):
            path.write_text(json.dumps(document, ensure_ascii=False), encoding="utf-8")
            imported = self.store.import_points(path)
            self.assertEqual(imported[0]["type"], "MoveJ")
            self.assertEqual(imported[0]["joints"], legacy["joints"])
            self.assertEqual(self.store.points, [])  # Import remains read-only until confirmed.

    def test_invalid_typed_points_leave_memory_and_file_unchanged(self):
        self.store.save_points([{"name": "有效", "joints": [0, 0, 0, 0, 0, 0, 0]}])
        path = Path(self.directory.name) / "points.json"
        before = path.read_bytes()
        invalid = [
            {"name": "坏", "type": "Other", "joints": [0] * 7},
            {"name": "坏", "type": "MoveL", "pose": [0] * 5},
            {"name": "坏", "type": "MoveL", "pose": [10001, 0, 0, 0, 0, 0]},
            {"name": "坏", "type": "MoveL", "pose": [0, 0, 0, 3601, 0, 0]},
            {"name": "坏", "type": "MoveL", "pose": [0, 0, float("inf"), 0, 0, 0]},
            {"name": "坏", "type": "MoveL", "pose": [0, 0, True, 0, 0, 0]},
            {"name": "坏", "type": "MoveL", "pose": [0] * 6, "gripper": True},
            {"name": "坏", "type": "MoveL", "pose": [0] * 6, "speed": False},
        ]
        for point in invalid:
            with self.subTest(point=point), self.assertRaises(ValueError):
                self.store.save_points([point])
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(self.store.points[0]["name"], "有效")

    def test_unknown_schema_is_rejected_on_import_and_startup(self):
        path = Path(self.directory.name) / "points.json"
        path.write_text(json.dumps({"schema_version": 999, "points": []}), encoding="utf-8")
        with self.assertRaises(ValueError):
            self.store.import_points(path)
        second = AppStore(self.directory.name)
        try:
            self.assertEqual(second.points, [])
            self.assertTrue(any("不支持" in error for error in second.load_errors))
        finally:
            second.close()

    def test_missing_points_key_is_not_treated_as_empty_import(self):
        self.store.save_points([{"name": "保留", "joints": [0] * 7}])
        before = (Path(self.directory.name) / "points.json").read_bytes()
        incoming = Path(self.directory.name) / "missing.json"
        for data in ({}, {"schema_version": 2}, {"schema_version": 2, "point": []}):
            incoming.write_text(json.dumps(data), encoding="utf-8")
            with self.subTest(data=data), self.assertRaises(ValueError):
                self.store.import_points(incoming)
            self.assertEqual(self.store.points[0]["name"], "保留")
            self.assertEqual((Path(self.directory.name) / "points.json").read_bytes(), before)
        incoming.write_text(json.dumps({"schema_version": 2, "points": []}), encoding="utf-8")
        self.assertEqual(self.store.import_points(incoming), [])


if __name__ == "__main__":
    unittest.main()
