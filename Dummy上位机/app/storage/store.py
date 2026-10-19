"""Persistent local data. Reading a profile never writes to a robot."""
from __future__ import annotations

from copy import deepcopy
import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import tempfile
import threading

from app.gamepad.config import default_config as default_gamepad_config
from app.gamepad.config import validate_config as validate_gamepad_config


DEFAULT_SETTINGS = {
    "theme": "light", "poll_hz": 5, "log_limit": 5000, "hide_joint_logs": True,
    "hide_rgb_logs": True,
    "port": "", "connection_type": "USB", "geometry": "",
    "window_state": "", "viewer_mode": "preview", "show_grid": True,
    "show_axes": True, "show_target_ghost": True,
    "auto_poll": True, "poll_enabled": True, "model_path": "",
    "gamepad_config": default_gamepad_config(),
    "gamepad_defaults_revision": 8,
    "cartesian_backend": "firmware",
}
JOINT_LIMITS = [(-175, 175), (-75, 90), (0, 180), (-270, 270),
                (-100, 100), (-180, 180), (-250, 250)]
POSE_LIMITS = [(-10000, 10000)] * 3 + [(-3600, 3600)] * 3
POINT_SCHEMA_VERSION = 3


class _Settings(dict):
    """Keep existing flat settings access while detaching nested profiles."""

    def __getitem__(self, key):
        value = super().__getitem__(key)
        return deepcopy(value) if key == "gamepad_config" else value

    def get(self, key, default=None):
        return self[key] if key in self else default

    def items(self):
        return {key: self[key] for key in self}.items()

    def values(self):
        return {key: self[key] for key in self}.values()

    def copy(self):
        return dict(self.items())


def _point_number(value, name, field, limits):
    if isinstance(value, bool):
        raise ValueError(f"{name}：{field} 必须是数值")
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name}：{field} 必须是数值") from exc
    if not math.isfinite(value) or not limits[0] <= value <= limits[1]:
        raise ValueError(f"{name}：{field} 超出范围 {limits[0]}～{limits[1]}")
    return value


def _file_points(data):
    if isinstance(data, dict):
        version = data.get("schema_version", 1)
        if isinstance(version, bool) or not isinstance(version, int) or version not in (1, 2, 3):
            raise ValueError("不支持该点位文件版本")
        if "points" not in data:
            raise ValueError("点位文件缺少 points 列表")
        return data["points"]
    return data


def validate_points(records):
    """Validate the whole file before replacing any existing taught points."""
    if not isinstance(records, list) or len(records) > 10000:
        raise ValueError("点位文件必须包含不超过 10000 条记录的列表")
    result = []
    for index, record in enumerate(records, 1):
        if not isinstance(record, dict):
            raise ValueError(f"第 {index} 个点位格式错误")
        name = str(record.get("name", f"点位 {index}")).strip()
        if not name or len(name) > 120:
            raise ValueError(f"第 {index} 个点位名称为空或过长")
        point_type = record.get("type", "MoveJ")
        if point_type == "MoveJ":
            key, limits = "joints", JOINT_LIMITS
            labels = ["J1", "J2", "J3", "J4", "J5", "J6", "地轨"]
            description = "需要六个关节角及一个地轨位置"
        elif point_type == "MoveL":
            key, limits = "pose", POSE_LIMITS
            labels = ["X", "Y", "Z", "A", "B", "C"]
            description = "需要末端坐标 X/Y/Z 和姿态角 A/B/C"
        else:
            raise ValueError(f"{name}：点位类型必须为 MoveJ 或 MoveL")
        coordinates = record.get(key)
        if not isinstance(coordinates, (list, tuple)) or len(coordinates) != len(limits):
            raise ValueError(f"{name}：{description}")
        values = [_point_number(value, name, label, bounds)
                  for value, label, bounds in zip(coordinates, labels, limits)]
        gripper = _point_number(record.get("gripper", 100), name, "夹爪开度", (0, 100))
        speed = _point_number(record.get("speed", 35), name, "速度", (1, 100))
        item = {"name": name, "type": point_type, key: values, "gripper": gripper, "speed": speed}
        if point_type == "MoveL" and any(field in record for field in ("frame", "target_kind", "tool")):
            frame, target_kind = record.get("frame", "base"), record.get("target_kind", "flange")
            if (frame, target_kind) not in (("base", "flange"), ("world", "tcp")):
                raise ValueError(f"{name}：坐标语义必须为 base/flange 或 world/tcp")
            item.update(frame=frame, target_kind=target_kind)
            if frame == "world" and "tool" not in record:
                raise ValueError(f"{name}：世界 TCP 点位必须保存工具快照")
            if "tool" in record:
                if type(record.get("tool_version", 1)) is not int or record.get("tool_version", 1) != 1:
                    raise ValueError(f"{name}：不支持该工具快照版本")
                tool = record["tool"]
                if not isinstance(tool, (list, tuple)) or len(tool) != 6:
                    raise ValueError(f"{name}：工具偏移需六个 XYZABC 数值")
                item["tool"] = [_point_number(value, name, "工具偏移", bound) for value, bound in zip(tool, POSE_LIMITS)]
                item["tool_version"] = 1
        result.append(item)
    return result


class AppStore:
    def __init__(self, base_dir=None):
        if base_dir is None:
            local = os.environ.get("LOCALAPPDATA")
            base_dir = Path(local) / "DummyRobot" if local else Path.home() / ".dummyrobot"
        self.base_dir = Path(base_dir).resolve()
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self.load_errors = []
        self.settings = _Settings(deepcopy(DEFAULT_SETTINGS))
        raw = self._read(self.base_dir / "settings.json", {})
        if isinstance(raw, dict):
            for key in DEFAULT_SETTINGS:
                if key in raw:
                    self.settings[key] = raw[key]
        try:
            self.settings["gamepad_config"] = validate_gamepad_config(self.settings["gamepad_config"])
            revision = raw.get("gamepad_defaults_revision", 1) if isinstance(raw, dict) else 1
            if type(revision) is int and revision == 1:
                profile = self.settings["gamepad_config"]
                robot = profile["robot"]
                defaults = default_gamepad_config()["robot"]
                if robot["tool_offset_z"] == 160:
                    robot["tool_offset_z"] = defaults["tool_offset_z"]
                if robot["flange_limits"]["z"] == [-150, 250]:
                    robot["flange_limits"]["z"] = list(defaults["flange_limits"]["z"])
                if robot["init_joints"] == [0, -45, 90, 0, 0, 0]:
                    if all(low <= value <= high for value, (low, high) in
                           zip(defaults["init_joints"], robot["joint_limits"])):
                        robot["init_joints"] = list(defaults["init_joints"])
                    else:
                        self.load_errors.append("新手柄初始化默认姿态超出自定义关节范围；已保留原初始化姿态和限制")
                self.settings["gamepad_config"] = validate_gamepad_config(profile)
            if type(revision) is int and revision in (1, 2):
                profile = self.settings["gamepad_config"]
                if profile["control"]["linear_speed_mm_s"] == 10:
                    profile["control"]["linear_speed_mm_s"] = default_gamepad_config()["control"]["linear_speed_mm_s"]
                self.settings["gamepad_config"] = validate_gamepad_config(profile)
                self.settings["gamepad_defaults_revision"] = 3
            if type(revision) is int and revision in (1, 2, 3):
                profile = self.settings["gamepad_config"]
                if profile["robot"]["cmd_speed"] == 10:
                    profile["robot"]["cmd_speed"] = default_gamepad_config()["robot"]["cmd_speed"]
                self.settings["gamepad_config"] = validate_gamepad_config(profile)
                self.settings["gamepad_defaults_revision"] = 4
            if type(revision) is int and revision in (1, 2, 3, 4):
                profile = self.settings["gamepad_config"]
                defaults = default_gamepad_config()["control"]
                for key, previous in (("linear_speed_mm_s", 50), ("angular_speed_deg_s", 5)):
                    if profile["control"][key] == previous:
                        profile["control"][key] = defaults[key]
                self.settings["gamepad_config"] = validate_gamepad_config(profile)
                self.settings["gamepad_defaults_revision"] = 5
            if type(revision) is int and revision in (1, 2, 3, 4, 5):
                profile = self.settings["gamepad_config"]
                if profile["control"]["gripper_speed_percent_s"] == 30:
                    profile["control"]["gripper_speed_percent_s"] = default_gamepad_config()["control"]["gripper_speed_percent_s"]
                self.settings["gamepad_config"] = validate_gamepad_config(profile)
                self.settings["gamepad_defaults_revision"] = 6
            if type(revision) is int and revision in (1, 2, 3, 4, 5, 6):
                profile = self.settings["gamepad_config"]
                if profile["control"]["gripper_speed_percent_s"] == 90:
                    profile["control"]["gripper_speed_percent_s"] = default_gamepad_config()["control"]["gripper_speed_percent_s"]
                self.settings["gamepad_config"] = validate_gamepad_config(profile)
                self.settings["gamepad_defaults_revision"] = 7
            if type(revision) is int and revision in (1, 2, 3, 4, 5, 6, 7):
                profile = self.settings["gamepad_config"]
                defaults = default_gamepad_config()["control"]
                for key, previous in (("linear_speed_mm_s", 100), ("send_interval_s", .05)):
                    if profile["control"][key] == previous:
                        profile["control"][key] = defaults[key]
                self.settings["gamepad_config"] = validate_gamepad_config(profile)
                self.settings["gamepad_defaults_revision"] = 8
        except ValueError as exc:
            self.settings["gamepad_config"] = default_gamepad_config()
            self.load_errors.append(f"手柄配置读取失败：{exc}；已使用默认配置")
        self._normalize_settings()
        raw = self._read(self.base_dir / "points.json", {"points": []})
        try:
            self.points = validate_points(_file_points(raw))
        except (ValueError, TypeError, OverflowError) as exc:
            self.points = []
            self.load_errors.append(f"点位读取失败：{exc}")
        self._logger = logging.getLogger(f"DummyRobot.{id(self)}")
        self._logger.setLevel(logging.INFO)
        self._logger.propagate = False
        self.log_dir = self.base_dir / "logs"
        self.log_dir.mkdir(exist_ok=True)
        self._handler = RotatingFileHandler(self.log_dir / "dummyrobot.log",
                                            maxBytes=5 * 1024 * 1024, backupCount=4,
                                            encoding="utf-8")
        self._handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
        self._logger.addHandler(self._handler)

    def _read(self, path, fallback):
        if not path.exists():
            return fallback
        try:
            with path.open("r", encoding="utf-8-sig") as stream:
                return json.load(stream)
        except (OSError, ValueError) as exc:
            self.load_errors.append(f"{path.name} 读取失败：{exc}")
            return fallback

    def _normalize_settings(self, settings=None):
        settings = self.settings if settings is None else settings
        if type(settings["gamepad_defaults_revision"]) is not int or settings["gamepad_defaults_revision"] < 8:
            settings["gamepad_defaults_revision"] = 8
        # Legacy host/stream preferences are local metadata only. They never
        # select an execution backend or claim the device's actual command mode.
        settings["cartesian_backend"] = "firmware"
        if settings["theme"] not in ("light", "dark"):
            settings["theme"] = "light"
        # The desktop UI connects only through the controller's USB interface.
        settings["connection_type"] = "USB"
        if settings["viewer_mode"] not in ("preview", "actual"):
            settings["viewer_mode"] = "preview"
        for key, low, high, default in (("poll_hz", 1, 50, 5), ("log_limit", 500, 50000, 5000)):
            try:
                settings[key] = max(low, min(high, int(settings[key])))
            except (TypeError, ValueError, OverflowError):
                settings[key] = default
        for key in ("port", "geometry", "window_state", "model_path"):
            if not isinstance(settings[key], str):
                settings[key] = ""
        for key in ("show_grid", "show_axes", "show_target_ghost", "auto_poll", "poll_enabled", "hide_joint_logs", "hide_rgb_logs"):
            if not isinstance(settings[key], bool):
                settings[key] = DEFAULT_SETTINGS[key]

    @staticmethod
    def _write(path, value):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                             dir=path.parent, prefix=path.name + ".",
                                             suffix=".tmp", delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()

    def update_settings(self, **kwargs):
        with self._lock:
            if "gamepad_config" in kwargs:
                # Validate and persist before replacing any in-memory settings.
                # Failed validation/writes cannot leave a partially applied profile.
                profile = validate_gamepad_config(kwargs["gamepad_config"])
                candidate = _Settings(deepcopy(self.settings.copy()))
                for key, value in kwargs.items():
                    if key in DEFAULT_SETTINGS:
                        candidate[key] = profile if key == "gamepad_config" else value
                self._normalize_settings(candidate)
                self._write(self.base_dir / "settings.json", candidate)
                self.settings.clear()
                self.settings.update(candidate)
                return
            for key, value in kwargs.items():
                if key in DEFAULT_SETTINGS:
                    self.settings[key] = value
            self._normalize_settings()
            self._write(self.base_dir / "settings.json", self.settings)

    def save_points(self, points):
        validated = validate_points(points)
        with self._lock:
            self._write(self.base_dir / "points.json", {"schema_version": POINT_SCHEMA_VERSION, "points": validated})
            self.points = validated

    def import_points(self, path):
        with Path(path).open("r", encoding="utf-8-sig") as stream:
            data = json.load(stream)
        return validate_points(_file_points(data))

    def export_points(self, path, points=None):
        validated = validate_points(self.points if points is None else points)
        self._write(Path(path), {"schema_version": POINT_SCHEMA_VERSION, "points": validated})

    def log(self, tag, message):
        self._logger.info("[%s] %s", str(tag), str(message))

    def close(self):
        self._logger.removeHandler(self._handler)
        self._handler.close()
