"""Persistent local data. Reading a profile never writes to a robot."""
from __future__ import annotations

import json
import logging
from logging.handlers import RotatingFileHandler
import math
import os
from pathlib import Path
import tempfile
import threading


DEFAULT_SETTINGS = {
    "theme": "light", "poll_hz": 5, "log_limit": 5000,
    "port": "", "connection_type": "USB", "geometry": "",
    "window_state": "", "viewer_mode": "preview", "show_grid": True,
    "show_axes": True, "auto_poll": True, "poll_enabled": True, "model_path": "",
}
JOINT_LIMITS = [(-175, 175), (-75, 90), (0, 180), (-270, 270),
                (-100, 100), (-180, 180), (-250, 250)]


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
        joints = record.get("joints")
        if not isinstance(joints, (list, tuple)) or len(joints) != 7:
            raise ValueError(f"{name}：需要六个关节角及一个地轨位置")
        values = []
        for axis, (value, limits) in enumerate(zip(joints, JOINT_LIMITS), 1):
            if isinstance(value, bool):
                raise ValueError(f"{name}：J{axis} 必须是数值")
            try:
                value = float(value)
            except (TypeError, ValueError) as exc:
                raise ValueError(f"{name}：J{axis} 必须是数值") from exc
            if not math.isfinite(value) or not limits[0] <= value <= limits[1]:
                raise ValueError(f"{name}：J{axis} 超出范围 {limits[0]}～{limits[1]}")
            values.append(value)
        gripper = float(record.get("gripper", 100))
        speed = float(record.get("speed", 35))
        if not math.isfinite(gripper) or not 0 <= gripper <= 100:
            raise ValueError(f"{name}：夹爪开度必须在 0～100")
        if not math.isfinite(speed) or not 1 <= speed <= 100:
            raise ValueError(f"{name}：速度必须在 1～100")
        result.append({"name": name, "joints": values,
                       "gripper": gripper, "speed": speed})
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
        self.settings = dict(DEFAULT_SETTINGS)
        raw = self._read(self.base_dir / "settings.json", {})
        if isinstance(raw, dict):
            for key in DEFAULT_SETTINGS:
                if key in raw:
                    self.settings[key] = raw[key]
        self._normalize_settings()
        raw = self._read(self.base_dir / "points.json", {"points": []})
        try:
            self.points = validate_points(raw.get("points", []) if isinstance(raw, dict) else raw)
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

    def _normalize_settings(self):
        if self.settings["theme"] not in ("light", "dark"):
            self.settings["theme"] = "light"
        if self.settings["connection_type"] not in ("USB", "UART4"):
            self.settings["connection_type"] = "USB"
        if self.settings["viewer_mode"] not in ("preview", "actual"):
            self.settings["viewer_mode"] = "preview"
        for key, low, high, default in (("poll_hz", 1, 10, 5), ("log_limit", 500, 50000, 5000)):
            try:
                self.settings[key] = max(low, min(high, int(self.settings[key])))
            except (TypeError, ValueError, OverflowError):
                self.settings[key] = default
        for key in ("port", "geometry", "window_state", "model_path"):
            if not isinstance(self.settings[key], str):
                self.settings[key] = ""
        for key in ("show_grid", "show_axes", "auto_poll", "poll_enabled"):
            if not isinstance(self.settings[key], bool):
                self.settings[key] = DEFAULT_SETTINGS[key]

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
            for key, value in kwargs.items():
                if key in DEFAULT_SETTINGS:
                    self.settings[key] = value
            self._normalize_settings()
            self._write(self.base_dir / "settings.json", self.settings)

    def save_points(self, points):
        validated = validate_points(points)
        with self._lock:
            self._write(self.base_dir / "points.json", {"schema_version": 1, "points": validated})
            self.points = validated

    def import_points(self, path):
        with Path(path).open("r", encoding="utf-8-sig") as stream:
            data = json.load(stream)
        if isinstance(data, dict) and data.get("schema_version", 1) != 1:
            raise ValueError("不支持该点位文件版本")
        return validate_points(data.get("points", []) if isinstance(data, dict) else data)

    def export_points(self, path, points=None):
        validated = validate_points(self.points if points is None else points)
        self._write(Path(path), {"schema_version": 1, "points": validated})

    def log(self, tag, message):
        self._logger.info("[%s] %s", str(tag), str(message))

    def close(self):
        self._logger.removeHandler(self._handler)
        self._handler.close()
