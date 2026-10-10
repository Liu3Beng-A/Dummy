"""Explicit Qt/SDL acquisition. This module never controls a serial transport."""
from __future__ import annotations

from copy import deepcopy
import importlib
import os
import time

from PySide6.QtCore import QCoreApplication, QObject, QThread, QTimer, Qt, Signal

from .config import default_config, validate_config
from .input import decode_gamepad


class GamepadReader(QObject):
    """Collect one selected joystick on the Qt main thread, at about 60Hz.

    Construction/set_config do not import SDL, initialize it, or enumerate
    devices. start is the explicit hardware-inspection boundary. A bound device
    is identified by SDL instance ID; removing it requires another explicit
    start, rather than adopting a different joystick or ambiguous same-GUID
    device. Existing held buttons establish a baseline, not a new press.
    """

    sampled = Signal(dict)
    status_changed = Signal(dict)
    _ACTIONS = ("start", "stop", "rest")

    def __init__(self, parent=None, *, pygame_module=None, clock=None):
        super().__init__(parent)
        self._provided_pg = pygame_module
        self._clock = clock or time.monotonic
        self._config = default_config()
        self._timer = QTimer(self)
        self._timer.setInterval(17)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.timeout.connect(self._poll)
        self._pg = self._joystick = None
        self._instance_id = None
        self._name = "未连接"
        self._guid = ""
        self._devices = []
        self._previous_buttons = None
        self._running = self._connected = self._closed = False
        self._requires_restart = False
        self._owns_display = self._owns_joystick = False
        self._error = ""
        self._last_status = None

    def _check_thread(self):
        app = QCoreApplication.instance()
        if QThread.currentThread() != self.thread() or (app is not None and self.thread() != app.thread()):
            raise RuntimeError("手柄采集必须在 Qt 主线程运行")

    def status(self):
        return deepcopy({"running": self._running, "connected": self._connected,
                         "name": self._name, "guid": self._guid, "instance_id": self._instance_id,
                         "devices": self._devices, "error": self._error,
                         "requires_restart": self._requires_restart})

    def _emit_status(self):
        current = self.status()
        if current != self._last_status:
            self._last_status = deepcopy(current)
            self.status_changed.emit(current)

    def _emit_sample(self, decoded=None, pressed=(), raw=None):
        snapshot = {"connected": self._connected, "decoded": decoded,
                    "pressed": list(pressed), "raw": raw or {"name": "未连接", "axes": [], "buttons": [], "hats": []},
                    "error": self._error, "timestamp": float(self._clock())}
        self.sampled.emit(deepcopy(snapshot))
        self._emit_status()

    def set_config(self, config):
        self._check_thread()
        validated = validate_config(config)
        if validated == self._config:
            return
        if self._running:
            self.stop()
        self._config = validated

    def start(self, config=None):
        self._check_thread()
        if self._closed:
            return False
        try:
            validated = validate_config(self._config if config is None else config)
        except ValueError as exc:
            self.stop()
            self._error = str(exc)
            self._emit_sample()
            return False
        if self._running and validated == self._config and not self._requires_restart:
            return True
        self.stop()
        self._config = validated
        self._error = ""
        self._requires_restart = False
        try:
            # These must precede importing/initializing SDL. No video/audio
            # driver override is installed, and pygame.init() is never called.
            os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
            os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
            self._pg = self._provided_pg or importlib.import_module("pygame")
            if not self._pg.display.get_init():
                self._pg.display.init()
                self._owns_display = True
            if self._pg.display.get_surface() is None:
                self._pg.display.set_mode((1, 1), self._pg.HIDDEN)
                self._owns_display = True
            if not self._pg.joystick.get_init():
                self._pg.joystick.init()
                self._owns_joystick = True
            self._running = True
            self._discover(select=True)
            self._poll()
            self._timer.start()
            self._emit_status()
            return True
        except ImportError:
            self._error = "未安装 pygame；请安装手柄支持依赖后重新检查"
        except Exception as exc:
            self._error = f"手柄采集启动失败：{exc}"
        self._cleanup_sdl()
        self._running = self._connected = False
        self._devices = []
        self._emit_sample()
        return False

    def _discover(self, *, select):
        candidates = []
        try:
            for index in range(self._pg.joystick.get_count()):
                joy = self._pg.joystick.Joystick(index)
                joy.init()
                try:
                    data = {"index": index, "name": str(joy.get_name()),
                            "guid": str(joy.get_guid()).strip().lower(),
                            "instance_id": int(joy.get_instance_id())}
                except Exception:
                    if joy is not self._joystick:
                        joy.quit()
                    raise
                candidates.append((joy, data))
            self._devices = [data for _, data in candidates]
            if select and self._joystick is None and not self._requires_restart:
                cfg = self._config["gamepad"]
                matches = [(joy, data) for joy, data in candidates
                           if data["guid"] == cfg["device_guid"]] if cfg["device_guid"] else candidates
                chosen = next(((joy, data) for joy, data in matches if data["index"] == cfg["device_index"]), None)
                if chosen is None and cfg["device_guid"] and matches:
                    chosen = matches[0]
                if chosen is not None:
                    self._joystick, data = chosen
                    self._instance_id = data["instance_id"]
                    self._name, self._guid = data["name"], data["guid"]
                    self._previous_buttons = None
        finally:
            # pygame may return a cached wrapper for the bound instance; never
            # quit it while refreshing the list of other attached devices.
            for joy, data in candidates:
                if data["instance_id"] != self._instance_id:
                    try:
                        joy.quit()
                    except Exception:
                        pass

    def _drop_device(self, reason):
        if self._joystick is not None:
            try:
                self._joystick.quit()
            except Exception:
                pass
        self._joystick = None
        self._instance_id = None
        self._name, self._guid = "未连接", ""
        self._previous_buttons = None
        self._connected = False
        self._requires_restart = True
        self._error = reason

    def _poll(self):
        if not self._running or self._pg is None:
            return
        self._check_thread()
        try:
            self._pg.event.pump()
            events = self._pg.event.get()
            removed = self._pg.JOYDEVICEREMOVED
            added = self._pg.JOYDEVICEADDED
            # Removal beats every read/button/add event in this same sample.
            if any(event.type == removed and getattr(event, "instance_id", getattr(event, "joy", None)) == self._instance_id
                   for event in events) and self._joystick is not None:
                self._drop_device("所选手柄已断开；请重新检查设备")
                self._discover(select=False)
                self._emit_sample()
                return
            if any(event.type in (removed, added) for event in events):
                self._discover(select=self._joystick is None and not self._requires_restart)
            if self._joystick is None:
                self._connected = False
                if not self._requires_restart:
                    self._error = "未检测到所选手柄"
                self._emit_sample()
                return
            joy = self._joystick
            raw = {"name": str(joy.get_name()),
                   "axes": [joy.get_axis(i) for i in range(joy.get_numaxes())],
                   "buttons": [joy.get_button(i) for i in range(joy.get_numbuttons())],
                   "hats": [joy.get_hat(i) for i in range(joy.get_numhats())]}
            try:
                decoded = decode_gamepad(raw["axes"], raw["buttons"], raw["hats"], self._config["gamepad"])
            except ValueError as exc:
                self._connected = False
                self._previous_buttons = None
                self._error = str(exc)
                self._emit_sample(raw=raw)
                return
            states = decoded["buttons"]
            pressed = [] if self._previous_buttons is None else [
                action for action in self._ACTIONS if states[action] and not self._previous_buttons[action]]
            self._previous_buttons = deepcopy(states)
            self._connected, self._error = True, ""
            self._emit_sample(decoded, pressed, raw)
        except Exception as exc:
            self._drop_device(f"手柄采集失败：{exc}；请重新检查设备")
            self._emit_sample()

    def _cleanup_sdl(self):
        self._timer.stop()
        if self._joystick is not None:
            try:
                self._joystick.quit()
            except Exception:
                pass
        self._joystick = None
        self._instance_id = None
        self._name, self._guid = "未连接", ""
        self._previous_buttons = None
        if self._pg is not None:
            for owned, subsystem in ((self._owns_joystick, self._pg.joystick),
                                     (self._owns_display, self._pg.display)):
                if owned:
                    try:
                        subsystem.quit()
                    except Exception:
                        pass
        self._owns_display = self._owns_joystick = False
        self._pg = None

    def stop(self):
        self._check_thread()
        changed = self._running or self._connected
        self._cleanup_sdl()
        self._running = self._connected = False
        self._devices = []
        self._requires_restart = False
        self._error = ""
        if changed:
            self._emit_sample()

    def close(self):
        self.stop()
        self._closed = True
