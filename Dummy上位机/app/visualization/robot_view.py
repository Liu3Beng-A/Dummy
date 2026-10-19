"""Qt widget wrapping a completely local URDF viewer."""

from __future__ import annotations

import json
import math
import time
from numbers import Integral
import sys
from pathlib import Path

from PySide6.QtCore import QEvent, QObject, QTimer, QUrl, Qt, Signal, Slot
from PySide6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget

from .asset_server import AssetServer
from .mapping import REST_POSE, cartesian_marker, firmware_to_urdf, normalize_pose, target_pose_matches


class _Bridge(QObject):
    state_changed = Signal(str)
    rgb_changed = Signal(str)
    telemetry_changed = Signal(str)
    presentation_changed = Signal(str)
    model_status_changed = Signal(str)
    def __init__(self, view):
        super().__init__(view)
        self.view = view

    @Slot()
    def viewerReady(self):
        self.view._on_ready()

    @Slot(str)
    def viewerError(self, message):
        self.view._show_error(message)

    @Slot(str)
    def viewerStatus(self, message):
        self.view.status_changed.emit(message)

    @Slot(str)
    def viewerDiagnostic(self, payload):
        try:
            event = json.loads(payload)
        except (TypeError, ValueError):
            return
        if isinstance(event, dict) and not self.view._closed:
            self.view.diagnostic_event.emit(event)

    @Slot()
    def viewerRecovered(self):
        if self.view._closed:
            return
        self.view._message.hide()
        self.view.status_changed.emit("三维显卡上下文已恢复")
        self.view._sync_presentation(force=True)

    @Slot(str)
    def viewerPerformance(self, payload):
        try:
            data = json.loads(payload)
            if not isinstance(data, dict):
                return
            for key in ("fps", "frame_ms_p95"):
                value = data.get(key)
                if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))
                                          or not math.isfinite(value) or value < 0):
                    return
        except (ValueError, TypeError):
            return
        self.view._render_performance = data
        self.view._render_performance_at = time.perf_counter()


class RobotView(QWidget):
    ready = Signal()
    status_changed = Signal(str)
    diagnostic_event = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(280, 250)
        self._ready = False
        self._closed = False
        self._server = None
        self.web_view = None
        self._bridge = _Bridge(self)
        self._telemetry_metrics = {"connected": False}
        self._render_performance = {}
        self._render_performance_at = None
        self._presentation_mode = "active"
        self._presentation_window = None
        self._presentation_pending = False
        self._scene_message = "正在加载本地 URDF…"
        self.status_changed.connect(self._set_scene_message)
        QApplication.instance().applicationStateChanged.connect(self._schedule_presentation)
        self._target_pose = list(REST_POSE)
        self._actual_pose = list(REST_POSE)
        self._target_gripper = 100.0
        self._actual_gripper = None
        self._commanded_gripper = None
        self._cartesian = None
        self._cartesian_frame = "base"
        self._rgb_frame = ((0, 0, 0),) * 24
        self._rgb_light_on = None
        self._rgb_effect_mode = None
        self._display_mode = "preview"
        self._state = {
            "mode": "preview", "theme": "light", "stale": True,
            "actualOnly": False, "feedbackPaused": False,
            "rgbPreview": False, "targetEnabled": True, "targetMatched": False,
            "actualKnown": False, "actualGripperKnown": False,
            "highlight": None, "camera": "isometric", "cameraSeq": 0,
            "grid": True, "axes": True, "visibilitySeq": 0,
        }
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self._message = QLabel("正在加载本地 URDF 模型…", self)
        self._message.setWordWrap(True)
        self._message.setStyleSheet("padding: 14px; color: #64748b; background: #edf2f8;")
        layout.addWidget(self._message)
        self._timeout = QTimer(self)
        self._timeout.setSingleShot(True)
        self._timeout.timeout.connect(lambda: self._show_error(
            "三维初始化超时。请检查 WebEngine、显卡驱动和本地模型资源。"))
        try:
            from PySide6.QtWebChannel import QWebChannel
            from PySide6.QtWebEngineCore import (QWebEnginePage, QWebEngineProfile,
                                                 QWebEngineSettings,
                                                 QWebEngineUrlRequestInterceptor)
            from PySide6.QtWebEngineWidgets import QWebEngineView

            resources = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parents[2])) / "resources"
            model = resources / "models/DummyRobot_URDF/urdf/DummyRobot_URDF.urdf"
            if not model.is_file():
                raise FileNotFoundError(f"缺少模型：{model}")
            self._server = AssetServer(resources)
            allowed_url = self._server.base_url

            class LocalOnly(QWebEngineUrlRequestInterceptor):
                def interceptRequest(self, info):
                    url = info.requestUrl().toString()
                    if not (url.startswith(allowed_url) or url.startswith("qrc:")
                            or url in {"about:blank", "data:,"}):
                        info.block(True)

            class LocalPage(QWebEnginePage):
                def acceptNavigationRequest(self, url, _type, _main):
                    return url.toString().startswith(allowed_url)

                def javaScriptConsoleMessage(page_self, level, message, line, source):
                    if level == QWebEnginePage.JavaScriptConsoleMessageLevel.ErrorMessageLevel:
                        self.status_changed.emit(f"三维脚本：{message}")

            self._profile = QWebEngineProfile(self)
            self._interceptor = LocalOnly(self._profile)
            self._profile.setUrlRequestInterceptor(self._interceptor)
            self._page = LocalPage(self._profile, self)
            self._channel = QWebChannel(self._page)
            self._channel.registerObject("robotBridge", self._bridge)
            self._page.setWebChannel(self._channel)
            self.web_view = QWebEngineView(self)
            self.web_view.setPage(self._page)
            settings = self.web_view.settings()
            settings.setAttribute(QWebEngineSettings.WebAttribute.JavascriptCanOpenWindows, False)
            settings.setAttribute(QWebEngineSettings.WebAttribute.LocalContentCanAccessRemoteUrls, False)
            layout.addWidget(self.web_view, 1)
            self.web_view.loadFinished.connect(self._load_finished)
            self.web_view.setUrl(QUrl(allowed_url + "viewer/index.html"))
            self._timeout.start(25000)
        except Exception as error:
            self._show_error(str(error))

    def _load_finished(self, ok):
        if not ok:
            self._show_error("本地三维页面加载失败。")

    def _on_ready(self):
        if self._closed or self._ready:
            return
        self._timeout.stop()
        self._ready = True
        self._message.hide()
        self._sync()
        self._emit_rgb_frame()
        self._emit_telemetry_metrics()
        self._sync_presentation(force=True)
        self.ready.emit()
        self.status_changed.emit("本地 URDF 与 12 个网格加载完成")

    def _show_error(self, message):
        if self._closed:
            return
        self._timeout.stop()
        self._message.setText("三维视图暂不可用\n" + message)
        self._message.show()
        self.status_changed.emit("三维视图：" + message)

    def _sync(self):
        if self._closed:
            return
        fresh = (self._state["actualKnown"] and not self._state["stale"]
                 and not self._state["feedbackPaused"])
        self._state["targetMatched"] = bool(fresh and target_pose_matches(
            self._target_pose, self._actual_pose, was_matched=self._state["targetMatched"],
            target_gripper=self._target_gripper, actual_gripper=self._actual_gripper))
        self._state["target"] = firmware_to_urdf(self._target_pose, self._target_gripper, preview=True)
        actual_only = self._state["actualOnly"]
        if actual_only:
            self._state["highlight"] = None
        self._state["mode"] = "actual" if actual_only else self._display_mode
        gripper = self._actual_gripper
        if gripper is None:
            # Gamepad fingers follow written commands without claiming telemetry.
            if actual_only:
                gripper = self._commanded_gripper if self._commanded_gripper is not None else 100.0
            else:
                gripper = self._target_gripper
        self._state["commandedGripper"] = self._commanded_gripper if actual_only else None
        self._state["actual"] = firmware_to_urdf(self._actual_pose, gripper)
        self._state["marker"] = (cartesian_marker(self._cartesian, self._target_pose[6] if self._cartesian_frame == "base" else 0.)
                                 if self._cartesian is not None and not actual_only else None)
        if self._ready and self._presentation_mode != "hidden":
            self._bridge.state_changed.emit(json.dumps(self._state, ensure_ascii=False, allow_nan=False))

    def set_target_pose(self, joints, gripper_pct=100.0):
        pose = normalize_pose(joints, preview=True)
        # Validate before storing or transmitting to the viewer.
        firmware_to_urdf(pose, gripper_pct, preview=True)
        if pose != self._target_pose or float(gripper_pct) != self._target_gripper:
            self._state["targetMatched"] = False
        self._target_pose = pose
        self._target_gripper = float(gripper_pct)
        self._sync()

    def set_rgb_frame(self, colors, *, light_on=None, effect_mode=None):
        """Update the model's diffuser without touching poses or telemetry age."""
        if self._closed:
            return
        if light_on is not None and not isinstance(light_on, bool):
            raise ValueError("灯环开灯状态须为布尔值")
        if effect_mode is not None and (isinstance(effect_mode, bool)
                or not isinstance(effect_mode, Integral) or not 0 <= effect_mode < 17):
            raise ValueError("灯效编号须为 0..16 的整数或 None")
        try:
            frame = tuple(tuple(color) for color in colors)
        except TypeError as error:
            raise ValueError("灯环帧须包含 24 组 RGB 整数") from error
        if len(frame) != 24 or any(len(color) != 3 or any(
                isinstance(channel, bool) or not isinstance(channel, Integral)
                or not 0 <= channel <= 255 for channel in color) for color in frame):
            raise ValueError("灯环帧须包含 24 组 0..255 的 RGB 整数")
        frame = tuple(tuple(int(channel) for channel in color) for color in frame)
        effect_mode = int(effect_mode) if effect_mode is not None else None
        if (frame != self._rgb_frame or light_on != self._rgb_light_on
                or effect_mode != self._rgb_effect_mode):
            self._rgb_frame = frame
            self._rgb_light_on = light_on
            self._rgb_effect_mode = effect_mode
            self._emit_rgb_frame()

    def _emit_rgb_frame(self):
        if self._ready and not self._closed and self._presentation_mode != "hidden":
            payload = self._rgb_frame
            if self._rgb_light_on is not None or self._rgb_effect_mode is not None:
                payload = {"colors": self._rgb_frame}
                if self._rgb_light_on is not None:
                    payload["light_on"] = self._rgb_light_on
                if self._rgb_effect_mode is not None:
                    payload["effect_mode"] = self._rgb_effect_mode
            self._bridge.rgb_changed.emit(json.dumps(payload, allow_nan=False))

    def set_performance_metrics(self, metrics):
        """Independent low-rate HUD updates must not replay joint/material state."""
        if not self._closed:
            self._telemetry_metrics = dict(metrics)
            self._emit_telemetry_metrics()

    def _emit_telemetry_metrics(self):
        if self._ready and not self._closed and self._presentation_mode != "hidden":
            self._bridge.telemetry_changed.emit(json.dumps(self._telemetry_metrics, allow_nan=False))

    def _set_scene_message(self, message):
        self._scene_message = str(message)
        if self._ready and not self._closed and self._presentation_mode != "hidden":
            self._bridge.model_status_changed.emit(self._scene_message)

    def _schedule_presentation(self, *_args):
        if self._closed or self._presentation_pending:
            return
        self._presentation_pending = True
        QTimer.singleShot(0, self, self._sync_presentation)

    def _sync_presentation(self, *, force=False):
        self._presentation_pending = False
        if self._closed or self.web_view is None:
            return
        window = self.window()
        if window is not self._presentation_window:
            if self._presentation_window is not None:
                self._presentation_window.removeEventFilter(self)
            self._presentation_window = window
            window.installEventFilter(self)
        mode = ("hidden" if not self.isVisible() or window.isMinimized()
                else "active" if QApplication.applicationState() == Qt.ApplicationState.ApplicationActive
                else "background")
        if mode == self._presentation_mode and not force:
            return
        self._presentation_mode = mode
        if self._ready:
            # Seed the current snapshot before waking the browser. Hidden views
            # retain data locally instead of queuing obsolete WebChannel frames.
            if mode != "hidden":
                self._sync()
                self._emit_rgb_frame()
                self._emit_telemetry_metrics()
                self._bridge.model_status_changed.emit(self._scene_message)
            self._bridge.presentation_changed.emit(mode)

    def showEvent(self, event):
        super().showEvent(event)
        self._schedule_presentation()

    def hideEvent(self, event):
        super().hideEvent(event)
        self._schedule_presentation()

    def eventFilter(self, watched, event):
        if watched is self._presentation_window and event.type() in (
                QEvent.Type.Show, QEvent.Type.Hide, QEvent.Type.WindowStateChange,
                QEvent.Type.WindowActivate, QEvent.Type.WindowDeactivate):
            self._schedule_presentation()
        return False

    def set_rgb_preview(self, enabled):
        """Allow an explicitly labelled local lamp model when no pose is known."""
        if self._closed:
            return
        value = bool(enabled)
        if self._state["rgbPreview"] != value:
            self._state["rgbPreview"] = value
            self._sync()

    def set_target_gripper(self, gripper_pct):
        """Update the display opening while preserving the last joint target.

        MoveL has a Cartesian marker without local IK, so changing its gripper
        preview must not manufacture a new robot joint pose or send commands.
        """
        firmware_to_urdf(self._target_pose, gripper_pct, preview=True)
        if float(gripper_pct) != self._target_gripper:
            self._state["targetMatched"] = False
        self._target_gripper = float(gripper_pct)
        self._sync()

    def set_commanded_gripper(self, gripper_pct):
        """Display the last written gamepad opening in actual-only mode.

        Preserve joint telemetry, its age, and the interface's preview opening.
        None clears the command illustration when the connection is lost.
        """
        if gripper_pct is not None:
            firmware_to_urdf(self._actual_pose, gripper_pct)
        self._commanded_gripper = None if gripper_pct is None else float(gripper_pct)
        self._sync()

    def set_actual_pose(self, joints, gripper_pct=None):
        pose = normalize_pose(joints, preview=False)
        if gripper_pct is not None:
            firmware_to_urdf(pose, gripper_pct)
            self._actual_gripper = float(gripper_pct)
        self._actual_pose = pose
        self._state["actualKnown"] = True
        self._state["actualGripperKnown"] = self._actual_gripper is not None
        self._state["stale"] = False
        self._sync()

    def set_display_mode(self, mode):
        if mode not in {"preview", "actual"}:
            raise ValueError("Display mode must be preview or actual")
        self._display_mode = mode
        self._sync()

    def set_actual_only(self, enabled):
        """Temporarily hide all target displays without inventing telemetry."""
        self._state["actualOnly"] = bool(enabled)
        self._sync()

    def set_target_visible(self, visible):
        """Master ghost preference; matching fresh feedback hides it automatically."""
        if self._closed:
            return
        visible = bool(visible)
        if self._state["targetEnabled"] != visible:
            self._state["targetEnabled"] = visible
            self._sync()

    def set_feedback_paused(self, paused):
        """Annotate retained telemetry; software targets never refresh it."""
        self._state["feedbackPaused"] = bool(paused)
        self._sync()

    def set_target_cartesian(self, pose):
        if pose is not None:
            cartesian_marker(pose)
        self._cartesian_frame = "base"
        self._cartesian = list(pose) if pose is not None else None
        self._sync()

    def set_world_cartesian_target(self, pose):
        if pose is not None:
            cartesian_marker(pose)
        self._cartesian_frame = "world"
        self._cartesian = list(pose) if pose is not None else None
        self._sync()

    set_cartesian_target = set_target_cartesian

    def set_theme(self, theme):
        if theme not in {"light", "dark"}:
            raise ValueError("Theme must be light or dark")
        self._state["theme"] = theme
        self._sync()

    def set_stale(self, stale):
        self._state["stale"] = bool(stale)
        self._sync()

    def set_actual_known(self, known):
        self._state["actualKnown"] = bool(known)
        if not known:
            self._actual_gripper = None
            self._state["actualGripperKnown"] = False
        self._sync()

    def focus_joint(self, joint):
        names = {**{f"J{i}": f"Joint{i}" for i in range(1, 7)},
                 **{f"Joint{i}": f"Joint{i}" for i in range(1, 7)},
                 "J7": "rail", "rail": "rail", "J8": "gripper", "gripper": "gripper"}
        if isinstance(joint, int):
            joint = f"J{joint}"
        self._state["highlight"] = names.get(str(joint))
        self._sync()

    def reset_view(self):
        self.set_camera_view("isometric")

    reset_camera = reset_view

    def set_camera_view(self, name):
        if name not in {"front", "top", "side", "isometric", "right"}:
            raise ValueError("Unknown camera view")
        self._state["camera"] = name
        self._state["cameraSeq"] += 1
        self._sync()

    set_camera = set_camera_view

    def set_preview_visible(self, visible):
        self.set_display_mode("preview" if visible else "actual")

    def set_follow_actual(self, follow):
        self.set_display_mode("actual" if follow else "preview")

    def set_grid_visible(self, visible):
        self._state["grid"] = bool(visible)
        self._state["visibilitySeq"] += 1
        self._sync()

    def set_axes_visible(self, visible):
        self._state["axes"] = bool(visible)
        self._state["visibilitySeq"] += 1
        self._sync()

    set_show_grid = set_grid_visible
    set_show_axes = set_axes_visible

    def cleanup(self):
        if self._closed:
            return
        self._closed = True
        self._timeout.stop()
        if self.web_view is not None:
            self.web_view.stop()
            self._page.setWebChannel(None)
        if self._server is not None:
            self._server.close()

    def closeEvent(self, event):
        self.cleanup()
        super().closeEvent(event)
