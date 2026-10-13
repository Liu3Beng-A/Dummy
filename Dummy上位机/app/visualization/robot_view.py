"""Qt widget wrapping a completely local URDF viewer."""

from __future__ import annotations

import json
from numbers import Integral
import sys
from pathlib import Path

from PySide6.QtCore import QObject, QTimer, QUrl, Signal, Slot
from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget

from .asset_server import AssetServer
from .mapping import REST_POSE, cartesian_marker, firmware_to_urdf, normalize_pose


class _Bridge(QObject):
    state_changed = Signal(str)
    rgb_changed = Signal(str)
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


class RobotView(QWidget):
    ready = Signal()
    status_changed = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(280, 250)
        self._ready = False
        self._closed = False
        self._server = None
        self.web_view = None
        self._bridge = _Bridge(self)
        self._target_pose = list(REST_POSE)
        self._actual_pose = list(REST_POSE)
        self._target_gripper = 100.0
        self._actual_gripper = None
        self._commanded_gripper = None
        self._cartesian = None
        self._cartesian_frame = "base"
        self._rgb_frame = ((0, 0, 0),) * 24
        self._rgb_light_on = None
        self._display_mode = "preview"
        self._state = {
            "mode": "preview", "theme": "light", "stale": True,
            "actualOnly": False, "feedbackPaused": False,
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
        self.ready.emit()
        self.status_changed.emit("本地 URDF 与 11 个网格加载完成")

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
        self._state["target"] = firmware_to_urdf(self._target_pose, self._target_gripper, preview=True)
        actual_only = self._state["actualOnly"]
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
        if self._ready:
            self._bridge.state_changed.emit(json.dumps(self._state, ensure_ascii=False, allow_nan=False))

    def set_target_pose(self, joints, gripper_pct=100.0):
        pose = normalize_pose(joints, preview=True)
        # Validate before storing or transmitting to the viewer.
        firmware_to_urdf(pose, gripper_pct, preview=True)
        self._target_pose = pose
        self._target_gripper = float(gripper_pct)
        self._sync()

    def set_rgb_frame(self, colors, *, light_on=None):
        """Update the model's diffuser without touching poses or telemetry age."""
        if self._closed:
            return
        if light_on is not None and not isinstance(light_on, bool):
            raise ValueError("灯环开灯状态须为布尔值")
        try:
            frame = tuple(tuple(color) for color in colors)
        except TypeError as error:
            raise ValueError("灯环帧须包含 24 组 RGB 整数") from error
        if len(frame) != 24 or any(len(color) != 3 or any(
                isinstance(channel, bool) or not isinstance(channel, Integral)
                or not 0 <= channel <= 255 for channel in color) for color in frame):
            raise ValueError("灯环帧须包含 24 组 0..255 的 RGB 整数")
        frame = tuple(tuple(int(channel) for channel in color) for color in frame)
        if frame != self._rgb_frame or light_on != self._rgb_light_on:
            self._rgb_frame = frame
            self._rgb_light_on = light_on
            self._emit_rgb_frame()

    def _emit_rgb_frame(self):
        if self._ready and not self._closed:
            payload = self._rgb_frame if self._rgb_light_on is None else {
                "colors": self._rgb_frame, "light_on": self._rgb_light_on,
            }
            self._bridge.rgb_changed.emit(json.dumps(payload, allow_nan=False))

    def set_target_gripper(self, gripper_pct):
        """Update the display opening while preserving the last joint target.

        MoveL has a Cartesian marker without local IK, so changing its gripper
        preview must not manufacture a new robot joint pose or send commands.
        """
        firmware_to_urdf(self._target_pose, gripper_pct, preview=True)
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
