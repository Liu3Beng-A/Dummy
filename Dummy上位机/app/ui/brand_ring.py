"""Transparent header diffuser fed by the model's shared RGB frame stream."""
import math
from numbers import Integral

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QConicalGradient, QPainter, QPainterPath, QPen, QRadialGradient
from PySide6.QtWidgets import QWidget

from app.ui.rgb_ring import _diffuse_colors


class BrandRing(QWidget):
    """Draw the existing annulus; own no simulator, timer or device connection."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("WorkstationMark")
        self.setFixedSize(72, 72)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setAutoFillBackground(False)
        self.setToolTip("灯环 · 等待设备配置与状态回读")
        self.setAccessibleName("与模型同步的灯环")
        self._light = True
        self.rgb_frame = ((0, 0, 0),) * 24
        self.rgb_light_on = None
        self.rgb_effect_mode = None
        self._displayed_frame = None
        self._last_frame_time = None
        self._angular_colors = None

    def set_rgb_frame(self, colors, *, light_on=None, effect_mode=None):
        """Accept the exact same immutable frame/power/mode as RobotView."""
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
        self.rgb_frame = tuple(tuple(int(channel) for channel in color) for color in frame)
        self.rgb_light_on = light_on
        self.rgb_effect_mode = int(effect_mode) if effect_mode is not None else None
        if self._displayed_frame is None:
            self._displayed_frame = [list(color) for color in self.rgb_frame]
            self._angular_colors = None
            self.update()

    def advance_rgb_frame(self, now):
        """Use the existing UI render tick, matching the model's 60ms damping.

        Only display interpolation happens here; color/effect phase comes from
        the shared input. A hidden-window pause snaps, as the model does.
        """
        dt = 0.26 if self._last_frame_time is None else max(0., now - self._last_frame_time)
        self._last_frame_time = now
        if self._displayed_frame is None:
            return
        factor = 1. if dt > .25 else 1. - math.exp(-dt / .06)
        changed = False
        for shown, goal in zip(self._displayed_frame, self.rgb_frame):
            for channel in range(3):
                previous = shown[channel]
                damped = previous + (goal[channel] - previous) * factor
                shown[channel] = goal[channel] if abs(goal[channel] - damped) <= .002 else damped
                changed |= shown[channel] != previous
        if changed:
            self._angular_colors = None
            self.update()

    def _light_colors(self):
        if self._angular_colors is None:
            # Same circular Gaussian scattering as the model (0.8 sources).
            scattered = _diffuse_colors(self._displayed_frame or self.rgb_frame)
            colors = []
            for rgb in scattered:
                peak = max(rgb) / 255.
                # A compact 2D opal cover: white at rest, colored emission
                # takes over smoothly. No separate effect/brightness is applied.
                strength = min(1., peak ** .45) if peak > 0 else 0.
                colors.append(QColor(*(round(255 * ((1. - strength) * .88
                    + strength * (.10 + .90 * value / (255 * peak))))
                    if peak > 0 else 224 for value in rgb)))
            self._angular_colors = colors
        return self._angular_colors

    def set_theme(self, theme):
        light = theme != "dark"
        if light != self._light:
            self._light = light
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        side = min(self.width(), self.height())
        center = QPointF(self.width() / 2, self.height() / 2)
        # Same center, silhouette and band width as the approved 1254px icon.
        outer, inner = side * 500 / 1254, side * 356 / 1254
        outer_rect = QRectF(center.x() - outer, center.y() - outer, 2 * outer, 2 * outer)
        inner_rect = QRectF(center.x() - inner, center.y() - inner, 2 * inner, 2 * inner)
        ring = QPainterPath()
        ring.setFillRule(Qt.FillRule.OddEvenFill)
        ring.addEllipse(outer_rect)
        ring.addEllipse(inner_rect)
        painter.setPen(Qt.PenStyle.NoPen)
        angular = QConicalGradient(center, 90.)
        colors = self._light_colors()
        # A top view of model light: source 0 is rear (-X), placed at the top;
        # subsequent sources go clockwise (+Y). Reverse Qt's CCW stops.
        for sample in range(len(colors)):
            angular.setColorAt(sample / len(colors), colors[-sample % len(colors)])
        angular.setColorAt(1., colors[0])
        painter.fillPath(ring, angular)
        if not self._light:
            return

        # The small radial fade stays inside the band. Multiplication keeps
        # the hue and placement of the shared light frame intact.
        edge = QColor("#d4dae2")
        soft = QColor(edge)
        soft.setAlpha(32)
        clear = QColor(edge)
        clear.setAlpha(0)
        fade = QRadialGradient(center, outer)
        fade.setColorAt(0, clear)
        fade.setColorAt(inner / outer, soft)
        fade.setColorAt((inner + .65) / outer, clear)
        fade.setColorAt((outer - .65) / outer, clear)
        fade.setColorAt(1, soft)
        painter.setCompositionMode(QPainter.CompositionMode.CompositionMode_Multiply)
        painter.fillPath(ring, fade)
        painter.setCompositionMode(QPainter.CompositionMode.CompositionMode_SourceOver)
        # Inset both strokes into the band; neither changes the silhouette
        # nor risks a clipped outline at a control boundary.
        width = .55
        inset = width / 2
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(edge, width))
        painter.drawEllipse(outer_rect.adjusted(inset, inset, -inset, -inset))
        painter.drawEllipse(inner_rect.adjusted(-inset, -inset, inset, inset))
