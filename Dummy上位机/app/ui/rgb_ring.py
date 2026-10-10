"""A continuous opal diffuser over a serial-independent 24-channel RGB ring."""
from __future__ import annotations

import math
from functools import lru_cache
from numbers import Integral

from PySide6.QtCore import QPointF, QRectF, QSize, Qt
from PySide6.QtGui import (QColor, QConicalGradient, QFont, QFontMetrics, QImage,
                          QPainter, QPen, QRadialGradient)
from PySide6.QtWidgets import QSizePolicy, QWidget


@lru_cache(maxsize=1)
def _diffusion_weights(samples=288):
    """Periodic Gaussian scattering, with no seam between channels 23 and 0."""
    result = []
    for sample in range(samples):
        position = sample * 24 / samples
        weights = [math.exp(-0.5 * (((index - position + 12) % 24 - 12) / .8) ** 2)
                   for index in range(24)]
        total = sum(weights)
        result.append(tuple(weight / total for weight in weights))
    return tuple(result)


def _diffuse_colors(colors):
    """Return 288 smoothly scattered RGB samples without altering input power.

    All 24 sources contribute through normalized circular Gaussian weights.
    Uniform light stays uniform; an isolated source and its moving tail mix
    across adjacent arcs instead of exposing individual lamp cores.
    """
    return tuple(tuple(sum(color[channel] * weight for color, weight in zip(colors, weights))
                       for channel in range(3)) for weights in _diffusion_weights())


def _emission_color(rgb):
    """Tint the white scattering layer while retaining supplied light intensity."""
    level = max(rgb)
    if level <= 0:
        return QColor(255, 255, 255, 0)
    # A little white scattering imitates an opal cover. Alpha controls emitted
    # power on top of the neutral cover, so black never paints a black lamp.
    color = [round(255 * (.16 + .84 * channel / level)) for channel in rgb]
    return QColor(*color, round(246 * (level / 255) ** .8))


class RgbRing(QWidget):
    """Paint supplied RGB values; animation and brightness belong to the caller.

    Source zero is at the top and subsequent sources run clockwise. This is a
    simulated orientation, independent of the physical ring's wiring.
    """

    PIXEL_COUNT = 24
    _PALETTES = {
        "light": {
            "canvas": "#f7fafc", "canvas_border": "#e2eaf2",
            "plate": "#edf2f7", "plate_border": "#cedbe7",
            "track": "#dce6ee", "center": "#f9fbfd",
            "bezel": "#61798e", "title": "#243c54", "muted": "#587087",
        },
        "dark": {
            "canvas": "#142031", "canvas_border": "#2a3d53",
            "plate": "#1c2d41", "plate_border": "#3b536a",
            "track": "#101b2a", "center": "#172538",
            "bezel": "#58718b", "title": "#e0edf9", "muted": "#a6bfd5",
        },
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._colors = ((0, 0, 0),) * self.PIXEL_COUNT
        self._theme = "light"
        self._title = "RGB 圆环"
        self._subtitle = "乳白扩散罩 · 24 路灯光"
        self._active = True
        self._light_cache_key = None
        self._light_image = None
        self.setMinimumSize(180, 180)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setAccessibleName("RGB 连续扩散灯环预览")
        self._update_description()

    def sizeHint(self):
        return QSize(320, 320)

    def minimumSizeHint(self):
        return QSize(180, 180)

    def set_frame(self, colors):
        """Atomically accept exactly 24 integer RGB triples in the range 0..255."""
        try:
            frame = tuple(tuple(color) for color in colors)
        except TypeError as exc:
            raise ValueError("圆环帧必须包含 24 组 RGB 数值") from exc
        if len(frame) != self.PIXEL_COUNT:
            raise ValueError("圆环帧必须包含 24 组 RGB 数值")
        for color in frame:
            if len(color) != 3 or any(
                isinstance(channel, bool) or not isinstance(channel, Integral)
                or not 0 <= channel <= 255 for channel in color
            ):
                raise ValueError("每颗灯珠需要三个 0..255 的 RGB 整数")
        frame = tuple(tuple(int(channel) for channel in color) for color in frame)
        if frame != self._colors:
            self._colors = frame
            self.update()

    def set_theme(self, theme):
        if not isinstance(theme, str) or theme not in self._PALETTES:
            raise ValueError("圆环主题必须为 light 或 dark")
        if theme != self._theme:
            self._theme = theme
            self.update()

    def set_caption(self, title, subtitle=""):
        title, subtitle = str(title), str(subtitle)
        if (title, subtitle) != (self._title, self._subtitle):
            self._title, self._subtitle = title, subtitle
            self._update_description()
            self.update()

    def set_active(self, active):
        """Accept the external animator's state without changing any RGB value.

        This widget owns no timer, so pausing never rewrites or dims its frame.
        """
        self._active = bool(active)

    def _update_description(self):
        caption = " · ".join(part for part in (self._title, self._subtitle) if part)
        orientation = "顶部对应 0 号光源方向，按顺时针排列；仅表示模拟方向。"
        self.setToolTip(f"{caption}\n{orientation}" if caption else orientation)
        self.setAccessibleDescription(self.toolTip())

    def paintEvent(self, event):
        palette = self._PALETTES[self._theme]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        # Own the complete background, including the rounded border's corners;
        # native Window palettes must not leak white into a dark preview.
        painter.fillRect(self.rect(), QColor(palette["canvas"]))
        painter.setPen(QPen(QColor(palette["canvas_border"]), 1))
        painter.setBrush(QColor(palette["canvas"]))
        painter.drawRoundedRect(QRectF(self.rect()).adjusted(.5, .5, -.5, -.5), 12, 12)

        diameter = min(520., max(0., min(self.width(), self.height()) - 20.))
        if diameter <= 0:
            return
        center = QPointF(self.width() / 2., self.height() / 2.)
        radius = diameter / 2.
        plate = QRectF(center.x() - radius, center.y() - radius, diameter, diameter)
        painter.setPen(QPen(QColor(palette["plate_border"]), 1))
        painter.setBrush(QColor(palette["plate"]))
        painter.drawEllipse(plate)

        inner_radius = diameter * .295
        painter.setPen(QPen(QColor(palette["plate_border"]), 1))
        painter.setBrush(QColor(palette["center"]))
        painter.drawEllipse(center, inner_radius, inner_radius)

        self._paint_diffuser(painter, center, diameter)
        self._paint_caption(painter, center, diameter, palette)

    def _paint_diffuser(self, painter, center, diameter):
        outer = diameter * .471
        inner = diameter * .359
        inner_stop = inner / outer
        # Neutral cover remains visible with a black frame in both themes.
        # Radial shading and soft edges describe one continuous moulded ring.
        cover = QRadialGradient(center, outer)
        cover.setColorAt(0., QColor(193, 201, 209, 0))
        cover.setColorAt(inner_stop, QColor(193, 201, 209, 0))
        cover.setColorAt(inner_stop + .018, QColor(193, 201, 209))
        cover.setColorAt(.855, QColor(220, 224, 228))
        cover.setColorAt(.9, QColor(211, 216, 221))
        cover.setColorAt(.983, QColor(187, 197, 207))
        cover.setColorAt(1., QColor(187, 197, 207, 0))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(cover)
        painter.drawEllipse(center, outer, outer)

        if not any(any(rgb) for rgb in self._colors):
            return
        dpr = painter.device().devicePixelRatioF()
        cache_key = (diameter, dpr, self._colors)
        origin = QPointF(center.x() - diameter / 2., center.y() - diameter / 2.)
        if cache_key == self._light_cache_key:
            painter.drawImage(origin, self._light_image)
            return
        local = QPointF(diameter / 2., diameter / 2.)
        angular = QConicalGradient(local, 90.)
        scattered = _diffuse_colors(self._colors)
        count = len(scattered)
        # Qt's conical stops run counterclockwise; the protocol preview runs
        # clockwise from the top. Closing with sample zero removes a seam.
        for sample in range(count):
            angular.setColorAt(sample / count, _emission_color(scattered[-sample % count]))
        angular.setColorAt(1., _emission_color(scattered[0]))

        light = QImage(math.ceil(diameter * dpr), math.ceil(diameter * dpr),
                       QImage.Format.Format_ARGB32_Premultiplied)
        light.setDevicePixelRatio(dpr)
        light.fill(Qt.GlobalColor.transparent)
        layer = QPainter(light)
        layer.fillRect(QRectF(0., 0., diameter, diameter), angular)
        mask = QRadialGradient(local, outer)
        mask.setColorAt(0., QColor(0, 0, 0, 0))
        mask.setColorAt(inner_stop, QColor(0, 0, 0, 0))
        mask.setColorAt(inner_stop + .024, QColor(0, 0, 0, 230))
        mask.setColorAt(.84, QColor(0, 0, 0, 255))
        mask.setColorAt(.93, QColor(0, 0, 0, 255))
        mask.setColorAt(1., QColor(0, 0, 0, 0))
        layer.setCompositionMode(QPainter.CompositionMode.CompositionMode_DestinationIn)
        layer.fillRect(QRectF(0., 0., diameter, diameter), mask)
        layer.end()
        self._light_cache_key, self._light_image = cache_key, light
        painter.drawImage(origin, light)

    def _paint_caption(self, painter, center, diameter, palette):
        width = diameter * .53
        left = center.x() - width / 2.
        font = QFont("Microsoft YaHei UI")
        font.setPixelSize(14 if diameter < 200 else 16)
        font.setWeight(QFont.Weight.DemiBold)
        painter.setFont(font)
        painter.setPen(QColor(palette["title"]))
        title = QFontMetrics(font).elidedText(self._title, Qt.TextElideMode.ElideRight,
                                            round(width))
        title_y = center.y() - 25. if self._subtitle else center.y() - 12.
        painter.drawText(QRectF(left, title_y, width, 24.),
                         Qt.AlignmentFlag.AlignCenter, title)
        if self._subtitle:
            font.setPixelSize(14)
            font.setWeight(QFont.Weight.Normal)
            painter.setFont(font)
            painter.setPen(QColor(palette["muted"]))
            painter.drawText(QRectF(left, center.y() + 2., width, 40.),
                             Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop
                             | Qt.TextFlag.TextWordWrap, self._subtitle)
