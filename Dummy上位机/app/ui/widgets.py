"""Reusable Qt controls, kept independent from serial communication."""
from html import escape

from PySide6.QtCore import QEvent, QSignalBlocker, QSize, Qt, Signal
from PySide6.QtWidgets import (
    QAbstractSpinBox, QCheckBox, QDoubleSpinBox, QHBoxLayout, QLabel,
    QPushButton, QSlider, QSpinBox, QStyle, QStyleOptionButton, QSizePolicy,
    QVBoxLayout, QWidget,
)

from app.ui.color_dialog import choose_color


def hint(text, role="hint"):
    label = QLabel(text)
    label.setWordWrap(True)
    label.setProperty("role", role)
    return label


def button(text, callback=None, role=None):
    control = QPushButton(text)
    if role:
        control.setProperty("role", role)
    if callback:
        control.clicked.connect(lambda checked=False: callback())
    return control


def number(value=0, minimum=-1e8, maximum=1e8, decimals=3, step=1):
    control = QDoubleSpinBox()
    control.setRange(minimum, maximum)
    control.setDecimals(decimals)
    control.setSingleStep(step)
    control.setValue(value)
    control.setKeyboardTracking(False)
    control.setMinimumWidth(90)
    return control


class AxisRow(QWidget):
    changed = Signal(float)

    def __init__(self, name, limits, value, unit="°", parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(7)
        title = QLabel(name)
        self.title = title
        title.setFixedWidth(38)
        layout.addWidget(title)
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(round(limits[0] * 100), round(limits[1] * 100))
        self.slider.setValue(round(value * 100))
        self.slider.setSingleStep(100)
        self.slider.setMinimumWidth(40)
        layout.addWidget(self.slider, 1)
        self.readback = QLabel("—")
        self.readback.setProperty("role", "muted")
        self.readback.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.readback.setFixedWidth(60)
        self.readback.setToolTip("实际回读；未收到有效数据时显示 —")
        layout.addWidget(self.readback)
        self.spin = number(value, *limits, decimals=2, step=1)
        self.spin.setSuffix(f" {unit}")
        self.spin.setFixedWidth(128)
        layout.addWidget(self.spin)
        self.spin.valueChanged.connect(self._spin_changed)
        self.slider.valueChanged.connect(lambda value: self.spin.setValue(value / 100))

    def set_motion_style(self):
        """Opt-in geometry, leaving teaching/parameter instances unchanged."""
        self.setProperty("motionAxis", True)
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        layout = self.layout()
        layout.setSpacing(8)
        # The workstation separates measured values from the editable target.
        # Other pages keep their original axis order.
        layout.removeWidget(self.readback)
        layout.insertWidget(1, self.readback)
        self.title.setFixedWidth(30)
        self.readback.setFixedWidth(59)
        self.spin.setFixedSize(119, 30)
        self.spin.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.setFixedHeight(32)

    def _spin_changed(self, value):
        self.slider.blockSignals(True)
        self.slider.setValue(round(value * 100))
        self.slider.blockSignals(False)
        self.changed.emit(value)

    def set_value(self, value, notify=False):
        """Copy an edited/readback target without triggering another command."""
        spin_blocker = QSignalBlocker(self.spin)
        slider_blocker = QSignalBlocker(self.slider)
        self.spin.setValue(value)
        self.slider.setValue(round(self.spin.value() * 100))
        del spin_blocker, slider_blocker
        if notify:
            self.changed.emit(self.spin.value())


class PointShortcutButton(QPushButton):
    """Two-line, width-independent preview shortcut for a one-based slot."""

    def __init__(self, slot, callback=None, parent=None):
        super().__init__(parent)
        self._slot = int(slot)
        self._caption_lines = ("", "")
        self.setObjectName("PointShortcutButton")
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setFixedHeight(50)
        self.setAccessibleDescription("仅加载目标，使用发送按钮执行")
        if callback is not None:
            self.clicked.connect(lambda checked=False: callback())
        self.set_point(None)

    def set_point(self, record_or_None):
        if record_or_None is None:
            self._caption_lines = (f"{self._slot} · 暂无点位", "")
            self.setAccessibleName(f"点位 {self._slot}，暂无点位")
            self.setToolTip("暂无点位；在点位示教页保存点位后可加载目标。")
            self.setEnabled(False)
        else:
            name = str(record_or_None.get("name", "未命名点位"))
            kind = str(record_or_None.get("type", "MoveJ"))
            speed = f"{float(record_or_None.get('speed', 35)):g}"
            display_name = " ".join(name.split())
            self._caption_lines = (f"{self._slot} · {display_name}",
                                   f"{kind} · 速度 {speed}")
            self.setAccessibleName(f"点位 {self._slot}，{name}，{kind}，速度 {speed}")
            safe_name = (escape(name).replace("\r\n", "\n")
                         .replace("\r", "\n").replace("\n", "<br>"))
            self.setToolTip(f"<qt>名称：{safe_name}<br>类型：{escape(kind)}"
                            f"<br>速度：{escape(speed)}<br>仅加载目标，使用发送按钮执行</qt>")
            self.setEnabled(True)
        self._refresh_caption()

    def sizeHint(self):
        # Full point names must never contribute to the layout's minimum width.
        return QSize(180, 46 if self.property("motionPoint") else 50)

    def minimumSizeHint(self):
        return QSize(64, 46 if self.property("motionPoint") else 50)

    def set_motion_style(self):
        self.setProperty("motionPoint", True)
        self.setFixedHeight(46)

    def _refresh_caption(self):
        if not hasattr(self, "_caption_lines"):
            return
        option = QStyleOptionButton()
        self.initStyleOption(option)
        contents = self.style().subElementRect(QStyle.SubElement.SE_PushButtonContents,
                                              option, self)
        width = max(0, contents.width() - 2)
        lines = [self.fontMetrics().elidedText(line, Qt.TextElideMode.ElideRight, width)
                 .replace("&", "&&") for line in self._caption_lines]
        text = "\n".join(lines)
        if self.text() != text:
            self.setText(text)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh_caption()

    def changeEvent(self, event):
        super().changeEvent(event)
        if event.type() in (QEvent.Type.FontChange, QEvent.Type.StyleChange):
            self._refresh_caption()


class ColorEditor(QWidget):
    """Three numeric RGB channels and a Chinese, themed Qt color picker."""
    def __init__(self, color=(0, 0, 255), parent=None):
        super().__init__(parent)
        row = QHBoxLayout(self)
        row.setContentsMargins(0, 0, 0, 0)
        self.channels = []
        for name, value in zip(("R", "G", "B"), color):
            row.addWidget(QLabel(name))
            spin = QSpinBox()
            spin.setRange(0, 255)
            spin.setValue(value)
            spin.setFixedWidth(74)
            row.addWidget(spin)
            self.channels.append(spin)
            spin.valueChanged.connect(self._refresh)
        self.swatch = QPushButton("选择颜色")
        self.swatch.setMinimumWidth(88)
        self.swatch.clicked.connect(self._pick)
        row.addWidget(self.swatch)
        row.addStretch()
        self._refresh()

    def _refresh(self):
        r, g, b = (spin.value() for spin in self.channels)
        foreground = "#142337" if (r * .299 + g * .587 + b * .114) > 145 else "white"
        self.swatch.setStyleSheet(f"background: rgb({r},{g},{b}); color: {foreground}; border: 1px solid #8fa7bb;")

    def _pick(self):
        from PySide6.QtGui import QColor
        old = QColor(*(spin.value() for spin in self.channels))
        chosen = choose_color(old, self, "选择灯效颜色")
        if chosen.isValid():
            for spin, value in zip(self.channels, (chosen.red(), chosen.green(), chosen.blue())):
                spin.setValue(value)
