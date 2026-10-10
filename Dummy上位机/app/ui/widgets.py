"""Reusable Qt controls, kept independent from serial communication."""
from PySide6.QtCore import QSignalBlocker, Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QColorDialog, QDoubleSpinBox, QHBoxLayout, QLabel,
    QPushButton, QSlider, QSpinBox, QVBoxLayout, QWidget,
)


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


class ColorEditor(QWidget):
    """Three numeric RGB channels and a native color picker."""
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
        chosen = QColorDialog.getColor(old, self, "选择灯效颜色")
        if chosen.isValid():
            for spin, value in zip(self.channels, (chosen.red(), chosen.green(), chosen.blue())):
                spin.setValue(value)
