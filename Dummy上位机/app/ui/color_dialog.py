"""A locally translated and themed Qt color picker, without global translators."""
from __future__ import annotations

import re

from PySide6.QtCore import QEvent
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import (
    QApplication, QColorDialog, QDialog, QDialogButtonBox, QLabel, QLineEdit, QPushButton,
)


class ChineseColorDialog(QColorDialog):
    """Keep Qt's color panels and inputs while localizing this dialog only."""

    _TEXT = {
        "Basic colors": "基础颜色", "Custom colors": "自定义颜色",
        "Add to Custom Colors": "保存到自定义颜色", "Pick Screen Color": "拾取屏幕颜色",
        "Hue:": "色相 H：", "Sat:": "饱和度 S：", "Val:": "明度 V：",
        "Red:": "红色 R：", "Green:": "绿色 G：", "Blue:": "蓝色 B：",
        "Alpha channel:": "透明度：", "HTML:": "HEX 色值：",
        "OK": "确定", "Cancel": "取消",
    }
    _INPUTS = {
        "Hue:": ("ColorHue", "色相，0～359"),
        "Sat:": ("ColorSaturation", "饱和度，0～255"),
        "Val:": ("ColorValue", "明度，0～255"),
        "Red:": ("ColorRed", "红色通道，0～255"),
        "Green:": ("ColorGreen", "绿色通道，0～255"),
        "Blue:": ("ColorBlue", "蓝色通道，0～255"),
    }
    _PALETTES = {
        "light": {
            "window": "#f2f5f9", "text": "#223248", "input": "#ffffff",
            "border": "#b9cbdc", "button": "#ffffff", "hover": "#e7f1fa",
            "muted": "#687f95", "selection": "#286caa", "edge": "#d4dfe9",
        },
        "dark": {
            "window": "#111419", "text": "#e5e9ef", "input": "#15181f",
            "border": "#4b5260", "button": "#22262e", "hover": "#2b303a",
            "muted": "#9ca5b4", "selection": "#297ab6", "edge": "#30343e",
        },
    }

    def __init__(self, initial, parent=None, title="选择灯效颜色", theme=None):
        self._localizing = False
        self._ready = False
        super().__init__(parent)
        self.setOption(QColorDialog.ColorDialogOption.DontUseNativeDialog, True)
        self.setOption(QColorDialog.ColorDialogOption.ShowAlphaChannel, False)
        self.setObjectName("ChineseColorDialog")
        self.setWindowTitle(title)
        self.setCurrentColor(QColor(initial))
        self._localize_text()
        self._apply_theme(theme or self._parent_theme(parent))
        self._ready = True

    @staticmethod
    def _parent_theme(parent):
        window = parent.window() if parent is not None else QApplication.activeWindow()
        palette = window.palette() if window is not None else QApplication.palette()
        return "dark" if palette.color(QPalette.ColorRole.Window).lightness() < 128 else "light"

    def _apply_theme(self, theme):
        if theme not in self._PALETTES:
            raise ValueError("颜色选择窗口主题必须为 light 或 dark")
        colors = self._PALETTES[theme]
        self.theme = theme
        palette = QPalette(self.palette())
        for group in (QPalette.ColorGroup.Active, QPalette.ColorGroup.Inactive,
                      QPalette.ColorGroup.Disabled):
            foreground = colors["muted"] if group == QPalette.ColorGroup.Disabled else colors["text"]
            for role, value in (
                (QPalette.ColorRole.Window, colors["window"]),
                (QPalette.ColorRole.WindowText, foreground),
                (QPalette.ColorRole.Base, colors["input"]),
                (QPalette.ColorRole.Text, foreground),
                (QPalette.ColorRole.Button, colors["button"]),
                (QPalette.ColorRole.ButtonText, foreground),
                (QPalette.ColorRole.Highlight, colors["selection"]),
                (QPalette.ColorRole.HighlightedText, "#ffffff"),
                (QPalette.ColorRole.Mid, colors["edge"]),
            ):
                palette.setColor(group, role, QColor(value))
        self.setPalette(palette)
        self.setStyleSheet(f"""
            QColorDialog#ChineseColorDialog {{ background: {colors['window']}; }}
            QColorDialog#ChineseColorDialog QWidget {{
                font-family: 'Microsoft YaHei UI'; font-size: 15px; color: {colors['text']};
            }}
            QColorDialog#ChineseColorDialog QLabel {{ background: transparent; }}
            QColorDialog#ChineseColorDialog QLineEdit,
            QColorDialog#ChineseColorDialog QSpinBox {{
                background: {colors['input']}; color: {colors['text']};
                border: 1px solid {colors['border']}; border-radius: 5px;
                selection-background-color: {colors['selection']}; selection-color: white;
            }}
            QColorDialog#ChineseColorDialog QPushButton {{
                background: {colors['button']}; color: {colors['text']};
                border: 1px solid {colors['border']}; border-radius: 6px;
                padding: 7px 12px;
            }}
            QColorDialog#ChineseColorDialog QPushButton:hover {{ background: {colors['hover']}; }}
            QColorDialog#ChineseColorDialog QPushButton:disabled {{ color: {colors['muted']}; }}
            QColorDialog#ChineseColorDialog QPushButton#ColorConfirm {{
                background: {colors['selection']}; color: white; font-weight: 600;
                border-color: {colors['selection']}; min-width: 56px;
            }}
            QColorDialog#ChineseColorDialog QPushButton#ColorCancel {{ min-width: 56px; }}
        """)

    def _localize_text(self):
        if self._localizing:
            return
        self._localizing = True
        try:
            for label in self.findChildren(QLabel):
                text = label.text().replace("&", "").strip()
                if label.text() == "\n" or text.startswith("Cursor at "):
                    label.setObjectName("ColorScreenHint")
                    label.installEventFilter(self)
                if text in self._INPUTS and label.buddy() is not None:
                    name, tooltip = self._INPUTS[text]
                    label.buddy().setObjectName(name)
                    label.buddy().setToolTip(tooltip)
                if text in self._TEXT:
                    label.setText(self._TEXT[text])
                else:
                    self._localize_cursor_hint(label)
            for control in self.findChildren(QPushButton):
                text = control.text().replace("&", "").strip()
                if text in self._TEXT:
                    control.setText(self._TEXT[text])
            buttons = self.findChild(QDialogButtonBox)
            if buttons is not None:
                for role, name, text in (
                    (QDialogButtonBox.StandardButton.Ok, "ColorConfirm", "确定"),
                    (QDialogButtonBox.StandardButton.Cancel, "ColorCancel", "取消"),
                ):
                    control = buttons.button(role)
                    if control is not None:
                        control.setObjectName(name)
                        if control.text() != text:
                            control.setText(text)
            # Qt owns validation and RGB/HSV synchronization of its HEX field.
            field = self.findChild(QLineEdit, "qt_colorname_lineedit")
            if field is not None:
                field.setToolTip("十六进制 RGB 色值，例如 #12ABEF")
        finally:
            self._localizing = False

    @staticmethod
    def _localize_cursor_hint(label):
        cursor = re.fullmatch(r"Cursor at (-?\d+), (-?\d+)\nPress ESC to cancel",
                              label.text().strip())
        if cursor:
            label.setText(f"光标位置：{cursor[1]}，{cursor[2]}\n按 Esc 取消取色")

    def eventFilter(self, watched, event):
        if (watched.objectName() == "ColorScreenHint" and
                event.type() in (QEvent.Type.Paint, QEvent.Type.Show)):
            # Qt's screen-picking timer replaces this label directly, without
            # necessarily requesting a layout. Translate before it is painted.
            self._localize_cursor_hint(watched)
        return super().eventFilter(watched, event)

    def event(self, event):
        result = super().event(event)
        if (getattr(self, "_ready", False) and
                event.type() in (QEvent.Type.LanguageChange, QEvent.Type.LayoutRequest)):
            # Screen picking updates its own cursor hint. Localize that text too,
            # while leaving application translators and other dialogs untouched.
            self._localize_text()
        return result


def choose_color(initial, parent=None, title="选择灯效颜色"):
    """Return the confirmed QColor, or an invalid QColor when cancelled."""
    dialog = ChineseColorDialog(initial, parent, title)
    try:
        if dialog.exec() == QDialog.DialogCode.Accepted:
            return QColor(dialog.selectedColor())
        return QColor()
    finally:
        dialog.deleteLater()
