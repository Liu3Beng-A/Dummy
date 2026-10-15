"""Reusable modeless RGB controls; all preview rendering stays in the main view."""
from PySide6.QtCore import QEvent, Qt, Signal
from PySide6.QtWidgets import QMainWindow, QVBoxLayout, QWidget


class RgbSettingsWindow(QMainWindow):
    """Closing hides the editor so its draft and geometry remain available."""

    visibility_changed = Signal(bool)
    editor_closed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("RgbSettingsWindow")
        self.setWindowTitle("RGB 灯效")
        self.setWindowModality(Qt.WindowModality.NonModal)
        self.resize(820, 700)
        self.setMinimumSize(680, 460)
        self._shutting_down = False
        self._preview_visible = False
        content = QWidget(self)
        content.setObjectName("Workspace")
        self.content_layout = QVBoxLayout(content)
        self.content_layout.setContentsMargins(14, 12, 14, 12)
        self.content_layout.setSpacing(8)
        self.setCentralWidget(content)

    def _notify_visibility(self):
        visible = self.isVisible() and not self.isMinimized() and not self._shutting_down
        if visible != self._preview_visible:
            self._preview_visible = visible
            self.visibility_changed.emit(visible)

    def showEvent(self, event):
        super().showEvent(event)
        self._notify_visibility()

    def hideEvent(self, event):
        super().hideEvent(event)
        self._notify_visibility()

    def changeEvent(self, event):
        super().changeEvent(event)
        if event.type() == QEvent.Type.WindowStateChange:
            self._notify_visibility()

    def closeEvent(self, event):
        if self._shutting_down:
            super().closeEvent(event)
        else:
            event.ignore()
            self.hide()
            self.editor_closed.emit()

    def shutdown(self):
        """Close the owned window when the main program really exits."""
        self._shutting_down = True
        self.close()
        self._notify_visibility()
