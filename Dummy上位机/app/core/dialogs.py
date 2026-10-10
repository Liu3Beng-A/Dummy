"""Qt dialogs used by migrated operations, deliberately with no Tk runtime."""
from PySide6.QtCore import QEvent, QObject
from PySide6.QtWidgets import (
    QApplication, QDialogButtonBox, QInputDialog, QMessageBox, QPushButton,
)


class _DialogAppearance(QObject):
    """Localize standard buttons without replacing their roles or defaults."""

    _BUTTON_TEXT = {
        QDialogButtonBox.StandardButton.Ok: "确定",
        QDialogButtonBox.StandardButton.Cancel: "取消",
        QDialogButtonBox.StandardButton.Yes: "是",
        QDialogButtonBox.StandardButton.No: "否",
        QDialogButtonBox.StandardButton.YesToAll: "全部是",
        QDialogButtonBox.StandardButton.NoToAll: "全部否",
        QDialogButtonBox.StandardButton.Save: "保存",
        QDialogButtonBox.StandardButton.SaveAll: "全部保存",
        QDialogButtonBox.StandardButton.Open: "打开",
        QDialogButtonBox.StandardButton.Close: "关闭",
        QDialogButtonBox.StandardButton.Abort: "中止",
        QDialogButtonBox.StandardButton.Retry: "重试",
        QDialogButtonBox.StandardButton.Ignore: "忽略",
        QDialogButtonBox.StandardButton.Help: "帮助",
        QDialogButtonBox.StandardButton.Apply: "应用",
        QDialogButtonBox.StandardButton.Reset: "重置",
        QDialogButtonBox.StandardButton.RestoreDefaults: "恢复默认",
    }

    def eventFilter(self, watched, event):
        if (isinstance(watched, (QMessageBox, QInputDialog)) and
                event.type() in (QEvent.Type.Show, QEvent.Type.LanguageChange)):
            self._prepare(watched)
        elif isinstance(watched, QPushButton) and event.type() == QEvent.Type.Paint:
            # Qt rewrites button captions after LanguageChange filters run.
            # Prepare again before drawing; the standard role stays untouched.
            box = watched.parentWidget()
            if isinstance(box, QDialogButtonBox):
                parent = box.parentWidget()
                while parent is not None:
                    if isinstance(parent, (QMessageBox, QInputDialog)):
                        self._localize_button(box, watched)
                        break
                    parent = parent.parentWidget()
        return super().eventFilter(watched, event)

    def _localize_button(self, box, button):
        text = self._BUTTON_TEXT.get(box.standardButton(button))
        if text is not None and button.text() != text:
            button.setText(text)

    def _prepare(self, dialog):
        for box in dialog.findChildren(QDialogButtonBox):
            for button in box.buttons():
                self._localize_button(box, button)
        layout = dialog.layout()
        if layout is not None:
            margins = layout.contentsMargins()
            if (margins.left(), margins.top(), margins.right(), margins.bottom()) != (18, 16, 18, 16):
                layout.setContentsMargins(18, 16, 18, 16)
            if layout.spacing() != 12:
                layout.setSpacing(12)


def install_dialog_appearance():
    """Install once per QApplication; its parent and Python reference persist."""
    app = QApplication.instance()
    if not isinstance(app, QApplication):
        return None
    appearance = getattr(app, "_dummy_dialog_appearance", None)
    if appearance is None:
        appearance = _DialogAppearance(app)
        appearance.setObjectName("DialogAppearance")
        app._dummy_dialog_appearance = appearance
        app.installEventFilter(appearance)
    return appearance


def _parent():
    app = QApplication.instance()
    return app.activeWindow() if isinstance(app, QApplication) else None


def showerror(title, message, **_kwargs):
    if isinstance(QApplication.instance(), QApplication):
        QMessageBox.critical(_parent(), title, message)


def showwarning(title, message, **_kwargs):
    if isinstance(QApplication.instance(), QApplication):
        QMessageBox.warning(_parent(), title, message)


def askyesno(title, message, **_kwargs):
    if not isinstance(QApplication.instance(), QApplication):
        return False
    return QMessageBox.question(
        _parent(), title, message, QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        QMessageBox.StandardButton.No,
    ) == QMessageBox.StandardButton.Yes
