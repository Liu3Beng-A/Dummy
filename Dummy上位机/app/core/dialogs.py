"""Qt dialogs used by migrated operations, deliberately with no Tk runtime."""
from PySide6.QtWidgets import QApplication, QMessageBox


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
