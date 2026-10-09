"""Thread-safe Qt replacement for the old assistant's scheduled callbacks."""
import itertools
import threading
from PySide6.QtCore import QObject, QTimer, Signal, Slot, Qt


class QtScheduler(QObject):
    requested = Signal(int, int, object)
    cancelled = Signal(int)
    callback_finished = Signal()
    callback_failed = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ids = itertools.count(1)
        self._lock = threading.Lock()
        self._timers = {}
        self._cancelled = set()
        self.requested.connect(self._schedule, Qt.ConnectionType.QueuedConnection)
        self.cancelled.connect(self._cancel, Qt.ConnectionType.QueuedConnection)

    def after(self, milliseconds, callback, *args):
        with self._lock:
            token = next(self._ids)
        self.requested.emit(token, int(milliseconds), lambda: callback(*args))
        return token

    def after_cancel(self, token):
        if token is not None:
            self.cancelled.emit(int(token))

    @Slot(int, int, object)
    def _schedule(self, token, milliseconds, callback):
        if token in self._cancelled:
            self._cancelled.remove(token)
            return
        timer = QTimer(self)
        timer.setSingleShot(True)
        self._timers[token] = timer

        def finish():
            self._timers.pop(token, None)
            timer.deleteLater()
            try:
                callback()
            except Exception as error:
                self.callback_failed.emit(f"定时任务失败：{error}")
            finally:
                self.callback_finished.emit()

        timer.timeout.connect(finish)
        timer.start(max(0, milliseconds))

    @Slot(int)
    def _cancel(self, token):
        timer = self._timers.pop(token, None)
        if timer is not None:
            timer.stop()
            timer.deleteLater()
        else:
            self._cancelled.add(token)

    def close(self):
        for timer in self._timers.values():
            timer.stop()
            timer.deleteLater()
        self._timers.clear()
        self._cancelled.clear()
