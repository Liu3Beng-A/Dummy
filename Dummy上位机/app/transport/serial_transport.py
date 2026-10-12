"""Serial workers with a single writer queue and per-connection identity."""
import queue
import threading
import time
from PySide6.QtCore import QObject, Signal
from app.protocol.parser import LineFramer


class SerialTransport(QObject):
    line_received = Signal(str, int)
    line_received_at = Signal(str, int, float)
    command_written = Signal(str, int)
    failed = Signal(str, int)

    def __init__(self, port, outbox, allowed, connection_id, parent=None):
        super().__init__(parent)
        self.port = port
        self.outbox = outbox
        self.allowed = allowed
        self.connection_id = connection_id
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._threads = []

    def start(self):
        self._threads = [threading.Thread(target=self._receive, daemon=True, name="Dummy RX"),
                         threading.Thread(target=self._write, daemon=True, name="Dummy TX")]
        for thread in self._threads:
            thread.start()

    def _receive(self):
        framer = LineFramer()
        while not self._stop.is_set():
            try:
                waiting = self.port.in_waiting
                if waiting:
                    chunk = self.port.read(min(waiting, 16384))
                    received_at = time.perf_counter()
                    for line in framer.feed(chunk):
                        self.line_received_at.emit(line, self.connection_id, received_at)
                        self.line_received.emit(line, self.connection_id)
                self._stop.wait(0.01)
            except Exception as error:
                if not self._stop.is_set():
                    self.failed.emit(f"串口读取失败：{error}", self.connection_id)
                return

    def _write(self):
        while not self._stop.is_set():
            try:
                item = self.outbox.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                return
            generation, command = item
            try:
                with self._write_lock:
                    if self._stop.is_set() or not self.allowed(generation, command):
                        continue
                    data = (command + "\n").encode("utf-8")
                    count = self.port.write(data)
                    if count != len(data):
                        raise IOError("串口发送不完整")
                self.command_written.emit(command, self.connection_id)
            except Exception as error:
                if not self._stop.is_set():
                    self.failed.emit(f"串口发送失败：{error}", self.connection_id)
                return

    def close(self):
        self._stop.set()
        try:
            if hasattr(self.port, "cancel_read"):
                self.port.cancel_read()
            if hasattr(self.port, "cancel_write"):
                self.port.cancel_write()
            self.port.close()
        except Exception:
            pass
        for thread in self._threads:
            if thread is not threading.current_thread():
                thread.join(timeout=1.2)
        self._threads.clear()
