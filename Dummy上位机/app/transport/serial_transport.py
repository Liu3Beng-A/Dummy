"""Serial workers with a single writer queue and per-connection identity."""
import queue
import threading
import time
from PySide6.QtCore import QObject, Signal, QTimer
from app.protocol.parser import finite_numbers
from app.transport.wire_log import WireLog
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
        self.line_observer = None
        self.written_observer = None
        self.telemetry_validator = None
        self.wire_log = WireLog()
        self._telemetry_lock = threading.Lock()
        self._latest_telemetry = None
        self._telemetry_timer = QTimer(self)
        self._telemetry_timer.setInterval(33)
        self._telemetry_timer.timeout.connect(self._flush_telemetry)

    def _flush_telemetry(self):
        with self._telemetry_lock:
            item, self._latest_telemetry = self._latest_telemetry, None
        if item is not None and not self._stop.is_set():
            line, stamp = item
            self.line_received_at.emit(line, self.connection_id, stamp)
            self.line_received.emit(line, self.connection_id)

    def start(self):
        self._telemetry_timer.start()
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
                    received_at, wall_time = time.perf_counter(), time.time()
                    for line in framer.feed(chunk):
                        self.wire_log.record('RX', line, received_at, wall_time)
                        suppressed = False
                        if self.line_observer is not None and self.line_observer(line, self.connection_id, received_at):
                            suppressed = True
                        nums = finite_numbers(line[3:], 7) if line.startswith('ok ') else None
                        if nums is not None and (self.telemetry_validator is None or self.telemetry_validator(nums)):
                            # GUI has one latest pose, never an unbounded queue
                            # of already superseded 30/50 Hz telemetry events.
                            with self._telemetry_lock:
                                self._latest_telemetry = (line, received_at)
                            continue
                        if suppressed:
                            continue
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
                    self.wire_log.record('TX', command, time.perf_counter(), time.time())
                if self.written_observer is not None and self.written_observer(command, generation, self.connection_id):
                    continue
                self.command_written.emit(command, self.connection_id)
            except Exception as error:
                if not self._stop.is_set():
                    self.failed.emit(f"串口发送失败：{error}", self.connection_id)
                return

    def close(self):
        self._telemetry_timer.stop()
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
        self.wire_log.close()
