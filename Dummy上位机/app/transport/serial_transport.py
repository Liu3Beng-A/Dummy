"""Serial workers with a single writer queue and per-connection identity."""
import queue
import threading
import time
from PySide6.QtCore import QObject, Signal, QTimer, Qt
from app.protocol.parser import finite_numbers
from app.transport.wire_log import WireLog
from app.protocol.parser import LineFramer
from app.transport.telemetry import JointPollSchedule, TelemetryMetrics


class SerialTransport(QObject):
    line_received = Signal(str, int)
    line_received_at = Signal(str, int, float)
    command_written = Signal(str, int)
    failed = Signal(str, int)
    telemetry_received = Signal(str, int, float)
    telemetry_timed_out = Signal(int)

    def __init__(self, port, outbox, allowed, connection_id, parent=None):
        super().__init__(parent)
        self.port = port
        self.outbox = outbox
        self.allowed = allowed
        self.connection_id = connection_id
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._threads = []
        self.write_in_progress = False
        self.last_activity_at = 0.0
        self.line_observer = None
        self.written_observer = None
        self.write_ready = None
        self.telemetry_validator = None
        self.wire_log = WireLog()
        self._telemetry_lock = threading.Lock()
        self._latest_telemetry = None
        self._managed_polling = False
        self._manual_position = None
        self.joint_poll = JointPollSchedule()
        self.metrics = TelemetryMetrics()
        self.telemetry_allowed = None
        self._reported_timeouts = 0
        self._telemetry_timer = QTimer(self)
        self._telemetry_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._telemetry_timer.setInterval(16)
        self._telemetry_timer.timeout.connect(self._flush_telemetry)

    def _flush_telemetry(self):
        with self._telemetry_lock:
            item, self._latest_telemetry = self._latest_telemetry, None
        if item is not None and not self._stop.is_set():
            line, stamp = item
            if self._managed_polling:
                self.telemetry_received.emit(line, self.connection_id, stamp)
            else:
                self.line_received_at.emit(line, self.connection_id, stamp)
                self.line_received.emit(line, self.connection_id)
        if self.joint_poll.timeouts != self._reported_timeouts and not self._stop.is_set():
            self._reported_timeouts = self.joint_poll.timeouts
            self.telemetry_timed_out.emit(self.connection_id)

    def configure_joint_polling(self, enabled, hz, generation):
        self._managed_polling = True
        self.joint_poll.configure(enabled, hz, generation)

    def adopt_joint_poll(self):
        """A manual joint sync may adopt a written, still outstanding poll."""
        with self._telemetry_lock:
            stamp = self.joint_poll.written_at()
            if stamp is None or self._manual_position is not None:
                return None
            self._manual_position = ("j", stamp)
            return stamp

    def performance_snapshot(self):
        return {**self.metrics.snapshot(), "query_timeouts": self.joint_poll.timeouts}

    @property
    def pending_writes(self):
        # Queue tracks the item even between get() and write_in_progress=True.
        return self.outbox.unfinished_tasks

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
                    self.last_activity_at = time.monotonic()
                    received_at, wall_time = time.perf_counter(), time.time()
                    for line in framer.feed(chunk):
                        self.wire_log.record('RX', line, received_at, wall_time)
                        suppressed = False
                        if self.line_observer is not None and self.line_observer(line, self.connection_id, received_at):
                            suppressed = True
                        nums = finite_numbers(line[3:], 7) if line.startswith('ok ') else None
                        if nums is not None and (self.telemetry_validator is None or self.telemetry_validator(nums)):
                            direct = False
                            # GUI has one latest pose, never an unbounded queue
                            # of already superseded 30/50 Hz telemetry events.
                            with self._telemetry_lock:
                                rtt = self.joint_poll.receive(received_at)
                                self.metrics.received(received_at, rtt)
                                if self._manual_position and self._manual_position[0] == "j":
                                    direct = received_at >= self._manual_position[1]
                                    if direct:
                                        self._manual_position = None
                                        self._latest_telemetry = None
                                if not direct:
                                    if self._latest_telemetry is not None:
                                        self.metrics.replaced()
                                    self._latest_telemetry = (line, received_at)
                            if direct:
                                self.line_received_at.emit(line, self.connection_id, received_at)
                                self.line_received.emit(line, self.connection_id)
                            continue
                        if line.startswith('ok ') and finite_numbers(line[3:], 6) is not None:
                            with self._telemetry_lock:
                                if self._manual_position and self._manual_position[0] == "l":
                                    self._manual_position = None
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
            automatic = False
            try:
                item = self.outbox.get(timeout=self.joint_poll.wait_seconds())
            except queue.Empty:
                with self._telemetry_lock:
                    manual = self._manual_position
                    if manual and time.perf_counter() - manual[1] >= 1.2:
                        self._manual_position = manual = None
                generation = self.joint_poll.take_due() if manual is None else None
                if generation is None:
                    continue
                item, automatic = (generation, "#GETJPOS"), True
            if item is None:
                return
            generation, command = item
            try:
                # Read-only requests yield while an unlabeled background reply
                # is outstanding; do not hold the writer ahead of STOP/DISABLE.
                if not automatic and command in ("#GETJPOS", "#GETLPOS") and self.joint_poll.pending():
                    self.outbox.put(item)
                    self._stop.wait(.002)
                    continue
                self.write_in_progress = True
                ready = True
                # Flash cannot service USB OUT. Keep a requested safety command
                # at the head, without holding the write lock or blocking RX/UI.
                while self.write_ready is not None and not self.write_ready(generation, command):
                    if automatic:
                        ready = False
                        break
                    if self._stop.wait(.01):
                        break
                with self._write_lock:
                    if self._stop.is_set():
                        if automatic:
                            self.joint_poll.cancel_unwritten()
                        continue
                    allowed = (self.telemetry_allowed(generation) if automatic and self.telemetry_allowed
                               else False if automatic else self.allowed(generation, command))
                    if not ready or not allowed:
                        if automatic:
                            self.joint_poll.cancel_unwritten()
                        continue
                    if automatic:
                        self.joint_poll.written()
                    elif command in ("#GETJPOS", "#GETLPOS"):
                        with self._telemetry_lock:
                            self._manual_position = ("j" if command == "#GETJPOS" else "l", time.perf_counter())
                    data = (command + "\n").encode("utf-8")
                    count = self.port.write(data)
                    if count != len(data):
                        raise IOError("串口发送不完整")
                    self.wire_log.record('TX', command, time.perf_counter(), time.time())
                    self.last_activity_at = time.monotonic()
                if automatic:
                    # Full wire audit is retained; polling never fills the
                    # control-command admission ledger or a queued GUI signal.
                    continue
                if self.written_observer is not None and self.written_observer(command, generation, self.connection_id):
                    continue
                self.command_written.emit(command, self.connection_id)
            except Exception as error:
                if not self._stop.is_set():
                    self.failed.emit(f"串口发送失败（{command}）：{error}", self.connection_id)
                return
            finally:
                self.write_in_progress = False
                if not automatic:
                    self.outbox.task_done()

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
