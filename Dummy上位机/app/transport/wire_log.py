"""Bounded asynchronous wire audit, independent of Qt delivery and observers."""
from collections import deque
from datetime import datetime
import queue
import threading


class WireLog:
    def __init__(self, sink=None, capacity=8192):
        self.sink = sink
        self.pending = queue.Queue(maxsize=capacity)
        self.history = deque(maxlen=capacity)
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.dropped = 0
        self.sequence = 0
        self.thread = threading.Thread(target=self._run, name='Dummy wire audit', daemon=True)
        self.thread.start()

    def record(self, direction, line, received_at, wall_time):
        with self.lock:
            self.sequence += 1
            entry = (self.sequence, direction, line, received_at, wall_time)
            self.history.append(entry)
        if self.sink is not None:
            try:
                self.pending.put_nowait(entry)
            except queue.Full:
                self.dropped += 1  # Explicit loss accounting, never block RX/TX.

    @staticmethod
    def format(entry):
        sequence, direction, line, stamp, wall = entry
        timestamp = datetime.fromtimestamp(wall).isoformat(timespec='milliseconds')
        return f'{timestamp} [WIRE {direction}] seq={sequence} rx_tx={stamp:.6f} {line}'

    def snapshot(self):
        with self.lock:
            records = tuple(self.history)
        return f'Wire audit dropped={self.dropped}; bounded history={len(records)}\n' + '\n'.join(map(self.format, records))

    def _run(self):
        reported = 0
        while not self.stopping.is_set() or not self.pending.empty():
            try:
                entry = self.pending.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                if self.dropped != reported:
                    self.sink('WARN', f'WIRE audit queue overflow/loss={self.dropped}')
                    reported = self.dropped
                self.sink('WIRE', self.format(entry))
            except Exception:
                self.dropped += 1

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=1.)
