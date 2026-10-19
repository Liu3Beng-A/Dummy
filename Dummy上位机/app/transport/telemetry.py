"""Bounded, thread-safe scheduling and measurements for read-only joint polls."""
from collections import deque
import math
import threading
import time


class JointPollSchedule:
    """One query in flight, no catch-up bursts and no GUI acknowledgement needed."""
    def __init__(self, clock=time.perf_counter, timeout=1.2):
        self.clock, self.timeout = clock, timeout
        self.lock = threading.Lock()
        self.enabled, self.hz, self.generation = False, 5., 0
        self.next_due = 0.
        self.sent_at = None
        self.was_written = False
        self.timeouts = 0

    def configure(self, enabled, hz, generation):
        if not math.isfinite(hz) or not 1 <= hz <= 50:
            raise ValueError("位置回读频率应为 1～50 Hz")
        with self.lock:
            changed = (bool(enabled), float(hz), generation) != (self.enabled, self.hz, self.generation)
            self.enabled, self.hz, self.generation = bool(enabled), float(hz), generation
            if changed:
                self.next_due = self.clock()
            # An already written query must drain even after disabling/changing
            # generation; ASCII pose replies do not carry a request identity.

    def _expire(self, now):
        if self.sent_at is not None and now - self.sent_at >= self.timeout:
            self.sent_at = None
            self.timeouts += 1

    def pending(self):
        with self.lock:
            self._expire(self.clock())
            return self.sent_at is not None

    def wait_seconds(self):
        with self.lock:
            now = self.clock()
            self._expire(now)
            if not self.enabled:
                return .05
            if self.sent_at is not None:
                return max(.001, min(.01, self.timeout - (now - self.sent_at)))
            return max(.001, min(.05, self.next_due - now))

    def take_due(self):
        with self.lock:
            now = self.clock()
            self._expire(now)
            if not self.enabled or self.sent_at is not None or now < self.next_due:
                return None
            self.sent_at = now
            self.was_written = False
            self.next_due = now + 1 / self.hz
            return self.generation

    def cancel_unwritten(self):
        with self.lock:
            self.sent_at = None
            self.was_written = False

    def written_at(self):
        with self.lock:
            self._expire(self.clock())
            return self.sent_at if self.was_written else None

    def written(self):
        with self.lock:
            now = self.clock()
            self.sent_at = now
            self.was_written = True
            self.next_due = now + 1 / self.hz

    def receive(self, stamp):
        with self.lock:
            if self.sent_at is None or not self.was_written or stamp < self.sent_at:
                return None
            elapsed = (stamp - self.sent_at) * 1000
            self.sent_at = None
            return elapsed


class TelemetryMetrics:
    """Host-side rates/latency only; no claim of device sampling timestamps."""
    def __init__(self, clock=time.perf_counter):
        self.clock = clock
        self.lock = threading.Lock()
        self.rx = deque(maxlen=512)
        self.gui = deque(maxlen=512)
        self.delivery = deque(maxlen=120)
        self.roundtrip = deque(maxlen=120)
        self.rx_total = self.coalesced = 0

    def received(self, stamp, rtt=None):
        with self.lock:
            self.rx.append(stamp)
            self.rx_total += 1
            if rtt is not None and math.isfinite(rtt) and rtt >= 0:
                self.roundtrip.append(rtt)

    def replaced(self):
        with self.lock:
            self.coalesced += 1

    def delivered(self, stamp):
        now = self.clock()
        if not math.isfinite(stamp) or now < stamp:
            return
        with self.lock:
            self.gui.append(now)
            self.delivery.append((now - stamp) * 1000)

    @staticmethod
    def _rate(values, now):
        samples = [stamp for stamp in values if now - 2 <= stamp <= now]
        if not samples:
            return 0.
        if len(samples) < 2 or samples[-1] <= samples[0]:
            return None
        # Silence reduces the measured rate instead of retaining an old Hz.
        return (len(samples) - 1) / (max(samples[-1], now) - samples[0])

    @staticmethod
    def _p95(values):
        ordered = sorted(values)
        return ordered[min(len(ordered)-1, int(len(ordered)*.95))] if ordered else None

    def snapshot(self):
        now = self.clock()
        with self.lock:
            recent = bool(self.rx and now - self.rx[-1] <= 2)
            return dict(rx_hz=self._rate(self.rx, now), gui_hz=self._rate(self.gui, now),
                        rx_total=self.rx_total, coalesced=self.coalesced,
                        rx_age_ms=(now-self.rx[-1])*1000 if self.rx else None,
                        latency_ms=self.delivery[-1] if self.delivery and recent else None,
                        latency_p95_ms=self._p95(self.delivery) if recent else None,
                        rtt_ms=self.roundtrip[-1] if self.roundtrip and recent else None,
                        rtt_p95_ms=self._p95(self.roundtrip) if recent else None)
