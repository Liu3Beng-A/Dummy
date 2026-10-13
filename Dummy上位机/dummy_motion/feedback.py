"""Receive time is not a device measurement timestamp. Prediction is bounded."""
import numpy as np
from .types import FeedbackSample, Status
from .frames import error


class FeedbackTracker:
    def __init__(self, model, stale_after=.3, max_prediction=.04):
        if stale_after <= 0 or max_prediction < 0:
            raise ValueError("Invalid feedback limits")
        self.model = model
        self.stale_after = stale_after
        self.max_prediction = max_prediction
        self.sample = None
        self.rejection_reason = ""
        self.velocity = np.zeros(7)

    def update(self, sample: FeedbackSample):
        if not self.model.valid(sample.q):
            self.rejection_reason = "七轴位置超出模型限位"
            return False
        previous = self.sample
        if previous is not None:
            dt = sample.received_at - previous.received_at
            if dt < 0:
                self.rejection_reason = "回读时间戳重复或倒序"
                return False
            if dt == 0:
                # One serial read may contain several ordered valid poses.
                # Keep its last pose without inventing measurement time/speed.
                self.sample = sample
                self.rejection_reason = ""
                return True
            raw = (sample.q - previous.q) / dt
            bounds = self.model.constraints.velocity
            self.velocity = .5 * self.velocity + .5 * np.clip(raw, -bounds, bounds)
        self.sample = sample
        self.rejection_reason = ""
        return True

    def estimate(self, now):
        if not np.isfinite(now):
            raise ValueError("Invalid clock")
        if self.sample is None or not 0 <= now - self.sample.received_at <= self.stale_after:
            return Status.FEEDBACK_STALE, None
        dt = min(now - self.sample.received_at, self.max_prediction)
        q = self.sample.q + self.velocity * dt
        return Status.OK, np.clip(q, self.model.constraints.lower, self.model.constraints.upper)

    def speed_scale(self, reference, tool, now):
        status, estimate = self.estimate(now)
        if status != Status.OK:
            return 0.
        residual = error(self.model.fk(estimate, tool), reference)
        lag = max(np.linalg.norm(residual[:3])/.02, np.linalg.norm(residual[3:])/np.deg2rad(8))
        return max(0., min(1., 2. - 2.*lag))
