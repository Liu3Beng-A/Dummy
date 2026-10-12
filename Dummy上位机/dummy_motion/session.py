"""Pure incremental planning. Reference, estimated feedback and wire state stay separate."""
import time
import numpy as np
from .types import Tool, MotionLimits, Status, Sample, vector
from .frames import error, interpolate
from .feedback import FeedbackTracker
from .ik import IKSolver
from .rail_policy import RailPolicy, step_weights


class MotionSession:
    def __init__(self, model, seed, tool=None, limits=None):
        self.model, self.tool = model, tool or Tool()
        self.limits = limits or MotionLimits()
        self.reference_q = vector(seed, 7)
        if not model.valid(seed):
            raise ValueError("Invalid session seed")
        self.feedback = FeedbackTracker(model)
        self.solver = IKSolver(model)
        # A 20 ms acceleration-limited first step can be below the offline
        # endpoint tolerance; solve it accurately instead of sticking at rest.
        self.solver.position_tolerance = 1e-7
        self.solver.rotation_tolerance = 1e-6
        self.velocity = np.zeros(7)
        self.elapsed = 0.

    def update_feedback(self, sample):
        return self.feedback.update(sample)

    def commit(self, q, dt):
        """Advance the planning reference; transport must keep its written reference separately."""
        q = vector(q, 7)
        if not np.isfinite(dt) or dt <= 0 or not self.model.valid(q):
            raise ValueError("Invalid committed reference")
        self.velocity = (q-self.reference_q) / dt
        self.reference_q = q
        self.elapsed += dt

    def _velocity_fraction(self, velocity, dt):
        bounds = self.model.constraints
        low = np.maximum(-bounds.velocity, self.velocity-bounds.acceleration*dt)
        high = np.minimum(bounds.velocity, self.velocity+bounds.acceleration*dt)
        minimum, maximum = 0., 1.
        for value, lower, upper in zip(velocity, low, high):
            if abs(value) < 1e-12:
                if not lower <= 0 <= upper:
                    return None
                continue
            ratios = sorted((lower/value, upper/value))
            minimum, maximum = max(minimum, ratios[0]), min(maximum, ratios[1])
        if maximum < max(0., minimum) or maximum <= 1e-8:
            return None
        return max(minimum, maximum*.9) if maximum < 1 else 1.

    def _brake(self, dt):
        # Direction changes need a finite deceleration interval. Holding the
        # previous velocity forever while rejecting a reverse IK would deadlock.
        limit = self.model.constraints.acceleration * dt
        velocity = np.sign(self.velocity) * np.maximum(0., np.abs(self.velocity)-limit)
        q = self.reference_q + velocity*dt
        if not self.model.valid(q):
            return Status.LIMIT_BLOCKED, None
        return Status.SLOWED, Sample(self.elapsed+dt, q, self.model.fk(q, self.tool))

    def step(self, dt, request, now, cancelled=None):
        if not np.isfinite(dt) or not 0 < dt <= .2:
            raise ValueError("Step must be in (0, 0.2] seconds")
        status, _ = self.feedback.estimate(now)
        if status != Status.OK:
            return status, None
        start = self.model.fk(self.reference_q, self.tool)
        e = error(start, request)
        distance, angle = np.linalg.norm(e[:3]), np.linalg.norm(e[3:])
        if distance < 1e-7 and angle < 1e-7:
            return Status.OK, Sample(self.elapsed, self.reference_q, start)
        rate = self.feedback.speed_scale(start, self.tool, now)
        if rate <= 0:
            return Status.SLOWED, None
        fraction = min(1., self.limits.linear_speed * dt * rate / max(distance, 1e-12),
                       self.limits.angular_speed * dt * rate / max(angle, 1e-12),
                       self.limits.max_translation_step / max(distance, 1e-12),
                       self.limits.max_rotation_step / max(angle, 1e-12))
        deadline = time.perf_counter() + .01
        # A local differential prediction selects the acceleration ramp before
        # expensive iterative IK. This is only a hint, never an accepted pose.
        scale = np.array([1.]*6+[.15])
        task_weight = np.array([1.]*3+[.15]*3)
        jac = self.model.jacobian(self.reference_q, self.tool)*scale[None, :]
        weights = step_weights(self.reference_q, self.model.constraints)
        a = np.vstack((task_weight[:, None]*jac, .002*np.diag(weights)))
        b = np.r_[task_weight*e*fraction, np.zeros(7)]
        velocity_hint = np.linalg.lstsq(a, b, rcond=None)[0]*scale/dt
        adjustment = self._velocity_fraction(velocity_hint, dt)
        if adjustment is None:
            return self._brake(dt)
        fraction *= adjustment
        for _ in range(3):
            remaining_ms = (deadline-time.perf_counter())*1000
            if remaining_ms <= 0:
                return Status.BUDGET_EXCEEDED, None
            pose = interpolate(start, request, fraction)
            result = self.solver.solve_pose(pose, self.reference_q, tool=self.tool,
                                            rail_policy=RailPolicy.AUTO, max_iterations=8,
                                            budget_ms=remaining_ms, cancelled=cancelled)
            if not result.ok:
                return result.status, None
            if time.perf_counter() > deadline:
                return Status.BUDGET_EXCEEDED, None
            velocity = (result.q-self.reference_q)/dt
            if (np.all(np.abs(velocity) <= self.model.constraints.velocity) and
                    np.all(np.abs(velocity-self.velocity) <= self.model.constraints.acceleration*dt)):
                return (Status.OK if fraction == 1 else Status.SLOWED), Sample(self.elapsed+dt, result.q, pose)
            # Estimate an admissible path fraction from the solved velocity.
            # This needs one correction solve at startup instead of repeatedly
            # halving and resolving the same local geometry.
            adjustment = self._velocity_fraction(velocity, dt)
            if adjustment is None:
                return self._brake(dt)
            fraction *= adjustment
        return Status.SLOWED, None
