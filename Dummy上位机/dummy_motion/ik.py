"""Bounded damped least squares. A failed local solve is never sent as a target."""
import time
import numpy as np
from scipy.optimize import lsq_linear
from .types import IKResult, Status, Tool, vector
from .frames import error
from .rail_policy import RailPolicy, step_weights


class IKSolver:
    position_tolerance = .0001
    rotation_tolerance = np.deg2rad(.05)

    def __init__(self, model):
        self.model = model

    def solve_pose(self, target, seed, constraints=None, rail_policy=RailPolicy.AUTO,
                   tool=None, max_iterations=48, budget_ms=None, cancelled=None):
        started = time.perf_counter()
        q = vector(seed, 7).copy()
        bounds = constraints or self.model.constraints
        policy = RailPolicy(rail_policy)
        tool = tool or Tool()
        if np.any(q < bounds.lower) or np.any(q > bounds.upper):
            raise ValueError("IK seed outside joint limits")
        if type(max_iterations) is not int or max_iterations < 1:
            raise ValueError("Invalid iteration budget")
        if budget_ms is not None and (not np.isfinite(budget_ms) or budget_ms <= 0):
            raise ValueError("Invalid time budget")
        scale = np.array([1.] * 6 + [.15])
        task_weight = np.array([1.] * 3 + [.15] * 3)
        step = np.array([.12] * 6 + [.012])
        status = Status.NO_LOCAL_SOLUTION
        iteration = 0
        for iteration in range(max_iterations + 1):
            if cancelled is not None and cancelled():
                status = Status.CANCELLED
                break
            e = error(self.model.fk(q, tool), target)
            if np.linalg.norm(e[:3]) <= self.position_tolerance and np.linalg.norm(e[3:]) <= self.rotation_tolerance:
                status = Status.OK
                break
            if budget_ms is not None and (time.perf_counter() - started) * 1000 >= budget_ms:
                status = Status.BUDGET_EXCEEDED
                break
            if iteration == max_iterations:
                break
            low = np.maximum(bounds.lower - q, -step) / scale
            high = np.minimum(bounds.upper - q, step) / scale
            active = np.arange(6 if policy == RailPolicy.LOCKED else 7)
            j = self.model.jacobian(q, tool) * scale[None, :]
            weights = step_weights(q, bounds, policy)
            a = np.vstack((task_weight[:, None] * j[:, active], .002 * np.diag(weights[active])))
            b = np.r_[task_weight * e, np.zeros(len(active))]
            result = lsq_linear(a, b, bounds=(low[active], high[active]), method="bvls", tol=1e-9, max_iter=40)
            dq = np.zeros(7)
            dq[active] = result.x * scale[active]
            baseline = np.linalg.norm(task_weight * e)
            improved = False
            for ratio in (1., .5, .25, .125):
                candidate = q + ratio * dq
                # BVLS may exceed a bound by a few ulps; never clip a real violation.
                if np.any(candidate < bounds.lower - 1e-12) or np.any(candidate > bounds.upper + 1e-12):
                    continue
                candidate = np.minimum(bounds.upper, np.maximum(bounds.lower, candidate))
                if np.linalg.norm(task_weight * error(self.model.fk(candidate, tool), target)) < baseline - 1e-12:
                    q = candidate
                    improved = True
                    break
            if not improved:
                status = Status.LIMIT_BLOCKED if np.any(np.minimum(q-bounds.lower, bounds.upper-q) < 1e-6) else Status.NO_LOCAL_SOLUTION
                break
        e = error(self.model.fk(q, tool), target)
        elapsed_ms = (time.perf_counter() - started) * 1000
        # A final FK/residual evaluation can cross the deadline even when the
        # previous iteration began on time. Discard such an otherwise valid IK.
        if status == Status.OK and budget_ms is not None and elapsed_ms > budget_ms:
            status = Status.BUDGET_EXCEEDED
        return IKResult(status, q, float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:])),
                        iteration, elapsed_ms)
