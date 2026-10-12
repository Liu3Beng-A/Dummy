"""Feedback-seeded, bounded correction of an already preflighted path target."""
from dataclasses import replace
import numpy as np
from .ik import IKSolver
from .types import Status, Sample
from .frames import error, interpolate
from .validation import movej_command


def correct_sample(model, target, tool, feedback, now, previous_q, velocity,
                   dt, speed, cancelled=None):
    status, measured = feedback.estimate(now)
    if status != Status.OK:
        return status, None
    # Limits apply to the reference actually sent to the device. Measured
    # state supplies the IK seed; neither state overwrites the other.
    dt = min(.5, max(.1, dt))
    c = model.constraints
    lower = np.maximum(c.lower, previous_q + np.maximum(-c.velocity, velocity-c.acceleration*dt)*dt)
    upper = np.minimum(c.upper, previous_q + np.minimum(c.velocity, velocity+c.acceleration*dt)*dt)
    if np.any(upper-lower <= 1e-10):
        return Status.LIMIT_BLOCKED, None
    solver = IKSolver(model)
    solver.position_tolerance = .00001
    solver.rotation_tolerance = np.deg2rad(.005)
    result = solver.solve_pose(target.pose, np.clip(measured, lower, upper),
        constraints=replace(c, lower=lower, upper=upper), tool=tool,
        max_iterations=12, budget_ms=10., cancelled=cancelled, weight_constraints=c)
    if not result.ok:
        return result.status, None
    try:
        _, q = movej_command(model, result.q, target.pose, tool, speed)
    except ValueError:
        return Status.LIMIT_BLOCKED, None
    # Quantization is checked against the same bounds, not just the FK error.
    if np.any(q < lower-1e-12) or np.any(q > upper+1e-12):
        return Status.SLOWED, None
    start = model.fk(previous_q, tool)
    for fraction in (.25, .5, .75):
        residual = error(model.fk(previous_q+(q-previous_q)*fraction, tool),
                         interpolate(start, target.pose, fraction))
        if np.linalg.norm(residual[:3]) > .0002 or np.linalg.norm(residual[3:]) > np.deg2rad(.1):
            return Status.LIMIT_BLOCKED, None
    return Status.OK, Sample(target.time, q, target.pose)
