"""Finite moves are completely preflighted before the caller sends any sample."""
import math
import numpy as np
from .types import MotionLimits, PlanResult, Sample, Status, Tool, vector
from .frames import error, interpolate
from .path import LinePath, smooth_progress
from .ik import IKSolver
from .rail_policy import RailPolicy
from .validation import movej_command


class LinePlanner:
    def __init__(self, model):
        self.model = model
        self.solver = IKSolver(model)

    def plan_line(self, start_state, target, motion_limits=None, tool=None,
                  rail_policy=RailPolicy.AUTO, cancelled=None):
        limits, tool = motion_limits or MotionLimits(), tool or Tool()
        q = vector(start_state, 7)
        if not self.model.valid(q):
            raise ValueError("Start state exceeds limits")
        path = LinePath(self.model.fk(q, tool), target)
        distance = error(path.start, target)
        linear, angular = np.linalg.norm(distance[:3]), np.linalg.norm(distance[3:])
        duration = max(limits.sample_period, 1.875 * linear / limits.linear_speed,
                       1.875 * angular / limits.angular_speed)
        n = max(1, math.ceil(duration / limits.sample_period),
                math.ceil(1.875 * linear / limits.max_translation_step),
                math.ceil(1.875 * angular / limits.max_rotation_step))
        if n + 1 > limits.max_samples:
            return PlanResult(Status.BUDGET_EXCEEDED, reason="Path sample budget exceeded")
        states = [(0., q, path.start)]
        for i in range(1, n + 1):
            s = smooth_progress(i / n)
            pose = path.at(s)
            result = self.solver.solve_pose(pose, q, rail_policy=rail_policy, tool=tool, cancelled=cancelled)
            if not result.ok:
                return PlanResult(result.status, reason=f"Local IK failed at progress {s:.6f}")
            q = result.q
            states.append((s, q, pose))
        # Refine geometric joint-interpolation error; rail/joint endpoint IK alone
        # cannot establish that the path between two commands is straight.
        for _ in range(10):
            refined, changed = [states[0]], False
            for left, right in zip(states, states[1:]):
                mid_s = (left[0] + right[0]) / 2
                mid_pose = path.at(mid_s)
                residual = error(self.model.fk((left[1]+right[1])/2, tool), mid_pose)
                if np.linalg.norm(residual[:3]) > .00015 or np.linalg.norm(residual[3:]) > np.deg2rad(.075):
                    if len(states) + len(refined) >= limits.max_samples:
                        return PlanResult(Status.BUDGET_EXCEEDED, reason="Refinement budget exceeded")
                    result = self.solver.solve_pose(mid_pose, left[1], rail_policy=rail_policy, tool=tool, cancelled=cancelled)
                    if not result.ok:
                        return PlanResult(result.status, reason="Path midpoint failed")
                    refined.append((mid_s, result.q, mid_pose))
                    changed = True
                refined.append(right)
            states = refined
            if not changed:
                break
        else:
            return PlanResult(Status.NO_LOCAL_SOLUTION, reason="Path refinement did not converge")
        # Keep at least one send period between all samples. Retiming is global,
        # preserving geometry; include starting/stopping velocity in acceleration.
        dt = max(limits.sample_period, duration / (len(states)-1))
        qs = np.stack([item[1] for item in states])
        velocity = np.diff(qs, axis=0) / dt
        acceleration = np.diff(np.vstack((np.zeros(7), velocity, np.zeros(7))), axis=0) / dt
        scale = max(1., float(np.max(np.abs(velocity) / self.model.constraints.velocity)),
                    math.sqrt(float(np.max(np.abs(acceleration) / self.model.constraints.acceleration))))
        dt *= scale * 1.001
        samples = []
        for i, (_, q, pose) in enumerate(states):
            if cancelled is not None and cancelled():
                return PlanResult(Status.CANCELLED)
            try:
                _, quantized = movej_command(self.model, q, pose, tool)
            except ValueError as exc:
                return PlanResult(Status.NO_LOCAL_SOLUTION, reason=str(exc))
            samples.append(Sample(i * dt, quantized, pose))
        for left, right in zip(samples, samples[1:]):
            for ratio in (.25, .5, .75):
                residual = error(self.model.fk(left.q+(right.q-left.q)*ratio, tool),
                                 interpolate(left.pose, right.pose, ratio))
                if np.linalg.norm(residual[:3]) > .0002 or np.linalg.norm(residual[3:]) > np.deg2rad(.1):
                    return PlanResult(Status.NO_LOCAL_SOLUTION, reason="Quantized segment exceeds path tolerance")
        # Quantization also perturbs the velocity and acceleration of tiny steps.
        qs = np.stack([item.q for item in samples])
        velocity = np.diff(qs, axis=0) / dt
        acceleration = np.diff(np.vstack((np.zeros(7), velocity, np.zeros(7))), axis=0) / dt
        factor = max(1., float(np.max(np.abs(velocity)/self.model.constraints.velocity)),
                     math.sqrt(float(np.max(np.abs(acceleration)/self.model.constraints.acceleration))))
        if factor > 1:
            samples = [Sample(item.time * factor * 1.001, item.q, item.pose) for item in samples]
        return PlanResult(Status.OK, tuple(samples))
