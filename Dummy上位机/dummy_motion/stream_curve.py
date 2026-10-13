"""Finite seven-axis 50 Hz q/v/a trajectories for STREAM_JOINT.

The PC performs IK and time parameterization before opening a motor session.
Motor reference curves are reconstructed from quantized quintic boundaries;
speed is never mapped to an ordinary MoveJ speed argument.
"""
from dataclasses import dataclass, replace
import math
import numpy as np
from scipy.interpolate import make_interp_spline
from .types import Tool, MotionLimits, Pose, vector
from .frames import error, interpolate
from .planner import LinePlanner
from .stream_protocol import (StreamKnot, STREAM_PERIOD, STREAM_SEED, STREAM_END,
                              STREAM_MAX_SEQUENCE, motor_coordinates, robot_coordinates)


@dataclass(frozen=True)
class StreamPlan:
    knots: tuple
    duration: float
    final_q: np.ndarray
    maximum_velocity_ratio: float
    maximum_acceleration_ratio: float
    maximum_jerk_ratio: float


def quintic_coefficients(left, right, period=STREAM_PERIOD):
    """Coefficients in normalized u=t/T, in motor subdivision units."""
    p0, v0, a0 = np.asarray(left, dtype=float).T
    p1, v1, a1 = np.asarray(right, dtype=float).T
    c0, c1, c2 = p0, v0*period, a0*period*period/2
    dp, dv, da = p1-c0-c1-c2, v1*period-c1-2*c2, a1*period*period-2*c2
    return np.array([c0, c1, c2, 10*dp-4*dv+da/2, -15*dp+7*dv-da, 6*dp-3*dv+da/2])


def evaluate_segment(left, right, fraction, derivative=0):
    coeff = quintic_coefficients(left, right)
    for _ in range(derivative):
        coeff = np.arange(1, len(coeff))[:, None]*coeff[1:] / STREAM_PERIOD
    return np.polynomial.polynomial.polyval(fraction, coeff, tensor=True).T


def _extrema(coeff, derivative):
    for _ in range(derivative):
        coeff = np.arange(1, len(coeff))[:, None]*coeff[1:] / STREAM_PERIOD
    peak = np.zeros(7)
    for axis in range(7):
        c = coeff[:, axis]
        roots = np.polynomial.polynomial.polyroots(np.arange(1, len(c))*c[1:])
        points = [0., 1.] + [r.real for r in roots if abs(r.imag) < 1e-8 and 0 < r.real < 1]
        peak[axis] = max(abs(np.polynomial.polynomial.polyval(points, c)))
    return peak


class StreamPlanner:
    def __init__(self, model, motor_acceleration=None, motor_velocity=None):
        self.model = model
        # Integer motor positions imply a bounded sub-microstep quantization
        # jerk at 20ms. This explicit envelope includes that floor; it is not a
        # claimed identified motor parameter. Continuous acceleration is kept.
        self.jerk = model.constraints.acceleration * 32
        self.motor_acceleration = vector(np.full(7, 30.) if motor_acceleration is None else motor_acceleration, 7)
        self.motor_velocity = vector(np.full(7, 30.) if motor_velocity is None else motor_velocity, 7)
        if np.any(self.motor_acceleration <= 0) or np.any(self.motor_velocity <= 0):
            raise ValueError('Motor velocity and acceleration must be positive')

    def plan_line(self, start, target, limits=None, tool=None, cancelled=None):
        limits, tool = limits or MotionLimits(), tool or Tool()
        start = vector(start, 7)
        if np.linalg.norm(error(self.model.fk(start, tool), target)) <= 1e-12:
            return self._stationary(start, cancelled)
        planner = LinePlanner(self.model)
        planner.solver.position_tolerance = 1e-6
        planner.solver.rotation_tolerance = np.deg2rad(.001)
        geometric = planner.plan_line(start, target,
            replace(limits, sample_period=.04, max_translation_step=.0008,
                    max_rotation_step=np.deg2rad(.4)), tool=tool,
            cancelled=cancelled, quantize=False)
        if not geometric.ok:
            raise ValueError(f'{geometric.status.value}: {geometric.reason}')
        times = np.array([sample.time for sample in geometric.samples])
        qs = np.stack([sample.q for sample in geometric.samples])
        return self._fit(times, qs, tool, geometric.samples, cancelled, limits)

    def plan_joint(self, start, target, speed=35., cancelled=None):
        start, target = vector(start, 7), vector(target, 7)
        if not self.model.valid(start) or not self.model.valid(target):
            raise ValueError('Joint stream target outside limits')
        if not np.isfinite(speed) or not 1 <= speed <= 100:
            raise ValueError('Speed must be 1..100')
        if np.array_equal(start, target):
            return self._stationary(start, cancelled)
        duration = max(STREAM_PERIOD, float(np.max(1.875*abs(target-start)/(self.model.constraints.velocity*speed/100))))
        return self._fit(np.array([0., duration]), np.stack((start, target)), Tool(), None, cancelled)

    def _stationary(self, q, cancelled):
        if cancelled is not None and cancelled():
            raise ValueError('Stream planning cancelled')
        motor = motor_coordinates(q)
        actual = robot_coordinates(motor)[0]
        if not self.model.valid(q) or not self.model.valid(actual):
            raise ValueError('Stationary stream outside joint limits')
        actual.setflags(write=False)
        return StreamPlan((StreamKnot(0, motor, STREAM_SEED), StreamKnot(1, motor, STREAM_END)),
                          STREAM_PERIOD, actual, 0., 0., 0.)

    def _fit(self, times, qs, tool, reference, cancelled, cartesian_limits=None):
        zeros = np.zeros(7)
        spline = make_interp_spline(times, qs, k=5,
            bc_type=([(1, zeros), (2, zeros)], [(1, zeros), (2, zeros)]))
        original_duration = float(times[-1])
        probes = np.unique(np.r_[times, np.linspace(0, original_duration, max(33, len(times)*8))])
        velocity, acceleration, jerk = (np.max(abs(spline(probes, order)), axis=0) for order in (1, 2, 3))
        ratio = max(1., float(np.max(velocity/self.model.constraints.velocity)),
                    math.sqrt(float(np.max(acceleration/self.model.constraints.acceleration))),
                    float(np.max(jerk/self.jerk))**(1/3))
        # Retiming and quantization are checked together. Extra margin absorbs
        # integer boundary rounding, which otherwise dominates microscopic moves.
        duration = original_duration * ratio * 1.03
        for attempt in range(8):
            count = max(1, math.ceil(duration/STREAM_PERIOD))
            if count > STREAM_MAX_SEQUENCE:
                raise ValueError('Stream exceeds 65534 segment budget (21.8 minutes)')
            duration = count*STREAM_PERIOD
            factor = original_duration/duration
            t = np.linspace(0, original_duration, count+1)
            q, v, a = spline(t), spline(t, 1)*factor, spline(t, 2)*factor**2
            v[[0, -1]], a[[0, -1]] = 0, 0
            motors = [motor_coordinates(p, vel, acc) for p, vel, acc in zip(q, v, a)]
            metrics = self._validate(motors, tool, reference, times, original_duration,
                                     cancelled, check_geometry=True, cartesian_limits=cartesian_limits)
            dynamic = max(metrics[0], math.sqrt(metrics[1]), metrics[2]**(1/3))
            if dynamic <= 1.00001:
                knots = tuple(StreamKnot(i, m, (STREAM_SEED if i == 0 else 0) |
                                        (STREAM_END if i == count else 0)) for i, m in enumerate(motors))
                final = robot_coordinates(motors[-1])[0]
                final.setflags(write=False)
                return StreamPlan(knots, duration, final, *metrics)
            # Microstep integer position creates irreducible tick-scale jerk.
            # Rechecking catches that floor; do not loop indefinitely or clip it.
            duration *= max(1.15, dynamic*1.03)
        raise ValueError('Quantized stream cannot satisfy dynamic limits')

    def _validate(self, motors, tool, reference, times, original_duration, cancelled, check_geometry, cartesian_limits=None):
        from .stream_protocol import MOTOR_SCALE, MOTOR_ZERO
        peak_v, peak_a, peak_j = np.zeros(7), np.zeros(7), np.zeros(7)
        cartesian_ratio = 0.
        count = len(motors)-1
        for index, (left, right) in enumerate(zip(motors, motors[1:])):
            if cancelled is not None and cancelled():
                raise ValueError('Stream planning cancelled')
            coeff = quintic_coefficients(left, right)
            # Exact polynomial velocity extrema also locate every position
            # extremum, so limits are not established by sparse sampling alone.
            for axis in range(7):
                c = coeff[:, axis]
                roots = np.polynomial.polynomial.polyroots(np.arange(1, 6)*c[1:])
                points = [0., 1.] + [r.real for r in roots if abs(r.imag) < 1e-8 and 0 < r.real < 1]
                positions = np.polynomial.polynomial.polyval(points, c)/MOTOR_SCALE[axis]+MOTOR_ZERO[axis]
                if min(positions) < self.model.constraints.lower[axis]-1e-10 or max(positions) > self.model.constraints.upper[axis]+1e-10:
                    raise ValueError('Stream curve crosses a joint/rail position limit')
            peak_v = np.maximum(peak_v, _extrema(coeff, 1)/abs(MOTOR_SCALE))
            peak_a = np.maximum(peak_a, _extrema(coeff, 2)/abs(MOTOR_SCALE))
            peak_j = np.maximum(peak_j, _extrema(coeff, 3)/abs(MOTOR_SCALE))
            # Match firmware's integer Bernstein convex-hull bounds exactly.
            # Core validates against the device's acceleration, not the UI
            # speed. A 20% margin covers boundary quantization near the cap.
            l, r = np.asarray(left, dtype=np.int64), np.asarray(right, dtype=np.int64)
            dp = r[:, 0]-l[:, 0]
            controls = np.array([np.zeros(7, dtype=np.int64), 200*l[:, 1],
                400*l[:, 1]+l[:, 2], 50000*dp-400*r[:, 1]+r[:, 2],
                50000*dp-200*r[:, 1], 50000*dp])
            bv = np.max(abs(np.diff(controls, axis=0)), axis=0)/200
            ba = np.max(abs(np.diff(controls, n=2, axis=0)), axis=0)
            peak_v = np.maximum(peak_v, bv/abs(MOTOR_SCALE)/.8)
            peak_v = np.maximum(peak_v, bv/(self.motor_velocity*51200*.8)*self.model.constraints.velocity)
            # Express motor acceleration bounds in the model-ratio metric so
            # the existing global time scaler handles both envelopes.
            peak_a = np.maximum(peak_a, ba/(self.motor_acceleration*51200*.8)*self.model.constraints.acceleration)
            if check_geometry and reference:
                for u in (0., .25, .5, .75, 1.):
                    q = np.polynomial.polynomial.polyval(u, coeff)/MOTOR_SCALE+MOTOR_ZERO
                    if cartesian_limits is not None:
                        velocity = np.polynomial.polynomial.polyval(u, np.arange(1, 6)[:, None]*coeff[1:]) / STREAM_PERIOD / MOTOR_SCALE
                        twist = self.model.jacobian(q, tool) @ velocity
                        cartesian_ratio = max(cartesian_ratio,
                            float(np.linalg.norm(twist[:3])/cartesian_limits.linear_speed),
                            float(np.linalg.norm(twist[3:])/cartesian_limits.angular_speed))
                    stamp = (index+u)/count*original_duration
                    j = min(len(times)-2, max(0, int(np.searchsorted(times, stamp)-1)))
                    part = np.clip((stamp-times[j])/(times[j+1]-times[j]), 0, 1)
                    expected = interpolate(reference[j].pose, reference[j+1].pose, part)
                    residual = error(self.model.fk(q, tool), expected)
                    if np.linalg.norm(residual[:3]) > .0002 or np.linalg.norm(residual[3:]) > np.deg2rad(.1):
                        raise ValueError('Stream quintic exceeds Cartesian path tolerance')
        return (max(cartesian_ratio, float(np.max(peak_v/self.model.constraints.velocity))),
                float(np.max(peak_a/self.model.constraints.acceleration)),
                float(np.max(peak_j/self.jerk)))
