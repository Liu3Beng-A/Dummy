"""Map timed joint samples to the existing firmware's scalar MoveJ ceiling.

The caller supplies motor acceleration in rev/s². This mirrors the current
firmware's zero-start/zero-end trapezoid model; it is an estimate, not a
promise of physical arrival time or velocity-continuous streaming.
"""
from dataclasses import dataclass
import math
import numpy as np
from .types import vector


@dataclass(frozen=True)
class SegmentTiming:
    speed_percent: float
    requested_seconds: float
    predicted_seconds: float
    minimum_speed_limited: bool
    duration_limited: bool


def motor_distances(start, target):
    delta = np.abs(vector(target, 7)-vector(start, 7))
    # J1..J5 50:1, J6 30:1; 1605 rail screw is 5 mm/revolution.
    return delta*np.r_[np.array([50.,50.,50.,50.,50.,30.])/(2*np.pi),200.]


def segment_timing(start, target, duration, motor_acceleration):
    """Derive protocol speed from geometry/time; there is no UI speed input.

    The current protocol clamps 1..100 (0.30..30 rev/s). Report when that
    floor prevents a small segment from occupying the requested duration.
    """
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("Segment duration must be finite and positive")
    acceleration = vector(motor_acceleration,7)
    if np.any(acceleration <= 0):
        raise ValueError("Motor acceleration must be positive rev/s²")
    distances = motor_distances(start,target)
    moving = distances >= 1e-6
    if not np.any(moving):
        return SegmentTiming(1.,float(duration),0.,False,False)
    d,a = distances[moving],acceleration[moving]
    def time_at(ceiling):
        return np.where(ceiling*ceiling/a >= d,2*np.sqrt(d/a),d/ceiling+ceiling/a)
    fastest = float(np.max(time_at(30.)))
    planned = max(float(duration),fastest)
    discriminant = np.maximum(0.,planned*planned-4*d/a)
    per_axis = 2*d/(planned+np.sqrt(discriminant))
    needed = float(np.max(per_axis))/.30
    # Round upward to the protocol's 0.01 precision: never understate the
    # required speed ceiling. Re-evaluate the duration after quantization.
    speed = min(100.,max(1.,math.ceil(needed*100-1e-10)/100))
    predicted = float(np.max(time_at(speed*.30)))
    return SegmentTiming(speed,float(duration),predicted,
                         needed < 1. and predicted+1e-6 < duration,
                         fastest > duration+1e-6)
