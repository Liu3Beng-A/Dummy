"""Continuous redundancy weights, without idle null-space motion."""
from enum import Enum
import numpy as np


class RailPolicy(str, Enum):
    AUTO = "auto"
    LOCKED = "locked"


def step_weights(q, constraints, policy=RailPolicy.AUTO):
    policy = RailPolicy(policy)
    # Smoothly favour rail as the shoulder/elbow approach their limits.
    mid = (constraints.upper + constraints.lower) / 2
    half = (constraints.upper - constraints.lower) / 2
    discomfort = max(abs((q[1:3] - mid[1:3]) / half[1:3]))
    weights = np.ones(7)
    weights[6] = 1.0 + 5.0 * (1.0 - min(1.0, discomfort)) ** 2
    return weights
