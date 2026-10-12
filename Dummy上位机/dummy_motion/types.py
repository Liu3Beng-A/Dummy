"""Immutable SI contracts. J1..J6 are radians; J7 and translations are metres."""
from dataclasses import dataclass, field
from enum import Enum
import numpy as np


def vector(value, size):
    result = np.array(value, dtype=float, copy=True)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"Expected {size} finite values")
    result.setflags(write=False)
    return result


@dataclass(frozen=True, eq=False)
class Pose:
    position: np.ndarray
    rotation: np.ndarray

    def __post_init__(self):
        object.__setattr__(self, "position", vector(self.position, 3))
        r = np.array(self.rotation, dtype=float, copy=True)
        if (r.shape != (3, 3) or not np.isfinite(r).all() or
                not np.allclose(r.T @ r, np.eye(3), atol=1e-8) or
                not np.isclose(np.linalg.det(r), 1., atol=1e-8)):
            raise ValueError("Expected a proper rotation matrix")
        r.setflags(write=False)
        object.__setattr__(self, "rotation", r)

    @staticmethod
    def identity():
        return Pose(np.zeros(3), np.eye(3))


@dataclass(frozen=True)
class Tool:
    flange_tcp: Pose = field(default_factory=lambda: Pose([0, 0, .114], np.eye(3)))
    name: str = "gripper-center-nominal"
    version: int = 1


class Status(str, Enum):
    OK = "OK"
    SLOWED = "SLOWED"
    NO_LOCAL_SOLUTION = "NO_LOCAL_SOLUTION"
    LIMIT_BLOCKED = "LIMIT_BLOCKED"
    BUDGET_EXCEEDED = "BUDGET_EXCEEDED"
    CANCELLED = "CANCELLED"
    FEEDBACK_STALE = "FEEDBACK_STALE"


@dataclass(frozen=True)
class Constraints:
    lower: np.ndarray = field(default_factory=lambda: np.r_[np.deg2rad([-175, -75, 0, -270, -100, -180]), -.25])
    upper: np.ndarray = field(default_factory=lambda: np.r_[np.deg2rad([175, 90, 180, 270, 100, 180]), .25])
    velocity: np.ndarray = field(default_factory=lambda: np.r_[np.deg2rad([40]*6), .04])
    acceleration: np.ndarray = field(default_factory=lambda: np.r_[np.deg2rad([80]*6), .08])

    def __post_init__(self):
        for name in ("lower", "upper", "velocity", "acceleration"):
            object.__setattr__(self, name, vector(getattr(self, name), 7))
        if (np.any(self.lower >= self.upper) or np.any(self.velocity <= 0) or
                np.any(self.acceleration <= 0)):
            raise ValueError("Invalid joint constraints")


@dataclass(frozen=True)
class MotionLimits:
    linear_speed: float = .02
    angular_speed: float = np.deg2rad(10)
    sample_period: float = .1
    max_translation_step: float = .002
    max_rotation_step: float = np.deg2rad(1)
    max_samples: int = 20000

    def __post_init__(self):
        for key in ("linear_speed", "angular_speed", "sample_period", "max_translation_step", "max_rotation_step"):
            if not np.isfinite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError("Motion limits must be finite and positive")
        if type(self.max_samples) is not int or not 2 <= self.max_samples <= 100000:
            raise ValueError("Invalid sample budget")


@dataclass(frozen=True, eq=False)
class IKResult:
    status: Status
    q: np.ndarray
    position_error: float
    rotation_error: float
    iterations: int
    elapsed_ms: float

    def __post_init__(self):
        object.__setattr__(self, "q", vector(self.q, 7))

    @property
    def ok(self):
        return self.status == Status.OK


@dataclass(frozen=True, eq=False)
class Sample:
    time: float
    q: np.ndarray
    pose: Pose

    def __post_init__(self):
        if not np.isfinite(self.time) or self.time < 0:
            raise ValueError("Invalid sample time")
        object.__setattr__(self, "q", vector(self.q, 7))


@dataclass(frozen=True)
class PlanResult:
    status: Status
    samples: tuple[Sample, ...] = ()
    reason: str = ""

    @property
    def ok(self):
        return self.status == Status.OK


@dataclass(frozen=True, eq=False)
class FeedbackSample:
    q: np.ndarray
    received_at: float

    def __post_init__(self):
        object.__setattr__(self, "q", vector(self.q, 7))
        if not np.isfinite(self.received_at):
            raise ValueError("Invalid receive timestamp")
