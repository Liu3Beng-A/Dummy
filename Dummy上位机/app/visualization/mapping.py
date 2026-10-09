"""Display conversions. They never alter outgoing robot commands.

J4/J5 signs and the gripper mapping follow the offline mathematical/mesh
comparison; their physical direction still needs individual axis validation.
Firmware FK has a 165 mm base height, versus 227.3 mm in this exported URDF.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

JOINT_LIMITS = ((-175.0, 175.0), (-75.0, 90.0), (0.0, 180.0),
                (-270.0, 270.0), (-100.0, 100.0), (-180.0, 180.0),
                (-250.0, 250.0))
REST_POSE = (0.0, -75.0, 180.0, 0.0, 0.0, 0.0, 0.0)
MODEL_Z_OFFSET_M = 0.0623
JOINT_NAMES = tuple(f"Joint{i}" for i in range(1, 7)) + ("rail",)


def finite_values(values: Sequence[float], count: int) -> list[float]:
    if len(values) != count:
        raise ValueError(f"Expected {count} values, got {len(values)}")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError("Robot display values must be finite")
    return result


def normalize_pose(pose: Sequence[float], *, preview: bool) -> list[float]:
    values = finite_values(pose, 7)
    if preview:
        return [min(max(value, low), high)
                for value, (low, high) in zip(values, JOINT_LIMITS)]
    # Actual telemetry must not be silently clipped to rounded URDF limits.
    return values


def firmware_to_urdf(pose: Sequence[float], gripper_pct: float = 100.0,
                     *, preview: bool = False) -> dict[str, float]:
    values = normalize_pose(pose, preview=preview)
    pct = finite_values([gripper_pct], 1)[0]
    finger = 0.026 * (1.0 - min(max(pct, 0.0), 100.0) / 100.0)
    result = {name: math.radians(value) * (-1 if i in (3, 4) else 1)
              for i, (name, value) in enumerate(zip(JOINT_NAMES[:6], values[:6]))}
    result.update(rail=values[6] / 1000.0, Hand_R=finger, Hand_L=finger)
    return result


def cartesian_marker(pose: Sequence[float], rail_mm: float = 0.0) -> dict:
    values = finite_values(pose, 6)
    rail = finite_values([rail_mm], 1)[0]
    return {
        "position": [values[0] / 1000.0,
                     (values[1] + rail) / 1000.0,
                     values[2] / 1000.0 + MODEL_Z_OFFSET_M],
        "orientation": [math.radians(value) for value in values[3:]],
    }
