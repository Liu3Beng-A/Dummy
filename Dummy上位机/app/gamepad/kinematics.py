"""Joint feedback to flange pose, using firmware_new's six-axis geometry.

The rail translates the displayed robot base. It is not part of the firmware
flange coordinates or the gamepad's TCP control frame.
"""
import math


def _multiply(left, right):
    return [[sum(left[row][k] * right[k][column] for k in range(3))
             for column in range(3)] for row in range(3)]


def _rotate(matrix, vector):
    return [sum(matrix[row][k] * vector[k] for k in range(3)) for row in range(3)]


def forward_kinematics(joints):
    """Return firmware XYZ (mm), ABC (roll/pitch/yaw degrees) from J1..J6.

    Accept a seventh finite rail value for complete JPOS samples, but do not
    add it to the flange position. Actual joint feedback is never clipped.
    Floating-point trig can differ slightly from the MCU's CMSIS interpolation.
    """
    if (not isinstance(joints, (list, tuple)) or len(joints) not in (6, 7) or
            any(isinstance(value, bool) or not isinstance(value, (int, float)) or
                not math.isfinite(value) for value in joints)):
        raise ValueError("关节回读必须包含六轴或七轴有限数值")
    home = (0., -math.pi / 2, math.pi / 2, 0., 0., 0.)
    alpha = (-math.pi / 2, 0., math.pi / 2, -math.pi / 2, math.pi / 2, 0.)
    rotations = []
    for joint, offset, twist in zip(joints, home, alpha):
        cq, sq = math.cos(math.radians(math.remainder(joint, 360.)) + offset), math.sin(math.radians(math.remainder(joint, 360.)) + offset)
        ca, sa = math.cos(twist), math.sin(twist)
        rotation = [[cq, -ca * sq, sa * sq], [sq, ca * cq, -sa * cq], [0., sa, ca]]
        rotations.append(rotation if not rotations else _multiply(rotations[-1], rotation))
    # dummy_robot.cpp constructs DOF6Kinematic(.2073, 0, .170, .117, .0695, .113).
    vectors = (_rotate(rotations[0], [0., -207.3, 0.]),
               _rotate(rotations[1], [170., 0., 0.]),
               _rotate(rotations[2], [-69.5, 0., 117.]),
               _rotate(rotations[5], [0., 0., 113.]))
    position = [sum(vector[i] for vector in vectors) for i in range(3)]
    r = rotations[5]
    # Match the new solver: preserve near-gimbal pitch instead of snapping it.
    horizontal = math.hypot(r[0][0], r[1][0])
    pitch = math.atan2(-r[2][0], horizontal)
    if horizontal > .000001:
        yaw = math.atan2(r[1][0], r[0][0])
        roll = math.atan2(r[2][1], r[2][2])
    else:
        roll = 0.
        yaw = math.atan2(-r[0][1], r[1][1])
    angles = [math.degrees(value) for value in (roll, pitch, yaw)]
    angles = [180. if abs(abs(value) - 180.) < 1e-10 else value for value in angles]
    return [0. if abs(value) < 1e-10 else value for value in position + angles]
