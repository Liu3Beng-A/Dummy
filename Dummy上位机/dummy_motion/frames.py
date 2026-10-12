"""Explicit transforms; XYZABC at the protocol boundary uses mm and RPY degrees."""
import warnings
import numpy as np
from scipy.spatial.transform import Rotation
from .types import Pose, Tool, vector


def compose(a: Pose, b: Pose):
    return Pose(a.position + a.rotation @ b.position, a.rotation @ b.rotation)


def inverse(pose: Pose):
    return Pose(-pose.rotation.T @ pose.position, pose.rotation.T)


def from_xyzabc(values):
    values = vector(values, 6)
    return Pose(values[:3] / 1000, Rotation.from_euler("xyz", values[3:], degrees=True).as_matrix())


def to_xyzabc(pose):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)  # Equivalent RPY at gimbal lock.
        angles = Rotation.from_matrix(pose.rotation).as_euler("xyz", degrees=True)
    return np.r_[pose.position * 1000, angles].tolist()


def from_wire_joints(values):
    values = vector(values, 7)
    return np.r_[np.deg2rad(values[:6]), values[6] / 1000]


def to_wire_joints(q):
    q = vector(q, 7)
    return np.r_[np.rad2deg(q[:6]), q[6] * 1000].tolist()


def error(actual, target):
    # Spatial angular error: same world frame as the geometric Jacobian.
    return np.r_[target.position - actual.position,
                 Rotation.from_matrix(target.rotation @ actual.rotation.T).as_rotvec()]


def interpolate(start, end, fraction):
    if not np.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError("Invalid interpolation fraction")
    rv = Rotation.from_matrix(end.rotation @ start.rotation.T).as_rotvec()
    return Pose(start.position + fraction * (end.position - start.position),
                Rotation.from_rotvec(fraction * rv).as_matrix() @ start.rotation)


def flange_to_tcp(flange, tool: Tool):
    return compose(flange, tool.flange_tcp)


def tcp_to_flange(tcp, tool: Tool):
    return compose(tcp, inverse(tool.flange_tcp))
