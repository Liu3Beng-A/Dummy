"""Firmware geometry, independent of URDF display offsets/sign conventions."""
import math
import numpy as np
from .types import Pose, Tool, Constraints, vector
from .frames import compose, inverse, flange_to_tcp, tcp_to_flange


class RobotModel:
    home = np.array([0, -math.pi/2, math.pi/2, 0, 0, 0])
    alpha = np.array([-math.pi/2, 0, math.pi/2, -math.pi/2, math.pi/2, 0])
    offsets = np.array([[0, -.2073, 0], [.170, 0, 0], [-.0695, 0, .117],
                        [0, 0, 0], [0, 0, 0], [0, 0, .113]])

    def __init__(self, world_rail=None, constraints=None):
        self.world_rail = world_rail or Pose.identity()
        self.constraints = constraints or Constraints()

    def base_pose(self, rail):
        if not np.isfinite(rail):
            raise ValueError("Invalid rail coordinate")
        return compose(self.world_rail, Pose([0, rail, 0], np.eye(3)))

    def _chain(self, q):
        q = vector(q, 7)
        r = np.eye(3)
        p = np.zeros(3)
        origins, axes = [], []
        for angle, alpha, offset in zip(q[:6] + self.home, self.alpha, self.offsets):
            origins.append(p.copy())
            axes.append(r[:, 2].copy())
            c, s, ca, sa = math.cos(angle), math.sin(angle), math.cos(alpha), math.sin(alpha)
            r = r @ np.array([[c, -ca*s, sa*s], [s, ca*c, -sa*c], [0, sa, ca]])
            p = p + r @ offset
        return p, r, origins, axes

    def fk(self, q, tool=None, frame="world"):
        p, r, _, _ = self._chain(q)
        result = flange_to_tcp(Pose(p, r), tool or Tool())
        if frame == "base":
            return result
        if frame != "world":
            raise ValueError("Frame must be world or base")
        return compose(self.base_pose(q[6]), result)

    def jacobian(self, q, tool=None):
        p, r, origins, axes = self._chain(q)
        p = p + r @ (tool or Tool()).flange_tcp.position
        base_r = self.world_rail.rotation
        jac = np.zeros((6, 7))
        for j, (origin, axis) in enumerate(zip(origins, axes)):
            jac[:3, j] = base_r @ np.cross(axis, p - origin)
            jac[3:, j] = base_r @ axis
        jac[:3, 6] = base_r[:, 1]
        return jac

    def legacy_target(self, flange, rail, tool=None):
        """Freeze an old base-flange point to a world TCP using the entry rail."""
        return compose(self.base_pose(rail), flange_to_tcp(flange, tool or Tool()))

    def native_target(self, world_tcp, rail, tool=None):
        return tcp_to_flange(compose(inverse(self.base_pose(rail)), world_tcp), tool or Tool())

    def valid(self, q):
        q = vector(q, 7)
        return bool(np.all(q >= self.constraints.lower) and np.all(q <= self.constraints.upper))
