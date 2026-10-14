"""Native MoveL frame/tool helpers; PC planning remains offline only."""
from enum import Enum
from dummy_motion import Tool, Pose
from dummy_motion.frames import from_xyzabc, to_xyzabc
from app.core.gamepad_session import motion_command
import numpy as np


class CartesianBackend(str, Enum):
    FIRMWARE = "firmware"
    STREAM = "stream"  # Deprecated identity for old callers; never selectable.


def configured_tool(controller):
    # Existing user tool length is retained. It is a nominal model value until
    # measured/calibrated; the full transform already exists in the package API.
    length = controller.gamepad.robot["tool_offset_z"] / 1000
    return Tool(Pose([0, 0, length], np.eye(3)))


def point_tool(point, fallback):
    return fallback if point.tool is None else Tool(from_xyzabc(point.tool), "point-snapshot")


def world_target(model, point, rail, tool):
    pose = from_xyzabc(point.target)
    if point.frame == "world" and point.target_kind == "tcp":
        return pose
    if point.frame == "base" and point.target_kind == "flange":
        return model.legacy_target(pose, rail, tool)
    raise ValueError("Unsupported Cartesian frame/target combination")


def native_command(model, target, rail, tool, speed):
    return motion_command("@", to_xyzabc(model.native_target(target, rail, tool)) + [speed])
