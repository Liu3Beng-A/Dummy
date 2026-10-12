"""The two backends share frame/tool contracts, but retain their own execution."""
from enum import Enum
from dummy_motion import RobotModel, Tool, Pose, MotionLimits
from dummy_motion.frames import from_xyzabc, to_xyzabc, from_wire_joints
from app.core.gamepad_session import motion_command
import numpy as np


class CartesianBackend(str, Enum):
    FIRMWARE = "firmware"
    HOST = "host"


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


def host_limits(speed):
    """UI MoveL speed controls only Cartesian planning time and rate.

    At 100 the PC envelope is 20 mm/s and 10 deg/s. Each wire MoveJ speed
    is derived separately from the segment geometry, time and motor model.
    """
    speed = float(speed)
    if not np.isfinite(speed) or not 1 <= speed <= 100:
        raise ValueError("Speed must be 1..100")
    return MotionLimits(linear_speed=.02*speed/100,
                        angular_speed=np.deg2rad(10)*speed/100)
