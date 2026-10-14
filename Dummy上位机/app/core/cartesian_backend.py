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


def tcp_command(target, tool, speed, *, rail_auto=True):
    """One world/TCP target and immutable tool snapshot; all planning is on MCU."""
    if not isinstance(rail_auto, (bool, int, float, np.bool_, np.integer, np.floating)) or rail_auto not in (0, 1):
        raise ValueError("自动地轨标志必须为0或1")
    if not 1 <= float(speed) <= 100:
        raise ValueError("速度档位必须为1～100")
    values = to_xyzabc(target) + [float(speed), int(bool(rail_auto))] + to_xyzabc(tool.flange_tcp)
    if not np.isfinite(values).all() or np.any(np.abs(values[8:11]) > 500):
        raise ValueError("TCP指令需要有限数值，工具偏移不得超过±500mm")
    fields = [f"{value:.4f}".rstrip("0").rstrip(".") for value in values]
    fields = ["0" if value in ("-0", "") else value for value in fields]
    command = "@T" + ",".join(fields)
    if len(command.encode("ascii")) > 159:
        raise ValueError("TCP指令超过主控159字节限制")
    return command
