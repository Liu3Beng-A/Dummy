"""Validated, independent gamepad profiles, without serial or vision settings."""
from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
import math


_JOINT_LIMITS = [[-175, 175], [-75, 90], [0, 180], [-270, 270], [-100, 100], [-180, 180]]
_DEFAULTS = {
    "gamepad": {
        "device_index": 0, "device_guid": "", "deadzone": .15, "trigger_deadzone": .1,
        "axes": {"left_x": 0, "left_y": 1, "right_y": 3, "lt": 4, "rt": 5},
        "buttons": {"start": 7, "stop": 6, "initialize": 0, "rest": 3,
                    "roll_minus": 4, "roll_plus": 5},
        "hat": 0, "trigger_min": -1., "trigger_max": 1.,
    },
    "control": {
        "linear_speed_mm_s": 200., "angular_speed_deg_s": 50., "gripper_speed_percent_s": 150.,
        "send_interval_s": .1, "velocity_smoothing_s": .06,
        "gripper_interval_s": .1, "feedback_interval_s": .2,
        "feedback_timeout_s": 1.5, "max_position_lead_mm": 5., "max_rotation_lead_deg": 5.,
        "response_timeout_s": 3.,
    },
    "robot": {
        "tool_offset_z": 114., "init_joints": [0., 10., 80., 0., 90., 0.],
        "rest_joints": [0., -75., 180., 0., 0., 0.], "cmd_speed": 35., "init_speed": 10.,
        "motion_timeout_s": 60., "z_min_height": -150.,
        "flange_limits": {"x": [-50., 300.], "y": [-100., 260.], "z": [-150., 300.]},
        "joint_limits": _JOINT_LIMITS, "rail_limits": [-250., 250.],
    },
}


def default_config() -> dict:
    return deepcopy(_DEFAULTS)


def _merge(base, value, label):
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} 必须是对象")
    unknown = set(value) - set(base)
    if unknown:
        raise ValueError(f"{label} 包含不支持的配置项：{', '.join(map(str, unknown))}")
    result = deepcopy(base)
    for key, item in value.items():
        result[key] = _merge(base[key], item, f"{label}.{key}") if isinstance(base[key], dict) else deepcopy(item)
    return result


def _number(value, label, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} 必须为有限数值")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f"{label} 必须为有限数值")
    if high is None:
        if value < low:
            raise ValueError(f"{label} 必须不小于 {low:g}")
    elif not low <= value <= high:
        raise ValueError(f"{label} 超出范围 {low:g}～{high:g}")
    return float(value)


def _index(value, label):
    if type(value) is not int or not 0 <= value <= 255:
        raise ValueError(f"{label} 必须为 0～255 的整数")
    return value


def _bounds(value, label, low, high):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"{label} 必须为 [最小值, 最大值]")
    pair = [_number(item, label, low, high) for item in value]
    if pair[0] >= pair[1]:
        raise ValueError(f"{label} 最小值必须小于最大值")
    return pair


def _compatible_gamepad(value):
    """Retire B/resync and preserve any existing A assignment in old profiles."""
    if not isinstance(value, Mapping):
        return value
    result = deepcopy(dict(value))
    if isinstance(result.get("buttons"), Mapping):
        result["buttons"] = dict(result["buttons"])
        result["buttons"].pop("manual", None)
        if "initialize" not in result["buttons"] and 0 in result["buttons"].values():
            result["buttons"]["initialize"] = -1
    return result


def _normalize_gamepad(value):
    result = _merge(_DEFAULTS["gamepad"], _compatible_gamepad(value), "gamepad")
    for key in ("device_index", "hat"):
        result[key] = _index(result[key], f"gamepad.{key}")
    guid = result["device_guid"]
    if not isinstance(guid, str) or len(guid) > 128:
        raise ValueError("gamepad.device_guid 必须是最多 128 个字符的字符串")
    result["device_guid"] = guid.strip().lower()
    for key in ("deadzone", "trigger_deadzone"):
        result[key] = _number(result[key], f"gamepad.{key}", 0, .95)
    for bank in ("axes", "buttons"):
        for key, index in result[bank].items():
            if bank == "buttons" and key == "initialize" and type(index) is int and index == -1:
                continue  # Only the optional initialization binding may be disabled.
            result[bank][key] = _index(index, f"gamepad.{bank}.{key}")
        active = [index for index in result[bank].values() if index >= 0]
        if len(set(active)) != len(active):
            raise ValueError(f"gamepad.{bank} 不能把多个控制功能映射到同一编号")
    for key in ("trigger_min", "trigger_max"):
        result[key] = _number(result[key], f"gamepad.{key}", -1, 1)
    if result["trigger_min"] >= result["trigger_max"]:
        raise ValueError("gamepad.trigger_min 必须小于 trigger_max")
    return result


def validate_config(config) -> dict:
    """Normalize partial overrides against defaults and return a deep copy.

    Profiles contain only gamepad/control/robot. Local limits may narrow the
    known firmware joint/rail bounds, never enlarge them. Linear and angular
    target rates must be finite and positive, with no application upper limit.
    Gripper rates, command steps and send/query intervals remain bounded.
    """
    if isinstance(config, Mapping):
        config = dict(config)
        if "gamepad" in config:
            config["gamepad"] = _compatible_gamepad(config["gamepad"])
    result = _merge(_DEFAULTS, config, "手柄配置")
    result["gamepad"] = _normalize_gamepad(result["gamepad"])
    control = result["control"]
    limits = {
        "linear_speed_mm_s": (.01, None), "angular_speed_deg_s": (.01, None),
        "gripper_speed_percent_s": (.01, 150), "send_interval_s": (.02, 1),
        "velocity_smoothing_s": (0, .5),
        "gripper_interval_s": (.02, 1), "feedback_interval_s": (.05, 5),
        "feedback_timeout_s": (.1, 30), "response_timeout_s": (.1, 30),
        "max_position_lead_mm": (.1, 50), "max_rotation_lead_deg": (.1, 45),
    }
    for key, (low, high) in limits.items():
        control[key] = _number(control[key], f"control.{key}", low, high)
    if control["feedback_timeout_s"] <= control["feedback_interval_s"]:
        raise ValueError("feedback_timeout_s 必须大于 feedback_interval_s")
    robot = result["robot"]
    robot["tool_offset_z"] = _number(robot["tool_offset_z"], "robot.tool_offset_z", 0, 1000)
    for key in ("cmd_speed", "init_speed"):
        robot[key] = _number(robot[key], f"robot.{key}", 1, 100)
    robot["motion_timeout_s"] = _number(robot["motion_timeout_s"], "robot.motion_timeout_s", 1, 600)
    if not isinstance(robot["joint_limits"], (list, tuple)) or len(robot["joint_limits"]) != 6:
        raise ValueError("robot.joint_limits 必须包含六轴范围")
    robot["joint_limits"] = [_bounds(pair, "robot.joint_limits", *allowed)
                             for pair, allowed in zip(robot["joint_limits"], _JOINT_LIMITS)]
    for key in ("init_joints", "rest_joints"):
        values = robot[key]
        if not isinstance(values, (list, tuple)) or len(values) != 6:
            raise ValueError(f"robot.{key} 必须包含六个关节角")
        robot[key] = [_number(value, f"robot.{key}", *allowed)
                      for value, allowed in zip(values, robot["joint_limits"])]
    robot["rail_limits"] = _bounds(robot["rail_limits"], "robot.rail_limits", -250, 250)
    if not robot["rail_limits"][0] <= 0 <= robot["rail_limits"][1]:
        raise ValueError("手柄地轨目标固定为 0；robot.rail_limits 必须包含 0")
    # Legacy profile fields remain readable; gamepad MoveL no longer applies
    # these workspace or minimum-height bounds.
    for key in ("x", "y", "z"):
        robot["flange_limits"][key] = _bounds(robot["flange_limits"][key], f"robot.flange_limits.{key}", -10000, 10000)
    robot["z_min_height"] = _number(robot["z_min_height"], "robot.z_min_height", *robot["flange_limits"]["z"])
    return result
