"""Pure Xbox-compatible decoding, with no SDL, Qt, or serial dependency."""
from __future__ import annotations

from collections.abc import Sequence
import math
from numbers import Real, Integral

from .config import _normalize_gamepad


def _sequence(value, label):
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ValueError(f"{label} 必须是输入列表")
    return value


def _finite(value, label, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{label} 必须为有限数值")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError(f"{label} 必须为有限数值")
    if (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(f"{label} 超出归一化范围")
    return float(value)


def decode_gamepad(axes, buttons, hats, cfg):
    """Decode a full snapshot atomically; reject absent or nonfinite inputs."""
    cfg = _normalize_gamepad(cfg)
    axes, buttons, hats = (_sequence(value, label) for value, label in
                           ((axes, "轴"), (buttons, "按钮"), (hats, "方向帽")))
    if (max(cfg["axes"].values()) >= len(axes) or max(cfg["buttons"].values()) >= len(buttons)
            or cfg["hat"] >= len(hats)):
        raise ValueError("手柄布局与配置不符；请核对轴、按键和方向帽编号")
    axes = [_finite(value, "轴") for value in axes]
    if any(not isinstance(value, (Integral, bool)) or value not in (0, 1) for value in buttons):
        raise ValueError("按键必须是 0 或 1")
    hat = _sequence(hats[cfg["hat"]], "方向帽")
    if len(hat) != 2 or any(isinstance(value, bool) or not isinstance(value, Integral)
                            or value not in (-1, 0, 1) for value in hat):
        raise ValueError("方向帽必须是两项 -1/0/1")
    states = {key: index >= 0 and bool(buttons[index]) for key, index in cfg["buttons"].items()}
    deadzone = cfg["deadzone"]

    def stick(name):
        value = max(-1., min(1., axes[cfg["axes"][name]]))
        return 0. if abs(value) <= deadzone else math.copysign((abs(value) - deadzone) / (1 - deadzone), value)

    def trigger(name):
        value = (axes[cfg["axes"][name]] - cfg["trigger_min"]) / (cfg["trigger_max"] - cfg["trigger_min"])
        value = max(0., min(1., value))
        return 0. if value <= cfg["trigger_deadzone"] else value

    decoded = {"lx": stick("left_x"), "ly": stick("left_y"), "ry": stick("right_y"),
               "lt": trigger("lt"), "rt": trigger("rt"), "hat": tuple(hat),
               "roll": int(states["roll_plus"]) - int(states["roll_minus"]), "buttons": states}
    # Opposing shoulders can cancel rotation, but neither is a released control.
    decoded["centered"] = not any((decoded["lx"], decoded["ly"], decoded["ry"],
                                   decoded["lt"], decoded["rt"], states["roll_plus"],
                                   states["roll_minus"], *hat))
    return decoded


def motion_delta(decoded, control, dt):
    """Return position mm and attitude degrees; cap stalls at 50ms per sample.

    Three-axis translation is limited to the configured scalar speed, including
    diagonal input. Attitude retains the original per-axis rotation behavior.
    """
    try:
        dt = max(0., min(.05, _finite(dt, "dt")))
        linear = _finite(control["linear_speed_mm_s"], "线速度", 0) * dt
        angular = _finite(control["angular_speed_deg_s"], "角速度", 0) * dt
        vector = [-_finite(decoded[key], key, -1, 1) for key in ("ly", "lx", "ry")]
        length = math.sqrt(sum(value * value for value in vector))
        scale = linear / max(1., length)
        hat = _sequence(decoded["hat"], "方向帽")
        if len(hat) != 2:
            raise ValueError("方向帽必须有两项")
        rotation = (_finite(decoded["roll"], "roll", -1, 1),
                    _finite(hat[1], "hat y", -1, 1), _finite(hat[0], "hat x", -1, 1))
    except (KeyError, TypeError, AttributeError) as exc:
        raise ValueError("运动输入或速度配置不完整") from exc
    return (dict(zip(("x", "y", "z"), (value * scale for value in vector))),
            dict(zip(("r", "p", "y"), (value * angular for value in rotation))))
