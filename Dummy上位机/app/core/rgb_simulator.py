"""Offline preview of the 24-LED firmware RGB renderer, without Qt or transport.

The scene selection, animation equations, smooth transitions, brightness ramps,
and gamma table follow firmware_new/dummy-ref-core-fw/Bsp/gpio/rgb.cpp. Frames
are logical RGB, rather than the firmware's GRB wire order. Python calculations
use double precision, so a channel can differ by one from MCU float rounding.
Device status has no scene/ramp timestamps or LED frames: a following preview
can match settings and state, but cannot recover the device's animation phase.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
import math


LED_COUNT = 24
MODE_COUNT = 13
STATE_NAMES = (
    "BOOT", "DISABLED", "READY_SEQ", "READY_INT", "READY_TRJ", "READY_TUN",
    "RUN_SEQ", "RUN_INT", "RUN_TRJ", "RUN_TUN", "CALIBRATION", "STOP",
    "STARTING", "FAULT",
)
_PROTECTED_STATES = frozenset(("FAULT", "STOP", "CALIBRATION", "STARTING"))
_TICK_MASK = 0xFFFFFFFF
_GAMMA = (
    0, 1, 1, 1, 1, 1, 2, 2, 3, 3, 4, 5, 6, 6, 7, 8, 9, 11, 12, 13,
    14, 15, 17, 18, 20, 21, 23, 24, 26, 27, 29, 31, 33, 35, 37, 39, 41,
    43, 45, 47, 49, 51, 54, 56, 58, 61, 63, 66, 68, 71, 73, 76, 79, 81,
    84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 114, 117, 121, 124, 127,
    131, 134, 138, 141, 145, 148, 152, 156, 159, 163, 167, 171, 175, 178,
    182, 186, 190, 194, 198, 203, 207, 211, 215, 219, 224, 228, 233, 237,
    241, 246, 250, 255,
)
Color = tuple[int, int, int]


def default_config() -> dict:
    """Return fresh editable firmware defaults, with no shared nested mappings.

    An effect is (r, g, b, r2, g2, b2, period_ms, reverse, tail, on_ms,
    off_ms). A state binding is (mode, color_source, r, g, b). Color sources
    0/1/2/3/4 mean preset/explicit RGB/auto SEQ/auto INT/auto TRJ.
    """
    effects = {mode: (255, 255, 255, 0, 0, 0, 4000, 0, 3, 200, 1000)
               for mode in range(3, 10)}
    effects[4] = (0, 0, 255, 0, 0, 0, 4000, 0, 3, 200, 1000)
    effects[5] = (255, 255, 255, 0, 0, 0, 3800, 0, 3, 200, 1000)
    effects[6] = (0, 255, 255, 160, 0, 255, 4200, 0, 3, 200, 1000)
    effects[7] = (255, 0, 0, 0, 0, 0, 1800, 0, 3, 200, 1000)
    effects[8] = (0, 255, 0, 0, 0, 0, 1400, 0, 3, 200, 1000)
    return {
        "enabled": True, "automatic": True, "brightness": 100,
        "manual_mode": 0, "fade_on_ms": 1000, "fade_off_ms": 1000,
        "brightness_ms": 300, "transition_ms": 300,
        "colors": {0: (0, 0, 255), 1: (0, 255, 0), 2: (255, 255, 255)},
        "auto_colors": {1: (0, 255, 0), 2: (0, 255, 255), 3: (0, 80, 255)},
        "effects": effects,
        "states": {
            "BOOT": (3, 0, 0, 0, 255), "DISABLED": (5, 0, 255, 255, 255),
            "READY_SEQ": (10, 0, 0, 0, 0), "READY_INT": (11, 0, 0, 0, 0),
            "READY_TRJ": (12, 0, 0, 0, 0), "READY_TUN": (2, 1, 160, 0, 255),
            "RUN_SEQ": (8, 2, 0, 0, 0), "RUN_INT": (8, 3, 0, 0, 0),
            "RUN_TRJ": (8, 4, 0, 0, 0), "RUN_TUN": (5, 1, 160, 0, 255),
            "CALIBRATION": (8, 1, 255, 180, 0), "STOP": (9, 1, 255, 180, 0),
            "STARTING": (4, 1, 0, 255, 255), "FAULT": (7, 1, 255, 0, 0),
        },
    }


def _integer(value, minimum: int, maximum: int, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f"{label} must be an integer in {minimum}..{maximum}")
    return value


def _boolean(value, label: str) -> bool:
    if not isinstance(value, (bool, int)) or value not in (0, 1):
        raise ValueError(f"{label} must be boolean")
    return bool(value)


def _sequence(value, length: int, label: str) -> tuple:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != length:
        raise ValueError(f"{label} must have {length} items")
    return tuple(value)


def _color(value, label: str) -> Color:
    return tuple(_integer(channel, 0, 255, label) for channel in _sequence(value, 3, label))


@dataclass(frozen=True)
class _Effect:
    first: Color = (255, 255, 255)
    second: Color = (0, 0, 0)
    period: int = 4000
    reverse: bool = False
    tail: int = 3
    on: int = 200
    off: int = 1000


@dataclass(frozen=True)
class _Scene:
    mode: int
    source: str
    solid: Color
    effect: _Effect
    scale: float


@dataclass(frozen=True)
class _Config:
    enabled: bool
    automatic: bool
    brightness: int
    manual_mode: int
    fade_on_ms: int
    fade_off_ms: int
    brightness_ms: int
    transition_ms: int
    colors: tuple[Color, ...]
    auto_colors: tuple[Color, ...]
    effects: tuple[_Effect, ...]
    states: tuple[tuple, ...]


def _config(value: Mapping) -> _Config:
    """Validate and detach caller-owned data before advancing any runtime state."""
    if not isinstance(value, Mapping):
        raise ValueError("config must be a mapping")
    try:
        colors = tuple(_color(value["colors"][i], f"colors[{i}]") for i in range(3))
        auto = tuple(_color(value["auto_colors"][i], f"auto_colors[{i}]") for i in range(1, 4))
        effects = [_Effect() for _ in range(3)]
        for i in range(3, 10):
            e = _sequence(value["effects"][i], 11, f"effects[{i}]")
            effects.append(_Effect(
                _color(e[:3], f"effects[{i}].first"), _color(e[3:6], f"effects[{i}].second"),
                _integer(e[6], 250, 30000, "period_ms"), _boolean(e[7], "reverse"),
                _integer(e[8], 1, 12, "tail"), _integer(e[9], 20, 30000, "on_ms"),
                _integer(e[10], 20, 30000, "off_ms"),
            ))
        states = []
        for name in STATE_NAMES:
            binding = _sequence(value["states"][name], 5, f"states[{name}]")
            states.append((_integer(binding[0], 0, MODE_COUNT - 1, "state mode"),
                           _integer(binding[1], 0, 4, "color_source"),
                           *_color(binding[2:], f"states[{name}].color")))
        if states[-1] != (7, 1, 255, 0, 0):
            raise ValueError("FAULT binding must be fixed red heartbeat (7, 1, 255, 0, 0)")
        return _Config(
            _boolean(value["enabled"], "enabled"), _boolean(value["automatic"], "automatic"),
            _integer(value["brightness"], 0, 100, "brightness"),
            _integer(value["manual_mode"], 0, MODE_COUNT - 1, "manual_mode"),
            _integer(value["fade_on_ms"], 0, 10000, "fade_on_ms"),
            _integer(value["fade_off_ms"], 0, 10000, "fade_off_ms"),
            _integer(value["brightness_ms"], 0, 10000, "brightness_ms"),
            _integer(value["transition_ms"], 0, 5000, "transition_ms"),
            colors, auto, tuple(effects), tuple(states),
        )
    except (KeyError, TypeError, IndexError) as exc:
        raise ValueError("config is missing a required field or contains an invalid bank") from exc


def _state(value) -> str:
    if isinstance(value, int) and not isinstance(value, bool) and 0 <= value < len(STATE_NAMES):
        return STATE_NAMES[value]
    if isinstance(value, str) and value in STATE_NAMES:
        return value
    raise ValueError(f"state must be one of {', '.join(STATE_NAMES)}")


def _smooth(t: float) -> float:
    return 1.0 if t >= 1 else 0.0 if t <= 0 else t * t * (3 - 2 * t)


def _elapsed(now: int, then: int) -> int:
    return (now - then) & _TICK_MASK


def _gain(brightness: float) -> float:
    index = brightness * 100
    if index <= 0:
        return 0.0
    if index >= 100:
        return 1.0
    lo = int(index)
    return (_GAMMA[lo] + (_GAMMA[lo + 1] - _GAMMA[lo]) * (index - lo)) / 255


def _select_scene(c: _Config, state: str, source: str | None) -> _Scene:
    # A reported source selects the accepted scene; protection always takes priority.
    if state in _PROTECTED_STATES:
        selected = state
    elif source in STATE_NAMES:
        selected = source
    elif source == "MANUAL":
        selected = "MANUAL"
    else:
        selected = state if c.automatic else "MANUAL"
    binding = c.states[STATE_NAMES.index(selected if selected != "MANUAL" else state)]
    mode = c.manual_mode if selected == "MANUAL" else binding[0]
    effect = c.effects[mode if mode < 10 else 0]
    solid = c.colors[mode] if mode < 3 else c.auto_colors[mode - 10] if mode >= 10 else effect.first
    if selected != "MANUAL" and binding[1]:
        solid = binding[2:] if binding[1] == 1 else c.auto_colors[binding[1] - 2]
        effect = replace(effect, first=solid)
    scale = 0.15 if selected == "DISABLED" else 0.25 if selected.startswith("READY_") else 1.0
    if state == "FAULT":
        mode, solid = 7, (255, 0, 0)
        effect = _Effect(first=solid, period=1800)
    return _Scene(mode, selected, solid, effect, scale)


def _render(scene: _Scene, elapsed: int) -> tuple[tuple[float, float, float], ...]:
    e = scene.effect
    p = (elapsed % e.period) / e.period
    if e.reverse:
        p = 1 - p
    amplitude, mix = 1.0, 0.0
    if scene.mode == 4:
        amplitude = 0.08 + 0.52 * (0.5 + 0.5 * math.cos(2 * math.pi * p))
    elif scene.mode in (5, 6):
        amplitude = 0.02 + 0.98 * (0.5 - 0.5 * math.cos(2 * math.pi * p))
    elif scene.mode == 7:
        amplitude = (0.04 + 0.96 * math.sin(math.pi * p / 0.16) if p < 0.16 else
                     0.04 + 0.74 * math.sin(math.pi * (p - 0.20) / 0.16)
                     if 0.20 <= p < 0.36 else 0.04)
    elif scene.mode == 9:
        amplitude = 1.0 if elapsed % (e.on + e.off) < e.on else 0.0
    if scene.mode == 6:
        mix = 0.5 - 0.5 * math.cos(2 * math.pi * p)
    output = []
    for i in range(LED_COUNT):
        color, level = scene.solid, amplitude
        if scene.mode == 3:
            hue = (p + i / LED_COUNT) % 1
            x = hue * 6
            segment, f = int(x), x - math.floor(x)
            color = ((255, 255 * f, 0), (255 * (1 - f), 255, 0),
                     (0, 255, 255 * f), (0, 255 * (1 - f), 255),
                     (255 * f, 0, 255), (255, 0, 255 * (1 - f)))[segment]
        elif scene.mode == 6:
            color = tuple(first + (second - first) * mix for first, second in zip(color, e.second))
        elif scene.mode == 8:
            head = p * LED_COUNT
            distance = i - head if e.reverse else head - i
            if distance < 0:
                distance += LED_COUNT
            if distance >= LED_COUNT:
                distance -= LED_COUNT
            level = 0.04
            if distance < e.tail:
                level += 0.96 * _smooth(1 - distance / e.tail)
        output.append(tuple(channel * level * scene.scale for channel in color))
    return tuple(output)


class RgbSimulator:
    """Stateful firmware preview driven by elapsed milliseconds, not frame count.

    Call frame repeatedly with an editable complete config and a physical state
    name (or 0..13 index). A reported source BOOT/MANUAL/state name suppresses
    inferred local BOOT and chooses that scene, except for physical protection
    states. OFF also suppresses BOOT, but never cuts short an ongoing fade-out:
    config.enabled/brightness still determine brightness. With source=None,
    local DISABLED startup shows BOOT for the first two seconds like firmware.

    now_ms is a nonnegative elapsed or monotonic integer millisecond clock. The
    low 32 bits and unsigned elapsed subtraction match MCU tick wrap. Calls
    should be chronological; resetting the time origin requires reset().
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self, initial_brightness: float | None = None) -> None:
        """Restart local phase; optionally seed current brightness in 0..100.

        A device's first complete snapshot can seed 100% without an invented
        fade-on. If seeded below the configured target, the next frame starts
        a local ramp. A positive seed implies previously enabled; the protocol
        does not expose the previous ramp start or elapsed time.
        """
        if initial_brightness is not None and (
                isinstance(initial_brightness, bool) or
                not isinstance(initial_brightness, (int, float)) or
                not math.isfinite(initial_brightness) or not 0 <= initial_brightness <= 100):
            raise ValueError("initial_brightness must be finite in 0..100")
        seed = 0.0 if initial_brightness is None else initial_brightness / 100
        self._brightness = self._ramp_from = self._ramp_target = seed
        self._ramp_at = self._ramp_ms = 0
        self._last_enabled = initial_brightness is not None and initial_brightness > 0
        self._scene: _Scene | None = None
        self._scene_at = self._blend_at = self._blend_ms = 0
        self._frame = self._from_frame = ((0.0, 0.0, 0.0),) * LED_COUNT
        self._boot_at: int | None = None
        self._boot_pending = True
        self._source = "OFF"

    @property
    def current_brightness(self) -> int:
        return int(self._brightness * 100 + 0.5)

    @property
    def mode(self) -> int:
        return self._scene.mode if self._scene is not None else 0

    @property
    def source(self) -> str:
        return self._source

    def frame(self, config: Mapping, state: str | int, now_ms: int,
              *, source: str | None = None) -> list[Color]:
        c, physical = _config(config), _state(state)
        if source is not None and (not isinstance(source, str) or
                                   source not in (*STATE_NAMES, "MANUAL", "OFF")):
            raise ValueError("source must be a state name, MANUAL, OFF, or None")
        now = _integer(now_ms, 0, (1 << 63) - 1, "now_ms") & _TICK_MASK
        if self._boot_at is None:
            self._boot_at = now
        if physical != "DISABLED" or _elapsed(now, self._boot_at) >= 2000:
            self._boot_pending = False
        visible = ("BOOT" if source is None and self._boot_pending and c.automatic and
                   physical == "DISABLED" else physical)
        next_scene = _select_scene(c, visible, source)
        if self._scene != next_scene:
            self._from_frame = self._frame
            self._scene_at = self._blend_at = now
            self._blend_ms = 0 if self._scene is None or visible in ("FAULT", "STOP") else c.transition_ms
            self._scene = next_scene
        rendered = _render(self._scene, _elapsed(now, self._scene_at))
        blend = _smooth(_elapsed(now, self._blend_at) / self._blend_ms) if self._blend_ms else 1.0
        self._frame = tuple(tuple(a + (b - a) * blend for a, b in zip(old, new))
                            for old, new in zip(self._from_frame, rendered))
        progress = _smooth(_elapsed(now, self._ramp_at) / self._ramp_ms) if self._ramp_ms else 1.0
        self._brightness = self._ramp_from + (self._ramp_target - self._ramp_from) * progress
        target = c.brightness / 100 if c.enabled else 0.0
        if target != self._ramp_target or c.enabled != self._last_enabled:
            self._ramp_from, self._ramp_target, self._ramp_at = self._brightness, target, now
            self._ramp_ms = (c.fade_on_ms if c.enabled else c.fade_off_ms
                             ) if c.enabled != self._last_enabled else c.brightness_ms
            self._last_enabled = c.enabled
            if self._ramp_ms == 0:
                self._brightness = target
        self._source = "OFF" if not c.enabled or not c.brightness else self._scene.source
        gain = _gain(self._brightness)
        return [tuple(min(255, max(0, int(channel * gain + 0.5))) for channel in led)
                for led in self._frame]
