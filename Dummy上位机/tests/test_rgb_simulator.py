"""Equation-derived RGB fixtures; no Qt, serial connection, or robot required."""

from copy import deepcopy
import importlib.util
import re
import unittest
from pathlib import Path

from app.core.rgb_simulator import (LED_COUNT, MODE_COUNT, PRESET_MODE_COUNT, STATE_NAMES, RgbSimulator,
                                    default_config, _spin_level, _rainbow_color,
                                    _config, _select_scene, _quantize_frame)


def manual_config(mode=2):
    config = default_config()
    config.update(automatic=False, manual_mode=mode, fade_on_ms=0,
                  fade_off_ms=0, brightness_ms=0, transition_ms=0)
    return config


class RainbowTableTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = Path(__file__).resolve().parents[2]
        spec = importlib.util.spec_from_file_location("rainbow_generator", cls.repo / "tools/generate_rgb_rainbow.py")
        cls.generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.generator)

    def test_generator_matches_both_checked_in_tables(self):
        for path, content in self.generator.outputs(self.repo).items():
            self.assertEqual(path.read_text(encoding="utf-8"), content, str(path))

    def test_color_conversion_primary_references_and_roundtrip(self):
        references = (((1, 0, 0), (.627955361, .224863061, .125846299)),
                      ((0, 1, 0), (.866439612, -.233887574, .179498480)),
                      ((0, 0, 1), (.452013718, -.032456984, -.311528148)))
        for rgb, expected in references:
            lab = self.generator.to_oklab(rgb)
            for actual, reference in zip(lab, expected):
                self.assertAlmostEqual(actual, reference, places=7)
            for actual, reference in zip(self.generator.from_oklab(lab), rgb):
                self.assertAlmostEqual(actual, reference, places=6)
        self.assertAlmostEqual(self.generator.decode_srgb(128), .2158605001)

    def test_new_rainbow_default_does_not_change_other_periods(self):
        effects = default_config()["effects"]
        self.assertEqual({mode: value[6] for mode, value in effects.items()},
                         {3: 6000, 4: 4000, 5: 3800, 6: 4200, 7: 1800, 8: 1400, 9: 4000})


class FirmwareFormulaTests(unittest.TestCase):
    """Firmware and preview share both rotation and seven-color rainbow."""

    def render_at(self, mode, elapsed=0, *, reverse=False):
        config = manual_config(mode)
        if reverse:
            effect = list(config["effects"][mode])
            effect[7] = 1
            config["effects"][mode] = effect
        simulator = RgbSimulator()
        simulator.frame(config, "DISABLED", 0)
        return simulator.frame(config, "DISABLED", elapsed)

    def test_all_thirteen_mode_first_frames(self):
        # Full-brightness samples, with +0.5 byte rounding (never Python round()).
        expected = ((0, 0, 255), (0, 255, 0), (255, 255, 255), (255, 0, 0),
                    (0, 0, 153), (5, 5, 5), (0, 5, 5), (10, 0, 0),
                    (0, 255, 0), (255, 255, 255), (0, 255, 0),
                    (0, 255, 255), (0, 80, 255))
        self.assertEqual(PRESET_MODE_COUNT, len(expected))
        self.assertEqual(MODE_COUNT, 17)
        for mode, color in enumerate(expected):
            with self.subTest(mode=mode):
                frame = self.render_at(mode)
                self.assertEqual(len(frame), LED_COUNT)
                self.assertEqual(frame[0], color)
                self.assertTrue(all(len(rgb) == 3 and all(type(v) is int and 0 <= v <= 255
                                                         for v in rgb) for rgb in frame))

    def test_rainbow_spatial_colors_period_and_reverse(self):
        frame = self.render_at(3)
        self.assertEqual(frame[::4], [(255, 0, 0), (255, 152, 0), (0, 255, 19),
                                     (0, 159, 255), (0, 18, 255), (83, 8, 191)])
        self.assertEqual(self.render_at(3, 1000)[0], (255, 152, 0))
        self.assertEqual(self.render_at(3, 1000, reverse=True)[0], (83, 8, 191))
        self.assertEqual(self.render_at(3, 6000), frame)
        reverse = self.render_at(3, 0, reverse=True)
        self.assertEqual(reverse, frame)
        self.assertEqual(self.render_at(3, 250), frame[1:] + frame[:1])

    def test_rainbow_visits_seven_colors_and_wraps_without_a_hard_edge(self):
        # Designed sRGB anchors become linear PWM; orange/violet are not
        # mistaken for equal channel duties on the physical LEDs.
        anchors = [(255, 0, 0), (255, 54, 0), (255, 255, 0),
                   (0, 255, 0), (0, 255, 255), (0, 0, 255), (66, 0, 255)]
        samples = [_rainbow_color(index / 6000) for index in range(6001)]
        for expected in anchors:
            # Unequal perceptual arc lengths get unequal widths; all seven
            # landmarks must still be reached within LUT/phase resolution.
            self.assertLess(min(max(abs(a-b) for a, b in zip(sample, expected))
                                for sample in samples), 5)
        self.assertEqual(samples[0], samples[-1])
        self.assertTrue(all(0 <= value <= 255 for sample in samples for value in sample))
        self.assertLess(max(abs(a-b) for first, second in zip(samples, samples[1:])
                            for a, b in zip(first, second)), 1.5)
        # LED 24 approaches LED 1 through violet/red, rather than restarting
        # a modulo-7 list that would end on yellow.
        self.assertGreater(_rainbow_color(23/24)[0], _rainbow_color(23/24)[2])
        self.assertEqual(_rainbow_color(1), _rainbow_color(0))

    def test_rainbow_retains_custom_period_and_direction(self):
        config = manual_config(3)
        effect = list(config["effects"][3])
        effect[6] = 4000
        config["effects"][3] = effect
        simulator = RgbSimulator()
        first = simulator.frame(config, "DISABLED", 0)
        self.assertEqual(simulator.frame(config, "DISABLED", 1000)[0], (174, 255, 0))
        self.assertEqual(simulator.frame(config, "DISABLED", 4000), first)
        self.assertEqual(config["effects"][3][6], 4000)

    def test_tide_breath_cyber_and_heartbeat_time_samples(self):
        fixtures = (
            (4, 1000, (0, 0, 87)), (4, 2000, (0, 0, 20)),
            (5, 1900, (255, 255, 255)),
            (6, 1050, (41, 65, 130)), (6, 2100, (160, 0, 255)),
            (7, 144, (255, 0, 0)), (7, 504, (199, 0, 0)), (7, 900, (10, 0, 0)),
        )
        for mode, elapsed, rgb in fixtures:
            with self.subTest(mode=mode, elapsed=elapsed):
                self.assertEqual(self.render_at(mode, elapsed)[0], rgb)

    def test_spin_tail_wrap_and_reverse_direction(self):
        frame = self.render_at(8)
        self.assertEqual([frame[i][1] for i in (0, 23, 22, 21, 1)], [255, 232, 170, 95, 71])
        forward = self.render_at(8, 350)
        backward = self.render_at(8, 350, reverse=True)
        self.assertEqual([forward[i][1] for i in (6, 5, 4, 7)], [255, 232, 170, 71])
        self.assertEqual([backward[i][1] for i in (18, 19, 20, 17)], [255, 232, 170, 71])
        self.assertEqual(self.render_at(8, 1400), frame)

    def test_spin_front_peak_tail_and_ring_seam_are_continuous(self):
        epsilon = 1e-6
        for tail in (1, 3, 12):
            with self.subTest(tail=tail):
                rear = tail + 2
                self.assertEqual(_spin_level(0, tail), 1)
                self.assertEqual(_spin_level(rear, tail), .04)
                self.assertEqual(_spin_level(-1.5, tail), .04)
                for boundary in (-1.5, 0, rear, LED_COUNT):
                    center = _spin_level(boundary, tail)
                    left = _spin_level(boundary - epsilon, tail)
                    right = _spin_level(boundary + epsilon, tail)
                    self.assertLess(abs(left - center), 1e-10)
                    self.assertLess(abs(right - center), 1e-10)
                    self.assertLess(abs((right - left) / (2 * epsilon)), 1e-5)
                samples = [_spin_level(index / 100, tail) for index in range(2400)]
                self.assertTrue(all(.04 <= value <= 1 for value in samples))
                self.assertTrue(any(value == .04 for value in samples))
                self.assertLess(max(abs(value - _spin_level(index / 100 + LED_COUNT, tail))
                                    for index, value in enumerate(samples)), 1e-12)

    def test_blink_uses_on_off_and_ignores_period_and_reverse(self):
        for elapsed, expected in ((0, 255), (199, 255), (200, 0), (1199, 0), (1200, 255)):
            with self.subTest(elapsed=elapsed):
                frame = self.render_at(9, elapsed, reverse=True)
                self.assertEqual(frame, [(expected,) * 3] * LED_COUNT)

    def test_effect_custom_colors_second_color_and_tail(self):
        config = manual_config(6)
        config["effects"][6] = (100, 0, 0, 0, 100, 0, 1000, 0, 1, 20, 20)
        simulator = RgbSimulator()
        self.assertEqual(simulator.frame(config, "DISABLED", 0)[0], (2, 0, 0))
        self.assertEqual(simulator.frame(config, "DISABLED", 500)[0], (0, 100, 0))
        config["manual_mode"] = 8
        config["effects"][8] = (100, 0, 0, 0, 0, 0, 1000, 0, 1, 20, 20)
        frame = simulator.frame(config, "DISABLED", 501)
        self.assertEqual(frame[0], (100, 0, 0))
        self.assertEqual(frame[23], (76, 0, 0))
        self.assertEqual(frame[21], (4, 0, 0))
        self.assertEqual(frame[1], (28, 0, 0))


class SceneSelectionTests(unittest.TestCase):
    def setUp(self):
        self.config = manual_config()
        self.config["automatic"] = True
        self.simulator = RgbSimulator()

    def test_ready_scaling_and_auto_color_banks(self):
        for state, mode, color in (("READY_SEQ", 10, (0, 64, 0)),
                                   ("READY_INT", 11, (0, 64, 64)),
                                   ("READY_TRJ", 12, (0, 20, 64)),
                                   ("READY_TUN", 2, (40, 0, 64))):
            with self.subTest(state=state):
                self.assertEqual(self.simulator.frame(self.config, state, 0)[0], color)
                self.assertEqual(self.simulator.mode, mode)
                self.assertEqual(self.simulator.source, state)
        self.config["auto_colors"][1] = (100, 40, 20)
        self.assertEqual(self.simulator.frame(self.config, "RUN_SEQ", 1)[0], (100, 40, 20))
        self.config["states"]["RUN_INT"] = (0, 4, 255, 255, 255)
        self.assertEqual(self.simulator.frame(self.config, "RUN_INT", 2)[0], (0, 80, 255))

    def test_disabled_scaling_and_local_boot_duration(self):
        first = self.simulator.frame(self.config, "DISABLED", 100)
        self.assertEqual(first[0], (255, 0, 0))
        self.assertEqual(self.simulator.source, "BOOT")
        self.simulator.frame(self.config, "DISABLED", 2099)
        self.assertEqual(self.simulator.source, "BOOT")
        self.assertEqual(self.simulator.frame(self.config, "DISABLED", 2100)[0], (1, 1, 1))
        self.assertEqual(self.simulator.source, "DISABLED")
        peak = self.simulator.frame(self.config, "DISABLED", 4000)
        self.assertEqual(sum(led[0] for led in peak), 918)  # 24 * 38.25.
        self.assertEqual(set(peak), {(38, 38, 38), (39, 39, 39)})

    def test_leaving_disabled_cancels_boot_permanently(self):
        self.simulator.frame(self.config, "DISABLED", 0)
        self.simulator.frame(self.config, "RUN_SEQ", 10)
        self.simulator.frame(self.config, "DISABLED", 20)
        self.assertEqual(self.simulator.source, "DISABLED")

    def test_reported_source_suppresses_fake_boot_and_selects_manual(self):
        self.assertEqual(self.simulator.frame(self.config, "DISABLED", 0,
                                             source="DISABLED")[0], (1, 1, 1))
        self.assertEqual(self.simulator.source, "DISABLED")
        self.assertEqual(self.simulator.frame(self.config, "READY_SEQ", 1,
                                             source="MANUAL")[0], (255, 255, 255))
        self.assertEqual(self.simulator.source, "MANUAL")
        self.assertEqual(self.simulator.frame(self.config, "DISABLED", 2,
                                             source="BOOT")[0], (255, 0, 0))

    def test_protection_overrides_manual_and_fault_ignores_edited_effect(self):
        self.config["automatic"] = False
        self.config["effects"][7] = (0, 255, 255, 0, 0, 0, 30000, 1, 12, 20, 20)
        for state, mode, color in (("CALIBRATION", 13, (255, 180, 0)),
                                   ("STOP", 7, (10, 7, 0)),
                                   ("STARTING", 4, (0, 153, 153)),
                                   ("FAULT", 7, (10, 0, 0))):
            with self.subTest(state=state):
                self.assertEqual(self.simulator.frame(self.config, state, 0,
                                                     source="MANUAL")[0], color)
                self.assertEqual(self.simulator.mode, mode)
                self.assertEqual(self.simulator.source, state)
        self.assertEqual(self.simulator.frame(self.config, "FAULT", 144,
                                             source="MANUAL")[0], (255, 0, 0))

    def test_stop_heartbeat_matches_fixed_fault_timing_with_binding_color(self):
        self.config["effects"][7] = (0, 255, 255, 0, 0, 0, 30000, 1, 12, 20, 20)
        self.assertEqual(self.simulator.frame(self.config, "STOP", 0)[0], (10, 7, 0))
        self.assertEqual(self.simulator.frame(self.config, "STOP", 144)[0], (255, 180, 0))
        self.assertEqual(self.simulator.frame(self.config, "STOP", 504)[0], (199, 140, 0))
        self.assertEqual(self.simulator.frame(self.config, "STOP", 1800)[0], (10, 7, 0))
        self.config["states"]["STOP"] = (7, 1, 60, 200, 20)
        self.simulator.frame(self.config, "STOP", 1801)
        self.assertEqual(self.simulator.frame(self.config, "STOP", 1945)[0], (60, 200, 20))

    def test_device_stop_binding_can_keep_another_effect(self):
        self.config["states"]["STOP"] = (9, 1, 60, 200, 20)
        self.assertEqual(self.simulator.frame(self.config, "STOP", 0)[0], (60, 200, 20))
        self.assertEqual(self.simulator.frame(self.config, "STOP", 250)[0], (0, 0, 0))

    def test_scene_blends_from_previous_sample_but_stop_fault_are_immediate(self):
        self.config.update(automatic=False, manual_mode=0, transition_ms=300)
        self.simulator.frame(self.config, "DISABLED", 0)
        self.config["manual_mode"] = 1
        self.assertEqual(self.simulator.frame(self.config, "DISABLED", 100)[0], (0, 0, 255))
        self.assertEqual(self.simulator.frame(self.config, "DISABLED", 250)[0], (0, 128, 128))
        self.assertEqual(self.simulator.frame(self.config, "DISABLED", 400)[0], (0, 255, 0))
        self.assertEqual(self.simulator.frame(self.config, "STOP", 401)[0], (10, 7, 0))
        self.assertEqual(self.simulator.frame(self.config, "FAULT", 402)[0], (10, 0, 0))

    def test_effect_edit_restarts_scene_even_for_unused_effect_field(self):
        self.config.update(automatic=False, manual_mode=3)
        self.simulator.frame(self.config, "DISABLED", 0)
        self.assertEqual(self.simulator.frame(self.config, "DISABLED", 1000)[0], (255, 152, 0))
        effect = list(self.config["effects"][3])
        effect[8] = 4  # Rainbow does not use tail, but firmware SameScene compares it.
        self.config["effects"][3] = effect
        self.assertEqual(self.simulator.frame(self.config, "DISABLED", 1001)[0], (255, 0, 0))


class DisabledWhiteSpatialTests(unittest.TestCase):
    def setUp(self):
        self.config = manual_config()
        self.config["automatic"] = True
        self.scene = _select_scene(_config(self.config), "DISABLED", "DISABLED")

    def quantize(self, level):
        return _quantize_frame(((level,) * 3,) * LED_COUNT, 1, self.scene)

    def test_fractional_white_mean_and_spread(self):
        frame = self.quantize(1.25)
        self.assertEqual(frame.count((1, 1, 1)), 18)
        self.assertEqual(frame.count((2, 2, 2)), 6)
        self.assertEqual(sum(pixel[0] for pixel in frame) / LED_COUNT, 1.25)
        positions = [i for i, pixel in enumerate(frame) if pixel[0] == 2]
        gaps = [(positions[(i + 1) % len(positions)] - pos) % LED_COUNT
                for i, pos in enumerate(positions)]
        self.assertLessEqual(max(gaps), 6)  # No adjacent block of six bright LEDs.
        self.assertEqual(frame, self.quantize(1.25))  # Fixed spatial mask.

    def test_average_error_bound_per_led_monotonicity_and_zero_full_scale(self):
        previous = [0] * LED_COUNT
        for n in range(4097):
            level = 255 * n / 4096
            frame = self.quantize(level)
            self.assertLessEqual(abs(sum(p[0] for p in frame) / LED_COUNT - level),
                                 1 / 48 + 1e-12)
            for i, pixel in enumerate(frame):
                self.assertEqual(len(set(pixel)), 1)
                self.assertGreaterEqual(pixel[0], previous[i])
                previous[i] = pixel[0]
        self.assertEqual(self.quantize(0), [(0, 0, 0)] * LED_COUNT)
        self.assertEqual(self.quantize(255), [(255, 255, 255)] * LED_COUNT)

    def test_other_states_effects_colors_and_nonuniform_transitions_keep_rounding(self):
        from dataclasses import replace
        uniform = ((1.25,) * 3,) * LED_COUNT
        scenes = [replace(self.scene, source=s) for s in (*STATE_NAMES, "MANUAL")
                  if s != "DISABLED"]
        scenes += [replace(self.scene, mode=m) for m in range(MODE_COUNT) if m != 5]
        scenes += [replace(self.scene, solid=(255, 254, 255))]
        for scene in scenes:
            self.assertEqual(_quantize_frame(uniform, 1, scene), [(1, 1, 1)] * LED_COUNT)
        for index in (0, 1, LED_COUNT - 1):
            for channel in range(3):
                frame = [list(p) for p in uniform]
                frame[index][channel] = 2.6
                expected = [tuple(int(c + .5) for c in p) for p in frame]
                self.assertEqual(_quantize_frame(frame, 1, self.scene), expected)

    def test_default_ten_percent_cycle_has_fifteen_average_levels(self):
        self.config["brightness"] = 10
        sparse, dense = RgbSimulator(), RgbSimulator()
        for sim in (sparse, dense):
            sim.reset(initial_brightness=10)
            sim.frame(self.config, "DISABLED", 0, source="DISABLED")
        averages = set()
        for now in range(0, 3800, 20):
            frame = dense.frame(self.config, "DISABLED", now, source="DISABLED")
            averages.add(sum(p[0] for p in frame))
            self.assertTrue(all(p[0] == p[1] == p[2] for p in frame))
        self.assertEqual(len(averages), 15)
        self.assertEqual(sparse.frame(self.config, "DISABLED", 3780, source="DISABLED"), frame)

    def test_disabled_color_transition_and_fade_to_zero_remain_valid(self):
        self.config.update(transition_ms=300, fade_off_ms=1000)
        sim = RgbSimulator()
        sim.frame(self.config, "READY_SEQ", 0, source="READY_SEQ")
        sim.frame(self.config, "DISABLED", 100, source="DISABLED")
        mixed = sim.frame(self.config, "DISABLED", 250, source="DISABLED")
        expected = [tuple(int(c + .5) for c in p) for p in sim._frame]
        self.assertEqual(mixed, expected)
        self.assertTrue(any(p[0] != p[1] for p in mixed))
        complete = sim.frame(self.config, "DISABLED", 400, source="DISABLED")
        self.assertTrue(all(p[0] == p[1] == p[2] for p in complete))
        self.config["enabled"] = False
        sim.frame(self.config, "DISABLED", 400, source="OFF")
        self.assertEqual(sim.frame(self.config, "DISABLED", 1400, source="OFF"),
                         [(0, 0, 0)] * LED_COUNT)
        self.config.update(enabled=True, brightness=0)
        self.assertEqual(sim.frame(self.config, "DISABLED", 2400, source="OFF"),
                         [(0, 0, 0)] * LED_COUNT)

    def test_blend_roundoff_is_ignored_but_small_visible_color_is_preserved(self):
        frame = [[1.25, 1.25 + 1e-5, 1.25 - 1e-5] for _ in range(LED_COUNT)]
        self.assertEqual(_quantize_frame(frame, 1, self.scene), self.quantize(1.25))
        frame[-1][1] += .001
        expected = [tuple(int(c + .5) for c in p) for p in frame]
        self.assertEqual(_quantize_frame(frame, 1, self.scene), expected)


class BrightnessAndClockTests(unittest.TestCase):
    def test_smooth_fade_on_gamma_and_non_shortcut_fade_off(self):
        config = manual_config()
        config.update(fade_on_ms=1000, fade_off_ms=1000)
        simulator = RgbSimulator()
        self.assertEqual(simulator.frame(config, "DISABLED", 0)[0], (0, 0, 0))
        self.assertEqual(simulator.frame(config, "DISABLED", 500)[0], (73, 73, 73))
        self.assertEqual(simulator.current_brightness, 50)
        config["enabled"] = False
        self.assertEqual(simulator.frame(config, "DISABLED", 500, source="OFF")[0], (73, 73, 73))
        self.assertEqual(simulator.source, "OFF")
        self.assertEqual(simulator.frame(config, "DISABLED", 1000, source="OFF")[0], (21, 21, 21))
        self.assertEqual(simulator.current_brightness, 25)
        self.assertEqual(simulator.frame(config, "DISABLED", 1500, source="OFF")[0], (0, 0, 0))

    def test_brightness_change_retargets_from_current_without_jump(self):
        config = manual_config()
        config["brightness_ms"] = 300
        simulator = RgbSimulator()
        simulator.frame(config, "DISABLED", 0)
        config["brightness"] = 0
        self.assertEqual(simulator.frame(config, "DISABLED", 100)[0], (255, 255, 255))
        self.assertEqual(simulator.source, "OFF")
        self.assertEqual(simulator.frame(config, "DISABLED", 250)[0], (73, 73, 73))
        config["brightness"] = 100
        self.assertEqual(simulator.frame(config, "DISABLED", 250)[0], (73, 73, 73))
        self.assertEqual(simulator.frame(config, "DISABLED", 400)[0], (152, 152, 152))
        self.assertEqual(simulator.current_brightness, 75)
        self.assertEqual(simulator.frame(config, "DISABLED", 550)[0], (255, 255, 255))

    def test_device_seed_does_not_invent_fade_on_and_partial_seed_continues(self):
        config = manual_config()
        config.update(fade_on_ms=1000, brightness_ms=300)
        simulator = RgbSimulator()
        simulator.reset(initial_brightness=100)
        self.assertEqual(simulator.frame(config, "DISABLED", 8000, source="MANUAL")[0], (255, 255, 255))
        self.assertEqual(simulator.current_brightness, 100)
        simulator.reset(initial_brightness=50)
        self.assertEqual(simulator.frame(config, "DISABLED", 8000, source="MANUAL")[0], (73, 73, 73))
        self.assertEqual(simulator.frame(config, "DISABLED", 8150, source="MANUAL")[0], (152, 152, 152))
        self.assertEqual(simulator.frame(config, "DISABLED", 8300, source="MANUAL")[0], (255, 255, 255))

    def test_gamma_interpolation_and_positive_half_up_quantization(self):
        config = manual_config()
        config.update(brightness=50, brightness_ms=300)
        simulator = RgbSimulator()
        simulator.reset(initial_brightness=50.5)
        # gamma[50]=73, gamma[51]=76 -> 74.5 -> 75, not bankers rounding.
        self.assertEqual(simulator.frame(config, "DISABLED", 0)[0], (75, 75, 75))
        self.assertEqual(simulator.current_brightness, 51)

    def test_unsigned_tick_wrap_for_animation_and_ramp(self):
        config = manual_config(3)
        simulator = RgbSimulator()
        simulator.frame(config, "DISABLED", 0xFFFFFF00)
        self.assertEqual(simulator.frame(config, "DISABLED", 744)[0], (255, 152, 0))
        config = manual_config()
        config["fade_on_ms"] = 1000
        simulator.reset()
        simulator.frame(config, "DISABLED", 0xFFFFFF00)
        self.assertEqual(simulator.frame(config, "DISABLED", 244)[0], (73, 73, 73))

    def test_elapsed_animation_is_independent_of_frame_rate(self):
        config = manual_config(6)
        sparse, dense = RgbSimulator(), RgbSimulator()
        sparse.frame(config, "DISABLED", 0)
        dense.frame(config, "DISABLED", 0)
        for now in range(1, 1050):
            dense.frame(config, "DISABLED", now)
        self.assertEqual(sparse.frame(config, "DISABLED", 1050),
                         dense.frame(config, "DISABLED", 1050))


class ConfigIntegrityTests(unittest.TestCase):
    def test_defaults_do_not_alias_or_mutate_callers(self):
        first, second = default_config(), default_config()
        first["colors"][0] = (1, 2, 3)
        self.assertEqual(second["colors"][0], (0, 0, 255))
        before = deepcopy(first)
        simulator = RgbSimulator()
        output = simulator.frame(first, "RUN_SEQ", 0)
        self.assertEqual(first, before)
        output[0] = (1, 2, 3)
        self.assertNotEqual(simulator.frame(first, "RUN_SEQ", 0)[0], (1, 2, 3))

    def test_invalid_config_is_rejected_without_advancing_runtime(self):
        config = manual_config(3)
        simulator = RgbSimulator()
        simulator.frame(config, "DISABLED", 0)
        for key, value in (("brightness", 101), ("brightness", -1), ("enabled", 2),
                           ("automatic", "true"), ("manual_mode", 13),
                           ("fade_on_ms", 10001), ("transition_ms", 5001)):
            bad = deepcopy(config)
            bad[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                simulator.frame(bad, "FAULT", 1000)
        self.assertEqual(simulator.mode, 3)
        self.assertEqual(simulator.frame(config, "DISABLED", 1000)[0], (255, 152, 0))

    def test_effect_binding_color_and_missing_bank_validation(self):
        config = manual_config()
        for index, value in ((0, 256), (6, 249), (7, 2), (8, 0), (9, 19), (10, 30001)):
            bad = deepcopy(config)
            effect = list(bad["effects"][3])
            effect[index] = value
            bad["effects"][3] = effect
            with self.subTest(index=index), self.assertRaises(ValueError):
                RgbSimulator().frame(bad, "DISABLED", 0)
        for mutate in (lambda c: c["states"].update(FAULT=(7, 1, 0, 0, 255)),
                       lambda c: c["states"].update(RUN_SEQ=(8, 5, 0, 0, 0)),
                       lambda c: c["colors"].update({0: (0, 0, -1)}),
                       lambda c: c["auto_colors"].pop(3)):
            bad = deepcopy(config)
            mutate(bad)
            with self.assertRaises(ValueError):
                RgbSimulator().frame(bad, "DISABLED", 0)

    def test_invalid_reset_and_frame_inputs_preserve_state(self):
        simulator = RgbSimulator()
        config = manual_config()
        simulator.reset(initial_brightness=100)
        simulator.frame(config, "RUN_SEQ", 0)
        for seed in (-1, 101, float("nan"), float("inf"), True, "100"):
            with self.subTest(seed=seed), self.assertRaises(ValueError):
                simulator.reset(initial_brightness=seed)
        self.assertEqual(simulator.current_brightness, 100)
        for state, now, source in (("typo", 1, None), (14, 1, None),
                                   ("RUN_SEQ", -1, None), ("RUN_SEQ", 1.5, None),
                                   ("RUN_SEQ", 1, "typo")):
            with self.assertRaises(ValueError):
                simulator.frame(config, state, now, source=source)
        self.assertEqual(simulator.source, "MANUAL")

    def test_gamma_table_matches_firmware_source_when_repository_available(self):
        firmware = Path(__file__).resolve().parents[2] / "firmware_new/dummy-ref-core-fw/Bsp/gpio/rgb.cpp"
        if not firmware.is_file():
            self.skipTest("standalone application checkout has no firmware reference")
        source = firmware.read_text(encoding="utf-8")
        values = re.search(r"gammaPercent\[101\]\s*=\s*\{([^}]+)\}", source)
        self.assertIsNotNone(values)
        expected = [int(value) for value in values[1].split(",")]
        self.assertEqual(len(expected), 101)
        config = manual_config()
        simulator = RgbSimulator()
        for percentage, level in enumerate(expected):
            config["brightness"] = percentage
            simulator.reset(initial_brightness=percentage)
            with self.subTest(percentage=percentage):
                self.assertEqual(simulator.frame(config, "DISABLED", 0)[0], (level,) * 3)


if __name__ == "__main__":
    unittest.main()
