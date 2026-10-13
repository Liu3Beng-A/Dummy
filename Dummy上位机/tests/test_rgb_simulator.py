"""Equation-derived RGB fixtures; no Qt, serial connection, or robot required."""

from copy import deepcopy
import re
import unittest
from pathlib import Path

from app.core.rgb_simulator import LED_COUNT, MODE_COUNT, RgbSimulator, default_config


def manual_config(mode=2):
    config = default_config()
    config.update(automatic=False, manual_mode=mode, fade_on_ms=0,
                  fade_off_ms=0, brightness_ms=0, transition_ms=0)
    return config


class FirmwareFormulaTests(unittest.TestCase):
    """Independent sample colors computed from the documented firmware equations."""

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
        self.assertEqual(MODE_COUNT, len(expected))
        for mode, color in enumerate(expected):
            with self.subTest(mode=mode):
                frame = self.render_at(mode)
                self.assertEqual(len(frame), LED_COUNT)
                self.assertEqual(frame[0], color)
                self.assertTrue(all(len(rgb) == 3 and all(type(v) is int and 0 <= v <= 255
                                                         for v in rgb) for rgb in frame))

    def test_rainbow_spatial_colors_period_and_reverse(self):
        frame = self.render_at(3)
        self.assertEqual(frame[::4], [(255, 0, 0), (255, 255, 0), (0, 255, 0),
                                     (0, 255, 255), (0, 0, 255), (255, 0, 255)])
        self.assertEqual(self.render_at(3, 1000)[0], (128, 255, 0))
        self.assertEqual(self.render_at(3, 1000, reverse=True)[0], (128, 0, 255))
        self.assertEqual(self.render_at(3, 4000), frame)

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
        self.assertEqual([frame[i][1] for i in (0, 23, 22, 21, 1)], [255, 192, 74, 10, 10])
        forward = self.render_at(8, 350)
        backward = self.render_at(8, 350, reverse=True)
        self.assertEqual([forward[i][1] for i in (6, 5, 4, 7)], [255, 192, 74, 10])
        self.assertEqual([backward[i][1] for i in (18, 19, 20, 17)], [255, 192, 74, 10])

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
        self.assertEqual(frame[23], (4, 0, 0))


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
        self.assertEqual(self.simulator.frame(self.config, "DISABLED", 4000)[0], (38, 38, 38))

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
        for state, mode, color in (("CALIBRATION", 8, (255, 180, 0)),
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
        self.assertEqual(self.simulator.frame(self.config, "DISABLED", 1000)[0], (128, 255, 0))
        effect = list(self.config["effects"][3])
        effect[8] = 4  # Rainbow does not use tail, but firmware SameScene compares it.
        self.config["effects"][3] = effect
        self.assertEqual(self.simulator.frame(self.config, "DISABLED", 1001)[0], (255, 0, 0))


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
        self.assertEqual(simulator.frame(config, "DISABLED", 744)[0], (128, 255, 0))
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
        self.assertEqual(simulator.frame(config, "DISABLED", 1000)[0], (128, 255, 0))

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
