"""Independent automatic RGB banks, atomic protocol writes and legacy compatibility."""
from copy import deepcopy
import math
import queue
import unittest

from PySide6.QtCore import QCoreApplication

from app.core.controller import RobotController
from app.core.rgb_simulator import (MOTION_STATES, RgbSimulator, default_config,
                                    _config, _select_scene, _render)
from tests.test_controller import FakePort, ManualScheduler
from tests.test_rgb_controller import READBACK


def motion_readback():
    rows = [line.replace("mode=8", "mode=13") if line.startswith("RGB_STATUS ") else line
            for line in READBACK]
    rows.insert(5, "RGB_CAPS motion=1")
    at = next(i for i, line in enumerate(rows) if line.startswith("RGB_STATE "))
    rows[at:at] = [f"RGB_MOTION {slot} {1400 + slot * 300} {slot % 2} {slot + 3}" for slot in range(4)]
    for index, line in enumerate(rows):
        for slot, state in enumerate(MOTION_STATES):
            if line.startswith(f"RGB_STATE {state} "):
                tokens = line.split()
                tokens[2] = str(13 + slot)
                rows[index] = " ".join(tokens)
    return rows


class MotionReadbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.c = RobotController(scheduler=ManualScheduler(), serial_factory=FakePort)
        self.c.is_connected, self.c.serial_port = True, FakePort()

    def tearDown(self):
        self.c.close()

    def feed(self, rows):
        for row in rows:
            self.c._process_rx_line(row)

    def commands(self):
        result = []
        while True:
            try:
                result.append(self.c._tx_queue.get_nowait()[1])
            except queue.Empty:
                return result

    def test_capability_requires_four_valid_records_and_complete_config(self):
        rows = motion_readback()
        self.feed(rows[:-1])
        self.assertFalse(self.c.rgb_snapshot()["motion_supported"])
        self.feed(rows[-1:])
        view = self.c.rgb_snapshot()
        self.assertTrue(view["complete"])
        self.assertTrue(view["motion_supported"])
        self.assertEqual(view["config"]["motion"], {slot: (1400 + slot * 300, slot % 2, slot + 3) for slot in range(4)})
        self.assertEqual([view["config"]["states"][name][0] for name in MOTION_STATES], [13, 14, 15, 16])
        self.assertEqual(self.c._rgb_motion_params[0].get(), "1400")
        self.assertEqual(self.c._rgb_motion_kind.current(), 0)
        self.c._rgb_motion_idx.current(3)
        self.c._rgb_fill_motion()
        self.assertEqual([value.get() for value in self.c._rgb_motion_params], ["2300", "6"])
        self.assertTrue(self.c._rgb_motion_reverse.get())
        self.assertEqual(self.c._rgb_motion_kind.current(), 3)
        self.assertFalse(self.c._rgb_applying_readback)
        view["config"]["motion"].clear()
        self.assertEqual(len(self.c.rgb_snapshot()["config"]["motion"]), 4)

    def test_legacy_config_completes_without_advertising_new_capability(self):
        self.feed(READBACK)
        snapshot = self.c.rgb_snapshot()
        self.assertTrue(snapshot["complete"])
        self.assertFalse(snapshot["motion_supported"])
        self.assertEqual(snapshot["config"]["states"]["RUN_SEQ"][0], 8)
        self.assertEqual(snapshot["config"]["motion"][3], (4000, 1, 3))
        sim = RgbSimulator()
        sim.frame(snapshot["config"], "RUN_SEQ", 0, source="RUN_SEQ")
        self.assertEqual(sim.mode, 8)
        self.assertFalse(self.c.send_cmd("!RGB_MOTION 0 0 1400 0 3"))
        self.assertFalse(self.c.send_cmd("!RGB_SET_STATE RUN_SEQ 13 AUTO_SEQ"))
        self.assertTrue(self.c.send_cmd("!RGB_MODE 8"))
        self.assertEqual(self.commands(), ["!RGB_MODE 8"])

    def test_new_cap_invalid_or_missing_rows_never_fall_back_to_old_schema(self):
        rows = motion_readback()
        cases = []
        for slot in range(4):
            cases.append([line for line in rows if not line.startswith(f"RGB_MOTION {slot} ")])
        for invalid in ("RGB_MOTION 0 249 0 3", "RGB_MOTION 0 30001 0 3", "RGB_MOTION 0 1400 2 3",
                        "RGB_MOTION 0 1400 0 0", "RGB_MOTION 0 1400 0 13", "RGB_MOTION 0 1400 0 3 extra",
                        "RGB_CAPS motion=2", "RGB_CAPS motion=1 extra=1", "RGB_STATE STOP 13 1 255 0 0"):
            cases.append(rows[:-1] + [invalid, rows[-1]])
        cases.append(rows[:-1] + ["RGB_CAPS motion=1", rows[-1]])
        cases.append(rows[:-1] + ["RGB_MOTION 0 1400 0 3", rows[-1]])
        cases.append([line for line in rows if not line.startswith("RGB_CAPS ")])
        # A repeated CONFIG must not clear the capability declaration and
        # turn a truncated new schema into an accepted legacy response.
        legacy_with_cap = list(READBACK)
        legacy_with_cap[5:5] = ["RGB_CAPS motion=1", READBACK[5]]
        cases.append(legacy_with_cap)
        for case in cases:
            with self.subTest(case=case):
                self.c._reset_rgb_state(emit=False)
                self.feed(case)
                self.assertFalse(self.c.rgb_snapshot()["complete"])
                self.assertFalse(self.c.rgb_snapshot()["motion_supported"])
                self.assertFalse(self.c.send_cmd("!RGB_MOTION 0 1 1400 0 3"))
                self.assertEqual(self.commands(), [])

    def test_duplicate_config_and_timing_records_are_invalid(self):
        for duplicate in (READBACK[5], READBACK[6]):
            self.c._reset_rgb_state(emit=False)
            self.feed(READBACK[:-1] + [duplicate, READBACK[-1]])
            self.assertFalse(self.c.rgb_snapshot()["complete"])

    def test_apply_only_one_atomic_command_and_snapshot_waits_for_readback(self):
        self.feed(motion_readback())
        confirmed = self.c.rgb_snapshot()
        self.c._rgb_motion_idx.current(3)
        self.c._rgb_motion_kind.current(2)
        self.c._rgb_motion_reverse.set(True)
        self.c._rgb_motion_params[0].set("2700")
        self.c._rgb_motion_params[1].set("7")
        self.c._rgb_apply_motion()
        self.assertEqual(self.commands(), ["!RGB_MOTION 3 2 2700 1 7"])
        self.assertEqual(self.c.rgb_snapshot(), confirmed)
        for command in ("!RGB_MODE 13", "!RGB_MODE 16", "!RGB_SET_STATE BOOT 13", "!RGB_SET_STATE STOP 16",
                        "!RGB_MOTION 4 0 1400 0 3", "!RGB_MOTION 0 4 1400 0 3", "!RGB_MOTION 0 0 1400 0 0"):
            self.assertFalse(self.c.send_cmd(command), command)
        self.assertEqual(self.commands(), [])

    def test_short_runtime_retains_motion_cap_and_reconnect_resets_it(self):
        self.feed(motion_readback())
        self.feed(motion_readback()[:5])
        self.assertTrue(self.c.rgb_snapshot()["motion_supported"])
        self.c._reset_rgb_state(emit=False)
        self.assertFalse(self.c.rgb_snapshot()["motion_supported"])
        self.feed(READBACK)
        self.assertTrue(self.c.rgb_snapshot()["complete"])
        self.assertFalse(self.c.rgb_snapshot()["motion_supported"])


class MotionEquationTests(unittest.TestCase):
    def test_default_circle_matches_old_preset8_but_is_independent_after_edits(self):
        current = default_config()
        self.assertEqual([current["states"][name][0] for name in MOTION_STATES], [13] * 4)
        old = deepcopy(current)
        for name in MOTION_STATES:
            old["states"][name] = (8,) + old["states"][name][1:]
        for name in MOTION_STATES:
            new_scene, old_scene = _select_scene(_config(current), name, None), _select_scene(_config(old), name, None)
            for elapsed in range(0, 3000, 17):
                self.assertEqual(_render(new_scene, elapsed), _render(old_scene, elapsed))
        before = _select_scene(_config(current), "RUN_SEQ", None)
        current["effects"][8] = (255, 0, 255, 255, 255, 255, 30000, 1, 12, 20, 20)
        self.assertEqual(before, _select_scene(_config(current), "RUN_SEQ", None))
        current["states"]["RUN_SEQ"] = (13, 0, 0, 0, 0)
        self.assertEqual(_select_scene(_config(current), "RUN_SEQ", None).solid, (0, 255, 0))

    def test_four_animations_are_periodic_bounded_and_have_distinct_shapes(self):
        config = default_config()
        shapes = []
        for kind in range(4):
            config["states"]["RUN_INT"] = (13 + kind, 1, 80, 160, 240)
            for reverse in (0, 1):
                for width in (1, 3, 12):
                    config["motion"][1] = (1400, reverse, width)
                    scene = _select_scene(_config(config), "RUN_INT", None)
                    full_cycle = 2800 if kind == 3 else 1400
                    for elapsed in range(0, 1400, 31):
                        frame = _render(scene, elapsed)
                        repeated = _render(scene, elapsed + full_cycle)
                        self.assertEqual(frame, repeated)
                        self.assertTrue(all(0 <= channel <= limit for led in frame for channel, limit in zip(led, (80, 160, 240))))
                    if reverse == 0 and width == 3:
                        shapes.append(_render(scene, 350))
        self.assertEqual(len(set(shapes)), 4)

    def test_two_peaks_and_convergence_cycle_have_expected_geometry(self):
        config = default_config()
        config["states"]["RUN_SEQ"] = (15, 1, 255, 0, 0)
        scene = _select_scene(_config(config), "RUN_SEQ", None)
        frame = _render(scene, 0)
        self.assertAlmostEqual(frame[0][0], 255 * .9)
        self.assertAlmostEqual(frame[12][0], frame[0][0])
        self.assertAlmostEqual(frame[6][0], 255 * .04)
        config["states"]["RUN_SEQ"] = (16, 1, 255, 0, 0)
        scene = _select_scene(_config(config), "RUN_SEQ", None)
        start = _render(scene, 0)
        self.assertAlmostEqual(start[12][0], 255 * .9)
        self.assertAlmostEqual(start[0][0], 255 * .04)
        at_half = _render(scene, 700)
        self.assertGreater(at_half[6][0], at_half[0][0])
        self.assertAlmostEqual(at_half[6][0], at_half[18][0])

    def test_convergence_reverses_continuously_with_fixed_width_and_peak(self):
        config = default_config()
        for slot, state in enumerate(MOTION_STATES):
            config["states"][state] = (16, 1, 80, 160, 240)
            for period in (250, 800, 1400, 30000):
                for reverse in (0, 1):
                    for width in (1, 3, 12):
                        config["motion"][slot] = (period, reverse, width)
                        scene = _select_scene(_config(config), state, None)
                        for tick in (0, 1, period // 7, period // 3, period * 2 // 3, period - 1):
                            first = _render(scene, tick)
                            second = _render(scene, tick + period)
                            third = _render(scene, tick + 2 * period)
                            self.assertEqual(first, third)
                            for led in range(24):
                                for channel in range(3):
                                    self.assertAlmostEqual(second[led][channel], first[(led + 12) % 24][channel], places=10)
                        # Both sides of a turn are the same position. The head
                        # remains lit and there is no endpoint width bloom.
                        for boundary in (period, 2 * period):
                            before, endpoint, after = (_render(scene, boundary + offset) for offset in (-1, 0, 1))
                            self.assertAlmostEqual(max(led[2] for led in endpoint), 240 * .9)
                            endpoint_led = (12 if reverse else 0) if boundary == period else (0 if reverse else 12)
                            width_turns = .055 * width / 3.0
                            for led in range(24):
                                distance = min((led - endpoint_led) % 24, (endpoint_led - led) % 24) / 24
                                gaussian = math.exp(-.5 * (distance / width_turns) ** 2)
                                expected = 240 * (.04 + .86 * (2 * gaussian - gaussian * gaussian))
                                self.assertAlmostEqual(endpoint[led][2], expected, places=10)
                                self.assertAlmostEqual(before[led][2], after[led][2], places=10)
                                self.assertLess(abs(before[led][2] - endpoint[led][2]) / 240, .001)
        # Under the model's common 90-degree registration LED12 is front, LED0 rear.
        config["motion"][0] = (1400, 0, 3)
        scene = _select_scene(_config(config), "RUN_SEQ", None)
        outbound = _render(scene, 140)
        returning = _render(scene, 1540)
        self.assertGreater(outbound[12][2], outbound[0][2])
        self.assertGreater(returning[0][2], returning[12][2])

    def test_convergence_800ms_head_has_no_relight_jump_at_20ms_updates(self):
        config = default_config()
        config["states"]["RUN_SEQ"] = (16, 1, 255, 0, 0)
        for reverse in (0, 1):
            config["motion"][0] = (800, reverse, 3)
            scene = _select_scene(_config(config), "RUN_SEQ", None)
            peaks = [max(led[0] for led in _render(scene, tick)) / 255
                     for tick in range(0, 1601, 20)]
            # About five percentage points for phase-aligned 20ms frames;
            # also bound every integer-ms start phase, including jitter.
            self.assertLess(max(b - a for a, b in zip(peaks, peaks[1:])), .051)
            all_phases = [max(led[0] for led in _render(scene, tick)) / 255
                          for tick in range(1620)]
            self.assertLess(max(all_phases[tick + 20] - all_phases[tick]
                                for tick in range(1600)), .059)
            self.assertLessEqual(max(peaks), .9 + 1e-12)
            for boundary in (0, 800, 1600):
                self.assertAlmostEqual(peaks[boundary // 20], .9)
            # Before/after both turns stay at a stable head rather than
            # falling to the background and starting another fade-in.
            self.assertGreater(min(peaks[index] for index in (1, 39, 41, 79)), .899)

    def test_invalid_automatic_modes_cannot_reach_manual_or_other_states(self):
        config = default_config()
        for mode in range(13, 17):
            bad = deepcopy(config)
            bad["manual_mode"] = mode
            with self.assertRaises(ValueError):
                _config(bad)
            for name in ("BOOT", "DISABLED", "READY_SEQ", "RUN_TUN", "STOP", "FAULT"):
                bad = deepcopy(config)
                bad["states"][name] = (mode, 0, 0, 0, 0)
                with self.assertRaises(ValueError):
                    _config(bad)
        for values in ((249, 0, 3), (30001, 0, 3), (1400, 2, 3), (1400, 0, 0), (1400, 0, 13)):
            bad = deepcopy(config)
            bad["motion"][2] = values
            with self.assertRaises(ValueError):
                _config(bad)

    def test_missing_motion_only_accepted_for_actual_legacy_bindings(self):
        legacy = default_config()
        del legacy["motion"]
        with self.assertRaises(ValueError):
            _config(legacy)
        for name in MOTION_STATES:
            legacy["states"][name] = (8,) + legacy["states"][name][1:]
        self.assertEqual(_config(legacy).motion, ((1400, False, 3),) * 4)


if __name__ == "__main__":
    unittest.main()
