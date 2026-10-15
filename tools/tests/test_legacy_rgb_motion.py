"""Real hidden Tk RGB controls, recorded commands only; never open a device."""
import importlib.util
from pathlib import Path
import queue
import tkinter as tk
import unittest
from unittest.mock import patch


REPO = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("legacy_rgb_motion_under_test", REPO / "串口助手_old.py")
LEGACY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LEGACY)


class LegacyRgbMotionTests(unittest.TestCase):
    def setUp(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.ports = patch.object(LEGACY.serial.tools.list_ports, "comports", return_value=[])
        self.serial = patch.object(LEGACY.serial, "Serial", side_effect=AssertionError("No real serial"))
        self.ports.start()
        self.serial_mock = self.serial.start()
        self.app = LEGACY.RobotSerialAssistantOld(self.root)
        self.app.is_connected = True
        self.app.serial_port = object()  # send_cmd only queues; no writer or receive thread starts.
        self.app._motor_state_known = True
        self.app._motor_enabled = False
        self.app._stop_locked = False
        self.app._calibration_busy = False
        self.app._seq_active = False

    def tearDown(self):
        self.serial_mock.assert_not_called()
        self.app._finish_rgb_save("测试结束")
        self.app.is_connected = False
        self.root.update()
        self.root.destroy()
        self.serial.stop()
        self.ports.stop()

    def commands(self):
        commands = []
        while True:
            try:
                commands.append(self.app._tx_queue.get_nowait()[1])
            except queue.Empty:
                return commands

    def feed(self, *lines):
        for line in lines:
            self.app._handle_rgb_status(line)

    @staticmethod
    def motion_rows(exclude=None, malformed=None):
        rows = ["RGB_CAPS motion=1"]
        for slot in range(4):
            if slot != exclude:
                rows.append(f"RGB_MOTION {slot} {1400 + slot * 100} {slot % 2} {3 + slot}")
            rows.append(f"RGB_STATE {LEGACY.RGB_MOTION_STATES[slot]} {13 + slot} {2 if slot < 3 else 1} 12 34 56")
        if malformed:
            rows.append(malformed)
        return rows + ["RGB_GET end"]

    def full_motion(self, exclude=None, malformed=None):
        self.feed(*self.motion_rows(exclude, malformed))

    def test_old_configuration_remains_editable_without_new_commands(self):
        self.feed("RGB_COLOR 0 1 2 3", "RGB_STATE RUN_SEQ 8 2 0 0 0", "RGB_GET end")
        self.assertFalse(self.app._rgb_motion_supported)
        self.assertIn("升级主控", self.app._rgb_motion_status.get())
        self.assertEqual(self.app._rgb_cache["colors"][0], [1, 2, 3])
        for editor in self.app._rgb_motions:
            self.assertEqual(str(editor["kind"].cget("state")), "disabled")
        self.app._rgb_apply_motion(0)
        self.assertFalse(self.app.send_cmd("!RGB_MOTION 0 1 2000 0 3"))
        self.assertFalse(self.app.send_cmd("!RGB_SET_STATE RUN_SEQ 13 AUTO_SEQ"))
        self.assertEqual(self.commands(), [])
        self.app._rgb_save()
        self.assertEqual(self.commands(), [])
        self.assertEqual(self.app._rgb_save_request["phase"], "preparing")
        self.assertEqual(self.app._rgb_save_request["command"], "!RGB_SAVE")
        self.assertEqual(len(LEGACY.RGB_MODES), 13)

    def test_capability_requires_complete_parameters_and_state_bindings(self):
        self.feed("RGB_CAPS motion=1")
        self.assertFalse(self.app._rgb_motion_supported)
        self.full_motion(exclude=3)
        self.assertFalse(self.app._rgb_motion_supported)
        self.app._rgb_save()
        self.assertEqual(self.commands(), [])
        for bad in ("RGB_MOTION 0 249 0 3", "RGB_MOTION 0 30001 0 3",
                    "RGB_MOTION 0 1500 2 3", "RGB_MOTION 0 1500 0 13",
                    "RGB_STATE RUN_SEQ 17 2 0 0 0"):
            with self.subTest(bad=bad):
                self.full_motion(malformed=bad)
                self.assertFalse(self.app._rgb_motion_supported)
        self.full_motion()
        self.assertTrue(self.app._rgb_motion_supported)
        # A fresh full reply must not reuse previous state bindings.
        self.feed("RGB_CAPS motion=1", *(f"RGB_MOTION {i} 2000 0 3" for i in range(4)), "RGB_GET end")
        self.assertFalse(self.app._rgb_motion_supported)

    def test_four_targets_readback_and_apply_one_atomic_command(self):
        self.full_motion()
        self.assertTrue(self.app._rgb_motion_supported)
        for slot, editor in enumerate(self.app._rgb_motions):
            self.assertEqual(editor["kind"].current(), slot)
            self.assertEqual(editor["period"].get(), str(1400 + slot * 100))
            self.assertEqual(editor["reverse"].get(), bool(slot % 2))
            self.assertEqual(editor["width"].get(), str(3 + slot))
            self.assertEqual(str(editor["kind"].cget("state")), "readonly")
            editor["kind"].current(3 - slot)
            editor["period"].set("2500")
            editor["width"].set("7")
            editor["reverse"].set(True)
            self.app._rgb_apply_motion(slot)
            self.assertEqual(self.commands(), [f"!RGB_MOTION {slot} {3 - slot} 2500 1 7"])
        self.app._rgb_save()
        self.assertEqual(self.commands(), [])
        self.assertEqual(self.app._rgb_save_request["phase"], "preparing")
        self.assertEqual(self.app._rgb_save_request["command"], "!RGB_SAVE")

    def test_motion_validation_rejects_bad_ranges_without_extra_queries(self):
        self.full_motion()
        editor = self.app._rgb_motions[0]
        for field, text in (("period", "249"), ("period", "30001"), ("width", "0"),
                            ("width", "13"), ("period", "1500.5")):
            editor["period"].set("1500")
            editor["width"].set("3")
            editor[field].set(text)
            self.app._rgb_apply_motion(0)
            self.assertEqual(self.commands(), [])

    def test_state_choices_only_extend_motion_and_calibration(self):
        self.full_motion()
        for state in LEGACY.RGB_MOTION_STATES:
            index = [name for name, _ in LEGACY.RGB_STATES].index(state)
            self.app._rgb_state.current(index)
            self.app._rgb_fill_state()
            self.assertEqual(len(self.app._rgb_state_mode.cget("values")), 17)
            self.app._rgb_state_mode.current(16)
            self.app._rgb_state_source.current(0)
            self.app._rgb_apply_state()
            self.assertEqual(self.commands(), [f"!RGB_SET_STATE {state} 16"])
        self.app._rgb_state.current(0)
        self.app._rgb_fill_state()
        self.assertEqual(len(self.app._rgb_state_mode.cget("values")), 13)
        self.assertFalse(self.app.send_cmd("!RGB_SET_STATE BOOT 16"))
        self.assertFalse(self.app.send_cmd("!RGB_MODE 13"))
        self.assertEqual(self.commands(), [])

    def test_status_accepts_extended_modes_and_reset_clears_capability(self):
        self.full_motion()
        self.feed("RGB_STATUS on=1 auto=1 bright=70 current=70 mode=16",
                  "RGB_RUNTIME state=RUN_INT source=RUN_INT",
                  "RGB_STORAGE dirty=1 storage=1 dma=0",
                  "RGB_DMA frames=10 errors=0", "RGB_TIMEOUT timeouts=0")
        self.assertIn("灯效 16", self.app._rgb_status_var.get())
        before = self.app._rgb_status_var.get()
        self.feed("RGB_STATUS on=1 auto=1 bright=70 current=70 mode=17",
                  "RGB_RUNTIME state=RUN_INT source=RUN_INT",
                  "RGB_STORAGE dirty=1 storage=1 dma=0",
                  "RGB_DMA frames=10 errors=0", "RGB_TIMEOUT timeouts=0")
        self.assertEqual(self.app._rgb_status_var.get(), before)
        self.app._rgb_reset_motion_capability()
        self.assertFalse(self.app._rgb_motion_supported)
        self.assertFalse(self.app._rgb_motion_caps)
        self.assertEqual(self.app._rgb_cache["motions"], {})
        self.app._rgb_apply_motion(3)
        self.assertEqual(self.commands(), [])

    def test_actual_tk_layout_has_four_distinct_rows(self):
        self.full_motion()
        self.root.update_idletasks()
        for editor in self.app._rgb_motions:
            block = editor["kind"].master
            columns = [int(widget.grid_info()["column"]) for widget in editor["widgets"]]
            self.assertEqual(columns, [0, 2, 4, 5, 6])
            self.assertLessEqual(block.winfo_reqwidth(), 540)
            self.assertEqual(block.grid_size(), (7, 1))

    def test_duplicate_slots_states_and_caps_poison_the_whole_extension(self):
        for extra in ("RGB_CAPS motion=1", "RGB_CAPS motion=2", "RGB_CAPS", "RGB_CAPS motion=1 extra=1",
                      *(f"RGB_MOTION {i} 2000 0 3" for i in range(4)),
                      *(f"RGB_STATE {state} 13 1 1 2 3" for state in LEGACY.RGB_MOTION_STATES)):
            with self.subTest(extra=extra):
                self.assertTrue(self.app.send_cmd("#RGB_GET"))
                self.assertEqual(self.commands(), ["#RGB_GET"])
                rows = self.motion_rows()
                self.feed(*rows[:-1], extra, rows[-1])
                self.assertFalse(self.app._rgb_motion_supported)
                self.assertTrue(self.app._rgb_motion_invalid)
                self.app._rgb_save()
                self.assertFalse(self.app.send_cmd("!RGB_SAVE"))
                self.assertFalse(self.app.send_cmd("!RGB_BRIGHT 50 &"))
                self.assertEqual(self.commands(), [])

    def test_invalid_caps_cannot_be_repaired_by_a_second_caps_in_the_same_reply(self):
        for caps in ("RGB_CAPS motion=0", "RGB_CAPS motion=2", "RGB_CAPS", "RGB_CAPS motion=1 extra=1"):
            with self.subTest(caps=caps):
                self.assertTrue(self.app.send_cmd("#RGB_GET"))
                self.commands()
                self.feed(caps, *self.motion_rows())
                self.assertFalse(self.app._rgb_motion_supported)
                self.assertTrue(self.app._rgb_motion_required)
                self.assertFalse(self.app.send_cmd("!RGB_SAVE"))
                self.assertEqual(self.commands(), [])

    def test_missing_target_states_cannot_reuse_an_earlier_complete_reply(self):
        for state in LEGACY.RGB_MOTION_STATES:
            with self.subTest(state=state):
                self.full_motion()
                self.assertTrue(self.app._rgb_motion_supported)
                self.assertTrue(self.app.send_cmd("#RGB_GET"))
                self.assertFalse(self.app._rgb_motion_supported)
                self.assertEqual(self.app._rgb_cache["motions"], {})
                self.commands()
                self.feed(*(row for row in self.motion_rows() if not row.startswith(f"RGB_STATE {state} ")))
                self.assertFalse(self.app._rgb_motion_supported)
                self.assertFalse(self.app.send_cmd("!RGB_SAVE"))
                self.assertEqual(self.commands(), [])

    def test_capsless_extension_never_downgrades_to_saveable_legacy(self):
        rows = [f"RGB_MOTION {i} 1800 0 3" for i in range(4)]
        rows += [f"RGB_STATE {state} 8 1 10 20 30" for state in LEGACY.RGB_MOTION_STATES]
        self.feed(*rows, "RGB_GET end")
        self.assertTrue(self.app._rgb_motion_required)
        self.assertFalse(self.app._rgb_motion_supported)
        self.app._rgb_save()
        self.assertFalse(self.app.send_cmd("!RGB_SAVE"))
        self.assertEqual(self.commands(), [])
        # An explicit fresh query containing only old fields cannot erase that proof.
        self.assertTrue(self.app.send_cmd("#RGB_GET"))
        self.commands()
        self.feed("RGB_CONFIG manual=0 fade_on=1000 fade_off=1000", "RGB_STATE RUN_SEQ 8 2 0 0 0", "RGB_GET end")
        self.assertFalse(self.app.send_cmd("!RGB_SAVE"))
        self.assertEqual(self.commands(), [])
        self.full_motion()
        self.assertTrue(self.app._rgb_motion_supported)

    def test_short_status_and_orphan_terminator_keep_confirmed_capability(self):
        self.full_motion()
        motion = dict(self.app._rgb_cache["motions"])
        self.feed("RGB_STATUS on=1 auto=1 bright=70 current=70 mode=13",
                  "RGB_RUNTIME state=RUN_SEQ source=RUN_SEQ", "RGB_STORAGE dirty=1 storage=1 dma=0",
                  "RGB_DMA frames=10 errors=0", "RGB_TIMEOUT timeouts=0", "RGB_GET end")
        self.assertTrue(self.app._rgb_motion_supported)
        self.assertEqual(self.app._rgb_cache["motions"], motion)
        self.assertFalse(self.app._rgb_motion_response_active)
        self.assertEqual(self.commands(), [])

    def test_explicit_query_recovers_incomplete_round_and_setter_starts_fresh(self):
        self.feed("RGB_CAPS motion=1", "RGB_MOTION 0 1500 0 13")
        self.assertTrue(self.app._rgb_motion_invalid)
        self.assertTrue(self.app.send_cmd("#RGB_GET"))
        self.assertFalse(self.app._rgb_motion_invalid)
        self.assertFalse(self.app._rgb_motion_caps_seen)
        self.full_motion()
        self.assertTrue(self.app._rgb_motion_supported)
        self.assertEqual(self.commands(), ["#RGB_GET"])
        rows = [row.replace("RGB_MOTION 0 1400 0 3", "RGB_MOTION 0 2300 1 7") for row in self.motion_rows()]
        self.feed(*rows)
        self.assertTrue(self.app._rgb_motion_supported)
        self.assertEqual(self.app._rgb_cache["motions"][0], [2300, 1, 7])
        self.assertEqual(self.commands(), [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
