"""Qt acquisition through a fake SDL module; never enumerate physical devices."""
from types import SimpleNamespace
import os
import unittest
from unittest.mock import patch

from PySide6.QtCore import QCoreApplication, Qt
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
from PySide6.QtWidgets import QApplication
APPLICATION = QApplication.instance() or QApplication([])

from app.gamepad.backend import GamepadReader
from app.gamepad.config import default_config


class FakeJoystick:
    def __init__(self, instance_id=10, name="Fake Xbox", guid="abc"):
        self.instance_id, self.name, self.guid = instance_id, name, guid
        self.axes = [0., 0., 0., 0., -1., -1.]
        self.buttons = [0] * 8
        self.hats = [(0, 0)]
        self.initialized = False
        self.quit_calls = self.read_calls = 0
        self.error = None

    def init(self):
        self.initialized = True

    def quit(self):
        self.quit_calls += 1
        self.initialized = False

    def get_name(self):
        return self.name

    def get_guid(self):
        return self.guid

    def get_instance_id(self):
        return self.instance_id

    def get_numaxes(self):
        self.read_calls += 1
        if self.error:
            raise self.error
        if not self.initialized:
            raise RuntimeError("device unavailable")
        return len(self.axes)

    def get_axis(self, index):
        return self.axes[index]

    def get_numbuttons(self):
        return len(self.buttons)

    def get_button(self, index):
        return self.buttons[index]

    def get_numhats(self):
        return len(self.hats)

    def get_hat(self, index):
        return self.hats[index]


class FakeSubsystem:
    def __init__(self, initialized=False):
        self.initialized = initialized
        self.init_calls = self.quit_calls = 0

    def get_init(self):
        return self.initialized

    def init(self):
        self.init_calls += 1
        self.initialized = True

    def quit(self):
        self.quit_calls += 1
        self.initialized = False


class FakeDisplay(FakeSubsystem):
    def __init__(self):
        super().__init__()
        self.surface = None
        self.mode_calls = []

    def get_surface(self):
        return self.surface

    def set_mode(self, size, flags):
        self.mode_calls.append((size, flags))
        self.surface = object()
        return self.surface

    def quit(self):
        super().quit()
        self.surface = None


class FakeJoystickSubsystem(FakeSubsystem):
    def __init__(self, devices):
        super().__init__()
        self.devices = devices
        self.count_calls = 0

    def get_count(self):
        self.count_calls += 1
        return len(self.devices)

    def Joystick(self, index):
        return self.devices[index]


class FakeEvents:
    def __init__(self):
        self.events = []
        self.pump_calls = 0

    def pump(self):
        self.pump_calls += 1

    def get(self):
        events, self.events = self.events, []
        return events


def fake_pygame(devices):
    return SimpleNamespace(display=FakeDisplay(), joystick=FakeJoystickSubsystem(devices),
                           event=FakeEvents(), HIDDEN=128, JOYDEVICEADDED=1541, JOYDEVICEREMOVED=1542)


class GamepadBackendTests(unittest.TestCase):
    def setUp(self):
        self.joy = FakeJoystick()
        self.pg = fake_pygame([self.joy])
        self.reader = GamepadReader(pygame_module=self.pg, clock=lambda: 123.25)
        self.samples, self.statuses = [], []
        self.reader.sampled.connect(self.samples.append)
        self.reader.status_changed.connect(self.statuses.append)

    def tearDown(self):
        self.reader.close()
        self.reader.deleteLater()
        APPLICATION.processEvents()

    def last(self):
        return self.samples[-1]

    def start(self, config=None):
        self.assertTrue(self.reader.start(default_config() if config is None else config))

    def test_constructor_and_set_config_do_not_import_or_enumerate_sdl(self):
        with patch("app.gamepad.backend.importlib.import_module", side_effect=AssertionError("unexpected import")):
            other = GamepadReader()
            other.set_config({"gamepad": {"device_index": 2}})
            self.assertFalse(other.status()["running"])
            self.assertEqual(other.status()["devices"], [])
            other.close()
        self.assertEqual(self.pg.joystick.count_calls, 0)
        self.assertEqual(self.pg.display.init_calls, 0)

    def test_explicit_start_hidden_window_background_events_and_snapshot(self):
        with patch.dict(os.environ, {}, clear=False):
            self.start()
            self.assertEqual(os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"], "1")
        self.assertEqual(self.pg.display.mode_calls, [((1, 1), self.pg.HIDDEN)])
        self.assertTrue(self.reader._timer.isActive())
        self.assertEqual(self.reader._timer.interval(), 17)
        self.assertTrue(self.last()["connected"])
        self.assertEqual(self.last()["timestamp"], 123.25)
        self.assertTrue(self.last()["decoded"]["centered"])
        self.assertEqual(self.last()["pressed"], [])
        self.assertEqual(self.last()["raw"]["name"], "Fake Xbox")
        status = self.reader.status()
        self.assertEqual(status["instance_id"], 10)
        self.assertEqual(status["devices"], [{"index": 0, "name": "Fake Xbox", "guid": "abc", "instance_id": 10}])

    def test_held_buttons_are_baseline_then_only_new_edges_are_reported(self):
        self.joy.buttons[7] = 1
        self.start()
        self.assertEqual(self.last()["pressed"], [])
        self.reader._poll()
        self.assertEqual(self.last()["pressed"], [])
        self.joy.buttons[7] = 0
        self.reader._poll()
        self.joy.buttons[7] = 1
        self.reader._poll()
        self.assertEqual(self.last()["pressed"], ["start"])
        self.reader._poll()
        self.assertEqual(self.last()["pressed"], [])
        self.joy.buttons[6] = self.joy.buttons[1] = self.joy.buttons[3] = 1
        self.reader._poll()
        self.assertEqual(self.last()["pressed"], ["stop", "rest"])

    def test_legacy_manual_binding_does_not_emit_b_action_or_lose_start_mapping(self):
        config = {"gamepad": {"device_guid": "abc", "buttons": {"start": 0, "manual": 1}}}
        self.start(config)
        self.joy.buttons[1] = 1
        self.reader._poll()
        self.assertEqual(self.last()["pressed"], [])
        self.assertNotIn("manual", self.last()["decoded"]["buttons"])
        self.assertEqual(self.reader.status()["instance_id"], 10)
        self.joy.buttons[0] = 1
        self.reader._poll()
        self.assertEqual(self.last()["pressed"], ["start"])
        self.assertEqual(config["gamepad"]["buttons"], {"start": 0, "manual": 1})

    def test_matching_removal_beats_new_buttons_and_added_device(self):
        self.start()
        before_reads = self.joy.read_calls
        self.joy.buttons[7] = 1
        replacement = FakeJoystick(99, "Another Xbox", "abc")
        self.pg.joystick.devices = [replacement]
        self.pg.event.events = [SimpleNamespace(type=self.pg.JOYDEVICEADDED, device_index=0),
                                SimpleNamespace(type=self.pg.JOYDEVICEREMOVED, instance_id=10)]
        self.reader._poll()
        self.assertFalse(self.last()["connected"])
        self.assertIsNone(self.last()["decoded"])
        self.assertEqual(self.last()["pressed"], [])
        self.assertEqual(self.last()["raw"]["axes"], [])
        self.assertEqual(self.joy.read_calls, before_reads)
        self.assertTrue(self.reader.status()["requires_restart"])
        self.reader._poll()
        self.assertFalse(self.last()["connected"])
        self.assertEqual(replacement.read_calls, 0)
        self.start()
        self.assertTrue(self.last()["connected"])
        self.assertEqual(self.reader.status()["instance_id"], 99)

    def test_other_instance_removal_does_not_drop_selected_device(self):
        self.start()
        self.pg.event.events = [SimpleNamespace(type=self.pg.JOYDEVICEREMOVED, instance_id=22)]
        self.reader._poll()
        self.assertTrue(self.last()["connected"])
        self.assertFalse(self.reader.status()["requires_restart"])
        self.assertEqual(self.joy.quit_calls, 0)

    def test_missing_device_can_be_detected_by_add_event_before_binding(self):
        self.pg.joystick.devices = []
        self.start()
        self.assertFalse(self.last()["connected"])
        self.pg.joystick.devices = [self.joy]
        self.pg.event.events = [SimpleNamespace(type=self.pg.JOYDEVICEADDED, device_index=0)]
        self.reader._poll()
        self.assertTrue(self.last()["connected"])
        self.assertEqual(self.last()["pressed"], [])

    def test_guid_and_index_selection_do_not_adopt_unmatched_device(self):
        other = FakeJoystick(22, "Second Xbox", "def")
        self.pg.joystick.devices.append(other)
        self.start({"gamepad": {"device_index": 0, "device_guid": " DEF "}})
        self.assertEqual(self.reader.status()["instance_id"], 22)
        self.assertFalse(self.joy.initialized)
        self.reader.stop()
        self.start({"gamepad": {"device_index": 0, "device_guid": "missing"}})
        self.assertFalse(self.last()["connected"])
        self.assertIsNone(self.last()["decoded"])
        self.assertEqual(self.reader.status()["instance_id"], None)

    def test_index_selects_between_same_guid_devices(self):
        other = FakeJoystick(22, "Second Xbox", "abc")
        self.pg.joystick.devices.append(other)
        self.start({"gamepad": {"device_index": 1, "device_guid": "abc"}})
        self.assertEqual(self.reader.status()["instance_id"], 22)

    def test_invalid_layout_clears_previous_input_and_recovery_baselines_buttons(self):
        self.start()
        self.joy.axes = [0.]
        self.reader._poll()
        self.assertFalse(self.last()["connected"])
        self.assertIsNone(self.last()["decoded"])
        self.assertEqual(self.last()["pressed"], [])
        self.assertIn("布局", self.last()["error"])
        self.assertEqual(self.last()["raw"]["axes"], [0.])
        self.joy.axes = [0., 0., 0., 0., -1., -1.]
        self.joy.buttons[7] = 1
        self.reader._poll()
        self.assertTrue(self.last()["connected"])
        self.assertEqual(self.last()["pressed"], [])

    def test_read_exception_drops_device_and_requires_explicit_restart(self):
        self.start()
        self.joy.error = RuntimeError("SDL device removed")
        self.reader._poll()
        self.assertFalse(self.last()["connected"])
        self.assertIsNone(self.last()["decoded"])
        self.assertTrue(self.reader.status()["requires_restart"])
        self.joy.error = None
        self.reader._poll()
        self.assertFalse(self.last()["connected"])
        self.start()
        self.assertTrue(self.last()["connected"])

    def test_status_and_sample_data_are_detached_from_reader(self):
        self.start()
        self.reader.status()["devices"][0]["guid"] = "bad"
        self.last()["raw"]["axes"][0] = 99
        self.last()["decoded"]["buttons"]["start"] = True
        self.reader._poll()
        self.assertEqual(self.reader.status()["devices"][0]["guid"], "abc")
        self.assertEqual(self.last()["raw"]["axes"][0], 0)
        self.assertFalse(self.last()["decoded"]["buttons"]["start"])
        self.assertEqual(self.last()["pressed"], [])

    def test_changed_config_stops_sampler_and_is_not_aliased(self):
        self.start()
        config = {"gamepad": {"deadzone": .3}}
        self.reader.set_config(config)
        config["gamepad"]["deadzone"] = .9
        self.assertFalse(self.reader.status()["running"])
        self.assertFalse(self.reader._timer.isActive())
        self.assertFalse(self.last()["connected"])
        self.joy.axes[0] = .25
        self.assertTrue(self.reader.start())
        self.assertEqual(self.last()["decoded"]["lx"], 0)
        with self.assertRaises(ValueError):
            self.reader.set_config({"gamepad": {"deadzone": 1}})
        self.assertTrue(self.reader.status()["running"])

    def test_start_idempotence_and_stop_close_cleanup(self):
        self.start()
        count = self.pg.joystick.count_calls
        samples = len(self.samples)
        self.start()
        self.assertEqual(self.pg.joystick.count_calls, count)
        self.assertEqual(len(self.samples), samples)
        self.reader.stop()
        self.assertFalse(self.last()["connected"])
        self.assertEqual(self.pg.display.quit_calls, 1)
        self.assertEqual(self.pg.joystick.quit_calls, 1)
        self.assertFalse(self.reader._timer.isActive())
        self.reader.stop()
        self.reader.close()
        self.reader.close()
        self.assertEqual(self.pg.display.quit_calls, 1)
        self.assertFalse(self.reader.start())

    def test_external_initialized_subsystems_are_not_quit(self):
        self.pg.display.initialized = self.pg.joystick.initialized = True
        self.pg.display.surface = object()
        self.start()
        self.reader.stop()
        self.assertEqual(self.pg.display.quit_calls, 0)
        self.assertEqual(self.pg.joystick.quit_calls, 0)

    def test_missing_pygame_reports_failure_without_crashing_or_starting_timer(self):
        other = GamepadReader()
        samples = []
        other.sampled.connect(samples.append)
        with patch("app.gamepad.backend.importlib.import_module", side_effect=ImportError("pygame")):
            self.assertFalse(other.start())
        self.assertFalse(samples[-1]["connected"])
        self.assertIn("pygame", samples[-1]["error"])
        self.assertFalse(other._timer.isActive())
        other.close()

    def test_enumeration_failure_releases_opened_candidate_and_owned_subsystems(self):
        self.joy.get_guid = lambda: (_ for _ in ()).throw(RuntimeError("bad metadata"))
        self.assertFalse(self.reader.start())
        self.assertFalse(self.reader.status()["running"])
        self.assertIn("bad metadata", self.last()["error"])
        self.assertFalse(self.joy.initialized)
        self.assertEqual(self.pg.display.quit_calls, 1)
        self.assertEqual(self.pg.joystick.quit_calls, 1)


if __name__ == "__main__":
    unittest.main()
