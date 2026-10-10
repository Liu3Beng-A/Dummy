"""Offline RGB readback, device identity and opt-in monitoring regressions."""
import queue
import unittest
from unittest.mock import patch

from PySide6.QtCore import QCoreApplication, QObject, Signal

from app.core.controller import RobotController
from app.core.fields import ValueField
from tests.test_controller import FakePort, ManualScheduler


READBACK = """RGB_STATUS on=1 auto=1 bright=40 current=35 mode=8
RGB_RUNTIME state=RUN_SEQ source=RUN_SEQ
RGB_STORAGE dirty=0 storage=1 dma=0
RGB_DMA frames=123 errors=0
RGB_TIMEOUT timeouts=0
RGB_CONFIG manual=3 fade_on=1000 fade_off=500
RGB_TIMING bright_ms=300 transition=200
RGB_COLOR 0 10 20 30
RGB_COLOR 1 40 50 60
RGB_COLOR 2 70 80 90
RGB_AUTO_COLOR 1 100 110 120
RGB_AUTO_COLOR 2 130 140 150
RGB_AUTO_COLOR 3 160 170 180
RGB_EFFECT 3 255 0 0 0 0 0 4000 0 3 200 1000
RGB_EFFECT 4 0 0 255 0 0 0 4000 0 3 200 1000
RGB_EFFECT 5 255 255 255 0 0 0 4000 0 3 200 1000
RGB_EFFECT 6 160 0 255 0 0 0 4000 1 3 200 1000
RGB_EFFECT 7 255 0 0 0 0 0 4000 0 3 200 1000
RGB_EFFECT 8 0 255 0 0 0 0 4000 1 3 200 1000
RGB_EFFECT 9 255 255 255 0 0 0 4000 0 3 200 1000
RGB_STATE BOOT 3 0 0 0 0
RGB_STATE DISABLED 5 0 0 0 255
RGB_STATE READY_SEQ 10 0 0 0 0
RGB_STATE READY_INT 11 0 0 0 0
RGB_STATE READY_TRJ 12 0 0 0 0
RGB_STATE READY_TUN 2 1 160 0 255
RGB_STATE RUN_SEQ 8 2 0 0 0
RGB_STATE RUN_INT 8 3 0 0 0
RGB_STATE RUN_TRJ 8 4 0 0 0
RGB_STATE RUN_TUN 5 1 160 0 255
RGB_STATE CALIBRATION 6 1 255 128 0
RGB_STATE STOP 7 1 255 0 0
RGB_STATE STARTING 5 1 0 255 255
RGB_STATE FAULT 7 1 255 0 0
RGB_GET end""".splitlines()


class FakeTransport(QObject):
    line_received = Signal(str, int)
    command_written = Signal(str, int)
    failed = Signal(str, int)

    def __init__(self, port, commands, validator, identity, parent):
        super().__init__(parent)
        self.port = port

    def start(self):
        pass

    def close(self):
        self.port.close()


class RGBControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.c = RobotController(scheduler=ManualScheduler(), serial_factory=FakePort)
        self.c.is_connected = True
        self.c.serial_port = FakePort()
        self.views = []
        self.c.rgb_changed.connect(self.views.append)

    def tearDown(self):
        self.c.close()

    def commands(self):
        result = []
        while True:
            try:
                item = self.c._tx_queue.get_nowait()
                if item:
                    result.append(item[1])
            except queue.Empty:
                return result

    def feed(self, lines=READBACK):
        for line in lines:
            self.c._process_rx_line(line)

    def test_complete_get_uses_valid_device_values_and_is_a_deep_snapshot(self):
        with patch("app.core.controller.time.monotonic", return_value=100.0):
            self.c.send_cmd("#RGB_GET")
            self.feed(READBACK[:-1])
            self.assertFalse(self.c.rgb_snapshot()["complete"])
            self.feed(READBACK[-1:])
        view = self.c.rgb_snapshot()
        self.assertTrue(view["complete"])
        self.assertTrue(view["connected"])
        self.assertEqual(view["received_at"], 100.0)
        config = view["config"]
        self.assertIs(config["enabled"], True)
        self.assertIs(config["automatic"], True)
        self.assertEqual(config["brightness"], 40)
        self.assertEqual(config["manual_mode"], 3)
        self.assertEqual([config[k] for k in ("fade_on_ms", "fade_off_ms", "brightness_ms", "transition_ms")],
                         [1000, 500, 300, 200])
        self.assertEqual(config["colors"], {0: (10, 20, 30), 1: (40, 50, 60), 2: (70, 80, 90)})
        self.assertEqual(config["auto_colors"][3], (160, 170, 180))
        self.assertEqual(config["effects"][8], (0, 255, 0, 0, 0, 0, 4000, 1, 3, 200, 1000))
        self.assertEqual(config["states"]["FAULT"], (7, 1, 255, 0, 0))
        self.assertEqual(len(config["states"]), 14)
        self.assertEqual(view["runtime"]["source"], "RUN_SEQ")
        config["colors"].clear()
        view["runtime"]["source"] = "OFF"
        self.views[-1]["config"]["states"].clear()
        self.assertEqual(len(self.c.rgb_snapshot()["config"]["colors"]), 3)
        self.assertEqual(len(self.c.rgb_snapshot()["config"]["states"]), 14)
        self.assertEqual(self.c.rgb_snapshot()["runtime"]["source"], "RUN_SEQ")

    def test_ui_drafts_and_unconfirmed_setters_never_enter_device_snapshot(self):
        self.c._rgb_bright.set("15")
        self.c._rgb_color_vars[0].set("99")
        self.c._rgb_fades[0].set("20")
        empty = self.c.rgb_snapshot()
        self.assertEqual(empty["runtime"], {})
        self.assertNotIn("brightness", empty["config"])
        self.assertFalse(empty["complete"])
        self.feed()
        confirmed = self.c.rgb_snapshot()
        self.c._rgb_bright.set("10")
        self.c._rgb_color_vars[0].set("250")
        self.c._rgb_apply_bright()
        self.assertEqual(self.commands(), ["!RGB_BRIGHT 10"])
        self.assertEqual(self.c.rgb_snapshot(), confirmed)
        self.feed([READBACK[0].replace("bright=40", "bright=10")] + READBACK[1:5])
        self.assertEqual(self.c.rgb_snapshot()["config"]["brightness"], 10)
        self.assertTrue(self.c.rgb_snapshot()["complete"])

    def test_partial_status_and_orphan_tail_do_not_advance_runtime_clock(self):
        self.feed(READBACK[1:5])
        self.assertIsNone(self.c.rgb_snapshot()["received_at"])
        self.feed(READBACK[:4])
        self.assertEqual(self.c.rgb_snapshot()["runtime"], {})
        self.assertEqual(self.c._rgb_bright.get(), "100")
        self.assertIsNone(self.c.rgb_snapshot()["received_at"])
        self.feed(READBACK[4:5])
        self.assertEqual(self.c._rgb_bright.get(), "40")
        self.assertEqual(self.c.rgb_snapshot()["runtime"]["frames"], 123)
        self.assertFalse(self.c.rgb_snapshot()["complete"])

    def test_invalid_runtime_batch_cannot_refresh_last_good_status(self):
        self.feed()
        before = self.c.rgb_snapshot()
        cases = [
            (0, "RGB_STATUS on=2 auto=1 bright=40 current=35 mode=8"),
            (0, "RGB_STATUS on=1 auto=1 bright=nan current=35 mode=8"),
            (0, "RGB_STATUS on=1 auto=1 bright=-1 current=35 mode=8"),
            (0, "RGB_STATUS on=1 auto=1 bright=101 current=35 mode=8"),
            (0, READBACK[0] + " on=1"), (0, READBACK[0] + " extra=1"),
            (1, "RGB_RUNTIME state=UNKNOWN source=RUN_SEQ"),
            (1, "RGB_RUNTIME state=RUN_SEQ source=AUTO_SEQ"),
            (2, "RGB_STORAGE dirty=0 storage=3 dma=0"),
            (2, "RGB_STORAGE dirty=0 storage=1 dma=2"),
            (3, "RGB_DMA frames=4294967296 errors=0"),
            (4, "RGB_TIMEOUT timeouts=4294967296"),
        ]
        for index, invalid in cases:
            with self.subTest(invalid=invalid):
                batch = list(READBACK[:5])
                batch[index] = invalid
                self.feed(batch)
                self.assertEqual(self.c.rgb_snapshot(), before)

    def test_partial_or_invalid_get_cannot_reuse_previous_complete_fields(self):
        self.feed()
        self.assertTrue(self.c.rgb_snapshot()["complete"])
        for invalid_line in (None, "RGB_COLOR 0 256 20 30", "RGB_EFFECT 8 0 255 0 0 0 0 249 1 3 200 1000",
                             "RGB_STATE FAULT 8 1 255 0 0", "RGB_CONFIG manual=13 fade_on=1000 fade_off=500"):
            with self.subTest(invalid=invalid_line):
                self.c.send_cmd("#RGB_GET")
                batch = [line for line in READBACK if not line.startswith("RGB_COLOR 0 ")]
                if invalid_line is not None:
                    batch.insert(-1, invalid_line)
                self.feed(batch)
                self.assertFalse(self.c.rgb_snapshot()["complete"])
                self.assertNotIn(0, self.c.rgb_snapshot()["config"]["colors"])
        # A structured SET response also starts a new batch without an explicit GET.
        self.feed()
        self.feed(READBACK[:8] + ["RGB_GET end"])
        self.assertFalse(self.c.rgb_snapshot()["complete"])
        self.assertEqual(self.c.rgb_snapshot()["config"]["colors"], {0: (10, 20, 30)})

    def test_monitoring_is_opt_in_and_throttles_get_timeout_and_status(self):
        with patch("app.core.controller.time.monotonic", return_value=100.0):
            self.c._poll_rgb()
            self.assertEqual(self.commands(), [])
            self.c.set_rgb_monitoring(True)
            self.assertEqual(self.commands(), ["#RGB_GET"])
            self.c._poll_rgb()
            self.assertEqual(self.commands(), [])
        with patch("app.core.controller.time.monotonic", return_value=101.9):
            self.c._poll_rgb()
            self.assertEqual(self.commands(), [])
        with patch("app.core.controller.time.monotonic", return_value=102.1):
            self.c._poll_rgb()
            self.assertEqual(self.commands(), ["#RGB_STATUS"])
            self.c._poll_rgb()
            self.assertEqual(self.commands(), [])
        with patch("app.core.controller.time.monotonic", return_value=104.0):
            self.c._poll_rgb()
            self.assertEqual(self.commands(), ["#RGB_STATUS"])
        with patch("app.core.controller.time.monotonic", return_value=105.0):
            self.c._poll_rgb()
            self.assertEqual(self.commands(), ["#RGB_GET"])
        self.c.set_rgb_monitoring(False)
        with patch("app.core.controller.time.monotonic", return_value=200.0):
            self.c._poll_rgb()
            self.assertEqual(self.commands(), [])
        self.assertFalse(self.c._rgb_monitor_timer.isActive())

    def test_repeated_monitor_activation_does_not_restart_timer_or_poll(self):
        self.feed()
        with patch("app.core.controller.time.monotonic", return_value=100.0):
            self.c.set_rgb_monitoring(True)
            self.assertEqual(self.commands(), ["#RGB_STATUS"])
        with patch.object(self.c._rgb_monitor_timer, "start", wraps=self.c._rgb_monitor_timer.start) as start:
            with patch.object(self.c, "_poll_rgb", wraps=self.c._poll_rgb) as poll:
                with patch("app.core.controller.time.monotonic", return_value=200.0):
                    for _ in range(5):
                        self.c.set_rgb_monitoring(True)
                start.assert_not_called()
                poll.assert_not_called()
                self.assertEqual(self.commands(), [])
        with patch("app.core.controller.time.monotonic", return_value=200.0):
            self.c._poll_rgb()  # Actual timer callback performs the next poll.
            self.assertEqual(self.commands(), ["#RGB_STATUS"])
        self.c._rgb_monitor_timer.stop()  # Disconnect stops it while retaining intent.
        with patch.object(self.c._rgb_monitor_timer, "start", wraps=self.c._rgb_monitor_timer.start) as start:
            with patch.object(self.c, "_poll_rgb", wraps=self.c._poll_rgb) as poll:
                self.c.set_rgb_monitoring(True)
                start.assert_called_once()
                poll.assert_called_once()
        self.c.set_rgb_monitoring(False)
        self.c.set_rgb_monitoring(False)
        self.assertFalse(self.c._rgb_monitor_timer.isActive())

    def test_monitor_preserves_complete_status_and_pauses_during_parameter_jobs_or_hidden(self):
        self.feed()
        with patch("app.core.controller.time.monotonic", return_value=100.0):
            self.c.set_rgb_monitoring(True)
            self.assertEqual(self.commands(), ["#RGB_STATUS"])
        self.feed(READBACK[:5])
        self.assertTrue(self.c.rgb_snapshot()["complete"])
        for busy in ("parameter", "acc_sync"):
            with self.subTest(busy=busy):
                self.c._parameter_request = {"command": "#STALL_GET 1"} if busy == "parameter" else None
                self.c._acc_sync_waiting = busy == "acc_sync"
                with patch("app.core.controller.time.monotonic", return_value=200.0):
                    self.c._poll_rgb()
                self.assertEqual(self.commands(), [])
        self.c._parameter_request = None
        self.c._acc_sync_waiting = False
        self.c.set_rgb_monitoring(False)
        self.c._poll_rgb()
        self.assertEqual(self.commands(), [])
        # A deliberate user command remains available while automatic monitoring is hidden.
        self.assertTrue(self.c.send_cmd("#RGB_STATUS"))
        self.assertEqual(self.commands(), ["#RGB_STATUS"])
        self.c.close()
        self.assertFalse(self.c._rgb_monitor_timer.isActive())
        self.assertFalse(self.c._rgb_monitoring)
        self.c._poll_rgb()
        self.assertEqual(self.commands(), [])
        self.assertEqual(self.c.rgb_snapshot()["runtime"], {})
        self.assertIsNone(self.c.rgb_snapshot()["received_at"])

    def test_monitor_does_not_interleave_queries_with_full_setter_readback(self):
        self.feed()
        with patch("app.core.controller.time.monotonic", return_value=100.0):
            self.c.set_rgb_monitoring(True)
            self.commands()
            self.feed(READBACK[:8])  # Structured SET response, no explicit GET.
            self.c._poll_rgb()
            self.assertEqual(self.commands(), [])
        with patch("app.core.controller.time.monotonic", return_value=101.9):
            self.c._poll_rgb()
            self.assertEqual(self.commands(), [])
        self.feed(READBACK[8:])
        self.assertTrue(self.c.rgb_snapshot()["complete"])
        with patch("app.core.controller.time.monotonic", return_value=102.0):
            self.c._poll_rgb()
            self.assertEqual(self.commands(), ["#RGB_STATUS"])

    def test_connect_query_is_not_duplicated_and_previous_device_packets_are_ignored(self):
        self.feed()
        old_identity = self.c.rgb_snapshot()["connection_id"]
        self.c.disconnect_port()
        disconnected = self.c.rgb_snapshot()
        self.assertFalse(disconnected["connected"])
        self.assertFalse(disconnected["complete"])
        self.assertIsNone(disconnected["received_at"])
        self.assertEqual(disconnected["runtime"], {})
        self.assertEqual(disconnected["config"]["colors"], {})
        self.c.set_polling(False)
        self.c.set_rgb_monitoring(True)
        self.assertEqual(self.commands(), [])
        with patch("app.core.controller.SerialTransport", FakeTransport):
            with patch("app.core.controller.time.monotonic", return_value=100.0):
                self.assertTrue(self.c.connect_port("FAKE_NEW", "USB"))
                sent = self.commands()
                self.assertEqual(sent.count("#RGB_GET"), 1)
                self.assertEqual(sent.count("#RGB_STATUS"), 0)
                self.c.set_rgb_monitoring(True)
                self.assertEqual(self.commands(), [])
            new_identity = self.c.rgb_snapshot()["connection_id"]
            self.assertGreater(new_identity, old_identity)
            for line in READBACK:
                self.c._on_transport_line(line, old_identity)
            self.assertFalse(self.c.rgb_snapshot()["complete"])
            self.assertEqual(self.c.rgb_snapshot()["runtime"], {})
            self.assertEqual(self.c.rgb_snapshot()["config"]["states"], {})
            self.c._on_transport_line(READBACK[0], new_identity)
            for line in READBACK[1:5]:
                self.c._on_transport_line(line, old_identity)
            self.assertIsNone(self.c.rgb_snapshot()["received_at"])
            for line in READBACK[1:]:
                self.c._on_transport_line(line, new_identity)
            self.assertTrue(self.c.rgb_snapshot()["complete"])

    def test_connected_snapshot_listener_cannot_reenter_and_duplicate_default_get(self):
        self.c.disconnect_port()
        self.c.set_polling(False)
        pending_when_published = []

        def follow_visible_rgb_page(snapshot):
            if snapshot["connected"]:
                pending_when_published.append(self.c._rgb_get_pending_until)
            self.c.set_rgb_monitoring(snapshot["connected"])

        self.c.rgb_changed.connect(follow_visible_rgb_page)
        with patch("app.core.controller.SerialTransport", FakeTransport):
            with patch("app.core.controller.time.monotonic", return_value=100.0):
                self.assertTrue(self.c.connect_port("FAKE_RGB_FOLLOW", "USB"))
                self.assertEqual(self.commands(), ["#RGB_GET", "#GET_CMDMODE 1"])
        self.assertEqual(pending_when_published, [102.0])
        self.assertTrue(self.c._rgb_monitor_timer.isActive())

    def test_all_rgb_device_field_fills_are_guarded_and_guard_survives_exception(self):
        c = self.c
        flags = []

        class ObservedField(ValueField):
            def set(self, value):
                flags.append(c._rgb_applying_readback)
                super().set(value)

            def current(self, index=None):
                if index is not None:
                    flags.append(c._rgb_applying_readback)
                return super().current(index)

        for name in ("_rgb_status_var", "_rgb_bright", "_rgb_scale", "_rgb_reverse",
                     "_rgb_state_mode", "_rgb_state_source"):
            setattr(c, name, ObservedField())
        for name in ("_rgb_color_vars", "_rgb_auto_color_vars", "_rgb_fades", "_rgb_effect_first",
                     "_rgb_effect_second", "_rgb_effect_params", "_rgb_state_color"):
            setattr(c, name, [ObservedField() for _ in getattr(c, name)])
        self.feed()
        self.assertTrue(flags)
        self.assertTrue(all(flags))
        self.assertFalse(c._rgb_applying_readback)
        flags.clear()
        for fill in (c._rgb_fill_color, c._rgb_fill_auto_color, c._rgb_fill_effect, c._rgb_fill_state):
            fill()
        self.assertTrue(flags)
        self.assertTrue(all(flags))
        c._rgb_applying_readback = True
        c._rgb_fill_state()
        self.assertTrue(c._rgb_applying_readback)
        c._rgb_applying_readback = False
        with patch.object(c._rgb_color_vars[0], "set", side_effect=RuntimeError("widget failure")):
            with self.assertRaises(RuntimeError):
                c._rgb_fill_color()
        self.assertFalse(c._rgb_applying_readback)

    def test_uint32_diagnostics_and_fault_off_sources_are_valid(self):
        self.feed()
        batch = [
            "RGB_STATUS on=0 auto=1 bright=100 current=0 mode=7",
            "RGB_RUNTIME state=FAULT source=OFF",
            "RGB_STORAGE dirty=1 storage=2 dma=1",
            "RGB_DMA frames=4294967295 errors=4294967295",
            "RGB_TIMEOUT timeouts=4294967295",
        ]
        with patch("app.core.controller.time.monotonic", return_value=200.0):
            self.feed(batch)
        view = self.c.rgb_snapshot()
        self.assertTrue(view["complete"])
        self.assertIs(view["config"]["enabled"], False)
        self.assertEqual(view["runtime"]["state"], "FAULT")
        self.assertEqual(view["runtime"]["source"], "OFF")
        self.assertEqual(view["runtime"]["frames"], 4294967295)
        self.assertEqual(view["received_at"], 200.0)


if __name__ == "__main__":
    unittest.main()
