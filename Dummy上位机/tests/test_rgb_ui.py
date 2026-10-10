"""Verify RGB preview isolation and page lifecycle with real Qt controls."""
from pathlib import Path
import queue
import tempfile
import unittest
from unittest.mock import patch

from tests.test_ui import APPLICATION, FakeView
from tests.test_controller import FakePort, ManualScheduler
from tests.test_rgb_controller import READBACK
from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow


class RGBPreviewUiTests(unittest.TestCase):
    def setUp(self):
        base = Path(__file__).resolve().parents[1] / "build" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=base)
        self.controller = RobotController(scheduler=ManualScheduler())
        self.store = AppStore(self.directory.name)
        with patch("app.ui.main_window.RobotView", FakeView):
            self.window = MainWindow(self.controller, self.store)
        self.window.show()
        self.window.navigation.setCurrentRow(4)
        APPLICATION.processEvents()
        self.assertIs(self.window.pages.currentWidget(), self.window.rgb_page)

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        APPLICATION.processEvents()
        self.directory.cleanup()

    def commands(self):
        result = []
        while True:
            try:
                item = self.controller._tx_queue.get_nowait()
                if item is not None:
                    result.append(item[1])
            except queue.Empty:
                return result

    def receive_config(self):
        self.controller.is_connected = True
        self.controller.serial_port = FakePort()
        for line in READBACK:
            self.controller._process_rx_line(line)
        self.assertTrue(self.controller.rgb_snapshot()["complete"])

    def test_local_edit_and_state_preview_never_send_commands(self):
        w = self.window
        w.fields["_rgb_color_vars"][0].widget.setValue(77)
        self.assertEqual(w._rgb_draft["colors"][0][0], 77)
        self.assertEqual(w.rgb_preview_mode.currentIndex(), 0)
        w.rgb_tabs.setCurrentIndex(2)
        w.fields["_rgb_effect_mode"].widget.setCurrentIndex(5)  # rotation
        w.fields["_rgb_effect_params"][1].widget.setValue(5)
        self.assertEqual(w._rgb_draft["effects"][8][8], 5)
        self.assertEqual(w.rgb_preview_mode.currentIndex(), 8)
        w.rgb_tabs.setCurrentIndex(3)
        w.fields["_rgb_state"].widget.setCurrentIndex(10)  # calibration
        w.fields["_rgb_state_mode"].widget.setCurrentIndex(9)
        self.assertEqual(w.rgb_preview_state.currentData(), "CALIBRATION")
        self.assertEqual(w._rgb_simulator.mode, 9)
        self.assertEqual(self.commands(), [])
        self.assertTrue(w.rgb_ring.isVisible())

    def test_offline_bank_switch_restores_defaults_and_each_local_edit(self):
        w = self.window
        w.fields["_rgb_color_vars"][0].widget.setValue(77)
        w.fields["_rgb_color_idx"].widget.setCurrentIndex(1)
        self.assertEqual(w._rgb_field_values("_rgb_color_vars"), (0, 255, 0))
        w.fields["_rgb_color_vars"][2].widget.setValue(88)
        w.fields["_rgb_color_idx"].widget.setCurrentIndex(0)
        self.assertEqual(w._rgb_field_values("_rgb_color_vars"), (77, 0, 255))
        w.fields["_rgb_color_idx"].widget.setCurrentIndex(1)
        self.assertEqual(w._rgb_field_values("_rgb_color_vars"), (0, 255, 88))
        w.rgb_tabs.setCurrentIndex(2)
        w.fields["_rgb_effect_mode"].widget.setCurrentIndex(5)
        self.assertEqual(w._rgb_field_values("_rgb_effect_params"), (1400, 3, 200, 1000))
        w.rgb_tabs.setCurrentIndex(3)
        w.fields["_rgb_state"].widget.setCurrentIndex(1)
        self.assertEqual(w.fields["_rgb_state_mode"].current(), 5)
        self.assertEqual(w._rgb_draft["states"]["DISABLED"], (5, 0, 255, 255, 255))
        self.assertEqual(self.commands(), [])

    def test_readback_preserves_draft_until_device_confirms_matching_value(self):
        self.receive_config()
        w = self.window
        w.fields["_rgb_color_vars"][0].widget.setValue(77)
        self.receive_config()
        self.assertEqual(w._rgb_field_values("_rgb_color_vars")[0], 77)
        self.assertEqual(w._rgb_draft["colors"][0][0], 77)
        for line in READBACK:
            self.controller._process_rx_line("RGB_COLOR 0 77 20 30" if line.startswith("RGB_COLOR 0 ") else line)
        self.assertEqual(w._rgb_draft["colors"][0], (77, 20, 30))
        self.assertNotIn(("colors", 0), w._rgb_dirty_paths)
        for line in READBACK:
            self.controller._process_rx_line("RGB_COLOR 0 88 20 30" if line.startswith("RGB_COLOR 0 ") else line)
        self.assertEqual(w._rgb_field_values("_rgb_color_vars"), (88, 20, 30))
        self.assertEqual(self.commands(), [])

    def test_device_readback_does_not_switch_source_or_use_ui_draft(self):
        self.receive_config()
        w = self.window
        w.rgb_preview_source.setCurrentIndex(1)
        self.assertEqual(w._rgb_simulator.mode, 8)
        self.assertFalse(w.rgb_preview_mode.isEnabled())
        self.assertIn("模拟", w.rgb_ring.toolTip())
        for line in READBACK[:5]:
            self.controller._process_rx_line(line)
        self.assertEqual(w.rgb_preview_source.currentIndex(), 1)
        original = self.controller.rgb_snapshot()["config"]["colors"][0]
        w.fields["_rgb_color_vars"][0].widget.setValue(222)
        self.assertEqual(w.rgb_preview_source.currentIndex(), 0)
        self.assertEqual(self.controller.rgb_snapshot()["config"]["colors"][0], original)
        self.assertEqual(w._rgb_draft["colors"][0][0], 222)
        self.assertTrue(all(cmd in ("#RGB_STATUS", "#RGB_GET") for cmd in self.commands()))

    def test_page_hidden_and_main_closed_stop_animation_and_monitoring(self):
        self.receive_config()
        w = self.window
        w.rgb_preview_source.setCurrentIndex(1)
        self.assertTrue(w._rgb_render_timer.isActive())
        w.navigation.setCurrentRow(0)
        self.assertFalse(w._rgb_render_timer.isActive())
        self.commands()
        self.controller._poll_rgb()
        self.assertEqual(self.commands(), [])
        w.navigation.setCurrentRow(4)
        self.assertIs(w.pages.currentWidget(), w.rgb_page)
        self.assertTrue(w._rgb_render_timer.isActive())
        w.hide()
        APPLICATION.processEvents()
        self.assertFalse(w._rgb_render_timer.isActive())
        w.show()
        APPLICATION.processEvents()
        self.assertTrue(w._rgb_render_timer.isActive())
        w.close()
        self.assertFalse(w._rgb_render_timer.isActive())
        self.commands()
        self.controller._poll_rgb()
        self.assertEqual(self.commands(), [])

    def test_incomplete_stale_and_disconnected_follow_are_clearly_marked(self):
        w = self.window
        w.rgb_preview_source.setCurrentIndex(1)
        self.assertIn("未连接", w.rgb_ring.toolTip())
        self.receive_config()
        w._render_rgb_preview()
        self.assertIn("设备配置跟随", w.rgb_preview_status.text())
        received = w._rgb_device["received_at"]
        with patch("app.ui.main_window.time.monotonic", return_value=received + 4):
            w._render_rgb_preview()
        self.assertIn("状态已过期", w.rgb_ring.toolTip())
        self.assertFalse(w._rgb_follow_seeded)
        for line in READBACK[:5]:
            self.controller._process_rx_line(line)
        self.assertTrue(w._rgb_follow_seeded)
        self.assertEqual(w._rgb_simulator.current_brightness, 35)
        self.controller.disconnect_port()
        self.assertIn("未连接", w.rgb_ring.toolTip())
        self.assertFalse(any(w.rgb_ring._colors[i][j] for i in range(24) for j in range(3)))


if __name__ == "__main__":
    unittest.main()
