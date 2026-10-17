"""Verify RGB preview isolation and page lifecycle with real Qt controls."""
import copy
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
from app.protocol.constants import RGB_MODES, RGB_MOTION_STATES, RGB_STATE_MODES, RGB_STATES


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
        self.assertEqual(self.window.pages.currentIndex(), 0)
        self.assertTrue(self.window.rgb_window.isVisible())

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
        self.controller.send_cmd("#RGB_GET")
        for line in READBACK:
            self.controller._process_rx_line(line)
        self.assertTrue(self.controller.rgb_snapshot()["complete"])
        self.commands()  # This helper models the initial read-only config fetch.

    def receive_motion_config(self, *, slot_one_period=1400):
        self.controller.is_connected = True
        self.controller.serial_port = FakePort()
        self.controller.send_cmd("#RGB_GET")
        for line in READBACK:
            if line == "RGB_GET end":
                self.controller._process_rx_line("RGB_CAPS motion=1")
                for slot in range(4):
                    period = slot_one_period if slot == 1 else 1400
                    self.controller._process_rx_line(f"RGB_MOTION {slot} {period} 0 3")
            if line.startswith("RGB_STATUS "):
                line = line.replace("mode=8", "mode=13")
            for state in RGB_MOTION_STATES:
                if line.startswith(f"RGB_STATE {state} "):
                    parts = line.split()
                    parts[2] = "13"
                    line = " ".join(parts)
            self.controller._process_rx_line(line)
        self.assertTrue(self.controller.rgb_snapshot()["complete"])
        self.assertTrue(self.controller.rgb_snapshot()["motion_supported"])
        self.commands()

    def test_motion_editor_preserves_state_colors_and_old_preset_offline(self):
        w = self.window
        old_effect = copy.deepcopy(w._rgb_draft["effects"][8])
        self.assertEqual(len(RGB_MODES), 13)
        self.assertEqual(w.rgb_preview_mode.count(), 13)
        self.assertEqual(w.fields["_rgb_effect_mode"].widget.count(), 7)
        self.assertEqual(w.fields["_rgb_state_mode"].widget.count(), len(RGB_STATE_MODES))
        for slot, kind in enumerate((1, 2, 3, 0)):
            state = RGB_MOTION_STATES[slot]
            original_color = w._rgb_draft["states"][state][1:]
            w.fields["_rgb_motion_idx"].widget.setCurrentIndex(slot)
            w.fields["_rgb_motion_kind"].widget.setCurrentIndex(kind)
            w.fields["_rgb_motion_params"][0].widget.setValue(2000 + slot * 100)
            w.fields["_rgb_motion_params"][1].widget.setValue(4 + slot)
            w.fields["_rgb_motion_reverse"].widget.setChecked(True)
            self.assertEqual(w._rgb_draft["states"][state], (13 + kind,) + original_color)
            self.assertEqual(w._rgb_draft["motion"][slot], (2000 + slot * 100, 1, 4 + slot))
            self.assertEqual(w.rgb_preview_state.currentData(), state)
            self.assertEqual(w.viewer.rgb_effect_mode, 13 + kind)
            self.assertEqual(w.rgb_preview_title.text(), RGB_STATE_MODES[13 + kind].split(" ", 1)[1])
        self.assertEqual(w._rgb_draft["effects"][8], old_effect)
        w.fields["_rgb_motion_idx"].widget.setCurrentIndex(0)
        self.assertEqual(w._rgb_field_values("_rgb_motion_params"), (2000, 4))
        self.assertEqual(w.fields["_rgb_motion_kind"].current(), 1)
        self.assertEqual(self.commands(), [])

    def test_legacy_motion_apply_is_disabled_but_local_preview_remains_available(self):
        self.receive_config()
        w = self.window
        self.assertFalse(w.rgb_motion_apply.isEnabled())
        self.assertIn("升级", w.rgb_motion_support.text())
        w.fields["_rgb_motion_kind"].widget.setCurrentIndex(2)
        self.assertEqual(w.rgb_preview_state.currentData(), "RUN_SEQ")
        self.assertEqual(w.viewer.rgb_effect_mode, 15)
        w.rgb_motion_apply.click()
        self.assertEqual(self.commands(), [])

    def test_legacy_rotation_preset_color_is_snapshotted_for_independent_preview(self):
        w = self.window
        w._rgb_draft["effects"][8] = (10, 20, 30) + w._rgb_draft["effects"][8][3:]
        w._rgb_draft["states"]["RUN_SEQ"] = (8, 0, 0, 0, 0)
        w.fields["_rgb_motion_kind"].widget.setCurrentIndex(1)
        self.assertEqual(w._rgb_draft["states"]["RUN_SEQ"], (14, 1, 10, 20, 30))
        self.assertEqual(w.rgb_preview_state.currentData(), "RUN_SEQ")
        self.assertEqual(w.viewer.rgb_effect_mode, 14)
        self.assertEqual(self.commands(), [])

    def test_supported_motion_apply_sends_one_atomic_command_without_leaving_auto(self):
        self.receive_motion_config()
        w = self.window
        self.assertTrue(w.rgb_motion_apply.isEnabled())
        w.fields["_rgb_motion_idx"].widget.setCurrentIndex(2)
        w.fields["_rgb_motion_kind"].widget.setCurrentIndex(3)
        w.fields["_rgb_motion_params"][0].widget.setValue(2200)
        w.fields["_rgb_motion_params"][1].widget.setValue(5)
        w.fields["_rgb_motion_reverse"].widget.setChecked(True)
        self.assertEqual(self.commands(), [])
        w.rgb_motion_apply.click()
        self.assertEqual(self.commands(), ["!RGB_MOTION 2 3 2200 1 5"])
        self.assertTrue(self.controller.rgb_snapshot()["config"]["automatic"])

    def test_motion_drafts_survive_device_readback_and_window_close(self):
        self.receive_motion_config()
        w = self.window
        w.fields["_rgb_motion_kind"].widget.setCurrentIndex(1)
        w.fields["_rgb_motion_params"][0].widget.setValue(2100)
        w.fields["_rgb_color_vars"][0].widget.setValue(77)
        self.receive_motion_config(slot_one_period=2700)
        self.assertEqual(w._rgb_draft["motion"][0], (2100, 0, 3))
        self.assertEqual(w._rgb_draft["motion"][1], (2700, 0, 3))
        self.assertEqual(w._rgb_draft["states"]["RUN_SEQ"][0], 14)
        self.assertEqual(w._rgb_draft["colors"][0][0], 77)
        w.rgb_window.close()
        APPLICATION.processEvents()
        self.assertEqual(w.rgb_preview_source.currentIndex(), 1)
        self.assertEqual(w._rgb_draft["motion"][0], (2100, 0, 3))
        self.assertEqual(w._rgb_draft["states"]["RUN_SEQ"][0], 14)
        self.assertEqual(w._rgb_draft["colors"][0][0], 77)
        self.assertTrue(all(cmd in ("#RGB_STATUS", "#RGB_GET") for cmd in self.commands()))

    def test_independent_state_modes_are_available_only_to_motion_and_calibration(self):
        w = self.window
        mode = w.fields["_rgb_state_mode"].widget
        for state in ("BOOT", "RUN_INT", "CALIBRATION", "DISABLED"):
            index = next(i for i, (code, _) in enumerate(RGB_STATES) if code == state)
            w.fields["_rgb_state"].widget.setCurrentIndex(index)
            for i in range(13, 17):
                self.assertEqual(mode.model().item(i).isEnabled(), state in RGB_MOTION_STATES)
        original = w._rgb_draft["states"]["DISABLED"]
        mode.setCurrentIndex(16)  # Programmatic invalid selection is rejected too.
        self.assertEqual(w._rgb_draft["states"]["DISABLED"], original)
        self.assertEqual(mode.currentIndex(), original[0])
        self.assertEqual(self.commands(), [])

    def test_default_follow_button_local_edit_and_model_light_share_frames(self):
        w = self.window
        self.assertEqual(w.rgb_preview_source.currentIndex(), 1)
        self.assertFalse(w.rgb_preview_source.isVisible())
        self.assertTrue(w.rgb_preview_source_button.isVisible())
        self.assertFalse(w.rgb_preview_mode.isEnabled())
        self.assertIn("未连接", w.rgb_preview_title.text())
        self.assertFalse(hasattr(w, "rgb_ring"))
        self.assertFalse(hasattr(w, "motion_rgb_ring"))
        self.assertFalse(hasattr(w, "motion_rgb_overlay"))
        w.rgb_preview_source_button.click()
        self.assertEqual(w.rgb_preview_source.currentIndex(), 0)
        self.assertTrue(w.rgb_preview_mode.isEnabled())
        self.assertIn("返回设备", w.rgb_preview_source_button.text())
        self.assertEqual(len(w.viewer.rgb_frame), 24)
        self.assertTrue(w.rgb_window.isAncestorOf(w.rgb_preview_source_button))
        w.rgb_preview_source_button.click()
        self.assertEqual(w.rgb_preview_source.currentIndex(), 1)
        w.fields["_rgb_bright"].widget.setValue(70)
        self.assertEqual(w.rgb_preview_source.currentIndex(), 0)
        self.assertEqual(self.commands(), [])

    def test_zero_breath_frames_keep_power_on_and_fade_out_keeps_cover_active(self):
        w = self.window
        w.rgb_preview_source.setCurrentIndex(0)
        w.rgb_preview_state.setCurrentIndex(w.rgb_preview_state.findData("DISABLED"))
        w._rgb_draft["brightness"] = 1
        w._rgb_simulator.reset(initial_brightness=0)
        zero = [(0, 0, 0)] * 24
        with patch.object(w._rgb_simulator, "frame", return_value=zero):
            w._render_rgb_preview()
            self.assertEqual(w.viewer.rgb_frame, tuple(zero))
            self.assertIs(w.viewer.rgb_light_on, True)
            # Turning the switch off must preserve an already running fade-out.
            w._rgb_simulator.reset(initial_brightness=30)
            w.rgb_preview_on.setChecked(False)
            w._render_rgb_preview()
            self.assertIs(w.viewer.rgb_light_on, True)
            w._rgb_simulator.reset(initial_brightness=0)
            w._render_rgb_preview()
            self.assertIs(w.viewer.rgb_light_on, False)
        self.assertEqual(self.commands(), [])

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
        self.assertEqual(w.viewer.rgb_effect_mode, 8)
        w.rgb_tabs.setCurrentIndex(3)
        w.fields["_rgb_state"].widget.setCurrentIndex(10)  # calibration
        w.fields["_rgb_state_mode"].widget.setCurrentIndex(9)
        self.assertEqual(w.rgb_preview_state.currentData(), "CALIBRATION")
        self.assertEqual(w._rgb_simulator.mode, 9)
        self.assertEqual(w.viewer.rgb_effect_mode, 9)
        self.assertEqual(self.commands(), [])
        self.assertTrue(w.rgb_window.isVisible())
        self.assertTrue(w.rgb_window.isAncestorOf(w.rgb_tabs))

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
        self.assertEqual(w.viewer.rgb_effect_mode, 8)
        self.assertFalse(w.rgb_preview_mode.isEnabled())
        self.assertIn("模拟", w.rgb_preview_brightness.text())
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
        self.assertTrue(w._rgb_render_timer.isActive())
        self.assertFalse(hasattr(w, "motion_rgb_ring"))
        self.assertEqual(len(w.viewer.rgb_frame), 24)
        self.assertTrue(self.controller._rgb_monitoring)
        w.navigation.setCurrentRow(1)
        self.assertTrue(w._rgb_render_timer.isActive())
        self.assertTrue(self.controller._rgb_monitoring)
        w.rgb_window.close()
        APPLICATION.processEvents()
        self.assertFalse(w.rgb_window.isVisible())
        self.assertFalse(w._rgb_render_timer.isActive())
        self.commands()
        self.controller._poll_rgb()
        self.assertEqual(self.commands(), [])
        w.navigation.setCurrentRow(4)
        self.assertEqual(w.pages.currentIndex(), 0)
        self.assertTrue(w.rgb_window.isVisible())
        self.assertTrue(w._rgb_render_timer.isActive())
        w.rgb_window.hide()
        w.hide()
        APPLICATION.processEvents()
        self.assertFalse(w._rgb_render_timer.isActive())
        w.show()
        APPLICATION.processEvents()
        self.assertTrue(w._rgb_render_timer.isActive())
        w.close()
        self.assertFalse(w.rgb_window.isVisible())
        self.assertFalse(w._rgb_render_timer.isActive())
        self.commands()
        self.controller._poll_rgb()
        self.assertEqual(self.commands(), [])

    def test_incomplete_stale_and_disconnected_follow_are_clearly_marked(self):
        w = self.window
        w.rgb_preview_source.setCurrentIndex(1)
        self.assertIn("未连接", w.rgb_preview_title.text())
        self.receive_config()
        w._render_rgb_preview()
        self.assertIn("设备配置跟随", w.rgb_preview_status.text())
        received = w._rgb_device["received_at"]
        with patch("app.ui.main_window.time.monotonic", return_value=received + 4):
            w._render_rgb_preview()
        self.assertIn("状态已过期", w.rgb_preview_title.text())
        self.assertFalse(w._rgb_follow_seeded)
        self.assertIs(w.viewer.rgb_light_on, False)
        self.assertIsNone(w.viewer.rgb_effect_mode)
        for line in READBACK[:5]:
            self.controller._process_rx_line(line)
        self.assertTrue(w._rgb_follow_seeded)
        self.assertEqual(w._rgb_simulator.current_brightness, 35)
        self.controller.disconnect_port()
        self.assertIn("未连接", w.rgb_preview_title.text())
        self.assertFalse(any(w.viewer.rgb_frame[i][j] for i in range(24) for j in range(3)))

    def test_modeless_window_reopens_without_losing_drafts_or_duplicate_queries(self):
        w = self.window
        window = w.rgb_window
        self.assertTrue(window.isWindow())
        self.assertFalse(window.isModal())
        w.fields["_rgb_color_vars"][0].widget.setValue(77)
        self.assertEqual(w.rgb_preview_source.currentIndex(), 0)
        window.close()
        APPLICATION.processEvents()
        self.assertFalse(window.isVisible())
        self.assertEqual(w.rgb_preview_source.currentIndex(), 1)
        self.assertEqual(w.rgb_preview_source_button.text(), "切换到本地编辑预览")
        self.assertTrue(w._rgb_render_timer.isActive(), 'The main model still needs animation')
        w.navigation.setCurrentRow(4)
        APPLICATION.processEvents()
        self.assertIs(w.rgb_window, window)
        self.assertTrue(window.isVisible())
        self.assertEqual(w._rgb_field_values("_rgb_color_vars")[0], 77)
        self.assertEqual(w.rgb_preview_source.currentIndex(), 1)
        self.assertEqual(self.commands(), [])

    def test_close_returns_to_device_follow_without_applying_the_local_draft(self):
        self.receive_config()
        w = self.window
        device_color = self.controller.rgb_snapshot()["config"]["colors"][0]
        w.fields["_rgb_color_vars"][0].widget.setValue(77)
        self.assertEqual(w.rgb_preview_source.currentIndex(), 0)
        self.commands()
        w.rgb_window.close()
        APPLICATION.processEvents()
        self.assertEqual(w.rgb_preview_source.currentIndex(), 1)
        self.assertEqual(w._rgb_draft["colors"][0][0], 77)
        self.assertEqual(self.controller.rgb_snapshot()["config"]["colors"][0], device_color)
        self.assertIn("设备配置跟随", w.rgb_preview_status.text())
        self.assertEqual(w.viewer.rgb_effect_mode, w._rgb_simulator.mode)
        self.assertTrue(all(cmd in ("#RGB_STATUS", "#RGB_GET") for cmd in self.commands()))

    def test_minimized_rgb_window_only_stops_animation_when_model_is_not_visible(self):
        w = self.window
        w.fields["_rgb_color_vars"][0].widget.setValue(77)
        w.rgb_window.showMinimized()
        APPLICATION.processEvents()
        self.assertEqual(w.rgb_preview_source.currentIndex(), 0)
        self.assertTrue(w._rgb_render_timer.isActive())
        w.navigation.setCurrentRow(1)
        APPLICATION.processEvents()
        self.assertFalse(w._rgb_render_timer.isActive())
        w.rgb_window.showNormal()
        APPLICATION.processEvents()
        self.assertTrue(w._rgb_render_timer.isActive())

    def test_save_stages_disable_duplicate_save_and_keep_feedback_separate_from_rgb_summary(self):
        self.receive_config()
        w = self.window
        clicked = []
        w.rgb_save_button.clicked.connect(lambda: clicked.append(True))
        snapshot = self.controller.rgb_snapshot()
        for stage, caption, message in (
            ("preparing", "等待回读…", "正在确认设备状态，等待回读"),
            ("waiting", "保存中…", "RGB 保存中，查询已暂停"),
            ("settling", "保存中…", "已收到保存回执，等待尾包"),
            ("uncertain", "结果未确认", "6 秒未确认，查询已暂停，请断开重连核实"),
        ):
            snapshot = copy.deepcopy(snapshot)
            snapshot.update(save_busy=True, save_stage=stage, save_message=message)
            w._rgb_device_changed(snapshot)
            self.assertFalse(w.rgb_save_button.isEnabled())
            self.assertEqual(w.rgb_save_button.text(), caption)
            self.assertTrue(w.rgb_save_status.isVisible())
            self.assertEqual(w.rgb_save_status.text(), message)
            snapshot["runtime"]["bright"] += 1
            w._rgb_device_changed(snapshot)
            self.assertEqual(w.rgb_save_status.text(), message)
            w.rgb_save_button.click()
        self.assertEqual(clicked, [])
        snapshot = copy.deepcopy(snapshot)
        snapshot.update(save_busy=False, save_stage="idle", save_message="RGB 保存成功")
        w._rgb_device_changed(snapshot)
        self.assertTrue(w.rgb_save_button.isEnabled())
        self.assertEqual(w.rgb_save_button.text(), "保存全部 RGB 配置")
        self.assertEqual(w.rgb_save_status.text(), "RGB 保存成功")
        snapshot["runtime"]["bright"] += 1
        w._rgb_device_changed(snapshot)
        self.assertEqual(w.rgb_save_status.text(), "RGB 保存成功")
        self.assertFalse(any(command.startswith("!RGB_SAVE") for command in self.commands()))

    def test_save_button_starts_one_controller_transaction_and_updates_immediately(self):
        self.receive_config()
        w = self.window
        self.controller._motor_state_known = True
        self.controller._motor_enabled = False
        with patch.object(self.controller.rgb_save, "begin", wraps=self.controller.rgb_save.begin) as begin:
            w.rgb_save_button.click()
            w.rgb_save_button.click()
            begin.assert_called_once_with("!RGB_SAVE")
        self.assertTrue(self.controller.rgb_snapshot()["save_busy"])
        self.assertTrue(self.controller.state()["rgb_save_busy"])
        self.assertEqual(w.rgb_save_button.text(), "等待回读…")
        self.assertFalse(w.rgb_save_button.isEnabled())
        self.assertIn("后台查询已暂停", w.rgb_save_status.text())

    def test_save_uncertainty_preserves_last_model_lamp_until_disconnect(self):
        self.receive_config()
        w = self.window
        last = (w.viewer.rgb_frame, w.viewer.rgb_light_on, w.viewer.rgb_effect_mode)
        self.assertIs(last[1], True)
        snapshot = self.controller.rgb_snapshot()
        received = snapshot["received_at"]
        snapshot.update(save_busy=True, save_stage="uncertain", save_message="")
        with patch("app.ui.main_window.time.monotonic", return_value=received + 20):
            w._rgb_device_changed(snapshot)
            w._render_rgb_preview()
        self.assertEqual((w.viewer.rgb_frame, w.viewer.rgb_light_on, w.viewer.rgb_effect_mode), last)
        self.assertTrue(w._rgb_follow_seeded)
        self.assertIn("保留上次模型灯光", w.rgb_preview_status.text())
        self.assertIn("6 秒", w.rgb_save_status.text())
        self.assertIn("断开重连", w.rgb_save_status.text())
        self.assertIn("迟到回执", w.rgb_save_status.text())
        disconnected = copy.deepcopy(snapshot)
        disconnected.update(connected=False, save_busy=False, save_stage="idle", save_message="连接断开，保存结果未确认")
        w._rgb_device_changed(disconnected)
        self.assertIs(w.viewer.rgb_light_on, False)
        self.assertFalse(any(channel for color in w.viewer.rgb_frame for channel in color))
        self.assertIn("未连接", w.rgb_preview_title.text())
        self.assertIsNone(w._rgb_save_resume_until)

    def test_save_resume_grace_is_bounded_and_fresh_status_restores_normal_follow(self):
        self.receive_config()
        w = self.window
        snapshot = self.controller.rgb_snapshot()
        received = snapshot["received_at"]
        last = (w.viewer.rgb_frame, w.viewer.rgb_light_on, w.viewer.rgb_effect_mode)
        snapshot.update(save_busy=True, save_stage="waiting", save_message="保存中")
        with patch("app.ui.main_window.time.monotonic", return_value=received + 10):
            w._rgb_device_changed(snapshot)
            finished = copy.deepcopy(snapshot)
            finished.update(save_busy=False, save_stage="idle", save_message="RGB 保存成功")
            w._rgb_device_changed(finished)
        with patch("app.ui.main_window.time.monotonic", return_value=received + 10.5):
            w._render_rgb_preview()
        self.assertEqual((w.viewer.rgb_frame, w.viewer.rgb_light_on, w.viewer.rgb_effect_mode), last)
        self.assertIn("等待恢复查询", w.rgb_preview_status.text())
        with patch("app.ui.main_window.time.monotonic", return_value=received + 11.1):
            w._render_rgb_preview()
        self.assertIn("状态已过期", w.rgb_preview_title.text())
        self.assertIs(w.viewer.rgb_light_on, False)
        finished["received_at"] = received + 11.2
        with patch("app.ui.main_window.time.monotonic", return_value=received + 11.2):
            w._rgb_device_changed(finished)
        self.assertIs(w.viewer.rgb_light_on, True)
        self.assertTrue(w._rgb_follow_seeded)
        self.assertIn("设备配置跟随", w.rgb_preview_status.text())
        self.assertEqual(w.rgb_save_status.text(), "RGB 保存成功")
        self.assertIsNone(w._rgb_save_resume_until)


if __name__ == "__main__":
    unittest.main()
