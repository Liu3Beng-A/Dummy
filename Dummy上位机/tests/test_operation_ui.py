"""Real host integration of passive operation feedback; no serial/SDL device."""
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from PySide6.QtCore import QEvent, QPoint, QRect, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QPushButton

from tests.test_ui import APPLICATION, FakeView
from tests.test_gamepad_ui import FakeReader
from tests.test_controller import FakePort, ManualScheduler
from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow


MOVEJ = ">0,10,80,0,90,0,0,35"


class OperationUiTests(unittest.TestCase):
    def setUp(self):
        base = Path(__file__).resolve().parents[1] / "build" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=base)
        self.previous_style = APPLICATION.styleSheet()
        self.store = AppStore(self.directory.name)
        self.store.update_settings(auto_poll=False)
        self.scheduler = ManualScheduler()
        self.c = RobotController(scheduler=self.scheduler, serial_factory=FakePort)
        self.reader = FakeReader()
        self.viewer_patch = patch("app.ui.main_window.RobotView", FakeView)
        self.viewer_patch.start()
        self.w = MainWindow(self.c, self.store, gamepad_reader=self.reader)
        self.scheduler.run_ready()
        # Existing RGB and position telemetry are separate from this feature.
        # Isolate its local timer without replacing controller/status methods.
        self.w.rgb_preview_source.setCurrentIndex(0)
        self.c.set_polling(False, 5)
        self.w.show()
        APPLICATION.processEvents()

    def tearDown(self):
        self.w.close()
        self.w.deleteLater()
        APPLICATION.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        APPLICATION.processEvents()
        APPLICATION.setStyleSheet(self.previous_style)
        self.viewer_patch.stop()
        self.directory.cleanup()

    def connect_fake(self):
        self.c.is_connected = True
        self.c.serial_port = FakePort()
        self.c._publish()
        APPLICATION.processEvents()

    def admit_next(self, expected=None):
        item = self.c._tx_queue.get_nowait()
        self.assertIsNotNone(item)
        generation, command = item
        if expected is not None:
            self.assertEqual(command, expected)
        self.assertTrue(self.c._can_write(generation, command), command)
        self.c._on_written(command, self.c._connection_id)
        APPLICATION.processEvents()
        return command

    def submit(self, command):
        self.assertTrue(self.c.send_cmd(command))
        return self.admit_next(command)

    def receive(self, *lines):
        for line in lines:
            self.c._on_transport_line(line, self.c._connection_id)
        APPLICATION.processEvents()

    def enable(self):
        self.submit("!START")
        self.receive("Start pending", "Started ok")
        # Consume the pre-existing mode query, rather than suppressing it or
        # attributing it to the new presentation feature.
        query = self.admit_next()
        self.assertTrue(query.startswith("#GET_CMDMODE "), query)
        request = query.split()[1]
        self.receive(f"ok CMDMODE mode=2 busy=0 issued=0 req={request}")
        self.assertTrue(self.c._tx_queue.empty())

    def disable(self):
        self.submit("!DISABLE")
        self.receive("Disable pending", "Disabled ok")

    @staticmethod
    def geometry(widget):
        return tuple(widget.geometry().getRect())

    def stable_geometry(self):
        return (self.geometry(self.w), self.geometry(self.w.pages),
                self.geometry(self.w.viewer), self.geometry(self.w.motion_controls_scroll),
                self.geometry(self.w.operation_panel))

    @staticmethod
    def settle_layout():
        # Theme and resize changes schedule existing zero-delay toolbar reflow;
        # complete that work before taking the fixed-status geometry baseline.
        for _ in range(8):
            APPLICATION.processEvents()

    def assert_overlay_bounds(self):
        panel, host = self.w.operation_panel, self.w.centralWidget()
        overlay = panel.details_overlay
        self.assertIs(overlay.parentWidget(), host)
        self.assertFalse(overlay.isWindow())
        self.assertGreaterEqual(overlay.x(), 0)
        self.assertGreaterEqual(overlay.y(), 0)
        self.assertLessEqual(overlay.geometry().right(), host.width() - 1)
        self.assertLessEqual(overlay.geometry().bottom(), host.height() - 1)
        self.assertEqual(panel.details_area.horizontalScrollBar().maximum(), 0)

    def test_global_panel_survives_pages_and_idle_reserves_fixed_slots(self):
        self.connect_fake()
        self.disable()
        panel = self.w.operation_panel
        self.assertEqual(panel.title_label.text(), "已失能")
        self.assertFalse(panel.reason_label.isVisible())
        self.assertFalse(panel.next_step_label.isVisible())
        self.assertFalse(panel.elapsed_label.isVisible())
        self.assertEqual(panel.height(), 48)
        for page in (1, 2, 3, 5, 0):
            self.w.navigation.setCurrentRow(page)
            APPLICATION.processEvents()
            self.assertTrue(panel.isVisible(), page)
            self.assertEqual(panel.height(), 48 if page == 0 else 80)
            self.assertIs(panel.parentWidget(), self.w.centralWidget())
            self.assertFalse(self.w.pages.isAncestorOf(panel))
        self.assertTrue(self.c._tx_queue.empty())

    def test_start_pending_and_stop_confirmation_have_distinct_visible_states(self):
        self.connect_fake()
        self.disable()
        self.w.motion_system_buttons[0].click()
        self.admit_next("!START")
        self.receive("Start pending")
        panel = self.w.operation_panel
        self.assertEqual(panel.title_label.text(), "使能确认中")
        self.assertEqual(self.w.motor_status.text(), "使能确认中")
        self.assertFalse(panel.reason_label.isVisible())
        self.assertFalse(panel.next_step_label.isVisible())
        self.assertFalse(panel.elapsed_label.isVisible())
        self.assertFalse(self.c.state()["stop_confirmed"])
        self.w.stop_button.click()
        self.admit_next("!STOP")
        self.receive("Stopping ok")
        self.assertEqual(self.w.motor_status.text(), "停止确认中")
        self.assertFalse(self.c.state()["stop_confirmed"])
        self.assertNotIn("已确认锁定", panel.title_label.text())
        self.receive("STOP locked")
        self.assertEqual(panel.title_label.text(), "STOP 锁定")
        self.assertEqual(self.w.motor_status.text(), "STOP 已确认锁定")
        self.assertFalse(panel.elapsed_label.isVisible())

    def test_accepted_remains_unconfirmed_and_late_receipts_cannot_regress_completion(self):
        self.connect_fake()
        self.enable()
        self.submit(MOVEJ)
        self.receive("queued MOVEJ id=71 mode=INT", "accepted MOVEJ id=71")
        panel = self.w.operation_panel
        self.assertIn("已受理", panel.title_label.text())
        self.assertIn("完成未确认", panel.title_label.text())
        self.assertEqual(self.w.motor_status.text(), "运动已受理")
        self.receive("ok MOVEJ id=70", "ok MOVEL id=71")
        self.assertEqual(self.c.state()["operation"]["phase"], "accepted")
        self.receive("ok MOVEJ id=71 ms=500")
        self.assertEqual(self.c.state()["operation"]["phase"], "completed")
        self.receive("MOTION id=71 kind=MOVEJ phase=2 mode=2 ms=200", "accepted MOVEJ id=71")
        self.assertEqual(panel.title_label.text(), "MoveJ #71 已完成")
        self.assertFalse(panel.elapsed_label.isVisible())
        self.assertTrue(self.c._tx_queue.empty())

    def test_expand_and_quarter_second_refresh_only_change_display(self):
        self.connect_fake()
        self.submit(MOVEJ)
        self.receive("queued MOVEJ id=81 mode=INT", "accepted MOVEJ id=81")
        panel = self.w.operation_panel
        before = self.stable_geometry()
        panel.details_button.click()
        APPLICATION.processEvents()
        self.assertTrue(panel.details_area.isVisible())
        self.assertTrue(panel.details_overlay.isVisible())
        self.assert_overlay_bounds()
        self.assertEqual(self.stable_geometry(), before)
        self.assertEqual(self.w._operation_timer.interval(), 250)
        self.assertTrue(self.w._operation_timer.isActive())
        initial = self.c.operations.latest["since"]
        with patch.object(self.c.operations, "clock", return_value=initial+1.2):
            QTest.qWait(300)
            APPLICATION.processEvents()
            self.assertEqual(panel.elapsed_label.text(), "已等待 1.2 秒")
            for _ in range(8):
                self.w._operation_timer.timeout.emit()
        self.assertTrue(panel.details_button.isChecked())
        self.assertTrue(panel.details_area.isVisible())
        self.assertEqual(self.stable_geometry(), before)
        self.assertTrue(self.c._tx_queue.empty())
        self.assertFalse(self.reader.running)

    def test_guidance_does_not_change_original_disabled_motor_motion_policy(self):
        self.connect_fake()
        self.disable()
        button = self.w.movej_send_button
        self.assertTrue(button.isEnabled())
        self.assertIn("失能", button.accessibleDescription())
        self.assertIn("START", button.accessibleDescription())
        self.w.operation_panel.details_button.click()
        condition = self.w.operation_panel.action_labels["movej"]
        self.assertEqual(condition.property("status"), "attention")
        before = [button.isEnabled(), self.w.movel_send_button.isEnabled(), self.w.stop_button.isEnabled()]
        for _ in range(5):
            self.w._refresh_operation_status()
        self.assertEqual([button.isEnabled(), self.w.movel_send_button.isEnabled(), self.w.stop_button.isEnabled()], before)
        button.click()
        command = self.admit_next()
        self.assertTrue(command.startswith(">"), command)
        self.assertTrue(self.c._tx_queue.empty())

    def test_finite_native_gamepad_gate_has_visible_reason_without_hover(self):
        self.connect_fake()
        self.receive("KIN v=2 xyz=mm abc=deg joints=deg rail=mm")
        self.w.input_mode.setCurrentIndex(1)
        self.w.operation_panel.details_button.click()
        APPLICATION.processEvents()
        button = self.w.gamepad_panel.claim_button
        condition = self.w.operation_panel.action_labels["gamepad"]
        self.assertFalse(button.isEnabled())
        self.assertTrue(condition.isVisible())
        self.assertTrue(condition.isEnabled())
        self.assertEqual(condition.property("status"), "blocked")
        self.assertIn("有限", condition.text())
        self.assertIn("连续手柄暂不可用", condition.text())
        self.assertIn("切换 INT 或 TRJ", condition.text())
        self.assertIn("有限", button.accessibleDescription())
        self.assertFalse(self.c.gamepad.claimed)
        self.assertTrue(self.c._tx_queue.empty())

    def test_status_text_and_overlay_never_reflow_main_layout_in_both_themes(self):
        self.connect_fake()
        panel = self.w.operation_panel
        self.w._operation_timer.stop()
        self.c._stale_timer.stop()
        states = (
            {"phase": "idle", "title": "已使能 · 待命", "tone": "success"},
            {"phase": "waiting", "title": "使能确认中", "tone": "warning",
             "reason": "主控正在检查电机连接", "next_step": "请等待使能结果", "elapsed_s": 0.4},
            {"phase": "accepted", "title": "MoveJ #71 已受理 · 完成未确认", "tone": "info",
             "reason": "主控已受理，等待回读位置到达目标。", "next_step": "观察实际位置", "elapsed_s": 1.2},
            {"phase": "observed_reached", "title": "MoveJ #71 · 回读位置已到目标", "tone": "success",
             "reason": "当前位置处于目标容差内", "next_step": "可发送下一个目标"},
            {"phase": "locked", "title": "保护锁定", "tone": "error",
             "reason": "PROTECT J5 failed reason=PROTECTION_STATUS_TIMEOUT", "next_step": "检查电机通信后重新使能"},
            {"phase": "uncertain", "title": "很长的设备状态提示" * 35, "tone": "warning",
             "reason": "完整原因应能在详情或辅助描述中查看" * 70,
             "next_step": "很长的下一步说明" * 70, "elapsed_s": 123.4},
        )
        for theme in ("light", "dark"):
            self.w.apply_theme(theme)
            for width, height in ((1120, 690), (1440, 900)):
                self.w.resize(width, height)
                self.settle_layout()
                baseline = self.stable_geometry()
                for state in states:
                    for expanded in (False, True, False):
                        with self.subTest(theme=theme, size=(width, height), phase=state["phase"], expanded=expanded):
                            panel.set_status(state, {})
                            panel.details_button.setChecked(expanded)
                            APPLICATION.processEvents()
                            self.assertEqual((self.w.width(), self.w.height()), (width, height))
                            self.assertEqual(panel.height(), 48)
                            self.assertEqual(self.stable_geometry(), baseline)
                            self.assertTrue(panel.title_label.isVisible())
                            self.assertTrue(self.w.topbar.rect().contains(QRect(
                                panel.title_label.mapTo(self.w.topbar, QPoint()), panel.title_label.size())))
                            for label in (panel.reason_label, panel.next_step_label, panel.elapsed_label):
                                self.assertFalse(label.isVisible())
                            self.assertTrue(panel.details_button.isVisible())
                            self.assertFalse(self.w.motor_status.isVisible())
                            self.assertLessEqual(panel.details_button.geometry().right(), panel.width())
                            self.assertEqual(panel.details_overlay.isVisible(), expanded)
                            if expanded:
                                self.assert_overlay_bounds()
                                for label in panel.action_labels.values():
                                    self.assertLessEqual(label.width(), panel.details_area.viewport().width())
        self.assertTrue(self.c._tx_queue.empty())

    def test_overlay_follows_resize_and_outside_click_escape_never_send_motion_or_release_gamepad(self):
        self.connect_fake()
        panel = self.w.operation_panel
        panel.details_button.click()
        APPLICATION.processEvents()
        self.w.resize(1120, 690)
        self.settle_layout()
        self.assert_overlay_bounds()
        # Also exercise repositioning at the larger supported layout.
        self.w.resize(1440, 900)
        self.settle_layout()
        self.w.motion_controls_scroll.ensureWidgetVisible(self.w.movej_send_button, 0, 0)
        self.settle_layout()
        with patch.object(self.w, "_suspend_gamepad") as suspend, \
                patch.object(self.c.gamepad, "release") as release:
            # A child overlay has no window activation cycle. Ordinary outside
            # controls consume their first click so it only dismisses details.
            outside_motion = self.w.motion_system_buttons[2]  # Visible HOME.
            point = outside_motion.mapTo(self.w.centralWidget(), outside_motion.rect().center())
            self.assertFalse(panel.details_overlay.geometry().contains(point),
                             (panel.details_overlay.geometry().getRect(), point.toTuple()))
            QTest.mouseClick(outside_motion, Qt.MouseButton.LeftButton)
            APPLICATION.processEvents()
            self.assertFalse(panel.details_overlay.isVisible())
            self.assertFalse(panel.details_button.isChecked())
            self.assertTrue(self.c._tx_queue.empty())
            panel.details_button.click()
            APPLICATION.processEvents()
            # Defensive receiver-level event: normally this button is covered
            # by details, so a real pointer hits the overlay. Even a directly
            # delivered event must not leak a MoveJ through its dismissal.
            QTest.mouseClick(self.w.movej_send_button, Qt.MouseButton.LeftButton)
            APPLICATION.processEvents()
            self.assertFalse(panel.details_overlay.isVisible())
            self.assertTrue(self.c._tx_queue.empty())
            panel.details_button.click()
            APPLICATION.processEvents()
            QTest.keyClick(self.w, Qt.Key.Key_Escape)
            APPLICATION.processEvents()
            self.assertFalse(panel.details_overlay.isVisible())
            self.assertTrue(self.c._tx_queue.empty())
            suspend.assert_not_called()
            release.assert_not_called()

    def test_overlay_safety_stop_keeps_original_single_click_command(self):
        self.connect_fake()
        self.w.operation_panel.details_button.click()
        APPLICATION.processEvents()
        self.assertTrue(self.w.operation_panel.details_overlay.isVisible())
        QTest.mouseClick(self.w.stop_button, Qt.MouseButton.LeftButton)
        APPLICATION.processEvents()
        self.assertFalse(self.w.operation_panel.details_overlay.isVisible())
        self.admit_next("!STOP")
        self.assertTrue(self.c._tx_queue.empty())

    def test_accepted_int_movej_uses_fresh_sent_target_not_editor_and_expiry_removes_reached(self):
        self.connect_fake()
        self.enable()
        # Permit the existing shared telemetry interpretation while stopping its
        # timer. The passive operation feature must not create any TX itself.
        self.c._poll_enabled = True
        self.c._poll_timer.stop()
        self.c._stale_timer.stop()
        self.submit(MOVEJ)
        self.receive("queued MOVEJ id=91 mode=INT", "accepted MOVEJ id=91")
        self.w.joint_spins[0].setValue(55)
        self.receive("ok 0 10 80 0 90 0 0")
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.assertIn("回读位置已到目标", self.w.operation_panel.title_label.text())
        self.assertFalse(self.w.operation_panel.elapsed_label.isVisible())
        self.assertEqual(self.w.motor_status.text(), "回读已到目标")
        self.assertEqual(self.w.joint_spins[0].value(), 55)
        self.assertTrue(self.c._tx_queue.empty())
        self.receive("ok 0.15 10 80 0 90 0 0")
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.receive("ok 0.21 10 80 0 90 0 0")
        self.assertNotEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.receive("ok 0 10 80 0 90 0 0")
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.c.last_actual_at = time.time() - 4
        self.w._refresh_operation_status()
        self.assertNotEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.assertNotIn("回读位置已到目标", self.w.operation_panel.title_label.text())
        self.assertTrue(self.c._tx_queue.empty())

    def test_readback_only_reaches_the_latest_accepted_target(self):
        self.connect_fake()
        self.enable()
        self.c._poll_enabled = True
        self.c._poll_timer.stop()
        self.submit(MOVEJ)
        self.receive("queued MOVEJ id=101 mode=INT", "accepted MOVEJ id=101", "ok 0 10 80 0 90 0 0")
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.submit(">15,10,80,0,90,0,0,35")
        self.receive("queued MOVEJ id=102 mode=INT", "ok 15 10 80 0 90 0 0")
        self.assertNotEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.receive("accepted MOVEJ id=102")
        self.assertNotEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.receive("ok 0 10 80 0 90 0 0")
        self.assertNotEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.receive("ok 15 10 80 0 90 0 0")
        self.assertEqual(self.c.state()["operation"]["phase"], "observed_reached")
        self.assertEqual(self.c.state()["operation"]["motion_id"], 102)
        self.receive("ok MOVEJ id=102 ms=500", "ok 16 10 80 0 90 0 0")
        self.assertEqual(self.c.state()["operation"]["phase"], "completed")
        self.assertEqual(self.w.operation_panel.title_label.text(), "MoveJ #102 已完成")
        self.assertTrue(self.c._tx_queue.empty())

    def test_expanded_protection_in_short_windows_keeps_motion_buttons_readable(self):
        self.connect_fake()
        self.receive("PROTECT J5 failed reason=PROTECTION_STATUS_TIMEOUT")
        self.w.operation_panel.details_button.setChecked(True)
        scroll = self.w.motion_controls_scroll
        self.assertTrue(scroll.widgetResizable())
        self.assertEqual(scroll.horizontalScrollBarPolicy(), Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.assertNotEqual(scroll.verticalScrollBarPolicy(), Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        buttons = [self.w.movej_send_button, self.w.movej_read_button,
                   *self.w.motion_gripper_group.findChildren(QPushButton)]
        self.assertGreaterEqual(len(buttons), 5)
        for theme in ("light", "dark"):
            self.w.apply_theme(theme)
            for height in (760, 690):
                with self.subTest(theme=theme, height=height):
                    self.w.resize(1120, height)
                    APPLICATION.processEvents()
                    APPLICATION.processEvents()
                    self.assertEqual(self.w.size().width(), 1120)
                    self.assertEqual(self.w.size().height(), height)
                    for button in buttons:
                        self.assertGreaterEqual(button.height(), button.minimumSizeHint().height(), button.text())
                    self.assertEqual(scroll.horizontalScrollBar().maximum(), 0)
                    if scroll.widget().height() > scroll.viewport().height():
                        self.assertGreater(scroll.verticalScrollBar().maximum(), 0)
                    self.assertTrue(self.w.operation_panel.details_area.isVisible())
        self.assertTrue(self.c._tx_queue.empty())


if __name__ == "__main__":
    unittest.main()
