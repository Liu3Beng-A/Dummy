"""Shared operation card occupies reserved page headers, never a global row."""
from pathlib import Path
import queue
import tempfile
import unittest
from unittest.mock import patch

from PySide6.QtCore import QEvent, QPoint, QRect, Qt
from PySide6.QtGui import QFontDatabase
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QFrame

from tests.test_ui import APPLICATION, FakeView
from tests.test_gamepad_ui import FakeReader
from tests.test_controller import FakePort, ManualScheduler
from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow


SIZES = ((1120, 690), (1440, 900), (2502, 1000))


class OperationLayoutTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Windows' offscreen plugin otherwise substitutes wider fallback glyphs.
        cls.font_ids = []
        for name in ("msyh.ttc", "msyhbd.ttc", "segoeui.ttf", "segoeuib.ttf", "seguisym.ttf"):
            path = Path("C:/Windows/Fonts") / name
            if path.exists():
                font_id = QFontDatabase.addApplicationFont(str(path))
                if font_id >= 0:
                    cls.font_ids.append(font_id)

    @classmethod
    def tearDownClass(cls):
        for font_id in cls.font_ids:
            QFontDatabase.removeApplicationFont(font_id)

    def setUp(self):
        base = Path(__file__).resolve().parents[1] / "build" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=base)
        self.previous_style = APPLICATION.styleSheet()
        self.store = AppStore(self.directory.name)
        self.store.update_settings(auto_poll=False)
        self.scheduler = ManualScheduler()
        self.c = RobotController(scheduler=self.scheduler, serial_factory=FakePort)
        with patch("app.ui.main_window.RobotView", FakeView):
            self.w = MainWindow(self.c, self.store, gamepad_reader=FakeReader())
        self.scheduler.run_ready()
        self.w.rgb_preview_source.setCurrentIndex(0)
        for timer in (self.c._poll_timer, self.c._rgb_monitor_timer, self.c._stale_timer,
                      self.w._operation_timer, self.w._rgb_render_timer):
            timer.stop()
        self.w.show()
        self.settle()

    def tearDown(self):
        self.w.close()
        self.w.deleteLater()
        APPLICATION.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        self.settle()
        APPLICATION.setStyleSheet(self.previous_style)
        self.store.close()
        self.directory.cleanup()

    @staticmethod
    def settle():
        for _ in range(10):
            APPLICATION.processEvents()

    def central_rect(self, widget):
        origin = self.w.centralWidget().mapFromGlobal(widget.mapToGlobal(QPoint()))
        return QRect(origin, widget.size())

    def assert_anchor(self):
        header, slot = self.w._page_header_slots[self.w.pages.currentWidget()]
        panel = self.w.operation_panel
        self.assertIs(panel.parentWidget(), self.w.centralWidget())
        self.assertTrue(panel.isVisible())
        motion = self.w.pages.currentWidget() is self.w.motion_page
        self.assertEqual(panel.height(), 48 if motion else 80)
        self.assertGreaterEqual(panel.width(), 120 if motion else 395)
        if motion:
            self.assertEqual(header.height(), 48)
        if not motion:
            self.assertLessEqual(panel.width(), 545)
        self.assertEqual(panel.geometry(), self.central_rect(slot))
        self.assertTrue(self.central_rect(header).contains(panel.geometry()))
        self.assertEqual(panel.geometry().right(), self.central_rect(header).right())
        self.assertFalse(self.w.pages.isAncestorOf(panel))
        return header, slot

    def assert_shell_starts_below_serial(self):
        top = self.w.findChild(QFrame, "TopBar")
        sidebar = self.w.findChild(QFrame, "Sidebar")
        expected_y = self.central_rect(top).bottom() + 1
        self.assertEqual(self.central_rect(self.w.pages).top(), expected_y)
        self.assertEqual(self.central_rect(sidebar).top(), expected_y)
        # The unified header belongs to the menu widget; central keeps body only.
        self.assertEqual(self.w.centralWidget().layout().count(), 1)

    def assert_header_controls_fit(self):
        header, slot = self.assert_anchor()
        all_controls = [self.w._motion_header_title, self.w.motion_enable_button, self.w.motion_disable_button,
                        *self.w.motion_system_buttons[2:4], self.w.motion_calibration_button]
        rectangles = [self.central_rect(control) for control in all_controls]
        for control, rect in zip(all_controls, rectangles):
            self.assertTrue(control.isVisible(), control.objectName())
            self.assertTrue(self.central_rect(header).contains(rect),
                            (control.objectName(), rect.getRect(), header.geometry().getRect()))
            self.assertFalse(rect.intersects(self.central_rect(slot)), rect.getRect())
            self.assertGreaterEqual(control.width(), control.minimumSizeHint().width(),
                                    (getattr(control, "text", lambda: "mode")(), control.width(),
                                     control.minimumSizeHint().width()))
            self.assertGreaterEqual(control.height(), control.minimumSizeHint().height())
        for index, first in enumerate(rectangles):
            for second in rectangles[index + 1:]:
                self.assertFalse(first.intersects(second), (first.getRect(), second.getRect()))

    def stable_geometry(self):
        return [widget.geometry() for widget in (
            self.w, self.w.pages, self.w.operation_panel, self.w.viewer,
            self.w.motion_controls_scroll, self.w._motion_header_left,
            *self.w.motion_system_buttons, self.w.mode_combo)]

    def test_card_uses_header_space_and_controls_fit_at_three_sizes_both_themes(self):
        for theme in ("light", "dark"):
            self.w.apply_theme(theme)
            for width, height in SIZES:
                with self.subTest(theme=theme, width=width):
                    self.w.resize(width, height)
                    self.settle()
                    self.assertEqual((self.w.width(), self.w.height()), (width, height))
                    self.assert_shell_starts_below_serial()
                    self.assert_header_controls_fit()
                    self.assertEqual(self.w.motion_controls_scroll.horizontalScrollBar().maximum(), 0)
        self.assertTrue(self.c._tx_queue.empty())

    def test_resize_and_splitter_extreme_preserve_left_header_budget(self):
        for width, height in SIZES:
            with self.subTest(width=width):
                self.w.resize(width, height)
                self.settle()
                self.w.motion_splitter.setSizes((320, 545))
                self.settle()
                self.assertGreaterEqual(self.w._motion_header_left.width(), 430)
                self.assert_header_controls_fit()
                self.assertEqual((self.w.width(), self.w.height()), (width, height))

    def test_status_and_configuration_notice_never_move_page_or_card(self):
        for width, height in SIZES:
            with self.subTest(width=width):
                self.w.resize(width, height)
                self.settle()
                baseline = self.stable_geometry()
                for phase, title in (("idle", "已失能"), ("accepted", "目标已受理"),
                                     ("locked", "保护锁定"), ("observed_reached", "回读位置已到目标")):
                    self.w.operation_panel.set_status({
                        "phase": phase, "title": title + " 很长的状态说明" * 8,
                        "reason": "具体原因说明" * 50, "next_step": "下一步建议" * 50,
                        "tone": "warning" if phase == "locked" else "info", "elapsed_s": 12.3,
                    }, {})
                    self.settle()
                    self.assertEqual(self.stable_geometry(), baseline)
                for phase in ("pending", "blocked", "invalid", "success", "error", "uncertain"):
                    self.c.notify_action("motor", "应用加速度", phase=phase,
                                         message="配置操作反馈很长的结果说明" * 50,
                                         next_step="处理建议" * 50)
                    self.settle()
                    self.assertEqual(self.stable_geometry(), baseline)
                    self.assert_anchor()
                self.w.action_feedback_notice.clear_event()
                self.settle()
                self.assertEqual(self.stable_geometry(), baseline)
        self.assertTrue(self.c._tx_queue.empty())

    def test_shared_card_repositions_to_each_top_level_page_without_new_outer_row(self):
        for size in ((1120, 690), (1440, 900)):
            self.w.resize(*size)
            self.settle()
            for index in (1, 2, 3, 5, 0):
                with self.subTest(size=size, index=index):
                    self.w.navigation.setCurrentRow(index)
                    self.settle()
                    self.assertEqual(self.w.pages.currentIndex(), index)
                    self.assert_anchor()
                    self.assert_shell_starts_below_serial()
            # Nested motor tabs do not each reserve another status header.
            self.assertNotIn(self.w.motor_tabs.widget(0), self.w._page_header_slots)
            self.assertNotIn(self.w.motor_tabs.widget(1), self.w._page_header_slots)
        self.assertTrue(self.c._tx_queue.empty())

    def test_rgb_navigation_returns_to_motion_anchor_and_keeps_one_shared_card(self):
        self.w.navigation.setCurrentRow(3)
        self.settle()
        self.w.navigation.setCurrentRow(4)
        self.settle()
        self.assertTrue(self.w.rgb_window.isVisible())
        self.assertEqual(self.w.pages.currentIndex(), 0)
        self.assertEqual(self.w.navigation.currentRow(), 0)
        self.assert_anchor()
        self.assertEqual(len(self.w.findChildren(type(self.w.operation_panel))), 1)
        self.assertTrue(self.c._tx_queue.empty())

    def test_motion_status_is_shown_once_in_topbar_and_header_only_keeps_conditions(self):
        panel = self.w.operation_panel
        title = panel.title_label
        for theme in ("dark", "light"):
            self.w.apply_theme(theme)
            for size in ((1120, 690), (1440, 900)):
                self.w.resize(*size)
                for index in (0, 2, 1, 3, 5, 0):
                    with self.subTest(theme=theme, size=size, page=index):
                        self.w.navigation.setCurrentRow(index)
                        self.settle()
                        self.assert_anchor()
                        self.assertIs(panel.title_label, title)
                        self.assertTrue(title.isVisible())
                        motion = index == 0
                        self.assertEqual(title.parentWidget() is self.w.caption_controls, motion)
                        self.assertEqual(self.w.motor_status.isVisible(), not motion)
                        self.assertEqual(panel.heading_widget.layout().indexOf(title) < 0, motion)
                        self.assertEqual(panel.body_widget.isVisible(), not motion)
                        self.assertEqual(panel.heading_widget.isVisible(), not motion)
                        self.assertTrue(panel.details_button.isVisible())
                        self.assertEqual(panel.details_button.iconSize().width(), 8)
                        if motion:
                            panel.set_status({
                                "phase": "observed_reached", "title": "MoveJ #9 · 回读位置已到目标",
                                "tone": "success", "reason": "回读已进入目标位置容差。",
                                "next_step": "可以继续编辑或发送下一目标。",
                            }, {})
                            self.settle()
                            self.assertEqual(title.text(), "MoveJ #9 · 回读位置已到目标")
                            self.assertEqual(title.displayedText(), title.fullText())
                            for label in (panel.reason_label, panel.next_step_label):
                                self.assertFalse(label.isVisible())
                            self.assertFalse(panel.elapsed_label.isVisible())
        self.assertTrue(self.c._tx_queue.empty())

    def test_header_only_keeps_conditions_after_theme_change_during_page_switch(self):
        for theme in ("dark", "light", "dark"):
            with self.subTest(theme=theme):
                self.w.navigation.setCurrentRow(5)
                self.settle()
                # Apply the theme before the queued motion header fit executes.
                self.w.navigation.setCurrentRow(0)
                self.w.apply_theme(theme)
                self.settle()
                panel = self.w.operation_panel
                self.assertIs(panel.title_label.parentWidget(), self.w.caption_controls)
                self.assertFalse(self.w.motor_status.isVisible())
                self.assertFalse(panel.body_widget.isVisible())
                self.assertFalse(panel.heading_widget.isVisible())
                self.assertTrue(panel.details_button.isVisible())
        self.assertTrue(self.c._tx_queue.empty())

    def test_details_overlay_stays_inside_workspace_and_stop_still_works_in_one_click(self):
        self.c.is_connected = True
        self.c.serial_port = FakePort()
        self.c._publish()
        self.settle()
        while not self.c._tx_queue.empty():
            self.c._tx_queue.get_nowait()  # Baseline RGB seed, never a device write.
        self.w.resize(1120, 690)
        self.settle()
        baseline = self.stable_geometry()
        self.w.operation_panel.details_button.click()
        self.settle()
        overlay = self.w.operation_panel.details_overlay
        self.assertTrue(overlay.isVisible())
        self.assertIs(overlay.parentWidget(), self.w.centralWidget())
        self.assertFalse(overlay.isWindow())
        self.assertTrue(self.w.centralWidget().rect().contains(overlay.geometry()))
        self.assertEqual(self.stable_geometry(), baseline)
        QTest.mouseClick(self.w.stop_button, Qt.MouseButton.LeftButton)
        self.settle()
        self.assertFalse(overlay.isVisible())
        commands = []
        while True:
            try:
                commands.append(self.c._tx_queue.get_nowait()[1])
            except queue.Empty:
                break
        self.assertEqual(commands, ["!STOP"])
        self.assertEqual(self.stable_geometry(), baseline)


if __name__ == "__main__":
    unittest.main()
