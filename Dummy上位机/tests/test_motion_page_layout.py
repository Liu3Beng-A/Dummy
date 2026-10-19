"""Workstation presentation must fit without changing targets or dispatching."""
from pathlib import Path
import unittest
from unittest.mock import patch

from PySide6.QtCore import QPoint, QRect, Qt
from PySide6.QtGui import QFontDatabase
from PySide6.QtTest import QTest

from tests import test_ui as base_ui
APPLICATION = base_ui.APPLICATION


class MotionPageLayoutTests(unittest.TestCase):
    setUp = base_ui.UiIntegrationTests.setUp
    tearDown = base_ui.UiIntegrationTests.tearDown
    commands = base_ui.UiIntegrationTests.commands
    connect_fake = base_ui.UiIntegrationTests.connect_fake

    @classmethod
    def setUpClass(cls):
        cls.font_ids = []
        for name in ("msyh.ttc", "msyhbd.ttc", "segoeui.ttf", "segoeuib.ttf"):
            path = Path("C:/Windows/Fonts") / name
            if path.exists():
                cls.font_ids.append(QFontDatabase.addApplicationFont(str(path)))

    @classmethod
    def tearDownClass(cls):
        for font in cls.font_ids:
            if font >= 0:
                QFontDatabase.removeApplicationFont(font)

    def settle(self):
        for _ in range(12):
            APPLICATION.processEvents()

    def test_both_themes_and_minimum_size_keep_all_seven_axes_and_actions_visible(self):
        w = self.window
        self.scheduler.run_ready()
        w.show()
        for theme in ("light", "dark"):
            w.apply_theme(theme)
            for size in ((1120, 690), (1440, 900), (1440, 1000), (1920, 1080)):
                with self.subTest(theme=theme, size=size):
                    w.resize(*size)
                    self.settle()
                    self.assertEqual((w.width(), w.height()), size)
                    viewport = w._motion_axis_scrolls[0].viewport()
                    for axis in w.joint_rows:
                        rect = QRect(axis.mapTo(viewport, QPoint(0, 0)), axis.size())
                        self.assertTrue(viewport.rect().contains(rect), (axis.spin.objectName(), rect, viewport.rect()))
                    for control in (w.stop_button, w.motion_enable_button, w.motion_disable_button,
                                    w.movej_send_button, w.movej_read_button,
                                    w.motion_hand_spin, w.motion_hand_drag, w.motion_hand_current,
                        w.port_combo, w.connect_button, w.motion_mode_button):
                        self.assertTrue(control.isVisible())
                        rect = QRect(control.mapTo(w, QPoint(0, 0)), control.size())
                        self.assertTrue(w.rect().contains(rect), control.objectName())
                    self.assertEqual(w.motion_controls_scroll.horizontalScrollBar().maximum(), 0)
                    self.assertEqual(w.operation_panel.height(), 48)
                    self.assertFalse(w.viewer_mode.isVisible())
                    self.assertFalse(w._motion_header_subtitle.isVisible())
                    mode, calibration = w.motion_mode_button, w.motion_calibration_button
                    self.assertIs(mode.parentWidget(), calibration.parentWidget())
                    self.assertGreater(mode.x(), calibration.geometry().right())
                    self.assertEqual(mode.y(), calibration.y())
                    self.assertEqual(mode.height(), calibration.height())
                    self.assertGreaterEqual(mode.width(), mode.minimumSizeHint().width())
                    self.assertFalse(w.motion_controls.isAncestorOf(mode))
                    self.assertFalse(w.mode_status.isVisible())
                    self.assertTrue(w.stop_button.icon().isNull())
                    self.assertEqual(w.stop_button.text(), "STOP")
                    w.motion_tabs.setCurrentIndex(1)
                    self.settle()
                    pose_viewport = w._motion_axis_scrolls[1].viewport()
                    for axis in w.pose_rows:
                        self.assertTrue(axis.isVisible())
                        rect = QRect(axis.mapTo(pose_viewport, QPoint()), axis.size())
                        self.assertTrue(pose_viewport.rect().contains(rect), (axis.spin.objectName(), rect, pose_viewport.rect()))
                    w.motion_tabs.setCurrentIndex(0)
                    self.settle()
        self.assertEqual(self.commands(), [])

    def test_resize_theme_and_points_disclosure_preserve_edit_and_pending_drag(self):
        w = self.window
        self.scheduler.run_ready()
        w.show()
        w.joint_spins[0].setValue(25)
        baseline = [field.value() for field in w.joint_spins]
        pending = dict(w._drag_pending)
        for theme in ("dark", "light"):
            w.apply_theme(theme)
            for size in ((1120, 690), (1440, 900), (1120, 690)):
                w.resize(*size)
                self.settle()
                if w.quick_points_toggle.isVisible():
                    w.quick_points_toggle.click()
                    self.settle()
                self.assertEqual([field.value() for field in w.joint_spins], baseline)
                self.assertEqual(w._drag_pending, pending)
        self.assertEqual(self.commands(), [])

    def test_eight_points_are_below_gripper_and_scroll_without_losing_slots(self):
        w = self.window
        self.scheduler.run_ready()
        w.show()
        for theme in ("dark", "light"):
            w.apply_theme(theme)
            for size in ((1120,690),(1440,900),(1440,1000),(1920,1440),(3072,1726)):
                w.resize(*size)
                self.settle()
                self.assertTrue(w.motion_controls.isAncestorOf(w.quick_points_panel))
                self.assertFalse(w.motion_layout.scene_column.isAncestorOf(w.quick_points_panel))
                self.assertGreater(w.quick_points_panel.y(),w.motion_gripper_group.geometry().bottom())
                if w.quick_points_toggle.isVisible() and not w.quick_points_scroll.isVisible():
                    w.quick_points_toggle.click()
                    self.settle()
                panel = w.quick_points_panel
                self.assertLessEqual(panel.height(),panel.sizeHint().height()+2)
                header = panel.layout().itemAt(0).layout().geometry()
                self.assertLessEqual(header.top(),panel.layout().contentsMargins().top()+2)
                self.assertLessEqual(w.quick_points_scroll.y()-header.bottom(),10)
                grid = w.motion_layout.points_grid
                self.assertEqual([grid.getItemPosition(grid.indexOf(button))[:2]
                                  for button in w.quick_point_buttons],
                                 [(index//2,index%2) for index in range(8)])
                for control in (w.quick_point_buttons[0],w.quick_point_buttons[7]):
                    w.motion_controls_scroll.ensureWidgetVisible(w.quick_points_scroll)
                    w.quick_points_scroll.ensureWidgetVisible(control)
                    self.settle()
                    viewport = w.quick_points_scroll.viewport()
                    rect = QRect(control.mapTo(viewport,QPoint()),control.size())
                    self.assertTrue(viewport.rect().contains(rect), (size,rect,viewport.rect()))
                if w.quick_points_toggle.isVisible():
                    w.quick_points_toggle.click()
                w.motion_controls_scroll.verticalScrollBar().setValue(0)
        self.assertEqual(self.commands(), [])

    def test_canvas_covers_caption_strip_and_hides_actual_status_line(self):
        w = self.window
        self.scheduler.run_ready()
        w.show()
        scene_text = "本地 URDF 与 12 个网格加载完成"
        actual_text = "实际位置未回读 / 已过期 · 夹爪显示目标开度"
        w.viewer.status_changed.emit(scene_text)
        w.actual_status.setText(actual_text)
        for theme in ("dark", "light"):
            w.apply_theme(theme)
            for size in ((1120,690),(1440,900)):
                w.resize(*size)
                self.settle()
                viewport = w.motion_layout.scene_viewport
                self.assertEqual(QRect(w.viewer.mapTo(viewport,QPoint()),w.viewer.size()),viewport.rect())
                self.assertFalse(hasattr(w.motion_layout,"scene_caption_overlay"))
                self.assertFalse(hasattr(w.motion_layout,"scene_corner_overlay"))
                self.assertEqual(w.scene_status.text(),scene_text)
                self.assertFalse(w.scene_status.isVisible())
                self.assertFalse(w.actual_status.isVisible())
                self.assertEqual(w.actual_status.text(),actual_text)
                hit = viewport.childAt(QPoint(viewport.width()//2,viewport.height()-10))
                self.assertIsNotNone(hit)
                self.assertTrue(w.viewer is hit or w.viewer.isAncestorOf(hit))
        self.assertEqual(self.commands(), [])

    def test_mode_menu_shows_all_choices_without_query_or_apply_controls(self):
        w = self.window
        self.scheduler.run_ready()
        w.show()
        self.settle()
        w.motion_mode_button.click()
        self.settle()
        self.assertTrue(w.mode_panel.isVisible())
        self.assertFalse(w.mode_panel.isWindow())
        self.assertTrue(w.centralWidget().rect().contains(w.mode_panel.geometry()))
        self.assertEqual([control.text() for control in w.mode_choice_buttons],
                         [w.mode_combo.itemText(i) for i in range(4)])
        self.assertTrue(all(control.isVisible() for control in w.mode_choice_buttons))
        for control in (w.mode_combo,w.mode_set_button,w.mode_query_button,w.mode_status):
            self.assertFalse(control.isVisible())
        self.assertEqual(self.commands(), [])

    def test_stop_is_one_click_with_mode_panel_open(self):
        w = self.window
        self.connect_fake()
        self.commands()
        w.show()
        self.settle()
        w.motion_mode_button.click()
        self.settle()
        QTest.mouseClick(w.stop_button, Qt.MouseButton.LeftButton)
        self.settle()
        self.assertFalse(w.mode_panel.isVisible())
        self.assertEqual(self.commands(), ["!STOP"])

    def test_mode_choice_sends_once_without_query_and_waits_for_confirmed_state(self):
        w, c = self.window, self.controller
        self.connect_fake()
        w.rgb_preview_source.setCurrentIndex(0)
        c.current_cmdmode = 2
        c._publish()
        w.show()
        self.settle()
        self.commands()
        self.assertEqual(w.motion_mode_button.text(), "模式：INT 打断 ▾")
        w.motion_mode_button.click()
        self.settle()
        self.assertTrue(w.mode_panel.isVisible())
        QTest.mouseClick(w.mode_choice_buttons[2], Qt.MouseButton.LeftButton)
        self.assertFalse(w.mode_panel.isVisible())
        self.assertEqual(self.commands(), ["#CMDMODE 3"])
        self.assertEqual(c.current_cmdmode, 2)
        self.assertEqual(w.motion_mode_button.text(), "模式：INT 打断 ▾")
        c._on_written("#CMDMODE 3", c._connection_id)
        c._on_transport_line("ok CMDMODE mode=3", c._connection_id)
        self.settle()
        self.assertEqual(w.motion_mode_button.text(), "模式：TRJ 连续 ▾")
        self.assertEqual(self.commands(), [])
        w.motion_mode_button.click()
        self.settle()
        self.assertTrue(w.mode_choice_buttons[2].property("currentMode"))
        QTest.mouseClick(w.mode_choice_buttons[0], Qt.MouseButton.LeftButton)
        self.assertEqual(self.commands(), ["#CMDMODE 1"])
        self.assertIsNone(c._mode_request)
        c._on_written("#CMDMODE 1", c._connection_id)
        c._on_transport_line("ok CMDMODE mode=1", c._connection_id)
        self.settle()
        self.assertEqual(w.motion_mode_button.text(), "模式：SEQ 顺序 ▾")
        c.is_connected = False
        c._publish()
        self.settle()
        self.assertEqual(w.motion_mode_button.text(), "模式：未确认 ▾")
        self.assertEqual(self.commands(), [])

    def test_outside_click_escape_and_disabled_setting_never_switch_modes(self):
        w, c = self.window, self.controller
        self.connect_fake()
        w.rgb_preview_source.setCurrentIndex(0)
        c.current_cmdmode = 2
        c._publish()
        w.show()
        self.settle()
        self.commands()
        for theme in ("light","dark"):
            w.apply_theme(theme)
            for size in ((1120,690),(1440,900)):
                w.resize(*size)
                self.settle()
                w.motion_mode_button.click()
                self.settle()
                self.assertTrue(w.mode_panel.isVisible())
                QTest.mouseMove(w.mode_choice_buttons[3])
                self.assertEqual(c.current_cmdmode,2)
                QTest.mouseClick(w._motion_header_title,Qt.MouseButton.LeftButton)
                self.assertFalse(w.mode_panel.isVisible())
                self.assertEqual(w.motion_mode_button.text(),"模式：INT 打断 ▾")
                self.assertEqual(self.commands(),[])
                w.motion_mode_button.click()
                self.settle()
                QTest.keyClick(w,Qt.Key.Key_Escape)
                self.assertFalse(w.mode_panel.isVisible())
                self.assertEqual(self.commands(),[])
        w.mode_set_button.setEnabled(False)
        w.motion_mode_button.click()
        self.assertFalse(w.mode_panel.isVisible())
        w.mode_choice_buttons[0].click()
        self.assertEqual(c.current_cmdmode,2)
        self.assertEqual(self.commands(),[])

    def test_points_disclosure_keeps_gamepad_inputs_hidden_and_restores_ui_choice(self):
        w = self.window
        self.scheduler.run_ready()
        w.resize(1120, 690)
        w.show()
        self.settle()
        self.assertFalse(w.quick_points_scroll.isVisible())
        w.quick_points_toggle.click()
        self.settle()
        self.assertTrue(w.quick_points_scroll.isVisible())
        w.input_mode.setCurrentIndex(1)
        self.settle()
        self.assertFalse(w.quick_points_scroll.isVisible())
        self.assertFalse(w.quick_points_toggle.isVisible())
        w.input_mode.setCurrentIndex(0)
        self.settle()
        self.assertTrue(w.quick_points_scroll.isVisible())
        self.assertEqual(self.commands(), [])

    def test_split_power_buttons_wait_for_device_confirmation_in_both_directions(self):
        w, c = self.window, self.controller
        self.connect_fake()
        self.commands()
        w.show()
        c._on_transport_line("Disabled ok", c._connection_id)
        self.settle()
        self.commands()  # Drain the pre-existing connection RGB seed.
        self.assertEqual(w.motion_enable_button.text(), "使能")
        self.assertEqual(w.motion_disable_button.text(), "失能")
        self.assertTrue(w.motion_enable_button.isVisible())
        self.assertTrue(w.motion_disable_button.isVisible())
        self.assertTrue(all(not control.isVisible() for control in w.motion_system_buttons[:2]))
        self.assertTrue(w.motion_enable_button.isEnabled())
        self.assertFalse(w.motion_disable_button.isEnabled())
        QTest.mouseClick(w.motion_disable_button, Qt.MouseButton.LeftButton)
        self.assertEqual(self.commands(), [])
        QTest.mouseClick(w.motion_enable_button, Qt.MouseButton.LeftButton)
        self.assertEqual(self.commands(), ["!START"])
        self.assertIs(c.state()["motor_enabled"], False)
        self.assertEqual(w.motion_enable_button.text(), "使能中")
        self.assertFalse(w.motion_enable_button.isEnabled())
        self.assertFalse(w.motion_disable_button.isEnabled())
        QTest.mouseClick(w.motion_enable_button, Qt.MouseButton.LeftButton)
        self.assertEqual(self.commands(), [])
        self.assertTrue(c._can_write(c._tx_generation, "!START"))
        c._on_written("!START", c._connection_id)
        c._on_transport_line("Start pending", c._connection_id)
        self.settle()
        self.assertEqual(w.motion_enable_button.text(), "使能中")
        self.assertFalse(w.motion_disable_button.isEnabled())
        c._on_transport_line("Started ok", c._connection_id)
        self.settle()
        self.assertEqual(w.motion_enable_button.text(), "使能")
        self.assertEqual(w.motion_disable_button.text(), "失能")
        self.assertIs(c.state()["motor_enabled"], True)
        self.assertFalse(w.motion_enable_button.isEnabled())
        self.assertTrue(w.motion_disable_button.isEnabled())
        # START's existing mode query is independent of the presentation adapter.
        self.commands()
        QTest.mouseClick(w.motion_disable_button, Qt.MouseButton.LeftButton)
        self.assertEqual(self.commands(), ["!DISABLE"])
        self.assertIs(c.state()["motor_enabled"], True)
        self.assertEqual(w.motion_disable_button.text(), "失能中")
        self.assertFalse(w.motion_disable_button.isEnabled())
        self.assertFalse(w.motion_enable_button.isEnabled())
        self.assertTrue(c._can_write(c._tx_generation, "!DISABLE"))
        c._on_written("!DISABLE", c._connection_id)
        c._on_transport_line("Disable pending", c._connection_id)
        self.settle()
        self.assertEqual(w.motion_disable_button.text(), "失能中")
        self.assertFalse(w.motion_enable_button.isEnabled())
        c._on_transport_line("Disabled ok", c._connection_id)
        self.settle()
        self.assertEqual(w.motion_enable_button.text(), "使能")
        self.assertEqual(w.motion_disable_button.text(), "失能")
        self.assertIs(c.state()["motor_enabled"], False)
        self.assertTrue(w.motion_enable_button.isEnabled())
        self.assertFalse(w.motion_disable_button.isEnabled())
        self.assertEqual(self.commands(), [])

    def test_visible_navigation_retains_active_item_and_rgb_returns_to_motion(self):
        w = self.window
        self.scheduler.run_ready()
        w.show()
        self.settle()
        for index in (0, 0, 1, 2, 3, 5, 4):
            w.sidebar_buttons[index].click()
            self.settle()
            expected = 0 if index == 4 else index
            self.assertEqual(w.pages.currentIndex(), expected)
            self.assertEqual([i for i, b in enumerate(w.sidebar_buttons) if b.isChecked()], [expected])
        self.assertEqual(self.commands(), [])

    def test_calibration_is_direct_and_keeps_original_callback_and_gate(self):
        w = self.window
        self.scheduler.run_ready()
        w.show()
        self.settle()
        self.assertIsNone(w.motion_calibration_button.menu())
        self.assertEqual(self.commands(), [])
        with patch.object(self.controller, "_cmd_calibration", return_value=True) as calibration:
            QTest.mouseClick(w.motion_calibration_button, Qt.MouseButton.LeftButton)
            calibration.assert_called_once_with()
        w.motion_system_buttons[4].setEnabled(False)
        self.assertFalse(w.motion_calibration_button.isEnabled())
        with patch.object(self.controller, "_cmd_calibration", return_value=True) as calibration:
            QTest.mouseClick(w.motion_calibration_button, Qt.MouseButton.LeftButton)
            calibration.assert_not_called()
        for theme in ("dark", "light", "dark"):
            w.apply_theme(theme)
            self.settle()
            self.assertTrue(w.motion_system_buttons[2].icon().isNull())
            self.assertTrue(w.motion_system_buttons[3].icon().isNull())
            self.assertFalse(w.motion_enable_button.icon().isNull())
        self.assertEqual(self.commands(), [])

    def test_explicit_disable_is_one_click_through_conditions_overlay(self):
        w, c = self.window, self.controller
        self.connect_fake()
        c._motor_state_known = c._motor_enabled = True
        c._publish()
        w.show()
        self.settle()
        self.commands()
        self.assertEqual(w.motion_disable_button.text(), "失能")
        w.operation_panel.details_button.click()
        self.settle()
        QTest.mouseClick(w.motion_disable_button, Qt.MouseButton.LeftButton)
        self.settle()
        self.assertFalse(w.operation_panel.details_overlay.isVisible())
        self.assertEqual(self.commands(), ["!DISABLE"])

    def test_unknown_state_still_allows_explicit_disable_during_start_confirmation(self):
        w, c = self.window, self.controller
        self.connect_fake()
        w.show()
        self.settle()
        self.commands()
        self.assertIsNone(c.state()["motor_enabled"])
        self.assertTrue(w.motion_enable_button.isEnabled())
        self.assertTrue(w.motion_disable_button.isEnabled())
        w.motion_enable_button.click()
        self.assertEqual(self.commands(), ["!START"])
        self.assertIsNone(c.state()["motor_enabled"])
        self.assertTrue(w.motion_disable_button.isEnabled())
        w.motion_disable_button.click()
        self.assertEqual(self.commands(), ["!DISABLE"])
        self.assertIsNone(c.state()["motor_enabled"])
        self.assertEqual(w.motion_disable_button.text(), "失能中")


if __name__ == "__main__":
    unittest.main()
