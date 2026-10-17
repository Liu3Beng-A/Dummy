"""Stable serial-toolbar slots across device states; no device is opened."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from PySide6.QtCore import QEvent, Qt
from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtWidgets import QFrame

from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow
from tests.test_controller import ManualScheduler
from tests.test_gamepad_ui import FakeReader
from tests.test_ui import APPLICATION, FakeView


class TopBarLayoutTests(unittest.TestCase):
    WIDTHS = (1120, 1440, 2502)
    THEMES = ("light", "dark")

    def setUp(self):
        base = Path(__file__).resolve().parents[1] / "build" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=base)
        self.previous_style = APPLICATION.styleSheet()
        self.previous_font = APPLICATION.font()
        self.font_ids = []
        # The offscreen Windows plugin does not enumerate system fonts. Load
        # real UI fonts temporarily so reserve-width tests use actual metrics.
        for name in ("msyh.ttc", "msyhbd.ttc", "segoeui.ttf", "segoeuib.ttf", "seguisym.ttf"):
            font_path = Path("C:/Windows/Fonts") / name
            if font_path.is_file():
                font_id = QFontDatabase.addApplicationFont(str(font_path))
                if font_id >= 0:
                    self.font_ids.append(font_id)
        if self.font_ids:
            APPLICATION.setFont(QFont("Microsoft YaHei", 10))
        self.store = AppStore(self.directory.name)
        self.store.update_settings(auto_poll=False)
        self.scheduler = ManualScheduler()
        self.controller = RobotController(scheduler=self.scheduler,
                                          serial_factory=lambda *_a, **_k: self.fail("must stay offline"))
        self.viewer_patch = patch("app.ui.main_window.RobotView", FakeView)
        self.viewer_patch.start()
        self.window = MainWindow(self.controller, self.store, gamepad_reader=FakeReader())
        self.window._operation_timer.stop()
        self.scheduler.run_ready()
        self.window.show()
        self.topbar = self.window.findChild(QFrame, "TopBar")
        self.controls = [item.widget() for index in range(self.topbar.layout().count())
                         if (item := self.topbar.layout().itemAt(index)).widget() is not None]
        self.settle()

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        APPLICATION.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        APPLICATION.processEvents()
        APPLICATION.setStyleSheet(self.previous_style)
        APPLICATION.setFont(self.previous_font)
        for font_id in reversed(self.font_ids):
            QFontDatabase.removeApplicationFont(font_id)
        self.store.close()
        self.viewer_patch.stop()
        self.directory.cleanup()

    @staticmethod
    def settle():
        for _ in range(6):
            APPLICATION.processEvents()

    @staticmethod
    def rect(widget):
        return widget.geometry().getRect()

    def geometry(self):
        return {
            "topbar": self.rect(self.topbar),
            "controls": tuple(self.rect(widget) for widget in self.controls),
            "pages": self.rect(self.window.pages),
            "viewer": self.rect(self.window.viewer),
            "controls_scroll": self.rect(self.window.motion_controls_scroll),
            "operation_card": self.rect(self.window.operation_panel),
            "window_size": (self.window.width(), self.window.height()),
        }

    def assert_controls_fit_without_overlap(self):
        for index, control in enumerate(self.controls):
            self.assertTrue(self.topbar.rect().contains(control.geometry()),
                            (control.objectName(), control.geometry(), self.topbar.rect()))
            for other in self.controls[index + 1:]:
                self.assertFalse(control.geometry().intersects(other.geometry()),
                                 (control.objectName(), control.geometry(),
                                  other.objectName(), other.geometry()))

    @staticmethod
    def state(connected=True, mode=2, motor=None, *, locked=False, calibrating=False,
              kind="connection", phase="idle"):
        return {"connected": connected, "cmdmode": mode, "motor_enabled": motor,
                "stop_locked": locked, "calibration_busy": calibrating,
                "operation": {"kind": kind, "phase": phase, "title": "状态说明",
                              "reason": "当前状态的原因", "next_step": "下一步操作",
                              "tone": "info", "elapsed_s": 1.0}}

    def states(self):
        # Every mode at both connection lengths, including the longest mode 7.
        for connected in (False, True):
            for index, mode in enumerate((None, 1, 2, 3, 4, 5, 6, 7)):
                yield self.state(connected, mode, (None, False, True)[index % 3])
        for motor in (None, False, True):
            yield self.state(mode=7, motor=motor)
        yield self.state(mode=7, motor=False, calibrating=True)
        yield self.state(mode=7, motor=False, locked=True)
        yield self.state(mode=7, motor=False, locked=True, kind="protection", phase="locked")
        for kind, phase in (("start", "queued"), ("start", "waiting"), ("start", "uncertain"),
                            ("stop", "queued"), ("stop", "waiting"), ("stop", "locked"),
                            ("disable", "waiting"), ("motion", "accepted"),
                            ("motion", "executing"), ("motion", "observed_reached")):
            yield self.state(mode=7, motor=True, kind=kind, phase=phase)

    def test_all_topbar_slots_and_workspace_remain_fixed_for_every_status_length(self):
        self.assertGreaterEqual(len(self.controls), 10)
        for theme in self.THEMES:
            self.window.apply_theme(theme)
            for width in self.WIDTHS:
                with self.subTest(theme=theme, width=width):
                    self.window.resize(width, 900)
                    self.window._update_state(self.state(connected=False, mode=None))
                    self.settle()
                    before = self.geometry()
                    self.assertEqual(before["window_size"], (width, 900))
                    self.assert_controls_fit_without_overlap()
                    for state in self.states():
                        self.window._update_state(state)
                        self.settle()
                        self.assertEqual(self.geometry(), before,
                                         (theme, width, state, self.geometry(), before))
                        self.assert_controls_fit_without_overlap()
                    self.assertLessEqual(self.window.minimumWidth(), 1120)
                    self.assertIsNone(self.controller.serial_port)
                    self.assertFalse(self.controller.is_connected)
                    self.assertTrue(self.controller._tx_queue.empty())

    def test_long_mode_and_motor_text_keep_full_tooltip_and_accessibility_without_resizing(self):
        for theme in self.THEMES:
            self.window.apply_theme(theme)
            for width in self.WIDTHS:
                with self.subTest(theme=theme, width=width):
                    self.window.resize(width, 900)
                    self.window._update_state(self.state(mode=7, kind="start", phase="uncertain"))
                    self.settle()
                    expected_mode = "模式 7 · 通信诊断 / 待退出 · 普通运动不可用"
                    self.assertEqual(self.window.mode_status.text(), expected_mode)
                    self.assertIn(expected_mode, self.window.mode_status.toolTip())
                    self.assertIn(expected_mode, self.window.mode_status.accessibleDescription())
                    self.assertIn(self.window.motor_status.text(), self.window.motor_status.toolTip())
                    self.assertIn(self.window.motor_status.text(), self.window.motor_status.accessibleDescription())
                    before = self.geometry()
                    long_text = "<b>完整设备状态</b>" + "非常长的状态说明" * 100
                    self.window.mode_status.setText(long_text)
                    self.window.motor_status.setText(long_text)
                    self.window.connection_status.setText(long_text)
                    self.settle()
                    self.assertEqual(self.geometry(), before)
                    for label in (self.window.mode_status, self.window.motor_status, self.window.connection_status):
                        self.assertEqual(label.text(), long_text)
                        self.assertEqual(label.textFormat(), Qt.TextFormat.PlainText)
                        self.assertIn(long_text, label.accessibleDescription())
                        self.assertIn("&lt;b&gt;", label.toolTip())
                        self.assertNotIn("<b>", label.toolTip())
                        self.assertLess(len(label.displayedText()), len(label.fullText()))

    def test_reserved_slots_may_resize_with_window_but_not_with_short_status_text(self):
        for theme in self.THEMES:
            self.window.apply_theme(theme)
            for width in (2502, 1120, 1440, 1120):
                self.window.resize(width, 900)
                self.window._update_state(self.state(mode=7, motor=False, locked=True))
                self.settle()
                before = self.geometry()
                self.window._update_state(self.state(connected=False, mode=None, motor=None))
                self.settle()
                self.assertEqual(self.geometry(), before)
                self.assertEqual(self.window.width(), width)
                self.assert_controls_fit_without_overlap()


if __name__ == "__main__":
    unittest.main()
