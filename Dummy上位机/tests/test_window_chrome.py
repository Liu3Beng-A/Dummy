"""Custom caption must preserve normal window actions and close rejection."""
import unittest
from unittest.mock import patch
from PySide6.QtCore import QEvent, QPoint, QPointF, Qt
from PySide6.QtGui import QColor, QIcon, QMouseEvent, QPixmap
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QMainWindow, QWidget
from tests.test_ui import APPLICATION
from app.ui.window_chrome import apply_window_theme, install_custom_titlebar, _edge_at, _EDGE_HITS
from tests import test_gamepad_ui as gp_tests


class GuardedWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.allow_close = False
        self.close_requests = 0

    def closeEvent(self, event):
        self.close_requests += 1
        event.setAccepted(self.allow_close)


class WindowChromeTests(unittest.TestCase):
    def setUp(self):
        self.window = GuardedWindow()
        self.window.setWindowTitle("Dummy Robot · 机械臂控制台")
        self.window.setCentralWidget(QWidget())
        self.window.setMinimumSize(1120, 690)
        self.window.title_bar = install_custom_titlebar(self.window)
        self.window.resize(1120, 690)
        self.window.show()
        self.settle()

    def tearDown(self):
        self.window.allow_close = True
        self.window.close()
        self.window.deleteLater()
        self.settle()

    @staticmethod
    def settle():
        for _ in range(8):
            APPLICATION.processEvents()

    def test_themes_and_small_window_keep_left_title_and_all_actions(self):
        w, bar = self.window, self.window.title_bar
        pixmap = QPixmap(16, 16)
        pixmap.fill(QColor("#627ff0"))
        w.setWindowIcon(QIcon(pixmap))
        original_icon = w.windowIcon().cacheKey()
        for theme in ("dark", "light"):
            apply_window_theme(theme)
            for size in ((1120,690),(1440,900)):
                w.resize(*size)
                self.settle()
                self.assertEqual(bar.title.x(),8)
                self.assertEqual(bar.height(),28)
                self.assertEqual(bar.width(),w.width())
                self.assertEqual(bar.theme,theme)
                self.assertEqual(bar.grab().toImage().pixelColor(400,14).name(),
                                 "#191c23" if theme=="dark" else "#ffffff")
                self.assertEqual(w.centralWidget().y(),28)
                self.assertTrue(w.mask().isEmpty())
                for button in (bar.minimize_button,bar.maximize_button,bar.close_button):
                    self.assertTrue(button.isVisible())
                    self.assertTrue(bar.rect().contains(button.geometry()))
                self.assertEqual(w.windowIcon().cacheKey(),original_icon)
        w.setWindowTitle("新的窗口标题")
        self.assertEqual(bar.title.text(),"新的窗口标题")

    def test_double_click_restore_and_buttons_use_window_state(self):
        w, bar = self.window, self.window.title_bar
        QTest.mouseDClick(bar,Qt.MouseButton.LeftButton,pos=QPoint(300,14))
        self.settle()
        self.assertTrue(w.isMaximized())
        self.assertTrue(w.mask().isEmpty())
        self.assertTrue(all(not grip.isVisible() for grip in bar.resize_edges))
        self.assertEqual(bar.maximize_button.accessibleName(),"还原")
        bar.maximize_button.click()
        self.settle()
        self.assertFalse(w.isMaximized())
        self.assertTrue(w.mask().isEmpty())
        self.assertTrue(all(grip.isVisible() for grip in bar.resize_edges))
        bar.minimize_button.click()
        self.settle()
        self.assertTrue(w.isMinimized())
        w.showNormal()
        self.settle()
        self.assertFalse(w.isMinimized())

    def test_close_button_honors_existing_close_event_rejection(self):
        w, bar = self.window, self.window.title_bar
        bar.close_button.click()
        self.settle()
        self.assertEqual(w.close_requests,1)
        self.assertTrue(w.isVisible())
        w.allow_close = True
        bar.close_button.click()
        self.settle()
        self.assertEqual(w.close_requests,2)
        self.assertFalse(w.isVisible())

    def test_resize_hit_boundaries_cover_edges_and_corners_only(self):
        size=self.window.size()
        for point,hit in ((QPoint(1,300),10),(QPoint(1119,300),11),
                          (QPoint(300,1),12),(QPoint(1,1),13),
                          (QPoint(1119,1),14),(QPoint(300,689),15),
                          (QPoint(1,689),16),(QPoint(1119,689),17)):
            self.assertEqual(_EDGE_HITS[_edge_at(point,size)],hit)
        self.assertFalse(_edge_at(QPoint(300,300),size))

    def test_all_eight_edges_have_cursors_and_drag_resize_without_changing_opposite_edges(self):
        w = self.window
        self.assertEqual(len(w.title_bar.resize_edges), 8)
        for grip in w.title_bar.resize_edges:
            with self.subTest(edges=grip.edges):
                w.setGeometry(80,80,1240,790)
                self.settle()
                before = w.geometry()
                self.assertTrue(grip.isVisible())
                self.assertTrue(w.rect().contains(grip.geometry()))
                self.assertNotEqual(grip.cursor().shape(), Qt.CursorShape.ArrowCursor)
                dx = -20 if grip.edges & Qt.Edge.LeftEdge else 20 if grip.edges & Qt.Edge.RightEdge else 0
                dy = -15 if grip.edges & Qt.Edge.TopEdge else 15 if grip.edges & Qt.Edge.BottomEdge else 0
                point = grip.rect().center()
                origin = grip.mapToGlobal(point)
                with patch.object(grip, '_start_system_resize', return_value=False):
                    QTest.mousePress(grip, Qt.MouseButton.LeftButton, pos=point)
                    event = QMouseEvent(QEvent.Type.MouseMove, QPointF(point+QPoint(dx,dy)),
                        QPointF(origin+QPoint(dx,dy)), Qt.MouseButton.NoButton,
                        Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier)
                    APPLICATION.sendEvent(grip, event)
                    QTest.mouseRelease(grip, Qt.MouseButton.LeftButton, pos=point)
                after = w.geometry()
                self.assertEqual(after.width(), before.width()+abs(dx))
                self.assertEqual(after.height(), before.height()+abs(dy))
                if grip.edges & Qt.Edge.LeftEdge:
                    self.assertEqual(after.right(),before.right())
                if grip.edges & Qt.Edge.TopEdge:
                    self.assertEqual(after.bottom(),before.bottom())
        grip = next(g for g in w.title_bar.resize_edges if g.edges == (Qt.Edge.RightEdge|Qt.Edge.BottomEdge))
        point = grip.rect().center()
        origin = grip.mapToGlobal(point)
        with patch.object(grip, '_start_system_resize', return_value=False):
            QTest.mousePress(grip, Qt.MouseButton.LeftButton, pos=point)
            event = QMouseEvent(QEvent.Type.MouseMove, QPointF(point-QPoint(1000,1000)),
                QPointF(origin-QPoint(1000,1000)), Qt.MouseButton.NoButton,
                Qt.MouseButton.LeftButton, Qt.KeyboardModifier.NoModifier)
            APPLICATION.sendEvent(grip,event)
            QTest.mouseRelease(grip, Qt.MouseButton.LeftButton, pos=point)
        self.assertEqual((w.width(),w.height()),(1120,690))


class CaptionCloseGateTests(unittest.TestCase):
    setUp = gp_tests.GamepadUiTests.setUp
    tearDown = gp_tests.GamepadUiTests.tearDown
    claim = gp_tests.GamepadUiTests.claim

    def test_title_close_waits_for_original_gamepad_release_confirmation(self):
        self.controller.is_connected = True
        self.controller.serial_port = gp_tests.FakePort()
        self.claim()
        self.window.title_bar.close_button.click()
        self.assertTrue(self.window._gamepad_close_pending)
        self.assertTrue(self.session.release_request_stop)
        self.assertFalse(self.window._closed)
        self.assertFalse(self.window.title_bar.close_button.isEnabled())
        self.window.title_bar.close_button.click()
        self.assertEqual(self.session.calls.count("release"),1)
        self.session.confirm_release()
        APPLICATION.processEvents()
        self.assertTrue(self.window._closed)
        self.assertFalse(self.controller.is_connected)
        self.assertEqual(self.reader.calls.count("close"),1)
