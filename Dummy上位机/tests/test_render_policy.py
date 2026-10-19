"""Presentation throttling must never change controller/feedback state."""
import ctypes
from ctypes import wintypes
import json
import sys
import unittest
from unittest.mock import Mock, patch

from PySide6.QtCore import Qt
from tests.test_ui import APPLICATION
from app.visualization.robot_view import RobotView
from tests.test_window_chrome import GuardedWindow
from app.ui.window_chrome import _WindowsMainFrame, install_custom_titlebar


class PresentationTests(unittest.TestCase):
    def setUp(self):
        with patch('app.visualization.robot_view.AssetServer', side_effect=RuntimeError('offline')):
            self.view = RobotView()
        # Only the WebChannel boundary is faked. Real QWidget visibility and
        # application-state events exercise the production presentation policy.
        self.view.web_view = Mock()
        self.view._ready = True
        self.states, self.frames, self.modes, self.messages = [], [], [], []
        self.view._bridge.state_changed.connect(lambda s: self.states.append(json.loads(s)))
        self.view._bridge.rgb_changed.connect(lambda s: self.frames.append(json.loads(s)))
        self.view._bridge.presentation_changed.connect(self.modes.append)
        self.view._bridge.model_status_changed.connect(self.messages.append)

    def tearDown(self):
        self.view.web_view = None
        self.view.cleanup()
        self.view.close()
        self.view.deleteLater()
        APPLICATION.processEvents()

    def settle(self):
        for _ in range(6):
            APPLICATION.processEvents()

    def test_hidden_updates_are_cached_and_resume_delivers_only_the_latest_snapshot(self):
        v = self.view
        v._sync_presentation(force=True)
        self.assertEqual(self.modes[-1], 'hidden')
        for i in range(100):
            v.set_actual_pose([i, -75, 180, 0, 0, 0, 0])
            v.set_rgb_frame([(i, 0, 0)] * 24, light_on=True)
        v.status_changed.emit('模型已加载')
        self.assertEqual(self.states, [])
        self.assertEqual(self.frames, [])
        camera = v._state['cameraSeq']
        with patch('app.visualization.robot_view.QApplication.applicationState',
                   return_value=Qt.ApplicationState.ApplicationActive):
            v.show()
            self.settle()
        self.assertEqual(self.modes[-1], 'active')
        self.assertEqual(len(self.states), 1)
        self.assertEqual(self.states[-1], v._state)
        self.assertEqual(self.frames[-1]['colors'], [[99, 0, 0]] * 24)
        self.assertEqual(v._actual_pose[0], 99)
        self.assertEqual(v._state['cameraSeq'], camera)
        self.assertEqual(self.messages[-1], '模型已加载')

    def test_background_visible_and_minimized_states_remain_distinct(self):
        v = self.view
        v.show()
        self.settle()
        with patch('app.visualization.robot_view.QApplication.applicationState',
                   return_value=Qt.ApplicationState.ApplicationInactive):
            v._sync_presentation(force=True)
        self.assertEqual(self.modes[-1], 'background')
        v.showMinimized()
        self.settle()
        self.assertEqual(self.modes[-1], 'hidden')
        v.showNormal()
        with patch('app.visualization.robot_view.QApplication.applicationState',
                   return_value=Qt.ApplicationState.ApplicationActive):
            self.settle()
            v._sync_presentation(force=True)
        self.assertEqual(self.modes[-1], 'active')

    def test_renderer_diagnostics_do_not_change_feedback_and_ignore_invalid_or_closed_events(self):
        events = []
        self.view.diagnostic_event.connect(events.append)
        snapshot = dict(self.view._state)
        event = {'event': 'ready', 'renderer': 'offline renderer', 'devicePixelRatio': 1.25}
        self.view._bridge.viewerDiagnostic(json.dumps(event))
        for payload in ('invalid', '[]', 'null'):
            self.view._bridge.viewerDiagnostic(payload)
        self.view._closed = True
        self.view._bridge.viewerDiagnostic(json.dumps(event))
        self.view._closed = False
        self.assertEqual(events, [event])
        self.assertEqual(self.view._state, snapshot)
        self.assertEqual(self.states, [])
        self.assertEqual(self.frames, [])


@unittest.skipUnless(sys.platform == 'win32', 'Win32 caption event filter')
class CaptionRepaintTests(unittest.TestCase):
    def test_mouse_moves_repaint_only_when_native_hover_state_changes(self):
        w = GuardedWindow()
        w.title_bar = install_custom_titlebar(w)
        frame = _WindowsMainFrame(w.title_bar)
        frame.handle = 123
        frame.track_mouse = Mock()
        message = wintypes.MSG()
        message.hWnd = frame.handle
        button = w.title_bar.maximize_button
        with patch.object(button, 'update') as update:
            for code, param, count in ((0x200, 0, 0), (0xa0, 9, 1), (0xa0, 9, 1), (0x2a2, 0, 2)):
                message.message, message.wParam = code, param
                for _ in range(50):
                    frame.nativeEventFilter(None, ctypes.addressof(message))
                self.assertEqual(update.call_count, count)
        self.assertEqual(frame.track_mouse.call_count, 1)
        w.allow_close = True
        w.close()
        w.deleteLater()
