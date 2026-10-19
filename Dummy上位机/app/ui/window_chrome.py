"""A themed main caption with optional live controls and native secondary captions."""
from __future__ import annotations

import ctypes
from ctypes import wintypes
import sys

from PySide6.QtCore import QAbstractNativeEventFilter, QEvent, QObject, QPoint, QRect, QRectF, QSize, Qt, QTimer
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QAbstractButton, QApplication, QHBoxLayout, QLabel, QSizePolicy, QWidget


_CAPTION_COLORS = {
    "dark": ("#191c23", "#e5e9ef", "#30343e", "#3c424e"),
    "light": ("#ffffff", "#243248", "#e0e5ed", "#d1d8e3"),
}
# Content surfaces retain their radius; DWM manages the top-level window corners.
CORNER_RADIUS = 16


def _edge_at(point, size, border=6):
    edges = Qt.Edge(0)
    if point.x() < border:
        edges |= Qt.Edge.LeftEdge
    elif point.x() >= size.width() - border:
        edges |= Qt.Edge.RightEdge
    if point.y() < border:
        edges |= Qt.Edge.TopEdge
    elif point.y() >= size.height() - border:
        edges |= Qt.Edge.BottomEdge
    return edges


_EDGE_HITS = {
    Qt.Edge.LeftEdge: 10, Qt.Edge.RightEdge: 11, Qt.Edge.TopEdge: 12,
    Qt.Edge.LeftEdge | Qt.Edge.TopEdge: 13, Qt.Edge.RightEdge | Qt.Edge.TopEdge: 14,
    Qt.Edge.BottomEdge: 15, Qt.Edge.LeftEdge | Qt.Edge.BottomEdge: 16,
    Qt.Edge.RightEdge | Qt.Edge.BottomEdge: 17,
}


class _CaptionButton(QAbstractButton):
    def __init__(self, caption, role, text, callback):
        super().__init__(caption)
        self.caption, self.role = caption, role
        self.native_hover = False
        self.setFixedSize(46, 28)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setAccessibleName(text)
        self.setToolTip(text)
        self.clicked.connect(callback)

    def enterEvent(self, event):
        self.update()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.update()
        super().leaveEvent(event)

    def paintEvent(self, event):
        bg, fg, hover, pressed = _CAPTION_COLORS[self.caption.theme]
        hovered = self.underMouse() or self.native_hover
        if not self.isEnabled():
            fg = "#69717f" if self.caption.theme == "dark" else "#9ca5b4"
        elif self.role == "close" and (hovered or self.isDown()):
            bg, fg = ("#ab303b" if self.isDown() else "#c9434b"), "#ffffff"
        elif self.isDown() or hovered:
            bg = pressed if self.isDown() else hover
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(bg))
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(QPen(QColor(fg), 1))
        x, y = self.width() / 2, self.height() / 2
        if self.caption.toolbar is not None:
            painter.translate(x, y)
            painter.scale(1.5, 1.5)
            x, y = 0, 0
        if self.role == "minimize":
            painter.drawLine(QPoint(round(x - 5), round(y)), QPoint(round(x + 5), round(y)))
        elif self.role == "close":
            painter.drawLine(QPoint(round(x - 4), round(y - 4)), QPoint(round(x + 4), round(y + 4)))
            painter.drawLine(QPoint(round(x - 4), round(y + 4)), QPoint(round(x + 4), round(y - 4)))
        elif self.caption.host.isMaximized():
            painter.drawRect(QRectF(x - 2, y - 5, 7, 7))
            painter.fillRect(QRectF(x - 5, y - 2, 7, 7), QColor(bg))
            painter.drawRect(QRectF(x - 5, y - 2, 7, 7))
        else:
            painter.drawRect(QRectF(x - 4, y - 4, 8, 8))


class _WindowsMainFrame(QAbstractNativeEventFilter):
    """Keep Windows move/resize/work-area behavior around the custom caption."""
    def __init__(self, caption):
        super().__init__()
        self.caption = caption
        self._refreshing = False
        self._max_pressed = False
        self.handle = 0
        self._rounded_handle = 0
        self.set_dwm_attribute = None
        try:
            dwm = ctypes.WinDLL("dwmapi", use_last_error=True)
            self.set_dwm_attribute = dwm.DwmSetWindowAttribute
            self.set_dwm_attribute.argtypes = (wintypes.HWND, wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD)
            self.set_dwm_attribute.restype = ctypes.c_long
        except (OSError, AttributeError):
            pass
        user = ctypes.WinDLL("user32", use_last_error=True)
        suffix = "PtrW" if ctypes.sizeof(ctypes.c_void_p) == 8 else "W"
        self.get_style = getattr(user, "GetWindowLong" + suffix)
        self.get_style.argtypes, self.get_style.restype = (wintypes.HWND, ctypes.c_int), ctypes.c_ssize_t
        self.set_style = getattr(user, "SetWindowLong" + suffix)
        self.set_style.argtypes = (wintypes.HWND, ctypes.c_int, ctypes.c_ssize_t)
        self.set_style.restype = ctypes.c_ssize_t
        self.set_position = user.SetWindowPos
        self.set_position.argtypes = (wintypes.HWND, wintypes.HWND, ctypes.c_int, ctypes.c_int,
                                      ctypes.c_int, ctypes.c_int, wintypes.UINT)
        self.set_position.restype = wintypes.BOOL
        self.get_rect = user.GetWindowRect
        self.get_rect.argtypes, self.get_rect.restype = (wintypes.HWND, ctypes.POINTER(wintypes.RECT)), wintypes.BOOL
        self.client_origin = user.ClientToScreen
        self.client_origin.argtypes = (wintypes.HWND, ctypes.POINTER(wintypes.POINT))
        self.client_origin.restype = wintypes.BOOL
        self.is_zoomed = user.IsZoomed
        self.is_zoomed.argtypes, self.is_zoomed.restype = (wintypes.HWND,), wintypes.BOOL
        self.show_window = user.ShowWindow
        self.show_window.argtypes, self.show_window.restype = (wintypes.HWND, ctypes.c_int), wintypes.BOOL
        self.capture_mouse = user.SetCapture
        self.capture_mouse.argtypes, self.capture_mouse.restype = (wintypes.HWND,), wintypes.HWND
        self.release_mouse = user.ReleaseCapture
        self.release_mouse.argtypes, self.release_mouse.restype = (), wintypes.BOOL
        class Tracking(ctypes.Structure):
            _fields_ = [("size", wintypes.DWORD), ("flags", wintypes.DWORD),
                        ("window", wintypes.HWND), ("hover_time", wintypes.DWORD)]
        self.tracking_type = Tracking
        self.track_mouse = user.TrackMouseEvent
        self.track_mouse.argtypes, self.track_mouse.restype = (ctypes.POINTER(Tracking),), wintypes.BOOL
        self.monitor = user.MonitorFromWindow
        self.monitor.argtypes, self.monitor.restype = (wintypes.HWND, wintypes.DWORD), wintypes.HANDLE

        class MonitorInfo(ctypes.Structure):
            _fields_ = [("size", wintypes.DWORD), ("screen", wintypes.RECT),
                        ("work", wintypes.RECT), ("flags", wintypes.DWORD)]
        self.monitor_info_type = MonitorInfo
        self.monitor_info = user.GetMonitorInfoW
        self.monitor_info.argtypes = (wintypes.HANDLE, ctypes.POINTER(MonitorInfo))
        self.monitor_info.restype = wintypes.BOOL

    def ensure_frame(self):
        window = self.caption.host
        if self._refreshing or not window.testAttribute(Qt.WidgetAttribute.WA_WState_Created):
            return
        handle = int(window.winId())
        self.handle = handle
        style = self.get_style(handle, -16)
        # WS_CAPTION is retained for Windows' normal maximize/work-area state
        # semantics; WM_NCCALCSIZE below removes its native drawing/layout.
        desired = (style & ~0x80000000) | 0x00c00000 | 0x00040000 | 0x00020000 | 0x00010000 | 0x00080000
        if desired != style:
            self._refreshing = True
            try:
                self.set_style(handle, -16, desired)
                self.set_position(handle, None, 0, 0, 0, 0, 0x37)
            finally:
                self._refreshing = False
        if self._rounded_handle != handle and self.set_dwm_attribute is not None:
            # DWMWA_WINDOW_CORNER_PREFERENCE / DWMWCP_ROUND. DWM handles
            # antialiasing and keeps maximized/snapped windows square. Older
            # Windows versions can reject this appearance-only preference.
            preference = wintypes.DWORD(2)
            self.set_dwm_attribute(handle, 33, ctypes.byref(preference), ctypes.sizeof(preference))
            self._rounded_handle = handle

    def _hit_test(self, handle, packed):
        window = self.caption.host
        if not window.isEnabled():
            return 1
        x, y = ctypes.c_short(packed & 0xffff).value, ctypes.c_short((packed >> 16) & 0xffff).value
        rect = wintypes.RECT()
        if not self.get_rect(handle, ctypes.byref(rect)):
            return 1
        scale = window.devicePixelRatioF()
        if not window.isMaximized() and not window.isFullScreen():
            edges = _edge_at(QPoint(x - rect.left, y - rect.top),
                             QSize(rect.right - rect.left, rect.bottom - rect.top), max(4, round(6 * scale)))
            if edges:
                # Visible Qt edge grips own both cursor feedback and the
                # startSystemResize entry, including on a frameless HWND.
                return 1
        origin = wintypes.POINT()
        self.client_origin(handle, ctypes.byref(origin))
        point = QPoint(round((x - origin.x) / scale), round((y - origin.y) / scale))
        button = self.caption.maximize_button
        if button.isEnabled() and button.geometry().contains(point):
            return 9  # HTMAXBUTTON also exposes the Windows 11 snap affordance.
        if self.caption.contains_client_point(point):
            return 1  # Real client controls, especially STOP, must receive clicks.
        if (self.caption.geometry().contains(point)
                and point.x() < self.caption.minimize_button.x()):
            return 2  # HTCAPTION: native dragging, drag-to-restore and double click.
        return 1

    def toggle_maximize(self):
        # Qt 6.10 emulates frameless maximization with MoveWindow. Use the
        # native state/placement so restore and caption double clicks agree.
        self.ensure_frame()
        self.show_window(self.handle, 9 if self.is_zoomed(self.handle) or self.caption.host.isMaximized() else 3)

    def nativeEventFilter(self, event_type, message):
        window = self.caption.host
        msg = wintypes.MSG.from_address(int(message))
        if not self.handle or msg.hWnd != self.handle:
            return False, 0
        if msg.message == 0x83 and msg.wParam:  # WM_NCCALCSIZE
            if (self.is_zoomed(msg.hWnd) or window.isMaximized()) and not (window.isFullScreen() and not window.isMaximized()):
                info = self.monitor_info_type()
                info.size = ctypes.sizeof(info)
                if self.monitor_info(self.monitor(msg.hWnd, 2), ctypes.byref(info)):
                    rect = ctypes.cast(msg.lParam, ctypes.POINTER(wintypes.RECT)).contents
                    rect.left, rect.top = max(rect.left, info.work.left), max(rect.top, info.work.top)
                    rect.right, rect.bottom = min(rect.right, info.work.right), min(rect.bottom, info.work.bottom)
                    if (info.work.left, info.work.top, info.work.right, info.work.bottom) == (
                            info.screen.left, info.screen.top, info.screen.right, info.screen.bottom):
                        # With no reserved taskbar area Qt otherwise mistakes a
                        # borderless maximized client for fullscreen. Keep a 2px
                        # reveal edge while retaining a real maximized state.
                        rect.bottom -= 2
            return True, 0
        if msg.message == 0x84:  # WM_NCHITTEST
            return True, self._hit_test(msg.hWnd, msg.lParam)
        button = self.caption.maximize_button
        if msg.message in (0xa0, 0x200, 0x2a2):  # non-client/client move or non-client leave
            hovered = msg.message == 0xa0 and msg.wParam == 9
            if hovered and not button.native_hover:
                tracking = self.tracking_type(ctypes.sizeof(self.tracking_type), 0x12, self.handle, 0)
                self.track_mouse(ctypes.byref(tracking))
            if button.native_hover != hovered:
                button.native_hover = hovered
                button.update()
        elif msg.message == 0xa1 and msg.wParam == 9:  # WM_NCLBUTTONDOWN
            self._max_pressed = True
            button.setDown(True)
            self.capture_mouse(self.handle)
            return True, 0
        elif msg.message in (0xa2, 0x202) and self._max_pressed:  # non-client/client button up
            self._max_pressed = False
            button.setDown(False)
            packed = msg.lParam
            if msg.message == 0x202:
                origin = wintypes.POINT()
                self.client_origin(self.handle, ctypes.byref(origin))
                x = ctypes.c_short(packed & 0xffff).value + origin.x
                y = ctypes.c_short((packed >> 16) & 0xffff).value + origin.y
                packed = ((y & 0xffff) << 16) | (x & 0xffff)
            self.release_mouse()
            if self._hit_test(self.handle, packed) == 9 and button.isEnabled():
                self.caption.toggle_maximize()
            return True, 0
        elif msg.message in (0x215, 0x1f) and self._max_pressed:  # capture changed / cancel mode
            self._max_pressed = False
            button.setDown(False)
        return False, 0


class _ResizeEdge(QWidget):
    """Explicit edge cursor and system resize entry on every platform."""
    def __init__(self, caption, edges):
        super().__init__(caption.host)
        self.host, self.edges = caption.host, edges
        self.origin = None
        self.setMouseTracking(True)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground)
        diagonal = edges in (Qt.Edge.LeftEdge | Qt.Edge.TopEdge, Qt.Edge.RightEdge | Qt.Edge.BottomEdge)
        cursor = (Qt.CursorShape.SizeFDiagCursor if diagonal else Qt.CursorShape.SizeBDiagCursor
                  if int(edges.value).bit_count() == 2 else Qt.CursorShape.SizeHorCursor
                  if edges & (Qt.Edge.LeftEdge | Qt.Edge.RightEdge) else Qt.CursorShape.SizeVerCursor)
        self.setCursor(cursor)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            if not self._start_system_resize():
                self.origin = (event.globalPosition().toPoint(), self.host.geometry())
            event.accept()

    def _start_system_resize(self):
        handle = self.host.windowHandle()
        return bool(handle is not None and handle.startSystemResize(self.edges))

    def mouseMoveEvent(self, event):
        if self.origin is None or not event.buttons() & Qt.MouseButton.LeftButton:
            return
        origin, initial = self.origin
        delta = event.globalPosition().toPoint() - origin
        rect = QRect(initial)
        if self.edges & Qt.Edge.LeftEdge:
            rect.setLeft(min(initial.left() + delta.x(), initial.right() - self.host.minimumWidth() + 1))
        if self.edges & Qt.Edge.RightEdge:
            rect.setRight(max(initial.right() + delta.x(), initial.left() + self.host.minimumWidth() - 1))
        if self.edges & Qt.Edge.TopEdge:
            rect.setTop(min(initial.top() + delta.y(), initial.bottom() - self.host.minimumHeight() + 1))
        if self.edges & Qt.Edge.BottomEdge:
            rect.setBottom(max(initial.bottom() + delta.y(), initial.top() + self.host.minimumHeight() - 1))
        self.host.setGeometry(rect)

    def mouseReleaseEvent(self, event):
        self.origin = None


class CustomTitleBar(QWidget):
    def __init__(self, window):
        super().__init__(window)
        self.host, self.theme = window, "light"
        self._drag_origin = None
        self.native_frame = None
        self.resize_edges = []
        self.status_widget = None
        self.toolbar = None
        self.client_widgets = ()
        self.setObjectName("CustomTitleBar")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground)
        self.setFixedHeight(28)
        row = QHBoxLayout(self)
        row.setContentsMargins(8, 0, 0, 0)
        row.setSpacing(0)
        self.title = QLabel(window.windowTitle())
        self.title.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.title.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        row.addWidget(self.title, 1)
        self.minimize_button = _CaptionButton(self, "minimize", "最小化", window.showMinimized)
        self.maximize_button = _CaptionButton(self, "maximize", "最大化", self.toggle_maximize)
        self.close_button = _CaptionButton(self, "close", "关闭", window.close)
        for button in (self.minimize_button, self.maximize_button, self.close_button):
            row.addWidget(button)
        window.windowTitleChanged.connect(self.title.setText)
        window.installEventFilter(self)
        app = QApplication.instance()
        if sys.platform == "win32" and app.platformName() == "windows":
            try:
                self.native_frame = _WindowsMainFrame(self)
                app.installNativeEventFilter(self.native_frame)
                window.destroyed.connect(lambda: app.removeNativeEventFilter(self.native_frame))
            except (OSError, AttributeError):
                self.native_frame = None
        self.resize_edges = [_ResizeEdge(self, edges) for edges in _EDGE_HITS]
        self.set_theme("light")

    def set_toolbar(self, toolbar, status_widget, client_widgets):
        """One header surface; native window actions stay in its top right."""
        self.toolbar, self.status_widget = toolbar, status_widget
        self.client_widgets = tuple(client_widgets)
        row = self.layout()
        while row.count():
            row.takeAt(0)
        self.title.hide()
        row.setContentsMargins(0, 0, 0, 0)
        self.host.centralWidget().layout().removeWidget(toolbar)
        row.addWidget(toolbar)
        self.setFixedHeight(112)
        self._place_window_buttons()

    def _place_window_buttons(self):
        buttons = (self.minimize_button, self.maximize_button, self.close_button)
        for index, button in enumerate(buttons):
            if self.toolbar is not None:
                button.setFixedSize(52, self.height())
                button.move(self.width() - (len(buttons) - index) * 52, 0)
            button.raise_()

    @property
    def window_buttons_width(self):
        return sum(button.width() for button in
                   (self.minimize_button, self.maximize_button, self.close_button))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.toolbar is not None:
            self._place_window_buttons()

    def contains_client_point(self, point):
        if self.toolbar is None:
            return self.contains_status_point(point)
        return any(widget.isVisible() and widget.rect().contains(widget.mapFrom(self.host, point))
                   for widget in self.client_widgets)

    def contains_status_point(self, point):
        widget = self.status_widget
        return bool(widget is not None and widget.isVisible() and
                    widget.rect().contains(widget.mapFrom(self.host, point)))

    def set_theme(self, theme):
        self.theme = theme
        bg, fg, _, _ = _CAPTION_COLORS[theme]
        self.setStyleSheet(f"QWidget#CustomTitleBar {{ background: {bg}; border: none; }}")
        self.title.setStyleSheet(f"color: {fg}; background: transparent; font-size: 12px; padding: 0;")
        for button in (self.minimize_button, self.maximize_button, self.close_button):
            button.update()

    def toggle_maximize(self):
        if self.native_frame is not None:
            self.native_frame.toggle_maximize()
        elif self.host.isMaximized():
            self.host.showNormal()
        else:
            self.host.showMaximized()

    def _update_frame(self):
        maximized = self.host.isMaximized()
        self.maximize_button.setToolTip("还原" if maximized else "最大化")
        self.maximize_button.setAccessibleName(self.maximize_button.toolTip())
        self.maximize_button.native_hover = False
        self.maximize_button.update()
        if self.native_frame is not None:
            self.native_frame.ensure_frame()
        width, height, border = self.host.width(), self.host.height(), 6
        for handle in self.resize_edges:
            edges = handle.edges
            corner = int(edges.value).bit_count() == 2
            extent = CORNER_RADIUS if corner else border
            x = 0 if edges & Qt.Edge.LeftEdge else width - extent if edges & Qt.Edge.RightEdge else border
            y = 0 if edges & Qt.Edge.TopEdge else height - extent if edges & Qt.Edge.BottomEdge else border
            handle.setGeometry(x, y, extent if corner or edges & (Qt.Edge.LeftEdge | Qt.Edge.RightEdge) else width - 2 * border,
                               extent if corner or edges & (Qt.Edge.TopEdge | Qt.Edge.BottomEdge) else height - 2 * border)
            handle.setVisible(not maximized and not self.host.isFullScreen())
            handle.raise_()

    def eventFilter(self, watched, event):
        if watched is self.host and event.type() in (QEvent.Type.Show, QEvent.Type.Resize,
                                                     QEvent.Type.WinIdChange, QEvent.Type.WindowStateChange,
                                                     QEvent.Type.DevicePixelRatioChange):
            QTimer.singleShot(0, self, self._update_frame)
        return False

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            handle = self.host.windowHandle()
            if handle is None or not handle.startSystemMove():
                self._drag_origin = (event.globalPosition().toPoint(), self.host.pos())
            event.accept()

    def mouseMoveEvent(self, event):
        if self._drag_origin is not None and event.buttons() & Qt.MouseButton.LeftButton:
            origin, position = self._drag_origin
            self.host.move(position + event.globalPosition().toPoint() - origin)

    def mouseReleaseEvent(self, event):
        self._drag_origin = None

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_origin = None
            self.toggle_maximize()
            event.accept()


def install_custom_titlebar(window):
    window.setWindowFlag(Qt.WindowType.FramelessWindowHint, True)
    caption = CustomTitleBar(window)
    window.setMenuWidget(caption)
    return caption


def _colorref(rgb):
    """Win32 COLORREF stores RGB in the opposite order to HTML colors."""
    return ((rgb & 0xff) << 16) | (rgb & 0xff00) | ((rgb >> 16) & 0xff)


class _WindowsCaption:
    def __init__(self):
        dwm = ctypes.WinDLL("dwmapi", use_last_error=True)
        user = ctypes.WinDLL("user32", use_last_error=True)
        self.set_attribute = dwm.DwmSetWindowAttribute
        self.set_attribute.argtypes = (wintypes.HWND, wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD)
        self.set_attribute.restype = ctypes.c_long
        self.set_icon = user.SendMessageW
        self.set_icon.argtypes = (wintypes.HWND, wintypes.UINT, ctypes.c_size_t, ctypes.c_ssize_t)
        self.set_icon.restype = ctypes.c_ssize_t
        create_icon = user.CreateIcon
        create_icon.argtypes = (wintypes.HINSTANCE, ctypes.c_int, ctypes.c_int, wintypes.BYTE,
                                wintypes.BYTE, ctypes.c_void_p, ctypes.c_void_p)
        create_icon.restype = wintypes.HICON
        # A null icon makes Windows draw its default icon. A single shared,
        # transparent small icon hides that glyph without replacing ICON_BIG
        # (taskbar/Alt+Tab) or changing the window style and frame geometry.
        mask = (ctypes.c_ubyte * 32)(*([255] * 32))
        pixels = (ctypes.c_ubyte * 1024)()
        self.caption_icon = create_icon(None, 16, 16, 1, 32, mask, pixels)
        if not self.caption_icon:
            raise ctypes.WinError()

    def apply(self, handle, theme):
        dark = theme == "dark"
        # Unsupported DWM attributes return HRESULT errors on older Windows;
        # they are appearance-only and must never interfere with the window.
        values = ((20, int(dark)),
                  (35, _colorref(0x191c23 if dark else 0xf2f4f8)),
                  (36, _colorref(0xe5e9ef if dark else 0x243248)))
        for attribute, value in values:
            native_value = wintypes.DWORD(value)
            self.set_attribute(handle, attribute, ctypes.byref(native_value), ctypes.sizeof(native_value))
        self.set_icon(handle, 0x0080, 0, self.caption_icon)  # WM_SETICON / ICON_SMALL


class _WindowChrome(QObject):
    def __init__(self, app):
        super().__init__(app)
        self.theme = "light"
        self.backend = None
        # Offscreen tests and other platforms have no native Windows caption.
        if sys.platform == "win32" and app.platformName() == "windows":
            try:
                self.backend = _WindowsCaption()
            except (OSError, AttributeError):
                pass

    def set_theme(self, theme):
        if theme not in ("dark", "light"):
            raise ValueError("Unknown window theme")
        self.theme = theme
        for window in QApplication.topLevelWidgets():
            if window.isVisible() or isinstance(getattr(window, "title_bar", None), CustomTitleBar):
                self._apply(window)

    def _apply(self, window):
        if not isinstance(window, QWidget) or not window.isWindow():
            return
        caption = getattr(window, "title_bar", None)
        if isinstance(caption, CustomTitleBar):
            caption.set_theme(self.theme)
        if self.backend is None:
            return
        flags = window.windowFlags()
        if not flags & Qt.WindowType.WindowTitleHint or flags & Qt.WindowType.FramelessWindowHint:
            return
        if not window.testAttribute(Qt.WidgetAttribute.WA_WState_Created):
            return
        self.backend.apply(int(window.winId()), self.theme)

    def eventFilter(self, watched, event):
        if (self.backend is not None and isinstance(watched, QWidget) and watched.isWindow()
                and event.type() in (QEvent.Type.Show, QEvent.Type.WinIdChange, QEvent.Type.WindowIconChange)):
            self._apply(watched)
            # Qt may assign its icon while finishing native window creation.
            # The context cancels this retry if that widget has been destroyed.
            QTimer.singleShot(0, watched, lambda: self._apply(watched))
        return False


def apply_window_theme(theme):
    """Install once; also style future Qt dialogs and recreated native handles."""
    app = QApplication.instance()
    if not isinstance(app, QApplication):
        return None
    chrome = getattr(app, "_dummy_window_chrome", None)
    if chrome is None:
        chrome = _WindowChrome(app)
        app._dummy_window_chrome = chrome
        app.installEventFilter(chrome)
    chrome.set_theme(theme)
    return chrome
