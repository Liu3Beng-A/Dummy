"""Workspace chrome and child overlays. No connection or motion decisions."""
from PySide6.QtCore import QEvent, QObject, QPoint, QRectF, QSize, Qt
from PySide6.QtGui import QColor, QIcon, QPainter, QPainterPath
from PySide6.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget,
)
from app.ui.brand_ring import BrandRing
from app.ui.icons import workstation_icon
from app.ui.widgets import button
from app.ui.window_chrome import CORNER_RADIUS


class InlinePanel(QFrame):
    """In-window panel: closing never consumes a click on STOP or DISABLE."""
    def __init__(self, window, anchor, title, owner=None, *, compact=False):
        super().__init__(window.centralWidget())
        self.window_host = window
        self.anchor = anchor
        self.owner = owner
        self.setObjectName("WorkstationPopover")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setMinimumWidth(188 if compact else 320)
        self.box = QVBoxLayout(self)
        self.box.setContentsMargins(*(4, 4, 4, 4) if compact else (14, 12, 14, 12))
        self.box.setSpacing(2 if compact else 10)
        if not compact:
            row = QHBoxLayout()
            caption = QLabel(title)
            caption.setObjectName("PopoverTitle")
            row.addWidget(caption, 1)
            close = button("×", self.hide)
            close.setAccessibleName("关闭" + title)
            close.setFixedSize(28, 28)
            row.addWidget(close)
            self.box.addLayout(row)
        anchor.clicked.connect(self.toggle)
        if owner is not None:
            owner.installEventFilter(self)
        self.hide()

    def toggle(self):
        if self.isVisible():
            self.hide()
        elif self.owner is None or self.owner.isEnabled():
            self.adjustSize()
            self.reposition()
            self.show()
            self.raise_()

    def reposition(self):
        host = self.window_host.centralWidget()
        origin = self.anchor.mapTo(host, QPoint(0, self.anchor.height() + 5))
        self.move(max(8, min(origin.x(), host.width() - self.width() - 8)),
                  max(8, min(origin.y(), host.height() - self.height() - 8)))

    def showEvent(self, event):
        super().showEvent(event)
        QApplication.instance().installEventFilter(self)

    def hideEvent(self, event):
        QApplication.instance().removeEventFilter(self)
        super().hideEvent(event)

    def eventFilter(self, watched, event):
        if watched is self.owner and event.type() == QEvent.Type.EnabledChange:
            if not self.owner.isEnabled():
                self.hide()
        if self.isVisible():
            if event.type() == QEvent.Type.KeyPress and event.key() == Qt.Key.Key_Escape:
                self.hide()
                return True
            if event.type() == QEvent.Type.MouseButtonPress:
                # QComboBox's own native popup remains part of this interaction.
                if QApplication.activePopupWidget() is not None:
                    return False
                point = event.globalPosition().toPoint()
                inside = self.rect().contains(self.mapFromGlobal(point))
                on_anchor = self.anchor.rect().contains(self.anchor.mapFromGlobal(point))
                if not inside and not on_anchor:
                    self.hide()
        return super().eventFilter(watched, event)


NAVIGATION = (("运动控制", "motion"), ("点位示教", "points"),
              ("电机设置", "motor"), ("夹爪设置", "gripper"),
              ("RGB 灯效", "rgb"), ("软件设置", "settings"))

SIDEBAR_WIDTH = 80
HEADER_HEIGHT = 65


class PageCorner(QWidget):
    """A small antialiased cover; works above opaque Qt and WebEngine children."""
    RADIUS = CORNER_RADIUS

    def __init__(self, surface, right, bottom):
        super().__init__(surface)
        self.right, self.bottom = right, bottom
        self.color = QColor("#ffffff")
        self.setFixedSize(self.RADIUS, self.RADIUS)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground)
        surface.installEventFilter(self)
        self._place()

    def _place(self):
        surface = self.parentWidget()
        self.move(surface.width() - self.width() if self.right else 0,
                  surface.height() - self.height() if self.bottom else 0)
        self.raise_()

    def eventFilter(self, watched, event):
        if event.type() in (QEvent.Type.Resize, QEvent.Type.Show):
            self._place()
        return False

    def set_theme(self, theme):
        self.color = QColor("#191c23" if theme == "dark" else "#ffffff")
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        cover = QPainterPath()
        cover.addRect(QRectF(self.rect()))
        inside = QPainterPath()
        radius = self.RADIUS
        inside.addEllipse(QRectF(-radius if self.right else 0,
                                -radius if self.bottom else 0, 2 * radius, 2 * radius))
        painter.fillPath(cover.subtracted(inside), self.color)


def install_shell(window):
    w = window
    w.sidebar = w.findChild(QFrame, "Sidebar")
    # One surface owns the outer corners and right/bottom gutters for every page.
    outer = w.centralWidget().layout()
    body = outer.itemAt(outer.count() - 1).layout()
    body.removeWidget(w.pages)
    w.page_host = QWidget()
    w.page_host.setObjectName("PageHost")
    host_layout = QVBoxLayout(w.page_host)
    host_layout.setContentsMargins(0, 0, 10, 10)
    host_layout.setSpacing(0)
    w.page_surface = QFrame()
    w.page_surface.setObjectName("PageSurface")
    surface_layout = QVBoxLayout(w.page_surface)
    surface_layout.setContentsMargins(0, 0, 0, 0)
    surface_layout.setSpacing(0)
    surface_layout.addWidget(w.pages)
    host_layout.addWidget(w.page_surface)
    body.addWidget(w.page_host, 1)
    w.page_corners = [PageCorner(w.page_surface, right, bottom)
                      for right in (False, True) for bottom in (False, True)]
    side = w.sidebar.layout()
    while side.count():
        item = side.takeAt(0)
        if item.widget():
            item.widget().hide()
    side.setContentsMargins(6, 10, 6, 12)
    side.setSpacing(8)
    w.sidebar_foot = w.sidebar.findChild(QLabel, "SidebarFoot")
    w.sidebar_foot.setText("6 轴 · 地轨 · 夹爪\nDummy Robot")
    # Keep the six original navigation indices and their existing routing.
    # Visible buttons let the sidebar have groups without adding fake pages.
    w.navigation.hide()
    w.sidebar_groups = []
    w.sidebar_buttons = []
    for index, (caption, _) in enumerate(NAVIGATION):
        if index in (0, 2):
            label = QLabel("工作区" if index == 0 else "设备配置")
            label.setObjectName("SidebarGroup")
            side.addWidget(label)
            w.sidebar_groups.append(label)
        if index == 2:
            side.insertSpacing(side.count() - 1, 18)
        if index == 5:
            side.addStretch(1)
            side.addWidget(w.log_toggle)
        control = button("", lambda value=index: navigate(w, value))
        control.setObjectName("SidebarItem")
        control.setCheckable(True)
        control.setToolTip(caption)
        control.setAccessibleName(caption)
        control.setFixedSize(52, 52)
        control.setIconSize(QSize(28, 28))
        w.sidebar_buttons.append(control)
        side.addWidget(control)
        side.setAlignment(control, Qt.AlignmentFlag.AlignHCenter)
    side.addWidget(w.sidebar_foot)
    w.sidebar_foot.hide()
    w.log_toggle.show()
    w.navigation.currentRowChanged.connect(lambda _: sync_navigation(w))
    top = w.topbar.layout()
    while top.count():
        top.takeAt(0)
    w.connection_controls = QWidget()
    w.connection_controls.setObjectName("ConnectionControls")
    row = QHBoxLayout(w.connection_controls)
    row.setContentsMargins(0, 0, 0, 0)
    row.setSpacing(6)
    for control in (w.serial_caption, w.port_combo, w.port_refresh_button,
                    w.connect_button, w.connection_status):
        row.addWidget(control)
    w.shell_brand = WorkstationBrand()
    top.addWidget(w.shell_brand)
    top.addWidget(w.connection_controls)
    top.addStretch(1)
    top.setContentsMargins(18, 10, 18, 10)
    top.setSpacing(12)
    w.mode_status.hide()  # Mounted in the target editor after it is constructed.
    w.control_source_label = QLabel("界面控制")
    w.control_source_label.setObjectName("ControlSourceLabel")
    w.caption_controls = QFrame()
    w.caption_controls.setObjectName("CaptionControls")
    caption = QHBoxLayout(w.caption_controls)
    caption.setContentsMargins(0, 0, 0, 0)
    caption.setSpacing(10)
    for control in (w.motor_status, w.control_source_label, w.stop_button):
        caption.addWidget(control)
    top.addWidget(w.caption_controls, 0, Qt.AlignmentFlag.AlignVCenter)
    w.title_bar.set_toolbar(w.topbar, w.caption_controls,
                            (w.shell_brand, w.connection_controls, w.caption_controls))
    w.motor_status.setObjectName("WorkstationState")
    w.log_toggle.setObjectName("SidebarTerminal")
    w.log_toggle.setFixedSize(52, 52)
    w.log_toggle.setIconSize(QSize(28, 28))
    side.setAlignment(w.log_toggle, Qt.AlignmentFlag.AlignHCenter)
    w.stop_button.setAccessibleName("STOP 请求主控软停")
    w._shell_panels = []
    w._shell_compact = None
    w._shell_theme = "light"
    sync_navigation(w)


class WorkstationBrand(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("WorkstationBrand")
        row = QHBoxLayout(self)
        # The fixed 63px mark sits 4px below its previous position.
        row.setContentsMargins(0, 5, 0, 0)
        row.setSpacing(0)
        self.mark = BrandRing()
        row.addWidget(self.mark, 0, Qt.AlignmentFlag.AlignCenter)


def sync_navigation(w):
    selected = w.navigation.currentRow()
    for index, control in enumerate(w.sidebar_buttons):
        control.setChecked(index == selected)


def navigate(w, index):
    w.navigation.setCurrentRow(index)
    sync_navigation(w)


def mount_motion_status(w, enabled):
    """Move the authoritative title; never copy or reinterpret device state."""
    enabled = bool(enabled)
    if getattr(w, "_motion_status_in_topbar", False) == enabled:
        return
    panel = w.operation_panel
    title = panel.title_label
    top = w.caption_controls.layout()
    heading = panel.heading_widget.layout()
    if enabled:
        heading.removeWidget(title)
        top.insertWidget(top.indexOf(w.motor_status), title)
        title.setFixedSize(250 if w.width() < 1440 else 300, 44)
        title.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        title.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        w.motor_status.hide()
        # Hide the parents so passive status/feedback refreshes cannot bring
        # individual explanation or elapsed labels back onto the motion page.
        panel.heading_widget.hide()
        panel.body_widget.hide()
        panel.layout().setAlignment(panel.details_button,
                                    Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
    else:
        top.removeWidget(title)
        heading.insertWidget(0, title, 1)
        title.setMinimumWidth(0)
        title.setMaximumWidth(16777215)
        title.setFixedHeight(24)
        title.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        title.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        panel.heading_widget.show()
        panel.body_widget.show()
        w.motor_status.show()
    # Ancestor selectors change with the destination page. Qt does not always
    # repolish descendants when only the ancestor's dynamic property changes.
    for control in (title, panel.reason_label, panel.next_step_label,
                    panel.elapsed_label, panel.details_button):
        control.style().unpolish(control)
        control.style().polish(control)
        control.update()
    title.show()
    w._motion_status_in_topbar = enabled


def fit_shell(w):
    compact = w.width() < 1280
    narrow = w.width() < 1440
    if compact != w._shell_compact:
        w._shell_compact = compact
        w.sidebar.setFixedWidth(SIDEBAR_WIDTH)
        w.sidebar_foot.hide()
        w.log_toggle.setText("")
        w.log_toggle.setToolTip("诊断终端")
        w.log_toggle.setAccessibleName("诊断终端")
        for control, (caption, _) in zip(w.sidebar_buttons, NAVIGATION):
            control.setText("")
        for label in w.sidebar_groups:
            label.hide()
    header_height, ring_size = HEADER_HEIGHT, 63
    w.title_bar.setFixedHeight(header_height)
    w.topbar.setFixedHeight(header_height)
    # Reserve the whole window-action column.
    # STOP's right edge must stay left of minimize at every window width.
    w.topbar.layout().setContentsMargins(0, 0,
                                        w.title_bar.window_buttons_width + 12, 0)
    w.topbar.layout().setSpacing(6 if narrow else 10)
    w.shell_brand.setFixedSize(SIDEBAR_WIDTH, header_height)
    w.shell_brand.mark.setFixedSize(ring_size, ring_size)
    w.caption_controls.setFixedHeight(44)
    w.caption_controls.layout().setSpacing(8 if narrow else 10)
    w.motor_status.setFixedSize(180 if narrow else 200, 44)
    if getattr(w, "_motion_status_in_topbar", False):
        w.operation_panel.title_label.setFixedSize(250 if narrow else 300, 44)
    w.control_source_label.setFixedSize(84 if narrow else 100, 44)
    w.stop_button.setFixedSize(104 if narrow else 124, 44)
    w.stop_button.setIconSize(QSize(18, 18))
    w.serial_caption.setFixedWidth(22 if narrow else 28)
    control_height = 34
    w.port_combo.setFixedSize(100 if narrow else 124, control_height)
    w.serial_caption.setFixedHeight(control_height)
    w.port_refresh_button.setFixedSize(44 if narrow else 52, control_height)
    w.connect_button.setFixedSize(84 if narrow else 94, control_height)
    w.connection_status.setFixedSize(68 if narrow else 82, control_height)
    controls = (w.serial_caption, w.port_combo, w.port_refresh_button,
                w.connect_button, w.connection_status)
    gap = 4 if narrow else 6
    w.connection_controls.layout().setSpacing(gap)
    widths = [control.width() for control in controls]
    connection_width = sum(widths) + (len(controls) - 1) * gap
    w.connection_controls.setFixedSize(connection_width, control_height)
    sync_navigation(w)
    for panel in w._shell_panels:
        if panel.isVisible():
            panel.reposition()


def update_shell(w):
    if not hasattr(w, "connection_controls"):
        return
    sync_navigation(w)
    if hasattr(w, "input_mode"):
        w.control_source_label.setText("手柄控制" if w.input_mode.currentIndex() else "界面控制")
    if hasattr(w, "motion_power_adapter"):
        w.motion_power_adapter.refresh()


class MotionPowerAdapter(QObject):
    """Two explicit presentation actions using the original command gates."""
    def __init__(self, w):
        super().__init__(w)
        self.w = w
        for control in w.motion_system_buttons[:2]:
            control.installEventFilter(self)
        for index, control in enumerate((w.motion_enable_button, w.motion_disable_button)):
            control.clicked.connect(lambda _checked=False, action=index: self.dispatch(action))

    def refresh(self):
        w = self.w
        facts = w.controller.state()
        motor_enabled = facts.get("motor_enabled")
        operation = facts.get("operation") or {}
        kind, phase = operation.get("kind"), operation.get("phase")
        pending = kind in ("start", "disable") and phase in ("queued", "sending", "waiting", "waiting_confirmation")
        for index, (control, caption, action) in enumerate((
                (w.motion_enable_button, "使能", "start"),
                (w.motion_disable_button, "失能", "disable"))):
            original = w.motion_system_buttons[index]
            confirming = pending and kind == action
            control.setText(caption + "中" if confirming else caption)
            # Only device-confirmed motor state suppresses a redundant action;
            # sending a request never flips this state locally.
            redundant = motor_enabled is (action == "start")
            blocked = pending if action == "start" else confirming
            control.setEnabled(original.isEnabled() and not blocked and not redundant)
            reason = ("设备已确认使能，无需重复请求。" if action == "start" else
                      "设备已确认失能，无需重复请求。") if redundant else ""
            control.setToolTip("\n".join(filter(None, (original.toolTip(), reason))))
            control.setAccessibleDescription("\n".join(filter(None, (original.accessibleDescription(), reason))))
        if hasattr(w, "motion_calibration_button"):
            calibration = w.motion_system_buttons[4]
            w.motion_calibration_button.setEnabled(calibration.isEnabled())
            w.motion_calibration_button.setToolTip(calibration.toolTip())

    def dispatch(self, index):
        self.w.motion_system_buttons[index].click()
        self.refresh()

    def eventFilter(self, watched, event):
        if event.type() == QEvent.Type.EnabledChange:
            self.refresh()
        return False


def theme_shell(w, theme):
    w._shell_theme = theme
    w.shell_brand.mark.set_theme(theme)
    for corner in w.page_corners:
        corner.set_theme(theme)
    color = "#acb6c4" if theme == "dark" else "#5e6c7d"
    for index, (_, name) in enumerate(NAVIGATION):
        w.navigation.item(index).setIcon(workstation_icon(name, color))
        w.sidebar_buttons[index].setIcon(workstation_icon(name, color, size=28))
    w.log_toggle.setIcon(workstation_icon("terminal", color, size=28))
    w.stop_button.setIcon(QIcon())
    if hasattr(w, "motion_system_buttons"):
        for control, name in zip(w.motion_system_buttons, ("power", "power", None, None, "settings")):
            control.setIcon(workstation_icon(name, color) if name else QIcon())
        if hasattr(w, "motion_enable_button"):
            w.motion_enable_button.setIcon(workstation_icon("power", color))
            w.motion_disable_button.setIcon(workstation_icon("power", color))
            w.quick_point_save.setIcon(workstation_icon("plus", color))
            w.viewer_reset_button.setIcon(workstation_icon("reset", color))
            w.movej_send_button.setIcon(workstation_icon("send", "#ffffff"))
            w.movel_send_button.setIcon(workstation_icon("send", "#ffffff"))
            for control in (w.movej_read_button, w.movel_read_button):
                control.setIcon(workstation_icon("refresh", color))
        for control, name in ((w.viewer_grid_button, "grid"), (w.viewer_axes_button, "axes"),
                              (w.viewer_target_button, "layers")):
            control.setIcon(workstation_icon(name, color))
