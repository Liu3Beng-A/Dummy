"""Read-only operation feedback with a stable footprint and in-window details."""
from html import escape
from math import isfinite

from PySide6.QtCore import QEvent, QPoint, QSize, Qt
from PySide6.QtGui import QPainter, QPalette
from PySide6.QtWidgets import (
    QApplication, QFrame, QHBoxLayout, QLabel, QLayout, QScrollArea, QSizePolicy,
    QStackedLayout, QStyle, QStyleOption, QToolButton, QVBoxLayout, QWidget,
)


class _ElidedLabel(QLabel):
    """Keep full plain text for accessibility while painting one bounded line."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTextFormat(Qt.TextFormat.PlainText)
        self.setWordWrap(False)
        self.setMinimumWidth(0)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)

    def setText(self, text):
        text = str(text)
        super().setText(text)
        # Qt auto-detects rich text in tooltips; escape device-provided content.
        self.setToolTip(f"<qt>{escape(text).replace(chr(10), '<br>')}</qt>" if text else "")
        self.setAccessibleName(text)
        self.setAccessibleDescription(text)
        self.update()

    def fullText(self):
        return self.text()

    def displayedText(self):
        return self.fontMetrics().elidedText(
            " ".join(self.text().splitlines()), Qt.TextElideMode.ElideRight,
            max(0, self.contentsRect().width() - 2 * self.margin()))

    def paintEvent(self, event):
        painter = QPainter(self)
        option = QStyleOption()
        option.initFrom(self)
        self.style().drawPrimitive(QStyle.PrimitiveElement.PE_Widget, option, painter, self)
        painter.setFont(self.font())
        painter.setPen(self.palette().color(QPalette.ColorRole.WindowText))
        rect = self.contentsRect().adjusted(self.margin(), 0, -self.margin(), 0)
        painter.drawText(rect, self.alignment(), self.displayedText())


class OperationStatusPanel(QFrame):
    """Present snapshots without transport calls or changing command admission.

    Summary slots always occupy 80 logical pixels. Conditions are a workspace
    child overlay, never part of this layout or a new top-level window.
    """

    HEIGHT = 80
    ACTIONS = (
        ("start", "START"), ("stop", "STOP"), ("movej", "MoveJ"),
        ("movel", "MoveL"), ("gamepad", "手柄接管"), ("rgb_save", "保存 RGB"),
    )
    TERMINAL_PHASES = frozenset((
        "idle", "completed", "observed_reached", "position_reached", "rejected",
        "cancelled", "locked", "disconnected",
    ))

    def __init__(self, parent=None, *, compact=False):
        super().__init__(parent)
        self.setObjectName("OperationStatusPanel")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setFixedHeight(self.HEIGHT)
        self.setMinimumWidth(0)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._overlay_host = None
        self._swallow_release_button = None
        self._swallow_escape_release = False
        self._event_filter_installed = False
        self._compact = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(0)

        heading_widget = QWidget(self)
        heading_widget.setFixedHeight(24)
        heading = QHBoxLayout(heading_widget)
        heading.setContentsMargins(0, 0, 0, 0)
        heading.setSpacing(10)
        self.title_label = self._summary_label("OperationStatusTitle", heading_widget)
        font = self.title_label.font()
        font.setBold(True)
        self.title_label.setFont(font)
        heading.addWidget(self.title_label, 1)
        self.elapsed_label = self._summary_label("OperationStatusElapsed", heading_widget)
        self.elapsed_label.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Fixed)
        heading.addWidget(self.elapsed_label)
        self.details_button = QToolButton(heading_widget)
        self.details_button.setObjectName("OperationStatusDetailsButton")
        self.details_button.setCheckable(True)
        self.details_button.setMaximumHeight(24)
        self.details_button.setToolButtonStyle(Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
        heading.addWidget(self.details_button)
        layout.addWidget(heading_widget)

        body = QWidget(self)
        body.setFixedHeight(44)
        self.body_widget = body
        self._body_stack = QStackedLayout(body)
        self._body_stack.setContentsMargins(0, 0, 0, 0)
        self._body_stack.setSpacing(0)
        self._normal_body = QWidget(body)
        body_layout = QHBoxLayout(self._normal_body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(12)
        self._normal_body_layout = body_layout
        self._compact_body = QWidget(body)
        self._compact_body.installEventFilter(self)
        self._compact_body_layout = QStackedLayout(self._compact_body)
        self._compact_body_layout.setContentsMargins(0, 0, 0, 0)
        self._compact_body_layout.setSpacing(0)
        self._body_stack.addWidget(self._normal_body)
        self._body_stack.addWidget(self._compact_body)
        description = QWidget(self._normal_body)
        self.description_widget = description
        description.setFixedHeight(44)
        description.setMinimumWidth(0)
        description.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        description_layout = QVBoxLayout(description)
        description_layout.setContentsMargins(0, 1, 0, 1)
        description_layout.setSpacing(2)
        self.reason_label = self._summary_label("OperationStatusReason", description)
        self.next_step_label = self._summary_label("OperationStatusNextStep", description)
        self.reason_label.setFixedHeight(20)
        self.next_step_label.setFixedHeight(20)
        description_layout.addWidget(self.reason_label)
        description_layout.addWidget(self.next_step_label)
        body_layout.addWidget(description, 1)
        # Keep this slot even when no feedback is present. Configuration results
        # never resize the robot summary or push the workspace below it.
        self.feedback_slot = QWidget(body)
        self.feedback_slot.setObjectName("OperationStatusFeedbackSlot")
        self.feedback_slot.setFixedSize(380, 44)
        self.feedback_layout = QVBoxLayout(self.feedback_slot)
        self.feedback_layout.setContentsMargins(0, 0, 0, 0)
        self.feedback_layout.setSpacing(0)
        self._feedback_widget = None
        body_layout.addWidget(self.feedback_slot)
        layout.addWidget(body)

        self.details_overlay = QFrame(self)
        self.details_overlay.setObjectName("OperationStatusOverlay")
        self.details_overlay.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.details_overlay.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        overlay_layout = QVBoxLayout(self.details_overlay)
        overlay_layout.setContentsMargins(12, 9, 12, 9)
        overlay_layout.setSpacing(6)
        overlay_layout.setSizeConstraint(QLayout.SizeConstraint.SetNoConstraint)
        overlay_title = QLabel("操作条件 · 仅供查看", self.details_overlay)
        overlay_title.setObjectName("OperationStatusOverlayTitle")
        overlay_title.setFixedHeight(24)
        overlay_layout.addWidget(overlay_title)
        self.details_area = QScrollArea(self.details_overlay)
        self.details_area.setObjectName("OperationStatusDetails")
        self.details_area.setWidgetResizable(True)
        self.details_area.setFrameShape(QFrame.Shape.NoFrame)
        self.details_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.details_area.setMinimumSize(0, 0)
        self.details_area.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        contents = QWidget()
        contents.setObjectName("OperationStatusConditions")
        content_layout = QVBoxLayout(contents)
        content_layout.setContentsMargins(0, 3, 5, 3)
        content_layout.setSpacing(10)
        self.action_labels = {}
        for key, _ in self.ACTIONS:
            label = QLabel(contents)
            label.setObjectName("OperationStatusAction")
            label.setTextFormat(Qt.TextFormat.PlainText)
            label.setWordWrap(True)
            label.setMinimumWidth(0)
            label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
            label.setProperty("action", key)
            label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            label.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            self.action_labels[key] = label
            content_layout.addWidget(label)
        content_layout.addStretch()
        self.details_area.setWidget(contents)
        overlay_layout.addWidget(self.details_area, 1)
        self.details_button.toggled.connect(self._set_expanded)
        self.destroyed.connect(self.details_overlay.deleteLater)
        self._set_expanded(False)
        self.set_status({}, {})
        self.set_compact(compact)

    def sizeHint(self):
        return QSize(395 if self._compact else 320, self.HEIGHT)

    def minimumSizeHint(self):
        return QSize(0, self.HEIGHT)

    def set_feedback_widget(self, widget):
        """Mount a passive notice without coupling operation and action state."""
        if widget is self._feedback_widget:
            return widget
        if self._feedback_widget is not None:
            changed = getattr(self._feedback_widget, "event_changed", None)
            if changed is not None:
                try:
                    changed.disconnect(self._feedback_event_changed)
                except (RuntimeError, TypeError):
                    pass
            self.feedback_layout.removeWidget(self._feedback_widget)
            self._feedback_widget.hide()
            self._feedback_widget.setParent(None)
        self._feedback_widget = widget
        if widget is not None:
            widget.setParent(self.feedback_slot)
            self.feedback_layout.addWidget(widget)
            widget.show()
            changed = getattr(widget, "event_changed", None)
            if changed is not None:
                changed.connect(self._feedback_event_changed)
        self._feedback_event_changed()
        return widget

    def set_compact(self, enabled=True):
        """Share the fixed body between reasons and feedback in a narrow card."""
        enabled = bool(enabled)
        if enabled == self._compact:
            return
        self._compact = enabled
        self._property(self, "compact", enabled)
        source = self._normal_body_layout if enabled else self._compact_body_layout
        source.removeWidget(self.description_widget)
        source.removeWidget(self.feedback_slot)
        if enabled:
            self.feedback_slot.setMinimumWidth(0)
            self.feedback_slot.setMaximumWidth(16777215)
            self.feedback_slot.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            self._compact_body_layout.addWidget(self.description_widget)
            self._compact_body_layout.addWidget(self.feedback_slot)
            self._body_stack.setCurrentWidget(self._compact_body)
        else:
            self.feedback_slot.setFixedWidth(380)
            self._normal_body_layout.addWidget(self.description_widget, 1)
            self._normal_body_layout.addWidget(self.feedback_slot)
            self._body_stack.setCurrentWidget(self._normal_body)
            self.description_widget.show()
            self.feedback_slot.show()
        self._feedback_event_changed()
        self.updateGeometry()

    def _feedback_event_changed(self, *_args):
        if self._compact:
            event = getattr(self._feedback_widget, "current_event", None)
            self._compact_body_layout.setCurrentWidget(
                self.feedback_slot if event else self.description_widget)
            self._sync_compact_geometry()

    def _sync_compact_geometry(self):
        """Lay out hidden stack pages too, before their first feedback event."""
        if not self._compact:
            return
        rect = self._compact_body.contentsRect()
        for page in (self.description_widget, self.feedback_slot):
            page.setGeometry(rect)
            if page.layout() is not None:
                page.layout().setGeometry(page.contentsRect())
        if self._feedback_widget is not None and self._feedback_widget.layout() is not None:
            self._feedback_widget.layout().setGeometry(self._feedback_widget.contentsRect())

    def _summary_label(self, name, parent=None):
        label = _ElidedLabel(parent or self)
        label.setObjectName(name)
        label.setFixedHeight(24)
        return label

    @staticmethod
    def _property(widget, name, value):
        if widget.property(name) == value:
            return
        widget.setProperty(name, value)
        widget.style().unpolish(widget)
        widget.style().polish(widget)
        widget.update()

    def _default_overlay_host(self):
        window = self.window()
        central = getattr(window, "centralWidget", lambda: None)()
        return central or self.parentWidget() or self

    def mount_overlay(self, host):
        """Attach inside a workspace, without creating a native window."""
        host = host or self._default_overlay_host()
        if host is self._overlay_host:
            return
        self.details_overlay.hide()
        self.details_overlay.setParent(host, Qt.WindowType.Widget)
        self._overlay_host = host
        if self.details_button.isChecked() and self.isVisible():
            self.reposition_overlay()
            self.details_overlay.show()
            self.details_overlay.raise_()

    def reposition_overlay(self):
        host = self._overlay_host
        if host is None:
            return
        width = max(1, min(520, host.width() - 16))
        anchor = self.mapTo(host, QPoint(self.width() - width, self.height() + 4))
        x = max(8, min(anchor.x(), host.width() - 8 - width))
        y = max(8, anchor.y())
        available = host.height() - 8 - y
        if available < 96:
            height = max(1, min(330, host.height() - 16))
            y = max(8, host.height() - 8 - height)
        else:
            height = min(330, available)
        self.details_overlay.setGeometry(x, y, width, height)

    def _set_expanded(self, expanded):
        self.details_button.setText("收起条件" if expanded else "操作条件")
        self.details_button.setArrowType(Qt.ArrowType.UpArrow if expanded else Qt.ArrowType.DownArrow)
        self.details_button.setAccessibleName("收起操作条件" if expanded else "展开操作条件")
        if expanded:
            self.mount_overlay(self._default_overlay_host())
            self.reposition_overlay()
            self.details_overlay.show()
            self.details_overlay.raise_()
        else:
            self.details_overlay.hide()
        self._sync_event_filter()

    def _sync_event_filter(self):
        # Idle status refreshes do not need an application-wide Python hook.
        # Keep it only until an outside click / ESC gesture has fully ended.
        needed = self.isVisible() and bool(
            self.details_button.isChecked() or self._swallow_release_button is not None
            or self._swallow_escape_release)
        if needed == self._event_filter_installed:
            return
        app = QApplication.instance()
        if app is None:
            return
        if needed:
            app.installEventFilter(self)
        else:
            app.removeEventFilter(self)
        self._event_filter_installed = needed

    def close_details(self, *, clear_pending=False):
        if clear_pending:
            self._swallow_release_button = None
            self._swallow_escape_release = False
        self.details_button.setChecked(False)
        self.details_overlay.hide()
        self._sync_event_filter()

    @staticmethod
    def _belongs_to(widget, parent):
        return widget is parent or parent.isAncestorOf(widget)

    @staticmethod
    def _safety_control(widget):
        while widget is not None:
            if bool(widget.property("operationOverlaySafety")):
                return True
            widget = widget.parentWidget()
        return False

    def eventFilter(self, watched, event):
        event_type = event.type()
        if watched is self._compact_body and event_type == QEvent.Type.Resize:
            self._sync_compact_geometry()
        if event_type == QEvent.Type.MouseButtonRelease and self._swallow_release_button is not None:
            if event.button() == self._swallow_release_button:
                self._swallow_release_button = None
                self._sync_event_filter()
                return True
        if event_type == QEvent.Type.KeyRelease and self._swallow_escape_release:
            if event.key() == Qt.Key.Key_Escape:
                self._swallow_escape_release = False
                self._sync_event_filter()
                return True
        if watched in (self, self._overlay_host, self.window()):
            if event_type in (QEvent.Type.Hide, QEvent.Type.Close, QEvent.Type.WindowDeactivate):
                self.close_details(clear_pending=True)
        if not self.details_button.isChecked():
            return super().eventFilter(watched, event)
        if event_type in (QEvent.Type.ShortcutOverride, QEvent.Type.KeyPress):
            if event.key() == Qt.Key.Key_Escape:
                if event_type == QEvent.Type.KeyPress:
                    self._swallow_escape_release = True
                    self.close_details()
                event.accept()
                return True
        if watched in (self, self._overlay_host, self.window()):
            if event_type in (QEvent.Type.Resize, QEvent.Type.Move):
                self.reposition_overlay()
        if event_type in (QEvent.Type.MouseButtonPress, QEvent.Type.MouseButtonDblClick):
            if isinstance(watched, QWidget):
                if self._belongs_to(watched, self.details_overlay) or self._belongs_to(watched, self.details_button):
                    return False
                if watched.window() is not self.window():
                    self.close_details()
                    return False
                safety = self._safety_control(watched)
                if not safety:
                    self._swallow_release_button = event.button()
                    self.close_details()
                    return True
                self.close_details()
        return super().eventFilter(watched, event)

    def showEvent(self, event):
        super().showEvent(event)
        self.mount_overlay(self._default_overlay_host())
        self._sync_event_filter()

    def hideEvent(self, event):
        self.close_details(clear_pending=True)
        super().hideEvent(event)

    def set_status(self, operation: dict, actions: dict):
        """Render detached UI state; never emit or decide device actions."""
        operation = operation or {}
        actions = actions or {}
        phase = str(operation.get("phase") or "idle")
        tone = operation.get("tone", "info")
        if tone not in ("info", "success", "warning", "error"):
            tone = "info"
        for widget in (self, self.title_label, self.reason_label, self.next_step_label):
            self._property(widget, "tone", tone)
        self._property(self, "phase", phase)
        self.title_label.setText(str(operation.get("title") or "状态待确认"))
        # Empty idle/completed slots remain in place; labels are never removed.
        show_description = phase not in ("idle", "completed") or tone in ("warning", "error")
        reason = str(operation.get("reason") or "")
        next_step = str(operation.get("next_step") or "")
        self.reason_label.setText(reason if show_description else "")
        self.next_step_label.setText(f"下一步：{next_step}" if show_description and next_step else "")
        try:
            elapsed = float(operation.get("elapsed_s"))
        except (TypeError, ValueError, OverflowError):
            elapsed = float("nan")
        show_elapsed = phase not in self.TERMINAL_PHASES and isfinite(elapsed)
        elapsed_text = "已用时" if phase == "executing" else "已等待"
        self.elapsed_label.setText(
            f"{elapsed_text} {max(0.0, elapsed):.1f} 秒" if show_elapsed else "")
        self.elapsed_label.setVisible(show_elapsed)
        for key, default_label in self.ACTIONS:
            action = actions.get(key) or {}
            available = bool(action.get("available", False))
            status = action.get("status") or ("available" if available else "blocked")
            if status not in ("available", "blocked", "attention"):
                status = "available" if available else "blocked"
            description = {"available": "可操作", "blocked": "暂不可用", "attention": "注意"}[status]
            label = self.action_labels[key]
            self._property(label, "status", status)
            title = str(action.get("label") or default_label)
            lines = [f"{title} · {description}"]
            reason = str(action.get("reason") or "")
            next_step = str(action.get("next_step") or "")
            if reason:
                lines.append(f"原因：{reason}")
            if next_step:
                lines.append(f"下一步：{next_step}")
            if not action:
                lines.append("原因：操作条件待确认")
            label.setText("\n".join(lines))
            label.setAccessibleName(title)
            label.setAccessibleDescription(label.text())
