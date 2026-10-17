"""Passive configuration feedback in a permanent, bounded UI slot."""
from copy import deepcopy
from html import escape
from math import ceil, isfinite
import time

from PySide6.QtCore import QEasingCurve, QPropertyAnimation, QSize, Qt, QTimer, Signal
from PySide6.QtWidgets import QFrame, QGraphicsOpacityEffect, QSizePolicy, QVBoxLayout

from app.ui.operation_panel import _ElidedLabel


def feedback_tone(event):
    """Style explicit evidence without promoting forwarded requests to success."""
    event = event or {}
    phase = str(event.get("phase") or "pending")
    if phase in ("success", "confirmed") and event.get("evidence") == "request_confirmed":
        return "info"
    return {
        "success": "success", "confirmed": "success", "error": "error",
        "blocked": "warning", "invalid": "warning", "uncertain": "warning",
    }.get(phase, "info")


class ActionFeedbackNotice(QFrame):
    """Show explicit action events; never infer results or interact with transport.

    The frame stays in the layout after expiry. Only its opacity changes, so
    feedback cannot move surrounding controls or take keyboard focus.
    """

    HEIGHT = 44
    FADE_MS = 180
    DURATIONS_MS = {
        "success": 3000, "confirmed": 3000,
        "blocked": 5000, "invalid": 5000,
        "error": 6000, "uncertain": 6000, "cancelled": 3000,
    }
    expired = Signal(object)
    event_changed = Signal(bool)

    def __init__(self, parent=None, *, clock=None):
        super().__init__(parent)
        self.setObjectName("ActionFeedbackNotice")
        self.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setFixedHeight(self.HEIGHT)
        self.setMinimumWidth(0)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
        self._clock = clock or time.monotonic
        self._event = None
        self._generation = 0
        self._deadline = None
        self._remaining_ms = None
        self._hovered = False
        self._animation = None
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._on_timeout)
        self._opacity = QGraphicsOpacityEffect(self)
        self._opacity.setOpacity(0.0)
        self.setGraphicsEffect(self._opacity)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(9, 2, 9, 2)
        layout.setSpacing(0)
        self.message_label = _ElidedLabel(self)
        self.message_label.setObjectName("ActionFeedbackMessage")
        self.message_label.setFixedHeight(20)
        self.next_step_label = _ElidedLabel(self)
        self.next_step_label.setObjectName("ActionFeedbackNextStep")
        self.next_step_label.setFixedHeight(20)
        layout.addWidget(self.message_label)
        layout.addWidget(self.next_step_label)

    def sizeHint(self):
        return QSize(380, self.HEIGHT)

    def minimumSizeHint(self):
        return QSize(0, self.HEIGHT)

    @property
    def current_event(self):
        """Return detached event data for consumers and accessibility checks."""
        return deepcopy(self._event)

    @staticmethod
    def _property(widget, name, value):
        if widget.property(name) != value:
            widget.setProperty(name, value)
            widget.style().unpolish(widget)
            widget.style().polish(widget)
            widget.update()

    def _stop_animation(self):
        animation = self._animation
        self._animation = None
        if animation is not None:
            animation.stop()
            animation.deleteLater()

    def set_event(self, event):
        """Replace this single card, including updates to the same task ID."""
        if not event:
            self.clear_event()
            return
        if not isinstance(event, dict):
            return
        event = deepcopy(event)
        if event == self._event:
            return
        # A delayed earlier update for the visible task cannot undo its result.
        if self._event is not None and event.get("task_id") == self._event.get("task_id"):
            try:
                incoming_time = float(event["updated_at"])
                current_time = float(self._event["updated_at"])
            except (KeyError, TypeError, ValueError, OverflowError):
                pass
            else:
                if isfinite(incoming_time) and isfinite(current_time) and incoming_time < current_time:
                    return
        self._generation += 1
        self._timer.stop()
        self._stop_animation()
        self._event = event
        phase = str(event.get("phase") or "pending")
        tone = feedback_tone(event)
        for widget in (self, self.message_label, self.next_step_label):
            self._property(widget, "tone", tone)
            self._property(widget, "phase", phase)
        self._property(self, "scope", str(event.get("scope") or ""))
        message = str(event.get("message") or "")
        next_step = str(event.get("next_step") or "")
        self.message_label.setText(message)
        self.next_step_label.setText(next_step)
        self.setAccessibleName(message)
        self.setAccessibleDescription("\n".join(text for text in (message, next_step) if text))
        self.setToolTip("<qt>" + "<br>".join(
            escape(text).replace("\n", "<br>") for text in (message, next_step) if text) + "</qt>")
        self._opacity.setOpacity(1.0)
        duration = self.DURATIONS_MS.get(phase)
        self._remaining_ms = duration
        self._deadline = self._clock() + duration / 1000 if duration is not None else None
        if duration is not None and not self._hovered:
            self._timer.start(duration)
        self.event_changed.emit(True)

    def clear_event(self):
        """Clear content while preserving the permanent geometry slot."""
        had_event = self._event is not None
        self._generation += 1
        self._timer.stop()
        self._stop_animation()
        self._event = None
        self._deadline = None
        self._remaining_ms = None
        self.message_label.setText("")
        self.next_step_label.setText("")
        self.setAccessibleName("")
        self.setAccessibleDescription("")
        self.setToolTip("")
        self._opacity.setOpacity(0.0)
        if had_event:
            self.event_changed.emit(False)

    def _on_timeout(self):
        if self._event is None or self._deadline is None or self._hovered:
            return
        remaining = self._deadline - self._clock()
        if remaining > 0:
            # A timer notification queued before set_event must not expire its
            # replacement. Recheck the current deadline instead of old payloads.
            self._timer.start(max(1, ceil(remaining * 1000)))
            return
        if self._animation is not None:
            return
        generation = self._generation
        self._stop_animation()
        animation = QPropertyAnimation(self._opacity, b"opacity", self)
        self._animation = animation
        animation.setDuration(self.FADE_MS)
        animation.setStartValue(self._opacity.opacity())
        animation.setEndValue(0.0)
        animation.setEasingCurve(QEasingCurve.Type.InOutQuad)
        animation.finished.connect(lambda: self._finish_expiry(generation, animation))
        animation.start()

    def _finish_expiry(self, generation, animation):
        if generation != self._generation or animation is not self._animation or self._hovered:
            return
        event = self.current_event
        self.clear_event()
        self.expired.emit(event)

    def enterEvent(self, event):
        self._hovered = True
        if self._deadline is not None:
            self._remaining_ms = max(1, ceil((self._deadline - self._clock()) * 1000))
            self._timer.stop()
            self._stop_animation()
            self._opacity.setOpacity(1.0)
        super().enterEvent(event)

    def leaveEvent(self, event):
        self._hovered = False
        if self._event is not None and self._remaining_ms is not None:
            self._deadline = self._clock() + self._remaining_ms / 1000
            self._timer.start(self._remaining_ms)
        super().leaveEvent(event)
