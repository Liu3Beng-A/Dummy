"""Passive parameter editors: local drafts and readbacks remain separate."""
import math
from html import escape

from PySide6.QtCore import QEvent, QObject, QSignalBlocker, Qt, QTimer
from PySide6.QtWidgets import QAbstractSpinBox, QCheckBox, QLineEdit, QPushButton, QWidget

from app.ui.adapters import FieldAdapter
from app.ui.operation_panel import _ElidedLabel


def same_number(first, second):
    if first is None or second is None:
        return False
    try:
        left, right = float(first), float(second)
        return math.isfinite(left) and math.isfinite(right) and left == right
    except (TypeError, ValueError):
        return str(first) == str(second)


def parameter_tone(state, *, valid=True):
    """Presentation only: a request receipt is not a verified readback."""
    if not valid:
        return "error"
    sent = state.get("sent")
    matches_sent = (sent is not None and sent.get("fresh", False) and
                    same_number(state.get("draft"), sent.get("value")))
    if state.get("dirty") and not matches_sent:
        return "warning"
    if state.get("readback") is not None and state.get("fresh") and not state.get("dirty"):
        return "success"
    if matches_sent:
        return "error" if sent.get("phase") in ("error", "blocked", "invalid") else "warning"
    return "muted"


def set_parameter_tone(widget, tone):
    """Repolish a presentation property without changing text or dimensions."""
    if widget.property("parameterTone") != tone:
        widget.setProperty("parameterTone", tone)
        widget.style().unpolish(widget)
        widget.style().polish(widget)
        widget.update()


class ParameterFieldAdapter(FieldAdapter):
    """Read drafts, including uncommitted spin text, through the old API."""
    def __init__(self, binding):
        super().__init__(binding.widget)
        self.binding = binding

    def get(self):
        text = self.binding.snapshot()["draft"]
        if isinstance(self.widget, QLineEdit):
            return text
        value = self.binding.numeric(text)
        return text if value is None else value

    def set(self, value):
        self.binding.set_value(value)

    def delete(self, *_):
        # Legacy fill helpers call delete/insert as one replacement. Avoid
        # recording an intermediate empty draft or losing readback provenance.
        pass

    def insert(self, _index, value):
        self.set(value)


class ParameterBinding(QObject):
    """One view of a node/key; mirrored views share controller draft storage."""
    def __init__(self, manager, family, node, key, widget, default, *,
                 minimum=-math.inf, maximum=math.inf, integer=False, queryable=True,
                 applicable=None):
        super().__init__(widget)
        self.manager, self.controller = manager, manager.controller
        self.family, self.node_getter, self.key = family, node, key
        self.widget, self.default = widget, str(default)
        self.minimum, self.maximum, self.integer = minimum, maximum, integer
        self.queryable = queryable
        self.applicable = applicable or (lambda: True)
        self._rendering = self._fixing = self._typing = False
        self.readback_label = self._label("ParameterReadback", manager.stash)
        self.status_label = self._label("ParameterEditStatus", manager.stash)
        self.restore_button = QPushButton("还原编辑", manager.stash)
        self.restore_button.setObjectName("ParameterRestore")
        self.restore_button.setFixedHeight(28)
        self.restore_button.clicked.connect(self.restore)
        if isinstance(widget, QAbstractSpinBox):
            widget.lineEdit().textEdited.connect(self._text_edited)
            widget.valueChanged.connect(self._value_changed)
            widget.installEventFilter(self)
            widget.lineEdit().installEventFilter(self)
        elif isinstance(widget, QCheckBox):
            widget.toggled.connect(lambda checked: self._text_edited("1" if checked else "0"))
        else:
            # A plain line field has no numeric fixup; visible programmatic
            # changes are drafts too unless this binding is rendering model data.
            widget.textChanged.connect(self._text_edited)
        self.snapshot()

    @staticmethod
    def _label(name, parent):
        label = _ElidedLabel(parent)
        label.setObjectName(name)
        label.setFixedHeight(20)
        return label

    @property
    def node(self):
        return int(self.node_getter() if callable(self.node_getter) else self.node_getter)

    def snapshot(self):
        store = self.controller.parameter_edits
        store.ensure(self.family, self.node, self.key, self.default, queryable=self.queryable)
        return store.snapshot(self.family, self.node, self.key)

    def numeric(self, text):
        try:
            value = float(text)
            if (not math.isfinite(value) or not self.minimum <= value <= self.maximum or
                    (self.integer and not value.is_integer())):
                return None
            return int(value) if self.integer else value
        except (TypeError, ValueError, OverflowError):
            return None

    def raw_text(self):
        if isinstance(self.widget, QCheckBox):
            return "1" if self.widget.isChecked() else "0"
        if isinstance(self.widget, QAbstractSpinBox):
            text = self.widget.lineEdit().text()
            prefix, suffix = self.widget.prefix(), self.widget.suffix()
            if prefix and text.startswith(prefix):
                text = text[len(prefix):]
            if suffix and text.endswith(suffix):
                text = text[:-len(suffix)]
            return text.strip()
        return self.widget.text()

    def _text_edited(self, _text=None):
        if self._rendering or self.manager.refreshing:
            return
        self._typing = True
        try:
            self.controller.parameter_edits.edit(self.family, self.node, self.key, self.raw_text())
        finally:
            self._typing = False

    def _value_changed(self, value):
        if self._rendering or self._fixing or self.manager.refreshing:
            return
        # Explicit value changes (arrows, sliders, test adapters) are edits.
        # Keyboard intermediates are captured by textEdited before Qt fixup.
        raw = self.raw_text()
        text = (raw if same_number(raw, value) else
                str(int(value)) if self.integer else repr(float(value)))
        self.controller.parameter_edits.edit(self.family, self.node, self.key, text)

    def set_value(self, value):
        text = str(value)
        snapshot = self.snapshot()
        if text != snapshot["draft"]:
            self.controller.parameter_edits.edit(self.family, self.node, self.key, text)
        self.refresh()

    def eventFilter(self, watched, event):
        if (isinstance(self.widget, QAbstractSpinBox) and not self._rendering and
                (event.type() == QEvent.Type.FocusOut or
                 (event.type() == QEvent.Type.KeyPress and event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter)))):
            # Qt normally restores an invalid number to its old value before a
            # save button receives clicked. Keep the original draft instead.
            self._fixing = True
            QTimer.singleShot(0, self._finish_fixup)
        return super().eventFilter(watched, event)

    def _finish_fixup(self):
        self._fixing = False
        if not self.manager.closed:
            self.refresh()

    def refresh(self):
        state = self.snapshot()
        text = state["draft"]
        numeric = self.numeric(text)
        self._rendering = True
        blocker = QSignalBlocker(self.widget)
        try:
            if isinstance(self.widget, QCheckBox):
                self.widget.setChecked(text == "1")
            elif isinstance(self.widget, QAbstractSpinBox):
                line = self.widget.lineEdit()
                line_blocker = QSignalBlocker(line)
                if not self._typing and self.raw_text() != text:
                    if numeric is not None:
                        self.widget.setValue(numeric)
                    line.setText(self.widget.prefix() + text + self.widget.suffix())
                # Keep invalid/unfinished and out-of-interface-range drafts as
                # text. Never clamp a uint32 readback into a signed HAND PID.
                del line_blocker
            elif self.widget.text() != text:
                self.widget.setText(text)
        finally:
            del blocker
            self._rendering = False
        readback = state.get("readback")
        fresh = state.get("fresh", False)
        suffix = " · 已过期" if readback is not None and not fresh else ""
        self.readback_label.setText("最近回读：" + (str(readback) + suffix if readback is not None else
                                                   "尚未读取" if self.queryable else "此参数不支持查询"))
        sent = state.get("sent")
        matches_sent = (sent is not None and sent.get("fresh", False) and
                        same_number(text, sent.get("value")))
        if numeric is None:
            status = "输入未完成 / 超出范围"
        elif state.get("dirty") and not matches_sent:
            status = "本地修改 · 未发送"
        elif readback is not None and fresh and not state.get("dirty"):
            status = "与最近回读一致"
        elif matches_sent and sent.get("phase") == "pending":
            status = "请求待确认"
        elif matches_sent and sent.get("evidence") == "request_confirmed":
            status = "请求已确认 · 待回读"
        elif matches_sent and sent.get("phase") in ("error", "uncertain", "cancelled", "blocked", "invalid"):
            status = "请求结果未确认"
        elif not self.queryable:
            status = "仅本地草稿 · 无回读"
        elif readback is None:
            status = "本地默认 · 未回读"
        elif not fresh:
            status = "最近回读已过期"
        elif state.get("dirty"):
            status = "已修改 · 与回读不同"
        else:
            status = "与最近回读一致"
        self.status_label.setText(status)
        tone = parameter_tone(state, valid=numeric is not None)
        set_parameter_tone(self.status_label, tone)
        set_parameter_tone(self.readback_label, "success" if tone == "success" else "muted")
        set_parameter_tone(self.widget, tone)
        detail = f"本地编辑：{text}\n{self.readback_label.text()}\n{status}"
        if sent:
            detail += (f"\n最近请求值：{sent.get('value', '')}" +
                       ("（历史连接）" if not sent.get("fresh", False) else "") +
                       f"\n{sent.get('message', '')}")
        self.status_label.setToolTip("<qt>" + escape(detail).replace("\n", "<br>") + "</qt>")
        changed_style = False
        for name, value in (("parameterDirty", bool(state.get("dirty"))),
                            ("parameterRangeError", numeric is None)):
            if self.widget.property(name) != value:
                self.widget.setProperty(name, value)
                changed_style = True
        if changed_style:
            self.widget.style().unpolish(self.widget)
            self.widget.style().polish(self.widget)
        self.restore_button.setEnabled(self.queryable and self.node > 0 and readback is not None and self.applicable())
        self.restore_button.setToolTip("仅将本地编辑还原为最近回读值，不发送指令。" +
                                       ("该回读已过期。" if not fresh and readback is not None else ""))

    def restore(self):
        self.controller.parameter_edits.restore(self.family, self.node, [self.key])


class ParameterEditsUi:
    def __init__(self, controller, parent=None):
        self.controller = controller
        # Some views use a group restore button instead of each field's
        # individual button. Keep those unused controls owned and hidden;
        # QLayout reparents the controls that are actually displayed.
        self.stash = QWidget(parent)
        self.stash.setObjectName("ParameterEditorStash")
        self.stash.hide()
        self.bindings = []
        self.groups = []
        self.refreshing = self.closed = False
        controller.parameter_edits_changed.connect(self.refresh)

    def register(self, family, node, key, widget, default, **options):
        binding = ParameterBinding(self, family, node, key, widget, default, **options)
        self.bindings.append(binding)
        binding.refresh()
        return binding

    def register_group(self, button, family, node, keys):
        self.groups.append((button, family, node, tuple(keys)))
        self.refresh()

    def refresh(self):
        if self.closed or self.refreshing:
            return
        self.refreshing = True
        try:
            for binding in self.bindings:
                binding.refresh()
            for button, family, getter, keys in self.groups:
                node = int(getter() if callable(getter) else getter)
                usable = any(binding.family == family and binding.node == node and binding.key in keys and
                             binding.queryable and binding.applicable() and
                             binding.snapshot().get("readback") is not None for binding in self.bindings)
                button.setEnabled(node > 0 and usable)
        finally:
            self.refreshing = False
