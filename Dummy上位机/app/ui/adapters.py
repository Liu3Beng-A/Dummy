"""Small UI adapters for the proven serial assistant's business methods.

Widgets live on the Qt GUI thread. These adapters deliberately do not create
timers or dispatch serial commands when controller values are refreshed.
"""
from PySide6.QtCore import QSignalBlocker
from PySide6.QtWidgets import (
    QAbstractButton, QComboBox, QDoubleSpinBox, QLabel, QLineEdit, QSlider,
    QSpinBox, QTableWidgetItem,
)


class FieldAdapter:
    def __init__(self, widget, cast=None, scale=1):
        self.widget = widget
        self.cast = cast
        self.scale = scale

    def get(self):
        w = self.widget
        if isinstance(w, QComboBox):
            value = w.currentText()
        elif isinstance(w, (QSpinBox, QDoubleSpinBox)):
            value = w.value()
        elif isinstance(w, QSlider):
            value = w.value() / self.scale
        elif isinstance(w, QAbstractButton) and w.isCheckable():
            value = w.isChecked()
        else:
            value = w.text()
        return self.cast(value) if self.cast else value

    def set(self, value):
        w = self.widget
        if isinstance(w, QComboBox):
            text = str(value)
            index = w.findText(text)
            if index >= 0:
                w.setCurrentIndex(index)
            elif w.isEditable():
                w.setEditText(text)
        elif isinstance(w, QSpinBox):
            w.setValue(int(float(value)))
        elif isinstance(w, QDoubleSpinBox):
            w.setValue(float(value))
        elif isinstance(w, QSlider):
            w.setValue(round(float(value) * self.scale))
        elif isinstance(w, QAbstractButton) and w.isCheckable():
            w.setChecked(bool(value))
        else:
            w.setText(str(value))

    def delete(self, start=0, end=None):
        if isinstance(self.widget, QLineEdit):
            self.widget.clear()

    def insert(self, index, value):
        self.set(value)

    def current(self, index=None):
        if index is None:
            return self.widget.currentIndex()
        self.widget.setCurrentIndex(int(index))

    def config(self, **kwargs):
        w = self.widget
        if "text" in kwargs:
            w.setText(str(kwargs["text"]))
        if "state" in kwargs:
            w.setEnabled(kwargs["state"] not in ("disabled", False))
        if "values" in kwargs and isinstance(w, QComboBox):
            text = w.currentText()
            blocker = QSignalBlocker(w)
            w.clear()
            w.addItems([str(value) for value in kwargs["values"]])
            if text in kwargs["values"]:
                w.setCurrentText(text)
            del blocker
        # Old color values are intentionally represented by application style
        # properties; they must not overwrite a whole Qt theme on every reply.
        if "fg" in kwargs and isinstance(w, QLabel):
            w.setProperty("statusColor", str(kwargs["fg"]))

    configure = config

    def trace_add(self, mode, callback):
        w = self.widget
        if isinstance(w, QComboBox):
            signal = w.currentIndexChanged
        elif isinstance(w, QAbstractButton):
            signal = w.toggled
        elif isinstance(w, (QSpinBox, QDoubleSpinBox, QSlider)):
            signal = w.valueChanged
        else:
            signal = w.textChanged
        signal.connect(lambda *_: callback(None, None, mode))

    def __getitem__(self, key):
        return self.get() if key == "text" else None

    def __setitem__(self, key, value):
        self.config(**{key: value})


class TableAdapter:
    """Compatibility with the old position list; UI owns rich table contents."""
    def __init__(self, table):
        self.widget = table

    def curselection(self):
        rows = self.widget.selectionModel().selectedRows()
        return tuple(sorted(index.row() for index in rows))

    def delete(self, start, end=None):
        if end == "end":
            self.widget.setRowCount(0)
        else:
            self.widget.removeRow(int(start))

    def insert(self, index, text):
        row = self.widget.rowCount() if index == "end" else int(index)
        self.widget.insertRow(row)
        self.widget.setItem(row, 0, QTableWidgetItem(str(text)))

    def size(self):
        return self.widget.rowCount()
