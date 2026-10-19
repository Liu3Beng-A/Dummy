"""Check local color-dialog presentation and Qt's retained color behavior."""
import unittest
from unittest.mock import patch

from PySide6.QtCore import QCoreApplication, QTimer, Qt
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import (
    QApplication, QColorDialog, QLabel, QLineEdit, QPushButton, QSpinBox, QWidget,
)
from PySide6.QtTest import QTest

APPLICATION = QApplication.instance() or QApplication([])

from app.core.controller import RobotController
from app.ui.color_dialog import ChineseColorDialog, choose_color
from app.ui.widgets import ColorEditor


class ColorDialogTests(unittest.TestCase):
    def setUp(self):
        self.dialog = ChineseColorDialog(QColor(10, 20, 30))

    def tearDown(self):
        self.dialog.close()
        self.dialog.deleteLater()
        APPLICATION.processEvents()

    def spin(self, name):
        return self.dialog.findChild(QSpinBox, name)

    def finish_modal(self, accept, color):
        dialog = QApplication.activeModalWidget()
        self.assertIsInstance(dialog, ChineseColorDialog)
        dialog.setCurrentColor(color)
        if accept:
            dialog.accept()
        else:
            dialog.reject()

    def test_labels_and_screen_hint_are_localized_without_global_translation(self):
        before = QApplication.translate("QColorDialog", "&Basic colors")
        self.dialog.show()
        APPLICATION.processEvents()
        labels = {item.text() for item in self.dialog.findChildren(QLabel)
                  if item.isVisible()}
        self.assertTrue({"基础颜色", "自定义颜色", "色相 H：", "饱和度 S：",
                         "明度 V：", "红色 R：", "绿色 G：", "蓝色 B：",
                         "HEX 色值："}.issubset(labels))
        buttons = {item.text() for item in self.dialog.findChildren(QPushButton)
                   if item.isVisible()}
        self.assertTrue({"拾取屏幕颜色", "保存到自定义颜色", "确定", "取消"}
                        .issubset(buttons))
        self.assertTrue(self.dialog.testOption(
            QColorDialog.ColorDialogOption.DontUseNativeDialog))
        self.assertFalse(self.dialog.testOption(
            QColorDialog.ColorDialogOption.ShowAlphaChannel))
        cursor_label = next(item for item in self.dialog.findChildren(QLabel)
                            if item.text() == "\n")
        cursor_label.setText("Cursor at -10, 220\nPress ESC to cancel")
        # The native screen-picking timer can change this fixed-size label
        # without issuing a LayoutRequest. It must be localized before paint.
        cursor_label.grab()
        self.assertEqual(cursor_label.text(), "光标位置：-10，220\n按 Esc 取消取色")
        self.assertEqual(QApplication.translate("QColorDialog", "&Basic colors"), before)

    def test_rgb_hsv_and_hex_inputs_keep_native_synchronization(self):
        for name, value in zip(("ColorRed", "ColorGreen", "ColorBlue"), (255, 128, 0)):
            self.spin(name).setValue(value)
        self.assertEqual(self.dialog.currentColor().getRgb()[:3], (255, 128, 0))
        for name, value in zip(("ColorHue", "ColorSaturation", "ColorValue"), (300, 200, 200)):
            self.spin(name).setValue(value)
        self.assertEqual(self.dialog.currentColor().rgb(), QColor.fromHsv(300, 200, 200).rgb())
        field = self.dialog.findChild(QLineEdit, "qt_colorname_lineedit")
        field.selectAll()
        QTest.keyClicks(field, "#12abef")
        self.assertEqual(self.dialog.currentColor().name(), "#12abef")
        self.assertEqual([self.spin(name).value() for name in
                          ("ColorRed", "ColorGreen", "ColorBlue")], [18, 171, 239])

    def test_custom_colors_remain_available(self):
        previous = QColorDialog.customColor(0)
        try:
            self.dialog.setCurrentColor(QColor(23, 45, 67))
            save = next(item for item in self.dialog.findChildren(QPushButton)
                        if item.text() == "保存到自定义颜色")
            save.click()
            self.assertEqual(QColorDialog.customColor(0).getRgb()[:3], (23, 45, 67))
        finally:
            QColorDialog.setCustomColor(0, previous)

    def test_modal_confirmation_returns_color_and_cancel_returns_invalid(self):
        selected = QColor(200, 100, 50)
        QTimer.singleShot(0, lambda: self.finish_modal(True, selected))
        self.assertEqual(choose_color(QColor(10, 20, 30)), selected)
        QTimer.singleShot(0, lambda: self.finish_modal(False, selected))
        self.assertFalse(choose_color(QColor(10, 20, 30)).isValid())

    def test_color_editor_real_picker_confirms_and_preserves_values_on_cancel(self):
        editor = ColorEditor((10, 20, 30))
        try:
            QTimer.singleShot(0, lambda: self.finish_modal(False, QColor(255, 0, 0)))
            editor._pick()
            self.assertEqual([item.value() for item in editor.channels], [10, 20, 30])
            QTimer.singleShot(0, lambda: self.finish_modal(True, QColor(200, 100, 50)))
            editor._pick()
            self.assertEqual([item.value() for item in editor.channels], [200, 100, 50])
        finally:
            editor.close()
            editor.deleteLater()

    def test_controller_picker_updates_only_confirmed_fields_without_commands(self):
        controller = RobotController()
        changes = []
        controller.fields_changed.connect(lambda: changes.append(True))
        variables = controller._rgb_color_vars
        try:
            original = [item.get() for item in variables]
            with patch("app.ui.color_dialog.choose_color", return_value=QColor()):
                controller._rgb_pick_color(variables)
            self.assertEqual([item.get() for item in variables], original)
            self.assertEqual(changes, [])
            with patch("app.ui.color_dialog.choose_color", return_value=QColor(23, 45, 67)):
                controller._rgb_pick_color(variables)
            self.assertEqual([item.get() for item in variables], ["23", "45", "67"])
            self.assertEqual(changes, [True])
            self.assertTrue(controller._tx_queue.empty())
        finally:
            controller.close()

    def test_parent_window_theme_sets_readable_dialog_background(self):
        for theme, background in (("light", "#f2f5f9"), ("dark", "#111419")):
            with self.subTest(theme=theme):
                parent = QWidget()
                child = QWidget(parent)
                palette = QPalette(parent.palette())
                palette.setColor(QPalette.ColorRole.Window, QColor(background))
                parent.setPalette(palette)
                dialog = ChineseColorDialog(QColor(10, 20, 30), child)
                try:
                    dialog.show()
                    APPLICATION.processEvents()
                    self.assertEqual(dialog.theme, theme)
                    self.assertEqual(dialog.grab().toImage().pixelColor(3, 3).name(), background)
                    self.assertLessEqual(dialog.width(), 900)
                    self.assertLessEqual(dialog.height(), 600)
                    for label in dialog.findChildren(QLabel):
                        if label.isVisible() and "\n" not in label.text():
                            self.assertGreaterEqual(label.width(),
                                                    label.fontMetrics().horizontalAdvance(label.text()))
                finally:
                    dialog.close()
                    dialog.deleteLater()
                    parent.deleteLater()


if __name__ == "__main__":
    unittest.main()
