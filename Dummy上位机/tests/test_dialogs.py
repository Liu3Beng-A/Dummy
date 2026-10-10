"""Exercise dialog appearance and native Qt confirmation/cancel semantics."""
from pathlib import Path
import unittest

from PySide6.QtCore import QCoreApplication, QEvent, QTimer, Qt
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
from PySide6.QtGui import QPalette
from PySide6.QtWidgets import (
    QApplication, QDialogButtonBox, QInputDialog, QLabel, QMessageBox, QWidget,
)
from PySide6.QtTest import QTest

APPLICATION = QApplication.instance() or QApplication([])

from app.core import dialogs
from app.paths import resource_path


def contrast(foreground, background):
    def luminance(color):
        values = [value / 12.92 if value <= 0.04045 else ((value + 0.055) / 1.055) ** 2.4
                  for value in (color.redF(), color.greenF(), color.blueF())]
        return sum(value * weight for value, weight in zip(values, (0.2126, 0.7152, 0.0722)))
    light, dark = sorted((luminance(foreground), luminance(background)), reverse=True)
    return (light + 0.05) / (dark + 0.05)


class DialogAppearanceTests(unittest.TestCase):
    def setUp(self):
        self.previous_style = APPLICATION.styleSheet()
        self.appearance = dialogs.install_dialog_appearance()
        self.parent = QWidget()
        self.parent.show()
        APPLICATION.processEvents()
        self.output = Path(__file__).resolve().parents[1] / "build" / "qa" / "dialog-readability"
        self.output.mkdir(parents=True, exist_ok=True)
        self.callback_errors = []

    def tearDown(self):
        self.parent.close()
        self.parent.deleteLater()
        APPLICATION.setStyleSheet(self.previous_style)
        APPLICATION.processEvents()

    def theme(self, value):
        stylesheet = resource_path(f"themes/{value}.qss").read_text(encoding="utf-8")
        APPLICATION.setStyleSheet(stylesheet.replace("{{ICONS}}", resource_path("icons").as_posix()))
        APPLICATION.processEvents()

    def modal(self, action, expected_type=QMessageBox):
        def interact():
            dialog = QApplication.activeModalWidget()
            try:
                self.assertIsInstance(dialog, expected_type)
                action(dialog)
            except Exception as error:
                self.callback_errors.append(error)
                if dialog is not None:
                    dialog.reject()
        QTimer.singleShot(20, interact)

    def check_callback(self):
        if self.callback_errors:
            raise self.callback_errors.pop(0)

    def test_installation_is_application_owned_and_idempotent(self):
        self.assertIs(dialogs.install_dialog_appearance(), self.appearance)
        self.assertIs(self.appearance.parent(), APPLICATION)

    def test_message_icons_have_readable_themed_body_and_chinese_buttons(self):
        for theme, background in (("light", "#f2f5f9"), ("dark", "#142031")):
            self.theme(theme)
            for name, icon in (("information", QMessageBox.Icon.Information),
                               ("warning", QMessageBox.Icon.Warning),
                               ("critical", QMessageBox.Icon.Critical),
                               ("question", QMessageBox.Icon.Question)):
                with self.subTest(theme=theme, icon=name):
                    dialog = QMessageBox(self.parent)
                    dialog.setWindowTitle("模式提示")
                    dialog.setIcon(icon)
                    dialog.setText("拖发建议使用连续模式（模式 3）。\n"
                                   "当前为打断模式（INT）。\n\n请手动选择模式后继续。")
                    question = name == "question"
                    dialog.setStandardButtons(
                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
                        if question else QMessageBox.StandardButton.Ok)
                    dialog.setDefaultButton(QMessageBox.StandardButton.No
                                            if question else QMessageBox.StandardButton.Ok)
                    try:
                        dialog.show()
                        APPLICATION.processEvents()
                        image = dialog.grab().toImage()
                        self.assertEqual(image.pixelColor(2, 2).name(), background)
                        label = dialog.findChild(QLabel, "qt_msgbox_label")
                        self.assertEqual(label.font().pixelSize(), 15)
                        self.assertGreaterEqual(contrast(label.palette().color(QPalette.ColorRole.WindowText),
                                                         image.pixelColor(2, 2)), 7)
                        for button in dialog.buttons():
                            kind = dialog.standardButton(button)
                            self.assertEqual(button.text(), {
                                QMessageBox.StandardButton.Ok: "确定",
                                QMessageBox.StandardButton.Yes: "是",
                                QMessageBox.StandardButton.No: "否",
                            }[kind])
                        if question:
                            self.assertEqual(dialog.standardButton(dialog.defaultButton()),
                                             QMessageBox.StandardButton.No)
                        self.assertTrue(dialog.grab().save(str(self.output / f"{theme}-{name}.png")))
                    finally:
                        dialog.close()
                        dialog.deleteLater()

    def test_warning_and_error_adapters_keep_static_ok_semantics(self):
        self.theme("dark")
        for action in (dialogs.showwarning, dialogs.showerror):
            def accept(dialog):
                self.assertEqual(dialog.button(QMessageBox.StandardButton.Ok).text(), "确定")
                dialog.button(QMessageBox.StandardButton.Ok).click()
            self.modal(accept)
            action("参数提示", "参数值需要核对。")
            self.check_callback()

    def test_confirmation_preserves_no_default_yes_no_return_and_escape(self):
        for action, expected in (("yes", True), ("no", False), ("enter", False), ("escape", False)):
            with self.subTest(action=action):
                def answer(dialog, choice=action):
                    self.assertEqual(dialog.standardButton(dialog.defaultButton()),
                                     QMessageBox.StandardButton.No)
                    self.assertEqual(dialog.button(QMessageBox.StandardButton.Yes).text(), "是")
                    self.assertEqual(dialog.button(QMessageBox.StandardButton.No).text(), "否")
                    if choice in ("yes", "no"):
                        dialog.button(QMessageBox.StandardButton.Yes if choice == "yes"
                                      else QMessageBox.StandardButton.No).click()
                    else:
                        QTest.keyClick(dialog, Qt.Key.Key_Return if choice == "enter" else Qt.Key.Key_Escape)
                self.modal(answer)
                self.assertEqual(dialogs.askyesno("确认", "继续执行当前操作吗？"), expected)
                self.check_callback()

    def test_input_dialog_matches_themes_and_keeps_accept_cancel_escape(self):
        for theme, background in (("light", "#f2f5f9"), ("dark", "#142031")):
            self.theme(theme)
            for action in ("accept", "cancel", "escape"):
                with self.subTest(theme=theme, action=action):
                    def answer(dialog, choice=action):
                        self.assertEqual(dialog.grab().toImage().pixelColor(2, 2).name(), background)
                        buttons = dialog.findChild(QDialogButtonBox)
                        self.assertEqual(buttons.button(QDialogButtonBox.StandardButton.Ok).text(), "确定")
                        self.assertEqual(buttons.button(QDialogButtonBox.StandardButton.Cancel).text(), "取消")
                        if choice == "accept":
                            dialog.setTextValue("测试点位&1")
                            self.assertTrue(dialog.grab().save(str(self.output / f"{theme}-input.png")))
                            buttons.button(QDialogButtonBox.StandardButton.Ok).click()
                        elif choice == "cancel":
                            buttons.button(QDialogButtonBox.StandardButton.Cancel).click()
                        else:
                            QTest.keyClick(dialog, Qt.Key.Key_Escape)
                    self.modal(answer, QInputDialog)
                    value, accepted = QInputDialog.getText(self.parent, "保存当前点位", "点位名称：", text="起始点")
                    self.assertEqual(accepted, action == "accept")
                    if accepted:
                        self.assertEqual(value, "测试点位&1")
                    self.check_callback()

    def test_language_change_preserves_custom_buttons_and_question_default(self):
        dialog = QMessageBox(self.parent)
        dialog.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        dialog.setDefaultButton(QMessageBox.StandardButton.No)
        custom = dialog.addButton("查看原始日志", QMessageBox.ButtonRole.ActionRole)
        try:
            dialog.show()
            APPLICATION.processEvents()
            dialog.button(QMessageBox.StandardButton.Yes).setText("Yes")
            QCoreApplication.sendEvent(dialog, QEvent(QEvent.Type.LanguageChange))
            APPLICATION.processEvents()
            self.assertEqual(dialog.button(QMessageBox.StandardButton.Yes).text(), "是")
            self.assertEqual(dialog.standardButton(dialog.defaultButton()), QMessageBox.StandardButton.No)
            self.assertEqual(custom.text(), "查看原始日志")
        finally:
            dialog.close()
            dialog.deleteLater()


if __name__ == "__main__":
    unittest.main()
