"""Capture all actual application pages without connecting any serial device."""
import argparse
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from PySide6.QtCore import QCoreApplication, QTimer, Qt
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
from PySide6.QtWidgets import QApplication, QScrollArea, QTabWidget
from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--theme", choices=("light", "dark"), default="light")
    parser.add_argument("--output", default="build/qa/pages")
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    application = QApplication([])
    store = AppStore(output / "data")
    store.update_settings(theme=args.theme, geometry="", window_state="")
    controller = RobotController()
    window = MainWindow(controller, store)
    window.resize(1366, 768)
    steps = [(f"{page}-{name}", lambda value=page: window.navigation.setCurrentRow(value))
             for page, name in enumerate(("motion", "teaching", "motor", "protection", "gripper", "rgb", "settings"))]

    def tab(page, index):
        window.navigation.setCurrentRow(page)
        widget = window.pages.widget(page).findChild(QTabWidget)
        widget.setCurrentIndex(index)

    steps += [("motion-movel", lambda: tab(0, 1)),
              ("protection-cal", lambda: tab(3, 1)),
              ("rgb-auto", lambda: tab(5, 1)),
              ("rgb-effects", lambda: tab(5, 2)),
              ("rgb-state", lambda: tab(5, 3))]

    def bottom(page):
        window.navigation.setCurrentRow(page)
        for area in window.pages.widget(page).findChildren(QScrollArea):
            area.verticalScrollBar().setValue(area.verticalScrollBar().maximum())

    steps += [("motor-bottom", lambda: bottom(2)), ("gripper-bottom", lambda: bottom(4))]

    def preview():
        tab(0, 0)
        for spin, value in zip(window.joint_spins, [25,-40,120,15,20,0,80]):
            spin.setValue(value)
        window.hand_row.spin.setValue(60)
        window.viewer.reset_camera()

    steps += [("motion-preview", preview), ("motion-terminal", lambda: window.log_dock.show())]
    progress = {"index": 0, "started": False}

    def capture():
        index = progress["index"]
        name = steps[index][0]
        path = output / f"{name}-{args.theme}.png"
        if not window.grab().save(str(path), "PNG"):
            application.exit(2)
            return
        print(f"CAPTURE {path.name}", flush=True)
        progress["index"] += 1
        QTimer.singleShot(50, advance)

    def advance():
        index = progress["index"]
        if index == len(steps):
            print(f"QA PASS: {index} pages; viewer ready; no serial connection", flush=True)
            application.exit(0)
            return
        steps[index][1]()
        QTimer.singleShot(500, capture)

    def ready():
        if not progress["started"]:
            progress["started"] = True
            QTimer.singleShot(1000, advance)

    window.viewer.ready.connect(ready)
    window.show()
    QTimer.singleShot(30000, lambda: application.exit(3) if not progress["started"] else None)
    try:
        return application.exec()
    finally:
        window.close()


if __name__ == "__main__":
    sys.exit(main())
