"""Capture all actual application pages without connecting any serial device."""
import argparse
from pathlib import Path
import sys

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from PySide6.QtCore import QCoreApplication, QTimer, Qt
QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
from PySide6.QtWidgets import QApplication, QScrollArea
from app.core.controller import RobotController
from app.storage.store import AppStore
from app.ui.main_window import MainWindow


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--theme", choices=("light", "dark"), default="light")
    parser.add_argument("--output", default="build/qa/pages")
    parser.add_argument("--size", default="1366x768")
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    application = QApplication([])
    store = AppStore(output / "data")
    store.update_settings(theme=args.theme, geometry="", window_state="")
    store.save_points([
        {"name": "关节取料点", "type": "MoveJ", "joints": [25, -40, 120, 15, 20, 0, 80], "gripper": 60, "speed": 35},
        {"name": "末端放料点", "type": "MoveL", "pose": [245, -20, 220, 180, 0, 90], "gripper": 35, "speed": 25},
        {"name": "关节休息点", "type": "MoveJ", "joints": [0, -75, 180, 0, 0, 0, 0], "gripper": 100, "speed": 20},
    ])
    controller = RobotController()
    window = MainWindow(controller, store)
    width, height = map(int, args.size.lower().split("x"))
    window.resize(width, height)
    steps = [(f"{page}-{name}", lambda value=page: window.navigation.setCurrentRow(value))
             for page, name in enumerate(("motion", "teaching", "motor", "gripper", "rgb", "settings"))]

    def tab(page, widget, index):
        window.navigation.setCurrentRow(page)
        widget.setCurrentIndex(index)

    def protection(index):
        tab(2, window.motor_tabs, 1)
        window.protection_tabs.setCurrentIndex(index)

    steps += [("motion-movel", lambda: tab(0, window.motion_tabs, 1)),
              ("motor-stall", lambda: protection(0)),
              ("motor-cal", lambda: protection(1)),
              ("rgb-auto", lambda: tab(4, window.rgb_tabs, 1)),
              ("rgb-effects", lambda: tab(4, window.rgb_tabs, 2)),
              ("rgb-state", lambda: tab(4, window.rgb_tabs, 3))]

    def teaching(index):
        window.navigation.setCurrentRow(1)
        window.point_table.selectRow(index)

    steps += [("teaching-movej", lambda: teaching(0)),
              ("teaching-movel", lambda: teaching(1))]

    def scroll_bottom(widget):
        for area in widget.findChildren(QScrollArea):
            if area.isVisible():
                area.verticalScrollBar().setValue(area.verticalScrollBar().maximum())

    def motor_bottom(index, protection_index=0):
        tab(2, window.motor_tabs, index)
        if index == 1:
            window.protection_tabs.setCurrentIndex(protection_index)
        # Let the selected subpage settle before positioning its scroll bars.
        QTimer.singleShot(50, lambda: scroll_bottom(window.motor_tabs.currentWidget()))

    def gripper_bottom():
        window.navigation.setCurrentRow(3)
        QTimer.singleShot(50, lambda: scroll_bottom(window.pages.currentWidget()))

    steps += [("motor-bottom", lambda: motor_bottom(0)),
              ("motor-stall-bottom", lambda: motor_bottom(1, 0)),
              ("motor-cal-bottom", lambda: motor_bottom(1, 1)),
              ("gripper-bottom", gripper_bottom)]

    def preview():
        tab(0, window.motion_tabs, 0)
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
        snapshot = window.grab()
        if index == 0:
            print(f"QA GEOMETRY: {window.width()}x{window.height()} logical; "
                  f"DPR {snapshot.devicePixelRatio():.3f}; "
                  f"PNG {snapshot.width()}x{snapshot.height()}", flush=True)
        if not snapshot.save(str(path), "PNG"):
            application.exit(2)
            return
        print(f"CAPTURE {path.name}", flush=True)
        progress["index"] += 1
        QTimer.singleShot(50, advance)

    def advance():
        index = progress["index"]
        if index == len(steps):
            if controller.is_connected or controller._transport is not None or not controller._tx_queue.empty():
                print("QA FAIL: offline capture dispatched a robot command", flush=True)
                application.exit(4)
                return
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
