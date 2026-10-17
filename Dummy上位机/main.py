"""Dummy Robot application entry point. Never connects or moves on startup."""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import traceback


def main():
    parser = argparse.ArgumentParser(description="Dummy Robot 上位机")
    parser.add_argument("--data-dir", help="本地配置和日志目录（默认 LOCALAPPDATA/DummyRobot）")
    parser.add_argument("--theme", choices=("light", "dark"))
    parser.add_argument("--size", help="窗口尺寸，例如 1366x768")
    parser.add_argument("--snapshot", help="保存主窗口截图的 PNG 路径")
    parser.add_argument("--quit-after", type=int, default=0, help="自动退出时间，单位毫秒")
    parser.add_argument("--smoke-test", action="store_true", help="验证模型加载后截图并退出；不连接设备")
    args = parser.parse_args()

    from PySide6.QtCore import QCoreApplication, Qt, QTimer
    QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts)
    from PySide6.QtWidgets import QApplication, QMessageBox
    from PySide6.QtGui import QIcon
    from app.core.controller import RobotController
    from app.storage.store import AppStore
    from app.ui.main_window import MainWindow
    from app.visualization.robot_view import RobotView
    from app.paths import resource_path

    application = QApplication(sys.argv[:1])
    application.setOrganizationName("DummyRobot")
    application.setApplicationName("Dummy Robot 上位机")
    application.setApplicationVersion("0.3.0")
    application.setWindowIcon(QIcon(str(resource_path("icons", "dummy.ico"))))
    store = AppStore(args.data_dir)
    if args.theme:
        store.update_settings(theme=args.theme)
    controller = RobotController()
    controller.wire_log_sink = store.log
    controller.log_message.connect(lambda text, tag: store.log(tag, text))
    window = MainWindow(controller, store)
    if args.size:
        try:
            width, height = map(int, args.size.lower().split("x"))
            if width < 640 or height < 480:
                raise ValueError()
            window.resize(width, height)
        except ValueError:
            parser.error("--size 必须为至少 640x480 的宽x高")

    def report_exception(exc_type, exc_value, exc_tb):
        detail = "".join(traceback.format_exception(exc_type, exc_value, exc_tb))
        store.log("ERROR", detail)
        if not args.smoke_test:
            QMessageBox.critical(window, "程序异常", f"{exc_value}\n\n详细信息已写入日志：{store.log_dir}")
        else:
            print(detail, file=sys.stderr)
            application.exit(2)

    sys.excepthook = report_exception
    outcome = {"ready": False, "snapshot": False, "error": ""}

    def snapshot():
        if args.snapshot:
            destination = Path(args.snapshot).resolve()
            destination.parent.mkdir(parents=True, exist_ok=True)
            if not window.grab().save(str(destination), "PNG"):
                outcome["error"] = f"截图保存失败：{destination}"
                print(outcome["error"], file=sys.stderr)
                application.exit(2)
                return
            outcome["snapshot"] = True
            print(f"SNAPSHOT {destination}", flush=True)
        if args.smoke_test:
            print("SMOKE PASS: offline URDF loaded; no hardware connected", flush=True)
            application.exit(0)

    def viewer_ready():
        if outcome["ready"]:
            return
        outcome["ready"] = True
        QTimer.singleShot(1000, snapshot)

    for viewer in window.findChildren(RobotView):
        viewer.ready.connect(viewer_ready)
        viewer.status_changed.connect(lambda message: store.log("VIEWER", message))
    window.show()
    if args.smoke_test:
        def timed_out():
            if not outcome["ready"]:
                print("SMOKE FAIL: URDF viewer did not become ready within 30 seconds", file=sys.stderr)
                application.exit(3)
        QTimer.singleShot(30000, timed_out)
    elif args.snapshot:
        QTimer.singleShot(12000, snapshot)
    if args.quit_after:
        QTimer.singleShot(args.quit_after, application.quit)
    try:
        result = application.exec()
    finally:
        controller.close()
        for viewer in window.findChildren(RobotView):
            viewer.cleanup()
        store.close()
    if args.smoke_test and not outcome["ready"] and result == 0:
        return 3
    return result


if __name__ == "__main__":
    import multiprocessing
    multiprocessing.freeze_support()
    sys.exit(main())
