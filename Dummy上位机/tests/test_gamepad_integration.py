"""End-to-end Qt handover through actual TX/RX workers, without hardware/SDL."""
from copy import deepcopy
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from PySide6.QtCore import QObject, QTimer, Qt, Signal
from PySide6.QtTest import QTest

from app.core.controller import RobotController
from app.gamepad.config import default_config
from app.gamepad.kinematics import forward_kinematics
from app.storage.store import AppStore
from app.ui.main_window import MainWindow
from tests.test_ui import APPLICATION, FakeView


class MemoryFirmwarePort:
    """Respond while write() is still running to exercise RX-before-written."""

    def __init__(self):
        self.lock = threading.Lock()
        self.rx = bytearray()
        self.writes = []
        self.closed = False
        self.mode, self.issued = 2, 40
        self.joints = [0., -75., 180., 0., 0., 0., 37.25]
        self.flange = [175., 187., -31., 90., -45., 180.]
        self.hold_stop = False
        self.joint_status_queries = 0

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.rx)

    def read(self, count):
        with self.lock:
            data = bytes(self.rx[:count])
            del self.rx[:count]
            return data

    def inject(self, *lines):
        with self.lock:
            self.rx.extend(("\n".join(lines) + "\n").encode("ascii"))

    def commands(self):
        with self.lock:
            return list(self.writes)

    def write(self, data):
        command = data.decode("ascii").strip()
        with self.lock:
            if self.closed:
                raise IOError("memory port closed")
            self.writes.append(command)
        if command == "!START":
            self.inject("Start pending", "Started ok")
        elif command.startswith("#GET_CMDMODE "):
            self.inject(f"ok CMDMODE mode={self.mode} busy=0 issued={self.issued} req={command.split()[1]}")
        elif command.startswith("#CMDMODE "):
            self.mode = int(command.split()[1])
            self.inject(f"ok CMDMODE mode={self.mode}")
        elif command == "#GETJPOS":
            self.inject("ok " + " ".join(map(str, self.joints)))
        elif command == "#GETLPOS":
            self.inject("ok " + " ".join(map(str, self.flange)))
        elif command == "#MOTION_STATUS":
            self.joint_status_queries += 1
            phase = 2 if self.joint_status_queries == 1 else 3
            self.inject(f"MOTION id={self.issued} kind=MOVEJ phase={phase} mode={self.mode} ms=80",
                        "MOTION ready=07F confirmed=07F progress=07F",
                        "MOTION send=7 query=7 reject=0")
        elif command.startswith((">", "@")):
            values = [float(value) for value in command[1:].split(",")]
            self.issued += 1
            kind = "MOVEJ" if command[0] == ">" else "MOVEL"
            self.inject(f"queued {kind} id={self.issued} mode={ {1: 'SEQ', 2: 'INT', 3: 'TRJ'}[self.mode]}")
            if kind == "MOVEJ":
                self.joints = values[:7]
                self.inject(f"accepted MOVEJ id={self.issued}" if self.mode == 2 else f"ok MOVEJ id={self.issued} ms=80")
            else:
                self.flange = values[:6]
                # Deterministic changing joint feedback, with no local IK or hardware.
                self.joints[0] += .05
                self.inject(f"accepted MOVEL id={self.issued}")
        elif command.startswith("!HAND_POS "):
            self.inject(f"ok hand pos={command.split()[1]}")
        elif command == "!STOP":
            self.inject("Stopping ok")
            if not self.hold_stop:
                self.inject("STOP locked")
        elif command == "#RGB_GET":
            self.inject("RGB_GET end")
        else:
            raise AssertionError(f"unexpected command: {command}")
        # The actual RX worker can publish responses before command_written.
        time.sleep(.06)
        return len(data)

    def close(self):
        self.closed = True

    def cancel_read(self):
        pass

    def cancel_write(self):
        pass


class TimerReader(QObject):
    sampled = Signal(dict)
    status_changed = Signal(dict)

    def __init__(self):
        super().__init__()
        self.running = self.closed = False
        self.config = default_config()
        self.decoded = {"lx": 0., "ly": 0., "ry": 0., "lt": 0., "rt": 0.,
                        "roll": 0, "hat": (0, 0), "centered": True}
        self.timer = QTimer(self)
        self.timer.setInterval(17)
        self.timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.timer.timeout.connect(self.sample)
        self.sample_count = 0

    def start(self, config=None):
        self.config = deepcopy(config or self.config)
        self.running = True
        self.status_changed.emit(self.status())
        self.sample()
        self.timer.start()
        return True

    def sample(self):
        self.sample_count += 1
        self.sampled.emit({"connected": self.running, "timestamp": time.monotonic(),
                           "decoded": deepcopy(self.decoded), "pressed": [], "error": "",
                           "raw": {"name": "60Hz fake reader", "axes": [self.decoded["ly"]],
                                   "buttons": [False] * 6, "hats": [(0, 0)]}})

    def status(self):
        return {"running": self.running, "connected": self.running,
                "name": "60Hz fake reader", "guid": "memory", "devices": [], "error": ""}

    def set_config(self, config):
        self.config = deepcopy(config)

    def stop(self):
        self.timer.stop()
        self.running = False
        self.status_changed.emit(self.status())

    def close(self):
        self.stop()
        self.closed = True


class GamepadThreadedIntegrationTests(unittest.TestCase):
    def setUp(self):
        base = Path(__file__).resolve().parents[1] / "build" / "test-data"
        base.mkdir(parents=True, exist_ok=True)
        self.directory = tempfile.TemporaryDirectory(dir=base)
        self.store = AppStore(self.directory.name)
        config = default_config()
        config["robot"]["tool_offset_z"] = 0
        config["control"]["linear_speed_mm_s"] = 10
        self.store.update_settings(gamepad_config=config, auto_poll=True, poll_hz=5)
        self.port = MemoryFirmwarePort()
        self.factory_calls = []

        def factory(*args, **kwargs):
            self.factory_calls.append((args, kwargs))
            return self.port

        self.controller = RobotController(serial_factory=factory)
        self.reader = TimerReader()
        self.patches = [patch("app.ui.main_window.RobotView", FakeView),
                        patch("app.core.controller.serial.tools.list_ports.comports", return_value=[]),
                        patch("app.core.controller.serial.Serial", side_effect=AssertionError("real serial forbidden")),
                        patch("app.ui.main_window.GamepadReader", side_effect=AssertionError("SDL reader forbidden"))]
        for replacement in self.patches:
            replacement.start()
        self.window = MainWindow(self.controller, self.store, gamepad_reader=self.reader)
        self.order = []

    def tearDown(self):
        self.port.hold_stop = False
        self.port.inject("STOP locked")
        self.window.close()
        self.wait_for(lambda: self.window._closed, "window cleanup", timeout=1.5)
        self.controller.close()
        self.reader.close()
        self.window.deleteLater()
        APPLICATION.processEvents()
        self.store.close()
        for replacement in reversed(self.patches):
            replacement.stop()
        self.directory.cleanup()

    def wait_for(self, predicate, message, timeout=4.):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            QTest.qWait(10)
            APPLICATION.processEvents()
        self.assertTrue(predicate(), f"{message}: session={self.controller.gamepad.state()}, commands={self.port.commands()}")

    def through_ui_to_ready(self):
        self.assertEqual(self.port.commands(), [])
        self.assertFalse(self.reader.running)
        self.window.port_combo.setEditText("MEMORY_ONLY")
        self.window.connect_button.click()
        self.assertEqual(len(self.factory_calls), 1)
        self.transport = self.controller._transport
        self.worker_threads = list(self.transport._threads)
        self.transport.line_received.connect(lambda line, _connection: self.order.append(("rx", line)), Qt.ConnectionType.QueuedConnection)
        self.transport.command_written.connect(lambda command, _connection: self.order.append(("tx", command)), Qt.ConnectionType.QueuedConnection)
        self.wait_for(lambda: self.controller._mode_request is None and self.controller._position_request is None,
                      "initial device queries complete")
        self.assertTrue(self.controller._request_position("j", False))
        self.wait_for(lambda: self.controller._position_request is None and self.controller.actual_pose is not None,
                      "actual pose captured before gamepad ownership")
        self.window.input_mode.setCurrentIndex(1)
        panel = self.window.gamepad_panel
        panel.check_button.click()
        self.wait_for(lambda: self.controller.gamepad.device_connected, "UI input inspection")
        self.assertFalse(self.controller.gamepad.claimed)
        panel.claim_button.click()
        self.wait_for(lambda: self.controller.gamepad.claimed, "UI control claim")
        self.claim_commands = len(self.port.commands())
        self.last_actual = deepcopy(self.controller.actual_pose)
        self.last_view_actual = deepcopy(self.window.viewer.actual)
        panel.action_buttons["start"].click()
        self.wait_for(lambda: self.controller.gamepad.stage == "enabled", "first Start enable")
        self.assertFalse(any(command.startswith(">") for command in self.port.commands()))
        before_init = len(self.port.commands())
        panel.action_buttons["start"].click()
        self.wait_for(lambda: self.controller.gamepad.stage == "init_confirmation", "second Start single MoveJ acceptance")
        QTest.qWait(300)
        self.assertEqual(self.port.commands()[before_init:], [">0,10,80,0,90,0,0,10"])
        self.assertEqual(panel.action_buttons["initialize"].text(), "确认到位")
        self.assertFalse(self.controller.gamepad.ready)
        panel.action_buttons["initialize"].click()
        self.wait_for(lambda: self.controller.gamepad.ready, "local arrival confirmation and centered unlock")
        self.assertEqual(self.port.commands()[before_init:], [">0,10,80,0,90,0,0,10"])
        commands = self.port.commands()
        self.assertEqual(commands.count("!START"), 1)
        joint = next(command for command in commands if command.startswith(">"))
        self.assertEqual(joint, ">0,10,80,0,90,0,0,10")
        self.assertEqual(len((joint + "\n").encode("ascii")), 21)
        self.assertNotIn("#CMDMODE 1", commands)
        self.assertEqual(self.port.joint_status_queries, 0)
        self.assertEqual([float(value) for value in joint[1:].split(",")], [0, 10, 80, 0, 90, 0, 0, 10])
        self.assertLess(self.order.index(("rx", "Start pending")), self.order.index(("tx", "!START")))
        self.assertLess(self.order.index(("rx", "queued MOVEJ id=41 mode=INT")), self.order.index(("tx", joint)))
        self.assertLess(self.order.index(("rx", "accepted MOVEJ id=41")), self.order.index(("tx", joint)))
        self.assertNotIn("#GETLPOS", commands)
        self.assertIsNone(self.controller.gamepad.actual_tcp)
        self.assertEqual(self.controller.gamepad.command_reference_tcp,
                         forward_kinematics([0, 10, 80, 0, 90, 0, 0]))
        self.assertEqual(self.controller.actual_pose, self.last_actual)
        self.assertEqual(self.window.viewer.actual, self.last_view_actual)
        self.assertFalse(any(command in ("#GETJPOS", "#GETLPOS", "#GETRAILPOS")
                             for command in commands[self.claim_commands:]))
        self.assertTrue(all(thread.is_alive() for thread in self.worker_threads))

    def test_ui_threaded_handover_motion_gripper_conflicts_and_release(self):
        self.through_ui_to_ready()
        self.reader.decoded.update(ly=-1., centered=False)
        self.wait_for(lambda: any(command.startswith("@") for command in self.port.commands()), "MoveL from 60Hz input")
        self.reader.decoded.update(ly=0., centered=True)
        self.wait_for(lambda: self.controller.gamepad._stream is None, "accepted MoveL releases in-flight slot")
        self.assertEqual(self.controller.actual_pose, self.last_actual)
        self.assertEqual(self.window.viewer.actual, self.last_view_actual)
        self.assertIsNone(self.controller.gamepad.actual_tcp)
        self.assertFalse(any(command in ("#GETJPOS", "#GETLPOS", "#GETRAILPOS")
                             for command in self.port.commands()[self.claim_commands:]))
        self.assertIsNone(self.window.viewer.cartesian)
        self.assertNotIn("#GETLPOS", self.port.commands())
        self.assertIsNone(self.controller.gamepad.gripper_target)
        panel = self.window.gamepad_panel
        panel.gripper_base_spin.setValue(40)
        panel.gripper_base_button.click()
        self.wait_for(lambda: "!HAND_POS 40" in self.port.commands() and self.controller.gamepad._hand is None,
                      "explicit gripper base is sent and accepted")
        self.assertEqual(self.window.hand_row.spin.value(), 40)
        self.assertTrue(self.window.viewer.actual_only)
        self.assertNotEqual(self.window.viewer.target[1], 40)
        self.assertEqual(self.window.viewer.actual[1], None)
        self.assertGreater(self.reader.sample_count, 5)
        forbidden = ["!HOME", "!RESET", "#CMDMODE 3", "!HAND_POS 0", "!CALIBRATION", ">0,-75,180,0,0,0,0,10"]
        for command in forbidden:
            self.assertFalse(self.controller.send_cmd(command), command)
        self.assertFalse(self.window.movej_send_button.isEnabled())
        self.window.movej_send_button.click()
        QTest.qWait(80)
        self.assertFalse(any(command in self.port.commands() for command in forbidden))
        self.port.hold_stop = True
        panel.release_button.click()
        panel.release_button.click()
        self.wait_for(lambda: "!STOP" in self.port.commands() and self.controller.gamepad._system["written"],
                      "one STOP reaches writer")
        self.assertEqual(self.port.commands().count("!STOP"), 1)
        self.assertTrue(self.controller.gamepad.release_pending)
        self.assertEqual(self.controller.state()["control_owner"], "gamepad")
        self.assertFalse(self.reader.running)
        self.assertFalse(self.controller.send_cmd("!HOME"))
        self.assertFalse(any(command in ("#GETJPOS", "#GETLPOS", "#GETRAILPOS")
                             for command in self.port.commands()[self.claim_commands:]))
        self.port.inject("STOP locked")
        self.wait_for(lambda: not self.controller.gamepad.claimed, "STOP confirmation releases owner")
        self.assertEqual(self.controller.state()["control_owner"], "ui")
        self.assertTrue(self.window.movej_send_button.isEnabled())
        self.window.close()
        self.wait_for(lambda: self.window._closed, "released window close")
        self.assertTrue(self.reader.closed)
        self.assertTrue(self.port.closed)
        self.assertTrue(self.window.viewer.closed)
        self.assertTrue(all(not thread.is_alive() for thread in self.worker_threads))

    def test_close_claimed_session_waits_for_real_stop_then_closes_workers(self):
        self.through_ui_to_ready()
        self.window.close()
        self.assertTrue(self.window._gamepad_close_pending)
        self.assertFalse(self.window._closed)
        self.wait_for(lambda: self.window._closed, "close waits for worker STOP completion")
        self.assertEqual(self.port.commands().count("!STOP"), 1)
        self.assertFalse(self.controller.is_connected)
        self.assertTrue(self.reader.closed)
        self.assertTrue(self.port.closed)
        self.assertTrue(all(not thread.is_alive() for thread in self.worker_threads))


if __name__ == "__main__":
    unittest.main()
