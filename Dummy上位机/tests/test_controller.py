"""Regression tests for protocol framing, cancellation and numbered sequences."""
import queue
import threading
import time
import unittest
from copy import deepcopy
from dataclasses import FrozenInstanceError
from PySide6.QtCore import QCoreApplication
from app.core.controller import RobotController
from app.protocol.parser import LineFramer, finite_numbers


class ManualScheduler:
    def __init__(self):
        self.calls = {}
        self.index = 0

    def after(self, delay, callback, *args):
        self.index += 1
        self.calls[self.index] = (delay, callback, args)
        return self.index

    def after_cancel(self, token):
        self.calls.pop(token, None)

    def run_ready(self):
        for token, (delay, callback, args) in list(self.calls.items()):
            if delay <= 1:
                self.calls.pop(token)
                callback(*args)

    def close(self):
        self.calls.clear()


class FakePort:
    def __init__(self, *_args, **_kwargs):
        self.is_open = True
        self.written = []
        self.incoming = bytearray()
        self.lock = threading.Lock()

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.incoming)

    def inject(self, data):
        with self.lock:
            self.incoming.extend(data)

    def read(self, count):
        with self.lock:
            data = bytes(self.incoming[:count])
            del self.incoming[:count]
            return data

    def write(self, data):
        if not self.is_open:
            raise IOError("closed")
        with self.lock:
            self.written.append(data)
        return len(data)

    def close(self):
        self.is_open = False


class ControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.scheduler = ManualScheduler()
        self.controller = RobotController(scheduler=self.scheduler)
        self.controller.is_connected = True
        self.controller.serial_port = FakePort()
        self.logs = []
        self.controller.log_message.connect(lambda message, tag: self.logs.append((message, tag)))

    def tearDown(self):
        self.controller.close()

    def commands(self):
        results = []
        while True:
            try:
                item = self.controller._tx_queue.get_nowait()
                if item is not None:
                    results.append(item[1])
            except queue.Empty:
                return results

    def test_frame_fragments_cannot_finish_a_motion(self):
        framer = LineFramer()
        self.assertEqual(framer.feed(b"queued MOVEJ id=9 mode=SEQ\r\nok MOVE"),
                         ["queued MOVEJ id=9 mode=SEQ"])
        self.assertEqual(framer.feed(b"J id=9"), [])
        self.assertEqual(framer.feed(b"\nDisabled ok\n"), ["ok MOVEJ id=9", "Disabled ok"])
        self.assertIsNone(finite_numbers("nan 1"))
        self.assertIsNone(finite_numbers("1 inf"))

    def test_stop_discards_queued_motion_and_old_callbacks(self):
        c = self.controller
        self.assertTrue(c.send_cmd(">0,-75,180,0,0,0,0,35"))
        generation = c._tx_generation
        fired = []
        c._schedule_motion_callback(lambda: fired.append(True))
        self.assertTrue(c.send_cmd("!STOP"))
        self.assertEqual(self.commands(), ["!STOP"])
        self.assertFalse(c._can_write(generation, ">0,-75,180,0,0,0,0,35"))
        c._process_rx_line("Disabled ok")
        self.scheduler.run_ready()
        self.assertFalse(fired)
        self.assertFalse(c._stop_locked)
        self.assertFalse(c.movej_drag_enable.get())

    def test_calibration_blocks_motion_but_allows_diagnostics(self):
        c = self.controller
        self.assertTrue(c.send_cmd("!CALIBRATION"))
        self.assertFalse(c.send_cmd("!START"))
        self.assertFalse(c.send_cmd(">0,-75,180,0,0,0,0,35"))
        self.assertTrue(c.send_cmd("#CAL_STATUS 7 LAST"))
        c._process_rx_line("Disabled ok")
        self.assertTrue(c._calibration_busy)
        c._process_rx_line("!CALIBRATION ok")
        self.assertFalse(c._calibration_busy)
        self.assertEqual(float(c.ent_joints[1].get()), -75)
        self.assertEqual(c.hand_pos.get(), 100)

    def test_readonly_diagnostics_pass_stop_calibration_and_gamepad_gates(self):
        c = self.controller
        commands = ["#FW_VERSION 0", "#FW_VERSION 8 65535", "#CAN_DIAG 1", "#CAN_DIAG 8 1",
                    "#STALL_STATUS 7 LAST", "#CAL_STATUS 8 LAST", "#MOTION_STATUS", "#RGB_STATUS"]
        c._stop_locked = True
        c._calibration_busy = True
        c._control_owner = "gamepad"
        c._parameter_request = {"command": "#STALL_GET 1"}
        try:
            for connection in ("USB", "UART4"):
                c.connection_type = connection
                for command in commands:
                    with self.subTest(connection=connection, command=command):
                        generation = c._tx_generation
                        self.assertTrue(c.send_cmd(command))
                        self.assertEqual(c._tx_generation, generation)
                        self.assertTrue(c._can_write(generation, command))
                self.assertEqual(self.commands(), commands)
        finally:
            c._parameter_request = None

    def test_diagnostic_permission_does_not_unlock_motion_or_parameter_writes(self):
        c = self.controller
        c._stop_locked = True
        for command in (">0,-75,180,0,0,0,0,35", "#CMDMODE 2", "!RGB_BRIGHT 100",
                        "#STALL_SET 1 0 1", "#STACK", "#MOTION_STATUS extra",
                        "#STALL_STATUS 1 LAST extra", "#CAL_STATUS 8 LAST extra"):
            with self.subTest(command=command):
                self.assertFalse(c.send_cmd(command))
                self.assertFalse(c._can_write(c._tx_generation, command))
        self.assertEqual(self.commands(), [])

    def test_version_and_can_queries_require_valid_single_node_and_request_id(self):
        c = self.controller
        for connection in ("USB", "UART4"):
            c.connection_type = connection
            for command in ("#FW_VERSION", "#FW_VERSION 9", "#FW_VERSION 0 0", "#FW_VERSION 1 65536",
                            "#FW_VERSION 1 -1", "#CAN_DIAG 0", "#CAN_DIAG ALL", "#CAN_DIAG 8 1 extra",
                            "#CAN_DIAG 1 1\n!START"):
                with self.subTest(connection=connection, command=command):
                    self.assertFalse(c.send_cmd(command))
        self.assertEqual(self.commands(), [])

    def test_sequence_requires_matching_mode_request_and_motion_id(self):
        c = self.controller
        c.stored_positions = [("A", [0, -75, 180, 0, 0, 0, 0]),
                              ("B", [10, -70, 170, 0, 0, 0, 20])]
        c.send_all_positions()
        request = c._mode_request
        self.assertEqual(self.commands(), [f"#GET_CMDMODE {request}"])
        c._process_rx_line(f"ok CMDMODE mode=1 busy=0 issued=20 req={request + 1}")
        self.assertEqual(self.commands(), [])
        c._process_rx_line(f"ok CMDMODE mode=1 busy=0 issued=20 req={request}")
        self.assertEqual(self.commands(), [">0,-75,180,0,0,0,0,35"])
        c._process_rx_line("ok MOVEJ id=21")  # Completion without queued identity.
        self.assertEqual(c._seq_idx, 0)
        c._process_rx_line("queued MOVEJ id=20 mode=SEQ")  # Previous session.
        self.assertIsNone(c._seq_motion_id)
        c._process_rx_line("queued MOVEJ id=21 mode=SEQ")
        c._process_rx_line("ok MOVEJ id=19")
        self.assertEqual(c._seq_idx, 0)
        c._process_rx_line("ok MOVEJ id=21 ms=502")
        self.scheduler.run_ready()
        self.assertEqual(self.commands(), [">10,-70,170,0,0,0,20,35"])
        c._process_rx_line("queued MOVEJ id=22 mode=SEQ")
        c._process_rx_line("ok MOVEJ id=22")
        self.assertFalse(c._seq_active)
        self.assertEqual(c._seq_idx, 2)

    def test_wrong_actual_sequence_mode_triggers_stop(self):
        c = self.controller
        c.stored_positions = [("A", [0, -75, 180, 0, 0, 0, 0])]
        c.send_all_positions()
        c._process_rx_line(f"ok CMDMODE mode=1 busy=0 issued=4 req={c._mode_request}")
        self.commands()
        c._process_rx_line("queued MOVEJ id=5 mode=INT")
        self.assertEqual(self.commands(), ["!STOP"])
        self.assertFalse(c._seq_active)

    def test_explicit_and_selected_typed_points_send_only_motion_at_correct_speed(self):
        c = self.controller
        point = {"name": "TCP", "type": "MoveL", "pose": [180, 20, 300, 90, 0, -180],
                 "gripper": 25, "speed": 55}
        self.assertTrue(c.send_typed_position(point))
        self.assertEqual(self.commands(), ["@180,20,300,90,0,-180,55"])
        c.stored_positions = [point]
        c._pos_listbox.selected = 0
        c.ent_pos_speed.set("42")
        self.assertTrue(c.send_selected_position())
        self.assertEqual(self.commands(), ["@180,20,300,90,0,-180,42"])
        self.assertTrue(c.send_typed_position(("旧点", [0, -75, 180, 0, 0, 0, 12]), 24))
        self.assertEqual(self.commands(), [">0,-75,180,0,0,0,12,24"])
        c._pos_listbox.selected = 99
        self.assertFalse(c.send_selected_position())
        self.assertEqual(self.commands(), [])

    def test_mixed_sequence_requires_each_expected_type_and_exact_id(self):
        c = self.controller
        c.stored_positions = [
            {"name": "J", "joints": [0, -75, 180, 0, 0, 0, 20], "speed": 5},
            {"name": "L", "type": "MoveL", "pose": [200, 30, 300, 90, 0, 180], "speed": 80},
            ("旧 J", [10, -70, 170, 0, 0, 0, 30]),
        ]
        c.ent_pos_speed.set("40")
        self.assertTrue(c.send_all_positions())
        request = c._mode_request
        self.assertEqual(self.commands(), [f"#GET_CMDMODE {request}"])
        c._process_rx_line(f"ok CMDMODE mode=1 busy=0 issued=10 req={request}")
        self.assertEqual(self.commands(), [">0,-75,180,0,0,0,20,40"])
        self.assertEqual(c.state()["sequence_type"], "MOVEJ")
        c._process_rx_line("queued MOVEL id=11 mode=INT")
        self.assertIsNone(c._seq_motion_id)
        self.assertTrue(c._seq_active)
        c._process_rx_line("queued MOVEJ id=11 mode=SEQ")
        for receipt in ("ok MOVEL id=11", "error MOVEL id=11 reason=IK_FAILED", "ok MOVEJ id=12"):
            c._process_rx_line(receipt)
            self.assertEqual(c._seq_idx, 0)
            self.assertEqual(c._seq_motion_id, 11)
        c._process_rx_line("ok MOVEJ id=11 ms=250")
        self.scheduler.run_ready()
        self.assertEqual(self.commands(), ["@200,30,300,90,0,180,40"])
        self.assertEqual(c.state()["sequence_type"], "MOVEL")
        c._process_rx_line("queued MOVEJ id=12 mode=SEQ")
        c._process_rx_line("ok MOVEL id=12")  # Needs a matching queued identity first.
        self.assertIsNone(c._seq_motion_id)
        self.assertEqual(c._seq_idx, 1)
        c._process_rx_line("queued MOVEL id=12 mode=SEQ")
        c._process_rx_line("ok MOVEJ id=12")
        c._process_rx_line("error MOVEJ id=12 reason=FAILED")
        self.assertTrue(c._seq_active)
        self.assertEqual(c._seq_idx, 1)
        c._process_rx_line("ok MOVEL id=12 ms=600")
        self.scheduler.run_ready()
        self.assertEqual(self.commands(), [">10,-70,170,0,0,0,30,40"])
        c._process_rx_line("queued MOVEJ id=13 mode=SEQ")
        c._process_rx_line("ok MOVEJ id=13")
        self.assertFalse(c._seq_active)
        self.assertEqual(c._seq_idx, 3)
        self.assertIsNone(c.state()["sequence_type"])

    def test_matching_movel_failure_ends_sequence_without_next_motion(self):
        c = self.controller
        c.stored_positions = [
            {"name": "L", "type": "MoveL", "pose": [200, 0, 300, 0, 0, 0]},
            ("J", [0, -75, 180, 0, 0, 0, 0]),
        ]
        self.assertTrue(c.send_all_positions())
        c._process_rx_line(f"ok CMDMODE mode=1 busy=0 issued=20 req={c._mode_request}")
        self.commands()
        c._process_rx_line("queued MOVEL id=21 mode=SEQ")
        c._process_rx_line("error MOVEL id=22 reason=IK_FAILED")
        self.assertTrue(c._seq_active)
        c._process_rx_line("error MOVEL id=21 reason=IK_FAILED")
        self.scheduler.run_ready()
        self.assertFalse(c._seq_active)
        self.assertEqual(c._seq_idx, 0)
        self.assertEqual(self.commands(), [])

    def test_sequence_snapshot_is_immutable_when_editor_changes_points(self):
        c = self.controller
        records = [
            {"name": "J", "type": "MoveJ", "joints": [0, -75, 180, 0, 0, 0, 5],
             "gripper": 20, "speed": 10},
            {"name": "L", "type": "MoveL", "pose": [200, 30, 300, 90, 0, 180],
             "gripper": 80, "speed": 90},
        ]
        c.stored_positions = records
        c.ent_pos_speed.set("60")
        self.assertTrue(c.send_all_positions())
        self.assertIsInstance(c._seq_pending, tuple)
        with self.assertRaises(FrozenInstanceError):
            c._seq_pending[0].kind = "MOVEL"
        with self.assertRaises(TypeError):
            c._seq_pending[0].target[0] = 80
        records[0]["joints"][0] = 90
        records[0]["type"] = "MoveL"
        records[1]["pose"][:] = [1, 2, 3, 4, 5, 6]
        records.reverse()
        c.ent_pos_speed.set("99")
        request = c._mode_request
        self.commands()
        c._process_rx_line(f"ok CMDMODE mode=1 busy=0 issued=30 req={request}")
        self.assertEqual(self.commands(), [">0,-75,180,0,0,0,5,60"])
        records.clear()
        c._process_rx_line("queued MOVEJ id=31 mode=SEQ")
        c._process_rx_line("ok MOVEJ id=31")
        self.scheduler.run_ready()
        self.assertEqual(self.commands(), ["@200,30,300,90,0,180,60"])

    def test_stop_calibration_and_disconnect_cancel_scheduled_next_typed_point(self):
        for action in ("!STOP", "!CALIBRATION", "disconnect"):
            with self.subTest(action=action):
                self.controller.close()
                self.scheduler = ManualScheduler()
                c = self.controller = RobotController(scheduler=self.scheduler)
                c.is_connected = True
                c.serial_port = FakePort()
                c.stored_positions = [
                    ("J", [0, -75, 180, 0, 0, 0, 0]),
                    {"name": "L", "type": "MoveL", "pose": [200, 0, 300, 0, 0, 0]},
                ]
                self.assertTrue(c.send_all_positions())
                c._process_rx_line(f"ok CMDMODE mode=1 busy=0 issued=40 req={c._mode_request}")
                self.commands()
                c._process_rx_line("queued MOVEJ id=41 mode=SEQ")
                c._process_rx_line("ok MOVEJ id=41")  # Next send is scheduled, not run yet.
                callbacks = list(self.scheduler.calls.values())
                old_generation = c.state()["generation"]
                if action == "disconnect":
                    c.disconnect_port()
                else:
                    self.assertTrue(c.send_cmd(action))
                    self.assertEqual(self.commands(), [action])
                    if action == "!STOP":
                        c._process_rx_line("Disabled ok")
                    else:
                        c._process_rx_line("!CALIBRATION ok")
                self.assertGreater(c.state()["generation"], old_generation)
                self.assertFalse(c._seq_active)
                self.assertIsNone(c.state()["sequence_type"])
                c._process_rx_line("queued MOVEL id=42 mode=SEQ")
                c._process_rx_line("ok MOVEL id=42")
                # Even watchdogs/callbacks already dispatched before cancellation
                # must not revive the old sequence after the lock clears.
                for _delay, callback, args in callbacks:
                    callback(*args)
                self.assertEqual(self.commands(), [])

    def test_invalid_typed_point_prevents_all_queries_and_motion(self):
        c = self.controller
        valid = {"name": "J", "joints": [0, -75, 180, 0, 0, 0, 0]}
        invalid = [
            {"name": "bad", "type": "MoveX", "joints": valid["joints"]},
            {"name": "bad", "type": "MoveL", "joints": valid["joints"]},
            {"name": "bad", "joints": [0, -75, 180, 0, 0, 0]},
            {"name": "bad", "joints": [float("nan"), -75, 180, 0, 0, 0, 0]},
            {"name": "bad", "joints": [True, -75, 180, 0, 0, 0, 0]},
            {"name": "bad", "joints": [1000, -75, 180, 0, 0, 0, 0]},
            {"name": "bad", "type": "MoveL", "pose": [200, 0, float("inf"), 0, 0, 0]},
            {"name": "bad", "type": "MoveL", "pose": [10001, 0, 300, 0, 0, 0]},
            dict(valid, gripper=-1), dict(valid, gripper=101),
            dict(valid, speed=0), dict(valid, speed=float("inf")), dict(valid, speed=True),
        ]
        for record in invalid:
            with self.subTest(record=record):
                self.assertFalse(c.send_typed_position(record))
                c.stored_positions = [deepcopy(valid), record]
                self.assertFalse(c.send_all_positions())
                c._pos_listbox.selected = 1
                self.assertFalse(c.send_selected_position())
                self.assertEqual(self.commands(), [])
                self.assertFalse(c._seq_active)
                self.assertIsNone(c._mode_request)
        for speed in (True, float("nan"), float("inf"), 0, 101):
            self.assertFalse(c.send_typed_position(valid, speed))
        self.assertEqual(self.commands(), [])

    def test_explicit_position_sync_signal_ignores_poll_timeout_and_canceled_reply(self):
        c = self.controller
        synced = []
        c.position_synced.connect(lambda kind, values: synced.append((kind, values)))
        c._poll_position()
        c._process_rx_line("ok 0 -75 180 0 0 0 10")
        self.assertEqual(synced, [])
        self.assertTrue(c.read_and_sync_from_robot())
        c._process_rx_line("ok 1 -74 179 2 3 4 11")
        self.assertEqual(synced, [("MoveJ", [1, -74, 179, 2, 3, 4, 11])])
        self.assertTrue(c.read_pose_from_robot())
        c._process_rx_line("ok 5 -70 170 6 7 8 12")  # Wrong coordinate kind.
        self.assertEqual(len(synced), 1)
        c._process_rx_line("ok 200 30 300 90 0 -180")
        self.assertEqual(synced[-1], ("MoveL", [200, 30, 300, 90, 0, -180]))
        self.assertTrue(c.read_pose_from_robot())
        c._position_timeout(c._position_request["token"])
        c._process_rx_line("ok 201 31 301 91 1 -179")
        self.assertEqual(len(synced), 2)
        self.assertTrue(c.read_and_sync_from_robot())
        c.send_cmd("!STOP")
        c._process_rx_line("ok 6 -69 169 7 8 9 13")
        self.assertEqual(len(synced), 2)

    def test_polling_updates_actual_without_overwriting_target(self):
        c = self.controller
        actual = []
        c.actual_pose_changed.connect(actual.append)
        c._poll_position()
        self.assertEqual(self.commands(), ["#GETJPOS"])
        c._process_rx_line("ok 10 -60 150 20 10 3 35")
        self.assertEqual(actual[-1], [10, -60, 150, 20, 10, 3, 35])
        self.assertEqual(float(c.ent_joints[0].get()), 0)
        self.assertFalse(c.state()["actual_stale"])
        c.read_and_sync_from_robot()
        c._process_rx_line("ok 11 -61 151 21 11 4 36")
        self.assertEqual(float(c.ent_joints[0].get()), 11)
        self.assertEqual(float(c.ent_j7.get()), 36)

    def test_poll_frequency_bounds_live_update_and_disabled_poll(self):
        c = self.controller
        for hz, interval in ((1, 1000), (5, 200), (10, 100), (30, 34), (50, 20)):
            c.set_polling(True, hz)
            self.assertEqual(c._poll_hz, hz)
            self.assertEqual(c._poll_timer.interval(), interval)
            self.assertTrue(c._poll_timer.isActive())
        for bad in (0, 51, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                c.set_polling(True, bad)
        self.assertEqual(c._poll_hz, 50)
        c.set_polling(False, 30)
        c._poll_position()
        self.assertFalse(c._poll_timer.isActive())
        self.assertEqual(self.commands(), [])
        c.set_polling(True, 30)
        c._poll_position()
        for _ in range(30):
            c._poll_position()
        self.assertEqual(self.commands(), ["#GETJPOS"])

    def test_stop_cancels_pending_target_sync(self):
        c = self.controller
        c.read_and_sync_from_robot()
        c.send_cmd("!STOP")
        c._process_rx_line("ok 10 -60 150 20 10 3 35")
        self.assertEqual(float(c.ent_joints[0].get()), 0)
        self.assertIsNone(c._position_request)

    def test_uart4_missing_handlers_blocked_without_generic_ok(self):
        c = self.controller
        c.connection_type = "UART4"
        for command in ("#GET_PID 1", "#GETJACC 0", "#GETI 8", "#SYNC_ACC", "#REBOOT 0", "#ENC_CAL 1", "!HAND_VEL 5"):
            self.assertFalse(c.send_cmd(command), command)
        for command in ("#GETJPOS", "#GET_CMDMODE 1", "#RGB_GET", "!HAND_POS 100", "#STALL_GET 1"):
            self.assertTrue(c.send_cmd(command), command)
        self.assertEqual(len(self.commands()), 5)

    def test_parameter_get_pauses_poll_until_all_fields_and_nodes_confirmed(self):
        c = self.controller
        keys = {"ENABLE", "CURRENT_A", "SPEED_RPS", "ERROR_R", "TRIGGER_MS", "STARTUP_MS",
                "CURRENT_FILTER_MS", "SPEED_FILTER_MS", "BACK_RPS", "BACK_ACCEL",
                "BACK_CURRENT_A", "HOLD_CURRENT_A", "BACK_POS_TOL_R", "BACK_TIMEOUT_MS",
                "REBLOCK_MS", "GRANT_WAIT_MS", "MIN_POSITION_R", "MAX_POSITION_R", "DIRECTION_MASK"}
        self.assertTrue(c.send_cmd("#STALL_GET 0"))
        self.assertTrue(c.state()["parameter_busy"])
        self.assertEqual(self.commands(), ["#STALL_GET 0"])
        c._poll_position()
        self.assertEqual(self.commands(), [])  # Suspend before even pending ack.
        c._process_rx_line("ok STALL [1] ENABLE=1")
        self.assertEqual(c._parameter_request["fields"][1], set())
        c._process_rx_line("ok STALL_GET pending")
        c._process_rx_line("ok CAL [7] SEEK_RPS=5")  # Other family.
        c._process_rx_line("ok STALL [8] ENABLE=1")  # Excluded node.
        for node in range(1, 8):
            # Put final firmware field first to prove that one tail field
            # alone cannot prematurely complete a partial query.
            c._process_rx_line(f"ok STALL [{node}] DIRECTION_MASK=3")
            for key in sorted(keys - {"DIRECTION_MASK"}):
                c._process_rx_line(f"ok STALL [{node}] {key}=1")
            if node < 7:
                c._process_rx_line(f"ok STALL [{node}] BACK_DEG=5")
            self.assertTrue(c.state()["parameter_busy"])
        c._poll_position()
        self.assertEqual(self.commands(), [])
        c._process_rx_line("ok STALL [7] BACK_MM=5")
        self.assertFalse(c.state()["parameter_busy"])
        self.assertFalse(c._stop_locked)
        c._poll_position()
        self.assertEqual(self.commands(), ["#GETJPOS"])

    def test_parameter_set_all_distances_matches_node_subset_and_key(self):
        c = self.controller
        c._update_motor_state(False)
        self.assertTrue(c.send_cmd("#STALL_SET 0 BACK_DEG 5"))
        c._process_rx_line("ok STALL_SET pending")
        c._process_rx_line("ok STALL_SET [7] BACK_MM=5")
        c._process_rx_line("ok STALL_SET [1] CURRENT_A=5")
        self.assertEqual(c._parameter_request["completed_nodes"], set())
        for node in range(1, 6):
            c._process_rx_line(f"ok STALL_SET [{node}] BACK_DEG=5")
        self.assertTrue(c.state()["parameter_busy"])
        c._process_rx_line("ok STALL_SET [6] BACK_DEG=5")
        self.assertFalse(c.state()["parameter_busy"])
        self.assertTrue(c.send_cmd("#STALL_SET 0 BACK_MM 5"))
        c._process_rx_line("ok STALL_SET pending")
        c._process_rx_line("ok STALL_SET [1] BACK_DEG=5")
        self.assertTrue(c.state()["parameter_busy"])
        c._process_rx_line("ok STALL_SET [7] BACK_MM=5")
        self.assertFalse(c.state()["parameter_busy"])

    def test_parameter_save_all_failure_and_cal_matching(self):
        c = self.controller
        c._update_motor_state(False)
        self.assertTrue(c.send_cmd("#STALL_SAVE 0"))
        c._process_rx_line("ok STALL_SAVE pending")
        for node in range(1, 7):
            c._process_rx_line(f"ok STALL_SAVE [{node}] saved")
        self.assertTrue(c.state()["parameter_busy"])
        c._process_rx_line("ok STALL_SAVE [8] saved")
        self.assertTrue(c.state()["parameter_busy"])
        c._process_rx_line("ok STALL_SAVE [7] saved")
        self.assertFalse(c.state()["parameter_busy"])
        self.assertTrue(c.send_cmd("#CAL_SET 7 SEEK_RPS 5"))
        c._process_rx_line("ok CAL_SET pending")
        c._process_rx_line("error CAL_SET J8 TIMEOUT")
        self.assertTrue(c.state()["parameter_busy"])
        c._process_rx_line("error CAL_SET J7 REJECTED")
        self.assertFalse(c.state()["parameter_busy"])
        self.assertTrue(c._stop_locked)  # MCU locks after accepted write fails.

    def test_parameter_query_failure_timeout_and_old_timeout_are_isolated(self):
        c = self.controller
        self.assertTrue(c.send_cmd("#CAL_GET 7"))
        c._process_rx_line("error CAL_GET busy")
        self.assertFalse(c.state()["parameter_busy"])
        self.assertFalse(c._stop_locked)
        self.assertTrue(c.send_cmd("#CAL_GET 7"))
        c._process_rx_line("ok CAL_GET pending")
        token = c._parameter_request["timeout_token"]
        c._process_rx_line("ok CAL [7] SEEK_RPS=5")
        c._parameter_timeout(token)  # Old watchdog after new matching progress.
        self.assertTrue(c.state()["parameter_busy"])
        c._parameter_timeout(c._parameter_request["timeout_token"])
        self.assertFalse(c.state()["parameter_busy"])
        self.assertFalse(c._stop_locked)
        c._poll_position()
        self.assertEqual(self.commands()[-1], "#GETJPOS")

    def test_real_stop_lock_during_parameter_query_is_not_ignored(self):
        c = self.controller
        self.assertTrue(c.send_cmd("#STALL_GET 1"))
        c._process_rx_line("ok STALL_GET pending")
        c._process_rx_line("error STOP_LOCKED")
        self.assertTrue(c._stop_locked)
        self.assertFalse(c.state()["parameter_busy"])
        self.assertFalse(c.send_cmd(">0,-75,180,0,0,0,0,35"))

    def test_real_stall_event_cancels_parameter_poll_pause_and_locks_motion(self):
        c = self.controller
        self.assertTrue(c.send_cmd("#STALL_GET 1"))
        c._process_rx_line("ok STALL_GET pending")
        token = c._parameter_request["timeout_token"]
        c._process_rx_line("STALL J1 phase=4 reason=REBLOCKED")
        self.assertFalse(c.state()["parameter_busy"])
        self.assertTrue(c._stop_locked)
        c._parameter_timeout(token)
        self.assertTrue(c._stop_locked)
        c._poll_position()
        self.assertEqual(self.commands(), [])

    def test_reject_nonfinite_broadcast_parameter_writes_and_multiline(self):
        c = self.controller
        for command in (">nan,-75,180,0,0,0,0,35", ">0,-75,180,0,0,0,0,0",
                        "#SET_PID 0 1 2 3 4", "#ACC_J 1 inf", "#I_LIMIT_J 0 1",
                        "#ACC_J 1 nan", "#I_LIMIT_J 1 -inf", "!HAND_I nan",
                        "!HAND_ACC inf", "!HAND_VEL nan", "#STALL_SET 1 ERROR_R nan",
                        "#CAL_SET 7 CURRENT_A inf",
                        "!START\n!HOME", "!HAND_POS 1000"):
            self.assertFalse(c.send_cmd(command), command)
        self.assertEqual(self.commands(), [])

    def test_pid_accepts_full_uint32_range_and_rejects_overflow(self):
        c = self.controller
        c.ent_set_pid_kp.set("4294967295")
        c.ent_set_pid_kv.set("2147483648")
        c.ent_set_pid_ki.set("0")
        c.ent_set_pid_kd.set("1")
        c._set_pid()
        self.assertEqual(self.commands(), ["#SET_PID 1 4294967295 2147483648 0 1"])
        self.assertFalse(c.send_cmd("#SET_PID 1 4294967296 1 0 1"))
        self.assertFalse(c.send_cmd("#SET_PID 1 -1 1 0 1"))
        self.assertEqual(self.commands(), [])

    def test_pid_readback_decodes_signed_uint32_and_rejects_invalid_ranges(self):
        c = self.controller
        c._process_rx_line("ok PID [1] kp=-1 kv=-2147483648 ki=2147483648 kd=4294967295")
        self.scheduler.run_ready()
        fields = (c.ent_set_pid_kp, c.ent_set_pid_kv, c.ent_set_pid_ki, c.ent_set_pid_kd)
        expected = ["4294967295", "2147483648", "2147483648", "4294967295"]
        self.assertEqual([field.get() for field in fields], expected)
        c._set_pid()
        self.assertEqual(self.commands(), ["#SET_PID 1 4294967295 2147483648 2147483648 4294967295"])
        for response in ("ok PID [1] kp=-2147483649 kv=1 ki=2 kd=3",
                         "ok PID [1] kp=1 kv=4294967296 ki=2 kd=3"):
            c._process_rx_line(response)
            self.scheduler.run_ready()
            self.assertEqual([field.get() for field in fields], expected)

    def test_poll_frequency_contract_is_one_to_fifty_hz(self):
        c = self.controller
        for hz in (1, 5, 10, 20, 30, 50):
            c.set_polling(False, hz)
            self.assertEqual(c.state()["polling_hz"], hz)
        for hz in (0, 0.5, 50.1, 51, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                c.set_polling(False, hz)

    def test_gripper_pid_uses_int32_and_onboot_is_boolean(self):
        c = self.controller
        for key in ("KP", "KV", "KI", "KD"):
            self.assertTrue(c.send_cmd(f"!HAND_{key} 0"))
            self.assertTrue(c.send_cmd(f"!HAND_{key} 2147483647"))
            for value in ("nan", "inf", "-1", "1.5", "2147483648", "4294967295"):
                self.assertFalse(c.send_cmd(f"!HAND_{key} {value}"))
        for command in ("!HAND_ONBOOT", "!HAND_ONBOOT 0", "!HAND_ONBOOT 1"):
            self.assertTrue(c.send_cmd(command))
        for command in ("!HAND_ONBOOT nan", "!HAND_ONBOOT 2", "!HAND_ONBOOT -1", "!HAND_ONBOOT 1 0"):
            self.assertFalse(c.send_cmd(command))
        self.assertEqual(len(self.commands()), 11)

    def test_motor_parameter_save_commands_preserve_firmware_state_contract(self):
        c = self.controller
        for enabled in (None, True, False):
            c._update_motor_state(enabled)
            for command in ("#SET_PID 8 4294967295 0 0 0", "#ACC_J 8 10 &", "#I_LIMIT_J 8 1 &"):
                self.assertTrue(c.send_cmd(command), (command, enabled))
        self.assertEqual(len(self.commands()), 9)
        c.send_cmd("!STOP")
        for command in ("#SET_PID 8 1 0 0 0", "#ACC_J 8 10 &", "#I_LIMIT_J 8 1 &"):
            self.assertFalse(c.send_cmd(command))

    def test_parameter_replies_only_fill_matching_node(self):
        c = self.controller
        c.param_node.set(7)
        c._process_rx_line("ok PID [1] kp=123 kv=456 ki=7 kd=8")
        self.scheduler.run_ready()
        self.assertNotEqual(c.ent_set_pid_kp.get(), "123")
        c._process_rx_line("ok PID [7] kp=123 kv=456 ki=7 kd=8")
        self.scheduler.run_ready()
        self.assertEqual(c.ent_set_pid_kp.get(), "123")
        c._protection_nodes["STALL"].set(7)
        c._process_rx_line("ok STALL [7] BACK_DISTANCE_R=1.5")
        self.assertEqual(float(c._protection_entries["STALL"]["BACK_MM"].get()), 7.5)

    def test_rgb_state_and_effect_query_ranges(self):
        c = self.controller
        c._process_rx_line("RGB_COLOR 0 10 20 30")
        c._process_rx_line("RGB_COLOR 0 999 20 30")
        c._process_rx_line("RGB_STATE BOOT 3 0 20 30 40")
        c._process_rx_line("RGB_GET end")
        self.assertEqual([field.get() for field in c._rgb_color_vars], ["10", "20", "30"])
        self.assertEqual(c._rgb_state_mode.current(), 3)

    def test_motion_timeout_stops_task_and_device(self):
        c = self.controller
        c._seq_active = c._seq_running = True
        c._seq_session = 5
        c._seq_motion_id = 101
        c._motion_completion_timeout(4, 101)
        self.assertTrue(c._seq_active)
        c._motion_completion_timeout(5, 101)
        self.assertFalse(c._seq_active)
        self.assertEqual(self.commands(), ["!STOP"])

    def test_previous_connection_reply_is_ignored(self):
        c = self.controller
        c._connection_id = 3
        c._on_transport_line("Started ok", 2)
        self.assertFalse(c._motor_state_known)
        c._on_transport_line("Disabled ok", 3)
        self.assertTrue(c._motor_state_known)

    def test_fake_serial_threads_close_and_only_queries_on_connect(self):
        self.controller.close()
        port = FakePort()
        c = self.controller = RobotController(serial_factory=lambda *_args, **_kwargs: port)
        c.set_polling(False)
        actual = []
        c.actual_pose_changed.connect(actual.append)
        self.assertTrue(c.connect_port("FAKE", "USB"))
        limit = time.monotonic() + 1
        while len(port.written) < 2 and time.monotonic() < limit:
            self.app.processEvents()
            time.sleep(0.01)
        self.assertEqual(port.written[0], b"#RGB_GET\n")
        self.assertTrue(all(data.startswith(b"#") for data in port.written))
        port.inject(b"ok 0 -75 180 0 0 0 0\n")
        limit = time.monotonic() + 1
        while not actual and time.monotonic() < limit:
            self.app.processEvents()
            time.sleep(0.01)
        self.assertEqual(len(actual), 1)
        transport = c._transport
        c.disconnect_port()
        self.assertFalse(port.is_open)
        self.assertEqual(transport._threads, [])
        self.assertFalse(c.is_connected)


if __name__ == "__main__":
    unittest.main()
