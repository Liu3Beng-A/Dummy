"""Offline control-session tests using real Qt/controller and a fake writer."""
import math
import json
import queue
import time
import unittest

from PySide6.QtCore import QCoreApplication

from app.core.controller import RobotController
from tests.test_controller import FakePort, ManualScheduler


class Clock:
    def __init__(self):
        self.now = time.monotonic()

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class GamepadFixture:
    def setup_gamepad(self):
        self.clock = Clock()
        self.c = RobotController(scheduler=ManualScheduler())
        self.c.is_connected = True
        self.c.serial_port = FakePort()
        self.g = self.c.gamepad
        self.g.clock = self.clock
        self.g.set_config({"robot": {"tool_offset_z": 0, "cmd_speed": 10},
                           "control": {"linear_speed_mm_s": 10, "angular_speed_deg_s": 5,
                                       "gripper_speed_percent_s": 30, "send_interval_s": .05,
                                       "velocity_smoothing_s": 0}})
        self.writes = []
        self.sample()

    def sample(self, pressed=(), **changes):
        decoded = {"lx": 0., "ly": 0., "ry": 0., "lt": 0., "rt": 0.,
                   "roll": 0, "hat": (0, 0), "centered": True}
        decoded.update(changes)
        self.g.handle_input({"connected": True, "decoded": decoded, "pressed": list(pressed),
                             "timestamp": self.clock(), "raw": {}, "error": ""})

    def target(self, dx=0, droll=0):
        target = list(self.seed_tcp)
        target[0] += dx
        target[3] += droll
        return target

    def take(self):
        result = []
        while True:
            try:
                item = self.c._tx_queue.get_nowait()
                if item:
                    result.append(item)
            except queue.Empty:
                return result

    def write(self, items=None):
        items = self.take() if items is None else items
        for generation, command in items:
            self.assertTrue(self.c._can_write(generation, command), command)
            self.c.serial_port.write((command + "\n").encode("ascii"))
            self.c._on_written(command, self.c._connection_id)
            self.writes.append(command)
        return [command for _generation, command in items]

    def feed(self, *lines):
        for line in lines:
            self.c._process_rx_line(line)

    def mode(self, mode, busy=0, issued=40):
        self.feed(f"ok CMDMODE mode={mode} busy={busy} issued={issued} req={self.g._mode_query['request']}")

    def begin_init(self, mode=1, issued=40):
        self.assertTrue(self.g.claim())
        self.assertTrue(self.g.start())
        self.assertEqual(self.write(), ["!START"])
        self.feed("Start pending", "Started ok")
        self.assertEqual(self.g.stage, "enabled")
        self.c.current_cmdmode = mode
        self.g._issued = issued
        return self.g.start()

    def ready_session(self, centered=True):
        self.assertTrue(self.begin_init(mode=2))
        self.write()
        self.feed("queued MOVEJ id=41 mode=INT", "accepted MOVEJ id=41")
        self.assertEqual(self.g.stage, "init_confirmation")
        self.assertTrue(self.g.confirm_initialization())
        self.assertEqual(self.take(), [])
        self.g.centered = centered
        self.g.tick()
        self.assertEqual(self.g.ready, centered)
        self.seed_tcp = list(self.g.command_reference_tcp)

    def begin_rest(self):
        self.ready_session()
        self.assertTrue(self.g.rest())
        self.write()
        self.mode(2, issued=41)


class GamepadSessionTests(GamepadFixture, unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.setup_gamepad()

    def tearDown(self):
        self.c.close()

    def test_unclaimed_inputs_and_default_timer_never_send(self):
        self.assertFalse(self.g._timer.isActive())
        self.sample(("start", "stop", "rest"), ly=-1, rt=1)
        self.g.tick()
        self.assertFalse(self.g.start())
        self.g.release()
        self.assertEqual(self.take(), [])
        self.assertEqual(self.c.state()["control_owner"], "ui")

    def test_a_sends_one_initial_movej_after_enable_and_never_confirms_arrival(self):
        self.sample(("initialize",))
        self.assertEqual(self.take(), [])
        self.assertTrue(self.g.claim())
        self.clock.advance(.02)
        self.sample(("initialize",))
        self.assertEqual(self.take(), [])  # A does not enable the motors.
        self.assertTrue(self.g.start())
        self.assertEqual(self.write(), ["!START"])
        self.feed("Started ok")
        self.c.current_cmdmode = 2
        self.clock.advance(.02)
        self.sample(("initialize",))
        self.assertEqual(self.write(), [">0,10,80,0,90,0,0,10"])
        self.clock.advance(.02)
        self.sample(("initialize",))
        self.assertEqual(self.take(), [])  # No repeated request while awaiting its receipt.
        self.feed("queued MOVEJ id=1 mode=INT", "accepted MOVEJ id=1")
        self.clock.advance(.02)
        self.sample(("initialize",))
        self.assertEqual(self.g.stage, "init_confirmation")
        self.assertFalse(self.g.ready)
        self.assertEqual(self.take(), [])
        self.clock.advance(.02)
        self.sample(("start",))
        self.g.tick()
        self.assertTrue(self.g.ready)
        self.assertEqual(self.take(), [])

    def test_a_reinitializes_ready_control_and_stop_takes_priority(self):
        self.ready_session()
        self.clock.advance(.02)
        self.sample(("initialize",))
        self.assertEqual(self.write(), [">0,10,80,0,90,0,0,10"])
        self.assertFalse(self.g.ready)
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=42")
        self.clock.advance(.02)
        self.sample(("initialize", "stop"))
        self.assertEqual(self.write(), ["!STOP"])

    def test_two_starts_send_one_movej_and_poll_does_not_confirm_arrival(self):
        completed = []
        self.g.initial_pose_completed.connect(completed.append)
        self.begin_init(mode=2)
        self.assertEqual(self.write(), [">0,10,80,0,90,0,0,10"])
        self.assertNotIn("#CMDMODE 1", self.writes)
        self.assertNotIn("#GETJPOS", self.writes)
        self.feed("ok MOVEJ id=41", "queued MOVEJ id=40 mode=INT", "queued MOVEL id=41 mode=SEQ")
        self.assertEqual(self.g.stage, "joint_queued")
        self.feed("queued MOVEJ id=41 mode=INT", "ok MOVEJ id=42", "accepted MOVEJ id=41")
        self.assertEqual(self.g.stage, "init_confirmation")
        self.assertEqual(completed, [])
        for _ in range(310):
            self.clock.advance(.2)
            self.sample(ly=-1, centered=False)
            self.g.tick()
            self.c._poll_position()
            self.assertEqual(self.write(), ["#GETJPOS"])
            self.feed("ok 0 10 80 0 90 0 0")
        self.assertEqual(self.take(), [])
        self.assertIsNone(self.g.target_tcp)
        self.assertTrue(self.g.initialize())  # The UI's initialization button now confirms locally.
        self.assertEqual(completed, [[0., 10., 80., 0., 90., 0., 0.]])
        self.g.centered = False
        self.g.tick()
        self.assertFalse(self.g.ready)
        self.g.centered = True
        self.g.tick()
        self.assertTrue(self.g.ready)
        self.assertEqual(self.take(), [])
        self.assertEqual(self.g.gripper_target, 100)
        self.assertFalse(self.g.start())

    def test_third_start_press_confirms_locally_then_requires_centered_input(self):
        self.c.current_cmdmode = 2
        self.assertTrue(self.g.claim())
        self.clock.advance(.02)
        self.sample(("start",))
        self.assertEqual(self.write(), ["!START"])
        self.feed("Started ok")
        self.clock.advance(.02)
        self.sample(("start",))
        self.assertEqual(self.write(), [">0,10,80,0,90,0,0,10"])
        self.assertFalse(self.g.start())  # Before the device's matching receipt.
        self.feed("queued MOVEJ id=1 mode=INT", "accepted MOVEJ id=1")
        self.clock.advance(.02)
        self.sample(("start",), ly=-1, centered=False)
        self.assertEqual(self.g.stage, "centering")
        self.g.tick()
        self.assertFalse(self.g.ready)
        self.assertEqual(self.take(), [])
        self.clock.advance(.02)
        self.sample()
        self.g.tick()
        self.assertTrue(self.g.ready)
        self.assertEqual(self.take(), [])
        self.assertFalse(self.g.start())  # Further presses do not reinitialize ready control.
        self.assertEqual(self.writes, ["!START", ">0,10,80,0,90,0,0,10"])

    def test_legacy_workspace_and_minimum_height_no_longer_reject_movel(self):
        self.g.set_config({"robot": {"flange_limits": {"x": [146.4, 146.6],
                                                      "y": [-.1, .1], "z": [288.8, 289.]},
                                     "z_min_height": 288.8}, "control": {"send_interval_s": .05}})
        self.sample()
        self.ready_session()
        # TCP Z is already below the old minimum; move through each old XYZ
        # boundary with a valid, small software step and matching receipts.
        for motion_id, (axis, delta) in enumerate(((0, 1), (0, -2), (1, 1), (1, -2),
                                                  (2, 1), (2, -2)), 42):
            with self.subTest(axis=axis, delta=delta):
                target = list(self.g.command_reference_tcp)
                target[axis] += delta
                self.assertTrue(self.g.set_target(target))
                self.clock.advance(.06)
                self.sample()
                self.g.tick()
                commands = self.write()
                self.assertEqual(len(commands), 1)
                self.assertTrue(commands[0].startswith("@"))
                self.feed(f"queued MOVEL id={motion_id} mode=INT", f"accepted MOVEL id={motion_id}")
        target = list(self.g.command_reference_tcp)
        target[0] += 6
        self.assertTrue(self.g.set_target(target))  # Dynamic buffer accepts a step above the old 5mm cap.
        target = list(self.g.command_reference_tcp)
        target[3] += 6
        self.assertTrue(self.g.set_target(target))
        from app.core.gamepad_session import angle_difference
        self.assertAlmostEqual(abs(angle_difference(self.g.target_tcp[3], self.g.written_reference_tcp[3])), 5)
        self.assertNotIn("!STOP", self.writes)

    def test_default_gripper_rate_is_150_without_changing_protocol(self):
        self.g.set_config({})
        self.sample()
        self.ready_session()
        self.clock.advance(.051)
        self.sample(lt=1, centered=False)
        self.g.tick()
        self.assertEqual(self.g.gripper_target, 92.5)
        self.assertEqual(100 - self.g.gripper_target, 150 * .05)
        self.assertEqual(self.write(), ["!HAND_POS 92"])
        self.feed("ok hand pos=91")
        self.assertIsNotNone(self.g._hand)
        self.feed("ok hand pos=92")
        self.clock.advance(.101)
        self.sample(rt=1, centered=False)
        self.g.tick()
        self.assertEqual(self.g.gripper_target, 100)
        self.assertEqual(self.write(), ["!HAND_POS 100"])

    def test_rx_before_written_cannot_confirm_init_or_emit_extra_commands(self):
        self.g.claim()
        self.g.start()
        pending = self.take()
        self.feed("Start pending", "Started ok")
        self.assertEqual(self.g.stage, "starting")
        self.write(pending)
        self.assertEqual(self.g.stage, "enabled")
        self.g.start()
        pending = self.take()
        self.feed("queued MOVEJ id=41 mode=INT", "accepted MOVEJ id=41")
        self.assertEqual(self.g.stage, "joint_queued")
        self.assertFalse(self.g.confirm_initialization())
        self.write(pending)
        self.assertEqual(self.g.stage, "init_confirmation")
        self.assertTrue(self.g.confirm_initialization())
        self.assertEqual(self.take(), [])

    def test_move_stream_sends_next_written_target_without_waiting_for_accepted(self):
        self.ready_session()
        self.assertTrue(self.g.set_target(self.target(1)))
        self.g.tick()
        first = self.write()
        self.assertEqual(first, ["@147.5,0,331.2,180,0,180,10"])
        self.g.set_target(self.target(2))
        self.clock.advance(.06)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["@148.5,0,331.2,180,0,180,10"])
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=43", "ok MOVEL id=42")
        self.assertEqual(len(self.g._streams), 2)
        self.feed("accepted MOVEL id=42")
        self.g.tick()
        self.assertEqual(self.write(), [])
        self.assertEqual(len(self.g._streams), 1)

    def test_translation_preserves_positive_180_for_all_axes_with_default_tool(self):
        self.g.set_config({"control": {"send_interval_s": .05, "velocity_smoothing_s": 0}})
        self.sample()
        self.ready_session()
        for motion_id, axis in enumerate(("ly", "lx", "ry"), 42):
            with self.subTest(axis=axis):
                self.clock.advance(.06)
                # Half stick preserves the +/-180 wire representation with the default tool.
                self.sample(centered=False, **{axis: -.5})
                self.g.tick()
                commands = self.write()
                self.assertEqual(len(commands), 1)
                fields = commands[0][1:].split(",")
                self.assertEqual(fields[3:], ["180", "0", "180", "35"])
                self.assertTrue(all("." not in value or len(value.split(".")[1]) == 1
                                    for value in fields[:6]))
                self.assertEqual(self.g.target_tcp[3:], [180., 0., 180.])
                self.feed(f"queued MOVEL id={motion_id} mode=INT", f"accepted MOVEL id={motion_id}")
        self.assertIsNone(self.g.actual_tcp)
        self.assertFalse(any(command.startswith("#GET") for command in self.writes))

    def test_rotation_changes_only_driven_angle_and_translation_retains_negative_180(self):
        self.ready_session()
        target = self.target(droll=-.25)
        self.assertTrue(self.g.set_target(target))
        self.clock.advance(.06)  # dt is capped at 50ms: 5deg/s * .05 = .25deg.
        self.sample(roll=1, centered=False)
        self.assertEqual(self.g.target_tcp[3:], [180., 0., 180.])
        self.clock.advance(.06)
        self.sample(roll=1, centered=False)
        self.g.tick()
        self.assertEqual(self.write(), ["@146.5,0,331.2,-179.8,0,180,10"])
        self.assertEqual(self.g.target_tcp[3:], [-179.75, 0., 180.])
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=42")
        target = list(self.g.target_tcp)
        target[3] = -180.
        self.assertTrue(self.g.set_target(target))
        self.clock.advance(.06)
        self.sample(ly=-1, centered=False)
        self.g.tick()
        self.assertEqual(self.write(), ["@147,0,331.2,-180,0,180,10"])
        self.assertEqual(self.g.target_tcp[3:], [-180., 0., 180.])

    def test_small_input_accumulates_but_duplicate_wire_targets_do_not_send(self):
        self.ready_session()
        for _ in range(5):
            self.clock.advance(.02)
            self.sample(ly=-.025, centered=False)  # About .005mm per sample.
            self.g.tick()
            self.assertEqual(self.take(), [])
        self.clock.advance(.02)
        self.sample(ly=-.025, centered=False)
        self.g.tick()
        self.assertEqual(self.write(), ["@146.6,0,331.2,180,0,180,10"])
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=42")
        for _ in range(10):
            self.clock.advance(.02)
            self.sample(ly=-.025, centered=False)
            self.g.tick()
            self.assertEqual(self.take(), [])
        self.assertAlmostEqual(self.g.target_tcp[0] - self.seed_tcp[0], .08, places=6)
        self.assertEqual(self.g.command_reference_tcp[0], 146.6)
        self.assertTrue(self.g.set_gripper_base(40))
        self.assertEqual(self.write(), ["!HAND_POS 40"])
        for _ in range(10):
            self.clock.advance(.02)
            self.sample(ly=-.025, centered=False)
            self.g.tick()
        self.assertEqual(self.write(), ["@146.7,0,331.2,180,0,180,10"])

    def test_quantization_follows_tool_conversion_and_receipt_uses_sent_pose(self):
        from app.core.gamepad_session import tool_translation
        self.g.set_config({})
        self.sample()
        self.ready_session()
        target = list(self.g.target_tcp)
        target[4] = .24
        offset = tool_translation(target, 114)
        target[0] = 147.0496 + offset[0]
        self.assertTrue(self.g.set_target(target))
        self.g.tick()
        command = self.write()[0]
        # No intermediate TCP rounding: 147.0496 rounds directly to 147.0.
        self.assertEqual(command, "@147,0,331.2,180,0.2,180,35")
        sent_flange = [float(value) for value in command[1:].split(",")[:6]]
        sent_offset = tool_translation(sent_flange, 114)
        expected = [sent_flange[i] + sent_offset[i] for i in range(3)] + sent_flange[3:]
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=42")
        self.assertEqual(self.g.command_reference_tcp, expected)
        self.assertEqual(self.g.target_tcp, target)
        self.assertIsNone(self.g.actual_tcp)

    def test_gripper_defaults_to_100_and_only_target_ack_unlocks(self):
        self.ready_session()
        self.clock.advance(.05)
        self.sample(rt=1, centered=False)
        self.g.tick()
        self.assertEqual(self.g.gripper_target, 100)
        self.assertEqual(self.take(), [])
        self.assertTrue(self.g.set_gripper_base(50))
        self.assertEqual(self.write(), ["!HAND_POS 50"])
        self.clock.advance(.051)  # Clamp to exact .05, away from clock subtraction roundoff.
        self.sample(rt=1, centered=False)
        self.g.tick()
        self.assertGreater(self.g.gripper_target, 50)
        self.assertEqual(self.take(), [])
        self.feed("ok hand pos=49")
        self.assertIsNotNone(self.g._hand)
        self.feed("ok hand pos=50")
        self.clock.advance(.051)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["!HAND_POS 52"])

    def test_gripper_display_signal_uses_immutable_written_opening_not_latest_target(self):
        self.ready_session()
        sent = []
        self.g.gripper_command_written.connect(sent.append)
        self.assertTrue(self.g.set_gripper_base(50.4))
        pending = self.take()
        self.assertEqual(sent, [])
        self.g.gripper_target = 20
        self.feed("ok hand pos=50")  # Receipt can arrive before the queued written signal.
        self.assertEqual(sent, [])
        self.assertEqual(self.write(pending), ["!HAND_POS 50"])
        self.assertEqual(sent, [50.])
        self.c._on_written("!HAND_POS 50", self.c._connection_id)
        self.assertEqual(sent, [50.])  # An unadmitted notification is not a new send.
        self.clock.advance(.11)
        self.sample()
        self.g.tick()
        self.assertEqual(sent, [50.])
        self.assertEqual(self.write(), ["!HAND_POS 20"])
        self.feed("ok hand pos=20")
        self.assertEqual(sent, [50., 20.])
        self.clock.advance(.11)
        self.sample()
        self.assertTrue(self.g.set_gripper_base(0))
        old = self.take()[0]
        self.g.stop()
        self.assertFalse(self.c._can_write(*old))
        self.c._on_written(old[1], self.c._connection_id)
        self.assertEqual(sent, [50., 20.])  # STOP canceled the unsent opening.
        self.assertEqual(self.write(), ["!STOP"])

    def test_raw_triggers_use_hand_pos_lt_decreases_rt_increases_and_both_cancel(self):
        from app.gamepad.input import decode_gamepad
        self.ready_session()
        self.assertEqual(self.g.gripper_target, 100)
        self.assertEqual(self.take(), [])

        def trigger(lt, rt):
            self.clock.advance(.11)
            axes = [0., 0., 0., 0., lt, rt]
            decoded = decode_gamepad(axes, [0] * 8, [(0, 0)], self.g.config["gamepad"])
            self.g.handle_input({"connected": True, "decoded": decoded, "pressed": [],
                                 "timestamp": self.clock(), "raw": {"axes": axes}, "error": ""})
            self.g.tick()

        trigger(-1, 1)  # RT at the default upper limit does not create a command.
        self.assertEqual(self.g.gripper_target, 100)
        self.assertEqual(self.take(), [])
        trigger(1, -1)
        self.assertEqual(self.write(), ["!HAND_POS 98"])
        self.assertEqual(self.g.gripper_target, 98.5)
        self.feed("ok hand pos=98")
        trigger(1, 0)  # Both triggers active, even with unequal pressure.
        self.assertEqual(self.take(), [])
        self.assertEqual(self.g.gripper_target, 98.5)
        trigger(-1, 1)
        self.assertEqual(self.write(), ["!HAND_POS 100"])
        self.feed("ok hand pos=100")
        self.clock.advance(.11)
        self.sample()
        self.assertTrue(self.g.set_gripper_base(1))
        self.assertEqual(self.write(), ["!HAND_POS 1"])
        self.feed("ok hand pos=1")
        trigger(1, -1)
        self.assertEqual(self.write(), ["!HAND_POS 0"])
        self.feed("ok hand pos=0")
        trigger(1, -1)
        self.assertEqual(self.take(), [])
        self.assertEqual(self.g.gripper_target, 0)
        self.assertFalse(any(command.startswith(("!HAND_O", "!HAND_C", "#GET")) for command in self.writes))

    def test_explicit_default_gripper_position_sends_and_reinitialization_only_seeds(self):
        self.ready_session()
        self.assertTrue(self.g.set_gripper_base(100))
        self.assertEqual(self.write(), ["!HAND_POS 100"])
        self.feed("ok hand pos=100")
        self.assertTrue(self.g.initialize())
        self.assertIsNone(self.g.gripper_target)
        self.assertEqual(self.write(), [">0,10,80,0,90,0,0,10"])
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=42")
        self.assertTrue(self.g.confirm_initialization())
        self.clock.advance(.11)
        self.sample()
        self.g.tick()
        self.assertEqual(self.g.gripper_target, 100)
        self.assertEqual(self.take(), [])

    def test_local_release_invalidates_queued_targets_without_stop(self):
        self.ready_session()
        self.g.set_target(self.target(dx=1))
        self.g.tick()
        self.g.set_gripper_base(40)
        pending = self.take()
        self.assertTrue(any(command.startswith("@") for _gen, command in pending))
        self.assertTrue(any(command.startswith("!HAND_POS") for _gen, command in pending))
        self.g.release()
        self.g.release()
        self.assertFalse(self.g.claimed)
        self.assertFalse(self.g._timer.isActive())
        self.assertEqual(self.g.stage, "released")
        self.assertIn("实机可能仍在执行最后目标", self.g.message)
        self.assertEqual(self.c.state()["control_owner"], "ui")
        self.assertTrue(all(not self.c._can_write(gen, command) for gen, command in pending))
        self.clock.advance(4)
        self.sample(ly=-1., rt=1., centered=False)
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.assertFalse(self.c._stop_locked)
        self.assertFalse(self.c.movej_drag_enable.get())

    def test_stopping_handover_waits_for_written_and_stop_confirmation(self):
        self.ready_session()
        self.g.release(request_stop=True)
        self.g.release()
        pending = self.take()
        self.assertEqual([command for _gen, command in pending], ["!STOP"])
        self.feed("STOP locked")
        self.assertTrue(self.g.claimed)
        self.assertTrue(self.g.release_pending)
        self.assertEqual(self.c.state()["control_owner"], "gamepad")
        self.assertFalse(self.c.send_cmd(">0,-75,180,0,0,0,0,35"))
        self.write(pending)
        self.assertFalse(self.g.claimed)
        self.assertEqual(self.c.state()["control_owner"], "ui")
        self.assertFalse(self.c.movej_drag_enable.get())

    def test_release_preserves_a_stop_already_queued(self):
        self.ready_session()
        self.g.stop()
        pending = self.take()
        self.g.release()
        self.assertTrue(self.g.release_pending)
        self.assertEqual(self.take(), [])
        self.assertEqual(self.write(pending), ["!STOP"])
        self.feed("STOP locked")
        self.assertFalse(self.g.claimed)

    def test_hand_receipt_timeout_pauses_only_gripper_and_explicit_position_retries(self):
        self.ready_session()
        self.g.set_gripper_base(40)
        self.assertEqual(self.write(), ["!HAND_POS 40"])
        self.clock.advance(3.1)
        self.sample(ly=-.5, centered=False)
        self.g.tick()
        commands = self.write()
        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0].startswith("@"))
        self.assertTrue(self.g.ready)
        self.assertTrue(self.g.state()["gripper_paused"])
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=42", "ok hand pos=40")
        for _ in range(10):
            self.clock.advance(.1)
            self.sample(rt=1., centered=False)
            self.g.tick()
        self.assertEqual(self.take(), [])
        self.assertEqual(self.g.gripper_target, 40)
        self.assertTrue(self.g.set_gripper_base(50))
        self.assertFalse(self.g.state()["gripper_paused"])
        self.assertEqual(self.write(), ["!HAND_POS 50"])
        self.feed("ok hand pos=40")
        self.assertIsNotNone(self.g._hand)
        self.feed("ok hand pos=50")
        self.assertIsNone(self.g._hand)
        self.assertNotIn("!STOP", self.writes)

    def test_hand_stuck_before_write_still_stops_shared_transport(self):
        self.ready_session()
        self.g.set_gripper_base(40)
        self.clock.advance(10.1)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["!STOP"])

    def test_loss_and_feedback_timeout_soft_stop_but_protection_does_not_interrupt_backoff(self):
        self.ready_session()
        self.g.handle_input({"timestamp": self.clock(), "connected": False, "decoded": None})
        self.assertEqual(self.write(), ["!STOP"])
        self.feed("STOP locked")
        self.assertEqual(self.g.stage, "fault")
        self.assertFalse(self.g.ready)
        self.assertFalse(self.c.send_cmd("!START"))

    def test_shared_position_timeout_does_not_stop_but_lost_stop_keeps_owner_locked(self):
        self.ready_session()
        for _ in range(20):
            self.clock.advance(.2)
            self.sample()
            self.g.tick()
            self.c._poll_position()
            self.assertEqual(self.write(), ["#GETJPOS"])
            self.c._position_timeout(self.c._position_request["token"])
        self.assertTrue(self.g.ready)
        self.assertEqual(self.take(), [])
        self.assertIsNone(self.g.actual_tcp)
        self.assertTrue(self.g.stop())
        self.assertEqual(self.write(), ["!STOP"])
        self.clock.advance(10.1)
        self.g.tick()
        self.assertEqual(self.g.stage, "fault")
        self.assertTrue(self.g.claimed)
        self.assertFalse(self.g.start())
        self.assertEqual(self.c.state()["control_owner"], "gamepad")

    def test_joint_motion_wrong_mode_and_timeout_stop_without_unlocking(self):
        self.begin_rest()
        self.write()
        self.feed("queued MOVEJ id=42 mode=SEQ")
        self.assertEqual(self.write(), ["!STOP"])
        self.feed("ok MOVEJ id=42 ms=10")
        self.assertFalse(self.g.ready)

    def test_rest_sets_rail_zero_completes_without_query_and_manual_action_is_retired(self):
        self.ready_session()
        self.clock.advance(.02)
        self.sample(("manual",))
        self.assertEqual(self.take(), [])
        self.assertTrue(self.g.ready)
        self.assertFalse(hasattr(self.g, "resync"))
        self.g.rest()
        self.write()
        self.mode(2, issued=41)
        self.assertEqual(self.write(), [">0,-75,180,0,0,0,0,10"])
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=42")
        self.g.tick()
        self.assertEqual(self.write(), ["#MOTION_STATUS"])
        self.feed("MOTION id=42 kind=MOVEJ phase=3 mode=2 ms=300")
        self.assertEqual(self.g.stage, "rested")
        self.assertFalse(self.g.ready)
        self.assertIsNone(self.g.actual_tcp)
        self.assertEqual(self.take(), [])
        for command in ("#CMDMODE 1", "#GETLPOS", "#GETJPOS", "#GETRAILPOS"):
            self.assertNotIn(command, self.writes)

    def test_custom_rail_limits_use_fixed_zero_target_without_rail_readback(self):
        self.g.set_config({"robot": {"tool_offset_z": 0, "rail_limits": [-10, 10]}})
        self.sample()
        self.begin_init()
        self.assertEqual(self.write(), [">0,10,80,0,90,0,0,10"])

    def test_completed_init_and_tool_114_seed_software_tcp_without_faking_actual(self):
        self.g.set_config({})
        self.sample()
        self.feed("ok 1 2 3 4 5 6 37.25")
        actual = list(self.c.actual_pose)
        self.begin_init()
        self.write()
        self.feed("queued MOVEJ id=41 mode=SEQ", "ok MOVEJ id=41 ms=812")
        self.assertEqual(self.take(), [])
        self.assertEqual(self.g.stage, "init_confirmation")
        self.assertFalse(self.g.confirm_initialization())  # SEQ is never silently changed to INT.
        self.assertIsNone(self.g.target_tcp)
        self.g.stop()
        self.write()
        self.feed("STOP locked")
        self.g.start()
        self.write()
        self.feed("Started ok")
        self.g.start()
        self.write()
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=42")
        self.assertTrue(self.g.confirm_initialization())
        self.assertIsNone(self.g.actual_tcp)
        self.assertEqual(self.c.actual_pose, actual)
        self.assertAlmostEqual(self.g.target_tcp[0], 146.520190203)
        self.assertAlmostEqual(self.g.target_tcp[1], 0)
        self.assertAlmostEqual(self.g.target_tcp[2], 217.217318012)
        self.assertEqual(self.g.command_reference_tcp, self.g.target_tcp)
        self.g.tick()
        self.assertTrue(self.g.ready)
        target = list(self.g.target_tcp)
        target[0] += .5
        self.assertTrue(self.g.set_target(target))
        self.g.tick()
        command = self.write()[0]
        self.assertEqual([float(value) for value in command[1:].split(",")],
                         [147, 0, 331.2, 180, 0, 180, 35])
        self.assertNotIn("#GETJPOS", self.writes)
        self.assertEqual(self.c.actual_pose, actual)

    def test_config_change_is_rejected_when_claimed_and_snapshots_are_independent(self):
        self.g.claim()
        with self.assertRaises(ValueError):
            self.g.set_config({"robot": {"tool_offset_z": 1}})
        state = self.g.state()
        state["raw"]["fake"] = True
        self.assertNotIn("fake", self.g.state()["raw"])
        self.assertFalse(hasattr(self.g, "resync"))

    def test_old_or_duplicate_centered_samples_do_not_unlock_or_replay_buttons(self):
        self.ready_session(centered=False)
        self.sample(("stop",), centered=True)
        self.g.tick()
        self.assertEqual(self.g.stage, "centering")
        self.assertFalse(self.g.centered)
        self.assertEqual(self.take(), [])
        self.clock.advance(.02)
        self.sample(centered=True)
        self.g.tick()
        self.assertTrue(self.g.ready)
        timestamp = self.g.last_input_at
        self.g.handle_input({"timestamp": timestamp - .01, "connected": True,
                             "decoded": {"centered": False}, "pressed": ["stop"]})
        self.assertTrue(self.g.centered)
        self.assertTrue(self.g.ready)
        self.assertEqual(self.take(), [])
        # Loss is a safety event even if the reader repeats its last timestamp.
        self.g.handle_input({"timestamp": timestamp, "connected": False, "decoded": None})
        self.assertEqual(self.write(), ["!STOP"])

    def test_actual_joint_motion_timeout_and_refused_stop_keep_owner(self):
        self.begin_init()
        self.write()
        self.feed("queued MOVEJ id=41 mode=SEQ")
        self.clock.advance(60.1)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["!STOP"])
        self.feed("STOP locked")
        self.assertEqual(self.g.stage, "fault")
        self.c._tx_queue.put = lambda _item: (_ for _ in ()).throw(RuntimeError("queue failure"))
        self.assertFalse(self.g.stop())
        self.assertEqual(self.g.stage, "fault")
        self.assertTrue(self.g.claimed)
        self.assertEqual(self.c.state()["control_owner"], "gamepad")

    def test_config_faster_intervals_still_apply_hard_caps(self):
        self.g.set_config({"control": {"send_interval_s": .02, "gripper_interval_s": .02,
                                       "feedback_interval_s": .05}, "robot": {"tool_offset_z": 0, "cmd_speed": 10}})
        self.sample()
        self.ready_session()
        self.g.set_target(self.target(1))
        self.g.set_gripper_base(50)
        self.assertEqual(len(self.write()), 2)
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=42", "ok hand pos=50")
        self.g.set_target(self.target(2))
        self.g.gripper_target = 51
        self.clock.advance(.021)
        self.sample()
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.clock.advance(.03)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["@148.5,0,331.2,180,0,180,10"])
        self.feed("queued MOVEL id=43 mode=INT", "accepted MOVEL id=43")
        self.clock.advance(.05)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["!HAND_POS 51"])
        self.clock.advance(.101)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), [])

    def test_tool_transform_software_seed_and_target_apply_offset_once(self):
        from app.core.gamepad_session import tool_translation
        self.g.set_config({"robot": {"tool_offset_z": 20, "cmd_speed": 10}})
        self.sample()
        self.ready_session()
        flange = [146.520190203, 0., 331.217318012, 180., 0., 180.]
        offset = tool_translation(flange, 20)
        for computed, expected in zip(self.g.target_tcp, [flange[i] + offset[i] for i in range(3)] + flange[3:]):
            self.assertAlmostEqual(computed, expected)
        self.assertIsNone(self.g.actual_tcp)
        target = list(self.g.target_tcp)
        target[0] += 1
        self.assertTrue(self.g.set_target(target))
        self.g.tick()
        self.assertEqual(self.write(), ["@147.5,0,331.2,180,0,180,10"])

    def test_input_dt_clamp_and_invalid_trigger_cannot_create_gripper_target(self):
        self.ready_session()
        self.clock.advance(.4)
        self.sample(ly=-1, centered=False)
        self.assertLessEqual(math.dist(self.g.target_tcp[:3], self.g.command_reference_tcp[:3]), .500001)
        self.clock.advance(.02)
        self.sample(lt=math.nan, centered=False)
        self.assertIsNone(self.g.gripper_target)
        self.assertEqual(self.write(), ["!STOP"])

    def test_late_motion_failure_matches_accepted_id_and_preserves_firmware_protection(self):
        self.ready_session()
        self.g.set_target(self.target(1))
        self.g.tick()
        self.write()
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=42")
        self.feed("MOTION fail id=40 reason=TARGET_TX_TIMEOUT", "MOTION fail id=999 reason=TARGET_TX_TIMEOUT")
        self.assertTrue(self.g.ready)
        self.g.set_target(self.target(2))
        self.clock.advance(.06)
        self.sample()
        self.g.tick()
        pending = self.take()
        self.feed("MOTION fail id=42 reason=TARGET_TX_TIMEOUT")
        self.assertEqual(self.g.stage, "protected")
        self.assertTrue(self.g.claimed)
        self.assertTrue(self.c._stop_locked)
        self.assertEqual(self.take(), [])
        self.assertFalse(self.c._can_write(*pending[0]))
        self.assertFalse(self.c.send_cmd("!HOME"))

    def test_motion_failure_before_written_is_deferred_and_old_epoch_is_ignored(self):
        self.ready_session()
        self.g.set_target(self.target(1))
        self.g.tick()
        pending = self.take()
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=42", "MOTION fail id=42 reason=TARGET_READY_TIMEOUT")
        self.assertTrue(self.g.ready)
        self.write(pending)
        self.assertEqual(self.g.stage, "protected")
        self.g.stop()
        self.write()
        self.feed("STOP locked")
        self.assertTrue(self.g.start())
        self.write()
        self.feed("Started ok", "MOTION fail id=42 reason=TARGET_READY_TIMEOUT")
        self.assertEqual(self.g.stage, "enabled")

    def test_int_init_acceptance_only_enables_local_confirmation(self):
        completed = []
        self.g.initial_pose_completed.connect(completed.append)
        self.begin_init(mode=2)
        pending = self.take()
        self.feed("queued MOVEJ id=41 mode=INT", "accepted MOVEJ id=999")
        self.assertFalse(self.g.confirm_initialization())
        self.write(pending)
        self.assertEqual(self.g.stage, "joint_moving")
        self.feed("ok MOVEJ id=41 ms=80", "MOTION id=41 kind=MOVEJ phase=3 mode=2 ms=80")
        self.assertFalse(self.g.confirm_initialization())
        self.assertEqual(completed, [])
        self.feed("accepted MOVEJ id=41")
        self.assertEqual(self.g.stage, "init_confirmation")
        self.assertIsNone(self.g.target_tcp)
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.assertTrue(self.g.confirm_initialization())
        self.assertEqual(completed, [[0., 10., 80., 0., 90., 0., 0.]])
        self.assertFalse(self.g.confirm_initialization())
        self.assertEqual(self.take(), [])

    def test_rest_int_joint_status_poll_is_single_inflight_and_maximum_five_hz(self):
        self.g.set_config({"control": {"feedback_interval_s": .05}, "robot": {"tool_offset_z": 0, "cmd_speed": 10}})
        self.sample()
        self.begin_rest()
        self.write()
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=999")
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.feed("accepted MOVEJ id=42")
        self.g.tick()
        self.assertEqual(self.write(), ["#MOTION_STATUS"])
        self.clock.advance(.19)
        self.sample()
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.feed("MOTION id=42 kind=MOVEJ phase=2 mode=2 ms=190")
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.clock.advance(.011)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["#MOTION_STATUS"])
        self.feed("MOTION id=42 kind=MOVEJ phase=1 mode=2 ms=201")
        self.assertEqual(self.g.stage, "joint_moving")

    def test_trj_init_uses_passive_actual_mode_and_never_switches_or_queries(self):
        self.begin_init(mode=2)  # Cached mode is deliberately stale.
        self.assertEqual(self.write(), [">0,10,80,0,90,0,0,10"])
        self.feed("queued MOVEJ id=41 mode=TRJ", "accepted MOVEJ id=41")
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.assertEqual(self.g.stage, "joint_moving")
        self.assertEqual(self.c.current_cmdmode, 3)
        self.feed("ok MOVEJ id=41 ms=300")
        self.assertEqual(self.g.stage, "init_confirmation")
        self.assertFalse(self.g.confirm_initialization())
        self.assertIsNone(self.g.target_tcp)
        self.assertEqual(self.take(), [])
        self.assertEqual(self.writes, ["!START", ">0,10,80,0,90,0,0,10"])

    def test_rest_busy_query_waits_and_tun_mode_is_rejected_without_motion(self):
        self.ready_session()
        self.g.rest()
        self.write()
        request = self.g._mode_query["request"]
        self.feed(f"ok CMDMODE mode=2 busy=0 issued=41 req={request + 1}")
        self.assertEqual(self.g.stage, "wait_idle")
        self.mode(2, busy=1, issued=41)
        self.assertEqual(self.take(), [])
        self.clock.advance(.101)
        self.sample()
        self.g.tick()
        self.assertEqual(len(self.write()), 1)
        self.mode(4, issued=41)
        self.assertEqual(self.g.stage, "enabled")
        self.assertIn("不支持 MoveJ", self.g.message)
        self.assertEqual(self.take(), [])
        self.assertEqual(sum(command.startswith(">") for command in self.writes), 1)

    def test_joint_id_floor_wrap_rejects_old_receipts_before_local_confirmation(self):
        self.begin_init(mode=2, issued=0xFFFFFFFF)
        self.write()
        self.feed("queued MOVEJ id=4294967295 mode=INT", "ok MOVEJ id=1 ms=1")
        self.assertEqual(self.g.stage, "joint_queued")
        self.feed("queued MOVEJ id=1 mode=INT", "accepted MOVEJ id=1")
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.assertEqual(self.g.stage, "init_confirmation")
        self.assertTrue(self.g.confirm_initialization())
        self.assertEqual(self.take(), [])

    def test_rest_int_status_failure_preserves_firmware_protection_without_second_stop(self):
        self.begin_rest()
        self.write()
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=42")
        self.g.tick()
        self.write()
        self.feed("MOTION id=42 kind=MOVEJ phase=5 mode=2 ms=300")
        self.assertEqual(self.g.stage, "protected")
        self.assertTrue(self.c._stop_locked)
        self.assertTrue(self.g.claimed)
        self.assertEqual(self.take(), [])
        self.feed("MOTION id=42 kind=MOVEJ phase=3 mode=2 ms=301")
        self.assertEqual(self.g.stage, "protected")

    def test_rest_completion_requires_matching_written_status(self):
        self.begin_rest()
        self.write()
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=42")
        self.g.tick()
        pending = self.take()
        self.assertEqual([command for _gen, command in pending], ["#MOTION_STATUS"])
        self.feed("MOTION id=41 kind=MOVEJ phase=3 mode=2 ms=80",
                  "MOTION id=42 kind=MOVEL phase=3 mode=2 ms=80",
                  "MOTION id=42 kind=MOVEJ phase=3 mode=1 ms=80",
                  "MOTION id=42 kind=MOVEJ phase=3 mode=2 ms=4294967296",
                  "MOTION id=42 kind=MOVEJ phase=6 mode=2 ms=80")
        self.assertEqual(self.g.stage, "joint_moving")
        self.feed("MOTION id=42 kind=MOVEJ phase=3 mode=2 ms=80")
        self.assertEqual(self.g.stage, "joint_moving")
        self.write(pending)
        self.assertEqual(self.g.stage, "rested")
        self.assertEqual(self.take(), [])
        self.feed("MOTION id=42 kind=MOVEJ phase=3 mode=2 ms=80")
        self.assertEqual(self.g.stage, "rested")

    def test_rest_int_single_status_timeout_retries_without_stop(self):
        self.begin_rest()
        self.write()
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=42")
        self.g.tick()
        self.write()
        self.clock.advance(3.1)
        self.sample()
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.assertEqual(self.g.stage, "joint_moving")
        self.assertIsNone(self.g._motion_status)
        self.feed("MOTION id=42 kind=MOVEJ phase=3 mode=2 ms=300")
        self.assertEqual(self.g.stage, "joint_moving")
        self.clock.advance(.21)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["#MOTION_STATUS"])
        self.feed("MOTION id=42 kind=MOVEJ phase=3 mode=2 ms=3500")
        self.assertEqual(self.g.stage, "rested")
        self.assertNotIn("!STOP", self.writes)

    def test_rest_final_motion_timeout_still_stops_after_status_timeout(self):
        self.begin_rest()
        self.write()
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=42")
        self.g.tick()
        self.assertEqual(self.write(), ["#MOTION_STATUS"])
        self.clock.advance(3.1)
        self.sample()
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.clock.advance(57.1)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["!STOP"])
        self.assertEqual(self.c.state()["control_owner"], "gamepad")

    def test_rest_int_status_cancelled_soft_stops_without_completion(self):
        completed = []
        self.g.initial_pose_completed.connect(completed.append)
        self.begin_rest()
        completed.clear()
        self.write()
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=42")
        self.g.tick()
        self.write()
        self.feed("MOTION id=42 kind=MOVEJ phase=4 mode=2 ms=300")
        self.assertEqual(self.write(), ["!STOP"])
        self.assertEqual(completed, [])
        self.assertEqual(self.g.stage, "stopping")

    def test_int_queued_without_accepted_times_out_and_fake_ok_does_not_complete(self):
        completed = []
        self.g.initial_pose_completed.connect(completed.append)
        self.begin_init(mode=2)
        self.write()
        self.feed("queued MOVEJ id=41 mode=INT", "ok MOVEJ id=41 ms=80")
        self.assertEqual(self.g.stage, "joint_moving")
        self.assertEqual(completed, [])
        self.g.tick()
        self.assertEqual(self.take(), [])
        self.clock.advance(3.1)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["!STOP"])

    def test_disconnect_clears_joint_status_and_ignores_old_connection_reply(self):
        completed = []
        self.g.initial_pose_completed.connect(completed.append)
        self.begin_rest()
        completed.clear()
        self.write()
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=42")
        self.g.tick()
        pending = self.take()
        old_connection = self.c._connection_id
        self.c.disconnect_port()
        self.assertIsNone(self.g._motion_status)
        self.assertEqual(self.g._issued, 0)
        self.assertFalse(self.g.claimed)
        self.c._on_transport_line("MOTION id=42 kind=MOVEJ phase=3 mode=2 ms=300", old_connection)
        self.g.command_written(pending[0][1], pending[0][0], old_connection)
        self.assertEqual(completed, [])
        self.assertEqual(self.g.stage, "disconnected")

    def test_stop_cancels_pending_init_and_late_old_epoch_cannot_confirm(self):
        self.begin_init(mode=2)
        pending = self.take()
        self.g.stop()
        self.assertFalse(self.c._can_write(*pending[0]))
        self.feed("queued MOVEJ id=41 mode=INT", "accepted MOVEJ id=41")
        self.assertFalse(self.g.confirm_initialization())
        self.assertEqual(self.write(), ["!STOP"])
        self.feed("STOP locked")
        self.g.start()
        self.write()
        self.feed("Started ok")
        self.g._issued = 41
        self.g.start()
        self.write()
        self.feed("queued MOVEJ id=41 mode=INT", "accepted MOVEJ id=41")
        self.assertEqual(self.g.stage, "joint_queued")
        self.feed("queued MOVEJ id=42 mode=INT", "accepted MOVEJ id=42")
        self.assertEqual(self.g.stage, "init_confirmation")
        self.assertTrue(self.g.confirm_initialization())
        self.assertEqual(self.take(), [])

    def test_compact_formatter_preserves_precision_zero_and_63_byte_limit(self):
        from app.core.gamepad_session import motion_command
        command = motion_command(">", [0, 10, 80, -0.0001, 90, 0, 0, 10])
        self.assertEqual(command, ">0,10,80,0,90,0,0,10")
        self.assertEqual(len(command), 20)
        self.assertEqual(len((command + "\n").encode("ascii")), 21)
        self.assertEqual(motion_command("@", [1.23449, -1.23451, 0, 180, 0, -0., 10]),
                         "@1.2,-1.2,0,180,0,0,10")
        self.assertEqual(motion_command(">", [1.23449, -1.23451, 0, 180, 0, -0., 0, 10]),
                         ">1.234,-1.235,0,180,0,0,0,10")
        self.assertEqual(motion_command("@", [146.582, -.663, 288.917, 180, -.019, -180, 35.125]),
                         "@146.6,-0.7,288.9,180,0,-180,35.125")
        self.assertEqual(len(motion_command("@", [1.1e61])), 63)
        with self.assertRaisesRegex(ValueError, "63"):
            motion_command("@", [1.1e62])
        for value in (math.nan, math.inf, True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                motion_command(">", [value])

    def test_oversized_joint_body_is_rejected_before_queueing_and_without_mode_change(self):
        self.g.config["robot"]["init_joints"] = [1e20] * 6
        self.begin_init(mode=2)
        self.assertEqual(self.g.stage, "enabled")
        self.assertIn("63", self.g.message)
        self.assertEqual(self.take(), [])
        self.assertNotIn("#CMDMODE 1", self.writes)

    def test_init_shared_actual_poll_cannot_seed_target_or_confirm_arrival(self):
        self.begin_init(mode=2)
        self.write()
        self.feed("queued MOVEJ id=41 mode=INT", "accepted MOVEJ id=41")
        self.clock.advance(.21)
        self.sample(ly=-1, rt=1, centered=False)
        self.g.tick()
        self.c._poll_position()
        self.assertEqual(self.write(), ["#GETJPOS"])
        self.feed("MOTION id=41 kind=MOVEJ phase=3 mode=2 ms=210", "ok 0 10 80 0 90 0 37.25")
        self.assertEqual(self.c.actual_pose[6], 37.25)
        self.assertIsNone(self.g.actual_tcp)
        self.assertIsNone(self.g.target_tcp)
        self.assertIsNone(self.g.command_reference_tcp)
        self.assertEqual(self.g.stage, "init_confirmation")
        self.assertFalse(self.g.ready)
        self.assertEqual(self.writes, ["!START", ">0,10,80,0,90,0,0,10", "#GETJPOS"])

    def test_matching_acceptance_and_local_confirmation_seed_canonical_sent_joint_pose(self):
        from app.gamepad.kinematics import forward_kinematics
        completed = []
        self.g.initial_pose_completed.connect(completed.append)
        self.g.config["robot"]["init_joints"] = [0, 10.12349, 80, 0, 90, 0]
        self.begin_init(mode=2)
        self.write()
        self.feed("queued MOVEJ id=41 mode=INT", "accepted MOVEJ id=42")
        self.assertFalse(self.g.confirm_initialization())
        self.assertIsNone(self.g.command_reference_tcp)
        self.feed("accepted MOVEJ id=41")
        self.assertIsNone(self.g.command_reference_tcp)
        self.assertTrue(self.g.confirm_initialization())
        expected_joints = [0, 10.123, 80, 0, 90, 0, 0]
        self.assertEqual(completed, [expected_joints])
        self.assertEqual(self.g.target_tcp, forward_kinematics(expected_joints))
        self.assertEqual(self.g.command_reference_tcp, self.g.target_tcp)
        self.assertIsNone(self.g.actual_tcp)
        self.assertEqual(self.g.stage, "centering")
        self.assertEqual(self.take(), [])

    def test_software_buffer_uses_written_pose_while_accepted_reference_remains_independent(self):
        self.ready_session()
        initial = list(self.g.command_reference_tcp)
        first = list(initial)
        first[0] += 4
        self.assertTrue(self.g.set_target(first))
        self.g.tick()
        pending = self.take()
        self.assertEqual(self.g.command_reference_tcp, initial)
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=43")
        self.write(pending)
        self.assertEqual(self.g.command_reference_tcp, initial)
        second = list(initial)
        second[0] += 8
        self.assertTrue(self.g.set_target(second))
        self.assertAlmostEqual(self.g.written_reference_tcp[0], 150.5)
        self.feed("accepted MOVEL id=42")
        self.assertAlmostEqual(self.g.command_reference_tcp[0], 150.5)
        self.assertTrue(self.g.set_target(second))
        self.assertIsNone(self.g.actual_tcp)
        snapshot = self.g.state()
        snapshot["command_reference_tcp"][0] = 999
        self.assertNotEqual(self.g.command_reference_tcp[0], 999)
        self.g.stop()
        self.assertIsNone(self.g.command_reference_tcp)
        self.feed("accepted MOVEL id=42")
        self.assertIsNone(self.g.command_reference_tcp)


    def test_plan_rejection_rolls_back_latest_unsent_target_and_requires_fresh_center(self):
        self.ready_session()
        reference = list(self.g.command_reference_tcp)
        self.g.set_target(self.target(dx=1))
        self.g.tick()
        pending = self.take()
        self.g.set_target(self.target(dx=4))  # Input continued before receipt.
        self.feed("queued MOVEL id=42 mode=INT", "error MOVEL id=42 reason=PLAN_FAILED")
        self.assertIsNotNone(self.g._stream)  # RX-before-written is deferred.
        self.write(pending)
        self.assertEqual(self.g.stage, "target_rejected")
        self.assertEqual(self.g.target_tcp, reference)
        self.assertFalse(self.c._stop_locked)
        self.assertIsNone(self.g._stream)
        self.feed("accepted MOVEL id=42", "error MOVEL id=42 reason=PLAN_FAILED")
        for _ in range(10):
            self.clock.advance(.1)
            self.sample(ly=-1., centered=False)
            self.g.tick()
        self.assertEqual(self.take(), [])
        self.assertEqual(self.g.target_tcp, reference)
        self.assertEqual(self.g.stage, "target_rejected")
        self.clock.advance(.02)
        self.sample()
        self.g.tick()
        self.assertTrue(self.g.ready)
        self.assertEqual(self.take(), [])  # No automatic retry/START/position query.
        self.assertTrue(self.g.set_target(self.target(dx=-1)))
        self.g.tick()
        commands = self.write()
        self.assertEqual(len(commands), 1)
        self.assertTrue(commands[0].startswith("@"))
        self.feed("error MOVEL id=42 reason=PLAN_FAILED")  # Late, unrelated failure.
        self.assertEqual(len(self.g._streams), 1)
        self.feed("queued MOVEL id=43 mode=INT", "accepted MOVEL id=43")
        self.assertTrue(self.g.ready)
        self.assertNotIn("!STOP", self.writes)
        self.assertIsNone(self.g.actual_tcp)

    def test_joint_plan_rejection_returns_to_initialization_without_stop(self):
        self.begin_init(mode=2)
        self.write()
        self.feed("queued MOVEJ id=41 mode=INT", "error MOVEJ id=41 reason=PLAN_FAILED")
        self.assertEqual(self.take(), [])
        self.assertEqual(self.g.stage, "enabled")
        self.assertFalse(self.g.ready)
        self.assertFalse(self.c._stop_locked)
        self.assertTrue(self.g.initialize())
        self.assertEqual(self.write(), [">0,10,80,0,90,0,0,10"])

    def test_invalid_new_target_preserves_pending_valid_motion_without_stop(self):
        self.ready_session()
        self.g.set_target(self.target(dx=1))
        self.g.tick()
        self.write()
        valid = list(self.g.target_tcp)
        self.assertFalse(self.g.set_target([math.nan] + valid[1:]))
        self.assertEqual(self.g.target_tcp, valid)
        self.assertEqual(len(self.g._streams), 1)
        self.assertEqual(self.take(), [])
        self.feed("queued MOVEL id=42 mode=INT", "accepted MOVEL id=42")
        self.assertIsNone(self.g._stream)
        self.clock.advance(.02)
        self.sample()
        self.g.tick()
        self.assertTrue(self.g.ready)
        self.assertEqual(self.take(), [])

    def test_unencodable_target_rolls_back_without_stop(self):
        self.ready_session()
        self.g.target_tcp = [1e100] * 6
        self.g.tick()
        self.assertEqual(self.g.stage, "target_rejected")
        self.assertEqual(self.g.target_tcp, self.seed_tcp)
        self.assertEqual(self.take(), [])
        self.assertFalse(self.c._stop_locked)

    def test_plan_rejection_while_local_target_is_paused_defers_rx_without_stop(self):
        self.ready_session()
        self.g.set_target(self.target(dx=1))
        self.g.tick()
        pending = self.take()
        self.assertFalse(self.g.set_target([math.nan] + self.g.target_tcp[1:]))
        self.feed("queued MOVEL id=42 mode=INT", "error MOVEL id=42 reason=PLAN_FAILED")
        self.assertIsNotNone(self.g._stream)
        self.assertEqual(len(self.write(pending)), 1)
        self.assertIsNone(self.g._stream)
        self.assertEqual(self.g.stage, "target_rejected")
        self.assertEqual(self.g.target_tcp, self.seed_tcp)
        self.assertEqual(self.take(), [])
        self.assertFalse(self.c._stop_locked)
        self.assertIsNone(self.g.last_stop)
        self.clock.advance(.02)
        self.sample()
        self.g.tick()
        self.assertTrue(self.g.ready)
        self.assertEqual(self.take(), [])

    def test_other_stream_error_still_stops(self):
        self.ready_session()
        self.g.set_target(self.target(dx=1))
        self.g.tick()
        self.write()
        self.feed("queued MOVEL id=42 mode=INT", "error MOVEL id=42 reason=MODE_UNSUPPORTED")
        self.assertEqual(self.write(), ["!STOP"])
        self.assertEqual(self.g.last_stop["source"], "motion_error")

    def test_stop_log_captures_timeout_once_and_keeps_cause_after_confirmation(self):
        records = []
        self.c.log_message.connect(lambda message, tag: records.append((message, tag)))
        self.ready_session()
        self.clock.advance(.501)
        self.g.tick()
        self.g.tick()
        self.assertEqual(self.write(), ["!STOP"])
        stops = [json.loads(text.removeprefix("GAMEPAD_STOP ")) for text, tag in records if text.startswith("GAMEPAD_STOP ")]
        self.assertEqual(len(stops), 1)
        record = stops[0]
        self.assertEqual(record["source"], "input_timeout")
        self.assertEqual(record["stage"], "ready")
        self.assertGreaterEqual(record["input_age_s"], .5)
        self.assertIn("accepted MOVEJ", record["last_rx"]["line"])
        self.feed("STOP locked")
        self.assertIn("手柄输入超时", self.g.message)
        snapshot = self.g.state()
        snapshot["last_stop"]["reason"] = "changed"
        self.assertEqual(self.g.last_stop["reason"], "手柄输入超时")

    def test_stopping_handover_preserves_original_reason_and_pending_motion_evidence(self):
        self.ready_session()
        self.g.set_target(self.target(dx=1))
        self.g.tick()
        self.write()
        self.feed("queued MOVEL id=42 mode=INT")
        self.g.release("切换至界面控制", source="ui_suspend", request_stop=True)
        record = self.g.last_stop
        self.assertEqual(record["source"], "ui_suspend")
        self.assertEqual(record["pending"][0]["id"], 42)
        self.assertTrue(record["last_tx"]["command"].startswith("@"))
        self.assertEqual(self.write(), ["!STOP"])
        self.feed("STOP locked")
        self.assertFalse(self.g.claimed)
        self.assertIn("切换至界面控制", self.g.message)

    def test_button_and_sampler_error_have_distinct_stop_evidence(self):
        self.ready_session()
        self.clock.advance(.02)
        self.g.handle_input({"connected": True, "timestamp": self.clock(),
                             "decoded": {"centered": True}, "pressed": ["stop"],
                             "raw": {"buttons": [0, 0, 0, 0, 0, 0, 1]}})
        self.assertEqual(self.g.last_stop["source"], "gamepad_button")
        self.assertEqual(self.g.last_stop["buttons"][6], 1)
        self.assertEqual(self.write(), ["!STOP"])

    def test_lost_sampler_reason_is_logged_before_state_is_cleared(self):
        self.ready_session()
        self.g.handle_input({"connected": False, "timestamp": self.clock(), "error": "SDL device removed"})
        self.assertEqual(self.g.last_stop["source"], "input_lost")
        self.assertEqual(self.g.last_stop["input_error"], "SDL device removed")
        self.assertEqual(self.write(), ["!STOP"])


if __name__ == "__main__":
    unittest.main()
