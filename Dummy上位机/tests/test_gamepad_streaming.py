"""Asynchronous gamepad MoveL cadence, proportional rates and receipt isolation."""
import math
import unittest

from PySide6.QtCore import QCoreApplication

from tests.test_gamepad_session import GamepadFixture


class GamepadStreamingTests(GamepadFixture, unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QCoreApplication.instance() or QCoreApplication([])

    def setUp(self):
        self.setup_gamepad()
        self.g.set_config({"robot": {"tool_offset_z": 0},
                           "control": {"velocity_smoothing_s": 0}})
        self.sample()
        self.ready_session()

    def tearDown(self):
        self.c.close()

    def move(self, dx, advance=0):
        self.clock.advance(advance)
        self.sample()
        self.assertTrue(self.g.set_target(self.target(dx)))
        self.g.tick()
        return self.write()

    def test_no_receipts_still_stream_at_default_cadence_and_full_stick_200(self):
        packets = []
        times = []
        for _ in range(150):
            self.clock.advance(.017)
            self.sample(ly=-1, centered=False)
            self.g.tick()
            sent = self.write()
            if sent:
                self.assertEqual(len(sent), 1)
                self.assertTrue(sent[0].startswith("@"))
                packets.append(sent[0])
                times.append(self.clock())
            self.assertIsNone(self.g._stream)  # A written packet never waits in the send slot.
        self.assertGreater(len(packets), 20)
        self.assertTrue(all(.1 <= b - a <= .12 for a, b in zip(times, times[1:])))
        self.assertAlmostEqual(self.g.target_tcp[0] - self.seed_tcp[0], 200 * 150 * .017, places=5)
        self.assertEqual(len(self.g._streams), len(packets))
        self.assertEqual(self.g.command_reference_tcp, self.seed_tcp)
        self.assertIsNone(self.g.actual_tcp)
        self.assertFalse(self.c._stop_locked)

    def test_quarter_stick_is_50_and_overshoot_is_clipped_without_dropping_sample(self):
        self.clock.advance(.02)
        self.sample(ly=-.25, centered=False)
        self.assertAlmostEqual(self.g.target_tcp[0] - self.seed_tcp[0], 50 * .02)
        self.assertTrue(self.g.set_target(self.target(100)))
        self.assertAlmostEqual(self.g.target_tcp[0] - self.seed_tcp[0], 40)
        self.g.tick()
        self.assertEqual(len(self.write()), 1)
        self.clock.advance(.02)
        self.sample(ly=-1, centered=False)
        self.assertGreater(self.g.target_tcp[0] - self.seed_tcp[0], 40)

    def test_smoothing_reaches_full_rate_and_center_discards_unsent_displacement(self):
        self.g.control["velocity_smoothing_s"] = .06
        increments = []
        for _ in range(60):
            previous = self.g.target_tcp[0]
            self.clock.advance(.02)
            self.sample(ly=-1, centered=False)
            increments.append(self.g.target_tcp[0] - previous)
            self.g.tick()
            self.write()
        self.assertGreater(increments[0], 0)
        self.assertLess(increments[0], 4)
        self.assertAlmostEqual(increments[-1], 4, places=5)
        self.clock.advance(.02)
        self.sample()
        self.assertEqual(self.g._velocity, [0.] * 6)
        self.assertEqual(self.g.target_tcp, self.g.written_reference_tcp)
        self.g.tick()
        self.assertEqual(self.write(), [])

    def test_late_and_early_receipts_match_jobs_without_regressing_newer_reference(self):
        self.g.set_target(self.target(1))
        self.g.tick()
        first = self.take()
        self.feed("queued MOVEL id=42 mode=INT")
        self.write(first)
        self.clock.advance(.11)
        self.sample()
        self.g.set_target(self.target(2))
        self.g.tick()
        second = self.take()
        self.feed("queued MOVEL id=43 mode=INT", "accepted MOVEL id=43")
        self.assertEqual(self.g.command_reference_tcp, self.seed_tcp)
        self.write(second)
        newer = list(self.g.command_reference_tcp)
        self.feed("accepted MOVEL id=42", "queued MOVEL id=42 mode=INT")
        self.assertEqual(self.g.command_reference_tcp, newer)
        self.assertEqual(self.g.written_reference_tcp, newer)
        self.assertEqual(self.g._streams, [])
        self.assertTrue(self.g.ready)

    def test_oldest_unconfirmed_receipt_times_out_even_while_new_packets_are_sent(self):
        self.move(1)
        self.feed("queued MOVEL id=42 mode=INT")
        self.move(2, .11)
        self.feed("queued MOVEL id=43 mode=INT")
        self.clock.advance(2.95)
        self.sample()
        self.g.tick()
        self.assertEqual(self.write(), ["!STOP"])
        self.assertEqual(self.g.last_stop["source"], "command_timeout")
        self.assertEqual(self.g._streams, [])
        self.feed("accepted MOVEL id=42", "accepted MOVEL id=43")
        self.assertIsNone(self.g.command_reference_tcp)

    def test_delayed_writer_has_one_queued_target_and_stop_invalidates_it(self):
        self.g.set_target(self.target(1))
        self.g.tick()
        first = self.take()
        for _ in range(100):
            self.clock.advance(.017)
            self.sample(ly=-1, centered=False)
            self.g.tick()
        self.assertEqual(self.take(), [])
        self.assertEqual(len(self.g._streams), 1)
        self.assertLessEqual(math.dist(self.g.target_tcp[:3], self.seed_tcp[:3]), 40.000001)
        self.g.stop()
        self.assertFalse(self.c._can_write(*first[0]))
        self.assertEqual(self.write(), ["!STOP"])

    def test_older_plan_rejection_does_not_cancel_a_newer_written_target(self):
        self.move(1)
        self.feed("queued MOVEL id=42 mode=INT")
        self.move(2, .11)
        self.feed("queued MOVEL id=43 mode=INT", "accepted MOVEL id=43")
        reference = list(self.g.command_reference_tcp)
        self.feed("error MOVEL id=42 reason=PLAN_FAILED")
        self.assertTrue(self.g.ready)
        self.assertEqual(self.g.command_reference_tcp, reference)
        self.assertEqual(self.g._streams, [])

    def test_latest_plan_rejection_retires_a_queued_successor_without_sending_stop(self):
        self.move(1)
        self.feed("queued MOVEL id=42 mode=INT")
        self.clock.advance(.11)
        self.sample()
        self.g.set_target(self.target(2))
        self.g.tick()
        successor = self.take()
        self.feed("error MOVEL id=42 reason=PLAN_FAILED")
        self.assertEqual(self.g.stage, "target_rejected")
        self.assertFalse(self.c._can_write(*successor[0]))
        self.assertEqual(self.g.target_tcp, self.seed_tcp)
        self.assertEqual(self.take(), [])

    def test_late_valid_acceptance_updates_rejection_fallback_without_automatic_retry(self):
        self.move(1)
        self.feed("queued MOVEL id=42 mode=INT")
        self.move(2, .11)
        self.feed("queued MOVEL id=43 mode=INT", "error MOVEL id=43 reason=PLAN_FAILED")
        self.assertEqual(self.g.stage, "target_rejected")
        self.feed("accepted MOVEL id=42")
        self.assertAlmostEqual(self.g.target_tcp[0], 147.5)  # Accepted wire target, rounded to 0.1 mm.
        self.assertEqual(self.g.target_tcp, self.g.command_reference_tcp)
        self.assertEqual(self.g.written_reference_tcp, self.g.command_reference_tcp)
        self.assertEqual(self.g.stage, "target_rejected")
        self.g.tick()
        self.assertEqual(self.take(), [])

    def test_writer_rechecks_current_mode_before_sending_a_queued_target(self):
        self.g.set_target(self.target(1))
        self.g.tick()
        pending = self.take()
        self.c.current_cmdmode = 3
        self.assertFalse(self.c._can_write(*pending[0]))
        self.c.current_cmdmode = 2
        self.assertEqual(len(self.write(pending)), 1)

    def test_unconfirmed_history_is_bounded_even_with_a_long_response_timeout(self):
        self.g.control["response_timeout_s"] = 30
        for index in range(self.g.STREAM_HISTORY_LIMIT):
            self.assertEqual(len(self.move(index + 1, .11)), 1)
        self.assertEqual(len(self.g._streams), 64)
        self.assertEqual(self.move(65, .11), ["!STOP"])
        self.assertEqual(self.g.last_stop["source"], "stream_backlog")
        self.assertEqual(self.g._streams, [])

    def test_periodic_ui_updates_are_throttled_and_stop_is_immediate(self):
        states, targets = [], []
        self.g.state_changed.connect(states.append)
        self.g.target_changed.connect(targets.append)
        for _ in range(100):
            self.clock.advance(.01)
            self.sample(ly=-.1, centered=False)
            self.g.tick()
            self.write()
        self.assertLessEqual(len(states), 21)
        self.assertLessEqual(len(targets), 21)
        self.clock.advance(.01)
        self.sample(("stop",))
        self.assertEqual(states[-1]["stage"], "stopping")
        self.assertEqual(self.write(), ["!STOP"])


if __name__ == "__main__":
    unittest.main()
