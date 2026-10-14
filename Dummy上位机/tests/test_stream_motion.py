"""Pure offline 50 Hz wire and reconstructed-curve tests; no serial hardware."""
import binascii
import struct
import unittest
import numpy as np
from dummy_motion import RobotModel, Pose, StreamPlanner, StreamKnot
from dummy_motion.frames import from_wire_joints, error
from dummy_motion.stream_protocol import (motor_coordinates, robot_coordinates,
    decode_knot, parse_stream_line, MOTOR_SCALE, MOTOR_ZERO, STREAM_SEED, STREAM_END)
from dummy_motion.stream_curve import quintic_coefficients, evaluate_segment


class StreamProtocolTests(unittest.TestCase):
    def setUp(self):
        self.q = from_wire_joints([0, 10, 80, 0, 90, 0, 37.25])

    def test_firmware_home_sign_reduction_and_rail_golden(self):
        values = motor_coordinates(self.q)
        np.testing.assert_array_equal(values[:, 0], [0, -604444, 711111, 0, -640000, 0, -381440])
        np.testing.assert_array_equal(motor_coordinates(MOTOR_ZERO), np.zeros((7, 3)))
        q, v, a = robot_coordinates(values)
        np.testing.assert_allclose(q, self.q, atol=1.3e-6)
        np.testing.assert_array_equal(v, np.zeros(7))

    def test_exact_little_endian_size_crc_and_roundtrip(self):
        knot = StreamKnot(0, motor_coordinates(self.q), STREAM_SEED)
        line = knot.encode(0x1234)
        self.assertEqual(len(line), 186)
        raw = bytes.fromhex(line[2:])
        self.assertEqual(raw[:6], b'\x02\x01\x34\x12\0\0')
        self.assertEqual(struct.unpack_from('<i', raw, 18)[0], -604444)
        self.assertEqual(struct.unpack_from('<H', raw, 90)[0], binascii.crc_hqx(raw[:90], 0xffff))
        session, decoded = decode_knot(line)
        self.assertEqual(session, 0x1234)
        np.testing.assert_array_equal(decoded.motor, knot.motor)

    def test_corruption_every_single_bit_rejected(self):
        raw = bytes.fromhex(StreamKnot(0, motor_coordinates(self.q), STREAM_SEED).encode(5)[2:])
        for byte in range(len(raw)):
            for bit in range(8):
                bad = bytearray(raw); bad[byte] ^= 1 << bit
                with self.assertRaises(ValueError):
                    decode_knot('~S'+bad.hex())

    def test_flags_session_sequence_and_int32_ranges_fail_closed(self):
        values = motor_coordinates(self.q)
        for seq, flags in [(0, 0), (0, 3), (1, 1), (1, 4), (65535, 0), (-1, 0)]:
            with self.assertRaises(ValueError):
                StreamKnot(seq, values, flags)
        for session in (0, 65536, -1, 1.5):
            with self.assertRaises(ValueError):
                StreamKnot(0, values, 1).encode(session)
        for invalid in (2**31, -(2**31)-1, np.nan, .5):
            bad = values.astype(float); bad[1, 1] = invalid
            with self.assertRaises(ValueError):
                StreamKnot(1, bad)
        for flags in (1, 2):
            bad = values.copy(); bad[0, 1] = 1
            with self.assertRaises(ValueError):
                StreamKnot(0 if flags == 1 else 1, bad, flags)

    def test_stationary_knots_have_distinct_clock_sequence(self):
        values = motor_coordinates(self.q)
        left = StreamKnot(1, values).encode(7)
        right = StreamKnot(2, values).encode(7)
        self.assertNotEqual(left, right)
        np.testing.assert_array_equal(decode_knot(left)[1].motor, decode_knot(right)[1].motor)

    def test_reply_contract_bounds_and_optional_diagnostics(self):
        self.assertEqual(parse_stream_line('STREAM_CAP v=1 hz=50 mode=7 mask=127')['mask'], 127)
        self.assertEqual(parse_stream_line('STREAM_STATE s=8 st=ARMED next=10 free=3')['state'], 'ARMED')
        self.assertEqual(parse_stream_line('STREAM_ACK s=8 n=9 free=3')['sequence'], 9)
        self.assertEqual(parse_stream_line('STREAM_DONE s=8 n=109')['kind'], 'done')
        self.assertEqual(parse_stream_line('STREAM_DIAG commit=8 exec=3 gap_ms=6 poll_us=210')['poll_us'], 210)
        self.assertEqual(parse_stream_line('STREAM_DIAG c=8 e=3 gap=6 us=210')['poll_us'], 210)
        self.assertEqual(parse_stream_line('STREAM_NODE node=3 n=1 field=0 value=-600000')['value'], -600000)
        self.assertIsNone(parse_stream_line('ok 0 1 2 3 4 5 6'))
        self.assertEqual(parse_stream_line('error STREAM busy')['kind'], 'rejected')
        self.assertEqual(parse_stream_line('STREAM_FAULT_AT s=2 src=1 st=3 n=2 age=12')['kind'], 'fault_context')
        self.assertEqual(parse_stream_line('STREAM_NODE node=3 n=1 field=7 value=-600000')['value'], -600000)
        for line in ('STREAM_STATE s=8 st=OPEN next=0 free=17', 'STREAM_ACK s=70000 n=1 free=2', 'STREAM_ACK nonsense'):
            with self.assertRaises(ValueError):
                parse_stream_line(line)


class StreamCurveTests(unittest.TestCase):
    def setUp(self):
        self.model = RobotModel()
        self.q = from_wire_joints([0, 10, 80, 0, 90, 0, 0])
        self.planner = StreamPlanner(self.model)

    def test_line_20ms_clock_zero_boundary_and_rail_participation(self):
        pose = self.model.fk(self.q)
        plan = self.planner.plan_line(self.q, Pose(pose.position+[0, .02, 0], pose.rotation))
        self.assertAlmostEqual(plan.duration, (len(plan.knots)-1)*.02)
        self.assertEqual(plan.knots[0].flags, STREAM_SEED)
        self.assertEqual(plan.knots[-1].flags, STREAM_END)
        self.assertGreater(abs(plan.final_q[6]), .001)
        for metric in (plan.maximum_velocity_ratio, plan.maximum_acceleration_ratio, plan.maximum_jerk_ratio):
            self.assertLessEqual(metric, 1.00001)
        for i, knot in enumerate(plan.knots):
            self.assertEqual(knot.sequence, i)
            if i in (0, len(plan.knots)-1):
                np.testing.assert_array_equal(knot.motor[:, 1:], 0)
        self.assertLess(np.linalg.norm(error(self.model.fk(plan.final_q), Pose(pose.position+[0, .02, 0], pose.rotation))[:3]), .00001)

    def test_quintic_matches_positions_velocities_accelerations_at_every_boundary(self):
        q2 = self.q.copy(); q2[0] += .04; q2[6] += .002
        plan = self.planner.plan_joint(self.q, q2)
        for left, right in zip(plan.knots, plan.knots[1:]):
            for order in range(3):
                actual = evaluate_segment(left.motor, right.motor, np.array([0., 1.]), order)
                np.testing.assert_allclose(actual[0], left.motor[:, order], atol=2e-6)
                np.testing.assert_allclose(actual[1], right.motor[:, order], atol=2e-6)

    def test_all_20khz_ticks_stay_in_bounds_and_limit_velocity(self):
        q2 = self.q.copy(); q2[0] += .025; q2[6] += .001
        plan = self.planner.plan_joint(self.q, q2, 80)
        for left, right in zip(plan.knots, plan.knots[1:]):
            fraction = np.arange(401)/400
            q = evaluate_segment(left.motor, right.motor, fraction)/MOTOR_SCALE+MOTOR_ZERO
            v = evaluate_segment(left.motor, right.motor, fraction, 1)/MOTOR_SCALE
            a = evaluate_segment(left.motor, right.motor, fraction, 2)/MOTOR_SCALE
            self.assertTrue(np.all(q >= self.model.constraints.lower-1e-10))
            self.assertTrue(np.all(q <= self.model.constraints.upper+1e-10))
            self.assertTrue(np.all(abs(v) <= self.model.constraints.velocity*1.00001))
            self.assertTrue(np.all(abs(a) <= self.model.constraints.acceleration*1.00001))

    def test_speed_changes_curve_time_without_movej_speed_argument(self):
        pose = self.model.fk(self.q)
        target = Pose(pose.position+[0, .01, 0], pose.rotation)
        # Pure offline planner limits must not depend on a retired UI backend.
        from dummy_motion import MotionLimits
        slow_limits = MotionLimits(linear_speed=.02*.2, angular_speed=np.deg2rad(10)*.2)
        fast_limits = MotionLimits(linear_speed=.02, angular_speed=np.deg2rad(10))
        slow = self.planner.plan_line(self.q, target, slow_limits)
        fast = self.planner.plan_line(self.q, target, fast_limits)
        self.assertGreater(slow.duration, fast.duration*2)
        self.assertTrue(all(k.encode(8).startswith('~S') for k in fast.knots))
        np.testing.assert_allclose(slow.final_q, fast.final_q, atol=1e-4)

    def test_cancellation_unreachable_and_limits_do_not_produce_stream(self):
        pose = self.model.fk(self.q)
        with self.assertRaises(ValueError):
            self.planner.plan_line(self.q, pose, cancelled=lambda: True)
        with self.assertRaises(ValueError):
            self.planner.plan_line(self.q, Pose([8, 0, 0], np.eye(3)))
        bad = self.q.copy(); bad[6] = .3
        with self.assertRaises(ValueError):
            self.planner.plan_joint(self.q, bad)

    def test_actual_motor_velocity_cap_retimes_every_bernstein_interval(self):
        q2 = self.q.copy(); q2[0] += .03
        fast = self.planner.plan_joint(self.q, q2, 100)
        caps = np.full(7, .2)
        limited = StreamPlanner(self.model, motor_acceleration=[30]*7, motor_velocity=caps).plan_joint(self.q, q2, 100)
        self.assertGreater(limited.duration, fast.duration*2)
        for left, right in zip(limited.knots, limited.knots[1:]):
            l, r = left.motor, right.motor
            dp = r[:, 0]-l[:, 0]
            b = np.array([np.zeros(7, dtype=np.int64), 200*l[:, 1], 400*l[:, 1]+l[:, 2],
                          50000*dp-400*r[:, 1]+r[:, 2], 50000*dp-200*r[:, 1], 50000*dp])
            self.assertTrue(np.all(abs(np.diff(b, axis=0)) <= 200*caps*51200*.80001))

    def test_stationary_cartesian_request_is_one_clocked_20ms_segment(self):
        plan = self.planner.plan_line(self.q, self.model.fk(self.q))
        self.assertEqual(len(plan.knots), 2)
        self.assertEqual(plan.duration, .02)
        self.assertEqual([k.flags for k in plan.knots], [1, 2])
        np.testing.assert_array_equal(plan.knots[0].motor, plan.knots[1].motor)

    def test_stationary_finite_trajectory_still_has_end(self):
        plan = self.planner.plan_joint(self.q, self.q)
        self.assertGreaterEqual(len(plan.knots), 2)
        self.assertEqual(plan.knots[-1].flags, 2)
        for knot in plan.knots:
            np.testing.assert_array_equal(knot.motor, plan.knots[0].motor)


if __name__ == '__main__':
    unittest.main()
