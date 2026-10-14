"""Numerical/physical contract checks; no Qt, serial ports, or robot required."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from dummy_motion import (RobotModel, Tool, Pose, Constraints, MotionLimits, IKSolver,
                          LinePlanner, MotionSession, FeedbackSample, RailPolicy, Status)
from dummy_motion.frames import (from_wire_joints, to_wire_joints, from_xyzabc,
                                 to_xyzabc, compose, inverse, error, interpolate)
from dummy_motion.feedback import FeedbackTracker
from dummy_motion.validation import movej_command


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.model = RobotModel()
        self.q = from_wire_joints([0, 10, 80, 0, 90, 0, 0])

    def test_firmware_golden_flange_poses(self):
        tool = Tool(Pose.identity())
        cases = [([0, 10, 80, 0, 90, 0, 0], [146.520190203, 0, 331.217318012, 180, 0, 180]),
                 ([0, -75, 180, 0, 0, 0, 0], [75.943473212, 0, 258.902702221, 180, 75, 180]),
                 ([0]*7, [-69.5, 0, 607.3, 0, 0, 0]),
                 ([15, -30, 120, 25, 20, -35, 0], [129.249304975, 51.541896535, 388.997084972, -155.341912232, 70.057488940, -130.226882438])]
        for joints, pose in cases:
            residual = error(self.model.fk(from_wire_joints(joints), tool), from_xyzabc(pose))
            np.testing.assert_allclose(residual, 0, atol=1e-10)

    def test_geometric_jacobian_matches_spatial_finite_difference(self):
        rng = np.random.default_rng(105)
        tool = Tool(from_xyzabc([20, -10, 114, 8, 5, -7]))
        for _ in range(15):
            q = rng.uniform(self.model.constraints.lower+.001, self.model.constraints.upper-.001)
            j = self.model.jacobian(q, tool)
            base = self.model.fk(q, tool)
            for i in range(7):
                shifted = q.copy()
                shifted[i] += 1e-7
                numeric = error(base, self.model.fk(shifted, tool))/1e-7
                np.testing.assert_allclose(j[:, i], numeric, atol=5e-7)

    def test_world_rail_and_rotated_tool_round_trip(self):
        model = RobotModel(world_rail=from_xyzabc([100, 80, 60, 10, 20, 30]))
        q = self.q.copy(); q[6] = .08
        tool = Tool(from_xyzabc([10, -20, 114, 20, 5, 10]))
        flange = model.fk(q, Tool(Pose.identity()), frame="base")
        target = model.legacy_target(flange, q[6], tool)
        np.testing.assert_allclose(error(model.fk(q, tool), target), 0, atol=1e-12)
        np.testing.assert_allclose(error(model.native_target(target, q[6], tool), flange), 0, atol=1e-12)

    def test_tcp_z_uses_flange_orientation_not_world_z(self):
        self.assertAlmostEqual(self.model.fk(self.q).position[2]*1000, 217.217318012, places=7)
        q = self.q.copy(); q[6] = .2
        np.testing.assert_allclose(self.model.fk(q).position-self.model.fk(self.q).position, [0, .2, 0])

    def test_pose_and_input_snapshots_and_validation(self):
        values = [1., 2., 3.]
        pose = Pose(values, np.eye(3)); values[0] = 40
        self.assertEqual(pose.position[0], 1.)
        with self.assertRaises(ValueError): pose.position[0] = 2
        for bad in ([np.nan]*7, [0]*6):
            with self.assertRaises(ValueError): self.model.fk(bad)
        with self.assertRaises(ValueError): Pose([0, 0, 0], -np.eye(3))
        with self.assertRaises(ValueError): Constraints(velocity=np.zeros(7))

    def test_rotation_interpolation_shortest_arc_and_gimbal_roundtrip(self):
        left, right = from_xyzabc([0, 0, 0, 0, 0, 179]), from_xyzabc([0, 0, 0, 0, 0, -179])
        self.assertAlmostEqual(np.linalg.norm(error(left, interpolate(left, right, .5))[3:]), np.deg2rad(1), places=10)
        pose = from_xyzabc([0, 0, 0, 30, 90, -40])
        np.testing.assert_allclose(error(from_xyzabc(to_xyzabc(pose)), pose), 0, atol=1e-12)


class SolverPlannerTests(unittest.TestCase):
    setUp = KinematicsTests.setUp

    def test_nearby_reachable_targets_continuous_and_rail_participates(self):
        solver = IKSolver(self.model)
        q = self.q
        start = self.model.fk(q)
        for y in np.linspace(.001, .06, 30):
            target = Pose(start.position+[0, y, 0], start.rotation)
            result = solver.solve_pose(target, q)
            self.assertTrue(result.ok, result)
            self.assertTrue(self.model.valid(result.q))
            self.assertLess(np.max(np.abs(result.q[:6]-q[:6])), .12)
            self.assertLess(result.position_error, .0001)
            q = result.q
        self.assertGreater(abs(q[6]), .001)

    def test_locked_rail_and_automatic_near_rail_limit(self):
        q = self.q.copy(); q[6] = .2499
        initial = self.model.fk(q)
        target = Pose(initial.position+[0, .005, 0], initial.rotation)
        for policy in (RailPolicy.LOCKED, RailPolicy.AUTO):
            result = IKSolver(self.model).solve_pose(target, q, rail_policy=policy)
            self.assertTrue(result.ok, result)
            self.assertLessEqual(result.q[6], .25)
            if policy == RailPolicy.LOCKED: self.assertEqual(result.q[6], q[6])

    def test_unreachable_and_cancellation_never_produce_executable_plan(self):
        target = Pose([5, 0, 1], np.eye(3))
        result = LinePlanner(self.model).plan_line(self.q, target, MotionLimits(max_samples=10))
        self.assertFalse(result.ok)
        self.assertEqual(result.samples, ())
        result = IKSolver(self.model).solve_pose(target, self.q, cancelled=lambda: True)
        self.assertEqual(result.status, Status.CANCELLED)
        result = IKSolver(self.model).solve_pose(target, self.q, budget_ms=.00001)
        self.assertEqual(result.status, Status.BUDGET_EXCEEDED)
        from unittest.mock import patch
        with patch("dummy_motion.ik.time.perf_counter", side_effect=[0., .02]):
            result = IKSolver(self.model).solve_pose(self.model.fk(self.q), self.q, budget_ms=10)
        self.assertEqual(result.status, Status.BUDGET_EXCEEDED)
        result = IKSolver(self.model).solve_pose(target, self.q)
        self.assertFalse(result.ok)

    def test_line_quantization_midpoints_and_dynamic_limits(self):
        start = self.model.fk(self.q)
        target = Pose(start.position+[.008, .018, -.005], Rotation.from_rotvec([.01, -.02, .02]).as_matrix()@start.rotation)
        plan = LinePlanner(self.model).plan_line(self.q, target)
        self.assertTrue(plan.ok, plan.reason)
        for sample in plan.samples:
            command, q = movej_command(self.model, sample.q, sample.pose, Tool())
            self.assertLessEqual(len(command), 63)
            self.assertTrue(self.model.valid(q))
            self.assertLess(np.linalg.norm(error(self.model.fk(q), sample.pose)[:3]), .0002)
        for left, right in zip(plan.samples, plan.samples[1:]):
            midpoint = interpolate(left.pose, right.pose, .5)
            e = error(self.model.fk((left.q+right.q)/2), midpoint)
            self.assertLess(np.linalg.norm(e[:3]), .0003)
            self.assertLess(np.linalg.norm(e[3:]), np.deg2rad(.15))
        dt = plan.samples[1].time-plan.samples[0].time
        velocity = np.diff(np.stack([sample.q for sample in plan.samples]), axis=0)/dt
        accel = np.diff(np.vstack((np.zeros(7), velocity, np.zeros(7))), axis=0)/dt
        self.assertTrue(np.all(abs(velocity) <= self.model.constraints.velocity+1e-10))
        self.assertTrue(np.all(abs(accel) <= self.model.constraints.acceleration+1e-10))

    def test_no_idle_redundancy_drift(self):
        target = self.model.fk(self.q)
        result = IKSolver(self.model).solve_pose(target, self.q)
        np.testing.assert_array_equal(result.q, self.q)
        self.assertEqual(result.iterations, 0)

    def test_feedback_order_prediction_limit_and_stale(self):
        tracker = FeedbackTracker(self.model)
        self.assertTrue(tracker.update(FeedbackSample(self.q, 1.)))
        self.assertFalse(tracker.update(FeedbackSample(self.q, .9)))
        shifted = self.q.copy(); shifted[6] = .01
        self.assertTrue(tracker.update(FeedbackSample(shifted, 1.02)))
        status, estimate = tracker.estimate(1.2)
        self.assertEqual(status, Status.OK)
        self.assertLessEqual(estimate[6]-shifted[6], .04*self.model.constraints.velocity[6])
        self.assertEqual(tracker.estimate(1.4)[0], Status.FEEDBACK_STALE)

    def test_online_step_uses_written_reference_and_refuses_stale_feedback(self):
        session = MotionSession(self.model, self.q)
        start = self.model.fk(self.q)
        target = Pose(start.position+[0, .01, 0], start.rotation)
        self.assertEqual(session.step(.02, target, 1.)[0], Status.FEEDBACK_STALE)
        session.update_feedback(FeedbackSample(self.q, 1.))
        status, sample = session.step(.1, target, 1.01)
        self.assertIn(status, (Status.OK, Status.SLOWED))
        self.assertIsNotNone(sample)
        np.testing.assert_array_equal(session.reference_q, self.q)
        session.commit(sample.q, .1)
        np.testing.assert_array_equal(session.reference_q, sample.q)

    def test_online_acceleration_reversal_brakes_instead_of_deadlocking(self):
        session = MotionSession(self.model, self.q)
        session.velocity[6] = .03
        session.update_feedback(FeedbackSample(self.q, 1.))
        start = self.model.fk(self.q)
        target = Pose(start.position+[0, -.01, 0], start.rotation)
        status, sample = session.step(.02, target, 1.01)
        self.assertEqual(status, Status.SLOWED)
        self.assertIsNotNone(sample)
        velocity = (sample.q-self.q)/.02
        self.assertGreater(velocity[6], 0)
        self.assertLess(velocity[6], .03)
        self.assertTrue(np.all(abs(velocity-session.velocity) <= self.model.constraints.acceleration*.02+1e-10))

    def test_equivalent_wrist_turn_keeps_the_nearby_seed_branch(self):
        q = self.q.copy(); q[3] = np.deg2rad(260)
        equivalent = q.copy(); equivalent[3] = np.deg2rad(-100)
        result = IKSolver(self.model).solve_pose(self.model.fk(equivalent), q)
        self.assertTrue(result.ok)
        np.testing.assert_array_equal(result.q, q)


if __name__ == '__main__':
    unittest.main()
