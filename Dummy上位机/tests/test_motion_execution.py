"""Motor timing contract checks; no Qt, serial or hardware."""
import math
import unittest
import numpy as np
from dummy_motion import segment_timing
from dummy_motion.execution import motor_distances
from dummy_motion.frames import from_wire_joints


class ExecutionTests(unittest.TestCase):
    def test_reduction_and_rail_units(self):
        q = from_wire_joints([7.2]*5+[12,5])
        np.testing.assert_allclose(motor_distances(np.zeros(7),q),np.ones(7))

    def test_one_revolution_has_known_trapezoid_duration(self):
        q = from_wire_joints([7.2,0,0,0,0,0,0])
        timing = segment_timing(np.zeros(7),q,1.,[30.]*7)
        expected = (30.-math.sqrt(780.))/2/.3
        self.assertAlmostEqual(timing.speed_percent,math.ceil(expected*100)/100)
        self.assertLessEqual(timing.predicted_seconds,1.)
        self.assertGreater(timing.predicted_seconds,.99)

    def test_small_distance_reports_protocol_floor(self):
        q = from_wire_joints([.01,0,0,0,0,0,0])
        t = segment_timing(np.zeros(7),q,.1,[30.]*7)
        self.assertEqual(t.speed_percent,1.)
        self.assertTrue(t.minimum_speed_limited)
        self.assertLess(t.predicted_seconds,.02)

    def test_unreachable_duration_reports_physical_limit(self):
        q = from_wire_joints([7.2,0,0,0,0,0,0])
        t = segment_timing(np.zeros(7),q,.01,[30.]*7)
        self.assertTrue(t.duration_limited)
        self.assertAlmostEqual(t.predicted_seconds,2*math.sqrt(1/30))

    def test_velocity_ceiling_reports_limit(self):
        q = from_wire_joints([0,0,0,0,0,0,1000])
        t = segment_timing(np.zeros(7),q,.1,[30.]*7)
        self.assertEqual(t.speed_percent,100.)
        self.assertTrue(t.duration_limited)

    def test_no_motion(self):
        t = segment_timing(np.zeros(7),np.zeros(7),.1,[30.]*7)
        self.assertEqual(t.predicted_seconds,0.)
        self.assertFalse(t.minimum_speed_limited or t.duration_limited)

    def test_invalid_input_rejected(self):
        for duration in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                segment_timing(np.zeros(7),np.ones(7),duration,[30.]*7)
        for acceleration in ([0.]*7,[-1.]*7,[float('nan')]*7,[30.]*6):
            with self.assertRaises(ValueError):
                segment_timing(np.zeros(7),np.ones(7),.1,acceleration)

    def test_seven_axes_against_independent_time_search(self):
        # Find the smallest ceiling by bisection of the forward trapezoid
        # model, independently of the inverse formula under test.
        rng = np.random.default_rng(1120)
        for _ in range(300):
            q = rng.uniform(-.01,.01,7)
            a = rng.uniform(1.,100.,7)
            requested = rng.uniform(.02,.5)
            d = motor_distances(np.zeros(7),q)
            def forward(speed):
                v = speed*.3
                times = []
                for distance,acc in zip(d,a):
                    if distance < 1e-6:
                        continue
                    ta = min(v/acc,math.sqrt(distance/acc))
                    cruise = max(0.,distance-acc*ta*ta)/v
                    times.append(2*ta+cruise)
                return max(times,default=0.)
            goal = max(requested,forward(100.))
            lo,hi = 0.,100.
            for _ in range(60):
                mid = (lo+hi)/2
                if forward(mid)>goal+1e-12: lo=mid
                else: hi=mid
            expected = min(100.,max(1.,math.ceil((hi-1e-8)*100)/100))
            t = segment_timing(np.zeros(7),q,requested,a)
            # At the triangular/trapezoidal boundary forward time is
            # flat to floating-point precision; allow one wire quantum.
            self.assertLessEqual(abs(t.speed_percent-expected),.010000001)
            self.assertAlmostEqual(t.predicted_seconds,forward(t.speed_percent))
