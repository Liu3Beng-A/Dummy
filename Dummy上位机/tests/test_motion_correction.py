"""Measured-state correction, including wire limits and stale samples."""
import unittest
import numpy as np
from dummy_motion import RobotModel, Tool, Pose, Sample, FeedbackSample, Status
from dummy_motion.frames import from_wire_joints, error
from dummy_motion.feedback import FeedbackTracker
from dummy_motion.correction import correct_sample


class CorrectionTests(unittest.TestCase):
    def setUp(self):
        self.model, self.tool = RobotModel(), Tool()
        self.q = from_wire_joints([0,10,80,0,90,0,0])
        start = self.model.fk(self.q,self.tool)
        self.target = Sample(.2,self.q,Pose(start.position+[0,.001,0],start.rotation))

    def solve(self, q, stamp=10., now=10., dt=.2, cancelled=None):
        tracker = FeedbackTracker(self.model)
        tracker.update(FeedbackSample(q,stamp))
        return correct_sample(self.model,self.target,self.tool,tracker,now,self.q,np.zeros(7),dt,35,cancelled)

    def test_measured_deviation_changes_joint_allocation_but_keeps_target(self):
        status, nominal = self.solve(self.q)
        measured = self.q.copy(); measured[6] += .0005
        status2, corrected = self.solve(measured)
        self.assertEqual((status,status2),(Status.OK,Status.OK))
        self.assertGreater(np.linalg.norm(nominal.q-corrected.q),1e-5)
        for sample in (nominal,corrected):
            residual = error(self.model.fk(sample.q,self.tool),self.target.pose)
            self.assertLess(np.linalg.norm(residual[:3]),.0002)
            self.assertLess(np.linalg.norm(residual[3:]),np.deg2rad(.1))
            velocity = (sample.q-self.q)/.2
            self.assertTrue(np.all(np.abs(velocity)<=self.model.constraints.velocity))
            self.assertTrue(np.all(np.abs(velocity)/.2<=self.model.constraints.acceleration))

    def test_stale_and_cancelled_never_return_a_candidate(self):
        status,sample=self.solve(self.q,now=10.31)
        self.assertEqual(status,Status.FEEDBACK_STALE)
        self.assertIsNone(sample)
        status,sample=self.solve(self.q,cancelled=lambda:True)
        self.assertEqual(status,Status.CANCELLED)
        self.assertIsNone(sample)

    def test_large_correction_cannot_jump_joint_limits(self):
        start=self.model.fk(self.q,self.tool)
        self.target=Sample(.1,self.q,Pose(start.position+[0,.04,0],start.rotation))
        status,sample=self.solve(self.q,dt=.1)
        self.assertIsNone(sample)
        self.assertNotEqual(status,Status.OK)

if __name__=='__main__':unittest.main()
