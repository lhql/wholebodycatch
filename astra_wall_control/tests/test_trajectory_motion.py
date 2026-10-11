from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from astra_wall_control.corridor import SafeCorridor, whole_body_radius
from astra_wall_control.body_motion import BoundedBodyMotion
from astra_wall_control.controller import ArmModel, WholeBodyController

ROOT = Path(__file__).resolve().parents[1]


class TrajectoryMotionTest(unittest.TestCase):
    def test_minimum_jerk_containment_continuity_and_extrema(self):
        corridor = SafeCorridor(ROOT/'config/corridor.json', whole_body_radius(ROOT/'models/my/arm.urdf'))
        corridor.plan(np.array([0., 0., 1.6]), np.array([2.7, 0., 1.6]))
        trajectory = corridor.trajectory
        for t in np.linspace(0., trajectory.duration, 2000):
            p, v, a, i = trajectory.sample(t)
            self.assertTrue(corridor.inside(p, corridor.cells[corridor.route_cells[i]]))
            self.assertLessEqual(np.max(np.abs(v)), .120001)
            self.assertLessEqual(np.max(np.abs(a)), .180001)
        for t in trajectory.ends[:-1]:
            for left, right in zip(trajectory.sample(t-1e-6)[:3], trajectory.sample(t+1e-6)[:3]):
                np.testing.assert_allclose(left, right, atol=1e-6)
        for t in (0., trajectory.duration):
            np.testing.assert_allclose(trajectory.sample(t)[1:3], np.zeros((2, 3)), atol=1e-8)
        self.assertLessEqual(trajectory.extrema()[0], .120001)
        # A genuine through-curve has nonzero velocity at the first portal.
        self.assertGreater(np.linalg.norm(trajectory.sample(trajectory.ends[0])[1]), .005)

    def test_body_motion_bounds_and_ramp(self):
        motion = BoundedBodyMotion()
        motion.set_offset([.03, -.02, .02, .1], [.01, .01, 0., -.05])
        np.testing.assert_allclose(motion.sample(0.)[0], [.01, .01, 0., -.05])
        np.testing.assert_allclose(motion.sample(0.)[1], np.zeros(4))
        for t in np.linspace(0., 80., 2000):
            position, velocity = motion.sample(t)
            self.assertTrue(np.all(np.abs(position) <= motion.bounds+1e-9))
            if t > .001:
                numeric = (motion.sample(t+1e-5)[0]-motion.sample(t-1e-5)[0])/2e-5
                np.testing.assert_allclose(velocity, numeric, atol=1e-7)
        with self.assertRaises(ValueError): motion.set_offset([1., 0., 0., 0.])

    def test_stationary_tool_with_joint_body_motion_and_tilt(self):
        model = ArmModel(ROOT/'models/my/arm.urdf'); control = WholeBodyController(model, wall_constraint=False)
        motion = BoundedBodyMotion()
        q = np.array([0., -.05, .1, .05, 0., .01, .01]); control.posture = (model.lower[:5]+model.upper[:5])/2
        centre = np.array([.6, 0., 1.6]); p = centre.copy(); yaw = 0.
        target = model.evaluate(p, np.eye(3), q)[0].copy()
        bounds = np.array([centre-motion.bounds[:3], centre+motion.bounds[:3]])
        errors = []; positions = []; arm = []
        for i in range(2000):
            t = i*.02; desired, feedforward = motion.sample(t)
            R = Rotation.from_euler('xyz', [.015*np.sin(t*.6), .015*np.sin(t*.5), yaw]).as_matrix()
            velocity, tip, _ = control.solve(p, R, q, target, centre+desired[:3], .02,
                                             body_yaw=desired[3], body_velocity=feedforward,
                                             body_bounds=bounds, fixed_tip=True)
            p += velocity[:3]*.02; yaw += velocity[3]*.02; q[:5] += velocity[4:]*.02
            self.assertTrue(np.all(p >= bounds[0]-1e-5) and np.all(p <= bounds[1]+1e-5))
            self.assertTrue(np.all(q >= model.lower-1e-6) and np.all(q <= model.upper+1e-6))
            errors.append(np.linalg.norm(tip-target)); positions.append(p.copy()); arm.append(q.copy())
        self.assertLess(max(errors[300:]), .015)
        self.assertGreater(np.ptp(np.array(positions)[:, 1]), .025)
        self.assertGreater(np.ptp(np.array(arm)[:, 0]), .05)
