import unittest
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from astra_wall_control.controller import ArmModel, WholeBodyController
from astra_wall_control.geometry import Kinematics
from wholebody_catch.mission import CatchMission, Settings, touches_target
from wholebody_catch.perception import StateHistory, detect_sphere, optical_to_map
from wholebody_catch.tracking import TargetTracker
from wholebody_catch.generic_perception import mask_grasp
from wholebody_catch.approach import ApproachPlan


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT.parent/'astra_wall_control/models/my/arm.urdf'


def render_sphere(center=(.02, -.01, .55), radius=.02):
    """Analytic ray/sphere depth for an independent perception fixture."""
    y, x = np.mgrid[:240, :320]
    k = np.array([[250., 0., 160.], [0., 250., 120.], [0., 0., 1.]])
    rays = np.stack(((x-160)/250, (y-120)/250, np.ones_like(x)), axis=-1)
    center = np.asarray(center)
    a = np.sum(rays*rays, axis=-1)
    b = -2*np.sum(rays*center, axis=-1)
    c = np.dot(center, center)-radius**2
    discriminant = b*b-4*a*c
    mask = discriminant > 0
    depth = np.full((240, 320), np.nan)
    depth[mask] = (-b[mask]-np.sqrt(discriminant[mask]))/(2*a[mask])
    bgr = np.zeros((240, 320, 3), dtype=np.uint8)
    bgr[mask, 2] = 240
    return bgr, depth, k


class PerceptionTest(unittest.TestCase):
    def test_depth_recovers_metric_center(self):
        for center in ((.02, -.01, .55), (-.1, .03, 1.1), (0., 0., .09)):
            result = detect_sphere(*render_sphere(center))
            self.assertIsNotNone(result)
            np.testing.assert_allclose(result[0], center, atol=.0001)

    def test_invalid_depth_ambiguous_targets_and_flat_patch_rejected(self):
        rgb, depth, k = render_sphere()
        self.assertIsNone(detect_sphere(rgb, np.zeros_like(depth), k))
        self.assertIsNone(detect_sphere(rgb, np.full_like(depth, np.nan), k))
        rgb2, depth2, _ = render_sphere((-.08, -.01, .55))
        depth[np.isfinite(depth2)] = depth2[np.isfinite(depth2)]
        self.assertIsNone(detect_sphere(np.maximum(rgb, rgb2), depth, k))
        rgb, depth, k = render_sphere()
        depth[np.isfinite(depth)] = .55
        self.assertIsNone(detect_sphere(rgb, depth, k))

    def test_timestamp_interpolation_and_no_extrapolation(self):
        history = StateHistory(max_gap=.5)
        for t, yaw in ((1., 0.), (1.2, .2)):
            history.add(dict(t=t, body=[t, 0., 1.6], body_orientation=Rotation.from_euler('z', yaw).as_quat(), q=[t]*7))
        p, R, q = history.sample(1.1)
        np.testing.assert_allclose(p, [1.1, 0., 1.6])
        np.testing.assert_allclose(q, [1.1]*7)
        np.testing.assert_allclose(R, Rotation.from_euler('z', .1).as_matrix(), atol=1e-12)
        self.assertIsNone(history.sample(.9))
        self.assertIsNone(history.sample(1.3))
        history.add(dict(t=.2, body=[0]*3, body_orientation=[0, 0, 0, 1], q=[0]*7))
        self.assertIsNone(history.sample(1.1))

    def test_partially_clipped_sphere_retains_metric_center(self):
        center = (0., -.044, .10)
        rgb, depth, k = render_sphere(center)
        self.assertTrue(np.any(rgb[0, :, 2]))
        result = detect_sphere(rgb, depth, k)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result[0], center, atol=.0001)

    def test_camera_transform_matches_independent_arm_model(self):
        q = np.array([.1, -.1, .09, .1, .2, .01, .01])
        p = np.array([1., -.2, 1.6])
        R = Rotation.from_euler('xyz', [.04, -.03, .2]).as_matrix()
        model = ArmModel(URDF)
        model.evaluate(p, R, q)
        optical = np.array([.01, -.02, .3])
        T = model.frames['arm_camera_optical_frame']
        expected = T[:3, 3]+T[:3, :3]@optical
        np.testing.assert_allclose(optical_to_map(optical, (p, R, q), Kinematics(URDF)), expected, atol=1e-10)


class MissionTest(unittest.TestCase):
    def setUp(self):
        self.machine = CatchMission()
        self.now = 0.
        self.point = np.array([1.15, 0., 1.3])

    def step(self, tip=None, contact=False, point=None, vision=True, ready=True, duration=.05):
        self.now += duration
        if vision:
            self.machine.observe(self.now, self.point if point is None else point)
        self.machine.update(self.now, ready, self.point if tip is None else tip, 0., 0., True, contact)

    def reach_close(self):
        self.step()
        for _ in range(35):
            self.step(tip=self.point-[.12, 0., 0.])
        self.assertEqual(self.machine.phase, 'INSERT')
        for _ in range(300):
            self.step(tip=self.machine.goal.copy())
            if self.machine.phase == 'CLOSE':
                break
        self.assertEqual(self.machine.phase, 'CLOSE')

    def test_planned_final_segment_completes_without_new_images(self):
        self.step()
        for _ in range(35):
            self.step(tip=self.point-[.12, 0., 0.])
        for _ in range(300):
            self.step(tip=self.machine.goal.copy())
            if np.linalg.norm(self.machine.plan.end-self.machine.goal) < .025:
                break
        last_stamp = self.machine.last_observation[0]
        saw_blind = False
        for _ in range(100):
            self.step(tip=self.machine.goal.copy(), vision=False)
            saw_blind |= self.machine.blind
            if self.machine.phase == 'CLOSE':
                break
        self.assertTrue(saw_blind)
        self.assertEqual(self.machine.phase, 'CLOSE')
        self.assertEqual(self.machine.last_observation[0], last_stamp)
        self.assertEqual(self.machine.recoveries, 0)
        # Continued tactile evidence and tool motion can verify an occluded grasp.
        for _ in range(220):
            self.step(tip=self.machine.goal.copy(), vision=False, contact=True)
            if self.machine.phase == 'SUCCESS':
                break
        self.assertEqual(self.machine.phase, 'SUCCESS')
        self.assertEqual(self.machine.verification, 'contact_and_tool_motion')

    def test_blind_segment_expires_or_rejects_tracking_error(self):
        for fault in ('expired', 'tracking'):
            self.setUp()
            self.reach_close()
            self.machine.enter('INSERT', self.now)
            self.machine.last_observation = (self.now-1., self.point.copy())
            if fault == 'expired':
                self.machine.last_observation = (self.now-5., self.point.copy())
            tip = self.machine.goal.copy()
            if fault == 'tracking':
                tip[1] += .03
            self.step(tip=tip, vision=False)
            self.assertEqual(self.machine.phase, 'RECOVER', fault)

    def test_planned_approach_speed_and_pause(self):
        plan = ApproachPlan([0., 0., 0.], [.144, .012, -.01], .025)
        points = [plan.sample(0.)]
        for _ in range(int(plan.duration/.002)+2):
            points.append(plan.sample(.002))
        speed = np.linalg.norm(np.diff(points, axis=0), axis=1)/.002
        self.assertLessEqual(max(speed), .0250001)
        self.assertLess(speed[0], .00001)
        self.assertLess(speed[-1], .00001)
        np.testing.assert_allclose(points[-1], plan.end)
        frozen = plan.sample(.1, advance=False)
        np.testing.assert_allclose(frozen, points[-1])

    def test_full_sequence_requires_contacts_and_observed_lift(self):
        self.reach_close()
        for _ in range(15):
            self.step(contact=True)
        self.assertEqual(self.machine.phase, 'LIFT')
        for _ in range(160):
            self.step(tip=self.machine.goal.copy(), contact=True)
            if self.machine.phase == 'VERIFY':
                break
        self.assertEqual(self.machine.phase, 'VERIFY')
        # Closed fingers plus a lifted EE cannot pass with an object left behind.
        for _ in range(40):
            self.step(tip=self.machine.goal.copy(), contact=True)
        self.assertEqual(self.machine.phase, 'VERIFY')
        for _ in range(40):
            self.step(tip=self.machine.goal.copy(), point=self.machine.goal.copy(), contact=True)
        self.assertEqual(self.machine.phase, 'SUCCESS')
        for _ in range(10):
            self.step(tip=self.machine.goal.copy(), point=self.machine.goal.copy(), contact=False)
        self.assertEqual(self.machine.phase, 'ABORT')

    def test_empty_close_times_out(self):
        self.reach_close()
        for _ in range(190):
            self.step()
        self.assertEqual(self.machine.phase, 'ABORT')
        self.assertIn('CLOSE timeout', self.machine.reason)

    def test_close_requires_precision_and_low_speed(self):
        self.reach_close()
        self.machine.enter('INSERT', self.now)
        for _ in range(20):
            self.step(tip=self.point+[self.machine.cfg.grasp_tip_offset, 0., .009])
        self.assertEqual(self.machine.phase, 'INSERT')
        for _ in range(20):
            self.now += .05
            self.machine.observe(self.now, self.point)
            self.machine.update(self.now, True, self.point+[self.machine.cfg.grasp_tip_offset, 0., 0.], .02, 0., True, False)
        self.assertEqual(self.machine.phase, 'INSERT')
        for _ in range(20):
            self.step(tip=self.point+[self.machine.cfg.grasp_tip_offset, 0., 0.])
        self.assertEqual(self.machine.phase, 'CLOSE')

    def test_visual_refinement_is_bounded_and_jump_still_aborts(self):
        self.machine.cfg.refine_approach = True
        self.reach_close()
        self.machine.enter('INSERT', self.now)
        before = self.machine.target.copy()
        self.step(tip=self.point-[.06, 0., 0.], point=self.point+[0., 0., .01])
        self.assertLessEqual(np.linalg.norm(self.machine.target-before), .000500001)
        self.step(point=self.point+[.1, 0., 0.])
        self.assertEqual(self.machine.phase, 'ABORT')

    def test_stale_vision_recovers_then_times_out_without_blind_close(self):
        for _ in range(20):
            self.step(tip=self.point-[.2, 0., 0.])
        self.assertEqual(self.machine.phase, 'PREGRASP')
        for _ in range(20):
            self.step(vision=False)
        self.assertEqual(self.machine.phase, 'RECOVER')
        self.assertEqual(self.machine.gripper, Settings().open_position)
        for _ in range(220):
            self.step(vision=False)
        self.assertEqual(self.machine.phase, 'ABORT')
        self.assertEqual(self.machine.reason, 'visual recovery timeout')

    def test_partial_view_does_not_move_committed_endpoint(self):
        self.reach_close()
        self.machine.enter('INSERT', self.now)
        endpoint = self.machine.plan.end.copy()
        for _ in range(10):
            self.step(tip=self.machine.goal.copy(), point=self.point+[0., 0., .01])
        np.testing.assert_allclose(self.machine.plan.end, endpoint)

    def test_reacquisition_requires_same_target_and_resumes_pregrasp(self):
        for _ in range(20):
            self.step(tip=self.point-[.2, 0., 0.])
        for _ in range(20):
            self.step(tip=self.point-[.19, 0., 0.], vision=False)
        self.assertEqual(self.machine.phase, 'RECOVER')
        for _ in range(20):
            self.step(tip=self.point-[.19, 0., 0.], point=self.point+[.2, 0., 0.])
        self.assertEqual(self.machine.phase, 'RECOVER')
        for _ in range(15):
            self.step(tip=self.point-[.19, 0., 0.])
        self.assertEqual(self.machine.phase, 'PREGRASP')
        self.assertEqual(self.machine.recoveries, 1)

    def test_mode_loss_and_clock_reset_latch_abort(self):
        self.step()
        self.step(ready=False)
        self.step(ready=True)
        self.assertEqual(self.machine.phase, 'ABORT')
        self.machine = CatchMission()
        self.step()
        self.step(duration=-.1)
        self.assertEqual(self.machine.phase, 'ABORT')

    def test_contact_model_identity(self):
        state = lambda name: SimpleNamespace(collision1_name='my::gripper_left_link::finger_collision', collision2_name=name)
        self.assertTrue(touches_target([state('catch_target::payload::collision')], 'catch_target'))
        self.assertFalse(touches_target([state('catch_target_fake::payload::collision')], 'catch_target'))
        self.assertFalse(touches_target([state('catch_pedestal::support::post')], 'catch_target'))


class SourceIntegrationTest(unittest.TestCase):
    def test_measured_body_motion_compensation_holds_tool(self):
        model = ArmModel(URDF)
        control = WholeBodyController(model, wall_constraint=False)
        q = np.array([0., -.05, .1, .05, 0., .018, .018])
        control.posture = q[:5].copy()
        center = np.array([.7, 0., 1.5])
        target = model.evaluate(center, np.eye(3), q)[0]
        errors = []
        for step in range(600):
            t = step*.02
            # Exogenous body motion is not the controller's requested velocity.
            amplitude = np.array([.015, .012, .015])
            p = center+amplitude*np.sin(.6*t)
            measured = amplitude*.6*np.cos(.6*t)
            v, tip, _ = control.solve(p, np.eye(3), q, target, center, .02,
                                      measured_body_velocity=measured)
            q[:5] += .02*v[4:]
            if step > 100:
                errors.append(np.linalg.norm(tip-target))
            self.assertTrue(np.all(q >= model.lower) and np.all(q <= model.upper))
        self.assertLess(max(errors), .006)

    def test_world_payload_is_dynamic_and_fits_gripper(self):
        world = ET.parse(ROOT/'worlds/catch.world').getroot().find('world')
        target = world.find("model[@name='catch_target']")
        self.assertEqual(target.findtext('static'), 'false')
        self.assertLess(float(target.findtext('link/inertial/mass')), .03)
        self.assertAlmostEqual(float(target.findtext('link/collision/geometry/sphere/radius')), .02)
        self.assertEqual(target.findtext('plugin/ros/namespace'), '/wholebody_catch/eval')

    def test_wholebody_reaches_pregrasp_grasp_and_lift_within_limits(self):
        model = ArmModel(URDF)
        control = WholeBodyController(model, wall_constraint=False)
        q = np.array([0., -.05, .1, .05, 0., .018, .018])
        control.posture = q[:5].copy()
        offset = model.evaluate(np.zeros(3), np.eye(3), q)[0]
        p, yaw = np.array([0., 0., 1.6]), 0.
        for goal in ([1.03, 0., 1.3], [1.15, 0., 1.3], [1.15, 0., 1.46]):
            target = np.array(goal)
            for _ in range(800):
                R = Rotation.from_euler('z', yaw).as_matrix()
                v, _, _ = control.solve(p, R, q, target, target-offset, .02)
                p += .02*v[:3]
                yaw += .02*v[3]
                q[:5] += .02*v[4:]
                self.assertTrue(np.all(q >= model.lower) and np.all(q <= model.upper))
            tip = model.evaluate(p, Rotation.from_euler('z', yaw).as_matrix(), q)[0]
            self.assertLess(np.linalg.norm(tip-target), .01)


if __name__ == '__main__':
    unittest.main()
