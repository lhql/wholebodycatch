import unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from astra_wall_control.visual_geometry import VehicleGeometry, box_edges, scene_boxes

ROOT = Path(__file__).resolve().parents[1]


class VisualGeometryTest(unittest.TestCase):
    def test_body_and_rotor_orientation_and_ground_contact(self):
        geometry = VehicleGeometry(ROOT)
        q = np.array([0., -.05, .1, .05, 0., .01, .01])
        flat = geometry.evaluate(np.zeros(3), np.eye(3), q)
        p = np.array([1., .2, 1.6]); R = Rotation.from_euler('xyz', [.1, -.2, .7]).as_matrix()
        moved = geometry.evaluate(p, R, q)
        self.assertEqual(sum(name.startswith('rotor') for name, _, _, _ in moved), 4)
        self.assertGreater(sum(name.startswith(('arm_', 'gripper')) for name, _, _, _ in moved), 15)
        for first, second in zip(flat, moved):
            np.testing.assert_allclose(second[2][:3, 3], p+R@first[2][:3, 3], atol=1e-12)
            np.testing.assert_allclose(second[2][:3, :3], R@first[2][:3, :3], atol=1e-12)
        gear = [T[2, 3]-size[2]/2 for name, _, T, size in flat if name == 'landing_gear']
        self.assertAlmostEqual(min(gear), -.5)

    def test_fixed_vehicle_fits_displayed_safety_radius(self):
        geometry = VehicleGeometry(ROOT)
        for name, shape, T, size in geometry.fixed:
            # Box corners also conservatively bound each rotor cylinder.
            for point in box_edges(-size/2, size/2):
                self.assertLess(np.linalg.norm(T[:3, 3]+T[:3, :3]@point), geometry.radius, name)

    def test_scene_has_the_real_obstacle_dimensions(self):
        scene = scene_boxes(ROOT/'worlds/corridor.world')
        obstacles = [(T, size) for name, T, size in scene if name == 'corridor_obstacle']
        self.assertEqual(len(obstacles), 1)
        np.testing.assert_allclose(obstacles[0][0][:3, 3], [1.4, 0., 1.575])
        np.testing.assert_allclose(obstacles[0][1], [.4, 1.4, 3.15])
