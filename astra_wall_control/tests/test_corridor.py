"""Geometric safety and traversal checks independent of ROS/PX4."""
import itertools
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from astra_wall_control.controller import ArmModel
from astra_wall_control.corridor import SafeCorridor, whole_body_radius
from astra_wall_control.geometry import transform

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT/'models/my/arm.urdf'


class CorridorTest(unittest.TestCase):
    def setUp(self):
        self.radius = whole_body_radius(URDF)
        self.corridor = SafeCorridor(ROOT/'config/corridor.json', self.radius)

    def test_envelope_contains_all_sampled_arm_collision_corners(self):
        model = ArmModel(URDF); root = ET.parse(URDF).getroot()
        rng = np.random.RandomState(8)
        for _ in range(100):
            q = rng.uniform(model.lower, model.upper)
            R = Rotation.from_rotvec(rng.normal(size=3)).as_matrix()
            model.evaluate(np.zeros(3), R, q)
            for link in root.findall('link'):
                for collision in link.findall('collision'):
                    origin = collision.find('origin')
                    xyz = np.fromstring(origin.get('xyz'), sep=' ')
                    rpy = np.fromstring(origin.get('rpy'), sep=' ')
                    half = np.fromstring(collision.find('geometry/box').get('size'), sep=' ')/2
                    T = model.frames[link.get('name')] @ transform(xyz, rpy)
                    for sign in itertools.product((-1, 1), repeat=3):
                        point = T[:3, 3]+T[:3, :3]@(half*sign)
                        self.assertLessEqual(np.linalg.norm(point), self.radius)

    def test_route_and_feedback_traversal_avoid_obstacle(self):
        p = np.array([0., 0., 1.6]); goal = np.array([2.7, 0., 1.6])
        route = self.corridor.plan(p, goal)
        self.assertEqual(len(route), 3)
        # Independently check sphere-to-obstacle distance throughout each segment.
        obstacle_low = np.array([1.2, -.7, 0.]); obstacle_high = np.array([1.6, .7, 3.15])
        for start, end in zip([p]+route[:-1], route):
            for alpha in np.linspace(0., 1., 101):
                centre = start*(1-alpha)+end*alpha
                nearest = np.clip(centre, obstacle_low, obstacle_high)
                self.assertGreaterEqual(np.linalg.norm(centre-nearest), self.radius+.099)
        for _ in range(6000):
            reference, done = self.corridor.step(p, .05)
            p += .1*(reference-p)
            if done: break
        self.assertTrue(done)
        self.assertLess(np.linalg.norm(p-goal), .04)
        # Reverse navigation must also connect through intersections.
        self.assertEqual(len(self.corridor.plan(p, np.array([0., 0., 1.6]))), 3)

    def test_rejected_goal_preserves_route_and_exit_fails_closed(self):
        p = np.array([0., 0., 1.6]); goal = np.array([2.7, 0., 1.6])
        previous = np.array(self.corridor.plan(p, goal))
        for bad in ([1.4, 0., 1.6], [0., 0., .6], [np.nan, 0., 1.6]):
            with self.assertRaises(ValueError): self.corridor.plan(p, bad)
            np.testing.assert_equal(self.corridor.waypoints, previous)
        with self.assertRaises(ValueError): self.corridor.step(np.array([1.4, 0., 1.6]))

    def test_narrow_corridor_rejected(self):
        data = dict(frame='map', tracking_margin=.1, cells=[dict(bounds=[[0, 0, 0], [1, 1, 1]])])
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json') as f:
            json.dump(data, f); f.flush()
            with self.assertRaises(ValueError): SafeCorridor(f.name, self.radius)

    def test_world_matches_declared_free_boxes(self):
        world = ET.parse(ROOT/'worlds/corridor.world')
        for model in world.findall('.//world/model'):
            pose = np.fromstring(model.findtext('pose'), sep=' ')[:3]
            size = np.fromstring(model.findtext('link/collision/geometry/box/size'), sep=' ')
            for box in self.corridor.raw:
                overlap = np.minimum(box[1], pose+size/2)-np.maximum(box[0], pose-size/2)
                self.assertFalse(np.all(overlap > 1e-8), model.get('name'))
