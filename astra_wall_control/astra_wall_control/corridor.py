"""Conservative whole-body safe boxes for a known static world (metres, ENU).

Every segment lies in one convex box eroded by an orientation-independent
whole-body sphere and a tracking margin. This is not an online mapping planner.
"""
from collections import deque
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from .trajectory import MinimumJerkTrajectory


def whole_body_radius(urdf, body_radius=.68):
    """Triangle-inequality bound over every arm configuration, including fingers.

    .68 covers this model's fixed body, rotors, sensors and landing gear; the
    lowest skid corner is approximately .66 m from aircraft_base.
    Unsupported collision geometry fails closed.
    """
    root = ET.parse(urdf).getroot()
    reach = {'aircraft_base': 0.}
    for joint in root.findall('joint'):
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        distance = np.linalg.norm(xyz)
        if joint.get('type') == 'prismatic':
            lim = joint.find('limit')
            distance += max(abs(float(lim.get('lower'))), abs(float(lim.get('upper'))))
        reach[joint.find('child').get('link')] = reach[joint.find('parent').get('link')] + distance
    radius = body_radius
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            origin = collision.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            box = collision.find('geometry/box')
            if box is None:
                raise ValueError('Corridor envelope requires box arm collisions')
            half = np.fromstring(box.get('size'), sep=' ')/2.
            radius = max(radius, reach[link.get('name')]+np.linalg.norm(xyz)+np.linalg.norm(half))
    return float(radius)


class SafeCorridor:
    def __init__(self, path, radius):
        data = json.loads(Path(path).read_text())
        if data['frame'] != 'map' or not np.isfinite(radius) or radius <= 0:
            raise ValueError('Invalid corridor frame or envelope')
        self.margin = float(data['tracking_margin'])
        if not np.isfinite(self.margin) or self.margin < .05:
            raise ValueError('Tracking margin must be at least 0.05 m')
        self.radius = radius
        self.raw = np.asarray([c['bounds'] for c in data['cells']], dtype=float)
        if self.raw.ndim != 3 or self.raw.shape[1:] != (2, 3) or not np.all(np.isfinite(self.raw)):
            raise ValueError('Expected finite [lower, upper] boxes')
        self.cells = self.raw + np.array([radius+self.margin, -radius-self.margin])[None, :, None]
        if np.any(self.cells[:, 0] >= self.cells[:, 1]):
            raise ValueError('A corridor is too narrow for the complete vehicle')
        self.waypoints = []; self.route_cells = []; self.index = 0

    @staticmethod
    def inside(point, box, tolerance=1e-8):
        return bool(np.all(np.isfinite(point)) and np.all(point >= box[0]-tolerance) and
                    np.all(point <= box[1]+tolerance))

    def plan(self, start, goal):
        start, goal = np.asarray(start, float), np.asarray(goal, float)
        if start.shape != (3,) or goal.shape != (3,):
            raise ValueError('Expected three-dimensional start and goal')
        starts = [i for i, b in enumerate(self.cells) if self.inside(start, b)]
        ends = [i for i, b in enumerate(self.cells) if self.inside(goal, b)]
        queue = deque((i, [i]) for i in starts); seen = set(starts)
        route = None
        while queue:
            i, path = queue.popleft()
            if i in ends:
                route = path; break
            for j, box in enumerate(self.cells):
                low = np.maximum(self.cells[i, 0], box[0])
                high = np.minimum(self.cells[i, 1], box[1])
                # Leave room for waypoint acceptance inside each overlap.
                if j not in seen and np.all(high-low > .12):
                    seen.add(j); queue.append((j, path+[j]))
        if route is None:
            raise ValueError('No whole-body-safe corridor connects start and goal')
        points = []
        for i, j in zip(route, route[1:]):
            overlap = np.array([np.maximum(self.cells[i, 0], self.cells[j, 0]),
                                np.minimum(self.cells[i, 1], self.cells[j, 1])])
            points.append(np.mean(overlap, axis=0))
        points.append(goal.copy())
        trajectory = MinimumJerkTrajectory([start]+points, self.cells[route])
        self.waypoints, self.route_cells, self.index = points, route, 0
        self.start = start.copy()
        self.trajectory = trajectory; self.elapsed = 0.; self.paused = False
        self.velocity = np.zeros(3); self.acceleration = np.zeros(3)
        self.smooth_path = trajectory.path()
        return [p.copy() for p in points]

    def step(self, position, dt=.02):
        if not self.waypoints:
            raise ValueError('No corridor route')
        if not np.isfinite(dt) or not 0. <= dt <= .1:
            raise ValueError('Invalid trajectory time step')
        box = self.cells[self.route_cells[self.index]]
        if not self.inside(position, box):
            raise ValueError('Measured vehicle left the eroded safe corridor')
        current = self.trajectory.sample(self.elapsed)[0]
        self.paused = bool(np.linalg.norm(current-position) > .12)
        proposed = min(self.trajectory.duration, self.elapsed+(0. if self.paused else dt))
        reference, velocity, acceleration, index = self.trajectory.sample(proposed)
        if index != self.index and not self.inside(position, self.cells[self.route_cells[index]]):
            proposed = max(self.elapsed, self.trajectory.ends[self.index]-1e-8)
            reference, velocity, acceleration, index = self.trajectory.sample(proposed)
            self.paused = True
        self.elapsed, self.index = proposed, index
        self.velocity = np.zeros(3) if self.paused else velocity
        self.acceleration = np.zeros(3) if self.paused else acceleration
        if not self.inside(reference, self.cells[self.route_cells[self.index]]):
            raise ValueError('Unsafe corridor reference')
        done = self.elapsed >= self.trajectory.duration and np.linalg.norm(self.waypoints[-1]-position) < .04
        return reference, done
