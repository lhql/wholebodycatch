"""Analytic URDF kinematics and bounded differential whole-body IK (ENU/FLU).

The vehicle contributes x/y/z/yaw; roll/pitch are measured disturbances,
not fictitious fully actuated degrees of freedom. PX4 closes flight dynamics.
"""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import lsq_linear
from .geometry import NAMES, transform, rotation


class ArmModel:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.tree = []
        limits = {}
        for joint in root.findall('joint'):
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
            axis = joint.find('axis')
            axis = np.fromstring(axis.get('xyz'), sep=' ') if axis is not None else np.array([1., 0., 0.])
            name = joint.get('name')
            idx = NAMES.index(name) if name in NAMES else -1
            self.tree.append((joint.find('parent').get('link'), joint.find('child').get('link'),
                              transform(xyz, rpy), joint.get('type'), axis, idx))
            if idx >= 0:
                lim = joint.find('limit')
                limits[idx] = [float(lim.get(k)) for k in ('lower', 'upper', 'velocity', 'effort')]
        self.lower, self.upper, self.speed, self.effort = np.array([limits[i] for i in range(7)]).T
        self.inertia = []
        for link in root.findall('link'):
            inertia = link.find('inertial')
            if inertia is not None:
                origin = inertia.find('origin')
                xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
                self.inertia.append((link.get('name'), float(inertia.find('mass').get('value')), xyz))

    def evaluate(self, p, R, q):
        B = np.eye(4); B[:3, :3] = R; B[:3, 3] = p
        frames = {'aircraft_base': B}; chains = {'aircraft_base': []}; axes = {}
        for parent, child, origin, kind, axis, idx in self.tree:
            T = frames[parent] @ origin
            chain = list(chains[parent])
            if idx >= 0:
                axes[idx] = (T[:3, 3].copy(), T[:3, :3] @ axis, kind)
                chain.append(idx)
                if kind == 'prismatic': T[:3, 3] += T[:3, :3] @ axis*q[idx]
                else: T[:3, :3] = T[:3, :3] @ rotation(axis, q[idx])
            frames[child], chains[child] = T, chain
        self.frames, self.chains, self.axes = frames, chains, axes
        T = frames['arm_tool_link']
        tip = T[:3, 3] + T[:3, :3] @ np.array([.078, 0., 0.])
        J = np.zeros((6, 9)); J[:3, :3] = np.eye(3)
        J[:3, 3] = np.cross([0, 0, 1], tip-p); J[5, 3] = 1.
        J[:, 4:] = self.jacobian(tip, 'arm_tool_link')[:, :5]
        return tip, T[:3, :3], J

    def jacobian(self, point, link):
        J = np.zeros((6, 7))
        for idx in self.chains[link]:
            origin, axis, kind = self.axes[idx]
            if kind == 'prismatic': J[:3, idx] = axis
            else: J[:3, idx], J[3:, idx] = np.cross(axis, point-origin), axis
        return J

    def gravity(self, acceleration=np.zeros(3)):
        torque = np.zeros(7)
        for name, mass, xyz in self.inertia:
            T = self.frames[name]; point = T[:3, 3] + T[:3, :3] @ xyz
            torque += self.jacobian(point, name)[:3].T @ (mass*(np.array([0., 0., 9.81])+acceleration))
        return np.clip(torque, -self.effort, self.effort)


class WholeBodyController:
    def __init__(self, model, wall_constraint=True):
        self.model = model
        self.wall_constraint = wall_constraint
        self.previous = np.zeros(9)
        self.posture = np.array([0., -.18, .07, .18, 0.])

    def solve(self, p, R, q, target, body_target, dt, omega=None,
              body_yaw=0., body_velocity=None, body_bounds=None, fixed_tip=False,
              measured_body_velocity=None):
        if not all(np.all(np.isfinite(v)) for v in (p, R, q, target, body_target)):
            raise ValueError('Nonfinite whole-body input')
        tip, tool_R, J = self.model.evaluate(p, R, q)
        error = np.r_[target-tip, Rotation.from_matrix(tool_R.T).as_rotvec()]
        # Orientation logarithm expressed in world coordinates.
        error[3:] = tool_R @ error[3:]
        desired = np.clip(2.8*error, [-.22]*3+[-.5]*3, [.22]*3+[.5]*3)
        if omega is not None:
            # Remove measured roll/pitch disturbance from desired tool velocity.
            disturbance = np.array(omega, float); disturbance[2] = 0.
            desired[:3] -= np.cross(disturbance, tip-p)
            desired[3:] -= disturbance
        yaw = np.arctan2(R[1, 0], R[0, 0])
        body_error = np.r_[body_target-p, np.arctan2(np.sin(body_yaw-yaw), np.cos(body_yaw-yaw))]
        body_velocity = np.zeros(4) if body_velocity is None else np.asarray(body_velocity, float)
        if body_velocity.shape != (4,) or not np.all(np.isfinite(body_velocity)) or not np.isfinite(body_yaw):
            raise ValueError('Invalid whole-body reference velocity/yaw')
        B = np.zeros((4, 9)); B[:, :4] = np.eye(4)
        Q = np.zeros((5, 9)); Q[:, 4:] = np.eye(5)
        # Tool translation dominates, body stabilizes remaining redundancy.
        # Fixed-point body motion prioritizes tool POSITION. A stiff tool yaw
        # objective would remove the lateral redundancy of this five-axis arm.
        weights = np.array([30., 30., 30., .2, .2, .2]) if fixed_tip else np.array([14., 14., 14., 2., 2., 2.])
        A = np.vstack([weights[:, None]*J, 2.*B, .12*Q, .12*np.eye(9)])
        b = np.r_[weights*desired, 2.*np.clip(1.2*body_error+body_velocity, -.2, .2),
                  .12*.5*(self.posture-q[:5]), .12*self.previous]
        speed = np.r_[[.18, .18, .15, .3], np.minimum(self.model.speed[:5]*.5, [.7, .7, .06, .9, .9])]
        lo, hi = -speed, speed.copy()
        if body_bounds is not None:
            body_bounds = np.asarray(body_bounds, float)
            if body_bounds.shape != (2, 3) or not np.all(np.isfinite(body_bounds)) or np.any(body_bounds[0] >= body_bounds[1]):
                raise ValueError('Invalid body motion bounds')
            lo[:3] = np.maximum(lo[:3], np.minimum(speed[:3], (body_bounds[0]-p)/.5))
            hi[:3] = np.minimum(hi[:3], np.maximum(-speed[:3], (body_bounds[1]-p)/.5))
        # Velocity damper anticipates hard joint limits over 0.3 seconds.
        lo[4:] = np.maximum(lo[4:], (self.model.lower[:5]+.001-q[:5])/.3)
        hi[4:] = np.minimum(hi[4:], (self.model.upper[:5]-.001-q[:5])/.3)
        lo[4:] = np.minimum(lo[4:], hi[4:]-.00001)
        # Body collision envelope for rotors and gear; +X wall normal.
        if self.wall_constraint:
            hi[0] = min(hi[0], max(-.18, (target[0]-.39-p[0])/.5))
        # Accelerations are bounded on all actuated coordinates.
        accel = np.r_[[.35, .35, .3, .7], [1.5, 1.5, .2, 2., 2.]]
        low = np.maximum(lo, self.previous-accel*dt)
        high = np.minimum(hi, self.previous+accel*dt)
        low = np.minimum(low, high-1e-8)
        result = lsq_linear(A, b, bounds=(low, high), method='bvls', tol=1e-7, max_iter=40)
        if not result.success or not np.all(np.isfinite(result.x)):
            raise RuntimeError('Whole-body bounded IK failed')
        if measured_body_velocity is not None:
            measured_body_velocity = np.asarray(measured_body_velocity, float)
            if measured_body_velocity.shape != (3,) or not np.isfinite(measured_body_velocity).all():
                raise ValueError('Invalid measured body velocity')
            # The flight loop cannot realize the solved body velocity
            # instantaneously. Reallocate the fast arm against measured body
            # motion, retaining the same joint speed/acceleration limits.
            actual_body = J[:, :3] @ measured_body_velocity
            if omega is not None:
                actual_body += J[:, 3]*omega[2]
            arm_A = np.vstack([weights[:, None]*J[:, 4:], .12*np.eye(5), .12*np.eye(5)])
            arm_b = np.r_[weights*(desired-actual_body),
                          .12*.5*(self.posture-q[:5]), .12*self.previous[4:]]
            arm_result = lsq_linear(arm_A, arm_b, bounds=(low[4:], high[4:]),
                                   method='bvls', tol=1e-7, max_iter=40)
            if not arm_result.success or not np.isfinite(arm_result.x).all():
                raise RuntimeError('Measured-motion arm allocation failed')
            result.x[4:] = arm_result.x
        self.previous = result.x
        return result.x, tip, error
