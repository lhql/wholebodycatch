"""Image geometry and timestamp interpolation; no simulator truth inputs."""
from collections import deque

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp


def detect_sphere(bgr, depth, intrinsics, radius=.02):
    """Fit a red sphere to measured depth, using its radius as a shape prior.

    RGB and depth must be registered, with depth along optical Z in metres.
    Returns optical-frame center and fit diagnostics, or None.
    """
    k = np.asarray(intrinsics, float).reshape(3, 3)
    if (bgr.ndim != 3 or bgr.shape[2] != 3 or depth.shape != bgr.shape[:2]
            or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.isfinite(radius) or radius <= 0):
        raise ValueError('Invalid registered RGB-D image, intrinsics or radius')
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = (((hsv[:, :, 0] < 10) | (hsv[:, :, 0] > 170))
            & (hsv[:, :, 1] > 140) & (hsv[:, :, 2] > 65)).astype('uint8')
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    candidates = []
    for index in range(1, count):
        x, y, w, h, area = stats[index]
        if area < 20:
            continue
        if not .55 < w/h < 1.8:
            continue
        ys, xs = np.where(labels == index)
        z = depth[ys, xs]
        valid = np.isfinite(z) & (z > .025) & (z < 2.8)
        if valid.sum() < 20 or valid.mean() < .65:
            continue
        xs, ys, z = xs[valid], ys[valid], z[valid]
        # Reject red backgrounds and depth edges before fitting.
        if np.percentile(z, 90)-np.percentile(z, 10) > 2.5*radius:
            continue
        points = np.column_stack(((xs-k[0, 2])*z/k[0, 0],
                                  (ys-k[1, 2])*z/k[1, 1], z))
        points = points[::max(1, len(points)//800)]
        visible = np.median(points, axis=0)
        guess = visible + radius*visible/np.linalg.norm(visible)
        fit = least_squares(lambda c: np.linalg.norm(points-c, axis=1)-radius,
                            guess, bounds=(guess-radius, guess+radius),
                            loss='soft_l1', f_scale=.001, max_nfev=30)
        residual = float(np.sqrt(np.mean((np.linalg.norm(points-fit.x, axis=1)-radius)**2)))
        # A fitted sphere must sit behind its observed front surface and have
        # an image footprint consistent with measured distance (not a red wall).
        expected = 2*radius*np.sqrt(k[0, 0]*k[1, 1])/fit.x[2]
        footprint = np.sqrt(w*h)
        # At grasp distance a sphere can touch the image boundary or be
        # partially occluded by a finger. Keep it only if the measured surface
        # has enough curvature in all three directions to constrain its center.
        normals = (points-fit.x)/radius
        singular = np.linalg.svd(normals, compute_uv=False)
        coverage = float(singular[-1]/singular[0])
        if (fit.success and residual < .0025 and fit.x[2] > np.median(z)
                and coverage > .12 and .65*expected < footprint < 1.4*expected):
            candidates.append((fit.x, dict(residual=residual, pixels=int(area), surface_coverage=coverage)))
    # Do not arbitrarily choose between multiple equally coloured objects.
    return candidates[0] if len(candidates) == 1 else None


class StateHistory:
    """Interpolate measured body/arm state at exposure time, never extrapolate."""
    def __init__(self, length=100, max_gap=.5):
        self.rows = deque(maxlen=length)
        self.max_gap = max_gap

    def add(self, row):
        values = np.r_[row['t'], row['body'], row['body_orientation'], row['q']]
        if values.shape != (15,) or not np.isfinite(values).all():
            raise ValueError('Invalid measured state')
        if np.linalg.norm(row['body_orientation']) < .5:
            raise ValueError('Invalid body quaternion')
        if self.rows and row['t'] < self.rows[-1]['t']:
            self.rows.clear()
        if self.rows and row['t'] == self.rows[-1]['t']:
            return
        self.rows.append(row)

    def sample(self, stamp):
        for a, b in zip(self.rows, list(self.rows)[1:]):
            if a['t'] <= stamp <= b['t'] and b['t']-a['t'] <= self.max_gap:
                alpha = (stamp-a['t'])/(b['t']-a['t'])
                p = (1-alpha)*np.array(a['body'])+alpha*np.array(b['body'])
                q = (1-alpha)*np.array(a['q'])+alpha*np.array(b['q'])
                R = Slerp([a['t'], b['t']], Rotation.from_quat(
                    [a['body_orientation'], b['body_orientation']]))([stamp]).as_matrix()[0]
                return p, R, q
        return None


def optical_to_map(center, state, kinematics):
    p, R, q = state
    # This URDF frame includes the 12 mm sensor offset and FLU -> optical axes.
    camera = kinematics.frames(q)['arm_camera_optical_frame']
    return p + R @ (camera[:3, 3] + camera[:3, :3] @ center)
