"""Visible-surface grasp proposals, without colour, category or CAD priors.

This is a constrained lateral parallel-jaw heuristic, not a force-closure
certificate: unseen surfaces cannot be certified from one depth image.
"""
import numpy as np


def mask_grasp(mask, depth, K, camera_to_map, workspace_min, workspace_max,
               max_width=.060, insertion=.010):
    K = np.asarray(K).reshape(3, 3)
    T = np.asarray(camera_to_map).reshape(4, 4)
    mask = np.asarray(mask, dtype=bool)
    ys, xs = np.where(mask & np.isfinite(depth) & (depth > .025) & (depth < 3.))
    if len(xs) < 40:
        return None
    step = max(1, len(xs)//1800)
    xs, ys = xs[::step], ys[::step]
    z = depth[ys, xs]
    optical = np.column_stack(((xs-K[0, 2])*z/K[0, 0], (ys-K[1, 2])*z/K[1, 1], z))
    points = optical@T[:3, :3].T+T[:3, 3]
    lower, upper = np.quantile(points, [.02, .98], axis=0)
    extent = upper-lower
    center = (upper+lower)/2
    # Reject support posts/backgrounds using the entire mask, before workspace
    # filtering. Merely cropping a post can otherwise look like a small object.
    if (np.any(center < workspace_min) or np.any(center > workspace_max)
            or not .012 <= extent[1] <= max_width
            or not .010 <= extent[2] <= .10 or extent[0] > .10):
        return None
    # +X is the supported approach of this five-axis flight adapter. The grasp
    # is placed just behind the visible front surface; no hidden back face is
    # invented. Check this insertion distance for the physical gripper.
    center[0] = float(np.quantile(points[:, 0], .2))+insertion
    return dict(point=center.tolist(), width=float(extent[1]), extent=extent.tolist(),
                pixels=int(len(xs)*step), pixel=[float(np.median(xs)), float(np.median(ys))])
