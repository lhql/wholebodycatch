#!/usr/bin/python3
"""Independent room-plane check of a scan captured by sitl_modes.py.

The Gazebo body pose is used only here for verification, never in control.
Use a ground capture for strict timing-independent geometry acceptance.
"""
import argparse
import json
import numpy as np
from scipy.spatial.transform import Rotation


def check(path, radial_correction=0.):
    data = np.load(path)
    points = data['points']; distance = np.linalg.norm(points, axis=1)
    points = points[(distance > 1.) & (distance < 35.)]
    points *= 1.+radial_correction/np.linalg.norm(points, axis=1)[:, None]
    world = (points+[0., 0., .13])@Rotation.from_quat(data['quat']).as_matrix().T+data['body']
    planes = []
    # Ground returns are sparse in the MID360 vertical field of view; use the
    # ceiling and four known room walls, excluding furniture/self returns.
    for name, axis, position in [('ceiling', 2, 3.15), ('rear', 0, -2.95),
                                 ('far', 0, 5.95), ('right', 1, -3.45), ('left', 1, 3.95)]:
        error = world[:, axis]-position
        error = error[np.abs(error) < .25]
        median = float(np.median(np.abs(error))) if len(error) else float('inf')
        planes.append(dict(plane=name, points=len(error), median_abs_error_m=median,
                           passed=bool(len(error) >= 50 and median < .005)))
    return dict(passed=all(p['passed'] for p in planes), radial_correction=radial_correction, planes=planes)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('capture')
    parser.add_argument('--radial-correction', type=float, default=0.)
    args = parser.parse_args()
    result = check(args.capture, args.radial_correction)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
