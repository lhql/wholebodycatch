"""Air-test goal coordinates: spawn-aligned ENU, metres, fixed tool attitude."""
import numpy as np


def validate_goal(frame, position):
    if frame != 'map':
        raise ValueError('Goal frame must be map')
    target = np.asarray(position, dtype=float)
    if target.shape != (3,) or not np.all(np.isfinite(target)):
        raise ValueError('Goal must contain three finite coordinates')
    # Clear volume inside air.world; keep distance from the LIO room walls.
    if np.any(target < [-1., -1.5, .8]) or np.any(target > [3., 1.5, 2.]):
        raise ValueError('Goal bounds: x [-1, 3], y [-1.5, 1.5], z [0.8, 2.0] m')
    return target.copy()
