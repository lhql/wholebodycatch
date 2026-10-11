import unittest
import numpy as np
from astra_wall_control.goals import validate_goal


class GoalTest(unittest.TestCase):
    def test_valid_volume(self):
        for p in ([-1., -1.5, .8], [3., 1.5, 2.], [2., 0., 1.4]):
            np.testing.assert_equal(validate_goal('map', p), p)

    def test_invalid_goals(self):
        for frame, p in [('odom', [2., 0., 1.4]), ('', [2., 0., 1.4]),
                         ('map', [2., 0., 0.]), ('map', [4., 0., 1.4]),
                         ('map', [np.nan, 0., 1.4]), ('map', [2., 0.]),
                         ('map', [2., np.inf, 1.4])]:
            with self.subTest(frame=frame, p=p), self.assertRaises(ValueError):
                validate_goal(frame, p)
