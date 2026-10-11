from pathlib import Path
import unittest
from check_lidar_geometry import check


class LidarGeometryTest(unittest.TestCase):
    def test_recorded_room_geometry_before_and_after_origin_fix(self):
        data = Path(__file__).parent/'data'
        before = check(data/'lidar_geometry_before.npz')
        after = check(data/'lidar_geometry_after.npz')
        self.assertFalse(before['passed'])
        self.assertTrue(after['passed'])
        # The diagnosis predicts all five planes, not just a fitted z offset.
        self.assertTrue(check(data/'lidar_geometry_before.npz', .2)['passed'])
        self.assertGreater(min(p['median_abs_error_m'] for p in before['planes']), .09)
        self.assertLess(max(p['median_abs_error_m'] for p in after['planes']), .0001)
