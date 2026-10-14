import unittest
import numpy as np
from wholebody_catch.tracking import TargetTracker
from wholebody_catch.generic_perception import mask_grasp


class GenericPerceptionTest(unittest.TestCase):
    def test_markerless_visible_patch_and_oversized_support(self):
        depth = np.full((240, 320), .5)
        K = np.array([[250., 0., 160.], [0., 250., 120.], [0., 0., 1.]])
        T = np.eye(4); T[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        T[:3, 3] = [.65, 0., 1.3]
        mask = np.zeros_like(depth, bool); mask[110:131, 150:171] = True
        proposal = mask_grasp(mask, depth, K, T, [.6, -1, .9], [2.5, 1., 1.9])
        self.assertIsNotNone(proposal)
        np.testing.assert_allclose(proposal['point'], [1.16, 0., 1.3], atol=.001)
        # SAM predictor returns floating 0/1 masks, automatic masks are bool.
        float_proposal = mask_grasp(mask.astype('float32'), depth, K, T, [.6, -1, .9], [2.5, 1., 1.9])
        np.testing.assert_allclose(float_proposal['point'], proposal['point'])
        mask[20:230, 154:167] = True
        self.assertIsNone(mask_grasp(mask, depth, K, T, [.6, -1, .9], [2.5, 1., 1.9]))

    def test_prediction_never_refreshes_observation_and_outlier_rejected(self):
        tracker = TargetTracker()
        for i in range(20):
            self.assertTrue(tracker.observe(i*.1, [1.+i*.001, 0., 1.3]))
        observed = tracker.last_observed
        prediction = tracker.predict(2.2)
        self.assertGreater(prediction[0][0], tracker.x[0])
        self.assertEqual(tracker.last_observed, observed)
        self.assertFalse(tracker.observe(2., [1.4, 0., 1.3]))
        self.assertEqual(tracker.last_observed, observed)
        self.assertFalse(tracker.observe(1., [1.01, 0., 1.3]))
        self.assertGreater(np.trace(prediction[1]), np.trace(tracker.P))


if __name__ == '__main__':
    unittest.main()
