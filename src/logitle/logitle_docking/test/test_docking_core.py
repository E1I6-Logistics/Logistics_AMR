import math
import unittest

import numpy as np

from logitle_docking.docking_core import (
    icp_2d,
    icp_is_safe,
    make_v_funnel,
    model_transform_from_relative_pose,
    relative_pose,
    staging_pose,
)


class DockingCoreTest(unittest.TestCase):
    def test_confirmed_charger_geometry_is_symmetric(self):
        points = make_v_funnel(0.145, 0.10, 22.5)
        self.assertTrue(np.isclose(np.min(points[:, 1]), -np.max(points[:, 1])))
        self.assertTrue(np.isclose(
            np.max(points[:, 0]), 0.10 * math.cos(math.radians(22.5))
        ))
        self.assertTrue(np.any(np.isclose(
            points[:, 1], 0.145 / 2.0, atol=0.003
        )))

    def test_map_staging_pose_and_relative_pose(self):
        waypoint = staging_pose(2.0, 3.0, math.pi / 2.0, 0.45)
        self.assertTrue(np.allclose(waypoint, (2.0, 3.45, math.pi / 2.0)))
        relative = relative_pose(
            2.0, 3.45, math.pi / 2.0, 2.0, 3.0, math.pi / 2.0
        )
        self.assertTrue(np.allclose(relative, (-0.45, 0.0, 0.0), atol=1e-6))

    def test_icp_accepts_measured_shape_near_map_prediction(self):
        target = make_v_funnel(0.145, 0.10, 22.5)
        expected = model_transform_from_relative_pose(
            -0.45, 0.01, math.radians(1.0)
        )
        rotation = expected[:2, :2]
        translation = expected[:2, 2]
        source = (target - translation) @ rotation
        source += np.random.default_rng(7).normal(0.0, 0.001, source.shape)
        result = icp_2d(source, target, expected, 30, 0.12, 0.05)
        self.assertTrue(icp_is_safe(
            result, expected, 12, 0.25, 0.035, 0.10, 0.05, 10.0
        ))

    def test_icp_rejects_map_inconsistent_solution(self):
        target = make_v_funnel(0.145, 0.10, 22.5)
        expected = model_transform_from_relative_pose(-0.45, 0.0, 0.0)
        wrong = expected.copy()
        wrong[0, 2] += 0.25
        source = (target - wrong[:2, 2]) @ wrong[:2, :2]
        result = icp_2d(source, target, wrong, 20, 0.12, 0.05)
        self.assertFalse(icp_is_safe(
            result, expected, 12, 0.25, 0.035, 0.10, 0.05, 10.0
        ))


if __name__ == '__main__':
    unittest.main()
