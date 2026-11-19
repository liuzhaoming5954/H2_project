"""Local candidate geometry checks with a known pose and an ambiguous single plane."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
from offline_localize import Matcher, rotation, trial


def surfaces():
    a, b = np.meshgrid(np.linspace(-3, 3, 60), np.linspace(-2, 2, 40))
    a, b = a.ravel(), b.ravel()
    planes = [np.column_stack((a, b, np.full_like(a, -1))),
              np.column_stack((a, np.full_like(a, 2.4), b)),
              np.column_stack((np.full_like(a, -2.8), a, b))]
    return planes


class TrialTests(unittest.TestCase):
    def test_independent_scans_known_transform(self):
        world = np.vstack(surfaces())
        r = rotation([2, -6, 14]); t = np.array([.4, -.2, .1]); rng = np.random.default_rng(17)
        scans = [(world[ids] + rng.normal(0, .01, (len(ids), 3)) - t) @ r
                 for ids in (rng.choice(len(world), 3800, replace=False) for _ in range(3))]
        result = trial(Matcher(world), scans, [.2, -.1, .2, 0, -3, 8])
        self.assertEqual(result['status'], 'candidate')
        np.testing.assert_allclose(result['T_map_lidar'], np.vstack((np.column_stack((r, t)), [0, 0, 0, 1])), atol=.015)

    def test_reject_geometry_with_unobservable_in_plane_translation(self):
        plane = surfaces()[0]
        res = trial(Matcher(plane), [plane] * 3, [0] * 6)
        self.assertEqual(res['status'], 'rejected')
        self.assertNotIn('T_map_lidar', res)

    def test_reject_motion_between_otherwise_good_scans(self):
        world = np.vstack(surfaces())
        scans = [world, world - [.3, 0, 0], world]
        res = trial(Matcher(world), scans, [0] * 6)
        self.assertEqual(res.get('reason'), 'stationary_poses_disagree')
        self.assertNotIn('T_map_lidar', res)
