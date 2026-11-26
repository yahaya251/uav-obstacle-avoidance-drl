import math
import unittest

import torch

from uav_isaac.geometry import PlanarGeometry, RAY_COUNT, limit_velocity_batch
from uav_nav.core import ObservationResult, PoseSample, ScanSample, apf_velocity, build_observation, limit_velocity
from uav_nav.kinematics import KinematicEnv


LAYOUTS = [
    {"start": (-4.4, 0.1), "goal": (4.4, -0.25), "obstacles": []},
    {"start": (-2.35, 0.35), "goal": (4.4, 0.2), "obstacles": [(0.0, 0.0, 0.65)]},
    {
        "start": (-0.5, -1.5),
        "goal": (3.5, 1.0),
        "obstacles": [(1.0, 0.2, 0.5), (-1.0, 1.1, 0.3), (1.3, -2.4, 0.38)],
    },
]


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = PlanarGeometry(LAYOUTS, dtype=torch.float64)
        cls.angles = [-math.pi + i * (math.tau / RAY_COUNT) for i in range(RAY_COUNT)]

    def test_raw_ranges_match_kinematic_reference(self):
        obstacles_and_starts = zip(LAYOUTS, self.geometry.ranges(self.geometry.start))
        for layout, actual in obstacles_and_starts:
            with self.subTest(layout=layout):
                env = KinematicEnv()
                env.reset(options=layout)
                expected = torch.tensor([env.ray_distance(a) for a in self.angles], dtype=actual.dtype)
                torch.testing.assert_close(actual, expected, atol=1e-9, rtol=0)

    def test_observation_matches_shared_pooling_at_nonzero_velocity(self):
        velocities = torch.tensor([(0.0, 0.0), (0.3, -0.4), (-0.1, 0.2)], dtype=torch.float64)
        actual, minima = self.geometry.observe(self.geometry.start, velocities)
        self.assertEqual(actual.shape, (len(LAYOUTS), 40))
        for i, layout in enumerate(LAYOUTS):
            env = KinematicEnv()
            env.reset(options=layout)
            pose = PoseSample((*env.position, 1.5), (*velocities[i].tolist(), 0.0), 0.0, 1.0)
            scan = ScanSample(
                tuple(env.ray_distance(a) for a in self.angles), -math.pi, math.tau / RAY_COUNT, 0.0, 6.0, 1.0
            )
            expected = build_observation(scan, pose, env.goal, now=1.0)
            self.assertTrue(expected.ready)
            torch.testing.assert_close(
                actual[i], torch.tensor(expected.values, dtype=actual.dtype), atol=1e-9, rtol=0
            )
            self.assertAlmostEqual(minima[i].item(), expected.min_range_m)

    def test_non_padded_geometry_and_start_inside_obstacle(self):
        points = torch.zeros_like(self.geometry.start)
        ranges = self.geometry.ranges(points)
        self.assertTrue((ranges[0] > 0).all())
        self.assertTrue((ranges[1] == 0).all())

    def test_wall_and_swept_collision(self):
        old = self.geometry.start.clone()
        new = torch.tensor([(-5.81, 0.0), (2.0, 0.0), (0.0, -1.5)], dtype=old.dtype)
        self.assertEqual(self.geometry.collision(old, new).tolist(), [True, True, False])

    def test_clearance_and_stationary_collision(self):
        points = torch.tensor([(5.9, 0.0), (0.0, 0.0), (-0.1, 0.1)], dtype=torch.float64)
        clearance = self.geometry.clearance(points)
        self.assertAlmostEqual(clearance[0].item(), -0.1)
        self.assertAlmostEqual(clearance[1].item(), -0.85)
        self.assertGreater(clearance[2].item(), 0)
        self.assertEqual(self.geometry.collision(points, points).tolist(), [True, True, False])

    def test_invalid_layouts_fail_early(self):
        for key, value in [("start", (6.5, 0)), ("goal", (0, math.nan)), ("obstacles", [(1, 0, -0.1)])]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                PlanarGeometry([{**LAYOUTS[0], key: value}])

    def test_speed_acceleration_and_stop_match_scalar(self):
        actions = [[1, 1], [1, -1], [math.nan, 0], [-2, 1.5], [1, 0], [-1, -1], [1, 1]]
        previous = [[0, 0], [0.5, 0.25], [0, 0], [-1.0, 0.5], [0, 0], [0.1, 0], [0.1, 0]]
        distances = [6.0, 0.55, 2.0, math.inf, 4.2, math.nan, -0.1]
        velocity, stops = limit_velocity_batch(
            torch.tensor(actions, dtype=torch.float64), torch.tensor(previous, dtype=torch.float64),
            torch.tensor(distances, dtype=torch.float64), 0.1,
        )
        expected = [limit_velocity(a, p, 0.1, min_range_m=d) for a, p, d in zip(actions, previous, distances)]
        torch.testing.assert_close(
            velocity, torch.tensor([result.velocity_enu for result in expected], dtype=velocity.dtype),
            atol=1e-10, rtol=0,
        )
        self.assertEqual(stops.tolist(), [result.reason == "minimum_range_stop" for result in expected])

    def test_batched_apf_matches_scalar_for_clear_blocked_and_invalid_observations(self):
        observations, _ = self.geometry.observe(self.geometry.start, torch.zeros_like(self.geometry.start))
        examples = observations[[0, 1, 2, 0, 1, 2]].clone()
        examples[3, 36:38] = 0.01  # goal already reached
        examples[4, 5] = 0.0  # saturated obstacle repulsion
        examples[5, 39] = math.nan
        expected = [
            apf_velocity(ObservationResult(tuple(row), True, (), (True,) * 36, math.inf, None, None))
            for row in examples.tolist()
        ]
        torch.testing.assert_close(
            self.geometry.apf_action(examples), torch.tensor(expected, dtype=examples.dtype), atol=1e-10, rtol=0,
        )


if __name__ == "__main__":
    unittest.main()
