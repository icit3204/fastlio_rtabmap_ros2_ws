#!/usr/bin/env python3
"""Static contract checks for the bounded R3G RTAB occupancy repair."""

from pathlib import Path
import unittest


LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'rtabmap_bridge.launch.py'
TEXT = LAUNCH.read_text(encoding='utf-8')


class R3GOccupancyContractTest(unittest.TestCase):
    def test_r3a_authority_is_preserved(self):
        self.assertIn("DeclareLaunchArgument('odom_frame_id', default_value='odom_chassis')", TEXT)
        self.assertIn("DeclareLaunchArgument('frame_id', default_value='base_footprint')", TEXT)
        self.assertIn("DeclareLaunchArgument('rtabmap_imu_topic', default_value='/unused_imu')", TEXT)

    def test_ray_tracing_and_planar_height_contract_are_explicit(self):
        for option in (
            '--Grid/3D true',
            '--Grid/NormalsSegmentation false',
            '--Grid/MinGroundHeight -0.20',
            '--Grid/MaxGroundHeight 0.15',
            '--Grid/MaxObstacleHeight 2.0',
            '--Grid/RayTracing true',
        ):
            self.assertIn(option, TEXT)

    def test_unrelated_planner_and_sensor_controls_are_not_in_this_repair(self):
        self.assertNotIn('minimum_turning_radius', TEXT)
        self.assertNotIn('SmacPlannerHybrid', TEXT)
        self.assertNotIn('/livox/imu', TEXT)


if __name__ == '__main__':
    unittest.main()
