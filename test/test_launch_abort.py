"""Launch test: aborted goals. Same setup as test_launch_goals.py, but the stand-in Nav2 server aborts
every goal: the node must keep exploring with new goals instead of stalling or resending the same one.

Original setup notes follow.

The test process provides what SLAM and Nav2 would: a known /map (10 x 10 m, free except an
occupied block), map -> base_link at the origin (static transform), and a stand-in
navigate_to_pose server that accepts every goal. Every goal the node sends must lie inside
exploration_bounds, at least min_goal_distance from the robot, and clear of the block.
Run: launch_test test/test_launch_goals.py (after sourcing the workspace).
"""
import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile

PARAMS = os.path.join(
    get_package_share_directory('random_explorer_bot'), 'config', 'exploration_params.yaml')
RES, SIZE, ORIGIN = 0.1, 100, -5.0           # 10 x 10 m map centred on the robot
BLOCK = (0.8, 1.6, -0.4, 0.4)                 # occupied x0, x1, y0, y1 (m)
BOUND, MIN_DIST = 2.0, 1.0                    # config/exploration_params.yaml
MARGIN = 0.25                                 # 0.35 m clearance in the code minus one cell


@pytest.mark.launch_test
def generate_test_description():
    tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'base_link'])
    node = launch_ros.actions.Node(
        package='random_explorer_bot', executable='exploration_controller',
        name='exploration_controller', output='screen', parameters=[PARAMS])
    return launch.LaunchDescription([tf, node, launch_testing.actions.ReadyToTest()]), {'node': node}


def make_map():
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = RES
    grid.info.width = grid.info.height = SIZE
    grid.info.origin.position.x = grid.info.origin.position.y = ORIGIN
    grid.info.origin.orientation.w = 1.0
    data = []
    for j in range(SIZE):
        y = ORIGIN + (j + 0.5) * RES
        for i in range(SIZE):
            x = ORIGIN + (i + 0.5) * RES
            inside = BLOCK[0] <= x <= BLOCK[1] and BLOCK[2] <= y <= BLOCK[3]
            data.append(100 if inside else 0)
    grid.data = data
    return grid


def distance_to_block(x, y):
    dx = max(BLOCK[0] - x, 0.0, x - BLOCK[1])
    dy = max(BLOCK[2] - y, 0.0, y - BLOCK[3])
    return math.hypot(dx, dy)


class TestGoals(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_aborted_goals_lead_to_new_goals(self, proc_output):
        fake = rclpy.create_node('fake_slam_nav2')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        fake.create_publisher(OccupancyGrid, '/map', latched).publish(make_map())
        goals = []

        def execute(goal_handle):
            p = goal_handle.request.pose.pose.position
            goals.append((round(p.x, 3), round(p.y, 3)))
            goal_handle.abort()
            return NavigateToPose.Result()

        server = ActionServer(fake, NavigateToPose, 'navigate_to_pose', execute)
        executor = MultiThreadedExecutor()
        executor.add_node(fake)
        try:
            start = time.monotonic()
            while len(goals) < 4 and time.monotonic() - start < 60.0:
                executor.spin_once(timeout_sec=0.1)
        finally:
            server.destroy()
            executor.shutdown()
            fake.destroy_node()
        self.assertGreaterEqual(len(goals), 4, f'goals seen: {goals}')
        self.assertEqual(len(set(goals[:4])), 4, f'a goal was resent: {goals[:4]}')
        proc_output.assertWaitFor('aborted', timeout=5)
