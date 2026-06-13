#!/usr/bin/env python3
"""One-shot demo: Gazebo Harmonic sim + SLAM + Nav2 + RViz + random explorer.

Sim comes up first; SLAM/Nav2/explorer are delayed so /scan, /odom and tf are
flowing before the navigation stack initializes.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('random_explorer_bot')
    launch_dir = os.path.join(pkg_dir, 'launch')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turtlebot3_gz.launch.py')))

    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'explorer_nav.launch.py')))

    return LaunchDescription([
        gz_sim,
        # Let Gazebo spawn the robot and start streaming sensors first
        TimerAction(period=8.0, actions=[nav_stack]),
    ])
