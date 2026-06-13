#!/usr/bin/env python3
"""Bring up TurtleBot3 (burger) in Gazebo Harmonic with the ros_gz bridge.

Publishes /scan, /odom, /tf, /joint_states, /clock; subscribes /cmd_vel.
This replaces the Gazebo Classic turtlebot3_gazebo path (Classic is EOL and
not installed on this machine).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('random_explorer_bot')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Let Gazebo resolve model://turtlebot3_world to the copy shipped in this
    # package, and model://turtlebot3_description/... to the installed share dir
    # so the robot's visual meshes load in the GUI.
    models_dir = os.path.join(pkg_dir, 'models')
    tb3_desc_parent = os.path.dirname(
        get_package_share_directory('turtlebot3_description'))
    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            models_dir,
            tb3_desc_parent,
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ]))

    world_file = os.path.join(pkg_dir, 'worlds', 'explore_world.sdf')
    xacro_file = os.path.join(pkg_dir, 'description', 'turtlebot3_burger.urdf.xacro')
    bridge_config = os.path.join(pkg_dir, 'config', 'gz_bridge.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock')

    # Start Gazebo Harmonic, running, with our world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items(),
    )

    robot_description = Command(['xacro ', xacro_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # Spawn the robot from the /robot_description topic (resolves package:// meshes)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_burger',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'burger',
            '-x', '-2.0', '-y', '-0.5', '-z', '0.01',
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        bridge,
        # Give Gazebo a moment to load the world before spawning
        TimerAction(period=3.0, actions=[spawn]),
    ])
