#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get directories
    pkg_dir = get_package_share_directory('random_explorer_bot')
    nav2_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Paths to config files
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    exploration_params_file = os.path.join(pkg_dir, 'config', 'exploration_params.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'rviz_config.rviz')
    
    # SLAM configuration
    slam_params_file = os.path.join(
        slam_toolbox_dir,
        'config',
        'mapper_params_online_sync.yaml'
    )
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    # Start SLAM Toolbox for online mapping
    slam_toolbox_cmd = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    # Launch Nav2 (without map_server since we're using SLAM)
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
            'use_lifecycle_mgr': 'true',
            'use_remappings': 'true'
        }.items()
    )
    
    # Launch RViz2
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Launch exploration controller with delay to ensure everything is ready
    exploration_controller_cmd = Node(
        package='random_explorer_bot',
        executable='exploration_controller',
        name='exploration_controller',
        parameters=[exploration_params_file,
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        slam_toolbox_cmd,  # Start SLAM first
        nav2_bringup_cmd,
        rviz_cmd,
        exploration_controller_cmd
    ])