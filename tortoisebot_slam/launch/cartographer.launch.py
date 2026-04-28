#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    slam_pkg = get_package_share_directory('tortoisebot_slam')

    config_dir   = os.path.join(slam_pkg, 'config')
    lua_basename = 'slam.lua'

    use_sim_time   = LaunchConfiguration('use_sim_time')
    resolution     = LaunchConfiguration('resolution')
    publish_period = LaunchConfiguration('publish_period_sec')

    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Occupancy grid resolution in metres per cell'
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='How often the occupancy grid is published'
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', lua_basename,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/odom'),
                ('imu',  '/imu'),
            ],
            output='screen'
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            arguments=[
                '-resolution', resolution,
                '-publish_period_sec', publish_period,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

    ])
