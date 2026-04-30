#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    slam_pkg = get_package_share_directory('tortoisebot_slam')
    config_dir = os.path.join(slam_pkg, 'config')

    use_sim_time   = LaunchConfiguration('use_sim_time')
    resolution     = LaunchConfiguration('resolution')
    publish_period = LaunchConfiguration('publish_period_sec')

    use_sim_str = context.perform_substitution(use_sim_time).lower()
    is_odom_only_str = context.perform_substitution(LaunchConfiguration('is_odom_only')).lower()
    
    if is_odom_only_str == 'true':
        lua_basename = 'slam_real_mapbased.lua'
    else:
        lua_basename = 'slam_sim.lua' if use_sim_str == 'true' else 'slam_real.lua'

    cartographer_node = Node(
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
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        arguments=[
            '-resolution', resolution,
            '-publish_period_sec', publish_period,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    nodes = [cartographer_node]
    if is_odom_only_str != 'true':
        nodes.append(occupancy_grid_node)

    return nodes


def generate_launch_description():

    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='True = simulation (slam_sim.lua), False = real robot (slam_real.lua / slam_real_mapbased.lua)'
        ),
        DeclareLaunchArgument(
            'is_odom_only',
            default_value='false',
            description='If true, runs in odometry-only mode (does not publish map frame)'
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

        OpaqueFunction(function=launch_setup),
    ])
