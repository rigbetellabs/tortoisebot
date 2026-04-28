#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    desc_pkg   = get_package_share_directory('tortoisebot_description')
    gazebo_pkg = get_package_share_directory('tortoisebot_gazebo')
    slam_pkg   = get_package_share_directory('tortoisebot_slam')
    nav_pkg    = get_package_share_directory('tortoisebot_navigation')

    default_map = os.path.join(nav_pkg, 'maps', 'explored_map.yaml')
    sim_rviz_config = os.path.join(desc_pkg, 'rviz', 'simulation.rviz')
    nav_rviz_config = os.path.join(desc_pkg, 'rviz', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration  = LaunchConfiguration('exploration')
    map_file     = LaunchConfiguration('map_file')

    world_file = os.path.join(
        gazebo_pkg,
        'worlds',
        'nav2_test_world.sdf'
    )

    ignition_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'ignition_sim.launch.py')),
        launch_arguments={
            'world': world_file,
            'spawn_x': '0.0',
            'spawn_y': '0.0',
        }.items()
    )

    cartographer = TimerAction(
        period=6.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'cartographer.launch.py')),
            launch_arguments={'use_sim_time': 'True'}.items(),
            condition=IfCondition(exploration)
        )]
    )

    navigation = TimerAction(
        period=14.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'navigation_mapbased.launch.py')),
            condition=UnlessCondition(exploration),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'True',
            }.items()
        )]
    )

    navigation_slam = TimerAction(
        period=14.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'navigation_slam.launch.py')),
            condition=IfCondition(exploration),
            launch_arguments={
                'use_sim_time': 'True',
                'nav_activation_delay': '20.0',
            }.items()
        )]
    )

    rviz = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'rviz.launch.py')),
            launch_arguments={
                'rvizconfig': nav_rviz_config,
                'use_sim_time': 'True',
            }.items(),
            condition=IfCondition(exploration)
        )]
    )

    rviz_map = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'rviz.launch.py')),
            launch_arguments={
                'rvizconfig': sim_rviz_config,
                'use_sim_time': 'True',
            }.items(),
            condition=UnlessCondition(exploration)
        )]
    )

    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('exploration',  default_value='True'),
        DeclareLaunchArgument('map_file',     default_value=default_map),

        ignition_sim,
        cartographer,
        navigation,
        navigation_slam,
        rviz,
        rviz_map,
    ])
