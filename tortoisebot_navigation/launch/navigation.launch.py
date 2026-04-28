#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    nav_pkg     = get_package_share_directory('tortoisebot_navigation')
    bringup_pkg = get_package_share_directory('tortoisebot_bringup')

    default_map  = os.path.join(nav_pkg, 'maps', 'explored_map.yaml')
    params_file  = os.path.join(nav_pkg, 'config', 'nav2_params_simulation.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file     = LaunchConfiguration('map',          default=default_map)
    params       = LaunchConfiguration('params_file',  default=params_file)

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map':          map_file,
            'use_sim_time': use_sim_time,
            'params_file':  params,
            'use_rviz':     'False',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map',          default_value=default_map),
        DeclareLaunchArgument('params_file',  default_value=params_file),
        nav2,
    ])
