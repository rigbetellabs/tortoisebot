#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    desc_pkg    = get_package_share_directory('tortoisebot_description')
    gazebo_pkg  = get_package_share_directory('tortoisebot_gazebo')
    slam_pkg    = get_package_share_directory('tortoisebot_slam')
    nav_pkg     = get_package_share_directory('tortoisebot_navigation')
    bringup_pkg = get_package_share_directory('tortoisebot_bringup')

    default_map = os.path.join(nav_pkg, 'maps', 'explored_map.yaml')
    rviz_sim    = os.path.join(desc_pkg, 'rviz', 'simulation.rviz')

    gui      = LaunchConfiguration('gui')
    slam     = LaunchConfiguration('slam')
    nav      = LaunchConfiguration('nav')

    declare_gui = DeclareLaunchArgument(
        'gui', default_value='True',
        description='Launch Ignition Gazebo with GUI'
    )
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='False',
        description='Enable Cartographer SLAM mapping'
    )
    declare_nav = DeclareLaunchArgument(
        'nav', default_value='False',
        description='Enable Nav2 navigation'
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'ignition_sim.launch.py')
        ),
        launch_arguments={'gui': gui}.items()
    )

    rviz_simonly = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'rviz.launch.py')
            ),
            launch_arguments={'rvizconfig': rviz_sim}.items(),
            condition=IfCondition(
                PythonExpression([
                    "'", slam, "' == 'False' and '", nav, "' == 'False'"
                ])
            )
        )]
    )

    rviz_nav = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'rviz.launch.py')
            ),
            launch_arguments={'rvizconfig': rviz_sim}.items(),
            condition=IfCondition(nav)
        )]
    )

    cartographer = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'cartographer.launch.py')
            ),
            launch_arguments={'use_sim_time': 'True'}.items(),
            condition=IfCondition(
                PythonExpression([
                    "'", slam, "' == 'True' and '", nav, "' == 'False'"
                ])
            )
        )]
    )

    nav_mapbased = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'navigation_mapbased.launch.py')
            ),
            launch_arguments={'use_sim_time': 'True'}.items(),
            condition=IfCondition(
                PythonExpression([
                    "'", nav, "' == 'True' and '", slam, "' == 'False'"
                ])
            )
        )]
    )

    cartographer_with_nav = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'cartographer.launch.py')
            ),
            launch_arguments={'use_sim_time': 'True'}.items(),
            condition=IfCondition(
                PythonExpression([
                    "'", nav, "' == 'True' and '", slam, "' == 'True'"
                ])
            )
        )]
    )

    nav_slam = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'navigation_slam.launch.py')
            ),
            launch_arguments={'use_sim_time': 'True'}.items(),
            condition=IfCondition(
                PythonExpression([
                    "'", nav, "' == 'True' and '", slam, "' == 'True'"
                ])
            )
        )]
    )

    return LaunchDescription([
        declare_gui,
        declare_slam,
        declare_nav,

        simulation,
        rviz_simonly,
        rviz_nav,
        cartographer,
        nav_mapbased,
        cartographer_with_nav,
        nav_slam,
    ])
