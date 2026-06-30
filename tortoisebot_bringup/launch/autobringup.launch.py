#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    desc_pkg    = get_package_share_directory('tortoisebot_description')
    gazebo_pkg  = get_package_share_directory('tortoisebot_gazebo')
    slam_pkg    = get_package_share_directory('tortoisebot_slam')
    nav_pkg     = get_package_share_directory('tortoisebot_navigation')
    lidar_pkg   = FindPackageShare('ydlidar_ros2_driver')

    default_map     = os.path.join(nav_pkg,   'maps',   'explored_map.yaml')
    sim_rviz_config = os.path.join(desc_pkg,  'rviz',   'simulation.rviz')
    nav_rviz_config = os.path.join(desc_pkg,  'rviz',   'nav2.rviz')
    lidar_params    = PathJoinSubstitution([lidar_pkg, 'params', 'ydlidar.yaml'])
    real_urdf       = os.path.join(desc_pkg, 'models', 'urdf', 'tortoisebotreal.xacro')
    ekf_slam_params = os.path.join(slam_pkg, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration  = LaunchConfiguration('exploration')
    slam_only    = LaunchConfiguration('slam_only')
    map_file     = LaunchConfiguration('map_file')
    camera_port  = LaunchConfiguration('camera_port')

    world_file = os.path.join(gazebo_pkg, 'worlds', 'nav2_test_world.sdf')

    ignition_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'ignition_sim.launch.py')),
        launch_arguments={
            'world':   world_file,
            'spawn_x': '0.0',
            'spawn_y': '0.0',
        }.items(),
        condition=IfCondition(use_sim_time)
    )

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'False'}.items(),
        condition=UnlessCondition(use_sim_time)
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([lidar_pkg, 'launch', 'ydlidar_launch.py'])),
        launch_arguments={'params_file': lidar_params}.items(),
        condition=UnlessCondition(use_sim_time)
    )

    # BNO055 IMU node
    # imu = Node(
    #     package='tortoisebot_imu',
    #     executable='imu_node.py',
    #     name='imu_publisher',
    #     output='screen',
    #     condition=UnlessCondition(use_sim_time)
    # )


    motors = Node(
        package='tortoisebot_firmware',
        executable='differential.py',
        name='differential',
        output='screen',
        condition=UnlessCondition(use_sim_time)
    )

    camera = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[
            {'camera': camera_port},
            {'format': 'RGB888'},
            {'width': 800},
            {'height': 600}
        ],
        condition=UnlessCondition(use_sim_time)
    )

    # EKF node — disabled (not used in any pipeline)
    # ekf = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_slam_params, {'use_sim_time': False}],
    #     condition=IfCondition(PythonExpression([
    #         "'true' if ('", use_sim_time, "' == 'false' or '", use_sim_time, "' == 'False') and ('", exploration, "' == 'false' or '", exploration, "' == 'False') else 'false'"
    #     ]))
    # )

    cartographer = TimerAction(
        period=6.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'cartographer.launch.py')),
            condition=IfCondition(PythonExpression([
                "'true' if ('", exploration, "' == 'true' or '", exploration, "' == 'True') or ('", use_sim_time, "' == 'false' or '", use_sim_time, "' == 'False') else 'false'"
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'is_odom_only': PythonExpression(["'false' if '", exploration, "' == 'true' or '", exploration, "' == 'True' else 'true'"])
            }.items()
        )]
    )

    navigation = TimerAction(
        period=14.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'navigation_mapbased.launch.py')),
            condition=UnlessCondition(exploration),
            launch_arguments={
                'map':         map_file,
                'use_sim_time': use_sim_time,
            }.items()
        )]
    )

 
    navigation_slam = TimerAction(
        period=20.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg, 'launch', 'navigation_slam.launch.py')),
            condition=IfCondition(PythonExpression([
                "'true' if ('", exploration, "' == 'true' or '", exploration, "' == 'True') and ('", slam_only, "' == 'false' or '", slam_only, "' == 'False') else 'false'"
            ])),
            launch_arguments={
                'use_sim_time': use_sim_time,
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
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(PythonExpression([
                "'true' if ('", exploration, "' == 'true' or '", exploration, "' == 'True') and ('", slam_only, "' == 'false' or '", slam_only, "' == 'False') else 'false'"
            ]))
        )]
    )

    rviz_slam = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'rviz.launch.py')),
            launch_arguments={
                'rvizconfig': nav_rviz_config,
                'use_sim_time': use_sim_time,
            }.items(),
            condition=IfCondition(PythonExpression([
                "'true' if ('", exploration, "' == 'true' or '", exploration, "' == 'True') and ('", slam_only, "' == 'true' or '", slam_only, "' == 'True') else 'false'"
            ]))
        )]
    )

    rviz_map = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(desc_pkg, 'launch', 'rviz.launch.py')),
            launch_arguments={
                'rvizconfig': sim_rviz_config,
                'use_sim_time': use_sim_time,
            }.items(),
            condition=UnlessCondition(exploration)
        )]
    )

    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('use_sim_time', default_value='True',
                              description='True=Ignition Sim, False=Real Robot'),
        DeclareLaunchArgument('exploration',  default_value='True',
                              description='True=SLAM mapping, False=Map-based Nav'),
        DeclareLaunchArgument('slam_only',     default_value='False',
                              description='True=SLAM-only mapping (Cartographer, no Nav2), False=Standard mapping (SLAM + Nav2)'),
        DeclareLaunchArgument('map_file',     default_value=default_map,
                              description='Path to saved map yaml (used when exploration=False)'),
        DeclareLaunchArgument('camera_port',  default_value='0',
                              description='Camera port (e.g. 0 for /dev/video0, or /base/soc/...)'),


        ignition_sim,
        state_publisher,
        lidar,
        # imu,
        motors,
        camera,
        cartographer,
        navigation,
        navigation_slam,
        rviz,
        rviz_slam,
        rviz_map,
    ])
