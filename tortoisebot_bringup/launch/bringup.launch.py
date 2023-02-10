import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
  rviz_launch_dir=os.path.join(get_package_share_directory('tortoisebot_description'), 'launch')
  ydlidar_launch_dir=os.path.join(get_package_share_directory('ydlidar'), 'launch')
  use_sim_time=LaunchConfiguration('use_sim_time')
  
  rviz_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'rviz.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items())


  ydlidar_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_launch_dir, 'x4_ydlidar_launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_sim_time])),
            launch_arguments={'use_sim_time':use_sim_time}.items())

    # camera_launch_cmd=
    #imu
  return LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
    Node(
        package='tortoisebot_firmware',
        executable='differential_publisher',
    ),
    Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in','/camera/points'),
                    ],
        parameters=[{
            'target_frame': 'lidar_1',
            'transform_tolerance': 0.01,
            'min_height': 0.2,
            'max_height': 1.0,
            'angle_min': -1.5708,  # -M_PI/2
            'angle_max': 1.5708,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 4.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    ),

    rviz_launch_cmd, 
    ydlidar_launch_cmd,

  ]
)
