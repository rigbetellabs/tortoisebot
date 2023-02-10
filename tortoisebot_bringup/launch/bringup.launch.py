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

  differential_drive_node = Node(
        package='tortoisebot_firmware',
        executable='differential.py',
        name ='differential_drive_publisher',
    )
  camera_node = Node(
      package='raspicam2',
      executable='raspicam2_node',
      name ='pi_camera',
    )
    #imu
  return LaunchDescription([
    launch.actions.
    DeclareLaunchArgument(
      name='use_sim_time', 
      default_value='False',
      description='Flag to enable use_sim_time'),

    rviz_launch_cmd, 
    ydlidar_launch_cmd,
    differential_drive_node,
    camera_node

  ]
)
