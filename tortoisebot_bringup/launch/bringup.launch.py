import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import launch_ros
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
  pkg_share = launch_ros.substitutions.FindPackageShare(package='tortoisebot_description').find('tortoisebot_description')
  default_model_path = os.path.join(pkg_share, 'models/urdf/tortoisebot.xacro')
  ydlidar_launch_dir=os.path.join(get_package_share_directory('ydlidar'), 'launch')
  use_sim_time=LaunchConfiguration('use_sim_time')

  robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time},{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),value_type=str)}]
    )

  joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters= [{'use_sim_time': use_sim_time}],
    )

  ydlidar_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_launch_dir, 'x2_ydlidar_launch.py')),
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

  return LaunchDescription([
    launch.actions.DeclareLaunchArgument(
      name='model', 
      default_value=default_model_path,
      description='Absolute path to robot urdf file'),
    launch.actions.
    DeclareLaunchArgument(
      name='use_sim_time', 
      default_value='False',
      description='Flag to enable use_sim_time'),
 
    ydlidar_launch_cmd,
    differential_drive_node,
    camera_node,
    joint_state_publisher_node,
    robot_state_publisher_node

  ]
)
