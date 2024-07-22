import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression,Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import launch_ros
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
  pkg_share = launch_ros.substitutions.FindPackageShare(package='tortoisebot_description').find('tortoisebot_description')
  rviz_launch_dir=os.path.join(get_package_share_directory('tortoisebot_description'), 'launch')
  gazebo_launch_dir=os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'launch')
  ydlidar_launch_dir=os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch')
  default_model_path = os.path.join(pkg_share, 'models/urdf/tortoisebot.xacro')
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/sensors.rviz')
  use_sim_time=LaunchConfiguration('use_sim_time')
  
  rviz_launch_cmd=launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters= [{'use_sim_time': use_sim_time}],
    )

  state_publisher_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'state_publisher.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  gazebo_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
            condition=IfCondition(use_sim_time),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  ydlidar_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_launch_dir, 'ydlidar_launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_sim_time])),
            launch_arguments={'use_sim_time':use_sim_time}.items())
  
  differential_drive_node = Node(
        package='tortoisebot_firmware',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        executable='differential.py',
        name ='differential_drive_publisher',
    )
  camera_node = Node(
      package='raspicam2',
      condition=IfCondition(PythonExpression(['not ', use_sim_time])),
      executable='raspicam2_node',
      name ='pi_camera',
    )
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
  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),

    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                          description='Absolute path to robot urdf file'),

    launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
    rviz_launch_cmd,
    state_publisher_launch_cmd,
    robot_state_publisher_node,
    joint_state_publisher_node,
    ydlidar_launch_cmd,
    differential_drive_node,
    gazebo_launch_cmd,
    camera_node

  ]
)

