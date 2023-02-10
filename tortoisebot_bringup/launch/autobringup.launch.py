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
  bringup_dir = os.path.join(get_package_share_directory('tortoisebot_bringup'), 'launch')
  navigation_dir = os.path.join(get_package_share_directory('tortoisebot_navigation'), 'launch')
  rviz_launch_dir=os.path.join(get_package_share_directory('tortoisebot_description'), 'launch')
  gazebo_launch_dir=os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'launch')
  ydlidar_launch_dir=os.path.join(get_package_share_directory('ydlidar'), 'launch')
  cartographer_launch_dir=os.path.join(get_package_share_directory('tortoisebot_slam'), 'launch')
  prefix_address = get_package_share_directory('tortoisebot_slam') 
  params_file= os.path.join(prefix_address, 'config', 'nav2_params.yaml')
  map_file = os.path.join(bringup_dir, 'maps', 'room2.yaml')
  use_sim_time=LaunchConfiguration('use_sim_time')
  slam=LaunchConfiguration('slam')   
  rviz_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'rviz.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  gazebo_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_launch_dir, 'gazebo.launch.py')),
            condition=IfCondition(use_sim_time),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  navigation_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'navigation.launch.py')),
            launch_arguments={'params_file': params_file,}.items())

  cartographer_launch_cmd=IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(cartographer_launch_dir, 'cartographer.launch.py')),
          launch_arguments={'params_file': params_file,
                            'slam':slam,
                            'use_sim_time':use_sim_time}.items())
  ydlidar_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_launch_dir, 'x4_ydlidar_launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_sim_time])),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='slam', default_value='True',
                                            description='Flag to enable use_sim_time'),
    Node(
        package='nav2_map_server',
        condition=IfCondition(PythonExpression(['not ', slam])),
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file}
                    ]),
    Node(
        package='nav2_lifecycle_manager',
        condition=IfCondition(PythonExpression(['not ', slam])),
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]),

    rviz_launch_cmd,
    gazebo_launch_cmd,
    navigation_launch_cmd, 
    cartographer_launch_cmd,  
    ydlidar_launch_cmd,

  ]
)
