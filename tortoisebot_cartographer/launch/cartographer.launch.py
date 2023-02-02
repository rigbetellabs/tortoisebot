import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
  nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
  display_launch_dir=os.path.join(get_package_share_directory('tortoisebot_description'), 'launch')
  prefix_address = get_package_share_directory('tortoisebot_cartographer') 
  params_file= os.path.join(prefix_address, 'config', 'nav2_params.yaml')
  config_directory = LaunchConfiguration('configuration_directory', default=os.path.join(prefix_address, 'config'))
  config_basename = LaunchConfiguration('configuration_basename', default='2d_localization.lua')
  res = LaunchConfiguration('resolution', default='0.05')
  publish_period = LaunchConfiguration('publish_period_sec', default='1.0')
  use_sim_time=LaunchConfiguration('use_sim_time')
  
  display_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(display_launch_dir, 'display.launch.py')),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  navigation_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={'params_file': params_file,}.items())

  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),

    DeclareLaunchArgument(
      'resolution',
      default_value=res,
      description='configure the resolution'
    ),

    DeclareLaunchArgument(
      'publish_period_sec',
      default_value=publish_period,
      description='publish period in seconds'
    ),

    ################### cartographer_ros_node ###################
    DeclareLaunchArgument(
      'configuration_directory',
      default_value=config_directory,
      description='path to the .lua files'
    ),
    DeclareLaunchArgument(
      'configuration_basename',
      default_value=config_basename,
      description='name of .lua file to be used'
    ),
    Node(
      package='cartographer_ros',
      executable='cartographer_node',
      name='as21_cartographer_node',
      arguments=[
        '-configuration_directory', config_directory,
        '-configuration_basename', config_basename
      ],
      parameters= [{'use_sim_time':use_sim_time}],
      output='screen'
    ),

    Node(
      package='cartographer_ros',
      executable='occupancy_grid_node',
      name='cartographer_occupancy_grid_node',
      arguments=[
        '-resolution', res,
        '-publish_period_sec', publish_period
      ]
    ),

    display_launch_cmd,
    navigation_launch_cmd,   

  ]
)
