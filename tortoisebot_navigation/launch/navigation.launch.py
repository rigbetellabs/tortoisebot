import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
  prefix_address = get_package_share_directory('tortoisebot_navigation') 
  params_file= os.path.join(prefix_address, 'config', 'nav2_params.yaml')

  navigation_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={'params_file': params_file,}.items())

  return LaunchDescription([
    navigation_launch_cmd,   
  ]
)
