import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav_pkg = get_package_share_directory('tortoisebot_navigation')
    
    default_map_path = os.path.join(nav_pkg, 'maps', 'explored_map')

    map_name = LaunchConfiguration('map_name')
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value=default_map_path,
        description='Full path and name of the map to save'
    )

    map_saver_node = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', map_name],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        declare_map_name,
        map_saver_node
    ])
