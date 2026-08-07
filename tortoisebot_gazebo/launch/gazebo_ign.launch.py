from ament_index_python import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
import os


def generate_launch_description():
    # world_path= os.path.join(get_package_share_directory('ttb_description'), 'models/worlds/house_env.world'),
    package_name='tortoisebot_gazebo'
    world_path=os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'worlds/second.sdf'),
    world = LaunchConfiguration('world')


    
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
        )   
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path ,
        description='World to load'
        )
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.1'],
                        output='screen')
    

    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return launch.LaunchDescription([
        world_arg,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_image_bridge
    ])
    