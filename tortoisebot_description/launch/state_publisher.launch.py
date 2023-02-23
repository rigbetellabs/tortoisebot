import launch
import os
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='tortoisebot_description').find('tortoisebot_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    default_model_path = os.path.join(pkg_share, 'models/urdf/tortoisebot.xacro')
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
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                    description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
    