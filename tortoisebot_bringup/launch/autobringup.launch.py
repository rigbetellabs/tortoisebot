import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression,Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable,IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import launch_ros
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
  pkg_share = launch_ros.substitutions.FindPackageShare(package='tortoisebot_description').find('tortoisebot_description')
  navigation_dir = os.path.join(get_package_share_directory('tortoisebot_navigation'), 'launch')
  rviz_launch_dir=os.path.join(get_package_share_directory('tortoisebot_description'), 'launch')
  gazebo_launch_dir=os.path.join(get_package_share_directory('tortoisebot_gazebo'), 'launch')
  ydlidar_launch_dir=os.path.join(get_package_share_directory('ydlidar'), 'launch')
  cartographer_launch_dir=os.path.join(get_package_share_directory('tortoisebot_slam'), 'launch')
  prefix_address = get_package_share_directory('tortoisebot_slam') 
  default_model_path = os.path.join(pkg_share, 'models/urdf/tortoisebot_simple.xacro')
  params_file= os.path.join(prefix_address, 'config', 'nav2_params.yaml')
  map_file=LaunchConfiguration('map')
  ros_ip = LaunchConfiguration('ros_ip')
  vr = LaunchConfiguration('vr')
  map_directory = os.path.join(get_package_share_directory(
        'tortoisebot_bringup'), 'maps','room2.yaml')
  use_sim_time=LaunchConfiguration('use_sim_time')
  exploration=LaunchConfiguration('exploration')   
  
  rviz_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'rviz.launch.py')),
            condition=IfCondition(use_sim_time),
            launch_arguments={'use_sim_time':use_sim_time}.items())

  state_publisher_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_launch_dir, 'state_publisher.launch.py')),
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
                            'slam':exploration,
                            'use_sim_time':use_sim_time}.items())
  
  ydlidar_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_launch_dir, 'x2_ydlidar_launch.py')),
            condition=IfCondition(PythonExpression(['not ', use_sim_time])),
            launch_arguments={'use_sim_time':use_sim_time}.items())
  
  # STAGE 1: Firmware node (launches immediately)
  differential_drive_node = Node(
        package='tortoisebot_firmware',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        executable='differential.py',
        name ='differential_drive_publisher',
    )
  
  camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_publisher',
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        parameters=[{
            'image_size': [160, 120],  # Very small resolution
            'camera_name': 'camera',
            'camera_info_url': '',
            'frame_id': 'camera_link',
            'framerate': 10.0,  # Low framerate
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )
  
  pose_publisher_cmd = Node(
    package='tortoisebot_bringup',  # Replace with your actual package name
    executable='robot_pose_publisher',
    name='robot_pose_publisher',
    condition=IfCondition(PythonExpression(['not ', use_sim_time, ' and ', vr])),
    output='screen',
    parameters=[{
        'base_frame': 'base_link',
        'reference_frame': 'map',
        'publish_rate': 20.0,
        'topic_name': 'robot_pose',
        'use_sim_time': use_sim_time
    }], 
    )
  
  ros_nav_status = Node(
        package='tortoisebot_bringup',
        executable='goal_status_publisher',
        name='ros_nav_status',
        condition=IfCondition(PythonExpression(['not ', use_sim_time, ' and ', vr])),
        output='screen',
    )
  
  nav2_goal_canceller_node = Node(
        package='tortoisebot_bringup',
        executable='nav2_goal_canceller',
        name='nav2_goal_canceller_node',
        condition=IfCondition(PythonExpression(['not ', use_sim_time, ' and ', vr])),
        output='screen',
        respawn=True,
    )
  
  robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time},{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),value_type=str)}]
    )
  
  ros_tcp_endpoint_node= Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='tcp_endpoint',
            condition=IfCondition(PythonExpression(['not ', use_sim_time, ' and ', vr])),
            parameters=[
                {'ROS_IP': ros_ip},
                {'ROS_TCP_PORT': 10000}
            ],
            respawn=True,  # Automatically restart if node dies
            respawn_delay=1.0,  # Wait 1 second before restart
            output='screen'
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
    launch.actions.DeclareLaunchArgument(name='exploration', default_value='True',
                                            description='Flag to enable use_sim_time'),
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                          description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='map',default_value=map_directory,
                                          description='Map to be used'),
    launch.actions.DeclareLaunchArgument(name='ros_ip', default_value='192.168.0.104',
                                    description='ROS IP address for TCP endpoint'),
    launch.actions.DeclareLaunchArgument(name='vr', default_value='False',
                                    description='ROS IP address for TCP endpoint'),

    # STAGE 1: Launch immediately - Core nodes and firmware
    robot_state_publisher_node,
    joint_state_publisher_node,
    differential_drive_node,  # Firmware launches first
    
    # STAGE 2: Launch after 3 seconds - LiDAR
    TimerAction(
        period=2.0,
        actions=[ydlidar_launch_cmd]
    ),
    
    # STAGE 3: Launch after 6 seconds - All other nodes
    TimerAction(
        period=5.0,
        actions=[
            Node(
                package='nav2_map_server',
                condition=IfCondition(PythonExpression(['not ', exploration])),
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'yaml_filename': map_file}
                            ]),
            Node(
                package='nav2_lifecycle_manager',
                condition=IfCondition(PythonExpression(['not ', exploration])),
                executable='lifecycle_manager',
                name='lifecycle_manager_mapper',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': True},
                            {'node_names': ['map_server']}]),
            rviz_launch_cmd,
            state_publisher_launch_cmd,
            gazebo_launch_cmd,
            navigation_launch_cmd, 
            cartographer_launch_cmd,  
            camera_node,
            ros_nav_status,
            ros_tcp_endpoint_node,
            pose_publisher_cmd,
            nav2_goal_canceller_node,
        ]
    ),

  ])
