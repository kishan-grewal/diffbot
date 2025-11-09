import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # Package directories
    diffbot_description_dir = get_package_share_directory('diffbot_description')
    
    # Process URDF
    xacro_file = os.path.join(diffbot_description_dir, 'urdf', 'diffbot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Force software rendering for Ignition
    set_software_rendering = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')
    
    # Launch Ignition Gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()  # -r for run on start, -v 4 for verbose
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Spawn robot in Ignition Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'diffbot',
            '-x', '0',
            '-y', '0',
            '-z', '0.3'
        ],
        output='screen'
    )
    
    # Bridge between ROS and Ignition
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # Set software rendering
        set_software_rendering,
        
        # Launch Ignition Gazebo
        ignition_gazebo,
        
        # Publish robot description
        robot_state_publisher,
        
        # Spawn robot
        spawn_entity,
        
        # Bridge
        ros_gz_bridge,
    ])