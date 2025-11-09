import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true',
                                                  description='Use simulation time')
    
    # Get configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Process URDF
    pkg_path = get_package_share_directory('diffbot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'diffbot.urdf.xacro')
    robot_description = {'robot_description': xacro.process_file(xacro_file).toxml()}

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Joint State Publisher (placeholder until ros2_control is added)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Spawn entity in Ignition Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', 
                   '-name', 'diffbot',
                   '-z', '0.3'],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        joint_state_publisher,  # placeholder
        spawn_entity
    ])