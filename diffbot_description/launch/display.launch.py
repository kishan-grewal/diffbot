import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Get urdf path
    diffbot_description_dir = get_package_share_directory('diffbot_description')
    xacro_file = os.path.join(diffbot_description_dir, 'urdf', 'diffbot.urdf.xacro')

    # Process urdf from xacro form into xml
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Robot State Publisher takes in xml
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Launch rviz with no config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    # Launch nodes
    return LaunchDescription([
        rsp_node,
        rviz_node
    ])