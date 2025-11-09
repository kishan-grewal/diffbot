import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='false',
                                             description='Start RViz')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true',
                                                  description='Use simulation time')
    
    # Get configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # RViz config
    pkg_path = get_package_share_directory('diffbot_description')
    rviz_config = os.path.join(pkg_path, 'rviz', 'sim.rviz')

    # RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_use_sim_time,
        rviz
    ])