from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('diffbot_bringup')
    launch_dir = os.path.join(pkg, 'launch')

    # Declare all top-level arguments
    declare_gui = DeclareLaunchArgument('gui', default_value='true',
                                        description='Start Ignition Gazebo GUI')
    declare_world = DeclareLaunchArgument('world', default_value='empty.sdf',
                                          description='World file to load')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='false',
                                             description='Start RViz')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true',
                                                 description='Use simulation time')

    # Get configurations
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        declare_gui,
        declare_world,
        declare_use_rviz,
        declare_use_sim_time,

        # Include Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ignition.launch.py')),
            launch_arguments={
                'gui': gui,
                'world': world
            }.items()
        ),

        # Include Robot State Publisher + spawn
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_state.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time
            }.items()
        ),

        # Include RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            launch_arguments={
                'use_rviz': use_rviz,
                'use_sim_time': use_sim_time
            }.items()
        ),
    ])