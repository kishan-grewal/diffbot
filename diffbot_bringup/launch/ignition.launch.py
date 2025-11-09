from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declare_gui = DeclareLaunchArgument('gui', default_value='true',
                                        description='Start Ignition Gazebo GUI')
    declare_world = DeclareLaunchArgument('world', default_value='empty.sdf',
                                          description='World file to load')

    # Get configurations
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')

    # Resolve full path to world file inside 'diffbot_description/worlds'
    world_path = PathJoinSubstitution([
        FindPackageShare('diffbot_description'),
        'worlds',
        world
    ])

    # Build gz_args with full world path
    gz_args = PythonExpression([
        "'-r -v 4 ' + '", world_path, "' if '", gui, "' == 'true' else '-r -v 4 -s ' + '", world_path, "'"
    ])

    # Launch Ignition Gazebo using official launch file
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': gz_args}.items()
    )

    return LaunchDescription([
        declare_gui,
        declare_world,
        ignition_gazebo
    ])