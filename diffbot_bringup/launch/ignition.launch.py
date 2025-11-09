from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable  # <-- Add SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    set_software_rendering = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')  # comment out later

    # Declare arguments
    declare_gui = DeclareLaunchArgument('gui', default_value='true',
                                        description='Start Ignition Gazebo GUI')
    declare_world = DeclareLaunchArgument('world', default_value='empty.sdf',
                                          description='World file to load')

    # Get configurations
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')

    # Set Gazebo resource path to find custom worlds
    worlds_dir = PathJoinSubstitution([
        FindPackageShare('diffbot_description'),
        'worlds'
    ])
    set_gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', worlds_dir)

    # Remove PathJoinSubstitution for world_path, just use world directly
    gz_args = PythonExpression([
        "'-r -v 4 ' + '", world, "' if '", gui, "' == 'true' else '-r -v 4 -s ' + '", world, "'"
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
        set_gz_resource_path,
        set_software_rendering,
        ignition_gazebo
    ])