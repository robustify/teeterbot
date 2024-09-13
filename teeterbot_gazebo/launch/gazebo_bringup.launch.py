import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    sdf_file = LaunchConfiguration('robot_sdf_file').perform(context)
    with open(sdf_file, 'r') as f:
        robot_desc = f.read()

    verbose_mode_str = LaunchConfiguration('verbose').perform(context)
    verbose_mode = (verbose_mode_str.lower() == 'true')

    world_sdf_file = LaunchConfiguration('world_sdf_file').perform(context)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': (f'{world_sdf_file} --verbose' if verbose_mode else world_sdf_file)
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    bridge_config_file = LaunchConfiguration('gz_bridge_file').perform(context)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_file,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return [gz_sim, bridge]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_sdf_file', default_value='', description='Full path to robot model SDF file'),
        DeclareLaunchArgument('world_sdf_file', default_value='', description='Full path to world SDF file'),
        DeclareLaunchArgument('gz_bridge_file', default_value='', description='Full path to ROS/GZ bridge configuration YAML file'),
        DeclareLaunchArgument('verbose', default_value='false', description='Configure Gazebo to put verbose output on terminal'),
        OpaqueFunction(function=launch_setup)
    ])