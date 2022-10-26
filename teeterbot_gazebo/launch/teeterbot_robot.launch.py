import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare(package='teeterbot_description').find('teeterbot_description')
    urdf_file = os.path.join(pkg_share, 'urdf/teeterbot.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='', description='Gazebo model name for this robot'),
        DeclareLaunchArgument('start_x', default_value='0.0', description='X coordinate of starting position'),
        DeclareLaunchArgument('start_y', default_value='0.0', description='Y coordinate of starting position'),
        DeclareLaunchArgument('start_z', default_value='0.0', description='Z coordinate of starting position'),
        DeclareLaunchArgument('start_yaw', default_value='0.0', description='Yaw angle of starting orientation'),

        DeclareLaunchArgument('body_mass', default_value='10.0', description='Mass of the main body'),
        DeclareLaunchArgument('body_length', default_value='0.8', description='Length of the main body'),
        DeclareLaunchArgument('body_width', default_value='0.3', description='Width of the main body'),
        DeclareLaunchArgument('body_depth', default_value='0.3', description='Depth of the main body'),
        DeclareLaunchArgument('wheel_mass', default_value='1.0', description='Mass of each wheel'),
        DeclareLaunchArgument('wheel_radius', default_value='0.2', description='Radius of each wheel'),
        DeclareLaunchArgument('training_wheels', default_value='false', description='Spawn training wheels so the robot does not tip over'),

        DeclareLaunchArgument('pub_ground_truth', default_value='false', description='Publish ground truth transform between world and base_footprint'),
        DeclareLaunchArgument('auto_reset_orientation', default_value='true', description='Automatically reset robot orientation if it falls over'),
        DeclareLaunchArgument('auto_reset_delay', default_value='2.0', description='Amount of time to wait before automatic orientation reset'),
        DeclareLaunchArgument('tf_freq', default_value='100.0', description='Rate at which to broadcast TF frames'),

        DeclareLaunchArgument('voltage_mode', default_value='false', description='Use raw voltage inputs to control the motors'),
        DeclareLaunchArgument('torque_mode', default_value='false', description='Use torque command inputs to control the motors'),
        DeclareLaunchArgument('speed_mode', default_value='false', description='Use speed command inputs to control the motors'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'publish_frequency': LaunchConfiguration('tf_freq'),
                'robot_description': Command([f'xacro {urdf_file}',
                    ' pub_ground_truth:=', LaunchConfiguration('pub_ground_truth'),
                    ' auto_reset_orientation:=', LaunchConfiguration('auto_reset_orientation'),
                    ' auto_reset_delay:=', LaunchConfiguration('auto_reset_delay'),
                    ' wheel_mass:=', LaunchConfiguration('wheel_mass'),
                    ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
                    ' body_mass:=', LaunchConfiguration('body_mass'),
                    ' body_width:=', LaunchConfiguration('body_width'),
                    ' body_depth:=', LaunchConfiguration('body_depth'),
                    ' body_length:=', LaunchConfiguration('body_length'),
                    ' training_wheels:=', LaunchConfiguration('training_wheels'),
                    ' voltage_mode:=', LaunchConfiguration('voltage_mode'),
                    ' torque_mode:=', LaunchConfiguration('torque_mode'),
                    ' speed_mode:=', LaunchConfiguration('speed_mode')])
            }]),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', LaunchConfiguration('robot_name'),
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('start_x'),
                '-y', LaunchConfiguration('start_y'),
                '-z', LaunchConfiguration('start_z'),
                '-Y', LaunchConfiguration('start_yaw')
            ],
            output='screen')
    ])