import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('teeterbot_gazebo'), 'launch', 'gzserver.launch.py')
        ])
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('teeterbot_gazebo'), 'launch', 'gzclient.launch.py')
        ])
    )

    teeterbot_options = dict(
        robot_name = 'teeterbot',
        start_x = '0',
        start_y = '0',
        start_z = '0.2',
        start_yaw = '0',
        body_length = '0.8',
        body_width = '0.3',
        body_depth = '0.3',
        body_mass = '10.0',
        wheel_mass = '1.0',
        wheel_radius = '0.2',
        training_wheels = 'false',
        pub_ground_truth = 'false',
        auto_reset_orientation = 'true',
        auto_reset_delay = '2.0',
        voltage_mode = 'false',
        torque_mode = 'true',
        speed_mode = 'false'
    )

    spawn_teeterbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('teeterbot_gazebo'), 'launch', 'teeterbot_robot.launch.py')
        ]),
        launch_arguments=teeterbot_options.items()
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,
        spawn_teeterbot
    ])
