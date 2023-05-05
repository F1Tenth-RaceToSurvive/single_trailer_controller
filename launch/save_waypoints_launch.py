import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_description = LaunchDescription()

    params_config = os.path.join(get_package_share_directory('single_trailer_controller'),
        'config',
        'params_save_wp.yaml'
        )

    save_waypoints_node = Node(
        package='single_trailer_controller',
        executable='save_waypoints_node',
        name='save_waypoints_node',
        output='screen',
        # prefix=['gdbserver localhost:3000'],
        emulate_tty=True,
        parameters=[params_config]
    )

    launch_description.add_action(save_waypoints_node)
    return launch_description
