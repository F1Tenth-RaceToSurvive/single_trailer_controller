import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    launch_description = LaunchDescription()

    """ ************** Declare Launch Arguments ********************** """
    use_sim = LaunchConfiguration("use_sim")

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description="Used to determine which nodes need to be run"
    )


    """ ********** Configure loading Parameters File ****************** """
    params_sim_config = os.path.join(get_package_share_directory('single_trailer_controller'),
        'config',
        'params_sim.yaml'
        )
    params_hw_config = os.path.join(get_package_share_directory('single_trailer_controller'),
        'config',
        'params_hw.yaml'
        )


    """ *************** Add Nodes and other launch files ****************"""
    controller_node_sim = Node(
        package='single_trailer_controller',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        # prefix=['gdbserver localhost:3000'],
        emulate_tty=True,
        parameters=[params_sim_config],
        condition=IfCondition(use_sim)
    )

    controller_node_hw = Node(
        package='single_trailer_controller',
        executable='pure_pursuit_node',
        name='pure_pursuit_node',
        output='screen',
        # prefix=['gdbserver localhost:3000'],
        emulate_tty=True,
        parameters=[params_hw_config],
        condition=UnlessCondition(use_sim)
    )


    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('particle_filter'),
                    'launch/localize_launch.py'
            ])
        ]),
        condition=UnlessCondition(use_sim)
    )


    """ ************ Add Nodes, Launch Files, and Arguments *************"""
    launch_description.add_action(use_sim_arg)
    launch_description.add_action(controller_node_hw)
    launch_description.add_action(controller_node_sim)
    launch_description.add_action(localization_launch)

    return launch_description
