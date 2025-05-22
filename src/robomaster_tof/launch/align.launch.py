from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("rm_name", default_value="rm0"),
            Node(
                package="robomaster_tof",
                executable="align_controller",
                namespace=LaunchConfiguration("rm_name"),
                output="screen",
                # Uncomment to enable DEBUG logging messages
                # arguments=['--ros-args', '--log-level', 'debug']
            ),
        ]
    )
