from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="homework_2",
                executable="homework_2_controller",
                name="homework_2_controller",
                output="screen",
            ),
        ]
    )
