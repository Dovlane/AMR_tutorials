from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="hello_world",
                executable="hello_world_publisher",
                name="hello_world_publisher",
                output="screen",
            ),
            Node(
                package="hello_world",
                executable="hello_world_subscriber",
                name="hello_world_subscriber",
                output="screen",
            ),
            Node(
                package="hello_world",
                executable="hello_world_service",
                name="hello_world_service",
                output="screen",
            ),
        ]
    )
