import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("ekf_line_localization")
    default_waypoint_file = os.path.join(package_share, "config", "waypoints.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoint_file),
            DeclareLaunchArgument("feedback_topic", default_value="/ekf_pose"),
            DeclareLaunchArgument("feedback_type", default_value="pose"),
            Node(
                package="ekf_line_localization",
                executable="ekf_waypoint_controller",
                name="ekf_waypoint_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                        "waypoint_file": LaunchConfiguration("waypoint_file"),
                        "feedback_topic": LaunchConfiguration("feedback_topic"),
                        "feedback_type": LaunchConfiguration("feedback_type"),
                    }
                ],
            ),
        ]
    )
