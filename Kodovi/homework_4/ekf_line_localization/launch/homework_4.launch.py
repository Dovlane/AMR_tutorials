import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("ekf_line_localization")
    default_map_file = os.path.join(
        package_share,
        "config",
        "turtlebot3_maze_lines.yaml",
    )
    default_waypoint_file = os.path.join(package_share, "config", "waypoints.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_file", default_value=default_map_file),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoint_file),
            DeclareLaunchArgument("enable_correction", default_value="true"),
            DeclareLaunchArgument("validation_gate", default_value="3.0"),
            DeclareLaunchArgument("map_marker_length", default_value="1.6"),
            DeclareLaunchArgument("feedback_topic", default_value="/ekf_pose"),
            DeclareLaunchArgument("feedback_type", default_value="pose"),
            DeclareLaunchArgument("start_delay_seconds", default_value="0.0"),
            Node(
                package="ekf_line_localization",
                executable="ekf_line_localization",
                name="ekf_line_localization",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"),
                            value_type=bool,
                        ),
                        "map_file": LaunchConfiguration("map_file"),
                        "enable_correction": ParameterValue(
                            LaunchConfiguration("enable_correction"),
                            value_type=bool,
                        ),
                        "validation_gate": ParameterValue(
                            LaunchConfiguration("validation_gate"),
                            value_type=float,
                        ),
                        "map_marker_length": ParameterValue(
                            LaunchConfiguration("map_marker_length"),
                            value_type=float,
                        ),
                    }
                ],
            ),
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
                        "start_delay_seconds": ParameterValue(
                            LaunchConfiguration("start_delay_seconds"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
