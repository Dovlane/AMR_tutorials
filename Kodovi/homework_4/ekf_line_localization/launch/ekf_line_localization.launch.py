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

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("map_file", default_value=default_map_file),
            DeclareLaunchArgument("enable_correction", default_value="true"),
            DeclareLaunchArgument("validation_gate", default_value="3.0"),
            DeclareLaunchArgument("pose_topic", default_value="/ekf_pose"),
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
                        "pose_topic": LaunchConfiguration("pose_topic"),
                    }
                ],
            ),
        ]
    )
