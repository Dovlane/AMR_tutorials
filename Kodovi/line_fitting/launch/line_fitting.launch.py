from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scan_topic = LaunchConfiguration("scan_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    split_threshold = LaunchConfiguration("split_threshold")
    merge_threshold = LaunchConfiguration("merge_threshold")
    min_points = LaunchConfiguration("min_points")
    max_point_gap = LaunchConfiguration("max_point_gap")

    return LaunchDescription(
        [
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("marker_topic", default_value="/line_markers"),
            DeclareLaunchArgument("split_threshold", default_value="0.04"),
            DeclareLaunchArgument("merge_threshold", default_value="0.04"),
            DeclareLaunchArgument("min_points", default_value="6"),
            DeclareLaunchArgument("max_point_gap", default_value="0.25"),
            Node(
                package="line_fitting",
                executable="line_fitting_node",
                name="line_fitting_node",
                output="screen",
                parameters=[
                    {
                        "scan_topic": scan_topic,
                        "marker_topic": marker_topic,
                        "split_threshold": ParameterValue(
                            split_threshold,
                            value_type=float,
                        ),
                        "merge_threshold": ParameterValue(
                            merge_threshold,
                            value_type=float,
                        ),
                        "min_points": ParameterValue(min_points, value_type=int),
                        "max_point_gap": ParameterValue(
                            max_point_gap,
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
