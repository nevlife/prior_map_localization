"""prior_map_matcher alone, on the topics of the localization stack.

ros2 launch prior_map_localization prior_map_matcher.launch.py map_config:=<param file with map_pcd>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_config"),
            DeclareLaunchArgument("transport", default_value="dds", choices=["dds", "flux"]),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="prior_map_localization",
                executable="prior_map_matcher",
                name="prior_map_matcher",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("prior_map_localization"),
                            "config",
                            "prior_map_matcher.param.yaml",
                        ]
                    ),
                    LaunchConfiguration("map_config"),
                    sim,
                    {"transport": LaunchConfiguration("transport")},
                ],
                remappings=[
                    ("~/input/points", "/lio/points"),
                    ("~/input/odom", "/odometry/local"),
                    ("~/input/initialpose", "/initialpose"),
                    ("~/output/pose", "/pcl_pose"),
                    ("~/output/reinitialization_requested", "/reinitialization_requested"),
                    ("~/debug/alignment_status", "/alignment_status"),
                ],
                output="screen",
            ),
        ]
    )
