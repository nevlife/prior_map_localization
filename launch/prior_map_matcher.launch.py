"""prior_map_matcher, on the topics of the localization stack.

ros2 launch prior_map_localization prior_map_matcher.launch.py \
    map_config:=<param file with map_pcd> [container:=/<running container>]

Without container it starts its own capa_component_container.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    sim = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    matcher = ComposableNode(
        package="prior_map_localization",
        plugin="prior_map_localization::PriorMapMatcher",
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
            ("~/input/points", "/velodyne_points"),
            ("~/input/odom", "/odometry/local"),
            ("~/input/initialpose", "/initialpose"),
            ("~/output/pose", "/pcl_pose"),
            ("~/output/reinitialization_requested", "/reinitialization_requested"),
            ("~/debug/alignment_status", "/alignment_status"),
        ],
    )
    container = LaunchConfiguration("container").perform(context)
    if container:
        return [
            LoadComposableNodes(target_container=container, composable_node_descriptions=[matcher])
        ]
    return [
        ComposableNodeContainer(
            package="capa_ros",
            executable="capa_component_container",
            name="prior_map_matcher_container",
            namespace="",
            composable_node_descriptions=[matcher],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_config"),
            DeclareLaunchArgument("transport", default_value="dds", choices=["dds", "flux"]),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("container", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
