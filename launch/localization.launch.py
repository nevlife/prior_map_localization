"""NDT scan matching fused with wheel, IMU, GPS and optionally FAST-LIO (robot_localization).

  ros2 launch prior_map_localization localization.launch.py [map_config:=<file>] [lio:=true]

map_config names the map files (config/map.param.yaml by default).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import SetEnvironmentVariable
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import yaml

SHARE = get_package_share_directory("prior_map_localization")


def config(name):
    return os.path.join(SHARE, "config", f"{name}.param.yaml")


def lidar_localization(map_config, transport, sim):
    with open(map_config) as f:
        map_pcd = yaml.safe_load(f)["/**"]["ros__parameters"]["map_pcd"]
    node = LifecycleNode(
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        name="lidar_localization",
        namespace="",
        parameters=[
            config("lidar_localization"),
            sim,
            {"map_path": map_pcd, "cloud_transport": transport},
        ],
        remappings=[("/cloud", "/velodyne_points"), ("/twist", "/odometry/local_twist")],
        output="screen",
    )

    def transition(transition_id):
        return EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(node), transition_id=transition_id
            )
        )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[transition(Transition.TRANSITION_ACTIVATE)],
        )
    )
    return [
        Node(
            package="prior_map_localization",
            executable="odom_twist_relay",
            name="odom_twist_relay",
            parameters=[sim],
            remappings=[
                ("~/input/odom", "/odometry/local"),
                ("~/output/twist", "/odometry/local_twist"),
            ],
        ),
        # lidar_localization always broadcasts map->base_link; ekf_map owns map->odom.
        GroupAction(
            [
                SetRemap(src="/tf", dst="/lidar_localization/tf"),
                node,
                activate,
                transition(Transition.TRANSITION_CONFIGURE),
            ]
        ),
    ]


def fast_lio(transport, sim):
    return [
        # FAST-LIO broadcasts camera_init->body; ekf_odom owns odom->base_link.
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fast_lio",
            parameters=[config("fast_lio"), sim, {"common.lid_transport": transport}],
            remappings=[("/tf", "/fast_lio/tf"), ("/Odometry", "/lio/odometry")],
        ),
        Node(
            package="prior_map_localization",
            executable="lio_twist_relay",
            name="lio_twist_relay",
            parameters=[config("lio_twist_relay"), sim],
            remappings=[("~/input/odom", "/lio/odometry"), ("~/output/twist", "/lio/twist")],
        ),
    ]


def ekf(name, output, lio, sim):
    return Node(
        package="robot_localization",
        executable="ekf_node",
        name=name,
        parameters=[config(name)] + ([config("ekf_lio")] if lio else []) + [sim],
        remappings=[("odometry/filtered", output), ("set_pose", f"/{name}/set_pose")],
    )


def launch_setup(context):
    arg = {
        n: LaunchConfiguration(n).perform(context)
        for n in ("map_config", "use_sim_time", "lio", "lidar_transport", "localizer")
    }
    sim = {"use_sim_time": arg["use_sim_time"] == "true"}
    lio = arg["lio"] == "true"
    use_prior_map_matcher = arg["localizer"] == "prior_map_matcher"
    actions = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="rear_axle_tf",
            arguments=["--x", "-0.28", "--frame-id", "base_link", "--child-frame-id", "rear_axle"],
            parameters=[sim],
        ),
        Node(
            package="prior_map_localization",
            executable="wheel_twist_relay",
            name="wheel_twist_relay",
            parameters=[config("wheel_twist_relay"), sim],
            remappings=[
                ("~/input/velocity", "/hunter/velocity"),
                ("~/output/twist", "/wheel/twist"),
            ],
        ),
        Node(
            package="prior_map_localization",
            executable="gps_cov_floor",
            name="gps_cov_floor",
            parameters=[config("gps_cov_floor"), sim],
            remappings=[
                ("~/input/fix", "/ublox_gps_node/fix"),
                ("~/output/fix", "/gps/fix_floored"),
            ],
        ),
        ekf("ekf_odom", "/odometry/local", lio, sim),
        ekf("ekf_map", "/odometry/global", lio, sim),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            parameters=[config("navsat_transform"), sim],
            remappings=[
                ("gps/fix", "/gps/fix_floored"),
                ("imu", "/navsat_transform/unused_imu"),
                ("odometry/filtered", "/odometry/global"),
            ],
        ),
        Node(
            package="prior_map_localization",
            executable="graph_initialpose",
            name="graph_initialpose",
            parameters=[config("graph_initialpose"), arg["map_config"], sim],
            remappings=[
                ("~/input/gps", "/odometry/gps"),
                ("~/input/odom", "/odometry/local"),
                ("~/input/reinitialization_requested", "/reinitialization_requested"),
                ("~/output/initialpose", "/initialpose"),
                ("~/output/set_pose", "/ekf_map/set_pose"),
            ],
        ),
    ]
    if lio:
        actions += fast_lio(arg["lidar_transport"], sim)
    if use_prior_map_matcher:
        actions.append(
            IncludeLaunchDescription(
                os.path.join(SHARE, "launch", "prior_map_matcher.launch.py"),
                launch_arguments={
                    "map_config": arg["map_config"],
                    "transport": arg["lidar_transport"],
                    "use_sim_time": arg["use_sim_time"],
                }.items(),
            )
        )
    else:
        actions += lidar_localization(arg["map_config"], arg["lidar_transport"], sim)
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_config", default_value=config("map")),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("lio", default_value="false"),
            # /velodyne_points: the same value as transport of bring_up lidar.launch.py.
            DeclareLaunchArgument("lidar_transport", default_value="dds", choices=["dds", "flux"]),
            DeclareLaunchArgument(
                "localizer",
                default_value="lidar_localization",
                choices=["lidar_localization", "prior_map_matcher"],
            ),
            # Empty leaves every thread as it starts; config/capa_chain.yaml is the schedule.
            DeclareLaunchArgument("capa_spec", default_value=""),
            SetEnvironmentVariable("CAPA_SPEC", LaunchConfiguration("capa_spec")),
            OpaqueFunction(function=launch_setup),
        ]
    )
