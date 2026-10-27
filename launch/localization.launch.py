"""NDT scan matching fused with wheel, IMU, GPS and optionally FAST-LIO (robot_localization).

  ros2 launch prior_map_localization localization.launch.py [map_config:=<file>] [lio:=true]

map_config names the map files (config/map.param.yaml by default).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory("prior_map_localization")


def config(name):
    return os.path.join(SHARE, "config", f"{name}.param.yaml")


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
        for n in ("map_config", "use_sim_time", "lio", "lidar_transport")
    }
    sim = {"use_sim_time": arg["use_sim_time"] == "true"}
    lio = arg["lio"] == "true"
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
            executable="bbs_initialpose",
            name="bbs_initialpose",
            parameters=[
                config("bbs_initialpose"),
                arg["map_config"],
                sim,
                {"transport": arg["lidar_transport"]},
            ],
            remappings=[
                ("~/input/points", "/velodyne_points"),
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
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_config", default_value=config("map")),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("lio", default_value="false"),
            # /velodyne_points: the same value as transport of bring_up lidar.launch.py.
            DeclareLaunchArgument("lidar_transport", default_value="dds", choices=["dds", "flux"]),
            # Empty leaves every thread as it starts; config/capa_chain.yaml is the schedule.
            DeclareLaunchArgument("capa_spec", default_value=""),
            SetEnvironmentVariable("CAPA_SPEC", LaunchConfiguration("capa_spec")),
            OpaqueFunction(function=launch_setup),
        ]
    )
