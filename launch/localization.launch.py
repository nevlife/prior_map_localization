"""Localization by one of four methods; only the nodes of that method start.

  ros2 launch prior_map_localization localization.launch.py [method:=<method>] \
      [lio:=false] [map_config:=<file>]

  fastlio_gps  (default) ekf_odom (wheel, FAST-LIO, IMU) and ekf_map (the same + GPS)
  mins_wheel   MINS (IMU, camera, wheel), ekf_odom (MINS twist) and ekf_map (MINS twist + GPS)
  mins_full    MINS (IMU, camera, wheel, GPS)
  prior_map    NDT prior_map_matcher, bbs_initialpose, ekf_odom and ekf_map (+ NDT, GPS)

Each method reads config/localization_<method>.param.yaml. lio and map_config apply to prior_map.
"""

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
import yaml

SHARE = get_package_share_directory("prior_map_localization")
CONTAINER = "localization_container"
METHODS = ["fastlio_gps", "mins_wheel", "mins_full", "prior_map"]


def config(name):
    return os.path.join(SHARE, "config", f"{name}.param.yaml")


def ros_parameters(path, node):
    with open(path) as stream:
        return yaml.safe_load(stream)[node]["ros__parameters"]


def map_files(map_config):
    """The map_config paths, relative ones taken from the workspace's maps/ directory."""
    # The isolated install layout puts the package prefix at <workspace>/install/<package>.
    prefix = get_package_prefix("prior_map_localization")
    maps = os.path.join(os.path.dirname(os.path.dirname(prefix)), "maps")
    with open(map_config) as stream:
        parameters = next(iter(yaml.safe_load(stream).values()))["ros__parameters"]
    return {key: os.path.join(maps, path) for key, path in parameters.items()}


def component(name, plugin, parameters, remappings):
    return ComposableNode(
        package="prior_map_localization",
        plugin=f"prior_map_localization::{plugin}",
        name=name,
        parameters=parameters,
        remappings=remappings,
    )


def container(components):
    # One thread per callback group; with CAPA_SPEC set, capa binds the groups its file names.
    return ComposableNodeContainer(
        package="capa_ros",
        executable="capa_component_container",
        name=CONTAINER,
        namespace="",
        composable_node_descriptions=components,
        output="screen",
    )


def wheel_twist_relay(sim):
    return component(
        "wheel_twist_relay",
        "WheelTwistRelay",
        [config("wheel_twist_relay"), config("wheel"), sim],
        [("~/input/odom", "/odometry/wheel"), ("~/output/twist", "/wheel/twist")],
    )


def wheel_joint_state(sim):
    return component(
        "wheel_joint_state",
        "WheelJointState",
        [config("wheel_joint_state"), config("wheel"), sim],
        [("~/input/odom", "/odometry/wheel"), ("~/output/joint_state", "/wheel/joint_state")],
    )


def gps_cov_floor(sim):
    return component(
        "gps_cov_floor",
        "GpsCovFloor",
        [config("gps_cov_floor"), sim],
        [("~/input/fix", "/ublox_gps_node/fix"), ("~/output/fix", "/gps/fix_floored")],
    )


def lio_twist_relay(sim):
    return component(
        "lio_twist_relay",
        "LioTwistRelay",
        [config("lio_twist_relay"), sim],
        [
            ("~/input/odom", "/lio/odometry"),
            ("~/input/imu", "/vectornav/imu"),
            ("~/output/twist", "/lio/twist"),
            ("~/output/imu", "/imu/corrected"),
        ],
    )


def mins_twist_relay(sim):
    return component(
        "mins_twist_relay",
        "MinsTwistRelay",
        [config("mins_twist_relay"), sim],
        [("~/input/odom", "/mins/imu/odom"), ("~/output/twist", "/mins/twist")],
    )


def rear_axle_tf(sim):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rear_axle_tf",
        arguments=["--x", "-0.28", "--frame-id", "base_link", "--child-frame-id", "rear_axle"],
        parameters=[sim],
    )


def ekf(name, output, parameters):
    return Node(
        package="robot_localization",
        executable="ekf_node",
        name=name,
        parameters=parameters,
        remappings=[("odometry/filtered", output), ("set_pose", f"/{name}/set_pose")],
    )


def navsat_transform(sim):
    return Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        parameters=[config("navsat_transform"), sim],
        remappings=[
            ("gps/fix", "/gps/fix_floored"),
            ("imu", "/navsat_transform/unused_imu"),
            ("odometry/filtered", "/odometry/global"),
        ],
    )


def fast_lio(sim):
    # FAST-LIO broadcasts camera_init->body; ekf_odom owns odom->base_link.
    return Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fast_lio",
        parameters=[config("fast_lio"), sim],
        remappings=[("/tf", "/fast_lio/tf"), ("/Odometry", "/lio/odometry")],
    )


def mins(method_config, sim):
    relative = ros_parameters(method_config, "mins")["config_path"]
    path = os.path.join(get_package_share_directory("mins"), "config", relative)
    # MINS broadcasts global->imu and its clone frames, which are not part of the robot's tree.
    return Node(
        package="mins",
        executable="subscribe",
        name="mins",
        output="screen",
        parameters=[{"config_path": path}, sim],
        remappings=[("/tf", "/mins/tf")],
    )


def fastlio_gps(arg, sim, method_config):
    ekf_parameters = [method_config, sim]
    return [
        container([wheel_twist_relay(sim), gps_cov_floor(sim), lio_twist_relay(sim)]),
        rear_axle_tf(sim),
        fast_lio(sim),
        ekf("ekf_odom", "/odometry/local", ekf_parameters),
        ekf("ekf_map", "/odometry/global", ekf_parameters),
        navsat_transform(sim),
    ]


def mins_wheel(arg, sim, method_config):
    return [
        container([wheel_joint_state(sim), mins_twist_relay(sim), gps_cov_floor(sim)]),
        mins(method_config, sim),
        ekf("ekf_odom", "/odometry/local", [method_config, sim]),
        ekf("ekf_map", "/odometry/global", [method_config, sim]),
        navsat_transform(sim),
    ]


def mins_full(arg, sim, method_config):
    # gps_cov_floor drops the fixes without a position; MINS takes every NavSatFix it gets.
    return [
        container([wheel_joint_state(sim), gps_cov_floor(sim)]),
        mins(method_config, sim),
    ]


def prior_map(arg, sim, method_config):
    lio = arg["lio"] == "true"
    ekf_parameters = [method_config, sim]
    if not lio:
        ekf_parameters.append({"imu0": "/vectornav/imu", "twist1_config": [False] * 15})
    bbs_initialpose = component(
        "bbs_initialpose",
        "BbsInitialPose",
        [
            config("bbs_initialpose"),
            map_files(arg["map_config"]),
            sim,
        ],
        [
            ("~/input/points", "/velodyne_points/filtered"),
            ("~/input/gps", "/odometry/gps"),
            ("~/input/odom", "/odometry/local"),
            ("~/input/reinitialization_requested", "/reinitialization_requested"),
            ("~/output/initialpose", "/initialpose"),
            ("~/output/set_pose", "/ekf_map/set_pose"),
        ],
    )
    components = [wheel_twist_relay(sim), gps_cov_floor(sim), bbs_initialpose]
    actions = [
        container(components + ([lio_twist_relay(sim)] if lio else [])),
        rear_axle_tf(sim),
        ekf("ekf_odom", "/odometry/local", ekf_parameters),
        ekf("ekf_map", "/odometry/global", ekf_parameters),
        navsat_transform(sim),
    ]
    if lio:
        actions.append(fast_lio(sim))
    actions.append(
        IncludeLaunchDescription(
            os.path.join(SHARE, "launch", "prior_map_matcher.launch.py"),
            launch_arguments={
                "map_config": arg["map_config"],
                "use_sim_time": arg["use_sim_time"],
                "container": "/" + CONTAINER,
            }.items(),
        )
    )
    return actions


def launch_setup(context):
    names = ("method", "map_config", "use_sim_time", "lio")
    arg = {n: LaunchConfiguration(n).perform(context) for n in names}
    sim = {"use_sim_time": arg["use_sim_time"] == "true"}
    method = {
        "fastlio_gps": fastlio_gps,
        "mins_wheel": mins_wheel,
        "mins_full": mins_full,
        "prior_map": prior_map,
    }[arg["method"]]
    return method(arg, sim, config(f"localization_{arg['method']}"))


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("method", default_value="fastlio_gps", choices=METHODS),
            DeclareLaunchArgument("map_config", default_value=config("map")),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("lio", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
