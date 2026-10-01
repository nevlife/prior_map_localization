"""NDT scan matching (lidar_localization_ros2) fused with wheel, IMU and GPS (robot_localization).

  ros2 launch prior_map_localization localization.launch.py bundle_dir:=<map bundle>

bundle_dir must contain ndt_map_0p4.pcd (scripts/prepare_map.py) and graph.json.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.actions import SetRemap
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = get_package_share_directory("prior_map_localization")

    def param_file(node_name):
        return PathJoinSubstitution([share, "config", f"{node_name}.param.yaml"])

    bundle_dir = LaunchConfiguration("bundle_dir")
    map_pcd = PathJoinSubstitution([bundle_dir, "ndt_map_0p4.pcd"])
    graph_json = PathJoinSubstitution([bundle_dir, "graph.json"])
    sim = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    lio = LaunchConfiguration("lio")
    lio_transport = LaunchConfiguration("lio_transport")
    ekf_odom_params = PathJoinSubstitution(
        [
            share,
            "config",
            PythonExpression(
                ["'ekf_odom_lio.param.yaml' if '", lio, "' == 'true' else 'ekf_odom.param.yaml'"]
            ),
        ]
    )

    lidar_localization = LifecycleNode(
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        name="lidar_localization",
        namespace="",
        parameters=[param_file("lidar_localization"), sim, {"map_path": map_pcd}],
        remappings=[("/cloud", "/velodyne_points"), ("/twist", "/odometry/local_twist")],
        output="screen",
    )

    def transition(transition_id):
        return EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(lidar_localization),
                transition_id=transition_id,
            )
        )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state="configuring",
            goal_state="inactive",
            entities=[transition(Transition.TRANSITION_ACTIVATE)],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("bundle_dir"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("lio", default_value="false"),
            DeclareLaunchArgument("lio_transport", default_value="dds", choices=["dds", "flux"]),
            Node(
                package="prior_map_localization",
                executable="wheel_twist_relay",
                name="wheel_twist_relay",
                parameters=[param_file("wheel_twist_relay"), sim],
                remappings=[
                    ("~/input/velocity", "/hunter/velocity"),
                    ("~/output/twist", "/wheel/twist"),
                ],
            ),
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
            Node(
                package="prior_map_localization",
                executable="gps_cov_floor",
                name="gps_cov_floor",
                parameters=[param_file("gps_cov_floor"), sim],
                remappings=[
                    ("~/input/fix", "/ublox_gps_node/fix"),
                    ("~/output/fix", "/gps/fix_floored"),
                ],
            ),
            GroupAction(
                [
                    Node(
                        package="prior_map_localization",
                        executable="scan_start_relay",
                        name="scan_start_relay",
                        parameters=[sim, {"transport": lio_transport}],
                        remappings=[
                            ("~/input/points", "/velodyne_points"),
                            ("~/output/points", "/lio/points"),
                        ],
                    ),
                    # FAST-LIO broadcasts camera_init->body; ekf_odom owns odom->base_link.
                    Node(
                        package="fast_lio",
                        executable="fastlio_mapping",
                        name="fast_lio",
                        parameters=[
                            param_file("fast_lio"),
                            sim,
                            {"common.lid_transport": lio_transport},
                        ],
                        remappings=[("/tf", "/fast_lio/tf"), ("/Odometry", "/lio/odometry")],
                    ),
                    Node(
                        package="prior_map_localization",
                        executable="lio_twist_relay",
                        name="lio_twist_relay",
                        parameters=[param_file("lio_twist_relay"), sim],
                        remappings=[
                            ("~/input/odom", "/lio/odometry"),
                            ("~/output/twist", "/lio/twist"),
                        ],
                    ),
                ],
                condition=IfCondition(lio),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_odom",
                parameters=[ekf_odom_params, sim],
                remappings=[
                    ("odometry/filtered", "/odometry/local"),
                    ("set_pose", "/ekf_odom/set_pose"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_map",
                parameters=[param_file("ekf_map"), sim],
                remappings=[
                    ("odometry/filtered", "/odometry/global"),
                    ("set_pose", "/ekf_map/set_pose"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                parameters=[param_file("navsat_transform"), sim],
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
                parameters=[
                    param_file("graph_initialpose"),
                    sim,
                    {"graph_json": graph_json, "map_pcd": map_pcd},
                ],
                remappings=[
                    ("~/input/gps", "/odometry/gps"),
                    ("~/input/odom", "/odometry/local"),
                    ("~/input/reinitialization_requested", "/reinitialization_requested"),
                    ("~/output/initialpose", "/initialpose"),
                    ("~/output/set_pose", "/ekf_map/set_pose"),
                ],
            ),
            # lidar_localization always broadcasts map->base_link; ekf_map owns map->odom.
            GroupAction([SetRemap(src="/tf", dst="/lidar_localization/tf"), lidar_localization]),
            activate,
            transition(Transition.TRANSITION_CONFIGURE),
        ]
    )
