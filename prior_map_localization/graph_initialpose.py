"""/initialpose for lidar_localization_ros2 from GPS and the map bundle's road graph.

  position : GPS in the map frame (navsat_transform)
  yaw      : GPS track heading after `min_travel` m, corrected by the local odometry heading
             change; snapped to the nearest graph link heading (or its reverse) when close
  z        : low percentile of NDT map points under the robot, plus `base_height`

The pose also goes to the map EKF's set_pose. A True /reinitialization_requested re-arms.
"""

import json
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load_pcd_xyz(path):
    with open(path, "rb") as f:
        while True:
            line = f.readline().decode("ascii")
            if line.startswith("FIELDS"):
                field_count = len(line.split()) - 1
            elif line.startswith("POINTS"):
                count = int(line.split()[1])
            elif line.startswith("DATA"):
                break
        data = np.fromfile(f, dtype=np.float32, count=count * field_count)
    return data.reshape(count, field_count)[:, :3]


def load_links(path):
    with open(path) as f:
        graph = json.load(f)
    nodes = {n["ID"]: (n["Position"]["X"], n["Position"]["Y"]) for n in graph["Node"]}
    rows = []
    for link in graph["Link"]:
        (ax, ay), (bx, by) = nodes[link["FromNodeID"]], nodes[link["ToNodeID"]]
        rows.append(((ax + bx) / 2, (ay + by) / 2, math.atan2(by - ay, bx - ax)))
    return np.array(rows)


class GraphInitialPose(Node):
    def __init__(self):
        super().__init__("graph_initialpose")
        self.links = load_links(self.declare_parameter("graph_json", Parameter.Type.STRING).value)
        self.map_xyz = load_pcd_xyz(self.declare_parameter("map_pcd", Parameter.Type.STRING).value)
        self.min_travel = self.declare_parameter("min_travel", Parameter.Type.DOUBLE).value
        self.max_gps_sigma = self.declare_parameter("max_gps_sigma", Parameter.Type.DOUBLE).value
        self.snap_tol = math.radians(
            self.declare_parameter("snap_tolerance_deg", Parameter.Type.DOUBLE).value
        )
        self.max_link_dist = self.declare_parameter(
            "max_link_distance", Parameter.Type.DOUBLE
        ).value
        self.ground_radius = self.declare_parameter("ground_radius", Parameter.Type.DOUBLE).value
        self.base_height = self.declare_parameter("base_height", Parameter.Type.DOUBLE).value
        self.xy_sigma = self.declare_parameter("initial_xy_sigma", Parameter.Type.DOUBLE).value
        self.yaw_sigma = math.radians(
            self.declare_parameter("initial_yaw_sigma_deg", Parameter.Type.DOUBLE).value
        )

        self.armed = True
        self.start = None
        self.odom = None

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "~/output/initialpose", 1
        )
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "~/output/set_pose", 1)
        self.create_subscription(Odometry, "~/input/gps", self.on_gps, qos_profile_sensor_data)
        self.create_subscription(Odometry, "~/input/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Bool, "~/input/reinitialization_requested", self.on_reinit, 10)

    def on_reinit(self, msg):
        if msg.data and not self.armed:
            self.get_logger().warn("reinitialization requested, re-arming")
            self.armed = True
            self.start = None

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (np.array([p.position.x, p.position.y]), yaw_of(p.orientation))

    def on_gps(self, msg):
        if not self.armed or self.odom is None:
            return
        if math.sqrt(max(msg.pose.covariance[0], msg.pose.covariance[7])) > self.max_gps_sigma:
            return
        gps_xy = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        odom_xy, odom_yaw = self.odom
        if self.start is None:
            self.start = (gps_xy, odom_xy)
            return
        gps_d = gps_xy - self.start[0]
        odom_d = odom_xy - self.start[1]
        if np.linalg.norm(gps_d) < self.min_travel:
            return
        if np.linalg.norm(odom_d) < 0.5 * self.min_travel:
            return

        # The odometry track and the GPS track cover the same motion, so their heading
        # difference is the odom->map rotation; apply it to the current odometry heading.
        yaw = wrap(math.atan2(gps_d[1], gps_d[0]) + odom_yaw - math.atan2(odom_d[1], odom_d[0]))
        source = "gps-track"
        graph_yaw = self.graph_heading(gps_xy, yaw)
        if graph_yaw is not None:
            yaw, source = graph_yaw, "graph"
        z = self.ground_z(gps_xy)
        self.publish(msg, gps_xy, z, yaw)
        self.armed = False
        self.get_logger().info(
            f"initial pose ({gps_xy[0]:.1f}, {gps_xy[1]:.1f}, z={z:.2f}) "
            f"yaw={math.degrees(yaw):.1f} deg from {source}"
        )

    def graph_heading(self, xy, yaw_guess):
        d = np.hypot(self.links[:, 0] - xy[0], self.links[:, 1] - xy[1])
        i = int(np.argmin(d))
        if d[i] > self.max_link_dist:
            return None
        for candidate in (self.links[i, 2], wrap(self.links[i, 2] + math.pi)):
            if abs(wrap(candidate - yaw_guess)) < self.snap_tol:
                return candidate
        return None

    def ground_z(self, xy):
        dist = np.hypot(self.map_xyz[:, 0] - xy[0], self.map_xyz[:, 1] - xy[1])
        near = self.map_xyz[dist < self.ground_radius, 2]
        return float(np.percentile(near, 5)) + self.base_height

    def publish(self, gps_msg, xy, z, yaw):
        out = PoseWithCovarianceStamped()
        out.header.stamp = gps_msg.header.stamp
        out.header.frame_id = "map"
        out.pose.pose.position.x = float(xy[0])
        out.pose.pose.position.y = float(xy[1])
        out.pose.pose.position.z = z
        out.pose.pose.orientation.z = math.sin(yaw / 2)
        out.pose.pose.orientation.w = math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = self.xy_sigma**2
        cov[14] = 1.0
        cov[21] = cov[28] = 0.01
        cov[35] = self.yaw_sigma**2
        out.pose.covariance = cov
        self.initialpose_pub.publish(out)
        self.set_pose_pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(GraphInitialPose())
