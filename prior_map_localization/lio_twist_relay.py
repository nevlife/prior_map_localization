"""FAST-LIO odometry -> base_link TwistWithCovarianceStamped for robot_localization.

FAST-LIO publishes the IMU pose in its start frame and no twist, so the twist is the motion
between consecutive scans, moved from the IMU to base_link.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rotation_log(r):
    angle = np.arccos(np.clip((np.trace(r) - 1) / 2, -1.0, 1.0))
    skew = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return 0.5 / np.sinc(angle / np.pi) * skew


class LioTwistRelay(Node):
    def __init__(self):
        super().__init__("lio_twist_relay")
        self.imu_frame = self.declare_parameter("imu_frame", Parameter.Type.STRING).value
        vx_sigma = self.declare_parameter("vx_sigma", Parameter.Type.DOUBLE).value
        vy_sigma = self.declare_parameter("vy_sigma", Parameter.Type.DOUBLE).value
        wz_sigma = self.declare_parameter("wz_sigma", Parameter.Type.DOUBLE).value
        self.covariance = [0.0] * 36
        self.covariance[0] = vx_sigma**2
        self.covariance[7] = vy_sigma**2
        self.covariance[35] = wz_sigma**2
        for i in (14, 21, 28):
            self.covariance[i] = 1e3
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_rotation = None
        self.prev = None
        self.pub = self.create_publisher(TwistWithCovarianceStamped, "~/output/twist", 20)
        self.create_subscription(Odometry, "~/input/odom", self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg):
        if self.base_rotation is None:
            # tf_static can arrive after FAST-LIO's first pose
            if not self.tf_buffer.can_transform("base_link", self.imu_frame, Time()):
                return
            tf = self.tf_buffer.lookup_transform("base_link", self.imu_frame, Time()).transform
            self.base_rotation = rotation_matrix(tf.rotation)
            self.imu_to_base = -np.array([tf.translation.x, tf.translation.y, tf.translation.z])

        stamp = Time.from_msg(msg.header.stamp)
        pose = msg.pose.pose
        rotation = rotation_matrix(pose.orientation)
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        if self.prev is not None:
            prev_stamp, prev_rotation, prev_position = self.prev
            dt = (stamp - prev_stamp).nanoseconds * 1e-9
            v_imu = prev_rotation.T @ (position - prev_position) / dt
            w_imu = rotation_log(prev_rotation.T @ rotation) / dt
            w = self.base_rotation @ w_imu
            v = self.base_rotation @ v_imu + np.cross(w, self.imu_to_base)
            self.publish(msg.header.stamp, v, w)
        self.prev = (stamp, rotation, position)

    def publish(self, stamp, v, w):
        out = TwistWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = "base_link"
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = v
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = w
        out.twist.covariance = self.covariance
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(LioTwistRelay())
