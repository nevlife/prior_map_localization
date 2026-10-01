"""nav_msgs/Odometry -> TwistWithCovarianceStamped for lidar_localization_ros2's scan seed.

lidar_localization_ros2 seeds a scan match from a twist topic, not from `use_odom`.
"""

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class OdomTwistRelay(Node):
    def __init__(self):
        super().__init__("odom_twist_relay")
        self.pub = self.create_publisher(TwistWithCovarianceStamped, "~/output/twist", 20)
        self.create_subscription(Odometry, "~/input/odom", self.on_odom, qos_profile_sensor_data)

    def on_odom(self, msg):
        out = TwistWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.child_frame_id
        out.twist = msg.twist
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(OdomTwistRelay())
