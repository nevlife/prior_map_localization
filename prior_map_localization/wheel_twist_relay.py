"""Hunter wheel speed (std_msgs/Float64) -> TwistWithCovarianceStamped for robot_localization.

The driver message has no header, so the stamp is the reception time. Lateral velocity is
published as 0: an Ackermann base does not slide sideways.
"""

from geometry_msgs.msg import TwistWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64


class WheelTwistRelay(Node):
    def __init__(self):
        super().__init__("wheel_twist_relay")
        self.scale = self.declare_parameter("wheel_scale", Parameter.Type.DOUBLE).value
        vx_sigma = self.declare_parameter("vx_sigma", Parameter.Type.DOUBLE).value
        vy_sigma = self.declare_parameter("vy_sigma", Parameter.Type.DOUBLE).value
        self.covariance = [0.0] * 36
        self.covariance[0] = vx_sigma**2
        self.covariance[7] = vy_sigma**2
        for i in (14, 21, 28, 35):
            self.covariance[i] = 1e3
        self.pub = self.create_publisher(TwistWithCovarianceStamped, "~/output/twist", 20)
        self.create_subscription(
            Float64, "~/input/velocity", self.on_speed, qos_profile_sensor_data
        )

    def on_speed(self, msg):
        out = TwistWithCovarianceStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.twist.twist.linear.x = msg.data * self.scale
        out.twist.covariance = self.covariance
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(WheelTwistRelay())
