#!/usr/bin/env python3
"""Hunter wheel speed (std_msgs/Float64) -> TwistWithCovarianceStamped for robot_localization.

The driver message has no header, so the stamp is the reception time. The twist is expressed
at the rear axle with lateral velocity 0, the only point of an Ackermann base that does not slide
sideways; robot_localization moves it to base_link with the yaw rate.
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
        out.header.frame_id = "rear_axle"
        out.twist.twist.linear.x = msg.data * self.scale
        out.twist.covariance = self.covariance
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(WheelTwistRelay())


if __name__ == "__main__":
    main()
