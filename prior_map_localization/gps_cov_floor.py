"""Clamp NavSatFix covariance to a per-fix-type floor before navsat_transform.

The receiver reports sub-metre sigma in float fixes, and the map bundle's georeference has
1.2 m RMSE, so GPS cannot agree with the PCD map better than that.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus


class GpsCovFloor(Node):
    def __init__(self):
        super().__init__("gps_cov_floor")
        self.rtk_floor = self.declare_parameter("rtk_sigma_floor", Parameter.Type.DOUBLE).value
        self.float_floor = self.declare_parameter("float_sigma_floor", Parameter.Type.DOUBLE).value
        self.pub = self.create_publisher(NavSatFix, "~/output/fix", 10)
        self.create_subscription(NavSatFix, "~/input/fix", self.on_fix, qos_profile_sensor_data)

    def on_fix(self, msg):
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        rtk = msg.status.status >= NavSatStatus.STATUS_GBAS_FIX
        floor = self.rtk_floor if rtk else self.float_floor
        reported = msg.position_covariance
        cov = [0.0] * 9
        cov[0] = max(reported[0], floor**2)
        cov[4] = max(reported[4], floor**2)
        cov[8] = max(reported[8], (2.0 * floor) ** 2)
        msg.position_covariance = cov
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(GpsCovFloor())
