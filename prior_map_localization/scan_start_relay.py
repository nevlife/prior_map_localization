"""Re-reference velodyne point times to the scan start for FAST-LIO.

The driver stamps a scan at its end and gives point times in (-0.1, 0] s. FAST-LIO takes the
header stamp as the scan start and expects point times in [0, scan period).

The input is a reliable DDS subscription: a 1.8 MB scan over best-effort DDS is lost whole when
one fragment is. `transport` picks DDS or flux for the output, so the two can be compared.
"""

import flux
import flux.ros
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_flux.point_cloud2 import PointCloud2 as FluxPointCloud2
from sensor_msgs_flux.point_cloud2_ros import msg_to_frame


class ScanStartRelay(Node):
    def __init__(self):
        super().__init__("scan_start_relay")
        transport = self.declare_parameter("transport", Parameter.Type.STRING).value
        if transport == "flux":
            self.flux_pub = flux.ros.create_publisher(
                self, "~/output/points", fingerprint=FluxPointCloud2.FINGERPRINT__
            )
            self.publish = self.publish_flux
        else:
            self.pub = self.create_publisher(
                PointCloud2, "~/output/points", qos_profile_sensor_data
            )
            self.publish = self.pub.publish
        self.create_subscription(PointCloud2, "~/input/points", self.on_cloud, 5)

    def on_cloud(self, msg):
        offset = next(f.offset for f in msg.fields if f.name == "time")
        data = np.frombuffer(msg.data, np.uint8).copy()
        times = np.ndarray(
            (msg.width * msg.height,), np.float32, data, offset, (msg.point_step,)
        )
        start = float(times.min())
        times -= start
        stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec + round(start * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 10**9)
        msg.data = data.tobytes()
        self.publish(msg)

    def publish_flux(self, msg):
        builder = FluxPointCloud2.build__(self.flux_pub)
        msg_to_frame(msg, builder)
        published = builder.commit__()
        if flux.faulted(published):
            self.get_logger().error(
                f"flux publish failed: {published}", throttle_duration_sec=5.0
            )


def main():
    rclpy.init()
    rclpy.spin(ScanStartRelay())
