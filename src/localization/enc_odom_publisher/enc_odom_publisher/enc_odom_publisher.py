import math
from datetime import UTC, datetime

import utils.config
import utils.lifecycle
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")

        self.config: EncOdomPublisherConfig = utils.config.load(self, EncOdomPublisherConfig)
        self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)
        self.publisher = self.create_publisher(Odometry, "odom", 10)
        self._last_env_vel_message_time: datetime | None = None
        self._heading: float = 0
        self._x: float = 0
        self._y: float = 0

    def enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        if self._last_env_vel_message_time:
            dt = (datetime.now(UTC) - self._last_env_vel_message_time).total_seconds()
            if dt > 1 or dt <= 0:
                return
            wz = msg.twist.twist.angular.z
            vx = msg.twist.twist.linear.x
            mid_heading = self._heading + 0.5 * wz * dt  # midpoint method, more accurate than basic Euler's Method
            self._x += vx * dt * math.cos(mid_heading)
            self._y += vx * dt * math.sin(mid_heading)
            self._heading += wz * dt
        self._last_env_vel_message_time = datetime.now(UTC)

    def publish_odom(self) -> None:
        return


def main() -> None:
    utils.lifecycle.run_node(EncOdomPublisher)
