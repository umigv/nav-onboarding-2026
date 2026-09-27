import math
from datetime import UTC, datetime

import utils.config
import utils.lifecycle
from geometry_msgs.msg import (
    Pose,
    PoseWithCovariance,
    Transform,
    TransformStamped,
    Twist,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
    Vector3,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros.transform_broadcaster import TransformBroadcaster
from utils.geometry import Point2d, Rotation2d

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")

        self.config: EncOdomPublisherConfig = utils.config.load(self, EncOdomPublisherConfig)
        self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_covariance = [0] * 36
        self.publisher = self.create_publisher(Odometry, "odom", 10)
        self._last_env_vel_message_time: datetime | None = None
        self.create_service(Trigger, "reset", self.from_reset_callback)
        self._heading: float = 0
        self._x: float = 0
        self._y: float = 0

    def enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        if self._last_env_vel_message_time:
            dt = (datetime.now(UTC) - self._last_env_vel_message_time).total_seconds()
            if dt > 1 or dt <= 0:
                return
            self.wz = msg.twist.twist.angular.z
            self.vx = msg.twist.twist.linear.x
            mid_heading = self._heading + 0.5 * self.wz * dt  # midpoint method, more accurate than basic Euler's Method
            self._x += self.vx * dt * math.cos(mid_heading)
            self._y += self.vx * dt * math.sin(mid_heading)
            self._heading += self.wz * dt
            self.time = self.get_clock().now().to_msg()
            self.publish_odom()
            self.publish_transform()
        self._last_env_vel_message_time = datetime.now(UTC)

    def publish_odom(self) -> None:
        self.publisher.publish(
            Odometry(
                header=Header(stamp=self.time, frame_id=self.config.odom_frame_id),
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point2d(x=self._x, y=self._y).to_ros(),
                        orientation=Rotation2d(angle=self._heading).to_ros(),
                    ),
                    covariance=self.pose_covariance,
                ),
                twist=TwistWithCovariance(
                    twist=Twist(linear=Vector3(x=self.vx, y=0, z=0), angular=Vector3(x=0, y=0, z=self.wz))
                ),
            )
        )

    def publish_transform(self) -> None:
        self.tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(stamp=self.time, frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                transform=Transform(
                    translation=Vector3(x=self._x, y=self._y, z=0.0), rotation=Rotation2d(self._heading).to_ros()
                ),
            )
        )

    def from_reset_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._x, self._y, self._heading = (0, 0, 0)
        self.time = self.get_clock().now().to_msg()
        self.publish_odom()
        response.success = True
        return response


def main() -> None:
    utils.lifecycle.run_node(EncOdomPublisher)
