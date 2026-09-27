import math

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
from rclpy.time import Time
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros.transform_broadcaster import TransformBroadcaster
from utils.geometry import Point2d, Rotation2d

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")

        self.config: EncOdomPublisherConfig = utils.config.load(self, EncOdomPublisherConfig)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(Odometry, "odom", 10)
        self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)
        self.create_service(Trigger, "reset", self.from_reset_callback)

        self._last_env_vel_message_time: Time | None = None
        self.heading: float = 0
        self.x: float = 0
        self.y: float = 0

        self.pose_covariance = [0] * 36

    def enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        msg_time = Time.from_msg(msg.header.stamp)
        if self._last_env_vel_message_time:
            dt = (msg_time - self._last_env_vel_message_time).nanoseconds / 1e9  # s
            if dt > 1 or dt <= 0:
                return

            self.wz = msg.twist.twist.angular.z  # radians/s
            self.vx = msg.twist.twist.linear.x  # m/s
            mid_heading = self.heading + 0.5 * self.wz * dt  # midpoint method, more accurate than basic Euler's Method
            self.x += self.vx * dt * math.cos(mid_heading)
            self.y += self.vx * dt * math.sin(mid_heading)
            self.heading += self.wz * dt

            self.publish_odom(msg_time)
            self.publish_transform(msg_time)

        self._last_env_vel_message_time = msg_time

    def publish_odom(self, time: Time) -> None:
        self.publisher.publish(
            Odometry(
                header=Header(stamp=time.to_msg(), frame_id=self.config.odom_frame_id),
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point2d(x=self.x, y=self.y).to_ros(),
                        orientation=Rotation2d(angle=self.heading).to_ros(),
                    ),
                    covariance=self.pose_covariance,
                ),
                twist=TwistWithCovariance(
                    twist=Twist(linear=Vector3(x=self.vx, y=0, z=0), angular=Vector3(x=0, y=0, z=self.wz))
                ),
            )
        )

    def publish_transform(self, time: Time) -> None:
        self.tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(stamp=time.to_msg(), frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                transform=Transform(
                    translation=Vector3(x=self.x, y=self.y, z=0.0), rotation=Rotation2d(self.heading).to_ros()
                ),
            )
        )

    def from_reset_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.x, self.y, self.heading = (0, 0, 0)
        now = self.get_clock().now()
        self.publish_odom(now)
        self.publish_transform(now)
        response.success = True
        return response


def main() -> None:
    utils.lifecycle.run_node(EncOdomPublisher)
