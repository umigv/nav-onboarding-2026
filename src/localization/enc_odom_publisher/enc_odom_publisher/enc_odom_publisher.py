import math

import utils.config
import utils.lifecycle
from geometry_msgs.msg import (
    Pose,
    PoseWithCovariance,
    Transform,
    Twist,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
    Vector3,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import Header, TransformBroadcaster, TransformStamped
from utils.geometry import Point2d, Rotation2d

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")
        self.config: EncOdomPublisherConfig = utils.config.load(self, EncOdomPublisherConfig)

        self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.srv = self.create_service(Trigger, "reset_odom", self.reset_callback)

        self.stamp: float | None = None
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.heading = 0.0

        self.pose_covariance = [0.0] * 36
        self.pose_covariance[0] = 1e-3  # x
        self.pose_covariance[7] = 1e-3  # y
        self.pose_covariance[35] = 1e-3  # z-rotation

        self.twist_covariance = [0.0] * 36
        self.twist_covariance[0] = 1e-3
        self.twist_covariance[35] = 1e-3

    def enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        if msg.header.frame_id != self.config.base_frame_id:
            return
        current_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        if self.stamp is None:
            self.stamp = current_time
            return

        dt = current_time - self.stamp
        self.stamp = current_time

        if dt <= 0 or dt > 1.0:
            return

        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z

        mid_heading = self.heading + 0.5 * self.wz * dt  # midpoint method, more accurate than basic Euler's Method
        self.x += self.vx * dt * math.cos(mid_heading)
        self.y += self.vx * dt * math.sin(mid_heading)
        self.heading += self.wz * dt
        self.heading = math.atan2(math.sin(self.heading), math.cos(self.heading))
        self.publish_odom()

    def reset_callback(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.publish_odom()

        res.success = True
        res.message = "Odometry successfully reset to zero."
        self.get_logger().info("Odometry successfully reset to zero.")
        return res

    def publish_odom(self) -> None:
        now = self.get_clock().now().to_msg()
        odom_msg = Odometry(
            header=Header(stamp=now, frame_id=self.config.odom_frame_id),
            child_frame_id=self.config.base_frame_id,
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point2d(self.x, self.y).to_ros(),
                    orientation=Rotation2d(self.heading).to_ros(),
                ),
                covariance=self.pose_covariance,
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=self.vx, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=self.wz),
                ),
                covariance=self.twist_covariance,
            ),
        )

        self.tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(stamp=now, frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                transform=Transform(
                    translation=Vector3(x=self.x, y=self.y, z=0.0), rotation=Rotation2d(self.heading).to_ros()
                ),
            )
        )
        self.get_logger().info("Publishing odometry message.")
        self.odom_publisher.publish(odom_msg)


def main() -> None:
    utils.lifecycle.run_node(EncOdomPublisher)
