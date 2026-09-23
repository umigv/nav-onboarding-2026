from math import cos, sin

import utils.config
import utils.lifecycle
from geometry_msgs.msg import Transform, TwistWithCovarianceStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Header, TransformBroadcaster, TransformStamped
from utils.geometry import Rotation2d

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")

        self.config: EncOdomPublisherConfig = utils.config.load(self, EncOdomPublisherConfig)
        self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)

        self.last_known_time: Time | None = None

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0

        self.vx = 0.0
        self.wz = 0.0

        self.reset_service = self.create_service(Trigger, "reset", self.reset_callback)
        self.tf_broadcaster = TransformBroadcaster(self)

    def enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        if not self.last_known_time:
            self.last_known_time = Time.from_msg(msg.header.stamp)
            return

        dt = (Time.from_msg(msg.header.stamp) - self.last_known_time).nanoseconds * 1e-9
        self.last_known_time = Time.from_msg(msg.header.stamp)

        if (dt <= 0.0) or (dt > 1.0):
            return

        self.vx = msg.twist.twist.linear.x
        self.wz = msg.twist.twist.angular.z

        mid_heading = self.heading + 0.5 * self.wz * dt  # midpoint method, more accurate than basic Euler's Method
        self.pos_x += self.vx * cos(mid_heading) * dt
        self.pos_y += self.vx * sin(mid_heading) * dt
        self.heading = mid_heading + 0.5 * self.wz * dt

        self.publish_odom()

    def publish_odom(self) -> None:
        odom_msg = Odometry()
        if not self.last_known_time:
            return
        odom_msg.header.stamp = self.last_known_time.to_msg()
        odom_msg.header.frame_id = self.config.odom_frame_id
        odom_msg.child_frame_id = self.config.base_frame_id

        odom_msg.pose.pose.position.x = self.pos_x
        odom_msg.pose.pose.position.y = self.pos_y
        odom_msg.pose.pose.orientation = Rotation2d(self.heading).to_ros()

        odom_msg.twist.twist.linear.y = 0
        odom_msg.twist.twist.linear.z = 0
        odom_msg.twist.twist.angular.x = 0
        odom_msg.twist.twist.angular.y = 0
        odom_msg.twist.twist.angular.z = 0

        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0] = 0.01
        odom_msg.twist.covariance[7] = 0.01
        odom_msg.twist.covariance[35] = 0.01

        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0] = 0.01
        odom_msg.pose.covariance[7] = 0.01
        odom_msg.pose.covariance[35] = 0.01

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.angular.z = self.wz

        time = self.last_known_time.to_msg()

        self.tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(stamp=time, frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                transform=Transform(
                    translation=Vector3(x=self.pos_x, y=self.pos_y, z=0.0), rotation=Rotation2d(self.heading).to_ros()
                ),
            )
        )

        self.odom_publisher.publish(odom_msg)

    def reset_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.heading = 0.0

        self.vx = 0.0
        self.wz = 0.0
        self.last_known_time = None

        time = self.get_clock().now().to_msg()

        self.tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(stamp=time, frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                transform=Transform(
                    translation=Vector3(x=self.pos_x, y=self.pos_y, z=0.0), rotation=Rotation2d(self.heading).to_ros()
                ),
            )
        )

        response.success = True
        response.message = "True"

        return response


def main() -> None:
    utils.lifecycle.run_node(EncOdomPublisher)
