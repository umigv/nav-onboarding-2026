import math

import utils.config
import utils.lifecycle
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
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
from tf2_ros import TransformBroadcaster

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")
        self.config = utils.config.load(self, EncOdomPublisherConfig)
        self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.srv_reset = self.create_service(Trigger, "reset", self.reset_callback)

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.pose_covariance = [0.0] * 36
        self.pose_covariance[0] = 0.001
        self.pose_covariance[7] = 0.001
        self.pose_covariance[35] = 0.001
        self.last_time: float | None = None

    def reset_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.last_time = None

        self.broadcast_transform()

        response.success = True
        response.message = "Reset successful"
        return response

    def enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if msg.header.frame_id != self.config.base_frame_id:
            return

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time

        if dt <= 0.0 or dt > 1.0:
            self.last_time = current_time
            return

        self.last_time = current_time

        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        mid_heading = self.heading + wz * dt * 0.5
        self.x += vx * dt * math.cos(mid_heading)
        self.y += vx * dt * math.sin(mid_heading)

        self.heading += wz * dt

        self.broadcast_transform()
        self.odom_publisher.publish(
            Odometry(
                header=Header(stamp=msg.header.stamp, frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=self.x, y=self.y, z=0.0),
                        orientation=Quaternion(
                            x=-0, y=0.0, z=math.sin(self.heading * 0.5), w=math.cos(self.heading * 0.5)
                        ),
                    ),
                    covariance=self.pose_covariance,
                ),
                twist=TwistWithCovariance(
                    twist=Twist(linear=Vector3(x=vx, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=wz)),
                    covariance=msg.twist.covariance,
                ),
            )
        )

    def broadcast_transform(self) -> None:
        self.tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                transform=Transform(
                    translation=Vector3(x=self.x, y=self.y, z=0.0),
                    rotation=Quaternion(x=-0, y=0.0, z=math.sin(self.heading * 0.5), w=math.cos(self.heading * 0.5)),
                ),
            )
        )


def main() -> None:
    utils.lifecycle.run_node(EncOdomPublisher)
