import utils.config
import utils.lifecycle
from geometry_msgs.msg import (
    Pose,
    PoseWithCovariance,
    Transform,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
    Vector3,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros.transform_broadcaster import TransformBroadcaster, TransformStamped
from utils.geometry import Point2d, Rotation2d

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")
        self.config = utils.config.load(self, EncOdomPublisherConfig)
        self.subscriber = self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self._enc_vel_callback, 10)

        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.create_service(Trigger, "_reset_all", self._reset_all)

        self.create_timer(self.config.publish_period_s, self._publish_odom)
        self.x: float = 0.0
        self.y: float = 0.0
        self.previous_time: Time | None = None  # assigned the object type when let the value = none
        self.pose_covariance = [0.0 for _ in range(36)]
        self.pose_covariance[0] = 0.01
        self.pose_covariance[7] = 0.01
        self.pose_covariance[35] = 0.01
        self.header: Header | None = None
        self.twist: TwistWithCovariance | None = None
        self.heading = Rotation2d(0.0)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("constructor")

    def _reset_all(self, req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        self.x = 0
        self.y = 0
        self.heading = Rotation2d(0)
        self.previous_time = None
        return res

    def _publish_odom(self) -> None:
        if self.header is not None and self.twist is not None and self.heading is not None:
            self.get_logger().info("if statement for header")
            self.tf_broadcaster.sendTransform(
                TransformStamped(
                    header=Header(stamp=self.header.stamp, frame_id=self.config.odom_frame_id),
                    child_frame_id=self.config.base_frame_id,
                    transform=Transform(translation=Vector3(x=self.x, y=self.y, z=0.0), rotation=self.heading.to_ros()),
                )
            )

            self.odom_msg = Odometry(
                header=self.header,
                child_frame_id=self.config.base_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point2d(x=self.x, y=self.y).to_ros(),
                        orientation=self.heading.to_ros(),
                    ),
                    covariance=self.pose_covariance,
                ),
                twist=self.twist,
            )

            self._odom_pub.publish(self.odom_msg)

    def _enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        self.header = msg.header
        self.twist = msg.twist

        if self.previous_time is None:
            self.previous_time = Time.from_msg(msg.header.stamp)
            self.get_logger().info("if statement for previous time called")
            return

        time = Time.from_msg(msg.header.stamp)
        dt = (time.nanoseconds - self.previous_time.nanoseconds) * 1e-9  # convert to seconds
        # previous_time be a Time obj enable us to use .nanoseconds() (LOOK AT COMMENT AT LINE 36)
        if dt <= 0 or dt > 1.0:
            self.previous_time = time
            self.get_logger().info(f"if statement for dt called {dt}")
            return
        mid_heading = self.heading + Rotation2d(
            0.5 * wz * dt
        )  # midpoint method, more accurate than basic Euler's Method
        self.x += vx * dt * mid_heading.cos
        self.y += vx * dt * mid_heading.sin
        self.heading = self.heading + Rotation2d(wz * dt)

        self.previous_time = time

        # self.get_logger().info(f"useful var here {self.x}, {self.y}, {self.heading}")


def main() -> None:
    utils.lifecycle.run_node(EncOdomPublisher)
