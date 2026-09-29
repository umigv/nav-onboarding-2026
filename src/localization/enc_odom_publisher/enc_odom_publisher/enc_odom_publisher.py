import utils.config
import utils.lifecycle
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from utils.geometry import Point2d, Pose2d, Rotation2d

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")

        self.config: EncOdomPublisherConfig = utils.config.load(self, EncOdomPublisherConfig)

        self.pose = Pose2d(
            point=Point2d(x=0.0, y=0.0),
            rotation=Rotation2d(0.0),
        )
        self.last_time_ns: int | None = None

        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.create_subscription(
            TwistWithCovarianceStamped,
            "enc_vel",
            self.enc_vel_callback,
            10,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.reset_service = self.create_service(
            Trigger,
            "reset_odom",
            self.reset_callback,
        )

    def enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        if msg.header.frame_id != self.config.base_frame_id:
            return

        current_time_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        if self.last_time_ns is None:
            self.last_time_ns = current_time_ns
            return

        dt = (current_time_ns - self.last_time_ns) / 1_000_000_000.0
        self.last_time_ns = current_time_ns

        if dt <= 0.0 or dt > 1.0:
            return

        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        mid_heading = self.pose.rotation.angle + 0.5 * wz * dt
        distance = vx * dt

        self.pose = Pose2d(
            point=self.pose.point
            + Point2d(
                x=distance * Rotation2d(mid_heading).cos,
                y=distance * Rotation2d(mid_heading).sin,
            ),
            rotation=Rotation2d(self.pose.rotation.angle + wz * dt),
        )

        publish_stamp = self.get_clock().now().to_msg()
        self.publish_odometry(publish_stamp, vx, wz)
        self.broadcast_transform(publish_stamp)

    def publish_odometry(self, stamp: Time, vx: float, wz: float) -> None:
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.config.odom_frame_id
        odom_msg.child_frame_id = self.config.base_frame_id

        odom_msg.pose.pose = self.pose.to_ros()

        pose_covariance = [0.0] * 36
        pose_covariance[0] = 0.001
        pose_covariance[7] = 0.001
        pose_covariance[35] = 0.001
        odom_msg.pose.covariance = pose_covariance

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = wz
        self.odom_publisher.publish(odom_msg)

    def broadcast_transform(self, stamp: Time) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.config.odom_frame_id
        transform.child_frame_id = self.config.base_frame_id

        transform.transform.translation.x = self.pose.point.x
        transform.transform.translation.y = self.pose.point.y
        transform.transform.rotation = self.pose.rotation.to_ros()

        self.tf_broadcaster.sendTransform(transform)

    def reset_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self.pose = Pose2d(
            point=Point2d(x=0.0, y=0.0),
            rotation=Rotation2d(0.0),
        )
        self.last_time_ns = None

        response.success = True
        response.message = "Odometry reset"

        self.broadcast_transform(self.get_clock().now().to_msg())
        stamp = self.get_clock().now().to_msg()
        self.publish_odometry(stamp, 0.0, 0.0)
        self.broadcast_transform(stamp)
        return response


def main() -> None:
    utils.lifecycle.run_node(EncOdomPublisher)
