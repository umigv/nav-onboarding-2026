import math as m

import utils.config
import utils.lifecycle
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
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
from tf2_ros import TransformBroadcaster

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.prev_time: Time | None = None
        self.pose_covariance = [0.0] * 36
        self.pose_covariance[0] = 0.001
        self.pose_covariance[7] = 0.001
        self.pose_covariance[35] = 0.001

        self.config = utils.config.load(self, EncOdomPublisherConfig)

        self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.enc_vel_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.reset_service = self.create_service(
            Trigger,
            "reset_odometry",
            self.reset_callback,
        )

    def enc_vel_callback(self, msg: TwistWithCovarianceStamped) -> None:
        if msg.header.frame_id != self.config.base_frame_id:
            return

        cur_time = Time.from_msg(msg.header.stamp)
        if self.prev_time is None:
            self.prev_time = cur_time
            return

        dt = (cur_time - self.prev_time).nanoseconds / 1e9

        self.prev_time = cur_time

        if dt <= 0 or dt > 1.0:
            return

        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z

        mid_heading = self.heading + 0.5 * wz * dt

        self.x += vx * dt * m.cos(mid_heading)
        self.y += vx * dt * m.sin(mid_heading)
        self.heading += wz * dt

        now = msg.header.stamp
        q = Quaternion(  # Tells ROS which way the robot is facing
            x=0.0,
            y=0.0,
            z=m.sin(self.heading / 2.0),  # Rotation is around z-axis
            w=m.cos(self.heading / 2.0),
        )
        odom = Odometry(
            header=Header(stamp=now, frame_id=self.config.odom_frame_id),
            child_frame_id=self.config.base_frame_id,
            pose=PoseWithCovariance(
                pose=Pose(  # Position + Orientation
                    position=Point(
                        x=self.x,
                        y=self.y,
                        z=0.0,  # calculaton is 2D but we need 3 dimensions for ROS
                    ),
                    orientation=q,
                ),
                covariance=self.pose_covariance,  # Represents how confident you are in those estimates.
            ),
            twist=TwistWithCovariance(
                twist=Twist(  # Robot's current velocity
                    linear=Vector3(
                        x=vx,
                        y=0.0,
                        z=0.0,
                    ),
                    angular=Vector3(
                        x=0.0,
                        y=0.0,
                        z=wz,
                    ),
                ),
            ),
        )

        self.odom_publisher.publish(odom)
        self.publish_transform(now)

    def reset_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        # x = 0
        # y = 0
        # heading = 0
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.prev_time = None
        now = self.get_clock().now().to_msg()
        # immediately update TF
        self.publish_transform(now)

        # fill response
        response.success = True
        response.message = "Odometry reset"
        return response

    def publish_transform(self, now: TimeMsg) -> None:
        # construct TransformStamped
        transform = TransformStamped()

        transform.header.stamp = now
        transform.header.frame_id = self.config.odom_frame_id
        transform.child_frame_id = self.config.base_frame_id

        # Translation: robot's x/y position
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # Rotation: heading (yaw) -> quaternion
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = m.sin(self.heading / 2.0)
        transform.transform.rotation.w = m.cos(self.heading / 2.0)

        # send transform
        self.tf_broadcaster.sendTransform(transform)


def main() -> None:
    utils.lifecycle.run_node(EncOdomPublisher)
