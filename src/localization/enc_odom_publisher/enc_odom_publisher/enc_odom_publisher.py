import numpy as np
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
from rclpy.time import Time
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")

        self.config: EncOdomPublisherConfig = utils.config.load(self, EncOdomPublisherConfig)

        self.create_subscription(TwistWithCovarianceStamped, "enc_vel", self.publish_enc_pos, 10)

        self.publisher = self.create_publisher(Odometry, "odom", 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.reset_service = self.create_service(Trigger, "reset", self.reset)

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0
        self.curr_time: Time | None = None

    def publish_enc_pos(self, msg: TwistWithCovarianceStamped) -> None:
        # self.get_logger().info(f'Message received: {msg}')

        if msg.header.frame_id != self.config.base_frame_id:
            self.get_logger().info(
                f"Frame IDs don't match:\n msg frame_id: {msg.header.frame_id}\n enc base_frame_id: {self.config.base_frame_id}"
            )
            return

        msg_time = Time.from_msg(msg.header.stamp)
        if self.curr_time is None:
            self.curr_time = msg_time
            return
        time_passed = msg_time - self.curr_time
        dt = time_passed.nanoseconds / 1e9

        self.curr_time = msg_time

        self.get_logger().info(f"Calculated timestep (seconds): {dt}")

        # only proceed if timestep is valid
        if dt <= 0 or dt > 1:
            self.get_logger().info("Invalid timestep! (dt is <= 0 or > 1e9)")
            return

        self.calc_global_position(msg.twist.twist.angular, msg.twist.twist.linear, dt)

        self.get_logger().info(f"Global position calculated: x - {self.x}, y - {self.y}, heading - {self.heading}")

        self.transform()
        self.publish(msg)

    def calc_global_position(self, wz: Vector3, vx: Vector3, dt: float) -> None:
        self.get_logger().info(f"Lin. Vel.: {vx}; Ang. Vel.: {wz}; dt: {dt}")
        mid_heading = self.heading + 0.5 * wz.z * dt  # midpoint method, more accurate than basic Euler's Method
        self.x += vx.x * dt * np.cos(mid_heading)
        self.y += vx.x * dt * np.sin(mid_heading)
        self.heading += wz.z * dt
        self.heading = np.atan2(np.sin(self.heading), np.cos(self.heading))

    def reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.x = 0
        self.y = 0
        self.z = 0
        self.heading = 0
        self.transform()
        self.publish()
        response.success = True
        response.message = "Odometry reset"
        return response

    def transform(self) -> None:
        if self.curr_time is None:
            return
        self.tf_broadcaster.sendTransform(
            TransformStamped(
                header=Header(stamp=self.curr_time.to_msg(), frame_id=self.config.odom_frame_id),
                child_frame_id=self.config.base_frame_id,
                transform=Transform(
                    translation=Vector3(x=self.x, y=self.y, z=0.0),
                    rotation=Quaternion(x=0.0, y=0.0, z=np.sin(self.heading / 2.0), w=np.cos(self.heading / 2.0)),
                ),
            )
        )

    def publish(self, msg: TwistWithCovarianceStamped | None = None) -> None:
        if self.curr_time is None:
            return
        self.publisher.publish(
            Odometry(
                header=Header(
                    stamp=self.curr_time.to_msg(), frame_id=self.config.odom_frame_id
                ),  # check this is correct frame
                child_frame_id=self.config.base_frame_id,  # check this is correct frame
                pose=PoseWithCovariance(
                    pose=Pose(
                        position=Point(x=self.x, y=self.y, z=0.0),
                        orientation=Quaternion(
                            x=0.0, y=0.0, z=np.sin(self.heading / 2.0), w=np.cos(self.heading / 2.0)
                        ),
                    ),
                    covariance=[0.0] * 36,
                ),
                twist=msg.twist
                if msg is not None
                else TwistWithCovariance(
                    twist=Twist(
                        linear=Vector3(x=0.0, y=0.0, z=0.0),
                        angular=Vector3(x=0.0, y=0.0, z=0.0),
                    ),
                    covariance=[0.0] * 36,
                ),
            )
        )


def main() -> None:
    utils.lifecycle.run_node(EncOdomPublisher)
