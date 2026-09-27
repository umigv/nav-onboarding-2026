import math
import random
from dataclasses import dataclass

import utils.config
import utils.lifecycle
from geometry_msgs.msg import Twist, TwistWithCovariance, TwistWithCovarianceStamped, Vector3
from rclpy.node import Node
from std_msgs.msg import Header

from .enc_vel_mock_publisher_config import EncVelMockPublisherConfig


@dataclass(frozen=True)
class Phase:
    duration_s: float
    linear_vel_mps: float
    angular_vel_radps: float


class EncVelMockPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_vel_mock_publisher")

        self.config: EncVelMockPublisherConfig = utils.config.load(self, EncVelMockPublisherConfig)

        self.enc_vel_publisher = self.create_publisher(TwistWithCovarianceStamped, "enc_vel", 10)

        self.phases: list[Phase] = self.build_phases()
        self.phase_index = 0
        self.phase_elapsed_s = 0.0

        self.create_timer(self.config.publish_period_s, self.publish_enc_vel)

    def build_phases(self) -> list[Phase]:
        straight = Phase(
            duration_s=self.config.side_length_m / self.config.linear_speed_mps,
            linear_vel_mps=self.config.linear_speed_mps,
            angular_vel_radps=0.0,
        )
        turn = Phase(
            duration_s=(math.pi / 2.0) / self.config.angular_speed_radps,
            linear_vel_mps=0.0,
            angular_vel_radps=self.config.angular_speed_radps,
        )
        return [straight, turn]

    def publish_enc_vel(self) -> None:
        phase = self.phases[self.phase_index]

        self.phase_elapsed_s += self.config.publish_period_s
        if self.phase_elapsed_s >= phase.duration_s:
            self.phase_elapsed_s = 0.0
            self.phase_index = (self.phase_index + 1) % len(self.phases)

        vx_noise = random.gauss(0.0, self.config.vx_noise_std_mps)
        wz_noise = random.gauss(0.0, self.config.wz_noise_std_radps)
        vx = phase.linear_vel_mps + self.config.vx_drift_bias_mps + vx_noise
        wz = phase.angular_vel_radps + self.config.wz_drift_bias_radps + wz_noise

        # Row-major 6x6 covariance [x, y, z, roll, pitch, yaw]
        covariance = [0.0] * 36
        covariance[0] = self.config.vx_noise_std_mps**2
        covariance[35] = self.config.wz_noise_std_radps**2

        self.enc_vel_publisher.publish(
            TwistWithCovarianceStamped(
                header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.config.base_frame_id),
                twist=TwistWithCovariance(
                    twist=Twist(linear=Vector3(x=vx), angular=Vector3(z=wz)),
                    covariance=covariance,
                ),
            )
        )


def main() -> None:
    utils.lifecycle.run_node(EncVelMockPublisher)
