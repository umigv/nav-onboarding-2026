from dataclasses import dataclass


@dataclass(frozen=True)
class EncVelMockPublisherConfig:
    """Config for EncVelMockPublisher.

    Attributes:
        side_length_m: Length of each side of the square (m).
        linear_speed_mps: Forward speed while driving a side (m/s).
        angular_speed_radps: Turning speed while executing a corner turn (rad/s).
        publish_period_s: Period (s) at which enc_vel is published.
        base_frame_id: Frame ID stamped on published enc_vel messages.
        vx_drift_bias_mps: Constant bias added to linear velocity (m/s), simulating wheel slip/miscalibration.
        wz_drift_bias_radps: Constant bias added to angular velocity (rad/s).
        vx_noise_std_mps: Standard deviation of gaussian noise added to linear velocity (m/s).
        wz_noise_std_radps: Standard deviation of gaussian noise added to angular velocity (rad/s).
    """

    side_length_m: float = 2.0
    linear_speed_mps: float = 1.0
    angular_speed_radps: float = 0.5
    publish_period_s: float = 0.01
    base_frame_id: str = "base_link"

    vx_drift_bias_mps: float = 0.02
    wz_drift_bias_radps: float = -0.01
    vx_noise_std_mps: float = 0.01
    wz_noise_std_radps: float = 0.01

    def __post_init__(self) -> None:
        if self.side_length_m <= 0:
            raise ValueError("EncVelMockPublisherConfig: side_length_m must be > 0")
        if self.linear_speed_mps <= 0:
            raise ValueError("EncVelMockPublisherConfig: linear_speed_mps must be > 0")
        if self.angular_speed_radps <= 0:
            raise ValueError("EncVelMockPublisherConfig: angular_speed_radps must be > 0")
        if self.publish_period_s <= 0:
            raise ValueError("EncVelMockPublisherConfig: publish_period_s must be > 0")
        if self.vx_noise_std_mps < 0:
            raise ValueError("EncVelMockPublisherConfig: vx_noise_std_mps must be >= 0")
        if self.wz_noise_std_radps < 0:
            raise ValueError("EncVelMockPublisherConfig: wz_noise_std_radps must be >= 0")
