from dataclasses import dataclass


@dataclass(frozen=True)
class EncOdomPublisherConfig:
    """Configuration for EncOdomPublisher.

    Attributes:
        odom_frame_id: Frame in which the estimated pose is reported.
        base_frame_id: Robot frame stamped on incoming encoder messages.
    """

    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"
