from dataclasses import dataclass


@dataclass(frozen=True)
class EncOdomPublisherConfig:
    """Config for EncOdomPublisher. See utils.config for supported field types and the YAML parameter mapping.

    Attributes:
        odom_frame_id: Frame ID stamped on ______
        base_frame_id: Frame ID stamped on published ___ messages
    """

    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"

    def __post_init__(self) -> None:
        # Validate fields here, e.g. raise ValueError on out-of-range values.
        pass
