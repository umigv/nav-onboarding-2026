from dataclasses import dataclass


@dataclass(frozen=True)  # frozen = true means you aren't supposed to change these fields later.
class EncOdomPublisherConfig:
    """Config for EncOdomPublisher. See utils.config for supported field types and the YAML parameter mapping.

    Attributes:
        (document each field here)
    """

    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"

    def __post_init__(self) -> None:
        # Validate fields here, e.g. raise ValueError on out-of-range values.
        pass
