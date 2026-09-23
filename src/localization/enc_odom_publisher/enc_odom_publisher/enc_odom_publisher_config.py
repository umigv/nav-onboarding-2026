from dataclasses import dataclass


@dataclass(frozen=True)
class EncOdomPublisherConfig:
    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"

    def __post_init__(self) -> None:
        # raise ValueError on invalid combinations
        pass
