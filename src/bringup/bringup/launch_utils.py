from pathlib import Path
from typing import cast

import yaml
from ament_index_python.packages import get_package_share_directory


def bringup_share() -> str:
    return str(get_package_share_directory("bringup"))


def load_frames() -> dict:
    with Path(f"{bringup_share()}/config/frames.yaml").open() as f:
        return cast(dict, yaml.safe_load(f))
