import utils.config
import utils.lifecycle
from rclpy.node import Node

from .enc_odom_publisher_config import EncOdomPublisherConfig


class EncOdomPublisher(Node):
    def __init__(self) -> None:
        super().__init__("enc_odom_publisher")

        self.config = utils.config.load(self, EncOdomPublisherConfig)


def main() -> None:
    utils.lifecycle.run_node(EncOdomPublisher)
