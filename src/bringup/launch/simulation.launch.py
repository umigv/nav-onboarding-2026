from bringup.launch_utils import load_frames
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    frames = load_frames()

    return LaunchDescription(
        [
            Node(
                package="enc_vel_mock_publisher",
                executable="enc_vel_mock_publisher",
                name="enc_vel_mock_publisher",
                parameters=[
                    {"base_frame_id": frames["base_frame"]},
                ],
                remappings=[
                    ("enc_vel", "enc_vel/raw"),
                ],
            ),
        ]
    )
