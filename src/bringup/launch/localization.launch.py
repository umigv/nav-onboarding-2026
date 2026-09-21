from bringup.launch_utils import load_frames
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    frames = load_frames()

    return LaunchDescription(
        [
            Node(
                package="enc_odom_publisher",
                executable="enc_odom_publisher",  # This is the compiled program ROS starts
                name="enc_odom_publisher",  # The node's ROS name while it is running
                output="screen",
                parameters=[
                    {"odom_frame_id": frames["odom_frame"]},
                    {"base_frame_id": frames["base_frame"]},
                ],
                # Don't have to modify every consumer when you add filtering to enc_vel, this node remain unaffected
                remappings=[("enc_vel", "enc_vel/raw")],  # Redirect its enc_vel input to the actual enc_vel/raw topic
            ),
        ]
    )
