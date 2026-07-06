from ament_index_python.packages import get_package_share_directory
from bringup.launch_utils import load_frames
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    frames = load_frames()
    urdf = f"{get_package_share_directory('maverick_description')}/urdf/maverick.xacro"
    robot_description = ParameterValue(
        Command(["xacro ", urdf, " base_frame_id:=", frames["base_frame"]]), value_type=str
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
            ),
        ]
    )
