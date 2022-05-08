from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    param_file = os.path.join(get_package_share_directory("f110_wall_follow"), "param", "wall_follow.yaml")
    return LaunchDescription(
        [
            Node(
                package="f110_wall_follow",
                executable="f110_wall_follow_node_exe",
                name="f110_wall_follow_node",
                output="screen",
                parameters=[param_file],
                remappings=[
                    ("scan", "/scan"),
                    ("nav", "/nav")
                ],
            ),
        ]
    )