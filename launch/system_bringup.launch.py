from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nav_package",
            executable="orchestrator",
            name="orchestrator",
            output="screen"
        ),
        Node(
            package="nav_package",
            executable="docking_node",
            name="docking_node",
            output="screen"
        ),
        Node(
            package="nav_package",
            executable="nav",
            name="nav",
            output="screen"
        )
    ])
