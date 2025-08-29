from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nav_package",
            executable="orchestrator_node",
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
            executable="mqtt_bridge",
            name="mqtt_bridge",
            output="screen",
            parameters=["config/mqtt_topics.yaml"]
        )
    ])
