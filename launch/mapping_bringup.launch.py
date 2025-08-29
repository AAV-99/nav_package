# mode_orchestrator/launch/mapping_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    slam = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=['install/mode_orchestrator/share/mode_orchestrator/config/slam_toolbox.yaml'],
        output='screen'
    )

    explore = Node(
        package='m_explore_ros2',
        executable='explore',     # verificar nombre exacto en tu fork
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapping',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['slam_toolbox']
        }]
    )

    return LaunchDescription([slam, explore, lifecycle])
