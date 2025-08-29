# mode_orchestrator/launch/nav_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': '/path/a/tu_mapa.yaml'}],
        output='screen'
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen'
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server'
    )
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server'
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator'
    )
    recoveries = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server'
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'bt_navigator',
                'recoveries_server'
            ]
        }]
    )

    return LaunchDescription([
        map_server, amcl, controller, planner, bt_navigator, recoveries, lifecycle
    ])
