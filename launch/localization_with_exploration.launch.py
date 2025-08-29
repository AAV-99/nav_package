from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

def generate_launch_description():
    # Argumentos
    declare_world = DeclareLaunchArgument(
        'world', default_value='warehouse',
        description='Nombre del mundo de Gazebo a cargar')

    declare_map = DeclareLaunchArgument(
        'map', default_value='/home/aav/ros2_ws/src/maps/warehouse.yaml',
        description='Ruta al archivo YAML del mapa')

    # Paths
    pkg_tb4_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    turtlebot4_gz_launch = PathJoinSubstitution([
        pkg_tb4_bringup, 'launch', 'turtlebot4_gz.launch.py'])

    # Lanzar Gazebo + spawn del robot + todo el sistema (ros_gz_bridge, robot, nav2, etc.)
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_gz_launch]),
        launch_arguments={
            'rviz': 'true',
            'slam': 'true',
            'nav2': 'true',
            'localization': 'false',
            'world': LaunchConfiguration('world'),
        }.items()
    )

    # Nodo para lanzar m-explore-ros2
    explore_node = Node(
        package='m_explore',
        executable='explore_node',
        name='explore',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'planner_frequency': 1.0,
            'visualize': True,
        }]
    )

    # Guardar el mapa automáticamente después de 300 segundos
    save_map = TimerAction(
        period=300.0,  # tiempo en segundos (5 minutos)
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                    '-f', '/home/aav/ros2_ws/src/maps/auto_saved_map'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_world,
        declare_map,
        launch_sim,
        explore_node,
        save_map
    ])
