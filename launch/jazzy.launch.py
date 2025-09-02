from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable

def generate_launch_description():
    package_name = 'nav_package'  
    
    # Ruta relativa por defecto para el mapa
    default_map_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'maps',
        'warehouse.yaml'
    ])
    
    # Argumentos con valores por defecto
    declare_world = DeclareLaunchArgument(
        'world', 
        default_value='warehouse',
        description='Nombre del mundo de Gazebo a cargar')
    
    declare_map = DeclareLaunchArgument(
        'map', 
        default_value=default_map_file,
        description='Ruta al archivo YAML del mapa')
    
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Lanzar RViz2')
    
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Habilitar SLAM')
    
    declare_nav2 = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Habilitar Nav2')
    
    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='true',
        description='Habilitar localización')
    
    # Paths para turtlebot4
    pkg_tb4_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    turtlebot4_gz_launch = PathJoinSubstitution([
        pkg_tb4_bringup, 'launch', 'turtlebot4_gz.launch.py'
    ])
    
    # Lanzar la simulación completa del TurtleBot4
    launch_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_gz_launch]),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'slam': LaunchConfiguration('slam'),
            'nav2': LaunchConfiguration('nav2'),
            'localization': LaunchConfiguration('localization'),
            'map': LaunchConfiguration('map'),
            'world': LaunchConfiguration('world'),
        }.items()
    )
    
    return LaunchDescription([

        #Definir variables de entorno para renderizado con GPU NVIDIA
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '0'),
        SetEnvironmentVariable('GZ_SIM_RENDER_ENGINE', 'ogre2'),
        
        # Declarar todos los argumentos
        declare_world,
        declare_map,
        declare_rviz,
        declare_slam,
        declare_nav2,
        declare_localization,
        
        # Lanzar la simulación
        launch_sim
    ])