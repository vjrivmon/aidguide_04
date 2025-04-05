import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Obtener la ruta del paquete world
    world_pkg_share = get_package_share_directory('aidguide_04_world')
    
    # Ruta al archivo de lanzamiento del mundo
    world_launch_file = os.path.join(world_pkg_share, 'launch', 'world.launch.py')
    
    # Incluir el lanzamiento del mundo como parte de este lanzamiento
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([world_launch_file]),
    )
    
    # Iniciar todos los nodos de monitorización
    battery_monitor_node = Node(
        package='aidguide_04_robot_monitoring',
        executable='battery_monitor',
        name='battery_monitor',
        output='screen',
    )
    
    hardware_monitor_node = Node(
        package='aidguide_04_robot_monitoring',
        executable='hardware_monitor',
        name='hardware_monitor',
        output='screen',
    )
    
    temperature_monitor_node = Node(
        package='aidguide_04_robot_monitoring',
        executable='temperature_monitor',
        name='temperature_monitor',
        output='screen',
    )
    
    log_monitor_node = Node(
        package='aidguide_04_robot_monitoring',
        executable='log_monitor',
        name='log_monitor',
        output='screen',
    )
    
    dashboard_node = Node(
        package='aidguide_04_robot_monitoring',
        executable='monitoring_dashboard',
        name='monitoring_dashboard',
        output='screen',
    )
    
    return LaunchDescription([
        # Lanzar primero el mundo
        world_launch,
        
        # Lanzar los nodos de monitorización
        battery_monitor_node,
        hardware_monitor_node,
        temperature_monitor_node,
        log_monitor_node,
        dashboard_node,
    ]) 