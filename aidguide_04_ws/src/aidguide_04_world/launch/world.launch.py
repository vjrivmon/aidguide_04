import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# Definir el modelo de TurtleBot3
TURTLEBOT3_MODEL = os.getenv('TURTLEBOT3_MODEL', 'burger_pi')

def generate_launch_description():
    # Obtener la ruta del paquete `world`
    pkg_share = FindPackageShare(package='aidguide_04_world').find('world')

    # Asegurar que se carga `burger_pi.model`
    world_file_name = 'world/burger_pi.model'  
    urdf_file_name = 'urdf/turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    world = os.path.join(pkg_share, world_file_name)
    urdf = os.path.join(pkg_share, urdf_file_name)

    # Definir la ruta a los archivos de lanzamiento de Gazebo
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        # Lanzar Gazebo con el mundo correcto
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        # Lanzar interfaz gráfica de Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # Publicador de estado del robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time', default='true')}.items(),
        ),
    ])
