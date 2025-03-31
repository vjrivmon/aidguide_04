from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aidguide_04_nav',
            executable='aidguide_navigation',
            output='screen'),
    ])