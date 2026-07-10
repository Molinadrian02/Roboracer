from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab1_pkg',
            executable='talker.py',
            name='talker_node',
            output='screen',
            parameters=[{'v': 1.0, 'd': 0.5}]
        ),
        Node(
            package='lab1_pkg',
            executable='relay.py',
            name='relay_node',
            output='screen'
        )
    ])