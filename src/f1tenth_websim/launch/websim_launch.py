import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('f1tenth_websim')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'websim.yaml'),
                              description='Fichero de configuración de websim'),
        DeclareLaunchArgument('port', default_value='0', description='Puerto de la interfaz web (0 = el de websim.yaml)'),
        DeclareLaunchArgument('map', default_value='', description='Mapa inicial (vacío = el de websim.yaml)'),
        Node(
            package='f1tenth_websim',
            executable='websim',
            name='websim',
            output='screen',
            emulate_tty=True,
            # '--opt=' + value as one argument, so an empty map name never breaks argparse
            arguments=[['--config=', LaunchConfiguration('config')],
                       ['--port=', LaunchConfiguration('port')],
                       ['--map=', LaunchConfiguration('map')]],
        ),
    ])
