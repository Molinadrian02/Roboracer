from launch import LaunchDescription 
from launch_ros.actions import Node # LaunchDescription is a class that describes a launch file, and Node is a class that represents a ROS2 node to be launched, its like a instructions set for the node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description(): # Define a function that generates the launch description for this launch file
    config_file_path = os.path.join(get_package_share_directory('lab3_pkg'), 'config', 'wall_follower.yaml') # Get the path to the configuration file for the wall follower node
    return LaunchDescription([ # return a LaunchDescription object that contains a list of nodes to be launched
        Node(
            package='lab3_pkg', # Name of the package that contains the node to be launched
            executable='wall_follower.py', # Name of the executable file that contains the node to be launched
            name='wall_follower', # Name of the node to be launched, this is the name that will be used to identify the node in the ROS2 system
            output='screen', # Output of the node will be printed to the screen
            parameters=[config_file_path] # Parameters to be passed to the node, in this case, the parameters are defined in a YAML file
        )
    ])
