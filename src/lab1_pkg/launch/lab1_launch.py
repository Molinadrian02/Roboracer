from launch import LaunchDescription 
from launch_ros.actions import Node # LaunchDescription is a class that describes a launch file, and Node is a class that represents a ROS2 node to be launched, its like a instructions set for the node

def generate_launch_description(): # Define a function that generates the launch description for this launch file
    return LaunchDescription([ # return a LaunchDescription object that contains a list of nodes to be launched
        Node(
            package='lab1_pkg', # Name of the package that contains the node to be launched
            executable='talker_c++', # Name of the executable file that contains the node to be launched
            name='talker_node', # Name of the node to be launched, this is the name that will be used to identify the node in the ROS2 system
            output='screen', # Output of the node will be printed to the screen
            parameters=[{'v': 1.0, 'd': 0.5}] # Parameters to be passed to the node, in this case, the parameters are 'v' and 'd' with values 1.0 and 0.5 respectively
        ),
        Node(
            package='lab1_pkg',
            executable='relay_cpp',
            name='relay_node',
            output='screen'
        )
    ])