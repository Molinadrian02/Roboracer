#!/usr/bin/env python3

# Import ROS2 dependencies and the AckermannDriveStamped message, which is required for this lab
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

# Create a publisher node class that inherits from Node, the base class for all ROS2 nodes
class PublisherNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)  # Create a publisher that publishes AckermannDriveStamped messages on the 'drive' topic with a queue size of 10
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)  # This calls the timer_callback function every 0.5 seconds
        self.declare_parameter('v', 0.0)  # Declare the 'v' and 'd' parameters, which can be set from the launch file
        self.declare_parameter('d', 0.0)

    def timer_callback(self):
        msg = AckermannDriveStamped()
        v = self.get_parameter('v').get_parameter_value().double_value  # Store the 'v' and 'd' values in local Python variables, retrieved from the ROS2 environment
        d = self.get_parameter('d').get_parameter_value().double_value  # These can be modified from the launch file or from the command line when running the node
        # Set the values for the AckermannDriveStamped message
        msg.drive.speed = v  # Set msg.drive.speed to the value of the v variable
        msg.drive.steering_angle = d
        self.publisher_.publish(msg)  # Publish the message on the 'drive' topic with the speed and steering angle values set by the 'v' and 'd' parameters
        self.get_logger().info('Publishing: speed="%s", steering_angle="%s"' % (msg.drive.speed, msg.drive.steering_angle))  # Debug and print the current values of this topic's message



def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 communication system

    talker = PublisherNode()  # Declare the talker variable, an instance of the PublisherNode class — it could be named anything else, it's just a variable name

    rclpy.spin(talker)  # Keeps the node running until an interrupt signal (Ctrl+C) is received or the node is shut down elsewhere

    talker.destroy_node()  # Destroy the talker node to release its associated resources
    rclpy.shutdown()

if __name__ == '__main__':
    main()