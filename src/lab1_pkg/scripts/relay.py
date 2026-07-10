#!/usr/bin/env python3

# Import ROS2 dependencies and the AckermannDriveStamped message, which is required for this lab
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

# Create a subscriber node class that inherits from Node, the base class for all ROS2 nodes
class SubscriberNode(Node):
    def __init__(self):
        super().__init__('relay') # Initialize the node with the name 'relay'
        self.subscription_ = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10
        ) # Create a subscription to the 'drive' topic, which will call the listener_callback function whenever a new message is received. The queue size is set to 10.
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10) # Creates a publisher that publishes AckermannDriveStamped messages on the 'drive_relay' topic with a queue size of 10

    def listener_callback(self, msg):
        relay_msg = AckermannDriveStamped() # Create a new AckermannDriveStamped message to relay the received message.0
        relay_msg.drive.speed = msg.drive.speed*3 # Set the speed of the relay message to be 3 times the speed of the received message
        relay_msg.drive.steering_angle = msg.drive.steering_angle*3 # Set the steering angle of the relay message to be 3 times the steering angle of the received message
        self.publisher_.publish(relay_msg) # Publish the relay message on the 'drive_relay' topic
        self.get_logger().info('Relaying: speed="%s", steering_angle="%s"' % (relay_msg.drive.speed, relay_msg.drive.steering_angle)) # Debug and print the current values of this topic's message


def main(args=None):
    rclpy.init(args=args) # Initialize the ROS2 communication system

    relay = SubscriberNode() # Declare the relay variable, an instance of the SubscriberNode class — it could be named anything else, it's just a variable name

    rclpy.spin(relay) # Keeps the node running until an interrupt signal (Ctrl+C) is received or the node is shut down elsewhere

    relay.destroy_node() # Destroy the relay node to release its associated resources
    rclpy.shutdown() # Shutdown the ROS2 communication system

if __name__ == '__main__':
    main()