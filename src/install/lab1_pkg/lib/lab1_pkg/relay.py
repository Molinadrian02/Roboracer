#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('relay')
        self.subscription_ = self.create_subscription(
            AckermannDriveStamped,
            'drive',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def listener_callback(self, msg):
        relay_msg = AckermannDriveStamped()
        relay_msg.drive.speed = msg.drive.speed*3
        relay_msg.drive.steering_angle = msg.drive.steering_angle*3
        self.publisher_.publish(relay_msg)

        


def main(args=None):
    rclpy.init(args=args)

    relay = SubscriberNode()

    rclpy.spin(relay)

    relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()