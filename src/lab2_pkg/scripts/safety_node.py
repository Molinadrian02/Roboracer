#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from math import cos, pi

#In this node we will use msgs of laserscan and odometry
#   Laserscan:
#   - angle_min: start angle of the scan [rad]
#   - angle_max: end angle of the scan [rad]
#   - angle_increment: angular distance between measurements [rad]
#   - time_increment: time between measurements [seconds] - if your scanner is moving, this will be used in interpolating position of 3d points
#   - scan_time: time between scans [seconds]
#   - range_min: minimum range value [m]
#   - range_max: maximum range value [m]
#   - ranges: range data [m] (Note: values < range_min or > range_max should be discarded)
#   - intensities: intensity data [device-specific units].
#   Odometry:
#   - header: standard metadata for higher-level stamped data types.        
class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        #Structure of the node: self.sub/pub_name = self.create_subscription/Publisher(MessageType, 'topic_name', self.callback_function, queue_size)
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)  # Create a subscriber that listens to LaserScan messages on the 'scan' topic and calls the scan_callback function when a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'ego_racecar/odom', self.odom_callback, 10)  # Create a subscriber that listens to Odometry messages on the 'odom' topic and calls the odom_callback function when a message is received  
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)  # Create a publisher that publishes AckermannDriveStamped messages on the 'drive' topic with a queue size of 10
        self.brake_publisher_ = self.create_publisher(Bool, 'emergency_brake', 10)  # The simulator gives this flag absolute priority over /drive, so another node driving the car cannot undo the braking
        self.current_speed = 0.0
        self.declare_parameter('ttc_threshold', 1.0)   # seconds
        self.declare_parameter('min_distance', 0.5)    # metres, floor for the iTTC criterion
        self.declare_parameter('front_angle', 0.35)    # radians, half-width of the cone the floor applies to
    def scan_callback(self, msg):
        ttc_threshold = self.get_parameter('ttc_threshold').value
        min_distance = self.get_parameter('min_distance').value
        front_angle = self.get_parameter('front_angle').value
        braking = False
        # Loop through the laser scan ranges to check for obstacles
        for i in range(len(msg.ranges)):
            # Compare the range value with the minimum and maximum range values of the laser scan. If the range value is less than the minimum range or greater than the maximum range, we skip that value and continue to the next one.
            if msg.ranges[i] < msg.range_min or msg.ranges[i] > msg.range_max:
                continue
            # Calculate the angle of the current laser scan measurement 
            theta_1 = msg.angle_min + i * msg.angle_increment
            # Calculate the relative distance in the direction of the car's movement using the current speed and the cosine of the angle
            rd_i = self.current_speed * cos(theta_1)
            if rd_i <= 0:
                continue
            # Calculate the inverse time-to-collision (iTTC) by dividing the range value by the relative distance. 
            iTTC = msg.ranges[i] / max(rd_i, 0)
            # iTTC alone never fires at low speed: it settles at range = speed * threshold,
            # so the car keeps creeping until it touches the wall. The distance floor covers
            # that, restricted to a frontal cone so the side walls of a narrow corridor do
            # not hold the brake down while the car drives along them.
            too_close = abs(theta_1) < front_angle and msg.ranges[i] < min_distance
            if iTTC < ttc_threshold or too_close:
                reason = 'too close' if too_close else 'iTTC'
                self.get_logger().warn(f'Obstacle detected! range={msg.ranges[i]:.2f}m at angle={theta_1:.2f}rad, iTTC={iTTC:.2f}s ({reason})')
                self.get_logger().warn('Obstacle detected! Stopping the vehicle.')
                stop_msg = AckermannDriveStamped()
                stop_msg.drive.speed = 0.0
                self.publisher_.publish(stop_msg)
                braking = True
                break
        self.brake_publisher_.publish(Bool(data=braking))  # True holds the brake, False releases it right away
    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x
        

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 communication system

    safety_node = SafetyNode()  # Declare the safety_node variable, an instance of the SafetyNode class — it could be named anything else, it's just a variable name

    rclpy.spin(safety_node)  # Keeps the node running until an interrupt signal (Ctrl+C) is received or the node is shut down elsewhere

    safety_node.destroy_node()  # Destroy the safety_node node to release its associated resources
    rclpy.shutdown()

if __name__ == '__main__':
    main()