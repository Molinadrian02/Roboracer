#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from rclpy.time import Time
from math import cos, pi, atan, radians, sin, isfinite

# imports de mensajes...

class WallFollower(Node):
    def __init__(self):
        # nombre del nodo, topics, sub y pub
        super().__init__('wall_follower')
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)  
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        # ganancias PID y parámetros geométricos (desired_distance, lookahead_distance, max_steering_angle)
        self.declare_parameter('k_p', 1.0)
        self.declare_parameter('k_i', 0.0)
        self.declare_parameter('k_d', 0.0)
        self.declare_parameter('desired_distance', 1.0)  # distancia deseada al muro
        self.declare_parameter('lookahead_distance', 1.0)  # distancia de anticipación para el control
        self.declare_parameter('max_steering_angle', 0.4189)  # ángulo máximo de dirección (24 grados en radianes)
        self.declare_parameter('angle_between_beams', 1.0)  # angulo entre los haces a y b (en radianes)
        # estado del PID
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        
    def get_range(self, range_data, angle):
        # In this function, we will calculate the index of the range data corresponding to the given angle and return the distance at that index. 
        #  If the distance is not finite, we will return the maximum range value.
         
        angle_index = round(((angle - self.angle_min) / self.angle_increment))
        distance = range_data[angle_index]
        if isfinite(distance):
            return distance
        return self.range_max  # Set a maximum distance if the value is not finite
        
    def get_error(self, range_data, dist):
        theta = self.get_parameter('angle_between_beams').value
        angle_b = pi/2
        angle_a = angle_b - theta
        distance_a = self.get_range(range_data, angle_a)
        distance_b = self.get_range(range_data, angle_b)
        alpha = atan((distance_a * cos(theta) - distance_b) / (distance_a * sin(theta)))
        error = dist - (distance_b * cos(alpha))
        return error

    def pid_control(self, error, velocity):
        

    def scan_callback(self, msg):
        current_time = Time.from_msg(msg.header.stamp)
        if self.prev_time is None:
            self.prev_time = current_time
            return
def main(args=None):
    