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
        L = self.get_parameter('lookahead_distance').value
        angle_b = pi/2
        angle_a = angle_b - theta
        distance_a = self.get_range(range_data, angle_a)
        distance_b = self.get_range(range_data, angle_b)
        alpha = atan((distance_a * cos(theta) - distance_b) / (distance_a * sin(theta)))
        D_t = distance_b * cos(alpha)
        D_tF = D_t + L * sin(alpha)
        error = dist - D_tF
        return error

    def pid_control(self, error, dt):
        kp = self.get_parameter('k_p').value
        ki = self.get_parameter('k_i').value
        kd = self.get_parameter('k_d').value
        self.integral += error * dt
        p = kp * error
        i = ki * self.integral
        d = kd * (error - self.prev_error) / dt if dt != 0 else 0
        self.prev_error = error
        return -(p + i + d) #Negative sign to correct the steering direction. Left wall: positive error means too close, but positive steering turns left (toward the wall), so invert

    def scan_callback(self, msg):
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_max = msg.range_max
        desired_distance = self.get_parameter('desired_distance').value
        error = self.get_error(msg.ranges, desired_distance)
        current_time = Time.from_msg(msg.header.stamp)
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.prev_time = current_time
        steering_angle = self.pid_control(error, dt)
        max_steering_angle = self.get_parameter('max_steering_angle').value
        steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle)) # Clamp the steering angle
        abs_steering_angle = abs(steering_angle)  # Absolute value of the steering angle for velocity adjustment
        if abs_steering_angle < radians(10): 
            velocity = 2.5  # Set a constant velocity for small steering angles
        elif abs_steering_angle < radians(20) and abs_steering_angle >= radians(10):  # If the steering angle is between 10 and 20 degrees, reduce the velocity
            velocity = 1.0 # Set a constant velocity for medium steering angles
        else:
            velocity = 0.5  # Set a constant velocity for large steering angles
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.drive.steering_angle = steering_angle
        ackermann_msg.drive.speed = velocity
        ackermann_msg.header.stamp = msg.header.stamp
        self.drive_publisher.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 communication system

    wall_follower = WallFollower()  # Declare the wall_follower variable, an instance of the WallFollower class — it could be named anything else, it's just a variable name

    rclpy.spin(wall_follower)  # Keeps the node running until an interrupt signal (Ctrl+C) is received or the node is shut down elsewhere

    wall_follower.destroy_node()  # Destroy the wall_follower node to release its associated resources
    rclpy.shutdown()

if __name__ == '__main__':
    main()