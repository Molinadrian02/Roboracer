#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from rclpy.time import Time
import numpy as np
from math import cos, pi, atan, radians, sin, isfinite

class ReactiveFollowGap(Node):

    def __init__(self):
        super().__init__('reactive_follow_gap')  # Initialize the node with the name 'reactive_follow_gap'

        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)  
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        #Declaración de parámetros
        self.declare_parameter('bubble_radius', 0.5)  # radio de la burbuja
        self.declare_parameter('angle_window', 30.0)  # ventana de ángulos para buscar el gap
        self.declare_parameter('max_steering_angle', 0.4189)  # ángulo máximo de dirección (24 grados en radianes)
        self.declare_parameter('speed', 1.0)  # velocidad constante del vehículo
        self.declare_parameter('angle_min', -2.35619449615)  # ángulo mínimo del LiDAR (en radianes)
        self.declare_parameter('angle_max', 2.35619449615)  # ángulo máximo del LiDAR (en radianes)
    def preprocess_lidar(self, ranges):
        # Paso 1: Preprocesar los datos del LiDAR
        # Aquí puedes implementar la lógica para filtrar o suavizar los datos de LiDAR según sea necesario.
        ranges = np.array(ranges)
        start_idx = int(((-90* pi/180) + abs(self.get_parameter('angle_min').value)) / self.get_parameter('angle_increment').value)
        end_idx = int(((90* pi/180) + abs(self.get_parameter('angle_max').value)) / self.get_parameter('angle_increment').value)
        ranges = ranges[start_idx:end_idx]
        pass
    def find_max_gap(self, free_space_ranges):
        # Paso 4: Encontrar el gap más grande
        # Aquí puedes implementar la lógica para encontrar el gap más grande en los datos de LiDAR preprocesados.
        pass
    def find_best_point(self, start, end, ranges):
        # Paso 5: Encontrar el mejor punto dentro del gap
        # Aquí puedes implementar la lógica para encontrar el mejor punto dentro del gap encontrado.
        for i in range(len(ranges)):
                    if ranges[i] < d:
                        d = ranges[i]
                        closest_idx = i
        pass
    def lidar_callback(self, msg):
        # Callback function for the LiDAR data
        # Aquí puedes implementar la lógica para procesar los datos del LiDAR y controlar el vehículo
        self.get_logger().info("Lidar callback triggered")
        self.get_logger().info(f"Received LiDAR data with {len(msg.ranges)} ranges")
        self.get_logger().info(f"Angulo mínimo, maximo e incremento {msg.angle_min}, {msg.angle_max}, {msg.angle_increment}")
        self.get_logger().info(f"Rango maximo: {msg.range_max}")
        pass

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS2 communication system

    wall_follower = ReactiveFollowGap()  # Declare the wall_follower variable, an instance of the WallFollower class — it could be named anything else, it's just a variable name

    rclpy.spin(wall_follower)  # Keeps the node running until an interrupt signal (Ctrl+C) is received or the node is shut down elsewhere

    wall_follower.destroy_node()  # Destroy the wall_follower node to release its associated resources
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
