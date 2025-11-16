#!/usr/bin/env python3
"""
Nœud ROS2 pour le suivi de mur utilisant un contrôleur PID.
Basé sur les interfaces low level : /cmd_dir (Float32, degrés), /cmd_vel (Float32, m/s normalisé -1 à 1).
Utilise le topic /scan pour les mesures lidar.
Convention angulaire : 0° à 360°, avec 180° devant le véhicule.
Angles devant : 165° à 195° environ.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
import numpy as np

class WallFollowPID(Node):
    def __init__(self):
        super().__init__('wall_follow_pid')

        # Paramètres PID
        self.declare_parameter('kp', 10.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 1.0)
        self.declare_parameter('target_distance', 0.8)  # Distance cible au mur en mètres
        self.declare_parameter('wall_side', -90.0)  # Angle du mur à suivre (degrés, négatif pour gauche)
        self.declare_parameter('front_angle_range', 15.0)  # Demi-plage pour détecter obstacles devant (degrés)
        self.declare_parameter('obstacle_threshold', 1.0)  # Distance seuil pour obstacle devant (m)
        self.declare_parameter('max_speed', 0.5)  # Vitesse max normalisée (0-1)
        self.declare_parameter('min_speed', 0.1)  # Vitesse min pour éviter arrêt

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.target_distance = self.get_parameter('target_distance').value
        self.wall_side = self.get_parameter('wall_side').value
        self.front_angle_range = self.get_parameter('front_angle_range').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value

        # Variables PID
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

        # Publishers
        self.dir_publisher = self.create_publisher(Float32, '/cmd_dir', 10)
        self.vel_publisher = self.create_publisher(Float32, '/cmd_vel', 10)

        # Subscriber
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info("Wall Follow PID node initialized")

    def scan_callback(self, msg):
        # Convertir les angles en degrés (ROS utilise radians)
        angle_min_deg = math.degrees(msg.angle_min)
        angle_max_deg = math.degrees(msg.angle_max)
        angle_increment_deg = math.degrees(msg.angle_increment)

        # Fonction pour obtenir l'index d'un angle
        def get_index(angle_deg):
            if angle_deg < angle_min_deg:
                angle_deg += 360.0  # Normaliser à 0-360
            index = int((angle_deg - angle_min_deg) / angle_increment_deg)
            return max(0, min(len(msg.ranges) - 1, index))

        # Calculer la distance au mur (côté)
        wall_index = get_index(self.wall_side)
        wall_distance = msg.ranges[wall_index]
        if math.isinf(wall_distance) or math.isnan(wall_distance) or wall_distance > msg.range_max:
            self.get_logger().warn("Invalid wall distance, skipping")
            return

        # Erreur : différence entre distance cible et actuelle
        error = self.target_distance - wall_distance

        # PID pour la direction
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # en secondes
        if dt > 0:
            self.integral += error * dt
            # Limiter l'intégrale pour éviter windup
            self.integral = max(-10.0, min(10.0, self.integral))
            derivative = (error - self.prev_error) / dt
            direction_cmd = self.kp * error + self.ki * self.integral + self.kd * derivative
            self.prev_error = error
            self.prev_time = current_time

            # Limiter la direction (degrés)
            direction_cmd = max(-15.0, min(15.0, direction_cmd))
        else:
            direction_cmd = 0.0

        # Vérifier obstacles devant (autour de 180° : -15° à +15°, soit 165° à 195°)
        front_start = get_index(180.0 - self.front_angle_range)
        front_end = get_index(180.0 + self.front_angle_range)
        front_distances = [msg.ranges[i] for i in range(front_start, front_end + 1)
                          if not math.isinf(msg.ranges[i]) and not math.isnan(msg.ranges[i]) and msg.ranges[i] <= msg.range_max]
        if front_distances:
            min_front_distance = min(front_distances)
            if min_front_distance < self.obstacle_threshold:
                # Obstacle devant : ralentir et tourner plus fort
                speed_cmd = self.min_speed
                direction_cmd *= 2.0  # Augmenter la correction
                direction_cmd = max(-15.0, min(15.0, direction_cmd))
                self.get_logger().warn(f"Obstacle detected at {min_front_distance:.2f}m, slowing down")
            else:
                # Pas d'obstacle : vitesse normale
                speed_cmd = self.max_speed
        else:
            speed_cmd = self.max_speed

        # Publier les commandes
        dir_msg = Float32()
        dir_msg.data = direction_cmd
        self.dir_publisher.publish(dir_msg)

        vel_msg = Float32()
        vel_msg.data = speed_cmd
        self.vel_publisher.publish(vel_msg)

        # Logging
        self.get_logger().info(f"Wall dist: {wall_distance:.2f}, Error: {error:.2f}, Dir: {direction_cmd:.2f}, Vel: {speed_cmd:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()