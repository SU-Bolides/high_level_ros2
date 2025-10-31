#!/usr/bin/env python3
"""
Noeud de detection d'obstacles et arret d'urgence
Verifie la presence d'obstacles dans un arc avant (-160 a 160 deg)
et declenche un arret d'urgence si necessaire.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
import math


class ObstacleChecker(Node):
    def __init__(self):
        super().__init__('obstacle_checker')
        
        # Parametres
        self.declare_parameter('obstacle_distance', 0.4)
        self.declare_parameter('debug', True)
        
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.debug = self.get_parameter('debug').value
        
        # Etat
        self.emergency_active = False
        self.last_obstacle_angle = 0
        self.last_obstacle_distance = 0.0
        
        # Publishers
        self.pub_speed = self.create_publisher(Float32, '/cmd_vel', 10)
        self.pub_dir = self.create_publisher(Float32, '/cmd_dir', 10)
        self.pub_emergency = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Subscriber
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Obstacle Checker Node Started")
        self.get_logger().info(f"Detection range: -160 to 160 degrees")
        self.get_logger().info(f"Obstacle threshold: {self.obstacle_distance}m")
        self.get_logger().info("=" * 60)
    
    def scan_callback(self, msg: LaserScan):
        """Verifie la presence d'obstacles dans l'arc avant."""
        
        obstacle_detected = False
        min_distance = float('inf')
        min_angle = 0
        
        # Scanner l'arc de -165 a 165 degres
        for angle_deg in range(-165, 165, 5):
            angle_rad = math.radians(angle_deg)
            
            # Verifier que l'angle est dans la plage du LIDAR
            if angle_rad < msg.angle_min or angle_rad > msg.angle_max:
                continue
            
            # Calculer l'index
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)
            index = max(0, min(index, len(msg.ranges) - 1))
            
            # Obtenir la distance
            distance = msg.ranges[index]
            
            # Ignorer les valeurs invalides
            if math.isnan(distance) or math.isinf(distance):
                continue
            
            if distance < msg.range_min or distance >= msg.range_max:
                continue
            
            # Traquer la distance minimale
            if distance < min_distance:
                min_distance = distance
                min_angle = angle_deg
            
            # Verifier si obstacle trop proche
            if distance < self.obstacle_distance:
                obstacle_detected = True
        
        # Gestion de l'arret d'urgence
        if obstacle_detected:
            if not self.emergency_active:
                # Premier declenchement
                self.get_logger().error(
                    f"ARRET D'URGENCE! Obstacle a {min_distance:.2f}m (angle {min_angle}deg)")
                self.emergency_active = True
                self.last_obstacle_angle = min_angle
                self.last_obstacle_distance = min_distance
            
            # Publier l'arret
            self.stop_vehicle()
            
            # Publier l'etat d'urgence
            emergency_msg = Bool()
            emergency_msg.data = True
            self.pub_emergency.publish(emergency_msg)
            
            if self.debug:
                self.get_logger().debug(
                    f"Emergency active - Obstacle: {min_distance:.2f}m at {min_angle}deg")
        
        else:
            # Pas d'obstacle
            if self.emergency_active:
                self.get_logger().info(
                    f"Voie libre - Distance minimale: {min_distance:.2f}m at {min_angle}deg")
                self.emergency_active = False
            
            # Publier l'etat normal
            emergency_msg = Bool()
            emergency_msg.data = False
            self.pub_emergency.publish(emergency_msg)
            
            if self.debug:
                self.get_logger().debug(
                    f"Clear path - Min distance: {min_distance:.2f}m at {min_angle}deg")
    
    def stop_vehicle(self):
        """Arrete la voiture en envoyant vitesse et direction a zero."""
        stop_speed = Float32()
        stop_dir = Float32()
        
        stop_speed.data = 0.0
        stop_dir.data = 0.0
        
        self.pub_speed.publish(stop_speed)
        self.pub_dir.publish(stop_dir)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleChecker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        # Arret final
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
