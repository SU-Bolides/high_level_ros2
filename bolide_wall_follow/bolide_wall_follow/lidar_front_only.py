#!/usr/bin/env python3
"""
Script pour diagnostiquer le LIDAR - affiche plusieurs angles autour de 0

Usage: ros2 run bolide_wall_follow lidar_front_only
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarFrontOnly(Node):
    def __init__(self):
        super().__init__('lidar_front_only')
        
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.count = 0
        self.first_scan = True
        
        self.get_logger().info("=" * 80)
        self.get_logger().info("LIDAR DIAGNOSTIC - Verification de l'angle 0 et ses voisins")
        self.get_logger().info("=" * 80)
    
    def scan_callback(self, msg: LaserScan):
        """Affiche les distances autour de l'angle 0."""
        
        # Afficher les infos du scan une seule fois
        if self.first_scan:
            self.get_logger().info(f"Infos LIDAR:")
            self.get_logger().info(f"  angle_min: {math.degrees(msg.angle_min):.1f} deg")
            self.get_logger().info(f"  angle_max: {math.degrees(msg.angle_max):.1f} deg")
            self.get_logger().info(f"  angle_increment: {math.degrees(msg.angle_increment):.2f} deg")
            self.get_logger().info(f"  range_min: {msg.range_min:.3f} m")
            self.get_logger().info(f"  range_max: {msg.range_max:.3f} m")
            self.get_logger().info(f"  Nombre de points: {len(msg.ranges)}")
            self.get_logger().info("=" * 80)
            self.first_scan = False
        
        # Afficher toutes les 10 mesures
        self.count += 1
        if self.count % 10 != 0:
            return
        
        # Tester plusieurs angles : de -180° à 180° par pas de 10°
        angles = list(range(-180, 181, 10))

        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("DISTANCES POUR TOUS LES ANGLES (-180 à 180 tous les 10°):")
        self.get_logger().info("-" * 80)

        valid_readings = []

        for angle_deg in angles:
            angle_rad = math.radians(angle_deg)
            
            # Vérifier que l'angle est dans la plage
            if angle_rad < msg.angle_min or angle_rad > msg.angle_max:
                self.get_logger().info(f"  {angle_deg:>4}deg : HORS PLAGE")
                continue
            
            # Calculer l'index
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)
            index = max(0, min(index, len(msg.ranges) - 1))
            
            # Obtenir la distance
            distance = msg.ranges[index]
            
            # Vérifier si valide
            if math.isnan(distance) or math.isinf(distance):
                self.get_logger().info(f"  {angle_deg:>4}deg : NAN/INF")
            elif distance < msg.range_min:
                self.get_logger().info(f"  {angle_deg:>4}deg : {distance:.3f}m (< range_min={msg.range_min:.3f}m) INVALIDE")
            elif distance >= msg.range_max:
                self.get_logger().info(f"  {angle_deg:>4}deg : {distance:.3f}m (>= range_max) HORS PORTEE")
            else:
                # Distance valide
                marker = ""
                if distance < 0.3:
                    marker = " *** TRES PROCHE ***"
                elif distance < 0.5:
                    marker = " ** PROCHE **"
                elif distance < 1.0:
                    marker = " * Visible *"
                
                self.get_logger().info(f"  {angle_deg:>4}deg : {distance:.3f}m{marker}")
                valid_readings.append((angle_deg, distance))
        
        # Résumé
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("RESUME:")
        self.get_logger().info("-" * 80)
             
        if valid_readings:
            min_reading = min(valid_readings, key=lambda x: x[1])
            self.get_logger().info(f"TOUS_ANGLES - Obstacle le plus proche: {min_reading[1]:.3f}m a {min_reading[0]}deg")
        else:
            self.get_logger().info("TOUS_ANGLES - Aucune lecture valide!")
        
        self.get_logger().info("=" * 80 + "\n")


def main():
    rclpy.init()
    node = LidarFrontOnly()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
