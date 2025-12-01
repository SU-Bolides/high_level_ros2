#!/usr/bin/env python3
"""
Noeud de detection d'obstacles et arret d'urgence
Verifie la presence d'obstacles DEVANT UNIQUEMENT (150-210 deg, 180°=front)
et declenche un arret d'urgence si necessaire.
Recule automatiquement si obstacle a 30cm ou moins.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from bolide_interfaces.msg import MultipleRange
from std_msgs.msg import Float32, Bool
import math
import time


class ObstacleChecker(Node):
    def __init__(self):
        super().__init__('obstacle_checker')
        
        # Parametres
        self.declare_parameter('obstacle_distance', 0.3)
        self.declare_parameter('debug', True)
        self.declare_parameter('neutral_duration', 2)  # Duree phase neutre (secondes)
        self.declare_parameter('reverse_duration', 1)  # Duree recul (secondes)
        self.declare_parameter('reverse_speed', -0.06)   # Vitesse de recul
        
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.debug = self.get_parameter('debug').value
        self.neutral_duration = self.get_parameter('neutral_duration').value
        self.reverse_duration = self.get_parameter('reverse_duration').value
        self.reverse_speed = self.get_parameter('reverse_speed').value
        
        # Etat
        self.emergency_active = False
        self.last_obstacle_angle = 0
        self.last_obstacle_distance = 0.0
        
        # Machine d'etats pour le recul
        # Etats possibles: 'normal', 'stopping', 'neutral', 'reversing'
        self.state = 'normal'
        self.state_start_time = 0.0
        
        # Publishers
        self.pub_speed = self.create_publisher(Float32, '/cmd_vel', 10)
        self.pub_dir = self.create_publisher(Float32, '/cmd_dir', 10)
        self.pub_emergency = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Subscriber
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.sub_ir_back = self.create_subscription(
            MultipleRange, '/raw_rear_range_data', self.check_back, 10)
        
        # Timer pour gerer les sequences temporelles
        self.timer = self.create_timer(0.1, self.state_machine_update)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Obstacle Checker Node Started")
        self.get_logger().info(f"Detection arc: 150° to 210° (180°=FRONT, ±30°)")
        self.get_logger().info(f"Obstacle threshold: {self.obstacle_distance}m")
        self.get_logger().info(f"Neutral duration: {self.neutral_duration}s")
        self.get_logger().info(f"Reverse duration: {self.reverse_duration}s")
        self.get_logger().info(f"Reverse speed: {self.reverse_speed}m/s")
        self.get_logger().info("=" * 60)
    
    def state_machine_update(self):
        """Gere les transitions d'etats temporelles."""
        current_time = time.time()
        elapsed = current_time - self.state_start_time
        
        if self.state == 'stopping':
            # Attendre un peu avant de passer au neutre
            if elapsed >= 0.2:
                self.get_logger().info("Phase NEUTRE - Preparation au recul")
                self.state = 'neutral'
                self.state_start_time = current_time
                self.set_neutral()
        
        elif self.state == 'neutral':
            # Phase neutre avant recul
            if elapsed >= self.neutral_duration:
                self.get_logger().warn(f"RECUL en cours ({self.reverse_duration}s)")
                self.state = 'reversing'
                self.state_start_time = current_time
                self.reverse_vehicle()
        
        elif self.state == 'reversing':
            # Continuer de reculer
            self.reverse_vehicle()
            
            # Fin du recul
            if elapsed >= self.reverse_duration:
                self.get_logger().info("Recul termine - Retour au mode normal")
                self.state = 'normal'
                self.emergency_active = False
                self.stop_vehicle()
    
    def scan_callback(self, msg: LaserScan):
        """Verifie la presence d'obstacles DEVANT UNIQUEMENT (150-210 deg)."""
        
        # Si on est en train de reculer, ignorer le scan
        if self.state in ['stopping', 'neutral', 'reversing']:
            return
        
        obstacle_detected = False
        min_distance = float('inf')
        min_angle = 0
        
        # Scanner UNIQUEMENT l'arc DEVANT: 150° à 210° (180° ± 30°)
        # 180° = devant de la voiture
        for angle_deg in range(150, 211, 5):
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
            
            # Verifier si obstacle trop proche DEVANT
            if distance < self.obstacle_distance:
                obstacle_detected = True
        
        # Gestion de l'arret d'urgence
        if obstacle_detected:
            if not self.emergency_active:
                # Premier declenchement
                self.get_logger().error(
                    f"OBSTACLE DEVANT! Distance: {min_distance:.2f}m (angle {min_angle}°)")
                self.emergency_active = True
                self.last_obstacle_angle = min_angle
                self.last_obstacle_distance = min_distance
                
                # Demarrer la sequence de recul
                self.state = 'stopping'
                self.state_start_time = time.time()
                self.stop_vehicle()
            
            # Publier l'etat d'urgence
            emergency_msg = Bool()
            emergency_msg.data = True
            self.pub_emergency.publish(emergency_msg)
            
            if self.debug:
                self.get_logger().debug(
                    f"Emergency active - Obstacle DEVANT: {min_distance:.2f}m at {min_angle}°")
        
        else:
            # Pas d'obstacle DEVANT
            if self.emergency_active and self.state == 'normal':
                self.get_logger().info(
                    f"Voie DEVANT libre - Distance: {min_distance:.2f}m")
                self.emergency_active = False
            
            # Publier l'etat normal
            emergency_msg = Bool()
            emergency_msg.data = False
            self.pub_emergency.publish(emergency_msg)
            
            if self.debug and self.state == 'normal':
                self.get_logger().debug(
                    f"Clear path FRONT - Min distance: {min_distance:.2f}m at {min_angle}°")
    
    def stop_vehicle(self):
        """Arrete la voiture en envoyant vitesse et direction a zero."""
        stop_speed = Float32()
        stop_dir = Float32()
        
        stop_speed.data = 0.0
        stop_dir.data = 0.0
        
        self.pub_speed.publish(stop_speed)
        self.pub_dir.publish(stop_dir)
    
    def set_neutral(self):
        """Phase neutre (vitesse 0) avant de reculer."""
        neutral_speed = Float32()
        neutral_dir = Float32()
        
        neutral_speed.data = 0.0
        neutral_dir.data = 0.0
        
        self.pub_speed.publish(neutral_speed)
        self.pub_dir.publish(neutral_dir)
    
    def reverse_vehicle(self):
        """Recule la voiture."""

        self.set_neutral()  # Assurer que la voiture est a l'arret avant de reculer
        reverse_speed = Float32()
        reverse_dir = Float32()
        
        reverse_speed.data = self.reverse_speed  # Vitesse negative pour reculer
        reverse_dir.data = 0.0  # Direction droite
        
        self.pub_speed.publish(reverse_speed)
        self.pub_dir.publish(reverse_dir)

    def check_back(self, msg):
        """Verifie les capteurs IR arrieres pour eviter les collisions en reculant."""
        min_distance = float('inf')
        
        # Acceder aux champs du message MultipleRange
        ranges = [msg.ir_rear_left.range, msg.ir_rear_right.range, msg.sonar_rear.range]
        
        for distance in ranges:
            if not math.isnan(distance) and not math.isinf(distance) and distance > 0:
                if distance < min_distance:
                    min_distance = distance
        
        # Seuil de securite pour l'arriere (par exemple 0.2m)
        back_obstacle_threshold = 0.1
        
        if self.state == 'reversing':
            if min_distance < back_obstacle_threshold:
                self.get_logger().error(
                    f"OBSTACLE ARRIERE DETECTE! Distance: {min_distance:.2f}m - Arret du recul")
                # Arreter le recul immediatement
                self.state = 'normal'
                self.emergency_active = False
                self.stop_vehicle()
        # else:
        #     self.get_logger().info(
        #         f"Clear path REAR - Min distance: {min_distance:.2f}m")


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
