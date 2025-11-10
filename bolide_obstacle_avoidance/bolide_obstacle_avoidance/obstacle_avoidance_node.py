#!/usr/bin/env python3
"""
Nœud d'évitement d'obstacles pour voiture autonome

Cet algorithme analyse les données du Lidar pour détecter les obstacles
et décide de tourner à gauche ou à droite pour les éviter.
Il divise l'espace en secteurs : gauche, devant, droite.
Si un obstacle est détecté devant, il choisit le côté avec le plus d'espace libre.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
import math


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Paramètres configurables
        self.declare_parameter('front_threshold', 0.5)  # Distance seuil pour obstacle devant (m)
        self.declare_parameter('side_threshold', 0.3)   # Distance seuil pour les côtés (m)
        self.declare_parameter('turn_speed', 0.3)       # Vitesse de rotation normalisée (-1 à 1)
        self.declare_parameter('forward_speed', 0.1)    # Vitesse en avant (m/s)
        self.declare_parameter('debug', True)

        self.front_threshold = self.get_parameter('front_threshold').value
        self.side_threshold = self.get_parameter('side_threshold').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.debug = self.get_parameter('debug').value

        # États
        self.emergency_stop_active = False

        # Publishers pour les commandes de mouvement
        self.pub_speed = self.create_publisher(Float32, '/cmd_vel', 10)
        self.pub_dir = self.create_publisher(Float32, '/cmd_dir', 10)

        # Subscriber pour les données du Lidar
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Subscriber pour l'arrêt d'urgence (de obstacle_checker)
        self.sub_emergency = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, 10)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Obstacle Avoidance Node Started")
        self.get_logger().info(f"Front threshold: {self.front_threshold}m")
        self.get_logger().info(f"Side threshold: {self.side_threshold}m")
        self.get_logger().info(f"Turn speed: {self.turn_speed}")
        self.get_logger().info(f"Forward speed: {self.forward_speed}")
        self.get_logger().info("=" * 60)

    def emergency_callback(self, msg: Bool):
        """Callback pour l'arrêt d'urgence"""
        if msg.data and not self.emergency_stop_active:
            self.get_logger().warn("EMERGENCY STOP received!")
            self.emergency_stop_active = True
            self.stop_vehicle()
        elif not msg.data and self.emergency_stop_active:
            self.get_logger().info("Emergency cleared - Resuming obstacle avoidance")
            self.emergency_stop_active = False

    def scan_callback(self, msg: LaserScan):
        """Analyse les données du scan Lidar et décide du mouvement"""

        # Priorité : arrêt d'urgence
        if self.emergency_stop_active:
            return

        # Analyser les secteurs
        left_distances = self.get_sector_distances(msg, 90, 150)   # Gauche : 90° à 150°
        front_distances = self.get_sector_distances(msg, 150, 210) # Devant : 150° à 210°
        right_distances = self.get_sector_distances(msg, 210, 270) # Droite : 210° à 270°

        # Calculer les distances moyennes pour chaque secteur
        left_avg = self.average_distance(left_distances)
        front_avg = self.average_distance(front_distances)
        right_avg = self.average_distance(right_distances)

        if self.debug:
            self.get_logger().debug(".2f")

        # Logique d'évitement
        if front_avg < self.front_threshold:
            # Obstacle devant : choisir le côté avec plus d'espace
            if left_avg > right_avg and left_avg > self.side_threshold:
                # Tourner à gauche
                self.turn_left()
                if self.debug:
                    self.get_logger().info("Obstacle devant - Tourne à GAUCHE")
            elif right_avg > self.side_threshold:
                # Tourner à droite
                self.turn_right()
                if self.debug:
                    self.get_logger().info("Obstacle devant - Tourne à DROITE")
            else:
                # Aucun côté libre : s'arrêter
                self.stop_vehicle()
                if self.debug:
                    self.get_logger().warn("Obstacle devant - Aucun côté libre - ARRÊT")
        else:
            # Pas d'obstacle devant : avancer tout droit
            self.move_forward()
            if self.debug:
                self.get_logger().debug("Voie libre - Avance tout droit")

    def get_sector_distances(self, scan_msg: LaserScan, start_deg: int, end_deg: int):
        """Extrait les distances valides dans un secteur angulaire"""
        distances = []
        for angle_deg in range(start_deg, end_deg + 1, 5):  # Pas de 5° pour optimisation
            angle_rad = math.radians(angle_deg)

            # Vérifier que l'angle est dans la plage du Lidar
            if angle_rad < scan_msg.angle_min or angle_rad > scan_msg.angle_max:
                continue

            # Calculer l'index
            index = int((angle_rad - scan_msg.angle_min) / scan_msg.angle_increment)
            index = max(0, min(index, len(scan_msg.ranges) - 1))

            # Obtenir la distance
            distance = scan_msg.ranges[index]

            # Ignorer les valeurs invalides
            if math.isnan(distance) or math.isinf(distance):
                continue
            if distance < scan_msg.range_min or distance >= scan_msg.range_max:
                continue

            distances.append(distance)

        return distances

    def average_distance(self, distances):
        """Calcule la distance moyenne, retourne une grande valeur si vide"""
        if not distances:
            return 10.0  # Valeur par défaut si pas de données
        return sum(distances) / len(distances)

    def move_forward(self):
        """Avancer tout droit"""
        speed_msg = Float32()
        dir_msg = Float32()
        speed_msg.data = self.forward_speed
        dir_msg.data = 0.0  # Direction neutre
        self.pub_speed.publish(speed_msg)
        self.pub_dir.publish(dir_msg)

    def turn_left(self):
        """Tourner à gauche"""
        speed_msg = Float32()
        dir_msg = Float32()
        speed_msg.data = self.forward_speed * 0.5  # Réduire la vitesse en tournant
        dir_msg.data = -self.turn_speed  # Négatif pour gauche
        self.pub_speed.publish(speed_msg)
        self.pub_dir.publish(dir_msg)

    def turn_right(self):
        """Tourner à droite"""
        speed_msg = Float32()
        dir_msg = Float32()
        speed_msg.data = self.forward_speed * 0.5  # Réduire la vitesse en tournant
        dir_msg.data = self.turn_speed  # Positif pour droite
        self.pub_speed.publish(speed_msg)
        self.pub_dir.publish(dir_msg)

    def stop_vehicle(self):
        """Arrêter la voiture"""
        speed_msg = Float32()
        dir_msg = Float32()
        speed_msg.data = 0.0
        dir_msg.data = 0.0
        self.pub_speed.publish(speed_msg)
        self.pub_dir.publish(dir_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested")
    finally:
        node.stop_vehicle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()