from math import pi, cos, sin, atan, isnan
import time

import rclpy
from rclpy.node import Node

from bolide_interfaces.msg import SpeedDirection
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool

# HELPERS FUNCTIONS


def get_range(scan_msg, angle):
    assert (angle >= scan_msg.angle_min and angle <= scan_msg.angle_max)
    i = int((angle - scan_msg.angle_min)/(scan_msg.angle_increment))
    if (isnan(scan_msg.ranges[i]) or scan_msg.ranges[i] > scan_msg.range_max):
        return scan_msg.range_max
    return scan_msg.ranges[i]


def to_radians(theta):
    return pi * theta / 180.0


def to_degrees(theta):
    return theta * 180.0 / pi


class WallFollow(Node):
    def __init__(self):
        super().__init__("wall_follow")

        # PARAMETERS
        self.declare_parameter('kp', 10.0)  # Réduit pour moins d'oscillations
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 3.0)   # Augmenté pour mieux amortir
        self.declare_parameter('target_distance', 0.6)  # Distance cible au mur (m)
        self.declare_parameter('debug', True)

        # PID CONTROL PARAMS
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        
        self.servo_offset = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.integral = 0.0
        self.start_t = -1
        self.curr_t = 0.0
        self.prev_t = 0.0
        
        # EMERGENCY STOP STATE
        self.emergency_stop_active = False
        
        # INITIALIZATION FLAG
        self.initialized = False
        self.init_scan_count = 0

        # PUBLISHER AND SUBSCRIBER
        self.pub_speed = self.create_publisher(Float32, '/cmd_vel', 10)
        self.pub_dir = self.create_publisher(Float32, '/cmd_dir', 10)

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # NOUVEAU: Subscriber pour l'arret d'urgence
        self.sub_emergency = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, 10)
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Wall Follow Node Started")
        self.get_logger().info(f"Target distance to wall: {self.target_distance}m")
        self.get_logger().info(f"PID gains - Kp:{self.kp}, Ki:{self.ki}, Kd:{self.kd}")
        self.get_logger().info(f"Debug mode: {self.debug}")
        self.get_logger().info("=" * 60)
    
    def emergency_callback(self, msg: Bool):
        """Callback pour recevoir l'etat d'urgence de obstacle_checker"""
        if msg.data and not self.emergency_stop_active:
            self.get_logger().warn("EMERGENCY STOP received from obstacle_checker!")
            self.emergency_stop_active = True
        elif not msg.data and self.emergency_stop_active:
            self.get_logger().info("Emergency cleared - Resuming wall follow")
            self.emergency_stop_active = False

    def get_error(self, scan_msg, dist):
        # Mesures pour suivi de mur À GAUCHE (90° = gauche perpendiculaire)
        # 180° = devant, donc mur à gauche = 90°
        a = get_range(scan_msg, to_radians(130.0))  # Gauche-avant (45° vers la gauche depuis devant)
        b = get_range(scan_msg, to_radians(90.0))   # Gauche perpendiculaire
        
        if self.debug:
            # Diagnostic complet - utiliser seulement les angles disponibles
            angle_min_deg = to_degrees(scan_msg.angle_min)
            angle_max_deg = to_degrees(scan_msg.angle_max)
            
            self.get_logger().info(f"LIDAR range: {angle_min_deg:.1f}° to {angle_max_deg:.1f}°")
            
            # Scanner seulement les angles disponibles
            front = get_range(scan_msg, to_radians(180.0))
            left_45 = get_range(scan_msg, to_radians(135.0))
            left_90 = get_range(scan_msg, to_radians(90.0))
            back = get_range(scan_msg, to_radians(0.0))
            
            # Vérifier si les angles droits sont disponibles
            if to_radians(270.0) <= scan_msg.angle_max:
                right_90 = get_range(scan_msg, to_radians(270.0))
                self.get_logger().info(f"=== SCAN (180°=FRONT, 90°=LEFT) ===")
                self.get_logger().info(f"Front(180°): {front:.2f}m | Back(0°): {back:.2f}m")
                self.get_logger().info(f"Left-45°(135°): {left_45:.2f}m | Left-90°(90°): {left_90:.2f}m")
                self.get_logger().info(f"Right-90°(270°): {right_90:.2f}m")
            else:
                self.get_logger().info(f"=== SCAN (180°=FRONT, 90°=LEFT) ===")
                self.get_logger().info(f"Front(180°): {front:.2f}m | Back(0°): {back:.2f}m")
                self.get_logger().info(f"Left-45°(135°): {left_45:.2f}m | Left-90°(90°): {left_90:.2f}m")
        
        # Calcul de la distance perpendiculaire au mur
        theta = to_radians(40.0)  # Angle entre 130° et 90°
        
        # Protection contre division par zéro
        sin_theta = sin(theta)
        if abs(sin_theta) < 0.01:
            self.get_logger().warn("Division par zéro évitée dans calcul alpha")
            alpha = 0.0
        else:
            alpha = atan((a * cos(theta) - b) / (a * sin_theta))
        
        D_t = b * cos(alpha)
        
        if self.debug:
            self.get_logger().info(f"Calculs: a(130°)={a:.2f}m, b(90°)={b:.2f}m")
            self.get_logger().info(f"theta={to_degrees(theta):.1f}°, alpha={to_degrees(alpha):.2f}°")
            self.get_logger().info(f"Distance au mur D_t={D_t:.3f}m, target={dist}m")

        self.prev_error = self.error
        self.error = dist - D_t  # Positif si trop loin, négatif si trop proche
        
        # Anti-windup
        self.integral += self.error
        self.integral = max(-10.0, min(10.0, self.integral))
        
        self.prev_t = self.curr_t
        self.curr_t = scan_msg.header.stamp.nanosec * 10e-9 + scan_msg.header.stamp.sec
        if self.start_t == -1:
            self.start_t = self.curr_t
        
        if self.debug:
            self.get_logger().info(f"ERROR={self.error:.3f}m (>0:trop loin, <0:trop proche), integral={self.integral:.3f}")

    def pid_control(self):
        angle = 0.0
        
        if self.prev_t == 0:
            # Initialisation: avancer tout droit
            drive_msg = Float32()
            speed_msg = Float32()
            drive_msg.data = 0.0
            speed_msg.data = 0.03
            self.pub_dir.publish(drive_msg)
            self.pub_speed.publish(speed_msg)
            return
        
        dt = self.curr_t - self.prev_t
        if dt <= 0 or dt > 1.0:
            self.get_logger().warn(f"dt invalide: {dt}")
            return
        
        # Calcul PID
        angle = (self.kp * self.error + 
                 self.ki * self.integral * (self.curr_t - self.start_t) + 
                 self.kd * (self.error - self.prev_error) / dt)
        
        if self.debug:
            self.get_logger().info(f"PID: P={self.kp * self.error:.3f}, " +
                                   f"I={self.ki * self.integral * (self.curr_t - self.start_t):.3f}, " +
                                   f"D={self.kd * (self.error - self.prev_error) / dt:.3f}")
            self.get_logger().info(f"Angle brut: {to_degrees(angle):.2f}°")
        
        drive_msg = Float32()
        speed_msg = Float32()
        
        # Normalisation: diviser par angle max de braquage (15°)
        drive_msg.data = angle / to_radians(15.0)
        drive_msg.data = max(-1.0, min(1.0, drive_msg.data))
        
        # Ajustement vitesse selon braquage
        abs_dir = abs(drive_msg.data)
        if abs_dir < 0.3:
            speed_msg.data = 0.04  # Ligne droite: plus rapide
        elif abs_dir < 0.6:
            speed_msg.data = 0.03  # Virage léger
        else:
            speed_msg.data = 0.02  # Virage serré: ralentir
        
        if self.debug:
            self.get_logger().info(f"Commande: dir={drive_msg.data:.3f} (-1=droite, +1=gauche), speed={speed_msg.data:.3f}m/s")
            self.get_logger().info("-" * 60)
        
        self.pub_dir.publish(drive_msg)
        self.pub_speed.publish(speed_msg)

    def scan_callback(self, scan_msg):
        # PRIORITE 1: Verifier si arret d'urgence actif
        if self.emergency_stop_active:
            # obstacle_checker gere l'arret et le recul
            return
        
        # Phase d'initialisation: attendre quelques scans pour stabiliser
        if not self.initialized:
            self.init_scan_count += 1
            if self.init_scan_count < 3:
                self.get_logger().info(f"Initializing... {self.init_scan_count}/3")
                return
            else:
                self.initialized = True
                self.get_logger().info("Initialization complete - Starting wall follow")
        
        # PRIORITE 2: Suivi de mur normal (distance cible paramétrable)
        self.get_error(scan_msg, self.target_distance)
        self.pid_control()

def main(args=None):
    rclpy.init(args=args)
    wall_follow = WallFollow()
    rclpy.spin(wall_follow)
    rclpy.shutdown()
