# #!/usr/bin/env python3
# """
# Potential Field based navigator for unknown circuits.
# - Combines an attractive forward vector and repulsive forces from lidar obstacles
# - Produces smooth commands for navigation on unknown tracks
# """

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import math
# import numpy as np
# from std_msgs.msg import Float32, Bool

# class PotentialFieldNavigator(Node):
#     def __init__(self):
#         super().__init__('potential_field_navigator')

#         # Variables
#         self.max_speed = 0.05 
#         self.min_speed = 0.025 
#         self.max_steering_angle_deg = 30.0  # degrees
#         self.influence_distance = 3.0  # m: obstacles within this distance contribute
#         self.k_repulsive = 0.3  # repulsive gain    TODO: adjust to try and have less oscillations
#         self.k_attractive = 0.6 # attractive gain
#         self.smoothing_alpha = 0.3

#         # Smoothed command state
#         self.smoothed_linear = 0.0
#         self.smoothed_steering = 0.0  # degrees

#         # Publisher / Subscriber (namespaced to allow a navigation master to select algorithm outputs)
#         self.cmd_vel_pub = self.create_publisher(Float32, '/potential_field/cmd_vel', 10)
#         self.cmd_dir_pub = self.create_publisher(Float32, '/potential_field/cmd_dir', 10)
#         # Also keep publishing on original topics for backward compatibility - you can comment these out if not needed
#         self.cmd_vel_pub_raw = self.create_publisher(Float32, '/cmd_vel', 10)
#         self.cmd_dir_pub_raw = self.create_publisher(Float32, '/cmd_dir', 10)
#         self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

#         # Emergency stop subscription (Bool): published by check_obstacle/obstacle_checker
#         self.emergency_stop = False
#         self.emergency_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

#         self.get_logger().info('Potential Field Navigator started')

#     def scan_callback(self, msg: LaserScan):
#         try:
#             if self.emergency_stop:
#                 self.get_logger().warn("Emergency stop active - skipping potential field processing")
#                 return
#             # Compute repulsive field from obstacles
#             rep_x = 0.0
#             rep_y = 0.0
#             min_front = float('inf')
#             min_front_angle = 0.0

#             for i, r in enumerate(msg.ranges):
#                 if not math.isfinite(r) or r <= 0.0:    # ignore invalid readings
#                     continue
#                 # Calculate the angle for the current laser scan point
#                 angle = msg.angle_min + i * msg.angle_increment
#                 # Check if this range is the smallest in front (within 40 degrees of 180°)
#                 if r < min_front and abs(math.degrees(angle) - 180.0) < 60.0:
#                     min_front = r   # update the closest front obstacle
#                     min_front_angle = angle  # not used for now, maybe to add to have better avoidance?

#                 if r < self.influence_distance: # if the obstacle is within influence distance aka close enough to matter
#                     # repulsive magnitude inverse-squared (limited)
#                     mag = self.k_repulsive * (1.0 / (r + 1e-6) - 1.0 / self.influence_distance)
#                     if mag < 0:
#                         continue
#                     fx = -mag * math.cos(angle)
#                     fy = -mag * math.sin(angle)
#                     rep_x += fx
#                     rep_y += fy

#             # Attractive force forward along x-axis (towards 180 degrees aka the front of the car)
#             att_x = self.k_attractive
#             att_y = 0.0

#             total_x = att_x + rep_x
#             total_y = att_y + rep_y

#             # Compute resulting angle and magnitude
#             resultant_angle = math.atan2(total_y, total_x)
#             resultant_mag = math.hypot(total_x, total_y)

#             # Convert to steering angle (degrees) based on resultant_angle
#             steering_deg = math.degrees(resultant_angle)
#             steering_deg = max(-self.max_steering_angle_deg, min(self.max_steering_angle_deg, steering_deg))

#             # Normalize steering to [-1, 1]
#             steering_norm = steering_deg / self.max_steering_angle_deg
#             steering_norm = max(-1.0, min(1.0, steering_norm))

#             # Linear command: map resultant magnitude to [min_speed, max_speed] to ensure we stay in bounds of the authorized speed
#             mag_fraction = resultant_mag / (1.0 + resultant_mag)
#             lin_cmd = self.min_speed + (self.max_speed - self.min_speed) * mag_fraction

#             # Reduce speed when turning sharply or when front obstacle is near
#             lin_cmd *= max(0.3, 1.0 - abs(steering_norm))
#             if min_front < 1.0:
#                 lin_cmd *= min_front / 1.0

#             # Ensure within bounds
#             lin_cmd = max(0.0, min(self.max_speed, lin_cmd))
#             if lin_cmd > 0 and lin_cmd < self.min_speed:
#                 lin_cmd = self.min_speed

#             # Smooth commands (steering normalized, not degrees)
#             self.smoothed_linear = self.smoothed_linear * (1 - self.smoothing_alpha) + lin_cmd * self.smoothing_alpha
#             self.smoothed_steering = self.smoothed_steering * (1 - self.smoothing_alpha) + steering_norm * self.smoothing_alpha

#             self.get_logger().debug(f"Resultant angle: {math.degrees(resultant_angle):.1f}°, mag: {resultant_mag:.2f}")
#             self.get_logger().info(f"Cmds -> lin: {self.smoothed_linear:.3f}, steering_norm: {self.smoothed_steering:.3f} (norm), steering_deg: {self.smoothed_steering * self.max_steering_angle_deg:.1f}°")

#             # Publish normalized steering and speed in [min_speed, max_speed]
#             speed_cmd = Float32()
#             speed_cmd.data = float(self.smoothed_linear)
#             self.cmd_vel_pub.publish(speed_cmd)
#             # Also publish to the original topic for compatibility
#             try:
#                 self.cmd_vel_pub_raw.publish(speed_cmd)
#             except Exception:
#                 pass

#             dir_cmd = Float32()
#             dir_cmd.data = float(self.smoothed_steering)
#             self.cmd_dir_pub.publish(dir_cmd)
#             try:
#                 self.cmd_dir_pub_raw.publish(dir_cmd)
#             except Exception:
#                 pass

#         except Exception as e:
#             self.get_logger().error(f"Error in scan_callback: {str(e)}")
#             v = Float32(); v.data = 0.0; self.cmd_vel_pub.publish(v)
#             d = Float32(); d.data = 0.0; self.cmd_dir_pub.publish(d)

#     def emergency_callback(self, msg):
#         """Callback pour l'arrêt d'urgence de check_obstacle."""
#         self.emergency_stop = msg.data
#         if self.emergency_stop:
#             self.get_logger().warn("EMERGENCY STOP activated - Potential Field Navigator disabled")
#         else:
#             self.get_logger().info("Emergency stop cleared - Potential Field Navigator resumed")


# def main(args=None):
#     rclpy.init(args=args)
#     node = PotentialFieldNavigator()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
Potential Field based navigator for unknown circuits. 
- Combines an attractive forward vector and repulsive forces from lidar obstacles
- Produces smooth commands for navigation on unknown tracks
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from std_msgs.msg import Float32, Bool

class PotentialFieldNavigator(Node):
    def __init__(self):
        super().__init__('potential_field_navigator')

        # Variables
        self.max_speed = 0.4  # AUGMENTÉ: vitesse maximale
        self.min_speed = 0.015  # RÉDUIT: vitesse minimale
        self.max_steering_angle_deg = 30.0  # degrees
        self.influence_distance = 1.0  # m:  obstacles within this distance contribute
        self.k_repulsive = 0.3  # repulsive gain - conservé
        self.k_attractive = 0.6  # attractive gain - conservé
        self.smoothing_alpha = 0.4  # conservé
        
        # Nouveaux paramètres pour un meilleur contrôle de vitesse
        self.front_obstacle_threshold = 2.0  # distance à partir de laquelle on commence à ralentir (augmenté pour la vitesse plus élevée)
        self.critical_obstacle_distance = 0.5  # distance critique pour ralentissement maximal

        # Smoothed command state
        self.smoothed_linear = 0.0
        self.smoothed_steering = 0.0  # degrees

        # Publisher / Subscriber (namespaced to allow a navigation master to select algorithm outputs)
        self.cmd_vel_pub = self.create_publisher(Float32, '/potential_field/cmd_vel', 10)
        self.cmd_dir_pub = self.create_publisher(Float32, '/potential_field/cmd_dir', 10)
        # Also keep publishing on original topics for backward compatibility - you can comment these out if not needed
        self.cmd_vel_pub_raw = self.create_publisher(Float32, '/cmd_vel', 10)
        self.cmd_dir_pub_raw = self.create_publisher(Float32, '/cmd_dir', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Emergency stop subscription (Bool): published by check_obstacle/obstacle_checker
        self.emergency_stop = False
        self.emergency_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        self.get_logger().info('Potential Field Navigator started with speed range [0.015, 0.4] m/s')

    def scan_callback(self, msg:  LaserScan):
        try:
            if self.emergency_stop:
                self.get_logger().warn("Emergency stop active - skipping potential field processing")
                return
            # Compute repulsive field from obstacles
            rep_x = 0.0
            rep_y = 0.0
            min_front = float('inf')
            min_front_angle = 0.0

            for i, r in enumerate(msg.ranges):
                if not math.isfinite(r) or r <= 0.0:    # ignore invalid readings
                    continue
                # Calculate the angle for the current laser scan point
                angle = msg.angle_min + i * msg.angle_increment
                # Check if this range is the smallest in front (within 40 degrees of 180°)
                if r < min_front and abs(math.degrees(angle) - 180.0) < 40.0:
                    min_front = r   # update the closest front obstacle
                    min_front_angle = angle  # not used for now, maybe to add to have better avoidance? 

                if r < self.influence_distance:  # if the obstacle is within influence distance aka close enough to matter
                    # repulsive magnitude inverse-squared (limited)
                    mag = self.k_repulsive * (1.0 / (r + 1e-6) - 1.0 / self.influence_distance)
                    if mag < 0:
                        continue
                    fx = -mag * math.cos(angle)
                    fy = -mag * math.sin(angle)
                    rep_x += fx
                    rep_y += fy

            # Attractive force forward along x-axis (towards 180 degrees aka the front of the car)
            att_x = self.k_attractive
            att_y = 0.0

            total_x = att_x + rep_x
            total_y = att_y + rep_y

            # Compute resulting angle and magnitude
            resultant_angle = math. atan2(total_y, total_x)
            resultant_mag = math.hypot(total_x, total_y)

            # Convert to steering angle (degrees) based on resultant_angle
            steering_deg = math.degrees(resultant_angle)
            steering_deg = max(-self.max_steering_angle_deg, min(self.max_steering_angle_deg, steering_deg))

            # Normalize steering to [-1, 1]
            steering_norm = steering_deg / self.max_steering_angle_deg
            steering_norm = max(-1.0, min(1.0, steering_norm))

            # ===== NOUVELLE LOGIQUE DE VITESSE =====
            # Commencer avec la vitesse maximale par défaut
            lin_cmd = self.max_speed
            
            # Facteur 1:  Réduction basée sur la proximité des obstacles frontaux
            if min_front < self.front_obstacle_threshold:
                if min_front <= self.critical_obstacle_distance:
                    # Très proche:  utiliser min_speed
                    obstacle_factor = 0.0
                else:
                    # Interpolation progressive entre critical_distance et threshold
                    obstacle_factor = (min_front - self.critical_obstacle_distance) / \
                                    (self.front_obstacle_threshold - self.critical_obstacle_distance)
                    # Courbe quadratique pour une transition plus douce
                    obstacle_factor = obstacle_factor ** 0.7
                
                lin_cmd = self.min_speed + (self. max_speed - self.min_speed) * obstacle_factor
            
            # Facteur 2: Réduction basée sur l'angle de braquage (réduction modérée)
            # Réduction progressive:  0% à steering=0, jusqu'à 50% à steering=max
            steering_factor = 1.0 - 0.5 * abs(steering_norm)
            lin_cmd *= steering_factor

            # S'assurer que la vitesse reste dans les limites strictes
            lin_cmd = max(self.min_speed, min(self.max_speed, lin_cmd))

            # Smooth commands (steering normalized, not degrees)
            self.smoothed_linear = self.smoothed_linear * (1 - self.smoothing_alpha) + lin_cmd * self.smoothing_alpha
            self.smoothed_steering = self. smoothed_steering * (1 - self.smoothing_alpha) + steering_norm * self.smoothing_alpha

            self.get_logger().debug(f"Resultant angle: {math.degrees(resultant_angle):.1f}°, mag: {resultant_mag:.2f}")
            self.get_logger().info(f"Cmds -> lin: {self.smoothed_linear:.3f} m/s, steering_norm: {self.smoothed_steering:.3f}, min_front: {min_front:.2f}m")

            # Publish normalized steering and speed in [min_speed, max_speed]
            speed_cmd = Float32()
            speed_cmd.data = float(self.smoothed_linear)
            self.cmd_vel_pub. publish(speed_cmd)
            # Also publish to the original topic for compatibility
            try:
                self. cmd_vel_pub_raw. publish(speed_cmd)
            except Exception:
                pass

            dir_cmd = Float32()
            dir_cmd.data = float(self.smoothed_steering)
            self.cmd_dir_pub.publish(dir_cmd)
            try:
                self. cmd_dir_pub_raw. publish(dir_cmd)
            except Exception:
                pass

        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {str(e)}")
            v = Float32(); v.data = 0.0; self.cmd_vel_pub.publish(v)
            d = Float32(); d.data = 0.0; self.cmd_dir_pub.publish(d)

    def emergency_callback(self, msg):
        """Callback pour l'arrêt d'urgence de check_obstacle."""
        self. emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn("EMERGENCY STOP activated - Potential Field Navigator disabled")
        else:
            self.get_logger().info("Emergency stop cleared - Potential Field Navigator resumed")


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()