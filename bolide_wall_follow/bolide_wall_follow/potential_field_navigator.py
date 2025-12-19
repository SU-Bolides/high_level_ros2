#!/usr/bin/env python3
"""
Potential Field based navigator for unknown circuits.
- Combines an attractive forward vector and repulsive forces from lidar obstacles
- Produces smooth Twist commands for navigation on unknown tracks
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

        # Configuration variables
        self.max_speed = 0.06  # max normalized speed (0-1)
        self.min_speed = 0.02  # minimum forward speed to avoid stopping
        self.max_steering_angle_deg = 45.0  # degrees
        self.influence_distance = 2.0  # m: obstacles within this distance contribute
        self.k_repulsive = 0.8
        self.k_attractive = 1.0
        self.stop_distance = 0.25
        self.smoothing_alpha = 0.25

        # Smoothed command state
        self.smoothed_linear = 0.0
        self.smoothed_steering = 0.0  # degrees

        # Publisher / Subscriber (Float32 messages)
        self.cmd_vel_pub = self.create_publisher(Float32, '/cmd_vel', 10)
        self.cmd_dir_pub = self.create_publisher(Float32, '/cmd_dir', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Emergency stop subscription (Bool): published by check_obstacle/obstacle_checker
        self.emergency_stop = False
        self.emergency_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        self.get_logger().info('Potential Field Navigator started')

    def scan_callback(self, msg: LaserScan):
        try:
            if self.emergency_stop:
                self.get_logger().warn("Emergency stop active - skipping potential field processing")
                v = Float32(); v.data = 0.0; self.cmd_vel_pub.publish(v)
                d = Float32(); d.data = 0.0; self.cmd_dir_pub.publish(d)
                return
            # Compute repulsive field from obstacles
            rep_x = 0.0
            rep_y = 0.0
            min_front = float('inf')
            min_front_angle = 0.0

            for i, r in enumerate(msg.ranges):
                if not math.isfinite(r) or r <= 0.0:
                    continue
                angle = msg.angle_min + i * msg.angle_increment
                if r < min_front and abs(math.degrees(angle) - 180.0) < 40.0:
                    min_front = r
                    min_front_angle = angle

                if r < self.influence_distance:
                    # repulsive magnitude inverse-squared (limited)
                    mag = self.k_repulsive * (1.0 / (r + 1e-6) - 1.0 / self.influence_distance)
                    if mag < 0:
                        continue
                    fx = -mag * math.cos(angle)
                    fy = -mag * math.sin(angle)
                    rep_x += fx
                    rep_y += fy

            # Attractive force forward
            att_x = self.k_attractive
            att_y = 0.0

            total_x = att_x + rep_x
            total_y = att_y + rep_y

            # Compute resulting angle and magnitude
            resultant_angle = math.atan2(total_y, total_x)
            resultant_mag = math.hypot(total_x, total_y)

            # Safety stop triggered by very close obstacle -> publish /emergency_stop for ObstacleChecker
            if min_front < self.stop_distance:
                self.get_logger().error(f"EMERGENCY STOP: obstacle at {min_front:.2f} m in front")
                em = Bool(); em.data = True
                self.pub_emergency.publish(em)
                # publish immediate halt locally
                v = Float32(); v.data = 0.0; self.cmd_vel_pub.publish(v)
                d = Float32(); d.data = 0.0; self.cmd_dir_pub.publish(d)
                return

            # Convert to steering angle (degrees) based on resultant_angle
            steering_deg = math.degrees(resultant_angle)
            steering_deg = max(-self.max_steering_angle_deg, min(self.max_steering_angle_deg, steering_deg))

            # Normalize steering to [-1, 1] to follow teleop logic
            steering_norm = steering_deg / self.max_steering_angle_deg
            steering_norm = max(-1.0, min(1.0, steering_norm))

            # Linear command: map resultant magnitude to [min_speed, max_speed]
            mag_fraction = resultant_mag / (1.0 + resultant_mag)
            lin_cmd = self.min_speed + (self.max_speed - self.min_speed) * mag_fraction

            # Reduce speed when turning sharply or when front obstacle is near
            lin_cmd *= max(0.2, 1.0 - abs(steering_norm))
            if min_front < 1.0:
                lin_cmd *= min_front / 1.0

            # Ensure within bounds
            lin_cmd = max(0.0, min(self.max_speed, lin_cmd))
            if lin_cmd > 0 and lin_cmd < self.min_speed:
                lin_cmd = self.min_speed

            # Smooth commands (steering normalized, not degrees)
            self.smoothed_linear = self.smoothed_linear * (1 - self.smoothing_alpha) + lin_cmd * self.smoothing_alpha
            self.smoothed_steering = self.smoothed_steering * (1 - self.smoothing_alpha) + steering_norm * self.smoothing_alpha

            self.get_logger().debug(f"Resultant angle: {math.degrees(resultant_angle):.1f}°, mag: {resultant_mag:.2f}")
            self.get_logger().info(f"Cmds -> lin: {self.smoothed_linear:.3f}, steering_norm: {self.smoothed_steering:.3f} (norm), steering_deg: {self.smoothed_steering * self.max_steering_angle_deg:.1f}°")

            # Publish normalized steering and small-range speed in [min_speed, max_speed]
            v = Float32(); v.data = float(self.smoothed_linear); self.cmd_vel_pub.publish(v)
            d = Float32(); d.data = float(self.smoothed_steering); self.cmd_dir_pub.publish(d)

        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {str(e)}")
            v = Float32(); v.data = 0.0; self.cmd_vel_pub.publish(v)
            d = Float32(); d.data = 0.0; self.cmd_dir_pub.publish(d)

    def emergency_callback(self, msg):
        """Callback pour l'arrêt d'urgence de check_obstacle.
        We simply update the local flag and defer recovery handling to the ObstacleChecker node (which publishes on /cmd_vel and /cmd_dir during its reversing sequence).
        """
        prev = self.emergency_stop
        self.emergency_stop = bool(msg.data)
        if self.emergency_stop:
            self.get_logger().warn("EMERGENCY STOP activated - deferring to ObstacleChecker")
        else:
            if prev:
                self.get_logger().info("Emergency stop cleared - resuming potential field navigation")
            else:
                self.get_logger().info("Emergency stop cleared")


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
