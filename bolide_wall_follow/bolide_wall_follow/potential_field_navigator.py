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
from std_msgs.msg import Float32

class PotentialFieldNavigator(Node):
    def __init__(self):
        super().__init__('potential_field_navigator')

        # Configuration variables
        self.max_speed = 0.6  # m/s
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

        self.get_logger().info('Potential Field Navigator started')

    def scan_callback(self, msg: LaserScan):
        try:
            # Compute repulsive field from obstacles
            rep_x = 0.0
            rep_y = 0.0
            min_front = float('inf')

            for i, r in enumerate(msg.ranges):
                if not math.isfinite(r) or r <= 0.0:
                    continue
                angle = msg.angle_min + i * msg.angle_increment
                if r < min_front and abs(math.degrees(angle) - 180.0) < 40.0:
                    min_front = r

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

            # Safety stop
            if min_front < self.stop_distance:
                self.get_logger().error(f"EMERGENCY STOP: obstacle at {min_front:.2f} m in front")
                self.publish_floats(0.0, 0.0)
                return

            # Convert to steering angle (degrees) based on resultant_angle
            steering_deg = math.degrees(resultant_angle)
            steering_deg = max(-self.max_steering_angle_deg, min(self.max_steering_angle_deg, steering_deg))

            # Linear command proportional to resultant magnitude
            lin_cmd = max(0.0, min(self.max_speed, self.max_speed * (resultant_mag / (1.0 + resultant_mag))))

            # Reduce speed when turning sharply or when front obstacle is near
            lin_cmd *= max(0.2, 1.0 - abs(steering_deg) / self.max_steering_angle_deg)
            if min_front < 1.0:
                lin_cmd *= min_front / 1.0

            # Smooth commands
            self.smoothed_linear = self.smoothed_linear * (1 - self.smoothing_alpha) + lin_cmd * self.smoothing_alpha
            self.smoothed_steering = self.smoothed_steering * (1 - self.smoothing_alpha) + steering_deg * self.smoothing_alpha

            self.get_logger().debug(f"Resultant angle: {math.degrees(resultant_angle):.1f}°, mag: {resultant_mag:.2f}")
            self.get_logger().info(f"Cmds -> lin: {self.smoothed_linear:.2f} m/s, steering: {self.smoothed_steering:.1f}°")

            self.publish_floats(self.smoothed_linear, self.smoothed_steering)

        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {str(e)}")
            self.publish_floats(0.0, 0.0)

    def publish_floats(self, linear, steering_deg):
        v = Float32()
        v.data = float(linear)
        self.cmd_vel_pub.publish(v)

        d = Float32()
        d.data = float(steering_deg)
        self.cmd_dir_pub.publish(d)


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
