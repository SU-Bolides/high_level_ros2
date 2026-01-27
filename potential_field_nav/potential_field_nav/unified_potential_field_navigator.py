#!/usr/bin/env python3
"""
Unified Potential Field Navigator combining all three versions:
- Base potential field functionality
- Threshold-based speed control for progressive slowdown
- Steering smoothing with deadzone to reduce oscillations when centered
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from std_msgs.msg import Float32, Bool

class UnifiedPotentialFieldNavigator(Node):

    def __init__(self):
        super().__init__('unified_potential_field_navigator')

        # Declare parameters
        self.declare_parameter('max_speed', 0.04)
        self.declare_parameter('min_speed', 0.015)
        self.declare_parameter('max_steering_angle_deg', 40.0)
        self.declare_parameter('influence_distance', 4.0)
        self.declare_parameter('k_repulsive', 0.4)
        self.declare_parameter('k_attractive', 1.0)
        self.declare_parameter('smoothing_alpha', 0.3)
        self.declare_parameter('front_obstacle_threshold', 2.0)
        self.declare_parameter('critical_obstacle_distance', 0.5)
        self.declare_parameter('repulsive_deadzone', 100.0)

        # Get parameter values
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_steering_angle_deg = self.get_parameter('max_steering_angle_deg').value
        self.influence_distance = self.get_parameter('influence_distance').value
        self.k_repulsive = self.get_parameter('k_repulsive').value
        self.k_attractive = self.get_parameter('k_attractive').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.front_obstacle_threshold = self.get_parameter('front_obstacle_threshold').value
        self.critical_obstacle_distance = self.get_parameter('critical_obstacle_distance').value
        self.repulsive_deadzone = self.get_parameter('repulsive_deadzone').value

        # Smoothed state
        self.smoothed_linear = 0.0
        self.smoothed_steering = 0.0

        # Publishers / Subscribers
        self.cmd_vel_pub = self.create_publisher(Float32, '/unified_pf/cmd_vel', 10)
        self.cmd_dir_pub = self.create_publisher(Float32, '/unified_pf/cmd_dir', 10)
        self.cmd_vel_pub_raw = self.create_publisher(Float32, '/cmd_vel', 10)
        self.cmd_dir_pub_raw = self.create_publisher(Float32, '/cmd_dir', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Emergency stop subscription
        self.emergency_stop = False
        self.emergency_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        self.get_logger().info('Unified Potential Field Navigator started')

    def apply_steering_deadzone(self, repulsive_magnitude, steering_norm):
        """
        Apply deadzone to steering to reduce oscillations when car is centered.
        Small repulsive forces produce reduced steering response.
        
        Args:
            repulsive_magnitude: Magnitude of the repulsive force vector
            steering_norm: Normalized steering command [-1, 1]
            
        Returns:
            Scaled steering command with deadzone applied
        """
        if abs(repulsive_magnitude) <= self.repulsive_deadzone:
            # Inside deadzone: quadratic scaling for smooth fade
            scale = (abs(repulsive_magnitude) / self.repulsive_deadzone) ** 2
        else:
            # Outside deadzone: full steering response
            scale = 1.0
        
        return steering_norm * scale

    def scan_callback(self, msg: LaserScan):

        try:
            if self.emergency_stop:
                self.get_logger().warn("Emergency stop active - skipping processing")
                return

            # Convert to numpy arrays for efficient vectorized processing
            ranges = np.array(msg.ranges, dtype=float)
            angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment

            # Filter valid readings : we don't want to consider invalid or zero readings
            valid_mask = np.isfinite(ranges) & (ranges > 0.0)

            # Find minimum front obstacle (±90° around 180°) : you can try different values
            angles_deg = np.degrees(angles)
            front_mask = np.abs(angles_deg - 180.0) < 90.0
            front_valid = ranges[valid_mask & front_mask]
            min_front = float(np.min(front_valid)) if front_valid.size > 0 else float('inf')

            # Compute repulsive forces from nearby obstacles
            influence_mask = valid_mask & (ranges < self.influence_distance)
            rep_x = 0.0
            rep_y = 0.0
            
            if np.any(influence_mask):
                r_infl = ranges[influence_mask]
                a_infl = angles[influence_mask]
                eps = 1e-6
                # Inverse-squared repulsive magnitude
                mags = self.k_repulsive * (1.0 / (r_infl + eps) - 1.0 / self.influence_distance)
                mags = np.clip(mags, 0.0, None)
                fx = -mags * np.cos(a_infl)     # forces on x axis
                fy = -mags * np.sin(a_infl)     # forces on y axis
                rep_x = float(np.sum(fx))       # sum of repulsive forces on both axis
                rep_y = float(np.sum(fy))

            # Attractive force forward along x-axis
            att_x = self.k_attractive   # we choose a high gain compared to repulsive gain to encourage the car to move forward
            att_y = 0.0     # no forces on y axis because we want the car to go straight not zigzag from the start

            # Resultant force vector
            total_x = att_x + rep_x
            total_y = att_y + rep_y
            resultant_angle = math.atan2(total_y, total_x)      # trigonometry formulas lol
            resultant_mag = math.hypot(total_x, total_y)
            
            # Calculate repulsive magnitude for just after when we use the deadzone
            repulsive_mag = math.hypot(rep_x, rep_y)

            # Convert to steering angle (degrees) and normalize bewteen [-1;1]
            steering_deg = math.degrees(resultant_angle)
            steering_deg = max(-self.max_steering_angle_deg, min(self.max_steering_angle_deg, steering_deg))    # clamp to ensure it's in bounds
            steering_norm = steering_deg / self.max_steering_angle_deg  # normalize by dividing by the max

            # Apply steering deadzone to reduce oscillations when centered
            steering_with_deadzone = self.apply_steering_deadzone(repulsive_mag, steering_norm)

            # Start with max speed by default
            lin_cmd = self.max_speed

            # Progressive slowdown based on front obstacle proximity
            if min_front < self.front_obstacle_threshold:
                if min_front <= self.critical_obstacle_distance:
                    # Very close: use minimum speed
                    obstacle_factor = 0.0
                else:
                    # Progressive interpolation between critical and threshold distances
                    obstacle_factor = (min_front - self.critical_obstacle_distance) / (self.front_obstacle_threshold - self.critical_obstacle_distance)
                    # Smooth curve for gentle transition
                    obstacle_factor = obstacle_factor ** 0.9

                lin_cmd = self.min_speed + (self.max_speed - self.min_speed) * obstacle_factor

                # Additional reduction based on steering angle when obstacle is close
                steering_factor = 1.0 - 0.4 * abs(steering_with_deadzone)
                lin_cmd *= steering_factor
            else:
                # No close obstacle: maintain higher speed with minimal steering reduction
                steering_factor = 1.0 - 0.2 * abs(steering_with_deadzone)
                lin_cmd *= steering_factor

            # Ensure speed stays within bounds
            lin_cmd = max(self.min_speed, min(self.max_speed, lin_cmd))

            # Smooth commands with exponential filtering
            self.smoothed_linear = (1 - self.smoothing_alpha) * self.smoothed_linear + self.smoothing_alpha * lin_cmd
            self.smoothed_steering = (1 - self.smoothing_alpha) * self.smoothed_steering + self.smoothing_alpha * steering_with_deadzone

            # Log debug information
            self.get_logger().info(
                f"UnifiedPF -> front:{min_front:.2f}m, rep_mag:{repulsive_mag:.3f}, "
                f"lin:{self.smoothed_linear:.3f}, steer:{self.smoothed_steering:.3f} "
                f"({self.smoothed_steering * self.max_steering_angle_deg:.1f}°)"
            )

            # Publish commands
            speed_msg = Float32()
            speed_msg.data = float(self.smoothed_linear)
            dir_msg = Float32()
            dir_msg.data = float(self.smoothed_steering)
            
            self.cmd_vel_pub.publish(speed_msg)
            self.cmd_dir_pub.publish(dir_msg)
            
            try:
                self.cmd_vel_pub_raw.publish(speed_msg)
                self.cmd_dir_pub_raw.publish(dir_msg)
            except Exception:
                pass

        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {str(e)}")
            zero = Float32()
            zero.data = 0.0
            self.cmd_vel_pub.publish(zero)
            self.cmd_dir_pub.publish(zero)

    def emergency_callback(self, msg):
        """Callback pour l'arrêt d'urgence de check_obstacle."""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn("EMERGENCY STOP activated - Unified Potential Field Navigator disabled")
        else:
            self.get_logger().info("Emergency stop cleared - Unified Potential Field Navigator resumed")


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedPotentialFieldNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
