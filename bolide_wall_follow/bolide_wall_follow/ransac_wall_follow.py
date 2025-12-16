#!/usr/bin/env python3
"""
RANSAC-based wall following node with improved PID control.
- Fits a line to LIDAR points on the chosen wall side using RANSAC
- Computes perpendicular distance and wall angle, applies PID + feed-forward
- Publishes steering (degrees) on /cmd_dir (Float32) and speed (normalized) on /cmd_vel (Float32)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math
import random
import numpy as np

class RansacWallFollow(Node):
    def __init__(self):
        super().__init__('ransac_wall_follow')

        # Tunable variables (not ROS parameters per request)
        self.kp = 12.0
        self.ki = 0.0
        self.kd = 1.5
        self.kff = 5.0  # feed-forward gain (degrees per rad of wall angle)
        self.target_distance = 0.30  # target distance to wall (m)
        self.wall_side = 90.0  # degrees (90=left, 270=right)
        self.scan_window = 40.0  # degrees around wall_side to collect points
        self.ransac_iters = 150
        self.ransac_threshold = 0.04  # meters
        self.ransac_min_inliers = 20
        self.max_steering_angle = 45.0  # degrees
        self.max_speed = 0.04
        self.min_speed = 0.01
        self.lowpass_alpha = 0.3  # smoothing for measured distance/angle

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

        # Smoothed measurements
        self.smoothed_distance = None
        self.smoothed_angle = None  # radians

        # Publishers
        self.dir_pub = self.create_publisher(Float32, '/cmd_dir', 10)
        self.vel_pub = self.create_publisher(Float32, '/cmd_vel', 10)

        # Subscriber
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.get_logger().info('RANSAC Wall Follow node started')

    # ----------------- RANSAC line fit -----------------
    def ransac_line(self, points):
        """Fit a 2D line using RANSAC. Points is an Nx2 numpy array."""
        if points.shape[0] < 2:
            return None

        best_inliers = []
        best_model = None

        for _ in range(self.ransac_iters):
            # pick two random distinct indices
            i, j = random.sample(range(points.shape[0]), 2)
            p1 = points[i]
            p2 = points[j]
            denom = np.hypot(*(p2 - p1))
            if denom == 0:
                continue

            # Distance from each point to the line through p1-p2
            # cross product magnitude / denom
            diffs = points - p1
            cross = np.abs(diffs[:,0] * (p2[1]-p1[1]) - diffs[:,1] * (p2[0]-p1[0]))
            dists = cross / denom

            inliers = np.where(dists <= self.ransac_threshold)[0]
            if inliers.size > len(best_inliers):
                best_inliers = inliers
                best_model = (p1, p2)

        if best_model is None or len(best_inliers) < self.ransac_min_inliers:
            return None

        # Refit line to inliers using least squares y = m x + b
        inlier_pts = points[best_inliers]
        xs = inlier_pts[:,0]
        ys = inlier_pts[:,1]
        m, b = np.polyfit(xs, ys, 1)
        # return slope m and intercept b
        return m, b, inlier_pts

    # ----------------- Helpers -----------------
    def smooth(self, prev, new):
        if prev is None:
            return new
        return prev * (1 - self.lowpass_alpha) + new * self.lowpass_alpha

    # ----------------- Main scan callback -----------------
    def scan_callback(self, msg: LaserScan):
        # Build index function
        angle_min_deg = math.degrees(msg.angle_min)
        angle_inc_deg = math.degrees(msg.angle_increment)

        def get_index(angle_deg):
            a = angle_deg
            if a < angle_min_deg:
                a += 360.0
            idx = int((a - angle_min_deg) / angle_inc_deg)
            return max(0, min(len(msg.ranges)-1, idx))

        # Collect points in window around wall_side
        start_angle = self.wall_side - self.scan_window
        end_angle = self.wall_side + self.scan_window
        angles = []
        ranges = []
        for ang in np.arange(start_angle, end_angle, angle_inc_deg):
            idx = get_index(ang)
            r = msg.ranges[idx]
            if math.isfinite(r) and r > 0 and r <= msg.range_max:
                theta = math.radians(ang)
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                ranges.append((x, y))

        if len(ranges) < 10:
            self.get_logger().warn('Not enough points to fit wall')
            # fallback: publish small correction based on previous
            self.publish_controls(0.0, self.max_speed)
            return

        pts = np.array(ranges)
        model = self.ransac_line(pts)
        if model is None:
            self.get_logger().warn('RANSAC failed to find a line')
            self.publish_controls(0.0, self.max_speed)
            return

        m, b, inliers = model
        # distance from origin to line y = m x + b: |b|/sqrt(m^2 + 1)
        wall_distance = abs(b) / math.hypot(m, 1.0)
        # compute line angle (radians) from x-axis
        line_angle = math.atan(m)
        # wall_point_y_mean sign tells left/right (positive y -> left side)
        mean_y = np.mean(inliers[:,1])
        side = 1.0 if mean_y > 0 else -1.0

        # perpendicular angle relative to forward
        wall_perp_angle = line_angle + (math.pi/2.0) * side

        # Smooth measurements
        self.smoothed_distance = self.smooth(self.smoothed_distance, wall_distance)
        self.smoothed_angle = self.smooth(self.smoothed_angle, line_angle)

        # PID control
        error = self.target_distance - self.smoothed_distance
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 1e-3
        self.integral += error * dt
        self.integral = max(-10.0, min(10.0, self.integral))
        derivative = (error - self.prev_error) / dt

        pid_out = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        self.prev_time = current_time

        # feed-forward from wall orientation (convert to degrees)
        ff = self.kff * math.degrees(line_angle)

        direction_deg = pid_out + ff
        direction_deg = max(-self.max_steering_angle, min(self.max_steering_angle, direction_deg))

        # Speed scheduling: reduce speed if large steering or front obstacle
        front_idx = get_index(180.0)
        front_dist = msg.ranges[front_idx]
        if not math.isfinite(front_dist) or front_dist > msg.range_max:
            front_dist = 10.0

        speed = self.max_speed
        # penalize for large steering
        speed *= max(0.2, 1.0 - abs(direction_deg) / (self.max_steering_angle * 1.0))
        # penalize for close front obstacle
        if front_dist < 1.0:
            speed *= max(0.2, front_dist / 1.0)
        speed = max(self.min_speed, min(self.max_speed, speed))

        self.get_logger().info(f"Wall dist: {self.smoothed_distance:.3f} m, wall angle: {math.degrees(line_angle):.1f}°, dir: {direction_deg:.2f}°, speed: {speed:.3f}")

        self.publish_controls(direction_deg, speed)

    def publish_controls(self, direction_deg: float, speed: float):
        dmsg = Float32()
        dmsg.data = float(direction_deg)
        self.dir_pub.publish(dmsg)

        vmsg = Float32()
        vmsg.data = float(speed)
        self.vel_pub.publish(vmsg)


def main(args=None):
    rclpy.init(args=args)
    node = RansacWallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
