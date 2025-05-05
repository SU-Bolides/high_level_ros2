from math import pi, cos, sin, atan, isnan
import time

import rclpy
from rclpy.node import Node

from bolide_interfaces.msg import SpeedDirection
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

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
        self.declare_parameter('kp', 14.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 1.0)


        # PID CONTROL PARAMS
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.servo_offset = 0.0
        self.prev_error = 0.0
        self.error = 0.0
        self.integral = 0.0
        self.start_t = -1
        self.curr_t = 0.0
        self.prev_t = 0.0

        # PUBLISHER AND SUBSCRIBER
        self.pub_speed = self.create_publisher(Float32, '/cmd_vel', 10)
        self.pub_dir = self.create_publisher(Float32, '/cmd_dir', 10)

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def get_error(self, scan_msg, dist):
        a = get_range(scan_msg, to_radians(-50.0))
        b = get_range(scan_msg, to_radians(-90.0))
        theta = to_radians(40.0)  # 90-50 = 40°
        alpha = atan((a * cos(theta) - b)/(a * sin(theta)))
        D_t = b * cos(alpha)
        self.get_logger().info(f"D_t : {D_t}")

        self.prev_error = self.error
        self.error = dist - D_t
        self.integral += self.error
        self.prev_t = self.curr_t
        self.curr_t = scan_msg.header.stamp.nanosec * 10e-9 + scan_msg.header.stamp.sec
        if self.start_t == 0.0:
            self.start_t = self.curr_t
        self.get_logger().info(f"error is {self.error}, a {a} et b {b}")
    def pid_control(self):
        a = time.time()
        angle = 0.0
        if self.prev_t == 0:
            return
        angle = self.kp * self.error + self.ki * self.integral * (self.curr_t - self.start_t) + self.kd * (self.error - self.prev_error)/(self.curr_t - self.prev_t)
        self.get_logger().info(f"angle : {angle}")
        drive_msg = Float32()
        speed_msg = Float32()
        drive_msg.data = angle/to_radians(15.0)
        if drive_msg.data < 0:
            drive_msg.data = max(-1.0, drive_msg.data)
        else:
            drive_msg.data = min(1.0, drive_msg.data)
        self.get_logger().info(f"drive direction {drive_msg.data}")
        # TODO check this
        if (abs(drive_msg.data) >= 0.0 and abs(drive_msg.data) < 0.5):
            speed_msg.data = 0.1
        elif (abs(drive_msg.data) >= 0.5 and abs(drive_msg.data) <= 1.0):
            speed_msg.data = 0.12
        else:
            speed_msg.data = 0.05
        self.pub_dir.publish(drive_msg)
        self.pub_speed.publish(speed_msg)

    def scan_callback(self, scan_msg):
        self.get_error(scan_msg, 0.8)
        self.pid_control()

def main(args=None):
    rclpy.init(args=args)
    wall_follow = WallFollow()
    rclpy.spin(wall_follow)
    rclpy.shutdown()
