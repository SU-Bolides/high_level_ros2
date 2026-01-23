#!/usr/bin/env python3
"""
Master node for navigation: selects among algorithms (potential_field, wall_follow, camera3d) and republishes their commands on /cmd_vel and /cmd_dir.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool


class NavigationMaster(Node):
    def __init__(self):
        super().__init__('navigation_master')

        # Parameters
        # publish_rate: how often the master republishes the currently active algorithm's outputs
        self.declare_parameter('publish_rate', 10.0)
        # active_algorithm: which algorithm to use for control (string)
        self.declare_parameter('active_algorithm', 'potential_field')

        self.active_algorithm = str(self.get_parameter('active_algorithm').value)  # 'potential_field', 'wall_follow_pid', 'camera3d'
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        # Publishers: these publish the info on the topics that sends the real commands
        self.cmd_vel_pub = self.create_publisher(Float32, '/cmd_vel', 10)
        self.cmd_dir_pub = self.create_publisher(Float32, '/cmd_dir', 10)

        # Configure subscriptions based on the currently active algorithm
        self.vel_sub = None
        self.dir_sub = None
        self._configure_subscriptions(self.active_algorithm)

        # Keep latest values for each algorithm to be republished on /cmd_vel and /cmd_dir
        self.latest = {
            'potential_field': {'vel': None, 'dir': None},
            'wall_follow_pid': {'vel': None, 'dir': None},
            'camera3d': {'vel': None, 'dir': None},
        }

        # Emergency stop
        self.emergency_sub = self.create_subscription(Bool, '/emergency_stop', self.emergency_cb, 10)
        self.emergency_stop = False

        # Periodic timer to publish selected algorithm output
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)

        self.get_logger().info(f"Navigation Master started (active_algorithm={self.active_algorithm})")

    def _configure_subscriptions(self, alg: str):
        """(Re)configure subscriptions to match the selected algorithm.
        If a previous subscription exists, it will be destroyed first.
        """
        # Destroy previous subscriptions if present
        if getattr(self, 'vel_sub', None) is not None:
            try:
                self.destroy_subscription(self.vel_sub)
            except Exception:
                pass
            self.vel_sub = None
        if getattr(self, 'dir_sub', None) is not None:
            try:
                self.destroy_subscription(self.dir_sub)
            except Exception:
                pass
            self.dir_sub = None

        # Create new subscriptions according to the selected algorithm
        if alg == 'potential_field':
            self.vel_sub = self.create_subscription(Float32, '/potential_field/cmd_vel', self.pf_vel_cb, 10)
            self.dir_sub = self.create_subscription(Float32, '/potential_field/cmd_dir', self.pf_dir_cb, 10)
        elif alg == 'wall_follow_pid':
            self.vel_sub = self.create_subscription(Float32, '/wall_follow_pid/cmd_vel', self.wf_vel_cb, 10)
            self.dir_sub = self.create_subscription(Float32, '/wall_follow_pid/cmd_dir', self.wf_dir_cb, 10)
        elif alg == 'camera3d':
            self.vel_sub = self.create_subscription(Float32, '/camera3d/cmd_vel', self.cam_vel_cb, 10)
            self.dir_sub = self.create_subscription(Float32, '/camera3d/cmd_dir', self.cam_dir_cb, 10)
        else:
            self.get_logger().error(f'Unknown active_algorithm: {alg}')
            self.vel_sub = None
            self.dir_sub = None
            return

        self.active_algorithm = alg
        self.get_logger().info(f"Active algorithm set to '{alg}'")

    # Callbacks to record latest values from each algorithm
    def pf_vel_cb(self, msg: Float32):
        self.latest['potential_field']['vel'] = float(msg.data)

    def pf_dir_cb(self, msg: Float32):
        self.latest['potential_field']['dir'] = float(msg.data)

    def wf_vel_cb(self, msg: Float32):
        self.latest['wall_follow_pid']['vel'] = float(msg.data)

    def wf_dir_cb(self, msg: Float32):
        self.latest['wall_follow_pid']['dir'] = float(msg.data)

    def cam_vel_cb(self, msg: Float32):
        self.latest['camera3d']['vel'] = float(msg.data)

    def cam_dir_cb(self, msg: Float32):
        self.latest['camera3d']['dir'] = float(msg.data)

    def emergency_cb(self, msg: Bool):
        self.emergency_stop = bool(msg.data)
        # if self.emergency_stop:
        #     self.get_logger().warn('Emergency stop active')
        # else:
        #     self.get_logger().info('Emergency stop cleared')

    def timer_cb(self):
        # Publish active algorithm output (vel, dir)
        if self.emergency_stop:
            # We let the obstacle checker node take control to maneuver the car
            return

        alg = self.active_algorithm
        if alg not in self.latest:
            self.get_logger().error(f"No stored values for algorithm '{alg}'")
            return

        vel_val = self.latest[alg]['vel']
        dir_val = self.latest[alg]['dir']

        # Ensure numeric defaults
        if vel_val is None:
            vel_val = 0.0
        if dir_val is None:
            dir_val = 0.0

        vel_msg = Float32(); 
        vel_msg.data = float(vel_val)
        dir_msg = Float32(); 
        dir_msg.data = float(dir_val)

        self.cmd_vel_pub.publish(vel_msg)
        self.cmd_dir_pub.publish(dir_msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
