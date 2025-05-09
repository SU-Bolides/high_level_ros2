import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# WORK IN PROGRESS - NEVER BEEN TESTED AND NOT FINISH AND NEED TO BE FIX
class ObstacleManager(Node):

    def __init__(self):
        super().__init__('obstacle_manager')

        # OUBLISHER AND SUBSCRIBER
        self.subscription = self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)

        self.publisher_ = self.create_publisher(String, '/cmd_dir', 10)

    def obstacle_callback(self, msg):
        if msg.data:
            self.get_logger().info('Obstacle détecté : on recule')
            command = String()
            command.data = 'backward'
            self.publisher_.publish(command)
        else:
            self.get_logger().info('Aucun obstacle : on avance')
            command = String()
            command.data = 'forward'
            self.publisher_.publish(command)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
