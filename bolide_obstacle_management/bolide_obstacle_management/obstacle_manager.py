import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class ObstacleManager(Node):

    def __init__(self):
        super().__init__('obstacle_manager')

        # Subscriber : écoute l'état de l'obstacle
        self.subscription = self.create_subscription(
            Bool,
            'obstacle_detected',  # à adapter selon ton capteur
            self.obstacle_callback,
            10
        )

        # Publisher : publie des commandes de direction
        self.publisher_ = self.create_publisher(String, 'cmd_dir', 10)

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
