import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# WORK IN PROGRESS - NEVER BEEN TESTED AND NOT FINISH AND NEED TO BE FIX

# We can may be see it like a process of front data of the lidar (maybe a 15° zone in front of the car) to check obstacle and where to go if needed
# If it can work with our controller it will be perfect for a beginning of a full autonomous car with obstacle avoidance
class ObstacleManager(Node):

    def __init__(self):
        super().__init__('obstacle_manager')

        # OUBLISHER AND SUBSCRIBER
        # TODO - change this sub in a pub to send it to cmd_vel_node or cmd_dir_node
        self.subscription = self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)

        self.publisher_ = self.create_publisher(String, '/cmd_dir', 10) # TODO - Why direction if after is forward or backward and a String message

    def obstacle_callback(self, msg):
        # TODO - rework here needed
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
    # will just destroy node here 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
