import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.computer_subscriber = self.create_subscription(Int32, 'computer_move_topic', self.computer_callback, 10)
        self.player_subscriber = self.create_subscription(Int32, 'player_move_topic', self.player_callback, 10)

    def computer_callback(self, msg):
        self.get_logger().info('Received computer move: %d' % msg.data)

    def player_callback(self, msg):
        self.get_logger().info('Received player move: %d' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
