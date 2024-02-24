import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import cv2

# tilecount: 9 = (645, 766)
# tilecount: 8 = (377, 766)
# tilecount: 7 = (95, 766)
# tilecount: 6 = (645, 497)
# tilecount: 5 = (377, 497)
# tilecount: 4 = (95, 497)
# tilecount: 3 = (645, 215)
# tilecount: 2 = (377, 215)
# tilecount: 1 = (95, 215)

class GUINode(Node):
    pos_x = [95, 377, 645, 95, 377, 645, 95, 377, 645]
    pos_y = [215, 215, 215, 497, 497, 497, 766, 766, 766]
    comp_moves = []
    player_moves = []

    def __init__(self):
        super().__init__('my_subscriber')
        self.computer_subscriber = self.create_subscription(Int32, 'computer_move_topic', self.computer_callback, 10)
        self.player_subscriber = self.create_subscription(Int32, 'player_move_topic', self.player_callback, 10)
        image_path = 'src/MagangBayu24-FinalProject/vision/resource/board.png'
        print(image_path)
        self.image = cv2.imread(image_path)
        self.game_over = False

    def computer_callback(self, msg):
        if not self.game_over:
            self.comp_moves.append(msg.data)
            self.get_logger().info('Received computer move: %d' % msg.data)
            self.put_text(msg.data, 'c')
            self.check_game()

    def player_callback(self, msg):
        if not self.game_over:
            self.player_moves.append(msg.data)
            self.get_logger().info('Received player move: %d' % msg.data)
            self.put_text(msg.data, 'p')
            self.check_game()

    def put_text(self, tilecount, move):
        tile_info = ['X', (0, 0, 255)] if move == 'p' else ['O', (255, 0, 0)]
        cv2.putText(
            self.image, 
            tile_info[0], 
            (self.pos_x[tilecount - 1], self.pos_y[tilecount - 1]), 
            cv2.FONT_HERSHEY_SIMPLEX, 
            5, 
            tile_info[1], 
            10
        )
        # Display the image with text
        cv2.imshow("Tictactoe", self.image)
        cv2.waitKey(500)
    
    def check_game(self):
        if len(self.player_moves) + len(self.comp_moves) >= 9:
            self.game_over = True
            return

        winning_combinations = [
            [1, 2, 3], [4, 5, 6], [7, 8, 9],
            [1, 4, 7], [2, 5, 8], [3, 6, 9],
            [1, 5, 9], [3, 5, 7]
        ]

        for combo in winning_combinations:
            if all(move in self.comp_moves for move in combo) or all(move in self.player_moves for move in combo):
                self.game_over = True
                return

        self.game_over = False


def main(args=None):
    rclpy.init(args=args)
    node = GUINode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
