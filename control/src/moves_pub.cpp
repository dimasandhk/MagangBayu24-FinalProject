#include <iostream>
#include <string>
#include <cstdlib>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class PlayerNode : public rclcpp::Node {
public:
  PlayerNode() : Node("player_node") {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("player_move_topic", 10);
        
    while (true) {
      int player_move = getPlayerMove();
      if (player_move >= 1 && player_move <= 9) {
        auto message = std::make_unique<std_msgs::msg::Int32>();
        message->data = player_move;
        publisher_->publish(std::move(message));
        break;  // Exit loop after successful move
      } else {
        std::cout << "Invalid move. Please enter a number between 1 and 9." << std::endl;
      }
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  int getPlayerMove() {
    std::cout << "Enter your move (1-9): ";
    std::string input;
    std::getline(std::cin, input);
    return std::atoi(input.c_str());
  }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlayerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
