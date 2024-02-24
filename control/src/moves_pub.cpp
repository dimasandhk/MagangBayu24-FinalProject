#include <iostream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class PlayerNode : public rclcpp::Node {
public:
  PlayerNode() : Node("player_node") {
    // pub player move
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("player_move_topic", 10);
    subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
      "computer_move_topic", 10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        std::cout << "Masuk bg";
        int computer_move = msg->data;
        computer_moves_.push_back(computer_move);

        std::cout << "Computer's move: " << computer_move << std::endl;
      }
    );

    // ambil input angka
    while (rclcpp::ok()) {
      int player_move = getPlayerMove();
      if (player_move >= 1 && player_move <= 9 && !isMoveAlreadyMade(player_move)) {
        player_moves_.push_back(player_move);
        
        auto message = std::make_unique<std_msgs::msg::Int32>();
        message->data = player_move;
        publisher_->publish(std::move(message));
        
        std::cout << "Player's move: " << player_move << std::endl;
      } else {
        std::cout << "Invalid move. Please enter a number between 1 and 9, and don't input the same number" << std::endl;
      }
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
  std::vector<int> player_moves_;
  std::vector<int> computer_moves_;

  int getPlayerMove() { // prompt input
    std::cout << "Enter your move (1-9): ";
    std::string input;
    std::getline(std::cin, input);
    return std::atoi(input.c_str());
  }

  bool isMoveAlreadyMade(int move) {
    for (int existing_move : player_moves_) {
      std::cout << existing_move;
      if (existing_move == move) {
        return true;
      }
    }
    for (int existing_move : computer_moves_) {
      std::cout << existing_move;
      if (existing_move == move) {
        return true;
      }
    }
    return false;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlayerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
