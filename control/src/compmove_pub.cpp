#include <iostream>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class ComputerNode : public rclcpp::Node {
public:
  ComputerNode() : Node("computer_node") {
    subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
      "player_move_topic",
      10,
      [this](const std_msgs::msg::Int32::SharedPtr msg) {
        // Player's move received
        int player_move = msg->data;
        player_moves_.push_back(player_move);
        std::cout << "Player's move: " << player_move << std::endl;
                
        // Calculate computer's move
        int computer_move = getComputerMove();
        computer_moves_.push_back(computer_move);
        std::cout << "Computer's move: " << computer_move << std::endl;
        for (int move: player_moves_) {
          std::cout << "P:" << move << ", ";
        }
        std::cout << std::endl;
        for (int move: computer_moves_) {
          std::cout << "C:" << move << ", ";
        }
        std::cout << std::endl << std::endl;

        // Publish computer's move
        auto message = std::make_unique<std_msgs::msg::Int32>();
        message->data = computer_move;
        publisher_->publish(std::move(message));
      }
    );

    publisher_ = this->create_publisher<std_msgs::msg::Int32>("computer_move_topic", 10);
  }

/*
1 2 3
4 5 6
7 8 9
*/

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  std::vector<int> player_moves_;
  std::vector<int> computer_moves_;

  bool isWinner(const std::vector<int>& moves) {
    std::vector<std::vector<int>> winningCombinations = {
      {1, 2, 3}, {4, 5, 6}, {7, 8, 9},
      {1, 4, 7}, {2, 5, 8}, {3, 6, 9},
      {1, 5, 9}, {3, 5, 7}
    };
    
    for (const auto& combo : winningCombinations) {
      bool isWinning = true;
      for (int move : combo) {
        if (find(moves.begin(), moves.end(), move) == moves.end()) {
          isWinning = false;
          break;
        }
      }
      if (isWinning) {
        return true;
      }
    }
    return false;
  }

  bool isForkingMove(int move, const std::vector<int>& moves) {
    std::vector<int> possibleMoves = moves;
    possibleMoves.push_back(move);
    int count = 0;
    std::vector<std::vector<int>> winningCombinations = {
      {1, 2, 3}, {4, 5, 6}, {7, 8, 9},
      {1, 4, 7}, {2, 5, 8}, {3, 6, 9},
      {1, 5, 9}, {3, 5, 7}
    };
    for (const auto& combo : winningCombinations) {
      bool isSubset = true;
      for (int move : combo) {
        if (find(possibleMoves.begin(), possibleMoves.end(), move) == possibleMoves.end()) {
          isSubset = false;
          break;
        }
      }
      if (isSubset) {
        count++;
      }
    }
    return count >= 2;
  }

  bool isBlockingFork(int move, const std::vector<int>& moves) {
    std::vector<int> possibleMoves = moves;
    possibleMoves.push_back(move);
    int count = 0;
    std::vector<std::vector<int>> winningCombinations = {
      {1, 2, 3}, {4, 5, 6}, {7, 8, 9},
      {1, 4, 7}, {2, 5, 8}, {3, 6, 9},
      {1, 5, 9}, {3, 5, 7}
    };
    for (const auto& combo : winningCombinations) {
      bool isSubset = true;
      for (int move : combo) {
        if (find(possibleMoves.begin(), possibleMoves.end(), move) == possibleMoves.end()) {
          isSubset = false;
          break;
        }
      }
      if (isSubset) {
        count++;
      }
    }
    return count < 2;
  }

  int getComputerMove() {
    // If both compMoves and player_moves_ are empty, return a random corner
    if (computer_moves_.empty() && player_moves_.empty()) {
      int randomCorner = rand() % 4 + 1; // Generate a random number between 1 and 4
      switch (randomCorner) {
        case 1: return 1;
        case 2: return 3;
        case 3: return 7;
        case 4: return 9;
      }
    }

    // Occupying the Center
    if (find(computer_moves_.begin(), computer_moves_.end(), 5) == computer_moves_.end() &&
      find(player_moves_.begin(), player_moves_.end(), 5) == player_moves_.end()) {
      return 5;
    }

    // Check for Winning Moves
    for (int move = 1; move <= 9; ++move) {
      if (find(computer_moves_.begin(), computer_moves_.end(), move) == computer_moves_.end() &&
        find(player_moves_.begin(), player_moves_.end(), move) == player_moves_.end()) {
        std::vector<int> possibleMoves = computer_moves_;
        possibleMoves.push_back(move);
        if (isWinner(possibleMoves)) {
          return move;
        }
      }
    }

    // Check for Blocking Moves
    for (int move = 1; move <= 9; ++move) {
      if (find(computer_moves_.begin(), computer_moves_.end(), move) == computer_moves_.end() &&
        find(player_moves_.begin(), player_moves_.end(), move) == player_moves_.end()) {
        std::vector<int> possibleMoves = player_moves_;
        possibleMoves.push_back(move);
        if (isWinner(possibleMoves)) {
          return move;
        }
      }
    }

    // Check for Forking Moves
    for (int move = 1; move <= 9; ++move) {
      if (find(computer_moves_.begin(), computer_moves_.end(), move) == computer_moves_.end() &&
        find(player_moves_.begin(), player_moves_.end(), move) == player_moves_.end()) {
        if (isForkingMove(move, computer_moves_)) {
          return move;
        }
      }
    }

    // Check for Blocking Forks
    for (int move = 1; move <= 9; ++move) {
      if (find(computer_moves_.begin(), computer_moves_.end(), move) == computer_moves_.end() &&
        find(player_moves_.begin(), player_moves_.end(), move) == player_moves_.end()) {
        if (isBlockingFork(move, player_moves_)) {
          return move;
        }
      }
    }

    // Occupying the Corners
    std::vector<int> corners = {1, 3, 7, 9};
    for (int corner : corners) {
      if (find(computer_moves_.begin(), computer_moves_.end(), corner) == computer_moves_.end() &&
        find(player_moves_.begin(), player_moves_.end(), corner) == player_moves_.end()) {
        return corner;
      }
    }

    // Occupying the Sides
    std::vector<int> sides = {2, 4, 6, 8};
    for (int side : sides) {
      if (find(computer_moves_.begin(), computer_moves_.end(), side) == computer_moves_.end() &&
        find(player_moves_.begin(), player_moves_.end(), side) == player_moves_.end()) {
        return side;
      }
    }

    return -1; // If no move is found
  }
};

int main(int argc, char **argv) {
  srand(time(NULL));  // Initialize random seed
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ComputerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
