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

    // ambil input angka
    while (rclcpp::ok()) {
      int player_move = getPlayerMove();
      if (player_move >= 1 && player_move <= 9 && player_move != getComputerMove() && !isMoveAlreadyMade(player_move)) {
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
    return false;
  }

  bool isWinner(const std::vector<int>& moves) { // fungsi cek kombinasi utk menang
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

  bool isForkingMove(int move, const std::vector<int>& moves) { // fungsi cek kombinasi fork
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

  bool isBlockingFork(int move, const std::vector<int>& moves) { // fungsi block fork player
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
    if (!player_moves_.empty()) {
      // isi center
      if (find(computer_moves_.begin(), computer_moves_.end(), 5) == computer_moves_.end() &&
        find(player_moves_.begin(), player_moves_.end(), 5) == player_moves_.end()) {
        return 5;
      }

      // cek kombinasi menang
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

      // cek gerakan block
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

      // cek kemungkinan fork
      for (int move = 1; move <= 9; ++move) {
        if (find(computer_moves_.begin(), computer_moves_.end(), move) == computer_moves_.end() &&
          find(player_moves_.begin(), player_moves_.end(), move) == player_moves_.end()) {
          if (isForkingMove(move, computer_moves_)) {
            return move;
          }
        }
      }

      // cek kemungkinan untuk block fork
      for (int move = 1; move <= 9; ++move) {
        if (find(computer_moves_.begin(), computer_moves_.end(), move) == computer_moves_.end() &&
          find(player_moves_.begin(), player_moves_.end(), move) == player_moves_.end()) {
          if (isBlockingFork(move, player_moves_)) {
            return move;
          }
        }
      }

      // isi korner
      std::vector<int> corners = {1, 3, 7, 9};
      for (int corner : corners) {
        if (find(computer_moves_.begin(), computer_moves_.end(), corner) == computer_moves_.end() &&
          find(player_moves_.begin(), player_moves_.end(), corner) == player_moves_.end()) {
          return corner;
        }
      }

      // isi edge (sisi)
      std::vector<int> sides = {2, 4, 6, 8};
      for (int side : sides) {
        if (find(computer_moves_.begin(), computer_moves_.end(), side) == computer_moves_.end() &&
          find(player_moves_.begin(), player_moves_.end(), side) == player_moves_.end()) {
          return side;
        }
      }
    }
    return -1; // handle no return
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlayerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
