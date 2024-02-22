#include <iostream>
#include <cstdlib>
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
                std::cout << "Player's move: " << player_move << std::endl;
                
                // Calculate computer's move
                int computer_move = getComputerMove();
                std::cout << "Computer's move: " << computer_move << std::endl;

                // Publish computer's move
                auto message = std::make_unique<std_msgs::msg::Int32>();
                message->data = computer_move;
                publisher_->publish(std::move(message));
            }
        );

        publisher_ = this->create_publisher<std_msgs::msg::Int32>("computer_move_topic", 10);
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

    int getComputerMove() {
        // Generate a random move for now (1-9)
        return (rand() % 9) + 1;
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
