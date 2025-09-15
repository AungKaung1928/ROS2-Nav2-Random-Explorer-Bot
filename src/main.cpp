#include <rclcpp/rclcpp.hpp>
#include "random_explorer_bot/exploration_controller.hpp"
#include <memory>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Random Explorer Bot");
    
    try {
        auto explorer = std::make_shared<random_explorer::ExplorationController>();
        rclcpp::spin(explorer);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}