#include <rclcpp/rclcpp.hpp>
#include "random_explorer_bot/exploration_controller.hpp"
#include <memory>
#include <csignal>

std::shared_ptr<random_explorer::ExplorationController> g_explorer = nullptr;

void signalHandler(int signum) {
    (void)signum;
    if (g_explorer) {
        RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down explorer...");
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Setup signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Random Explorer Bot...");
    
    try {
        g_explorer = std::make_shared<random_explorer::ExplorationController>();
        rclcpp::spin(g_explorer);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
