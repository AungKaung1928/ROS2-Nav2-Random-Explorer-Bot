#include <rclcpp/rclcpp.hpp>
#include "random_explorer_bot/exploration_controller.hpp"
#include <memory>
#include <csignal>
#include <atomic>

std::atomic<bool> g_shutdown_requested{false};

void signalHandler(int signum) {
    (void)signum;
    g_shutdown_requested = true;
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Random Explorer Bot...");
    
    try {
        auto explorer = std::make_shared<random_explorer::ExplorationController>();
        
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(explorer);
        
        while (rclcpp::ok() && !g_shutdown_requested) {
            executor.spin_some(std::chrono::milliseconds(100));
        }
        
        RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down...");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}