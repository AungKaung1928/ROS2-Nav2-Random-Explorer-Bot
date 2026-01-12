#include <rclcpp/rclcpp.hpp>
#include "random_explorer_bot/exploration_controller.hpp"
#include <memory>
#include <csignal>
#include <atomic>

// Global flag for graceful shutdown on SIGINT/SIGTERM
std::atomic<bool> g_shutdown_requested{false};

// Signal handler: sets shutdown flag and triggers rclcpp shutdown
void signalHandler(int signum) {
    (void)signum;
    g_shutdown_requested = true;
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // Register signal handlers for Ctrl+C and kill commands
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Random Explorer Bot...");
    
    try {
        // Create exploration controller node
        auto explorer = std::make_shared<random_explorer::ExplorationController>();
        
        // Single-threaded executor for callbacks and timers
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(explorer);
        
        // Main loop: spin executor until shutdown requested
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
