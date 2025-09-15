#ifndef RANDOM_GOAL_GENERATOR_HPP
#define RANDOM_GOAL_GENERATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <random>
#include <chrono>
#include "random_explorer_bot/map_validator.hpp"

namespace random_explorer {

class RandomGoalGenerator {
public:
    struct ExplorationBounds {
        double min_x = -2.0;
        double max_x = 2.0;
        double min_y = -2.0;
        double max_y = 2.0;
    };
    
    RandomGoalGenerator(const ExplorationBounds& bounds)
        : bounds_(bounds),
          gen_(std::chrono::steady_clock::now().time_since_epoch().count()) {
        x_dist_ = std::uniform_real_distribution<>(bounds_.min_x, bounds_.max_x);
        y_dist_ = std::uniform_real_distribution<>(bounds_.min_y, bounds_.max_y);
        theta_dist_ = std::uniform_real_distribution<>(-M_PI, M_PI);
    }
    
    // Generate a random valid goal within bounds
    std::optional<geometry_msgs::msg::PoseStamped> generateGoal(
        const MapValidator& validator,
        int max_attempts = 50) {
        
        for (int i = 0; i < max_attempts; ++i) {
            double x = x_dist_(gen_);
            double y = y_dist_(gen_);
            
            // Check if point is valid using map validator
            if (validator.isValidPoint(x, y)) {
                geometry_msgs::msg::PoseStamped goal;
                goal.header.frame_id = "map";
                goal.header.stamp = rclcpp::Clock().now();
                
                goal.pose.position.x = x;
                goal.pose.position.y = y;
                goal.pose.position.z = 0.0;
                
                // Random orientation
                double theta = theta_dist_(gen_);
                goal.pose.orientation.z = std::sin(theta / 2.0);
                goal.pose.orientation.w = std::cos(theta / 2.0);
                
                return goal;
            }
        }
        return std::nullopt;
    }
    
    // Update exploration bounds
    void setBounds(const ExplorationBounds& bounds) {
        bounds_ = bounds;
        x_dist_ = std::uniform_real_distribution<>(bounds_.min_x, bounds_.max_x);
        y_dist_ = std::uniform_real_distribution<>(bounds_.min_y, bounds_.max_y);
    }

private:
    ExplorationBounds bounds_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> x_dist_;
    std::uniform_real_distribution<> y_dist_;
    std::uniform_real_distribution<> theta_dist_;
};

} // namespace random_explorer

#endif // RANDOM_GOAL_GENERATOR_HPP