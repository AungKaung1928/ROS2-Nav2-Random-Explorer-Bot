#ifndef RANDOM_GOAL_GENERATOR_HPP
#define RANDOM_GOAL_GENERATOR_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <chrono>
#include <cmath>
#include <optional>
#include "random_explorer_bot/map_validator.hpp"

namespace random_explorer {

class RandomGoalGenerator {
public:
    struct ExplorationBounds {
        double min_x = -5.0;
        double max_x = 5.0;
        double min_y = -5.0;
        double max_y = 5.0;
    };
    
    explicit RandomGoalGenerator(const ExplorationBounds& bounds)
        : bounds_(bounds),
          gen_(std::random_device{}()) {
        updateDistributions();
    }
    
    // Generate a random valid goal within bounds
    // min_distance: minimum distance from current_pose
    // max_distance: maximum distance from current_pose (0 = no limit)
    std::optional<geometry_msgs::msg::PoseStamped> generateGoal(
        const MapValidator& validator,
        const geometry_msgs::msg::Pose& current_pose,
        double min_distance = 1.0,
        double max_distance = 0.0,
        int max_attempts = 200,
        bool allow_unknown = false) {
        
        double curr_x = current_pose.position.x;
        double curr_y = current_pose.position.y;
        
        for (int i = 0; i < max_attempts; ++i) {
            double x = x_dist_(gen_);
            double y = y_dist_(gen_);
            
            // Calculate distance from current position
            double dx = x - curr_x;
            double dy = y - curr_y;
            double distance = std::sqrt(dx * dx + dy * dy);
            
            // Check distance constraints
            if (distance < min_distance) continue;
            if (max_distance > 0 && distance > max_distance) continue;
            
            // Check if point is valid using map validator
            if (validator.isValidPoint(x, y, 0.35, allow_unknown)) {
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
    
    // Generate goal biased towards unexplored areas (edges of known map)
    std::optional<geometry_msgs::msg::PoseStamped> generateFrontierBiasedGoal(
        const MapValidator& validator,
        const geometry_msgs::msg::Pose& current_pose,
        double min_distance = 1.0,
        int max_attempts = 200) {
        
        // Try frontier-biased generation first (allow goals near unknown space)
        auto goal = generateGoal(validator, current_pose, min_distance, 0.0, 
                                 max_attempts / 2, true);
        if (goal.has_value()) return goal;
        
        // Fall back to regular generation
        return generateGoal(validator, current_pose, min_distance, 0.0, 
                           max_attempts / 2, false);
    }
    
    void setBounds(const ExplorationBounds& bounds) {
        bounds_ = bounds;
        updateDistributions();
    }
    
    const ExplorationBounds& getBounds() const { return bounds_; }
    
private:
    void updateDistributions() {
        x_dist_ = std::uniform_real_distribution<>(bounds_.min_x, bounds_.max_x);
        y_dist_ = std::uniform_real_distribution<>(bounds_.min_y, bounds_.max_y);
        theta_dist_ = std::uniform_real_distribution<>(-M_PI, M_PI);
    }
    
    ExplorationBounds bounds_;
    std::mt19937 gen_;
    std::uniform_real_distribution<> x_dist_;
    std::uniform_real_distribution<> y_dist_;
    std::uniform_real_distribution<> theta_dist_;
};

} // namespace random_explorer

#endif // RANDOM_GOAL_GENERATOR_HPP