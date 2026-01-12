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
    
    // Generate collision-free random goal within bounds and distance constraints
    // min_distance: reject goals too close to robot
    // max_distance: reject goals too far from robot (0 = unlimited)
    // max_attempts: sampling iterations before giving up
    // allow_unknown: treat unknown cells as valid (for frontier exploration)
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
            
            // Calculate Euclidean distance from robot
            double dx = x - curr_x;
            double dy = y - curr_y;
            double distance = std::sqrt(dx * dx + dy * dy);
            
            // Reject if outside distance constraints
            if (distance < min_distance) continue;
            if (max_distance > 0 && distance > max_distance) continue;
            
            // Validate collision-free with 0.35m clearance radius
            if (validator.isValidPoint(x, y, 0.35, allow_unknown)) {
                geometry_msgs::msg::PoseStamped goal;
                goal.header.frame_id = "map";
                goal.header.stamp = rclcpp::Clock().now();
                
                goal.pose.position.x = x;
                goal.pose.position.y = y;
                goal.pose.position.z = 0.0;
                
                // Random orientation using quaternion (z, w components)
                double theta = theta_dist_(gen_);
                goal.pose.orientation.z = std::sin(theta / 2.0);
                goal.pose.orientation.w = std::cos(theta / 2.0);
                
                return goal;
            }
        }
        return std::nullopt;  // Failed to generate valid goal
    }
    
    // Generate goal near unexplored map edges (frontier exploration)
    // First attempts with allow_unknown=true, then falls back to known-free space
    std::optional<geometry_msgs::msg::PoseStamped> generateFrontierBiasedGoal(
        const MapValidator& validator,
        const geometry_msgs::msg::Pose& current_pose,
        double min_distance = 1.0,
        int max_attempts = 200) {
        
        // Try frontier-biased: goals near unknown cells (half attempts)
        auto goal = generateGoal(validator, current_pose, min_distance, 0.0, 
                                 max_attempts / 2, true);
        if (goal.has_value()) return goal;
        
        // Fallback: goals in known-free space only (remaining attempts)
        return generateGoal(validator, current_pose, min_distance, 0.0, 
                           max_attempts / 2, false);
    }
    
    // Update XY bounds and reconfigure random distributions
    void setBounds(const ExplorationBounds& bounds) {
        bounds_ = bounds;
        updateDistributions();
    }
    
    const ExplorationBounds& getBounds() const { return bounds_; }
    
private:
    // Recreate uniform distributions after bounds change
    void updateDistributions() {
        x_dist_ = std::uniform_real_distribution<>(bounds_.min_x, bounds_.max_x);
        y_dist_ = std::uniform_real_distribution<>(bounds_.min_y, bounds_.max_y);
        theta_dist_ = std::uniform_real_distribution<>(-M_PI, M_PI);
    }
    
    ExplorationBounds bounds_;
    std::mt19937 gen_;  // Mersenne Twister RNG
    std::uniform_real_distribution<> x_dist_;
    std::uniform_real_distribution<> y_dist_;
    std::uniform_real_distribution<> theta_dist_;
};

} // namespace random_explorer

#endif // RANDOM_GOAL_GENERATOR_HPP
