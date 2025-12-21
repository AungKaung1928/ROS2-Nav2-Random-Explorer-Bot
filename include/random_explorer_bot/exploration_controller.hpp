#ifndef EXPLORATION_CONTROLLER_HPP
#define EXPLORATION_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "random_explorer_bot/random_goal_generator.hpp"
#include "random_explorer_bot/map_validator.hpp"
#include <memory>
#include <chrono>

namespace random_explorer {

class ExplorationController : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    explicit ExplorationController();
    
private:
    // Action client for Nav2
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    
    // Publisher
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr exploration_timer_;
    rclcpp::TimerBase::SharedPtr goal_timeout_timer_;
    rclcpp::TimerBase::SharedPtr retry_timer_;
    
    // Components
    std::unique_ptr<RandomGoalGenerator> goal_generator_;
    std::unique_ptr<MapValidator> map_validator_;
    
    // State variables
    bool is_navigating_ = false;
    bool nav2_ready_ = false;
    geometry_msgs::msg::Pose current_pose_;
    bool have_pose_ = false;
    int goal_count_ = 0;
    int consecutive_failures_ = 0;
    std::chrono::steady_clock::time_point goal_start_time_;
    
    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void explorationLoop();
    void goalTimeoutCallback();
    void retryCallback();
    
    // Navigation methods
    void sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal);
    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr& goal_handle);
    void feedbackCallback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult& result);
    void cancelCurrentGoal();
    
    // Visualization
    void publishGoalMarker(const geometry_msgs::msg::PoseStamped& goal);
    
    // Parameters
    void loadParameters();
    RandomGoalGenerator::ExplorationBounds bounds_;
    double exploration_frequency_ = 1.0;
    double goal_timeout_sec_ = 60.0;
    double min_goal_distance_ = 1.0;
    double max_goal_distance_ = 8.0;
    int max_goal_attempts_ = 200;
};

} // namespace random_explorer

#endif // EXPLORATION_CONTROLLER_HPP