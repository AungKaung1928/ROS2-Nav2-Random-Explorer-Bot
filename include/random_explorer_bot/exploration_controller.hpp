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
    // Nav2 action client and active goal handle
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    GoalHandleNavigateToPose::SharedPtr current_goal_handle_;
    
    // Subscriptions: costmap updates + robot pose from AMCL
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    
    // Publishes RViz markers for goal visualization
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
    
    // Timers: exploration loop, goal timeout watchdog, retry after failures
    rclcpp::TimerBase::SharedPtr exploration_timer_;
    rclcpp::TimerBase::SharedPtr goal_timeout_timer_;
    rclcpp::TimerBase::SharedPtr retry_timer_;
    
    // Goal generation and validation components
    std::unique_ptr<RandomGoalGenerator> goal_generator_;
    std::unique_ptr<MapValidator> map_validator_;
    
    // State tracking: navigation active, Nav2 ready, current pose, initialization flags
    bool is_navigating_ = false;
    bool nav2_ready_ = false;
    geometry_msgs::msg::Pose current_pose_;
    bool have_pose_ = false;
    int goal_count_ = 0;
    int consecutive_failures_ = 0;
    std::chrono::steady_clock::time_point goal_start_time_;
    
    // Callbacks: map updates, pose updates, exploration loop, timeouts
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void explorationLoop();
    void goalTimeoutCallback();
    void retryCallback();
    
    // Navigation: send goal, handle response/feedback/result, cancel goal
    void sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal);
    void goalResponseCallback(const GoalHandleNavigateToPose::SharedPtr& goal_handle);
    void feedbackCallback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void resultCallback(const GoalHandleNavigateToPose::WrappedResult& result);
    void cancelCurrentGoal();
    
    // Publish RViz arrow marker at goal location
    void publishGoalMarker(const geometry_msgs::msg::PoseStamped& goal);
    
    // Load parameters from ROS2 parameter server
    void loadParameters();
    
    // Parameters: bounds, timing, distance constraints, retry limits
    RandomGoalGenerator::ExplorationBounds bounds_;
    double exploration_frequency_ = 1.0;  // Goal generation check rate (Hz)
    double goal_timeout_sec_ = 60.0;      // Max time per goal before cancel (sec)
    double min_goal_distance_ = 1.0;      // Min distance from robot (m)
    double max_goal_distance_ = 8.0;      // Max distance from robot (m)
    int max_goal_attempts_ = 200;         // Max sampling iterations per goal
};

} // namespace random_explorer

#endif // EXPLORATION_CONTROLLER_HPP
