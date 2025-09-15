#include <thread>
#include "random_explorer_bot/exploration_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace random_explorer {

ExplorationController::ExplorationController()
    : Node("exploration_controller") {
    
    // Load parameters
    loadParameters();
    
    // Initialize components
    goal_generator_ = std::make_unique<RandomGoalGenerator>(bounds_);
    map_validator_ = std::make_unique<MapValidator>();
    
    // Create action client for Nav2
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");
    
    // Create subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&ExplorationController::mapCallback, this, std::placeholders::_1));
    
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&ExplorationController::amclCallback, this, std::placeholders::_1));
    
    // Create publishers
    goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/exploration_goal_marker", 10);
    
    // Create exploration timer
    auto period = std::chrono::duration<double>(1.0 / exploration_frequency_);
    exploration_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&ExplorationController::explorationLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "Exploration Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Exploration bounds: X[%.1f, %.1f] Y[%.1f, %.1f]",
                bounds_.min_x, bounds_.max_x, bounds_.min_y, bounds_.max_y);
}

void ExplorationController::loadParameters() {
    // Declare and get parameters with larger default bounds
    this->declare_parameter("exploration_bounds.min_x", -3.0);
    this->declare_parameter("exploration_bounds.max_x", 3.0);
    this->declare_parameter("exploration_bounds.min_y", -3.0);
    this->declare_parameter("exploration_bounds.max_y", 3.0);
    this->declare_parameter("exploration_frequency", 0.5);
    
    bounds_.min_x = this->get_parameter("exploration_bounds.min_x").as_double();
    bounds_.max_x = this->get_parameter("exploration_bounds.max_x").as_double();
    bounds_.min_y = this->get_parameter("exploration_bounds.min_y").as_double();
    bounds_.max_y = this->get_parameter("exploration_bounds.max_y").as_double();
    exploration_frequency_ = this->get_parameter("exploration_frequency").as_double();
}

void ExplorationController::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_validator_->updateMap(msg);
    RCLCPP_INFO_ONCE(this->get_logger(), "Map received! Size: %d x %d, Resolution: %.3f",
                     msg->info.width, msg->info.height, msg->info.resolution);
}

void ExplorationController::amclCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
}

void ExplorationController::explorationLoop() {
    // Skip if already navigating
    if (is_navigating_) {
        RCLCPP_DEBUG(this->get_logger(), "Still navigating, skipping goal generation");
        return;
    }
    
    // Check if map is available
    if (!map_validator_->hasMap()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "Waiting for map...");
        return;
    }
    
    // Wait for Nav2 to be ready
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Nav2 action server not available");
        return;
    }
    
    // Generate a random valid goal with increased attempts
    auto goal_opt = goal_generator_->generateGoal(*map_validator_, 100);
    
    if (goal_opt.has_value()) {
        auto goal = goal_opt.value();
        RCLCPP_INFO(this->get_logger(), 
                    "Goal #%d: Navigating to (%.2f, %.2f)",
                    ++goal_count_, goal.pose.position.x, goal.pose.position.y);
        
        publishGoalMarker(goal);
        sendNavigationGoal(goal);
    } else {
        RCLCPP_WARN(this->get_logger(), 
                    "Failed to generate valid goal after 100 attempts. "
                    "Current bounds: X[%.1f, %.1f] Y[%.1f, %.1f]. "
                    "Consider increasing exploration boundaries or reducing clearance requirements.",
                    bounds_.min_x, bounds_.max_x, bounds_.min_y, bounds_.max_y);
    }
}

void ExplorationController::sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal) {
    auto nav_goal = NavigateToPose::Goal();
    nav_goal.pose = goal;
    nav_goal.behavior_tree = "";  // Use default behavior tree
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        std::bind(&ExplorationController::goalResponseCallback, this, std::placeholders::_1);
    
    send_goal_options.feedback_callback =
        std::bind(&ExplorationController::feedbackCallback, this, 
                  std::placeholders::_1, std::placeholders::_2);
    
    send_goal_options.result_callback =
        std::bind(&ExplorationController::resultCallback, this, std::placeholders::_1);
    
    is_navigating_ = true;
    nav_client_->async_send_goal(nav_goal, send_goal_options);
}

void ExplorationController::goalResponseCallback(
    const GoalHandleNavigateToPose::SharedPtr& goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2");
        is_navigating_ = false;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2");
    }
}

void ExplorationController::feedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    // Log progress with throttling to avoid spam
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Distance to goal: %.2f m, ETA: %.1f s",
                         feedback->distance_remaining,
                         feedback->estimated_time_remaining.sec + 
                         feedback->estimated_time_remaining.nanosec / 1e9);
}

void ExplorationController::resultCallback(
    const GoalHandleNavigateToPose::WrappedResult& result) {
    is_navigating_ = false;
    
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), 
                       "✓ Goal #%d reached successfully! Ready for next goal.",
                       goal_count_);
            // Small delay before next goal to ensure smooth transitions
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            break;
            
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), 
                       "Goal #%d was aborted. Will try a new goal.",
                       goal_count_);
            // Wait a bit before trying again
            std::this_thread::sleep_for(std::chrono::seconds(2));
            break;
            
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), 
                       "Goal #%d was canceled. Will try a new goal.",
                       goal_count_);
            break;
            
        default:
            RCLCPP_ERROR(this->get_logger(), 
                        "Unknown result code for goal #%d",
                        goal_count_);
            break;
    }
}

void ExplorationController::publishGoalMarker(const geometry_msgs::msg::PoseStamped& goal) {
    visualization_msgs::msg::Marker marker;
    marker.header = goal.header;
    marker.ns = "exploration_goals";
    marker.id = goal_count_;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose = goal.pose;
    marker.scale.x = 0.5;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    // Color based on goal number (cycles through colors)
    float hue = (goal_count_ * 60) % 360;  // Different color every goal
    float r, g, b;
    // Simple HSV to RGB conversion (S=1, V=1)
    int hi = static_cast<int>(hue / 60) % 6;
    float f = hue / 60 - hi;
    
    switch(hi) {
        case 0: r = 1; g = f; b = 0; break;
        case 1: r = 1-f; g = 1; b = 0; break;
        case 2: r = 0; g = 1; b = f; break;
        case 3: r = 0; g = 1-f; b = 1; break;
        case 4: r = f; g = 0; b = 1; break;
        default: r = 1; g = 0; b = 1-f; break;
    }
    
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    
    marker.lifetime = rclcpp::Duration::from_seconds(60.0);  // Longer lifetime
    
    goal_marker_pub_->publish(marker);
    
    // Also publish a text marker showing the goal number
    visualization_msgs::msg::Marker text_marker = marker;
    text_marker.id = goal_count_ + 1000;  // Different ID for text
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.text = "Goal #" + std::to_string(goal_count_);
    text_marker.scale.z = 0.3;  // Text size
    text_marker.pose.position.z += 0.5;  // Above the arrow
    
    goal_marker_pub_->publish(text_marker);
}

} // namespace random_explorer