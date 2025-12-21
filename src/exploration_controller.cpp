#include "random_explorer_bot/exploration_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace random_explorer {

ExplorationController::ExplorationController()
    : Node("exploration_controller") {
    
    loadParameters();
    
    goal_generator_ = std::make_unique<RandomGoalGenerator>(bounds_);
    map_validator_ = std::make_unique<MapValidator>();
    
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
        this, "navigate_to_pose");
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(10).transient_local(),
        std::bind(&ExplorationController::mapCallback, this, std::placeholders::_1));
    
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10,
        std::bind(&ExplorationController::amclCallback, this, std::placeholders::_1));
    
    goal_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/exploration_goal_marker", 10);
    
    auto period = std::chrono::duration<double>(1.0 / exploration_frequency_);
    exploration_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&ExplorationController::explorationLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "  Random Explorer Bot Initialized");
    RCLCPP_INFO(this->get_logger(), "===========================================");
    RCLCPP_INFO(this->get_logger(), "Bounds: X[%.1f, %.1f] Y[%.1f, %.1f]",
                bounds_.min_x, bounds_.max_x, bounds_.min_y, bounds_.max_y);
    RCLCPP_INFO(this->get_logger(), "Min goal distance: %.1f m", min_goal_distance_);
    RCLCPP_INFO(this->get_logger(), "Goal timeout: %.0f sec", goal_timeout_sec_);
    RCLCPP_INFO(this->get_logger(), "===========================================");
}

void ExplorationController::loadParameters() {
    // Exploration bounds - cover the entire TurtleBot3 world
    this->declare_parameter("exploration_bounds.min_x", -4.5);
    this->declare_parameter("exploration_bounds.max_x", 4.5);
    this->declare_parameter("exploration_bounds.min_y", -4.5);
    this->declare_parameter("exploration_bounds.max_y", 4.5);
    this->declare_parameter("exploration_frequency", 0.5);
    this->declare_parameter("goal_timeout_sec", 60.0);
    this->declare_parameter("min_goal_distance", 1.0);
    this->declare_parameter("max_goal_distance", 8.0);
    this->declare_parameter("max_goal_attempts", 200);
    
    bounds_.min_x = this->get_parameter("exploration_bounds.min_x").as_double();
    bounds_.max_x = this->get_parameter("exploration_bounds.max_x").as_double();
    bounds_.min_y = this->get_parameter("exploration_bounds.min_y").as_double();
    bounds_.max_y = this->get_parameter("exploration_bounds.max_y").as_double();
    exploration_frequency_ = this->get_parameter("exploration_frequency").as_double();
    goal_timeout_sec_ = this->get_parameter("goal_timeout_sec").as_double();
    min_goal_distance_ = this->get_parameter("min_goal_distance").as_double();
    max_goal_distance_ = this->get_parameter("max_goal_distance").as_double();
    max_goal_attempts_ = this->get_parameter("max_goal_attempts").as_int();
}

void ExplorationController::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_validator_->updateMap(msg);
    RCLCPP_INFO_ONCE(this->get_logger(), "Map received: %d x %d (%.3f m/cell)",
                     msg->info.width, msg->info.height, msg->info.resolution);
}

void ExplorationController::amclCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    have_pose_ = true;
}

void ExplorationController::explorationLoop() {
    // Skip if already navigating
    if (is_navigating_) {
        return;
    }
    
    // Check prerequisites
    if (!map_validator_->hasMap()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "Waiting for map...");
        return;
    }
    
    // Check Nav2 availability
    if (!nav2_ready_) {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for Nav2...");
            return;
        }
        nav2_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Nav2 is ready!");
    }
    
    // Use origin if we don't have AMCL pose yet
    if (!have_pose_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for robot pose...");
        current_pose_.position.x = 0.0;
        current_pose_.position.y = 0.0;
    }
    
    // Generate goal with distance constraints
    auto goal_opt = goal_generator_->generateGoal(
        *map_validator_,
        current_pose_,
        min_goal_distance_,
        max_goal_distance_,
        max_goal_attempts_,
        true  // Allow goals near unknown space
    );
    
    if (goal_opt.has_value()) {
        auto goal = goal_opt.value();
        
        double dx = goal.pose.position.x - current_pose_.position.x;
        double dy = goal.pose.position.y - current_pose_.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        RCLCPP_INFO(this->get_logger(), 
                    ">>> Goal #%d: (%.2f, %.2f) - Distance: %.2f m",
                    ++goal_count_, goal.pose.position.x, goal.pose.position.y, distance);
        
        consecutive_failures_ = 0;
        publishGoalMarker(goal);
        sendNavigationGoal(goal);
    } else {
        consecutive_failures_++;
        RCLCPP_WARN(this->get_logger(), 
                    "Failed to find valid goal (attempt %d). Bounds: X[%.1f, %.1f] Y[%.1f, %.1f]",
                    consecutive_failures_,
                    bounds_.min_x, bounds_.max_x, bounds_.min_y, bounds_.max_y);
        
        // If failing repeatedly, try with relaxed constraints
        if (consecutive_failures_ >= 3) {
            RCLCPP_WARN(this->get_logger(), "Relaxing goal constraints...");
            auto relaxed_goal = goal_generator_->generateGoal(
                *map_validator_, current_pose_,
                0.5,   // Smaller min distance
                0.0,   // No max distance
                300,   // More attempts
                true
            );
            
            if (relaxed_goal.has_value()) {
                auto goal = relaxed_goal.value();
                RCLCPP_INFO(this->get_logger(), 
                            ">>> Goal #%d (relaxed): (%.2f, %.2f)",
                            ++goal_count_, goal.pose.position.x, goal.pose.position.y);
                consecutive_failures_ = 0;
                publishGoalMarker(goal);
                sendNavigationGoal(goal);
            }
        }
    }
}

void ExplorationController::sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal) {
    auto nav_goal = NavigateToPose::Goal();
    nav_goal.pose = goal;
    nav_goal.behavior_tree = "";
    
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        std::bind(&ExplorationController::goalResponseCallback, this, std::placeholders::_1);
    
    send_goal_options.feedback_callback =
        std::bind(&ExplorationController::feedbackCallback, this, 
                  std::placeholders::_1, std::placeholders::_2);
    
    send_goal_options.result_callback =
        std::bind(&ExplorationController::resultCallback, this, std::placeholders::_1);
    
    is_navigating_ = true;
    goal_start_time_ = std::chrono::steady_clock::now();
    
    // Start timeout timer
    goal_timeout_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(goal_timeout_sec_),
        std::bind(&ExplorationController::goalTimeoutCallback, this));
    
    nav_client_->async_send_goal(nav_goal, send_goal_options);
}

void ExplorationController::goalResponseCallback(
    const GoalHandleNavigateToPose::SharedPtr& goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected by Nav2");
        is_navigating_ = false;
        if (goal_timeout_timer_) {
            goal_timeout_timer_->cancel();
            goal_timeout_timer_.reset();
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted");
        current_goal_handle_ = goal_handle;
    }
}

void ExplorationController::feedbackCallback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Distance remaining: %.2f m",
                         feedback->distance_remaining);
}

void ExplorationController::goalTimeoutCallback() {
    if (is_navigating_) {
        RCLCPP_WARN(this->get_logger(), 
                    "Goal #%d timed out after %.0f sec. Canceling and trying new goal.",
                    goal_count_, goal_timeout_sec_);
        cancelCurrentGoal();
    }
    
    if (goal_timeout_timer_) {
        goal_timeout_timer_->cancel();
        goal_timeout_timer_.reset();
    }
}

void ExplorationController::cancelCurrentGoal() {
    if (current_goal_handle_) {
        nav_client_->async_cancel_goal(current_goal_handle_);
    }
    is_navigating_ = false;
    current_goal_handle_.reset();
}

void ExplorationController::retryCallback() {
    // This timer is used for delayed retry - just cancel it
    if (retry_timer_) {
        retry_timer_->cancel();
        retry_timer_.reset();
    }
}

void ExplorationController::resultCallback(
    const GoalHandleNavigateToPose::WrappedResult& result) {
    // Cancel timeout timer
    if (goal_timeout_timer_) {
        goal_timeout_timer_->cancel();
        goal_timeout_timer_.reset();
    }
    
    auto elapsed = std::chrono::steady_clock::now() - goal_start_time_;
    double elapsed_sec = std::chrono::duration<double>(elapsed).count();
    
    is_navigating_ = false;
    current_goal_handle_.reset();
    
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), 
                       "Goal #%d reached in %.1f sec",
                       goal_count_, elapsed_sec);
            break;
            
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), 
                       "Goal #%d aborted after %.1f sec",
                       goal_count_, elapsed_sec);
            // Use a one-shot timer for delayed retry instead of blocking sleep
            retry_timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&ExplorationController::retryCallback, this));
            break;
            
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), 
                       "Goal #%d canceled after %.1f sec",
                       goal_count_, elapsed_sec);
            break;
            
        default:
            RCLCPP_ERROR(this->get_logger(), 
                        "Unknown result for goal #%d",
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
    
    // Color based on goal number
    float hue = (goal_count_ * 45) % 360;
    float r, g, b;
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
    marker.lifetime = rclcpp::Duration::from_seconds(120.0);
    
    goal_marker_pub_->publish(marker);
    
    // Text marker
    visualization_msgs::msg::Marker text_marker = marker;
    text_marker.id = goal_count_ + 1000;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.text = "#" + std::to_string(goal_count_);
    text_marker.scale.z = 0.3;
    text_marker.pose.position.z += 0.5;
    
    goal_marker_pub_->publish(text_marker);
}

} // namespace random_explorer