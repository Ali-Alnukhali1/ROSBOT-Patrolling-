#include "my_patrol_node.hpp"

MyPatrolNode::MyPatrolNode() : Node("patrolling_action_client")
{
    // Declare waypoint parameters for all 5 waypoints
    this->declare_parameter("cycle_count", 1);
    this->declare_parameter("waypoint1", std::vector<double>{});
    this->declare_parameter("waypoint2", std::vector<double>{});
    this->declare_parameter("waypoint3", std::vector<double>{});
    this->declare_parameter("waypoint4", std::vector<double>{});
    this->declare_parameter("waypoint5", std::vector<double>{});

    // Get cycle count
    cycle_count_ = this->get_parameter("cycle_count").as_int();
    RCLCPP_INFO(this->get_logger(), "Cycle count: %d", cycle_count_);

    // Load waypoints from parameters
    loadWaypoint("waypoint1");
    loadWaypoint("waypoint2");
    loadWaypoint("waypoint3");
    loadWaypoint("waypoint4");
    loadWaypoint("waypoint5");

    // Initialize action client
    action_client_ = rclcpp_action::create_client<FollowWaypointsAction>(this, "follow_waypoints");

    RCLCPP_INFO(this->get_logger(), "Waiting for FollowWaypoints action server...");

    if (!action_client_->wait_for_action_server(10s))
    {
        RCLCPP_ERROR(this->get_logger(), "FollowWaypoints action server not available after waiting");
        rclcpp::shutdown();
        return;
    }

    current_cycle_ = 0;
    startNextCycle();
}

/**
 * @brief Starts the next patrol cycle.
 *
 * Checks if more cycles remain, increments the cycle count, and sends a new goal.
 * Logs completion when all cycles are finished.
 */
void MyPatrolNode::startNextCycle()
{
    if (current_cycle_ >= cycle_count_) {
        RCLCPP_INFO(this->get_logger(), "Completed all %d patrol cycles", cycle_count_);
        
        return;
    }
    current_cycle_++;
    RCLCPP_INFO(this->get_logger(), "Starting patrol cycle %d/%d", current_cycle_, cycle_count_);
    send_goal();
}

/**
 * @brief Sends waypoints goal to the FollowWaypoints action server.
 *
 * Populates the goal with loaded waypoints and sets the goal, feedback, and result callbacks.
 * @note Does nothing if waypoints_ is empty.
 */
void MyPatrolNode::send_goal()
{
    if (waypoints_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No waypoints loaded, cannot send goal");
        return;
    }

    auto goal_msg = FollowWaypointsAction::Goal();
    goal_msg.poses = waypoints_;

    auto send_goal_options = rclcpp_action::Client<FollowWaypointsAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&MyPatrolNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&MyPatrolNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&MyPatrolNode::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending waypoints goal...");
    action_client_->async_send_goal(goal_msg, send_goal_options);
}

/**
 * @brief Callback executed when a goal completes.
 *
 * Logs the number of missed waypoints and automatically starts the next patrol cycle.
 *
 * @param result The result received from the FollowWaypoints action server.
 */
void MyPatrolNode::result_callback(const GoalHandle::WrappedResult & result)
{
    auto missed = result.result->missed_waypoints.size(); 
    RCLCPP_INFO(this->get_logger(), "Number of waypoints missed: %zu", missed);

    startNextCycle();
}

/**
 * @brief Callback executed when the action server responds to a goal.
 *
 * Logs whether the goal was accepted or rejected.
 *
 * @param goal_handle Shared pointer to the goal handle.
 */
void MyPatrolNode::goal_response_callback(GoalHandle::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server for cycle %d/%d", current_cycle_, cycle_count_);
    }
}

/**
 * @brief Callback executed when the action server sends feedback.
 *
 * Logs the current waypoint index for the active patrol cycle.
 *
 * @param feedback Shared pointer to the feedback message.
 */
void MyPatrolNode::feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const FollowWaypointsAction::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Cycle %d/%d - Current waypoint: %d", current_cycle_, cycle_count_, feedback->current_waypoint);
}

/**
 * @brief Loads a single waypoint from a ROS parameter.
 *
 * Converts a 2-element double array into a PoseStamped and appends it to waypoints_.
 *
 * @param param_name Name of the ROS parameter storing the waypoint [x, y].
 *
 * @note If the parameter size is not 2, the waypoint is skipped with a warning.
 */
void MyPatrolNode::loadWaypoint(const std::string& param_name)
{
    auto wp_vec = this->get_parameter(param_name).as_double_array();

    if (wp_vec.size() != 2) {
        RCLCPP_WARN(this->get_logger(), "Waypoint %s has incorrect size, skipping", param_name.c_str());
        return;
    }

    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.header.stamp = this->now();
    wp.pose.position.x = wp_vec[0];
    wp.pose.position.y = wp_vec[1];

    waypoints_.push_back(wp);

    RCLCPP_INFO(this->get_logger(), "Loaded waypoint %s: [%.2f, %.2f]", param_name.c_str(), wp_vec[0], wp_vec[1]);
}

/**
 * @brief Main entry point for the patrol node.
 *
 * Initializes ROS2, spins the MyPatrolNode, and shuts down cleanly.
 */
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyPatrolNode>());
    rclcpp::shutdown();
    return 0;
}
