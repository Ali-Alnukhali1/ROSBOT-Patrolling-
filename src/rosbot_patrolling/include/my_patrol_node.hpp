#ifndef MY_PATROL_NODE_HPP
#define MY_PATROL_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::literals::chrono_literals;
using FollowWaypointsAction = nav2_msgs::action::FollowWaypoints;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypointsAction>;

/// @brief Represents a single waypoint in 2D space.
struct Waypoint
{
    double x; ///< X coordinate
    double y; ///< Y coordinate
};

/**
 * @class MyPatrolNode
 * @brief ROS2 node for patrolling through waypoints using the FollowWaypoints action.
 *
 * This node loads waypoints from ROS parameters, connects to the FollowWaypoints
 * action server, and executes patrol cycles. It handles feedback, goal responses,
 * and results for the action server.
 *
 * @note Waypoints are specified in the "map" frame.
 */
class MyPatrolNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructs a MyPatrolNode object and initializes ROS2 parameters and action client.
     *
     * Loads waypoints from parameters, connects to the FollowWaypoints action server,
     * and starts the first patrol cycle.
     */
    MyPatrolNode();
private:
    rclcpp_action::Client<FollowWaypointsAction>::SharedPtr action_client_; ///< Action client for FollowWaypoints
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;                ///< List of waypoints to patrol
    int current_cycle_ = 0;                                                 ///< Tracks the current patrol cycle
    int cycle_count_ = 1;                                                   ///< Total number of patrol cycles

    /**
     * @brief Starts the next patrol cycle if remaining cycles exist.
     *
     * @note Automatically called after completion of the previous cycle.
     */
    void startNextCycle();

     /**
     * @brief Sends the waypoints goal to the FollowWaypoints action server.
     *
     * @note Requires that waypoints_ be populated.
     */
    void send_goal();

    /**
     * @brief Callback executed when a goal completes.
     *
     * @param result The result from the FollowWaypoints action server.
     *
     * @note Can be used to trigger the next patrol cycle or log statistics.
     */
    void result_callback(const GoalHandle::WrappedResult & result);

    /**
     * @brief Callback executed when the action server responds to a goal.
     *
     * @param goal_handle Shared pointer to the goal handle.
     */
    void goal_response_callback(GoalHandle::SharedPtr goal_handle);

    /**
     * @brief Callback executed when the action server sends feedback.
     *
     * @param feedback Shared pointer to the feedback message.
     */
    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const FollowWaypointsAction::Feedback> feedback);
    
    /**
     * @brief Loads a waypoint from a ROS parameter.
     *
     * @param param_name Name of the ROS parameter containing the waypoint as a double array [x, y].
     *
     * @note Waypoints with invalid size are skipped and generate a warning log.
     */
    void loadWaypoint(const std::string& param_name);

};

#endif // MY_PATROL_NODE_HPP
