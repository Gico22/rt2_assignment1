#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_interfaces/action/move_x.hpp"
#include <tf2/LinearMath/Quaternion.h>

namespace rt2_assignment
{

class SetPositionClient : public rclcpp::Node
{
public:
  using MoveX = rt2_interfaces::action::MoveX;
  // GoalHandle is a token to track/cancel an active goal
  using GoalHandle = rclcpp_action::ClientGoalHandle<MoveX>;

  // Constructor must accept NodeOptions for components
  explicit SetPositionClient(const rclcpp::NodeOptions & options) : Node("set_position_client", options)
  {
    // Create action client
    _action_client = rclcpp_action::create_client<MoveX>(this, "/MoveX");

    // Use a one-shot timer to trigger the goal send
    // (replaces action_client.send_goal(2.0) that was called from main() in Python)
    _timer = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SetPositionClient::send_goal, this));
  }

  void send_goal()
  {
    _timer->cancel();  // fire once only
    // Wait for server availability
    if (!_action_client->wait_for_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = MoveX::Goal();
    goal_msg.goal_x = 2.0;

    // Send_goal. Everything is wrapped in a struct
    auto options = rclcpp_action::Client<MoveX>::SendGoalOptions();
    options.goal_response_callback = std::bind(&SetPositionClient::goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback = std::bind(&SetPositionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback = std::bind(&SetPositionClient::get_result_callback, this, std::placeholders::_1);

    // Send goal
    _action_client->async_send_goal(goal_msg, options);
  }

private:
  rclcpp_action::Client<MoveX>::SharedPtr _action_client;
  rclcpp::TimerBase::SharedPtr _timer;

  // The goal_handle is passed directly (no .result() unwrapping needed)
  void goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Goal refused");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted");
    }
    // get_result_async() is not needed since result_callback is already
    // registered in SendGoalOptions above and fires automatically
  }

  // C++ gets both the goal_handle (for cancel) and the feedback payload
  void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const MoveX::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback: %f", feedback->remaining_x);
  }

  // Wraps result + status + goal_id in a WrappedResult struct
  void get_result_callback(const GoalHandle::WrappedResult & result)
  {
    RCLCPP_INFO(this->get_logger(),
      "Translation to %f: %s",
      result.result->final_x,
      result.result->success ? "true" : "false");
    // Avoid calling shutdown() inside a component;
    // just log the result and let the container manage lifecycle
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment::SetPositionClient)