#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_interfaces/action/move_x.hpp"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <atomic>
#include <iostream>
#include <string>

namespace rt2_assignment
{

class SetPositionClient : public rclcpp::Node
{
public:
  using MoveX = rt2_interfaces::action::MoveX;
  // GoalHandle is a token to track/cancel an active goal
  using GoalHandle = rclcpp_action::ClientGoalHandle<MoveX>;

  // Constructor must accept NodeOptions for components
  explicit SetPositionClient(const rclcpp::NodeOptions & options) : Node("set_position_client", options), goal_active_(false)
  {
    // Create action client
    _action_client = rclcpp_action::create_client<MoveX>(this, "/MoveX");

    // Spin up the UI thread which lives for the lifetime of the node.
    // It's detached because the node's destructor will trigger process exit anyway.
    input_thread_ = std::thread(&SetPositionClient::input_loop, this);

    // Create static broadcaster
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }
  // Destructor
  ~SetPositionClient()
  {
    // Signal the input thread to stop before destruction
    running_ = false;
    if (input_thread_.joinable()) input_thread_.join();
  }

private:
  std::thread input_thread_;
  std::atomic<bool> running_{true};
  std::atomic<bool> goal_active_{false};
  GoalHandle::SharedPtr current_goal_handle_{nullptr};  // guarded by goal_active_
  rclcpp_action::Client<MoveX>::SharedPtr _action_client;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // UI loop
  void input_loop()
  {
    while (running_) {
      if (goal_active_) {
        std::cout << "\n[Goal Active] Press 'c' to cancel current goal: ";
        std::string cmd;
        std::getline(std::cin, cmd);
        if (cmd == "c") cancel_current_goal();
        continue;
      }

      try {
        double tx, ty, tth;
        std::string raw;

        std::cout << "\nNew navigation goal" << std::endl;
        
        // Get X
        std::cout << "Enter target -10 < x < 10 (or 'c' to cancel): ";
        std::getline(std::cin, raw);
        if (raw == "c") { cancel_current_goal(); continue; }
        tx = std::stod(raw);
        if (tx < -10 || tx > 10) {std::cout << "\nInvalid input, x must belong to [-10, 10]" << std::endl; continue;}

        // Get Y
        std::cout << "Enter target -10 < y < 10 (or 'c' to cancel): ";
        std::getline(std::cin, raw);
        if (raw == "c") { cancel_current_goal(); continue; }
        ty = std::stod(raw);
        if (tx < -10 || tx > 10) {std::cout << "\nInvalid input, y must belong to [-10, 10]" << std::endl; continue;}

        // Get Theta
        std::cout << "Enter target Theta in radians (or 'c' to cancel): ";
        std::getline(std::cin, raw);
        if (raw == "c") { cancel_current_goal(); continue; }
        tth = std::stod(raw);

        // Final confirmation and validation
        std::cout << "Sending Goal: (" << tx << ", " << ty << ", " << tth << "). Proceed? (y/n): ";
        std::getline(std::cin, raw);
        if (raw == "y") {
          send_goal(tx, ty, tth);
        } else {
          std::cout << "Goal discarded." << std::endl;
        }

      } catch (const std::invalid_argument &) {
        std::cout << "Invalid numeric input. Please restart the entry process." << std::endl;
      } catch (const std::exception & e) {
        std::cout << "Error: " << e.what() << std::endl;
      }
    }
  }

  void send_goal(double x, double y, double theta)
  {
    // Wait for server availability
    if (!_action_client->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available");
      return;
    }

    auto goal_msg = MoveX::Goal();
    goal_msg.goal_x = x;
    goal_msg.goal_y = y;
    goal_msg.goal_theta = theta;

    goal_active_ = true;   // block new goals until this one finishes

    // Send_goal. Everything is wrapped in a struct
    auto options = rclcpp_action::Client<MoveX>::SendGoalOptions();
    options.goal_response_callback = std::bind(&SetPositionClient::goal_response_callback, this, std::placeholders::_1);
    options.feedback_callback = std::bind(&SetPositionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    options.result_callback = std::bind(&SetPositionClient::get_result_callback, this, std::placeholders::_1);

    // Broadcast goal frame
    broadcast_goal_frame(x, y, theta);
    // Send goal, thread safe
    _action_client->async_send_goal(goal_msg, options);
  }

  void cancel_current_goal()
  {
    if (!current_goal_handle_) {
      std::cout << "No active goal to cancel\n";
      return;
    }
    // cancel_goal_async is also thread-safe
    _action_client->async_cancel_goal(current_goal_handle_);
  }

  // The goal_handle is passed directly (no .result() unwrapping needed)
  void goal_response_callback(const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_INFO(this->get_logger(), "Goal refused");
      goal_active_ = false;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    current_goal_handle_ = goal_handle;   // save for cancellation
  }

  // C++ gets both the goal_handle (for cancel) and the feedback payload
  void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const MoveX::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback:\n Remaining distance: %f m\n Remaining Theta: %f rad", feedback->remaining_distance, feedback->remaining_theta);
  }

  // Wraps result + status + goal_id in a WrappedResult struct
  void get_result_callback(const GoalHandle::WrappedResult & result)
  {
    RCLCPP_INFO(this->get_logger(),
    "Translation to %f, %f, %f: %s",
    result.result->final_x,
    result.result->final_y,
    result.result->final_theta,
    result.result->success ? "true" : "false");
    current_goal_handle_ = nullptr;
    goal_active_ = false;   // input_loop will now offer a new goal prompt
  }

  void broadcast_goal_frame(double x, double y, double theta) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world"; 
    t.child_frame_id = "goal_frame";

    // Position
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    // Convert Euler Theta to Quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    static_broadcaster_->sendTransform(t);
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment::SetPositionClient)