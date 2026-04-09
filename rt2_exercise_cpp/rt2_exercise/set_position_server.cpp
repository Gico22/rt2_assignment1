#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rt2_interfaces/action/move_x.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <atomic>   // thread-safe position_x_ 
#include <thread>   // std::thread for execute_callback
#include <chrono>   

namespace rt2_assignment
{

class SetPositionServer : public rclcpp::Node
{
public:
  using MoveX = rt2_interfaces::action::MoveX;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveX>;

  explicit SetPositionServer(const rclcpp::NodeOptions & options) : Node("set_position_server", options), position_x_(0.0)
  {
    using namespace std::placeholders;

    // Create the callback group from the node and pass it explicitly to each subscriber/server
    auto cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Create action server
    // We have three separate callbacks instead of a single one:
    _action_server = rclcpp_action::create_server<MoveX>(
      this, "/MoveX",
      std::bind(&SetPositionServer::handle_goal, this, _1, _2),
      std::bind(&SetPositionServer::handle_cancel, this, _1),
      std::bind(&SetPositionServer::handle_accepted, this, _1),
      rcl_action_server_get_default_options(),
      cb_group);

    // Create velocity publisher
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Create position subscription
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;
    pos_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&SetPositionServer::get_position_callback, this, _1),
      sub_options);
  }

private:
  rclcpp_action::Server<MoveX>::SharedPtr _action_server;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_subscriber_;

  // std::atomic<double> makes reads/writes thread-safe without a mutex,
  // since position_x_ is written by the odom callback and read by the execute thread
  std::atomic<double> position_x_;

  // 3 callbacks (instead of the single Python ActionServer callback)
  // handle_goal → decides whether to accept or reject the incoming goal
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveX::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: %f", goal->goal_x);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // handle_cancel → decides whether to accept a cancel request
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // handle_accepted fires right after the goal is accepted.
  // We spawn a detached thread so this callback returns immediately,
  // freeing the executor (the same reason Python used MultiThreadedExecutor)
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{
      std::bind(&SetPositionServer::execute_callback, this, goal_handle)
    }.detach();
  }

  void execute_callback(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    auto feedback_msg = std::make_shared<MoveX::Feedback>();
    double goal = goal_handle->get_goal()->goal_x;

    geometry_msgs::msg::Twist vel_x;   
    geometry_msgs::msg::Twist stop;    
    vel_x.linear.x = (position_x_ < goal) ? 0.5 : -0.5;

    while (std::abs(position_x_.load() - goal) > 0.1) {
      // Check for cancel requests
      if (goal_handle->is_canceling()) {
        vel_publisher_->publish(stop);
        auto result = std::make_shared<MoveX::Result>();
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal cancelled");
        return;
      }

      vel_publisher_->publish(vel_x);

      // Publish feedback
      feedback_msg->remaining_x = std::abs(position_x_.load() - goal);
      goal_handle->publish_feedback(feedback_msg);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    vel_publisher_->publish(stop);

    auto result = std::make_shared<MoveX::Result>();
    result->success = true;
    result->final_x = position_x_.load();
    goal_handle->succeed(result);
  }

  void get_position_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    position_x_.store(msg->pose.pose.position.x);
  }
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment::SetPositionServer)