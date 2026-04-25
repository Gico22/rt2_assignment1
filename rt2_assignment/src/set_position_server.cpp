#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rt2_interfaces/action/move_x.hpp"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <chrono>
#include <cmath>
#include <optional>

namespace rt2_assignment
{

class SetPositionServer : public rclcpp::Node
{
public:
  using MoveX = rt2_interfaces::action::MoveX;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveX>;

  explicit SetPositionServer(const rclcpp::NodeOptions & options) : Node("set_position_server", options)
  {
    using namespace std::placeholders;

    // Create the callback group from the node and pass it explicitly to each subscriber/server
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_; 

    // Create action server
    // We have three separate callbacks instead of a single one:
    _action_server = rclcpp_action::create_server<MoveX>(
      this, "/MoveX",
      std::bind(&SetPositionServer::handle_goal, this, _1, _2),
      std::bind(&SetPositionServer::handle_cancel, this, _1),
      std::bind(&SetPositionServer::handle_accepted, this, _1),
      rcl_action_server_get_default_options(),
      cb_group_);

    // Create velocity publisher
    vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    pos_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&SetPositionServer::get_position_callback, this, _1), sub_options);

    // tf2 setup: the Buffer stores the transform tree,
    // the TransformListener populates it by subscribing to /tf and /tf_static
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //Odom tf broadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Started server component");
  }

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_; 
  rclcpp_action::Server<MoveX>::SharedPtr _action_server;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_subscriber_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped odom_t;

  // 3 callbacks
  // handle_goal → decides whether to accept or reject the incoming goal
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const MoveX::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: %.2f, %.2f, %.2f", goal->goal_x, goal->goal_y, goal->goal_theta);
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
     RCLCPP_INFO(this->get_logger(), "handle_accepted called, spawning thread");
    std::thread{
      std::bind(&SetPositionServer::execute_callback, this, goal_handle)
    }.detach();
  }

  void execute_callback(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    auto feedback_msg = std::make_shared<MoveX::Feedback>();
    auto goal = goal_handle->get_goal();

    geometry_msgs::msg::Twist vel, stop;

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Control loop
    // 1) Rotate to face the goal
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {return cancel_execution(goal_handle, stop);}

      // lookup gives us the goal position expressed in the robot's own frame
      auto t = lookup("base_link", "goal_frame");
      if (!t) {std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue;}

      double heading_err = std::atan2(t->transform.translation.y, t->transform.translation.x);
      if (std::abs(heading_err) < 0.01) break;

      vel.linear.x  = 0.0;
      vel.angular.z = 1.5 * heading_err;
      vel_publisher_->publish(vel);

      feedback_msg->remaining_distance = std::hypot(t->transform.translation.x, t->transform.translation.y);
      feedback_msg->remaining_theta = std::abs(heading_err*(180/M_PI));
      goal_handle->publish_feedback(feedback_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 2) Drive to the goal
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {return cancel_execution(goal_handle, stop);}

      auto t = lookup("base_link", "goal_frame");
      if (!t) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }

      double dist = std::hypot(t->transform.translation.x, t->transform.translation.y);
      double heading_err = std::atan2(t->transform.translation.y, t->transform.translation.x);
      if (dist < 0.05) break;

      vel.linear.x  = std::min(0.5, 0.5 * dist);  // slow down near goal
      vel.angular.z = 1.0 * heading_err;          // keep heading corrected
      vel_publisher_->publish(vel);

      feedback_msg->remaining_distance = dist;
      feedback_msg->remaining_theta = std::abs(heading_err*(180/M_PI));
      goal_handle->publish_feedback(feedback_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 3) Rotate to final theta
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) { return cancel_execution(goal_handle, stop); }

      auto t = lookup("world", "base_link");
      if (!t) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }

      double dist = std::hypot(t->transform.translation.x, t->transform.translation.y);
      double current_yaw = tf2::getYaw(t->transform.rotation);
      double th_err = normalize_angle(goal->goal_theta - current_yaw);
      if (std::abs(th_err) < 0.01) break;

      vel.linear.x  = 0.0;
      vel.angular.z = 1.5 * th_err;
      vel_publisher_->publish(vel);

      feedback_msg->remaining_distance = dist;
      feedback_msg->remaining_theta = std::abs(th_err*(180/M_PI));
      goal_handle->publish_feedback(feedback_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    vel_publisher_->publish(stop);

    auto t_final = lookup("world", "base_link");
    auto result  = std::make_shared<MoveX::Result>();
    result->success = true;
    if (t_final) {
      result->final_x = t_final->transform.translation.x;
      result->final_y = t_final->transform.translation.y;
      result->final_theta = tf2::getYaw(t_final->transform.rotation)*(180/M_PI);
    }
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
  }

  void get_position_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id  = "base_footprint";

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  std::optional<geometry_msgs::msg::TransformStamped> lookup(const std::string & target, const std::string & source)
  {
    try {
      return tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", e.what());
      return std::nullopt;
    }
  }

  void cancel_execution(const std::shared_ptr<GoalHandle> & goal_handle, const geometry_msgs::msg::Twist & stop)
  {
    vel_publisher_->publish(stop);
    auto result = std::make_shared<MoveX::Result>();
    result->success = false;
    goal_handle->canceled(result);
    RCLCPP_INFO(this->get_logger(), "Goal cancelled");
  }

  double normalize_angle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment::SetPositionServer)