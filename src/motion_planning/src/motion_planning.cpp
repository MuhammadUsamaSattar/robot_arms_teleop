#include <memory>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

using namespace std::chrono_literals;

class MotionPlanningNode : public rclcpp::Node
{
public:
  MotionPlanningNode()
  : Node("motion_planning", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // Delay actual MoveGroup creation
    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&MotionPlanningNode::init, this)
    );
  }

private:
  void init()
  {
    timer_->cancel();

    std::string ns = this->get_namespace();
    std::string ns_no_slash = (ns.empty() || ns[0] != '/') ? ns : ns.substr(1);

    gripper_group_name_ = ns_no_slash + "_gripper";
    arm_group_name_     = ns_no_slash + "_arm";

    RCLCPP_INFO(this->get_logger(), "Gripper group: %s", gripper_group_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Arm group: %s", arm_group_name_.c_str());

    gripper_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), gripper_group_name_);
    arm_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), arm_group_name_);

    jaw_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      ns + "/claw_closed", 10,
      std::bind(&MotionPlanningNode::jaw_callback, this, std::placeholders::_1)
    );

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/goal_pose", 10,
      std::bind(&MotionPlanningNode::goal_pose_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s and %s", 
                (ns + "/claw_closed").c_str(),
                (ns + "/goal_pose").c_str());
  }

  void jaw_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    auto now = this->now();
    if ((now - last_jaw_callback_time_).seconds() < 0.5) {
      RCLCPP_DEBUG(this->get_logger(), "Skipping jaw_callback due to rate limiting");
      return;
    }
    last_jaw_callback_time_ = now;

    try
    {
      bool close_jaw = msg->data;
      std::string target_state = gripper_group_name_ + (close_jaw ? "_close" : "_open");

      RCLCPP_INFO(this->get_logger(), "Setting gripper target: %s", target_state.c_str());

      gripper_interface_->setNamedTarget(target_state);
      bool success = static_cast<bool>(gripper_interface_->move());

      if (success)
        RCLCPP_INFO(this->get_logger(), "Gripper moved to %s", target_state.c_str());
      else
        RCLCPP_ERROR(this->get_logger(), "Failed to move gripper to %s", target_state.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception in jaw_callback: %s", e.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in jaw_callback");
    }
  }

  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    auto now = this->now();
    if ((now - last_goal_callback_time_).seconds() < 0.5) {
      RCLCPP_DEBUG(this->get_logger(), "Skipping goal_pose_callback due to rate limiting");
      return;
    }
    last_goal_callback_time_ = now;

    try
    {
      RCLCPP_INFO(this->get_logger(), "Received goal pose");

      arm_interface_->setPoseTarget(*msg);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = static_cast<bool>(arm_interface_->plan(plan));

      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Planning successful. Executing...");
        arm_interface_->execute(plan);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Planning failed.");
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception in goal_pose_callback: %s", e.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in goal_pose_callback");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_interface_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr jaw_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
  std::string gripper_group_name_;
  std::string arm_group_name_;

  // Timestamp tracking
  rclcpp::Time last_jaw_callback_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_goal_callback_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionPlanningNode>());
  rclcpp::shutdown();
  return 0;
}
