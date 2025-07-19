#include <memory>
#include <string>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_arms_teleop_interfaces/msg/combined_pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

using namespace std::chrono_literals;

class MotionPlanningNode : public rclcpp::Node
{
public:
  MotionPlanningNode()
  : Node("motion_planning", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    moveit_node_ = std::make_shared<rclcpp::Node>("move_group_interface");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(moveit_node_);
    executor_thread_ = std::thread([this]() {
      RCLCPP_INFO(moveit_node_->get_logger(), "MoveIt executor thread started");
      executor_->spin();
    });

    timer_ = this->create_wall_timer(100ms, std::bind(&MotionPlanningNode::init, this));
  }

  ~MotionPlanningNode()
  {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

private:
  void init()
  {
    timer_->cancel();

    left_arm_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node_, "left_arm");
    right_arm_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node_, "right_arm");
    left_gripper_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node_, "left_gripper");
    right_gripper_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node_, "right_gripper");

    rclcpp::QoS qos_profile(1);
    qos_profile.best_effort();
    combined_pose_sub_ = this->create_subscription<robot_arms_teleop_interfaces::msg::CombinedPoseStamped>(
      "/combined_goal_pose", qos_profile,
      std::bind(&MotionPlanningNode::combined_callback, this, std::placeholders::_1)
    );

    left_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_left_goal_pose", 10);
    right_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_right_goal_pose", 10);

    RCLCPP_INFO(this->get_logger(), "Subscribed to /goal_poses");
  }

  void combined_callback(const robot_arms_teleop_interfaces::msg::CombinedPoseStamped::SharedPtr msg)
  {
    try {
      // --- Left Arm Planning ---
      left_goal_pose_pub_->publish(msg->left);
      left_arm_interface_->setPoseTarget(msg->left);
      moveit::planning_interface::MoveGroupInterface::Plan left_plan;
      if (left_arm_interface_->plan(left_plan)) {
        RCLCPP_INFO(this->get_logger(), "Left arm plan succeeded.");
        left_arm_interface_->execute(left_plan);
      } else {
        RCLCPP_WARN(this->get_logger(), "Left arm planning failed.");
      }

      // --- Right Arm Planning ---
      right_goal_pose_pub_->publish(msg->right);
      right_arm_interface_->setPoseTarget(msg->right);
      moveit::planning_interface::MoveGroupInterface::Plan right_plan;
      if (right_arm_interface_->plan(right_plan)) {
        RCLCPP_INFO(this->get_logger(), "Right arm plan succeeded.");
        right_arm_interface_->execute(right_plan);
      } else {
        RCLCPP_WARN(this->get_logger(), "Right arm planning failed.");
      }

      // --- Left Gripper ---
      std::string left_gripper_target = msg->left_jaw_closed ? "left_gripper_close" : "left_gripper_open";
      left_gripper_interface_->setNamedTarget(left_gripper_target);
      if (left_gripper_interface_->move()) {
        RCLCPP_INFO(this->get_logger(), "Left gripper moved to %s", left_gripper_target.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Left gripper move failed.");
      }

      // --- Right Gripper ---
      std::string right_gripper_target = msg->right_jaw_closed ? "right_gripper_close" : "right_gripper_open";
      right_gripper_interface_->setNamedTarget(right_gripper_target);
      if (right_gripper_interface_->move()) {
        RCLCPP_INFO(this->get_logger(), "Right gripper moved to %s", right_gripper_target.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Right gripper move failed.");
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in combined_callback: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in combined_callback");
    }
  }

  // MoveIt executor
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::SharedPtr moveit_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  // MoveGroupInterfaces
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_gripper_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_gripper_interface_;

  // Subscriber
  rclcpp::Subscription<robot_arms_teleop_interfaces::msg::CombinedPoseStamped>::SharedPtr combined_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_goal_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_goal_pose_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionPlanningNode>());
  rclcpp::shutdown();
  return 0;
}
