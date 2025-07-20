#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_arms_teleop_interfaces/msg/combined_pose_stamped.hpp"

using namespace std::chrono_literals;

class MotionPlanningNode : public rclcpp::Node
{
public:
  MotionPlanningNode()
  : Node("motion_planning", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    //Call the init method for this node and creates another node for MoveIt2 on a separate thread
    //since the robot state becomes inaccessible otherwise
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

    // Creates an interface for each move group
    left_arm_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node_, "left_arm");
    right_arm_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node_, "right_arm");
    arms_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node_, "arms");
    left_gripper_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node_, "left_gripper");
    right_gripper_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node_, "right_gripper");

    // QoS of depth 1 is used to avoid building a queue of callbacks
    rclcpp::QoS qos_profile(1);
    qos_profile.best_effort();
    combined_pose_sub_ = this->create_subscription<robot_arms_teleop_interfaces::msg::CombinedPoseStamped>(
      "combined_goal_pose", qos_profile,
      std::bind(&MotionPlanningNode::combined_callback, this, std::placeholders::_1)
    );

    // Publish the pose for current goals
    left_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_left_goal_pose", 10);
    right_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_right_goal_pose", 10);

    RCLCPP_INFO(this->get_logger(), "Subscribed to /goal_poses");
  }

  void combined_callback(const robot_arms_teleop_interfaces::msg::CombinedPoseStamped::SharedPtr msg)
  {
    try {
      // Moves the left gripper to open/close
      std::string left_gripper_target = msg->left_jaw_closed ? "left_gripper_close" : "left_gripper_open";
      left_gripper_interface_->setNamedTarget(left_gripper_target);
      if (left_gripper_interface_->move()) {
        RCLCPP_INFO(this->get_logger(), "Left gripper moved to %s", left_gripper_target.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Left gripper move failed.");
      }

      // Moves the right gripper to open/close
      std::string right_gripper_target = msg->right_jaw_closed ? "right_gripper_close" : "right_gripper_open";
      right_gripper_interface_->setNamedTarget(right_gripper_target);
      if (right_gripper_interface_->move()) {
        RCLCPP_INFO(this->get_logger(), "Right gripper moved to %s", right_gripper_target.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Right gripper move failed.");
      }

      // Combined arm planning
      left_goal_pose_pub_->publish(msg->left);
      right_goal_pose_pub_->publish(msg->right);

      // Calculates the joint value for both arms using IK and builds a 
      // combined target of joint values for the combined arm move group
      left_arm_interface_->setJointValueTarget(msg->left);
      right_arm_interface_->setJointValueTarget(msg->right);

      std::vector<double> leftJointValue;
      std::vector<double> rightJointValue;
      std::vector<double> bothJointsValue;

      left_arm_interface_->getJointValueTarget(leftJointValue);
      right_arm_interface_->getJointValueTarget(rightJointValue);

      bothJointsValue.insert(bothJointsValue.end(), leftJointValue.begin(),
                            leftJointValue.end());
      bothJointsValue.insert(bothJointsValue.end(), rightJointValue.begin(),
                            rightJointValue.end());

      arms_interface_->setJointValueTarget(bothJointsValue);

      // Motion planning using the combined arm move group to avoid collisions 
      // between arms as well
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (arms_interface_->plan(plan)) {
        RCLCPP_INFO(this->get_logger(), "Arm plan succeeded.");
        arms_interface_->execute(plan);
      } else {
        RCLCPP_WARN(this->get_logger(), "Arm planning failed.");
      }

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in combined_callback: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Unknown exception in combined_callback");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::SharedPtr moveit_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arms_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_gripper_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_gripper_interface_;

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
