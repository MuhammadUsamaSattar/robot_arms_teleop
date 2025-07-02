#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

int main(int argc, char * argv[]) {
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "motion_planning",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("motion_planning");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface_left_arm = MoveGroupInterface(node, "left_arm");
  
  // Set a target Pose
  // auto const target_pose = []{
  // geometry_msgs::msg::Pose msg;

  // msg.position.x = -0.267000;
  // msg.position.y = -0.121000;
  // msg.position.z = 0.277000;
  // msg.orientation.x = 0.500000;
  // msg.orientation.y = 0.500000;
  // msg.orientation.z = 0.500000;
  // msg.orientation.w = 0.500000;
  // return msg;
  // }();
  // move_group_interface_left_arm.setPoseTarget(target_pose);
  // move_group_interface_left_arm.setGoalTolerance(0.01);

  // RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface_left_arm.getPlanningFrame().c_str());
  // RCLCPP_INFO(logger, "End effector link: %s", move_group_interface_left_arm.getEndEffectorLink().c_str());

  // // Create a plan to that target pose
  // auto const [success, plan] = [&move_group_interface_left_arm]{
  // moveit::planning_interface::MoveGroupInterface::Plan msg;
  // auto const ok = static_cast<bool>(move_group_interface_left_arm.plan(msg));
  // return std::make_pair(ok, msg);
  // }();
  // 
  // // Execute the plan
  // if(success) {
  // move_group_interface_left_arm.execute(plan);
  // } else {
  // RCLCPP_ERROR(logger, "Planning failed!");
  // }

  move_group_interface_left_arm.setNamedTarget("left_arm_default");
  move_group_interface_left_arm.move();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}