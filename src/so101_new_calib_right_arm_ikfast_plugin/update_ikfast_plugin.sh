search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=so101_new_calib.srdf
robot_name_in_srdf=so101_new_calib
moveit_config_pkg=so101_new_calib_moveit_config
robot_name=so101_new_calib
planning_group_name=right_arm
ikfast_plugin_pkg=so101_new_calib_right_arm_ikfast_plugin
base_link_name=combined_base
eef_link_name=right_gripper
ikfast_output_path=/home/usama/robot_arms_teleop/src/so101_new_calib_right_arm_ikfast_plugin/src/so101_new_calib_right_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
