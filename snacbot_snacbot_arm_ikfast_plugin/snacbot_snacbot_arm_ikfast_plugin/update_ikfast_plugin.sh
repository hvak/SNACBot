search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=snacbot.srdf
robot_name_in_srdf=snacbot
moveit_config_pkg=snacbot_moveit
robot_name=snacbot
planning_group_name=snacbot_arm
ikfast_plugin_pkg=snacbot_snacbot_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=ee_link
ikfast_output_path=/home/hersh/eecs467_ws/src/SNACBot/snacbot_snacbot_arm_ikfast_plugin/snacbot_snacbot_arm_ikfast_plugin/src/snacbot_snacbot_arm_ikfast_solver.cpp

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
