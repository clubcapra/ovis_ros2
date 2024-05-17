search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=ovis.srdf
robot_name_in_srdf=ovis
moveit_config_pkg=ovis_moveit_config
robot_name=ovis
planning_group_name=arm
ikfast_plugin_pkg=ovis_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=ovis_end_effector
ikfast_output_path=/home/capra/rove/rove/src/ovis_arm_ikfast_plugin/src/ovis_arm_ikfast_solver.cpp

ros2 run moveit_kinematics create_ikfast_moveit_plugin.py\
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
