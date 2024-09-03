ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.2 use_tool_communication:=true use_mock_hardware:=false launch_rviz:=true

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true description_package:=ur_description_gripper description_file:=ur5e_robotiq_2f_85_urdf.xacro moveit_config_package:=moveit_config moveit_config_file:=ur5e_robotiq.srdf

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.2 use_tool_communication:=true use_mock_hardware:=false launch_rviz:=true description_package:=ur_description_gripper description_file:=ur5e_robotiq_2f_85_urdf.xacro

ros2 launch urdf_tutorial display.launch.py model:=/home/robot/ros2_arm/src/ur_description_gripper/urdf/ur5e_robotiq_2f_85_urdf.xacro

conda activate /home/robot/anaconda3/envs/ur5
source /opt/ros/humble/setup.bash
source install/setup.bash 

ros2 launch ur_robotiq_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true description_package:=ur_robotiq_description moveit_config_package:=ur_robotiq_moveit_config moveit_config_name:=ur5e_with_robotiq_hande.xacro

ros2 launch robotiq_hande_ros2_driver gripper_bringup.launch.py
