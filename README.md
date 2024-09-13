1. First cd into folder the ros environment is stored in and enter next commands to source ros and activate the environment
conda activate /home/robot/anaconda3/envs/ur5
source /opt/ros/humble/setup.bash
source install/setup.bash 

2. This command launches the driver for the arm and allows data to be published in the ros environment
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.2 use_tool_communication:=true use_mock_hardware:=false launch_rviz:=true

3. This activates moveit and sets up the ros environment to include the gripper
ros2 launch ur_robotiq_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true description_package:=ur_robotiq_description moveit_config_package:=ur_robotiq_moveit_config moveit_config_name:=ur5e_with_robotiq_hande.xacro

4. This command activates the gripper driver
ros2 launch robotiq_hande_ros2_driver gripper_bringup.launch.py

5. This is example script to grasp vial and move it slightly using joint, to do end effector position use moveit_gripper 
ros2 run moveit_gripper moveit_gripperway

6.ros2 run moveit_pose moveit_pose will print the robots current pose

7.Opencvtools can be used to publish and subscribe to a camera feed so that camera can be used to detect colour and aruco stamps

8. ros2 can be called with launch or run depending on if launch file or script is run. Taking the command ros2 run moveit_gripper(1) moveit_gripper(2) (1) is the package name, and (2) is the name of the file, without the file type due to how the package is set up (in the setup.py/cmakeslist.txt and package.xml)

9. UR5eWithRobotiqHandE includes the files to include the robotiq hand with the robot arm, and the config files for moveit. For example, there is an alternative kinematics.yaml file in the config folder which you can replace the kinematics.yaml to change what kinematic plugin is used.

10.pick_vial demonstrates how you can move the arm without moveit, and the positions are loaded from the config file and you run them via a launch file.
