#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <gripper_srv/srv/gripper_service.hpp>
int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_with_gripper");

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.position.x =-0.114413;
    msg.position.y = 0.452296;
    msg.position.z = 0.552035;
    msg.orientation.w = 0.085851;
    msg.orientation.x = 0.995389;
    msg.orientation.y = 0.041887;
    msg.orientation.z = 0.008711;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
auto const collision_object = [frame_id = move_group_interface.getPlanningFrame(), &node, &logger] {
   
    // Print the planning frame for debugging purposes
    RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());
 
    // Initialize a CollisionObject message
    moveit_msgs::msg::CollisionObject collision_object;
   
    // Set the frame ID for the collision object
    collision_object.header.frame_id = frame_id;
   
    // Set the timestamp to the current time
    collision_object.header.stamp = node->now();
   
    // Assign a unique ID to the collision object
    collision_object.id = "box1";
 
    // Define the shape of the collision object as a box
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
   
    // Set the dimensions of the box (in meters)
    primitive.dimensions[primitive.BOX_X] = 0.5;  // Width
    primitive.dimensions[primitive.BOX_Y] = 0.05;  // Depth
    primitive.dimensions[primitive.BOX_Z] = 0.20;  // Height
 
    // Define the pose (position and orientation) of the box
    geometry_msgs::msg::Pose box_pose;
   
    // Set the position of the box center
    box_pose.position.x = 0.26; // meters in x-direction
    box_pose.position.y = -0.36;   // Centered in y-direction
    box_pose.position.z = 0.33;  // meters in z-direction
   
    // Set the orientation of the box (no rotation in this case)
    box_pose.orientation.x = 1.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 0.0007;
 
    // Add the shape and pose to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
   
    // Set the operation to add the object to the planning scene
    collision_object.operation = collision_object.ADD;
 
    // Log information about the created collision object
    RCLCPP_INFO(logger, "Created collision object: %s", collision_object.id.c_str());
     
    RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f", 
      primitive.dimensions[primitive.BOX_X],
      primitive.dimensions[primitive.BOX_Y],
      primitive.dimensions[primitive.BOX_Z]);
       
    RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
      box_pose.position.x, box_pose.position.y, box_pose.position.z);
 
    // Return the fully defined collision object
    return collision_object;
  }();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
