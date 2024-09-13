#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gripper_srv/srv/gripper_service.hpp>
int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto gripper_client = node->create_client<gripper_srv::srv::GripperService>("gripper_service");
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_gripper");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
  request->position = 0; 
  request->speed = 255;
  request->force = 255;
  // Call the gripper service asynchronously
  auto future = gripper_client->async_send_request(request);

  // Wait for the response
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
  {
      RCLCPP_INFO(logger, "Gripper opened successfully");
  }
  else
  {
      RCLCPP_ERROR(logger, "Failed to open gripper");
  }
  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_with_gripper");
  for (const auto& str : move_group_interface.getCurrentJointValues()) {
    RCLCPP_INFO(logger, "%2f",str);
  }
  // Set a target Pose

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
  planning_scene_interface.applyCollisionObject(collision_object);
auto const vial = [frame_id = move_group_interface.getPlanningFrame(), &node, &logger] {
   
    // Print the planning frame for debugging purposes
    RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());
 
    // Initialize a CollisionObject message
    moveit_msgs::msg::CollisionObject vial;
   
    // Set the frame ID for the collision object
    vial.header.frame_id = frame_id;
   
    // Set the timestamp to the current time
    vial.header.stamp = node->now();
   
    // Assign a unique ID to the collision object
    vial.id = "vial";
 
    // Define the shape of the collision object as a box
    shape_msgs::msg::SolidPrimitive primitivev;
    primitivev.type = primitivev.BOX;
    primitivev.dimensions.resize(3);
   
    // Set the dimensions of the box (in meters)
    primitivev.dimensions[primitivev.BOX_X] = 0.0125;  // Width
    primitivev.dimensions[primitivev.BOX_Y] = 0.0125;  // Depth
    primitivev.dimensions[primitivev.BOX_Z] = 0.045;  // Height
 
    // Define the pose (position and orientation) of the box
    geometry_msgs::msg::Pose box_pose2;
    // Set the position of the box center
    box_pose2.position.x = -0.13; // meters in x-direction
    box_pose2.position.y = 0.38;   // Centered in y-direction
    box_pose2.position.z = 0.07; // meters in z-direction
   
    // Set the orientation of the box (no rotation in this case)
    box_pose2.orientation.x = 1.0;
    box_pose2.orientation.y = 0.0;
    box_pose2.orientation.z = 0.0;
    box_pose2.orientation.w = 0.000;
 
    // Add the shape and pose to the collision object
    vial.primitives.push_back(primitivev);
    vial.primitive_poses.push_back(box_pose2);
   
    // Set the operation to add the object to the planning scene
    vial.operation = vial.ADD;
 
    // Log information about the created collision object
    RCLCPP_INFO(logger, "Created collision object: %s", vial.id.c_str());
     
    RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f", 
      primitivev.dimensions[primitivev.BOX_X],
      primitivev.dimensions[primitivev.BOX_Y],
      primitivev.dimensions[primitivev.BOX_Z]);
       
    RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
      box_pose2.position.x, box_pose2.position.y, box_pose2.position.z);
 
    // Return the fully defined collision object
    return vial;
  }();
  planning_scene_interface.applyCollisionObject(vial);

  auto const ceiling = [frame_id = move_group_interface.getPlanningFrame(), &node, &logger] {
   
    // Print the planning frame for debugging purposes
    RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());
 
    // Initialize a CollisionObject message
    moveit_msgs::msg::CollisionObject ceiling;
   
    // Set the frame ID for the collision object
    ceiling.header.frame_id = frame_id;
   
    // Set the timestamp to the current time
    ceiling.header.stamp = node->now();
   
    // Assign a unique ID to the collision object
    ceiling.id = "ceiling";
 
    // Define the shape of the collision object as a box
    shape_msgs::msg::SolidPrimitive primitivev;
    primitivev.type = primitivev.BOX;
    primitivev.dimensions.resize(3);
   
    // Set the dimensions of the box (in meters)
    primitivev.dimensions[primitivev.BOX_X] = 5.0;  // Width
    primitivev.dimensions[primitivev.BOX_Y] = 0.0125;  // Depth
    primitivev.dimensions[primitivev.BOX_Z] = 5.0;  // Height
 
    // Define the pose (position and orientation) of the box
    geometry_msgs::msg::Pose box_pose2;
    // Set the position of the box center
    box_pose2.position.x = 0.0; // meters in x-direction
    box_pose2.position.y = 0.4;   // Centered in y-direction
    box_pose2.position.z = 0.7; // meters in z-direction
   
    // Set the orientation of the box (no rotation in this case)
    box_pose2.orientation.x = 0.47;
    box_pose2.orientation.y = 0.52;
    box_pose2.orientation.z = 0.49;
    box_pose2.orientation.w = 0.51;
 
    // Add the shape and pose to the collision object
    ceiling.primitives.push_back(primitivev);
    ceiling.primitive_poses.push_back(box_pose2);
   
    // Set the operation to add the object to the planning scene
    ceiling.operation = ceiling.ADD;
 
    // Log information about the created collision object
    RCLCPP_INFO(logger, "Created collision object: %s", ceiling.id.c_str());
     
    RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f", 
      primitivev.dimensions[primitivev.BOX_X],
      primitivev.dimensions[primitivev.BOX_Y],
      primitivev.dimensions[primitivev.BOX_Z]);
       
    RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
      box_pose2.position.x, box_pose2.position.y, box_pose2.position.z);
 
    // Return the fully defined collision object
    return ceiling;
  }();
  planning_scene_interface.applyCollisionObject(ceiling);
  auto const wall1 = [frame_id = move_group_interface.getPlanningFrame(), &node, &logger] {
   
    // Print the planning frame for debugging purposes
    RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());
 
    // Initialize a CollisionObject message
    moveit_msgs::msg::CollisionObject wall1;
   
    // Set the frame ID for the collision object
    wall1.header.frame_id = frame_id;
   
    // Set the timestamp to the current time
    wall1.header.stamp = node->now();
   
    // Assign a unique ID to the collision object
    wall1.id = "wall1";
 
    // Define the shape of the collision object as a box
    shape_msgs::msg::SolidPrimitive primitivev;
    primitivev.type = primitivev.BOX;
    primitivev.dimensions.resize(3);
   
    // Set the dimensions of the box (in meters)
    primitivev.dimensions[primitivev.BOX_X] = 5.0;  // Width
    primitivev.dimensions[primitivev.BOX_Y] = 0.0125;  // Depth
    primitivev.dimensions[primitivev.BOX_Z] = 5.0;  // Height
 
    // Define the pose (position and orientation) of the box
    geometry_msgs::msg::Pose box_pose2;
    // Set the position of the box center
    box_pose2.position.x = 0.0; // meters in x-direction
    box_pose2.position.y = -0.3;   // Centered in y-direction
    box_pose2.position.z = 0.8; // meters in z-direction
   
    // Set the orientation of the box (no rotation in this case)
    box_pose2.orientation.x = 0.0;
    box_pose2.orientation.y = 0.0;
    box_pose2.orientation.z = 1.0;
    box_pose2.orientation.w = 0.0;
 
    // Add the shape and pose to the collision object
    wall1.primitives.push_back(primitivev);
    wall1.primitive_poses.push_back(box_pose2);
   
    // Set the operation to add the object to the planning scene
    wall1.operation = wall1.ADD;
 
    // Log information about the created collision object
    RCLCPP_INFO(logger, "Created collision object: %s", wall1.id.c_str());
     
    RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f", 
      primitivev.dimensions[primitivev.BOX_X],
      primitivev.dimensions[primitivev.BOX_Y],
      primitivev.dimensions[primitivev.BOX_Z]);
       
    RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
      box_pose2.position.x, box_pose2.position.y, box_pose2.position.z);
 
    // Return the fully defined collision object
    return wall1;
  }();
  planning_scene_interface.applyCollisionObject(wall1);

  auto const wall2 = [frame_id = move_group_interface.getPlanningFrame(), &node, &logger] {
   
    // Print the planning frame for debugging purposes
    RCLCPP_INFO(logger, "Planning frame: %s", frame_id.c_str());
 
    // Initialize a CollisionObject message
    moveit_msgs::msg::CollisionObject wall2;
   
    // Set the frame ID for the collision object
    wall2.header.frame_id = frame_id;
   
    // Set the timestamp to the current time
    wall2.header.stamp = node->now();
   
    // Assign a unique ID to the collision object
    wall2.id = "wall2";
 
    // Define the shape of the collision object as a box
    shape_msgs::msg::SolidPrimitive primitivev;
    primitivev.type = primitivev.BOX;
    primitivev.dimensions.resize(3);
   
    // Set the dimensions of the box (in meters)
    primitivev.dimensions[primitivev.BOX_X] = 5.0;  // Width
    primitivev.dimensions[primitivev.BOX_Y] = 0.0125;  // Depth
    primitivev.dimensions[primitivev.BOX_Z] = 5.0;  // Height
 
    // Define the pose (position and orientation) of the box
    geometry_msgs::msg::Pose box_pose2;
    // Set the position of the box center
    box_pose2.position.x = 0.0; // meters in x-direction
    box_pose2.position.y = 0.5;   // Centered in y-direction
    box_pose2.position.z = 0.8; // meters in z-direction
   
    // Set the orientation of the box (no rotation in this case)
    box_pose2.orientation.x = 0.0;
    box_pose2.orientation.y = 1.0;
    box_pose2.orientation.z = 0.0;
    box_pose2.orientation.w = 0.0;
 
    // Add the shape and pose to the collision object
    wall2.primitives.push_back(primitivev);
    wall2.primitive_poses.push_back(box_pose2);
   
    // Set the operation to add the object to the planning scene
    wall2.operation = wall2.ADD;
 
    // Log information about the created collision object
    RCLCPP_INFO(logger, "Created collision object: %s", wall2.id.c_str());
     
    RCLCPP_INFO(logger, "Box dimensions: %.2f x %.2f x %.2f", 
      primitivev.dimensions[primitivev.BOX_X],
      primitivev.dimensions[primitivev.BOX_Y],
      primitivev.dimensions[primitivev.BOX_Z]);
       
    RCLCPP_INFO(logger, "Box position: (%.2f, %.2f, %.2f)",
      box_pose2.position.x, box_pose2.position.y, box_pose2.position.z);
 
    // Return the fully defined collision object
    return wall2;
  }();
  planning_scene_interface.applyCollisionObject(wall2);


  // Create a plan to that target pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.position.x =-0.125447;
    msg.position.y = 0.379617;
    msg.position.z = 0.204256;
    msg.orientation.w = 0.013801;
    msg.orientation.x = 0.998445;
    msg.orientation.y = 0.051797;
    msg.orientation.z = 0.015303;
    return msg;
  }();
  auto const current_pose=[]{
    geometry_msgs::msg::Pose msg;
    msg.position.x =-0.125456;
    msg.position.y = 0.379510;
    msg.position.z = 0.282653;

    return msg;
  }();
  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.header.frame_id = move_group_interface.getPoseReferenceFrame();
  box_constraint.link_name = move_group_interface.getEndEffectorLink();
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = { 0.1, 0.4, 0.4 };
  box_constraint.constraint_region.primitives.emplace_back(box);
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = current_pose.position.x;
  RCLCPP_INFO(logger,"%.2f",box_pose.position.x);
  box_pose.position.y = 0.15;
  box_pose.position.z = 0.45;
  box_pose.orientation.w = 1.0;
  box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
  box_constraint.weight = 1.0;
  moveit_msgs::msg::Constraints box_constraints;
  box_constraints.position_constraints.emplace_back(box_constraint);
  //move_group_interface.setPathConstraints(box_constraints);
  //move_group_interface.setPlanningTime(60.0);
  //move_group_interface.setNumPlanningAttempts(1000);
  move_group_interface.setPoseTarget(target_pose);
  
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);

    // Wait for the gripper service to be available
    while (!gripper_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(logger, "Gripper service not available, waiting again...");
    }

    // Create a request to close the gripper
    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
    request->position = 255;  
    request->speed = 255;
    request->force = 255;

    // Call the gripper service asynchronously
    auto future = gripper_client->async_send_request(request);

    // Wait for the response
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(logger, "Gripper closed successfully");
    }
    else
    {
        RCLCPP_ERROR(logger, "Failed to close gripper");
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
//Pick and place
  auto const target_place = [] {
    geometry_msgs::msg::Pose msg2;
    msg2.position.x =-0.433011;
    msg2.position.y = 0.013971;
    msg2.position.z = 0.203771;
    msg2.orientation.w = 0.013801;
    msg2.orientation.x = 0.998445;
    msg2.orientation.y = 0.051797;
    msg2.orientation.z = 0.015303;
    return msg2;
  }();
  move_group_interface.setPlanningTime(60.0);
  move_group_interface.setNumPlanningAttempts(1000);
  move_group_interface.setPoseTarget(target_place);
  auto const [success2, plan2] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg2;
    auto const ok2 = static_cast<bool>(move_group_interface.plan(msg2));
    return std::make_pair(ok2, msg2);
  }();

  // Execute the plan
  if (success2)
  {
    move_group_interface.execute(plan2);

    // Wait for the gripper service to be available
    while (!gripper_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(logger, "Gripper service not available, waiting again...");
    }

    // Create a request to close the gripper
    auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
    request->position = 0;  
    request->speed = 255;
    request->force = 255;

    // Call the gripper service asynchronously
    auto future = gripper_client->async_send_request(request);

    // Wait for the response
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(logger, "Gripper closed successfully");
    }
    else
    {
        RCLCPP_ERROR(logger, "Failed to close gripper");
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

void open(){

}