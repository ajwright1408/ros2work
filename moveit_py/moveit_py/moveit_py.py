import rclpy
from rclpy.node import Node
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import Pose

def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Initialize MoveIt Commander
    roscpp_initialize(args)
    
    # Create a ROS node
    node = Node('hello_moveit')

    # Initialize the RobotCommander and PlanningSceneInterface
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group = MoveGroupCommander('ur_manipulator')

    # Set a target Pose
    target_pose = Pose()
    target_pose.position.x = -0.118655
    target_pose.position.y = 0.448053
    target_pose.position.z = 0.557705
    target_pose.orientation.w = 0.079539
    target_pose.orientation.x = 0.995758
    target_pose.orientation.y = 0.045232
    target_pose.orientation.z = 0.009672
    group.set_pose_target(target_pose)

    # Create a plan to that target pose
    plan = group.plan()

    # Execute the plan
    if plan:
        group.go(wait=True)
    else:
        node.get_logger().error('Planning failed!')

    # Shutdown ROS and MoveIt Commander
    roscpp_shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

