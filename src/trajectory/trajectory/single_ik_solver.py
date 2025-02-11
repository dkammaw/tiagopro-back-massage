# SERVICE IMPLEMENTATION of Collision Object
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from moveit_msgs.msg import RobotState
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.srv import GetPlanningScene
import math

class MoveItIKExample(Node):
    def __init__(self):
        super().__init__('moveit_ik_example')

        # Initialize Action Client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("Connected to MoveGroup action server.")
        
        self.state2_pub= self.create_publisher(RobotState, 'robot_state_topic', 10)
        
        # Initialize target_pose as None
        self.target_pose = None
        
        self.planning_scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.get_logger().info("Connected to planning scene service.")
        import time
        time.sleep(2)
        #self.get_planning_scene() # output the object that has been added to the planning scene
        self.add_collision_object_human()  # Füge den Mensch als Hindernis hinzu

    def add_collision_object_human(self):
        while not self.planning_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for planning scene service...")

        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = "human"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.6, 0.6, 1.2]

        human_pose = PoseStamped()
        human_pose.header.frame_id = "base_link"
        human_pose.pose.position.x = 1.125
        human_pose.pose.position.y = 0.0
        human_pose.pose.position.z = 0.6

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(human_pose.pose)
        collision_object.operation = CollisionObject.ADD

        planning_scene = ApplyPlanningScene.Request()
        planning_scene.scene.world.collision_objects.append(collision_object)
        planning_scene.scene.is_diff = True

        # Send request and wait for response
        future = self.planning_scene_client.call_async(planning_scene)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info("Collision object for human added.")
        else:
            self.get_logger().error("Failed to add collision object.")
 
    
    '''
    def get_planning_scene(self):
        self.get_logger().info("Requesting current planning scene...")

        planning_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')

        while not planning_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for /get_planning_scene service...")

        request = GetPlanningScene.Request()

        future = planning_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            scene = future.result().scene
            self.get_logger().info(f"Received planning scene with {len(scene.world.collision_objects)} collision objects.")
            for obj in scene.world.collision_objects:
                self.get_logger().info(f"Collision Object: {obj.id} at {obj.primitive_poses[0].position}")
        else:
            self.get_logger().error("Failed to get planning scene.")
    '''
    
            
    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        
        # Create target pose
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "base_link" 
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z

        # Convert Euler angles to quaternion
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        self.target_pose.pose.orientation.x = quat[0]
        self.target_pose.pose.orientation.y = quat[1]
        self.target_pose.pose.orientation.z = quat[2]
        self.target_pose.pose.orientation.w = quat[3]

        # Define MoveGroup Goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm_left"  # Set to the planning group arm_left
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.goal_constraints.append(self.create_pose_constraint(self.target_pose))
        
        # Send the goal to MoveGroup action server for planning
        self.get_logger().info("Planning and executing to target pose...")
        result = self.move_group_client.send_goal_async(goal_msg)

        # Add plan callback
        result.add_done_callback(self.move_callback)
        
            
    def move_callback(self, future):
        # Feedback after movement execution
        result = future.result()
        if result:
            if result.status == 0 or result.status == 2:  # Successfully completed
                self.get_logger().info("Movement executed successfully!")
            else:
                self.get_logger().error(f"Movement failed with status: {result.status}")
        else:
            self.get_logger().error("Movement execution failed with no result.")


    def create_pose_constraint(self, pose_stamped):
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        constraints = Constraints()

        # Position Constraint
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = "arm_left_tool_link"  # Update to match the arm_left group end-effector

        # Define a constraint region as a box around the target position
        region_primitive = SolidPrimitive()
        region_primitive.type = SolidPrimitive.BOX
        region_primitive.dimensions = [0.01, 0.01, 0.01]  # Tolerances: 1 cm x 1 cm x 1 cm

        # Set the center of the region to the target pose's position
        position_constraint.constraint_region.primitives.append(region_primitive)
        position_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)

        # Add the position constraint to the list
        constraints.position_constraints.append(position_constraint)

        # Orientation Constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = "arm_left_tool_link"
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints


    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles (roll, pitch, yaw) to quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    moveit_example = MoveItIKExample()
    # Example target position and orientation (-0.8)
    moveit_example.move_to_pose(0.6, -0.11, 0.62, math.pi / 2, 0, math.pi / 2)
    rclpy.spin(moveit_example)


if __name__ == '__main__':
    main()    
 

