import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from std_msgs.msg import Float32MultiArray
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import MarkerArray
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import CollisionObject
import math
import time
import subprocess
import os


class MoveItIKExample(Node):
    def __init__(self):
        super().__init__('moveit_ik_example')
        
        # Subscribe to tapping positions
        self.create_subscription(Float32MultiArray, '/tapping_positions', self.tapping_positions_callback, 10)
        self.get_logger().info("Subscribed to tapping positions topic.")
        
        # Initialize Action Client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("Connected to MoveGroup action server.")

        # Publisher for visualization markers
        self.marker_publisher = self.create_publisher(MarkerArray, '/visualization_marker', 10)
        self.get_logger().info("Created publisher for visualization markers.")
        
        
        self.planning_scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.get_logger().info("Connected to planning scene service.")
        
        self.target_pose = None
        self.movement_status = False  # Initialize as False  
        
        time.sleep(2)
        self.add_collision_object_human()  # Füge den Mensch als Hindernis hinzu

    def add_collision_object_human(self):
        while not self.planning_scene_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Waiting for planning scene service...")

        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = "human"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.0, 0.8, 1.2]

        human_pose = PoseStamped()
        human_pose.header.frame_id = "base_link"
        human_pose.pose.position.x = 0.75
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
        goal_msg.request.goal_constraints.append(self.create_pose_constraint(self.target_pose))
        goal_msg.request.allowed_planning_time = 5.0
        # goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.start_state.is_diff = True

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

    def tapping_positions_callback(self, msg):
        """
        Callback to process tapping positions and move the robot to each position.
        """
        # Start the subprocess
        '''
        subprocess.run(
            ['ros2', 'run', 'trajectory', 'move'],
            check=True,
            preexec_fn=lambda: os.sched_setaffinity(0, {3})  # Nutzt nur CPU-Kern 2
        )'''
        
        # Reshape tapping positions into Nx3 array
        positions = np.array(msg.data).reshape(-1, 3)  
        self.get_logger().info(f"Received {len(positions)} tapping positions.")

        # Adjusting distance of tapping positions to the tip of the gripper
        positions[:, 0] -= 0.23 
        # Move to each position sequentially
        for index, pos in enumerate(positions):
            x, y, z = map(float, pos)  # Explicit casting
            roll, pitch, yaw = math.pi / 2, 0, math.pi / 2  # Fixed orientation
            
            # Plan and execute the move
            self.get_logger().info(f"Planning to position {index + 1}: x={x}, y={y}, z={z}")
            self.move_to_pose(x, y, z, roll, pitch, yaw)

            # Wait until the movement is complete or timeout
            self.get_logger().info("Waiting for the movement to complete...")
            timeout = 10.0  # 10 seconds timeout
            start_time = time.time()
            while not self.movement_completed():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - start_time > timeout:
                    self.get_logger().error("Timeout waiting for movement to complete.")
                    break

            if self.movement_completed():
                self.get_logger().info(f"Position {index + 1} reached successfully.")
            else:
                self.get_logger().error(f"Failed to reach position {index + 1}. Skipping to next.")

            # Wait for 5 seconds before moving to the next position
            self.get_logger().info("Waiting for 5 seconds before moving to the next position...")
            time.sleep(5)

    def movement_completed(self):
        """
        Check if the movement has been successfully completed.
        """
        return self.movement_status

    def create_pose_constraint(self, pose_stamped):
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        constraints = Constraints()

        # Position Constraint
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = "arm_left_tool_link"
        region_primitive = SolidPrimitive()
        region_primitive.type = SolidPrimitive.BOX
        region_primitive.dimensions = [0.01, 0.01, 0.01]
        position_constraint.constraint_region.primitives.append(region_primitive)
        position_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
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

    moveit_example.get_logger().info("MoveIt IK Example node started.")
    moveit_example.get_logger().info("Waiting for tapping positions...")

    try:
        rclpy.spin(moveit_example)
    except KeyboardInterrupt:
        moveit_example.get_logger().info("Node interrupted by user.")
    finally:
        moveit_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




