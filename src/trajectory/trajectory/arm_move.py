import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
import math

class MoveItIKExample(Node):
    def __init__(self):
        super().__init__('moveit_ik_example')

        # Initialize Action Client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("Connected to MoveGroup action server.")

        # Initialize target_pose as None
        self.target_pose = None

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        # Create target pose
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "world"  # Adjust this frame to match your robot setup
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
        self.get_logger().info("Planning to target pose...")
        result = self.move_group_client.send_goal_async(goal_msg)

        # Add plan callback
        result.add_done_callback(self.plan_callback)

    def plan_callback(self, future):
        # This gets called when planning is complete
        result = future.result()
        if result:
            self.get_logger().info("Planning was successful.")
            # If planning was successful, execute the movement
            self.execute_move()
        else:
            self.get_logger().error("Planning failed. Unable to reach the target pose.")

    def execute_move(self):
        # Send execution command and wait for feedback
        self.get_logger().info("Executing the planned move...")

        if not self.target_pose:
            self.get_logger().error("Target pose not set!")
            return

        # Define MoveGroup Goal with constraints or target pose
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm_left"  # Ensure consistent use of planning group
        goal_msg.request.start_state.is_diff = False

        # Add the pose constraints again or use the target pose
        goal_msg.request.goal_constraints.append(self.create_pose_constraint(self.target_pose))

        # Send the command to MoveGroup and wait for feedback
        result = self.move_group_client.send_goal_async(goal_msg)
        result.add_done_callback(self.move_callback)  # Callback after the movement

    def move_callback(self, future):
        # Feedback after movement execution
        result = future.result()
        if result:
            if result.status == 3:  # Successfully completed
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
        position_constraint.target_point_offset.x = pose_stamped.pose.position.x
        position_constraint.target_point_offset.y = pose_stamped.pose.position.y
        position_constraint.target_point_offset.z = pose_stamped.pose.position.z
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

    # Example target position and orientation
    #moveit_example.move_to_pose(0.763226, -0.413192, 0.350760, 1.570745, -0.497005, 2.792608)
    moveit_example.move_to_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rclpy.spin(moveit_example)


if __name__ == '__main__':
    main()
   




