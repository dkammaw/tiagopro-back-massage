import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
import math

class MoveItIKExample(Node):
    def __init__(self):
        super().__init__('moveit_ik_example')

        # Initialisiere Action Client für MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_group_client.wait_for_server()
        self.get_logger().info("Connected to MoveGroup action server.")

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        # Zielpose erstellen
        target_pose = PoseStamped()
        target_pose.header.frame_id = "world"  # Passe den Frame an deinen Roboter an
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z

        # Eulerwinkel in Quaternion umwandeln
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        target_pose.pose.orientation.x = quat[0]
        target_pose.pose.orientation.y = quat[1]
        target_pose.pose.orientation.z = quat[2]
        target_pose.pose.orientation.w = quat[3]

        # MoveGroup-Goal definieren
        goal_msg = MoveGroup.Goal()
        goal_msg.request.start_state.is_diff = False
        goal_msg.request.goal_constraints.append(self.create_pose_constraint(target_pose))

        # Sende das Ziel an den MoveGroup-Action-Server zur Planung
        self.get_logger().info("Planning to target pose...")
        result = self.move_group_client.send_goal_async(goal_msg)
        
        # Plan Callback hinzufügen
        result.add_done_callback(self.plan_callback)

    def plan_callback(self, future):
        # Dies wird aufgerufen, wenn die Planung abgeschlossen ist
        result = future.result()
        if result:
            self.get_logger().info("Planning was successful.")
            # Wenn die Planung erfolgreich war, starte die Bewegung
            self.execute_move()
        else:
            self.get_logger().error("Planning failed. Unable to reach the target pose.")

    def execute_move(self):
        # Sende den Ausführungsbefehl und warte auf die Rückmeldung
        self.get_logger().info("Executing the planned move...")
        goal_msg = MoveGroup.Goal()
        goal_msg.request.start_state.is_diff = False

        # Sende den Befehl an MoveGroup und warte auf die Rückmeldung
        result = self.move_group_client.send_goal_async(goal_msg)
        result.add_done_callback(self.move_callback)  # Callback nach der Bewegung

    def move_callback(self, future):
        # Rückmeldung nach der Ausführung der Bewegung
        result = future.result()
        if result and result.status == 3:  # Erfolgreich abgeschlossen
            self.get_logger().info("Movement executed successfully!")
        else:
            self.get_logger().error("Movement failed.")


    def create_pose_constraint(self, pose_stamped):
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        constraints = Constraints()

        # Position Constraint
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = "arm_right_6_link"  # Passe den Endeffektor-Link an
        position_constraint.target_point_offset.x = pose_stamped.pose.position.x
        position_constraint.target_point_offset.y = pose_stamped.pose.position.y
        position_constraint.target_point_offset.z = pose_stamped.pose.position.z
        constraints.position_constraints.append(position_constraint)

        # Orientation Constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = "arm_right_6_link"
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Eulerwinkel (Roll, Pitch, Yaw) in Quaternion umwandeln."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    moveit_example = MoveItIKExample()

    # Beispielzielposition und -orientierung
    moveit_example.move_to_pose(0.763226, -0.413192, 0.350760, 1.570745, -0.497005, 2.792608)

    rclpy.spin(moveit_example)

if __name__ == '__main__':
    main()




