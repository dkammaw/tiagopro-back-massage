import rclpy
from rclpy.node import Node
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
import math

class MoveItIKExample(Node):
    def __init__(self):
        super().__init__('moveit_ik_example')

        # Initialisiere MoveIt!-Komponenten
        rclpy.init()

        # RobotCommander und MoveGroupCommander für den rechten Arm (arm_right)
        self.robot_commander = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("arm_right")  # Verwende den Armnamen aus deiner Launch-Konfiguration

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        # Zielpose erstellen
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z

        # Eulerwinkel in Quaternion umwandeln
        quat = self.euler_to_quaternion(roll, pitch, yaw)
        target_pose.orientation.x = quat[0]
        target_pose.orientation.y = quat[1]
        target_pose.orientation.z = quat[2]
        target_pose.orientation.w = quat[3]

        # Zielpose setzen
        self.group.set_pose_target(target_pose)

        # Plane die Bewegung und führe sie aus
        plan = self.group.go(wait=True)
        if plan:
            self.get_logger().info(f"Move successful to position {target_pose.position}")
        else:
            self.get_logger().info("Move failed")

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
    moveit_example.move_to_pose(0.5, 0.0, 0.3, math.radians(0), math.radians(90), math.radians(0))

    rclpy.spin(moveit_example)

if __name__ == '__main__':
    main()


