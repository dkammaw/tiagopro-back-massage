import rclpy, time
from rclpy.action import ActionClient
from rclpy.node import Node

from numpy import pi
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.srv import ListParameters
from trajectory_msgs.msg import (     
    JointTrajectory,
    JointTrajectoryPoint,
)
    
class TappingActionClient(Node):
    def __init__(self):
        super().__init__('arm_move_action_client')
        self.controller = '/arm_right_controller'
        self.action_client = ActionClient(self, FollowJointTrajectory, self.controller + "/follow_joint_trajectory")
        self.current_goal_index = 0
        self.goals = []  # Liste der Bewegungsziele

    def setup_goals(self):
        duration1 = Duration()
        duration1.sec = 5
        duration1.nanosec = 0

        first_target = JointTrajectoryPoint()
        first_target.time_from_start = duration1
        first_target.positions = [
            -1.8577, # pi/2, # rotation shoulder-internal 0
            -1.5916, # 0.2,    # translation shoulder 3
            0.35538, # -1.0,    # rotation upper-arm-internal 0
            -2.0502, # pi/2+0.4,  # translation elbow 3
            0.10524, # 2.0,     # rotation under-arm-internal
            -1.5976, # -0.1,   # translation under-arm -0,5
            1.57 # 0.0       # rotation gripper 0
        ]

        duration2 = Duration()
        duration2.sec = 5
        duration2.nanosec = 0

        second_target = JointTrajectoryPoint()
        second_target.time_from_start = duration2
        second_target.positions = [
            0.0,  # Neue Zielposition 1
            3.0,  # Neue Zielposition 2
            0.0,  # Neue Zielposition 3
            3.0,  # Neue Zielposition 4
            0.0,  # Neue Zielposition 5
            -0.5,  # Neue Zielposition 6
            0.0  # Neue Zielposition 7
        ]

        # Ziele zur Liste hinzufügen
        self.goals.append(first_target)
        self.goals.append(second_target)

    def send_goal(self):
        if self.current_goal_index ==  len(self.goals):
            print("Alle Bewegungen abgeschlossen.")
            rclpy.shutdown()
            return

        duration_tolerance = Duration()
        duration_tolerance.sec = 1
        duration_tolerance.nanosec = 0

        target = self.goals[self.current_goal_index]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.points.append(target)
        goal.goal_time_tolerance = duration_tolerance
        goal.trajectory.joint_names = [
            "arm_right_1_joint",
            "arm_right_2_joint",
            "arm_right_3_joint",
            "arm_right_4_joint",
            "arm_right_5_joint",
            "arm_right_6_joint",
            "arm_right_7_joint"
        ]

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_callback)
        print(f"Ziel {self.current_goal_index + 1} gesendet.")

    def goal_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            print(f"Ziel {self.current_goal_index + 1} abgelehnt :(")
            return

        print(f"Ziel {self.current_goal_index + 1} angenommen :)")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        print(f"Ergebnis für Ziel {self.current_goal_index + 1}: {result.error_code}")
        self.current_goal_index += 1  # Zum nächsten Ziel wechseln
        self.send_goal()  # Nächste Bewegung starten

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print("\nZwischen-Feedback erhalten:")
        for i, position_error in enumerate(feedback.error.positions):
            print(f"Position Fehler für Gelenk {i + 1}: {position_error}")


def main(args=None):
    rclpy.init(args=args)

    arm_action_client = TappingActionClient()
    arm_action_client.setup_goals()  # Ziele vorbereiten
    arm_action_client.send_goal()  # Erste Bewegung starten
    rclpy.spin(arm_action_client)


if __name__ == '__main__':
    main()
    