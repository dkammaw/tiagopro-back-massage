import rclpy, time
from rclpy.action import ActionClient
from rclpy.node import Node

from numpy import pi
import math
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.srv import ListParameters
from trajectory_msgs.msg import (     
    JointTrajectory,
    JointTrajectoryPoint,
)
    
class RelaxActionClient(Node):
    def __init__(self):
        super().__init__('relax_action_client')

        # Action Clients for both arms
        self.left_controller = '/arm_left_controller'
        self.right_controller = '/arm_right_controller'

        self.left_action_client = ActionClient(self, FollowJointTrajectory, self.left_controller + "/follow_joint_trajectory")
        self.right_action_client = ActionClient(self, FollowJointTrajectory, self.right_controller + "/follow_joint_trajectory")

        self.left_goal = None
        self.right_goal = None

        # Tracking variables
        self.left_done = False
        self.right_done = False

    def setup_goals(self):
        # Setup durations
        duration = Duration(sec=5, nanosec=0)

        # Left arm single target
        self.left_goal = FollowJointTrajectory.Goal()
        self.left_goal.trajectory.joint_names = [
            "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint",
            "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint"
        ]
        self.left_goal.trajectory.points.append(
            JointTrajectoryPoint(
                time_from_start=duration,
                positions=[
                    math.radians(35), math.radians(-100), math.radians(-4),
                    math.radians(-133), math.radians(145), math.radians(91), math.radians(-60)
                ]
            )
        )
        self.left_goal.goal_time_tolerance = duration

        # Right arm single target
        self.right_goal = FollowJointTrajectory.Goal()
        self.right_goal.trajectory.joint_names = [
            "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint",
            "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"
        ]
        self.right_goal.trajectory.points.append(
            JointTrajectoryPoint(
                time_from_start=duration,
                positions=[
                    math.radians(-35), math.radians(-100), math.radians(4),
                    math.radians(-133), math.radians(-145), math.radians(91), math.radians(60)
                ]
            )
        )
        self.right_goal.goal_time_tolerance = duration

    def send_goal(self):
        # Wait for action servers
        self.left_action_client.wait_for_server()
        self.right_action_client.wait_for_server()

        # Send goals
        self.left_goal_future = self.left_action_client.send_goal_async(
            self.left_goal, feedback_callback=self.left_feedback_callback)
        self.right_goal_future = self.right_action_client.send_goal_async(
            self.right_goal, feedback_callback=self.right_feedback_callback)

        # Register callbacks
        self.left_goal_future.add_done_callback(self.left_goal_callback)
        self.right_goal_future.add_done_callback(self.right_goal_callback)
        print("Goals sent for both arms.")

    def left_goal_callback(self, future):
        self._handle_goal_callback(future, "left")

    def right_goal_callback(self, future):
        self._handle_goal_callback(future, "right")

    def _handle_goal_callback(self, future, arm_name):
        goal_handle = future.result()

        if not goal_handle.accepted:
            print(f"{arm_name.capitalize()} arm goal rejected :(")
            return

        print(f"{arm_name.capitalize()} arm goal accepted :)")

        if arm_name == "left":
            self.left_result_future = goal_handle.get_result_async()
            self.left_result_future.add_done_callback(self.left_result_callback)
        elif arm_name == "right":
            self.right_result_future = goal_handle.get_result_async()
            self.right_result_future.add_done_callback(self.right_result_callback)

    def left_result_callback(self, future):
        self._handle_result_callback(future, "left")

    def right_result_callback(self, future):
        self._handle_result_callback(future, "right")

    def _handle_result_callback(self, future, arm_name):
        try:
            result = future.result().result
            print(f"Result for {arm_name} arm: {result.error_code}")
        except Exception as e:
            print(f"Error in {arm_name} arm result callback: {e}")
            return

        # Mark the arm as done
        if arm_name == "left":
            self.left_done = True
        elif arm_name == "right":
            self.right_done = True

        # Shutdown when both arms are done
        if self.left_done and self.right_done:
            print("Both arms completed their tasks!")
            rclpy.shutdown()

    def left_feedback_callback(self, feedback_msg):
        self._handle_feedback_callback(feedback_msg, "left")

    def right_feedback_callback(self, feedback_msg):
        self._handle_feedback_callback(feedback_msg, "right")

    def _handle_feedback_callback(self, feedback_msg, arm_name):
        feedback = feedback_msg.feedback
        print(f"\n{arm_name.capitalize()} arm feedback received:")
        for i, position_error in enumerate(feedback.error.positions):
            print(f"Position error for joint {i + 1}: {position_error}")


def main(args=None):
    rclpy.init(args=args)

    dual_arm_client = RelaxActionClient()
    dual_arm_client.setup_goals()
    dual_arm_client.send_goal()
    rclpy.spin(dual_arm_client)


if __name__ == '__main__':
    main()
