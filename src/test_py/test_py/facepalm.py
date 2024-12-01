
from action_tutorials_interfaces.action import Fibonacci

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

class FacepalmActionClient(Node):

    def __init__(self):
        super().__init__('arm_move_action_client')
        self.controller='/arm_right_controller'
        self.action_client = ActionClient(self, FollowJointTrajectory, self.controller + "/follow_joint_trajectory")

    def send_goal(self):
        duration = Duration()
        duration.sec = 5
        duration.nanosec = 0

        target = JointTrajectoryPoint()
        target.time_from_start = duration
        target.positions=[
            0.0, # pi/2, # rotation forward shoulder
            1.1, # 0.2,    # rotation sidewards shoulder
            0.0, # -1.0,    # rotation arm-internal
            -1.57, # pi/2+0.4,  # rotation ellbow
            0.0, # 2.0,
            0.5, # -0.1,
            0.0 # 0.0
        ]
        for i in target.positions:
            print("target position", i)

        duration.sec = 0
        duration.nanosec = 500000000

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.points.append(target)
        goal.goal_time_tolerance=duration
        goal.trajectory.joint_names=[
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
        print("sent goal")

    def goal_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            print("Goal rejected :(")
            return

        print("Goal Accepted :)")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        print("got final result:", result.error_code)
        print("Done. Shutting down")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        print("\ngot feedback")
        for i in feedback.error.positions:
            print("position error", i)

def main(args=None):
    rclpy.init(args=args)

    arm_action_client = FacepalmActionClient()

    arm_action_client.send_goal()

    rclpy.spin(arm_action_client)


if __name__ == '__main__':
    main()