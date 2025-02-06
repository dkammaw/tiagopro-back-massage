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
    
class FKSolver(Node):
    def __init__(self):
        super().__init__('arm_move_action_client')
        self.controller = '/arm_left_controller'
        self.action_client = ActionClient(self, FollowJointTrajectory, self.controller + "/follow_joint_trajectory")

    def setup_goal(self):
        duration1 = Duration()
        duration1.sec = 2
        duration1.nanosec = 0

        first_target = JointTrajectoryPoint()
        first_target.time_from_start = duration1
        
        first_target.positions = [
            0.275066,
            -1.26958,
            -1.9176,
            -1.44634,
            -0.662545,
            1.23098,
            -1.3148,


        ]


        self.goal = FollowJointTrajectory.Goal()
        self.goal.trajectory.joint_names = [
            "arm_left_1_joint",  
            "arm_left_2_joint",  
            "arm_left_3_joint",  
            "arm_left_4_joint",  
            "arm_left_5_joint",  
            "arm_left_6_joint",  
            "arm_left_7_joint"  
        ]

        # Add trajectory point
        self.goal.trajectory.points.append(first_target)



    def send_goal(self):

        duration_tolerance = Duration()
        duration_tolerance.sec = 5
        duration_tolerance.nanosec = 0

        goal = FollowJointTrajectory.Goal()
        goal.goal_time_tolerance = duration_tolerance
        goal.trajectory.joint_names = [
            "arm_left_1_joint", # rotation shoulder-internal 
            "arm_left_2_joint", # translation shoulder 
            "arm_left_3_joint", # rotation upper-arm-internal
            "arm_left_4_joint", # translation elbow 
            "arm_left_5_joint", # rotation under-arm-internal
            "arm_left_6_joint", # translation under-arm
            "arm_left_7_joint"  # rotation gripper 
        ]

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            self.goal,
            feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_callback)

    def goal_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            print(f"Goal rejected :(")
            return

        print(f"Goal accepted :)")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        print(f"Result for goal : {result.error_code}")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print("\nFeedback received:")
        for i, position_error in enumerate(feedback.error.positions):
            print(f"Position error for joint {i + 1}: {position_error}")


def main(args=None):
    rclpy.init(args=args)

    arm_action_client = FKSolver()
    arm_action_client.setup_goal()   # Setup goal
    arm_action_client.send_goal()    # Start first movement
    rclpy.spin(arm_action_client)


if __name__ == '__main__':
    main()