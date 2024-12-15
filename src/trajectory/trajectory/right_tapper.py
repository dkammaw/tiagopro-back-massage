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
    
class RightTapper(Node):
    def __init__(self):
        super().__init__('arm_move_action_client')
        self.controller = '/arm_right_controller'
        self.action_client = ActionClient(self, FollowJointTrajectory, self.controller + "/follow_joint_trajectory")
        self.current_goal_index = 0
        self.goals = []  # list of targets

    def setup_goals(self):
        
        # TARGET 1
        duration1 = Duration()
        duration1.sec = 5
        duration1.nanosec = 0

        first_target = JointTrajectoryPoint()
        first_target.time_from_start = duration1
        first_target.positions = [
            # initial massage config    #single arm     #home config
            math.radians(-140),          #-70            #-1.8577,
            math.radians(-33),           #-76            #-1.5916, 
            math.radians(114),           #94             #0.35538,
            math.radians(-129),          #-125           #-2.0502,
            math.radians(32),            #60             #0.10524,
            math.radians(105),           #75             #-1.5976,
            math.radians(50)             #15             #1.57 
        ]

        # TARGET 2
        duration2 = Duration()
        duration2.sec = 7 # tapping velocity needs to be lower
        duration2.nanosec = 0

        second_target = JointTrajectoryPoint()
        second_target.time_from_start = duration2
        second_target.positions = self.tapping(first_target.positions)
        
        # TARGET 3
        duration3 = Duration()
        duration3.sec = 7 # tapping velocity needs to be lower
        duration3.nanosec = 0

        third_target = JointTrajectoryPoint()
        third_target.time_from_start = duration2
        third_target.positions = first_target.positions
        
        
        # Add targets to list
        self.goals.append(first_target)
        self.goals.append(second_target)
        self.goals.append(third_target)
        
        
    # Function to target positions for each tapping motion similarly 
    def tapping(self, positions):
        grad_changes = [2, 7, -10, 21, -11, -30, -7]  # Updates in degrees
        new_positions = []
    
        for i in range(len(positions)):
            # Add Update in degrees to positions and convert back to radians
            new_pos = positions[i] + math.radians(grad_changes[i])
            new_positions.append(new_pos)
    
        return new_positions
            

    def send_goal(self):
        if self.current_goal_index ==  len(self.goals):
            print("All movements completed!")
            rclpy.shutdown()
            return

        duration_tolerance = Duration()
        duration_tolerance.sec = 5
        duration_tolerance.nanosec = 0

        target = self.goals[self.current_goal_index]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.points.append(target)
        goal.goal_time_tolerance = duration_tolerance
        goal.trajectory.joint_names = [
            "arm_right_1_joint", # rotation shoulder-internal 
            "arm_right_2_joint", # translation shoulder 
            "arm_right_3_joint", # rotation upper-arm-internal
            "arm_right_4_joint", # translation elbow 
            "arm_right_5_joint", # rotation under-arm-internal
            "arm_right_6_joint", # translation under-arm
            "arm_right_7_joint"  # rotation gripper 
        ]

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_callback)
        print(f"Goal {self.current_goal_index + 1} sent.")

    def goal_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            print(f"Goal {self.current_goal_index + 1} rejected :(")
            return

        print(f"Goal {self.current_goal_index + 1} accepted :)")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        print(f"Result for goal {self.current_goal_index + 1}: {result.error_code}")
        self.current_goal_index += 1  # Switch to next goal
        self.send_goal()  # Start next movement

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print("\nFeedback received:")
        for i, position_error in enumerate(feedback.error.positions):
            print(f"Position error for joint {i + 1}: {position_error}")


def main(args=None):
    rclpy.init(args=args)

    arm_action_client = RightTapper()
    arm_action_client.setup_goals()  # Setup goals
    arm_action_client.send_goal()    # Start first movement
    rclpy.spin(arm_action_client)


if __name__ == '__main__':
    main()