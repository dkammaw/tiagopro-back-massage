import subprocess
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
    
class TorsoLifter(Node):
    def __init__(self):
        super().__init__('torso_lift_action_client')
        self.controller = '/torso_controller'
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
        first_target.positions = [0.33]  

        # TARGET 2
        duration2 = Duration()
        duration2.sec = 5 
        duration2.nanosec = 0

        second_target = JointTrajectoryPoint()
        second_target.time_from_start = duration2
        second_target.positions = [0.22]  

        # TARGET 3
        duration3 = Duration()
        duration3.sec = 5
        duration3.nanosec = 0

        third_target = JointTrajectoryPoint()
        third_target.time_from_start = duration3
        third_target.positions = [0.11]  

        # Add targets to list
        self.goals.append(first_target)
        self.goals.append(second_target)
        self.goals.append(third_target)
        
            
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
        goal.trajectory.joint_names = ["torso_lift_joint"]

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
        
        # Execute additional nodes after each goal
        self.start_tapping_nodes()
        
        self.current_goal_index += 1  # Switch to next goal
        self.send_goal()  # Start next movement

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print("\nFeedback received:")
        for i, position_error in enumerate(feedback.error.positions):
            print(f"Position error for joint {i + 1}: {position_error}")

    def start_tapping_nodes(self):
        """
        Start the additional nodes using a launch file and wait for them to finish.
        """
        try:
            # Launch the file to start the other nodes
            print("Starting additional nodes...")
            process = subprocess.Popen(['ros2', 'launch', 'trajectory', 'tapping.launch.py'])
            process.wait()  # Wait for the process to finish
            print("Additional nodes completed.")
        except Exception as e:
            self.get_logger().error(f"Failed to start nodes: {e}")
            rclpy.shutdown()
            
def main(args=None):
    rclpy.init(args=args)

    arm_action_client = TorsoLifter()
    arm_action_client.setup_goals()  # Setup goals
    arm_action_client.send_goal()    # Start first movement
    rclpy.spin(arm_action_client)


if __name__ == '__main__':
    main()