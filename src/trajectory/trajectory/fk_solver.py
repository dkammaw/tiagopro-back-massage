import rclpy, time
from rclpy.action import ActionClient
from rclpy.node import Node

from numpy import pi
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import (     
    JointTrajectory,
    JointTrajectoryPoint,
)
from std_msgs.msg import Float64MultiArray
    
class FKSolver(Node):
    def __init__(self):
        super().__init__('arm_move_action_client')
        self.controller = '/arm_left_controller'
        self.action_client = ActionClient(self, FollowJointTrajectory, self.controller + "/follow_joint_trajectory")
        self.best_config_subscriber_ = self.create_subscription(
            Float64MultiArray,
            'best_config_topic',  # Name des Topics für die empfangene Konfiguration
            self.best_config_callback,
            10
        )

        self.best_config = None  # Speichert die empfangene beste Konfiguration

    def best_config_callback(self, msg):
        self.best_config = list(msg.data)  # Speichert empfangene Werte
        self.get_logger().info(f'Best configuration received: {self.best_config}')
        self.setup_goal()

    def setup_goal(self):
        duration1 = Duration()
        duration1.sec = 2
        duration1.nanosec = 0

        first_target = JointTrajectoryPoint()
        first_target.time_from_start = duration1
        
        first_target.positions = self.best_config
        
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
        self.send_goal()
        
    def setup_initial_goal(self):
        duration1 = Duration()
        duration1.sec = 2
        duration1.nanosec = 0

        initial_target = JointTrajectoryPoint()
        initial_target.time_from_start = duration1
        
        initial_target.positions = [
            3.61511491712742,
            -0.4368600062398942,
            1.74232659636899,
            -1.7351726149287279,
            -0.20690108233648452,
            2.5557943991060093,
            0.31778610106314265,                              
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
        self.goal.trajectory.points.append(initial_target)
        self.send_goal()

    def send_goal(self):
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            self.goal,
            feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_callback)

    def goal_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted!")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result for goal: {result.error_code}")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Feedback received:")
        for i, position_error in enumerate(feedback.error.positions):
            self.get_logger().info(f"Position error for joint {i + 1}: {position_error}")

def main(args=None):
    rclpy.init(args=args)
    arm_action_client = FKSolver()  
    arm_action_client.setup_initial_goal()
    arm_action_client.get_logger().info("Waiting for best configuration...")
    rclpy.spin(arm_action_client)


if __name__ == '__main__':
    main()
