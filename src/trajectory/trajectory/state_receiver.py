import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState

class StateReceiver(Node):
    def __init__(self):
        super().__init__('state_receiver')

        # Subscriber for the JointState topic
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for the RobotState topic
        self.state_receiver = self.create_publisher(RobotState, '/robot_state_topic', 10)

    def joint_state_callback(self, msg):
        # Create RobotState message
        robot_state_msg = RobotState()

        # Initialize joint_state explicitly
        robot_state_msg.joint_state = JointState()

        # List of joints
        arm_left_joints = ["arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", 
                           "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", 
                           "arm_left_7_joint"]

        # Update joint positions and names
        for joint in arm_left_joints:
            if joint in msg.name:
                index = msg.name.index(joint)
                position = msg.position[index]
                robot_state_msg.joint_state.name.append(joint)  # Add joint name
                robot_state_msg.joint_state.position.append(position)  # Add position
                # Log joint position
                self.get_logger().info(f"Joint: {joint}, Position: {position}")
            else:
                robot_state_msg.joint_state.name.append(joint)  # Add joint name with default position
                robot_state_msg.joint_state.position.append(0.0)  # Default position
                self.get_logger().info(f"Joint: {joint}, Position: 0.0 (default)")

        # Publish RobotState message
        self.state_receiver.publish(robot_state_msg)
        self.get_logger().info(f"Published RobotState for joints: {arm_left_joints}")

def main(args=None):
    rclpy.init(args=args)
    state_receiver = StateReceiver()
    rclpy.spin_once(state_receiver)  # Use spin to handle callbacks continuously
    rclpy.shutdown()

if __name__ == '__main__':
    main()


