import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter

class ImpedanceGainAdjuster(Node):

    def __init__(self):
        super().__init__('impedance_gain_adjuster')

        # Subscribe to the joint_states topic to get current joint positions
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',  # This is the default topic for joint states in ROS 2
            self.joint_states_callback,
            20
        )

        # Define the initial impedance gains (will be updated dynamically)
        self.impedance_gains = {
            'arm_left_1_joint': {'kp': -2.5, 'kd': 0.0},
            'arm_left_2_joint': {'kp': -8.0, 'kd': 0.0},
            'arm_left_3_joint': {'kp': -2.5, 'kd': 0.0},
            'arm_left_4_joint': {'kp': 6.0, 'kd': 0.0},
            'arm_left_5_joint': {'kp': -2.0, 'kd': 0.0},
            'arm_left_6_joint': {'kp': -3.0, 'kd': -0.25},
            'arm_left_7_joint': {'kp': -2.0, 'kd': 0.0},
        }

        # Define the joint position thresholds for when to adjust the impedance gains
        self.position_thresholds = {
            'arm_left_1_joint': math.radians(140),
            'arm_left_2_joint': math.radians(-33),
            'arm_left_3_joint': math.radians(-114),
            'arm_left_4_joint': math.radians(-129),
            'arm_left_5_joint': math.radians(-32), 
            'arm_left_6_joint': math.radians(105), 
            'arm_left_7_joint': math.radians(-50)   
        }

    def joint_states_callback(self, msg):
        # Get the current joint positions
        joint_positions = dict(zip(msg.name, msg.position))

        # Check if the current joint positions meet the thresholds for impedance gain adjustment
        for joint, threshold in self.position_thresholds.items():
            if joint in joint_positions:
                # If the joint position exceeds the threshold, adjust the impedance gains
                if joint_positions[joint] == threshold:
                    self.get_logger().info(f'{joint} position exceeded threshold, adjusting gains...')
                    # Example: Adjust impedance gains for the affected joint
                    self.adjust_impedance_gain(joint)

    def adjust_impedance_gain(self, joint_name):
        """Adjust impedance gains based on joint name."""
        # Define new impedance gains for this joint (you can set different gains based on the joint's configuration)
        if joint_name == 'arm_left_1_joint':
            new_kp, new_kd = -5.0, -0.5
        elif joint_name == 'arm_left_2_joint':
            new_kp, new_kd = -9.0, -0.5
        elif joint_name == 'arm_left_3_joint':
            new_kp, new_kd = -3.0, 0.0
        elif joint_name == 'arm_left_4_joint':
            new_kp, new_kd = 7.0, 0.0
        else:
            # For other joints, use the default (you can customize this logic)
            new_kp, new_kd = self.impedance_gains.get(joint_name, {}).values()

        # Set the new impedance gains for the joint
        self.set_impedance_gains(joint_name, new_kp, new_kd)

    def set_impedance_gains(self, joint_name, kp, kd):
        """Sets the new impedance gains for a specific joint."""
        # Log the new gain values being set
        self.get_logger().info(f'Setting impedance gains for {joint_name}: kp={kp}, kd={kd}')
        
        # Create the parameters for the impedance gains
        param_kp = Parameter(joint_name + '_kp', Parameter.Type.DOUBLE, kp)
        param_kd = Parameter(joint_name + '_kd', Parameter.Type.DOUBLE, kd)

        # Set the new parameters on the node
        self.get_node_parameters_interface().set_parameters([param_kp, param_kd])


def main(args=None):
    rclpy.init(args=args)
    impedance_gain_adjuster = ImpedanceGainAdjuster()
    
    # Spin to keep the node active and process callbacks
    rclpy.spin(impedance_gain_adjuster)
    
    # Clean up after shutdown
    impedance_gain_adjuster.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

