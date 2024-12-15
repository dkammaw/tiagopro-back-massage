import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

class FTSensorHandler(Node):
    def __init__(self):
        super().__init__('ft_sensor_handler')

        # Subscribe to the FT sensor's topic
        self.ft_sub = self.create_subscription(
            WrenchStamped,
            '/ft_sensor_data',  # Replace with actual FT sensor topic
            self.ft_callback,
            10
        )
        
        # Threshold for contact detection
        self.force_threshold = 5.0  # Adjust based on application

    def ft_callback(self, msg: WrenchStamped):
        """
        Callback to process FT sensor data.
        """
        force = msg.wrench.force
        torque = msg.wrench.torque

        # Example: Print the force and torque values
        self.get_logger().info(f"Force: x={force.x:.2f}, y={force.y:.2f}, z={force.z:.2f}")
        self.get_logger().info(f"Torque: x={torque.x:.2f}, y={torque.y:.2f}, z={torque.z:.2f}")

        # Example: Detect contact based on the Z-axis force
        if abs(force.z) > self.force_threshold:
            self.get_logger().info("Contact detected! Reducing pressure...")
            # Implement control logic to reduce pressure or stop movement

def main(args=None):
    rclpy.init(args=args)

    ft_node = FTSensorHandler()
    rclpy.spin(ft_node)

    ft_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
