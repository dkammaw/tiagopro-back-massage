import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time  # For time-based distance calculation


class Move(Node):
    def __init__(self):
        """
        Initialize the Move node, which moves the robot forward exactly 0.9m 
        and then stops.
        """
        super().__init__('move_node')
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 5)
        
        # Timer for movement control (executed every 50 ms)
        self.control_timer = self.create_timer(0.05, self.command_publisher)
        
        # Initialize velocity message
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        
        # Distance settings
        self.target_distance = 1.4  # Target distance in meters
        self.speed = 0.3  # Speed in m/s

        # Time tracking for distance calculation
        self.start_time = time.time()  # Record start time
        self.traveled_distance = 0.0  # Initialize traveled distance

    def command_publisher(self):
        """
        Publishes velocity commands to move the robot forward and stops it after 0.9m.
        """
        # Compute traveled distance based on time and speed
        elapsed_time = time.time() - self.start_time  
        self.traveled_distance = self.speed * elapsed_time  

        # Check if the robot has reached the target distance
        if self.traveled_distance >= self.target_distance:  
            self.cmd.linear.x = 0.0  # Stop immediately
            self.cmd_pub.publish(self.cmd)  # Send stop command
            self.get_logger().info(f"Target reached: {self.traveled_distance:.2f}m. Robot stopping.")
            self.destroy_node()  # Shut down the node
            return  

        # Continue moving forward
        self.cmd.linear.x = self.speed
        self.cmd_pub.publish(self.cmd)


def main(args=None):
    """
    Initializes the ROS2 node and starts the Move class.
    """
    rclpy.init(args=args)
    move_node = Move()

    try:
        rclpy.spin(move_node)
    finally:
        move_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



