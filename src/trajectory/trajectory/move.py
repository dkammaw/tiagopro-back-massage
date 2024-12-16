import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Move(Node):
    def __init__(self):
        super().__init__('move_node')
        
        # Create a subscription to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_front_raw',
            self.sensor_callback,
            10
        )
        
        # Create a publisher for publishing velocity commands
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            5
        )
        
        # Create a timer for controlling the robot's movement
        self.control_timer = self.create_timer(
            0.05,  
            self.command_publisher
        )
        
        # Initialize the velocity command message
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        
        # Initialize variables for laser scan data
        self.front = float('inf')  # Use 'inf' to represent no obstacle detected
        
        # Constants for distance thresholds
        self.min_distance = 0.75

        # Variable to track if the nodes have been started
        self.nodes_started = False

    def sensor_callback(self, msg: LaserScan):
        """
        Callback for processing LaserScan data. Updates the 'front' distance.
        """
        # Calculate the range directly in front of the robot
        ranges = msg.ranges
        if ranges:
            # Assuming the front is the center of the scan
            self.front = min(ranges[len(ranges)//2 - 150 : len(ranges)//2 + 150])
        else:
            self.front = float('inf')  # Default to no obstacle detected

    def start_lifter_node(self):
        """
        Start the additional nodes using a launch file.
        """
        try:
            # Launch the file to start the other nodes
            subprocess.Popen(['ros2', 'run', 'trajectory', 'torsoLifter'])
            self.nodes_started = True  # Prevent multiple launches
        except Exception as e:
            self.get_logger().error(f"Failed to start nodes: {e}")

    def command_publisher(self):
        """
        Control the robot's movement based on laser scan data.
        """
        # Default linear velocity
        linear_vel = 0.1

        # Stop the robot if an obstacle is detected within the threshold distance
        if self.front < self.min_distance:
            linear_vel = 0.0
            self.get_logger().info(f"Obstacle detected at {self.front:.2f} meters. Stopping.")
            if not self.nodes_started:
                self.start_lifter_node()
        else:
            self.get_logger().info(f"Path is clear. Moving forward.")

        # Update the velocity command
        self.cmd.linear.x = linear_vel
        self.cmd.angular.z = 0.0  # Adjust angular velocity if needed

        # Publish the velocity command
        self.cmd_pub.publish(self.cmd)


def main(args=None):
    """
    Main function to initialize the ROS2 node and run the Move class.
    """
    rclpy.init(args=args)

    # Create an instance of the Move class
    move_node = Move()

    try:
        # Spin the node to handle callbacks
        rclpy.spin(move_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the node
        move_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()