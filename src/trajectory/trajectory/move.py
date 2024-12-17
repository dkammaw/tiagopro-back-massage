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
            0.05,  # 20 Hz control loop
            self.command_publisher
        )
        
        # Initialize the velocity command message
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        
        # Initialize variables for laser scan data
        self.front = float('inf')  # Use 'inf' to represent no obstacle detected
        self.cluster_count = 0  # Count of clusters in front of the robot
        
        # Constants for distance thresholds
        self.min_distance = 0.75  # Minimum distance to stop
        self.cluster_threshold = 0.25  # Maximum gap between points in a cluster
        self.cluster_min_points = 6  # Minimum points to consider a cluster valid

        # Variable to track if the nodes have been started
        self.nodes_started = False

    def sensor_callback(self, msg: LaserScan):
        """
        Callback for processing LaserScan data. Updates the 'front' distance and cluster count.
        """
        ranges = msg.ranges
        if ranges:
            # Define the front view range (e.g., +/- 150 points from center)
            front_ranges = ranges[len(ranges)//2 - 150 : len(ranges)//2 + 150]

            # Calculate the minimum distance in the front view
            self.front = min(front_ranges) if front_ranges else float('inf')

            # Detect clusters (e.g., chair legs or other narrow obstacles)
            self.cluster_count = self.detect_clusters(front_ranges)
        else:
            self.front = float('inf')  # Default to no obstacle detected
            self.cluster_count = 0

    def detect_clusters(self, ranges):
        """
        Detect clusters of points in the given range data.

        Args:
            ranges (list): List of range data from the LaserScan message.

        Returns:
            int: Number of clusters detected.
        """
        clusters = 0
        current_cluster_size = 0
        
        for i in range(1, len(ranges)):
            if ranges[i] < float('inf') and abs(ranges[i] - ranges[i-1]) < self.cluster_threshold:
                current_cluster_size += 1
            else:
                if current_cluster_size >= self.cluster_min_points:
                    clusters += 1
                current_cluster_size = 0

        # Check the last cluster
        if current_cluster_size >= self.cluster_min_points:
            clusters += 1

        return clusters

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

        # Stop the robot if an obstacle is detected within the threshold distance or clusters are found
        if self.front < self.min_distance and self.cluster_count > 0:
            linear_vel = 0.0
            self.get_logger().info(f"Obstacle detected at {self.front:.2f} meters or {self.cluster_count} clusters. Stopping.")
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
