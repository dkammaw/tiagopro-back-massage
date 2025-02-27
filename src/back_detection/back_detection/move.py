import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MoveExactDistance(Node):
    def __init__(self):
        super().__init__('move_exact_distance')
        self.odom_sub = self.create_subscription(Odometry, '/mobile_base_controller/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.initial_x = None
        self.target_distance = 0.84  # Drive in x direction (destination similar as projection of the back)
        self.speed = 0.25  # velocity in m/s

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x

        if self.initial_x is None:
            self.initial_x = current_x  # Save the start position

        distance_traveled = abs(current_x - self.initial_x)

        if distance_traveled < self.target_distance:
            twist = Twist()
            twist.linear.x = self.speed  # Drive foward by the defined speed
            self.cmd_vel_pub.publish(twist)
        else:
            self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info("Destination reached, robot stopped!")
        self.destroy_node()  # End the node

def main():
    rclpy.init()
    move_node = MoveExactDistance()
    try:
        rclpy.spin(move_node)
    finally:
        move_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

  
            
       




