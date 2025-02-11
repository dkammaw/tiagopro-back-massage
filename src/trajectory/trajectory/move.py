import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Move(Node):
    def __init__(self):
        super().__init__('move_node')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.speed = 0.3  # Speed in m/s
        self.target_distance = 2.2  # Target distance in meters
        self.start_time = None  # Initialize when movement starts

    def move_forward(self):
        move_cmd = Twist()
        move_cmd.linear.x = self.speed

        self.start_time = self.get_clock().now()  # Record actual start time

        while rclpy.ok():
            elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9  # Convert to seconds
            traveled_distance = self.speed * elapsed_time

            if traveled_distance >= self.target_distance:
                break  # Stop when distance is reached

            self.cmd_pub.publish(move_cmd)
            rclpy.spin_once(self, timeout_sec=0.01)  # Ensure constant updates

        # Send Stop Command Multiple Times to Ensure Stop
        stop_cmd = Twist()
        for _ in range(10):  # Send stop command multiple times
            self.cmd_pub.publish(stop_cmd)
            rclpy.spin_once(self, timeout_sec=0.001)  

        self.get_logger().info(f"Target reached: {traveled_distance:.2f}m. Robot stopping.")

def main(args=None):
    rclpy.init(args=args)
    node = Move()
    node.move_forward()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# LIDAR BASED
'''
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Move(Node):
    def __init__(self):
        super().__init__('move_node')
        
        # Subscription für LaserScan-Daten
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_front_raw',
            self.sensor_callback,
            10
        )
        
        # Publisher für Velocity-Kommandos
        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            5
        )
        
        # Timer zur Steuerung der Bewegung
        self.control_timer = self.create_timer(
            0.05,  
            self.command_publisher
        )
        
        # Initialisiere die Twist-Nachricht
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        
        # Variable zur Speicherung des Messwertes für den vorderen Bereich
        self.front = float('inf')
        
        # Distanzschwelle, ab der gestoppt werden soll
        self.min_distance = 0.75
        

    def sensor_callback(self, msg: LaserScan):
        """
        Callback zum Verarbeiten der LaserScan-Daten. Aktualisiert den 'front'-Wert.
        """
        self.get_logger().info("LaserScan message received!")
        ranges = msg.ranges
        
        # Berechne die Indizes für den interessierenden Bereich
        center_index = len(ranges) // 2
        start_index = max(0, center_index - 150)
        end_index = min(len(ranges), center_index + 150)
        
        # Hole den relevanten Abschnitt des Scan-Arrays
        slice_ranges = ranges[start_index:end_index]
        
        # Filtere nur gültige Messwerte (größer 0 und endlich)
        valid_ranges = [r for r in slice_ranges if r > 0 and math.isfinite(r)]
        
        if valid_ranges:
            self.front = min(valid_ranges)
        else:
            self.front = float('inf')  # Keine gültigen Messwerte gefunden
        
        self.get_logger().debug(f"Valid ranges: {valid_ranges}")
        self.get_logger().debug(f"Computed front: {self.front}")

    def command_publisher(self):
        """
        Steuert die Bewegung des Roboters basierend auf den LaserScan-Daten.
        """
        # Standard-Geschwindigkeit
        linear_vel = 0.1

        # Stoppe den Roboter, wenn ein Hindernis innerhalb des Schwellenwerts liegt
        if self.front < self.min_distance:
            linear_vel = 0.0
            self.get_logger().info(f"Obstacle detected at {self.front:.2f} meters. Stopping.")
        else:
            self.get_logger().info("Path is clear. Moving forward.")

        # Aktualisiere das Velocity-Kommando
        self.cmd.linear.x = linear_vel
        self.cmd.angular.z = 0.0

        # Sende das Kommando
        self.cmd_pub.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    move_node = Move()

    try:
        rclpy.spin(move_node)
    except KeyboardInterrupt:
        pass
    finally:
        move_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
  
            
       




