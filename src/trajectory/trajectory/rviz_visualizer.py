import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MeshMarkerPublisher(Node):
    def __init__(self):
        super().__init__('mesh_marker_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_marker)

    def publish_marker(self):
        self.get_logger().info('Publishing marker...')
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'male_visitor'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        # Position of the model
        marker.pose.position.x = 3.08988
        marker.pose.position.y = -0.03417
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Scale (1.0 means original size)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        # Color
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Path to the mesh file
        marker.mesh_resource = '/opt/pal/alum/share/pal_gazebo_worlds_private/models/MaleVisitorSit/meshes/MaleVisitorSit.obj'

        self.publisher.publish(marker)
        self.get_logger().info('Marker published successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = MeshMarkerPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
