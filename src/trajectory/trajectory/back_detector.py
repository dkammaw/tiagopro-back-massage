import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
from sklearn.decomposition import PCA
import numpy as np
from sklearn.cluster import DBSCAN
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation
from sensor_msgs_py import point_cloud2 as pc2
import subprocess
import matplotlib.pyplot as plt

class BackDetector(Node):
    def __init__(self):
        super().__init__('back_detector')
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Define QoS profile for Best Effort (to match the publisher)
        qos_profile_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Match the publisher's QoS
            history=HistoryPolicy.KEEP_LAST,
            depth=300
        )
        
        # Subscriptions
        self.create_subscription(PointCloud2, '/head_front_camera/depth/color/points', self.pointcloud_callback, qos_profile_best_effort)
        # Comment out the following line to visualize human mask
        # self.create_subscription(Image, '/head_front_camera/color/image_raw', self.image_callback, qos_profile_best_effort)
        
        # Publisher for tapping positions
        self.tapping_positions_pub = self.create_publisher(Float32MultiArray, '/tapping_positions', 10)

        # Publisher for point cloud
        self.back_cloud_pub = self.create_publisher(PointCloud2, '/filtered_cloud', 10)
        
        # Publisher for visualization markers
        self.marker_publisher = self.create_publisher(MarkerArray, '/rviz_visual_tools', 10)

        # OpenCV bridge
        self.bridge = CvBridge()

        # Storage
        self.point_cloud = None
        self.rgb_image = None
        self.human_mask = None
        
        self.received_cloud = False


    def frame_transformation(self, msg):
        """
        Transform the point cloud from the camera frame into base_footprint frame.
        """
        transform = self.tf_buffer.lookup_transform(
            'base_footprint',  # Target frame
             msg.header.frame_id,  # Source frame
             rclpy.time.Time(),  # Time (use latest available)
             timeout=rclpy.duration.Duration(seconds=1.0)
        )

        # Read the points from the PointCloud2 message and convert to numpy array
        points_msg = pc2.read_points_numpy(msg)

        # Create a new array with 4 columns (x, y, z, 1 for homogeneous)
        points = np.hstack([points_msg, np.ones((points_msg.shape[0], 1))])

        # Get the transform (rotation and translation)
        rotation_matrix = Rotation.from_quat([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
             transform.transform.rotation.w
         ]).as_matrix()

        translation_vector = np.array([transform.transform.translation.x,
                                          transform.transform.translation.y,
                                          transform.transform.translation.z]) 
        
        points_transformed = np.dot(points[:, :3], rotation_matrix.T) + translation_vector  

        # Create a new PointCloud2 message with the transformed points
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
            
        pcl_header = msg.header
        pcl_header.frame_id = "base_footprint"
            
        # Create the PointCloud2 message
        return pc2.create_cloud(pcl_header, fields, points_transformed)


    def pointcloud_callback(self, msg):
        """
        Callback function for receiving the point cloud from the RGBD camera.
        """
        if self.received_cloud == False:
            pcl_msg = self.frame_transformation(msg)
            
            # Converts pc to array
            self.point_cloud  = pc2.read_points_numpy(pcl_msg, field_names=("x", "y", "z"), skip_nans=True)
            
            if self.point_cloud is not None and len(self.point_cloud) > 0:
                self.get_logger().info(f"Received transformed point cloud with {len(self.point_cloud)} points.")
                # if self.human_mask is not None:
                    # self.filter_points_by_human_mask()
                self.process_back_detection()
            else:
                self.get_logger().info("No valid point cloud data received.")
        
        self.received_cloud = True 
            
            
    def image_callback(self, msg):
        """
        Callback function for receiving the rgb-image from the RGBD camera.
        """
        # Convert ROS Image message to OpenCV format
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_human_from_rgb(self.rgb_image)
              
    
    
    def process_back_detection(self):
        """
        Identify points corresponding to the back by filtering based on their alignment.
        """
        # Apply heuristic to extract back (e.g., points with vertical alignment)
        back_points = self.filter_vertical_points(self.point_cloud)
        
        # Select largest cluster (which is the Back)
        back_points = self.get_largest_cluster(back_points)
        
        self.publish_processed_pointcloud(back_points)
        '''
        if back_points is not None:
            self.get_logger().info(f"Detected back region with {len(back_points)} points.")

            try:
                # Start the subprocess of moving the robot towards the human
                process = subprocess.Popen(['ros2', 'run', 'trajectory', 'move'])
                # Wait for up to 13 seconds for the process to finish
                process.wait(timeout=13)
            except subprocess.TimeoutExpired:
                print("Process did not finish in 13 seconds. Continuing with other tasks.")
                
            # Publish the processed point cloud
            self.publish_processed_pointcloud(back_points)
            # Calculate tapping positions
            self.calculate_tapping_positions(back_points)
        
        else:
            self.get_logger().info("No back points detected.")'''
        

        
    def filter_vertical_points(self, points):
        """
        Identify points corresponding to the back by filtering based on their alignment.
        """
        # Define the ranges for filtering based on the coordinate system:
        # Z is height (vertical), X is depth (front-back), Y is width (side-to-side)

        y_min, y_max = -0.3, 0.3   # Width range (side-to-side, in meters) - adjust based on the body width
           
        # Filtering for width (y-coordinate)
        filtered_points = points[(points[:, 1] > y_min) & (points[:, 1] < y_max)]
        self.get_logger().info(f"Filtered by Y axes: {len(filtered_points)} points remaining.")
        
        z_min, z_max = 0.5, filtered_points[:, 2].max()    # Height range (vertical, in meters) - adjust based on the expected back height

        # Filtering for height (z-coordinate)
        filtered_points = filtered_points[(filtered_points[:, 2] > z_min) & (filtered_points[:, 2] < z_max)]
        self.get_logger().info(f"Filtered by Z-axis: {len(filtered_points)} points remaining.")
        
        x_min, x_max = filtered_points[:, 0].min(), filtered_points[:, 0].min() + 0.3   # Depth range (front-back, in meters) - adjust based on the body depth
        
        # Filter for depth (x-coordinate)
        filtered_points = filtered_points[(filtered_points[:, 0] > x_min) & (filtered_points[:, 0] < x_max)]
        self.get_logger().info(f"Filtered by X-axis: {len(filtered_points)} points remaining.")
        
        # Logging for debugging
        if len(filtered_points) > 0:
            x_range = (filtered_points[:, 0].min(), filtered_points[:, 0].max())
            y_range = (filtered_points[:, 1].min(), filtered_points[:, 1].max())
            z_range = (filtered_points[:, 2].min(), filtered_points[:, 2].max())
            self.get_logger().info(
                f"Filtered {len(filtered_points)} points. "
                f"x range: {x_range}, y range: {y_range}, z range: {z_range}"
            )
        else:
            self.get_logger().info("No points in the filtered range.")
        
        return filtered_points


    def get_largest_cluster(self, points):
        """
        Remove noise and outlier points by clustering and selecting the largest cluster (the back).
        """
        # Cluster the points
        clustering = DBSCAN(eps=0.065, min_samples=2000).fit(points)
        labels = clustering.labels_

        # Calculate the cluster sizes
        unique_labels, counts = np.unique(labels, return_counts=True)

        # Remove the label -1 (noise)
        valid_counts = counts[unique_labels != -1]
        valid_labels = unique_labels[unique_labels != -1]

        # Check for valid clusters
        if len(valid_counts) == 0:
            self.get_logger().info("No valid clusters found.")
            return np.array([])

        # Find the largest cluster
        largest_cluster_label = valid_labels[np.argmax(valid_counts)]
        largest_cluster_points = points[labels == largest_cluster_label]

        self.visualize_clusters_3d(points, labels)
        
        return largest_cluster_points
    

    def visualize_clusters_3d(self, points, labels):
        """
        Visualizes clusters in 3D using Matplotlib.
        """
        unique_labels = np.unique(labels)
        colors = plt.cm.jet(np.linspace(0, 1, len(unique_labels)))

        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')

        for label, color in zip(unique_labels, colors):
            mask = labels == label
            ax.scatter(points[mask, 0], points[mask, 1], points[mask, 2], c=[color], label=f"Cluster {label}" if label != -1 else "Noise", s=1)

        ax.set_xlabel("X-Axis")
        ax.set_ylabel("Y-Axis")
        ax.set_zlabel("Z-Axis")
        ax.set_title("DBSCAN Clustering (3D)")
        plt.legend(markerscale=3)
        plt.show()


    def publish_processed_pointcloud(self, points):
        """
        Publish processed point cloud data for RViz.
        """
        header = Header()
        header.frame_id = 'base_footprint'  
        
        # Project the points to the robot in x-direction
        points[:, 0] -= 0.84

        # Create a PointCloud2 message
        pointcloud_msg = create_cloud_xyz32(header, points.tolist())
        
        # Publish the point cloud for visualization
        self.back_cloud_pub.publish(pointcloud_msg)
        self.get_logger().info("Published processed point cloud.")   
        


    # TAPPING POSITIONS   
    ###########################################################################################################   
        
        
    
    def calculate_tapping_positions(self, back_points):
        """
        Calculate six tapping positions on the back that correspond to actual points in the back_points.
        """
        # Find the centroid of the back points
        centroid = np.mean(back_points, axis=0)

        # Perform PCA to calculate the orientation of the back plane
        pca = PCA(n_components=3)
        pca.fit(back_points)

        # The normal vector of the back plane
        normal_vector = pca.components_[2]  # Third principal component (smallest variance)

        # Two orthogonal basis vectors within the back plane
        basis_vector1 = pca.components_[0]  # First principal component (largest variance)
        basis_vector2 = pca.components_[1]  # Second principal component (second-largest variance)

        # Normalize the basis vectors
        basis_vector1 /= np.linalg.norm(basis_vector1)
        basis_vector2 /= np.linalg.norm(basis_vector2)

        # Project all back points onto the plane
        back_points_on_plane = []
        for point in back_points:
            # Calculate the projection of the point onto the plane
            vector_to_plane = point - centroid
            distance_to_plane = np.dot(vector_to_plane, normal_vector)
            projected_point = point - distance_to_plane * normal_vector
            back_points_on_plane.append(projected_point)
        back_points_on_plane = np.array(back_points_on_plane)

        # Define offsets for the rows and columns
        row_offsets = [-0.05, 0.05]   # Vertical offsets (three rows)
        col_offsets = np.linspace(-0.1, 0.1, 3)  # Horizontal offsets (two columns)

        # Calculate candidate tapping positions
        tapping_positions = []
        for row_offset in row_offsets:
            for col_offset in col_offsets:
                offset = row_offset * basis_vector2 + col_offset * basis_vector1
                candidate_position = centroid + offset

                # Find the closest back point on the plane
                distances = np.linalg.norm(back_points_on_plane - candidate_position, axis=1)
                closest_point_idx = np.argmin(distances)
                tapping_positions.append(back_points[closest_point_idx])  # Use the original point

        tapping_positions = np.array(tapping_positions)
        
         # Reorder positions as for the desired tapping order [5, 4, 3, 0, 1, 2]
        ordered_indices = [5, 4, 3, 0, 1, 2]
        tapping_positions = tapping_positions[ordered_indices]
        
        # Publish tapping positions
        tapping_msg = Float32MultiArray()
        tapping_msg.data = tapping_positions.flatten().tolist()
        self.tapping_positions_pub.publish(tapping_msg)
        
        # Publish markers for visualization in RViz
        self.publish_markers(tapping_positions)
        
        # Visualize tapping positions
        self.visualize_back(back_points, tapping_positions)
        self.get_logger().info(f"Tapping positions \n: {tapping_positions}")
        return tapping_positions



    # VISUALIZATION
    #################################################################################################
    
    
    
    def publish_markers(self, positions):
        """
        Publish RViz markers for the tapping positions.
        """
        marker_array = MarkerArray()
        for i, pos in enumerate(positions):
            marker = Marker()
            marker.header.frame_id = "base_footprint"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tapping_positions"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05  # Sphere diameter
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)

            # Log positions
            self.get_logger().info(f"Published marker {i} at position: x={pos[0]}, y={pos[1]}, z={pos[2]}")

        # Publish all markers
        self.marker_publisher.publish(marker_array)

    
    def visualize_back(self, back_points, tapping_positions):
        """
        Visualize the tapping positions in 3D along with the back points.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot back points with lower z-order to ensure tapping positions are on top
        ax.scatter(
            back_points[:, 0], back_points[:, 1], back_points[:, 2],
            c='b', alpha=0.5, label='Back Points', zorder=1
        )

        # Plot tapping positions with larger markers and a higher z-order
        ax.scatter(
            tapping_positions[:, 0],
            tapping_positions[:, 1],
            tapping_positions[:, 2],
            c='r', marker='o', s=200, label='Tapping Positions', zorder=2
        )

        # Annotate tapping positions
        for i, pos in enumerate(tapping_positions):
            ax.text(
                pos[0], pos[1], pos[2], f'P{i+1}',
                color='red', fontsize=14, fontweight='bold', zorder=3
            )

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()



def main(args=None):
    rclpy.init(args=args)
    node = BackDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

