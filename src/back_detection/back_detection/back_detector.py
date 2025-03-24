import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
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
from mayavi import mlab
from sklearn.neighbors import NearestNeighbors
from kneed import KneeLocator

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
        
        # Subscription to the point cloud topic of the RGBD-Camera
        self.create_subscription(PointCloud2, '/head_front_camera/depth/color/points', self.pointcloud_callback, qos_profile_best_effort)
        
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
                self.process_back_detection()
            else:
                self.get_logger().info("No valid point cloud data received.")
        
        self.received_cloud = True 
            

    
    def process_back_detection(self):
        """
        Identify points corresponding to the back by filtering based on their alignment.
        """
        # Apply heuristic to extract back (e.g., points with vertical alignment)
        back_points = self.filter_vertical_points(self.point_cloud)
        #self.parameter_selection(back_points)
        #Select largest cluster (which is the Back)
        back_points = self.get_largest_cluster(back_points)
        
        # Publish the processed point cloud
        self.publish_processed_pointcloud(back_points)
        
        if back_points is not None:
            self.get_logger().info(f"Detected back region with {len(back_points)} points.")
            
            try:
                # Start the subprocess of moving the robot towards the human
                process = subprocess.Popen(['ros2', 'run', 'back_detection', 'move'])
                # Wait for up to 20 seconds for the process to finish
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                print("Process did not finish in 20 seconds. Continuing with other tasks.")
            
            # Calculate tapping positions
            self.calculate_tapping_positions(back_points)
        
        else:
            self.get_logger().info("No back points detected.")
        

        
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

    def parameter_selection(self, points):
        X = points
        k = 1000  # Choose MinPts
        
        neigh = NearestNeighbors(n_neighbors=k)
        neigh.fit(X)
        distances, _ = neigh.kneighbors(X)

        # Sort distances of k-th nearest neighbor
        distances = np.sort(distances[:, k-1], axis=0)

        # Erste Ableitung berechnen
        first_derivative = np.diff(distances)

        # Stelle finden, wo die Steigung stark zunimmt
        elbow_index = np.argmax(first_derivative > np.percentile(first_derivative, 95))

        # Optimalen Epsilon-Wert festlegen
        optimal_eps = distances[elbow_index]

        # Plot mit eingezeichneter "Elbow"-Linie
        plt.figure(figsize=(8, 6))
        plt.plot(distances, marker="o", linestyle="-", label=f"{k}-th Nearest Neighbor Distance")
        plt.axvline(x=elbow_index, color='r', linestyle="--", label=f"Elbow at {optimal_eps:.3f}")

        plt.xlabel("Points sorted by distance")
        plt.ylabel(f"{k}-th Nearest Neighbor Distance")
        plt.title("k-Distance Plot with Manual Elbow")
        plt.legend()
        plt.show()

        print(f"Optimal Epsilon (eps) estimated manually: {optimal_eps:.3f}")


        print(f"Optimal Epsilon (eps) estimated by Kneedle: {optimal_eps:.3f}")

    def get_largest_cluster(self, points):
        """
        Cluster all points and highlight the largest cluster.
        """
        # Cluster the points
        clustering = DBSCAN(eps=0.038, min_samples=1000).fit(points)
        labels = clustering.labels_

        # Berechne Clustergrößen
        unique_labels, counts = np.unique(labels, return_counts=True)

        # Entferne Noise (-1)
        valid_counts = counts[unique_labels != -1]
        valid_labels = unique_labels[unique_labels != -1]

        if len(valid_counts) == 0:
            self.get_logger().info("No valid clusters found.")
            return np.array([])

        # Bestimme das größte Cluster
        largest_cluster_label = valid_labels[np.argmax(valid_counts)]
        
        # Visualisierung aller Cluster mit Hervorhebung des größten
        self.visualize_clusters_3d(points, labels, largest_cluster_label)

        return points, labels, largest_cluster_label  # Alle Punkte und Labels zurückgeben

    def visualize_clusters_3d(self, points, labels, largest_cluster_label):
        """
        3D-Visualisierung aller Cluster mit Hervorhebung des größten.
        """
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        unique_labels = np.unique(labels)

        for label in unique_labels:
            cluster_points = points[labels == label]

            if label == -1:
                color, marker, alpha, size = "black", "x", 0.1, 10  # Noise
            elif label == largest_cluster_label:
                color, marker, alpha, size = "red", "o", 1.0, 50  # Largest cluster
            else:
                color, marker, alpha, size = np.random.rand(3,), "o", 0.6, 20  # Other clusters

            ax.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2], 
                    c=[color], marker=marker, alpha=alpha, s=size, label=f"Cluster {label}")

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        plt.title("3D Cluster Visualization")
        plt.legend()
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
        row_offsets = [-0.1, 0.0, 0.1]   # Vertical offsets (three rows)
        col_offsets = [-0.045, 0.045]    # Horizontal offsets (two columns)

        # Calculate candidate tapping positions
        tapping_positions = []
        for row_offset in row_offsets:
            for col_offset in col_offsets:
                offset = row_offset * basis_vector1 + col_offset * basis_vector2
                candidate_position = centroid + offset
                # Find the closest back point on the plane
                distances = np.linalg.norm(back_points_on_plane - candidate_position, axis=1)
                closest_point_idx = np.argmin(distances)
                tapping_positions.append(back_points[closest_point_idx])  # Use the original point
                
        tapping_positions = np.array(tapping_positions)
        
         # Reorder positions as for the desired tapping order [4, 5, 2, 3, 0, 1]
        ordered_indices = [4, 5, 2, 3, 0, 1]
        tapping_positions = tapping_positions[ordered_indices]
        
        # Publish tapping positions
        tapping_msg = Float32MultiArray()
        tapping_msg.data = tapping_positions.flatten().tolist()
        
        # Publish markers for visualization in RViz
        self.publish_markers(tapping_positions)
        
        # Visualize results
        # self.visualize_pca_plane_mayavi(back_points, tapping_positions, centroid, normal_vector, basis_vector1, basis_vector2)
        
        self.get_logger().info(f"Tapping positions \n: {tapping_positions}")
        
        # Publish the tapping positions to the topic
        self.tapping_positions_pub.publish(tapping_msg)
        
        return tapping_positions



 # VISUALIZATION
    #################################################################################################
    
    
    
    def visualize_pca_plane_matplot(self, back_points, tapping_positions, centroid, normal, basis_vector1, basis_vector2):
        """
        Visualizes the PCA plane fitted to the back, along with the principal components and tapping positions.
        """
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Plot the back points (dimmed for contrast)
        ax.scatter(back_points[:, 0], back_points[:, 1], back_points[:, 2], s=1, alpha=0.3, color="gray", label="Back Points")

        # Plot tapping positions with a larger size, bright color, and edge
        ax.scatter(
            tapping_positions[:, 0], tapping_positions[:, 1], tapping_positions[:, 2], 
            color='yellow', edgecolors='black', s=200, marker='o', label="Tapping Positions"
        )

        # Add text labels for tapping positions
        for i, pos in enumerate(tapping_positions):
            ax.text(pos[0], pos[1], pos[2], f"P{i}", color='black', fontsize=12, fontweight='bold')

        # Define a plane based on the PCA normal
        xlim = np.linspace(np.min(back_points[:, 0]), np.max(back_points[:, 0]), 10)
        ylim = np.linspace(np.min(back_points[:, 1]), np.max(back_points[:, 1]), 10)
        X, Y = np.meshgrid(xlim, ylim)

        # Plane equation: ax + by + cz = d  --> solve for Z
        d = np.dot(normal, centroid)
        Z = (d - normal[0] * X - normal[1] * Y) / normal[2]

        # Plot the PCA plane
        ax.plot_surface(X, Y, Z, color='cyan', alpha=0.3)

        # Plot the centroid
        ax.scatter(centroid[0], centroid[1], centroid[2], color='black', s=150, marker='x', label="Centroid")

        # Define vector scaling
        vector_scale = 0.1  # Adjust for visibility

        # Plot the PCA vectors with arrows
        ax.quiver(*centroid, *(vector_scale * normal), color='blue', label="Normal Vector", linewidth=2)
        ax.quiver(*centroid, *(vector_scale * basis_vector1), color='green', label="Basis Vector 1", linewidth=2)
        ax.quiver(*centroid, *(vector_scale * basis_vector2), color='purple', label="Basis Vector 2", linewidth=2)

        # Labels and title
        ax.set_xlabel("X-Axis")
        ax.set_ylabel("Y-Axis")
        ax.set_zlabel("Z-Axis")
        ax.set_title("Back Plane Approximation using PCA")
        ax.legend()
        
        plt.show()
        
    def visualize_pca_plane_mayavi(self, back_points, tapping_positions, centroid, normal, basis_vector1, basis_vector2):
        """
        Visualizes the PCA plane fitted to the back using Mayavi, including principal components and tapping positions.
        """
        # Open a Mayavi figure
        mlab.figure(size=(900, 700), bgcolor=(1, 1, 1))

        # Plot back points (gray, semi-transparent for contrast)
        mlab.points3d(back_points[:, 0], back_points[:, 1], back_points[:, 2], 
                    mode="point", color=(0.5, 0.5, 0.5), opacity=0.3)

        # Plot tapping positions (yellow spheres with black edges)
        mlab.points3d(tapping_positions[:, 0], tapping_positions[:, 1], tapping_positions[:, 2], 
                    scale_factor=0.02, color=(1, 1, 0), mode="sphere")

        # Add text labels for tapping positions
        for i, pos in enumerate(tapping_positions):
            mlab.text3d(pos[0], pos[1], pos[2], f"P{i}", scale=0.02, color=(0, 0, 0))

        # Compute PCA plane
        xlim = np.linspace(np.min(back_points[:, 0]), np.max(back_points[:, 0]), 50)
        ylim = np.linspace(np.min(back_points[:, 1]), np.max(back_points[:, 1]), 50)
        X, Y = np.meshgrid(xlim, ylim)
        
        # Plane equation: ax + by + cz = d  --> solve for Z
        d = np.dot(normal, centroid)
        Z = (d - normal[0] * X - normal[1] * Y) / normal[2]

        # Plot PCA plane (cyan, semi-transparent)
        mlab.surf(X, Y, Z, color=(1, 0, 0), opacity=1.0)

        # Plot centroid (black cross)
        mlab.points3d(centroid[0], centroid[1], centroid[2], scale_factor=0.03, color=(0, 0, 0), mode="2dcross")

        # Define vector scaling
        vector_scale = 0.1

        # Plot PCA vectors as arrows
        mlab.quiver3d(centroid[0], centroid[1], centroid[2], 
                    normal[0], normal[1], normal[2], 
                    color=(0, 0, 1), scale_factor=vector_scale)
        
        mlab.quiver3d(centroid[0], centroid[1], centroid[2], 
                    basis_vector1[0], basis_vector1[1], basis_vector1[2], 
                    color=(0, 1, 0), scale_factor=vector_scale)
        
        mlab.quiver3d(centroid[0], centroid[1], centroid[2], 
                    basis_vector2[0], basis_vector2[1], basis_vector2[2], 
                    color=(1, 0, 1), scale_factor=vector_scale)

        # Show the plot
        mlab.show()
    
    
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



def main(args=None):
    rclpy.init(args=args)
    node = BackDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

