import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from cv_bridge import CvBridge
import cv2
from sklearn.decomposition import PCA
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.interpolate import griddata
from std_msgs.msg import Float32MultiArray


class BackDetector(Node):
    def __init__(self):
        super().__init__('back_detector')

        # Subscriptions
        self.create_subscription(PointCloud2, '/head_front_camera/depth/color/points', self.pointcloud_callback, 10)
        self.create_subscription(Image, '/head_front_camera/color/image_raw', self.image_callback, 10)
        
        # Publisher for tapping positions
        self.tapping_positions_pub = self.create_publisher(Float32MultiArray, '/tapping_positions', 10)

        # OpenCV bridge
        self.bridge = CvBridge()

        # Storage
        self.rgb_image = None
        self.point_cloud = None

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def pointcloud_callback(self, msg):
        # Manually parse the PointCloud2 message
        self.point_cloud = self.convert_pointcloud2_to_array(msg)

        if self.point_cloud is not None and len(self.point_cloud) > 0:
            self.get_logger().info(f"Received point cloud with {len(self.point_cloud)} points.")
            if self.rgb_image is not None:
                self.process_back_detection()
        else:
            self.get_logger().info("No valid point cloud data received.")

    def convert_pointcloud2_to_array(self, cloud_msg):
        """
        Convert PointCloud2 message to a simple NumPy array of points (x, y, z).
        """
        cloud_array = np.frombuffer(cloud_msg.data, dtype=np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)
        ]))
        # Extract x, y, z fields into a simple array
        points = np.vstack((cloud_array['x'], cloud_array['y'], cloud_array['z'])).T

        return points

    def process_back_detection(self):
        # Detect human using RGB image
        human_mask = self.detect_human_from_rgb(self.rgb_image)

        if human_mask is None:
            self.get_logger().info("No human detected in RGB image.")
            return
        else:
            contours, _ = cv2.findContours(human_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                cv2.drawContours(self.rgb_image, contours, -1, (0, 255, 0), 2)
            cv2.imshow("RGB with Human Detection", self.rgb_image)
            cv2.waitKey(1)

        # Visualize the human detection result
        cv2.imshow("Human Detection Mask", human_mask)
        cv2.waitKey(1)

        # Extract depth points corresponding to the human
        back_points = self.extract_back_from_pointcloud(human_mask)

        if back_points is not None:
            self.get_logger().info(f"Detected back region with {len(back_points)} points.")
            # Visualize the detected back points in 3D (if you have matplotlib installed)
            self.calculate_tapping_positions(back_points)
            
        else:
            self.get_logger().info("No back points detected.")

    def detect_human_from_rgb(self, image):
        """
        Detect human in the RGB image by combining skin and clothing segmentation.
        """
        # Convert to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect skin
        lower_skin = np.array([0, 10, 60], dtype=np.uint8)
        upper_skin = np.array([25, 150, 255], dtype=np.uint8)
        skin_mask = cv2.inRange(hsv_image, lower_skin, upper_skin)

        # Detect clothing (example: dark and light clothing ranges)
        lower_clothes_dark = np.array([0, 0, 10], dtype=np.uint8)
        upper_clothes_dark = np.array([180, 255, 50], dtype=np.uint8)

        lower_clothes_light = np.array([0, 0, 200], dtype=np.uint8)
        upper_clothes_light = np.array([180, 50, 255], dtype=np.uint8)

        clothes_dark_mask = cv2.inRange(hsv_image, lower_clothes_dark, upper_clothes_dark)
        clothes_light_mask = cv2.inRange(hsv_image, lower_clothes_light, upper_clothes_light)

        # Combine masks
        combined_mask = cv2.bitwise_or(skin_mask, clothes_dark_mask)
        combined_mask = cv2.bitwise_or(combined_mask, clothes_light_mask)

        # Morphological operations to clean up the mask
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        combined_mask = cv2.dilate(combined_mask, np.ones((5, 5), np.uint8), iterations=2)

        # Find contours
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None

        # Create a mask for the detected human
        human_mask = np.zeros_like(combined_mask)
        cv2.drawContours(human_mask, contours, -1, 255, thickness=cv2.FILLED)

        return human_mask


    def extract_back_from_pointcloud(self, human_mask):
        """
        Extract 3D points corresponding to the human's back.
        """
        if self.point_cloud is None or len(self.point_cloud) == 0:
            self.get_logger().info("No point cloud data available.")
            return None

        # Filter point cloud points using the human mask
        filtered_points = self.mask_point_cloud(self.point_cloud, human_mask)

        if len(filtered_points) == 0:
            self.get_logger().info("No points corresponding to the human mask.")
            return None
        
        # Apply heuristic to extract back (e.g., points with vertical alignment)
        back_points = self.filter_vertical_points(self.point_cloud)
        
        # Select largest cluster (which is the Back)
        back_points = self.get_largest_cluster(back_points)
        
        return back_points


    def mask_point_cloud(self, points, human_mask):
        """
        Filter point cloud to include only points corresponding to the human mask.
        """
        height, width = human_mask.shape
        masked_points = []

        # Camera intrinsic parameters (to be replaced with your actual values)
        fx, fy = 500, 500  # Focal lengths
        cx, cy = width / 2, height / 2  # Principal point

        for point in points:
            x, y, z = point

            # Skip invalid points
            if np.isnan(x) or np.isnan(y) or np.isnan(z) or np.isinf(x) or np.isinf(y) or np.isinf(z):
                continue

            # Skip points with z <= 0 (behind the camera)
            if z <= 0:
                continue

            # Project the 3D point onto the 2D image plane
            pixel_x = int((x / z) * fx + cx)
            pixel_y = int((y / z) * fy + cy)

            # Check if the projected point is within the image bounds
            if 0 <= pixel_x < width and 0 <= pixel_y < height:
                # Check if the point corresponds to the human mask
                if human_mask[pixel_y, pixel_x] > 0:
                    masked_points.append([x, y, z])

        return np.array(masked_points)


    def filter_vertical_points(self, points):
        """
        Identify points corresponding to the back by filtering based on their alignment.
        """
        # Define the ranges for filtering based on the assumed coordinate system:
        # Z is height (vertical), Y is depth (front-back), X is width (side-to-side)
        
        z_min, z_max = 0.5, 3.0   # Height range (vertical, in meters) - adjust based on the expected back height
        x_min, x_max = -0.5, 0.5  # Width range (side-to-side, in meters) - adjust based on the body width
        y_min, y_max = 0.5, 3.0   # Depth range (front-back, in meters) - adjust based on the body depth
        
        # Filter based on height (z-coordinate)
        filtered_points = points[(points[:, 2] > z_min) & (points[:, 2] < z_max)]

        # Additional filtering for width (x-coordinate) and depth (y-coordinate)
        filtered_points = filtered_points[
            (filtered_points[:, 0] > x_min) & (filtered_points[:, 0] < x_max) &
            (filtered_points[:, 1] > y_min) & (filtered_points[:, 1] < y_max)
        ]

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

        # Cluster the points
        clustering = DBSCAN(eps=0.05, min_samples=10).fit(points)
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

        return largest_cluster_points

        
       
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
        row_offsets = np.linspace(-0.1, 0.1, 3)  # Vertical offsets (three rows)
        col_offsets = [-0.05, 0.05]  # Horizontal offsets (two columns)

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
        
        # Publish tapping positions
        tapping_msg = Float32MultiArray()
        tapping_msg.data = tapping_positions.flatten().tolist()
        self.tapping_positions_pub.publish(tapping_msg)
        
        # Visualize tapping positions
        self.visualize_back(back_points, tapping_positions)
        self.get_logger().info(f"Tapping positions: {tapping_positions}")
        return tapping_positions

        
    def visualize_back(self, back_points, tapping_positions):
        """
        Visualize the tapping positions in 3D along with the back points.
        """
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

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
                color='red', fontsize=10, zorder=3
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

