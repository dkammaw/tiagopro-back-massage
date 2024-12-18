import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from cv_bridge import CvBridge
import cv2
from sklearn.decomposition import PCA
import numpy as np


class BackDetector(Node):
    def __init__(self):
        super().__init__('back_detector')

        # Subscriptions
        self.create_subscription(PointCloud2, '/head_front_camera/depth/color/points', self.pointcloud_callback, 10)
        self.create_subscription(Image, '/head_front_camera/color/image_raw', self.image_callback, 10)

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
        
        # Downsampling
        points = points[::5]  # Verarbeite jeden 5. Punkt
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
            # self.visualize_back_points(back_points)
            self.calculate_tapping_positions(back_points)
            
        else:
            self.get_logger().info("No back points detected.")

    def detect_human_from_rgb(self, image):
        """
        Detect human in the RGB image.
        This function uses simple segmentation (e.g., color range) as a placeholder.
        Replace with advanced methods like OpenPose or YOLO for better results.
        """
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_skin = np.array([0, 20, 70], dtype=np.uint8)
        upper_skin = np.array([20, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv_image, lower_skin, upper_skin)

        # Perform morphological operations to clean the mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None

        # Find the largest contour (assuming it's the human)
        human_contour = max(contours, key=cv2.contourArea)

        # Create a mask for the detected human
        human_mask = np.zeros_like(mask)
        cv2.drawContours(human_mask, [human_contour], -1, 255, thickness=cv2.FILLED)

        return human_mask


    def extract_back_from_pointcloud(self, human_mask):
        """
        Extract 3D points corresponding to the human's back.
        """
        if self.point_cloud is None or len(self.point_cloud) == 0:
            self.get_logger().info("No point cloud data available.")
            return None

        # Apply heuristic to extract back (e.g., points with vertical alignment)
        back_points = self.filter_vertical_points(self.point_cloud)
        return back_points

            
    def filter_vertical_points(self, points):
        """
        Identify points corresponding to the back by filtering vertical alignment.
        """
        # Assume back is between 0.5 and 1.5 meters from the robot
        z_min = 0.5  # Min distance from robot (in meters)
        z_max = 3.0  # Max distance from robot (in meters)

        # Filter based on depth (z-coordinate)
        filtered_points = points[(points[:, 2] > z_min) & (points[:, 2] < z_max)]

        # Optional: zusätzliche Filter für Höhe und Breite
        x_min, x_max = -0.5, 0.5  # Begrenzung seitlich
        y_min, y_max = 0.5, 2.0   # Begrenzung vertikal
        filtered_points = filtered_points[
            (filtered_points[:, 0] > x_min) & (filtered_points[:, 0] < x_max) &
            (filtered_points[:, 1] > y_min) & (filtered_points[:, 1] < y_max)
        ]
        # Log the range of filtered z values
        if len(filtered_points) > 0:
            self.get_logger().info(f"Filtered {len(filtered_points)} points for z: [{z_min}, {z_max}] and x/y ranges.")
        else:
            self.get_logger().info("No points in the filtered range.")

        return filtered_points
        



    '''def calculate_tapping_positions(self, back_points):
        """
        Calculate six tapping positions directly on the detected back plane.
        """
        # Fit the back plane using PCA
        plane_normal, plane_center = self.fit_plane_to_back(back_points)

        # Generate tapping positions
        tapping_positions = self.generate_points_on_plane(plane_normal, plane_center)

        # Visualize the tapping positions
        self.visualize_tapping_positions(back_points, tapping_positions)
        self.get_logger().info(f"Tapping positions: {tapping_positions}")
        return tapping_positions

    def fit_plane_to_back(self, back_points):
        """
        Fit a plane to the back points using PCA.
        :param back_points: Nx3 array of back points.
        :return: Plane normal vector and plane center.
        """
        # Center the points around the mean
        mean_point = np.mean(back_points, axis=0)
        centered_points = back_points - mean_point

        # Perform PCA to find the normal
        pca = PCA(n_components=3)
        pca.fit(centered_points)

        normal_vector = pca.components_[-1]  # The last component is the plane normal
        self.get_logger().info(f"Plane normal: {normal_vector}")
        return normal_vector, mean_point

    def generate_points_on_plane(self, normal_vector, plane_center):
        """
        Generate six tapping positions on the plane in a 3x2 grid.
        """
        # Define grid dimensions
        row_spacing = 0.1  # Distance between rows
        col_spacing = 0.05  # Distance between columns

        # Create two orthogonal vectors to define the plane
        basis_vector1 = np.cross(normal_vector, [1, 0, 0])
        if np.linalg.norm(basis_vector1) == 0:
            basis_vector1 = np.cross(normal_vector, [0, 1, 0])  # Handle edge case
        basis_vector1 /= np.linalg.norm(basis_vector1)

        basis_vector2 = np.cross(normal_vector, basis_vector1)
        basis_vector2 /= np.linalg.norm(basis_vector2)

        # Generate points in a 3x2 grid
        tapping_positions = []
        for row in range(3):  # 3 rows
            for col in range(2):  # 2 columns
                offset = (row - 1) * row_spacing * basis_vector2 + (col - 0.5) * col_spacing * basis_vector1
                tapping_positions.append(plane_center + offset)

        return np.array(tapping_positions)

    def calculate_tapping_positions(self, back_points):
        """
        Calculate six tapping positions on the back in three rows:
        - Each row has two points next to each other.
        - Rows are aligned vertically, close to the centroid of the back points.
        """
        # Find the centroid of the back points (average position)
        centroid = np.mean(back_points, axis=0)

        # Define offsets for the rows
        row_offsets = np.linspace(-0.1, 0.1, 3)  # Three rows, vertical offset (Y-axis)
        col_offsets = [-0.05, 0.05]  # Two columns, horizontal offset (X-axis)

        # Calculate tapping positions
        tapping_positions = []
        for row_offset in row_offsets:
            for col_offset in col_offsets:
                tapping_positions.append([
                    centroid[0] + col_offset,  # X-coordinate (side-to-side)
                    centroid[1] + row_offset,  # Y-coordinate (up-down/back height)
                    centroid[2]               # Z-coordinate (depth)
                ])

        tapping_positions = np.array(tapping_positions)

        # Visualize tapping positions
        self.visualize_tapping_positions(back_points, tapping_positions)
        self.get_logger().info(f"Tapping positions: {tapping_positions}")
        return tapping_positions'''
        
    def calculate_tapping_positions(self, back_points):
        """
        Calculate six tapping positions on the back in three rows:
        - Each row has two points next to each other.
        - Rows are aligned vertically, close to the centroid of the back points, on the back plane.
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

        # Define offsets for the rows and columns
        row_offsets = np.linspace(-0.1, 0.1, 3)  # Vertical offsets (three rows)
        col_offsets = [-0.05, 0.05]  # Horizontal offsets (two columns)

        # Calculate tapping positions
        tapping_positions = []
        for row_offset in row_offsets:
            for col_offset in col_offsets:
                offset = row_offset * basis_vector2 + col_offset * basis_vector1
                tapping_positions.append(centroid + offset)

        tapping_positions = np.array(tapping_positions)

        # Ensure points are on the plane (for verification)
        for point in tapping_positions:
            distance_to_plane = np.dot(point - centroid, normal_vector)
            assert abs(distance_to_plane) < 1e-6, "Point not on the plane!"

        # Visualize tapping positions
        self.visualize_tapping_positions(back_points, tapping_positions)
        self.get_logger().info(f"Tapping positions: {tapping_positions}")
        return tapping_positions

    def visualize_tapping_positions(self, back_points, tapping_positions):
        """
        Visualize the tapping positions in 3D along with the back points.
        """
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot back points
        ax.scatter(back_points[:, 0], back_points[:, 1], back_points[:, 2], c='b', label='Back Points')

        # Plot tapping positions
        ax.scatter(
            tapping_positions[:, 0], 
            tapping_positions[:, 1], 
            tapping_positions[:, 2], 
            c='r', marker='o', s=100, label='Tapping Positions'
        )

        # Annotate tapping positions
        for i, pos in enumerate(tapping_positions):
            ax.text(pos[0], pos[1], pos[2], f'P{i+1}', color='red')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()
        plt.show()


    '''def visualize_back_points(self, back_points):
        """
        Visualize the detected back points in 3D using matplotlib.
        """
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(back_points[:, 0], back_points[:, 1], back_points[:, 2], c='r', marker='o')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()'''

def main(args=None):
    rclpy.init(args=args)
    node = BackDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


