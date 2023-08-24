import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Define the region of interest (ROI) limits
ROI_X_MIN = -0.2  # Minimum x-coordinate of the ROI
ROI_X_MAX = 0.2   # Maximum x-coordinate of the ROI
ROI_Y_MIN = -0.2  # Minimum y-coordinate of the ROI
ROI_Y_MAX = 0.2   # Maximum y-coordinate of the ROI
ROI_Z_MIN = 0.2   # Minimum z-coordinate of the ROI
ROI_Z_MAX = 1.5   # Maximum z-coordinate of the ROI

# Reference point coordinates for distance calculation
REFERENCE_X = 0.0
REFERENCE_Y = 0.0
REFERENCE_Z = 0.0

# Maximum distance from the reference point for visualization
MAX_DISTANCE = 1.2

def calculate_distance(x, y, z):
    return np.sqrt((x - REFERENCE_X)**2 + (y - REFERENCE_Y)**2 + (z - REFERENCE_Z)**2)

def normalize_distance(dist):
    return (dist - 0) / (MAX_DISTANCE - 0)  # Normalize to [0, 1] range

def point_cloud_callback(msg):
    # Extract point cloud data
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

    # Extract x, y, z coordinates from point cloud
    x = []
    y = []
    z = []
    for point in pc_data:
        x.append(point[0])
        y.append(point[1])
        z.append(point[2])

    # Filter points within the region of interest (ROI)
    filtered_points = [(x_val, y_val, z_val) for x_val, y_val, z_val in zip(x, y, z)
                       if ROI_X_MIN <= x_val <= ROI_X_MAX and
                       ROI_Y_MIN <= y_val <= ROI_Y_MAX and
                       ROI_Z_MIN <= z_val <= ROI_Z_MAX]

    # Unzip filtered points
    filtered_x, filtered_y, filtered_z = zip(*filtered_points)

    # Apply 180-degree rotation along x-axis
    rotated_x = filtered_x
    rotated_y = [-val for val in filtered_y]
    rotated_z = [-val for val in filtered_z]

    # Calculate distances from reference point and normalize
    distances = [calculate_distance(x_val, y_val, z_val) for x_val, y_val, z_val in zip(rotated_x, rotated_y, rotated_z)]
    normalized_distances = [normalize_distance(dist) for dist in distances]

    # Filter points within the desired distance range
    selected_indices = [i for i, dist in enumerate(distances) if dist <= MAX_DISTANCE]
    selected_x = [rotated_x[i] for i in selected_indices]
    selected_y = [rotated_y[i] for i in selected_indices]
    selected_z = [rotated_z[i] for i in selected_indices]
    selected_distances = [normalized_distances[i] for i in selected_indices]

    # Create 3D scatter plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(selected_x, selected_y, selected_z, c=selected_distances, cmap='viridis', marker='o')

    # Add color bar to show normalized distance values
   # plt.colorbar(scatter, ax=ax, label='Normalized Distance from Reference Point')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
   # plt.title('Point Cloud Visualization with 180-degree Rotation along X-Axis (Within {} m from Reference)'.format(MAX_DISTANCE))

    # Show the plot
    plt.show()

def main():
    # Initialize ROS node
    rospy.init_node('point_cloud_subscriber', anonymous=True)

    # Subscribe to point cloud topic
    rospy.Subscriber("/royale_cam0/point_cloud", PointCloud2, point_cloud_callback)

    # Keep the program running until it's terminated
    rospy.spin()

if __name__ == "__main__":
    main()
