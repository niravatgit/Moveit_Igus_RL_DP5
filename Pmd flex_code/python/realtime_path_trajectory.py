import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import random

# Define the region of interest (ROI) limits
ROI_X_MIN = -0.1  # Minimum x-coordinate of the ROI
ROI_X_MAX = 0.1 # Maximum x-coordinate of the ROI
ROI_Y_MIN = -0.1  # Minimum y-coordinate of the ROI
ROI_Y_MAX = 0.1 # Maximum y-coordinate of the ROI
ROI_Z_MIN = -5  # Minimum z-coordinate of the ROI (for filtering out)
ROI_Z_MAX = 5    # Maximum z-coordinate of the ROI (for filtering out)

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

    # Create 3D scatter plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Visualize points in red
    ax.scatter(rotated_x, rotated_y, rotated_z, c='red', marker='o')

    # Select random points based on distance
    num_selected = min(len(rotated_x), 20)  # Select a maximum of 20 points
    selected_indices = random.sample(range(len(rotated_x)), num_selected)
    selected_x = [rotated_x[i] for i in selected_indices]
    selected_y = [rotated_y[i] for i in selected_indices]
    selected_z = [rotated_z[i] for i in selected_indices]

    # Visualize selected points in blue
    ax.scatter(selected_x, selected_y, selected_z, c='blue', marker='o')

    # Draw a simple trajectory path using lines
    for i in range(num_selected - 1):
        ax.plot([selected_x[i], selected_x[i+1]],
                [selected_y[i], selected_y[i+1]],
                [selected_z[i], selected_z[i+1]], c='blue')

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.title('Point Cloud Visualization with 180-degree Rotation along X-Axis')

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
