# This script loads a point cloud, rounds its coordinates for easier processing, and identifies the nearest neighbors to two target positions.

#!/usr/bin/env python3

import open3d as o3d
import numpy as np
from sklearn.neighbors import NearestNeighbors
import random

# Load the snapshot
pcd = o3d.io.read_point_cloud("smoothed_pointcloud.ply")

# Convert point cloud to a numpy array (in meters)
points = np.asarray(pcd.points)

# Convert points to millimeters for more precise rounding
points_mm = points * 100

# Print 86 random points from the original point cloud (in mm)
print("\n--- 45 Random Points from Original Point Cloud (Before Rounding, in mm) ---")
random_indices = random.sample(range(len(points_mm)), min(45, len(points_mm)))
for i, idx in enumerate(random_indices):
    print(f"{i+1}. Index {idx}: {points_mm[idx]} mm")

# Round in mm space (x/z to 2 decimals, y to 1 decimal)
rounded_points_mm = points_mm.copy()
rounded_points_mm[:, 0] = np.round(points_mm[:, 0], decimals=0)  # x coordinate (mm)
rounded_points_mm[:, 1] = np.round(points_mm[:, 1], decimals=0)  # y coordinate (mm)
rounded_points_mm[:, 2] = np.round(points_mm[:, 2], decimals=0)  # z coordinate (mm)

# Convert back to meters for MoveIt!
rounded_points = rounded_points_mm / 100

# Define target positions (in meters)
target_position1 = np.array([0.4248, 0.0756, 0.3567])  
target_position2 = np.array([0.4, 0.0, 0.3])

# Convert targets to mm for nearest neighbor search
target_position1_mm = target_position1 * 100
target_position2_mm = target_position2 * 100

# Find k-nearest points to target positions (using mm for precision)
def find_k_nearest_points(target_mm, points_mm, k=5):
    nbrs = NearestNeighbors(n_neighbors=k).fit(points_mm)
    distances_mm, indices = nbrs.kneighbors([target_mm])
    return indices[0], points_mm[indices[0]]/100, distances_mm[0]/100  # Return in meters

# Number of nearest points to find
k = 15

# Find and print nearest points to targets
indices1, nearest_points1, distances1 = find_k_nearest_points(target_position1_mm, rounded_points_mm, k)
indices2, nearest_points2, distances2 = find_k_nearest_points(target_position2_mm, rounded_points_mm, k)

print("\n--- Nearest Points to Target Positions (in meters) ---")
print(f"\nTop {k} nearest to target_position1 ({target_position1}):")
for i, (idx, point, dist) in enumerate(zip(indices1, nearest_points1, distances1)):
    print(f"{i+1}. Index {idx}: {point} (distance: {dist:.6f} m)")

print(f"\nTop {k} nearest to target_position2 ({target_position2}):")
for i, (idx, point, dist) in enumerate(zip(indices2, nearest_points2, distances2)):
    print(f"{i+1}. Index {idx}: {point} (distance: {dist:.6f} m)")

# Function to get random points excluding the target neighbors
def get_other_points(points, exclude_indices, sample_size=20):
    all_indices = set(range(len(points)))
    exclude_set = set(exclude_indices)
    available_indices = list(all_indices - exclude_set)
    selected_indices = random.sample(available_indices, min(sample_size, len(available_indices)))
    return selected_indices, points[selected_indices]

# Get other points from the cloud (using rounded meters)
sample_size = 45
other_indices, other_points = get_other_points(rounded_points, list(indices1) + list(indices2), sample_size)

print("\n--- Other Points in the Cloud (excluding target neighbors, in meters) ---")
print(f"\nShowing {len(other_points)} other points:")
for i, (idx, point) in enumerate(zip(other_indices, other_points)):
    print(f"{i+1}. Index {idx}: {point}")

# Save rounded point cloud (in meters)
rounded_pcd = o3d.geometry.PointCloud()
rounded_pcd.points = o3d.utility.Vector3dVector(rounded_points)
o3d.io.write_point_cloud("rounded_pointcloud.ply", rounded_pcd)
