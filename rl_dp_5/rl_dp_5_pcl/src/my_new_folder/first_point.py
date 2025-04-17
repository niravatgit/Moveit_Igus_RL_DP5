# This script loads a 3D point cloud and finds the nearest neighbors in the cloud for a set of specified taget points from the rounded_pointcloud.ply 

#!/usr/bin/env python3

import open3d as o3d
import numpy as np
from sklearn.neighbors import NearestNeighbors

# Load the point cloud
pcd = o3d.io.read_point_cloud("rounded_pointcloud.ply")
points = np.asarray(pcd.points)

# Points you want to find (as a numpy array)
target_points = np.array([
    [0.38, 0.08, 0.33],
    [0.37, 0.08, 0.33],
    [0.41, -0.0, 0.36],
    [0.38, -0.08, 0.33],
    [0.26, 0.03, 0.35]
])

# Use nearest neighbors to find the closest points
nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(points)
distances, indices = nbrs.kneighbors(target_points)

# Print the results
for i, (point, idx, dist) in enumerate(zip(target_points, indices.flatten(), distances.flatten())):
    print(f"Point {i+1}: {point}")
    print(f"  Found at index: {idx}")
    print(f"  Cloud point: {points[idx]}")
    print(f"  Distance: {dist:.8f}")
    print()

# If you need just the indices array
print("All indices:", indices.flatten())
