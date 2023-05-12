

# Reslove paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Import
from vision.constants import PLY_PATH, PLY_FROM_ROS_PATH

import numpy as np
import open3d as o3d

# Load the two point clouds
source_cloud = o3d.io.read_point_cloud(PLY_PATH)
target_cloud = o3d.io.read_point_cloud(PLY_FROM_ROS_PATH)

# Convert point clouds to numpy arrays
source_points = np.asarray(source_cloud.points)
target_points = np.asarray(target_cloud.points)

# Initialize the Isomap algorithm with the target point cloud
isomap = Isomap(n_components=3)
isomap.fit(target_points)

# Find the nearest neighbors in the target point cloud for each point in the source point cloud
distances, indices = KDTree(target_points).query(source_points)

# Construct the adjacency matrix
n = len(source_points)
adjacency = np.zeros((n, n))
for i in range(n):
    for j in range(n):
        if indices[i] == j:
            adjacency[i, j] = 1.0

# Compute the ICP transformation matrix
transformation_matrix = isomap.embedding_ @ np.linalg.pinv(adjacency @ isomap.embedding_)

# Apply the transformation matrix to the source point cloud
source_points_transformed = source_points @ transformation_matrix[:3, :3].T + transformation_matrix[:3, 3]

# Convert the transformed points back to an Open3D point cloud
source_cloud_transformed = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(source_points_transformed))

# Visualize the result
o3d.visualization.draw_geometries([source_cloud_transformed, target_cloud])