

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

# Estimate normals for both point clouds
source_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
target_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# Apply initial alignment (identity matrix)
transformation_matrix = np.identity(4)
source_cloud.transform(transformation_matrix)

# Apply ICP registration
icp = o3d.pipelines.registration.registration_icp(
    source_cloud, target_cloud, max_correspondence_distance=0.05,
    init=transformation_matrix,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))

# Get the final transformation matrix
final_transformation_matrix = icp.transformation

# Apply the final transformation to the source point cloud
source_cloud.transform(final_transformation_matrix)

# Visualize the result
o3d.visualization.draw_geometries([source_cloud, target_cloud])