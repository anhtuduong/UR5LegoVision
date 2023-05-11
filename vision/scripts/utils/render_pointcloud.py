
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Import
import numpy as np
import pyvista as pv
from vision.constants import MODEL

# Render point cloud from STL file
def render_pointcloud_from_stl(stl_path):
    # Load the STL file
    mesh = pv.read(stl_path)

    # Convert the mesh to a point cloud
    point_cloud = mesh.points

    # Visualize the point cloud
    point_cloud = pv.PolyData(point_cloud)
    point_cloud.plot(scalars=np.arange(point_cloud.n_points), render_points_as_spheres=True, point_size=15)

    # Return the point cloud
    return point_cloud
    

# Main
if __name__ == "__main__":

    render_pointcloud_from_stl(MODEL['X1-Y1-Z2'])
