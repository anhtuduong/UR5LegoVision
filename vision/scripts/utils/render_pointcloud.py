
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

#  Global constants
MODEL_MESH_PATH = os.path.join(ROOT, 'models/X1-Y3-Z2-FILLET/mesh/X1-Y3-Z2-FILLET.stl')

# Import
import numpy as np
import pyvista as pv

# Load the STL file
mesh = pv.read(MODEL_MESH_PATH)

# Convert the mesh to a point cloud
point_cloud = mesh.points

# Visualize the point cloud
plotter = pv.Plotter()
plotter.add_points(point_cloud)
plotter.show()

