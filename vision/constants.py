
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Log paths
IMG_ZED_PATH = os.path.abspath(os.path.join(ROOT, "logs/img_ZED_cam.png"))
POINT_CLOUD_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud.txt"))
PLY_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud.ply"))
PLY_FROM_ROS_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud_from_ros.ply"))

# Names of all blocks
BLOCK_NAMES = [ 
    'X1-Y1-Z2',
    'X1-Y2-Z1',
    'X1-Y2-Z2',
    'X1-Y2-Z2-CHAMFER',
    'X1-Y2-Z2-TWINFILLET',
    'X1-Y3-Z2',
    'X1-Y3-Z2-FILLET',
    'X1-Y4-Z1',
    'X1-Y4-Z2',
    'X2-Y2-Z2',
    'X2-Y2-Z2-FILLET'
]

# Mesh paths of all blocks
MODEL = {
    'X1-Y1-Z2'            : os.path.join(ROOT, 'models/X1-Y1-Z2/mesh/X1-Y1-Z2.stl'),
    'X1-Y2-Z1'            : os.path.join(ROOT, 'models/X1-Y2-Z1/mesh/X1-Y2-Z1.stl'),
    'X1-Y2-Z2'            : os.path.join(ROOT, 'models/X1-Y2-Z2/mesh/X1-Y2-Z2.stl'),
    'X1-Y2-Z2-CHAMFER'    : os.path.join(ROOT, 'models/X1-Y2-Z2-CHAMFER/mesh/X1-Y2-Z2-CHAMFER.stl'),
    'X1-Y2-Z2-TWINFILLET' : os.path.join(ROOT, 'models/X1-Y2-Z2-TWINFILLET/mesh/X1-Y2-Z2-TWINFILLET.stl'),
    'X1-Y3-Z2'            : os.path.join(ROOT, 'models/X1-Y3-Z2/mesh/X1-Y3-Z2.stl'),
    'X1-Y3-Z2-FILLET'     : os.path.join(ROOT, 'models/X1-Y3-Z2-FILLET/mesh/X1-Y3-Z2-FILLET.stl'),
    'X1-Y4-Z1'            : os.path.join(ROOT, 'models/X1-Y4-Z1/mesh/X1-Y4-Z1.stl'),
    'X1-Y4-Z2'            : os.path.join(ROOT, 'models/X1-Y4-Z2/mesh/X1-Y4-Z2.stl'),
    'X2-Y2-Z2'            : os.path.join(ROOT, 'models/X2-Y2-Z2/mesh/X2-Y2-Z2.stl'),
    'X2-Y2-Z2-FILLET'     : os.path.join(ROOT, 'models/X2-Y2-Z2-FILLET/mesh/X2-Y2-Z2-FILLET.stl')
}

# Main
if __name__ == "__main__":
    print(PLY_FROM_ROS_PATH)