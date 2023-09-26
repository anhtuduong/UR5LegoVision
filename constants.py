"""!
@file constants.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-01

@brief Constants used in the project
"""

# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Import other modules
import numpy as np

# Log paths
IMG_ZED_PATH = os.path.abspath(os.path.join(ROOT, "logs/img_ZED_cam.png"))
POINT_CLOUD_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud.txt"))
PLY_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud.ply"))
PLY_FROM_ROS_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud_from_ros.ply"))
PLY_AFTER_TRANSFORM_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud_after_transform.ply"))
PLY_AFTER_CLEAN_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud_after_clean.ply"))
PLY_AFTER_ALIGN_PATH = os.path.abspath(os.path.join(ROOT, "logs/point_cloud_after_align.ply"))

# Planner path
MOTION_PLANNER_PATH = os.path.abspath(os.path.join(ROOT, "logs/motion_planner.json"))
PLANNING_CONFIG = os.path.abspath(os.path.join(ROOT, "planning/config.json"))

# Robot and simulation constants
WORLD_NAME = 'lego.world'
ROBOT_NAME = 'ur5'
GRIPPER_LINK = 'wrist_3_link'
LEGO_SCENE = os.path.abspath(os.path.join(ROOT, 'lego_scene.json'))
URDF_PATH = os.path.abspath(os.path.join(ROOT, 'locosim/robot_urdf/generated_urdf/ur5.urdf'))
XACRO_PATH = os.path.abspath(os.path.join(ROOT, 'locosim/robot_descriptions/ur_description/urdf/ur5.urdf.xacro'))
CUSTOM_MODELS_PATH = os.path.abspath(os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models'))
LAUNCH_PATH = os.path.abspath(os.path.join(ROOT, 'locosim/ros_impedance_controller/launch/ros_impedance_controller_ur5.launch'))

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

# Mesh and point cloud paths of all blocks
MODEL = {
    'X1-Y1-Z2': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y1-Z2/mesh/X1-Y1-Z2.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y1-Z2/pointcloud_X1-Y1-Z2.ply'),
        'sdf_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y1-Z2/mesh_X1-Y1-Z2.sdf'),
        'pickup_point': np.array([0.0, 0.0, 0.02]),
    },
    'X1-Y2-Z1': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z1/mesh/X1-Y2-Z1.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z1/pointcloud_X1-Y2-Z1.ply'),
        'sdf_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z1/mesh_X1-Y2-Z1.sdf'),
        'pickup_point': np.array([0.0, 0.0, 0.01]),

    },
    'X1-Y2-Z2': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2/mesh/X1-Y2-Z2.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2/pointcloud_X1-Y2-Z2.ply'),
        'sdf_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2/mesh_X1-Y2-Z2.sdf'),
        'pickup_point': np.array([0.0, 0.0, 0.02]),
    },
    'X1-Y2-Z2-CHAMFER': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2-CHAMFER/mesh/X1-Y2-Z2-CHAMFER.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2-CHAMFER/pointcloud_X1-Y2-Z2-CHAMFER.ply'),
        'sdf_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2-CHAMFER/mesh_X1-Y2-Z2-CHAMFER.sdf'),
        'pickup_point': np.array([0.0, 0.016104, 0.02]),
    },
    'X1-Y2-Z2-TWINFILLET': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2-TWINFILLET/mesh/X1-Y2-Z2-TWINFILLET.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2-TWINFILLET/pointcloud_X1-Y2-Z2-TWINFILLET.ply'),
        'sdf_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y2-Z2-TWINFILLET/mesh_X1-Y2-Z2-TWINFILLET.sdf'),
        'pickup_point': np.array([0.0, 0.0, 0.02]),
    },
    'X1-Y3-Z2': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y3-Z2/mesh/X1-Y3-Z2.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y3-Z2/pointcloud_X1-Y3-Z2.ply'),
        'sdf_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y3-Z2/mesh_X1-Y3-Z2.sdf'),
        'pickup_point': np.array([0.0, 0.0, 0.02]),
    },
    'X1-Y3-Z2-FILLET': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y3-Z2-FILLET/mesh/X1-Y3-Z2-FILLET.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y3-Z2-FILLET/pointcloud_X1-Y3-Z2-FILLET.ply'),
        'sdf_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y3-Z2-FILLET/mesh_X1-Y3-Z2-FILLET.sdf'),
        'pickup_point': np.array([0.0, 0.03207, 0.02]),
    },
    'X1-Y4-Z1': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y4-Z1/mesh/X1-Y4-Z1.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y4-Z1/pointcloud_X1-Y4-Z1.ply'),
        'sdf_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y4-Z1/mesh_X1-Y4-Z1.sdf'),
        'pickup_point': np.array([0.0, 0.0, 0.01]),
    },
    'X1-Y4-Z2': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y4-Z2/mesh/X1-Y4-Z2.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y4-Z2/pointcloud_X1-Y4-Z2.ply'),
        'sdf_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X1-Y4-Z2/mesh_X1-Y4-Z2.sdf'),
        'pickup_point': np.array([0.0, 0.0, 0.02]),
    },
    'X2-Y2-Z2': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X2-Y2-Z2/mesh/X2-Y2-Z2.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X2-Y2-Z2/pointcloud_X2-Y2-Z2.ply'),
        'sdf_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X2-Y2-Z2/mesh_X2-Y2-Z2.sdf'),
        'pickup_point': np.array([0.0, 0.0, 0.02]),
    },
    'X2-Y2-Z2-FILLET': {
        'mesh_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X2-Y2-Z2-FILLET/mesh/X2-Y2-Z2-FILLET.stl'),
        'pointcloud_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X2-Y2-Z2-FILLET/pointcloud_X2-Y2-Z2-FILLET.ply'),
        'sdf_file': os.path.join(ROOT, 'locosim/ros_impedance_controller/worlds/models/X2-Y2-Z2-FILLET/mesh_X2-Y2-Z2-FILLET.sdf'),
        'pickup_point': np.array([0.0, 0.0, 0.02]),
    }
}

TABLE_HEIGHT = 0.86992