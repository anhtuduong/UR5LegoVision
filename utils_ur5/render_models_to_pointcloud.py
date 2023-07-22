
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
from UR5BlokVision.vision.scripts.camera.PointCloudService import PointCloud
from vision.scripts.utils.Logger import Logger as log
from constants import MODEL, PLY_FROM_ROS_PATH
    
# Main
if __name__ == "__main__":

    # Render all models to point cloud
    count = 0
    for block_name in MODEL:
        log.debug(f'Start rendering {block_name} to point cloud...')
        pointcloud = PointCloud.render_pointcloud_from_stl(MODEL[block_name]['mesh_file'])
        PointCloud.save_pointcloud_to_ply(pointcloud, MODEL[block_name]['pointcloud_file'])
        count += 1
        # PointCloud.visualize_pointcloud(pointcloud)
    log.debug(f'Finished rendering {count} models to point cloud')