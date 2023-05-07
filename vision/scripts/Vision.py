"""!
@package vision.scripts.Vision
@file vision/scripts/Vision.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief Defines the Vision node that communicates with Motion node.
@date 2023-05-04
"""

import os
import sys
from pathlib import Path

# Resolve paths
FILE = Path(__file__).resolve()
ROOT = FILE.parents[2]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import rospy as ros
import numpy as np
from localization.BlockDetect import BlockDetect
from camera.ZED import ZED
from camera.PointCloud import PointCloud

# Global constants
IMG_ZED_PATH = os.path.abspath(os.path.join(ROOT, "logs/img_ZED_cam.png"))

# ---------------------- CLASS ----------------------

class Vision:
    """
    @brief This class detects blocks from ZED camera and communicates with different ROS node
    """

    def __init__(self):
        """ @brief Class constructor
        """

        ros.init_node('vision', anonymous=True)
        zed = ZED()
        img = zed.save_image(IMG_ZED_PATH, zed.get_image())
        block_detect = BlockDetect(img)
        block_list = block_detect.get_block_list()
        pc = PointCloud()

        for block in block_list:
            print(pc.get_pointcloud(block.get_pixels()))
        
            
# ---------------------- MAIN ----------------------
# To use in command:
# python3 Vision.py
 
if __name__ == '__main__':

    vision = Vision()

    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
