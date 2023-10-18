"""!
@package vision.scripts.Vision
@file vision/scripts/main.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-04

@brief Defines the Vision node that detects and localizes blocks from ZED camera
"""

# Resolve paths
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[2]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

import rospy as ros
import numpy as np
from camera.ZED import ZED
from localization.PointCloudRegistration import PointCloudRegistration
from localization.BlockDetect import BlockDetect
from camera.PointCloudService import PointCloudService
from utils_ur5.TransformationUtils import TransformationUtils
from utils_ur5.Logger import Logger as log
from constants import *

# ---------------------- CLASS ----------------------

class Vision:
    """
    The class that starts the vision node
    """

    def __init__(self):
        """
        Constructor
        """

        ros.init_node('vision', anonymous=True)
        log.info('VISION NODE STARTED')

        zed = ZED()
        img = zed.get_image(verbose=True)
        zed.save_image(IMG_ZED_PATH, img)

        block_detect = BlockDetect(IMG_ZED_PATH, save_result=True, open_result=True)
        self.block_list = block_detect.block_list

        for block in self.block_list:
            PointCloudRegistration(block)
            log.debug(f'Block {block.name} localized at:')
            log.debug(f'Position: {block.position}')
            log.debug(f'Rotation: {block.rotation}')

        log.info('Done localizing blocks')
                
        
            
# ---------------------- MAIN ----------------------
# To use in command:
# python3 Vision.py
 
if __name__ == '__main__':

    vision = Vision()

    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
