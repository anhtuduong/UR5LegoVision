"""!
@package vision.scripts.camera.ZED
@file vision/scripts/camera/ZED.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-04

@brief Defines the ZED camera class.
"""

# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import rospy as ros
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
from utils_ur5.Logger import Logger as log

# ---------------------- CLASS ----------------------

class ZED:
    """
    This class represents the ZED camera
    """

    def __init__(self):
        """
        Constructor
        """
        self.cv_image = None
        self.cv_bridge = CvBridge()
        self.image_sub = ros.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, self.receive_image)
        # wait for camera to be ready
        ros.sleep(2)
        log.debug('ZED camera initialized')

    def receive_image(self, data):
        """
        Callback function whenever take msg from ZED camera
        :param msg: data taken from ZED node, ``Image``
        """
        # Convert ROS image to OpenCV image
        try:
            self.cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
            # log.debug('image received from zed_node')
        except CvBridgeError as e:
            print(e)

    def get_image(self):
        """
        Get image from ZED camera
        :return: image from ZED camera, ``numpy.ndarray``
        """
        log.debug('image returned')
        return self.cv_image
    
    def save_image(self, path, img):
        """
        Save image from ZED camera
        :param path: path to save image, ``str``
        """
        try:
            cv.imwrite(path, img)
            log.debug('image saved to:' + path)
        except CvBridgeError as e:
            print(e)
        
