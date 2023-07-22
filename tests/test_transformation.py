"""!
@package tests.test_transformation
@file tests/test_transformation.py
@brief This module contains the unit tests for the transformation utils module.
@author Anh Tu Duong
@date 2023-06-07
"""

# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Import
import unittest
import numpy as np
import rospkg
import geometry_msgs.msg
from utilities.TransformationUtils import TransformationUtils as transformUtils
from utilities.Logger import Logger as log



class TestTransformation(unittest.TestCase):
    """
    @brief This class contains the unit tests for the transformation utils module.
    """

    # def __init__(self, methodName: str = "runTest") -> None:
    #     super().__init__(methodName)
        
    def test_transform_pose_across_frame():
        """
        @brief This method tests the transform_pose_across_frame method.
        """
        pose = geometry_msgs.msg.Pose()
        pose.position.x = -0.5
        pose.position.y = -0.35
        pose.position.z = -1.75
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        pose = transformUtils.transform_pose_across_frame(pose, "base_link", "world")
        log.info(f"Transformed pose:\n{pose}")
        # self.assertTrue(np.allclose(pose, np.array([0.30276, 0.60204, 0.89147, 0, 0, 0, 1])))



if __name__ == '__main__':
    # unittest.main()
    TestTransformation.test_transform_pose_across_frame()
    