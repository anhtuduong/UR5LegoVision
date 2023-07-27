"""!
@package vision.scripts.Vision
@file vision/scripts/Vision.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief Defines the Vision node that communicates with Motion node.
@date 2023-05-04
"""

# Resolve paths
import os
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
    @brief This class detects blocks from ZED camera and communicates with different ROS node
    """

    def __init__(self):
        """
        @brief Class constructor
        """

        ros.init_node('vision', anonymous=True)
        zed = ZED()
        zed.save_image(IMG_ZED_PATH, zed.get_image())
        block_detect = BlockDetect(IMG_ZED_PATH)
        self.block_list = block_detect.get_block_list()
        pc = PointCloudService()

        for block in self.block_list:
            pointcloud = pc.get_pointcloud(block.get_pixels())
            pointcloud = PointCloudService.transform_pointcloud_to_world_frame(pointcloud)
            pointcloud = PointCloudService.clean_pointcloud(pointcloud)
            
            # Load the point clouds
            target = PointCloudRegistration.load_point_cloud(pointcloud)
            source = PointCloudRegistration.load_point_cloud(MODEL[block.get_name()]['pointcloud_file'])

            # Set the color of the point clouds
            source.paint_uniform_color([1, 0, 0]) # Red
            target.paint_uniform_color([0, 0, 1]) # Blue

            # Downsample the point clouds
            source = PointCloudRegistration.downsample_point_cloud(source, 0.001)
            target = PointCloudRegistration.downsample_point_cloud(target, 0.001)

            # Compute the FPFH features
            fpfh_source = PointCloudRegistration.compute_FPFH_features(source)
            fpfh_target = PointCloudRegistration.compute_FPFH_features(target)

            # Perform FPFH feature matching
            transformation_matrix = PointCloudRegistration.match_FPFH_features(source,
                                                                            target, 
                                                                            fpfh_source, 
                                                                            fpfh_target
            )

            # Perform iterative refinement ICP
            transformation_matrix = PointCloudRegistration.iterative_refinement(source,
                                                                                target,
                                                                                transformation_matrix,
                                                                                num_iterations=10
            )

            # Compute the 6DOF transformation parameters
            translation, euler_angles = TransformationUtils.compute_6DoF(transformation_matrix)

            # Transform the source point cloud
            transformed_cloud = source
            transformed_cloud.transform(transformation_matrix)

            # Visualize the point clouds
            PointCloudRegistration.visualize_pointcloud([source, target])

            # Save the point clouds
            PointCloudRegistration.save_pointcloud_to_PLY([source, target], PLY_AFTER_ALIGN_PATH)

            block.set_point_cloud(transformed_cloud)
            block.set_transformation_matrix(transformation_matrix)
            block.set_pose(translation, euler_angles)

    def get_block_list(self):
        """ @brief Get list of detected blocks
            @return (list): list of detected blocks
        """
        return self.block_list

            
                
        
            
# ---------------------- MAIN ----------------------
# To use in command:
# python3 Vision.py
 
if __name__ == '__main__':

    vision = Vision()

    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
