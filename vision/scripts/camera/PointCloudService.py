"""!
@package vision.scripts.camera.PointCloudService
@file vision/scripts/camera/PointCloudService.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-04

@brief Defines the PointCloudService class.
"""

# Reslve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Import
import rospy as ros
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2
from utils_ur5.Logger import Logger as log
import pyvista as pv
import numpy as np
from constants import *

# Global variables
w_R_c = np.matrix([[0, -0.499, 0.866], [-1, 0, 0], [0, -0.866, -0.499]])
x_c = np.array([-0.9, 0.24, -0.35])
base_offset = np.array([0.5, 0.35, 1.75])

# ---------------------- CLASS ----------------------

class PointCloudService:
    """
    This class provides services for point cloud
    """

    def __init__(self):
        """
        Constructor
        """
        self.pointcloud_received = None
        self.pointcloud_sub = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, self.receive_pointcloud, queue_size=1)
        self.pointcloud_pub = ros.Publisher("/ur5/zed_node/point_cloud/block", PointCloud2, queue_size=1)
        # wait for point cloud to be ready
        ros.sleep(2)

    def receive_pointcloud(self, msg):
        """
        Callback function whenever take msg from ZED camera
        :param msg: data taken from ZED node
        """
        self.pointcloud_received = msg

    def get_pointcloud(self, pixels, verbose=False):
        """
        Get point_cloud from ZED camera
        :param pixels: list of pixel, ``list``
        :return point_cloud: list of point_cloud, ``list``
        """
        point_cloud = []
        for pixel in pixels:
            for data in point_cloud2.read_points(self.pointcloud_received, field_names=['x','y','z'], skip_nans=True, uvs=[pixel]):
                point_cloud.append((data[0], data[1], data[2]))

        if verbose:
            log.debug('point_cloud returned')
        return point_cloud

    def publish_pointcloud(self, pixels, verbose=False):
        """
        Publish point_cloud to ROS topic
        :param pixels: list of pixel, ``list``
        """
        pc = point_cloud2.create_cloud_xyz32(self.pointcloud_received.header, self.get_pointcloud(pixels))
        self.pointcloud_pub.publish(pc)
        if verbose:
            log.debug('point_cloud published')

    def save_pointcloud_to_ply(point_cloud, output_path, verbose=False):
        """
        Save point_cloud to PLY file
        :param point_cloud: list of point_cloud, ``list``
        :param output_path: path to PLY output file, ``str``
        """
        point_cloud = np.array(point_cloud)
        polydata = pv.PolyData(point_cloud)
        polydata.save(output_path)
        if verbose:
            log.debug(f'point_cloud saved to {output_path}')

    def clean_pointcloud(point_cloud, verbose=False):
        """
        Clean point_cloud that remove Z value between min(Z) and min(Z) + 0.001
        :param point_cloud: list of point_cloud, ``list``
        :return point_cloud: list of point_cloud, ``list``
        """
        point_cloud = np.array(point_cloud)
        z_min = np.min(point_cloud[:,2])
        z_max = z_min + 0.001
        point_cloud = point_cloud[np.where(point_cloud[:,2] > z_max)]
        if verbose:
            log.debug(f'point_cloud cleaned: Z value > {z_max}')
        return point_cloud

    # Visualize point clouds
    def visualize_pointcloud(point_cloud):
        """
        Visualize point cloud
        :param point_cloud: list of point_cloud, ``list``
        """

        cloud = pv.PolyData(point_cloud)
        cloud.plot(
            scalars=np.arange(cloud.n_points),
            render_points_as_spheres=True,
            point_size=5,
            show_grid=True,
            show_axes=True,
            show_bounds=True,
        )

    # Transform point cloud to world frame
    def transform_pointcloud_to_world_frame(point_cloud, verbose=False):
        """
        Transform point_cloud to world frame
        :param point_cloud: list of point_cloud, ``list``
        :return point_cloud: list of point_cloud, ``list``
        """
        point_cloud = np.array(point_cloud)
        for i in range(len(point_cloud)):
            point_cloud[i] = point_cloud[i].dot(w_R_c.T) + x_c + base_offset
        
        if verbose:
            log.debug('point_cloud transformed to world frame')
        return point_cloud
    
    # Get point cloud from ply file
    def get_pointcloud_from_ply(ply_path, verbose=False):
        """
        Get point_cloud from ply file
        :param ply_path: path to ply file, ``str``
        :return point_cloud: list of point_cloud, ``list``
        """
        point_cloud = pv.read(ply_path)
        point_cloud = np.array(point_cloud.points)
        if verbose:
            log.debug(f'point_cloud returned from {ply_path}')
        return point_cloud
    
    # Render point cloud from STL file
    def render_pointcloud_from_stl(stl_path, verbose=False):
        """
        Render point_cloud from STL file
        :param stl_path: path to STL file, ``str``
        :return point_cloud: list of point_cloud, ``list``
        """
        # Load the STL file
        mesh = pv.read(stl_path)

        # Convert the mesh to a point cloud
        point_cloud = mesh.points

        # Return the point cloud
        if verbose:
            log.debug(f'point_cloud returned from {stl_path}')
        return point_cloud

    # Save point cloud to PLY file
    def save_pointcloud_to_PLY(point_cloud, ply_path, verbose=False):
        """
        Save point_cloud to PLY file
        :param point_cloud: list of point_cloud, ``list``
        :param ply_path: path to PLY file, ``str``
        """
        point_cloud.save(ply_path)
        if verbose:
            log.debug(f"Saved point cloud to PLY file: {ply_path}")

if __name__ == '__main__':

    point_cloud = PointCloudService.get_pointcloud_from_ply(PLY_AFTER_TRANSFORM_PATH)
    PointCloudService.visualize_pointcloud(point_cloud)

    point_cloud = PointCloudService.clean_pointcloud(point_cloud)
    PointCloudService.save_pointcloud_to_ply(point_cloud, PLY_AFTER_CLEAN_PATH)
    
    point_cloud = PointCloudService.get_pointcloud_from_ply(PLY_AFTER_CLEAN_PATH)
    PointCloudService.visualize_pointcloud(point_cloud)
