
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


class PointCloudService:

    def __init__(self):
        self.pointcloud_received = None
        self.pointcloud_sub = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, self.receive_pointcloud, queue_size=1)
        self.pointcloud_pub = ros.Publisher("/ur5/zed_node/point_cloud/block", PointCloud2, queue_size=1)
        # wait for point cloud to be ready
        ros.sleep(2)
        log.debug('Point cloud initialized')

    def receive_pointcloud(self, msg):
        """ @brief Callback function whenever take point_cloud msg from ZED camera
            @param msg (msg): msg taken from ZED node
        """
        self.pointcloud_received = msg

    def get_pointcloud(self, pixels):
        """ @brief Get point_cloud from ZED camera
            @param pixels (list): list of pixels
            @return point_cloud (list): list of point_cloud
        """
        point_cloud = []
        for pixel in pixels:
            for data in point_cloud2.read_points(self.pointcloud_received, field_names=['x','y','z'], skip_nans=True, uvs=[pixel]):
                point_cloud.append((data[0], data[1], data[2]))

        log.debug('point_cloud returned')
        return point_cloud

    def publish_pointcloud(self, pixels):
        """ @brief Publish point_cloud to ZED camera
            @param pixels (list): list of pixel
        """
        pc = point_cloud2.create_cloud_xyz32(self.pointcloud_received.header, self.get_pointcloud(pixels))
        self.pointcloud_pub.publish(pc)
        log.debug('point_cloud published')

    def save_pointcloud_to_ply(point_cloud, output_path):
        """ @brief Save point_cloud to ply file
            @param pointcloud (list): list of point_cloud
            @param output_path (str): output path of ply file
        """
        point_cloud = np.array(point_cloud)
        polydata = pv.PolyData(point_cloud)
        polydata.save(output_path)
        log.debug(f'point_cloud saved to {output_path}')

    # Clean point_cloud that remove Z value between min(Z) and min(Z) + 0.001
    def clean_pointcloud(point_cloud):
        """ @brief Clean point_cloud that remove Z value between min(Z) and min(Z) + 0.001
            @param point_cloud (list): list of point_cloud
            @return point_cloud (list): list of point_cloud
        """
        point_cloud = np.array(point_cloud)
        z_min = np.min(point_cloud[:,2])
        z_max = z_min + 0.001
        point_cloud = point_cloud[np.where(point_cloud[:,2] > z_max)]
        log.debug(f'point_cloud cleaned: Z value > {z_max}')
        return point_cloud

    # Visualize point clouds
    def visualize_pointcloud(point_cloud):
        """
        @brief Visualize point cloud
        @param point_clouds (list): list of point_clouds as NumPy arrays
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
    def transform_pointcloud_to_world_frame(point_cloud):
        """ @brief Transform point_cloud to world frame
            @param point_cloud (list): list of point_cloud
            @return point_cloud (list): list of point_cloud
        """
        point_cloud = np.array(point_cloud)
        for i in range(len(point_cloud)):
            point_cloud[i] = point_cloud[i].dot(w_R_c.T) + x_c + base_offset
        
        log.debug('point_cloud transformed to world frame')
        return point_cloud
    
    # Get point cloud from ply file
    def get_pointcloud_from_ply(ply_path):
        """ @brief Get point_cloud from ply file
            @param ply_path (str): path to ply file
            @return point_cloud (list): list of point_cloud
        """
        point_cloud = pv.read(ply_path)
        point_cloud = np.array(point_cloud.points)
        log.debug(f'point_cloud returned from {ply_path}')
        return point_cloud
    
    # Render point cloud from STL file
    def render_pointcloud_from_stl(stl_path):
        """ @brief Render point_cloud from STL file
            @param stl_path (str): path to stl file
            @return point_cloud (list): list of point_cloud
        """
        # Load the STL file
        mesh = pv.read(stl_path)

        # Convert the mesh to a point cloud
        point_cloud = mesh.points

        # Return the point cloud
        log.debug(f'point_cloud returned from {stl_path}')
        return point_cloud

    # Save point cloud to PLY file
    def save_pointcloud_to_PLY(point_cloud, ply_path):
        """ @brief Save point_cloud to PLY file
            @param point_cloud (list): list of point_cloud
            @param ply_path (str): path to PLY output file
        """
        point_cloud.save(ply_path)
        log.debug(f"Saved point cloud to PLY file: {ply_path}")

if __name__ == '__main__':

    point_cloud = PointCloudService.get_pointcloud_from_ply(PLY_AFTER_TRANSFORM_PATH)
    PointCloudService.visualize_pointcloud(point_cloud)

    point_cloud = PointCloudService.clean_pointcloud(point_cloud)
    PointCloudService.save_pointcloud_to_ply(point_cloud, PLY_AFTER_CLEAN_PATH)
    
    point_cloud = PointCloudService.get_pointcloud_from_ply(PLY_AFTER_CLEAN_PATH)
    PointCloudService.visualize_pointcloud(point_cloud)
