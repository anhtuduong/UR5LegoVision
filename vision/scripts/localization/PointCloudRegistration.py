

# Reslove paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Constants
from constants import *

SOURCE_PATH = MODEL['X1-Y3-Z2-FILLET']['pointcloud_file']
TARGET_PATH = PLY_AFTER_CLEAN_PATH

# Import
import numpy as np
from scipy.spatial.transform import Rotation
from vision.scripts.camera.PointCloudService import PointCloudService
from utils_ur5.Logger import Logger as log
from utils_ur5.TimeExecution import TimeExecution
from utils_ur5.TransformationUtils import TransformationUtils
import open3d as o3d

class PointCloudRegistration:
    """
    @brief Class for point cloud registration
    """

    def load_point_cloud(point_cloud):
        """
        @brief Load point clouds regard to PLY file or point cloud taken from PLY file
        @param point_cloud (str or list): path to PLY file or point cloud taken from PLY file
        @return point_cloud (open3d.geometry.PointCloud): point cloud
        """
        
        # If point_cloud is a string, it is a path to a PLY file
        if isinstance(point_cloud, str):
            cloud = o3d.io.read_point_cloud(point_cloud)
        
        # If point_cloud is a list, it is a point cloud taken from PLY file
        elif isinstance(point_cloud, list) or isinstance(point_cloud, np.ndarray):
            point_cloud = np.array(point_cloud)
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(point_cloud)

        else:
            raise TypeError("point_cloud must be a string of PLY path or a list")

        # Print the point cloud information
        log.info(f'Point cloud loaded: {len(cloud.points)} points')

        return cloud

    def visualize_pointcloud(point_clouds):
        """
        @brief Visualize the point clouds
        @param point_clouds (list): list of point clouds
        """
        if not isinstance(point_clouds, list):
            point_clouds = [point_clouds]

        # Create a visualization window
        vis = o3d.visualization.Visualizer()
        vis.create_window()

        # Add the point clouds to the visualization
        for point_cloud in point_clouds:
            vis.add_geometry(point_cloud)

        # Set the camera viewpoint
        vis.get_view_control().set_front([0, 0, -1])
        vis.get_view_control().set_lookat([0, 0, 0])
        vis.get_view_control().set_up([0, -1, 0])
        vis.get_view_control().set_zoom(0.8)

        # Run the visualization
        vis.run()
        vis.destroy_window()

    # Convert open3d.geometry.PointCloud to list of point_cloud
    def convert_open3d_pointcloud_to_list(point_cloud):
        """
        @brief Convert open3d.geometry.PointCloud to list of point_cloud
        @param point_cloud (open3d.geometry.PointCloud): point cloud
        @return point_cloud (list): list of point_cloud
        """
        point_cloud = np.array(point_cloud.points)
        return point_cloud
    
    # Save list point cloud to one PLY file
    def save_pointcloud_to_PLY(point_clouds, ply_path):
        """
        @brief Save list point cloud to one PLY file
        @param point_clouds (list): list of point clouds
        @param ply_path (str): path to PLY output file
        """
        if not isinstance(point_clouds, list):
            point_clouds = [point_clouds]
        
        result_cloud = o3d.geometry.PointCloud()
        for point_cloud in point_clouds:
            result_cloud = result_cloud + point_cloud

        o3d.io.write_point_cloud(ply_path, result_cloud)
        log.debug(f"Point cloud saved to {ply_path}")

    def downsample_point_cloud(point_cloud, voxel_size=0.05):
        """
        @brief Downsample the point cloud
        @param point_cloud: point cloud
        @return downsampled_point_cloud (np.array): downsampled point cloud
        """

        # Downsample the point cloud
        downsampled_point_cloud = point_cloud.voxel_down_sample(voxel_size)
        log.info(f"Point cloud downsampled: {len(downsampled_point_cloud.points)} points")

        return downsampled_point_cloud

    def compute_FPFH_features(point_cloud):
        """
        @brief Compute FPFH features
        @param point_cloud: point cloud
        @return fpfh_features (np.array): FPFH features
        """
        time_execution = TimeExecution()

        try:
            # Compute normals
            point_cloud.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
            )

            # Compute FPFH features
            radius_feature = 0.2
            fpfh_features = o3d.pipelines.registration.compute_fpfh_feature(
                point_cloud,
                o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=30)
            )
            log.debug(f"FPFH features computed in {time_execution.get_duration():.3f}s")

        except Exception as e:
            log.error(f"FPFH features not computed: {e}")
            fpfh_features = None

        return fpfh_features

    def match_FPFH_features(source, target, fpfh_source, fpfh_target):
        """
        @brief Perform FPFH feature matching
        @param source: source point cloud
        @param target: target point cloud
        @param fpfh_source (np.array): FPFH features of the source point cloud
        @param fpfh_target (np.array): FPFH features of the target point cloud
        @return transformation_matrix (np.array): 4x4 transformation matrix
        """
        time_execution = TimeExecution()

        distance_threshold = 0.005
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source, target, fpfh_source, fpfh_target,
            mutual_filter=True,
            max_correspondence_distance=distance_threshold,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=4,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.999)
        )

        transformation_matrix = result.transformation
        log.debug(f"FPFH features matched in {time_execution.get_duration():.3f}s")

        return transformation_matrix

    def icp_alignment(source, target, initial_transformation):
        """
        @brief Alignment using ICP
        @param source: source point cloud
        @param target: target point cloud
        @param initial_transformation (np.array): 4x4 initial transformation matrix
        @return transformation_matrix (np.array): 4x4 transformation matrix
        """

        transformation_matrix = initial_transformation

        max_correspondence_distance = 0.005
        result_icp = o3d.pipelines.registration.registration_icp(
            source, target, max_correspondence_distance,
            transformation_matrix,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
        )

        transformation_matrix = result_icp.transformation

        evaluation = o3d.pipelines.registration.evaluate_registration(
            source, target, max_correspondence_distance, transformation_matrix
        )
        log.debug(f"fitness={evaluation.fitness:.3f}, inlier_rmse={evaluation.inlier_rmse:.3f}, correspondence_set={len(result_icp.correspondence_set)}")

        return transformation_matrix
    
    def iterative_refinement(source, target, initial_transformation, num_iterations=1):
        """
        @brief Iterative refinement
        @param source: source point cloud
        @param target: target point cloud
        @param initial_transformation (np.array): 4x4 initial transformation matrix
        @return transformation_matrix (np.array): 4x4 transformation matrix
        """
        time_execution = TimeExecution()

        log.debug(f"Iterative refinement ICP: {num_iterations} iterations")

        transformation_matrix = initial_transformation

        for i in range(num_iterations):
            transformation_matrix = PointCloudRegistration.icp_alignment(
                source, target, transformation_matrix
            )

        log.debug(f"Iterative refinement ICP in {time_execution.get_duration():.3f}s")

        return transformation_matrix


# Main
if __name__ == "__main__":

    # # Load the point clouds
    raw_point_clouds = PointCloudRegistration.load_point_cloud(PLY_AFTER_TRANSFORM_PATH)
    target = PointCloudRegistration.load_point_cloud(TARGET_PATH)
    source = PointCloudRegistration.load_point_cloud(SOURCE_PATH)

    # Set the color of the point clouds
    raw_point_clouds.paint_uniform_color([0, 0, 1]) # Blue
    target.paint_uniform_color([0, 0, 1]) # Blue
    source.paint_uniform_color([1, 0, 0]) # Red

    # Visualize the point clouds
    PointCloudRegistration.visualize_pointcloud(raw_point_clouds)
    PointCloudRegistration.visualize_pointcloud(target)
    PointCloudRegistration.visualize_pointcloud(source)

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