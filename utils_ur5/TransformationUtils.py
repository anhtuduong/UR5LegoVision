
# Reslove paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path to ROOT

# Import
import numpy as np
import rospy as ros
import tf
from geometry_msgs.msg import PoseStamped
from utils_ur5.Logger import Logger as log

class TransformationUtils:
    """
    @brief Transformation utilities
    """

    def compute_6DoF(transformation_matrix, verbose=False):
        """
        @brief Compute the 6DOF transformation parameters
        @param transformation_matrix_icp (np.array): 4x4 transformation matrix
        @return translation (np.array): 3x1 translation vector
        @return euler_angles (np.array): 3x1 Euler angles
        """

        translation = transformation_matrix[:3, 3]
        rotation_matrix = transformation_matrix[:3, :3]

        # Compute the Euler angles from the rotation matrix
        euler_angles = np.degrees(TransformationUtils.rotation_matrix_to_euler_angles(rotation_matrix))

        if verbose:
            # Print the 6DOF transformation parameters
            log.info(f"Translation: {translation}")
            log.info(f"Euler angles: {euler_angles}")

        return translation, euler_angles

    def rotation_matrix_to_euler_angles(rotation):
        """
        @brief Extract the Euler angles from the rotation matrix
        @param rotation (np.array): 3x3 rotation matrix
        @return euler_angles (np.array): 3x1 Euler angles
        """

        sy = np.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])

        if sy < 1e-6:
            # Singular case (cos(theta) = 0)
            x = np.arctan2(rotation[1, 2], rotation[1, 1])
            y = np.arctan2(-rotation[2, 0], sy)
            z = 0.0
        else:
            x = np.arctan2(rotation[2, 1], rotation[2, 2])
            y = np.arctan2(-rotation[2, 0], sy)
            z = np.arctan2(rotation[1, 0], rotation[0, 0])

        euler_angles = np.array([x, y, z])
        return euler_angles
    
    def transform_pose_across_frame(input_pose, from_frame, to_frame):
        """
        @brief Transform a pose from one frame to another
        @param input_pose (Pose): Pose to transform
        @param from_frame (str): Frame to transform from
        @param to_frame (str): Frame to transform to
        @return output_pose (Pose): Transformed pose
        """

        # PoseStamped
        pose_in = PoseStamped()
        pose_in.header.frame_id = from_frame
        pose_in.pose = input_pose

        listener = tf.TransformListener()

        # Allow tf to catch up
        ros.sleep(2.0)

        # Transform the pose
        try:
            pose_out = listener.transformPose(to_frame, pose_in)
            return pose_out.pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            ros.logerr("Transform failed")
            return None

    def pose_to_position_rotation(pose_msg):
        """
        @brief Extract position and rotation matrix from a pose message
        @param pose_msg (Pose): Pose message
        @return position (np.array): 3x1 position vector
        @return rotation_matrix (np.array): 3x3 rotation matrix
        """
        # Extract position from the pose message
        position = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
        
        # Extract quaternion components from the pose message
        quaternion = np.array([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w])
        
        # Normalize the quaternion
        quaternion /= np.linalg.norm(quaternion)
        
        # Calculate the elements of the rotation matrix
        x, y, z, w = quaternion
        rotation_matrix = np.array([
            [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
        ])
        
        return position, rotation_matrix

    def get_original_position(transformed_position):
        """
        @brief Get the original position from the transformed position
        @param transformed_position (np.array): 3x1 transformed position vector
        @return original_position (np.array): 3x1 original position vector
        """
        # Calculate the original position by subtracting the transformed position
        original_position = (
            transformed_position[0] - 0.5,
            transformed_position[1] - 0.35,
            transformed_position[2] - 1.75
        )
        return original_position


