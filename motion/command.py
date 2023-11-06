"""!
@package motion.command
@file motion/command.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-25

@brief Defines the Command class
"""

# Resolve paths
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Import
from geometry_msgs.msg import Pose
from motion.utils import *

# ---------------------- CLASS ----------------------

class Command():
    """
    The class that defines the commands
    """

    def move_to(description='', pose=[]):
        """
        Command to move to a pose
        :param description: description of the command, ``str``
        :param pose: pose to move to, ``list``
        :return: command, ``dict``
        """
        pose_target = Pose()
        pose_target.position.x = pose[0]
        pose_target.position.y = pose[1]
        pose_target.position.z = pose[2]
        # Convert Euler angles to quaternions
        qx, qy, qz, qw = euler_to_quaternion(pose[3], pose[4], pose[5])
        pose_target.orientation.x = qx
        pose_target.orientation.y = qy
        pose_target.orientation.z = qz
        pose_target.orientation.w = qw
        
        return {
            'type': 'move to',
            'description': description,
            'pose': Pose_to_list(pose_target)
        }

    def move_joints(description='', joints=[]):
        """
        Command to move joints
        :param description: description of the command, ``str``
        :param joints: joints to move to, ``list``
        :return: command, ``dict``
        """
        return {
            'type': 'move joints',
            'description': description,
            'joints': joints
        }
    
    def move_gripper(description='', diameter=60):
        """
        Command to move the gripper
        :param description: description of the command, ``str``
        :param diameter: diameter of the gripper, ``float``
        :return: command, ``dict``
        """
        return {
            'type': 'move gripper',
            'description': description,
            'diameter': diameter
        }
    
    def attach_models(description='', model_name_1='', link_name_1='', model_name_2='', link_name_2=''):
        """
        Command to attach models
        :param description: description of the command, ``str``
        :param model_name_1: name of the first model, ``str``
        :param link_name_1: name of the first link, ``str``
        :param model_name_2: name of the second model, ``str``
        :param link_name_2: name of the second link, ``str``
        :return: command, ``dict``
        """
        return {
            'type': 'attach models',
            'description': description,
            'model_name_1': model_name_1,
            'link_name_1': link_name_1,
            'model_name_2': model_name_2,
            'link_name_2': link_name_2
        }
    
    def detach_models(description='', model_name_1='', link_name_1='', model_name_2='', link_name_2=''):
        """
        Command to detach models
        :param description: description of the command, ``str``
        :param model_name_1: name of the first model, ``str``
        :param link_name_1: name of the first link, ``str``
        :param model_name_2: name of the second model, ``str``
        :param link_name_2: name of the second link, ``str``
        :return: command, ``dict``
        """
        return {
            'type': 'detach models',
            'description': description,
            'model_name_1': model_name_1,
            'link_name_1': link_name_1,
            'model_name_2': model_name_2,
            'link_name_2': link_name_2
        }