
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Import
from geometry_msgs.msg import Pose

class Command():

    def move_to(description='', pose=[]):
        """
        """
        pose_target = Pose()
        pose_target.position.x = pose[0]
        pose_target.position.y = pose[1]
        pose_target.position.z = pose[2]
        pose_target.orientation.x = pose[3]
        pose_target.orientation.y = pose[4]
        pose_target.orientation.z = pose[5]
        pose_target.orientation.w = pose[6]
        
        return {
            'type': 'move to',
            'description': description,
            'pose': pose
        }

    def move_joints(description='', joints=[]):
        """
        """
        return {
            'type': 'move joints',
            'description': description,
            'joints': joints
        }
    
    def move_gripper(description='', diameter=60):
        """
        """
        return {
            'type': 'move gripper',
            'description': description,
            'diameter': diameter
        }
    
    def spawn_model_static(description='', model_name='', is_static=True):
        """
        """
        return {
            'type': 'spawn model static',
            'description': description,
            'model_name': model_name,
            'is_static': is_static
        }
    
    def delete_model(description='', model_name=''):
        """
        """
        return {
            'type': 'delete model',
            'description': description,
            'model_name': model_name
        }
    
    def attach_models(description='', model_name_1='', link_name_1='', model_name_2='', link_name_2=''):
        """
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
        """
        return {
            'type': 'detach models',
            'description': description,
            'model_name_1': model_name_1,
            'link_name_1': link_name_1,
            'model_name_2': model_name_2,
            'link_name_2': link_name_2
        }