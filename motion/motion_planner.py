"""!
@package motion.motion_planner
@file motion/motion_planner.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-25

@brief Defines the MotionPlanner class that plans the motion
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
import json
import numpy as np
from geometry_msgs.msg import Pose
import locosim.robot_control.base_controllers.params as conf
from motion.utils import *
from utils_ur5.Logger import Logger as log
from motion.command import Command

# Constants
from constants import *
from motion.action_list import ACTION_LIST

USE_ACTION_LIST = True

# ---------------------- CLASS ----------------------

class MotionPlanner():
    """
    The class that plans the motion
    """

    def __init__(self, motion_planner_path=MOTION_PLANNER_PATH):
        """
        Constructor
        :param motion_planner_path: path to the motion planner file, ``str``
        """
        self.motion_planner_path = motion_planner_path
        self.action_list = []
        self.run()
        self.to_json()

    def extract_json(self):
        """
        Parses the JSON file and extracts the list of objects
        """
        # Load the JSON content from the file
        with open(self.motion_planner_path, 'r') as file:
            json_data = json.load(file)

        # Extract the list of objects
        for item in json_data:

            # Type move to
            if item['type'] == 'move to':
                pose = Pose()
                pose.position.x = item['pose']['position']['x']
                pose.position.y = item['pose']['position']['y']
                pose.position.z = item['pose']['position']['z']
                pose.orientation.x = item['pose']['orientation']['x']
                pose.orientation.y = item['pose']['orientation']['y']
                pose.orientation.z = item['pose']['orientation']['z']
                pose.orientation.w = item['pose']['orientation']['w']
                self.action_list.append({'type': item['type'],
                                         'description': item['description'],
                                         'pose': pose})
            
            # Type move joints
            if item['type'] == 'move joints':
                self.action_list.append({'type': item['type'],
                                         'description': item['description'],
                                         'joints': np.array(item['joints'])})
                
            # Type move gripper
            if item['type'] == 'move gripper':
                self.action_list.append({'type': item['type'],
                                         'description': item['description'],
                                         'diameter': item['diameter']})
                
            # Type attach models
            if item['type'] == 'attach models':
                self.action_list.append({'type': item['type'],
                                         'description': item['description'],
                                         'model_name_1': item['model_name_1'],
                                         'link_name_1': item['link_name_1'],
                                         'model_name_2': item['model_name_2'],
                                         'link_name_2': item['link_name_2']})
                
            # Type detach models
            if item['type'] == 'detach models':
                self.action_list.append({'type': item['type'],
                                         'description': item['description'],
                                         'model_name_1': item['model_name_1'],
                                         'link_name_1': item['link_name_1'],
                                         'model_name_2': item['model_name_2'],
                                         'link_name_2': item['link_name_2']})

    def to_json(self):
        """
        Saves the list of objects to a JSON file
        """
        # Create a list of objects
        json_data = []
        for item in self.action_list:
            json_data.append(item)
                
        # Save the JSON content to the file
        with open(self.motion_planner_path, 'w') as file:
            json.dump(json_data, file, indent=4)


    def pick_and_place(self, model_name, pose_pick, pose_place):
        """
        Creates a pick and place sequence
        :param model_name: The name of the model, ``str``
        :param pose_pick: The pick pose, ``list``
        :param pose_place: The place pose, ``list``
        :return: The list of actions, ``list``
        """
        # Extract pose
        pick = list_to_Pose(pose_pick)
        place = list_to_Pose(pose_place)

        action_list = []

        # Pick
        action_list.append(Command.move_to(f'Move to middle', pose=conf.robot_params[ROBOT_NAME]['pose_middle']))
        pick.position.z += 0.15
        action_list.append(Command.move_to(f'Move above {model_name}', pose=Pose_to_list(pick)))
        action_list.append(Command.move_gripper(f'Open gripper', diameter=60))
        pick.position.z -= 0.1
        action_list.append(Command.move_to(f'Move down {model_name}', pose=Pose_to_list(pick)))
        pick.position.z -= 0.05
        action_list.append(Command.move_to(f'Move down to {model_name}', pose=Pose_to_list(pick)))
        action_list.append(Command.move_gripper(f'Grasp {model_name}', diameter=35))
        action_list.append(Command.attach_models(f'Attach {model_name} to gripper', model_name_1=model_name, link_name_1='link', model_name_2=ROBOT_NAME, link_name_2=GRIPPER_LINK))
        pick.position.z += 0.05
        action_list.append(Command.move_to(f'Pick up {model_name}', pose=Pose_to_list(pick)))
        pick.position.z += 0.1
        action_list.append(Command.move_to(f'Move up {model_name}', pose=Pose_to_list(pick)))

        # Place
        action_list.append(Command.move_to(f'Move to middle', pose=conf.robot_params[ROBOT_NAME]['pose_middle']))
        place.position.z += 0.15
        action_list.append(Command.move_to('Move above the place position', pose=Pose_to_list(place)))
        place.position.z -= 0.1
        action_list.append(Command.move_to('Move down', pose=Pose_to_list(place)))
        place.position.z -= 0.05
        action_list.append(Command.move_to('Move down to the place position', pose=Pose_to_list(place)))
        action_list.append(Command.attach_models(f'Attach {model_name} to table', model_name_1=model_name, link_name_1='link', model_name_2='tavolo', link_name_2='link'))
        action_list.append(Command.detach_models(f'Detach {model_name} from gripper', model_name_1=model_name, link_name_1='link', model_name_2=ROBOT_NAME, link_name_2=GRIPPER_LINK))
        action_list.append(Command.move_gripper(f'Release {model_name}', diameter=60))
        place.position.z += 0.05
        action_list.append(Command.move_to(f'Move above {model_name}', pose=Pose_to_list(place)))
        place.position.z += 0.1
        action_list.append(Command.move_to(f'Move up', pose=Pose_to_list(place)))

        return action_list

    def run(self):
        """
        Executes the actions in the motion plan
        """

        if USE_ACTION_LIST:
            self.action_list = ACTION_LIST
            return

        # Actions
        model = 'X1-Y2-Z2-CHAMFER 1'
        pick = [0.19, 0.48, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]
        place = [0.663671, 0.7, 0.91, -1.0, 0.0, 0.0, 0.0]
        self.action_list.extend(self.pick_and_place(model, pick, place))

        model = 'X1-Y2-Z2-CHAMFER 2'
        pick = [0.26, 0.63, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]
        place = [0.536209, 0.7, 0.91, 0.0, 1.0, 0.0, 0.0]
        self.action_list.extend(self.pick_and_place(model, pick, place))

        model = 'X1-Y2-Z2'
        pick = [0.09, 0.51, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]
        place = [0.599847, 0.7, 0.91, 0.0, 1.0, 0.0, 0.0]
        self.action_list.extend(self.pick_and_place(model, pick, place))

        model = 'X1-Y4-Z2'
        pick = [0.09, 0.69, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]
        place = [0.6, 0.7, 0.9483, 0.0, 1.0, 0.0, 0.0]
        self.action_list.extend(self.pick_and_place(model, pick, place))

        model = 'X1-Y2-Z2-TWINFILLET'
        pick = [0.19, 0.71, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]
        place = [0.6, 0.7, 0.9863, 0.0, 1.0, 0.0, 0.0]
        self.action_list.extend(self.pick_and_place(model, pick, place))

        model = 'X1-Y1-Z2'
        pick = [0.16, 0.58, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]
        place = [0.600155, 0.7, 1.0245, 0.0, 1.0, 0.0, 0.0]
        self.action_list.extend(self.pick_and_place(model, pick, place))



if __name__ == "__main__":
    mp = MotionPlanner()
    log.debug(mp.action_list)
