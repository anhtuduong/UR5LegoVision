
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
from utils_ur5.Logger import Logger as log
from motion.command import Command

# Constants
from constants import *
from motion.action_list import ACTION_LIST

class MotionPlanner():
    """
    """

    def __init__(self, motion_planner_path=MOTION_PLANNER_PATH):
        """
        """
        self.motion_planner_path = motion_planner_path
        self.action_list = []
        self.run()
        self.to_json()

    def extract_json(self):
        """
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
                
            # Type make model static
            if item['type'] == 'spawn model static':
                self.action_list.append({'type': item['type'],
                                         'description': item['description'],
                                         'model_name': item['model_name'],
                                         'is_static': item['is_static']})
                
            # Type delete model
            if item['type'] == 'delete model':
                self.action_list.append({'type': item['type'],
                                         'description': item['description'],
                                         'model_name': item['model_name']})
                
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
        """
        # Extract pose
        pick_x = pose_pick[0]
        pick_y = pose_pick[1]
        pick_z = pose_pick[2]
        place_x = pose_place[0]
        place_y = pose_place[1]
        place_z = pose_place[2]

        return [
            # Pick
            Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
            Command.move_to(f'Move above {model_name}', pose=[pick_x, pick_y, pick_z + 0.1, 0.7071055, -0.7071081, 9e-7, 9e-7]),
            Command.move_gripper(f'Open gripper', diameter=50),
            Command.move_to(f'Move down to {model_name}', pose=[pick_x, pick_y, pick_z, 0.7071055, -0.7071081, 9e-7, 9e-7]),
            Command.move_gripper(f'Grasp {model_name}', diameter=35),
            Command.attach_models(f'Attach {model_name} to gripper', model_name_1=model_name, link_name_1='link', model_name_2=ROBOT_NAME, link_name_2=GRIPPER_LINK),
            Command.move_to(f'Pick up {model_name}', pose=[pick_x, pick_y, pick_z + 0.1, 0.7071055, -0.7071081, 9e-7, 9e-7]),

            # Place
            Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
            Command.move_to('Move above the place position', pose=[place_x, place_y, place_z + 0.1, 0.0, 1.0, 0.0, 0.0]),
            Command.move_to('Move down to the place position', pose=[place_x, place_y, place_z, 0.0, 1.0, 0.0, 0.0]),
            Command.detach_models(f'Detach {model_name} from gripper', model_name_1=model_name, link_name_1='link', model_name_2=ROBOT_NAME, link_name_2=GRIPPER_LINK),
            Command.move_gripper(f'Release {model_name}', diameter=50),
            Command.move_to(f'Move above {model_name}', pose=[place_x, place_y, place_z + 0.1, 0.0, 1.0, 0.0, 0.0]),
        ]

    def run(self):
        """
        """
        # Actions from manual list
        self.action_list = ACTION_LIST

        # Actions
        # model_1 = 'X1-Y1-Z2 1'
        # model_1_pose_pick = [0.185, 0.557, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]
        # model_1_pose_place = [0.72, 0.557, 0.91, 0.0, 1.0, 0.0, 0.0]
        # self.action_list.extend(self.pick_and_place(model_1, model_1_pose_pick, model_1_pose_place))

        # model_2 = 'X1-Y1-Z2 2'
        # model_2_pose_pick = [0.185, 0.7, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]
        # model_2_pose_place = [0.72, 0.557, 0.9475, 0.0, 1.0, 0.0, 0.0]
        # self.action_list.extend(self.pick_and_place(model_2, model_2_pose_pick, model_2_pose_place))



if __name__ == "__main__":
    mp = MotionPlanner()
    log.debug(mp.action_list)
