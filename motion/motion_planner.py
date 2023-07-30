
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
from utils_ur5.Logger import Logger as log

# Constants
from constants import PLANNER_PATH
from motion.action_list import ACTION_LIST

class MotionPlanner():
    """
    """

    def __init__(self, planner_path=PLANNER_PATH):
        """
        """
        self.planner_path = planner_path
        self.action_list = []
        self.run()
        self.to_json()

    def extract_json(self):
        """
        """
        # Load the JSON content from the file
        with open(self.planner_path, 'r') as file:
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
        with open(self.planner_path, 'w') as file:
            json.dump(json_data, file, indent=4)

    def run(self):
        """
        """
        # Add actions
        self.action_list = ACTION_LIST

if __name__ == "__main__":
    mp = MotionPlanner()
    log.debug(mp.action_list)
