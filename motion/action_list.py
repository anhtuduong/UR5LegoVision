"""!
@package motion/action_list
@file motion/action_list.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-25

@brief Defines the manual action list.
This is a list of manual commands that the robot will execute.
To enable this, set ``USE_ACTION_LIST = True`` in ``motion/motion_planner.py``.
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
from motion.command import Command

# Constants
robot_model = 'ur5'
gripper_link = 'wrist_3_link'
lego_link = 'link'

model_1 = 'X1-Y2-Z2-CHAMFER 1'
model_2 = 'X1-Y2-Z2-CHAMFER 2'
model_3 = 'X1-Y2-Z2'
model_4 = 'X1-Y4-Z2'
model_5 = 'X1-Y2-Z2-TWINFILLET'
model_6 = 'X1-Y1-Z2'

ACTION_LIST = [

    # Command.move_gripper(f'Open gripper', diameter=60),

    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to(f'Move above {model_1}', pose=[0.19, 0.48, 1.0, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    Command.move_to(f'Move side {model_1}', pose=[0.19, 0.48, 1.0, 0, -0.7071068, 0, 0.7071068]),
    # Command.move_to(f'Move down to {model_1}', pose=[0.19, 0.48, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_gripper(f'Grasp {model_1}', diameter=35),
    # Command.attach_models(f'Attach {model_1} to gripper', model_name_1=model_1, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_to(f'Pick up {model_1}', pose=[0.19, 0.48, 1.1, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move above the place position', pose=[0.663671, 0.7, 1.0, -1.0, 0.0, 0.0, 0.0]),
    # Command.move_to('Move down to the place position', pose=[0.663671, 0.7, 0.91, -1.0, 0.0, 0.0, 0.0]),
    # Command.attach_models(f'Attach {model_1} to table', model_name_1=model_1, link_name_1=lego_link, model_name_2='tavolo', link_name_2='link'),
    # Command.detach_models(f'Detach {model_1} from gripper', model_name_1=model_1, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_gripper(f'Release {model_1}', diameter=60),
    # Command.move_to(f'Move above {model_1}', pose=[0.663671, 0.7, 1.1, -1.0, 0.0, 0.0, 0.0]),

    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to(f'Move above {model_2}', pose=[0.26, 0.63, 1.0, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move down to {model_2}', pose=[0.26, 0.63, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_gripper(f'Grasp {model_2}', diameter=35),
    # Command.attach_models(f'Attach {model_2} to gripper', model_name_1=model_2, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_to(f'Pick up {model_2}', pose=[0.26, 0.63, 1.1, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move above the place position', pose=[0.536209, 0.7, 1.0, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move down to the place position', pose=[0.536209, 0.7, 0.91, 0.0, 1.0, 0.0, 0.0]),
    # Command.attach_models(f'Attach {model_2} to table', model_name_1=model_2, link_name_1=lego_link, model_name_2='tavolo', link_name_2='link'),
    # Command.detach_models(f'Detach {model_2} from gripper', model_name_1=model_2, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_gripper(f'Release {model_2}', diameter=60),
    # Command.move_to(f'Move above {model_2}', pose=[0.536209, 0.7, 1.1, 0.0, 1.0, 0.0, 0.0]),

    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to(f'Move above {model_3}', pose=[0.09, 0.51, 1.0, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move down to {model_3}', pose=[0.09, 0.51, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_gripper(f'Grasp {model_3}', diameter=35),
    # Command.attach_models(f'Attach {model_3} to gripper', model_name_1=model_3, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_to(f'Pick up {model_3}', pose=[0.09, 0.51, 1.1, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move above the place position', pose=[0.599847, 0.7, 1.0, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move down to the place position', pose=[0.599847, 0.7, 0.91, 0.0, 1.0, 0.0, 0.0]),
    # Command.attach_models(f'Attach {model_3} to table', model_name_1=model_3, link_name_1=lego_link, model_name_2='tavolo', link_name_2='link'),
    # Command.detach_models(f'Detach {model_3} from gripper', model_name_1=model_3, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_gripper(f'Release {model_3}', diameter=60),
    # Command.move_to(f'Move above {model_3}', pose=[0.599847, 0.7, 1.1, 0.0, 1.0, 0.0, 0.0]),

    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to(f'Move above {model_4}', pose=[0.09, 0.69, 1.0, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move down to {model_4}', pose=[0.09, 0.69, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_gripper(f'Grasp {model_4}', diameter=35),
    # Command.attach_models(f'Attach {model_4} to gripper', model_name_1=model_4, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_to(f'Pick up {model_4}', pose=[0.09, 0.69, 1.1, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move above the place position', pose=[0.6, 0.7, 1.05, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move down to the place position', pose=[0.6, 0.7, 0.9483, 0.0, 1.0, 0.0, 0.0]),
    # Command.attach_models(f'Attach {model_4} to table', model_name_1=model_4, link_name_1=lego_link, model_name_2='tavolo', link_name_2='link'),
    # Command.detach_models(f'Detach {model_4} from gripper', model_name_1=model_4, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_gripper(f'Release {model_4}', diameter=60),
    # Command.move_to(f'Move above {model_4}', pose=[0.6, 0.7, 1.1, 0.0, 1.0, 0.0, 0.0]),

    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to(f'Move above {model_5}', pose=[0.19, 0.71, 1.0, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move down to {model_5}', pose=[0.19, 0.71, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_gripper(f'Grasp {model_5}', diameter=35),
    # Command.attach_models(f'Attach {model_5} to gripper', model_name_1=model_5, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_to(f'Pick up {model_5}', pose=[0.19, 0.71, 1.1, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move above the place position', pose=[0.6, 0.7, 1.03, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move down to the place position', pose=[0.6, 0.7, 0.9863, 0.0, 1.0, 0.0, 0.0]),
    # Command.attach_models(f'Attach {model_5} to table', model_name_1=model_5, link_name_1=lego_link, model_name_2='tavolo', link_name_2='link'),
    # Command.detach_models(f'Detach {model_5} from gripper', model_name_1=model_5, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_gripper(f'Release {model_5}', diameter=60),
    # Command.move_to(f'Move above {model_5}', pose=[0.6, 0.7, 1.1, 0.0, 1.0, 0.0, 0.0]),

    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to(f'Move above {model_6}', pose=[0.16, 0.58, 1.0, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move down to {model_6}', pose=[0.16, 0.58, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_gripper(f'Grasp {model_6}', diameter=35),
    # Command.attach_models(f'Attach {model_6} to gripper', model_name_1=model_6, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_to(f'Pick up {model_6}', pose=[0.16, 0.58, 1.1, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move above the place position', pose=[0.600155, 0.7, 1.07, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move down to the place position', pose=[0.600155, 0.7, 1.0245, 0.0, 1.0, 0.0, 0.0]),
    # Command.attach_models(f'Attach {model_6} to table', model_name_1=model_6, link_name_1=lego_link, model_name_2='tavolo', link_name_2='link'),
    # Command.detach_models(f'Detach {model_6} from gripper', model_name_1=model_6, link_name_1=lego_link, model_name_2=robot_model, link_name_2=gripper_link),
    # Command.move_gripper(f'Release {model_6}', diameter=60),
    # Command.move_to(f'Move above {model_6}', pose=[0.600155, 0.7, 1.1, 0.0, 1.0, 0.0, 0.0]),

    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),

]