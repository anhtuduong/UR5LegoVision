
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
model_1 = 'X1-Y4-Z2'
model_2 = 'X1-Y2-Z2'
model_3 = 'X1-Y1-Z2'

ACTION_LIST = [

    Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    Command.move_to(f'Move above {model_1}', pose=[0.185, 0.557, 1.1, 0.0, 1.0, 0.0, 0.0]),
    Command.move_to(f'Move above {model_1}', pose=[0.185, 0.557, 1.1, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    Command.move_gripper(f'Open gripper', diameter=50),
    Command.move_to(f'Move down to {model_1}', pose=[0.185, 0.557, 0.91, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    Command.move_gripper(f'Grasp {model_1}', diameter=29),
    Command.move_to(f'Pick up {model_1}', pose=[0.185, 0.557, 1.0, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    Command.move_to('Move above the place position', pose=[0.72, 0.557, 1.0, 0.0, 1.0, 0.0, 0.0]),
    Command.move_to('Move down to the place position', pose=[0.72, 0.557, 0.911, 0.0, 1.0, 0.0, 0.0]),
    Command.move_gripper(f'Release {model_1}', diameter=60),
    Command.move_to(f'Move above {model_1}', pose=[0.72, 0.557, 1.2, 0.0, 1.0, 0.0, 0.0]),

    Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    Command.move_to(f'Move above {model_2}', pose=[0.185, 0.7, 1.0, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    Command.move_to(f'Move down to {model_2}', pose=[0.185, 0.7, 0.905, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    Command.move_gripper(f'Grasp {model_2}', diameter=29),
    Command.move_to(f'Pick up {model_2}', pose=[0.185, 0.7, 1.0, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    Command.move_to('Move above the place position', pose=[0.72, 0.557, 1.1, 0.0, 1.0, 0.0, 0.0]),
    Command.move_to('Move down to the place position', pose=[0.72, 0.557, 0.942, 0.0, 1.0, 0.0, 0.0]),
    Command.move_gripper(f'Release {model_2}', diameter=60),
    Command.move_to(f'Move above {model_2}', pose=[0.72, 0.557, 1.2, 0.0, 1.0, 0.0, 0.0]),

    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to(f'Move above {model_3}', pose=[0.25, 0.6, 1.0, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move down to {model_3}', pose=[0.25, 0.6, 0.9, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_gripper(f'Grasp {model_3}', diameter=29),
    # Command.move_to(f'Pick up {model_3}', pose=[0.25, 0.6, 1.0, 0.7071055, -0.7071081, 9e-7, 9e-7]),
    # Command.move_to(f'Move to middle', pose=[0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move above the place position', pose=[0.72, 0.557, 1.1, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_to('Move down to the place position', pose=[0.72, 0.557, 0.975, 0.0, 1.0, 0.0, 0.0]),
    # Command.move_gripper(f'Release {model_3}', diameter=60),
    # Command.move_to(f'Move above {model_3}', pose=[0.72, 0.557, 1.1, 0.0, 1.0, 0.0, 0.0]),

]