
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

# Import
import random
import locosim.robot_control.base_controllers.params as conf

pick_limit = conf.robot_params['ur5']['pick_limit']
place_limit = conf.robot_params['ur5']['place_limit']

# Generate random lego positions
x_pick = random.uniform(pick_limit['x'][0], pick_limit['x'][1])
y_pick = random.uniform(pick_limit['y'][0], pick_limit['y'][1])
x_place = random.uniform(place_limit['x'][0], place_limit['x'][1])
y_place = random.uniform(place_limit['y'][0], place_limit['y'][1])

print(f'x_pick: {x_pick:.4f}')
print(f'y_pick: {y_pick:.4f}')
print(f'x_place: {x_place:.4f}')
print(f'y_place: {y_place:.4f}')