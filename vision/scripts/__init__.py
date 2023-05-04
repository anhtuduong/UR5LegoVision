from pathlib import Path
import os
import sys

# Resolve paths
FILE = Path(__file__).resolve()
ROOT = FILE.parents[2]
VISION_PATH = os.path.abspath(os.path.join(ROOT, "vision"))
if VISION_PATH not in sys.path:
    sys.path.append(VISION_PATH)  # add VISION_SCRIPTS_PATH to PATH