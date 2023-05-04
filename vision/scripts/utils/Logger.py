"""!
@file Logger.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief Utility functions for logging
@date 2023-05-04
"""

# Resolve paths
from pathlib import Path
import os
import sys

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
VISION_SCRIPTS_PATH = os.path.abspath(os.path.join(ROOT, ".."))

# Class Logger
class Logger:

    def info(text):
        print("[INFO] " + text)
        return "[INFO] " + text

    def warning(text):
        print("[WARNING] " + text)
        return "[WARNING] " + text

    def error(text):
        print("[ERROR] " + text)
        return "[ERROR] " + text
    
    def debug(text):
        print("[DEBUG] " + text)
        return "[DEBUG] " + text
