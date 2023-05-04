"""!
@file Logger.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief Utility functions for logging
@date 2023-05-04
"""

from pathlib import Path
import os
import sys
import datetime

# Resolve paths
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
VISION_PATH = os.path.abspath(os.path.join(ROOT, "vision"))
if VISION_PATH not in sys.path:
    sys.path.append(VISION_PATH)  # add VISION_SCRIPTS_PATH to PATH
LOG_PATH = os.path.abspath(os.path.join(ROOT, "vision/logs"))

# Class Logger
class Logger:
    """!
    @brief This class contains utility functions for logging.
    """
    def info(text):
        print("[INFO]" + text)
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
    
    # TODO: implement log to file