"""!
@file Logger.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief Utility functions for logging
@date 2023-05-04
"""

# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if ROOT not in sys.path:
    sys.path.append(ROOT)  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Import
from termcolor import colored

# Global constants
LOG_PATH = os.path.abspath(os.path.join(ROOT, "vision/logs"))

# Class Logger
class Logger:
    """!
    @brief This class contains utility functions for logging.
    """
    def info(text):
        out = colored(f"[INFO]\t{text}", "green", attrs=["bold"])
        print(out)
        return out

    def debug(text):
        out = f"[DEBUG]\t{text}"
        print(out)
        return out

    def debug_highlight(text):
        out = colored(f"[DEBUG]\t{text}", "blue", attrs=["bold"])
        print(out)
        return out

    def error(text):
        out = colored(f"[ERROR]\t{text}", "red")
        print(out)
        return out

    def warning(text):
        out = colored(f"[WARNING] {text}", "yellow")
        print(out)
        return out
    
    # TODO: implement log to file