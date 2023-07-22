
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[3]
if ROOT not in sys.path:
    sys.path.append(ROOT)  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# Import
import time
from utils_ur5.Logger import Logger as log

class TimeExecution:
    """ @brief Time execution of functions
    """
    def __init__(self):
        """ @brief Initialize the TimeExecution class
        """
        self.start = time.time()

    def get_duration(self):
        """ @brief End the time execution
        """
        self.time_executed = time.time() - self.start
        return self.time_executed
    

    