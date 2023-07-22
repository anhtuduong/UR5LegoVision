"""!
@package tests.test_logger
@file tests/test_logger.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief This module contains the unit tests for the logger module.
@date 2023-05-04
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
import unittest
from termcolor import colored
from utils_ur5.src.Logger import Logger

class TestLogger(unittest.TestCase):
    """!
    @brief This class contains the unit tests for the logger module.
    """

    def test_logger(self):
        """!
        @brief This method tests the info method.
        """
        text = "test"
        assert Logger.info(text) == colored(f"[INFO]\t{text}", "green", attrs=["bold"])
        assert Logger.warning(text) == colored(f"[WARNING]\t{text}", "yellow")
        assert Logger.error(text) == colored(f"[ERROR]\t{text}", "red")
        assert Logger.debug(text) == "[DEBUG]\ttest"

if __name__ == '__main__':
    unittest.main()
        