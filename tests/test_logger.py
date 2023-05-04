"""!
@package tests.test_logger
@file tests/test_logger.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief This module contains the unit tests for the logger module.
@date 2023-05-04
"""

# Resolve path
from pathlib import Path
import sys
import os

FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
VISION_PATH = os.path.abspath(os.path.join(ROOT, "vision"))
if VISION_PATH not in sys.path:
    sys.path.append(VISION_PATH)  # add VISION_SCRIPTS_PATH to PATH
VISION_PATH = Path(os.path.relpath(VISION_PATH, Path.cwd()))  # relative

# Import
import unittest
from vision.scripts.utils.Logger import Logger

class TestLogger(unittest.TestCase):
    """!
    @brief This class contains the unit tests for the logger module.
    """

    def test_logger(self):
        """!
        @brief This method tests the info method.
        """
        assert Logger.info("test") == "[INFO] test"
        assert Logger.warning("test") == "[WARNING] test"
        assert Logger.error("test") == "[ERROR] test"
        assert Logger.debug("test") == "[DEBUG] test"
        