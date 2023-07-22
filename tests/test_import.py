"""!
@package tests.test_import
@file tests/test_import.py
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

class TestImport(unittest.TestCase):
    """!
    @brief This class contains the unit tests for the imports.
    """

    def test_import_motion(self):
        """!
        @brief This method tests the import of motions package.
        """
        from motion.test import test_root as test_root_motion
        assert test_root_motion() == ROOT
        
    def test_import_vision(self):
        """!
        @brief This method tests the import of vision package.
        """
        from vision.scripts.test import test_root as test_root_vision
        assert test_root_vision() == ROOT

    def test_import_utils(self):
        """!
        @brief This method tests the import of utils_ur5 package.
        """
        from utils_ur5.test import test_root as test_root_utils
        assert test_root_utils() == ROOT


if __name__ == '__main__':
    unittest.main()