"""!
@package tests.test_logger
@file tests/test_logger.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@brief This module contains the unit tests for the logger module.
@date 2023-05-04
"""

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
        