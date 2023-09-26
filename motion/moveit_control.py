"""!
@package motion.moveit_control
@file motion/moveit_control.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-25

@brief Defines the MoveitControl class that uses Moveit to plan trajectories
"""

# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

import rospy as ros
import moveit_commander
import geometry_msgs.msg
from moveit_msgs.msg import RobotTrajectory

import numpy as np
from utils_ur5.Logger import Logger as log

# ---------------------- CLASS ----------------------

class MoveitControl():
    """
    The class that uses Moveit to plan trajectories
    """
    def __init__(self):
        """
        Constructor
        """
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "ur5_arms"  # Or whichever group you defined in your SRDF
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        log.info(f"MoveitControl initiated. Group name: {self.group_name}")

    def get_trajectory(self, target):
        """
        Plans a trajectory to the target
        :param target: target pose or joint states, ``geometry_msgs.msg.Pose`` or ``tuple``
        :return: joint trajectory, ``moveit_msgs.msg.RobotTrajectory``
        """
        log.debug_highlight(f'Start planning trajectory')

        if isinstance(target, geometry_msgs.msg.Pose):
            # Set the pose target
            self.group.set_pose_target(target)
            log.debug(f"Pose target:\n{target}")
        elif isinstance(target, tuple):
            # Set the joint values as the target
            target = np.array(target)
            self.group.set_joint_value_target(target)
            log.debug(f"Joint states:\n{target}")
        else:
            log.error(f"Target type {type(target)} not supported")
            return None

        # Plan
        success, plan, planning_time, _ = self.group.plan()

        if success:
            joint_trajectory = plan.joint_trajectory
            
            log.info(f'PLANNING SUCCESSFUL in {planning_time} seconds')
            return joint_trajectory
        else:
            log.error(f'PLANNING FAILED in {planning_time} seconds')
            return None
