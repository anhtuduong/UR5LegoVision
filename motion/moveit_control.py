
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
# from moveit_commander import FKUtils
import geometry_msgs.msg
from moveit_msgs.msg import Constraints, OrientationConstraint

import numpy as np
from utils_ur5.Logger import Logger as log

class MoveitControl():
    """
    """
    def __init__(self):

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "ur5_arms"  # Or whichever group you defined in your SRDF
        self.group = moveit_commander.MoveGroupCommander(self.group_name)
        log.info(f"MoveitControl initiated. Group name: {self.group_name}")

    def get_trajectory(self, target):
        """
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

        # Apply orientation constraint
        # self.apply_orientation_constraint()

        # Plan
        success, plan, planning_time, _ = self.group.plan()

        if success:
            joint_trajectory = plan.joint_trajectory
            log.info(f'PLANNING SUCCESSFUL in {planning_time} seconds')
            return joint_trajectory
        else:
            log.error(f'PLANNING FAILED in {planning_time} seconds')
            return None

    def apply_orientation_constraint(self):
        """
        """
        # Define the orientation constraint
        orientation_constraint = Constraints()
        orientation_constraint.orientation_constraints.append(OrientationConstraint())
        orientation_constraint.orientation_constraints[0].link_name = self.group.get_end_effector_link()
        orientation_constraint.orientation_constraints[0].header.frame_id = self.group.get_planning_frame()
        orientation_constraint.orientation_constraints[0].orientation.x = 0.0
        orientation_constraint.orientation_constraints[0].orientation.y = 1.0
        orientation_constraint.orientation_constraints[0].orientation.z = 0.0
        orientation_constraint.orientation_constraints[0].orientation.w = 0.0
        orientation_constraint.orientation_constraints[0].absolute_x_axis_tolerance = 0.1
        orientation_constraint.orientation_constraints[0].absolute_y_axis_tolerance = 0.1
        orientation_constraint.orientation_constraints[0].absolute_z_axis_tolerance = 0.1
        orientation_constraint.orientation_constraints[0].weight = 0.1

        # Apply the orientation constraint
        self.group.set_path_constraints(orientation_constraint)
