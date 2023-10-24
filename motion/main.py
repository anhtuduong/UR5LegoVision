"""!
@file motion/main.py
@author Anh Tu Duong (anhtu.duong@studenti.unitn.it)
@date 2023-05-09

@brief Defines the Motion node that executes the motion plan
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
import rospy as ros
import locosim.robot_control.base_controllers.params as conf
import numpy as np
from motion.utils import *
from motion.motion_planner import MotionPlanner
from motion.gazebo_command import GazeboCommand
from utils_ur5.Logger import Logger as log

# Ros msg and srv
from geometry_msgs.msg import Pose
from ros_impedance_controller.srv import MoveJoints, MoveJointsRequest, MoveJointsResponse
from ros_impedance_controller.srv import MoveTo, MoveToRequest, MoveToResponse
from ros_impedance_controller.srv import generic_float
from sensor_msgs.msg import JointState

# ---------------------- CLASS ----------------------
class Motion():
    """
    The class that starts the motion node
    """

    def __init__(self):
        """
        Constructor
        """
        # Start motion node
        ros.init_node('motion_node', anonymous=True)

        # Init variables
        self.robot_name = 'ur5'
        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        self.dt = conf.robot_params[self.robot_name]['dt']
        self.rate = ros.Rate(1 / self.dt)
        self.real_robot = conf.robot_params[self.robot_name]['real_robot']
        if self.real_robot:
            self.v_des = 0.2
        else:
            self.v_des = 0.6

        self.q = np.zeros(len(self.joint_names))
        self.qd = np.zeros(len(self.joint_names))
        self.tau = np.zeros(len(self.joint_names))

        # Ros subscribers
        self.joint_states_sub = ros.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.ee_pose_sub = ros.Subscriber('/ur5/ee_pose', Pose, self.ee_pose_callback)

        # Ros service clients
        ros.wait_for_service('/ur5/move_joints')
        self.move_joints_srv = ros.ServiceProxy('/ur5/move_joints', MoveJoints)
        ros.wait_for_service('/ur5/move_to')
        self.move_to_srv = ros.ServiceProxy('/ur5/move_to', MoveTo)
        ros.wait_for_service('move_gripper')
        self.move_gripper_srv = ros.ServiceProxy('/ur5/move_gripper', generic_float)
        
        self.gazebo_command = GazeboCommand()

    def joint_states_callback(self, msg):
        """
        Callback function for the joint states subscriber
        :param msg: The message received from the subscriber, ``JointState``
        """
        for msg_idx in range(len(msg.name)):          
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]: 
                    self.q[joint_idx] = msg.position[msg_idx]
                    self.qd[joint_idx] = msg.velocity[msg_idx]
                    self.tau[joint_idx] = msg.effort[msg_idx]

    def ee_pose_callback(self, msg):
        """
        Callback function for the end-effector pose subscriber
        :param msg: The message received from the subscriber, ``Pose``
        """
        self.ee_pose = msg

    def move_joints(self, q_des, text = ''):
        """
        Moves the robot to the desired joint configuration
        :param q_des: The desired joint configuration, ``list``
        :param text: The description of the movement, ``str``
        """
        # Create a request object for MoveJoints
        req = MoveJointsRequest()
        req.q_des = q_des
        req.dt = self.dt
        req.v_des = self.v_des

        # Call the service
        log.debug(f'Start movement! {text}')
        res = self.move_joints_srv(req)

        if res.success:
            log.info(f'Movement succeeded! {text}')
        else:
            log.error(f'Movement failed! {text}')
        return res
        
    def move_to(self, pose_target, text = ''):
        """
        Moves the robot to the desired pose
        :param pose_target: The desired pose, ``Pose`` or ``list``
        :param text: The description of the movement, ``str``
        """
        if isinstance(pose_target, Pose):
            pose_target = Pose_to_list(pose_target)
        elif isinstance(pose_target, list):
            pass
        else:
            raise TypeError('pose_target must be a Pose or a list')
        
        # Create a request object for MoveTo
        req = MoveToRequest()
        req.pose_target = pose_target
        req.dt = self.dt
        req.v_des = self.v_des

        # Call the service
        log.debug(f'Start movement! {text}')
        res = self.move_to_srv(req)

        if res.success:
            log.info(f'Movement succeeded! {text}')
        else:
            log.error(f'Movement failed {text}')
        return res

    def move_gripper(self, diameter, text = ''):
        """
        Opens or closes the gripper to the desired diameter
        :param diameter: The desired diameter, ``float``
        :param text: The description of the movement, ``str``
        """
        log.debug(f'Start gripper')
        # Create a request object
        request = generic_float._request_class()
        request.data = diameter

        # Send the request to the service
        res = self.move_gripper_srv(request)

        if res:
            log.info(f'Gripper command sent: {diameter}mm. {text}')
        else:
            log.error(f'Gripper command failed: {diameter}mm. {text}')

    

    def run(self, planner:MotionPlanner):
        """
        Executes the actions in the motion plan
        :param planner: The motion planner, ``MotionPlanner``
        """
        for action in planner.action_list:
            if action['type'] == 'move to':
                self.move_to(action['pose'], action['description'])
            elif action['type'] == 'move joints':
                self.move_joints(action['joints'], action['description'])
            elif action['type'] == 'move gripper':
                self.move_gripper(action['diameter'], action['description'])
            elif action['type'] == 'attach models':
                self.gazebo_command.attach_models(action['model_name_1'], action['link_name_1'], action['model_name_2'], action['link_name_2'])
            elif action['type'] == 'detach models':
                self.gazebo_command.dettach_models(action['model_name_1'], action['link_name_1'], action['model_name_2'], action['link_name_2'])

        

# ----------- MAIN ----------- #
if __name__ == '__main__':
    motion = Motion()
    planner = MotionPlanner()
    motion.run(planner)