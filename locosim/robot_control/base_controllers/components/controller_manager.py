
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[4]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import rospkg
import numpy as np
import rospy as ros
from locosim.robot_control.base_controllers.components.gripper_manager import GripperManager
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from termcolor import colored


class ControllerManager():
    def __init__(self, conf):
        self.conf = conf
        self.control_type = conf['control_type']
        self.gripper_sim = conf['gripper_sim']
        self.real_robot = conf['real_robot']
        self.number_of_joints = len(conf['joint_names'])

        if  (self.control_type == 'torque'):
            print(colored("Controller Manager: torque", "blue"))
        if (self.control_type == 'position'):
            print(colored("Controller Manager: position", "blue"))

    def initPublishers(self, robot_name):
        # publisher for ros_impedance_controller
        self.pub_full_jstate = ros.Publisher("/command", JointState, queue_size=1, tcp_nodelay=True)
        # specific publisher for joint_group_pos_controller that publishes only position
        self.pub_reduced_des_jstate = ros.Publisher("/" + robot_name + "/joint_group_pos_controller/command",
                                                    Float64MultiArray, queue_size=10)
        # instantiate the gripper manager that will read soft gripper param from param server
        self.gm = GripperManager(self.real_robot, self.conf['dt'])



    def send_full_jstate(self, q_des, qd_des, tau_ffwd):
         # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
         msg = JointState()
         if self.gripper_sim:
             msg.position = np.append(q_des, self.gm.getDesGripperJoints())
             msg.velocity = np.append(qd_des, np.array([0., 0., 0.]))
             msg.effort = np.append(tau_ffwd, np.array([0., 0., 0.]))
         else:
             msg.position = q_des
             msg.velocity = qd_des
             msg.effort = tau_ffwd
         self.pub_full_jstate.publish(msg)

    def send_reduced_des_jstate(self, q_des):
        msg = Float64MultiArray()
        if  self.gripper_sim and not self.real_robot:
            msg.data = np.append(q_des, self.gm.getDesGripperJoints())
        else:
            msg.data = q_des
        self.pub_reduced_des_jstate.publish(msg)

    def sendReference(self, q_des, qd_des = None, tau_ffwd = None):
        if (self.control_type == 'torque'):
            if qd_des is None:
                qd_des = np.zeros(self.number_of_joints)
            if tau_ffwd is None:
                tau_ffwd = np.zeros(self.number_of_joints)
            self.send_full_jstate(q_des, qd_des, tau_ffwd)
        else:
            self.send_reduced_des_jstate(q_des)




