# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: mfocchi
"""

from __future__ import print_function
import rospy as ros
from base_controllers.utils.math_tools import *
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import matplotlib.pyplot as plt
import rospkg
from base_controllers.base_controller import BaseController
from base_controllers.base_controller_fixed import BaseControllerFixed
from base_controllers.utils.common_functions import plotFrame, plotJoint

import params as conf
# robotName = "myrobot" # needs to inherit BaseController
robotName = "ur5"  # needs to inherit BaseControllerFixed


class GenericSimulator(BaseController):
    
    def __init__(self, robot_name="myrobot"):
        super().__init__(robot_name=robot_name, external_conf = conf)
        self.freezeBaseFlag = False
        print("Initialized murobot controller---------------------------------------------------------------")

    def initVars(self):
        super().initVars()
        ## add your variables to initialize here
        self.q_des_q0 = conf.robot_params[self.robot_name]['q_0']

    def logData(self):
            if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
                ## add your logs here
                pass
            super().logData()

def talker(p):
    p.start()
    additional_args = None #'gui:=false'
    p.startSimulator(additional_args = additional_args)
    #xacro_path = rospkg.RosPack().get_path('ur_description') + '/urdf/' + p.robot_name + '.urdf.xacro'
    #p.loadModelAndPublishers(xacro_path)
    p.loadModelAndPublishers()
    p.initVars()
    p.initSubscribers()
    p.startupProcedure()
    #loop frequency
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt'])

    p.q_des = np.copy(p.q_des_q0)

    while not ros.is_shutdown():
        p.tau_ffwd = np.zeros(p.robot.na)
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)

        # log variables
        p.logData()

        # wait for synconization of the control loop
        rate.sleep()
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

if __name__ == '__main__':
    p = GenericSimulator(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        plotJoint('position', 0, p.time_log, p.q_log, p.q_des_log, joint_names = p.joint_names)


