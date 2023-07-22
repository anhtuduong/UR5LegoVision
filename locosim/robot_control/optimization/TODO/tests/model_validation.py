#% Description
# Model validation of Continuous/Discrete-time nonlinear and LTV model
# The model has been changed to affine removing the gravity states
# Author: Niraj Rathod
# Date: 19/11/2019

import sys
import os
#print(sys.version_info)
if sys.version_info[:2] == (2, 7):
    print ("USING python 2.7")
else:
    sys.path = ['/usr/lib/python3.5/lib-dynload',  # package mmap
                '/usr/lib/python3.5/',  # numbers.py is here
                '/usr/local/lib/python3.5/dist-packages/',  # sudo pip3 install installs here numpy
                '/usr/lib/python3/dist-packages/',  # sudo pip3 install installs here yaml
                '/opt/ros/kinetic/lib/python2.7/dist-packages/',  # rospy genpy
                '/usr/lib/python2.7/dist-packages/',  # rospkg is here
                os.environ['HOME'] + '/' + os.environ['ROS_WORKSPACE_NAME'] + '/install/lib/python2.7/dist-packages',
                '../.']  # this is the current directory

import scipy.io
import scipy.sparse as sparse
import numpy as np
from refgen.robotDataFile import extractROSdata
from model.model_simulation import SimulationClass



data = scipy.io.loadmat("../../matlab/DataFiles/SortValidData40ms18Sept19.mat")
rosData = extractROSdata(data)

Ts = 0.04
simCount = 2
simClass = SimulationClass(Ts)

# Define simulation start point from the data
simStart = 0  # Remember in python the counter starts from zero not 1!

x_init = np.array([rosData.actual_CoMXW[0, simStart],
                             rosData.actual_CoMYW[0, simStart],
                             rosData.actual_CoMZW[0, simStart],
                             rosData.com_VxW[0, simStart],
                             rosData.com_VyW[0, simStart],
                             rosData.com_VzW[0, simStart],
                             rosData.rollW[0, simStart],
                             rosData.pitchW[0, simStart],
                             rosData.yawW[0, simStart],
                             rosData.omegaXW[0, simStart],
                             rosData.omegaYW[0, simStart],
                             rosData.omegaZW[0, simStart]])
# Flag to turn on simple PD control
PDctrlOn = 0

# Flag to turn on Virtual PD
VirtualPDon = 1

# Flag to run LTV simulation
LTVModOn = 0

# Simulation steps
simEnd=1

xp = np.zeros((12,simEnd+1))
xp[:, 0] = x_init

# Instantiate the linearization class
# LinearizeModel = RunLinearization(nx, nu, Ts, robotInertia, robotMass)
# Run discrete time nonlinear simulation
for i in range(simEnd):
    xp[:, i+1] = simClass.runDiscreteTimeSim(rosData, i, xp[:, i], PDctrlOn, VirtualPDon, LTVModOn)



