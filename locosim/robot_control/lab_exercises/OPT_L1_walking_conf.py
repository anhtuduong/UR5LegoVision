# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os
from base_controllers.utils.utils import Utils

LINE_WIDTH = 60

dt = 0.004  # controller time step
exp_duration = 3.0 #simulation duration
CONTINUOUS = False
verbose = False

# Initial configuration / velocity / Acceleration
q0  = np.matrix([ 0.0, -0.6, 0.6, -1.67, -1.57, 0.0]).T
qd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T
qdd0 = np.matrix([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]).T



# CONTROLLER PARAMS
buffer_size = 30001

# Gains for the virtual model
control_params = {}
control_params['hyq'] = {'Kp_lin_x': 2000, 'Kp_lin_y': 2000, 'Kp_lin_z': 2000,    
                       'Kd_lin_x': 200, 'Kd_lin_y': 200, 'Kd_lin_z': 200,                       
                       'KpRoll': 1000, 'KpPitch': 1000, 'KpYaw': 1000, 
                       'KdRoll': 100, 'KdPitch': 100, 'KdYaw': 100, 'gravity': 9.81}


control_params['solo'] = {'Kp_lin_x': 200, 'Kp_lin_y': 200, 'Kp_lin_z': 200,    
                       'Kd_lin_x': 20, 'Kd_lin_y': 20, 'Kd_lin_z': 20,                       
                       'KpRoll': 10, 'KpPitch': 10, 'KpYaw': 10, 
                       'KdRoll': 1, 'KdPitch': 1, 'KdYaw': 1, 'gravity': 9.81}
    
class params:
    pass 

utils= Utils()

# Terrain parameters
params.normals = [None]*4                 
params.normals[utils.leg_map["LF"]] = np.array([0.0,0.0,1.0])
params.normals[utils.leg_map["RF"]] = np.array([0.0,0.0,1.0])
params.normals[utils.leg_map["LH"]] = np.array([0.0,0.0,1.0])
params.normals[utils.leg_map["RH"]] = np.array([0.0,0.0,1.0])    
params.friction_coeff = np.array([0.6,0.6,0.6, 0.6])    
   
# Trunk controller optios   
params.isCoMControlled = True 
params.gravityComp = True
params.ffwdOn = True    
params.frictionCones = True
params.f_min = np.array([0.0,0.0,0.0, 0.0])    

# PLANNER PARAMS
class desired_velocity():
    pass
desired_velocity.lin_x = 0.05
desired_velocity.lin_y = 0.0
desired_velocity.ang_z = 0.0

prediction_horizon= 600
cycle_time = 8.0
time_resolution = 0.04
gait_type = 0 # 0: crawl 1: pace 2: trot 
duty_factor = 0.85
offset_a = 0.05
offset_b = 0.3
offset_c = 0.55
offset_d = 0.8
robot_name = 'hyq'
robot_mass = 85
robot_height = 0.555
foot_hip_y_offset = 0.15
verbose = 0 # 1 /2 
      


# configuration for LIPM trajectory optimization
# ----------------------------------------------
#alpha       = 10**(2)   # CoP error squared cost weight
#beta        = 0         # CoM position error squared cost weight
#gamma       = 10**(-1)  # CoM velocity error squared cost weight
#h           = 0.58      # fixed CoM height
#g           = 9.81      # norm of the gravity vector
#foot_step_0   = np.array([0.0, -0.096])   # initial foot step position in x-y
#dt_mpc                = 0.2               # sampling time interval
#T_step                = 0.8               # time needed for every step
#step_length           = 0.2              # fixed step length 
#step_height           = 0.05              # fixed step height
#nb_steps              = 6                 # number of desired walking steps