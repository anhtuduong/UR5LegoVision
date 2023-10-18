# -*- coding: utf-8 -*-
"""
Created on Fri Jun 02 2019

@author: Anh Tu Duong 
"""

import numpy as np

robot_params = {}

robot_params['ur5'] ={  'dt': 0.001, 
                        'kp': np.array([300, 300, 300, 30, 30, 1]),
                        'kd':  np.array([20, 20, 20, 5, 5, 0.5]),
                        #'q_0':  np.array([ 0.3, -1.3, 1.0, -0.7, 0.7, 0.5]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                        'q_0':  np.array([-0.32067, -0.94173, -2.49191, -1.2787, -1.56982, 0.32103]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                        'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
                        'ee_frame': 'tool0',
                        'control_mode': 'point', # 'trajectory','point'
                        'real_robot': False,
                        'control_type': 'position', # 'position', 'torque'
                        'gripper_sim': True,
                        'soft_gripper': True,
                        # 'spawn_x' : 0.5,
                        # 'spawn_y' : 0.35,
                        # 'spawn_z' : 1.75,
                        'spawn_x' : 0,
                        'spawn_y' : 0,
                        'spawn_z' : 0,
                        'buffer_size': 50000, # note the frames are all aligned with base for joints = 0
                        'pose_middle': [0.456, 0.619, 1.2, 0.0, 1.0, 0.0, 0.0],
                        'pick_limit': {
                              'x': [0.0, 0.25286],
                              'y': [0.39775, 0.76],
                        },
                        'place_limit': {
                              'x': [0.6, 0.94776],
                              'y': [0.45482, 0.76],
                        }

                     }

verbose = False
plotting = True


