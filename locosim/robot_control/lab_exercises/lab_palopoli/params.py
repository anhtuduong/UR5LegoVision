# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np

robot_params = {}

robot_params['myrobot'] ={'dt': 0.001,
                        'kp': np.array([10.,   10.,    10.,  10.]),
                        'kd':  np.array([1.,    1.,    1.,   1.  ]),
                        'q_0':  np.array([0, 0, 0, 0]),
                        'joint_names': ['lf_shoulder_pan', 'lh_shoulder_pan',  'rf_shoulder_pan', 'rh_shoulder_pan'],
                        'ee_frames': ['lf_foot', 'lh_foot', 'rf_foot','rh_foot'],
                        'spawn_x': 0.0,
                        'spawn_y': 0.0,
                        'spawn_z': 1.0,
                        'buffer_size': 30001}


robot_params['tractor'] ={'dt': 0.001,
                        'kp': np.array([100.,   100.,    100.,  100.]),
                        'kd':  np.array([10.,    10.,    10.,   10.  ]),
                        'q_0':  np.array([0, 0, 0, 0]),
                        'joint_names': ['front_left_wheel_joint', 'front_right_wheel_joint',  'rear_left_wheel_joint', 'rear_right_wheel_joint'],
                        'ee_frames': ['lf_foot', 'lh_foot', 'rf_foot','rh_foot'],
                        'spawn_x': -0.5,
                        'spawn_y': 0.0,
                        'spawn_z': 1.0,
                        'buffer_size': 30001}

robot_params['ur5'] ={'dt': 0.001,
                       'kp': np.array([300, 300, 300,30,30,1]),
                       'kd':  np.array([20,20,20,5, 5,0.5]),
                       'q_0':  np.array([ -0.32,-0.78, -2.56,-1.63, -1.57, 3.49]), #limits([0,pi],   [0, -pi], [-pi/2,pi/2],)
                       'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
                       'ee_frame': 'tool0',
                       'control_mode': 'point', # 'trajectory','point'
                       'real_robot': False,
                       'control_type': 'position', # 'position', 'torque'
                       'gripper_sim': False, # False: the gripper is treated as a Rigid Body, True: you can move the finger joints
                       'soft_gripper': False, # False: 3 finger rigid gripper, True: 2 finger soft gripper
                       'spawn_x' : 0.5,
                       'spawn_y' : 0.35,
                       'spawn_z' : 1.75}

plotting = True


