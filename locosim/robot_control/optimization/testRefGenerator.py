# -*- coding: utf-8 -*-
"""
Created on Mon Oct 11 10:11:28 2021

@author: mfocchi
"""
import matplotlib.pyplot as plt

from ref_generation import ReferenceGenerator
from base_controller.utils.common_functions import  State
from base_controller.utils.utils import Utils
import os
import numpy as np

class desired_velocity():
    pass
class active_plots:
        pass
class config:
        pass

if __name__ == "__main__":
    u = Utils()
    
    # config
    config.prediction_horizon= 600
    config.cycle_time = 8.0
    config.time_resolution = 0.04
    config.gait_type = 0
    config.duty_factor = 0.85
    config.offset_a = 0.05
    config.offset_b = 0.3
    config.offset_c = 0.55
    config.offset_d = 0.8
    config.robot_name = 'hyq'
    config.robot_mass = 85
    config.robot_height = 0.555
    config.foot_hip_y_offset = 0.15
    config.verbose = 0 # 1 /2 
      
    # startRefGenNode
    refclass = ReferenceGenerator(config)
    
    # input to reference generator
    initial_state = State() 
    # intial com position (Z noot needed cause we use robot height)
    initial_state.pose.position = np.array([0.0, 0.0, 0.0])
    # intial com orientation
    initial_state.pose.orientation = np.array([0.0, 0.0, 0.0])
    # intial com velocity (only useful for dist rejection)
    initial_state.twist.linear = np.array([0.0, 0.0, 0.0])
    
    #desired velocity
    desired_velocity.lin_x = 0.05
    desired_velocity.lin_y = 0.0
    desired_velocity.ang_z = 0.0
    
    # stance status
    swing_status = [np.nan]*4
    swing_status[u.leg_map.get("LF")] = False
    swing_status[u.leg_map.get("RF")] = False
    swing_status[u.leg_map.get("LH")] = False
    swing_status[u.leg_map.get("RH")] = False
    
    #initial feet position in WF
    initial_feet =  [np.zeros((3))]*4
    initial_feet[u.leg_map.get("LF")] =  np.array([0.3, 0.3, 0.0])
    initial_feet[u.leg_map.get("RF")] =  np.array([0.3, -0.3, 0.0])
    initial_feet[u.leg_map.get("LH")] =  np.array([-0.3, 0.3, 0.0])
    initial_feet[u.leg_map.get("RH")] =  np.array([-0.3, -0.3, 0.0])
    

    refclass.getReferenceData(initial_state, desired_velocity,  initial_feet,  swing_status, config.robot_height)

    active_plots.swing = False
    active_plots.feet = True
    active_plots.com = True
    active_plots.orientation = False
    active_plots.grforces = False    
    
    refclass.plotReference(active_plots)

    plt.show(block=True)
