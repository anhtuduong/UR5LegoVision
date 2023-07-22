# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

#!/usr/bin/env python

#inherit from base controller
from base_controllers.base_controller import BaseController
import rospy as ros 
import numpy as np
from numpy import nan
import copy

# L5 Controller specific
from base_controllers.utils.controlRoutines import projectionBasedController, QPController
from base_controllers.utils.common_functions import plotCoM, plotGRFs, plotConstraitViolation, plotJoint
from scipy.linalg import block_diag
from base_controllers.utils.math_tools import motionVectorTransform
from base_controllers.utils.common_functions import State
from optimization.ref_generation import ReferenceGenerator

# config file
import OPT_L1_walking_conf as conf

robotName = "hyq"

des_state = State(desired = True)
act_state = State() 
initial_state = State() 

class AdvancedController(BaseController): 

    def __init__(self):  
        BaseController.__init__(self, robot_name=robotName)
        
        #send data to param server
        self.verbose = conf.verbose                                                                                                          
        self.u.putIntoGlobalParamServer("verbose", self.verbose)	
        self.refclass = ReferenceGenerator(conf)
  
        
    def initVars(self):
        BaseController.initVars(self)	

        self.des_basePoseW_log = np.empty((6, conf.buffer_size))*nan
        self.des_baseTwistW_log = np.empty((6,conf.buffer_size ))*nan        
        self.des_baseAccW_log = np.empty((6,conf.buffer_size ))*nan        
        self.des_forcesW_log = np.empty((12,conf.buffer_size ))*nan        
        self.Wffwd_log = np.empty((6, conf.buffer_size ))*nan        
        self.Wfbk_log = np.empty((6,conf.buffer_size ))*nan        
        self.Wg_log = np.empty((6,conf.buffer_size))*nan        
        self.constr_viol_log = np.empty((4,conf.buffer_size ))*nan               							


    def logData(self):
        if (self.log_counter < conf.buffer_size):
            BaseController.logData(self)
            self.des_basePoseW_log[:, self.log_counter] = self.des_pose
            self.des_baseTwistW_log[:, self.log_counter] =  self.des_twist           
            self.des_baseAccW_log[:, self.log_counter] =  self.des_acc       
            self.des_forcesW_log[:, self.log_counter] =  self.des_forcesW            
            self.Wffwd_log[:, self.log_counter] =  self.Wffwd            
            self.Wfbk_log[:, self.log_counter] =  self.Wfbk            
            self.Wg_log[:, self.log_counter] =  self.Wg            
            self.constr_viol_log[:, self.log_counter] =  self.constr_viol        
        

def talker(p):
    
    p.start()
    p.initVars()          
    p.startupProcedure() 
    rate = ros.Rate(1/conf.dt) # 10hz
    
                                
    # Reset reference to actual value  
    p.x0 = copy.deepcopy(p.basePoseW)
    p.des_pose  = p.x0
    p.des_twist = np.zeros(6)
    p.des_acc = np.zeros(6)  
     
    #initial_state.set(act_state)
#    p.refclass.getReferenceData(initial_state, conf.desired_velocity,  p.W_contacts,  np.logical_not(p.stance_legs), conf.robot_height)
#    print(conf.desired_velocity.lin_x)
#    import time
#    time.sleep(3)    
    # Control loop               
    while (p.time  < conf.exp_duration) or conf.CONTINUOUS:
        #update the kinematics
        p.updateKinematics()
     
        des_state.pose.set(p.des_pose)
        des_state.twist.set(p.des_twist)
        des_state.accel.set(p.des_acc)         
        act_state.pose.set(p.basePoseW)
        act_state.twist.set(p.baseTwistW)  
                                                       
                              
        # set robot specific params                             
        conf.params.robot = p.robot                      
        conf.params.W_base_to_com = p.u.linPart(p.comPoseW)   -   p.u.linPart(p.basePoseW)         
        conf.params.robotInertiaB = p.compositeRobotInertiaB        
     
        #QP controller
        p.des_forcesW, p.Wffwd, p.Wfbk, p.Wg, p.constr_viol =  QPController(conf.control_params[p.robot_name], act_state, des_state, p.W_contacts, p.stance_legs, conf.params)

        # map desired contact forces into torques                                     
        p.jacsT = block_diag(np.transpose(p.wJ[p.u.leg_map["LF"]]), 
                        np.transpose(p.wJ[p.u.leg_map["RF"]] ), 
                        np.transpose(p.wJ[p.u.leg_map["LH"]] ), 
                        np.transpose(p.wJ[p.u.leg_map["RH"]]  ))
        p.tau_ffwd =   p.u.mapFromRos(p.h_joints) - p.jacsT.dot(p.des_forcesW)         
 
        # send desired command to the ros controller     
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
        p.logData()    
        p.time = p.time + conf.dt 
        # plot actual (green) and desired (blue) contact forces 
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.grForcesW/400),"green")        
            p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.des_forcesW/400),"blue")        
        p.ros_pub.publishVisual()                        
                                
        #wait for synconization of the control loop
        rate.sleep()       
                # stops the while loop if  you prematurely hit CTRL+C                    
        if ros.is_shutdown():
            print ("Shutting Down")                    
            break;                                                
                             
    # restore PD when finished        
    p.pid.setPDs(400.0, 6.0, 0.0) 
    
    
    class active_plots:
        pass
    active_plots.swing = False
    active_plots.feet = True
    active_plots.com = True
    active_plots.orientation = False
    active_plots.grforces = False   
    #TODO    
    #p.refclass.plotReference(active_plots)
    
    ros.sleep(1.0)                
    print ("Shutting Down")                 
    ros.signal_shutdown("killed")           
    p.deregister_node()        
    
    #plotCoM('position', 0, p.time_log, p.des_basePoseW_log, p.basePoseW_log, p.des_baseTwistW_log, p.baseTwistW_log, p.des_baseAccW_log, p.Wffwd_log  + p.Wfbk_log + p.Wg_log             )
    #plotCoM('wrench', 1, p.time_log, p.des_basePoseW_log, p.basePoseW_log, p.des_baseTwistW_log, p.baseTwistW_log, p.des_baseAccW_log, p.Wffwd_log  + p.Wfbk_log + p.Wg_log             )
    #plotGRFs(2, p.time_log, p.des_forcesW_log, p.grForcesW_log)
    #plotConstraitViolation(3,p.constr_viol_log)            
    #plotJoint('torque',4, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log)
         
if __name__ == '__main__':
    p = AdvancedController()
    try:
        talker(p)
    except ros.ROSInterruptException:
        pass
    
        