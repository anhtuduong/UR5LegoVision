
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  2 16:52:08 2018

@author: rorsolino
"""

#!/usr/bin/env python


#to be compatible with python3
from __future__ import print_function
import sys
import os
print()
current_dir = os.path.dirname(os.path.realpath(__file__))
#print(sys.version_info)

#export  needed for t_renderer
os.environ['ACADOS_SOURCE_DIR'] = os.environ['HOME'] + '/' + os.environ['ROS_WORKSPACE_NAME'] + '/src/dls-distro/acados'

if sys.version_info[:2] == (2, 7):
    print ("USING python 2.7")
else:
    sys.path =   [ '/usr/lib/python3.5/lib-dynload', #package mmap
                 '/usr/lib/python3.5/', #numbers.py is here
                '/usr/local/lib/python3.5/dist-packages/', #sudo pip3 install installs here numpy
                 '/usr/lib/python3/dist-packages/' , #sudo pip3 install installs here yaml
                 '/opt/ros/kinetic/lib/python2.7/dist-packages/', #rospy genpy
                 '/usr/lib/python2.7/dist-packages/',#rospkg is here
                   os.environ['HOME'] + '/' + os.environ['ROS_WORKSPACE_NAME'] + '/install/lib/python2.7/dist-packages',#where reference generator and ros ipedance controlelr messages are
                 os.environ['ACADOS_SOURCE_DIR'],
                 current_dir ]#this is the current directory

#print (sys.path)
# Standard packages
import numpy as np
import rospy as ros
from std_srvs.srv    import Empty, EmptyRequest

# Custom packages
from refgen.ref_generation import ReferenceGenerator
from opti.optimizer import ConstructOptimizer
from hyq_kinematics.hyq_kinematics import HyQKinematics
from tools.plottingFeatures import *
from control.controlThread import *
from tools.getConfig import getConfig

#important
np.set_printoptions(precision = 3, linewidth = 200, suppress = True)
#dont trukate printing of matrices!
np.set_printoptions(threshold=np.inf)
#prevent creating pyc files
sys.dont_write_bytecode = True

# Setting this will stop python from printing any errors. Makes it difficult to debug
#stderr = sys.stderr
#sys.stderr = open(os.devnull, 'w')

def talker(p):

    name = "Python Controller"
    p.start()

    #load configs
    # get optimization params
    optiConfig = getConfig()
    p.loadConfig(optiConfig)
    p.register_nodes(optiConfig)

    # create the objects
    optimizer = ConstructOptimizer()
    p.kin = HyQKinematics()
    p.refclass = ReferenceGenerator()

    optimizer.load_config(optiConfig)
    p.refclass.loadConfig()
    rate = ros.Rate(1/p.control_resolution)

    p.startupProcedure(optiConfig)
    #init the kinematics (homogeneous and jacobians for feet position I guess)
    p.initKinematics(p.kin)
    #update the kinematics
    p.updateKinematics(p.kin)
    p.initVars(optimizer.nu)

    # create reference and linearizes
    p.getNewReference(optimizer, p.refclass, p.solver, 0)
    # plotReference(p.refclass)
    p.ref_states, p.ref_forces, p.foot_positionW, p.stance = optimizer.extract_ref_traj(p.refclass)
    # generate Acados OCP object if it is selected as solver
    if p.solver ==3:
        optimizer.generate_acados_ocp(p.initialCom, p.initialOrient, p.initialLinVel, p.initialAngVel)

    #reducing joint pdi gains
    p.pid.setPDs(400.0, 6.0, 0.0)

    print("Starting MPC")


    if p.replanning_on:
        start_time = ros.get_time()
        while True:
            p.counter_control_loop += 1

            ###################################
            #loop at control frequency (4ms)
            ###################################

            if (p.counter_control_loop > p.control_planning_freq_ratio) or p.firstTime:

                ###################################
                # frequency (40ms)
                ###################################
                p.counter_control_loop = 0
                p.updateKinematics(p.kin)
                # replan optimizer.N samples every replanning_window samples
                remainder = divmod(p.counter_inside_reference_window, p.replanning_window)[1]
                if (remainder == 0): #start of replanning event
                    p.pause_physics_client(EmptyRequest())
                    #we get a new reference everytime we optimize
                    p.getNewReference(optimizer, p.refclass, p.solver, p.counter_inside_reference_window) #will consider the actual leg and elapsed time in swing
                    # plotReference(p.refclass)
                    p.ref_states, p.ref_forces, p.foot_positionW, p.stance = optimizer.extract_ref_traj(p.refclass)
                    # we will update the step length with the new reference
                    p.computeStepLength(p.current_swing, p.foot_positionW)

                    p.counter_inside_reference_window = 0
                    p.startIndex = 0


                    p.replanning_times += 1
                    p.replanning_flag = not p.replanning_flag
                    print("################################################################################################")
                    print("replanning_times :", p.replanning_times, "[  range", p.startIndex, " to ",
                          p.startIndex + optimizer.N,'  ]')
                    print("################################################################################################")
                    optimizer_states, optimizer_forces, des_force_dot = p.computeOptimization(optimizer, p.startIndex, p.startIndex + optimizer.N, p.firstTime)

                    p.unpause_physics_client(EmptyRequest())

                    p.counter_inside_replanned_window = 0
                    p.startIndex += p.replanning_window

                # extract data
                des_state = optimizer_states[:, p.counter_inside_replanned_window + 1]
                des_forces = optimizer_forces[:, p.counter_inside_replanned_window]


                # log optimization data
                p.logData(p.counter_inside_reference_window, optimizer_states[:, p.counter_inside_replanned_window], des_forces, des_force_dot)

                # update counters
                p.counter_inside_reference_window += 1
                p.counter_inside_replanned_window += 1

                p.firstTime = False

                #update swing count
                if (any(p.swinging_legs)):
                    p.swing_counter += 1  # NB a trot is not possible with this

            ###################################
            #loop at control frequency (4ms)
            ###################################
            # compute q_des and qd_des from feet com traj
            p.computeJointVariables(p, des_state, p.foot_positionW, p.counter_inside_reference_window - 1) #I put -1 cause it was increamented already
            p.computeControl(p, des_state, des_forces)

            #wait for synconization
            rate.sleep()

            if (ros.get_time() - start_time) > optiConfig.experiment_duration:
            #if (p.replanning_times > p.number_of_replans) or ros.is_shutdown():  # stops the while loop if  you prematurely hit CTRL+C

                plotResult(p, optiConfig.active_plots)
                break


    else:

        # first optimize (stop physics cause you have low PD gains)
        p.pause_physics_client(EmptyRequest())
        optimizer_states, optimizer_forces, des_force_dot = p.computeOptimization(optimizer, p.startIndex, p.startIndex + optimizer.N, p.firstTime)
        p.unpause_physics_client(EmptyRequest())

        while p.counter_inside_reference_window < p.prediction_horizon:
            p.counter_control_loop +=1
            if (p.counter_control_loop > p.control_planning_freq_ratio) or p.firstTime:
                ###############################################
                # extract data at planning frequency (40ms)
                ##############################################
                p.firstTime = False
                p.counter_control_loop = 0
                p.updateKinematics(p.kin)
                des_state = optimizer_states[:, p.counter_inside_reference_window + 1]
                des_forces = optimizer_forces[:,p.counter_inside_reference_window]

                #log
                p.logData(p.counter_inside_reference_window, optimizer_states[:,p.counter_inside_reference_window], des_forces, des_force_dot)
                #update counters
                p.counter_inside_reference_window += 1
                #update swing count
                if (any(p.swinging_legs)):
                    p.swing_counter += 1  # NB a trot is not possible with this

            ###################################
            #control at control frequency (4ms)
            ###################################
            p.computeJointVariables(p, des_state, p.foot_positionW, p.counter_inside_reference_window - 1) #I put -1 cause it was increamented already
            p.computeControl(p, des_state, des_forces)

            #wait for synconization
            rate.sleep()

        # save the input to the optimization and its output in a mat file
        # saveMpcOut = {'x_predMpcPy': optimizer_states, 'u_optmlMpcPy': optimizer_forces,
        #               'u_ref_traj': p.ref_forces,
        #               'x_ref_traj': p.ref_states,
        #               'actual_comPos':p.comPoseW_log,
        #               'actual_baseTwist':p.baseTwistW_log,
        #               'actual_basePos':p.basePoseW_log,
        #               'actual_u':p.grForcesW_log,
        #               'ref_feetW':p.foot_positionW,
        #               'stancVec':optimizer.stance}
        # scipy.io.savemat('./data/saveMpcOut', saveMpcOut)
        # saveMpcOut = {'x_pred': optimizer_states, 'u_optml': optimizer_forces, 'u_ref_traj': p.ref_forces,'x_ref_traj': p.ref_states, 'comPoseW_log':p.comPoseW_log, 'baseTwistW_log':p.baseTwistW_log, 'grForcesW_log':p.grForcesW_log}
        # scipy.io.savemat('saveMpcOut', saveMpcOut)
        plotResult(p, optiConfig.active_plots)


        #plotJoints(p.q_log, p.q_des_log)
        #plotVelocity(p)
        # print "grfs", p.grForcesW
        #plotTorques(p)
        p.pid.setPDs(400.0, 26.0, 0.0)


    p.deregister_nodes(p)
    p.join()

if __name__ == '__main__':
    p = ControlThread()
    try:
        talker(p)
    except ros.ROSInterruptException:
        pass
    
