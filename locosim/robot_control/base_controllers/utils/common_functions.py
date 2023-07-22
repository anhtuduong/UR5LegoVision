# -*- coding: utf-8 -*-
"""
Created on Thu Apr  2 18:07:44 2020

@author: mfocchi
"""
# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[4]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import os
import psutil
#from pinocchio.visualize import GepettoVisualizer
from locosim.robot_control.base_controllers.utils.custom_robot_wrapper import RobotWrapper
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection

import sys
from termcolor import colored
import rospkg
import rospy as ros
import rosnode
import roslaunch
import rosgraph
from roslaunch.parent import ROSLaunchParent
import copy
from locosim.robot_control.base_controllers.utils.utils import Utils

#from urdf_parser_py.urdf import URDF
#make plot interactive
plt.ion()
plt.close() 

lw_des=7
lw_act=4   
marker_size= 0   

u = Utils()

# globals
labels_ur = ["1 - Shoulder Pan", "2 - Shoulder Lift", "3 - Elbow", "4 - Wrist 1", "5 - Wrist 2", "6 - Wrist 3"]
labels_quadruped = ["LF_HAA", "LF_HFE","LF_KFE","LH_HAA", "LH_HFE","LH_KFE","RF_HAA", "RF_HFE","RF_KFE","RH_HAA", "RH_HFE","RH_KFE"]
labels_flywheel2 = labels_quadruped + ["left_wheel", "right_wheel"]
labels_flywheel4 = labels_quadruped + ["back_wheel", "front_wheel", "left_wheel", "right_wheel"]

class Twist:
    linear = np.empty((3))*np.nan
    angular = np.empty((3))*np.nan
    def set(self, value):
        self.linear = copy.deepcopy(value[:3])
        self.angular = copy.deepcopy(value[3:])

class Pose:
    position = np.empty((3))*np.nan
    orientation = np.empty((3))*np.nan
    def set(self, value):
        self.position = copy.deepcopy(value[:3])
        self.orientation = copy.deepcopy(value[3:])
            
    
class State:
    
    def __init__(self, desired = False):
        self.pose = Pose()
        self.twist = Twist()
        if (desired):
            self.accel = Twist()
            
    def set(self, value):
        self.pose.set(value.getPose())
        self.twist.set(value.getTwist())
            
    def getPose(self):
        return np.hstack([self.pose.position, self.pose.orientation]) 
            
    def getTwist(self):
        return np.hstack([self.twist.linear, self.twist.angular])
        
def checkRosMaster():
    if rosgraph.is_master_online():  # Checks the master uri and results boolean (True or False)
        print(colored('ROS MASTER is Online','red'))
    else:
        print(colored('ROS MASTER is NOT Online, Starting roscore!!','red'))
        parent = ROSLaunchParent("roscore", [], is_core=True)  # run_id can be any string
        parent.start()

def startNode(node_name):
    
    nodes = rosnode.get_node_names()
    if "/reference_generator" in nodes:
        print(colored("Re Starting ref generator","red"))
        os.system("rosnode kill /"+node_name)
    package = node_name
    executable = node_name
    name = node_name
    namespace = ''
    node = roslaunch.core.Node(package, executable, name, namespace, output="screen")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)


def getRobotModel(robot_name="hyq", generate_urdf = False, xacro_path = None, additional_urdf_args = None):
    ERROR_MSG = 'You should set the environment variable LOCOSIM_DIR"\n';
    path  = os.environ.get('LOCOSIM_DIR', ERROR_MSG)
    srdf      = path + "/robot_urdf/" + robot_name + ".srdf"

    if (generate_urdf):  
        try:       
            #old way
            if (xacro_path is None):
                xacro_path = rospkg.RosPack().get_path(robot_name+'_description')+ '/robots/'+robot_name+'.urdf.xacro'
            
            package = 'xacro'
            executable = 'xacro'
            name = 'xacro'
            namespace = '/'
            # with gazebo 11 you should set in the ros_impedance_controllerXX.launch the new_gazebo_version = true
            # note we generate the urdf with the floating base joint (new gazebo version should be false by default in the xacro of the robot! because Pinocchio needs it!
            args = xacro_path+ ' --inorder -o '+os.environ['LOCOSIM_DIR']+'/robot_urdf/generated_urdf/'+robot_name+'.urdf'
     
     
       
            try:
                flywheel = ros.get_param('/flywheel4')
                args+=' flywheel4:='+flywheel
            except:
                pass

            try:
                flywheel2 = ros.get_param('/flywheel2')
                args += ' flywheel2:=' + flywheel2
            except:
                pass

            try:
                angle = ros.get_param('/angle_deg')
                args += ' angle_deg:=' + angle
            except:
                pass

            try:
                anchorZ = ros.get_param('/anchorZ')
                args += ' anchorZ:=' + anchorZ
            except:
                pass

            if additional_urdf_args is not None:
                args += ' '+additional_urdf_args
            
            os.system("rosrun xacro xacro "+args)  
            #os.system("rosparam get /robot_description > "+os.environ['LOCOSIM_DIR']+'/robot_urdf/'+robot_name+'.urdf')  
            #urdf = URDF.from_parameter_server()
            print("URDF generated_commons")
            urdf_location      = path + "/robot_urdf/generated_urdf/" + robot_name+ ".urdf"
            print(urdf_location)
            robot = RobotWrapper.BuildFromURDF(urdf_location)
            print("URDF loaded in Pinocchio")
        except:
            print ('Issues in URDF generation for Pinocchio, did not succeed')
    else:

        urdf      = path + "/robot_urdf/" + robot_name+ ".urdf"
        robot = RobotWrapper.BuildFromURDF(urdf, [path,srdf ])
    
    return robot                    


def subplot(n_rows, n_cols, n_subplot, sharex=False, sharey=False, ax_to_share=None):
    if sharex and sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot, sharex=ax_to_share, sharey=ax_to_share)
    if sharex and not sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot, sharex=ax_to_share)
    if not sharex and sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot, sharey=ax_to_share)
    if not sharex and not sharey:
        ax = plt.subplot(n_rows, n_cols, n_subplot)
    return ax

def plotJoint(name, time_log, q_log=None, q_des_log=None, qd_log=None, qd_des_log=None, qdd_log=None, qdd_des_log=None, tau_log=None, tau_ffwd_log = None, tau_des_log = None, joint_names = None, q_adm = None,
              sharex=False, sharey=False, start=0, end=-1):
    plot_var_des_log = None
    if name == 'position':
        unit = '[rad]'
        if   (q_log is not None):
            plot_var_log = q_log
        else:
            plot_var_log = None
        if   (q_des_log is not None):
            plot_var_des_log = q_des_log
        else:
            plot_var_des_log = None

    elif name == 'velocity':
        unit = '[rad/s]'
        if   (qd_log is not None):
            plot_var_log = qd_log
        else:
            plot_var_log = None
        if   (qd_des_log is not None):
            plot_var_des_log  = qd_des_log
        else:
            plot_var_des_log = None

    elif name == 'acceleration':
        unit = '[rad/s^2]'
        if   (qdd_log is not None):
            plot_var_log = qdd_log
        else:
            plot_var_log = None
        if   (qdd_des_log is not None):
            plot_var_des_log  = qdd_des_log
        else:
            plot_var_des_log = None

    elif name == 'torque':
        unit = '[Nm]'
        if   (tau_log is not None):
            plot_var_log = tau_log
        else:
            plot_var_log = None
        if   (tau_des_log is not None):
            plot_var_des_log  = tau_des_log
        else:
          plot_var_des_log = None                                                
    else:
       print(colored("plotJoint error: wrong input string", "red") )
       return

    dt = time_log[1] - time_log[0]
    if type(start) == str:
        start = max(0, int(float(start) / dt + 1))
    if type(end) == str:
        end = min(int(float(end) / dt + 1), time_log.shape[0])

    if plot_var_log is not None:
        njoints = min(plot_var_log.shape)
    elif plot_var_des_log is not None:
        njoints = min(plot_var_des_log.shape)

    if len(plt.get_fignums()) == 0:
        figure_id = 1
    else:
        figure_id = max(plt.get_fignums())+1
    fig = plt.figure(figure_id)                
    fig.suptitle(name, fontsize=20)

    if joint_names is None:
        if njoints <= 6:
            labels = labels_ur
        if njoints == 12:
            labels = labels_quadruped
        if njoints == 14:
            labels = labels_flywheel2
        if njoints == 16:
            labels = labels_flywheel4
    else:
        labels = joint_names

    if (njoints % 3 == 0): #divisible by 3
        n_rows = int(njoints/ 3)
        n_cols = 3
    elif (njoints % 2 == 0): #divisible by 3
        n_rows = int(njoints / 2)
        n_cols = 2
    else:  # put in a single columnn
        n_rows = njoints
        n_cols = 1


    for jidx in range(njoints):
        if (njoints % 3 == 0): #divisible by 3
            if jidx == 0:
                ax = subplot(n_rows, n_cols, jidx + 1)
            else:
                subplot(n_rows, n_cols, jidx + 1, sharex=sharex, sharey=sharey, ax_to_share=ax)

            if jidx + n_cols >= njoints:
                plt.xlabel("Time [s]")


        plt.ylabel(labels[jidx] + ' '+ unit)

        if name == 'torque' and tau_ffwd_log is not None:
            plt.plot(time_log[start:end], tau_ffwd_log[jidx, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_des,
                     color='green')
        if   (plot_var_des_log is not None):
             plt.plot(time_log[start:end], plot_var_des_log[jidx, start:end], linestyle='-', marker="o",markersize=marker_size, lw=lw_des,color = 'red')
        if (plot_var_log is not None):
            plt.plot(time_log[start:end], plot_var_log[jidx,start:end],linestyle='-',marker="o",markersize=marker_size, lw=lw_act,color = 'blue')

        if (q_adm is not None):
            plt.plot(time_log[start:end], q_adm[jidx, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='black')
        plt.grid()

    if njoints == 12:
        fig.align_ylabels(fig.axes[0:12:3])
        fig.align_ylabels(fig.axes[1:12:4])
        fig.align_ylabels(fig.axes[2:12:4])

    return fig


def plotEndeff(name, figure_id, time_log, plot_var_log, plot_var_des_log = None):

    fig = plt.figure(figure_id)
    fig.suptitle(name, fontsize=20)                   
    plt.subplot(3,1,1)
    plt.ylabel("x")
    if   (plot_var_des_log is not None):
         plt.plot(time_log, plot_var_des_log[0,:], lw=lw_des, color = 'red')                    
    plt.plot(time_log, plot_var_log[0,:], lw=lw_act, color = 'blue')
    plt.grid()
    
    plt.subplot(3,1,2)
    plt.ylabel("y")
    if   (plot_var_des_log is not None):
         plt.plot(time_log, plot_var_des_log[1,:], lw=lw_des, color = 'red')                    
    plt.plot(time_log, plot_var_log[1,:], lw=lw_act, color = 'blue')
    plt.grid()
    
    plt.subplot(3,1,3)
    plt.ylabel("z")
    if   (plot_var_des_log is not None):
        plt.plot(time_log, plot_var_des_log[2,:], lw=lw_des, color = 'red')                                        
    plt.plot(time_log, plot_var_log[2,:], lw=lw_act, color = 'blue')
    plt.grid()


def plotAdmittanceTracking(figure_id, time_log, x_log, x_des_log, x_des_log_adm, f_log):

    fig = plt.figure(figure_id)
    fig.suptitle("admittance tracking", fontsize=20)
    plt.subplot(4, 1, 1)
    plt.ylabel("end-effector x")
    plt.plot(time_log, x_log[0, :], lw=3, color='blue')
    plt.plot(time_log, x_des_log[0, :], lw=2, color='red')
    plt.plot(time_log, x_des_log_adm[0, :], lw=2, color='black')
    plt.grid()

    plt.subplot(4, 1, 2)
    plt.ylabel("end-effector y")
    plt.plot(time_log, x_log[1, :], lw=3, color='blue')
    plt.plot(time_log, x_des_log[1, :], lw=2, color='red')
    plt.plot(time_log, x_des_log_adm[1, :], lw=2, color='black')
    plt.grid()

    plt.subplot(4, 1, 3)
    plt.ylabel("end-effector z")
    plt.plot(time_log, x_log[2, :], lw=3, color='blue')
    plt.plot(time_log, x_des_log[2, :], lw=2, color='red')
    plt.plot(time_log, x_des_log_adm[2, :], lw=2, color='black')
    plt.grid()

    f_norm = []
    for i in range(f_log.shape[1]):
        f_norm.append(np.linalg.norm(f_log[:,i]))

    plt.subplot(4, 1, 4)
    plt.plot(time_log, f_norm, lw=2, color='blue')
    plt.ylabel("norm of ee force")
    plt.grid()

def plotFrame(name, time_log, des_Pose_log=None, Pose_log=None, des_Twist_log=None, Twist_log=None, des_Acc_log=None, Acc_log=None,
              des_Wrench_log=None, Wrench_log=None, title=None, frame=None, sharex=False, sharey=False, start=0, end=-1):
    plot_var_des_log = None
    if name == 'position':
        labels = ["x", "y", "z", "R", "P", "Y"]
        lin_unit = '[m]'
        ang_unit = '[rad]'
        if Pose_log is not None:
            plot_var_log = Pose_log
        if (des_Pose_log is not None):
            plot_var_des_log = des_Pose_log
    elif name == 'velocity':
        labels = ["x", "y", "z", "R", "P", "Y"]
        lin_unit = '[m/s]'
        ang_unit = '[rad/s]'
        if Twist_log is not None:
            plot_var_log = Twist_log
        if   (des_Twist_log is not None):
            plot_var_des_log  = des_Twist_log
    elif name == 'acceleration':
        labels = ["x", "y", "z", "R", "P", "Y"]
        lin_unit = '[m/s^2]'
        ang_unit = '[rad/s^2]'
        if Acc_log is not None:
            plot_var_log = Acc_log
        if   (des_Acc_log is not None):
            plot_var_des_log  = des_Acc_log
    elif name == 'wrench':
        labels = ["FX", "FY", "FZ", "MX", "MY", "MX"]
        lin_unit = '[N]'
        ang_unit = '[Nm]'
        if Wrench_log is not None:
            plot_var_log = Wrench_log
        if (des_Wrench_log is not None):
            plot_var_des_log = des_Wrench_log
    else:
       print("wrong choice")

    if title is None:
        title = name
    else:
        title = title + ' ' + name
    if frame is not None:
        title+= ' ' + frame

    dt = time_log[1] - time_log[0]
    if type(start) == str:
        start = int(float(start)/dt + 1)
    if type(end) == str:
        end = int(float(end)/dt + 1)

    if len(plt.get_fignums()) == 0:
        figure_id = 1
    else:
        figure_id = max(plt.get_fignums()) + 1
    fig = plt.figure(figure_id)
    fig.suptitle(title, fontsize=20)
    ax = subplot(3, 2, 1, sharex=False, sharey=False, ax_to_share=None)
    plt.ylabel(labels[0] + " "+lin_unit)
    if (plot_var_des_log is not None):
        plt.plot(time_log[start:end], plot_var_des_log[0, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[0, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='blue')
    plt.grid()
    ax.ticklabel_format(useOffset=False)

    subplot(3, 2, 3, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[1] + " "+lin_unit)
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[1, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[1, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()
    plt.ticklabel_format(useOffset=False)

    subplot(3, 2, 5, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[2] + " "+lin_unit)
    plt.xlabel("Time [s]")
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[2, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[2, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()
    plt.ticklabel_format(useOffset=False)

    subplot(3, 2, 2, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[3] + " "+ang_unit)
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[3, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[3, start:end].T, linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()
    plt.ticklabel_format(useOffset=False)

    subplot(3, 2, 4, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[4] + " "+ang_unit)
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[4, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[4, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()
    plt.ticklabel_format(useOffset=False)

    subplot(3, 2, 6, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[5] + " "+ang_unit)
    plt.xlabel("Time [s]")
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[5, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[5, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()
    plt.ticklabel_format(useOffset=False)


    fig.align_ylabels(fig.axes[:3])
    fig.align_ylabels(fig.axes[3:])

    return fig

def plotFrameLinear(name, time_log, des_Pose_log=None, Pose_log=None, des_Twist_log=None, Twist_log=None, des_Acc_log=None, Acc_log=None,
              des_Wrench_log=None, Wrench_log=None, title=None, frame=None, sharex=True, sharey=True, start=0, end=-1):
    plot_var_log = None
    plot_var_des_log = None
    if name == 'position':
        labels = ["x", "y", "z"]
        lin_unit = '[m]'
        if Pose_log is not None:
            if Pose_log.shape[0] == 6:
                plot_var_log = u.linPart(Pose_log)
            elif Pose_log.shape[0] == 3:
                plot_var_log = Pose_log
        if (des_Pose_log is not None):
            if des_Pose_log.shape[0] == 6:
                plot_var_des_log = u.linPart(des_Pose_log)
            elif des_Pose_log.shape[0] == 3:
                plot_var_des_log = des_Pose_log

    elif name == 'velocity':
        labels = ["x", "y", "z"]
        lin_unit = '[m/s]'
        if Twist_log is not None:
            if Twist_log.shape[0] == 6:
                plot_var_log = u.linPart(Twist_log)
            elif Twist_log.shape[0] == 3:
                plot_var_log = Twist_log
        if (des_Twist_log is not None):
            if des_Twist_log.shape[0] == 6:
                plot_var_des_log = u.linPart(des_Twist_log)
            elif des_Twist_log.shape[0] == 3:
                plot_var_des_log = des_Twist_log

    elif name == 'acceleration':
        labels = ["x", "y", "z"]
        lin_unit = '[m/s^2]'
        if Acc_log is not None:
            if Acc_log.shape[0] == 6:
                plot_var_log = u.linPart(Acc_log)
            elif Acc_log.shape[0] == 3:
                plot_var_log = Acc_log
        if (des_Acc_log is not None):
            if des_Acc_log.shape[0] == 6:
                plot_var_des_log = u.linPart(des_Acc_log)
            elif des_Acc.shape[0] == 3:
                plot_var_des_log = des_Acc_log

    elif name == 'wrench':
        labels = ["FX", "FY", "FZ"]
        lin_unit = '[N]'
        if Wrench_log is not None:
            if Wrench_log.shape[0] == 6:
                plot_var_log = u.linPart(Wrench_log)
            elif Wrench_log.shape[0] == 3:
                plot_var_log = Wrench_log
        if (des_Wrench_log is not None):
            if des_Wrench_log.shape[0] == 6:
                plot_var_des_log = u.linPart(des_Wrench_log)
            elif des_Wrench_log.shape[0] == 3:
                plot_var_des_log = des_Wrench_log
    else:
       print("wrong choice")

    if title is None:
        title = name
    else:
        title = title + ' ' + name
    if frame is not None:
        title+= ' ' + frame

    dt = time_log[1] - time_log[0]
    if type(start) == str:
        start = max(0, int(float(start) / dt + 1))
    if type(end) == str:
        end = min(int(float(end) / dt + 1), time_log.shape[0])

    if len(plt.get_fignums()) == 0:
        figure_id = 1
    else:
        figure_id = max(plt.get_fignums()) + 1
    fig = plt.figure(figure_id)
    fig.suptitle(title, fontsize=20)
    ax = subplot(3, 1, 1, sharex=False, sharey=False, ax_to_share=None)
    plt.ylabel(labels[0] + " "+lin_unit)
    if (plot_var_des_log is not None):
        plt.plot(time_log[start:end], plot_var_des_log[0, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[0, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act, color='blue')
    plt.grid()

    subplot(3, 1, 2, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[1] + " "+lin_unit)
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[1, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[1, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()

    subplot(3, 1, 3, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[2] + " "+lin_unit)
    plt.xlabel("Time [s]")
    if (plot_var_des_log is not None):
       plt.plot(time_log[start:end], plot_var_des_log[2, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[2, start:end], linestyle='-', marker="o", markersize=marker_size, lw=lw_act,
            color='blue')
    plt.grid()

    fig.align_ylabels(fig.axes[:3])

    return fig


def plotFrameAngular(name, time_log, des_Pose_log=None, Pose_log=None, des_Twist_log=None, Twist_log=None, des_Acc_log=None, Acc_log=None,
                    des_Wrench_log=None, Wrench_log=None, title=None, frame=None, sharex=True, sharey=True, start=0, end=-1):
    plot_var_log = None
    plot_var_des_log = None
    if name == 'position':
        labels = ["R", "P", "Y"]
        ang_unit = '[rad]'
        if Pose_log is not None:
            if Pose_log.shape[0] == 6:
                plot_var_log = u.angPart(Pose_log)
            elif Pose_log.shape[0] == 3:
                plot_var_log = Pose_log
        if (des_Pose_log is not None):
            if des_Pose_log.shape[0] == 6:
                plot_var_des_log = u.angPart(des_Pose_log)
            elif des_Pose_log.shape[0] == 3:
                plot_var_des_log = Pose_log

    elif name == 'velocity':
        labels = ["R", "P", "Y"]
        ang_unit = '[rad]'
        if Twist_log is not None:
            if Twist_log.shape[0] == 6:
                plot_var_log = u.angPart(Twist_log)
            elif Twist_log.shape[0] == 3:
                plot_var_log = Twist_log
        if (des_Twist_log is not None):
            if des_Twist_log.shape[0] == 6:
                plot_var_des_log = u.angPart(des_Twist_log)
            elif des_Twist_log.shape[0] == 3:
                plot_var_des_log = Twist_log

    elif name == 'acceleration':
        labels = ["R", "P", "Y"]
        ang_unit = '[rad]'
        if Acc_log is not None:
            if Acc_log.shape[0] == 6:
                plot_var_log = u.angPart(Acc_log)
            elif Acc_log.shape[0] == 3:
                plot_var_log = Acc_log
        if (des_Acc_log is not None):
            if des_Acc_log.shape[0] == 6:
                plot_var_des_log = u.angPart(des_Acc_log)
            elif des_Acc.shape[0] == 3:
                plot_var_des_log = des_Acc_log

    elif name == 'wrench':
        labels = ["MX", "MY", "MZ"]
        ang_unit = '[Nm]'
        if Wrench_log is not None:
            if Wrench_log.shape[0] == 6:
                plot_var_log = u.angPart(Wrench_log)
            elif Wrench_log.shape[0] == 3:
                plot_var_log = Wrench_log
        if (des_Wrench_log is not None):
            if des_Wrench_log.shape[0] == 6:
                plot_var_des_log = u.angPart(des_Wrench_log)
            elif des_Wrench_log.shape[0] == 3:
                plot_var_des_log = des_Wrench_log
    else:
        print("wrong choice")

    if title is None:
        title = name
    else:
        title = title + ' ' + name
    if frame is not None:
        title += ' ' + frame

    dt = time_log[1] - time_log[0]
    if type(start) == str:
        start = max(0, int(float(start) / dt + 1))
    if type(end) == str:
        end = min(int(float(end) / dt + 1), time_log.shape[0])

    if len(plt.get_fignums()) == 0:
        figure_id = 1
    else:
        figure_id = max(plt.get_fignums()) + 1
    fig = plt.figure(figure_id)
    fig.suptitle(title, fontsize=20)
    ax = subplot(3, 1, 1, sharex=False, sharey=False, ax_to_share=None)
    plt.ylabel(labels[0] + " " + ang_unit)
    if (plot_var_des_log is not None):
        plt.plot(time_log[start:end], plot_var_des_log[0, start:end], linestyle='-', marker="o", markersize=marker_size,
                 lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[0, start:end], linestyle='-', marker="o", markersize=marker_size,
                 lw=lw_act, color='blue')
    plt.grid()

    subplot(3, 1, 2, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[1] + " " + ang_unit)
    if (plot_var_des_log is not None):
        plt.plot(time_log[start:end], plot_var_des_log[1, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[1, start:end], linestyle='-', marker="o", markersize=marker_size,
                 lw=lw_act,
                 color='blue')
    plt.grid()

    subplot(3, 1, 3, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel(labels[2] + " " + ang_unit)
    plt.xlabel("Time [s]")
    if (plot_var_des_log is not None):
        plt.plot(time_log[start:end], plot_var_des_log[2, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        plt.plot(time_log[start:end], plot_var_log[2, start:end], linestyle='-', marker="o", markersize=marker_size,
                 lw=lw_act,
                 color='blue')
    plt.grid()

    fig.align_ylabels(fig.axes[:3])

    return fig


def plotContacts(name, time_log, des_LinPose_log=None, LinPose_log=None, des_LinTwist_log=None, LinTwist_log=None, des_Forces_log=None,
                 Forces_log=None, gt_Forces_log=None, contact_states=None, frame=None, sharex=True, sharey=True, start=0, end=-1):
    # %% Input plots
    plot_var_log = None
    plot_var_des_log = None
    if name == 'position':
        unit = '[m]'
        if LinPose_log is not None:
            plot_var_log = LinPose_log
        if (des_LinPose_log is not None):
            plot_var_des_log = des_LinPose_log

    elif name == 'velocity':
        unit = '[m/s]'
        if LinTwist_log is not None:
            plot_var_log = LinTwist_log
        if (des_LinTwist_log is not None):
            plot_var_des_log = des_LinTwist_log

    elif name == 'GRFs':
        labels = ["FX", "FY", "FZ"]
        unit = '[N]'
        if Forces_log is not None:
            plot_var_log = Forces_log
        if (des_Forces_log is not None):
            plot_var_des_log = des_Forces_log
    else:
        print("wrong choice")

    title = 'Contacts ' + name
    if frame is not None:
        title += ' ' + frame

    dt = time_log[1] - time_log[0]
    if type(start) == str:
        start = max(0, int(float(start) / dt + 1))
    if type(end) == str:
        end = min(int(float(end) / dt + 1), time_log.shape[0])

    if len(plt.get_fignums()) == 0:
        figure_id = 1
    else:
        figure_id = max(plt.get_fignums())+1
    fig = plt.figure(figure_id)
    fig.suptitle(title, fontsize=20)

    ##########
    # LF leg #
    ##########
    # x
    idx = u.leg_map['LF']
    ax = subplot(6, 2, 1)
    plt.ylabel("$LF_x " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax.twinx()
        ax2.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax.plot(time_log[start:end], plot_var_des_log[3 * idx, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        ax.plot(time_log[start:end], plot_var_log[3 * idx, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax.plot(time_log[start:end], gt_Forces_log[3*idx, start:end], linestyle='-', lw=lw_act, color='green')
    ax.grid()


    # y
    ax1 = subplot(6, 2, 3, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$LF_y " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        ax2.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 1, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # z
    ax1 = subplot(6, 2, 5, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$LF_z " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])

    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 2, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()

    ##########
    # RF leg #
    ##########
    # x
    idx = u.leg_map['RF']
    ax1 = subplot(6,2,2, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RF_x " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # y
    ax1 = subplot(6,2,4, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RF_y " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 1, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # z
    ax1 = subplot(6,2,6, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RF_z " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 2, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    ##########
    # LH leg #
    ##########
    # x
    idx = u.leg_map['LH']
    ax1 = subplot(6,2,7, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$LH_x " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3 * idx, start:end], linestyle='-', lw=lw_des, color='red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # y
    ax1 = subplot(6,2,9, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$LH_y " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 1, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # z
    ax1 = subplot(6, 2, 11, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$LH_z " + unit +"$", fontsize=10)
    plt.xlabel("Time [s]")
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 2, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    ##########
    # RH leg #
    ##########
    # x
    idx = u.leg_map['RH']
    ax1 = subplot(6, 2, 8, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RH_x " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # y
    ax1 = subplot(6, 2, 10, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RH_y " + unit +"$", fontsize=10)
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 1, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 1, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()


    # z
    ax1 = subplot(6, 2, 12, sharex=sharex, sharey=sharey, ax_to_share=ax)
    plt.ylabel("$RH_z " + unit +"$", fontsize=10)
    plt.xlabel("Time [s]")
    if contact_states is not None:
        ax2 = ax1.twinx()
        plt.plot(time_log[start:end], contact_states[idx, start:end], linestyle='-', lw=2, color='black')
        ax2.set_ylim([-1.5, 1.5])
        ax2.set_yticks([0, 1])
    if plot_var_des_log is not None:
        ax1.plot(time_log[start:end], plot_var_des_log[3*idx + 2, start:end], linestyle='-', lw=lw_des, color = 'red')
    if plot_var_log is not None:
        ax1.plot(time_log[start:end], plot_var_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='blue')
    if name == 'GRFs' and gt_Forces_log is not None:
        ax1.plot(time_log[start:end], gt_Forces_log[3*idx + 2, start:end], linestyle='-', lw=lw_act, color='green')
    ax1.grid()

    

    # axes = fig.axes
    # for i in range(6):
    #     yticks = axes[i].get_yticks()
    #     ymin = min(-0.01, min(yticks))
    #     ymax = max(0.01, max(yticks))
    #     axes[i].set_ylim([ymin, ymax])
    #     yticks = axes[i].get_yticks()
    #     axes[i].set_yticks(np.unique(np.around(yticks, 2)))


    fig.align_ylabels(fig.axes[0:12:4])
    fig.align_ylabels(fig.axes[1:12:4])
    fig.align_ylabels(fig.axes[2:12:4])
    fig.align_ylabels(fig.axes[3:12:4])

    return fig



def plotConstraitViolation(figure_id,constr_viol_log):
    fig = plt.figure(figure_id)            
    plt.plot(constr_viol_log[0,:],label="LF")
    plt.plot(constr_viol_log[1,:],label="RF")
    plt.plot(constr_viol_log[2,:],label="LH")
    plt.plot(constr_viol_log[3,:],label="RH")
    plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
    plt.ylabel("Constr violation", fontsize=10)
    plt.grid()                                                                     

def plotEndeffImpedance(name, figure_id, x_log, x_des_log, f_log):                  
    
    title=""    
    
    if name == 'position':
        title="Force vs Displacement" 
    elif name == 'velocity':
        title="Force vs Velocity" 
    elif name == 'acceleration':
        title="Force vs Acceleration"                           
    else:
        print("wrong choice in impedance plotting")
 
    lw_act=4  
    lw_des=7
                    
#    fig = plt.figure(figure_id)    
    fig, axs = plt.subplots(3, 3)
    fig.suptitle(title, fontsize=20)
    
    axs[0, 0].plot((x_log[0,:].T-x_des_log[0,:].T), f_log[0,:].T, lw=lw_act, color = 'blue')
    axs[0, 0].set_title('Fx vs X')
    axs[0, 0].grid()
    
    axs[0, 1].plot((x_log[1,:].T-x_des_log[1,:].T), f_log[0,:].T, lw=lw_act, color = 'blue')
    axs[0, 1].set_title('Fx vs Y')
    axs[0, 1].grid()
    
    axs[0, 2].plot((x_log[2,:].T-x_des_log[2,:].T), f_log[0,:].T, lw=lw_act, color = 'blue')
    axs[0, 2].set_title('Fx vs Z')
    axs[0, 2].grid()
    
    axs[1, 0].plot((x_log[0,:].T-x_des_log[0,:].T), f_log[1,:].T, lw=lw_act, color = 'blue')
    axs[1, 0].set_title('Fy vs X')
    axs[1, 0].grid()
    
    axs[1, 1].plot((x_log[1,:].T-x_des_log[1,:].T), f_log[1,:].T, lw=lw_act, color = 'blue')
    axs[1, 1].set_title('Fy vs Y')
    axs[1, 1].grid()
    
    axs[1, 2].plot((x_log[2,:].T-x_des_log[2,:].T), f_log[1,:].T, lw=lw_act, color = 'blue')
    axs[1, 2].set_title('Fy vs Z')
    axs[1, 2].grid()
    
    axs[2, 0].plot((x_log[0,:].T-x_des_log[0,:].T), f_log[2,:].T, lw=lw_act, color = 'blue')
    axs[2, 0].set_title('Fz vs X')
    axs[2, 0].grid()
    
    axs[2, 1].plot((x_log[1,:].T-x_des_log[1,:].T), f_log[2,:].T, lw=lw_act, color = 'blue')
    axs[2, 1].set_title('Fz vs Y')
    axs[2, 1].grid()
    
    axs[2, 2].plot((x_log[2,:].T-x_des_log[2,:].T), f_log[2,:].T, lw=lw_act, color = 'blue')
    axs[2, 2].set_title('Fz vs Z')
    axs[2, 2].grid()

    return fig
    
def plotJointImpedance(name, q_log, q_des_log, tau_log):
    
    title=""
    
    if name == 'position':
        title="Torque vs Angular Displacement"      
    elif name == 'velocity':
        title="Torue vs Angular Velocity" 
    elif name == 'acceleration':
        title="Torque vs Angular Acceleration"                           
    else:
        print("wrong choice in impedance plotting")
 
    lw_act=4  
    lw_des=3

    #Number of joints
    njoints = q_log.shape[0]                                                            
    
    #neet to transpose the matrix other wise it cannot be plot with numpy array    
    fig = plt.figure()                
    fig.suptitle(name, fontsize=20)             
    labels_ur = ["1 - Shoulder Pan", "2 - Shoulder Lift","3 - Elbow","4 - Wrist 1","5 - Wrist 2","6 - Wrist 3"]
    labels_hyq = ["LF_HAA", "LF_HFE","LF_KFE","RF_HAA", "RF_HFE","RF_KFE","LH_HAA", "LH_HFE","LH_KFE","RH_HAA", "RH_HFE","RH_KFE"]

    if njoints == 6:
        labels = labels_ur         
    if njoints == 12:
        labels = labels_hyq                  
                
    
    for jidx in range(njoints):
                
        plt.subplot(njoints/2,2,jidx+1)
        plt.ylabel(labels[jidx])    
        plt.plot(q_log[jidx,:].T-q_des_log[jidx,:].T, tau_log[jidx,:].T, linestyle='-', lw=lw_des,color = 'blue')
        plt.grid()


def polar_char(name, figure_id, phase_deg, mag0, mag1=None, mag2=None):
    import matplotlib as mpl
    size_font = 24
    mpl.rcdefaults()
    mpl.rcParams['lines.linewidth'] = 10
    mpl.rcParams['lines.markersize'] = 6
    mpl.rcParams['patch.linewidth'] = 2
    mpl.rcParams['axes.grid'] = True
    mpl.rcParams['axes.labelsize'] = size_font
    mpl.rcParams['font.family'] = 'sans-serif'
    mpl.rcParams['font.size'] = size_font
    mpl.rcParams['font.serif'] = ['Times New Roman', 'Times', 'Bitstream Vera Serif', 'DejaVu Serif',
                                  'New Century Schoolbook',
                                  'Century Schoolbook L', 'Utopia', 'ITC Bookman', 'Bookman', 'Nimbus Roman No9 L',
                                  'Palatino',
                                  'Charter', 'serif']
    mpl.rcParams['text.usetex'] = True
    mpl.rcParams['legend.fontsize'] = size_font
    mpl.rcParams['legend.loc'] = 'best'
    mpl.rcParams['figure.facecolor'] = 'white'
    mpl.rcParams['figure.figsize'] = 14, 14
    mpl.rcParams['savefig.format'] = 'pdf'

    phase_rad = []
    for deg in phase_deg:
        rad = deg * np.pi/180
        phase_rad.append(rad)

    patches = []
    for mag in [mag0, mag1, mag2]:
        if mag is not None:
            poly = np.zeros((len(phase_rad), 2))
            for i in range(len(phase_rad)):
                poly[i, :] = np.array([phase_rad[i], mag[i]])
            patches.append(Polygon(poly))

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    # plt.subplots_adjust(left=0.04, bottom=0.04, top=0.96, right=0.96)

    p = PatchCollection(patches, alpha=0.7)
    fcolors = ['g', 'dodgerblue', 'coral']
    ecolors = ['darkgreen', 'b', 'r']
    p.set_edgecolor(ecolors)
    p.set_facecolor(fcolors)


    ax.set_rmax(3)
    step = np.abs(phase_deg[0]-phase_deg[1])
    phase_rad =np.arange(0,360, step)*np.pi/180
    ax.set_xticks(phase_rad)
    ax.tick_params(axis='x', which='major', pad=15)

    rticks = np.arange(0,4,0.5)
    ax.set_rticks(rticks)

    #rticks_show = np.arange(0, 4, 1)
    ax.set_yticklabels(['0', '', '1', '', '2', '', '3'])
    ax.add_collection(p)

    fig.suptitle(name)

    plt.show()
    return fig, ax

    
def plotWrenches(name, figure_id, time_log, des_Wrench_fb_log=None, des_Wrench_ffwd_log=None, des_Wrench_g_log=None):
    labels = ["FX", "FY", "FZ", "MX", "MY", "MZ"]
    lin_unit = '[N]'
    ang_unit = '[Nm]'
    plot_var_des_log = None
    if name=='feedback' or name=='fb':
        plot_var_des_log = des_Wrench_fb_log
    elif name=='feedforward' or name=='ffwd':
        plot_var_des_log = des_Wrench_ffwd_log
    elif name=='gravity' or name=='g':
        plot_var_des_log = des_Wrench_g_log

    # neet to transpose the matrix other wise it cannot be plot with numpy array
    fig = plt.figure(figure_id)
    fig.suptitle('Wrench ' + name, fontsize=20)
    plt.subplot(3, 2, 1)
    plt.ylabel(labels[0])
    plt.plot(time_log, plot_var_des_log[0, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 3)
    plt.ylabel(labels[1])
    plt.plot(time_log, plot_var_des_log[1, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 5)
    plt.ylabel(labels[2])
    plt.xlabel("Time [s]")
    plt.plot(time_log, plot_var_des_log[2, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 2)
    plt.ylabel(labels[3] )
    plt.plot(time_log, plot_var_des_log[3, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 4)
    plt.ylabel(labels[4] )
    plt.plot(time_log, plot_var_des_log[4, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    plt.subplot(3, 2, 6)
    plt.ylabel(labels[5] )
    plt.xlabel("Time [s]")
    plt.plot(time_log, plot_var_des_log[5, :], linestyle='-', marker="o", markersize=marker_size, lw=lw_des, color='red')
    plt.grid()

    fig.align_ylabels(fig.axes[:3])
    fig.align_ylabels(fig.axes[3:])

    return fig
