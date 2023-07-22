
#!/usr/bin/env python

from __future__ import print_function


import numpy as np
import sys
import os


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
                 '','..' ]#this is the current directory

from reference_generator.srv import request_reference
from reference_generator.srv import request_referenceRequest
from geometry_msgs.msg import Point
import yaml
from tools.math_tools import Math
from refgen.ref_generation import ReferenceGenerator
from tools.plottingFeatures import plotReference
from tools.utils import Utils
from tools.mathutils import tic, toc
from tools.getConfig import getConfig
from opti.optimizer import ConstructOptimizer

from termcolor import colored

import rospy as ros
import rospkg
import roslaunch
import rosnode
import rosgraph
from roslaunch.parent import ROSLaunchParent

from grid_map_msgs.msg import GridMap
from std_msgs.msg import MultiArrayDimension, Float32MultiArray

current_dir = os.path.dirname(os.path.realpath(__file__))

if __name__ == "__main__":

    OLD_REF_GEN = False # ref gen commit 732f4709ddfc44c712ddf27d0fbc7f8202351
    math = Math()
    utils = Utils()
    #define empty struct
    class active_plots:
        pass

    #load param from yaml
    with open(current_dir + "/../config/config.yaml", 'r') as stream:
        data_loaded = yaml.load(stream)
    #overwrite some values
    data_loaded['verbose'] = 0
    data_loaded['mapper_on'] = 0
    data_loaded['pallet_height'] = 0.0
    # data_loaded['detailed_horizon'] = 25
    # data_loaded['max_Ts'] = 0.1

    if rosgraph.is_master_online():  # Checks the master uri and results boolean (True or False)
        print(colored('ROS MASTER is Online','red'))
    else:
        print(colored('ROS MASTER is NOT Online, Starting roscore!!','red'))
        parent = ROSLaunchParent("roscore", [], is_core=True)  # run_id can be any string
        parent.start()

    # save it into param server for ref gen node
    utils.putIntoParamServer(data_loaded)
    utils.putIntoGlobalParamServer("verbose", data_loaded['verbose'])
    utils.putIntoGlobalParamServer("mapper_on", data_loaded['mapper_on'])


    # start reference generator node
    ros.init_node('sub_pub_node_python', anonymous=False)
    nodes = rosnode.get_node_names()
    if "/mpc/reference_gen" in nodes:
        print(colored("Re Starting ref generator","red"))
        os.system("rosnode kill /mpc/reference_gen")
    package = 'reference_generator'
    executable = 'reference_generator'
    name = 'reference_gen'
    namespace = '/mpc'
    node = roslaunch.core.Node(package, executable, name, namespace, output="screen")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

    # start mapper  node
    if data_loaded['mapper_on']:
        nodes = rosnode.get_node_names()
        print(colored("TO WORK YO NEED DWL CONTROLLER ON!", "red"))
        if "/dls_mapper_node" in nodes:
            print("ROSNODE: re-starting dls_mapper node")
            os.system("rosnode kill /dls_mapper_node")
        else:
            print(colored("ROSNODE: starting dls_mapper","red"))
        ros.sleep(5.0)
        rospack = rospkg.RosPack()
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        node_path = [rospack.get_path('dls_mapper') + "/launch/simulation.launch"]
        launch = roslaunch.parent.ROSLaunchParent(uuid, node_path)
        launch.start()

    # # publish fake gridmap message (TODO)
    # grid_map_pub_ = ros.Publisher('/local_gridmap', GridMap, queue_size=10)
    # grid_map_msg_ = GridMap()
    # grid_map_msg_.info.header.frame_id = 'world'
    # grid_map_msg_.info.header.stamp = ros.get_rostime()
    #
    # msg = Float32MultiArray()
    # data = np.ones((20, 20))
    # msg.data = data.reshape([400])
    # # This is almost always zero there is no empty padding at the start of your data
    # msg.layout.data_offset = 0
    # # create two dimensions in the dim array
    # msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
    # # dim[0] is the vertical dimension of your matrix
    # msg.layout.dim[0].label = "channels"
    # msg.layout.dim[0].size = 2
    # msg.layout.dim[0].stride = 400
    # # dim[1] is the horizontal dimension of your matrix
    # msg.layout.dim[1].label = "samples"
    # msg.layout.dim[1].size = 200
    # msg.layout.dim[1].stride = 200
    # grid_map_msg_.data = msg
    # grid_map_pub_.publish(grid_map_msg_)

    ref_request_client = ros.ServiceProxy("/mpc/reference_generator/request_reference", request_reference)
    refclass = ReferenceGenerator()

    ros.sleep(1.0) #THIS SLEEP IS fundamental to allow the dls_mapper to trigger the callback of the gridmapinterface!

    #load param you need from param server here
    refclass.loadConfig()

    # start filling  request message
    print("filling message")
    req = request_referenceRequest()

    # intial com position
    req.com_initial_position.x = 0.0
    req.com_initial_position.y = 0.0
    req.com_initial_position.z = 0.5

    # intial com orientation
    req.com_initial_orientation.x = 0.0
    req.com_initial_orientation.y = 0.0
    req.com_initial_orientation.z = 0.0

    #req.com_initial_velocity.x = 0.0
    #req.com_initial_velocity.y = 0.0
    #req.com_initial_velocity.z = 0.0

    if OLD_REF_GEN:
        # OLD desired velocity
        req.com_desired_velocity.x = 0.1
        req.com_desired_velocity.y = 0.02
        req.com_desired_velocity.z = 0.02
    else:
        # NEW desired velocity commands (linear and heading)
        req.desired_command.x = 0.1 #this is lin vel x
        req.desired_command.y = 0.02 #this is lin vel y
        req.desired_command.z = 0.02 #this is heading
        req.desired_command.x = 0.03 # this is lin vel x
        req.ref_counter = 0
        req.haptic_touch_down = False

    active_plots.swing = True
    active_plots.feet = False
    active_plots.com = False
    active_plots.orientation = False
    active_plots.grforces = True

    # intial feet position
    feet = [Point(), Point(), Point(), Point()]
    feet[utils.leg_map("LF")].x = 0.3
    feet[utils.leg_map("LF")].y = 0.3
    feet[utils.leg_map("LF")].z = 0.0
    feet[utils.leg_map("RF")].x = 0.3
    feet[utils.leg_map("RF")].y = -0.3
    feet[utils.leg_map("RF")].z = -0.0
    feet[utils.leg_map("LH")].x = -0.3
    feet[utils.leg_map("LH")].y = 0.3
    feet[utils.leg_map("LH")].z = 0.0
    feet[utils.leg_map("RH")].x = -0.3
    feet[utils.leg_map("RH")].y = -0.3
    feet[utils.leg_map("RH")].z = 0.0
    req.initial_feet_positions = feet

    req.robot_height = 0.555

    # initial swing state
    #req.current_swing = 3

    if(req.ref_counter == 120):

        req.swing_status[0] = True
        req.swing_status[1] = False
        req.swing_status[2] = False
        req.swing_status[3] = True

    else:
        req.swing_status[0] = True
        req.swing_status[1] = True
        req.swing_status[2] = True
        req.swing_status[3] = True
    req.swing_counter = 0

    #actual_delta_step = [ 0.1, 0.0]
    #req.actual_delta_step = actual_delta_step  # TODO fix this I put a random value, work with replanning not during swing

    tic()
    # send request and get response
    print("sending request")
    refclass.response = ref_request_client(req)

    toc()

    # unpack response message
    print("unpacking response")
    refclass.actual_CoMXW = refclass.getComPosition("x")
    refclass.actual_CoMYW = refclass.getComPosition("y")
    refclass.actual_CoMZW = refclass.getComPosition("z")

    refclass.com_VxW = refclass.getComVelocity("x")
    refclass.com_VyW = refclass.getComVelocity("y")
    refclass.com_VzW = refclass.getComVelocity("z")


    if OLD_REF_GEN:
        refclass.omegaXW = []
        for i in range(0, refclass.prediction_horizon):
            refclass.omegaXW.append(0.0)
        refclass.omegaYW = []
        for i in range(0, refclass.prediction_horizon):
            refclass.omegaYW.append(0.0)
        refclass.omegaZW = []
        for i in range(0, refclass.prediction_horizon):
            refclass.omegaZW.append(0.0)

        refclass.rollW = []
        for i in range(0, refclass.prediction_horizon):
            refclass.rollW.append(0.0)
        refclass.pitchW = []
        for i in range(0, refclass.prediction_horizon):
            refclass.pitchW.append(0.0)
        refclass.yawW = []
        for i in range(0, refclass.prediction_horizon):
            refclass.yawW.append(0.0)

    else:
        #NEW reference for roll pitch (we set them to zero) for the yaw we integrate omegaZ inside reference generator
        refclass.rollW = np.zeros(refclass.prediction_horizon + 1)
        refclass.pitchW = np.zeros(refclass.prediction_horizon + 1)
        refclass.yawW = refclass.getHeading()

        # NEW reference for omegaZ is the heading velocity command itself
        refclass.omegaXW = np.zeros(refclass.prediction_horizon + 1)
        refclass.omegaYW = np.zeros(refclass.prediction_horizon + 1)
        refclass.omegaZW = refclass.getHeadingVelocity()

    refclass.swing = np.zeros((4, refclass.prediction_horizon+1)) #reference creates N+1 values
    refclass.swing[0 ,:] = refclass.get_swingState("LF")
    refclass.swing[1 ,:] = refclass.get_swingState("RF")
    refclass.swing[2 ,:] = refclass.get_swingState("LH")
    refclass.swing[3 ,:] = refclass.get_swingState("RH")

    refclass.grForcesLFWx_gt=  refclass.get_grForcesW("LF" ,"x")
    refclass.grForcesLFWy_gt=  refclass.get_grForcesW("LF" ,"y")
    refclass.grForcesLFWz_gt=  refclass.get_grForcesW("LF" ,"z")

    refclass.grForcesLHWx_gt=  refclass.get_grForcesW("LH" ,"x")
    refclass.grForcesLHWy_gt=  refclass.get_grForcesW("LH" ,"y")
    refclass.grForcesLHWz_gt=  refclass.get_grForcesW("LH" ,"z")

    refclass.grForcesRFWx_gt=  refclass.get_grForcesW("RF" ,"x")
    refclass.grForcesRFWy_gt=  refclass.get_grForcesW("RF" ,"y")
    refclass.grForcesRFWz_gt=  refclass.get_grForcesW("RF" ,"z")

    refclass.grForcesRHWx_gt=  refclass.get_grForcesW("RH" ,"x")
    refclass.grForcesRHWy_gt=  refclass.get_grForcesW("RH" ,"y")
    refclass.grForcesRHWz_gt=  refclass.get_grForcesW("RH" ,"z")

    refclass.footPosLFWx = refclass.get_footPosW("LF" ,"x")
    refclass.footPosLFWy = refclass.get_footPosW("LF" ,"y")
    refclass.footPosLFWz = refclass.get_footPosW("LF" ,"z")

    refclass.footPosLHWx = refclass.get_footPosW("LH" ,"x")
    refclass.footPosLHWy = refclass.get_footPosW("LH" ,"y")
    refclass.footPosLHWz = refclass.get_footPosW("LH" ,"z")

    refclass.footPosRFWx = refclass.get_footPosW("RF" ,"x")
    refclass.footPosRFWy = refclass.get_footPosW("RF" ,"y")
    refclass.footPosRFWz = refclass.get_footPosW("RF" ,"z")

    refclass.footPosRHWx = refclass.get_footPosW("RH" ,"x")
    refclass.footPosRHWy = refclass.get_footPosW("RH" ,"y")
    refclass.footPosRHWz = refclass.get_footPosW("RH" ,"z")

    refclass.terrain_normalLFWx = refclass.get_normal("LF", "x")
    refclass.terrain_normalLFWy = refclass.get_normal("LF", "y")
    refclass.terrain_normalLFWz = refclass.get_normal("LF", "z")

    refclass.terrain_normalRFWx = refclass.get_normal("LH", "x")
    refclass.terrain_normalRFWy = refclass.get_normal("LH", "y")
    refclass.terrain_normalRFWz = refclass.get_normal("LH", "z")

    refclass.terrain_normalLHWx = refclass.get_normal("RF", "x")
    refclass.terrain_normalLHWy = refclass.get_normal("RF", "y")
    refclass.terrain_normalLHWz = refclass.get_normal("RF", "z")

    refclass.terrain_normalRHWx = refclass.get_normal("RH", "x")
    refclass.terrain_normalRHWy = refclass.get_normal("RH", "y")
    refclass.terrain_normalRHWz = refclass.get_normal("RH", "z")

    # Save the reference generator data in the text files
    # - get optimization params
    opti_config = getConfig()
    optimizer = ConstructOptimizer()

    # - Load configuration for the class
    optimizer.load_config(opti_config)
    optimizer.extract_ref_traj(refclass)

    # - save references for optimizer
    np.savetxt(current_dir+"/../data/ref_states_py.txt", optimizer.ref_states)
    np.savetxt(current_dir+"/../data/ref_forces_py.txt", optimizer.ref_forces)
    np.savetxt(current_dir+"/../data/ref_param_py.txt", np.vstack([optimizer.feet_positionW, optimizer.stance]))
    np.savetxt(current_dir+"/../data/ref_time_steps_py.txt", refclass.response.time_parametrization)

    # print(len(refclass.response.time_parametrization))

    #get footholds (temporary commented you can add as a message  leg_trajectory[] footholds and uncommenting code in ref gen
    # for leg in range(0, 4):
    #     for index in range(0, len(refclass.response.footholds[leg].data)):
    #         print("\nleg ", leg, "/ foothold ", index+1, ":\n", refclass.response.footholds[leg].data[index])
    plotReference(refclass, active_plots)