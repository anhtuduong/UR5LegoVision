import rospy as ros
import numpy as np

from reference_generator.msg import com_state
from reference_generator.msg import leg_contact_state
from reference_generator.msg import  leg_trajectory
from geometry_msgs.msg import Point
from reference_generator.srv import request_reference
from reference_generator.srv import request_referenceResponse
from reference_generator.srv import request_referenceRequest
from base_controllers.utils.utils import Utils
from base_controllers.utils.common_functions import startNode, checkRosMaster

import matplotlib.pyplot as plt
import matplotlib

#prevent creating pyc files
import sys
sys.dont_write_bytecode = True

class ReferenceGenerator:
    
    def __init__(self, config):

        checkRosMaster()
        startNode("reference_generator")
        ros.wait_for_service("/reference_generator/request_reference")
        self.response = request_referenceResponse()
        self.u = Utils()
        
        # extract  config relevant for ref gen and put into param server
        config_dict = {}
        config_dict['prediction_horizon'] = config.prediction_horizon 
        config_dict['cycle_time'] = config.cycle_time 
        config_dict['time_resolution'] = config.time_resolution 
        config_dict['gait_type'] = config.gait_type 
        config_dict['duty_factor'] = config.duty_factor 
        config_dict['offset_a'] = config.offset_a 
        config_dict['offset_b'] = config.offset_b 
        config_dict['offset_c'] = config.offset_c 
        config_dict['offset_d'] = config.offset_d 
        config_dict['robot_name'] = config.robot_name 
        config_dict['robot_mass'] = config.robot_mass                 
        config_dict['verbose'] = config.verbose  
        config_dict['foot_hip_y_offset'] = config.foot_hip_y_offset  
        # save it into param server for ref gen node
        for label in config_dict:
            self.u.putIntoGlobalParamServer(label, config_dict[label])
        
        self.prediction_horizon = config.prediction_horizon 
        self.reference_length = config.prediction_horizon  + 1

    def getReferenceData(self, initial_state,
                               desired_velocity, 
                               initial_feetW,
                               swing_status,
                               robot_height,
                               delta_step =  [np.zeros((2, 1)), np.zeros((2, 1)), np.zeros((2, 1)), np.zeros((2, 1))],
                               swing_counter = 0, 
                               ref_counter = 0,
                               haptic_td = False):

   
        ref_request = ros.ServiceProxy("/reference_generator/request_reference", request_reference)

        # prepare request
        
        # start filling  request message
        req = request_referenceRequest()
        self.prediction_horizon = ros.get_param("/prediction_horizon")
        self.reference_length =  self.prediction_horizon + 1


        #intial com position
        req.com_initial_position.x = initial_state.pose.position[self.u.crd.get("X")]
        req.com_initial_position.y = initial_state.pose.position[self.u.crd.get("Y")]
        req.com_initial_position.z = initial_state.pose.position[self.u.crd.get("Z")]
        
        #let's assuse they are the same for now
        req.base_initial_position.x = req.com_initial_position.x
        req.base_initial_position.y = req.com_initial_position.y
        req.base_initial_position.z = req.com_initial_position.z

        # intial com orientation
        req.com_initial_orientation.x = initial_state.pose.orientation[self.u.crd.get("X")]
        req.com_initial_orientation.y = initial_state.pose.orientation[self.u.crd.get("Y")]
        req.com_initial_orientation.z = initial_state.pose.orientation[self.u.crd.get("Z")]

         # desired velocity commands (linear and heading)
        req.desired_command.x = desired_velocity.lin_x
        req.desired_command.y = desired_velocity.lin_y
        req.desired_command.z = desired_velocity.ang_z

        req.com_initial_velocity.x = initial_state.twist.linear[self.u.crd.get("X")]
        req.com_initial_velocity.y = initial_state.twist.linear[self.u.crd.get("Y")]
        req.com_initial_velocity.z = initial_state.twist.linear[self.u.crd.get("Z")]

        #intial feet position
        feet = [Point(), Point(), Point(), Point()]
        feet[self.u.leg_map.get("LF")].x = initial_feetW[self.u.leg_map.get("LF")][self.u.crd.get("X")]
        feet[self.u.leg_map.get("LF")].y = initial_feetW[self.u.leg_map.get("LF")][self.u.crd.get("Y")]
        feet[self.u.leg_map.get("LF")].z = initial_feetW[self.u.leg_map.get("LF")][self.u.crd.get("Z")]
        feet[self.u.leg_map.get("RF")].x = initial_feetW[self.u.leg_map.get("RF")][self.u.crd.get("X")]
        feet[self.u.leg_map.get("RF")].y = initial_feetW[self.u.leg_map.get("RF")][self.u.crd.get("Y")]
        feet[self.u.leg_map.get("RF")].z = initial_feetW[self.u.leg_map.get("RF")][self.u.crd.get("Z")]
        feet[self.u.leg_map.get("LH")].x = initial_feetW[self.u.leg_map.get("LH")][self.u.crd.get("X")]
        feet[self.u.leg_map.get("LH")].y = initial_feetW[self.u.leg_map.get("LH")][self.u.crd.get("Y")]
        feet[self.u.leg_map.get("LH")].z = initial_feetW[self.u.leg_map.get("LH")][self.u.crd.get("Z")]
        feet[self.u.leg_map.get("RH")].x = initial_feetW[self.u.leg_map.get("RH")][self.u.crd.get("X")]
        feet[self.u.leg_map.get("RH")].y = initial_feetW[self.u.leg_map.get("RH")][self.u.crd.get("Y")]
        feet[self.u.leg_map.get("RH")].z = initial_feetW[self.u.leg_map.get("RH")][self.u.crd.get("Z")]
        req.initial_feet_positions = feet

        req.robot_height = robot_height
 
        # initial swing state
        req.swing_status = swing_status
        req.swing_counter = swing_counter
               
        #TODO
        req.actual_delta_step_LF = delta_step[self.u.leg_map.get("LF")]        
        req.actual_delta_step_RF = delta_step[self.u.leg_map.get("RF")]        
        req.actual_delta_step_LH = delta_step[self.u.leg_map.get("LH")]    
        req.actual_delta_step_RH = delta_step[self.u.leg_map.get("RH")]

        req.ref_counter = ref_counter    
        req.haptic_touch_down = haptic_td 
             
        ####Filling request message finished
             
        #send request and get response
        self.response = ref_request(req)

        #unpack response
        #reference for com position
        self.actual_CoMXW = self.getComPosition("x")
        self.actual_CoMYW = self.getComPosition("y")
        self.actual_CoMZW = self.getComPosition("z")

        #reference for com velocity
        self.com_VxW = self.getComVelocity("x")
        self.com_VyW = self.getComVelocity("y")
        self.com_VzW = self.getComVelocity("z")


        # reference for roll pitch (we set them to zero) for the yaw we integrate omegaZ inside reference generator
        self.rollW = np.zeros(self.reference_length)
        self.pitchW = np.zeros(self.reference_length)
        self.yawW = self.getHeading()

        # reference for omegaZ is the heading velocity command itself
        self.omegaXW = np.zeros(self.reference_length)
        self.omegaYW = np.zeros(self.reference_length)
        self.omegaZW = self.getHeadingVelocity()

        # reference for swing
        self.swing = np.zeros((4, self.reference_length))
        self.swing[0,:] = self.get_swingState("LF")
        self.swing[1,:] = self.get_swingState("RF")
        self.swing[2,:] = self.get_swingState("LH")
        self.swing[3,:] = self.get_swingState("RH")

        # reference for foot pos
        self.footPosLFWx = self.get_footPosW("LF","x")
        self.footPosLFWy = self.get_footPosW("LF","y")
        self.footPosLFWz = self.get_footPosW("LF","z")

        self.footPosLHWx = self.get_footPosW("LH","x")
        self.footPosLHWy = self.get_footPosW("LH","y")
        self.footPosLHWz = self.get_footPosW("LH","z")

        self.footPosRFWx = self.get_footPosW("RF","x")
        self.footPosRFWy = self.get_footPosW("RF","y")
        self.footPosRFWz = self.get_footPosW("RF","z")

        self.footPosRHWx = self.get_footPosW("RH","x")
        self.footPosRHWy = self.get_footPosW("RH","y")
        self.footPosRHWz = self.get_footPosW("RH","z")

        # reference for grforces
        self.grForcesLFWx_gt=  self.get_grForcesW("LF","x")
        self.grForcesLFWy_gt=  self.get_grForcesW("LF","y")
        self.grForcesLFWz_gt=  self.get_grForcesW("LF","z")

        self.grForcesLHWx_gt=  self.get_grForcesW("LH","x")
        self.grForcesLHWy_gt=  self.get_grForcesW("LH","y")
        self.grForcesLHWz_gt=  self.get_grForcesW("LH","z")

        self.grForcesRFWx_gt=  self.get_grForcesW("RF","x")
        self.grForcesRFWy_gt=  self.get_grForcesW("RF","y")
        self.grForcesRFWz_gt=  self.get_grForcesW("RF","z")

        self.grForcesRHWx_gt=  self.get_grForcesW("RH","x")
        self.grForcesRHWy_gt=  self.get_grForcesW("RH","y")
        self.grForcesRHWz_gt=  self.get_grForcesW("RH","z")
        
        self.terrain_normalLFWx = self.get_normal("LF", "x")
        self.terrain_normalLFWy = self.get_normal("LF", "y")
        self.terrain_normalLFWz = self.get_normal("LF", "z")
    
        self.terrain_normalRFWx = self.get_normal("LH", "x")
        self.terrain_normalRFWy = self.get_normal("LH", "y")
        self.terrain_normalRFWz = self.get_normal("LH", "z")
    
        self.terrain_normalLHWx = self.get_normal("RF", "x")
        self.terrain_normalLHWy = self.get_normal("RF", "y")
        self.terrain_normalLHWz = self.get_normal("RF", "z")
    
        self.terrain_normalRHWx = self.get_normal("RH", "x")
        self.terrain_normalRHWy = self.get_normal("RH", "y")
        self.terrain_normalRHWz = self.get_normal("RH", "z")


    def getComPosition(self, coord):
        temp =[]
        for i in range(0, self.reference_length):
            temp.append(eval("self.response.desired_com[i].position[0]."+coord))
        return temp


    def getComVelocity(self, coord):
        temp =[]
        for i in range(0, self.reference_length):
            temp.append(eval("self.response.desired_com[i].velocity[0]."+coord))
        return temp

    def getHeading(self):
        temp = []
        for i in range(0, self.reference_length):
            temp.append(eval("self.response.desired_orient[i].position[0].z"))
        return temp


    def getHeadingVelocity(self):
        temp = []
        for i in range(0, self.reference_length):
            temp.append(eval("self.response.desired_orient[i].velocity[0].z"))
        return temp


    def get_swingState(self, leg):
        temp =[]
        # first extract the map message
        leg_state = leg_contact_state()
        leg_state = self.response.swing_legs[self.u.leg_map.get(leg)]
        # python3 transform the map into a list to make it callable
        datas = list(leg_state.data)
        for i in range(0, self.reference_length):
            temp.append(datas[i])
        return temp

    def get_footPosW(self, leg, coord):
        temp = []
        for i in range(0, self.reference_length):
            temp.append(eval(" self.response.desired_feet[self.u.leg_map.get(leg)].data[i]."+coord))
        return temp

    def get_normal(self, leg, coord):
        temp = []
        for i in range(0, self.reference_length):
            temp.append(eval(" self.response.terrain_normals[self.u.leg_map.get(leg)].data[i]."+coord))
        
        return temp

    def get_grForcesW(self, leg, coord):
        temp = []
        for i in range(0, self.reference_length):
            temp.append(eval(" self.response.grforces[self.u.leg_map.get(leg)].data[i]."+coord))
        return temp

    def plotReference(self, active_plots):
    
        plt.close('all')
    
        number_of_samples = self.prediction_horizon
    
        time = np.zeros(len(self.response.time_parametrization))
        accumulated_time = 0
        for i in range(len(self.response.time_parametrization)):
            time[i] = accumulated_time
            accumulated_time += self.response.time_parametrization[i]
    
        if active_plots.swing:
            plt.figure()
            plt.rcParams['axes.grid'] = True
            plt.plot(time, np.transpose(self.swing[0,:]) * 0.25, linestyle='--', marker='o',label="LF")
            plt.plot(time, np.transpose(self.swing[1,:]) * 0.5, linestyle='--', marker='o',label="RF")
            plt.plot(time, np.transpose(self.swing[2,:]) * 0.75,linestyle='--', marker='o', label="LH")
            plt.plot(time, np.transpose(self.swing[3,:]) * 1, linestyle='--', marker='o',label="RH")
            plt.legend()
            plt.title('Swing vector: reference')
            plt.show()
        #
        if active_plots.grforces:
            plt.figure()
            plt.rcParams['axes.grid'] = True
            plt.plot(time, np.transpose(self.grForcesLFWz_gt), marker='o', label="LF")
            plt.plot(time, np.transpose(self.grForcesRFWz_gt), marker='o', label="RF")
            plt.plot(time, np.transpose(self.grForcesLHWz_gt), marker='o', label="LH")
            plt.plot(time, np.transpose(self.grForcesRHWz_gt), marker='o', label="RH")
            plt.legend()
            plt.title('Ground reaction force: reference')
            plt.show()
    
        if active_plots.feet:
            plt.figure()
            plt.rcParams['axes.grid'] = True
            plt.plot(time, np.transpose(self.footPosLFWx),linestyle='--', marker='o', label="LF")
            plt.plot(time, np.transpose(self.footPosRFWx),linestyle='--', marker='o', label="RF")
            plt.plot(time, np.transpose(self.footPosLHWx),linestyle='--', marker='o', label="LH")
            plt.plot(time,  np.transpose(self.footPosRHWx),linestyle='--', marker='o', label="RH")
            plt.legend()
            plt.title('Foot location: reference x')
            plt.show()
    
            plt.figure()
            #plt.subplot(1, 2, 1)
            plt.rcParams['axes.grid'] = True
            plt.plot(time, np.transpose(self.footPosLFWy),linestyle='--', marker='o', label="LF")
            plt.plot(time, np.transpose(self.footPosRFWy),linestyle='--', marker='o', label="RF")
            plt.plot(time, np.transpose(self.footPosLHWy),linestyle='--', marker='o', label="LH")
            plt.plot(time, np.transpose(self.footPosRHWy),linestyle='--', marker='o', label="RH")
            plt.legend()
            plt.title('Foot location: reference y')
            plt.show()
    
            plt.figure()
            # plt.subplot(1, 2, 1)
            plt.rcParams['axes.grid'] = True
            plt.plot(time, np.transpose(self.footPosLFWz),linestyle='--', marker='o', label="LF")
            plt.plot(time, np.transpose(self.footPosRFWz),linestyle='--', marker='o', label="RF")
            plt.plot(time, np.transpose(self.footPosLHWz),linestyle='--', marker='o', label="LH")
            plt.plot(time, np.transpose(self.footPosRHWz),linestyle='--', marker='o',label="RH")
            plt.legend()
            plt.title('Foot location: reference z')
            plt.show()
    
        if active_plots.com:
            plt.figure()
            plt.rcParams['axes.grid'] = True
            plt.plot(time, np.transpose(self.actual_CoMXW),label="X", marker='.', linewidth=1.)
            plt.plot(time, np.transpose(self.actual_CoMYW),label="Y",marker='.',  linewidth=2.)
            plt.plot(time, np.transpose(self.actual_CoMZW),label="Z",marker='.',  linewidth=3.)
            plt.title('COM position: reference')
            plt.legend()
            plt.show()
    
            plt.figure()
            plt.rcParams['axes.grid'] = True
            plt.plot(time, np.transpose(self.com_VxW),label="X", marker='.',  linewidth=1.)
            plt.plot(time, np.transpose(self.com_VyW),label="Y", marker='.', linewidth=2.)
            plt.plot(time, np.transpose(self.com_VzW),label="Z", marker='.', linewidth=3.)
            plt.title('COM velocity: reference')
            plt.legend()
            plt.show()
    
        if active_plots.orientation:
            plt.figure()
            plt.rcParams['axes.grid'] = True
            plt.plot(time, np.transpose(self.rollW),label="X", marker='.', linewidth=1.)
            plt.plot(time,  np.transpose(self.pitchW),label="Y",marker='.',  linewidth=2.)
            plt.plot(time, np.transpose(self.yawW),label="Z",  marker='.',linewidth=3.)
            plt.title('Orientation angular: reference')
            plt.legend()
            plt.show()
    
            plt.figure()
            plt.rcParams['axes.grid'] = True
            plt.plot(time, np.transpose(self.omegaXW),label="X",  linewidth=1.)
            plt.plot(time, np.transpose(self.omegaYW),label="Y",  linewidth=2.)
            plt.plot(time, np.transpose(self.omegaZW),label="Z",  linewidth=3.)
            plt.title('Orientation ang. velocity: reference')
            plt.legend()
            plt.show()
    
            
            
    
