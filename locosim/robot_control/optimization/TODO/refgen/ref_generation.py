import rospy as ros
import numpy as np

from reference_generator.msg import com_state
from reference_generator.msg import leg_contact_state
from reference_generator.msg import  leg_trajectory
from geometry_msgs.msg import Point
from reference_generator.srv import request_reference
from reference_generator.srv import request_referenceResponse
from reference_generator.srv import request_referenceRequest
from tools.utils import Utils
from tools.math_tools import Math

import matplotlib.pyplot as plt
import matplotlib

#prevent creating pyc files
import sys
sys.dont_write_bytecode = True

class ReferenceGenerator:
    
    def __init__(self):

        ros.wait_for_service("/mpc/reference_generator/request_reference")
        self.response = request_referenceResponse()
        self.u = Utils()

    def loadConfig(self):
        self.prediction_horizon = ros.get_param("/mpc/prediction_horizon")
        self.reference_length = self.prediction_horizon + 1
        self.Ts = ros.get_param("/mpc/time_resolution")


    def getReferenceData(self, current_swing, swing_counter, robotHeight, actual_com, actual_orient,
                         desired_lin_velocity, desired_heading_velocity, actual_feetW, actual_delta_step, ref_counter):

        math = Math()
        ref_request = ros.ServiceProxy("/mpc/reference_generator/request_reference", request_reference)
        # prepare request
        # start filling  request message

        req = request_referenceRequest()

        #intial com position
        req.com_initial_position.x = actual_com[self.u.crd("X")]
        req.com_initial_position.y = actual_com[self.u.crd("Y")]
        req.com_initial_position.z = actual_com[self.u.crd("Z")]

        # intial com orientation
        req.com_initial_orientation.x = actual_orient[self.u.crd("X")]
        req.com_initial_orientation.y = actual_orient[self.u.crd("Y")]
        req.com_initial_orientation.z = actual_orient[self.u.crd("Z")]

         # desired velocity commands (linear and heading)
        req.desired_command.x = desired_lin_velocity[self.u.crd("X")]
        req.desired_command.y = desired_lin_velocity[self.u.crd("Y")]
        req.desired_command.z = desired_heading_velocity

        #intial feet position
        feet = [Point(), Point(), Point(), Point()]
        feet[self.u.leg_map("LF")].x = actual_feetW[self.u.leg_map("LF"), self.u.crd("X")]
        feet[self.u.leg_map("LF")].y = actual_feetW[self.u.leg_map("LF"), self.u.crd("Y")]
        feet[self.u.leg_map("LF")].z = actual_feetW[self.u.leg_map("LF"), self.u.crd("Z")]
        feet[self.u.leg_map("RF")].x = actual_feetW[self.u.leg_map("RF"), self.u.crd("X")]
        feet[self.u.leg_map("RF")].y = actual_feetW[self.u.leg_map("RF"), self.u.crd("Y")]
        feet[self.u.leg_map("RF")].z = actual_feetW[self.u.leg_map("RF"), self.u.crd("Z")]
        feet[self.u.leg_map("LH")].x = actual_feetW[self.u.leg_map("LH"), self.u.crd("X")]
        feet[self.u.leg_map("LH")].y = actual_feetW[self.u.leg_map("LH"), self.u.crd("Y")]
        feet[self.u.leg_map("LH")].z = actual_feetW[self.u.leg_map("LH"), self.u.crd("Z")]
        feet[self.u.leg_map("RH")].x = actual_feetW[self.u.leg_map("RH"), self.u.crd("X")]
        feet[self.u.leg_map("RH")].y = actual_feetW[self.u.leg_map("RH"), self.u.crd("Y")]
        feet[self.u.leg_map("RH")].z = actual_feetW[self.u.leg_map("RH"), self.u.crd("Z")]
        req.initial_feet_positions = feet

        req.robot_height = robotHeight
        # initial swing state
        req.current_swing = current_swing
        req.swing_counter = swing_counter

        req.actual_delta_step[0] = actual_delta_step[current_swing][0]
        req.actual_delta_step[1] = actual_delta_step[current_swing][1]

        #how much is elapsed from last replanning (important to keep gait scheduler syncronized)
        req.ref_counter = ref_counter

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
        leg_state = self.response.swing_legs[self.u.leg_map(leg)]
        # python3 transform the map into a list to make it callable
        datas = list(leg_state.data)
        for i in range(0, self.reference_length):
            temp.append(datas[i])
        return temp

    def get_footPosW(self, leg, coord):
        temp = []
        for i in range(0, self.reference_length):
            temp.append(eval(" self.response.desired_feet[self.u.leg_map(leg)].data[i]."+coord))
        return temp

    def get_normal(self, leg, coord):
        temp = []
        for i in range(0, self.reference_length):
            temp.append(eval(" self.response.terrain_normals[self.u.leg_map(leg)].data[i]."+coord))
        
        return temp

    def get_grForcesW(self, leg, coord):
        temp = []
        for i in range(0, self.reference_length):
            temp.append(eval(" self.response.grforces[self.u.leg_map(leg)].data[i]."+coord))
        return temp


        
        

