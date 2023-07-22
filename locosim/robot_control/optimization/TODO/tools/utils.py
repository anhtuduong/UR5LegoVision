
from roslib.scriptutil import get_param_server
from roslib.names import ns_join, get_ros_namespace, make_caller_id
import socket

import matplotlib.pyplot as plt
import sys
import os
import numpy as np
import rospy as ros
import copy

class Utils:



    def crd(self, input):
        names = ['X', 'Y', 'Z']
        return (names.index(input))

    def sp_crd(self, input):
        names = ['LX', 'LY', 'LZ', 'AX','AY','AZ']
        return (names.index(input))

    def leg_map(self, input):
        names = ['LF', 'RF', 'LH', 'RH']
        return (names.index(input))

    def leg_name(self, leg_index):
        names = ['LF', 'RF', 'LH', 'RH']
        return (names[leg_index])

    def getSegment(self, var, index, size):
        return var[index:index+size]

    def linPart(self, var):
        index = self.sp_crd("LX")
        return var[index:index+3]

    def angPart(self, var):
        index = self.sp_crd("AX")
        return var[index:index+3]

    ########################################################################

    #manage param server

    ########################################################################



    def succeed(self,args):
        """
        Utility routine for checking ROS XMLRPC API call
        @return: value field from ROS xmlrpc call
        @rtype: XmlRpcLegalValue
        @raise ROSParamException: if call to ROS xmlrpc API did not succeed
        """
        code, msg, val = args
        if code != 1:
            raise ROSParamException(msg)
        return val

    def get_caller_id(self):
        """
        @return: caller ID for rosparam ROS client calls
        @rtype: str
        """
        return make_caller_id('rosparam-%s' % os.getpid())

    def putIntoParamServer(self, data):

        try:
            self.succeed(get_param_server().setParam(self.get_caller_id(), 'mpc/', data))
        except socket.error:
            raise ROSParamIOException("Unable to communicate with master!")
        print ("set parameter [%s] to [%s]" % ('mpc', data))
        pass

    def putIntoGlobalParamServer(self,label, data):

        try:
            ros.set_param(label, data)
        except socket.error:
            raise ROSParamIOException("Unable to communicate with master!")
        print ("set parameter [%s] into global param server" % label)
        pass

#########################################################################

    def getIdx(self, leg, coord):
        return self.leg_map(leg)*3 + self.crd(coord)

    def setLegJointState(self, legid,  input, jointState):

        jointState[self.leg_map[legid]*3:self.leg_map[legid]*3+3] = input

    def setLegJointState(self, legid,  input, jointState):
        jointState[legid*3:legid*3+3] = input

    def getLegJointState(self, legid,  jointState):
        return jointState[self.leg_map[legid]*3:self.leg_map[legid]*3+3]

    def getLegJointState(self, legid,  jointState):
        return jointState[legid*3:legid*3+3]

    def spy(self, var):
        plt.spy(var)
        plt.show()


    def detectLiftOff(self, swing,idx, leg, prediction_horizon):
        if (idx < prediction_horizon-1) and ((swing[leg, idx] == 0) and (swing[leg, idx+1] == 1)):
            return True
        else:

            return False

    def detectTouchDown(self, swing,idx, leg):
        if ((swing[leg, idx] == 1) and (swing[leg, idx+1] == 0)):

            return True
        else:

            return False

    def detectHapticTouchDown(self, grForcesW, leg, force_th):
        grfleg = self.getLegJointState(leg, grForcesW)
        if grfleg[2]>=force_th:
            return True
        else:
            return False


###################################################################### Put in class geometry to be shared with ABDO TODO

    class LineCoeff2d:
        def __init__(self):
            self.p = 0.0
            self.q = 0.0
            self.r = 0.0



    # def clockwise_sort(self, polygon):
    #
    #     vertices_number = np.size(polygon, 0)
    #     angle = np.zeros((1, vertices_number))
    #     #        print polygon, vertices_number
    #     centroidX = np.sum(polygon[:, 0])
    #     centroidX = centroidX / float(vertices_number)
    #     centroidY = np.sum(polygon[:, 1])
    #     centroidY = centroidY / float(vertices_number)
    #     #        print centroidX, centroidY
    #
    #     for j in range(0, vertices_number):
    #         angle[0, j] = np.arctan2(polygon[j, 0] - centroidX, polygon[j, 1] - centroidY)
    #
    #     index = np.array(np.argsort(angle))
    #     #        print index
    #     sorted_vertices = np.zeros((vertices_number, 2))
    #     for j in range(0, vertices_number):
    #         #            print j, index[0,j]
    #         sorted_vertices[j, :] = polygon[index[0, j], :]
    #
    #         # adding an extra point to close the polytop (last point equal to the first)
    #     sorted_vertices = np.vstack([sorted_vertices, sorted_vertices[0, :]])
    #
    #     return sorted_vertices

    """
    Computes the distance of a point from a line

    Parameters
    ----------
    v1 first 2D endpoint of linesegment
    v2 second 2D endpoint of linesegment
    p 2D point to find distance to

    Returns
    ----------
    distance of p from line
    """

    def distance_from_line(self, pt, v1, v2):
        a = v1 - v2
        b = pt - v2
        distance = np.linalg.norm(np.cross(a, b)) / np.linalg.norm(a)
        return distance

    def point_is_right_of_line(self, p0, p1, p2):

        return (p2[self.crd("X")] - p0[self.crd("X")]) *(p1[self.crd("Y")] - p0[self.crd("Y")]) \
               - (p1[self.crd("X")] - p0[self.crd("X")]) *(p2[self.crd("Y")] - p0[self.crd("Y")])

    def   clock_wise_sort(self, vertices):
        #  sort   clockwise assuming vertices are a list
        for i in range(1, len(vertices)-1):
            for j in range(i+1, len(vertices)):
                # the point p2 should always be on the right to be cwise thus if it is on the left < 0 i swap
                if (self.point_is_right_of_line(vertices[0], vertices[i], vertices[j]) < 0.0):
                    tmp = vertices[i]
                    vertices[i] = vertices[j]
                    vertices[j] = tmp

    def counter_clock_wise_sort(self, vertices):
        # sort   clockwise assuming vertices are a list
        for i in range(1, len(vertices) - 1):
            for j in range(i + 1, len(vertices)):
                # the point p2 should always be on the left of the line to be ccwise thus if it is on the right  >0  swap
                if (self.point_is_right_of_line(vertices[ 0], vertices[ i], vertices[ j]) > 0.0):
                    tmp = vertices[ i]
                    vertices[ i] = vertices[ j]
                    vertices[ j] = tmp


    def margin_from_poly(self,point_to_test,  stance_legs, actual_feetW):

        distances = []
        stance_idx = []
        stance_feetW = []

        #project test point on horizontal plane
        pointXY = copy.deepcopy(point_to_test)
        pointXY[2] = 0.0
        #compute stance indexes and collect stance feet (projectoed on horizontal plane)
        for leg in range(4):
            if (stance_legs[leg]):
                stance_idx.append(leg)
                stance_feetW.append(np.array([actual_feetW[leg][0], actual_feetW[leg][1], 0.0]))

        #sort them
        stance_feetW_sorted = copy.deepcopy(stance_feetW)
        self.clock_wise_sort(stance_feetW_sorted)

        #compute distances from lines
        for idx in range(len(stance_feetW_sorted)):
             relative_distance = self.distance_from_line(pointXY , stance_feetW_sorted[idx] , stance_feetW_sorted[(idx + 1) % len(stance_feetW_sorted)])
             # print("stance_feetW_sorted", stance_feetW_sorted[idx].transpose())
             # print("stance_feetW_sorted+1", stance_feetW_sorted[(idx + 1) % len(stance_feetW_sorted)].transpose())
             # print(relative_distance)
             distances.append(relative_distance)
        #find the minimum
        #print("distances", distances)
        margin = min(distances)
        return margin


    """
    Computes the halfplane description of the polygon

    Parameters
    ----------
    vertices: numpy array of 3D vertices

    Returns
    ----------
    A, b: Half space descriprion of the polygon (assuming created with vertex sorted in CCWise order)
    """

    def compute_half_plane_description(self, vertices):

        number_of_constraints = np.size(vertices, 0)
        # vertices_ccwise_sorted = np.zeros((number_of_constraints, 3-1))
        A = np.zeros((number_of_constraints, 2))
        b = np.zeros(number_of_constraints)

        # for vertix in range(0, number_of_constraints):
        # 	vertices_ccwise_sorted[vertix] = [vertices[vertix][0], vertices[vertix][1]]
        vertices_ccwise_sorted = vertices

        # Sort feet positions
        self.counter_clock_wise_sort(vertices_ccwise_sorted)
        print("vertices_ccwise_sorted feet : ", vertices_ccwise_sorted)

        # cycle along the ordered vertices to compute the line coeff p*xcp + q*ycp  +r  > + stability_margin
        for vertix in range(0, number_of_constraints):
            # Compute the coeffs of the line between two vertices (normal p,q pointing on the left of (P1 - P0) vector
            line_coeff = self.compute_line_coeff(vertices_ccwise_sorted[vertix],
                                                 vertices_ccwise_sorted[(vertix + 1) % number_of_constraints])


            print (vertices_ccwise_sorted[vertix], vertices_ccwise_sorted[(vertix+1)% number_of_constraints])
            if (not np.isfinite(line_coeff.p)) or (not np.isfinite(line_coeff.q)):
                print ("There are two coincident vertices in the polygon, there could be NaNs in the HP description matrix")

            A[vertix, 0] = line_coeff.p
            A[vertix, 1] = line_coeff.q
           # A[vertix, 2] = 0.0  # Z component is not considered
            b[vertix] = line_coeff.r

        return A, b



    """
    Compute the coefficients p,q of the line p*x + q*y + r = 0 (in 2D) passing through pt0 and pt1
    """

    def compute_line_coeff(self, pt0, pt1):
        ret = self.LineCoeff2d()
        ret.p = pt0[1] - pt1[1]
        ret.q = pt1[0] - pt0[0]
        ret.r = -ret.p * pt0[0] - ret.q * pt0[1]

        # Normalize the equation in order to intuitively use stability margins (?)
        norm = np.hypot(ret.p, ret.q)
        ret.p /= norm
        ret.q /= norm
        ret.r /= norm

        return ret