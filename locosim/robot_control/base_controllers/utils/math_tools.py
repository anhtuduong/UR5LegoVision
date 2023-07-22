# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 09:43:27 2018

@author: romeo orsolino
"""
from __future__ import print_function

# Resolve paths
import os
import sys
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[4]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

import numpy as np
import scipy as sp
import math as math
from locosim.robot_control.base_controllers.utils.utils import Utils

class LineCoeff2d:
    def __init__(self):
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

class Math:
    def __init__(self):
        self._Tomega_mat = np.diag([0.,0.,1.])
        self._Tomega_dot_mat = np.zeros([3,3])
        self._Tomega_inv_mat = np.zeros([3,3])

    def normalize(self, n):
        norm1 = np.linalg.norm(n)
        n = np.true_divide(n, norm1)
        return n

    def skew(self, v):
        if len(v) == 4: v = v[:3]/v[3]
        skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
        return skv - skv.T

    def rotation_matrix_from_normal(self, n):
        n = n.reshape((3,))
        e_x = np.array([1., 0., 0.])
        t = e_x - np.dot(e_x, n) * n
        t = t / np.linalg.norm(t)
        b = np.cross(n, t)
        return np.vstack([t, b, n]).T

    def getGraspMatrix(self, r):
        math = Math()
        G = block([[np.eye(3), np.zeros((3, 3))],
                       [math.skew(r), np.eye(3)]])
        return G    

    def plane_z_intercept(self, point_on_plane, plane_normal):
        return point_on_plane[2] + \
               plane_normal[0] / plane_normal[2] * point_on_plane[0] + \
               plane_normal[1] / plane_normal[2] * point_on_plane[1]

    def compute_z_component_of_plane(self, xy_components, plane_normal, z_intercept):
        return -plane_normal[0]/plane_normal[2]*xy_components[0] - \
               plane_normal[1]/plane_normal[2]*xy_components[1] + z_intercept
    # from the rpy angles into ZYX configuration returns b_R_w
    def rpyToRot(self, *args):
        if len(args) == 3:  # equivalent to rpyToRot(self, roll, pitch, yaw)
            roll = args[0]
            pitch = args[1]
            yaw = args[2]
        elif len(args) == 1:  # equivalent to rpyToRot(self, roll_pitch_yaw)
            roll = args[0][0]
            pitch = args[0][1]
            yaw = args[0][2]
        else:
            print('Wrong number of arguments')
            return

        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), np.sin(roll)],
                       [0, -np.sin(roll), np.cos(roll)]])

        Ry = np.array([[np.cos(pitch), 0, -np.sin(pitch)],
                       [0, 1, 0],
                       [np.sin(pitch), 0, np.cos(pitch)]])

        Rz = np.array([[np.cos(yaw), np.sin(yaw), 0],
                       [-np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])

        R = Rx.dot(Ry.dot(Rz))
        return R
                                
    # the dual of rpyToRot()
     # set of Euler angles (according to ZYX convention) representing the orientation of frame represented by b_R_w
    def rotTorpy(self, b_R_w):
        rpy = np.array([0.0,0.0,0.0])                    
        rpy[0] = np.arctan2(b_R_w[1,2], b_R_w[2,2])
        rpy[1] = -np.arcsin( b_R_w[0,2])
        rpy[2] = np.arctan2(b_R_w[0,1], b_R_w[0,0])
    
        return rpy;
                
    # dual of eul2Rot from w_R_b returns the rpy angles into ZYX configuration
    def rot2eul(self, R):
        phi = np.arctan2(R[1,0], R[0,0])
        theta = np.arctan2(-R[2,0], np.sqrt(pow(R[2,1],2) + pow(R[2,2],2) ))
        psi = np.arctan2(R[2,1], R[2,2])
       
        #unit test should return roll = 0.5 pitch = 0.2  yaw = 0.3
        # rot2eul(np.array([ [0.9363,   -0.1684,    0.3082], [0.2896 ,   0.8665  , -0.4065], [-0.1987 ,   0.4699  ,  0.8601]]))    
        
        # returns roll = psi, pitch = theta,  yaw = phi
        return np.array((psi, theta, phi))

    #  from w_R_b returns the rpy angles into XYZ configuration about fixed axes

    def rot2eulFixed(self, R):
        phi = np.arctan2(R[1, 0], R[0, 0])
        theta = np.arctan2(-R[2, 0], np.sqrt(pow(R[2, 1], 2) + pow(R[2, 2], 2)))
        psi = np.arctan2(R[2, 1], R[2, 2])

        # unit test should return roll = 0.5 pitch = 0.2  yaw = 0.3
        # rot2eul(np.array([ [0.9363,   -0.1684,    0.3082], [0.2896 ,   0.8665  , -0.4065], [-0.1987 ,   0.4699  ,  0.8601]]))

        # returns roll = psi, pitch = theta,  yaw = phi
        return np.array((psi, theta, phi))
            
    # from the rpy angles into ZYX configuration returns w_R_b                            
    def eul2Rot(self, rpy):
        c_roll =  np.cos(rpy[0])
        s_roll = np.sin(rpy[0])
        c_pitch =      np.cos(rpy[1])        
        s_pitch = np.sin(rpy[1])
        c_yaw = np.cos(rpy[2])
        s_yaw = np.sin(rpy[2])
                                
        Rx =  np.array([ [   1   ,         0           ,        0], 
                         [   0   ,        c_roll  ,  -s_roll],
                         [   0   ,      s_roll,      c_roll ]]);


        Ry = np.array([[c_pitch     ,     0  ,   s_pitch],
                       [      0       ,    1  ,   0],
                       [ -s_pitch     ,    0   ,  c_pitch]]);
          
        
        Rz = np.array([[ c_yaw  ,  -s_yaw ,        0],
                      [  s_yaw ,  c_yaw ,          0],
                      [0      ,     0     ,       1]]);
        


        R =  Rz.dot(Ry.dot(Rx));
        return R

    """
        Computes the mapping between euler rates  and angular acceleration vector (expressed in the world frame), of the rotating frame
        w_omega_dot = T_omega(rpy) * euler_rates_dot + T_omega_dot(rpy, rpyd)*euler_rates  
        
        Parameters
        ----------
        rpy vector of Euler Angles that describes the orientation of the rotating frame (we assume the ZYX convention). 
        Note,  the coordinates of the input vector are expressed as XYZ (roll, pitch, yaw)!
        
        rpyd vector of Euler Angles rates. Note, we assume the ZYX convention but the coordinates of the input vector are expressed as XYZ (roll, pitch, yaw)!
        
        Returns
        ----------
        3x3 matrix T_omega_dot
    """


    def Tomega_dot(self, rpy, rpyd):
    
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]
        rolld = rpyd[0]
        pitchd = rpyd[1]
        yawd = rpyd[2]
    
        # Tomega_dot = np.array([[ -np.cos(yaw)*np.sin(pitch)*pitchd - np.cos(pitch)*np.sin(yaw)*yawd,  -np.cos(yaw)*yawd, 0],
        #                       [ np.cos(yaw)*np.cos(pitch)*yawd - np.sin(yaw)*np.sin(pitch)*pitchd,    -np.sin(yaw)*yawd, 0  ],
        #                       [ -np.cos(pitch)*pitchd,  0, 0 ]])
        #
        #
        # return Tomega_dot

        # faster way
        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        self._Tomega_dot_mat[0, 0] = -sp * cy * pitchd - cp * sy * yawd
        self._Tomega_dot_mat[1, 0] = -sp * sy * pitchd + cp * cy * yawd
        self._Tomega_dot_mat[2, 0] = -cp*pitchd

        self._Tomega_dot_mat[0, 1] = -cy * yawd
        self._Tomega_dot_mat[1, 1] = -sy * yawd
        
        return self._Tomega_dot_mat

    """
        Computes the mapping matrix between euler rates and angular velocity vector (expressed in the world frame), of the rotating frame
        w_omega= T_omega(rpy) * euler_rates
        
        Parameters
        ----------
        rpy vector of Euler Angles that describes the orientation of the rotating frame (we assume the ZYX convention). 
        Note,  the coordinates of the input vector are expressed as XYZ (roll, pitch, yaw)!
        
        Returns
        ----------
        3x3 matrix T_omega
    """
    def Tomega(self, rpy):
    
        #convention yaw pitch roll
    
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]
        

        # Tomega = np.array([[np.cos(pitch)*np.cos(yaw),       -np.sin(yaw),                    0],
        #                    [ np.cos(pitch)*np.sin(yaw),       np.cos(yaw),                    0],
        #                    [ -np.sin(pitch),                           0 ,                    1]])
        # return Tomega

        # faster way

        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        self._Tomega_mat[0, 0] = cp * cy
        self._Tomega_mat[1, 0] = cp * sy
        self._Tomega_mat[2, 0] = -sp
        self._Tomega_mat[0, 1] = -sy
        self._Tomega_mat[1, 1] = cy

        return self._Tomega_mat

    def Tomega_inv(self, rpy):

        # convention yaw pitch roll

        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]

        # Tomega_inv = 1/np.cos(pitch) * np.array([[                np.cos(yaw),                np.sin(yaw),             0],
        #                                          [ -np.cos(pitch)*np.sin(yaw),  np.cos(pitch)*np.cos(yaw),             0],
        #                                          [ -np.sin(pitch)*np.cos(yaw), -np.sin(pitch)*np.sin(yaw), np.cos(pitch)]])
        # return Tomega

        # faster way

        cp = np.cos(pitch)
        sp = np.sin(pitch)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        self._Tomega_inv_mat[0, 0] = cy
        self._Tomega_inv_mat[1, 0] = -cp * sy
        self._Tomega_inv_mat[2, 0] = -sp * cy
        self._Tomega_inv_mat[0, 1] = sy
        self._Tomega_inv_mat[1, 1] = cp * cy
        self._Tomega_inv_mat[2, 1] = -sp * sy
        self._Tomega_inv_mat[2, 2] = cp

        return self._Tomega_inv_mat/cp

    ##################
    # Geometry Utils
    ###################

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
        utils = Utils()
        return (p2[utils.crd["X"]] - p0[utils.crd["X"]]) * (p1[utils.crd["Y"]] - p0[utils.crd["Y"]]) \
               - (p1[utils.crd["X"]] - p0[utils.crd["X"]]) * (p2[utils.crd["Y"]] - p0[utils.crd["Y"]])

    def clock_wise_sort(self, vertices):
        #  sort   clockwise assuming vertices are a list
        for i in range(1, len(vertices) - 1):
            for j in range(i + 1, len(vertices)):
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
                if (self.point_is_right_of_line(vertices[0], vertices[i], vertices[j]) > 0.0):
                    tmp = vertices[i]
                    vertices[i] = vertices[j]
                    vertices[j] = tmp

    def margin_from_poly(self, point_to_test, stance_legs, actual_feetW):

        distances = []
        stance_idx = []
        stance_feetW = []

        # project test point on horizontal plane
        pointXY = np.copy(point_to_test)
        pointXY[2] = 0.0
        # compute stance indexes and collect stance feet (projectoed on horizontal plane)
        for leg in range(4):
            if (stance_legs[leg]):
                stance_idx.append(leg)
                stance_feetW.append(np.array([actual_feetW[leg][0], actual_feetW[leg][1], 0.0]))

        # sort them
        stance_feetW_sorted = np.copy(stance_feetW)
        self.clock_wise_sort(stance_feetW_sorted)

        # compute distances from lines
        for idx in range(len(stance_feetW_sorted)):
            relative_distance = self.distance_from_line(pointXY, stance_feetW_sorted[idx],
                                                        stance_feetW_sorted[(idx + 1) % len(stance_feetW_sorted)])
            # print("stance_feetW_sorted", stance_feetW_sorted[idx].transpose())
            # print("stance_feetW_sorted+1", stance_feetW_sorted[(idx + 1) % len(stance_feetW_sorted)].transpose())
            # print(relative_distance)
            distances.append(relative_distance)
        # find the minimum
        # print("distances", distances)
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

            print(vertices_ccwise_sorted[vertix], vertices_ccwise_sorted[(vertix + 1) % number_of_constraints])
            if (not np.isfinite(line_coeff.p)) or (not np.isfinite(line_coeff.q)):
                print(
                    "There are two coincident vertices in the polygon, there could be NaNs in the HP description matrix")

            A[vertix, 0] = line_coeff.p
            A[vertix, 1] = line_coeff.q
            # A[vertix, 2] = 0.0  # Z component is not considered
            b[vertix] = line_coeff.r

        return A, b

    """
    Compute the coefficients p,q of the line p*x + q*y + r = 0 (in 2D) passing through pt0 and pt1
    """

    def compute_line_coeff(self, pt0, pt1):
        ret = LineCoeff2d()
        ret.p = pt0[1] - pt1[1]
        ret.q = pt1[0] - pt0[0]
        ret.r = -ret.p * pt0[0] - ret.q * pt0[1]

        # Normalize the equation in order to intuitively use stability margins (?)
        norm = np.hypot(ret.p, ret.q)
        ret.p /= norm
        ret.q /= norm
        ret.r /= norm

        return ret

    def line(self, p1, p2):
        A = (p1[1] - p2[1])
        B = (p2[0] - p1[0])
        C = (p1[0]*p2[1] - p2[0]*p1[1])
        return A, B, -C

    def two_lines_intersection(self, L1, L2):
        D  = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return x,y
        else:
            return False

    def is_point_inside_segment(self, first_input_point, second_input_point, point_to_check):
        epsilon = 0.001

        if (np.abs(first_input_point[0] - second_input_point[0]) < 1e-02):                     
            alpha = (point_to_check[1] - second_input_point[1]) / (first_input_point[1] - second_input_point[1]) 
        else:
            alpha = (point_to_check[0] - second_input_point[0]) / (first_input_point[0] - second_input_point[0])                 

        if(alpha>=-epsilon)&(alpha<=1.0+epsilon):
            new_point = point_to_check
        else:
            new_point = False

        return new_point, alpha

    def find_point_to_line_signed_distance(self, segment_point1, segment_point2, point_to_check):
        # this function returns a positive distance if the point is on the right side of the segment. This will return 
        # a positive distance for a polygon queried in clockwise order and with a point_to_check which lies inside the polygon itself 
        num = (segment_point2[0] - segment_point1[0])*(segment_point1[1] - point_to_check[1]) - (segment_point1[0] - point_to_check[0])*(segment_point2[1] - segment_point1[1])
        denum_sq = (segment_point2[0] - segment_point1[0])*(segment_point2[0] - segment_point1[0]) + (segment_point2[1] - segment_point1[1])*(segment_point2[1] - segment_point1[1])
        dist = num/np.sqrt(denum_sq)
#        print segment_point1, segment_point2, point_to_check, dist
        return dist

    def find_residual_radius(self, polygon, point_to_check):
        # this function returns a positive distance if the point is on the right side of the segment. This will return 
        # a positive distance for a polygon queried in clockwise order and with a point_to_check which lies inside the polygon itself 
        # print 'poly',polygon
        numberOfVertices = np.size(polygon,0)
        # print 'polygon in residual radius computation', polygon
        # print 'number of vertices', numberOfVertices
        residual_radius = 1000000.0
        for i in range(0,numberOfVertices-1):
            s1 = polygon[i,:]
            s2 = polygon[i+1,:]
            # print s1, s2, point_to_check
            d_temp = self.find_point_to_line_signed_distance(s1, s2, point_to_check)
#            print i, s1, s2, d_temp
            if d_temp < 0.0:
                print ('Warning! found negative distance. Polygon might not be in clockwise order...')
            elif d_temp < residual_radius:
                residual_radius = d_temp

        # we dont need to compute for the last edge cause we added an extra point to close the polytop (last point equal to the first)
        
#        print polygon[numberOfVertices-1,:], polygon[0,:], d_temp
        return residual_radius

    def find_polygon_segment_intersection(self, vertices_input, desired_direction, starting_point):
        desired_direction = desired_direction/np.linalg.norm(desired_direction)*10.0
        #print "desired dir: ", desired_direction

        desired_com_line = self.line(starting_point, starting_point+desired_direction)
        #print "des line : ", desired_com_line
        tmp_vertices = np.vstack([vertices_input, vertices_input[0]])
        intersection_points = np.zeros((0,2))
        points_along_direction = np.zeros((0,2))
        point_to_com_distance = np.zeros((0,1))

        for i in range(0,len(vertices_input)):
            v1 = tmp_vertices[i,:]
            v2 = tmp_vertices[i+1,:]
            actuation_region_edge = self.line(v1, v2)
            #print desired_com_line, actuation_region_edge
            new_point = self.two_lines_intersection(desired_com_line, actuation_region_edge)

            if new_point:
                intersection_points = np.vstack([intersection_points, new_point])
                new_point, alpha = self.is_point_inside_segment(starting_point, starting_point+desired_direction, new_point)
                if new_point:
                    points_along_direction = np.vstack([points_along_direction, new_point])
                    d = np.sqrt((new_point[0] - starting_point[0])*(new_point[0] - starting_point[0]) + (new_point[1] - starting_point[1])*(new_point[1] - starting_point[1]))
                    point_to_com_distance = np.vstack([point_to_com_distance, d])

            else:
                print( "lines are parallel!")
                #while new_point is False:
                #    desired_com_line = self.line(starting_point, starting_point+desired_direction)
                #    new_point = self.two_lines_intersection(desired_com_line, actuation_region_edge)
                #    intersection_points = np.vstack([intersection_points, new_point])
                #    print new_point

        #print points_along_direction, point_to_com_distance
        idx = np.argmin(point_to_com_distance)
        final_point = points_along_direction[idx,:]
        #print points_along_direction, point_to_com_distance, idx
        return final_point, intersection_points
        

def cross_mx(v):
    mx =np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return mx
def cross_mx_casadi(v):
    # mx =[[MX(0.0), -v[2], v[1]], [v[2], MX(0.0), -v[0]], [-v[1], v[0], MX(0.0)]]
    mx =MX.zeros(3,3)
    mx[0, 1] =  -v[2]
    mx[0, 2] =   v[1]
    mx[1, 0] =   v[2]
    mx[1, 2] =  -v[0]
    mx[2, 0] =  -v[1]
    mx[2, 1] =   v[0]
    return mx
def skew_simToVec(Ra):
    # This is similar to one implemented in the framework
    v = np.zeros(3)
    v[0] = 0.5*(Ra[2,1] - Ra[1,2])
    v[1] = 0.5*(Ra[0,2] - Ra[2,0])
    v[2] = 0.5*(Ra[1,0] - Ra[0,1])

    return v

# this function computes the orientation error (expressed in the world frame) to drive the frame  is expressed in the end-effector frame
def computeOrientationError(w_R_e, w_R_des):
    #This has a bug
    # c = 0.5 * (Ra[0, 0] + Ra[1, 1] + Ra[2, 2] - 1)
    # w = -skew_simToVec(Ra)
    # s = np.linalg.norm(w) # w = sin(theta) * axis
    #
    # if abs(s) <= 1e-10:
    #     err = np.zeros(3)
    # else:
    #     angle = math.atan2(s, c)
    #     axis = w / s
    #     err = angle * axis
    # return err

    # compute the relative rotation to move w_R_b into w_R_des
    #w_R_des = w_R_b * b_R_des  => b_R_des = w_R_b.T * w_R_des
    e_R_des = w_R_e.T.dot(w_R_des)

    # compute the angle-axis representation of the associated orientation error
    # compute the angle: method 1) with arc cos
    arg = (e_R_des[0, 0] + e_R_des[1, 1] + e_R_des[2, 2] - 1) / 2

    # compute the angle: method 2) with atan2
    delta_theta = math.atan2(np.sqrt(
        pow(e_R_des[2, 1] - e_R_des[1, 2], 2) + pow(e_R_des[0, 2] - e_R_des[2, 0], 2) + pow(
            e_R_des[1, 0] - e_R_des[0, 1], 2)), e_R_des[0, 0] + e_R_des[1, 1] + e_R_des[2, 2] - 1)
    # compute the axis (deal with singularity)
    if delta_theta == 0.0:
        e_error_o = np.zeros(3)
    else:
        r_hat = 1 / (2 * np.sin(delta_theta)) * np.array(
            [e_R_des[2, 1] - e_R_des[1, 2], e_R_des[0, 2] - e_R_des[2, 0], e_R_des[1, 0] - e_R_des[0, 1]])
        # compute the orientation error
        e_error_o = delta_theta * r_hat
    #    # the error is expressed in the end-effector frame
    #    # we need to map it in the world frame to compute the moment because the jacobian is in the WF
    w_error_o = w_R_e.dot(e_error_o)
    return w_error_o

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> np.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (np.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not np.allclose(R0, R1): print axes, "failed"

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence.

    >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
    >>> np.allclose(angles, [0.123, 0, 0])
    True

    """
    return euler_from_matrix(quaternion_matrix(quaternion), axes)

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> np.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)


def MxInv(A):
    # Determinant of matrix A
    sb1 = A[0, 0] * ((A[1, 1] * A[2, 2]) - (A[1, 2] * A[2, 1]))
    sb2 = A[0, 1] * ((A[1, 0] * A[2, 2]) - (A[1, 2] * A[2, 0]))
    sb3 = A[0, 2] * ((A[1, 0] * A[2, 1]) - (A[1, 1] * A[2, 0]))

    Adetr = sb1 - sb2 + sb3
    #    print(Adetr)
    # Transpose matrix A
    TransA = A.T

    # Find determinant of the minors
    a01 = (TransA[1, 1] * TransA[2, 2]) - (TransA[2, 1] * TransA[1, 2])
    a02 = (TransA[1, 0] * TransA[2, 2]) - (TransA[1, 2] * TransA[2, 0])
    a03 = (TransA[1, 0] * TransA[2, 1]) - (TransA[2, 0] * TransA[1, 1])

    a11 = (TransA[0, 1] * TransA[2, 2]) - (TransA[0, 2] * TransA[2, 1])
    a12 = (TransA[0, 0] * TransA[2, 2]) - (TransA[0, 2] * TransA[2, 0])
    a13 = (TransA[0, 0] * TransA[2, 1]) - (TransA[0, 1] * TransA[2, 0])

    a21 = (TransA[0, 1] * TransA[1, 2]) - (TransA[1, 1] * TransA[0, 2])
    a22 = (TransA[0, 0] * TransA[1, 2]) - (TransA[0, 2] * TransA[1, 0])
    a23 = (TransA[0, 0] * TransA[1, 1]) - (TransA[0, 1] * TransA[1, 0])

    # Inverse of determinant
    invAdetr = (float(1) / Adetr)
    #    print(invAdetr)
    # Inverse of the matrix A
    invA = MX.zeros(3,3)
    invA[0, 0] = invAdetr * a01
    invA[0, 1] = -invAdetr * a02
    invA[0, 2] = invAdetr * a03

    invA[0, 0] = -invAdetr * a11
    invA[0, 1] = invAdetr * a12
    invA[0, 2] = -invAdetr * a13

    invA[0, 0] = invAdetr * a21
    invA[0, 1] = -invAdetr * a22
    invA[0, 2] = invAdetr * a23

    # Return the matrix
    return invA


#/**
#brief motionVectorTransform Tranforms twists from A to B (b_X_a)   \in R^6 \times 6
#here A is the origin frame and B the destination frame.
#param position coordinate vector expressing OaOb in A coordinates
#param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
#return
#
def motionVectorTransform(position, rotationMx):
    utils = Utils()
    b_X_a = np.zeros((6,6))
    b_X_a[utils.sp_crd["AX"]:utils.sp_crd["AX"]+ 3 ,   utils.sp_crd["AX"]: utils.sp_crd["AX"] + 3] = rotationMx
    b_X_a[utils.sp_crd["LX"]:utils.sp_crd["LX"] + 3 ,   utils.sp_crd["AX"]: utils.sp_crd["AX"] + 3] = -rotationMx*cross_mx(position)
    b_X_a[utils.sp_crd["LX"]:utils.sp_crd["LX"] + 3 ,   utils.sp_crd["LX"]: utils.sp_crd["LX"] + 3] = rotationMx
    return b_X_a


#
#brief forceVectorTransform Tranforms wrenches from A to B (b_X_a)   \in R^6 \times 6
#here A is the origin frame and B the destination frame.
#param position coordinate vector expressing OaOb in A coordinates
#param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
#return
#


def forceVectorTransform(position, rotationMx):
    utils = Utils()
    b_X_a = np.zeros((6,6))
    b_X_a[utils.sp_crd("AX"):utils.sp_crd("AX") + 3 ,   utils.sp_crd("AX"): utils.sp_crd("AX") + 3] = rotationMx
    b_X_a[utils.sp_crd("AX"):utils.sp_crd("AX") + 3 ,   utils.sp_crd("LX"): utils.sp_crd("LX") + 3] = -rotationMx*cross_mx(position)
    b_X_a[utils.sp_crd("LX"):utils.sp_crd("LX") + 3 ,   utils.sp_crd("LX"): utils.sp_crd("LX") + 3] = rotationMx
    return b_X_a


def polynomialRef(x0, xf, v0, vf, a0, af, T):
    # 7-th order polynomial (impose zero jerk at boundary)

    M = np.array([[1, 0,      0,        0,           0,           0,            0,            0],
                  [1, T, T ** 2,   T ** 3,      T ** 4,      T ** 5,       T ** 6,       T ** 7],
                  [0, 1,      0,        0,           0,           0,            0,            0],
                  [0, 1,  2 * T, 3*T ** 2,    4*T ** 3,    5*T ** 4,     6*T ** 5,     7*T ** 6],
                  [0, 0,      2,        0,           0,           0,            0,            0],
                  [0, 0,      2,    6 * T, 12 * T ** 2, 20 * T ** 3,    30*T ** 4,    42*T ** 5],
                  [0, 0,      0,        6,           0,           0,            0,            0],
                  [0, 0,      0,        6,      24 * T, 60 * T ** 2, 120 * T ** 3, 210 * T ** 4],
                  ])

    j0 = np.zeros_like(x0)
    jf = np.zeros_like(xf)
    boundary_conds = np.vstack([x0, xf, v0, vf, a0, af, j0, jf])

    p_coeffs = np.linalg.inv(M) @ boundary_conds
    v_coeffs = np.array([p_coeffs[1], 2 * p_coeffs[2],3 * p_coeffs[3], 4 * p_coeffs[4], 5 * p_coeffs[5], 6 * p_coeffs[6], 7 * p_coeffs[7]])
    a_coeffs = np.array([v_coeffs[1], 2 * v_coeffs[2],3 * v_coeffs[3], 4 * v_coeffs[4], 5 * v_coeffs[5], 6 * v_coeffs[6]])

    pos = lambda t: (p_coeffs[0] +
                     p_coeffs[1] * t +
                     p_coeffs[2] * t ** 2 +
                     p_coeffs[3] * t ** 3 +
                     p_coeffs[4] * t ** 4 +
                     p_coeffs[5] * t ** 5 +
                     p_coeffs[6] * t ** 6 +
                     p_coeffs[7] * t ** 7) if 0 <= t <= T else x0 if t <0 else xf

    vel = lambda t: (v_coeffs[0] +
                     v_coeffs[1] * t +
                     v_coeffs[2] * t ** 2 +
                     v_coeffs[3] * t ** 3 +
                     v_coeffs[4] * t ** 4 +
                     v_coeffs[5] * t ** 5 +
                     v_coeffs[6] * t ** 6) if 0 <= t <= T else v0 if t <0 else vf

    acc = lambda t: (a_coeffs[0] +
                     a_coeffs[1] * t +
                     a_coeffs[2] * t ** 2 +
                     a_coeffs[3] * t ** 3 +
                     a_coeffs[4] * t ** 4 +
                     a_coeffs[5] * t ** 5) if 0 <= t <= T else a0 if t <0 else af

    return pos, vel, acc





