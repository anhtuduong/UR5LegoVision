# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 09:43:27 2018

@author: romeo orsolino
"""
import numpy as np

class Math:
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

    def rpyToRot(self, roll, pitch, yaw):
    

        
        Rx =  np.array([ [   1   ,    0     	  ,  	  0], 
                         [0   ,    np.cos(roll) ,  np.sin(roll)],
                         [0   ,    -np.sin(roll),  np.cos(roll)]]);


        Ry = np.array([[np.cos(pitch) 	,	 0  ,   -np.sin(pitch)],
              [      0       ,    1  ,   0],
              [np.sin(pitch) 	,	0   ,  np.cos(pitch)]]);
          
        
        Rz = np.array([[ np.cos(yaw)  ,  np.sin(yaw) ,		0],
                      [-np.sin(yaw) ,  np.cos(yaw) ,  		0],
                      [0      ,     0     ,       1]]);
        
        

        R =  Rx.dot(Ry.dot(Rz));
        return R

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
        

#p1 = [1,1.5]
#s1 = [0,0]
#s2 = [0,2]
#s3 = [2,2]
#s4 = [2,0]
#poly = np.vstack([s1,s2,s3,s4])
#math = Math()
##d =  math.find_point_to_line_signed_distance(s4, s1, p1)
##print d
##math = Math()
#d = math.find_residual_radius(poly, p1)
#print d
