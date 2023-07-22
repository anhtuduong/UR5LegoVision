
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import sys

# Resolve paths
from pathlib import Path
FILE = Path(__file__).resolve()
ROOT = FILE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH

import pinocchio
from pinocchio.utils import *
import yaml
import math

from locosim.robot_control.base_controllers.utils.utils import Utils

class robotKinematics():
    def __init__(self, robot, ee_frames):
        self.u = Utils()	
        self.robot = robot

        self.ik_success = False
        self.urdf_feet_names = ee_frames

        # Get feet frame names in an alphabatical order to match pinocchio kinematics
        self.urdf_feet_names_pinocchio = []
        for frame in self.robot.model.frames:
               
            if frame.name in ee_frames:
                self.urdf_feet_names_pinocchio.append(frame.name)   
        
    def getBlockIndex(self, frame_name):
        for i in range(len(self.urdf_feet_names_pinocchio)):
            if frame_name == self.urdf_feet_names_pinocchio[i]:
                idx = i * 3
                break

        return idx


    '''
    This function is a 6 DoFs manipulator, the input is the endeffector position
    The end-effector position should be specified in the BASE FRAME
    The output is the set of joints relative to the end-effector position
    The redundancy (you are not providying an orientation) is solved by setting a postural task configuration
    '''
    def endeffectorInverseKinematicsLineSearch(self, ee_pos_des, frame_name, q0=np.zeros(6), verbose=False, 
                                               use_error_as_termination_criteria = False, 
                                               postural_task = True, 
                                               w_postural = 0.001, 
                                               q_postural = np.zeros(6),
                                               wrap = False):

        # Error initialization
        e_bar = 1
        niter = 0
        # Recursion parameters
        epsilon = 1e-06  # Tolerance
        # alpha = 0.1
        alpha = 1  # Step size
        lambda_ = 0.0000001  # Damping coefficient for pseudo-inverse
        max_iter = 200 # Maximum number of iterations

        # For line search only
        gamma = 0.5
        beta = 0.5
        out_of_workspace  = False

        # Inverse kinematics with line search
        while True:
            # compute foot position
            self.robot.computeAllTerms(q0, np.zeros(6))
            ee_pos0 = self.robot.framePlacement(q0, self.robot.model.getFrameId(frame_name)).translation
        
            # get the square matrix jacobian that is smaller (3x6)
            J_ee = self.robot.frameJacobian(q0, self.robot.model.getFrameId(frame_name), True, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,:]
           
            # compute newton step              
            if not postural_task:
                # computed error wrt the des cartesian position               
                e_bar = ee_pos_des - ee_pos0
                Jpinv = (np.linalg.inv(  J_ee.T.dot(J_ee)  + lambda_ * np.identity(J_ee.shape[1]) )).dot(J_ee.T)
                grad = J_ee.T.dot(e_bar)
                dq = Jpinv.dot(e_bar)
            else:
                # computed error wrt the des cartesian position
                e_bar = np.hstack((ee_pos_des - ee_pos0, w_postural*(q_postural - q0) ))               
                Je = np.vstack((J_ee, w_postural*np.identity(6)))                                
                grad = Je.T.dot(e_bar)
                dq = (np.linalg.inv(  J_ee.T.dot(J_ee)  + pow(w_postural,2)* np.identity(6) )).dot(grad)
                
            if (use_error_as_termination_criteria):
                print("ERROR",np.linalg.norm(e_bar))
                if np.linalg.norm(e_bar) < epsilon:
                    IKsuccess = True
                    if verbose:
                        print("IK Convergence achieved!, norm(error) :", np.linalg.norm(e_bar))
                        print("Inverse kinematics solved in {} iterations".format(niter))
                    break
            else:
                if np.linalg.norm(grad) < epsilon:
                    IKsuccess = True
                    if verbose:
                        print("IK Convergence achieved!, norm(grad) :", np.linalg.norm(grad))
                        print("Inverse kinematics solved in {} iterations".format(niter))
                        if np.linalg.norm(e_bar)> 0.1:
                            print("THE END EFFECTOR POSITION IS OUT OF THE WORKSPACE, norm(error) :", np.linalg.norm(e_bar))
                            out_of_workspace  = True
                    break

            if niter >= max_iter:
                if verbose:
                    print("\n Warning: Max number of iterations reached, the iterative algorithm has not reached convergence to the desired precision. Error is: ",
                          np.linalg.norm(e_bar))
                IKsuccess = False
                break

            #line search
            while True:
                if verbose:
                    print("Iter #:", niter)
                # Update
                q1 = q0 + dq * alpha
                self.robot.computeAllTerms(q1, np.zeros(6))
                ee_pos1 =  self.robot.framePlacement(q1, self.robot.model.getFrameId(frame_name)).translation

                if not postural_task:
                    # Compute error of next step
                    e_bar_new = ee_pos_des - ee_pos1
                else: 
                    e_bar_new = np.hstack((ee_pos_des - ee_pos1,w_postural*( q_postural - q1) ))
               
                # print "e_bar_new", np.linalg.norm(e_bar_new), "e_bar", np.linalg.norm(e_bar)
                error_reduction = np.linalg.norm(e_bar) - np.linalg.norm(e_bar_new)

                threshold = 0.0  # even more strict: gamma*alpha*np.linalg.norm(e_bar)
                if error_reduction < threshold:
                    alpha = beta * alpha
                    if verbose:
                        print(" line search: alpha: ", alpha)
                else:
                    q0 = q1
                    alpha = 1
                    break
            niter += 1

        # wrapping prevents from outputs larger than 2pi
        if (wrap):
            for i in range(len(q0)):
                while q0[i] >= 2 * math.pi:
                    q0[i] -= 2 * math.pi
                while q0[i] < -2 * math.pi:
                    q0[i] += 2 * math.pi

        return q0, IKsuccess, out_of_workspace

    def errorInSO3(self, R_e, R_e_des):
        error = pinocchio.log3(R_e.T.dot(R_e_des))
        return error

    '''
      This function is a 6 DoFs manipulator, the input is the endeffector position AND orientation
      The end-effector position should be specified in the BASE FRAME
      The output is the set of joints relative to the end-effector position
    '''
    def endeffectorFrameInverseKinematicsLineSearch(self, ee_pos_des, w_R_e_des, frame_name, q0=np.zeros(6), verbose=False, wrap = False):

        # Error initialization
        e_bar = 1
        niter = 0
        # Recursion parameters
        epsilon = 1e-06  # Tolerance
        # alpha = 0.1
        alpha = 1  # Step size
        lambda_ = 0.0000001  # Damping coefficient for pseudo-inverse
        max_iter = 200  # Maximum number of iterations

        # For line search only
        gamma = 0.5
        beta = 0.5
        out_of_workspace = False

        # Inverse kinematics with line search
        while True:
            # compute foot position
            self.robot.computeAllTerms(q0, np.zeros(6))
            ee_pos0 = self.robot.framePlacement(q0, self.robot.model.getFrameId(frame_name)).translation
            w_R_e0 = self.robot.framePlacement(q0, self.robot.model.getFrameId(frame_name)).rotation

            # get the square matrix jacobian that is smaller
            J6 = self.robot.frameJacobian(q0, self.robot.model.getFrameId(frame_name), True,
                                          pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)

            # computed error
            e_bar = np.hstack((ee_pos_des - ee_pos0, w_R_e0.dot(self.errorInSO3(w_R_e0, w_R_e_des))))
            grad = J6.T.dot(e_bar)
            dq = np.linalg.inv(J6 + 1e-08* np.identity(6)).dot(e_bar)

            if np.linalg.norm(grad) < epsilon:
                IKsuccess = True
                if verbose:
                    print("IK Convergence achieved!, norm(grad) :", np.linalg.norm(grad))
                    print("Inverse kinematics solved in {} iterations".format(niter))
                    if np.linalg.norm(e_bar) > 0.1:
                        print("THE END EFFECTOR POSITION IS OUT OF THE WORKSPACE, norm(error) :", np.linalg.norm(e_bar))
                        out_of_workspace = True
                break

            if niter >= max_iter:
                if verbose:
                    print(
                        "\n Warning: Max number of iterations reached, the iterative algorithm has not reached convergence to the desired precision. Error is: ",
                        np.linalg.norm(e_bar))
                IKsuccess = False
                break

            # q0 += dq
            # while False:

            while True:
                if verbose:
                    print("Iter #:", niter)
                # Update
                q1 = q0 + dq * alpha
                self.robot.computeAllTerms(q1, np.zeros(6))
                ee_pos1 = self.robot.framePlacement(q1, self.robot.model.getFrameId(frame_name)).translation
                w_R_e1 = self.robot.framePlacement(q1, self.robot.model.getFrameId(frame_name)).rotation

                e_bar_new = np.hstack((ee_pos_des - ee_pos1, w_R_e1.dot(self.errorInSO3(w_R_e1, w_R_e_des))))
                # print "e_bar_new", np.linalg.norm(e_bar_new), "e_bar", np.linalg.norm(e_bar)

                error_reduction = np.linalg.norm(e_bar) - np.linalg.norm(e_bar_new)
                threshold = 0.0  # even more strict: gamma*alpha*np.linalg.norm(e_bar)

                if error_reduction < threshold:
                    alpha = beta * alpha
                    if verbose:
                        print(" line search: alpha: ", alpha)
                else:
                    q0 = q1
                    alpha = 1
                    break
            niter += 1

        # wrapping prevents from outputs larger than 2pi
        if (wrap):
            for i in range(len(q0)):
                while q0[i] >= 2 * math.pi:
                    q0[i] -= 2 * math.pi
                while q0[i] < -2 * math.pi:
                    q0[i] += 2 * math.pi

        return q0, IKsuccess, out_of_workspace

    def isOutOfJointLims(self, joint_positions, joint_limits_max, joint_limits_min):

        no_of_legs_to_check = joint_positions.size/3
        q = joint_positions.reshape((no_of_legs_to_check, 3))

        # print "q: ", q
        # print "leq than max ", np.all(np.less_equal(q, joint_limits_max))
        # print "geq than min ", np.all(np.greater_equal(q, joint_limits_min))
        return not np.all(np.less_equal(q, joint_limits_max)) \
               or not np.all(np.greater_equal(q, joint_limits_min))

    def isOutOfWorkSpace(self, contactsBF_check, joint_limits_max, joint_limits_min, stance_index, foot_vel):
        q = self.fixedBaseInverseKinematics(contactsBF_check)
        out = self.isOutOfJointLims(q, joint_limits_max, joint_limits_min)
        
        return out

    