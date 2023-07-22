"""
This script contains all the accessary functions related to
kinematics
Author: Niraj Rathod
Date : 08/06/21
"""
import numpy as np
from casadi import *
from tools.mathutils import rpyToRot, cross_mx
from tools.utils import Utils

''' Function to compute feet positions in the BASE FRAME with forward kinematics '''

def forward_kinematics(q):

    upperLegLength = 0.35
    lowerLegLength = 0.341

    BASE2HAA_offset_x = 0.3735
    BASE2HAA_offset_y = 0.207
    BASE2HAA_offset_z = 0.08
    HAA2HFE = 0.08

    s__q_LF_HFE = np.sin(q[1])
    s__q_LF_KFE = np.sin(q[2])
    s__q_LF_HAA = np.sin(q[0])
    s__q_RF_HFE = np.sin(q[4])
    s__q_RF_KFE = np.sin(q[5])
    s__q_RF_HAA = np.sin(q[3])
    s__q_LH_HFE = np.sin(q[7])
    s__q_LH_KFE = np.sin(q[8])
    s__q_LH_HAA = np.sin(q[6])
    s__q_RH_HFE = np.sin(q[10])
    s__q_RH_KFE = np.sin(q[11])
    s__q_RH_HAA = np.sin(q[9])
    c__q_LF_HFE = np.cos(q[1])
    c__q_LF_KFE = np.cos(q[2])
    c__q_LF_HAA = np.cos(q[0])
    c__q_RF_HFE = np.cos(q[4])
    c__q_RF_KFE = np.cos(q[5])
    c__q_RF_HAA = np.cos(q[3])
    c__q_LH_HFE = np.cos(q[7])
    c__q_LH_KFE = np.cos(q[8])
    c__q_LH_HAA = np.cos(q[6])
    c__q_RH_HFE = np.cos(q[10])
    c__q_RH_KFE = np.cos(q[11])
    c__q_RH_HAA = np.cos(q[9])


    if type(q[0]) is type(SX(0)):
        fr_trunk_Xh_LF_foot = SX.sym("fr_trunk_Xh_LF_foot",3)
        fr_trunk_Xh_RF_foot = SX.sym("fr_trunk_Xh_RF_foot",3)
        fr_trunk_Xh_LH_foot = SX.sym("fr_trunk_Xh_LH_foot",3)
        fr_trunk_Xh_RH_foot = SX.sym("fr_trunk_Xh_RH_foot",3)
    else:
        fr_trunk_Xh_LF_foot = np.zeros(3)
        fr_trunk_Xh_RF_foot = np.zeros(3)
        fr_trunk_Xh_LH_foot = np.zeros(3)
        fr_trunk_Xh_RH_foot = np.zeros(3)

    # LF leg
    fr_trunk_Xh_LF_foot[0] = (- lowerLegLength * c__q_LF_HFE * s__q_LF_KFE) - \
                             ( lowerLegLength * s__q_LF_HFE * c__q_LF_KFE) - \
                             (upperLegLength * s__q_LF_HFE) + BASE2HAA_offset_x
    fr_trunk_Xh_LF_foot[1] = (lowerLegLength * s__q_LF_HAA * s__q_LF_HFE * s__q_LF_KFE) - \
                             (lowerLegLength * s__q_LF_HAA * c__q_LF_HFE * c__q_LF_KFE) - \
                             (upperLegLength * s__q_LF_HAA * c__q_LF_HFE) - \
                             (BASE2HAA_offset_z * s__q_LF_HAA) + BASE2HAA_offset_y
    fr_trunk_Xh_LF_foot[2] = (lowerLegLength * c__q_LF_HAA * s__q_LF_HFE * s__q_LF_KFE) -\
                             (lowerLegLength * c__q_LF_HAA * c__q_LF_HFE * c__q_LF_KFE) - \
                             (upperLegLength * c__q_LF_HAA * c__q_LF_HFE) - \
                             (BASE2HAA_offset_z * c__q_LF_HAA)

    # RF leg
    fr_trunk_Xh_RF_foot[0] = (- lowerLegLength * c__q_RF_HFE * s__q_RF_KFE) - \
                                (lowerLegLength * s__q_RF_HFE * c__q_RF_KFE) - \
                                (upperLegLength * s__q_RF_HFE) + BASE2HAA_offset_x
    fr_trunk_Xh_RF_foot[1] = (- lowerLegLength * s__q_RF_HAA * s__q_RF_HFE * s__q_RF_KFE) + \
                                (lowerLegLength * s__q_RF_HAA * c__q_RF_HFE * c__q_RF_KFE) + \
                                (upperLegLength * s__q_RF_HAA * c__q_RF_HFE) + \
                                (BASE2HAA_offset_z * s__q_RF_HAA) - BASE2HAA_offset_y
    fr_trunk_Xh_RF_foot[2] = (lowerLegLength * c__q_RF_HAA * s__q_RF_HFE * s__q_RF_KFE) - \
                                (lowerLegLength * c__q_RF_HAA * c__q_RF_HFE * c__q_RF_KFE) - \
                                (upperLegLength * c__q_RF_HAA * c__q_RF_HFE) - \
                                (BASE2HAA_offset_z * c__q_RF_HAA)

    # LH leg
    fr_trunk_Xh_LH_foot[0] = (- lowerLegLength * c__q_LH_HFE * s__q_LH_KFE) - \
                                (lowerLegLength * s__q_LH_HFE * c__q_LH_KFE) - \
                                (upperLegLength * s__q_LH_HFE) - BASE2HAA_offset_x
    fr_trunk_Xh_LH_foot[1] = (lowerLegLength * s__q_LH_HAA * s__q_LH_HFE * s__q_LH_KFE) - \
                                (lowerLegLength * s__q_LH_HAA * c__q_LH_HFE * c__q_LH_KFE) - \
                                (upperLegLength * s__q_LH_HAA * c__q_LH_HFE) - \
                                (BASE2HAA_offset_z * s__q_LH_HAA) + BASE2HAA_offset_y
    fr_trunk_Xh_LH_foot[2] = (lowerLegLength * c__q_LH_HAA * s__q_LH_HFE * s__q_LH_KFE) - \
                                (lowerLegLength * c__q_LH_HAA * c__q_LH_HFE * c__q_LH_KFE) - \
                                (upperLegLength * c__q_LH_HAA * c__q_LH_HFE) - \
                                (BASE2HAA_offset_z * c__q_LH_HAA)

    # RH leg
    fr_trunk_Xh_RH_foot[0] = (- lowerLegLength * c__q_RH_HFE * s__q_RH_KFE) - \
                                (lowerLegLength * s__q_RH_HFE * c__q_RH_KFE) - \
                                (upperLegLength * s__q_RH_HFE) - BASE2HAA_offset_x
    fr_trunk_Xh_RH_foot[1] = (- lowerLegLength * s__q_RH_HAA * s__q_RH_HFE * s__q_RH_KFE) + \
                                (lowerLegLength * s__q_RH_HAA * c__q_RH_HFE * c__q_RH_KFE) + \
                                (upperLegLength * s__q_RH_HAA * c__q_RH_HFE) + \
                                (BASE2HAA_offset_z * s__q_RH_HAA) - BASE2HAA_offset_y
    fr_trunk_Xh_RH_foot[2] = (lowerLegLength * c__q_RH_HAA * s__q_RH_HFE * s__q_RH_KFE) - \
                                (lowerLegLength * c__q_RH_HAA * c__q_RH_HFE * c__q_RH_KFE) - \
                                (upperLegLength * c__q_RH_HAA * c__q_RH_HFE) - \
                                (BASE2HAA_offset_z * c__q_RH_HAA)

    if type(q[0]) is type(SX(0)):
        foot_positionB = vertcat(fr_trunk_Xh_LF_foot,
                             fr_trunk_Xh_RF_foot,
                             fr_trunk_Xh_LH_foot,
                             fr_trunk_Xh_RH_foot)
    else:
        foot_positionB = np.hstack([fr_trunk_Xh_LF_foot,
                             fr_trunk_Xh_RF_foot,
                             fr_trunk_Xh_LH_foot,
                             fr_trunk_Xh_RH_foot])
    return foot_positionB


''' Function to compute feet positions in the WORLD FRAME with forward kinematics '''

def feet_posW_using_fwdkin(states):
    util = Utils()

    # CoM offset from origin of the base frame expressed in the base frame
    com_offset = vertcat(0.0094095, 0.00023, -0.04509)

    # Radius of the foot ball in the base frame
    foot_ball_correction = vertcat(0.0, 0.0, 0.02155)
    # foot_ball_correction = vertcat(0.0, 0.0, 0.0)

    # Rotation matrix (maps a vector from world to CoM frame)
    c_R_w = rpyToRot(states[6], states[7], states[8])

    # compute feet positions in the base frame
    foot_positionB = forward_kinematics(states[12:])

    # Base frame origin defined in world frame
    base_PosW = states[:3]- c_R_w.T @ com_offset

    # foot position in the world frame
    foot_position_LFW = base_PosW + c_R_w.T @ (util.getLegJointState(util.leg_map("LF"),
                                                            foot_positionB) - foot_ball_correction)
    foot_position_RFW = base_PosW + c_R_w.T @ (util.getLegJointState(util.leg_map("RF"),
                                                            foot_positionB) - foot_ball_correction)
    foot_position_LHW = base_PosW + c_R_w.T @ (util.getLegJointState(util.leg_map("LH"),
                                                            foot_positionB) - foot_ball_correction)
    foot_position_RHW = base_PosW + c_R_w.T @ (util.getLegJointState(util.leg_map("RH"),
                                                            foot_positionB) - foot_ball_correction)

    foot_positionW = vertcat(foot_position_LFW,
                             foot_position_RFW,
                             foot_position_LHW,
                             foot_position_RHW)

    if type(states[0]) is not(type(SX(0))):
        foot_positionW = foot_positionW.full()
        foot_positionW = foot_positionW.reshape(foot_positionW.shape[0])

    return foot_positionW


''' Function to compute the jacobian for each leg'''


def update_jacobians(q):
    # Leg parameters
    upperLegLength = 0.35
    lowerLegLength = 0.341
    BASE2HAA_offset_x = 0.3735
    BASE2HAA_offset_y = 0.207
    BASE2HAA_offset_z = 0.08
    HAA2HFE = 0.08

    # Initialize Jacobians
    fr_trunk_J_LF_foot = SX.zeros(6, 3)
    fr_trunk_J_RF_foot = SX.zeros(6, 3)
    fr_trunk_J_LH_foot = SX.zeros(6, 3)
    fr_trunk_J_RH_foot = SX.zeros(6, 3)

    fr_trunk_J_LF_foot[0, 0] = - 1.0
    fr_trunk_J_RF_foot[0, 0] = 1.0
    fr_trunk_J_LH_foot[0, 0] = - 1.0
    fr_trunk_J_RH_foot[0, 0] = 1.0

    # Left Front leg jacobian
    s__q_LF_HAA = np.sin(q[0])
    s__q_LF_HFE = np.sin(q[1])
    s__q_LF_KFE = np.sin(q[2])
    c__q_LF_HAA = np.cos(q[0])
    c__q_LF_HFE = np.cos(q[1])
    c__q_LF_KFE = np.cos(q[2])


    fr_trunk_J_LF_foot[1, 1] =  c__q_LF_HAA
    fr_trunk_J_LF_foot[1, 2] =  c__q_LF_HAA
    fr_trunk_J_LF_foot[2, 1] = - s__q_LF_HAA
    fr_trunk_J_LF_foot[2, 2] = - s__q_LF_HAA
    fr_trunk_J_LF_foot[3, 1] = ( lowerLegLength *  s__q_LF_HFE *  s__q_LF_KFE) - \
                                  ( lowerLegLength *  c__q_LF_HFE *  c__q_LF_KFE) - \
                                  ( upperLegLength *  c__q_LF_HFE)
    fr_trunk_J_LF_foot[3, 2] = ( lowerLegLength *  s__q_LF_HFE *  s__q_LF_KFE) - \
                                  ( lowerLegLength *  c__q_LF_HFE *  c__q_LF_KFE)
    fr_trunk_J_LF_foot[4, 0] = ( lowerLegLength *  c__q_LF_HAA *  s__q_LF_HFE *  s__q_LF_KFE) - \
                                  ( lowerLegLength *  c__q_LF_HAA *  c__q_LF_HFE *  c__q_LF_KFE) - \
                                  ( upperLegLength *  c__q_LF_HAA *  c__q_LF_HFE) - \
                                  ( BASE2HAA_offset_z *  c__q_LF_HAA)
    fr_trunk_J_LF_foot[4, 1] = ( lowerLegLength *  s__q_LF_HAA *  c__q_LF_HFE *  s__q_LF_KFE) + \
                                  ( lowerLegLength *  s__q_LF_HAA *  s__q_LF_HFE *  c__q_LF_KFE) + \
                                  ( upperLegLength *  s__q_LF_HAA *  s__q_LF_HFE)
    fr_trunk_J_LF_foot[4, 2] = ( lowerLegLength *  s__q_LF_HAA *  c__q_LF_HFE *  s__q_LF_KFE) + \
                                  ( lowerLegLength *  s__q_LF_HAA *  s__q_LF_HFE *  c__q_LF_KFE)
    fr_trunk_J_LF_foot[5, 0] = (- lowerLegLength *  s__q_LF_HAA *  s__q_LF_HFE *  s__q_LF_KFE) + \
                                  ( lowerLegLength *  s__q_LF_HAA *  c__q_LF_HFE *  c__q_LF_KFE) + \
                                  ( upperLegLength *  s__q_LF_HAA *  c__q_LF_HFE) + \
                                  ( BASE2HAA_offset_z *  s__q_LF_HAA)
    fr_trunk_J_LF_foot[5, 1] = ( lowerLegLength *  c__q_LF_HAA *  c__q_LF_HFE *  s__q_LF_KFE) + \
                                  ( lowerLegLength *  c__q_LF_HAA *  s__q_LF_HFE *  c__q_LF_KFE) + \
                                  ( upperLegLength *  c__q_LF_HAA *  s__q_LF_HFE)
    fr_trunk_J_LF_foot[5, 2] = ( lowerLegLength *  c__q_LF_HAA *  c__q_LF_HFE *  s__q_LF_KFE) + \
                                  ( lowerLegLength *  c__q_LF_HAA *  s__q_LF_HFE *  c__q_LF_KFE)

    # Right Front leg jacobian
    s__q_RF_HAA = np.sin(q[3])
    s__q_RF_HFE = np.sin(q[4])
    s__q_RF_KFE = np.sin(q[5])
    c__q_RF_HAA = np.cos(q[3])
    c__q_RF_HFE = np.cos(q[4])
    c__q_RF_KFE = np.cos(q[5])

    fr_trunk_J_RF_foot[1, 1] =  c__q_RF_HAA
    fr_trunk_J_RF_foot[1, 2] =  c__q_RF_HAA
    fr_trunk_J_RF_foot[2, 1] =  s__q_RF_HAA
    fr_trunk_J_RF_foot[2, 2] =  s__q_RF_HAA
    fr_trunk_J_RF_foot[3, 1] = ( lowerLegLength *  s__q_RF_HFE *  s__q_RF_KFE) - \
                                  ( lowerLegLength *  c__q_RF_HFE *  c__q_RF_KFE) - \
                                  ( upperLegLength *  c__q_RF_HFE)
    fr_trunk_J_RF_foot[3, 2] = ( lowerLegLength *  s__q_RF_HFE *  s__q_RF_KFE) - \
                                  ( lowerLegLength *  c__q_RF_HFE *  c__q_RF_KFE)
    fr_trunk_J_RF_foot[4, 0] = (- lowerLegLength *  c__q_RF_HAA *  s__q_RF_HFE *  s__q_RF_KFE) + \
                                  ( lowerLegLength *  c__q_RF_HAA *  c__q_RF_HFE *  c__q_RF_KFE) + \
                                  ( upperLegLength *  c__q_RF_HAA *  c__q_RF_HFE) + \
                                  ( BASE2HAA_offset_z *  c__q_RF_HAA)
    fr_trunk_J_RF_foot[4, 1] = (- lowerLegLength *  s__q_RF_HAA *  c__q_RF_HFE *  s__q_RF_KFE) - \
                                  ( lowerLegLength *  s__q_RF_HAA *  s__q_RF_HFE *  c__q_RF_KFE) - \
                                  ( upperLegLength *  s__q_RF_HAA *  s__q_RF_HFE)
    fr_trunk_J_RF_foot[4, 2] = (- lowerLegLength *  s__q_RF_HAA *  c__q_RF_HFE *  s__q_RF_KFE) - \
                                  ( lowerLegLength *  s__q_RF_HAA *  s__q_RF_HFE *  c__q_RF_KFE)
    fr_trunk_J_RF_foot[5, 0] = (- lowerLegLength *  s__q_RF_HAA *  s__q_RF_HFE *  s__q_RF_KFE) + \
                                  ( lowerLegLength *  s__q_RF_HAA *  c__q_RF_HFE *  c__q_RF_KFE) + \
                                  ( upperLegLength *  s__q_RF_HAA *  c__q_RF_HFE) + \
                                  ( BASE2HAA_offset_z *  s__q_RF_HAA)
    fr_trunk_J_RF_foot[5, 1] = ( lowerLegLength *  c__q_RF_HAA *  c__q_RF_HFE *  s__q_RF_KFE) + \
                                  ( lowerLegLength *  c__q_RF_HAA *  s__q_RF_HFE *  c__q_RF_KFE) + \
                                  ( upperLegLength *  c__q_RF_HAA *  s__q_RF_HFE)
    fr_trunk_J_RF_foot[5, 2] = ( lowerLegLength *  c__q_RF_HAA *  c__q_RF_HFE *  s__q_RF_KFE) + \
                                  ( lowerLegLength *  c__q_RF_HAA *  s__q_RF_HFE *  c__q_RF_KFE)

    # Left hind leg jacobian
    s__q_LH_HAA = np.sin(q[6])
    s__q_LH_HFE = np.sin(q[7])
    s__q_LH_KFE = np.sin(q[8])
    c__q_LH_HAA = np.cos(q[6])
    c__q_LH_HFE = np.cos(q[7])
    c__q_LH_KFE = np.cos(q[8])

    fr_trunk_J_LH_foot[1, 1] =  c__q_LH_HAA
    fr_trunk_J_LH_foot[1, 2] =  c__q_LH_HAA
    fr_trunk_J_LH_foot[2, 1] = - s__q_LH_HAA
    fr_trunk_J_LH_foot[2, 2] = - s__q_LH_HAA
    fr_trunk_J_LH_foot[3, 1] = ( lowerLegLength *  s__q_LH_HFE *  s__q_LH_KFE) - \
                                  ( lowerLegLength *  c__q_LH_HFE *  c__q_LH_KFE) - \
                                  ( upperLegLength *  c__q_LH_HFE)
    fr_trunk_J_LH_foot[3, 2] = ( lowerLegLength *  s__q_LH_HFE *  s__q_LH_KFE) - \
                                  ( lowerLegLength *  c__q_LH_HFE *  c__q_LH_KFE)
    fr_trunk_J_LH_foot[4, 0] = ( lowerLegLength *  c__q_LH_HAA *  s__q_LH_HFE *  s__q_LH_KFE) - \
                                  ( lowerLegLength *  c__q_LH_HAA *  c__q_LH_HFE *  c__q_LH_KFE) - \
                                  ( upperLegLength *  c__q_LH_HAA *  c__q_LH_HFE) - \
                                  ( BASE2HAA_offset_z *  c__q_LH_HAA)
    fr_trunk_J_LH_foot[4, 1] = ( lowerLegLength *  s__q_LH_HAA *  c__q_LH_HFE *  s__q_LH_KFE) + \
                                  ( lowerLegLength *  s__q_LH_HAA *  s__q_LH_HFE *  c__q_LH_KFE) + \
                                  ( upperLegLength *  s__q_LH_HAA *  s__q_LH_HFE)
    fr_trunk_J_LH_foot[4, 2] = ( lowerLegLength *  s__q_LH_HAA *  c__q_LH_HFE *  s__q_LH_KFE) + \
                                  ( lowerLegLength *  s__q_LH_HAA *  s__q_LH_HFE *  c__q_LH_KFE)
    fr_trunk_J_LH_foot[5, 0] = (- lowerLegLength *  s__q_LH_HAA *  s__q_LH_HFE *  s__q_LH_KFE) + \
                                  ( lowerLegLength *  s__q_LH_HAA *  c__q_LH_HFE *  c__q_LH_KFE) + \
                                  ( upperLegLength *  s__q_LH_HAA *  c__q_LH_HFE) + \
                                  ( BASE2HAA_offset_z *  s__q_LH_HAA)
    fr_trunk_J_LH_foot[5, 1] = ( lowerLegLength *  c__q_LH_HAA *  c__q_LH_HFE *  s__q_LH_KFE) + \
                                  ( lowerLegLength *  c__q_LH_HAA *  s__q_LH_HFE *  c__q_LH_KFE) + \
                                  ( upperLegLength *  c__q_LH_HAA *  s__q_LH_HFE)
    fr_trunk_J_LH_foot[5, 2] = ( lowerLegLength *  c__q_LH_HAA *  c__q_LH_HFE *  s__q_LH_KFE) + \
                                  ( lowerLegLength *  c__q_LH_HAA *  s__q_LH_HFE *  c__q_LH_KFE)

    # Right hind leg jacobian
    s__q_RH_HAA = np.sin(q[9])
    s__q_RH_HFE = np.sin(q[10])
    s__q_RH_KFE = np.sin(q[11])
    c__q_RH_HAA = np.cos(q[9])
    c__q_RH_HFE = np.cos(q[10])
    c__q_RH_KFE = np.cos(q[11])

    fr_trunk_J_RH_foot[1, 1] =  c__q_RH_HAA
    fr_trunk_J_RH_foot[1, 2] =  c__q_RH_HAA
    fr_trunk_J_RH_foot[2, 1] =  s__q_RH_HAA
    fr_trunk_J_RH_foot[2, 2] =  s__q_RH_HAA
    fr_trunk_J_RH_foot[3, 1] = ( lowerLegLength *  s__q_RH_HFE *  s__q_RH_KFE) - \
                                  ( lowerLegLength *  c__q_RH_HFE *  c__q_RH_KFE) - \
                                  ( upperLegLength *  c__q_RH_HFE)
    fr_trunk_J_RH_foot[3, 2] = ( lowerLegLength *  s__q_RH_HFE *  s__q_RH_KFE) - \
                                  ( lowerLegLength *  c__q_RH_HFE *  c__q_RH_KFE)
    fr_trunk_J_RH_foot[4, 0] = (- lowerLegLength *  c__q_RH_HAA *  s__q_RH_HFE *  s__q_RH_KFE) + \
                                  ( lowerLegLength *  c__q_RH_HAA *  c__q_RH_HFE *  c__q_RH_KFE) + \
                                  ( upperLegLength *  c__q_RH_HAA *  c__q_RH_HFE) + \
                                  ( BASE2HAA_offset_z *  c__q_RH_HAA)
    fr_trunk_J_RH_foot[4, 1] = (- lowerLegLength *  s__q_RH_HAA *  c__q_RH_HFE *  s__q_RH_KFE) - \
                                  ( lowerLegLength *  s__q_RH_HAA *  s__q_RH_HFE *  c__q_RH_KFE) - \
                                  ( upperLegLength *  s__q_RH_HAA *  s__q_RH_HFE)
    fr_trunk_J_RH_foot[4, 2] = (- lowerLegLength *  s__q_RH_HAA *  c__q_RH_HFE *  s__q_RH_KFE) - \
                                  ( lowerLegLength *  s__q_RH_HAA *  s__q_RH_HFE *  c__q_RH_KFE)
    fr_trunk_J_RH_foot[5, 0] = (- lowerLegLength *  s__q_RH_HAA *  s__q_RH_HFE *  s__q_RH_KFE) + \
                                  ( lowerLegLength *  s__q_RH_HAA *  c__q_RH_HFE *  c__q_RH_KFE) + \
                                  ( upperLegLength *  s__q_RH_HAA *  c__q_RH_HFE) + \
                                  ( BASE2HAA_offset_z *  s__q_RH_HAA)
    fr_trunk_J_RH_foot[5, 1] = ( lowerLegLength *  c__q_RH_HAA *  c__q_RH_HFE *  s__q_RH_KFE) + \
                                  ( lowerLegLength *  c__q_RH_HAA *  s__q_RH_HFE *  c__q_RH_KFE) + \
                                  ( upperLegLength *  c__q_RH_HAA *  s__q_RH_HFE)
    fr_trunk_J_RH_foot[5, 2] = ( lowerLegLength *  c__q_RH_HAA *  c__q_RH_HFE *  s__q_RH_KFE) + \
                                  ( lowerLegLength *  c__q_RH_HAA *  s__q_RH_HFE *  c__q_RH_KFE)

    return fr_trunk_J_LF_foot[3:6, :] , fr_trunk_J_RF_foot[3:6, :], \
           fr_trunk_J_LH_foot[3:6, :], fr_trunk_J_RH_foot[3:6, :]

''' Compute the foot velocity as the function of the joint variable and
the state'''

def compute_feet_velocities(states, q_dot):
    util = Utils()

    # Base velocity contribution to the leg
    com_position = states[:3]
    com_velocity = states[3:6]
    omega = states[9:12]
    foot_positionW = feet_posW_using_fwdkin(states)
    foot_pos_skew_matrix_LF = cross_mx(util.getLegJointState(util.leg_map("LF"), foot_positionW) -
                                       com_position)
    foot_pos_skew_matrix_RF = cross_mx(util.getLegJointState(util.leg_map("RF"), foot_positionW) -
                                       com_position)
    foot_pos_skew_matrix_LH = cross_mx(util.getLegJointState(util.leg_map("LH"), foot_positionW) -
                                       com_position)
    foot_pos_skew_matrix_RH = cross_mx(util.getLegJointState(util.leg_map("RH"), foot_positionW) -
                                       com_position)
    LF_velocity_dueto_base = com_velocity - foot_pos_skew_matrix_LF @ omega
    RF_velocity_dueto_base = com_velocity - foot_pos_skew_matrix_RF @ omega
    LH_velocity_dueto_base = com_velocity - foot_pos_skew_matrix_LH @ omega
    RH_velocity_dueto_base = com_velocity - foot_pos_skew_matrix_RH @ omega

    # Compute leg jacobians
    LF_jac, RF_jac, LH_jac, RH_jac = update_jacobians(states[12:])

    # Rotation matrix
    c_R_w = rpyToRot(states[6], states[7], states[8])

    # Feet velocities
    LF_foot_velocityW = LF_velocity_dueto_base + \
                       c_R_w.T @ LF_jac @ util.getLegJointState(util.leg_map("LF"), q_dot)
    RF_foot_velocityW = RF_velocity_dueto_base + \
                       c_R_w.T @ RF_jac @ util.getLegJointState(util.leg_map("RF"), q_dot)
    LH_foot_velocityW = LH_velocity_dueto_base + \
                       c_R_w.T @ LH_jac @ util.getLegJointState(util.leg_map("LH"), q_dot)
    RH_foot_velocityW = RH_velocity_dueto_base + \
                       c_R_w.T @ RH_jac @ util.getLegJointState(util.leg_map("RH"), q_dot)
    if type(states[0]) is not(type(SX(0))):
        feet_velocityW = vertcat(LF_foot_velocityW,
                                 RF_foot_velocityW,
                                 LH_foot_velocityW,
                                 RH_foot_velocityW)
        # convert casadi symbolics to numerical values
        feet_velocityW_num = DM(feet_velocityW)
        feet_velocityW = np.array(feet_velocityW_num)
        return feet_velocityW.reshape(feet_velocityW.shape[0])
    else:
        return LF_foot_velocityW, RF_foot_velocityW, LH_foot_velocityW, RH_foot_velocityW