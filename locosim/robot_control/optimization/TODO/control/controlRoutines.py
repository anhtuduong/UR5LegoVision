# Description
# File contains some necessary control algorithms for HyQ
# Author: Niraj Rathod
# Date: 19-11-2019

# Standard packages
import scipy.io
import scipy.sparse as sparse
import numpy as np
import yaml

# User defined packages
from tools.mathutils import *
from tools.math_tools import Math
from tools.utils import Utils

# Whole body controller for HyQ that includes ffd wrench + fb Wrench (Virtual PD) + gravity compensation
#every vector is in the wf
def wholeBodyController(act_com_pose, act_com_twist,  xf,  des_com_pose, des_com_twist, des_com_acc, stance_legs):
    util = Utils()
    # Mass of a robot
    # self.robotMass = data_loaded ['robotMass']
    robotMass = 85.446



    # The inertia matrix
    Inertia = np.array([[4.0745, 0.1458, -0.2245],
                             [0.1458, 11.3576, -0.0133],
                             [-0.2245, -0.0133, 12.5675]])

    # Load math functions from jet-leg
    mathJet = Math()

    Rm = mathJet.rpyToRot(util.angPart(act_com_pose)[util.crd("X")], util.angPart(act_com_pose)[util.crd("Y")],
                          util.angPart(act_com_pose)[util.crd("Z")])

    # - Rotation matrix for the desired values of the orientations
    Rdes = mathJet.rpyToRot(des_com_pose[util.sp_crd("AX")], des_com_pose[util.sp_crd("AY")], des_com_pose[util.sp_crd("AZ")])
    Re = Rdes.dot(Rm.transpose())

    err = rotMatToRotVec(Re)  # errors of the orientation w.r.t desire values

    # This is a skew symmetric matrix for (xfi-xc) in the second matrix of equation 1.3
    # corressponding to omega_dot (difference between the foothold locations
    # and COM trajectories)
    d1 = cross_mx(xf[:,util.leg_map("LF")] - util.linPart(act_com_pose))
    d2 = cross_mx(xf[:,util.leg_map("RF")] - util.linPart(act_com_pose))
    d3 = cross_mx(xf[:,util.leg_map("LH")] - util.linPart(act_com_pose))
    d4 = cross_mx(xf[:,util.leg_map("RH")] - util.linPart(act_com_pose))

    # Gains for the virtual model
    Kpcomx = 1500
    Kpcomy = 1500
    Kpcomz = 1500

    Kdcomx = 300
    Kdcomy = 300
    Kdcomz = 300

    KpbRoll = 1500
    KpbPitch = 1500
    KpbYaw = 1500

    Kdbasex = 200
    Kdbasey = 200
    Kdbasez = 200

    Kpcom = np.diag([Kpcomx, Kpcomy, Kpcomz])
    Kpbase = np.diag([KpbRoll, KpbPitch, KpbYaw])
    Kdcom = np.diag([Kdcomx, Kdcomy, Kdcomz])
    Kdbase = np.diag([Kdbasex, Kdbasey, Kdbasez])

    # Virtual model based control (Feedback Wrench)
    Wfbk = np.zeros(6)
    Wfbk[0:3] = Kpcom.dot(util.linPart(des_com_pose) - util.linPart(act_com_pose)) + Kdcom.dot(util.linPart(des_com_twist) - util.linPart(act_com_twist))
    Wfbk[3:6] = Kpbase.dot(Rm.transpose().dot(err)) + Kdbase.dot(util.angPart(des_com_twist)- util.angPart(act_com_twist))

    # FFd linear
    ffdLinear = robotMass * util.linPart(des_com_acc) 
    ffdAngular = (Rm.transpose().dot(Inertia)).dot(util.angPart(des_com_acc))

    # Gravity Wrench
    Wg = - robotMass * np.array([0, 0, -9.81, 0, 0, 0])

    # Total Wrench
    TotWrench = np.hstack([ffdLinear, ffdAngular]) + Wfbk+ Wg
    # TotWrench =  Wg

    # Stance matrix
    S_mat = np.diag(np.hstack([stance_legs[util.leg_map("LF")] * np.ones(3), stance_legs[util.leg_map("RF")] * np.ones(3),
                              stance_legs[util.leg_map("LH")] * np.ones(3), stance_legs[util.leg_map("RH")] * np.ones(3)]))

    # A matrix used in equation 1.17 only for the stance feet
    AtmPre = np.vstack([np.hstack([np.eye(3), np.eye(3), np.eye(3), np.eye(3)]),
                        np.hstack([d1, d2, d3, d4])])
    Atm = AtmPre.dot(S_mat)

    # Map the total Wrench to grf
    u_vpd = np.linalg.pinv(Atm, 1e-04).dot(TotWrench)

    # Mutliply with stance matrix
    u = u_vpd

    return u