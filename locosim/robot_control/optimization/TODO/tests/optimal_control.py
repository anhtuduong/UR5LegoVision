# -*- coding: utf-8 -*-
"""
Created on Wed Apr 10 00:08:28 2019

@author: Niraj Rathod

"""
from __future__ import print_function
import sys
import os
print()

#print(sys.version_info)
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
                 '../.' ]#this is the current directory

import scipy.io
import scipy.sparse as sparse
import numpy as np
import cvxpy as cp
import osqp as osqp
import matplotlib
import yaml
import time
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from gurobi.GurobiSolver import GUROBISolver

# from gurobipy import *
from model.run_linearization import RunLinearization
from refgen.robotDataFile import extractROSdata
from tools.mathutils import *
from tools.utils import Utils

np.set_printoptions(linewidth = 200, suppress = True)
# import time
data = scipy.io.loadmat("../data/crawl_references_40ms.mat")
# LinMatrices =  scipy.io.loadmat("QPmatlabLinearMat.mat")

# %% Bounds on inputs
util = Utils()
# read optimization params
with open("../config/config.yaml", 'r') as stream:
    data_loaded = yaml.load(stream)

# no of states and inputs
nx = 12  # no of states
nu = 12  # no of inputs


# state limits
x_min = -np.inf*np.ones((nx))
# x_min = np.array(np.zeros(nx))
#
# x_min[0] = data_loaded['x_limits']['com_x']['min']
# x_min[1] = data_loaded['x_limits']['com_y']['min']
# x_min[2] = data_loaded['x_limits']['com_z']['min']
#
# x_min[3] = data_loaded['x_limits']['linvel_x']['min']
# x_min[4] = data_loaded['x_limits']['linvel_y']['min']
# x_min[5] = data_loaded['x_limits']['linvel_z']['min']
#
# x_min[6] = data_loaded['x_limits']['ang_x']['min']
# x_min[7] = data_loaded['x_limits']['ang_y']['min']
# x_min[8] = data_loaded['x_limits']['ang_z']['min']
#
# x_min[9] = data_loaded['x_limits']['angvel_x']['min']
# x_min[10] = data_loaded['x_limits']['angvel_y']['min']
# x_min[11] = data_loaded['x_limits']['angvel_z']['min']


x_max = np.inf*np.ones((nx))
# x_max = np.array(np.zeros(nx))
# x_max[0] = data_loaded['x_limits']['com_x']['max']
# x_max[1] = data_loaded['x_limits']['com_y']['max']
# x_max[2] = data_loaded['x_limits']['com_z']['max']
#
# x_max[3] = data_loaded['x_limits']['linvel_x']['max']
# x_max[4] = data_loaded['x_limits']['linvel_y']['max']
# x_max[5] = data_loaded['x_limits']['linvel_z']['max']
#
# x_max[6] = data_loaded['x_limits']['ang_x']['max']
# x_max[7] = data_loaded['x_limits']['ang_y']['max']
# x_max[8] = data_loaded['x_limits']['ang_z']['max']
#
# x_max[9] = data_loaded['x_limits']['angvel_x']['max']
# x_max[10] = data_loaded['x_limits']['angvel_y']['max']
# x_max[11] = data_loaded['x_limits']['angvel_z']['max']



#print x_max, x_min

# z_force_max = 1000*np.ones((4))
z_force_max = np.array(np.zeros(4))
z_force_max[util.leg_map("LF")] = data_loaded['z_force_limits']['LF']['max']
z_force_max[util.leg_map("RF")] = data_loaded['z_force_limits']['RF']['max']
z_force_max[util.leg_map("LH")] = data_loaded['z_force_limits']['LH']['max']
z_force_max[util.leg_map("RH")] = data_loaded['z_force_limits']['RH']['max']

z_force_min = np.array(np.zeros(4))
z_force_min[util.leg_map("LF")] = data_loaded['z_force_limits']['LF']['min']
z_force_min[util.leg_map("RF")] = data_loaded['z_force_limits']['RF']['min']
z_force_min[util.leg_map("LH")] = data_loaded['z_force_limits']['LH']['min']
z_force_min[util.leg_map("RH")] = data_loaded['z_force_limits']['RH']['min']

mu = np.array(np.zeros(4))
mu[util.leg_map("LF")] = data_loaded['friction_coeff']['LF']
mu[util.leg_map("RF")] = data_loaded['friction_coeff']['RF']
mu[util.leg_map("LH")] = data_loaded['friction_coeff']['LH']
mu[util.leg_map("RH")] = data_loaded['friction_coeff']['RH']


# robot inertia
robotInertia = np.array([[4.0745, 0.1458, -0.2245], [0.1458, 11.3576, -0.0133], [-0.2245, -0.0133, 12.5675]])

# Prediction horizon
N = 100
print ('Prediction horizon:', N)

# Sampling time
Ts = 0.04

# Define simulation start point from the data
simStart = 0  # Remember in python the counter starts from zero not 1!

# Robot mass
# robotMass = np.mean(data['grForcesLFWz_gt'][1:50] + data['grForcesRFWz_gt'][1:50]
#                     + data['grForcesLHWz_gt'][1:50] + data['grForcesRHWz_gt'][1:50]) / 9.81

robotMass =  data_loaded['robotMass']

# Essential flags
selOsqpsolver = 1
include_ltv_model = 1

# gravity
gravity = np.array([0, 0, -9.81])

# %% Extract linearization data from ROS

rosData = extractROSdata(data)

x_lin_data = np.zeros((nx, N))
u_lin_data = np.zeros((nu, N))

A_lin_data = []
B_lin_data = []
S_mat_data = []
f_xu_lin_data = []
affineTerm = []
stance = np.zeros((4, N))
sortSwing = np.zeros((4, N))
# LinMatDataA=LinMatrices['Lin_A']
# LinMatDataB=LinMatrices['Lin_B']
# LinMatDataRk=LinMatrices['rk']
# # A_lin_data = []
# # B_lin_data = []
# # affineTerm = []
# for i in range(100):
#     A_lin_data.append(LinMatDataA[i].tolist()[0])
#     B_lin_data.append(LinMatDataB[i].tolist()[0])
#     affineTerm.append(LinMatDataRk[i].tolist()[0].flatten())


# Instantiate the linearization class
linearize_model = RunLinearization(nx,
                                   nu,
                                   Ts,
                                   robotInertia,
                                   robotMass,
                                   0,
                                   data_loaded['include_coneCons'])

start_t_lin = time.time()

for i in range(N):
    x_lin_data[:, i] = np.array([rosData.actual_CoMXW[simStart + i],
                                 rosData.actual_CoMYW[simStart + i],
                                 rosData.actual_CoMZW[simStart + i],
                                 rosData.com_VxW[0,simStart + i],
                                 rosData.com_VyW[0,simStart + i],
                                 rosData.com_VzW[0,simStart + i],
                                 rosData.rollW[simStart + i],
                                 rosData.pitchW[simStart + i],
                                 rosData.yawW[simStart + i],
                                 rosData.omegaXW[0, simStart + i],
                                 rosData.omegaYW[0, simStart + i],
                                 rosData.omegaZW[0, simStart + i]])

    u_lin_data[:, i] = np.array([rosData.grForcesLFWx_gt[simStart + i],
                                 rosData.grForcesLFWy_gt[simStart + i],
                                 rosData.grForcesLFWz_gt[simStart + i],
                                 rosData.grForcesRFWx_gt[simStart + i],
                                 rosData.grForcesRFWy_gt[simStart + i],
                                 rosData.grForcesRFWz_gt[simStart + i],
                                 rosData.grForcesLHWx_gt[simStart + i],
                                 rosData.grForcesLHWy_gt[simStart + i],
                                 rosData.grForcesLHWz_gt[simStart + i],
                                 rosData.grForcesRHWx_gt[simStart + i],
                                 rosData.grForcesRHWy_gt[simStart + i],
                                 rosData.grForcesRHWz_gt[simStart + i]]).transpose()

    footPos = np.array([rosData.footPosLFWx[simStart + i],
                        rosData.footPosLFWy[simStart + i],
                        rosData.footPosLFWz[simStart + i],
                        rosData.footPosRFWx[simStart + i],
                        rosData.footPosRFWy[simStart + i],
                        rosData.footPosRFWz[simStart + i],
                        rosData.footPosLHWx[simStart + i],
                        rosData.footPosLHWy[simStart + i],
                        rosData.footPosLHWz[simStart + i],
                        rosData.footPosRHWx[simStart + i],
                        rosData.footPosRHWy[simStart + i],
                        rosData.footPosRHWz[simStart + i]])

    # if  i == 0 or (rosData.swingLF[simStart + i, 0] != rosData.swingLF[simStart + i - 1, 0]) \
    #     or (rosData.swingRF[simStart + i, 0] != rosData.swingRF[simStart + i - 1, 0])\
    #     or (rosData.swingLH[simStart + i, 0] != rosData.swingLH[simStart + i - 1, 0])\
    #     or (rosData.swingRH[simStart + i, 0] != rosData.swingRH[simStart + i - 1, 0]):
    # if  i == 0 or (rosData.swingLF[simStart + i, 0] + rosData.swingRF[simStart + i, 0] +
    #                rosData.swingLH[simStart + i, 0] + rosData.swingRH[simStart + i, 0]) != \
    #         (rosData.swingLF[simStart + i - 1, 0] + rosData.swingRF[simStart + i - 1, 0] +
    #         rosData.swingLH[simStart + i - 1, 0] + rosData.swingRH[simStart + i - 1, 0]):
    #
    #
    #     Adisc, Bdisc, Acon, Bcon, fxu = LinearizeModel.getLinearizeMats(x_lin_data[:, i], u_lin_data[:, i], robotInertia, footPos, robotMass, gravity)
    #     A_lin_data.append(Adisc)
    #     B_lin_data.append(Bdisc)
    #     f_xu_lin_data.append(fxu)
    #     print 'indices' , i
    # else:
    #     A_lin_data.append(A_lin_data[i - 1])
    #     B_lin_data.append(B_lin_data[i - 1])
    #     f_xu_lin_data.append(f_xu_lin_data[i - 1])
        # f_xu_lin_data.append(f_xu_lin_data[0])
    sortSwingSmat = np.array(
        [rosData.swingLF[0, simStart + i] * np.ones(3), rosData.swingRF[0, simStart + i] * np.ones(3),
         rosData.swingLH[0, simStart + i] * np.ones(3), rosData.swingRH[0, simStart + i] * np.ones(3)])

    S_mat_data.append(np.diag(sortSwingSmat.flatten()))

    sortSwing[:, i] = np.array([rosData.swingLF[0, simStart + i], rosData.swingRF[0, simStart + i],
                                rosData.swingLH[0, simStart + i], rosData.swingRH[0, simStart + i]]).flatten()
    stance[:, i] = np.logical_not(sortSwing[:, i]).astype(int)
    # -----------------------------------------------------------------------------------
    if include_ltv_model == 1:
        Adisc, Bdisc,  fxu = linearize_model.get_lin_matrices(x_lin_data[:, i], u_lin_data[:, i],
                                                               footPos, stance[:, i])
        A_lin_data.append(Adisc)
        B_lin_data.append(Bdisc)
        f_xu_lin_data.append(fxu)
    else:
        if i == 0:
            Adisc, Bdisc, Acon, Bcon, rk = linearize_model.get_lin_matrices(x_lin_data[:, 0], u_lin_data[:, 0],
                                                                            footPos, stance[:, i])
        A_lin_data.append(Adisc)
        B_lin_data.append(Bdisc)
        f_xu_lin_data.append(rk)

    # -----------------------------------------------------------------------------------
    if include_ltv_model == 1:
        affineTerm.append(f_xu_lin_data[i])
    else:
        affineTerm.append(f_xu_lin_data[0])
lin_time = (time.time() - start_t_lin)
print('Linearization time:', lin_time)
# %% QP formulation

# # Objective function
# Q = 1e3 * sparse.eye(nx)
# QN = Q
# R = 1e-5 * sparse.eye(nu)
# weight on states
q_array = np.array([data_loaded['weight_Q']['com_x'], data_loaded['weight_Q']['com_y'], data_loaded['weight_Q']['com_z'],
                                 data_loaded['weight_Q']['linvel_x'], data_loaded['weight_Q']['linvel_y'], data_loaded['weight_Q']['linvel_z'],
                                 data_loaded['weight_Q']['ang_x'], data_loaded['weight_Q']['ang_y'], data_loaded['weight_Q']['ang_z'],
                                 data_loaded['weight_Q']['angvel_x'], data_loaded['weight_Q']['angvel_y'], data_loaded['weight_Q']['angvel_z']])
Q = sparse.diags(q_array)
qn_array = q_array
QN = sparse.diags(qn_array)
#Weights on inputs
r_array = np.array([data_loaded['weight_R']['LF_x'], data_loaded['weight_R']['LF_y'], data_loaded['weight_R']['LF_z'],
             data_loaded['weight_R']['RF_x'], data_loaded['weight_R']['RF_y'], data_loaded['weight_R']['RF_z'],
             data_loaded['weight_R']['LH_x'], data_loaded['weight_R']['LH_y'], data_loaded['weight_R']['LH_z'],
             data_loaded['weight_R']['RH_x'], data_loaded['weight_R']['RH_y'], data_loaded['weight_R']['RH_z']])
R = sparse.diags(r_array)

# Lambda=1e-3 * sparse.eye(nu)
Lambda= 0.0 * sparse.eye(nu)

x_init_data = x_lin_data[:, 0]

#%%
#==============================================================================================================
# using the same code as in optimizationClass

# - Equality constraints
# linear dynamics
Afull = np.kron(np.eye(N + 1), -np.eye(nx))  # Big A matrix for Prediction model
Bdiag = np.zeros((nx * N, nu * N))  # Big B matrix for Prediction model
Sdiag = np.zeros((nu * N, nu * N))  # Initialize the Swing matrix

# Loop to create appropriate constraints for linear dynamic constraints for LTV system
if include_ltv_model == 1:
    for j in range(N):
        Afull[nx + (j * nx): (nx * 2) + (j * nx), (j * nx): nx + (j * nx)] = A_lin_data[j]
        Bdiag[(j * nx): nx + (j * nx), (j * nu): nu + (j * nu)] = B_lin_data[j]
        Sdiag[(j * nu): nu + (j * nu), (j * nu): nu + (j * nu)] = S_mat_data[j]
else:
    for j in range(N):
        Afull[nx + (j * nx): (nx * 2) + (j * nx), (j * nx): nx + (j * nx)] = A_lin_data[0]
        Bdiag[(j * nx): nx + (j * nx), (j * nu): nu + (j * nu)] = B_lin_data[0]
        Sdiag[(j * nu): nu + (j * nu), (j * nu): nu + (j * nu)] = S_mat_data[j]


# Stack B matrix with zeros for x(k) = x0 (initial value for MPC) constraint
Bfull = np.vstack((np.zeros((nx, nu * N)), Bdiag))

# Swing matrix -  (no constraints on states)
Sfull = np.hstack([np.zeros((nu * N, nx * (N + 1))), Sdiag])

# stack everything
Aeq_dyn = np.hstack([Afull, Bfull])
Aeq = np.vstack([Aeq_dyn, Sfull])
leq = np.hstack((-x_init_data, -np.hstack(affineTerm), np.zeros(Sfull.shape[0])))
ueq = leq


# ==============================================================================================================
# Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
x_ref_data = x_lin_data
u_ref_data = u_lin_data

# - quadratic objective
Px = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN])
# Pu = sparse.block_diag([sparse.kron(sparse.eye(N), R)])

# Define the diagonal matrix for delta_u in the cost
Q_RLamda = sparse.block_diag([R + Lambda, sparse.kron(sparse.eye(N - 2), (R + 2*Lambda)), R + Lambda])

# Create the off diagonal matrices
OffdiagVec = np.hstack(np.tile(-Lambda.diagonal(),(N - 1, 1)))
diagonals = [OffdiagVec, OffdiagVec]
OffdiagMat = sparse.diags(diagonals,[-12, 12],)

# Add these matrices to create the Pu weight matrix
Pu = Q_RLamda + OffdiagMat


# splitting the costs for better debug
P = sparse.block_diag((Px, Pu)).tocsc()

# to debug the sum of costs is the same
# P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
#                        sparse.kron(sparse.eye(N), R)]).tocsc()

# - linear objective
q_temp = []
qu_temp = []
j = 0

# Loop to create appropriate constraints for linear dynamic constraints for LTV system
while j < N:
    # for linear objective
    q_temp.append(-Q.dot(x_ref_data[:, j]))
    qu_temp.append(-R.dot(u_ref_data[:, j]))
    j = j + 1
qx = np.hstack([np.hstack(q_temp), -QN.dot(x_ref_data[:, N - 1])])
# qu = np.zeros(N * nu)
qu = np.array(qu_temp).flatten()

# to debug the sum of costs is the same
# q = np.hstack([np.hstack(q_temp), -QN.dot(x_ref_data[:, N-1]), np.zeros(N*nu)])

# splitting the costs for better debug
q = np.hstack([qx, qu])

# ==============================================================================================================
# Inequality constraints

# 1- bounds on states
selectStates = np.hstack([np.eye((N + 1) * nx), np.zeros(((N + 1) * nx, N * nu))])
lowerBoundX = np.kron(np.ones(N + 1), x_min)
upperBoundX = np.kron(np.ones(N + 1), x_max)

# 2- For inputs
util = Utils()
Stance_diag = np.eye(nu * N) - Sdiag  # this are ones for stance legs

# this is a selection matrix that sets to zero the elements of the matrix correspondent to swing states
selectStance = np.hstack([np.zeros((N * nu, (N + 1) * nx)), Stance_diag])

# friction cone constraints
Xpos = np.vstack([np.hstack([1, 0, mu[util.leg_map("LF")], np.zeros(nu - 3)]),
                  np.hstack([np.zeros(3), 1, 0, mu[util.leg_map("RF")], np.zeros(nu - 6)]),
                  np.hstack([np.zeros(nu - 6), 1, 0, mu[util.leg_map("LH")], np.zeros(3)]),
                  np.hstack([np.zeros(nu - 3), 1, 0, mu[util.leg_map("RH")]])])
# zeroing  X components for swing legs
coneMatrixXpos = np.kron(np.eye(N), Xpos).dot(selectStance)

# --------------------------------------------
Xneg = np.vstack([np.hstack([-1, 0, mu[util.leg_map("LF")], np.zeros(nu - 3)]),
                  np.hstack([np.zeros(3), -1, 0, mu[util.leg_map("RF")], np.zeros(nu - 6)]),
                  np.hstack([np.zeros(nu - 6), -1, 0, mu[util.leg_map("LH")], np.zeros(3)]),
                  np.hstack([np.zeros(nu - 3), -1, 0, mu[util.leg_map("RH")]])])
# zeroing X components for swing legs
coneMatrixXneg = np.kron(np.eye(N), Xneg).dot(selectStance)

# --------------------------------------------
Ypos = np.vstack([np.hstack([0, 1, mu[util.leg_map("LF")], np.zeros(nu - 3)]),
                  np.hstack([np.zeros(3), 0, 1, mu[util.leg_map("RF")], np.zeros(nu - 6)]),
                  np.hstack([np.zeros(nu - 6), 0, 1, mu[util.leg_map("LH")], np.zeros(3)]),
                  np.hstack([np.zeros(nu - 3), 0, 1, mu[util.leg_map("RH")]])])
# zeroing  Y components for swing legs
coneMatrixYpos = np.kron(np.eye(N), Ypos).dot(selectStance)

# --------------------------------------------
# 4 X nu
Yneg = np.vstack([np.hstack([0, -1, mu[util.leg_map("LF")], np.zeros(nu - 3)]),
                  np.hstack([np.zeros(3), 0, -1, mu[util.leg_map("RF")], np.zeros(nu - 6)]),
                  np.hstack([np.zeros(nu - 6), 0, -1, mu[util.leg_map("LH")], np.zeros(3)]),
                  np.hstack([np.zeros(nu - 3), 0, -1, mu[util.leg_map("RH")]])])
# 4*N X (nu*N)
# zeroing  Y components for swing legs
coneMatrixYneg = np.kron(np.eye(N), Yneg).dot(selectStance)

# put together
coneLHS = np.vstack([coneMatrixXpos, coneMatrixXneg, coneMatrixYpos, coneMatrixYneg])
lowerBoundCones = np.hstack([np.zeros((4 * N)), np.zeros((4 * N)), np.zeros((4 * N)), np.zeros((4 * N))])
upperBoundCones = np.hstack(
    [np.inf * np.ones(4 * N), np.inf * np.ones(4 * N), np.inf * np.ones(4 * N), np.inf * np.ones(4 * N)])

# 3  -unilaterality (bounds) Z component
selectZcomp = np.kron(np.eye(4 * N), [0, 0, 1])
# todo ref gen should provide n[j] at each sample instead of [0 0 1]
unilateralLHS = selectZcomp.dot(selectStance)

selectZcompVector = selectZcomp.dot(Stance_diag).dot(np.ones(N * nu))
upperBoundUz = np.diag(selectZcompVector).dot(np.kron(np.ones(N), [z_force_max[util.leg_map("LF")],
                                                                   z_force_max[util.leg_map("RF")],
                                                                   z_force_max[util.leg_map("LH")],
                                                                   z_force_max[util.leg_map("RH")]]))
lowerBoundUz = np.diag(selectZcompVector).dot(np.kron(np.ones(N), [z_force_min[util.leg_map("LF")],
                                                                   z_force_min[util.leg_map("RF")],
                                                                   z_force_min[util.leg_map("LH")],
                                                                   z_force_min[util.leg_map("RH")]]))

# ==============================================================================================================
# - Wrapping up all the constraints for the solver

# A = sparse.vstack([Aeq, selectStates,  coneLHS,         unilateralLHS]).tocsc()
# l = np.hstack(    [leq, lowerBoundX,   lowerBoundCones, lowerBoundUz])
# u = np.hstack(    [ueq, upperBoundX,   upperBoundCones, upperBoundUz])

A = sparse.csc_matrix(Aeq)
l = leq
u = ueq

#---------------------------------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------------------------------
# # ==============================================================================================================
# # - input and state constraints (Inequality constraints)
# mu=0.8
# conmat = np.array([[1, 0, mu], [0, 1, mu], [0, 0, 1]])  # Friction cone constraint matrix for lower bounds
#
# # Inequalities for the lower bounds
# tempMat1 = np.hstack([np.eye((N + 1) * nx), np.zeros(((N + 1) * nx, N * nu))])  # Place holder matrix
# tempMat2 = np.hstack([np.zeros((N * nu, (N + 1) * nx)), np.kron(np.eye(N * 4), conmat)])  # Place holder matrix
# Aineql = sparse.vstack([tempMat1, tempMat2])
#
# conmat2 = np.array([[-1, 0, mu], [0, -1, mu], [0, 0, -1]])  # Friction cone constraint matrix for upper bounds
#
# # Inequalities for the upper bounds
# tempMat3 = np.hstack([-np.eye((N + 1) * nx), np.zeros(((N + 1) * nx, N * nu))])  # Place holder matrix
# tempMat4 = np.hstack([np.zeros((N * nu, (N + 1) * nx)), np.kron(np.eye(N * 4), conmat2)])  # Place holder matrix
# Ainequ = sparse.vstack([tempMat3, tempMat4])
#
# # constraint vectors for grf forces
# u_max = 1000
# u_min = 0
# uz_min = np.kron(np.ones(4), np.array([0.0, 0.0, u_min]))
# uz_max = np.kron(np.ones(4), np.array([0.0, 0.0, u_max]))
#
# # RHS of lower and upper bounds
# lineql = np.hstack([np.kron(np.ones(N + 1), -np.inf * np.ones(nx)), np.kron(np.ones(N), uz_min)])
# uinequ = -np.hstack([np.kron(np.ones(N + 1), np.inf * np.ones(nx)), np.kron(np.ones(N), uz_max)])
#
# # ==============================================================================================================
# # - Wrapping up all the constraints for the solver
# A = sparse.vstack([Aeq, Aineql, Ainequ]).tocsc()
# l = np.hstack([leq, lineql, uinequ])
# u = np.hstack([ueq, np.inf * np.ones(2 * np.size(lineql))])

#---------------------------------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------------------------------
#%%
# QP problem parameter wrapped in a dictionary for the solvers
problem = {}
problem['P'] = P
problem['Px'] = Px
problem['Pu'] = Pu
problem['qx'] = qx
problem['qu'] = qu
problem['q'] = q
problem['A'] = A
problem['l'] = l
problem['u'] = u
problem['m'] = A.shape[0]
problem['n'] = A.shape[1]

#==============================================================================================================
# Create an OSQP object

if selOsqpsolver == 1:
    prob = osqp.OSQP()

    # Setup workspace
    prob.setup(P, q, A, l, u, eps_abs = 1e-03, eps_rel = 1e-08, polish = True) #, warm_start=False, eps_abs = 1e-03, eps_rel = 1e-03, polish = True
    # prob.update_settings(eps_abs = 1e-08)

    tic()
    res = prob.solve()
    toc()


    #unfold the optimization variables
    x_pred=np.zeros((nx,N+1))
    u_opt=np.zeros((nu,N))
    k=0
    for k in range(N+1):
        x_pred[:,k] = res.x[(k*nx):(k*nx)+nx]
        if k <= N-1:
            u_opt[:,k] = res.x[(nx*(N+1))+(k*12):(nx*(N+1))+(k*12)+12]
else:
    # Gurobi solver
    solveOpt=GUROBISolver(problem)

    tic()
    Result = solveOpt.solve(problem)
    toc()
    # print "Elapsed time is " + str(time.time() - start_time) + " seconds."

    #unfold the optimization result
    x_pred=np.zeros((nx,N+1))
    u_opt=np.zeros((nu,N))
    k=0
    for k in range(N+1):
        x_pred[:,k] = Result.x[(k*nx):(k*nx)+nx]
        if k <= N-1:
            u_opt[:,k] = Result.x[(nx*(N+1))+(k*12):(nx*(N+1))+(k*12)+12]
XX = x_pred.T.flatten()
UU = u_opt.T.flatten()

xrefcost =  np.hstack([x_lin_data.transpose().flatten(), x_lin_data[:,-1]])
urefcost =  u_lin_data.transpose().flatten()
# State cost
Xobj_val = 0.5 * XX.dot(Px.todense()).dot(XX) - xrefcost.dot(Px.todense()).dot(XX)

# Input cost
Uobj_val = 0.5 * UU.dot(Pu.todense()).dot(UU) - urefcost.dot(Pu.todense()).dot(UU)

# Total calc. cost
TotObj_val = Xobj_val + Uobj_val


#to debug the sum of costs is the same
#TotObj_val = 0.5 * np.hstack([XX, UU]).dot(P.todense()).dot(np.hstack([XX, UU])) + q.dot(np.hstack([XX, UU]))

# Difference between Gurboi and calc. cost
# Diff_obj = result.obj_val - TotObj_val

print ('state cost:', Xobj_val[0, 0])
print ('force cost:', Uobj_val[0, 0])
print ('total cost:', TotObj_val[0, 0])
# %% Input plots

plt.rcParams['axes.grid'] = True
plt.close('all')

plt.figure()
plt.subplot(6, 2, 1)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[0,:], label="Optimal")
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[0, :], label="Reference",
         linestyle='--')
# plt.legend()
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
plt.ylabel(r"$(u_{LF})_x$", fontsize=10)

plt.subplot(6, 2, 3)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[1,:])
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[1, :], linestyle='--')
plt.ylabel(r"$(u_{LF})_y$", fontsize=10)

plt.subplot(6, 2, 5)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[2,:])
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[2, :], linestyle='--')
plt.ylabel(r"$(u_{LF})_z$", fontsize=10)

plt.subplot(6, 2, 2)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[3,:], label="Optimal")
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[3, :], linestyle='--', label="Reference")
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
plt.ylabel(r"$(u_{RF})_x$", fontsize=10)

plt.subplot(6, 2, 4)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[4,:])
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[4, :], linestyle='--')
plt.ylabel(r"$(u_{RF})_y$", fontsize=10)

plt.subplot(6, 2, 6)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[5,:])
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[5, :], linestyle='--')
plt.ylabel(r"$(u_{RF})_z$", fontsize=10)

plt.subplot(6, 2, 7)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[6,:])
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[6, :], linestyle='--')
plt.ylabel(r"$(u_{LH})_x$", fontsize=10)

plt.subplot(6, 2, 9)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[7,:])
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[7, :], linestyle='--')
plt.ylabel(r"$(u_{LH})_y$", fontsize=10)

plt.subplot(6, 2, 11)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[8,:])
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[8, :], linestyle='--')
plt.ylabel(r"$(u_{LH})_z$", fontsize=10)

plt.subplot(6, 2, 8)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[9,:])
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[9, :], linestyle='--')
plt.ylabel(r"$(u_{RH})_x$", fontsize=10)

plt.subplot(6, 2, 10)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[10,:])
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[10, :], linestyle='--')
plt.ylabel(r"$(u_{RH})_y$", fontsize=10)

plt.subplot(6, 2, 12)
plt.plot(rosData.simTime[0, simStart:simStart + N], u_opt[11,:])
plt.plot(rosData.simTime[0, simStart:simStart + N], u_lin_data[11, :], linestyle='--')
plt.ylabel("$(u_{RH})_z$", fontsize=10)
plt.suptitle('Inputs: Ground reaction forces')
plt.show()

# %% COM position and speed reference and predictions by optimizer

plt.figure()
plt.subplot(3, 2, 1)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[0, :], label="Predicted")
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[0, :], label="Reference",
         linestyle='--')
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
plt.ylabel("$COM_x$", fontsize=10)
# plt.legend()

plt.subplot(3, 2, 3)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[1, :])
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[1, :], linestyle='--')
plt.ylabel("$COM_y$", fontsize=10)

plt.subplot(3, 2, 5)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[2, :])
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[2, :], linestyle='--')
plt.ylabel("$COM_z$", fontsize=10)

plt.subplot(3, 2, 2)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[3, :], label="Predicted")
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[3, :], linestyle='--', label="Reference")
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
plt.ylabel("$COMV_x$", fontsize=10)

plt.subplot(3, 2, 4)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[4, :])
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[4, :], linestyle='--')
plt.ylabel("$COMV_y$", fontsize=10)

plt.subplot(3, 2, 6)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[5, :])
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[5, :], linestyle='--')
plt.ylabel("$COMV_z$", fontsize=10)
plt.suptitle('COM linear position and velocity')
plt.show()

# %% COM angular position and speed reference and predictions by optimizer

plt.figure()
plt.subplot(3, 2, 1)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[6, :], label="Predicted")
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[6, :], label="Reference",
         linestyle='--')
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
plt.ylabel("$COM\,roll$", fontsize=10)
# plt.legend()

plt.subplot(3, 2, 3)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[7, :])
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[7, :], linestyle='--')
plt.ylabel("$COM\,pitch$", fontsize=10)

plt.subplot(3, 2, 5)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[8, :])
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[8, :], linestyle='--')
plt.ylabel("$COM\,yaw$", fontsize=10)

plt.subplot(3, 2, 2)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[9, :], label="Predicted")
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[9, :], linestyle='--', label="Reference")
plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
plt.ylabel("$\omega_x$", fontsize=10)

plt.subplot(3, 2, 4)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[10, :])
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[10, :], linestyle='--')
plt.ylabel("$\omega_y$", fontsize=10)

plt.subplot(3, 2, 6)
plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[11, :])
plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[11, :], linestyle='--')
plt.ylabel("$\omega_z$", fontsize=10)
plt.suptitle('COM angular position and velocity')
plt.show()


# %% Gravity terms
# plt.figure()
# plt.subplot(3, 1, 1)
# plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[12, :], label="Predicted")
# plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[12, :], label="Reference",
#          linestyle='--')
# plt.ylabel("$g\,x$", fontsize=10)
# # plt.legend()
#
# plt.subplot(3, 1, 2)
# plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[13, :])
# plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[13, :], linestyle='--')
# plt.ylabel("$g\,y$", fontsize=10)
#
# plt.subplot(3, 1, 3)
# plt.plot(rosData.simTime[0, simStart:simStart + N + 1],x_pred[14, :])
# plt.plot(rosData.simTime[0, simStart:simStart + N],x_lin_data[14, :], linestyle='--')
# plt.ylabel("$g\,z$", fontsize=10)
# plt.suptitle('Gravity states')
# plt.show()


# plt.figure()
# plt.plot(rosData.swingLF.T)
# plt.plot(rosData.swingRF.T)
# plt.plot(rosData.swingLH.T)
# plt.plot(rosData.swingRH.T)
# plt.show()

# OptimalCntrl = {'x_pred': x_pred, 'u_optml': u_opt, 'A_lin_data': A_lin_data, 'B_lin_data':B_lin_data, 'affineTerm':affineTerm}
# scipy.io.savemat('OptimalCntrl', {'OptimalCntrl':OptimalCntrl})
