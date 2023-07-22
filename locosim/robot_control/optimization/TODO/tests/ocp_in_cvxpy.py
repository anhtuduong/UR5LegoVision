# -*- coding: utf-8 -*-
"""
Created on Wed Apr 10 00:08:28 2019

@author: Niraj Rathod

"""
import sys
import os
print()

#print(sys.version_info)
if sys.version_info[:2] == (2, 7):
    print ("USING python 2.7")
else:
    sys.path = ['/usr/lib/python3.5/lib-dynload',  # package mmap
                '/usr/lib/python3.5/',  # numbers.py is here
                '/usr/local/lib/python3.5/dist-packages/',  # sudo pip3 install installs here numpy
                '/usr/lib/python3/dist-packages/',  # sudo pip3 install installs here yaml
                '/opt/ros/kinetic/lib/python2.7/dist-packages/',  # rospy genpy
                '/usr/lib/python2.7/dist-packages/',  # rospkg is here
                os.environ['HOME'] + '/' + os.environ['ROS_WORKSPACE_NAME'] + '/install/lib/python2.7/dist-packages',
                '../.']  # this is the current directory
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

# from gurobipy import *
from model.run_linearization import RunLinearization
from refgen.robotDataFile import extractROSdata
from tools.mathutils import *
from tools.utils import Utils

np.set_printoptions(linewidth = 200, suppress = True)
# import time
data = scipy.io.loadmat("../data/crawl_references_40ms.mat")


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

robotMass =  84.756

# Essential flags
selOsqpsolver = 0
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
stance = np.zeros((4, N))
sortSwing = np.zeros((4, N))
f_xu_lin_data = []
affineTerm = []

# Instantiate the linearization class
linearize_model = RunLinearization(nx, nu, Ts, robotInertia, robotMass, 0)

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
    sortSwingSmat = np.array([rosData.swingLF[0, simStart + i] * np.ones(3), rosData.swingRF[0, simStart + i] * np.ones(3),
                          rosData.swingLH[0, simStart + i] * np.ones(3), rosData.swingRH[0, simStart + i] * np.ones(3)])

    S_mat_data.append(np.diag(sortSwingSmat.flatten()))

    sortSwing[:, i] = np.array([rosData.swingLF[0, simStart + i], rosData.swingRF[0, simStart + i],
                                rosData.swingLH[0, simStart + i], rosData.swingRH[0, simStart + i]]).flatten()
    stance[:, i] = np.logical_not(sortSwing[:, i]).astype(int)

    if include_ltv_model == 1:
        Adisc, Bdisc, fxu = linearize_model.get_lin_matrices(x_lin_data[:, i], u_lin_data[:, i],
                                                                        footPos, stance[:, i])
        A_lin_data.append(Adisc)
        B_lin_data.append(Bdisc)
        f_xu_lin_data.append(fxu)
    else:
        if i == 0:
            Adisc, Bdisc, rk = linearize_model.get_lin_matrices(x_lin_data[:, 0], u_lin_data[:, 0],
                                                                          footPos, stance[:, i])
        A_lin_data.append(Adisc)
        B_lin_data.append(Bdisc)
        f_xu_lin_data.append(rk)
    # sortSwing = np.array([rosData.swingLF[simStart + i], rosData.swingRF[simStart + i],
    #                       rosData.swingLH[simStart + i], rosData.swingRH[simStart + i]])



    if include_ltv_model == 1:
        affineTerm.append(f_xu_lin_data[i])
    else:
        affineTerm.append(f_xu_lin_data[0])
lin_time = (time.time() - start_t_lin)
print ('Linearization time:', lin_time)
# %% QP formulation

# # Objective function
# Q = 1e3 * sparse.eye(nx)
# QN = Q
# R = 1e-5 * sparse.eye(nu)
# weight on states
com_x = 0.0
com_y = 0.0
com_z = 1.0e+03

linvel_x = 1.0e+02
linvel_y = 1.0e+02
linvel_z = 1.0e+03

ang_x = 1.0e+02
ang_y = 1.0e+02
ang_z = 1.0e+02

angvel_x = 1.0e+01
angvel_y = 1.0e+01
angvel_z = 1.0e+01

Q = sparse.diags(np.array([com_x, com_y, com_z, linvel_x, linvel_y, linvel_z, ang_x, ang_y, ang_z, angvel_x, angvel_y, angvel_z]))

QN = Q.todense()
#Weights on inputs
LF_x = 1.0e-03
LF_y = 1.0e-03
LF_z = 1.0e-05

RF_x = 1.0e-03
RF_y = 1.0e-03
RF_z = 1.0e-05

LH_x = 1.0e-03
LH_y = 1.0e-03
LH_z = 1.0e-05

RH_x = 1.0e-03
RH_y = 1.0e-03
RH_z = 1.0e-05

R= sparse.diags(np.array([LF_x, LF_y, LF_z, RF_x, RF_y, RF_z,LH_x, LH_y, LH_z, RH_x, RH_y, RH_z]))

# Lambda=1e-3 * sparse.eye(nu)
Lambda= 0.0 * sparse.eye(nu)

x_init_data = x_lin_data[:, 0]

util = Utils()
#definition of Symbolic variables for optimization used by CVXPY
# state vector
x = cp.Variable((nx, N+1))

# input forces
u = cp.Variable((nu, N))

# Parameter for the initial state
x_init = cp.Parameter(nx)


# Parameters for the references of states and inputs
x_ref = cp.Parameter((nx, N))
u_ref = cp.Parameter((nx, N))

# Matrix parameterization
A_lin = [cp.Parameter((nx, nx))]  # Matrix A
B_lin = [cp.Parameter((nx, nu))]  # Matrix B
S_mat = [cp.Parameter((nu, nu), diag = True)]  # Matrix D
f_xu_lin = [cp.Parameter(nx)]  # Parameter for function evaluated at x_lin and u_lin

ij = 1
while ij < N:
    A_lin.append(cp.Parameter((nx, nx)))
    B_lin.append(cp.Parameter((nx, nu)))
    S_mat.append(cp.Parameter((nu, nu)))
    f_xu_lin.append(cp.Parameter(nx))
    ij = ij+1

# Construct the problem


# Initialization of opt vars
objective = 0
constraints = []

# Formulate the optimazation problem
for t in range(N):

    # Quadratic cost: equation 1.8a
    objective += 0.5*cp.quad_form(x[:, t] - x_ref[:, t], Q) + 0.5*cp.quad_form(u[:, t]  - u_ref[:, t], R)
    # if t < N-1:
    #     objective += 0.5*cp.quad_form(u[:, t + 1] - u[:, t], Lambda)

    #dynamics xk = Ax + Bx Constraints on grfW_x and grfW_y : equation 1.8d and 1.8e
    constraints += [x[:, t + 1] == A_lin[t] * x[:, t] + B_lin[t] * u[:, t]  + f_xu_lin[t]]

    # #Constraints on grfW_z: equation 1.8c UNILATERALITY
    # constraints += [z_force_min[util.leg_map("LF")] <= u[util.getIdx("LF","Z"), t], u[util.getIdx("LF","Z"), t] <= z_force_max[util.leg_map("LF")]]
    # constraints += [z_force_min[util.leg_map("RF")] <= u[util.getIdx("RF","Z"), t], u[util.getIdx("RF","Z"), t] <= z_force_max[util.leg_map("RF")]]
    # constraints += [z_force_min[util.leg_map("LH")] <= u[util.getIdx("LH","Z"), t], u[util.getIdx("LH","Z"), t] <= z_force_max[util.leg_map("LH")]]
    # constraints += [z_force_min[util.leg_map("RH")] <= u[util.getIdx("RH","Z"), t], u[util.getIdx("RH","Z"), t] <= z_force_max[util.leg_map("RH")]]
    #
    # # # Constraints on grfW_x and grfW_y : equation 1.8d and 1.8e FRICTION CONES -mu fz < tx < mu fz
    # constraints += [-mu[util.leg_map("LF")]*u[util.getIdx("LF","Z"), t] <= u[util.getIdx("LF","X"), t] , u[util.getIdx("LF","X"), t]  <= mu[util.leg_map("LF")]*u[util.getIdx("LF","Z"), t] ]
    # constraints += [-mu[util.leg_map("RF")]*u[util.getIdx("RF","Z"), t] <= u[util.getIdx("RF","X"), t] , u[util.getIdx("RF","X"), t]  <= mu[util.leg_map("RF")]*u[util.getIdx("RF","Z"), t] ]
    # constraints += [-mu[util.leg_map("LH")]*u[util.getIdx("LH","Z"), t] <= u[util.getIdx("LH","X"), t] , u[util.getIdx("LH","X"), t]  <= mu[util.leg_map("LH")]*u[util.getIdx("LH","Z"), t] ]
    # constraints += [-mu[util.leg_map("RH")]*u[util.getIdx("RH","Z"), t] <= u[util.getIdx("RH","X"), t] , u[util.getIdx("RH","X"), t]  <= mu[util.leg_map("RH")]*u[util.getIdx("RH","Z"), t] ]
    # # Constraints on grfW_x and grfW_y : equation 1.8d and 1.8e FRICTION CONES -mu fz < ty < mu fz
    # constraints += [-mu[util.leg_map("LF")]*u[util.getIdx("LF","Z"), t] <= u[util.getIdx("LF","Y"), t] , u[util.getIdx("LF","Y"), t]  <= mu[util.leg_map("LF")]*u[util.getIdx("LF","Z"), t] ]
    # constraints += [-mu[util.leg_map("RF")]*u[util.getIdx("RF","Z"), t] <= u[util.getIdx("RF","Y"), t] , u[util.getIdx("RF","Y"), t]  <= mu[util.leg_map("RF")]*u[util.getIdx("RF","Z"), t] ]
    # constraints += [-mu[util.leg_map("LH")]*u[util.getIdx("LH","Z"), t] <= u[util.getIdx("LH","Y"), t] , u[util.getIdx("LH","Y"), t]  <= mu[util.leg_map("LH")]*u[util.getIdx("LH","Z"), t] ]
    # constraints += [-mu[util.leg_map("RH")]*u[util.getIdx("RH","Z"), t] <= u[util.getIdx("RH","Y"), t] , u[util.getIdx("RH","Y"), t]  <= mu[util.leg_map("RH")]*u[util.getIdx("RH","Z"), t] ]

    # Contraints for swing and stance: equation 1.18f
    # constraints += [S_mat[t] * u[:, t] == 0]

# indexZeroInputs = sortSwing[:, j].nonzero()[0]
# if len(indexZeroInputs) == 0:
#     pass
# else:
#     for indx in indexZeroInputs:
#         u_zero_min[indx] = 0
#         u_zero_max[indx] = 0

# Terminal cost and initial condition
objective += 0.5*cp.quad_form(x[:, N] - x_ref[:, N-1], QN)
constraints += [x[:, 0] == x_init]
# noinspection PyTypeChecker
problem = cp.Problem(cp.Minimize(objective), constraints)

start_t_opt = time.time()
# we define the ref traj equal to the lin traj

x_ref_data = x_lin_data
u_ref_data = u_lin_data

# Parameter updates with real numeric values
x_init.value = x_ref_data[:, 0]

#assign real values to the simbolic params
x_ref.value = x_ref_data
u_ref.value = u_ref_data

for jj in range(N):
    S_mat[jj].value = S_mat_data[jj]
    A_lin[jj].value = A_lin_data[jj]
    B_lin[jj].value = B_lin_data[jj]
    f_xu_lin[jj].value = affineTerm[jj]

# Solve the problem
result = problem.solve(solver='GUROBI', verbose=True)
#problem.solve(solver='ECOS',verbose=True)
opt_time = (time.time() - start_t_opt)
# print np.asarray(u.value)

print("status:", problem.status)
x_pred=np.asarray(x.value)
u_opt=np.asarray(u.value)

# %% Calculate the cost
# XX = x_pred.T.flatten()
# UU = u_opt.T.flatten()
#
# # - quadratic objective
# Px = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN])
# # Pu = sparse.block_diag([sparse.kron(sparse.eye(N), R)])
#
# # Define the diagonal matrix for delta_u in the cost
# Q_RLamda = sparse.block_diag([R + Lambda, sparse.kron(sparse.eye(N - 2), (R + 2*Lambda)), R + Lambda])
#
# # Create the off diagonal matrices
# OffdiagVec = np.hstack(np.tile(-Lambda.diagonal(),(N - 1, 1)))
# diagonals = [OffdiagVec, OffdiagVec]
# OffdiagMat = sparse.diags(diagonals,[-12, 12],)
#
# # Add these matrices to create the Pu weight matrix
# Pu = Q_RLamda + OffdiagMat
#
#
# # splitting the costs for better debug
# P = sparse.block_diag((Px, Pu)).tocsc()
#
# # to debug the sum of costs is the same
# # P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
# #                        sparse.kron(sparse.eye(N), R)]).tocsc()
#
# # - linear objective
# q_temp = []
# j = 0
#
# # Loop to create appropriate constraints for linear dynamic constraints for LTV system
# while j < N:
#     # for linear objective
#     q_temp.append(-Q.dot(x_ref_data[:, j]))
#     j = j + 1
# qx = np.hstack([np.hstack(q_temp), -QN.dot(x_ref_data[:, N - 1])])
# qu = np.zeros(N * nu)
#
# # to debug the sum of costs is the same
# # q = np.hstack([np.hstack(q_temp), -QN.dot(x_ref_data[:, N-1]), np.zeros(N*nu)])
#
# # splitting the costs for better debug
# q = np.hstack([qx, qu])
#
# # State cost
# Xobj_val = 0.5 * XX.dot(Px.todense()).dot(XX) + qx.dot(XX)
#            # +x_ref_data.transpose().flatten().dot(x_ref_data.transpose().flatten())
# # Input cost
# Uobj_val = 0.5 * UU.dot(Pu.todense()).dot(UU) + qu.dot(UU)
#            # +UU.transpose().dot()
#
# # Total calc. cost
# TotObj_val = Xobj_val + Uobj_val
#
# #to debug the sum of costs is the same
# # TotObj_val = 0.5 * np.hstack([XX, UU]).dot(P.todense()).dot(np.hstack([XX, UU])) + q.dot(np.hstack([XX, UU]))
#
# # Difference between Gurboi and calc. cost
# # Diff_obj = result.obj_val - TotObj_val
#
# print 'Obj value for states:', Xobj_val
# print 'Obj value for inputs:', Uobj_val
# print 'Total Obj value:', TotObj_val
# %% Calculate cost by hand
Xobj_val = 0
Uobj_val = 0
for t in range(N):
    xholder=x_pred[:, t] - x_ref_data[:, t]
    uholder=u_opt[:, t] - u_ref_data[:, t]

    # Quadratic cost: equation 1.8a
    Xobj_val +=  (xholder.transpose()).dot(0.5 * Q.todense()).dot(xholder)
    Uobj_val += (uholder.transpose()).dot(0.5 * R.todense()).dot(uholder)
    # if t < N-1:
    #       udiffholder = u_opt[:, t + 1] - u_opt[:, t]
    #       Uobj_val += (udiffholder.transpose()).dot(0.5 * Lambda.todense()).dot(udiffholder)
    #--------------------------------------------
          # 0.5*cp.quad_form(u[:, t + 1] - u[:, t], Lambda)

# Terminal cost and initial condition
xholderN=x_pred[:, N] - x_ref_data[:, N-1]
# Xobj_val += (xholderN.transpose()).dot(0.5 * QN.todense()).dot(xholderN)
Xobj_val += (xholderN.transpose()).dot(0.5 * QN).dot(xholderN)
    # 0.5*cp.quad_form(x_pred[:, N] - x_ref_data[:, N-1], QN)

# Total calc. cost
TotObj_val = Xobj_val + Uobj_val

print ('Obj value for states:', Xobj_val)
print ('Obj value for inputs:', Uobj_val)
print ('Total Obj value:', TotObj_val)
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

OptimalCntrlCVX = {'x_pred': x_pred, 'u_optml': u_opt, 'A_lin_data': A_lin_data, 'B_lin_data':B_lin_data, 'affineTerm':affineTerm}
scipy.io.savemat('OptimalCntrlCVX', {'OptimalCntrlCVX':OptimalCntrlCVX})
