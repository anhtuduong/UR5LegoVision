"""
Optimal control of a quadruped based on centroidal dynamic model
"""
# Author: Niraj Rathod
import sys
import os

#export  needed for t_renderer
os.environ['ACADOS_SOURCE_DIR'] = os.environ['HOME'] + '/' + os.environ['ROS_WORKSPACE_NAME'] + '/src/dls-distro/acados'

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
                 os.environ['ACADOS_SOURCE_DIR'],
                 '' ]#this is the current directory
sys.path.append('../../.')
# print(sys.path)
import time, os
import numpy as np
from hyqAcados_settings import *
from refgen.robotDataFile import extractROSdata
from plotOcpResults import plotOcpResults
from tabulate import tabulate
from tools.mathutils import *
import scipy.io
import scipy as sp

# Mass
robotMass = 84.756

# Inertia
robotInertia = np.array([[4.07454, 0.145765, -0.224491],
                 [0.145765, 11.3576, -0.0133459],
                 [-0.224491, -0.0133459, 12.5675]])

# Friction cone parameters
friction_coeff = 0.8*np.ones(4)
friction_upperBound = 2000*np.ones(4)

# Reference distance between hip to foot position
hiptoFootPos = 0.55

# Time and horizon for acados
Ts = 0.04  # sampling time
N = 100  # number of discretization steps
Tf = N*Ts # prediction horizon

# Flags
model_type = 0 # 0 - world frame model, 1 - CoM frame model, 2 - delta_u model (control regulation)
include_mobility = 0

# unilaterality constraints
include_uniCons = 0
include_uniSlack = 0

# cone constraints
include_coneCons = 0
include_coneSlack = 0

# Include slacks for both cone and unilaterality
include_coneNuniSlack = 0

# Define the dimension of state and input
nx = 12
nu = 12
ng = 16 # number of cone constraints
ny = nx + nu

# %% Extract reference trajectory
# Load mat file and sort
# data = scipy.io.loadmat("../saveMpcOut.mat")
# x_lin_data = data["x_ref_traj"]
# x_predMpcpy = data["x_predMpcPy"]
# x_lin_data[:,0] = x_predMpcpy[:,0]
# u_lin_data = data["u_ref_traj"]
# xf_data = data["ref_feetW"]
# sortStance = data["stancVec"]
# simTime = np.arange(0, Tf, Ts)

data = scipy.io.loadmat("../../data/crawl_references_40ms.mat")
rosData = extractROSdata(data)
x_ref_data = np.zeros((nx, N))
u_ref_data = np.zeros((nu, N))
xf_data = np.zeros((nx, N))
sortSwing = np.zeros((4, N))
sortStance = np.zeros((4, N))
simStart =0
simTime = rosData.simTime[0, simStart:simStart + N + 1]

for i in range(N):
    sortSwing[:, i] = np.array([rosData.swingLF[0, simStart + i], rosData.swingRF[0, simStart + i],
                          rosData.swingLH[0, simStart + i], rosData.swingRH[0, simStart + i]]).flatten()
    sortStance[:, i] = np.logical_not(sortSwing[:, i]).astype(int)

for i in range(N):
    omega_c = rpyToRot(rosData.rollW[simStart + i],
                       rosData.pitchW[simStart + i],
                       rosData.yawW[simStart + i],)@ np.array([rosData.omegaXW[0, simStart + i],
                       rosData.omegaYW[0, simStart + i],
                       rosData.omegaZW[0, simStart + i]])

    x_ref_data[:, i] = np.hstack([rosData.actual_CoMXW[simStart + i],
                                 rosData.actual_CoMYW[simStart + i],
                                 rosData.actual_CoMZW[simStart + i],
                                 rosData.com_VxW[0,simStart + i],
                                 rosData.com_VyW[0,simStart + i],
                                 rosData.com_VzW[0,simStart + i],
                                 rosData.rollW[simStart + i],
                                 rosData.pitchW[simStart + i],
                                 rosData.yawW[simStart + i],
                                 omega_c])

    u_ref_data[:, i] = np.array([rosData.grForcesLFWx_gt[simStart + i],
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

    xf_data [:, i] = np.array([rosData.footPosLFWx[simStart + i, 0],
                        rosData.footPosLFWy[simStart + i, 0],
                        rosData.footPosLFWz[simStart + i, 0],
                        rosData.footPosRFWx[simStart + i, 0],
                        rosData.footPosRFWy[simStart + i, 0],
                        rosData.footPosRFWz[simStart + i, 0],
                        rosData.footPosLHWx[simStart + i, 0],
                        rosData.footPosLHWy[simStart + i, 0],
                        rosData.footPosLHWz[simStart + i, 0],
                        rosData.footPosRHWx[simStart + i, 0],
                        rosData.footPosRHWy[simStart + i, 0],
                        rosData.footPosRHWz[simStart + i, 0]])

    # sortSwing[:, i] = np.array([rosData.swingLF[0, simStart + i] * np.ones(3), rosData.swingRF[0, simStart + i] * np.ones(3),
                          # rosData.swingLH[0, simStart + i] * np.ones(3), rosData.swingRH[0, simStart + i] * np.ones(3)]).flatten()
    sortSwing[:, i] = np.array([rosData.swingLF[0, simStart + i], rosData.swingRF[0, simStart + i],
                          rosData.swingLH[0, simStart + i], rosData.swingRH[0, simStart + i]]).flatten()
    sortStance[:, i] = np.logical_not(sortSwing[:, i]).astype(int)

#%% Define preliminaries
if model_type == 2:
    x_lin_data = np.vstack([x_ref_data,u_ref_data])
    u_lin_data = np.zeros((nu,N))

    # Set state penalties
    Q = np.zeros((nx + nu, nx + nu))
    Q[0, 0] = 0.0
    Q[1, 1] = 0.0
    Q[2, 2] = 1000.0

    Q[3, 3] = 1.0e2
    Q[4, 4] = 1.0e2
    Q[5, 5] = 1.0e2

    Q[6, 6] = 0.0e2
    Q[7, 7] = 0.0e2
    Q[8, 8] = 1.0e2

    Q[9, 9] = 1.0e1
    Q[10, 10] = 1.0e1
    Q[11, 11] = 1.0e2

    Q[12, 12] = 1.0e-3
    Q[13, 13] = 1.0e-3
    Q[14, 14] = 1.0e-5

    Q[15, 15] = 1.0e-3
    Q[16, 16] = 1.0e-3
    Q[17, 17] = 1.0e-5

    Q[18, 18] = 1.0e-3
    Q[19, 19] = 1.0e-3
    Q[20, 20] = 1.0e-5

    Q[21, 21] = 1.0e-3
    Q[22, 22] = 1.0e-3
    Q[23, 23] = 1.0e-5

    # Set input penalties
    R = np.zeros((nu, nu))
    R[0, 0] = 1.0e-8
    R[1, 1] = 1.0e-8
    R[2, 2] = 1.0e-8

    R[3, 3] = 1.0e-8
    R[4, 4] = 1.0e-8
    R[5, 5] = 1.0e-8

    R[6, 6] = 1.0e-8
    R[7, 7] = 1.0e-8
    R[8, 8] = 1.0e-8

    R[9, 9] = 1.0e-8
    R[10, 10] = 1.0e-8
    R[11, 11] = 1.0e-8

else:
    x_lin_data = x_ref_data
    u_lin_data = u_ref_data

    # Set state penalties

    Q = np.zeros((nx, nx))
    Q[0, 0] = 0.0
    Q[1, 1] = 0.0
    Q[2, 2] = 1000.0

    Q[3, 3] = 1.0e2
    Q[4, 4] = 1.0e2
    Q[5, 5] = 1.0e2

    Q[6, 6] = 0.0e2
    Q[7, 7] = 0.0e2
    Q[8, 8] = 1.0e2

    Q[9, 9] = 1.0e1
    Q[10, 10] = 1.0e1
    Q[11, 11] = 1.0e2

    # Set input penalties
    R = np.zeros((nu, nu))
    R[0, 0] = 1.0e-3
    R[1, 1] = 1.0e-3
    R[2, 2] = 1.0e-5

    R[3, 3] = 1.0e-3
    R[4, 4] = 1.0e-3
    R[5, 5] = 1.0e-5

    R[6, 6] = 1.0e-3
    R[7, 7] = 1.0e-3
    R[8, 8] = 1.0e-5

    R[9, 9] = 1.0e-3
    R[10, 10] = 1.0e-3
    R[11, 11] = 1.0e-5

# For discrete to continuous-time cost
Q = Q/Ts

# For discrete to continuous-time cost
R = R/Ts

# Mobility weight
M = np.zeros((4, 4))
M[0, 0] = 1.0e+02
M[1, 1] = 1.0e+02
M[2, 2] = 1.0e+02
M[3, 3] = 1.0e+02
M = M/Ts

# Set terminal penalties (Same for continuous-time)
QN = Q*Ts

# Bounds on states
x_min = -1.0e10 * np.ones(nx)
x_max = 1.0e10 * np.ones(nx)

# Bounds on inputs
u_min = 0*np.ones(4)
u_max = 1000*np.ones(4)

# feasibleTraj = scipy.io.loadmat("NLPoutput.mat")
# x_predNLP = feasibleTraj['x_predNLP']
# u_optmlNLP = feasibleTraj['u_optmlNLP']

# Parameters for acados
args = {'robotMass': robotMass,
        'robotInertia': robotInertia,
        'friction_upperBound': friction_upperBound,
        'mu': friction_coeff,
        'ng': ng,
        'nx': nx,
        'nu': nu,
        'Q': Q,
        'R': R,
        'Qe': QN,
        'M': M,
        'x_min': x_min,
        'x_max': x_max,
        'u_min': u_min,
        'u_max': u_max,
        'model_type': model_type,
        'include_mobility': include_mobility,
        'include_coneCons': include_coneCons,
        'include_uniCons': include_uniCons,
        'include_coneSlack' : include_coneSlack,
        'include_uniSlack' : include_uniSlack,
        'include_coneNuniSlack' : include_coneNuniSlack,
        'xf0' : np.hstack([xf_data[:,0],sortStance[:, 0]])
        }

if include_mobility == 1:
    args['yref'] = np.hstack([x_lin_data[:,0], u_lin_data[:,0], np.zeros(4)])
else:
    args['yref'] = np.hstack([x_lin_data[:,0], u_lin_data[:,0]])

if model_type == 2:
    args['yref_e'] = np.hstack([x_lin_data[:12, -1], np.zeros(nu)])
    if include_uniCons:
        args['x0'] = np.hstack([x_lin_data[:12, 0], 0*np.ones(4)])
    else:
        args['x0'] = x_lin_data[:12, 0]
else:
    args['yref_e']= x_lin_data[:, -1]
    args['x0'] = x_lin_data[:12, 0]

#=====================================================================
# use feasible trajectory for debugging
# args.x0 = x_predNLP[:,0]
# args.xf0 = xf_data[:,0]
# args.yref = np.hstack([x_predNLP[:,0], u_optmlNLP[:,0]])
# args.yref_e = x_predNLP[:, -1]

# use constant reference/linearization trajectory for debugging
# m = 85.446
# force=m*9.81/4
# u_ref = np.array([0, 0,force, 0, 0,force, 0, 0,force, 0, 0,force])
#
# args.x0 = np.array(np.hstack([0, 0, 0.55, np.zeros(9)]))
# args.xf0 = xf_data[:,0]
# args.yref = np.hstack([args.x0, u_ref])
# args.yref_e = args.x0
#=====================================================================

# load the model
model, acados_solver = hyqAcados_settings(Tf, N, args)

# %% Run the controller
# Initialize opt outputs
if model_type == 2:
    x_acados = np.zeros((nx+nu, N+1))
    u_acados = np.zeros((nu, N))
else:
    x_acados = np.zeros((nx, N+1))
    u_acados = np.zeros((nu, N))
# update references, foot locations and input bounds over the horizon N
Nsim = 1 # number of RTI runs
tcomp = np.zeros(Nsim)
tcomp_sum = 0
tcomp_max = 0

# update initial condition
u0_lb =np.array([u_min[0] * sortStance[0, 0],
                 u_min[1] * sortStance[1, 0],
                 u_min[2] * sortStance[2, 0],
                 u_min[3] * sortStance[3, 0]])
u0_ub = 1000*np.ones(4)
x0_lb = np.hstack([x_lin_data[:12,0], u0_lb])
x0_ub = np.hstack([x_lin_data[:12,0], u0_ub])

if model_type == 2 and include_uniCons:
    acados_solver.set(0, "lbx", x0_lb)
    acados_solver.set(0, "ubx", x0_ub)
else:
    acados_solver.set(0, "lbx", args['x0'])
    acados_solver.set(0, "ubx", args['x0'])

# State intial guess to SQP (state linerization trajectory)
for i in range(N):
    acados_solver.set(i, "x", x_lin_data[:,i])

acados_solver.set(N, "x", x_lin_data[:,N-1])

# Input initial guess to SQP (input linerization trajectory)
for i in range(N):
    acados_solver.set(i, "u", u_lin_data[:,i])

for i in range(Nsim):
    if i <1:
        for j in range(N):
            # update state reference
            if include_mobility == 1:
                yref = np.hstack([x_lin_data[:, j], u_lin_data[:, j], hiptoFootPos*np.ones(4)])
            else:
                yref = np.hstack([x_lin_data[:, j], u_lin_data[:, j]])

            acados_solver.set(j, "y_ref", yref)
            if j> 0 and model_type == 2 and include_uniCons == 1:
                acados_solver.set(j, "lbx", np.array([u_min[0] * sortStance[0, j],
                                   u_min[1] * sortStance[1, j],
                                   u_min[2] * sortStance[2, j],
                                   u_min[3] * sortStance[3, j]]))
            elif model_type != 2 and include_uniCons == 1:
                lbound =np.array([u_min[0]*sortStance[0, j],
                                                  u_min[1]*sortStance[1, j],
                                                  u_min[2]*sortStance[2, j],
                                                  u_min[3]*sortStance[3, j]])
                acados_solver.set(j, "lbu",lbound )
            # acados_solver.cost_set(j, "W", sp.linalg.block_diag(Q, R) )

            # update the foot location and stance vector
            acados_solver.set(j, "p", np.hstack([xf_data[:,j],sortStance[:, j]]))

        # print('update only for first iter')
    # print(i)

    if i<1:
        # update terminal state reference
        # yref_N = x_lin_data[:,-1] WATCHOUT
        if model_type == 2:
            yref_N = np.hstack([x_lin_data[:12, -1], np.zeros(nu)])
        else:
            yref_N = x_lin_data[:, -1]

        acados_solver.set(N, "y_ref", yref_N)

    # solve ocp
    t = time.time()

    status = acados_solver.solve()
    if status != 0:
        raise Exception("acados returned status {}. Exiting.".format(status))

    elapsed = time.time() - t
    tcomp[i] = elapsed

    # manage timings
    tcomp_sum += elapsed
    if elapsed > tcomp_max:
        tcomp_max = elapsed

#%% get solution
for j in range(N+1):
    # x_pred[:,j] = acados_solver.get(j, "x")
    x_extract = acados_solver.get(j, "x") # contains in omega in CoM frame
    if model_type == 0:
        x_acados[:, j] = x_extract
    elif model_type == 1:
        omega_w = rpyToRot(x_extract[6],x_extract[7],x_extract[8]).T @ x_extract[9:12] # omega in world frame
        x_acados[:,j] = np.hstack([x_extract[0:9], omega_w])
    else:
        omega_w = rpyToRot(x_extract[6], x_extract[7], x_extract[8]).T @ x_extract[9:12]  # omega in world frame
        x_acados[:, j] = np.hstack([x_extract[0:9], omega_w, x_extract[12:]])
    if j<N:
        u_acados[:,j] = acados_solver.get(j, "u")

if model_type == 2:
    x_pred = x_acados[:12, :]
    u_optml = x_acados[12:, :-1]
    delta_u = u_acados
else:
    x_pred = x_acados
    u_optml = u_acados

# Print slacks
# sl = acados_solver.get(1, "sl")
# su = acados_solver.get(1, "su")
# print("sl", sl, "su", su)

# Compute the cost
Xobj_val = 0
Uobj_val = 0
Mobj_val = 0

# This cost is only valid for one iteration of RTI
for i in range(N):
    # State cost
    Xobj_val = Xobj_val + 0.5 * ((x_pred[:, i]-x_lin_data[:nx, i]) @ (Q[:nx,:nx]*Ts) @ (x_pred[:, i]-x_lin_data[:nx, i]))

    # Input cost
    if model_type == 2:
        Uobj_val = Uobj_val + 0.5 * (
                    (u_optml[:, i] - x_lin_data[nu:, i]) @ (Q[nu:,nu:] * Ts) @ (u_optml[:, i] - x_lin_data[nu:, i]))
    else:
        Uobj_val = Uobj_val + 0.5 * ((u_optml[:, i] - u_lin_data[:, i]) @ (R*Ts) @ (u_optml[:, i] - u_lin_data[:, i]))

    # Mobility cost
    if include_mobility == 1:
        # Euclidean distance of hip to foot position
        # [hPosLFwPred, hPosRFwPred, hPosLHwPred, hPosRHwPred] = hipPositionW(x_pred[0:3, i],
        #                                                                     x_pred[6, i],
        #                                                                     x_pred[7, i],
        #                                                                     x_pred[8, i])
        # hipToFposLF = np.linalg.norm(hPosLFwPred - self.xf_data[0:3, i])
        # hipToFposRF = np.linalg.norm(hPosRFwPred - self.xf_data[3:6, i])
        # hipToFposLH = np.linalg.norm(hPosLHwPred - self.xf_data[6:9, i])
        # hipToFposRH = np.linalg.norm(hPosRHwPred - self.xf_data[9:12, i])

        # Z distance of hip to foot position
        [LF_z, RF_z, LH_z, RH_z] = hipZPositionW(x_pred[0:3, i], x_pred[6, i], x_pred[7, i], x_pred[8, i])
        hipToFposLF = LF_z - xf_data[2, i]
        hipToFposRF = RF_z - xf_data[5, i]
        hipToFposLH = LH_z - xf_data[8, i]
        hipToFposRH = RH_z - xf_data[11, i]

        hipResidual = np.array([hipToFposLF - hiptoFootPos,
                                hipToFposRF - hiptoFootPos,
                                hipToFposLH - hiptoFootPos,
                                hipToFposRH - hiptoFootPos])

        Mobj_val = Mobj_val + 0.5 * hipResidual @ (M * Ts) @ hipResidual

Xobj_val = Xobj_val + 0.5 * ((x_pred[:, -1]-x_lin_data[:nx, -1]) @ QN[:nx,:nx] @ (x_pred[:, -1]-x_lin_data[:nx, -1]))

# Total calc. cost
TotObj_val = Xobj_val + Uobj_val + Mobj_val
print(tabulate([['SQP iter', acados_solver.get_stats("sqp_iter")[0]],
                ['Total time [ms]', acados_solver.get_stats("time_tot")[0]*1000],
                ['lin time [ms]', acados_solver.get_stats("time_lin")[0]*1000],
                ['QP solve time [ms]', acados_solver.get_stats("time_qp")[0]*1000],
                ['state Obj', Xobj_val],
                ['input Obj', Uobj_val],
                ['Total Obj', TotObj_val]], headers=['Param', 'value']))

acados_solver.print_statistics()

# Save .mat file for plots in matlab
# SqpOutputPython = {'x_pred': x_pred, 'u_optml': u_optml, 'xf_data': xf_data, 'x_lin_data':x_lin_data, 'u_lin_data':u_lin_data, 'simTime':simTime}
# scipy.io.savemat('SqpOutputPython', {'SqpOutputPython':SqpOutputPython})

#%% plot result
if N==simTime.__len__():
    simTime=np.hstack([simTime, simTime[-1]+Ts])

if model_type == 2:
    plotOcpResults(simTime, x_pred, x_lin_data, u_optml, x_lin_data[12:, :])
else:
    plotOcpResults(simTime, x_pred, x_lin_data, u_optml, u_lin_data)

# plotOcpResults(simTime, x_pred, x_predNLP[:,:N], u_optml, u_optmlNLP[:,:N])


