"""
Test cvxpy, gurobi and osqp solvers with this script
Author: Niraj Rathod
Date : 17/10/20

"""

import sys
import os
import scipy.sparse as sparse
# print(sys.version_info)
if sys.version_info[:2] == (2, 7):
    print("USING python 2.7")
else:
    sys.path = ['/usr/lib/python3.5/lib-dynload',  # package mmap
                '/usr/lib/python3.5/',  # numbers.py is here
                '/usr/local/lib/python3.5/dist-packages/',  # sudo pip3 install installs here numpy
                '/usr/lib/python3/dist-packages/',  # sudo pip3 install installs here yaml
                '/opt/ros/kinetic/lib/python2.7/dist-packages/',  # rospy genpy
                '/usr/lib/python2.7/dist-packages/',  # rospkg is here
                os.environ['HOME'] + '/' + os.environ['ROS_WORKSPACE_NAME'] +
                '/install/lib/python2.7/dist-packages',
                '../.']  # this is the current directory

import scipy.io
import numpy as np
from refgen.robotDataFile import extractROSdata, defStruct, TextfileToStruct
from tools.mathutils import *
from tools.utils import Utils
from tools.plottingFeatures import plotOptimizationOutput
from opti.optimizer import ConstructOptimizer
from tools.getConfig import getConfig

util = Utils()

# get optimization params
opti_config = getConfig()

if opti_config.solver == 3:
    raise ValueError('Set any solver than 3 in config.yaml..!!')

# Function to extract optimization results
def unfoldResult(result, opt):
    # unfold the optimization result
    des_states = np.zeros((opt.nx, opt.N + 1))
    des_forces = np.zeros((opt.nu, opt.N))
    k = 0

    for k in range(opt.N + 1):
        des_states[:, k] = result.x[(k * opt.nx):(k * opt.nx) + opt.nx]
        if k <= opt.N - 1:
            des_forces[:, k] = result.x[(opt.nx * (opt.N + 1)) +
                                        (k * 12):(opt.nx * (opt.N + 1)) + (k * 12) + 12]
    # print (x_pred)
    # print (u_opt)
    return des_states, des_forces

# Define simulation start point from the data
simStart = 0  # Remember in python the counter starts from zero not 1!

# Prediction horizon
N = opti_config.prediction_horizon
print('Prediction horizon:', N)

# Initialize the class
optimizer = ConstructOptimizer()

# Load configuration for the class
optimizer.load_config(opti_config)

# no of states and inputs
nx = optimizer.nx
nu = optimizer.nu

load_from_text_file = opti_config.load_from_txt
txt_path = os.environ['HOME'] +"/dls_ws/src/dls-distro/opti_planning_c/test/txt"
if load_from_text_file == 1:
    ref_state_raw = np.loadtxt(txt_path+"/xreffile.txt")
    reshape_index = int(ref_state_raw.shape[0]/nx)
    ref_state_cpp = np.loadtxt(txt_path+"/xreffile.txt").reshape(reshape_index,nx).T
    ref_force_cpp = np.loadtxt(txt_path+"/ureffile.txt").reshape(reshape_index,nu).T
    param_cpp = np.loadtxt(txt_path+"/paramfile.txt").reshape(reshape_index-1,16).T
    param_cpp = np.hstack([param_cpp, param_cpp[:,-1].reshape(16,1)])

    sorted_ref = TextfileToStruct(ref_state_cpp[:, :N + 1],
                                  ref_force_cpp[:, :N + 1],
                                  param_cpp[:, :N + 1])
    sorted_ref.prediction_horizon = N
    sorted_ref.swing = np.logical_not(param_cpp[nu:,:]).astype(int)
    sorted_ref.simTime = np.arange(0, optimizer.N * optimizer.Ts, optimizer.Ts)
    time_steps = optimizer.Ts * np.ones(N)
else:
    # Load reference data file
    data = scipy.io.loadmat("../data/crawl_references_40ms.mat")

    # %% Extract linearization data
    reference_data = extractROSdata(data)
    sorted_ref = defStruct(reference_data,simStart, N +1 )
    sorted_ref.prediction_horizon=N
    sorted_ref.simTime = sorted_ref.simTime[:-1]

    # Swing matrix
    sorted_ref.swing=np.array([sorted_ref.swingLF.flatten(),
                               sorted_ref.swingRF.flatten(),
                               sorted_ref.swingLH.flatten(),
                               sorted_ref.swingRH.flatten()])


# Extract reference trajectory
optimizer.extract_ref_traj(sorted_ref)

# Extract linearization trajectory and initial condition
ref_states = optimizer.ref_states
ref_forces = optimizer.ref_forces
initial_state = ref_states[:, 0]

# Linearization trajectories
optimizer.linearize(sorted_ref, initial_state)

# get the solution
if opti_config.solver == 0:
    # CVXPY
    des_states, des_forces, des_forces_dot = optimizer.cvx_solve(initial_state[0:3],
                                                 initial_state[6:9],
                                                 initial_state[3:6],
                                                 initial_state[9:12],
                                                 0, N)
elif opti_config.solver == 1:
    # Gurobi
    optimizer.construct_standard_qp(initial_state[0:3],
                                     initial_state[6:9],
                                     initial_state[3:6],
                                     initial_state[9:12],
                                     0, N)
    result = optimizer.gurobi_solve()
    des_states, des_forces = unfoldResult(result, optimizer)
    # optimizer.compute_qp_cost(des_states, des_forces, result)

elif opti_config.solver == 2:
    #perform optimization with osqp
    optimizer.construct_standard_qp(initial_state[0:3],
                                     initial_state[6:9],
                                     initial_state[3:6],
                                     initial_state[9:12],
                                     0, N)
    result = optimizer.osqp_solve()
    des_states, des_forces = unfoldResult(result, optimizer)


XX = des_states.T.flatten()
UU = des_forces.T.flatten()

Q = opti_config.Q
QN = opti_config.QN
R = opti_config.R
Lambda= 0.0 * sparse.eye(nu)

Px = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN])

# Define the diagonal matrix for delta_u in the cost
Q_RLamda = sparse.block_diag([R + Lambda, sparse.kron(sparse.eye(N - 2)
                                                      , (R + 2*Lambda)), R + Lambda])

# Create the off diagonal matrices
OffdiagVec = np.hstack(np.tile(-Lambda.diagonal(),(N - 1, 1)))
diagonals = [OffdiagVec, OffdiagVec]
OffdiagMat = sparse.diags(diagonals,[-12, 12],)

# Add these matrices to create the Pu weight matrix
Pu = Q_RLamda + OffdiagMat

# state_linear_cost = np.hstack([ref_states.transpose().flatten(), ref_states[:, -1]])
state_linear_cost = ref_states.transpose().flatten()
forces_linear_cost = ref_forces[:, :-1].transpose().flatten()

# # State cost
# state_cost = 0.5 * XX.dot(Px.todense()).dot(XX) - state_linear_cost.dot(Px.todense()).dot(XX)
#
# # Input cost
# force_cost = 0.5 * UU.dot(Pu.todense()).dot(UU) - forces_linear_cost.dot(Pu.todense()).dot(UU)
#
# # Total calc. cost
# total_cost = state_cost + force_cost
#
# print('state cost:', state_cost[0, 0])
# print('force cost:', force_cost[0, 0])
# print('total cost:', total_cost[0, 0])

# Plot results
legend = ['desired', 'ref']
active_plots = opti_config.active_plots
plot_data = {"model_type": optimizer.model_type,
             "Ts": opti_config.Ts,
             "simTime": sorted_ref.simTime ,
             "des_controls": des_forces,
             "ref_controls": ref_forces[:, :-1],
             "des_states": des_states,
             "ref_states": ref_states,
             "des_controls_dot": des_forces_dot,
             "des_foot_pos": optimizer.feet_positionW, # place holder
             "feet_positionW": optimizer.feet_positionW, # place holder
             "des_foot_vel": np.zeros((nu, N+1)), # place holder
             "des_joint_pos": np.zeros((nu, N+1)), # place holder
             "ref_joint_pos": np.zeros((nu, N+1)), # place holder
             "des_joint_vel": np.zeros((nu, N+1)), # place holder
             "ref_joint_vel": np.zeros((nu, N+1)), # place holder
             "legend": legend,
             "active_plots": active_plots}

plotOptimizationOutput(plot_data)