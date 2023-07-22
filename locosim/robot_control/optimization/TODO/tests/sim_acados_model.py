""" Description
Get acados sensitivities and also simulate the model
Author: Niraj Rathod
Date: 06/11/2020
"""
import sys
import os
if sys.version_info[:2] == (2, 7):
    print("USING python 2.7")
else:
    sys.path = ['/usr/lib/python3.5/lib-dynload',  # package mmap
                '/usr/lib/python3.5/',  # numbers.py is here
                '/usr/local/lib/python3.5/dist-packages/',  # sudo pip3 install installs here numpy
                '/usr/lib/python3/dist-packages/',  # sudo pip3 install installs here yaml
                '/opt/ros/kinetic/lib/python2.7/dist-packages/',  # rospy genpy
                '/usr/lib/python2.7/dist-packages/',  # rospkg is here
                os.environ['HOME'] + '/' + os.environ[
                    'ROS_WORKSPACE_NAME'] + '/install/lib/python2.7/dist-packages',
                '../.']  # this is the current directory

import scipy.io
import numpy as np
from refgen.robotDataFile import extractROSdata, defStruct, TextfileToStruct
from tools.utils import Utils
from tools.plottingFeatures import plotOptimizationOutput
from opti.optimizer import ConstructOptimizer
from opti.acados_sim import AcadosSimClass
from tools.getConfig import getConfig
from casadi import *
from model.centroidal_model import centroidal_model_comFrame, centroidal_model, \
                                   centroidal_comFrameModel_DeltaU
util = Utils()

# get optimization params
opti_config = getConfig()

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

# 0: load data from .mat , 1: load data from .txt
load_from_text_file = opti_config.load_from_txt
txt_path = os.environ['HOME'] +"/dls_ws/src/dls-distro/opti_planning_c/test/txt"

if load_from_text_file == 1:
    ref_state_raw = np.loadtxt(txt_path+"/xreffile.txt")
    reshape_index = int(ref_state_raw.shape[0]/nx)
    ref_state_cpp = np.loadtxt(txt_path+"/xreffile.txt").reshape(reshape_index,nx).T
    ref_force_cpp = np.loadtxt(txt_path+"/ureffile.txt").reshape(reshape_index,nu).T
    param_cpp = np.loadtxt(txt_path+"/paramfile.txt").reshape(reshape_index - 1, 16).T
    param_cpp = np.hstack([param_cpp, param_cpp[:, -1].reshape(16, 1)])
    sorted_ref = TextfileToStruct(ref_state_cpp, ref_force_cpp, param_cpp)
    sorted_ref.prediction_horizon = N
    sorted_ref.swing = np.logical_not(param_cpp[nu:, :]).astype(int)
    sorted_ref.simTime = np.arange(0, optimizer.N * optimizer.Ts, optimizer.Ts)
else:
    # Load reference data file
    data = scipy.io.loadmat("../data/crawl_references_40ms.mat")

    # %% Extract linearization data
    reference_data = extractROSdata(data)
    sorted_ref = defStruct(reference_data, simStart, N + 1)
    sorted_ref.prediction_horizon = N
    sorted_ref.simTime = sorted_ref.simTime[:-1]

    # Swing matrix
    sorted_ref.swing = np.array([sorted_ref.swingLF.flatten(),
                                 sorted_ref.swingRF.flatten(),
                                 sorted_ref.swingLH.flatten(),
                                 sorted_ref.swingRH.flatten()])

# Extract reference trajectory
optimizer.extract_ref_traj(sorted_ref)

# Instantiate acados simulation class
sim_args = {'robotMass': optimizer.robotMass,
            'robotInertia': optimizer.robotInertia,
            'include_coneCons': optimizer.include_coneCons,
            'nx': opti_config.nx,
            'nu': opti_config.nu,
            'N': N,
            'Ts': optimizer.Ts,
            'model_type': opti_config.model_type,
            'lin_states': optimizer.ref_states,
            'lin_forces': optimizer.ref_forces ,
            'feet_positionW': optimizer.feet_positionW,
            'stance': optimizer.stance}

if optimizer.include_coneCons:
    sim_args['initial_parameters'] = np.hstack([optimizer.feet_positionW[:, 0],
                                                optimizer.stance[:, 0],
                                                optimizer.cone_normals[:, 0]])

else:
    sim_args['initial_parameters'] = np.hstack([optimizer.feet_positionW[:, 0],
                                             optimizer.stance[:, 0]])

acados_integrator = AcadosSimClass(sim_args)

# data to run simulation
des_force_py = np.loadtxt("../data/des_forces_py.txt")
reshape_index = int(des_force_py.shape[0] / nu)
des_force_py = des_force_py.reshape(reshape_index, nu).T
des_force_dot_py = np.loadtxt("../data/des_forces_dot_py.txt").reshape(reshape_index, nu).T
des_state_py = np.loadtxt("../data/des_states_py.txt").reshape(reshape_index+1, nu).T
param_py = np.loadtxt("../data/params_py.txt").reshape(reshape_index+1, 16).T

if optimizer.include_coneCons:
    param_py = np.vstack([param_py, np.hstack([optimizer.cone_normals,
                                               optimizer.cone_normals[:, -1].reshape(12,1)])])

# simulate the model using acados integrator and custom casadi integrator
if optimizer.model_type == 2:
    # Resize number of states
    nx = nx + nu

    # initial state
    initial_state = np.hstack([des_state_py[:, 0], des_force_py[:, 0]])

    # simulate using explicit Euler [Note that this is not integrated with linearized model!]
    sim_states = acados_integrator.simulate_model(initial_state, des_force_dot_py, param_py)

    # Load casADi centroidal model
    model = centroidal_comFrameModel_DeltaU(nu,
                                            optimizer.robotMass,
                                            optimizer.robotInertia,
                                            optimizer.include_coneCons)

    # define feasible control input
    feasible_forces = des_force_dot_py

    # linearization traj
    lin_state = np.vstack([optimizer.ref_states, optimizer.ref_forces])
    lin_force = np.zeros((nx, N))

else:
    # initial state
    initial_state = des_state_py[:, 0]

    # Simulate using explicit Euler [Note that this is not integrated with linearized model!]
    sim_states = acados_integrator.simulate_model(initial_state, des_force_py, param_py)

    # - load casADi centroidal model
    model = centroidal_model(optimizer.robotMass,
                             optimizer.robotInertia,
                             optimizer.include_coneCons)

    # define feasible control input
    feasible_forces = des_force_py

    # linearization traj
    lin_state = optimizer.ref_states
    lin_force = optimizer.ref_forces


# get acados sensitivities
param = np.vstack([optimizer.feet_positionW, optimizer.stance])
A_B_matrices, A_matrix, B_matrix = acados_integrator.get_acados_sensitivities(lin_state,
                                                                              lin_force,
                                                                              param)
# save simulated state from acados integrator
np.savetxt("../data/sim_states_py.txt", sim_states.T.flatten())

# Simulate with explicit Euler integrator
model_exp = model.f_expl_expr
model = Function('model', [model.x, model.u, model.p], [model_exp])

sim_custom_state = np.zeros((nx, N+1))
sim_custom_state[:,0]= initial_state

for i in range(N):
    sim_custom_state[:, i + 1] = np.array(sim_custom_state[:, i] + optimizer.Ts *
                                          model(sim_custom_state[:, i],
                                                feasible_forces[:, i],
                                                param_py[:, i])).reshape(nx)


# plot optimizer output against prediction from acados integrator
# plotOptimizationOutput(optimizer.model_type,
#                        opti_config.Ts,
#                        sorted_ref.simTime,
#                        des_force_py,
#                        des_force_py,
#                        sim_states,
#                        des_state_py,
#                        des_force_dot_py,
#                        legend=["SQP_RTI", "integrator"])

# Plot results
legend = ["SQP_RTI", "integrator"]
active_plots = opti_config.active_plots
plot_data = {"model_type": optimizer.model_type,
             "Ts": opti_config.Ts,
             "simTime": sorted_ref.simTime ,
             "des_controls": des_force_py,
             "ref_controls": des_force_py,
             "des_states": sim_states,
             "ref_states": des_state_py,
             "des_controls_dot": des_force_dot_py,
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