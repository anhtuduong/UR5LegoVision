"""
# Centroidal model consturcted using CasADi toolbox
# Author: Niraj Rathod
# Date: 05-04-19

"""
import numpy as np
import scipy.linalg
from acados_template import AcadosModel
from casadi import *
from tools.utils import Utils
from tools.mathutils import *

def centroidal_model(robotMass, robotInertia):
    # Define structs
    model = AcadosModel()
    model_name = "centroidalDyn_model"

    #-----------------------------------------------------------------------------------
    # Define model params
    Icom = robotInertia # Inertia
    m = robotMass # mass
    g = np.array([0, 0, -9.81]) # gravity

    # -----------------------------------------------------------------------------------
    # Symbolic variables in Casadi
    # - States
    comx = SX.sym("comx")
    comy = SX.sym("comy")
    comz = SX.sym("comz")
    velx = SX.sym("velx")
    vely = SX.sym("vely")
    velz = SX.sym("velz")
    roll = SX.sym("roll")
    pitch = SX.sym("pitch")
    yaw = SX.sym("yaw")
    omegax = SX.sym("omegax")
    omegay = SX.sym("omegay")
    omegaz = SX.sym("omegaz")
    x = vertcat(comx, comy, comz, velx, vely, velz, roll, pitch, yaw, omegax, omegay, omegaz)

    # - State derivatives
    xcdot=SX.sym("xcdot", 3, 1)
    vdot=SX.sym("vdot", 3, 1)
    phidot=SX.sym("phidot", 3, 1)
    omegadot = SX.sym("omegadot", 3, 1)
    xdot = vertcat(xcdot, vdot, phidot, omegadot)

    # - Inputs (ground reaction forces)
    # u = SX.sym('u', nu, 1)
    grfLFx = SX.sym("grfLFx")
    grfLFy = SX.sym("grfLFy")
    grfLFz = SX.sym("grfLFz")

    grfRFx = SX.sym("grfRFx")
    grfRFy = SX.sym("grfRFy")
    grfRFz = SX.sym("grfRFz")

    grfLHx = SX.sym("grfLHx")
    grfLHy = SX.sym("grfLHy")
    grfLHz = SX.sym("grfLHz")

    grfRHx = SX.sym("grfRHx")
    grfRHy = SX.sym("grfRHy")
    grfRHz = SX.sym("grfRHz")
    u = vertcat(grfLFx, grfLFy, grfLFz, grfRFx, grfRFy, grfRFz, grfLHx, grfLHy, grfLHz, grfRHx, grfRHy, grfRHz)

    # - Foot location
    # xf = SX.sym('xf', nu, 1)
    xfLFx = SX.sym("xfLFx")
    xfLFy = SX.sym("xfLFy")
    xfLFz = SX.sym("xfLFz")

    xfRFx = SX.sym("xfRFx")
    xfRFy = SX.sym("xfRFy")
    xfRFz = SX.sym("xfRFz")

    xfLHx = SX.sym("xfLHx")
    xfLHy = SX.sym("xfLHy")
    xfLHz = SX.sym("xfLHz")

    xfRHx = SX.sym("xfRHx")
    xfRHy = SX.sym("xfRHy")
    xfRHz = SX.sym("xfRHz")
    xf = vertcat(xfLFx, xfLFy, xfLFz, xfRFx, xfRFy, xfRFz, xfLHx, xfLHy, xfLHz, xfRHx, xfRHy, xfRHz)

    # - Stance status for each foot
    stanceLF = SX.sym("swingLF")
    stanceRF = SX.sym("swingRF")
    stanceLH = SX.sym("swingLH")
    stanceRH = SX.sym("swingRH")
    stanceVec=vertcat(stanceLF, stanceRF, stanceLH, stanceRH)

    # Model parameters that need update in runtime
    p = vertcat(xf, stanceVec)

    # -----------------------------------------------------------------------------------
    # Auxiliary equations
    # Rotation matrix (maps a vector from world to CoM frame)
    c_R_w = rpyToRot(roll, pitch, yaw)

    # Inertia matrix in the world frame
    Inertia_w = c_R_w.T @ Icom @ c_R_w

    # Inverse of a inertia matrix
    inv_Inertia_w = inv(Inertia_w)

    # Product of I_com^-1 and (omega_cross * I_com)
    omgMat = -inv_Inertia_w @ (cross_mx(np.array([omegax, omegay, omegaz])) @ Inertia_w)

    # Skew symmetric matrix for (xfi-xc)
    d1=cross_mx([xfLFx-comx, xfLFy-comy, xfLFz-comz])
    d2=cross_mx([xfRFx-comx, xfRFy-comy, xfRFz-comz])
    d3=cross_mx([xfLHx-comx, xfLHy-comy, xfLHz-comz])
    d4=cross_mx([xfRHx-comx, xfRHy-comy, xfRHz-comz])

    # I_com^-1* (xfi-xc)*fi
    tau_LF=d1 @ vertcat(grfLFx , grfLFy , grfLFz)
    tau_RF=d2 @ vertcat(grfRFx , grfRFy , grfRFz)
    tau_LH=d3 @ vertcat(grfLHx , grfLHy , grfLHz)
    tau_RH=d4 @ vertcat(grfRHx , grfRHy , grfRHz)

    # Input selection matrix
    inputSelMat=np.tile(np.array([[1, 0, 0],[0, 1, 0], [0, 0, 1]]), (1, 4))

    # -----------------------------------------------------------------------------------
    # Continuous time model xdot=f(x,u,footPos,stanceVec)
    # CoM velocity in world frame
    comX_dot = vertcat(velx,vely,velz)

    # CoM acceleration in world frame
    stanceVecFull = vertcat(stanceLF, stanceLF, stanceLF, stanceRF, stanceRF, stanceRF,
                          stanceLH, stanceLH, stanceLH, stanceRH, stanceRH, stanceRH)
    stanceMatrix = diag(stanceVecFull)
    vel_dot = g + inputSelMat @ (stanceMatrix @ (u/m))

    # Euler angle rates of a base
    phi_dot = rpyToEarInv(roll, pitch, yaw) @ vertcat(omegax, omegay, omegaz)

    # Angular acceleration of a base in world frame
    omega_dot_w = omgMat @ vertcat(omegax, omegay, omegaz) \
                  + inputSelMat @ vertcat(inv_Inertia_w@(tau_LF*stanceLF), inv_Inertia_w@(tau_RF*stanceRF),
                                          inv_Inertia_w@(tau_LH*stanceLH), inv_Inertia_w@(tau_RH*stanceRH))
    # -----------------------------------------------------------------------------------
    # Explicit model
    f_expl = vertcat(comX_dot, vel_dot, phi_dot, omega_dot_w)

    # Define model struct
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p # These parameters can be updated in runtime
    model.name = model_name
    return  model

# Cone constraints (will be formulated as polytopic in acados lg<= D*u+C*x<=ug)
def coneConstraints(nx, ng, mu, friction_upperBound):
    constraint = types.SimpleNamespace()
    util = Utils()

    # cones
    mu_LF=mu[util.leg_map("LF")]
    mu_RF=mu[util.leg_map("RF")]
    mu_LH=mu[util.leg_map("LH")]
    mu_RH=mu[util.leg_map("RH")]

    # - LF
    mu_max_LF = mu_LF
    mu_min_LF = -mu_max_LF
    coneLF = np.array([[1, 0, - mu_min_LF],
                     [0, 1, - mu_min_LF],
                     [-1, 0, mu_max_LF],
                     [0, - 1, mu_max_LF]])

    # - RF
    mu_max_RF = mu_RF
    mu_min_RF = -mu_max_RF
    coneRF = np.array([[1, 0, - mu_min_RF],
                       [0, 1, - mu_min_RF],
                       [-1, 0, mu_max_RF],
                       [0, - 1, mu_max_RF]])

    # - LH
    mu_max_LH = mu_LH
    mu_min_LH = -mu_max_LH
    coneLH = np.array([[1, 0, - mu_min_LH],
                       [0, 1, - mu_min_LH],
                       [-1, 0, mu_max_LH],
                       [0, - 1, mu_max_LH]])

    # - RH
    mu_max_RH = mu_RH
    mu_min_RH = -mu_max_RH
    coneRH = np.array([[1, 0, - mu_min_RH],
                       [0, 1, - mu_min_RH],
                       [-1, 0, mu_max_RH],
                       [0, - 1, mu_max_RH]])

    constraint.C = np.zeros((ng, nx))
    constraint.D = scipy.linalg.block_diag(coneLF,coneRF,coneLH,coneRH)
    constraint.lg = np.zeros(ng)
    constraint.ug = np.hstack([friction_upperBound[util.leg_map("LF")] * np.ones(4),
                               friction_upperBound[util.leg_map("RF")] * np.ones(4),
                               friction_upperBound[util.leg_map("LH")] * np.ones(4),
                               friction_upperBound[util.leg_map("RH")] * np.ones(4)])
    constraint.ng = ng
    return constraint


def centroidal_model_comFrame(robotMass, robotInertia):
    # Define structs
    model = AcadosModel()
    model_name = "centroidalDyn_model"

    #-----------------------------------------------------------------------------------
    # Define model params
    Icom = robotInertia # Inertia
    m = robotMass # mass
    g = np.array([0, 0, -9.81]) # gravity

    # -----------------------------------------------------------------------------------
    # Symbolic variables in Casadi
    # - States
    comx = SX.sym("comx")
    comy = SX.sym("comy")
    comz = SX.sym("comz")
    velx = SX.sym("velx")
    vely = SX.sym("vely")
    velz = SX.sym("velz")
    roll = SX.sym("roll")
    pitch = SX.sym("pitch")
    yaw = SX.sym("yaw")
    omegax = SX.sym("omegax")
    omegay = SX.sym("omegay")
    omegaz = SX.sym("omegaz")
    x = vertcat(comx, comy, comz, velx, vely, velz, roll, pitch, yaw, omegax, omegay, omegaz)

    # - State derivatives
    xcdot=SX.sym("xcdot", 3, 1)
    vdot=SX.sym("vdot", 3, 1)
    phidot=SX.sym("phidot", 3, 1)
    omegadot = SX.sym("omegadot", 3, 1)
    xdot = vertcat(xcdot, vdot, phidot, omegadot)

    # - Inputs (ground reaction forces)
    # u = SX.sym('u', nu, 1)
    grfLFx = SX.sym("grfLFx")
    grfLFy = SX.sym("grfLFy")
    grfLFz = SX.sym("grfLFz")

    grfRFx = SX.sym("grfRFx")
    grfRFy = SX.sym("grfRFy")
    grfRFz = SX.sym("grfRFz")

    grfLHx = SX.sym("grfLHx")
    grfLHy = SX.sym("grfLHy")
    grfLHz = SX.sym("grfLHz")

    grfRHx = SX.sym("grfRHx")
    grfRHy = SX.sym("grfRHy")
    grfRHz = SX.sym("grfRHz")
    u = vertcat(grfLFx, grfLFy, grfLFz, grfRFx, grfRFy, grfRFz, grfLHx, grfLHy, grfLHz, grfRHx, grfRHy, grfRHz)

    # - Foot location
    # xf = SX.sym('xf', nu, 1)
    xfLFx = SX.sym("xfLFx")
    xfLFy = SX.sym("xfLFy")
    xfLFz = SX.sym("xfLFz")

    xfRFx = SX.sym("xfRFx")
    xfRFy = SX.sym("xfRFy")
    xfRFz = SX.sym("xfRFz")

    xfLHx = SX.sym("xfLHx")
    xfLHy = SX.sym("xfLHy")
    xfLHz = SX.sym("xfLHz")

    xfRHx = SX.sym("xfRHx")
    xfRHy = SX.sym("xfRHy")
    xfRHz = SX.sym("xfRHz")
    xf = vertcat(xfLFx, xfLFy, xfLFz, xfRFx, xfRFy, xfRFz, xfLHx, xfLHy, xfLHz, xfRHx, xfRHy, xfRHz)

    # - Stance status for each foot
    stanceLF = SX.sym("swingLF")
    stanceRF = SX.sym("swingRF")
    stanceLH = SX.sym("swingLH")
    stanceRH = SX.sym("swingRH")
    stanceVec=vertcat(stanceLF, stanceRF, stanceLH, stanceRH)

    # Model parameters that need update in runtime
    p = vertcat(xf, stanceVec)

    # -----------------------------------------------------------------------------------
    # Auxiliary equations
    # Rotation matrix (maps a vector from world to CoM frame)
    c_R_w = rpyToRot(roll, pitch, yaw)

    # Inertia matrix in the CoM frame
    invInertia_c = np.linalg.inv(Icom)

    # Omega in CoM frame
    # omega_c = c_R_w @ vertcat(omegax,omegay,omegaz)
    omega_c = vertcat(omegax,omegay,omegaz)

    # Product of I_com^-1 and (omega_cross * I_com)
    # omgMat = -invInertia_c @ (cross_mx([omegax,omegay,omegaz]) @ Icom)
    omgMat = -invInertia_c @ (cross_mx(omega_c) @ Icom)

    # Skew symmetric matrix for (xfi-xc)
    d1=cross_mx([xfLFx-comx, xfLFy-comy, xfLFz-comz])
    d2=cross_mx([xfRFx-comx, xfRFy-comy, xfRFz-comz])
    d3=cross_mx([xfLHx-comx, xfLHy-comy, xfLHz-comz])
    d4=cross_mx([xfRHx-comx, xfRHy-comy, xfRHz-comz])

    # Compute torques (xfi-xc)*fi in CoM frame
    tau_LF= c_R_w @ (d1 @ vertcat(grfLFx , grfLFy , grfLFz))
    tau_RF= c_R_w @ (d2 @ vertcat(grfRFx , grfRFy , grfRFz))
    tau_LH= c_R_w @ (d3 @ vertcat(grfLHx , grfLHy , grfLHz))
    tau_RH= c_R_w @ (d4 @ vertcat(grfRHx , grfRHy , grfRHz))

    # Input selection matrix
    inputSelMat = np.tile(np.array([[1, 0, 0],[0, 1, 0], [0, 0, 1]]), (1, 4))

    # -----------------------------------------------------------------------------------
    # Continuous time model xdot=f(x,u,footPos,stanceVec)
    # CoM velocity in world frame
    comX_dot = vertcat(velx,vely,velz)

    # CoM acceleration in world frame
    stanceVecFull = vertcat(stanceLF, stanceLF, stanceLF, stanceRF, stanceRF, stanceRF,
                          stanceLH, stanceLH, stanceLH, stanceRH, stanceRH, stanceRH)
    stanceMatrix = diag(stanceVecFull)
    vel_dot = g + inputSelMat @ (stanceMatrix @ (u/m))

    # Euler angle rates of a base
    phi_dot = rpyToConjEarInv(roll, pitch, yaw) @ omega_c

    # Angular acceleration of a base in CoM frame
    omega_dot_c = omgMat @ omega_c \
                  + inputSelMat @ vertcat(invInertia_c@(tau_LF*stanceLF), invInertia_c@(tau_RF*stanceRF),
                                          invInertia_c@(tau_LH*stanceLH), invInertia_c@(tau_RH*stanceRH))
    # omega_dot_w =  c_R_w.T @ omega_dot_c #
    omega_dot_w =   omega_dot_c #
    # -----------------------------------------------------------------------------------
    # Explicit model
    f_expl = vertcat(comX_dot, vel_dot, phi_dot, omega_dot_w)

    # Define model struct
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p # These parameters can be updated in runtime
    model.name = model_name
    return  model


def centroidal_comFrameModel_DeltaU(nu, robotMass, robotInertia):
    # Define structs
    model = AcadosModel()
    model_name = "centroidalDyn_model"

    #-----------------------------------------------------------------------------------
    # Define model params
    Icom = robotInertia # Inertia
    m = robotMass # mass
    g = np.array([0, 0, -9.81]) # gravity
    # -----------------------------------------------------------------------------------
    # Symbolic variables in Casadi
    # - States
    comx = SX.sym("comx")
    comy = SX.sym("comy")
    comz = SX.sym("comz")
    velx = SX.sym("velx")
    vely = SX.sym("vely")
    velz = SX.sym("velz")
    roll = SX.sym("roll")
    pitch = SX.sym("pitch")
    yaw = SX.sym("yaw")
    omegax = SX.sym("omegax")
    omegay = SX.sym("omegay")
    omegaz = SX.sym("omegaz")

    # - Stance status for each foot
    stanceLF = SX.sym("swingLF")
    stanceRF = SX.sym("swingRF")
    stanceLH = SX.sym("swingLH")
    stanceRH = SX.sym("swingRH")
    stanceVec = vertcat(stanceLF, stanceRF, stanceLH, stanceRH)

    grfLFx = SX.sym("grfLFx")
    grfLFy = SX.sym("grfLFy")
    grfLFz = SX.sym("grfLFz")

    grfRFx = SX.sym("grfRFx")
    grfRFy = SX.sym("grfRFy")
    grfRFz = SX.sym("grfRFz")

    grfLHx = SX.sym("grfLHx")
    grfLHy = SX.sym("grfLHy")
    grfLHz = SX.sym("grfLHz")

    grfRHx = SX.sym("grfRHx")
    grfRHy = SX.sym("grfRHy")
    grfRHz = SX.sym("grfRHz")
    uold = vertcat(grfLFx, grfLFy, grfLFz,
                   grfRFx, grfRFy, grfRFz,
                   grfLHx, grfLHy, grfLHz,
                   grfRHx, grfRHy, grfRHz)

    x = vertcat(comx, comy, comz, velx, vely, velz, roll, pitch, yaw, omegax, omegay, omegaz, uold)

    # - State derivatives
    xcdot = SX.sym("xcdot", 3, 1)
    vdot = SX.sym("vdot", 3, 1)
    phidot = SX.sym("phidot", 3, 1)
    omegadot = SX.sym("omegadot", 3, 1)
    uolddot = SX.sym("uolddot", nu, 1)

    xdot = vertcat(xcdot, vdot, phidot, omegadot, uolddot)

    # - Inputs (ground reaction forces)

    grfLFx_dot = SX.sym("grfLFx_dot")
    grfLFy_dot = SX.sym("grfLFy_dot")
    grfLFz_dot = SX.sym("grfLFz_dot")

    grfRFx_dot = SX.sym("grfRFx_dot")
    grfRFy_dot = SX.sym("grfRFy_dot")
    grfRFz_dot = SX.sym("grfRFz_dot")

    grfLHx_dot = SX.sym("grfLHx_dot")
    grfLHy_dot = SX.sym("grfLHy_dot")
    grfLHz_dot = SX.sym("grfLHz_dot")

    grfRHx_dot = SX.sym("grfRHx_dot")
    grfRHy_dot = SX.sym("grfRHy_dot")
    grfRHz_dot = SX.sym("grfRHz_dot")

    u = vertcat(grfLFx_dot, grfLFy_dot, grfLFz_dot,
                grfRFx_dot, grfRFy_dot, grfRFz_dot,
                grfLHx_dot, grfLHy_dot, grfLHz_dot,
                grfRHx_dot, grfRHy_dot, grfRHz_dot)

    # - Foot location
    # xf = SX.sym('xf', nu, 1)
    xfLFx = SX.sym("xfLFx")
    xfLFy = SX.sym("xfLFy")
    xfLFz = SX.sym("xfLFz")

    xfRFx = SX.sym("xfRFx")
    xfRFy = SX.sym("xfRFy")
    xfRFz = SX.sym("xfRFz")

    xfLHx = SX.sym("xfLHx")
    xfLHy = SX.sym("xfLHy")
    xfLHz = SX.sym("xfLHz")

    xfRHx = SX.sym("xfRHx")
    xfRHy = SX.sym("xfRHy")
    xfRHz = SX.sym("xfRHz")
    xf = vertcat(xfLFx, xfLFy, xfLFz, xfRFx, xfRFy, xfRFz, xfLHx, xfLHy, xfLHz, xfRHx, xfRHy, xfRHz)

    # Model parameters that need update in runtime
    p = vertcat(xf, stanceVec)

    # -----------------------------------------------------------------------------------
    # Auxiliary equations
    # Rotation matrix (maps a vector from world to CoM frame)
    c_R_w = rpyToRot(roll, pitch, yaw)

    # Inertia matrix in the CoM frame
    invInertia_c = np.linalg.inv(Icom)

    # Omega in CoM frame
    # omega_c = c_R_w @ vertcat(omegax,omegay,omegaz)
    omega_c = vertcat(omegax, omegay, omegaz)

    # Product of I_com^-1 and (omega_cross * I_com)
    # omgMat = -invInertia_c @ (cross_mx([omegax,omegay,omegaz]) @ Icom)
    omgMat = -invInertia_c @ (cross_mx(omega_c) @ Icom)

    # Skew symmetric matrix for (xfi-xc)
    d1 = cross_mx([xfLFx - comx, xfLFy - comy, xfLFz - comz])
    d2 = cross_mx([xfRFx - comx, xfRFy - comy, xfRFz - comz])
    d3 = cross_mx([xfLHx - comx, xfLHy - comy, xfLHz - comz])
    d4 = cross_mx([xfRHx - comx, xfRHy - comy, xfRHz - comz])

    # Compute torques (xfi-xc)*fi in CoM frame
    tau_LF = c_R_w @ (d1 @ vertcat(grfLFx, grfLFy, grfLFz))
    tau_RF = c_R_w @ (d2 @ vertcat(grfRFx, grfRFy, grfRFz))
    tau_LH = c_R_w @ (d3 @ vertcat(grfLHx, grfLHy, grfLHz))
    tau_RH = c_R_w @ (d4 @ vertcat(grfRHx, grfRHy, grfRHz))

    # Input selection matrix
    inputSelMat = np.tile(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]), (1, 4))

    # -----------------------------------------------------------------------------------
    # Continuous time model xdot=f(x,u,footPos,stanceVec)
    # CoM velocity in world frame
    comX_dot = vertcat(velx, vely, velz)

    # CoM acceleration in world frame
    stanceVecFull = vertcat(stanceLF, stanceLF, stanceLF, stanceRF, stanceRF, stanceRF,
                            stanceLH, stanceLH, stanceLH, stanceRH, stanceRH, stanceRH)
    stanceMatrix = diag(stanceVecFull)
    vel_dot = g + inputSelMat @ (stanceMatrix @ (uold / m))

    # Euler angle rates of a base
    phi_dot = rpyToConjEarInv(roll, pitch, yaw) @ omega_c

    # Angular acceleration of a base in CoM frame
    omega_dot_c = omgMat @ omega_c \
                  + inputSelMat @ vertcat(invInertia_c @ (tau_LF * stanceLF), invInertia_c @ (tau_RF * stanceRF),
                                          invInertia_c @ (tau_LH * stanceLH), invInertia_c @ (tau_RH * stanceRH))
    # omega_dot_w =  c_R_w.T @ omega_dot_c #
    omega_dot_w = omega_dot_c  #

    # du/dt
    u_dot = u

    # -----------------------------------------------------------------------------------
    # Explicit model
    f_expl = vertcat(comX_dot, vel_dot, phi_dot, omega_dot_w, u_dot)

    # Structure model param
    params = types.SimpleNamespace()
    params.Icom = Icom
    params.m = m
    params.g = g

    # Define model struct
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p  # These parameters can be updated in runtime
    model.name = model_name
    return model

# Cone constraints for deltaU model (will be formulated as polytopic in acados lg<= D*u+C*x<=ug)
def coneConstraints_DeltaU(nx, nu, ng, mu, friction_upperBound):
    constraint = types.SimpleNamespace()
    util = Utils()

    # cones
    mu_LF=mu[util.leg_map("LF")]
    mu_RF=mu[util.leg_map("RF")]
    mu_LH=mu[util.leg_map("LH")]
    mu_RH=mu[util.leg_map("RH")]

    # - LF
    mu_max_LF = mu_LF
    mu_min_LF = -mu_max_LF
    coneLF = np.array([[1, 0, - mu_min_LF],
                     [0, 1, - mu_min_LF],
                     [-1, 0, mu_max_LF],
                     [0, - 1, mu_max_LF]])

    # - RF
    mu_max_RF = mu_RF
    mu_min_RF = -mu_max_RF
    coneRF = np.array([[1, 0, - mu_min_RF],
                       [0, 1, - mu_min_RF],
                       [-1, 0, mu_max_RF],
                       [0, - 1, mu_max_RF]])

    # - LH
    mu_max_LH = mu_LH
    mu_min_LH = -mu_max_LH
    coneLH = np.array([[1, 0, - mu_min_LH],
                       [0, 1, - mu_min_LH],
                       [-1, 0, mu_max_LH],
                       [0, - 1, mu_max_LH]])

    # - RH
    mu_max_RH = mu_RH
    mu_min_RH = -mu_max_RH
    coneRH = np.array([[1, 0, - mu_min_RH],
                       [0, 1, - mu_min_RH],
                       [-1, 0, mu_max_RH],
                       [0, - 1, mu_max_RH]])

    constraint.C = np.hstack([np.zeros((ng,nx-nu)),
                              scipy.linalg.block_diag(coneLF,coneRF,coneLH,coneRH)])
    constraint.D = np.zeros((ng, nu))
    constraint.lg = np.zeros(ng)
    constraint.ug = np.hstack([friction_upperBound[util.leg_map("LF")] * np.ones(4),
                               friction_upperBound[util.leg_map("RF")] * np.ones(4),
                               friction_upperBound[util.leg_map("LH")] * np.ones(4),
                               friction_upperBound[util.leg_map("RH")] * np.ones(4)])
    constraint.ng = ng
    return constraint