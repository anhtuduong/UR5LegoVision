#to be compatible with python3
from __future__ import print_function
import numpy as np
import scipy as sp
import math as math
import matplotlib.pyplot as plt
from casadi import *
from tools.utils import *


def cross_mx(v):
    mx =np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    # Check if it is float or casadi symbolics
    if type(v[0]) is type(SX(0)):
        mx = SX(mx)
    return mx

# def checkCasadiObjtype(obj):
#     if type(obj[0]) is type(SX(0)):
#         objToReturn = SX(obj)
#     elif type(obj[0]) is type(MX(0)):
#         objToReturn = MX(obj)
#     elif type(obj[0]) is type(DM(0)):
#         objToReturn = MX(obj)
#     else:
#         objToReturn = obj
#     return objToReturn

def cross_mx_casadi(v):
    # mx =[[MX(0.0), -v[2], v[1]], [v[2], MX(0.0), -v[0]], [-v[1], v[0], MX(0.0)]]
    mx =MX.zeros(3,3)
    mx[0, 1] =  -v[2]
    mx[0, 2] =   v[1]
    mx[1, 0] =   v[2]
    mx[1, 2] =  -v[0]
    mx[2, 0] =  -v[1]
    mx[2, 1] =   v[0]
    return mx
def skew_simToVec(Ra):
    # This is similar to one implemented in the framework
    v = np.zeros(3)
    v[0] = 0.5*(Ra[2,1] - Ra[1,2])
    v[1] = 0.5*(Ra[0,2] - Ra[2,0])
    v[2] = 0.5*(Ra[1,0] - Ra[0,1])

    return v

def rpyToRot(roll, pitch, yaw):
    Rx = np.array([[1.0,   0.0 ,	  0.0 ],
          [0.0,  cos(roll),  sin(roll)],
          [0.0, -sin(roll),  cos(roll)]])

    Ry =np.array([[cos(pitch),	 0.0,   -sin(pitch)],
         [0.0   ,    1.0,   0.0],
         [sin(pitch),	 0.0,  cos(pitch)]])


    Rz = np.array([[cos( yaw) ,  sin(yaw),	0.0],
          [-sin( yaw),  cos(yaw),   0.0],
          [0.0   ,  0.0 ,   1.0]])

    # Check if it is float or casadi symbolics
    if type(roll) is type(SX(0)):
        Rx = SX(Rx)
        Ry = SX(Ry)
        Rz = SX(Rz)

    R =  Rx @ Ry @ Rz
    return R

def rotMatToRotVec(Ra):
    c = 0.5 * (Ra[0, 0] + Ra[1, 1] + Ra[2, 2] - 1)
    w = -skew_simToVec(Ra)
    s = np.linalg.norm(w) # w = sin(theta) * axis

    if abs(s) <= 1e-10:
        err = np.zeros(3)
    else:
        angle = math.atan2(s, c)
        axis = w / s
        err = angle * axis
    return err

def rpyToEarInv(r,p,y):
    #convention yaw pitch roll
    phi = r
    theta = p
    psi = y

    #Inverse of Euler angle rates matrix whose multiplication with the vector of
    #angular velocity in world frame yields the Euler angle rates
    EarInv = np.array([[cos(psi)/cos(theta),        sin(psi)/cos(theta),         0.0],
                       [-sin(psi),                  cos(psi),                  0.0],
                       [cos(psi)*tan(theta),        sin(psi)*tan(theta) ,        1.0]])
    # Check if it is float or casadi symbolics
    if type(p) is type(SX(0)):
        EarInv = SX(EarInv)
    return EarInv

def rpyToEar(r,p,y):
    phi = r
    theta = p
    psi = y

    Ear = np.array([[cos(theta) * cos(psi), -sin(psi), 0],
                    [cos(theta) * sin(psi), cos(psi), 0],
                    [-sin(theta), 0, 1]])
    return Ear

def rpyToConjEarInv(r,p,y):
    #convention yaw pitch roll
    phi = r
    theta = p
    psi = y

    # Inverse of Conjugate Euler angle rates matrix whose multiplication with the vector of
    # body-fixed angular velocity yields the Euler angle rates
    EarConjInv = np.array([[1.0,          sin(phi)*tan(theta),         cos(phi)*tan(theta)],
                           [0.0,               cos(phi),                   -sin(phi)],
                           [0.0,          sin(phi)/cos(theta),         cos(phi)/cos(theta)]])
    # Check if it is float or casadi symbolics
    if type(p) is type(SX(0)):
        EarConjInv = SX(EarConjInv)
    return EarConjInv

# epsilon for testing whether a number is close to zero
_EPS = numpy.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print axes, "failed"

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence.

    >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(angles, [0.123, 0, 0])
    True

    """
    return euler_from_matrix(quaternion_matrix(quaternion), axes)

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    q = numpy.array(quaternion[:4], dtype=numpy.float64, copy=True)
    nq = numpy.dot(q, q)
    if nq < _EPS:
        return numpy.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = numpy.outer(q, q)
    return numpy.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=numpy.float64)

def invincasadi(A):
    # Determinant of matrix A
    sb1=A[0,0]*((A[1,1]*A[2,2])-(A[1,2]*A[2,1]))
    sb2=A[0,1]*((A[1,0]*A[2,2])-(A[1,2]*A[2,0]))
    sb3=A[0,2]*((A[1,0]*A[2,1])-(A[1,1]*A[2,0]))

    Adetr=sb1-sb2+sb3
#    print(Adetr)
    # Transpose matrix A
    TransA=A.T

    # Find determinant of the minors
    a01=(TransA[1,1]*TransA[2,2])-(TransA[2,1]*TransA[1,2])
    a02=(TransA[1,0]*TransA[2,2])-(TransA[1,2]*TransA[2,0])
    a03=(TransA[1,0]*TransA[2,1])-(TransA[2,0]*TransA[1,1])
    
    a11=(TransA[0,1]*TransA[2,2])-(TransA[0,2]*TransA[2,1])
    a12=(TransA[0,0]*TransA[2,2])-(TransA[0,2]*TransA[2,0])
    a13=(TransA[0,0]*TransA[2,1])-(TransA[0,1]*TransA[2,0])

    a21=(TransA[0,1]*TransA[1,2])-(TransA[1,1]*TransA[0,2])
    a22=(TransA[0,0]*TransA[1,2])-(TransA[0,2]*TransA[1,0])
    a23=(TransA[0,0]*TransA[1,1])-(TransA[0,1]*TransA[1,0])

    # Inverse of determinant
    invAdetr=(float(1)/Adetr)
#    print(invAdetr)
    # Inverse of the matrix A
    invA=np.array([[invAdetr*a01, -invAdetr*a02, invAdetr*a03], [-invAdetr*a11, invAdetr*a12, -invAdetr*a13], [invAdetr*a21, -invAdetr*a22, invAdetr*a23]])

    # Return the matrix
    return invA


def MxInv(A):
    # Determinant of matrix A
    sb1 = A[0, 0] * ((A[1, 1] * A[2, 2]) - (A[1, 2] * A[2, 1]))
    sb2 = A[0, 1] * ((A[1, 0] * A[2, 2]) - (A[1, 2] * A[2, 0]))
    sb3 = A[0, 2] * ((A[1, 0] * A[2, 1]) - (A[1, 1] * A[2, 0]))

    Adetr = sb1 - sb2 + sb3
    #    print(Adetr)
    # Transpose matrix A
    TransA = A.T

    # Find determinant of the minors
    a01 = (TransA[1, 1] * TransA[2, 2]) - (TransA[2, 1] * TransA[1, 2])
    a02 = (TransA[1, 0] * TransA[2, 2]) - (TransA[1, 2] * TransA[2, 0])
    a03 = (TransA[1, 0] * TransA[2, 1]) - (TransA[2, 0] * TransA[1, 1])

    a11 = (TransA[0, 1] * TransA[2, 2]) - (TransA[0, 2] * TransA[2, 1])
    a12 = (TransA[0, 0] * TransA[2, 2]) - (TransA[0, 2] * TransA[2, 0])
    a13 = (TransA[0, 0] * TransA[2, 1]) - (TransA[0, 1] * TransA[2, 0])

    a21 = (TransA[0, 1] * TransA[1, 2]) - (TransA[1, 1] * TransA[0, 2])
    a22 = (TransA[0, 0] * TransA[1, 2]) - (TransA[0, 2] * TransA[1, 0])
    a23 = (TransA[0, 0] * TransA[1, 1]) - (TransA[0, 1] * TransA[1, 0])

    # Inverse of determinant
    invAdetr = (float(1) / Adetr)
    #    print(invAdetr)
    # Inverse of the matrix A
    invA = MX.zeros(3,3)
    invA[0, 0] = invAdetr * a01
    invA[0, 1] = -invAdetr * a02
    invA[0, 2] = invAdetr * a03

    invA[0, 0] = -invAdetr * a11
    invA[0, 1] = invAdetr * a12
    invA[0, 2] = -invAdetr * a13

    invA[0, 0] = invAdetr * a21
    invA[0, 1] = -invAdetr * a22
    invA[0, 2] = invAdetr * a23

    # Return the matrix
    return invA


#/**
#brief motionVectorTransform Tranforms twists from A to B (b_X_a)   \in R^6 \times 6
#here A is the origin frame and B the destination frame.
#param position coordinate vector expressing OaOb in A coordinates
#param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
#return
#
def motionVectorTransform(position, rotationMx):
    utils = Utils()
    b_X_a = np.zeros((6,6))
    b_X_a[utils.sp_crd("AX"):utils.sp_crd("AX") + 3 ,   utils.sp_crd("AX"): utils.sp_crd("AX") + 3] = rotationMx
    b_X_a[utils.sp_crd("LX"):utils.sp_crd("LX") + 3 ,   utils.sp_crd("AX"): utils.sp_crd("AX") + 3] = -rotationMx*cross_mx(position)
    b_X_a[utils.sp_crd("LX"):utils.sp_crd("LX") + 3 ,   utils.sp_crd("LX"): utils.sp_crd("LX") + 3] = rotationMx
    return b_X_a


#
#brief forceVectorTransform Tranforms wrenches from A to B (b_X_a)   \in R^6 \times 6
#here A is the origin frame and B the destination frame.
#param position coordinate vector expressing OaOb in A coordinates
#param rotationMx rotation matrix that transforms 3D vectors from A to B coordinates
#return
#


def forceVectorTransform(position, rotationMx):
    utils = Utils()
    b_X_a = np.zeros((6,6))
    b_X_a[utils.sp_crd("AX"):utils.sp_crd("AX") + 3 ,   utils.sp_crd("AX"): utils.sp_crd("AX") + 3] = rotationMx
    b_X_a[utils.sp_crd("AX"):utils.sp_crd("AX") + 3 ,   utils.sp_crd("LX"): utils.sp_crd("LX") + 3] = -rotationMx*cross_mx(position)
    b_X_a[utils.sp_crd("LX"):utils.sp_crd("LX") + 3 ,   utils.sp_crd("LX"): utils.sp_crd("LX") + 3] = rotationMx
    return b_X_a

def tic():
    #Homemade version of matlab tic and toc functions
    import time
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()

def toc():
    import time
    if 'startTime_for_tictoc' in globals():
        print("Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds.")
    else:
        print("Toc: start time not set")
def close_all():
    plt.close('all')

def hipPositionW(comPosW, r, p, y):
    # Parameters
    comXposOffsetB = 0.009 # CoM x position offset in base frame
    hipPosXB = 0.37            # x position of hip from base origin
    hipPosYB = 0.207           # y position of hip from base origin

    # Rotation matrix
    c_R_w = rpyToRot(r, p, y)

    # Hip position in the world frame

    # - all hip positions from CoM in world frame
    x_h_LFB = c_R_w.T @ [hipPosXB-comXposOffsetB, hipPosYB, 0]
    x_h_RFB = c_R_w.T @ [hipPosXB-comXposOffsetB, -hipPosYB, 0]
    x_h_LHB = c_R_w.T @ [-(hipPosXB+comXposOffsetB), hipPosYB, 0]
    x_h_RHB = c_R_w.T @ [-(hipPosXB+comXposOffsetB), -hipPosYB, 0]

    # - all hip positions in world frame
    hipPosLFW = comPosW + x_h_LFB
    hipPosRFW = comPosW + x_h_RFB
    hipPosLHW = comPosW + x_h_LHB
    hipPosRHW = comPosW + x_h_RHB
    # print(hipPosLFW)

    return hipPosLFW, hipPosRFW, hipPosLHW, hipPosRHW

def hipZPositionW(comPosW, r, p, y):
    # Parameters
    comXposOffsetB = 0.009 # CoM x position offset in base frame
    hipPosXB = 0.37            # x position of hip from base origin
    hipPosYB = 0.207           # y position of hip from base origin

    # Rotation matrix
    c_R_w = rpyToRot(r, p, y)

    # Hip position in the world frame

    # - all hip positions from CoM in world frame
    x_h_LFB = c_R_w.T @ [hipPosXB-comXposOffsetB, hipPosYB, 0]
    x_h_RFB = c_R_w.T @ [hipPosXB-comXposOffsetB, -hipPosYB, 0]
    x_h_LHB = c_R_w.T @ [-(hipPosXB+comXposOffsetB), hipPosYB, 0]
    x_h_RHB = c_R_w.T @ [-(hipPosXB+comXposOffsetB), -hipPosYB, 0]

    # - Z hip positions of all the legs in world frame
    hipPosLFW_Z = comPosW[2] + x_h_LFB[2]
    hipPosRFW_Z = comPosW[2] + x_h_RFB[2]
    hipPosLHW_Z = comPosW[2] + x_h_LHB[2]
    hipPosRHW_Z = comPosW[2] + x_h_RHB[2]
    # print(hipPosLFW_Z)

    return hipPosLFW_Z, hipPosRFW_Z, hipPosLHW_Z, hipPosRHW_Z

def get_dict_keys(dict):
    names=list(dict.keys())
    names.sort()
    return  names