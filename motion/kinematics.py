import numpy as np
from numpy import linalg
from scipy.spatial.transform import Rotation

from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from cmath import acos as acos
from cmath import asin as asin
from math import sqrt as sqrt
from cmath import pi as pi


A = np.array([0, -0.425, -0.39225, 0, 0, 0])
D = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.145])
ALPHA = np.array([pi / 2, 0, 0, pi / 2, -pi / 2, 0])

def dh_matrix(theta, d, a, alpha):
    return np.array([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1],]
    )

def forward_kinematics(theta):
    T = np.identity(4)
    for i in range(6):
        T = T @ dh_matrix(theta[i], D[i], A[i], ALPHA[i])
    return T

def inverse_kinematics(T):
    '''
    [
    ROT, pos
    0, 0, 0, 1
    ]
    '''
    T60 = T
    p60 = T60[:3, 3]
    p50 = T60 @ np.array([0, 0, -D[5], 1])

    th1_1 = np.real((atan2(p50[1], p50[0]) + acos(D[3] / np.hypot(p50[1], p50[0])))) + pi / 2
    th1_2 = np.real((atan2(p50[1], p50[0]) - acos(D[3] / np.hypot(p50[1], p50[0])))) + pi / 2
    th5_1 = + np.real(acos((p60[0]*sin(th1_1) - p60[1]*cos(th1_1) - D[3]) / D[5]))
    th5_2 = - np.real(acos((p60[0]*sin(th1_1) - p60[1]*cos(th1_1) - D[3]) / D[5])) 
    th5_3 = + np.real(acos((p60[0]*sin(th1_2) - p60[1]*cos(th1_2) - D[3]) / D[5]))
    th5_4 = - np.real(acos((p60[0]*sin(th1_2) - p60[1]*cos(th1_2) - D[3]) / D[5]))

    T06 = linalg.inv(T60)
    Xhat = T06[:3,0]
    Yhat = T06[:3,1]

    th6_1 = np.real(atan2(((-Xhat[1] * sin(th1_1) + Yhat[1] * cos(th1_1))) / sin(th5_1), ((Xhat[0] * sin(th1_1) - Yhat[0] * cos(th1_1))) / sin(th5_1)))
    th6_2 = np.real(atan2(((-Xhat[1] * sin(th1_1) + Yhat[1] * cos(th1_1))) / sin(th5_2), ((Xhat[0] * sin(th1_1) - Yhat[0] * cos(th1_1))) / sin(th5_2)))
    th6_3 = np.real(atan2(((-Xhat[1] * sin(th1_2) + Yhat[1] * cos(th1_2))) / sin(th5_3), ((Xhat[0] * sin(th1_2) - Yhat[0] * cos(th1_2))) / sin(th5_3)))
    th6_4 = np.real(atan2(((-Xhat[1] * sin(th1_2) + Yhat[1] * cos(th1_2))) / sin(th5_4), ((Xhat[0] * sin(th1_2) - Yhat[0] * cos(th1_2))) / sin(th5_4)))

    T41m = linalg.inv(dh_matrix(th1_1, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_1, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_1, D[4], A[4], ALPHA[4]))
    p41_1 = T41m[0:3, 3]
    p41xz_1 = np.hypot(p41_1[0], p41_1[2])

    T41m = linalg.inv(dh_matrix(th1_1, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_2, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_2, D[4], A[4], ALPHA[4]))
    p41_2 = T41m[0:3, 3]
    p41xz_2 = np.hypot(p41_2[0], p41_2[2])

    T41m = linalg.inv(dh_matrix(th1_2, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_3, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_3, D[4], A[4], ALPHA[4]))
    p41_3 = T41m[0:3, 3]
    p41xz_3 = np.hypot(p41_3[0], p41_3[2])

    T41m = linalg.inv(dh_matrix(th1_2, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_4, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_4, D[4], A[4], ALPHA[4]))
    p41_4 = T41m[0:3, 3]
    p41xz_4 = np.hypot(p41_4[0], p41_4[2])

    th3_1 = np.real(acos((p41xz_1**2 - A[1]**2 - A[2]**2) / (2 * A[1] * A[2])))
    th3_2 = np.real(acos((p41xz_2**2 - A[1]**2 - A[2]**2) / (2 * A[1] * A[2])))
    th3_3 = np.real(acos((p41xz_3**2 - A[1]**2 - A[2]**2) / (2 * A[1] * A[2])))
    th3_4 = np.real(acos((p41xz_4**2 - A[1]**2 - A[2]**2) / (2 * A[1] * A[2])))
    th3_5 = -th3_1
    th3_6 = -th3_2
    th3_7 = -th3_3
    th3_8 = -th3_4

    th2_1 = np.real(atan2(-p41_1[2], -p41_1[0]) - asin((-A[2] * sin(th3_1)) / p41xz_1))
    th2_2 = np.real(atan2(-p41_2[2], -p41_2[0]) - asin((-A[2] * sin(th3_2)) / p41xz_2))
    th2_3 = np.real(atan2(-p41_3[2], -p41_3[0]) - asin((-A[2] * sin(th3_3)) / p41xz_3))
    th2_4 = np.real(atan2(-p41_4[2], -p41_4[0]) - asin((-A[2] * sin(th3_4)) / p41xz_4))
    th2_5 = np.real(atan2(-p41_1[2], -p41_1[0]) - asin((A[2] * sin(th3_1)) / p41xz_1))
    th2_6 = np.real(atan2(-p41_2[2], -p41_2[0]) - asin((A[2] * sin(th3_2)) / p41xz_2))
    th2_7 = np.real(atan2(-p41_3[2], -p41_3[0]) - asin((A[2] * sin(th3_3)) / p41xz_3))
    th2_8 = np.real(atan2(-p41_4[2], -p41_4[0]) - asin((A[2] * sin(th3_4)) / p41xz_4))

    T43m = linalg.inv(dh_matrix(th3_1, D[2], A[2], ALPHA[2])) @ linalg.inv(dh_matrix(th2_1, D[1], A[1], ALPHA[1])) @ linalg.inv(dh_matrix(th1_1, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_1, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_1, D[4], A[4], ALPHA[4]))
    Xhat43 = T43m[:3, 0]
    th4_1 = np.real(atan2(Xhat43[1], Xhat43[0]))

    T43m = linalg.inv(dh_matrix(th3_2, D[2], A[2], ALPHA[2])) @ linalg.inv(dh_matrix(th2_2, D[1], A[1], ALPHA[1])) @ linalg.inv(dh_matrix(th1_1, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_2, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_2, D[4], A[4], ALPHA[4]))
    Xhat43 = T43m[:3, 0]
    th4_2 = np.real(atan2(Xhat43[1], Xhat43[0]))

    T43m = linalg.inv(dh_matrix(th3_3, D[2], A[2], ALPHA[2])) @ linalg.inv(dh_matrix(th2_3, D[1], A[1], ALPHA[1])) @ linalg.inv(dh_matrix(th1_2, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_3, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_3, D[4], A[4], ALPHA[4]))
    Xhat43 = T43m[:3, 0]
    th4_3 = np.real(atan2(Xhat43[1], Xhat43[0]))

    T43m = linalg.inv(dh_matrix(th3_4, D[2], A[2], ALPHA[2])) @ linalg.inv(dh_matrix(th2_4, D[1], A[1], ALPHA[1])) @ linalg.inv(dh_matrix(th1_2, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_4, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_4, D[4], A[4], ALPHA[4]))
    Xhat43 = T43m[:3, 0]
    th4_4 = np.real(atan2(Xhat43[1], Xhat43[0]))

    T43m = linalg.inv(dh_matrix(th3_5, D[2], A[2], ALPHA[2])) @ linalg.inv(dh_matrix(th2_5, D[1], A[1], ALPHA[1])) @ linalg.inv(dh_matrix(th1_1, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_1, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_1, D[4], A[4], ALPHA[4]))
    Xhat43 = T43m[:3, 0]
    th4_5 = np.real(atan2(Xhat43[1], Xhat43[0]))

    T43m = linalg.inv(dh_matrix(th3_6, D[2], A[2], ALPHA[2])) @ linalg.inv(dh_matrix(th2_6, D[1], A[1], ALPHA[1])) @ linalg.inv(dh_matrix(th1_1, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_2, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_2, D[4], A[4], ALPHA[4]))
    Xhat43 = T43m[:3, 0]
    th4_6 = np.real(atan2(Xhat43[1], Xhat43[0]))

    T43m = linalg.inv(dh_matrix(th3_7, D[2], A[2], ALPHA[2])) @ linalg.inv(dh_matrix(th2_7, D[1], A[1], ALPHA[1])) @ linalg.inv(dh_matrix(th1_2, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_3, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_3, D[4], A[4], ALPHA[4]))
    Xhat43 = T43m[:3, 0]
    th4_7 = np.real(atan2(Xhat43[1], Xhat43[0]))

    T43m = linalg.inv(dh_matrix(th3_8, D[2], A[2], ALPHA[2])) @ linalg.inv(dh_matrix(th2_8, D[1], A[1], ALPHA[1])) @ linalg.inv(dh_matrix(th1_2, D[0], A[0], ALPHA[0])) @ T60 @ linalg.inv(dh_matrix(th6_4, D[5], A[5], ALPHA[5])) @ linalg.inv(dh_matrix(th5_4, D[4], A[4], ALPHA[4]))
    Xhat43 = T43m[:3, 0]
    th4_8 = np.real(atan2(Xhat43[1], Xhat43[0]))

    Th = np.array([
        [th1_1, th2_1, th3_1, th4_1, th5_1, th6_1],
        [th1_1, th2_2, th3_2, th4_2, th5_2, th6_2],
        [th1_2, th2_3, th3_3, th4_3, th5_3, th6_3],
        [th1_2, th2_4, th3_4, th4_4, th5_4, th6_4],
        [th1_1, th2_5, th3_5, th4_5, th5_1, th6_1],
        [th1_1, th2_6, th3_6, th4_6, th5_2, th6_2],
        [th1_2, th2_7, th3_7, th4_7, th5_3, th6_3],
        [th1_2, th2_8, th3_8, th4_8, th5_4, th6_4],
    ])

    return Th

def eul2Rot(angles):
    r = Rotation.from_euler('xyz', angles, degrees=True)
    return r.as_matrix()

def rot2Eul(R):
    r = Rotation.from_matrix(R)
    return r.as_euler('xyz', degrees=True)

def cubic_trajectory_planning(q0 , qf, qd0, qdf, m=100):

    n = len(q0)

    a0 = np.copy(q0)
    a1 = np.zeros(n)
    a2 = 3 * (qf - q0) - 2 * qd0 - qdf
    a3 = -2 * (qf - q0) + qd0 + qdf

    timesteps = np.linspace(0, 1, m)

    q = np.zeros((n, m))
    qd = np.zeros((n, m))
    qdd = np.zeros((n, m))

    for i in range(len(timesteps)):
        t = timesteps[i]
        t_2 = t**2
        t_3 = t**3
        q[:, i] = a0 + a1 * t + a2 * t_2 + a3 * t_3
        qd[:, i] = a1 + 2 * a2 * t + 3 * a3 * t_2
        qdd[:, i] = 2 * a2 + 6 * a3 * t

    return q, qd, qdd

def findClosestQ(q0, q):
        minDistance = float('inf')
        closestQ = None
        qf_ = np.zeros(6)
        for q_ in q:
            distance = eucledianDistance(q0, q_)
            if distance < minDistance:
                minDistance = distance
                for i in range(6):
                    qf_[i] = q_[0,i]
                closestQ = qf_
        return closestQ

def eucledianDistance(q1, q2):
        q2f = np.zeros(6)
        for i in range(6):
            q2f[i] = q2[0,i]
        return np.linalg.norm(q1-q2)