import numpy as np
from math import atan2, acos, asin, sin, cos, hypot, pi
from scipy.spatial.transform import Rotation

class frame:
    def __init__(self):
        self.xyz = np.zeros(3)
        self.rot = np.zeros((3, 3))

def t10f(th1):
    aux = np.zeros((4, 4))
    a = np.array([0, -0.425, -0.3922, 0, 0, 0])
    d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14])
    aux[0, 0] = np.cos(th1)
    aux[0, 1] = -np.sin(th1)
    aux[2, 3] = d[0]
    aux[3, 3] = 1
    return aux

def t21f(th2):
    aux = np.zeros((4, 4))
    a = np.array([0, -0.425, -0.3922, 0, 0, 0])
    d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14])
    aux[0, 0] = np.cos(th2)
    aux[0, 1] = -np.sin(th2)
    aux[1, 2] = -1
    aux[2, 0] = np.sin(th2)
    aux[2, 1] = np.cos(th2)
    aux[3, 3] = 1
    return aux

def t32f(th3):
    aux = np.zeros((4, 4))
    a = np.array([0, -0.425, -0.3922, 0, 0, 0])
    d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14])
    aux[0, 0] = np.cos(th3)
    aux[0, 1] = -np.sin(th3)
    aux[0, 3] = a[1]
    aux[1, 0] = np.sin(th3)
    aux[1, 1] = np.cos(th3)
    aux[2, 2] = 1
    aux[2, 3] = d[2]
    aux[3, 3] = 1
    return aux

def t43f(th4):
    aux = np.zeros((4, 4))
    a = np.array([0, -0.425, -0.3922, 0, 0, 0])
    d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14])
    aux[0, 0] = np.cos(th4)
    aux[0, 1] = -np.sin(th4)
    aux[0, 3] = a[2]
    aux[1, 0] = np.sin(th4)
    aux[1, 1] = np.cos(th4)
    aux[2, 2] = 1
    aux[2, 3] = d[3]
    aux[3, 3] = 1
    return aux

def t54f(th5):
    aux = np.zeros((4, 4))
    a = np.array([0, -0.425, -0.3922, 0, 0, 0])
    d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14])
    aux[0, 0] = np.cos(th5)
    aux[0, 1] = -np.sin(th5)
    aux[1, 2] = -1
    aux[2, 0] = np.sin(th5)
    aux[2, 1] = np.cos(th5)
    aux[2, 3] = -d[4]
    aux[3, 3] = 1
    return aux

def t65f(th6):
    aux = np.zeros((4, 4))
    a = np.array([0, -0.425, -0.3922, 0, 0, 0])
    d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14])
    aux[0, 0] = np.cos(th6)
    aux[0, 1] = -np.sin(th6)
    aux[1, 2] = 1
    aux[2, 0] = -np.sin(th6)
    aux[2, 1] = -np.cos(th6)
    aux[2, 3] = d[5]
    aux[3, 3] = 1
    return aux

def forward_kinematics(th):
    ret = frame()
    T60 = t10f(th[0]) @ t21f(th[1]) @ t32f(th[2]) @ t43f(th[3]) @ t54f(th[4]) @ t65f(th[5])

    ret.rot = T60[:3, :3]
    ret.xyz = T60[:3, 3]

    return ret

def inverse_kinematics(frame):
    a = np.array([0, -0.425, -0.3922, 0, 0, 0], dtype=np.float32)
    d = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14], dtype=np.float32)

    T60 = np.zeros((4, 4), dtype=np.float32)
    T60[:3, :3] = frame.rot
    T60[:3, 3] = frame.xyz
    T60[3, 3] = 1

    p50 = np.dot(T60, np.array([0, 0, -d[5], 1], dtype=np.float32))
    th1_1 = atan2(p50[1], p50[0]) + acos(d[3] / hypot(p50[1], p50[0])) + pi / 2
    th1_2 = atan2(p50[1], p50[0]) - acos(d[3] / hypot(p50[1], p50[0])) + pi / 2

    th5_1 = acos((frame.xyz[0] * sin(th1_1) - frame.xyz[1] * cos(th1_1) - d[3]) / d[5])
    th5_2 = -acos((frame.xyz[0] * sin(th1_1) - frame.xyz[1] * cos(th1_1) - d[3]) / d[5])
    th5_3 = acos((frame.xyz[0] * sin(th1_2) - frame.xyz[1] * cos(th1_2) - d[3]) / d[5])
    th5_4 = -acos((frame.xyz[0] * sin(th1_2) - frame.xyz[1] * cos(th1_2) - d[3]) / d[5])

    t06 = np.linalg.inv(T60)

    Xhat = t06[:3, 0]
    Yhat = t06[:3, 1]

    th6_1 = atan2(-Xhat[1] * sin(th1_1) + Yhat[1] * cos(th1_1), Xhat[0] * sin(th1_1) - Yhat[0] * cos(th1_1)) / sin(th5_1)
    th6_2 = atan2(-Xhat[1] * sin(th1_1) + Yhat[1] * cos(th1_1), Xhat[0] * sin(th1_1) - Yhat[0] * cos(th1_1)) / sin(th5_2)
    th6_3 = atan2(-Xhat[1] * sin(th1_2) + Yhat[1] * cos(th1_2), Xhat[0] * sin(th1_2) - Yhat[0] * cos(th1_2)) / sin(th5_3)
    th6_4 = atan2(-Xhat[1] * sin(th1_2) + Yhat[1] * cos(th1_2), Xhat[0] * sin(th1_2) - Yhat[0] * cos(th1_2)) / sin(th5_4)

    t41m = np.zeros((4, 4), dtype=np.float32)
    p41_1 = np.zeros(3, dtype=np.float32)
    p41_2 = np.zeros(3, dtype=np.float32)
    p41_3 = np.zeros(3, dtype=np.float32)
    p41_4 = np.zeros(3, dtype=np.float32)
    p41xz_1 = 0
    p41xz_2 = 0
    p41xz_3 = 0
    p41xz_4 = 0

    t41m = np.dot(np.dot(np.dot(np.dot(t10f(th1_1).inv, T60), t65f(th6_1).inv), t54f(th5_1).inv), t43f(th4_1).inv)
    p41_1 = t41m[:3, 3]
    p41xz_1 = hypot(p41_1[0], p41_1[2])

    t41m = np.dot(np.dot(np.dot(np.dot(t10f(th1_1).inv, T60), t65f(th6_2).inv), t54f(th5_2).inv), t43f(th4_2).inv)
    p41_2 = t41m[:3, 3]
    p41xz_2 = hypot(p41_2[0], p41_2[2])

    t41m = np.dot(np.dot(np.dot(np.dot(t10f(th1_2).inv, T60), t65f(th6_3).inv), t54f(th5_3).inv), t43f(th4_3).inv)
    p41_3 = t41m[:3, 3]
    p41xz_3 = hypot(p41_3[0], p41_3[2])

    t41m = np.dot(np.dot(np.dot(np.dot(t10f(th1_2).inv, T60), t65f(th6_4).inv), t54f(th5_4).inv), t43f(th4_4).inv)
    p41_4 = t41m[:3, 3]
    p41xz_4 = hypot(p41_4[0], p41_4[2])

    th3_1 = 0
    if (p41xz_1 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]) > 1:
        th3_1 = 0
    elif (p41xz_1 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]) < -1:
        th3_1 = pi
    else:
        th3_1 = acos((p41xz_1 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]))

    th3_2 = 0
    if (p41xz_2 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]) > 1:
        th3_2 = 0
    elif (p41xz_2 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]) < -1:
        th3_2 = pi
    else:
        th3_2 = acos((p41xz_2 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]))

    th3_3 = 0
    if (p41xz_3 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]) > 1:
        th3_3 = 0
    elif (p41xz_3 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]) < -1:
        th3_3 = pi
    else:
        th3_3 = acos((p41xz_3 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]))

    th3_4 = 0
    if (p41xz_4 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]) > 1:
        th3_4 = 0
    elif (p41xz_4 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]) < -1:
        th3_4 = pi
    else:
        th3_4 = acos((p41xz_4 ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]))

    th3_5 = -th3_1
    th3_6 = -th3_2
    th3_7 = -th3_3
    th3_8 = -th3_4

    th2_1 = atan2(-p41_1[2], -p41_1[0]) - asin((-a[2] * sin(th3_1)) / p41xz_1)
    th2_2 = atan2(-p41_2[2], -p41_2[0]) - asin((-a[2] * sin(th3_2)) / p41xz_2)
    th2_3 = atan2(-p41_3[2], -p41_3[0]) - asin((-a[2] * sin(th3_3)) / p41xz_3)
    th2_4 = atan2(-p41_4[2], -p41_4[0]) - asin((-a[2] * sin(th3_4)) / p41xz_4)

    th2_5 = atan2(-p41_1[2], -p41_1[0]) - asin((a[2] * sin(th3_1)) / p41xz_1)
    th2_6 = atan2(-p41_2[2], -p41_2[0]) - asin((a[2] * sin(th3_2)) / p41xz_2)
    th2_7 = atan2(-p41_3[2], -p41_3[0]) - asin((a[2] * sin(th3_3)) / p41xz_3)
    th2_8 = atan2(-p41_4[2], -p41_4[0]) - asin((a[2] * sin(th3_4)) / p41xz_4)

    t43m = np.zeros((4, 4), dtype=np.float32)
    xhat43 = np.zeros(3, dtype=np.float32)

    t43m = np.dot(np.dot(np.dot(np.dot(t32f(th3_1).inv, t21f(th2_1).inv), t10f(th1_1).inv), T60), t65f(th6_1).inv)
    xhat43 = t43m[:3, 0]
    th4_1 = atan2(xhat43[1], xhat43[0])

    t43m = np.dot(np.dot(np.dot(np.dot(t32f(th3_2).inv, t21f(th2_2).inv), t10f(th1_1).inv), T60), t65f(th6_2).inv)
    xhat43 = t43m[:3, 0]
    th4_2 = atan2(xhat43[1], xhat43[0])

    t43m = np.dot(np.dot(np.dot(np.dot(t32f(th3_3).inv, t21f(th2_3).inv), t10f(th1_2).inv), T60), t65f(th6_3).inv)
    xhat43 = t43m[:3, 0]
    th4_3 = atan2(xhat43[1], xhat43[0])

    t43m = np.dot(np.dot(np.dot(np.dot(t32f(th3_4).inv, t21f(th2_4).inv), t10f(th1_2).inv), T60), t65f(th6_4).inv)
    xhat43 = t43m[:3, 0]
    th4_4 = atan2(xhat43[1], xhat43[0])

    t43m = np.dot(np.dot(np.dot(np.dot(t32f(th3_5).inv, t21f(th2_5).inv), t10f(th1_1).inv), T60), t65f(th6_1).inv)
    xhat43 = t43m[:3, 0]
    th4_5 = atan2(xhat43[1], xhat43[0])

    t43m = np.dot(np.dot(np.dot(np.dot(t32f(th3_6).inv, t21f(th2_6).inv), t10f(th1_1).inv), T60), t65f(th6_2).inv)
    xhat43 = t43m[:3, 0]
    th4_6 = atan2(xhat43[1], xhat43[0])

    t43m = np.dot(np.dot(np.dot(np.dot(t32f(th3_7).inv, t21f(th2_7).inv), t10f(th1_2).inv), T60), t65f(th6_3).inv)
    xhat43 = t43m[:3, 0]
    th4_7 = atan2(xhat43[1], xhat43[0])

    t43m = np.dot(np.dot(np.dot(np.dot(t32f(th3_8).inv, t21f(th2_8).inv), t10f(th1_2).inv), T60), t65f(th6_4).inv)
    xhat43 = t43m[:3, 0]
    th4_8 = atan2(xhat43[1], xhat43[0])

    ret = np.zeros((8, 6), dtype=np.float32)
    ret[0] = [th1_1, th2_1, th3_1, th4_1, th5_1, th6_1]
    ret[1] = [th1_1, th2_2, th3_2, th4_2, th5_2, th6_2]
    ret[2] = [th1_2, th2_3, th3_3, th4_3, th5_3, th6_3]
    ret[3] = [th1_2, th2_4, th3_4, th4_4, th5_4, th6_4]
    ret[4] = [th1_1, th2_5, th3_5, th4_5, th5_1, th6_1]
    ret[5] = [th1_1, th2_6, th3_6, th4_6, th5_2, th6_2]
    ret[6] = [th1_2, th2_7, th3_7, th4_7, th5_3, th6_3]
    ret[7] = [th1_2, th2_8, th3_8, th4_8, th5_4, th6_4]

    return ret

def jacobian(q):
    A = np.array([0, -0.425, -0.3922, 0, 0, 0])
    D = np.array([0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14])

    J = np.zeros((6, 6))

    J1 = np.array([
        D[4] * (np.cos(q[0]) * np.cos(q[4]) + np.cos(q[1] + q[2] + q[3]) * np.sin(q[0]) * np.sin(q[4])) + D[2] * np.cos(q[0]) + D[3] * np.cos(q[0]) - A[2] * np.cos(q[1] + q[2]) * np.sin(q[0]) - A[1] * np.cos(q[1]) * np.sin(q[0]) - D[4] * np.sin(q[1] + q[2] + q[3]) * np.sin(q[0]),
        D[4] * (np.cos(q[4]) * np.sin(q[0]) - np.cos(q[1] + q[2] + q[3]) * np.cos(q[0]) * np.sin(q[4])) + D[2] * np.sin(q[0]) + D[3] * np.sin(q[0]) + A[2] * np.cos(q[1] + q[2]) * np.cos(q[0]) + A[1] * np.cos(q[0]) * np.cos(q[1]) + D[4] * np.sin(q[1] + q[2] + q[3]) * np.cos(q[0]),
        0,
        0,
        0,
        1
    ])

    J2 = np.array([
        -np.cos(q[0]) * (A[2] * np.sin(q[1] + q[2]) + A[1] * np.sin(q[1]) + D[4] * (np.sin(q[1] + q[2]) * np.sin(q[3]) - np.cos(q[1] + q[2]) * np.cos(q[3]))) - D[4] * np.sin(q[4]) * (np.cos(q[1] + q[2]) * np.sin(q[3]) + np.sin(q[1] + q[2]) * np.cos(q[3])),
        -np.sin(q[0]) * (A[2] * np.sin(q[1] + q[2]) + A[1] * np.sin(q[1]) + D[4] * (np.sin(q[1] + q[2]) * np.sin(q[3]) - np.cos(q[1] + q[2]) * np.cos(q[3]))) - D[4] * np.sin(q[4]) * (np.cos(q[1] + q[2]) * np.sin(q[3]) + np.sin(q[1] + q[2]) * np.cos(q[3])),
        A[2] * np.cos(q[1] + q[2]) - (D[4] * np.sin(q[1] + q[2] + q[3] + q[4])) / 2 + A[1] * np.cos(q[1]) + (D[4] * np.sin(q[1] + q[2] + q[3] - q[4])) / 2 + D[4] * np.sin(q[1] + q[2] + q[3]),
        np.sin(q[0]),
        -np.cos(q[0]),
        0
    ])

    J3 = np.array([
        np.cos(q[0]) * (D[4] * np.cos(q[1] + q[2] + q[3]) - A[2] * np.sin(q[1] + q[2]) + D[4] * np.sin(q[1] + q[2] + q[3]) * np.sin(q[4])),
        np.sin(q[0]) * (D[4] * np.cos(q[1] + q[2] + q[3]) - A[2] * np.sin(q[1] + q[2]) + D[4] * np.sin(q[1] + q[2] + q[3]) * np.sin(q[4])),
        A[2] * np.cos(q[1] + q[2]) - (D[4] * np.sin(q[1] + q[2] + q[3] + q[4])) / 2 + (D[4] * np.sin(q[1] + q[2] + q[3] - q[4])) / 2 + D[4] * np.sin(q[1] + q[2] + q[3]),
        np.sin(q[0]),
        -np.cos(q[0]),
        0
    ])

    J4 = np.array([
        D[4] * np.cos(q[0]) * (np.cos(q[1] + q[2] + q[3]) + np.sin(q[1] + q[2] + q[3]) * np.sin(q[4])),
        D[4] * np.sin(q[0]) * (np.cos(q[1] + q[2] + q[3]) + np.sin(q[1] + q[2] + q[3]) * np.sin(q[4])),
        D[4] * (np.sin(q[1] + q[2] + q[3] - q[4]) / 2 + np.sin(q[1] + q[2] + q[3]) - np.sin(q[1] + q[2] + q[3] + q[4]) / 2),
        np.sin(q[0]),
        -np.cos(q[0]),
        0
    ])

    J5 = np.array([
        -D[4] * np.sin(q[0]) * np.sin(q[4]) - D[4] * np.cos(q[1] + q[2] + q[3]) * np.cos(q[0]) * np.cos(q[4]),
        D[4] * np.cos(q[0]) * np.sin(q[4]) - D[4] * np.cos(q[1] + q[2] + q[3]) * np.cos(q[4]) * np.sin(q[0]),
        -D[4] * (np.sin(q[1] + q[2] + q[3] - q[4]) / 2 + np.sin(q[1] + q[2] + q[3] + q[4]) / 2),
        np.sin(q[1] + q[2] + q[3]) * np.cos(q[0]),
        np.sin(q[1] + q[2] + q[3]) * np.sin(q[0]),
        -np.cos(q[1] + q[2] + q[3])
    ])

    J6 = np.array([
        0,
        0,
        0,
        np.cos(q[4]) * np.sin(q[0]) - np.cos(q[1] + q[2] + q[3]) * np.cos(q[0]) * np.sin(q[4]),
        -np.cos(q[0]) * np.cos(q[4]) - np.cos(q[1] + q[2] + q[3]) * np.sin(q[0]) * np.sin(q[4]),
        -np.sin(q[1] + q[2] + q[3]) * np.sin(q[4])
    ])

    J[:, 0] = J1
    J[:, 1] = J2
    J[:, 2] = J3
    J[:, 3] = J4
    J[:, 4] = J5
    J[:, 5] = J6

    return J

def eul2rotm(rpy):
    rpy = np.array([rpy.x, rpy.y, rpy.z])
    r = Rotation.from_euler('zyx', rpy, degrees=False)
    R = r.as_matrix()
    return R

# [ERROR] [1685526937.147981]: bad callback: <function pos_callback at 0x7fa33d60d550>
# Traceback (most recent call last):
#   File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
#     cb(msg)
#   File "/home/toto/ros_ws/src/UR5BlokVision/motion2/motionPlanner.py", line 144, in pos_callback
#     move()
#   File "/home/toto/ros_ws/src/UR5BlokVision/motion2/motionPlanner.py", line 169, in move
#     inv_diff_kinematic_control_sim_complete(target, pose.orientation, dt)
#   File "/home/toto/ros_ws/src/UR5BlokVision/motion2/motionPlanner.py", line 129, in inv_diff_kinematic_control_sim_complete
#     dotqk = inv_diff_kinematic_control_complete(qk, now.xyz, pd(i, xef, start.xyz), vd, now.rot, phief)
#   File "/home/toto/ros_ws/src/UR5BlokVision/motion2/motionPlanner.py", line 71, in inv_diff_kinematic_control_complete
#     w_R_d = eul2rotm(phief)
#   File "/home/toto/ros_ws/src/UR5BlokVision/motion2/kinematics.py", line 313, in eul2rotm
#     rpy = np.array([rpy.x, rpy.y, rpy.z])
# AttributeError: 'numpy.ndarray' object has no attribute 'x'
