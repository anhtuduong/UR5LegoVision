"""
Generate acados code required for the
cpp interface in the ROS
Author: Niraj Rathod
Date : 10/06/21

"""

import sys
import os
current_dir = os.path.dirname(os.path.dirname( __file__ ))
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
               current_dir]  # this is the current directory

import numpy as np
from tools.mathutils import *
from tools.utils import Utils
from tools.kinematics_utils import feet_posW_using_fwdkin, compute_feet_velocities, forward_kinematics

util = Utils()


q =np.array([-0.253301,
    0.709564,
   -1.6188,
  -0.15376,
  0.696625,
  -1.47141,
 -0.288594,
 -0.793954,
   1.39208,
-0.0915037,
  -0.68222,
   1.28396])

qd =np.array([     -0.280354,
 -0.15428,
 0.474958,
 0.349705,
-0.566016,
 0.677527,
0.0212077,
 0.357983,
-0.521826,
0.0650535,
 0.374369,
 -1.09902])

com =np.array([    0.00526017,
0.00174553,
   0.56522])

com_vel = np.array([    -0.00602104,
  0.0110262,
   0.152876])



omega =np.array([-0.159796,   0.06291,-0.236714])

orient = np.array([-0.0714579,  0.064307, 0.0466642])


foot_vel = np.array([ -0.0262146, 0.181075, -0.0785915,
                      0.124778,  0.181084, -0.183612  ,
                      -0.0416642, 0.0220396,  -0.11948  ,
                      0.102001,0.0206603,-0.225329])

print(compute_feet_velocities(np.hstack([com, com_vel, orient, omega, q]), qd))

# foot_pos =np.array([ 0.414533,0.346089,-0.53731 ,
#                         0.38748, -0.297686,-0.585137 ,
#                        -0.31592, 0.379801,-0.582051,
#                       -0.345852, -0.26482,-0.630117])
#
#
#
#
# print (forward_kinematics(q))